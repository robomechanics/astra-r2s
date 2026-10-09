"""Separate post-qpos robot/jaw kinematics; no contact solving or integration."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'thread_lab/runtime.py').is_file())
sys.path.insert(0,str(ROOT/'scripts'))
import mujoco
from audit_m8_insertion_trace import recorded_model
from thread_lab.runtime import require_micron_engine


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def inner_face_gap(vertices,center,orientation,halfsize,bolt_head_center):
    """Signed head-to-INNER-pad-face support gap; finite-face caveat explicit."""
    axis=orientation[:,1]
    if (bolt_head_center-center)@axis<0:
        axis=-axis
    face=center+axis*halfsize[1]
    gap=float(np.min((vertices-face)@axis))
    return gap,axis


def audit(path,parent):
    path,parent=Path(path),Path(parent)
    archive=np.load(path/'checkpoint_trace.npz',allow_pickle=False)
    rows=json.loads(str(archive['info_json'].item()))
    declaration=json.loads((path/'declaration.json').read_text())
    parent_archive=np.load(parent,allow_pickle=False)
    metadata=json.loads(str(parent_archive['metadata_json'].item()))
    assert sha(parent)==declaration['parent_trace_sha256']
    assert sha(path/'scene.xml')==metadata['model_xml_sha256']
    runtime=require_micron_engine()
    model,identity=recorded_model(parent,metadata)
    data=mujoco.MjData(model)
    site=model.site('right_grasp_site').id
    hole=model.body('female_frame').id
    bolt=model.body('male_bolt').id
    head=model.geom('bolt_head').id
    pads=[model.geom('right_m8_pad_'+side).id for side in('left','right')]
    fingers=[model.joint('right_'+side+'_finger').id for side in('left','right')]
    finger_indices=model.jnt_qposadr[fingers]
    mesh=model.geom_dataid[head]
    vertices=model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh]+model.mesh_vertnum[mesh]].copy()
    frozen_R=np.asarray(declaration['frozen_open_tool_rotation_relative_hole'])
    frozen_p=np.asarray(declaration['frozen_open_tool_position_relative_hole_m'])
    offset=np.asarray(declaration['frozen_open_head_to_tool_offset_m'])
    open_clock=declaration['actual_open_start_tool_clock_rad']
    records=[]
    for i,row in enumerate(rows):
        data.qpos[:]=archive['qpos'][i]
        mujoco.mj_kinematics(model,data)
        hp=data.xpos[hole];hr=data.xmat[hole].reshape(3,3)
        sp=data.site_xpos[site];sr=data.site_xmat[site].reshape(3,3)
        theta=row['desired_turn_theta_rad']
        target_R=hr@Rotation.from_euler('z',theta-open_clock).as_matrix()@frozen_R
        target_p=hp+hr@(frozen_p+frozen_R@offset)-target_R@offset
        head_vertices=vertices@data.geom_xmat[head].reshape(3,3).T+data.geom_xpos[head]
        head_center=data.xpos[bolt]+data.xmat[bolt].reshape(3,3)@np.array([0,0,-metadata['scene_config']['head_height']/2])
        gaps=[inner_face_gap(head_vertices,data.geom_xpos[g],data.geom_xmat[g].reshape(3,3),model.geom_size[g],head_center)[0]for g in pads]
        transverse=head_center-sp
        error_R=Rotation.from_matrix(target_R@sr.T).as_rotvec()
        actual_delta=hr.T@sr@frozen_R.T
        actual_clock=open_clock+np.arctan2(actual_delta[1,0],actual_delta[0,0])
        finger_positions=data.qpos[finger_indices]
        aperture=float(finger_positions[0]-finger_positions[1]-2*metadata['scene_config']['base']['pad_inner_offset'])
        records.append(dict(saved_state_index=i,elapsed_s=row['elapsed_s'],original_force_time_s=row['time_s']-model.opt.timestep,
            post_qpos_site_position_m=sp.copy().tolist(),post_qpos_planned_site_position_m=target_p.tolist(),
            post_qpos_tracking_position_error_m=(target_p-sp).tolist(),
            post_qpos_tracking_position_error_norm_m=float(np.linalg.norm(target_p-sp)),
            post_qpos_tracking_rotation_error_rad=error_R.tolist(),post_qpos_tracking_rotation_error_norm_rad=float(np.linalg.norm(error_R)),
            post_qpos_actual_tool_clock_rad=float(actual_clock),original_commanded_clock_rad=theta,
            post_qpos_head_to_tool_translation_m=(sr.T@transverse).tolist(),
            post_qpos_finger_positions_m=finger_positions.copy().tolist(),commanded_aperture_m=row['right_commanded_aperture_m'],
            post_qpos_actual_aperture_m=aperture,post_qpos_signed_inner_pad_face_gaps_m=gaps,
            original_whole_right_bolt_contact_count=row['right_bolt_contact_count'],
            physical_phase=row['physical_phase']))
    reset=[r for r in records if r['physical_phase']=='reset_open_search_2']
    across_flats=metadata['scene_config']['head_across_flats']
    circum=2*across_flats/np.sqrt(3)
    return dict(scope='One hash-verified archived model compile and mj_kinematics on saved post-qpos only. '
        'No mj_forward, collision solving, force reconstruction or integration. '
        'Post-state planned-pose comparison uses the same saved hole frame, not the unavailable exact pre-command derived frame. '
        'Plane support gaps use exact compiled mesh vertices but ignore finite pad face bounds and are not an independent collision solution.',
        auditor_source_sha256=sha(__file__),original_trace_sha256=sha(path/'checkpoint_trace.npz'),model_identity=identity,
        runtime=runtime,original_commanded_open_aperture_m=metadata['control_config']['arm']['open_aperture'],
        nominal_head_circumdiameter_m=float(circum),
        nominal_centered_worst_orientation_per_pad_clearance_m=float((metadata['control_config']['arm']['open_aperture']-circum)/2),
        native_finger_joint_ranges_m=model.jnt_range[fingers].tolist(),
        declared_controller_open_aperture_upper_bound_m=.040,
        maximum_reset_saved_position_tracking_error_m=max(r['post_qpos_tracking_position_error_norm_m']for r in reset),
        maximum_reset_saved_rotation_tracking_error_rad=max(r['post_qpos_tracking_rotation_error_norm_rad']for r in reset),
        minimum_reset_saved_inner_face_gaps_m=[min(r['post_qpos_signed_inner_pad_face_gaps_m'][j]for r in reset)for j in range(2)],
        final_saved_geometry=records[-1],saved_tracking_rows=records,
        original_force_evidence_scope='The independent original-force audit controls whether a native contact happened. '
            'This post-qpos geometry cannot overwrite the retained original pre-integration contact/load evidence.',
        full_capture=False,qualified_reset=False,full_trajectory_qualified=False)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('directory',type=Path)
    parser.add_argument('--parent',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=audit(args.directory,args.parent)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k]for k in('scope','nominal_centered_worst_orientation_per_pad_clearance_m',
        'maximum_reset_saved_position_tracking_error_m','maximum_reset_saved_rotation_tracking_error_rad','final_saved_geometry')},indent=2))
