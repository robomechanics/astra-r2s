"""Read the closed da69 open-reset failure; pure arrays, no native replay."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np

PRODUCER='da69a9cd44a8312cc7b97365faf5e09c27a646e2'
OFFICIAL='4a8b018a1326bce540a0351d5d715ab58ffb505d2ae44be6d34aa8d8cd68fe3f'


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024),b''):
            h.update(b)
    return h.hexdigest()


def require(ok,message):
    if not ok:
        raise ValueError(message)


def capped(v,limit):
    return v*(limit/np.maximum(np.linalg.norm(v,axis=-1,keepdims=True),limit))


def pd_error_components(wrench,position_error,rotation_error,arm):
    wrench=np.asarray(wrench,float);p=np.asarray(position_error,float);r=np.asarray(rotation_error,float)
    require(wrench.shape==(6,) and p.shape==r.shape==(3,) and np.isfinite(np.r_[wrench,p,r]).all(),'Invalid original uncapped PD/error vectors')
    kp,dp,kr,dr=(float(arm[k]) for k in ('position_stiffness','position_damping','rotation_stiffness','rotation_damping'))
    require(np.isfinite([kp,dp,kr,dr]).all() and min(kp,dp,kr,dr)>0,'Invalid native PD coefficients')
    spring_force=kp*p;spring_torque=kr*r
    return {'original_position_stiffness_force_N':spring_force.tolist(),
        'original_rotation_stiffness_torque_Nm':spring_torque.tolist(),
        'original_velocity_damping_force_N':(wrench[:3]-spring_force).tolist(),
        'original_angular_damping_torque_Nm':(wrench[3:]-spring_torque).tolist(),
        'inferred_same_command_linear_velocity_error_m_per_s':((wrench[:3]-spring_force)/dp).tolist(),
        'inferred_same_command_angular_velocity_error_rad_per_s':((wrench[3:]-spring_torque)/dr).tolist(),
        'scope':'Uncapped original OPEN PD wrench and original same-command retained pose errors. Full XYZ, no axial feed/projection and FF disabled. Damping velocity error is algebraically inferred, not a new motion/force replay or a saved post-state measurement.'}


def phase_baseline(time,phase,phase_id,dt):
    ids=np.flatnonzero(phase==phase_id)
    require(len(ids)>0,'Unexecuted phase has no measured baseline')
    return {'native_steps':len(ids),'actual_duration_s':len(ids)*dt,
        'first_post_label_time_s':float(time[ids[0]]),'last_post_label_time_s':float(time[ids[-1]]),
        'original_force_baseline_time_s':float(time[ids[0]]-2*dt),
        'first_original_force_time_s':float(time[ids[0]]-dt),'last_original_force_time_s':float(time[ids[-1]]-dt)}


def first_open_contact(opened,counts):
    require(opened.dtype.kind=='b','Open flags must be native booleans')
    require(counts.dtype.kind in 'iu' and np.all(counts>=0),'Contact counts must be native nonnegative integers')
    ids=np.flatnonzero(opened & (counts>0))
    return None if not len(ids) else int(ids[0])


def command_stats(commands,mask):
    require(mask.dtype.kind=='b' and np.any(mask),'No executed commands in selected scope')
    a={k:v[mask] for k,v in commands.items()}
    pd=a['pd_wrench_world_N_Nm'];ff=a['ff_wrench_world_N_Nm']
    total=pd+ff
    actual=a['capped_wrench_world_N_Nm']
    require(np.allclose(total,a['combined_uncapped_wrench_world_N_Nm'],rtol=1e-12,atol=1e-12),'Original PD+FF sum differs')
    require(np.allclose(np.c_[capped(total[:,:3],8.),capped(total[:,3:],2.)],actual,rtol=1e-12,atol=1e-12),'Actual combined Cartesian cap differs')
    require(np.array_equal(a['cartesian_force_clipped'],np.linalg.norm(total[:,:3],axis=1)>8.) and np.array_equal(a['cartesian_torque_clipped'],np.linalg.norm(total[:,3:],axis=1)>2.),'Original Cartesian cap flags differ')
    return {'executed_commands':int(np.sum(mask)),
        'uncapped_PD_force_maximum_N':float(np.max(np.linalg.norm(pd[:,:3],axis=1))),
        'uncapped_PD_torque_maximum_Nm':float(np.max(np.linalg.norm(pd[:,3:],axis=1))),
        'applied_force_maximum_N':float(np.max(np.linalg.norm(actual[:,:3],axis=1))),
        'applied_torque_maximum_Nm':float(np.max(np.linalg.norm(actual[:,3:],axis=1))),
        'cartesian_force_clipped_native_ticks':int(np.sum(a['cartesian_force_clipped'])),
        'cartesian_torque_clipped_native_ticks':int(np.sum(a['cartesian_torque_clipped'])),
        'any_native_motor_clipped_ticks':int(np.sum(np.any(a['motor_clipped'],axis=1))),
        'force_under_cap_native_ticks':int(np.sum(~a['cartesian_force_clipped'])),
        'minimum_actual_motor_cap_margin_Nm':float(np.min(np.array([28.,28.,28.,10.,10.,10.])-np.abs(a['motor_torques_Nm'])))}


def audit(run,audit_dir):
    run=Path(run).resolve();audit_dir=Path(audit_dir).resolve()
    original=json.loads((run/'insertion_validation.json').read_text())
    closure=json.loads((run/'run_publication_identity_after.json').read_text())
    identity=json.loads((audit_dir/'source_identity_after.json').read_text())
    official=json.loads((audit_dir/'independent_supported_audit.json').read_text())
    require(closure['producer_commit']==identity['producer_commit']==PRODUCER,'Wrong closed producer')
    require(closure['native_exit_code']==identity['closure_native_exit_code']==1,'Original failure exit changed')
    require(identity['complete_current_and_archived_tested_sources_match_original'] is True and identity['source_files_unchanged']==74,'Original tested source identity failed')
    require(official['auditor_source_sha256']==OFFICIAL and digest(run/'frozen_audit_sources/scripts/audit_m8_supported_trace.py')==OFFICIAL,'Official source identity differs')
    require(official['original_report_passed']==original['passed'] is False and official['original_acceptance_checks']==original['acceptance_checks'],'Original failed acceptance rows changed')
    require(official['trajectory_sha256']==digest(run/'insertion_trace.npz'),'Official trace bytes changed')
    ff_official=official['original_physical_feedback_history']['original_robot_inertia_history']
    require(ff_official['passed'] is True,'Official FF/cache/mapping verification did not pass')
    dt=float(original['scene_config']['base']['thread']['timestep'])
    fb_keys=('time','phase_index','right_robot_bolt_contact_count','right_pad_normal_force_N','all_hard_guards_held','fully_open_unassisted','right_actual_aperture_postintegration_m','right_command_wrench_N_Nm','right_motor_torques_Nm','desired_independent_clock_rad','desired_independent_angular_speed_rad_s','open_peak_axial_drift_m','open_peak_yaw_drift_rad','thread_gravity_opposing_force_N','hand_gravity_opposing_force_N','external_drive_zero','loaded_actual_interior_flank_contact_count','right_applied_axial_feed_N','right_axial_float_active','open_weight_window_ready')
    with np.load(run/'native_feedback_force_history.npz',allow_pickle=False) as z:
        feedback={k:z[k] for k in fb_keys};labels=json.loads(str(z['phase_labels_json']))
    require(digest(run/'native_feedback_force_history.npz')==original['native_feedback_force_history']['sha256']==official['original_physical_feedback_history']['raw_sha256'],'Original native feedback bytes changed')
    names=np.asarray(labels,dtype=object)[feedback['phase_index']]
    reset=names=='reset_open_search_2';opened=feedback['fully_open_unassisted']
    time=feedback['time'];last=len(time)-1
    require(len(time)==441657 and time[last]==original['aborted']['time'],'Wrong original native endpoints')
    bad=first_open_contact(opened,feedback['right_robot_bolt_contact_count'])
    require(bad==last,'This original failure must retain its first final open contact')
    require(np.all(feedback['all_hard_guards_held'][:last]) and not feedback['all_hard_guards_held'][last],'Original hard guard failure scope differs')
    require(np.all(feedback['external_drive_zero']),'Original free-object drive guard failed')
    cmd_keys=('time','phase_index','enabled','inertia_inputs_present','jacobian_derivatives_present','pd_wrench_world_N_Nm','ff_wrench_world_N_Nm','combined_uncapped_wrench_world_N_Nm','capped_wrench_world_N_Nm','cartesian_force_clipped','cartesian_torque_clipped','motor_clipped','motor_torques_Nm')
    with np.load(run/'robot_inertia_command_history.npz',allow_pickle=False) as z:
        commands={k:z[k] for k in cmd_keys}
    require(digest(run/'robot_inertia_command_history.npz')==original['robot_inertia_command_history']['sha256']==ff_official['raw_sha256'],'Original robot command bytes changed')
    require(np.array_equal(commands['time'],time) and np.array_equal(commands['phase_index'],feedback['phase_index']),'Command and force labels differ')
    require(not np.any(commands['enabled'][opened]) and not np.any(commands['inertia_inputs_present'][opened]) and not np.any(commands['jacobian_derivatives_present'][opened]) and not np.any(commands['ff_wrench_world_N_Nm'][opened]),'Fully-open disabled FF scope differs')
    require(not np.any(feedback['right_axial_float_active'][opened]) and not np.any(feedback['right_applied_axial_feed_N'][opened]),'Fully-open robot must have no closed axial float/feed')
    require(np.array_equal(commands['capped_wrench_world_N_Nm'],feedback['right_command_wrench_N_Nm']) and np.array_equal(commands['motor_torques_Nm'],feedback['right_motor_torques_Nm']),'Executed robot and native feedback commands differ')
    clipping=np.flatnonzero(reset & commands['cartesian_force_clipped'])
    with np.load(run/'insertion_trace.npz',allow_pickle=False) as z:
        rows=json.loads(str(z['info_json']));saved_times=z['time']
    samples=[r for r in rows if r['phase']=='reset_open_search_2']
    require(rows[-1]['time']==time[-1] and rows[-1]['right_robot_bolt_contact_count']==1,'Failure lacks exact original saved row')
    p_error=np.array([r['right_position_error_m'] for r in samples]);R_error=np.array([r['right_rotation_error_rad'] for r in samples])
    require(np.isfinite(p_error).all() and np.isfinite(R_error).all(),'Nonfinite saved original tracking errors')
    last_row=rows[-1]
    command_final={k:commands[k][-1].tolist() for k in cmd_keys if k not in ('time','phase_index')}
    original_final={k:feedback[k][-1].tolist() for k in fb_keys}
    original_final.update({'original_force_geometry_time_s':float(time[-1]-dt),'retained_command_pose_time_s':float(time[-1]-2*dt),
        'right_position_error_m':last_row['right_position_error_m'],'right_rotation_error_rad':last_row['right_rotation_error_rad'],
        'position_error_norm_m':float(np.linalg.norm(last_row['right_position_error_m'])),
        'rotation_error_norm_rad':float(np.linalg.norm(last_row['right_rotation_error_rad'])),
        'original_sampled_hand_contact':last_row['contact'],'original_unexpected_native_contacts':last_row['unexpected_native_contacts'],
        'minimum_original_native_joint_margin_rad':last_row['minimum_native_joint_margin_rad']})
    original_final['same_command_open_PD_components']=pd_error_components(commands['pd_wrench_world_N_Nm'][-1],last_row['right_position_error_m'],last_row['right_rotation_error_rad'],original['control_config']['arm'])
    release=next(e for e in original['physical_motion_events'] if e['event']=='Frozen actual release calibration')
    maximum=next(p[1] for p in original['maximum_phase_plan'] if p[0]=='reset_open_search_2')
    head_af=float(original['scene_config']['head_across_flats']);aperture=float(feedback['right_actual_aperture_postintegration_m'][-1])
    fully_open_before_bad=np.flatnonzero(opened & (np.arange(len(time))<bad))
    return {'kind':'Independent supplemental original open-reset contact failure arithmetic','reader_source_sha256':digest(__file__),
        'producer_commit':PRODUCER,'native_child_exit_code':1,'original_report_passed':False,
        'original_aborted':original['aborted'],'official_auditor_source_sha256':OFFICIAL,
        'official_supported_report_sha256':digest(audit_dir/'independent_supported_audit.json'),
        'official_FF_cache_caps_mapping_reused':ff_official,
        'original_native_steps':len(time),'saved_original_post_states':len(rows),
        'reset_phase':{**phase_baseline(time,feedback['phase_index'],labels.index('reset_open_search_2'),dt),'maximum_scheduled_duration_s':maximum,'completed':False},
        'first_fully_open_contact_native_index':bad,'first_fully_open_contact_post_label_time_s':float(time[bad]),
        'fully_open_contact_native_ticks':int(np.sum(opened & (feedback['right_robot_bolt_contact_count']>0))),
        'all_original_hard_guard_failure_indices':np.flatnonzero(~feedback['all_hard_guards_held']).tolist(),
        'preceding_fully_open_contact_free_ticks':len(fully_open_before_bad),
        'open_reset_commands':command_stats(commands,reset),
        'first_reset_force_cap_post_label_time_s':None if not len(clipping) else float(time[clipping[0]]),
        'final_original_command':command_final,'final_original_observation':original_final,
        'saved_reset_tracking_error_extrema':{'samples':len(samples),'maximum_position_error_norm_m':float(np.max(np.linalg.norm(p_error,axis=1))),'maximum_rotation_error_norm_rad':float(np.max(np.linalg.norm(R_error,axis=1)))},
        'frozen_release_calibration':release,
        'simple_geometric_context':{'actual_postintegration_jaw_aperture_m':aperture,'native_hex_head_across_flats_m':head_af,'hex_circumdiameter_m':2*head_af/np.sqrt(3),'centered_circumcircle_side_margin_m':(aperture-2*head_af/np.sqrt(3))/2,'scope':'Centered planar circumcircle aperture estimate only, excludes pad finite extent, actual tilt/translation and contact solve. Exact saved target/geometry analysis is a separate source-bound supplemental artifact.'},
        'actual_loaded_interior_contact_steps':int(np.sum(feedback['loaded_actual_interior_flank_contact_count']>0)),
        'regrasp_executed':bool(np.any(np.char.startswith(names.astype(str),'regrip_'))),
        'full_fresh_trajectory_qualified':False,'capture_or_open_reset_qualified':False,
        'source_original_acceptance_checks':original['acceptance_checks'],
        'scope':'Reuse closed official source-bound all-step FF/cache/observer/force audits; independent selected-array arithmetic for the sole actual unassisted-search recontact. Original false and unexecuted capture/regrasp flags retained. Large finite PD tracking error and force clipping precede recontact; they support a prospective control hypothesis, not sole causality or successful slower-reset prediction. No MuJoCo import, model build, forward/collision/force replay or integration.'}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('audit_dir',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=audit(a.run,a.audit_dir);a.output.write_text(json.dumps(r,indent=2,allow_nan=False)+'\n')
    print(json.dumps({'output':str(a.output),'original_native_steps':r['original_native_steps'],'first_contact':r['first_fully_open_contact_post_label_time_s'],'original_passed':r['original_report_passed']}))
