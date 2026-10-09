"""Closed-run planned reset reachability: compile archived geometry, FK/IK only.

Never calls forward, collision, contacts, force solves, dynamics or plugin build.
Actual saved post-state is separate from original retained command-time errors.
"""
from pathlib import Path
import ast
import hashlib
import importlib.util
import itertools
import json
import math
import shutil
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path('/workspace/astra-r2s')
RUN = ROOT/'outputs/m8_table_supported/full_c2_inertia_v2'
OUT = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT),str(ROOT/'outputs/m8_table_supported')]
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
AFTER = '481b930e3dadaddd54240ea9548f5decdb98abfdad9c41db05871c84cf5ccaa3'
PLUGIN = ROOT/'thread_lab/plugins/libm8_sdf.so'
PLUGIN_SHA = '53571638b1f6146e1dfd297e8dd5f750bb19c1efc70743180f40b94649489e18'
CORE = Path('/workspace/.venvs/m8-contact/lib/python3.12/site-packages/mujoco/libmujoco.so.3.15.0')
CORE_SHA = '58039d439c6504448aafd0a4d5b655c8a0d3bf1d77123ac7e0733cd078367433'

def copied_ast(path,names,namespace):
    tree=ast.parse(path.read_text())
    nodes=[n for n in tree.body if isinstance(n,(ast.FunctionDef,ast.ClassDef)) and n.name in names]
    assert {n.name for n in nodes}==set(names)
    exec(compile(ast.Module(nodes,type_ignores=[]),str(path),'exec'),namespace)
    return namespace

def main():
    assert (RUN/'run_publication_identity_after.json').is_file(), 'Refuse LIVE'
    assert sha(RUN/'run_publication_identity_after.json')==AFTER
    after=json.loads((RUN/'run_publication_identity_after.json').read_text())
    assert after['native_exit_code']==1 and after['producer_commit']=='da69a9cd44a8312cc7b97365faf5e09c27a646e2'
    original_sources=after['source_hashes_before']
    assert original_sources==after['source_hashes_after'] and after['source_hashes_unchanged'] is True
    assert len(original_sources)==74
    assert sha(PLUGIN)==PLUGIN_SHA and sha(CORE)==CORE_SHA
    import numpy as np
    import mujoco
    from scipy.spatial.transform import Rotation
    from thread_lab.runtime import engine_info
    from yam_twin.kinematics import ArmIK
    original_calls={}
    # Python entry points are prohibited, including geometry collision queries.
    forbidden=('mj_forward','mj_step','mj_step1','mj_step2','mj_collision','mj_geomDistance',
        'mj_contactForce','mj_fwdPosition','mj_fwdVelocity','mj_fwdActuation','mj_fwdAcceleration','mj_fwdConstraint')
    for name in forbidden:
        if hasattr(mujoco,name):
            def reject(*args,_name=name,**kwargs):
                original_calls[_name]=original_calls.get(_name,0)+1
                raise RuntimeError('Forbidden native operation '+_name)
            setattr(mujoco,name,reject)
    runtime=engine_info()
    assert len(runtime['libraries'])==1 and runtime['libraries'][0]['sha256']==CORE_SHA
    assert runtime['thread_plugin_source_sha256']=='1c8b5207c5f6c141cc034983ce76c6c1e9cca4114d16e17a4496b94cb637b42a'
    with np.load(RUN/'insertion_trace.npz',allow_pickle=False) as z:
        times=z['time'].copy(); q=z['qpos'].copy(); qv=z['qvel'].copy()
        samples=json.loads(str(z['info_json'])); report=json.loads(str(z['metadata_json']))
    assert report==json.loads((RUN/'insertion_validation.json').read_text())
    assert report['passed'] is False and report['aborted']['phase']=='reset_open_search_2'
    inputs=[RUN/'insertion_trace.npz',RUN/'insertion_validation.json',RUN/'scene.xml',
        RUN/'run_publication_identity.json',RUN/'run_publication_identity_after.json',
        RUN/'controller_source.py',RUN/'recorded_sources/yam_twin/kinematics.py',
        RUN/'recorded_sources/yam_twin/m8_supported_start.py',
        ROOT/'yam_twin/kinematics.py',ROOT/'yam_twin/m8_supported_start.py',
        ROOT/'scripts/audit_m8_insertion_trace.py',
        ROOT/'outputs/m8_table_supported/render_full_c2_inertia_v2_closed.py',PLUGIN,CORE]
    before={str(p):sha(p) for p in inputs}
    assert sha(ROOT/'yam_twin/kinematics.py')==sha(RUN/'recorded_sources/yam_twin/kinematics.py')
    assert sha(ROOT/'yam_twin/m8_supported_start.py')==sha(RUN/'recorded_sources/yam_twin/m8_supported_start.py')
    for relative in ('yam_twin/kinematics.py','yam_twin/m8_supported_start.py','scripts/audit_m8_insertion_trace.py'):
        assert sha(ROOT/relative)==original_sources[relative]
    assert sha(RUN/'controller_source.py')==report['controller_module_sha256']==original_sources['yam_twin/m8_supported_simulation.py']
    # Reuse the reviewed archived compiler, never recorded_model/build_plugin.
    helper_path=ROOT/'outputs/m8_table_supported/render_full_c2_inertia_v2_closed.py'
    spec=importlib.util.spec_from_file_location('reviewed_closed_renderer',helper_path)
    helper=importlib.util.module_from_spec(spec);spec.loader.exec_module(helper)
    helper.mujoco=mujoco
    mujoco.mj_loadPluginLibrary(str(PLUGIN))
    model,model_identity=helper.compile_archived_model(RUN/'insertion_trace.npz',report)
    native_runtime_after_compile=engine_info()
    assert native_runtime_after_compile==runtime
    ns=copied_ast(RUN/'recorded_sources/yam_twin/m8_supported_start.py',
        ('_finite_vector','_proper_rotation','_rz','frozen_open_target'),{'np':np})
    target_fn=ns['frozen_open_target']
    event=next(x for x in report['physical_motion_events'] if x['event']=='Frozen actual release calibration' and x['phase']=='release_search_2')
    cal=event['calibration']
    reset_indices=[i for i,s in enumerate(samples) if s['phase']=='reset_open_search_2']
    release_index=int(np.argmin(abs(times-event['time_s'])))
    reset_index=reset_indices[0]-1
    end_index=len(times)-1
    ik_template=ArmIK(model,'right')
    qadr=ik_template.qadr.copy()
    def geometry(index):
        data=mujoco.MjData(model);data.qpos[:]=q[index];data.qvel[:]=qv[index]
        mujoco.mj_kinematics(model,data)
        return data
    hole_id=model.body('female_frame').id;bolt_id=model.body('male_bolt').id
    site_id=model.site('right_grasp_site').id
    def target(data,clock):
        return target_fn(data.xpos[hole_id],data.xmat[hole_id].reshape(3,3),np.zeros(6),
            cal['tool_position_relative_hole_m'],cal['tool_rotation_relative_hole'],
            cal['head_offset_in_tool_m'],clock-cal['release_clock_rad'],0.)
    def residual_record(ik,t,clock):
        p,r=ik.pose();margins=np.minimum(ik.q-ik.bounds[0],ik.bounds[1]-ik.q)
        k=int(np.argmin(margins))
        return {'clock_rad':float(clock),'target_position_world_m':t['position_m'].tolist(),
            'target_rotation_world':t['rotation'].tolist(),'arm_q_rad':ik.q.tolist(),
            'position_residual_m':float(np.linalg.norm(p-t['position_m'])),
            'orientation_residual_rad':float(np.linalg.norm(Rotation.from_matrix(t['rotation']@r.T).as_rotvec())),
            'minimum_joint_margin_rad':float(margins[k]),'limiting_joint':f'right_joint{k+1}',
            'joint_margins_rad':margins.tolist()}
    data_end=geometry(end_index)
    endpoint_ik=ArmIK(model,'right'); endpoint_ik.q=q[end_index,qadr].copy()
    full_goal=target(data_end,cal['reset_clock_rad'])
    endpoint_ik.solve(full_goal['position_m'],full_goal['rotation'],thorough=False)
    first={'scope':'Immediate geometry-only full remaining-goal IK from actual saved failed six-joint state; local bounded solution only',
        'goal':residual_record(endpoint_ik,full_goal,cal['reset_clock_rad'])}
    (OUT/'first_endpoint_result.json').write_text(json.dumps(first,indent=2))
    print('FIRST_ENDPOINT',json.dumps(first),flush=True)
    # Complete release-to-minus-pi sweep, independently warm-started at actual release and reset-start.
    sweeps=[]
    for label,frame_index,seed_index in [('release_frame_actual_release_seed',release_index,release_index),
            ('reset_start_frame_actual_reset_seed',reset_index,reset_index),
            ('abort_frame_actual_reset_seed',end_index,reset_index)]:
        data=geometry(frame_index);ik=ArmIK(model,'right');ik.q=q[seed_index,qadr].copy()
        rows=[]
        for clock in np.linspace(cal['release_clock_rad'],cal['reset_clock_rad'],65):
            t=target(data,float(clock));ik.solve(t['position_m'],t['rotation'],thorough=False)
            rows.append(residual_record(ik,t,clock))
        summary={'label':label,'hole_frame_saved_index':frame_index,'hole_frame_post_state_time_s':float(times[frame_index]),
            'seed_saved_index':seed_index,'seed_post_state_time_s':float(times[seed_index]),'samples':65,
            'maximum_position_residual_m':max(x['position_residual_m'] for x in rows),
            'maximum_orientation_residual_rad':max(x['orientation_residual_rad'] for x in rows),
            'minimum_joint_margin_rad':min(x['minimum_joint_margin_rad'] for x in rows),
            'minimum_margin_sample':min(rows,key=lambda x:x['minimum_joint_margin_rad']),
            'maximum_joint_step_rad':float(np.max(np.abs(np.diff(np.asarray([x['arm_q_rad'] for x in rows]),axis=0)))),
            'rows':rows}
        sweeps.append(summary)
        print('SWEEP',json.dumps({k:v for k,v in summary.items() if k not in ('rows','minimum_margin_sample')}),flush=True)
    end=samples[-1];clock=end['desired_independent_clock_rad'];actual_target=target(data_end,clock)
    tool_p=data_end.site_xpos[site_id].copy();tool_r=data_end.site_xmat[site_id].reshape(3,3).copy()
    actual_head=data_end.xpos[bolt_id]+data_end.xmat[bolt_id].reshape(3,3)@np.array([0,0,-report['scene_config']['head_height']/2])
    original_error=np.asarray(end['right_position_error_m'])
    endpoint={'saved_index':end_index,'post_state_time_s':float(times[end_index]),
        'original_command_retained_geometry_time_s':float(times[end_index]-2*model.opt.timestep),
        'original_solved_force_geometry_time_s':float(times[end_index]-model.opt.timestep),
        'actual_tool_position_world_m':tool_p.tolist(),'actual_tool_rotation_world':tool_r.tolist(),
        'actual_head_position_world_m':actual_head.tolist(),'actual_head_in_tool_m':(tool_r.T@(actual_head-tool_p)).tolist(),
        'hole_position_world_m':data_end.xpos[hole_id].tolist(),'hole_rotation_world':data_end.xmat[hole_id].reshape(3,3).tolist(),
        'original_clock_rad':clock,'exact_frozen_target_in_saved_post_hole_frame':{k:v.tolist() for k,v in actual_target.items()},
        'geometry_only_post_state_position_error_vector_m':(actual_target['position_m']-tool_p).tolist(),
        'geometry_only_post_state_position_error_m':float(np.linalg.norm(actual_target['position_m']-tool_p)),
        'geometry_only_post_state_orientation_error_rad':float(np.linalg.norm(Rotation.from_matrix(actual_target['rotation']@tool_r.T).as_rotvec())),
        'original_retained_command_position_error_vector_m':original_error.tolist(),
        'original_retained_command_position_error_m':float(np.linalg.norm(original_error)),
        'original_retained_command_rotation_error_vector_rad':end['right_rotation_error_rad'],
        'original_retained_command_rotation_error_rad':float(np.linalg.norm(end['right_rotation_error_rad']))}
    lj=model.joint('right_left_finger').qposadr[0];rj=model.joint('right_right_finger').qposadr[0]
    endpoint['actual_aperture_postintegration_m']=float(q[end_index,lj]-q[end_index,rj]-2*report['scene_config']['base']['pad_inner_offset'])
    # Analytic convex SAT only: a positive axis gap certifies separation. Negative overlap is not a signed native distance.
    xml=ET.fromstring((RUN/'scene.xml').read_bytes())
    head_mesh=next(x for x in xml.findall('asset/mesh') if x.get('name')=='bolt_head_shape')
    head_local=np.fromstring(head_mesh.get('vertex'),sep=' ').reshape(-1,3)
    head_world=data_end.xpos[bolt_id]+head_local@data_end.xmat[bolt_id].reshape(3,3).T
    pad_results=[]
    for name in ('right_m8_pad_left','right_m8_pad_right'):
        gid=model.geom(name).id;R=data_end.geom_xmat[gid].reshape(3,3);p=data_end.geom_xpos[gid]
        local=(head_world-p)@R;half=model.geom_size[gid]
        gaps=np.maximum(np.min(local,axis=0)-half,-half-np.max(local,axis=0))
        pad_results.append({'pad':name,'head_axis_projection_min_in_pad_m':local.min(axis=0).tolist(),
            'head_axis_projection_max_in_pad_m':local.max(axis=0).tolist(),'pad_half_size_m':half.tolist(),
            'separation_gaps_on_pad_axes_m':gaps.tolist(),'certified_separated_by_pad_axis':bool(np.max(gaps)>0),
            'scope':'Conservative pad-axis projection; no separating axis here is inconclusive, not a collision/penetration measurement.'})
    endpoint['conservative_head_pad_axis_projections']=pad_results
    assert {str(p):sha(p) for p in inputs}==before
    assert original_calls=={}
    identity={'producer':after['producer_commit'],'before_after_input_bytes_unchanged':True,
        'original_source_map_count':len(original_sources),'original_source_map':original_sources,
        'input_sha256':before,'model_identity':model_identity,'runtime_actual':runtime,
        'explicit_replay_plugin_binary_sha256':PLUGIN_SHA,
        'replay_plugin_binary_scope':'Explicit supplied existing replay binary anchor; original run did not report a plugin binary SHA. Original plugin SOURCE SHA is recorded independently.',
        'script_sha256':sha(__file__),'forbidden_native_calls':original_calls,
        'native_calls_scope':['MjSpec compile exact archived model/assets','MjData allocation','mj_loadPluginLibrary existing explicit binary','mj_kinematics only'],
        'calibration_event':event,'release_seed_saved_state_offset_from_event_s':float(times[release_index]-event['time_s']),
        'frame_scope':'Frozen exact release calibration from original event; saved post-state hole frames held fixed separately across each full 65-clock sweep. No future moving-hole prediction. Actual event calibration originated retained original geometry t−dt at phase boundary; saved post-state seeds are labeled.',
        'scope':'Finite-joint kinematic reachability local IK of planned frozen OPEN orbit. No collision/contact/force solve/dynamics, trajectory feasibility/actuation proof, source edits, interpolation, object writes, capture or physics success claim.'}
    result={'identity':identity,'immediate_endpoint_ik':first,'complete_sweeps':sweeps,'actual_abort_endpoint_geometry':endpoint}
    (OUT/'reset_reachability_report.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    archive=OUT/'source_archive';archive.mkdir(exist_ok=True)
    for p in inputs:
        if p.suffix=='.py':
            destination=archive/(p.parent.name+'__'+p.name)
            if destination.exists(): assert sha(destination)==sha(p)
            else: shutil.copyfile(p,destination)
    print('FINAL_ENDPOINT',json.dumps(endpoint),flush=True)
    print('REPORT',str(OUT/'reset_reachability_report.json'),flush=True)

if __name__=='__main__':main()
