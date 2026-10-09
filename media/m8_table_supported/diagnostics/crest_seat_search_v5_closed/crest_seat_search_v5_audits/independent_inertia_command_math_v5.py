"""Independent recorded V5 command arithmetic; no MuJoCo import or live helper."""
from __future__ import annotations
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

HELPER_SHA = "0913f964c940d2fdf0bcbc92a0734c70c4868499439a4e5915e359a186dc17ff"
SHAPES = {
    'enabled': (), 'retained_native_state_time_s': (), 'retained_arm_velocity_rad_s': (6,),
    'arm_jacobian_position': (3,6), 'arm_jacobian_rotation': (3,6),
    'arm_bias_torque_Nm': (6,), 'native_drag_torque_Nm': (6,),
    'arm_jacobian_position_derivative': (3,6), 'arm_jacobian_rotation_derivative': (3,6),
    'requested_site_acceleration_world_m_s2': (3,), 'requested_angular_acceleration_world_rad_s2': (3,),
    'pd_wrench_world_N_Nm': (6,), 'ff_wrench_world_N_Nm': (6,),
    'combined_uncapped_wrench_world_N_Nm': (6,), 'capped_wrench_world_N_Nm': (6,),
    'motor_uncapped_torques_Nm': (6,), 'motor_torques_Nm': (6,),
    'cartesian_force_clipped': (), 'cartesian_torque_clipped': (), 'motor_clipped': (6,),
    'mass_aa': (6,6), 'hybrid_jacobian5': (5,6), 'projected_world_jdot_qdot5': (5,),
    'desired_world_acceleration5': (5,), 'mobility5': (5,5), 'transverse_basis_world': (3,2),
    'mass_condition': (), 'mobility_condition': (), 'minimum_mobility_eigenvalue': (),
    'scheduled_angular_velocity_world_rad_s': (3,), 'scheduled_lever_world_m': (3,),
    'command_time_s': (), 'time': (), 'phase_index': (), 'postintegration_arm_velocity_rad_s': (6,)}


def require(ok, message):
    if not ok:
        raise ValueError(message)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for b in iter(lambda:f.read(1024*1024), b''):
            h.update(b)
    return h.hexdigest()


def validate_schema(a):
    require(set(a)==set(SHAPES), 'Inertia columns differ from exact original schema')
    n=len(a['time'])
    for key,shape in SHAPES.items():
        require(a[key].shape==(n,)+shape and np.isfinite(a[key]).all(), 'Wrong inertia shape/finite values: '+key)
        if key in {'enabled','cartesian_force_clipped','cartesian_torque_clipped','motor_clipped'}:
            require(a[key].dtype.kind=='b', 'Inertia flags must be actual booleans')
    require(a['phase_index'].dtype.kind in 'iu' and np.all(a['phase_index']>=0), 'Inertia phase indices must be nonnegative integers')


def capped(v, limit):
    norm=np.linalg.norm(v,axis=-1,keepdims=True)
    return v*(limit/np.maximum(norm,limit))


def independent_solve(M,jp,jr,jpd,jrd,qdot,basis,linear,angular):
    """Batched SI-mixed instantaneous five-axis equations, with no regularization."""
    require(np.allclose(M,np.swapaxes(M,-1,-2),rtol=0.,atol=1e-10), 'Recorded mass is not symmetric')
    M=(M+np.swapaxes(M,-1,-2))/2
    eigen=np.linalg.eigvalsh(M)
    require(np.all(eigen>0), 'Recorded mass is not positive definite')
    require(np.allclose(np.swapaxes(basis,-1,-2)@basis,np.eye(2),rtol=0.,atol=1e-10), 'Transverse basis is not orthonormal')
    J=np.concatenate((np.swapaxes(basis,-1,-2)@jp,jr),axis=-2)
    bias=np.concatenate((np.einsum('nki,nij,nj->nk',np.swapaxes(basis,-1,-2),jpd,qdot),np.einsum('nij,nj->ni',jrd,qdot)),axis=-1)
    desired=np.concatenate((np.einsum('nki,ni->nk',np.swapaxes(basis,-1,-2),linear),angular),axis=-1)
    mobility=J@np.linalg.solve(M,np.swapaxes(J,-1,-2))
    mobility=(mobility+np.swapaxes(mobility,-1,-2))/2
    geig=np.linalg.eigvalsh(mobility)
    require(np.all(geig>0), 'Hybrid mobility is not positive definite/full rank')
    mass_cond=np.linalg.cond(M)
    mobility_cond=np.linalg.cond(mobility)
    require(np.all(mass_cond<=1e12) and np.all(mobility_cond<=1e12), 'Declared numerical condition bound exceeded')
    projected=np.linalg.solve(mobility,(desired-bias)[...,None])[...,0]
    world=np.concatenate((np.einsum('nij,nj->ni',basis,projected[...,:2]),projected[...,2:]),axis=-1)
    return {'hybrid_jacobian5':J,'projected_world_jdot_qdot5':bias,'desired_world_acceleration5':desired,
            'mobility5':mobility,'mass_condition':mass_cond,'mobility_condition':mobility_cond,
            'minimum_mobility_eigenvalue':geig[...,0],'ff_wrench_world_N_Nm':world}


def verify_cache(a, initial_qdot, dt):
    n=len(a['time'])
    require(np.allclose(a['command_time_s'],a['time']-dt,rtol=0.,atol=1e-10), 'Command timestamp does not precede saved time by dt')
    expected_time=np.maximum(a['time']-2*dt,0.)
    require(np.allclose(a['retained_native_state_time_s'],expected_time,rtol=0.,atol=1e-10), 'Retained native timestamp differs')
    require(np.array_equal(a['retained_arm_velocity_rad_s'][:min(n,2)],np.tile(initial_qdot,(min(n,2),1))), 'Initialized retained velocity differs')
    require(np.array_equal(a['retained_arm_velocity_rad_s'][2:],a['postintegration_arm_velocity_rad_s'][:-2]), 'Retained qdot is not the original preprevious-step qdot')
    return {'cross_row_velocity_links_checked':max(n-2,0),'initialized_rows_checked':min(n,2),
            'maximum_timestamp_residual_s':float(np.max(np.abs(a['retained_native_state_time_s']-expected_time))) if n else None}


def xml_arm_indices(tree):
    """Resolve native XML joint/actuator order without compiling or solving a model."""
    dofs={};cursor=0
    damping={}
    for e in tree.find('worldbody').iter():
        if e.tag not in {'joint','freejoint'}:
            continue
        kind='free' if e.tag=='freejoint' else e.get('type','hinge')
        dofs[e.get('name')]=cursor
        damping[e.get('name')]=float(e.get('damping','0'))
        cursor+={'free':6,'ball':3}.get(kind,1)
    actuators=list(tree.find('actuator'))
    names={e.get('name'):i for i,e in enumerate(actuators)}
    joints=['right_joint'+str(i) for i in range(1,7)]
    ids=[names['right_servo'+str(i)] for i in range(1,7)]
    limits=np.array([np.fromstring(actuators[i].get('ctrlrange'),sep=' ') for i in ids])
    require(np.array_equal(-limits[:,0],limits[:,1]), 'Native right motor bounds are asymmetric')
    return np.array([dofs[j] for j in joints]),np.array(ids),limits[:,1],np.array([damping[j] for j in joints])


def verify_inertia_ledger(run,metadata,feedback,labels,table,saved_time,qvel,controls,rows,dt,declaration):
    run=Path(run)
    path=run/'robot_inertia_command_history.npz'
    declared=metadata['robot_inertia_command_history']
    require(digest(path)==declared['sha256'], 'Closed inertia ledger bytes changed')
    require(digest(run/'robot_inertia_source.py')==HELPER_SHA and digest(run/'recorded_sources/yam_twin/robot_inertia_feedforward_v5.py')==HELPER_SHA, 'Whole archived inertia helper changed')
    with np.load(path,allow_pickle=False) as z:
        a={k:z[k].copy() for k in z.files if k!='metadata_json'}
        identity=json.loads(str(z['metadata_json']))
    validate_schema(a)
    n=len(a['time'])
    require(n==declared['executed_native_commands']==len(feedback['time']) and list(a)==declared['columns'], 'Executed inertia prefix/schema differs')
    for k in ('model_fingerprint','controller_sha256','runtime'):
        require(identity[k]==metadata[k], 'Inertia native identity differs: '+k)
    require(identity['phase_labels']==labels and identity['timestep_s']==dt and identity['inertia_helper_source_sha256']==HELPER_SHA, 'Inertia helper/phase/timestep identity differs')
    require(np.array_equal(a['time'],feedback['time']) and np.array_equal(a['phase_index'],feedback['phase_index']), 'Inertia/native feedback timing differs')
    require(np.all(a['enabled']) and np.all(feedback['right_axial_float_active']), 'Closed five-axis controller/axial-float scope differs')
    tree=ET.parse(run/'scene.xml').getroot()
    dofs,motors,caps,drag=xml_arm_indices(tree)
    parent=Path(declaration['parent'])
    with np.load(parent,allow_pickle=False) as z:
        initial=z['qvel'][declaration['parent_native_checkpoint_index'],dofs].copy()
    cache=verify_cache(a,initial,dt)
    residuals={}
    def match(key,expected,actual=None):
        actual=a[key] if actual is None else actual
        require(np.allclose(actual,expected,rtol=2e-9,atol=3e-10), 'Independent inertia equation mismatch: '+key)
        residuals[key]=max(residuals.get(key,0.),float(np.max(np.abs(actual-expected))))
    for start in range(0,n,2048):
        sl=slice(start,min(n,start+2048))
        c={k:v[sl] for k,v in a.items()}
        axis=np.cross(c['transverse_basis_world'][:,:,0],c['transverse_basis_world'][:,:,1])
        omega=feedback['desired_independent_angular_speed_rad_s'][sl,None]*axis
        alpha=feedback['desired_independent_angular_acceleration_rad_s2'][sl,None]*axis
        lever=c['scheduled_lever_world_m']
        orbital=-np.cross(alpha,lever)-np.cross(omega,np.cross(omega,lever))
        match('scheduled_angular_velocity_world_rad_s',omega,c['scheduled_angular_velocity_world_rad_s'])
        match('requested_angular_acceleration_world_rad_s2',alpha,c['requested_angular_acceleration_world_rad_s2'])
        match('requested_site_acceleration_world_m_s2',orbital,c['requested_site_acceleration_world_m_s2'])
        predicted=independent_solve(c['mass_aa'],c['arm_jacobian_position'],c['arm_jacobian_rotation'],c['arm_jacobian_position_derivative'],c['arm_jacobian_rotation_derivative'],c['retained_arm_velocity_rad_s'],c['transverse_basis_world'],orbital,alpha)
        for key,expected in predicted.items():
            match(key,expected,c[key])
        require(np.max(np.abs(np.einsum('ni,ni->n',predicted['ff_wrench_world_N_Nm'][:,:3],axis)))<1e-9, 'FF applies an excluded axial force')
        match('native_drag_torque_Nm',c['retained_arm_velocity_rad_s']*drag,c['native_drag_torque_Nm'])
        total=c['pd_wrench_world_N_Nm']+predicted['ff_wrench_world_N_Nm']
        limited=np.c_[capped(total[:,:3],8.),capped(total[:,3:],2.)]
        mapped=c['arm_bias_torque_Nm']+c['native_drag_torque_Nm']+np.einsum('nij,ni->nj',c['arm_jacobian_position'],limited[:,:3])+np.einsum('nij,ni->nj',c['arm_jacobian_rotation'],limited[:,3:])
        clipped=np.clip(mapped,-caps,caps)
        match('combined_uncapped_wrench_world_N_Nm',total,c['combined_uncapped_wrench_world_N_Nm'])
        match('capped_wrench_world_N_Nm',limited,c['capped_wrench_world_N_Nm'])
        match('motor_uncapped_torques_Nm',mapped,c['motor_uncapped_torques_Nm'])
        match('motor_torques_Nm',clipped,c['motor_torques_Nm'])
        require(np.array_equal(c['cartesian_force_clipped'],np.linalg.norm(total[:,:3],axis=1)>8.) and np.array_equal(c['cartesian_torque_clipped'],np.linalg.norm(total[:,3:],axis=1)>2.) and np.array_equal(c['motor_clipped'],np.abs(mapped)>caps), 'Original combined-cap clipping flags differ')
    match('same_command_native_wrench',a['capped_wrench_world_N_Nm'],feedback['right_command_wrench_N_Nm'])
    match('same_command_native_motor',a['motor_torques_Nm'],feedback['right_motor_torques_Nm'])
    ids=np.searchsorted(a['time'],saved_time)
    require(np.array_equal(a['time'][ids],saved_time), 'Saved command rows are not actual native ticks')
    match('sampled_actual_motor_ctrl',a['motor_torques_Nm'][ids],controls[:,motors])
    require(np.array_equal(a['postintegration_arm_velocity_rad_s'][ids],qvel[:,dofs]), 'Raw postintegration arm velocity differs from saved native qvel')
    for i,row in zip(ids,rows):
        record=row['inertia_control']
        require(set(record)==set(SHAPES), 'Saved inertia scalar/vector schema differs')
        for k in SHAPES:
            require(np.array_equal(np.asarray(record[k]),a[k][i]), 'Saved command differs from original dense command: '+k)
    return {'observer':'independent-recorded-hybrid5d-command-math-v5','reader_source_sha256':digest(__file__),
        'original_ledger_sha256':digest(path),'executed_native_commands_checked':n,'original_columns_checked':len(a),
        'every_completed_five_axis_equation_matches':True,'every_combined_cap_and_native_mapping_matches':True,
        'same_command_feedback_and_sampled_ctrl_match':True,'coherent_retained_velocity':cache,
        'maximum_absolute_equation_residuals':residuals,
        'peak_ff_force_N':float(np.max(np.linalg.norm(a['ff_wrench_world_N_Nm'][:,:3],axis=1))),
        'peak_ff_torque_Nm':float(np.max(np.linalg.norm(a['ff_wrench_world_N_Nm'][:,3:],axis=1))),
        'peak_combined_uncapped_force_N':float(np.max(np.linalg.norm(a['combined_uncapped_wrench_world_N_Nm'][:,:3],axis=1))),
        'peak_combined_uncapped_torque_Nm':float(np.max(np.linalg.norm(a['combined_uncapped_wrench_world_N_Nm'][:,3:],axis=1))),
        'combined_force_clipped_native_ticks':int(np.sum(a['cartesian_force_clipped'])),
        'combined_torque_clipped_native_ticks':int(np.sum(a['cartesian_torque_clipped'])),
        'any_motor_clipped_native_ticks':int(np.sum(np.any(a['motor_clipped'],axis=1))),
        'maximum_mass_condition_SI':float(np.max(a['mass_condition'])),
        'maximum_hybrid_mobility_condition_SI':float(np.max(a['mobility_condition'])),
        'native_motor_caps_Nm':caps.tolist(),
        'timing_scope':'Commands execute at t-dt using retained M/J/Jdot/velocity at t-2dt after startup. Original force geometry is t-dt; saved q/v is t. First two retained velocity rows equal the exact initialized cold qvel. No native force replay.',
        'mechanical_scope':'Original recorded six-arm M_aa at actual finger configuration, held-finger acceleration approximation; downstream rigid inertia included, independent finger acceleration and coupled object/contact inverse dynamics excluded. Instantaneous world 2-transverse/3-angular projections; no axial inversion or prescribed lead.',
        'PD_scope':'Recorded original PD wrench is an input to this independent mapping audit, not recomputed from unarchived every-tick target/pose-error fields. Hash-bound source specifies retained-velocity damping and axial projection.'}
