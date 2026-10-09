"""Strict native ledgers and orderly pre-step command rejection.

Every row describes an actual solved step. Rejected commands are retained
separately from the previous applied controls and never manufacture a tick.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from .m8_robot_inertia import InertiaFeedforwardError
from .m8_insertion_simulation import PadLoadHistory

class SupportedControlRejection(Exception):
    def __init__(self, report, buffers, checkpoint):
        super().__init__(report['aborted']['reason'])
        self.report, self.buffers, self.checkpoint = report, buffers, checkpoint

def flatten_inertia_record(record):
    """Disabled rows retain PD inputs; absent inverse/Jdot fields are explicit."""
    inertia = record['inertia']
    result = {k:np.asarray(v).copy() for k,v in record.items() if k != 'inertia'}
    if inertia is None:
        result['arm_jacobian_position_derivative'] = np.zeros_like(result['arm_jacobian_position'])
        result['arm_jacobian_rotation_derivative'] = np.zeros_like(result['arm_jacobian_rotation'])
    result['inertia_inputs_present'] = np.asarray(inertia is not None)
    result['jacobian_derivatives_present'] = np.asarray(inertia is not None)
    mapping = {'mass_aa':('mass_aa',(6,6)), 'hybrid_jacobian5':('jacobian5',(5,6)),
        'projected_world_jdot_qdot5':('projected_world_jdot_qdot5',(5,)),
        'desired_world_acceleration5':('desired_world_acceleration5',(5,)),
        'mobility5':('mobility5',(5,5)), 'transverse_basis_world':('transverse_basis_world',(3,2)),
        'mass_condition':('mass_condition',()), 'mobility_condition':('mobility_condition',()),
        'minimum_mobility_eigenvalue':('minimum_mobility_eigenvalue',())}
    for key,(original,shape) in mapping.items():
        result[key] = np.asarray(inertia[original]).copy() if inertia is not None else np.zeros(shape)
    if bool(record['enabled']) != (inertia is not None):
        raise InertiaFeedforwardError('Active inertia command must contain its native inverse inputs')
    return result

def force_arrays(rows, columns):
    shapes={'table_wrench_world_at_block_origin_N_Nm':(6,),
        'left_hand_wrench_world_at_block_origin_N_Nm':(6,), 'pad_normal_force_N':(2,),
        'block_position_m':(3,), 'block_rotation_matrix':(3,3),
        'thread_wrench_on_bolt_world_N_Nm':(6,), 'hand_wrench_on_bolt_world_N_Nm':(6,),
        'right_pad_normal_force_N':(2,), 'right_command_wrench_N_Nm':(6,),
        'left_command_wrench_N_Nm':(6,), 'right_motor_torques_Nm':(6,),
        'left_motor_torques_Nm':(6,)}
    bools={'external_drive_zero','supported_task_active','fully_open_unassisted',
        'right_grasp_guard_active','right_axial_float_active','weight_window_ready',
        'open_weight_window_ready','all_hard_guards_held','weight_observation_valid',
        'open_observation_valid','robot_yaw_brake_active'}
    ints={'phase_index','table_contact_candidates','table_loaded_contacts'}
    if any(len(row) != len(columns) for row in rows):
        raise ValueError('Original force row does not match declared schema')
    for row in rows:
        for i,name in enumerate(columns):
            value=np.asarray(row[i])
            if value.shape != shapes.get(name,()) or not np.isfinite(value).all():
                raise ValueError(f'Invalid original force shape/value for {name}')
            if name in bools and not np.isin(value,[False,True]).all():
                raise ValueError(f'Invalid original force boolean {name}')
            if name in ints or name.endswith('_count') or name.endswith('_contacts'):
                if np.any(value < 0) or not np.equal(value,np.floor(value)).all():
                    raise ValueError(f'Invalid original force count {name}')
    values = {name:np.asarray([row[i] for row in rows],
        dtype=(bool if name in bools else np.int64 if name in ints or name.endswith('_count') or name.endswith('_contacts') else float))
        .reshape((len(rows),)+shapes.get(name,())) for i,name in enumerate(columns)}
    if any(not np.isfinite(value).all() for value in values.values()):
        raise ValueError('Original force rows must remain finite')
    return values


def write_inertia_ledger(output, records, metadata):
    shapes={'enabled':(), 'inertia_inputs_present':(), 'jacobian_derivatives_present':(), 'retained_native_state_time_s':(), 'retained_arm_velocity_rad_s':(6,),
        'arm_jacobian_position':(3,6),'arm_jacobian_rotation':(3,6),
        'arm_bias_torque_Nm':(6,),'native_drag_torque_Nm':(6,),
        'arm_jacobian_position_derivative':(3,6),'arm_jacobian_rotation_derivative':(3,6),
        'requested_site_acceleration_world_m_s2':(3,), 'requested_angular_acceleration_world_rad_s2':(3,),
        'pd_wrench_world_N_Nm':(6,), 'ff_wrench_world_N_Nm':(6,),
        'combined_uncapped_wrench_world_N_Nm':(6,), 'capped_wrench_world_N_Nm':(6,),
        'motor_uncapped_torques_Nm':(6,), 'motor_torques_Nm':(6,),
        'cartesian_force_clipped':(), 'cartesian_torque_clipped':(), 'motor_clipped':(6,),
        'mass_aa':(6,6), 'hybrid_jacobian5':(5,6), 'projected_world_jdot_qdot5':(5,),
        'desired_world_acceleration5':(5,), 'mobility5':(5,5), 'transverse_basis_world':(3,2),
        'mass_condition':(), 'mobility_condition':(), 'minimum_mobility_eigenvalue':(),
        'scheduled_angular_velocity_world_rad_s':(3,), 'scheduled_lever_world_m':(3,),
        'command_time_s':(), 'time':(), 'phase_index':(), 'postintegration_arm_velocity_rad_s':(6,)}
    if any(set(record) != set(shapes) for record in records):
        raise ValueError('Executed inertia record columns differ from declared stable schema')
    keys=list(shapes)
    bools={'enabled','inertia_inputs_present','jacobian_derivatives_present','cartesian_force_clipped','cartesian_torque_clipped','motor_clipped'}
    for record in records:
        for key,shape in shapes.items():
            value=np.asarray(record[key])
            if value.shape != shape or not np.isfinite(value).all():
                raise ValueError(f'Invalid executed command shape/value for {key}')
            if key in bools and not np.isin(value,[False,True]).all():
                raise ValueError(f'Invalid executed command boolean {key}')
        if bool(record['enabled']) != bool(record['inertia_inputs_present']) or bool(record['enabled']) != bool(record['jacobian_derivatives_present']):
            raise ValueError('Executed inertia presence flags must match enablement')
        if not record['enabled']:
            for key in ('mass_aa','hybrid_jacobian5','projected_world_jdot_qdot5','desired_world_acceleration5','mobility5','transverse_basis_world','mass_condition','mobility_condition','minimum_mobility_eigenvalue','arm_jacobian_position_derivative','arm_jacobian_rotation_derivative','ff_wrench_world_N_Nm','requested_site_acceleration_world_m_s2','requested_angular_acceleration_world_rad_s2','scheduled_angular_velocity_world_rad_s','scheduled_lever_world_m'):
                if np.any(np.asarray(record[key]) != 0):
                    raise ValueError(f'Disabled uncomputed inertia field {key} must be explicit zero placeholder')
    values={key:np.asarray([row[key] for row in records],
        dtype=bool if key in bools else np.int64 if key=='phase_index' else float)
        .reshape((len(records),)+shape) for key,shape in shapes.items()}
    if any(not np.isfinite(value).all() for value in values.values()):
        raise ValueError('Executed robot command ledger must remain finite')
    path=output/'robot_inertia_command_history.npz'
    np.savez_compressed(path,**values,metadata_json=np.asarray(json.dumps({
        'model_fingerprint':metadata['model_fingerprint'],
        'controller_sha256':metadata['controller_sha256'],'runtime':metadata['runtime'],
        'timestep_s':metadata['scene_config']['base']['thread']['timestep'],
        'inertia_helper_source_sha256':metadata['robot_inertia_feedforward']['source_sha256'],
        'phase_labels':list(metadata['maximum_phase_plan'][i][0] for i in range(len(metadata['maximum_phase_plan']))),
        'scope':'All executed right commands. Disabled FF rows retain actual PD/J/bias/drag inputs, with zero placeholders and false presence flags for uncomputed M/Jdot/inverse fields. Active rows contain actual retained native M/J/Jdot/pre-solve velocity and independently scheduled acceleration used for finite robot control. Command state precedes force state by one native timestep; actual retained timestamps are authoritative. No original contact forces are reconstructed.'})))
    return {'filename':path.name,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
        'executed_native_commands':len(records),'columns':keys}

def preserve_control_rejection(output, rejection):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    metadata = rejection.report
    json.dumps(metadata, allow_nan=False)
    checkpoint = rejection.checkpoint
    if set(checkpoint) != {'time','qpos','qvel','controller','unapplied_rejected_controller'}:
        raise ValueError('Rejection checkpoint requires exact native state and separate previous/attempted controls')
    if np.asarray(checkpoint['time']).shape != () or not np.isfinite(checkpoint['time']):
        raise ValueError('Rejection checkpoint time must be a finite scalar')
    for key in ('qpos','qvel','controller','unapplied_rejected_controller'):
        value=np.asarray(checkpoint[key])
        if value.ndim != 1 or not np.isfinite(value).all():
            raise ValueError(f'Rejection checkpoint {key} must be a finite vector')
    if np.asarray(checkpoint['controller']).shape != np.asarray(checkpoint['unapplied_rejected_controller']).shape:
        raise ValueError('Previous and attempted controls must have identical native dimensions')
    counts=[len(rejection.buffers[key]) for key in ('support_rows','feedback_rows','inertia_rows')]
    if len(set(counts)) != 1 or counts[0] != metadata['original_native_steps']:
        raise ValueError('Rejection ledgers must cover the exact same executed native steps')
    if counts[0] and (abs(float(rejection.buffers['feedback_rows'][-1][0])-float(checkpoint['time']))>1e-10):
        raise ValueError('Last solved original force ledger must match rejection native post-state time')
    sample_counts=[len(rejection.buffers[key]) for key in ('times','positions','velocities','controls','rows')]
    if len(set(sample_counts)) != 1:
        raise ValueError('Sparse native states and original records must stay aligned')
    pad_counts=[len(rejection.buffers[key]) for key in ('left_pad_times','left_pad_forces','left_pad_phase_indices')]
    if len(set(pad_counts)) != 1:
        raise ValueError('Original left-pad rows must stay aligned')
    for filename, rows, columns in (
        ('table_support_force_history.npz', rejection.buffers['support_rows'], ORIGINAL_SUPPORT_COLUMNS),
        ('native_feedback_force_history.npz', rejection.buffers['feedback_rows'], ORIGINAL_FEEDBACK_COLUMNS)):
        values=force_arrays(rows,columns)
        observer_identity = ({'observer':'native-supported-block-load-window-v1'}
            if filename == 'table_support_force_history.npz' else
            {'observer':'supported-original-native-feedback-ledger-v1',
             'helper_source_sha256':metadata['feedback_source_sha256']})
        np.savez_compressed(output/filename,**values,
            phase_labels_json=np.asarray(json.dumps(rejection.buffers['phase_labels'])),
            metadata_json=np.asarray(json.dumps({'model_fingerprint':metadata['model_fingerprint'],
                'controller_sha256':metadata['controller_sha256'],'runtime':metadata['runtime'],
                'timestep_s':metadata['scene_config']['base']['thread']['timestep'],
                'force_timing':metadata['native_force_recording_note'], **observer_identity,
                'scope':'All original solved native observations before orderly control rejection. No native tick or force sample was invented at rejection.'})))
        metadata[filename.removesuffix('.npz')]={'filename':filename,
            'sha256':hashlib.sha256((output/filename).read_bytes()).hexdigest(),
            'observed_physics_steps':len(rows),'columns':list(columns)}
        if filename == 'native_feedback_force_history.npz':
            metadata['native_feedback_force_history']['helper_source_sha256']=metadata['feedback_source_sha256']
    pad_path=output/'left_pad_force_history.npz'
    np.savez_compressed(pad_path,time=np.asarray(rejection.buffers['left_pad_times'],dtype=float),
        pad_normal_force_N=np.asarray(rejection.buffers['left_pad_forces'],dtype=float).reshape(-1,2),
        phase_index=np.asarray(rejection.buffers['left_pad_phase_indices'],dtype=np.int16),
        phase_labels_json=np.asarray(json.dumps(rejection.buffers['phase_labels'])),
        metadata_json=np.asarray(json.dumps({'model_fingerprint':metadata['model_fingerprint'],
            'controller_sha256':metadata['controller_sha256'],'runtime':metadata['runtime'],
            'observer':PadLoadHistory.version,'timestep_s':metadata['scene_config']['base']['thread']['timestep'],
            'minimum_loaded_force_N':rejection.buffers['left_pad_history_threshold'],
            'scope':metadata['left_pad_history_scope'],'timing':metadata['native_force_recording_note']})))
    metadata['left_pad_force_history']={'filename':pad_path.name,
        'sha256':hashlib.sha256(pad_path.read_bytes()).hexdigest(),
        'observed_physics_steps':len(rejection.buffers['left_pad_times'])}
    metadata['all_substep_left_pad_loads']=rejection.buffers['left_pad_history_report']
    metadata['robot_inertia_command_history']=write_inertia_ledger(output,rejection.buffers['inertia_rows'],metadata)
    np.savez_compressed(output/'insertion_trace.npz',time=np.asarray(rejection.buffers['times'],dtype=float),
        qpos=np.asarray(rejection.buffers['positions'],dtype=float).reshape(-1,len(rejection.checkpoint['qpos'])),
        qvel=np.asarray(rejection.buffers['velocities'],dtype=float).reshape(-1,len(rejection.checkpoint['qvel'])),
        controller=np.asarray(rejection.buffers['controls'],dtype=float).reshape(-1,len(rejection.checkpoint['controller'])),
        info_json=np.asarray(json.dumps(rejection.buffers['rows'])),
        metadata_json=np.asarray(json.dumps(metadata)))
    np.savez_compressed(output/'control_rejection_checkpoint.npz',**rejection.checkpoint,
        metadata_json=np.asarray(json.dumps({'scope':'Exact last native post-state or declared initialization state, with no new integration/force solve at rejection','original_native_steps':len(rejection.buffers['feedback_rows'])})))
    (output/'insertion_validation.json').write_text(json.dumps(metadata,indent=2,allow_nan=False))
    (output/'control_rejection.json').write_text(json.dumps(metadata['aborted'],indent=2,allow_nan=False))
    return metadata


ORIGINAL_SUPPORT_COLUMNS = ('time', 'phase_index', 'table_wrench_world_at_block_origin_N_Nm', 'left_hand_wrench_world_at_block_origin_N_Nm', 'pad_normal_force_N', 'block_position_m', 'block_rotation_matrix', 'external_drive_zero', 'unexpected_world_support_count', 'supported_task_active', 'bolt_world_support_count', 'table_upward_normal_force_N', 'table_contact_candidates', 'table_loaded_contacts', 'bolt_tip_table_clearance_m', 'bolt_base_insertion_m', 'bolt_yaw_unwrapped_rad', 'right_hand_bolt_contact_count', 'head_block_seating_contact_count', 'formed_flank_overlap_m', 'loaded_formed_thread_normal_force_N', 'loaded_formed_thread_contact_count', 'unexpected_native_contact_count', 'unexpected_native_contact_peak_depth_m', 'bolt_tip_rest_top_clearance_m')
ORIGINAL_FEEDBACK_COLUMNS = ('time', 'phase_index', 'thread_wrench_on_bolt_world_N_Nm', 'hand_wrench_on_bolt_world_N_Nm', 'right_pad_normal_force_N', 'thread_gravity_opposing_force_N', 'hand_gravity_opposing_force_N', 'thread_summed_normal_force_N', 'relative_bolt_axial_velocity_m_per_s', 'relative_bolt_angular_speed_rad_per_s', 'relative_hand_angular_speed_rad_per_s', 'right_robot_bolt_contact_count', 'fully_open_unassisted', 'right_grasp_guard_active', 'right_axial_float_active', 'right_applied_axial_feed_N', 'right_commanded_aperture_m', 'desired_independent_clock_rad', 'desired_independent_angular_speed_rad_s', 'open_peak_axial_drift_m', 'open_peak_yaw_drift_rad', 'weight_window_ready', 'open_weight_window_ready', 'actual_quiet_regrasp_streak_s', 'all_hard_guards_held', 'loaded_actual_interior_flank_contact_count', 'interior_flank_gravity_opposing_force_N', 'interior_flank_summed_normal_force_N', 'left_downward_feed_N', 'right_command_wrench_N_Nm', 'left_command_wrench_N_Nm', 'right_motor_torques_Nm', 'left_motor_torques_Nm', 'right_actual_aperture_postintegration_m', 'external_drive_zero', 'bolt_world_support_contact_count', 'nonthread_block_bolt_contact_count', 'radial_offset_m', 'bolt_tilt_rad', 'native_thread_contact_count', 'weight_observation_valid', 'open_observation_valid', 'right_grip_slip_m', 'right_grip_rotation_slip_rad', 'left_grip_slip_m', 'left_grip_rotation_slip_rad', 'robot_yaw_brake_active', 'desired_independent_angular_acceleration_rad_s2', 'desired_independent_angular_jerk_rad_s3')


def closed_body_and_hand_quiet(sample):
    return bool(sample['relative_bolt_angular_speed_rad_per_s'] <= .01
        and sample['relative_hand_angular_speed_rad_per_s'] <= .01
        and abs(sample['relative_bolt_axial_velocity_m_per_s']) <= .0002)


def open_bolt_quiet(sample):
    return bool(sample['relative_bolt_angular_speed_rad_per_s'] <= .01
        and abs(sample['relative_bolt_axial_velocity_m_per_s']) <= .0002)


def reverse_stop_readiness(observer_ready, weight_ready, sample, *, brake_active):
    """Braking never counts as a stopped direction; all independent gates apply."""
    return bool(not brake_active and observer_ready and weight_ready
                and closed_body_and_hand_quiet(sample))
