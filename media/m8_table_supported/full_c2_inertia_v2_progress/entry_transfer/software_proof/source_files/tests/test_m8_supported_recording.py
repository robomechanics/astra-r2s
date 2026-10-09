"""Native-recording rejection contracts on synthetic saved inputs only.

Actual rejection block and writer preserve the executed prefix; no integration
occurs and these contracts do not qualify a physics or trajectory outcome.
"""
import ast
import copy
import inspect
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from yam_twin import m8_supported_simulation as controller
from yam_twin import m8_supported_recording as recording
from yam_twin import m8_robot_inertia as inertia

SOURCE=Path(controller.__file__)


@pytest.fixture(scope='module')
def harness():
    return SimpleNamespace(run_supported_demo=controller._run_supported_demo,
        SupportedControlRejection=recording.SupportedControlRejection,
        InertiaFeedforwardError=inertia.InertiaFeedforwardError,
        hybrid5d_feedforward=inertia.hybrid5d_feedforward,
        combine_capped_robot_command=inertia.combine_capped_robot_command,
        flatten_inertia_record=recording.flatten_inertia_record,
        preserve_control_rejection=recording.preserve_control_rejection,
        ORIGINAL_SUPPORT_COLUMNS=recording.ORIGINAL_SUPPORT_COLUMNS,
        ORIGINAL_FEEDBACK_COLUMNS=recording.ORIGINAL_FEEDBACK_COLUMNS,
        PadLoadHistory=recording.PadLoadHistory,mujoco=controller.mujoco)


@pytest.fixture(scope='module')
def actual_rejection_block(harness):
    tree = ast.parse(inspect.getsource(harness.run_supported_demo))
    matches = []
    for container in ast.walk(tree):
        for name in ('body', 'orelse', 'finalbody'):
            statements = getattr(container, name, ())
            if not isinstance(statements, (list, tuple)):
                continue
            for index, statement in enumerate(statements):
                if not isinstance(statement, ast.Try):
                    continue
                raised = [node.exc.func.id for handler in statement.handlers
                    for node in ast.walk(handler) if isinstance(node, ast.Raise)
                    and isinstance(node.exc, ast.Call) and isinstance(node.exc.func, ast.Name)]
                if 'SupportedControlRejection' in raised:
                    assert index > 0
                    previous = statements[index-1]
                    assert isinstance(previous, ast.Assign)
                    assert any(isinstance(target, ast.Name) and target.id == 'applied_ctrl_before_command'
                               for target in previous.targets)
                    matches.append([previous, statement])
    assert len(matches) == 1
    return compile(ast.Module(body=matches[0], type_ignores=[]), str(SOURCE), 'exec')


# Independent archive shapes/units, rather than generating expectations with
# the serializer under test. Original count/flag conventions remain explicit.
VECTOR_SHAPES = {
    'table_wrench_world_at_block_origin_N_Nm': (6,),
    'left_hand_wrench_world_at_block_origin_N_Nm': (6,),
    'pad_normal_force_N': (2,), 'block_position_m': (3,), 'block_rotation_matrix': (3, 3),
    'thread_wrench_on_bolt_world_N_Nm': (6,), 'hand_wrench_on_bolt_world_N_Nm': (6,),
    'right_pad_normal_force_N': (2,), 'right_command_wrench_N_Nm': (6,),
    'left_command_wrench_N_Nm': (6,), 'right_motor_torques_Nm': (6,), 'left_motor_torques_Nm': (6,),
}
FLAG_FIELDS = {
    'external_drive_zero', 'supported_task_active', 'fully_open_unassisted',
    'right_grasp_guard_active', 'right_axial_float_active', 'weight_window_ready',
    'open_weight_window_ready', 'all_hard_guards_held', 'weight_observation_valid',
    'open_observation_valid', 'cold_table_window_warmup', 'robot_yaw_brake_active',
}


def dtype_for(name):
    if name in FLAG_FIELDS:
        return np.dtype(bool)
    if name in ('phase_index', 'table_contact_candidates', 'table_loaded_contacts') or name.endswith(('_count', '_contacts')):
        return np.dtype(np.int64)
    return np.dtype(np.float64)


def original_rows(columns, count):
    records = []
    for tick in range(1, count+1):
        values = []
        for field, name in enumerate(columns):
            shape, dtype = VECTOR_SHAPES.get(name, ()), dtype_for(name)
            if name == 'time':
                value = np.float64(tick*50e-6)
            elif name == 'phase_index':
                value = np.int64(0)
            elif dtype.kind == 'b':
                value = np.bool_(tick % 2 == 1)
            elif dtype.kind == 'i':
                value = np.int64(tick+field)
            elif shape:
                value = (np.arange(np.prod(shape), dtype=float).reshape(shape)+field+.125)*tick
            else:
                value = np.float64((field+.125)*tick)
            values.append(value)
        records.append(tuple(values))
    return records


def actual_pure_inertia_record(harness, tick, *, enabled=True, stale_derivatives=False):
    # Obtain real output keys/types from the frozen helper instead of inventing
    # a schema-shaped dummy row that could conceal serializer drift.
    jp, jr = np.zeros((3, 6)), np.zeros((3, 6))
    jp[0, 0], jp[1, 1], jp[2, 5] = .3, .4, .2
    jr[:, 2:5] = np.eye(3)
    qdot = np.linspace(.01, .06, 6)
    ades, alpha = np.array([.1, -.2, 0.]), np.array([.2, .3, -.1])
    inertia = harness.hybrid5d_feedforward(np.eye(6), jp, jr, np.zeros_like(jp),
        np.zeros_like(jr), qdot, np.eye(3)[:, :2], ades, alpha)
    combined = harness.combine_capped_robot_command(np.array([0.,0.,.2,0.,0.,0.]),
        inertia['wrench_world_N_Nm'] if enabled else np.zeros(6), jp, jr, np.arange(6)*.01,
        qdot*.03, np.ones(6)*10.)
    record = dict(enabled=enabled, retained_native_state_time_s=(tick-2)*50e-6,
        retained_arm_velocity_rad_s=qdot, arm_jacobian_position=jp,
        arm_jacobian_rotation=jr, arm_bias_torque_Nm=np.arange(6)*.01,
        native_drag_torque_Nm=qdot*.03, arm_jacobian_position_derivative=np.full_like(jp,.31) if stale_derivatives else np.zeros_like(jp),
        arm_jacobian_rotation_derivative=np.full_like(jr,-.42) if stale_derivatives else np.zeros_like(jr),
        requested_site_acceleration_world_m_s2=ades if enabled else np.zeros(3),
        requested_angular_acceleration_world_rad_s2=alpha if enabled else np.zeros(3), inertia=inertia if enabled else None, **combined)
    actual = harness.flatten_inertia_record(record)
    actual.update(scheduled_angular_velocity_world_rad_s=np.array([0., 0., -1.8]) if enabled else np.zeros(3),
        scheduled_lever_world_m=np.array([.02, -.01, 0.]) if enabled else np.zeros(3), command_time_s=(tick-1)*50e-6,
        time=tick*50e-6, phase_index=0, postintegration_arm_velocity_rad_s=qdot+.0001*tick)
    return actual


def saved_prefix(harness, ticks):
    history = harness.PadLoadHistory()
    pad_forces = [np.array([12.25+tick, 13.5+tick]) for tick in range(ticks)]
    for forces in pad_forces:
        history.observe(forces, 50e-6)
    checkpoint = dict(time=np.asarray(ticks*50e-6), qpos=np.linspace(-.2,.7,30),
        qvel=np.linspace(-.1,.2,28), controller=np.linspace(-.3,.4,16))
    # Sparse saved pose deliberately predates the authoritative last state.
    sparse = int(ticks > 0)
    buffers = dict(support_rows=original_rows(harness.ORIGINAL_SUPPORT_COLUMNS, ticks),
        feedback_rows=original_rows(harness.ORIGINAL_FEEDBACK_COLUMNS, ticks),
        inertia_rows=[actual_pure_inertia_record(harness, tick) for tick in range(1,ticks+1)],
        phase_labels=['settle_table', 'reverse_seat_1'],
        times=[50e-6]*sparse, positions=[checkpoint['qpos']-.01]*sparse,
        velocities=[checkpoint['qvel']-.02]*sparse, controls=[checkpoint['controller']-.03]*sparse,
        rows=[{'time':50e-6, 'synthetic_recorded_prefix':True}]*sparse,
        left_pad_times=[(tick+1)*50e-6 for tick in range(ticks)],
        left_pad_forces=pad_forces, left_pad_phase_indices=[0]*ticks,
        left_pad_history_report=history.report(), left_pad_history_threshold=history.threshold)
    metadata = dict(model_fingerprint='synthetic-preservation-contract', controller_sha256='0'*64,
        feedback_source_sha256='a'*64,
        runtime={'scope':'synthetic saved inputs; no integration'},
        scene_config={'base':{'thread':{'timestep':50e-6}}},
        robot_inertia_feedforward={'source_sha256':'0913f964c940d2fdf0bcbc92a0734c70c4868499439a4e5915e359a186dc17ff'},
        maximum_phase_plan=[['settle_table'], ['reverse_seat_1']],
        native_force_recording_note='Preserved synthetic original native-label contract',
        left_pad_history_scope='Original all-substep stabilization pad loads')
    return buffers, checkpoint, metadata, history


def assert_same_tree(actual, before):
    if isinstance(before, dict):
        assert actual.keys() == before.keys()
        for key in before:
            assert_same_tree(actual[key], before[key])
    elif isinstance(before, (list, tuple)):
        assert len(actual) == len(before)
        for a,b in zip(actual,before):
            assert_same_tree(a,b)
    elif isinstance(before, np.ndarray):
        assert actual.dtype == before.dtype and actual.shape == before.shape
        assert actual.tobytes() == before.tobytes()
    else:
        assert actual == before


@pytest.mark.parametrize('ticks', [0, 3])
@pytest.mark.parametrize('failure_stage', ['orbital', 'command', 'flatten'])
def test_rejected_pre_step_command_preserves_only_actual_prefix_and_last_post_checkpoint(
        harness, actual_rejection_block, tmp_path, monkeypatch, ticks, failure_stage):
    buffers, checkpoint, metadata, history = saved_prefix(harness, ticks)
    before_buffers, before_checkpoint = copy.deepcopy(buffers), copy.deepcopy(checkpoint)
    data = SimpleNamespace(time=float(checkpoint['time']), qpos=checkpoint['qpos'].copy(),
        qvel=checkpoint['qvel'].copy(), ctrl=checkpoint['controller'].copy())
    calls = dict(orbital=0, command=0, flatten=0, integration=0)
    def forbidden_integration(*args, **kwargs):
        calls['integration'] += 1
        raise AssertionError('Rejection testing must never execute native integration')
    monkeypatch.setattr(harness.mujoco, 'mj_step', forbidden_integration)
    monkeypatch.setattr(harness.mujoco, 'mj_forward', forbidden_integration)
    def orbital(*args):
        calls['orbital'] += 1
        if failure_stage == 'orbital':
            raise FloatingPointError('scheduled orbital overflow')
        return np.array([.1,-.2,0.])
    def command(*args, **kwargs):
        calls['command'] += 1
        right.last_position_error = np.array([.001,.002,.003])
        if failure_stage == 'command':
            raise harness.InertiaFeedforwardError('rank-deficient robot task')
        # Successful controller generation may write ctrl before flattening;
        # this control was never integrated and must not become lastpostctrl.
        data.ctrl[:] = np.linspace(2.,3.,len(data.ctrl))
    def flatten(record):
        calls['flatten'] += 1
        raise np.linalg.LinAlgError('record numerical validation failed')
    right = SimpleNamespace(command=command,last_inertia_record={'attempt_only':True})
    namespace = dict(np=np, time=SimpleNamespace(perf_counter=lambda:10.25), start_wall=10.,
        SupportedControlRejection=harness.SupportedControlRejection,
        InertiaFeedforwardError=harness.InertiaFeedforwardError,
        orbital_site_acceleration=orbital, flatten_inertia_record=flatten,
        data=data, right=right, down=np.array([0.,0.,1.]), alpha=1.2, omega=-1.8,
        target_r=np.eye(3),target={'measured_head_to_tool_p':np.array([.02,-.01,0.])},
        target_p=np.zeros(3),target_v=np.zeros(3),target_w=np.zeros(3),
        gap0=.0184,gap1=.0184,blend=.5,axial_float=True,feedback_closed=True,feed=.2,hole_r=np.eye(3),
        metadata=metadata,summaries=[{'phase':'prior-completed-prefix'}],
        label='reverse_seat_1',seat_event=SimpleNamespace(report=lambda:{'confirmed_search_direction_event':False}),
        selected=[['settle_table'],['reverse_seat_1']],phases=[['settle_table'],['reverse_seat_1']],left_pad_history=history,
        **{key:value for key,value in buffers.items() if key not in
           ('phase_labels','left_pad_history_report','left_pad_history_threshold')})
    with pytest.raises(harness.SupportedControlRejection) as raised:
        exec(actual_rejection_block,namespace)
    rejection=raised.value
    assert calls['integration'] == 0
    assert calls['orbital'] == 1
    assert calls['command'] == int(failure_stage != 'orbital')
    assert calls['flatten'] == int(failure_stage == 'flatten')
    assert rejection.report['wall_seconds'] == .25
    assert rejection.report['original_native_steps'] == ticks
    assert rejection.report['aborted']['new_native_step_executed'] is False
    assert not rejection.report['passed']
    assert not rejection.report['acceptance_checks']['no_solver_or_state_abort']['passed']
    assert_same_tree(buffers,before_buffers)
    for key,before in before_checkpoint.items():
        assert np.asarray(rejection.checkpoint[key]).dtype == before.dtype
        assert np.asarray(rejection.checkpoint[key]).tobytes() == before.tobytes()
    assert np.array_equal(data.qpos,before_checkpoint['qpos'])
    assert np.array_equal(data.qvel,before_checkpoint['qvel'])
    if failure_stage == 'flatten':
        assert not np.array_equal(data.ctrl,before_checkpoint['controller'])
        assert np.array_equal(rejection.checkpoint['unapplied_rejected_controller'],data.ctrl)
    harness.preserve_control_rejection(tmp_path,rejection)
    assert calls['integration'] == 0
    assert_same_tree(buffers,before_buffers)

    for filename, columns, rows in (
        ('table_support_force_history.npz',harness.ORIGINAL_SUPPORT_COLUMNS,buffers['support_rows']),
        ('native_feedback_force_history.npz',harness.ORIGINAL_FEEDBACK_COLUMNS,buffers['feedback_rows'])):
        with np.load(tmp_path/filename,allow_pickle=False) as archive:
            assert set(archive.files) == set(columns)|{'phase_labels_json','metadata_json'}
            for index,name in enumerate(columns):
                expected=np.asarray([row[index] for row in rows],dtype=dtype_for(name)).reshape(
                    (ticks,)+VECTOR_SHAPES.get(name,()))
                actual=archive[name]
                assert actual.dtype == expected.dtype and actual.shape == expected.shape
                assert actual.tobytes() == expected.tobytes()
            assert json.loads(archive['phase_labels_json'].item()) == buffers['phase_labels']
            raw_metadata=json.loads(archive['metadata_json'].item())
            assert raw_metadata['model_fingerprint'] == metadata['model_fingerprint']
            assert raw_metadata['controller_sha256'] == metadata['controller_sha256']
            assert raw_metadata['runtime'] == metadata['runtime']
            assert raw_metadata['timestep_s'] == metadata['scene_config']['base']['thread']['timestep']
            assert raw_metadata['force_timing'] == metadata['native_force_recording_note']
            if filename == 'table_support_force_history.npz':
                assert raw_metadata['observer'] == 'native-supported-block-load-window-v1'
            else:
                assert raw_metadata['observer'] == 'supported-original-native-feedback-ledger-v1'
                assert raw_metadata['helper_source_sha256'] == metadata['feedback_source_sha256']

    with np.load(tmp_path/'left_pad_force_history.npz',allow_pickle=False) as archive:
        assert set(archive.files) == {'time','pad_normal_force_N','phase_index','phase_labels_json','metadata_json'}
        for name,expected in (
            ('time',np.asarray(buffers['left_pad_times'],dtype=float)),
            ('pad_normal_force_N',np.asarray(buffers['left_pad_forces'],dtype=float).reshape(-1,2)),
            ('phase_index',np.asarray(buffers['left_pad_phase_indices'],dtype=np.int16))):
            actual=archive[name]
            assert actual.dtype == expected.dtype and actual.shape == expected.shape
            assert actual.tobytes() == expected.tobytes()
        pad_metadata=json.loads(archive['metadata_json'].item())
        assert pad_metadata['observer'] == harness.PadLoadHistory.version
        assert pad_metadata['minimum_loaded_force_N'] == history.threshold

    with np.load(tmp_path/'robot_inertia_command_history.npz',allow_pickle=False) as archive:
        required={'time','phase_index','mass_aa','arm_jacobian_position','arm_jacobian_rotation',
            'arm_jacobian_position_derivative','arm_jacobian_rotation_derivative',
            'hybrid_jacobian5','mobility5','motor_torques_Nm','motor_clipped',
            'combined_uncapped_wrench_world_N_Nm','motor_uncapped_torques_Nm'}
        assert required.issubset(archive.files)
        if ticks:
            assert set(archive.files)-{'metadata_json'} == set(buffers['inertia_rows'][0])
            for key in buffers['inertia_rows'][0]:
                expected=np.asarray([row[key] for row in buffers['inertia_rows']])
                actual=archive[key]
                assert actual.dtype == expected.dtype and actual.shape == expected.shape
                assert actual.tobytes() == expected.tobytes()
        else:
            for key,shape,dtype in [('time',(0,),np.float64),('phase_index',(0,),np.int64),
                ('mass_aa',(0,6,6),np.float64),('hybrid_jacobian5',(0,5,6),np.float64),
                ('mobility5',(0,5,5),np.float64),('motor_clipped',(0,6),bool),
                ('motor_torques_Nm',(0,6),np.float64)]:
                assert archive[key].shape == shape and archive[key].dtype == np.dtype(dtype)

    with np.load(tmp_path/'insertion_trace.npz',allow_pickle=False) as archive:
        for name,buffer_key,width in [('qpos','positions',30),('qvel','velocities',28),('controller','controls',16)]:
            expected=np.asarray(buffers[buffer_key],dtype=float).reshape(-1,width)
            assert archive[name].shape == expected.shape and archive[name].dtype == expected.dtype
            assert archive[name].tobytes() == expected.tobytes()
        assert archive['time'].tobytes() == np.asarray(buffers['times'],dtype=float).tobytes()
        assert len(archive['time']) == int(ticks>0)
        assert json.loads(archive['info_json'].item()) == buffers['rows']
    with np.load(tmp_path/'control_rejection_checkpoint.npz',allow_pickle=False) as archive:
        for key,before in before_checkpoint.items():
            assert archive[key].dtype == before.dtype and archive[key].tobytes() == before.tobytes()
        assert json.loads(archive['metadata_json'].item())['original_native_steps'] == ticks
        if ticks:
            assert archive['time'].item() > buffers['times'][-1]
    saved=json.loads((tmp_path/'insertion_validation.json').read_text())
    assert saved['native_feedback_force_history']['helper_source_sha256'] == metadata['feedback_source_sha256']
    assert saved['all_substep_left_pad_loads'] == buffers['left_pad_history_report']
    assert saved['left_pad_force_history']['observed_physics_steps'] == ticks
    assert saved['robot_inertia_command_history']['executed_native_commands'] == ticks
    assert json.loads((tmp_path/'control_rejection.json').read_text()) == rejection.report['aborted']


def test_active_to_disabled_command_ledger_retains_pd_inputs_and_marks_inverse_and_stale_jdot_absent(harness,tmp_path):
    active=actual_pure_inertia_record(harness,1)
    disabled=actual_pure_inertia_record(harness,2,enabled=False,stale_derivatives=True)
    for key in ('inertia_inputs_present','jacobian_derivatives_present'):
        assert active[key].item() is True and disabled[key].item() is False
    for key in ('mass_aa','mobility5','hybrid_jacobian5','projected_world_jdot_qdot5',
                'arm_jacobian_position_derivative','arm_jacobian_rotation_derivative'):
        assert np.count_nonzero(disabled[key]) == 0
    assert np.count_nonzero(disabled['arm_jacobian_position']) > 0
    assert np.count_nonzero(disabled['arm_bias_torque_Nm']) > 0
    assert np.count_nonzero(disabled['native_drag_torque_Nm']) > 0
    assert disabled['pd_wrench_world_N_Nm'][2] == .2
    assert np.count_nonzero(disabled['ff_wrench_world_N_Nm']) == 0
    _,_,metadata,_=saved_prefix(harness,0)
    summary=recording.write_inertia_ledger(tmp_path,[active,disabled],metadata)
    assert summary['executed_native_commands'] == 2
    with np.load(tmp_path/summary['filename'],allow_pickle=False) as archive:
        assert archive['enabled'].tolist() == [True,False]
        assert archive['inertia_inputs_present'].tolist() == [True,False]
        assert archive['jacobian_derivatives_present'].tolist() == [True,False]
        assert archive['phase_index'].dtype == np.int64
        assert np.count_nonzero(archive['arm_jacobian_position_derivative'][1]) == 0
        assert archive['pd_wrench_world_N_Nm'][1].tobytes() == disabled['pd_wrench_world_N_Nm'].tobytes()


def quiet_sample(**changes):
    values=dict(relative_bolt_angular_speed_rad_per_s=.005,
        relative_hand_angular_speed_rad_per_s=.006,
        relative_bolt_axial_velocity_m_per_s=.0001)
    values.update(changes)
    return values


@pytest.mark.parametrize('changes,open_ready,closed_ready',[
    (dict(relative_hand_angular_speed_rad_per_s=2.),True,False),
    (dict(relative_bolt_angular_speed_rad_per_s=.02),False,False),
    (dict(relative_bolt_axial_velocity_m_per_s=-.000201),False,False),
])
def test_open_reset_observes_free_bolt_quiet_separately_from_moving_empty_hand(changes,open_ready,closed_ready):
    sample=quiet_sample(**changes)
    assert recording.open_bolt_quiet(sample) is open_ready
    assert recording.closed_body_and_hand_quiet(sample) is closed_ready


@pytest.mark.parametrize('observer,weight,brake,changes,ready',[
    (True,True,False,{},True),
    (True,True,True,{},False),
    (False,True,False,{},False),
    (True,False,False,{},False),
    (True,True,False,dict(relative_bolt_angular_speed_rad_per_s=.01001),False),
    (True,True,False,dict(relative_hand_angular_speed_rad_per_s=.01001),False),
    (True,True,False,dict(relative_bolt_axial_velocity_m_per_s=.000201),False),
    (True,True,False,dict(relative_bolt_angular_speed_rad_per_s=.01,
        relative_hand_angular_speed_rad_per_s=.01,relative_bolt_axial_velocity_m_per_s=-.0002),True),
])
def test_reverse_stop_requires_finished_brake_and_each_live_physical_observer(observer,weight,brake,changes,ready):
    assert recording.reverse_stop_readiness(observer,weight,quiet_sample(**changes),brake_active=brake) is ready


def rejection_payload(harness,ticks):
    buffers,checkpoint,metadata,_=saved_prefix(harness,ticks)
    checkpoint['unapplied_rejected_controller']=checkpoint['controller'].copy()
    metadata.update(original_native_steps=ticks,passed=False,partial=True,
        aborted={'reason':'synthetic pre-step numerical rejection','new_native_step_executed':False})
    return recording.SupportedControlRejection(metadata,buffers,checkpoint)


@pytest.mark.parametrize('ticks',[0,3])
def test_public_supported_runner_closes_structured_rejection_without_native_execution(harness,tmp_path,monkeypatch,ticks):
    rejection=rejection_payload(harness,ticks)
    calls=[]
    def rejected_run(output,**kwargs):
        calls.append((output,kwargs))
        raise rejection
    def forbidden(*args,**kwargs):
        raise AssertionError('Public rejection close must not integrate or solve forces')
    monkeypatch.setattr(controller,'_run_supported_demo',rejected_run)
    monkeypatch.setattr(controller.mujoco,'mj_step',forbidden)
    monkeypatch.setattr(controller.mujoco,'mj_forward',forbidden)
    result=controller.run_supported_demo(tmp_path,maximum_phases=1)
    assert len(calls) == 1 and calls[0][1]['maximum_phases'] == 1
    assert result is rejection.report and result['passed'] is False
    assert result['original_native_steps'] == ticks
    with np.load(tmp_path/'control_rejection_checkpoint.npz',allow_pickle=False) as archive:
        assert archive['qpos'].tobytes() == rejection.checkpoint['qpos'].tobytes()


@pytest.mark.parametrize('failure',['native_count','checkpoint_time','checkpoint_nonfinite','attempted_width','pad_alignment'])
def test_rejection_archive_refuses_inconsistent_last_state_or_missing_original_samples(harness,tmp_path,failure):
    rejection=rejection_payload(harness,3)
    if failure == 'native_count':
        rejection.report['original_native_steps']=4
    elif failure == 'checkpoint_time':
        rejection.checkpoint['time']=np.asarray(.002)
    elif failure == 'checkpoint_nonfinite':
        rejection.checkpoint['qvel'][2]=np.nan
    elif failure == 'attempted_width':
        rejection.checkpoint['unapplied_rejected_controller']=np.zeros(2)
    else:
        rejection.buffers['left_pad_phase_indices'].pop()
    with pytest.raises(ValueError):
        recording.preserve_control_rejection(tmp_path,rejection)


@pytest.mark.parametrize('field,bad',[
    ('phase_index',.5),('table_loaded_contacts',-1),
    ('external_drive_zero',2),('pad_normal_force_N',np.array([np.nan,1.])),
    ('table_wrench_world_at_block_origin_N_Nm',np.ones(5)),
])
def test_original_force_writer_rejects_corruption_before_silent_cast_or_padding(harness,field,bad):
    rows=original_rows(harness.ORIGINAL_SUPPORT_COLUMNS,1)
    changed=list(rows[0]);changed[harness.ORIGINAL_SUPPORT_COLUMNS.index(field)]=bad
    with pytest.raises(ValueError):
        recording.force_arrays([tuple(changed)],harness.ORIGINAL_SUPPORT_COLUMNS)


def test_post_step_direction_request_with_insufficient_braking_room_records_rejection_not_consumed_readiness():
    tree=ast.parse(inspect.getsource(controller._run_supported_demo))
    matches=[node for node in ast.walk(tree) if isinstance(node,ast.If)
        and isinstance(node.test,ast.Compare) and isinstance(node.test.left,ast.Name)
        and node.test.left.id=='gate' and any(isinstance(value,ast.Constant) and value.value=='complete'
            for value in node.test.comparators) and any(isinstance(call,ast.Call)
                and isinstance(call.func,ast.Name) and call.func.id=='C2ReverseBrake' for call in ast.walk(node))]
    assert len(matches)==1
    block=compile(ast.Module(body=matches,type_ignores=[]),str(SOURCE),'exec')
    metadata={'physical_motion_events':[]}
    state=dict(gate='complete',feedback_phase_completed=False,label='reverse_seat_1',
        reverse_brake=None,C2ReverseBrake=controller.C2ReverseBrake,
        theta=-2.69,omega=-1.8,alpha=1.2,
        control=SimpleNamespace(reverse_brake_duration_s=.15,
            maximum_reverse_angle_rad=2.7,reverse_angular_speed_rad_s=2.),
        data=SimpleNamespace(time=1.234),model=SimpleNamespace(opt=SimpleNamespace(timestep=50e-6)),
        metadata=metadata,elapsed_phase=.2,duration=2.5,weight_report={'ready':True},
        open_report={'ready':False},seat_event=SimpleNamespace(report=lambda:{'stop_requested':True}))
    exec(block,state)
    assert state['gate']=='invalid' and state['reverse_brake'] is None
    assert state['feedback_phase_completed'] is False
    assert (state['theta'],state['omega'],state['alpha']) == (-2.69,-1.8,1.2)
    assert state['aborted']['scheduled_theta_rad']==-2.69
    assert state['aborted']['time']==1.234
    assert len(metadata['physical_motion_events'])==1
    assert metadata['physical_motion_events'][0]['event']=='Live direction request could not form a bounded braking plan'


def test_reverse_stop_budget_contains_braking_plus_observation_without_extending_other_stops():
    config=controller.SupportedControlConfig()
    phases={phase[0]:phase[1] for phase in controller.supported_phases(config)}
    assert phases['stop_reverse_seat_1']==pytest.approx(.15+.75)
    assert phases['stop_start_1']==pytest.approx(.75)


def test_actual_pre_step_capture_uses_prior_velocity_before_simulated_advance_without_engine_execution():
    tree=ast.parse(inspect.getsource(controller._run_supported_demo))
    matches=[]
    for node in ast.walk(tree):
        if not isinstance(getattr(node,'body',None),list):
            continue
        for index,statement in enumerate(node.body):
            if isinstance(statement,ast.Expr) and isinstance(statement.value,ast.Call):
                call=statement.value
                if isinstance(call.func,ast.Attribute) and call.func.attr=='mj_step':
                    matches.append(node.body[index-1:index+1])
    assert len(matches)==1
    first=matches[0][0]
    assert isinstance(first,ast.Expr) and first.value.func.attr=='capture_velocity_before_step'
    data=SimpleNamespace(qvel=np.array([.1,.2,.3,.4,.5,.6]),time=1.)
    right=inertia.HybridInertiaController.__new__(inertia.HybridInertiaController)
    right.data=data;right.dof_indices=np.arange(6)
    previous=data.qvel.copy()
    def simulate_step(model,state):
        state.qvel+=.07
        state.time+=50e-6
    block=compile(ast.Module(body=matches[0],type_ignores=[]),str(SOURCE),'exec')
    exec(block,dict(right=right,data=data,model=object(),mujoco=SimpleNamespace(mj_step=simulate_step)))
    assert np.array_equal(right.retained_qdot,previous)
    assert right.retained_time_s==1.
    assert np.array_equal(data.qvel,previous+.07)


def test_first_retained_velocity_capture_occurs_after_normal_supported_initialization():
    tree=ast.parse(inspect.getsource(controller._run_supported_demo))
    statements=tree.body[0].body
    initializer=[index for index,node in enumerate(statements) if isinstance(node,ast.Assign)
        and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Name)
        and node.value.func.id=='initialize_supported_pose']
    capture=[index for index,node in enumerate(statements) if isinstance(node,ast.Expr)
        and isinstance(node.value,ast.Call) and isinstance(node.value.func,ast.Attribute)
        and node.value.func.attr=='capture_velocity_before_step']
    assert len(initializer)==len(capture)==1 and initializer[0]<capture[0]
    data=SimpleNamespace(qvel=np.linspace(-.1,.2,6),time=0.)
    right=inertia.HybridInertiaController.__new__(inertia.HybridInertiaController)
    right.data=data;right.dof_indices=np.arange(6)
    def initialize(model,state,scene,control):
        # A known initialized arm state must be captured, not a stale copy.
        state.qvel[:]=np.linspace(.3,.8,6)
        return {},np.zeros(3),np.eye(3)
    block=compile(ast.Module(body=[statements[initializer[0]],statements[capture[0]]],type_ignores=[]),str(SOURCE),'exec')
    exec(block,dict(initialize_supported_pose=initialize,model=object(),data=data,
        scene=object(),control=object(),right=right))
    assert np.array_equal(right.retained_qdot,np.linspace(.3,.8,6))
    assert right.retained_time_s==0.
