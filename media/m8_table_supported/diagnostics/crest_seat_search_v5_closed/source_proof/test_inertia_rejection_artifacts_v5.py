"""Narrow rejection-path tests with synthetic saved inputs, never integration.

Execute the actual command/rejection block and its actual artifact writer.
These tests qualify preservation and provenance, not native physics outcomes.
"""
import ast
import copy
import importlib.util
import inspect
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

SOURCE = Path(__file__).with_name('crest_seat_inertia_probe_v5.py')


@pytest.fixture(scope='module')
def harness():
    old_path = list(sys.path)
    sys.path.insert(0, str(SOURCE.parent))
    spec = importlib.util.spec_from_file_location('yam_twin.ignored_inertia_rejection_test', SOURCE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    try:
        spec.loader.exec_module(module)
        yield module
    finally:
        sys.path[:] = old_path
        sys.modules.pop(spec.name, None)


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
                if 'DiagnosticControlRejection' in raised:
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


def actual_pure_inertia_record(harness, tick):
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
        inertia['wrench_world_N_Nm'], jp, jr, np.arange(6)*.01,
        qdot*.03, np.ones(6)*10.)
    record = dict(enabled=True, retained_native_state_time_s=(tick-2)*50e-6,
        retained_arm_velocity_rad_s=qdot, arm_jacobian_position=jp,
        arm_jacobian_rotation=jr, arm_bias_torque_Nm=np.arange(6)*.01,
        native_drag_torque_Nm=qdot*.03, arm_jacobian_position_derivative=np.zeros_like(jp),
        arm_jacobian_rotation_derivative=np.zeros_like(jr),
        requested_site_acceleration_world_m_s2=ades,
        requested_angular_acceleration_world_rad_s2=alpha, inertia=inertia, **combined)
    actual = harness._flatten_inertia_record(record)
    actual.update(scheduled_angular_velocity_world_rad_s=np.array([0., 0., -1.8]),
        scheduled_lever_world_m=np.array([.02, -.01, 0.]), command_time_s=(tick-1)*50e-6,
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
        phase_labels=['cold_window_hold', 'reverse_seat_1'],
        times=[50e-6]*sparse, positions=[checkpoint['qpos']-.01]*sparse,
        velocities=[checkpoint['qvel']-.02]*sparse, controls=[checkpoint['controller']-.03]*sparse,
        rows=[{'time':50e-6, 'synthetic_recorded_prefix':True}]*sparse,
        left_pad_times=[(tick+1)*50e-6 for tick in range(ticks)],
        left_pad_forces=pad_forces, left_pad_phase_indices=[0]*ticks,
        left_pad_history_report=history.report(), left_pad_history_threshold=history.threshold)
    metadata = dict(model_fingerprint='synthetic-preservation-contract', controller_sha256='0'*64,
        runtime={'scope':'synthetic saved inputs; no integration'},
        scene_config={'base':{'thread':{'timestep':50e-6}}},
        robot_inertia_feedforward={'source_sha256':'0913f964c940d2fdf0bcbc92a0734c70c4868499439a4e5915e359a186dc17ff'},
        maximum_phase_plan=[['cold_window_hold'], ['reverse_seat_1']],
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
        DiagnosticControlRejection=harness.DiagnosticControlRejection,
        InertiaFeedforwardError=harness.InertiaFeedforwardError,
        orbital_site_acceleration=orbital, _flatten_inertia_record=flatten,
        data=data, right=right, down=np.array([0.,0.,1.]), alpha=1.2, omega=-1.8,
        target_r=np.eye(3),target={'measured_head_to_tool_p':np.array([.02,-.01,0.])},
        target_p=np.zeros(3),target_v=np.zeros(3),target_w=np.zeros(3),
        gap0=.0184,gap1=.0184,blend=.5,axial_float=True,feed=.2,hole_r=np.eye(3),
        metadata=metadata,summaries=[{'phase':'prior-completed-prefix'}],
        label='reverse_seat_1',seat_event=SimpleNamespace(report=lambda:{'confirmed_search_direction_event':False}),
        selected=[['cold_window_hold'],['reverse_seat_1']],left_pad_history=history,
        **{key:value for key,value in buffers.items() if key not in
           ('phase_labels','left_pad_history_report','left_pad_history_threshold')})
    with pytest.raises(harness.DiagnosticControlRejection) as raised:
        exec(actual_rejection_block,namespace)
    rejection=raised.value
    assert calls['integration'] == 0
    assert calls['orbital'] == 1
    assert calls['command'] == int(failure_stage != 'orbital')
    assert calls['flatten'] == int(failure_stage == 'flatten')
    assert rejection.report['wall_seconds'] == .25
    assert rejection.report['original_native_steps'] == ticks
    assert rejection.report['aborted']['new_native_step_executed'] is False
    assert not rejection.report['passed'] and not rejection.report['diagnostic_completed']
    assert not rejection.report['full_fresh_trajectory_qualified']
    assert not rejection.report['capture_or_open_reset_qualified']
    assert_same_tree(buffers,before_buffers)
    for key,before in before_checkpoint.items():
        assert np.asarray(rejection.checkpoint[key]).dtype == before.dtype
        assert np.asarray(rejection.checkpoint[key]).tobytes() == before.tobytes()
    assert np.array_equal(data.qpos,before_checkpoint['qpos'])
    assert np.array_equal(data.qvel,before_checkpoint['qvel'])
    if failure_stage == 'flatten':
        assert not np.array_equal(data.ctrl,before_checkpoint['controller'])
        assert np.array_equal(rejection.checkpoint['unapplied_rejected_controller'],data.ctrl)
    harness._preserve_control_rejection(tmp_path,rejection)
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
    assert saved['all_substep_left_pad_loads'] == buffers['left_pad_history_report']
    assert saved['left_pad_force_history']['observed_physics_steps'] == ticks
    assert saved['robot_inertia_command_history']['executed_native_commands'] == ticks
    assert json.loads((tmp_path/'control_rejection.json').read_text()) == rejection.report['aborted']
