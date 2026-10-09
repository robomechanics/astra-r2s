"""Separate cold-state lineage, original motor control and quaternion tests."""
from pathlib import Path
import importlib.util
import json
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location('chain', Path(__file__).with_name(
    'independent_cold_chain_audit.py'))
chain = importlib.util.module_from_spec(spec)
spec.loader.exec_module(chain)


def test_free_joint_qpos_order_counts_interleaved_hinge_and_ball_widths():
    xml = ET.fromstring('<mujoco><worldbody><body><freejoint name="fixture_block_free"/>'
        '<body><joint name="hinge"/><body><joint name="ball" type="ball"/></body></body></body>'
        '<body><freejoint name="male_bolt_free"/></body></worldbody></mujoco>')
    assert chain.free_joint_addresses(xml) == {'fixture_block_free': 0, 'male_bolt_free': 12}


def fixture(tmp_path):
    xml = '<mujoco><worldbody><body><freejoint name="fixture_block_free"/>'
    xml += '<body name="female_frame" quat="0 1 0 0"/></body><body><freejoint name="male_bolt_free"/></body></worldbody><actuator>'
    names = [f'{side}_servo{i}' for side in ('left', 'right') for i in range(1, 7)]
    names += ['left_finger', 'right_finger']
    xml += ''.join(f'<motor name="{name}"/>' for name in names)+'</actuator></mujoco>'
    (tmp_path/'scene.xml').write_text(xml)
    qpos = np.zeros(14)
    qpos[3] = 1.
    relative = Rotation.from_euler('z', -.4).as_matrix()
    male = np.diag([1., -1., -1.])@relative
    xyzw = Rotation.from_matrix(male).as_quat()
    qpos[10:14] = xyzw[[3, 0, 1, 2]]
    qvel = np.zeros(12)
    parent = tmp_path/'canonical.npz'
    state = tmp_path/'state.npz'
    native = dict(time_s=3., left_motor_torques_Nm=[2.]*6, right_motor_torques_Nm=[3.]*6)
    ctrl = np.arange(14, dtype=float)
    np.savez(parent, qpos=[qpos], qvel=[qvel], controller=[ctrl],
        info_json=json.dumps([dict(bolt_yaw_unwrapped_rad=.3)]))
    np.savez(state, qpos=[qpos], qvel=[qvel], info_json=json.dumps([native]))
    ctrl[:6], ctrl[6:12] = 2., 3.
    np.savez(tmp_path/'declared_cold_initialization.npz', qpos=qpos, qvel=qvel, ctrl=ctrl, time=3.)
    declaration = dict(initial_saved_state_index=0, canonical_parent_saved_state_index=0,
        parent_trace_sha256=chain.sha(parent), state_parent_trace_sha256=chain.sha(state),
        initial_saved_state_time_s=3.,
        initial_qpos_qvel_sha256=chain.hashlib.sha256(qpos.tobytes()+qvel.tobytes()).hexdigest())
    (tmp_path/'declaration.json').write_text(json.dumps(declaration))
    (tmp_path/'diagnostic_source.py').write_text('original diagnostic bytes\n')
    return parent, state, qpos


def test_actual_cold_parent_qpos_and_native_motor_controls_are_verified(tmp_path):
    parent, state, _ = fixture(tmp_path)
    result = chain.verify_cold_chain(tmp_path, parent, state)
    assert result['exact_qpos_qvel_and_reconstructed_ctrl_verified']
    assert result['actual_initial_relative_wrapped_yaw_rad'] == pytest.approx(-.4)
    assert result['actual_relative_yaw_equals_raw_plus_constant_rad'] == pytest.approx(-.7)
    assert not result['full_trajectory_qualified']


def test_wrong_cold_parent_bytes_are_rejected_even_if_model_parent_matches(tmp_path):
    parent, state, qpos = fixture(tmp_path)
    qpos[0] += .0001
    np.savez(state, qpos=[qpos], qvel=[np.zeros(12)], info_json='[]')
    with pytest.raises(AssertionError):
        chain.verify_cold_chain(tmp_path, parent, state)


def test_finger_ctrl_is_preserved_while_arm_ctrl_comes_from_native_state_parent(tmp_path):
    parent, state, _ = fixture(tmp_path)
    initial = tmp_path/'declared_cold_initialization.npz'
    values = dict(np.load(initial, allow_pickle=False))
    values['ctrl'][-1] += .01
    np.savez(initial, **values)
    with pytest.raises(AssertionError):
        chain.verify_cold_chain(tmp_path, parent, state)


@pytest.mark.parametrize('offset', [-.7, -.6])
def test_yaw_correction_note_must_match_independent_initial_quaternion_composition(tmp_path, offset):
    parent, state, _ = fixture(tmp_path)
    note = dict(state_initialization_sha256=chain.sha(tmp_path/'declared_cold_initialization.npz'),
        source_sha256=chain.sha(tmp_path/'diagnostic_source.py'),
        actual_relative_yaw_equals_raw_plus_constant_rad=offset)
    note_path = tmp_path/'note.json'
    note_path.write_text(json.dumps(note))
    if offset == -.7:
        assert chain.verify_cold_chain(tmp_path, parent, state, yaw_note=note_path)['yaw_origin_note']['independently_verified']
    else:
        with pytest.raises(AssertionError):
            chain.verify_cold_chain(tmp_path, parent, state, yaw_note=note_path)
