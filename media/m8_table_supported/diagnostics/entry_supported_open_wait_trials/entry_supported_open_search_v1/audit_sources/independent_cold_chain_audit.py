"""Verify a second cold state's declared lineage and yaw origin without MuJoCo."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def free_joint_addresses(xml):
    addresses, offset = {}, 0
    for body in xml.find('worldbody').iter('body'):
        for joint in body:
            if joint.tag not in ('joint', 'freejoint'):
                continue
            kind = 'free' if joint.tag == 'freejoint' else joint.get('type', 'hinge')
            if kind == 'free':
                addresses[joint.get('name')] = offset
            offset += {'free': 7, 'ball': 4}.get(kind, 1)
    return addresses


def quaternion_matrix(wxyz):
    w, x, y, z = wxyz
    return Rotation.from_quat([x, y, z, w]).as_matrix()


def relative_bolt_yaw(xml, qpos):
    addresses = free_joint_addresses(xml)
    male, block = addresses['male_bolt_free'], addresses['fixture_block_free']
    bolt_R = quaternion_matrix(qpos[male+3:male+7])
    block_R = quaternion_matrix(qpos[block+3:block+7])
    female_R = quaternion_matrix(np.fromstring(
        xml.find(".//body[@name='female_frame']").get('quat'), sep=' '))
    relative = (block_R@female_R).T@bolt_R
    return float(np.arctan2(relative[1, 0], relative[0, 0]))


def verify_cold_chain(path, canonical_parent, state_parent, *, yaw_note=None):
    path, canonical_parent, state_parent = map(Path, (path, canonical_parent, state_parent))
    declaration = json.loads((path/'declaration.json').read_text())
    canonical = np.load(canonical_parent, allow_pickle=False)
    states = np.load(state_parent, allow_pickle=False)
    state_rows = json.loads(str(states['info_json'].item()))
    initial = np.load(path/'declared_cold_initialization.npz', allow_pickle=False)
    index = declaration['initial_saved_state_index']
    canonical_index = declaration['canonical_parent_saved_state_index']
    xml = ET.fromstring((path/'scene.xml').read_text())
    assert sha(canonical_parent) == declaration['parent_trace_sha256']
    assert sha(state_parent) == declaration['state_parent_trace_sha256']
    assert np.array_equal(initial['qpos'], states['qpos'][index])
    assert np.array_equal(initial['qvel'], states['qvel'][index])
    assert float(initial['time']) == state_rows[index]['time_s'] == declaration['initial_saved_state_time_s']
    assert hashlib.sha256(initial['qpos'].tobytes()+initial['qvel'].tobytes()).hexdigest() == declaration['initial_qpos_qvel_sha256']
    motor_names = [motor.get('name') for motor in xml.find('actuator')]
    expected_ctrl = canonical['controller'][canonical_index].copy()
    for side in ('left', 'right'):
        indices = [motor_names.index(f'{side}_servo{i}') for i in range(1, 7)]
        expected_ctrl[indices] = state_rows[index][f'{side}_motor_torques_Nm']
    assert np.array_equal(initial['ctrl'], expected_ctrl)
    source_rows = json.loads(str(canonical['info_json'].item()))
    retained_origin = float(source_rows[canonical_index]['bolt_yaw_unwrapped_rad'])
    actual_yaw = relative_bolt_yaw(xml, initial['qpos'])
    correction = actual_yaw-retained_origin
    note_result = None
    if yaw_note is not None:
        yaw_note = Path(yaw_note)
        note = json.loads(yaw_note.read_text())
        assert note['state_initialization_sha256'] == sha(path/'declared_cold_initialization.npz')
        assert note['source_sha256'] == sha(path/'diagnostic_source.py')
        assert abs(note['actual_relative_yaw_equals_raw_plus_constant_rad']-correction) < 1e-12
        note_result = dict(sha256=sha(yaw_note), independently_verified=True)
    return dict(scope='Separate cold diagnostic state/model parents explicitly verified. Quaternion composition from declared native free-joint qpos and exact local female frame; no native replay, integration or force reconstruction.',
        auditor_source_sha256=sha(__file__), state_parent_trace_sha256=sha(state_parent),
        canonical_model_reference_parent_sha256=sha(canonical_parent),
        exact_qpos_qvel_and_reconstructed_ctrl_verified=True,
        original_warmstart_solver_state_missing=True,
        actual_initial_relative_wrapped_yaw_rad=actual_yaw,
        retained_canonical_counter_origin_rad=retained_origin,
        actual_relative_yaw_equals_raw_plus_constant_rad=correction,
        original_raw_counter_is_unchanged=True, yaw_origin_note=note_result,
        full_trajectory_qualified=False)
