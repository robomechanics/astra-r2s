"""Original native table/left full wrenches, without contact-force replay."""
from pathlib import Path
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def block_wrench(contact, block_geom_names):
    g1, g2 = contact['geom1'], contact['geom2']
    assert (g1 in block_geom_names) != (g2 in block_geom_names)
    sign = -1. if g1 in block_geom_names else 1.
    local = np.asarray(contact['local_contact_force_N_Nm'])
    frame = np.asarray(contact['frame'])
    force = sign*frame.T@local[:3]
    torque = sign*frame.T@local[3:]+np.cross(
        np.asarray(contact['contact_position_world_m'])-contact['block_origin_world_m'], force)
    assert np.max(abs(force-np.asarray(contact['signed_force_contribution_on_block_world_N']))) < 1e-10
    return np.r_[force, torque]


def audit(path, parent):
    path, parent = Path(path), Path(parent)
    raw = np.load(path/'original_native_force_ledger.npz', allow_pickle=False)
    saved = np.load(path/'checkpoint_trace.npz', allow_pickle=False)
    rows = json.loads(str(saved['info_json'].item()))
    declaration = json.loads((path/'declaration.json').read_text())
    meta = json.loads(str(np.load(parent, allow_pickle=False)['metadata_json'].item()))
    xml = ET.fromstring((path/'scene.xml').read_text())
    assert sha(parent) == declaration['parent_trace_sha256']
    assert sha(path/'scene.xml') == declaration['parent_model']['model_xml_sha256']
    block_geoms = {geom.get('name') for geom in xml.find(".//body[@name='fixture_block']").iter('geom')}
    names = meta['table_support_geom_names']
    dt = float(xml.find('option').get('timestep'))
    maximum_error = 0.
    counts = {'table': 0, 'left': 0}
    full_table, full_left = [], []
    peak_force_command = peak_torque_command = peak_motor_ratio = 0.
    motor_caps = {side: np.array([float(xml.find(f".//actuator/motor[@name='{side}_servo{i}']").get('ctrlrange').split()[1])
        for i in range(1, 7)]) for side in ('left', 'right')}
    for row in rows:
        index = round((row['time_s']-raw['time_s'][0])/dt)
        assert abs(row['time_s']-raw['time_s'][index]) < 1e-9
        table, left = np.zeros(6), np.zeros(6)
        for c in row['native_table_contact_records']:
            assert c['table_geom'] in names and c['block_geom'] in block_geoms
            table += block_wrench(c, block_geoms)
            counts['table'] += 1
        for c in row['native_left_contact_records']:
            assert c['geom1'].startswith('left_') != c['geom2'].startswith('left_')
            left += block_wrench(c, block_geoms)
            counts['left'] += 1
        maximum_error = max(maximum_error, abs(table[2]-raw['table_upward_force_N'][index]),
            abs(left[2]-raw['left_hand_upward_force_N'][index]),
            float(np.max(abs(left-np.asarray(row['left_hand_wrench_world_N_Nm'])))))
        full_table.append(table)
        full_left.append(left)
        for side in ('left', 'right'):
            wrench = np.asarray(row[f'{side}_motor_wrench_N_Nm'])
            peak_force_command = max(peak_force_command, float(np.linalg.norm(wrench[:3])))
            peak_torque_command = max(peak_torque_command, float(np.linalg.norm(wrench[3:])))
            peak_motor_ratio = max(peak_motor_ratio, float(np.max(abs(np.asarray(row[f'{side}_motor_torques_Nm']))/motor_caps[side])))
    assert maximum_error < 1e-10
    assert peak_force_command <= 8.+1e-10 and peak_torque_command <= 2.+1e-10 and peak_motor_ratio <= 1.+1e-10
    n = round(.1/dt)
    sample_table, sample_left = np.asarray(full_table), np.asarray(full_left)
    weight = meta['known_block_weight_N']
    return dict(scope='Original contact-local table/left forces, frame/action-reaction signs and full moments re-transformed. No force solving or native replay. All-step scalar history remains separate from sparse original local-force archives.',
        auditor_source_sha256=sha(__file__), original_ledger_sha256=sha(path/'original_native_force_ledger.npz'),
        original_trace_sha256=sha(path/'checkpoint_trace.npz'), original_sample_rows=len(rows),
        original_contact_solves_retransformed=counts, maximum_original_wrench_scalar_error_N_Nm=maximum_error,
        native_sampled_left_wrench_range_N_Nm=[sample_left.min(axis=0).tolist(), sample_left.max(axis=0).tolist()],
        native_sampled_table_wrench_range_N_Nm=[sample_table.min(axis=0).tolist(), sample_table.max(axis=0).tolist()],
        observed_finite_motor_commands=dict(maximum_cartesian_force_norm_N=peak_force_command,
            maximum_cartesian_torque_norm_Nm=peak_torque_command, maximum_native_motor_command_fraction_of_caps=peak_motor_ratio,
            scope='Bounds apply to commanded motor wrench; actual solved contact reactions include inertia/contact coupling and need not equal the commanded preload.'),
        original_final100ms=dict(mean_signed_left_upward_N=float(raw['left_hand_upward_force_N'][-n:].mean()),
            mean_positive_left_upward_N=float(np.maximum(raw['left_hand_upward_force_N'][-n:], 0.).mean()),
            mean_table_upward_compressive_load_N=float(raw['table_upward_force_N'][-n:].mean()),
            mean_table_load_in_units_of_block_weight=float(raw['table_upward_force_N'][-n:].mean()/weight),
            scope='More than1 unit of block weight denotes extra compressive clamp/contact load, not a fractional gravity share.'),
        strict_all_step_pad_history=dict(left_minimum_N=[float(raw[f'left_pad_{i}_N'].min()) for i in (0, 1)],
            left_unloaded_native_ticks=[int(np.sum(raw[f'left_pad_{i}_N'] <= .1)) for i in (0, 1)],
            right_minimum_N=[float(raw[f'right_pad_{i}_N'].min()) for i in (0, 1)]),
        original_final_native_loads=dict(left_upward_N=float(raw['left_hand_upward_force_N'][-1]),
            table_upward_N=float(raw['table_upward_force_N'][-1])),
        original_guard_failure=json.loads((path/'report.json').read_text())['aborted'],
        full_trajectory_qualified=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    parser.add_argument('--parent', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = audit(args.directory, args.parent)
    Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2))
