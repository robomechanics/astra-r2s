"""Read original cold-diagnostic forces and archived source bytes; no simulation.

This additive auditor does not alter the frozen historical diagnostic auditors.
Provide the exact parent trace explicitly. No current application file is used.
"""
from pathlib import Path
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rolling(values, count):
    values = np.asarray(values, dtype=float)
    prefix = np.r_[0., np.cumsum(values)]
    return (prefix[count:]-prefix[:-count])/count


def native_weight_windows(raw, weight, dt, *, require_interior=False):
    """Exact last100ms from every original solve, with positive-hand clipping."""
    count = round(.1/dt)
    if len(raw['time_s']) < count:
        return None
    bad = ((abs(raw['relative_bolt_axial_velocity_m_per_s']) > .0002)
        | (raw['radial_offset_m'] > 150e-6)
        | (raw['bolt_tilt_rad'] > np.deg2rad(2))
        | (raw['external_drive_zero'] != 1)
        | (raw['bolt_world_support_contact_count'] != 0)
        | (raw['nonthread_block_bolt_contact_count'] != 0)
        | (raw['all_checks_held'] != 1))
    if require_interior:
        bad |= ((raw['formed_flank_overlap_m'] <= 0)
            | (raw['loaded_actual_interior_flank_contact_count'] <= 0))
    thread = rolling(raw['thread_gravity_opposing_force_N'], count)/weight
    hand = rolling(np.maximum(raw['hand_gravity_opposing_force_N'], 0.), count)/weight
    good = rolling(bad.astype(float), count) == 0
    ready = (good & (thread >= .9) & (hand <= .1)
        & (raw['thread_gravity_opposing_force_N'][count-1:] > .1*weight))
    return dict(count=count, thread=thread, positive_hand=hand, good=good, ready=ready)


def transformed_thread_contacts(row):
    """Independent action/reaction and lever arm; recorded interior tags separate."""
    total, interior = np.zeros(6), np.zeros(6)
    maximum_error = 0.
    loaded_interior = 0
    for c in row['native_contact_records']:
        assert {c['geom1'], c['geom2']} == {'bolt_thread', 'female_thread'}
        local, frame = np.asarray(c['local_force_N_Nm']), np.asarray(c['frame'])
        sign = -1. if c['geom1'] == 'bolt_thread' else 1.
        force = sign*frame.T@local[:3]
        torque = sign*frame.T@local[3:]+np.cross(
            np.asarray(c['contact_position_world_m'])-c['bolt_origin_world_m'], force)
        wrench = np.r_[force, torque]
        maximum_error = max(maximum_error, float(np.max(abs(
            wrench-np.asarray(c['wrench_on_bolt_world_N_Nm'])))))
        total += wrench
        if c['is_actual_interior_flank_contact']:
            interior += wrench
            loaded_interior += local[0] > 1e-5
    return total, interior, int(loaded_interior), maximum_error


def window_summary(window, raw, weight, dt):
    if window is None:
        return dict(observed=False, ready=False, ever_ready=False)
    n, ready = window['count'], window['ready']
    indices = np.flatnonzero(ready)+n-1
    return dict(observed=True, duration_s=n*dt, original_native_samples=n,
        mean_signed_thread_weight_fraction=float(window['thread'][-1]),
        mean_positive_hand_upward_weight_fraction=float(window['positive_hand'][-1]),
        mean_signed_hand_weight_fraction=float(raw['hand_gravity_opposing_force_N'][-n:].mean()/weight),
        contiguous_geometry_motion_drive_guard_window=bool(window['good'][-1]),
        final_thread_force_loaded=bool(raw['thread_gravity_opposing_force_N'][-1] > .1*weight),
        loaded_thread_gravity_support_duty=float(np.mean(raw['thread_gravity_opposing_force_N'][-n:] > .1*weight)),
        ready=bool(ready[-1]), ever_ready=bool(np.any(ready)),
        ready_endpoint_count=len(indices),
        first_ready_elapsed_s=float(raw['elapsed_s'][indices[0]]) if len(indices) else None,
        last_ready_elapsed_s=float(raw['elapsed_s'][indices[-1]]) if len(indices) else None,
        scope='Individual qualifying trailing windows; earliest/latest do not assert a continuous ready interval. No engagement or unsupported-reset qualification.')


def audit(path, parent, state_parent=None):
    path, parent = Path(path), Path(parent)
    declaration = json.loads((path/'declaration.json').read_text())
    producer = json.loads((path/'report.json').read_text())
    saved_parent = np.load(parent, allow_pickle=False)
    parent_meta = json.loads(str(saved_parent['metadata_json'].item()))
    raw_archive = np.load(path/'original_native_force_ledger.npz', allow_pickle=False)
    raw = {key: raw_archive[key] for key in raw_archive.files}
    saved = np.load(path/'checkpoint_trace.npz', allow_pickle=False)
    samples = json.loads(str(saved['info_json'].item()))
    xml = ET.fromstring((path/'scene.xml').read_text())
    option = xml.find('option')
    dt = float(option.attrib['timestep'])
    gravity = np.fromstring(option.attrib['gravity'], sep=' ')
    up = -gravity/np.linalg.norm(gravity)
    weight = float(declaration['bolt_weight_N'])
    xml_bolt_mass = float(xml.find(".//body[@name='male_bolt']/inertial").attrib['mass'])
    assert np.isclose(weight, xml_bolt_mass*np.linalg.norm(gravity), rtol=1e-12, atol=1e-12)
    identity = dict(parent_trace_matches=sha(parent) == declaration['parent_trace_sha256'],
        archived_xml_matches=sha(path/'scene.xml') == declaration['parent_model']['model_xml_sha256'],
        archived_xml_equals_parent=sha(path/'scene.xml') == sha(parent.parent/'scene.xml'),
        archived_scene_bundle_equals_parent=sha(path/'supported_scene.zip') == sha(parent.parent/'supported_scene.zip'),
        loaded_observer_copy_matches=sha(path/'observer_source.py') == declaration['observer_sha256'],
        archived_observer_dependency_matches=sha(path/'recorded_sources/yam_twin/m8_supported_start.py') == declaration['observer_sha256'],
        archived_diagnostic_matches=sha(path/'diagnostic_source.py') == declaration['diagnostic_source_sha256'],
        producer_ledger_sha_matches=sha(path/'original_native_force_ledger.npz') == producer['ledger_sha256'],
        producer_trace_sha_matches=sha(path/'checkpoint_trace.npz') == producer['trace_sha256'])
    parent_sources = {**{name.removeprefix('recorded_sources/'): digest for name, digest
        in parent_meta['recorded_source_dependencies_sha256'].items()},
        'yam_twin/m8_supported_simulation.py': parent_meta['controller_module_sha256'],
        'yam_twin/m8_supported_scene.py': parent_meta['scene_source_sha256']}
    source_checks = {}
    for name, digest in declaration['parent_source'].items():
        expected = parent_sources.get(name)
        source_checks[name] = bool(expected == digest and sha(path/'recorded_sources'/name) == digest)
    assert len(source_checks) == len(parent_sources)
    assert all(identity.values()) and all(source_checks.values())
    initial = np.load(path/'declared_cold_initialization.npz', allow_pickle=False)
    initial_index = declaration['initial_saved_state_index']
    cold_chain = None
    if state_parent is None:
        assert np.array_equal(initial['qpos'], saved_parent['qpos'][initial_index])
        assert np.array_equal(initial['qvel'], saved_parent['qvel'][initial_index])
        assert np.array_equal(initial['ctrl'], saved_parent['controller'][initial_index])
    else:
        from independent_cold_chain_audit import verify_cold_chain
        cold_chain = verify_cold_chain(path, parent, state_parent)
        from independent_yaw_counter_origin import verify_recorded_counter_origin
        cold_chain = verify_recorded_counter_origin(path, cold_chain)
    assert hashlib.sha256(initial['qpos'].tobytes()+initial['qvel'].tobytes()).hexdigest() == declaration['initial_qpos_qvel_sha256']
    time = raw['time_s']
    assert all(len(value) == len(time) and np.isfinite(value).all() for value in raw.values())
    assert np.allclose(np.diff(time), dt, rtol=1e-7, atol=1e-10)
    assert abs(time[0]-declaration['initial_saved_state_time_s']-dt) < 1e-10
    assert np.allclose(raw['elapsed_s'], np.arange(1, len(time)+1)*dt, rtol=1e-10, atol=1e-10)
    assert all(row['source_sha256'] == declaration['observer_sha256'] for row in samples)
    force_errors, allstep_errors, detailed_count, missing = [], [], 0, []
    for row in samples:
        index = round((row['time_s']-time[0])/dt)
        assert abs(time[index]-row['time_s']) < 1e-9
        total, interior, loaded, error = transformed_thread_contacts(row)
        force_errors.append(error)
        # Older producers did not request contact-local data on one final row.
        # Current seat harness explicitly records its final native contacts.
        detailed = (index % 100 == 0 or bool(row['native_contact_records'])
            or row['native_thread_contact_count'] == 0)
        if detailed:
            detailed_count += 1
            force_errors.append(float(np.max(abs(total-np.asarray(row['thread_wrench_on_bolt_world_N_Nm'])))))
            allstep_errors += [abs(float(total[:3]@up)-raw['thread_gravity_opposing_force_N'][index]),
                abs(float(interior[:3]@up)-raw['interior_flank_gravity_opposing_force_N'][index]),
                abs(loaded-raw['loaded_actual_interior_flank_contact_count'][index])]
        else:
            missing.append(index)
        for scalar, vector in [('hand_gravity_opposing_force_N', 'hand_wrench_on_bolt_world_N_Nm'),
                ('right_pad_gravity_opposing_force_N', 'right_pad_wrench_on_bolt_world_N_Nm')]:
            allstep_errors.append(abs(float(np.asarray(row[vector])[:3]@up)-raw[scalar][index]))
    assert max(force_errors, default=0.) < 1e-10 and max(allstep_errors, default=0.) < 1e-10
    windows = native_weight_windows(raw, weight, dt)
    partial = native_weight_windows(raw, weight, dt, require_interior=True)
    phases = {}
    actual_yaw = raw['yaw_unwrapped_rad']+(cold_chain['actual_relative_yaw_equals_raw_plus_constant_rad'] if cold_chain else 0.)
    names = declaration.get('phase_names', ['whole_diagnostic'])
    labels = raw.get('phase_index', np.zeros(len(time), dtype=int)).astype(int)
    for index, name in enumerate(names):
        mask = labels == index
        if not mask.any():
            phases[name] = dict(observed=False)
            continue
        phases[name] = dict(observed=True, original_native_steps=int(mask.sum()),
            first_elapsed_s=float(raw['elapsed_s'][mask][0]), last_elapsed_s=float(raw['elapsed_s'][mask][-1]),
            base_z_range_m=[float(raw['base_z_m'][mask].min()), float(raw['base_z_m'][mask].max())],
            actual_yaw_range_rad=[float(actual_yaw[mask].min()), float(actual_yaw[mask].max())],
            original_recorded_yaw_counter_range_rad=[float(raw['yaw_unwrapped_rad'][mask].min()), float(raw['yaw_unwrapped_rad'][mask].max())],
            maximum_formed_full_ring_overlap_m=float(raw['formed_flank_overlap_m'][mask].max()),
            loaded_actual_interior_substeps=int(np.sum(raw['loaded_actual_interior_flank_contact_count'][mask] > 0)),
            radial_range_m=[float(raw['radial_offset_m'][mask].min()), float(raw['radial_offset_m'][mask].max())])
    count = round(.1/dt)
    table = None
    if len(time) >= count:
        block_weight = float(parent_meta['known_block_weight_N'])
        table_mean = rolling(raw['table_upward_force_N'], count)/block_weight
        left_mean = rolling(np.maximum(raw['left_hand_upward_force_N'], 0.), count)/block_weight
        duty = rolling((raw['table_upward_force_N'] > .1*block_weight).astype(float), count)
        good = (table_mean >= .9) & (left_mean <= .1) & (duty >= .99) & (raw['table_upward_force_N'][count-1:] > .1*block_weight)
        table = dict(minimum_rolling_weight_fraction=float(table_mean.min()),
            maximum_positive_left_upward_weight_fraction=float(left_mean.max()), minimum_loaded_duty=float(duty.min()),
            failed_rolling_window_endpoints=int(np.sum(~good)),
            strict_unloaded_native_ticks=int(np.sum(raw['table_upward_force_N'] <= .1*block_weight)))
    return dict(scope='Original all-step scalar forces and original sampled contact-local forces, without integration, model recompilation or force replay. Cold diagnostic only. Interior tags are archived producer geometry classifications, not independently rebuilt body transforms.',
        auditor_source_sha256=sha(__file__), input_sha256={name: sha(path/name) for name in
            ['declaration.json', 'diagnostic_source.py', 'observer_source.py', 'report.json', 'original_native_force_ledger.npz', 'checkpoint_trace.npz', 'declared_cold_initialization.npz']},
        identity={**identity, 'archived_source_checks': source_checks},
        cold_initialization_matches_parent_saved_post_state=state_parent is None,
        cold_chain=cold_chain, missing_original_warmstart_state=True,
        original_native_steps=len(time), actual_duration_s=len(time)*dt,
        force_frame=dict(original_sampled_local_solves_retransformed=detailed_count,
            extra_final_rows_without_contact_local_archive=missing,
            maximum_wrench_error_N_Nm=max(force_errors, default=0.), maximum_scalar_error=max(allstep_errors, default=0.),
            right_pad_scope='Original right-pad world wrench reprojected; individual original pad-local solves are not archived here.'),
        independent_exact100ms_weight_transfer=window_summary(windows, raw, weight, dt),
        independent_exact100ms_continuously_formed_support=window_summary(partial, raw, weight, dt),
        table_support=table, observed_phases=phases,
        all_native_diagnostic_checks_held=bool(np.all(raw['all_checks_held'] == 1)),
        producer_original_observer_window=producer['final_weight_window'],
        producer_failure=producer['aborted'], capture_qualified=False, full_trajectory_qualified=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    parser.add_argument('--parent', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--state-parent')
    args = parser.parse_args()
    result = audit(args.directory, args.parent, args.state_parent)
    Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: result[k] for k in ['force_frame', 'independent_exact100ms_weight_transfer',
        'independent_exact100ms_continuously_formed_support', 'table_support', 'observed_phases', 'producer_failure']}, indent=2))
