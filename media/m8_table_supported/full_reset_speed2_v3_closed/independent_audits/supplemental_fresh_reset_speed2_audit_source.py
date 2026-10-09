"""Outcome-neutral supplemental reader, executable only after fresh v3 closure.

This file is prepared while the native child is live. Its execution is deferred
until AFTER, source identity, and all three official audit records are closed.
No MuJoCo import, model construction, contact solve or integration occurs here.
"""
import argparse
import hashlib
import json
from pathlib import Path
import types

PRODUCER = '9ae1a9fe76968a4013ea6c39e67718026622b9c0'
BEFORE = '1aceb2b906b6201ba715bde99dfe1394bb0880907b0f9914529a9c50adff1ec9'
PROOF = '9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7'
SERIAL = 'e532c407cd35b51ad3a55bcbbdf98fc30676e8e994f427895c6a813875081e2c'
OFFICIAL = '4a8b018a1326bce540a0351d5d715ab58ffb505d2ae44be6d34aa8d8cd68fe3f'
ARITHMETIC = 'b714fed754d21b4a4684394802912b33b6f562b4e3936e90d70e3ad56fb23a0e'


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def read_json(path):
    return json.loads(Path(path).read_text())


def closure_gate(run, official_dir):
    """Refuse absent AFTER before opening any state/force archive or importing it."""
    after_path = run/'run_publication_identity_after.json'
    require(after_path.is_file(), 'Native child has no closed AFTER identity; do not read its arrays')
    after = read_json(after_path)
    before_path = run/'run_publication_identity.json'
    require(digest(before_path) == BEFORE, 'Original fresh v3 BEFORE bytes differ')
    before = read_json(before_path)
    require(after.get('producer_commit') == before.get('producer_commit') == PRODUCER,
            'Wrong fresh v3 producer')
    require(type(after.get('native_exit_code')) is int, 'Native exit is an integer outcome, not a success flag')
    for key, value in before.items():
        require(after.get(key) == value, 'AFTER altered inherited BEFORE field: '+key)
    identities = {}
    for name in ('source_identity_before.json', 'source_identity_after.json'):
        identity = read_json(official_dir/name)
        require(identity['producer_commit'] == PRODUCER
                and identity['before_identity_sha256'] == BEFORE
                and identity['after_identity_sha256'] == digest(after_path)
                and identity['closure_native_exit_code'] == after['native_exit_code'],
                'Closed official identity does not bind the original native outcome')
        require(identity['complete_current_and_archived_tested_sources_match_original'] is True
                and identity['source_files_unchanged'] == 74
                and identity['software_tests_passed'] == 672
                and identity['software_proof_sha256'] == PROOF,
                'Original 74-source / 672-test producer identity did not pass')
        require(identity['frozen_audit_sources']['scripts/audit_m8_supported_trace.py'] == OFFICIAL,
                'Official supported reader identity differs')
        identities[name] = identity
    require(identities['source_identity_before.json']['software_proof_sha256']
            == identities['source_identity_after.json']['software_proof_sha256'],
            'Software proof changed during official audit execution')
    declaration = read_json(official_dir/'serial_audit_declaration.json')
    serial = read_json(official_dir/'serial_audit_result.json')
    require(declaration['schema'] == 'closed-supported-serial-audit-v3-declaration'
            and declaration['wrapper_sha256'] == SERIAL
            and digest(official_dir/'serial_audit_launcher_source.py') == SERIAL
            and declaration['producer'] == PRODUCER
            and declaration['original_before_sha256'] == BEFORE
            and declaration['software_proof_sha256'] == PROOF,
            'Closed serial wrapper declaration/source differs')
    require(serial['schema'] == 'closed-supported-serial-audit-v3-result'
            and serial['reader_execution_completed'] is True
            and serial['identity_and_original_bytes_unchanged'] is True
            and serial['original_native_exit_code'] == after['native_exit_code']
            and serial['original_after_sha256'] == digest(after_path)
            and serial['original_trace_sha256'] == declaration['original_trace_sha256'],
            'Official readers have not completed intact; execution is separate from physics outcome')
    official_records = {}
    for label, name in (('supported', 'independent_supported_audit'),
                        ('left_pad', 'independent_left_pad_force_history_audit'),
                        ('free_joint', 'independent_free_joint_properties')):
        record = read_json(official_dir/(label+'_execution.json'))
        report_path = official_dir/(name+'.json')
        require(record['status'] == 'exited' and record['exit_code'] == 0
                and record['output_sha256'] == digest(report_path)
                and record['original_after_sha256_before'] == record['original_after_sha256_after'] == digest(after_path)
                and record['original_trace_sha256_before'] == record['original_trace_sha256_after'] == serial['original_trace_sha256']
                and record['source_modules_unchanged'] is True
                and record['original_after_unchanged'] is True
                and record['original_trace_unchanged'] is True
                and not record['inspection_exceptions']
                and record['log_sha256'] == digest(official_dir/(label+'_stdout_stderr.log')),
                'Official serial audit has not completed intact: '+name)
        require(record == next(r for r in serial['readers'] if r['reader'] == label),
                'Serial result differs from its original per-reader execution record')
        official_records[label] = record
    return after, identities, official_records


def load_arithmetic(path):
    """Import only the exactly frozen old pure helper, after the closure gate."""
    require(digest(path) == ARITHMETIC, 'Frozen historical arithmetic source bytes differ')
    module = types.ModuleType('frozen_old_open_reset_arithmetic_b714')
    module.__file__ = str(path)
    exec(compile(Path(path).read_bytes(), str(path), 'exec'), module.__dict__)
    return module


def audit(run, official_dir, arithmetic_path):
    run, official_dir, arithmetic_path = map(lambda p: Path(p).resolve(),
                                             (run, official_dir, arithmetic_path))
    after, identities, executions = closure_gate(run, official_dir)
    reader_sha = digest(__file__)
    old = load_arithmetic(arithmetic_path)
    np = old.np
    original = read_json(run/'insertion_validation.json')
    official = read_json(official_dir/'independent_supported_audit.json')
    require(official['auditor_source_sha256'] == OFFICIAL
            and digest(run/'frozen_audit_sources/scripts/audit_m8_supported_trace.py') == OFFICIAL,
            'Closed official supported source bytes differ')
    require(official['original_report_passed'] == original['passed']
            and official['original_acceptance_checks'] == original['acceptance_checks'],
            'Official reader changed the original acceptance outcome')
    ff_official = official['original_physical_feedback_history']['original_robot_inertia_history']
    require(type(ff_official['passed']) is bool, 'Official FF/cache outcome must be retained as a boolean')
    dt = float(original['scene_config']['base']['thread']['timestep'])
    require(np.isfinite(dt) and dt > 0, 'Invalid original native timestep')
    raw_hashes = {}

    def load(declaration_name, keys):
        declaration = original[declaration_name]
        filename = declaration['filename']
        require(Path(filename).name == filename, 'Raw force/command ledger must be a sibling filename')
        path = run/filename
        raw_hashes[filename] = digest(path)
        require(raw_hashes[filename] == declaration['sha256'], 'Original raw ledger SHA differs: '+filename)
        with np.load(path, allow_pickle=False) as saved:
            columns = {name:saved[name] for name in keys}
            labels = (json.loads(str(saved['metadata_json']))['phase_labels']
                      if declaration_name == 'robot_inertia_command_history'
                      else json.loads(str(saved['phase_labels_json'])))
        return columns, labels

    feedback, labels = load('native_feedback_force_history', (
        'time', 'phase_index', 'right_robot_bolt_contact_count', 'right_pad_normal_force_N',
        'all_hard_guards_held', 'fully_open_unassisted', 'right_grasp_guard_active',
        'right_actual_aperture_postintegration_m', 'right_command_wrench_N_Nm',
        'right_motor_torques_Nm', 'desired_independent_clock_rad',
        'open_peak_axial_drift_m', 'open_peak_yaw_drift_rad',
        'thread_gravity_opposing_force_N', 'hand_gravity_opposing_force_N',
        'external_drive_zero', 'loaded_actual_interior_flank_contact_count',
        'right_applied_axial_feed_N', 'right_axial_float_active', 'open_weight_window_ready',
        'actual_quiet_regrasp_streak_s', 'right_grip_slip_m', 'right_grip_rotation_slip_rad'))
    commands, command_labels = load('robot_inertia_command_history', (
        'time', 'phase_index', 'enabled', 'inertia_inputs_present', 'jacobian_derivatives_present',
        'pd_wrench_world_N_Nm', 'ff_wrench_world_N_Nm', 'combined_uncapped_wrench_world_N_Nm',
        'capped_wrench_world_N_Nm', 'cartesian_force_clipped', 'cartesian_torque_clipped',
        'motor_clipped', 'motor_torques_Nm'))
    table, table_labels = load('table_support_force_history', (
        'time', 'phase_index', 'pad_normal_force_N', 'supported_task_active',
        'table_wrench_world_at_block_origin_N_Nm', 'left_hand_wrench_world_at_block_origin_N_Nm',
        'bolt_base_insertion_m', 'bolt_yaw_unwrapped_rad', 'formed_flank_overlap_m',
        'loaded_formed_thread_normal_force_N', 'loaded_formed_thread_contact_count'))
    time = feedback['time']; count = len(time)
    require(count > 0 and np.isfinite(time).all() and np.all(np.abs(np.diff(time)-dt) < max(1e-10, dt*1e-6))
            and abs(time[0]-dt) < 1e-10, 'Original all-step native coverage differs')
    require(labels == command_labels == table_labels, 'Original ledger phase labels differ')
    for other in (commands, table):
        require(np.array_equal(other['time'], time)
                and np.array_equal(other['phase_index'], feedback['phase_index']),
                'Original command/force timing or phase labels differ')
    require(count == original['native_feedback_force_history']['observed_physics_steps'],
            'Original declared native step count differs')
    require(raw_hashes[original['native_feedback_force_history']['filename']]
            == official['original_physical_feedback_history']['raw_sha256'],
            'Official feedback report is bound to different original bytes')
    require(raw_hashes[original['robot_inertia_command_history']['filename']] == ff_official['raw_sha256'],
            'Official FF/cache result is bound to different original bytes')
    require(np.array_equal(commands['capped_wrench_world_N_Nm'], feedback['right_command_wrench_N_Nm'])
            and np.array_equal(commands['motor_torques_Nm'], feedback['right_motor_torques_Nm']),
            'Original executed robot and feedback command rows differ')
    names = np.asarray(labels, dtype=object)[feedback['phase_index']]
    opened = feedback['fully_open_unassisted']
    first_contact = old.first_open_contact(opened, feedback['right_robot_bolt_contact_count'])
    if np.any(opened):
        require(not np.any(commands['enabled'][opened])
                and not np.any(commands['inertia_inputs_present'][opened])
                and not np.any(commands['jacobian_derivatives_present'][opened])
                and not np.any(commands['ff_wrench_world_N_Nm'][opened])
                and not np.any(feedback['right_axial_float_active'][opened])
                and not np.any(feedback['right_applied_axial_feed_N'][opened]),
                'Original fully-open controls acquired closed-grip FF/axial drive')
    trace = run/'insertion_trace.npz'
    raw_hashes[trace.name] = digest(trace)
    require(raw_hashes[trace.name] == official['trajectory_sha256'], 'Closed official trace bytes differ')
    with np.load(trace, allow_pickle=False) as saved:
        rows = json.loads(str(saved['info_json'])); saved_times = saved['time']
    require(len(rows) == len(saved_times) and rows[-1]['time'] == time[-1],
            'Closed trace must retain its exact original final native row')
    summaries = {row['phase']:row for row in original['phases']}
    pitch = float(original['scene_config']['base']['thread']['pitch'])
    phase_reports = []
    for phase_id, label in enumerate(labels):
        ids = np.flatnonzero(feedback['phase_index'] == phase_id)
        if not len(ids):
            phase_reports.append({'phase':label, 'observed':False, 'native_steps':0})
            continue
        mask = feedback['phase_index'] == phase_id
        previous = max(0, int(ids[0])-1); last = int(ids[-1])
        sample_rows = [r for r in rows if r['phase'] == label]
        guarded = mask & feedback['right_grasp_guard_active'].astype(bool)
        advance = float(table['bolt_base_insertion_m'][last]-table['bolt_base_insertion_m'][previous])
        rotation = float(table['bolt_yaw_unwrapped_rad'][last]-table['bolt_yaw_unwrapped_rad'][previous])
        summary = {'phase':label, 'observed':True,
            **old.phase_baseline(time, feedback['phase_index'], phase_id, dt),
            'native_command_caps':old.command_stats(commands, mask),
            'fully_open_contact_native_ticks':int(np.sum(opened[mask] & (feedback['right_robot_bolt_contact_count'][mask] > 0))),
            'maximum_whole_right_robot_bolt_contact_count':int(feedback['right_robot_bolt_contact_count'][mask].max()),
            'original_hard_guard_failed_ticks':int(np.sum(~feedback['all_hard_guards_held'][mask])),
            'actual_guarded_right_grasp_ticks':int(np.sum(guarded)),
            'maximum_native_guarded_right_grip_slip_m':float(feedback['right_grip_slip_m'][guarded].max()) if guarded.any() else None,
            'maximum_native_guarded_right_rotation_slip_rad':float(feedback['right_grip_rotation_slip_rad'][guarded].max()) if guarded.any() else None,
            'minimum_original_right_pad_load_N':feedback['right_pad_normal_force_N'][mask].min(axis=0).tolist(),
            'aperture_postintegration_minimum_m':float(feedback['right_actual_aperture_postintegration_m'][mask].min()),
            'aperture_postintegration_maximum_m':float(feedback['right_actual_aperture_postintegration_m'][mask].max()),
            'actual_measured_advance_m':advance, 'actual_measured_rotation_rad':rotation,
            'pitch_reference_residual_m':float(advance-pitch*rotation/(2*np.pi)),
            'nominal_formed_overlap_maximum_m':float(table['formed_flank_overlap_m'][mask].max()),
            'actual_loaded_interior_contact_ticks':int(np.sum(feedback['loaded_actual_interior_flank_contact_count'][mask] > 0)),
            'started_with_recorded_capture':bool(summaries.get(label, {}).get('started_engaged', False)),
            'saved_original_tracking_error_samples':len(sample_rows)}
        if ids[0] == 0:
            summary['original_force_baseline_time_s'] = 0.
            summary['baseline_initialization_note'] = 'No preceding solved row exists; the initialized original state is t=0.'
        if sample_rows:
            summary.update(maximum_saved_retained_position_error_norm_m=float(max(np.linalg.norm(r['right_position_error_m']) for r in sample_rows)),
                           maximum_saved_retained_rotation_error_norm_rad=float(max(np.linalg.norm(r['right_rotation_error_rad']) for r in sample_rows)))
        phase_reports.append(summary)
    active = table['supported_task_active'].astype(bool)
    acquisition_reports = []
    for event in original['right_grasp_acquisitions']:
        i = int(np.searchsorted(time, event['time_s']))
        require(i < count and abs(time[i]-event['time_s']) < 1e-9,
                'Declared measured regrasp event is absent from native rows')
        acquisition_reports.append({'original_event':event, 'native_index':i,
            'native_guard_active_on_acquisition_row':bool(feedback['right_grasp_guard_active'][i]),
            'native_guard_active_on_next_row':bool(feedback['right_grasp_guard_active'][i+1]) if i+1 < count else None,
            'original_pad_loads_N':feedback['right_pad_normal_force_N'][i].tolist(),
            'actual_quiet_regrasp_streak_s':float(feedback['actual_quiet_regrasp_streak_s'][i])})
    failures = np.flatnonzero(~feedback['all_hard_guards_held'])
    require(digest(__file__) == reader_sha and digest(arithmetic_path) == ARITHMETIC,
            'Supplemental or reused arithmetic source changed during the selected-array read')
    return {'kind':'Outcome-neutral supplemental closed fresh reset-speed2 original-array audit',
        'reader_source_sha256':reader_sha, 'reader_and_reused_source_unchanged':True,
        'reused_arithmetic_source_sha256':ARITHMETIC,
        'reused_arithmetic_source_path':str(arithmetic_path), 'producer_commit':PRODUCER,
        'native_child_exit_code':after['native_exit_code'], 'original_report_passed':original['passed'],
        'original_partial':original['partial'], 'original_aborted':original['aborted'],
        'original_acceptance_checks':original['acceptance_checks'],
        'independent_official_passed':official['passed'],
        'independent_official_acceptance_checks':official['independent_acceptance_checks'],
        'official_report_sha256':digest(official_dir/'independent_supported_audit.json'),
        'official_FF_cache_caps_mapping_reused':ff_official,
        'official_source_bound_grasp_flags_reused':official['original_physical_feedback_history']['original_grasp_flag_boundaries'],
        'official_actual_events_reused':official['original_physical_feedback_history']['actual_saved_physical_events'],
        'official_qualified_lead_and_passive_reset_rows_reused':official['original_all_step_bolt_phases'],
        'closed_identity_reports_sha256':{name:digest(official_dir/name) for name in identities},
        'official_execution_reports_sha256':{name:digest(official_dir/(name+'_execution.json')) for name in executions},
        'official_serial_result_sha256':digest(official_dir/'serial_audit_result.json'),
        'official_serial_declaration_sha256':digest(official_dir/'serial_audit_declaration.json'),
        'original_raw_files_sha256':raw_hashes, 'original_native_ticks':count,
        'saved_original_post_states':len(rows), 'final_post_label_time_s':float(time[-1]),
        'final_original_force_time_s':float(time[-1]-dt),
        'final_retained_command_pose_time_s':float(time[-1]-2*dt),
        'first_fully_open_contact_native_index':first_contact,
        'first_fully_open_contact_post_label_time_s':float(time[first_contact]) if first_contact is not None else None,
        'first_fully_open_contact_force_time_s':float(time[first_contact]-dt) if first_contact is not None else None,
        'actual_whole_right_robot_fully_open_contact_ticks':int(np.sum(opened & (feedback['right_robot_bolt_contact_count'] > 0))),
        'original_hard_guard_failed_native_indices':failures.tolist(),
        'original_external_drive_zero_all_ticks':bool(np.all(feedback['external_drive_zero'])),
        'active_stabilization_ticks':int(np.sum(active)),
        'active_left_pad_load_minimum_N':table['pad_normal_force_N'][active].min(axis=0).tolist() if active.any() else None,
        'active_left_pad_unloaded_ticks':np.sum(table['pad_normal_force_N'][active] <= .1, axis=0).tolist(),
        'official_strict_left_pad_gap_audit_reused_sha256':digest(official_dir/'independent_left_pad_force_history_audit.json'),
        'official_table_weight_and_strict_gap_audit_reused':official['original_load_history'],
        'nominal_formed_overlap_maximum_m':float(table['formed_flank_overlap_m'].max()),
        'actual_loaded_interior_contact_native_ticks':int(np.sum(feedback['loaded_actual_interior_flank_contact_count'] > 0)),
        'original_right_grasp_acquisitions':acquisition_reports,
        'original_physical_motion_events':original['physical_motion_events'],
        'actual_per_phase_summaries':phase_reports,
        'scope':'Actual closed outcome retained, including failures and unexecuted stages. Nominal formed overlap, actual loaded interior contacts, native quiet/support readiness, full-pitch capture, qualified lead and passive reset remain distinct. Original forces/current derived poses are t−dt; original retained command geometry is t−2dt except initialization; sparse saved qpos and actual jaw aperture are post-state t. Original finite PD/FF cap arithmetic is checked with unchanged b714 helpers, while full source-bound 5D/cache/mapping/observer/contact-frame/strict-gap checks reuse the preceding official reports. No MuJoCo import, native state replay, force reconstruction or integration. Supplemental observations do not replace any original or official acceptance gate.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('official_audit_dir', type=Path)
    parser.add_argument('--arithmetic-source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    require((args.run/'run_publication_identity_after.json').is_file(),
            'Native child has no closed AFTER identity; do not read its arrays')
    require(not args.output.exists(), 'Never overwrite an existing supplemental report')
    require(args.run.resolve() not in args.output.resolve().parents,
            'Supplemental derivative must be outside the immutable original run')
    result = audit(args.run, args.official_audit_dir, args.arithmetic_source)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({'output':str(args.output), 'original_native_ticks':result['original_native_ticks'],
                      'original_passed':result['original_report_passed'],
                      'native_child_exit_code':result['native_child_exit_code']}))


if __name__ == '__main__':
    main()
