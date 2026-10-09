"""Audit original opening forces and quiet-window events without native replay."""
import argparse
import ast
from collections import deque
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def runs(mask):
    edges = np.diff(np.r_[False, np.asarray(mask, bool), False].astype(int))
    return [(int(s), int(e)-1) for s, e in zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))]


def predicates(raw):
    """Exact original fully-open predicate, without an invented hand-speed gate."""
    return {
        'fully_open': raw['fully_open_unassisted'] == 1,
        'zero_whole_right_robot_contacts': raw['right_bolt_contact_count'] == 0,
        'all_original_native_checks': raw['all_checks_held'] == 1,
        'actual_bolt_angular_speed': raw['relative_bolt_angular_speed_rad_per_s'] <= .01,
        'actual_bolt_axial_velocity': abs(raw['relative_bolt_axial_velocity_m_per_s']) <= .0002,
        'external_drive_zero': raw['external_drive_zero'] == 1,
        'no_bolt_world_support': raw['bolt_world_support_contact_count'] == 0,
        'no_nonthread_seating': raw['nonthread_block_bolt_contact_count'] == 0,
        'radial_alignment': raw['radial_offset_m'] <= .000150,
        'tilt_alignment': raw['bolt_tilt_rad'] <= np.deg2rad(2),
    }


def exact_readiness(raw, weight, dt):
    """Independent fixed-native-tick trailing calculation; positive hand is clipped."""
    good = np.logical_and.reduce(list(predicates(raw).values()))
    n = round(.1/dt)
    ready = np.zeros(len(good), bool)
    if len(good) >= n:
        def rolling(x):
            c = np.r_[0., np.cumsum(x, dtype=float)]
            return c[n:] - c[:-n]
        ready[n-1:] = ((rolling(good) == n)
            & (rolling(raw['thread_gravity_opposing_force_N'])/n >= .9*weight)
            & (rolling(np.maximum(raw['hand_gravity_opposing_force_N'], 0.))/n <= .1*weight)
            & (raw['thread_gravity_opposing_force_N'][n-1:] > .1*weight))
    return good, ready


def archived_window_class(path):
    """Execute only the archived pure arithmetic class, never simulation imports."""
    tree = ast.parse(Path(path).read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'BoltWeightTransferWindow')
    namespace = {'np': np, 'deque': deque}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace['BoltWeightTransferWindow']


def source_open_predicate(path):
    tree = ast.parse(Path(path).read_text())
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute) and isinstance(n.func.value, ast.Name)
        and n.func.value.id == 'open_window' and n.func.attr == 'observe']
    assert len(calls) == 1
    actual = next(k.value for k in calls[0].keywords if k.arg == 'valid')
    expected = ast.parse('bool(fully_open and right_bolt_contacts==0 and all(checks.values()) '
        'and sample["relative_bolt_angular_speed_rad_per_s"]<=.01)', mode='eval').body
    assert ast.dump(actual) == ast.dump(expected), 'Source opening predicate differs; do not silently reinterpret.'
    return ast.unparse(actual)


def audit(path):
    path = Path(path)
    declaration = json.loads((path/'declaration.json').read_text())
    report = json.loads((path/'report.json').read_text())
    with np.load(path/'original_native_force_ledger.npz', allow_pickle=False) as archive:
        raw = {k: archive[k].copy() for k in archive.files}
    with np.load(path/'checkpoint_trace.npz', allow_pickle=False) as archive:
        saved = json.loads(str(archive['info_json'].item()))
    dt = float(ET.parse(path/'scene.xml').getroot().find('option').get('timestep'))
    weight = float(declaration['bolt_weight_N'])
    assert len(raw['time_s']) > 0
    assert all(len(v) == len(raw['time_s']) and np.isfinite(v).all() for v in raw.values())
    assert np.allclose(np.diff(raw['time_s']), dt, atol=1e-10, rtol=1e-7)
    assert np.allclose(raw['elapsed_s'], np.arange(1, len(raw['time_s'])+1)*dt, atol=1e-12)
    assert sha(path/'diagnostic_source.py') == declaration['diagnostic_source_sha256']
    assert sha(path/'observer_source.py') == declaration['observer_sha256']
    assert sha(path/'recorded_sources/yam_twin/m8_supported_start.py') == declaration['observer_sha256']
    assert sha(path/'original_native_force_ledger.npz') == report['ledger_sha256']
    assert sha(path/'checkpoint_trace.npz') == report['trace_sha256']
    predicate_text = source_open_predicate(path/'diagnostic_source.py')
    mask = predicates(raw)
    good, exact_ready = exact_readiness(raw, weight, dt)
    original_class = archived_window_class(path/'observer_source.py')
    original_open, original_general = original_class(weight), original_class(weight)
    open_reports, general_reports, source_ready = [], [], []
    sample_keys = ('thread_gravity_opposing_force_N','hand_gravity_opposing_force_N',
        'relative_bolt_axial_velocity_m_per_s','radial_offset_m','bolt_tilt_rad',
        'native_thread_contact_count','loaded_actual_interior_flank_contact_count',
        'bolt_world_support_contact_count','nonthread_block_bolt_contact_count','external_drive_zero')
    for i, time in enumerate(raw['time_s']):
        sample = {k: float(raw[k][i]) for k in sample_keys}
        valid = all(mask[k][i] for k in ('fully_open','zero_whole_right_robot_contacts',
            'all_original_native_checks','actual_bolt_angular_speed'))
        wr = original_open.observe(float(time), dt, sample, valid=valid)
        general = original_general.observe(float(time), dt, sample, valid=bool(mask['all_original_native_checks'][i]))
        source_ready.append(bool(wr['ready_for_diagnostic_release_attempt']))
        open_reports.append(wr)
        general_reports.append(general)
    # Compare every saved ORIGINAL report at its matching native step, including
    # the v1 sparse endpoint gap. The absent final qpos/contact archive is never fabricated.
    max_saved_window_error = 0.
    for sample in saved:
        i = round(float(sample['elapsed_s'])/dt)-1
        assert abs(raw['time_s'][i]-sample['time_s']) < 1e-9
        actual, expected = sample['open_weight_window'], open_reports[i]
        assert actual['ready_for_diagnostic_release_attempt'] == expected['ready_for_diagnostic_release_attempt']
        assert actual['original_native_samples'] == expected['original_native_samples']
        for k in ('observed_window_s','mean_thread_weight_fraction','mean_positive_hand_upward_weight_fraction'):
            if actual[k] is None or expected[k] is None:
                assert actual[k] is expected[k]
            else:
                max_saved_window_error = max(max_saved_window_error, abs(actual[k]-expected[k]))
    assert max_saved_window_error < 1e-12
    for k, v in report['final_weight_window'].items():
        assert v == general_reports[-1][k], (k, v, general_reports[-1][k])
    opened = np.flatnonzero(mask['fully_open'])
    assert len(opened) > 0
    first, last = int(opened[0]), int(opened[-1])
    reference = report['physical_closed_turn']['entry_supported_open_reference']
    assert reference['time_s'] == raw['time_s'][first]
    assert reference['base_z_m'] == raw['base_z_m'][first]
    assert reference['yaw_unwrapped_rad'] == raw['yaw_unwrapped_rad'][first]
    axial = abs(raw['base_z_m'][opened]-reference['base_z_m'])
    yaw = abs(raw['yaw_unwrapped_rad'][opened]-reference['yaw_unwrapped_rad'])
    axial_peak = np.maximum.accumulate(axial)
    yaw_peak = np.maximum.accumulate(yaw)
    assert np.array_equal(axial_peak, raw['open_drift_peak_axial_m'][opened])
    assert np.array_equal(yaw_peak, raw['open_drift_peak_yaw_rad'][opened])
    def endpoints(ready):
        ids = np.flatnonzero(ready)
        return dict(count=len(ids), first_elapsed_s=float(raw['elapsed_s'][ids[0]]) if len(ids) else None,
            last_elapsed_s=float(raw['elapsed_s'][ids[-1]]) if len(ids) else None,
            final_fully_open_tick_ready=bool(ready[last]))
    streaks = runs(good)
    longest = max(streaks, key=lambda p:p[1]-p[0])
    final_streak = (streaks[-1][1]-streaks[-1][0]+1)*dt if streaks[-1][1] == last else 0.
    invalid = {}
    for k, values in mask.items():
        if k == 'fully_open':
            continue
        bad = np.flatnonzero(mask['fully_open'] & ~values)
        invalid[k] = dict(invalid_native_ticks=len(bad),
            first_elapsed_s=float(raw['elapsed_s'][bad[0]]) if len(bad) else None,
            last_elapsed_s=float(raw['elapsed_s'][bad[-1]]) if len(bad) else None,
            native_tick_intervals=[dict(first_elapsed_s=float(raw['elapsed_s'][s]),
                last_elapsed_s=float(raw['elapsed_s'][e]), native_ticks=e-s+1)
                for s,e in runs(mask['fully_open'] & ~values)])
    omega = raw['relative_bolt_angular_speed_rad_per_s'][opened]
    w = slice(max(first,last-round(.1/dt)+1), last+1)
    abort = report['aborted']
    return dict(scope='Independent original all-step opening-window and contact/drive/drift audit. '
            'Pure archived arithmetic; no native integration or replay, no capture, qualified reset or continuous-rollout claim.',
        auditor_source_sha256=sha(__file__), source_sha256=declaration['diagnostic_source_sha256'],
        helper_sha256=declaration['observer_sha256'], original_ledger_sha256=report['ledger_sha256'],
        original_trace_sha256=report['trace_sha256'], source_valid_predicate=predicate_text,
        force_time_scope=declaration['force_timing'], native_steps=len(raw['time_s']), timestep_s=dt,
        all_recorded_native_hard_checks_held=bool(np.all(mask['all_original_native_checks'])),
        original_producer_overall_guards_held=report['all_original_guards_held'],
        original_producer_failure=abort,
        timing=dict(last_original_solved_data_time_s=float(raw['time_s'][-1]-dt),
            last_integrated_elapsed_s=float(raw['elapsed_s'][-1]),
            pre_step_abort_elapsed_s=abort['elapsed_s'] if abort else None,
            last_saved_post_state_elapsed_s=float(saved[-1]['elapsed_s']),
            saved_state_tail_gap_s=float(raw['elapsed_s'][-1]-saved[-1]['elapsed_s']),
            scope='Retained solved forces/geometry describe time-dt; saved qpos/qvel are post-integration. '
                'A pre-command timeout step was not integrated; any sparse trace tail gap remains absent.'),
        fully_open=dict(native_ticks=len(opened), first_elapsed_s=float(raw['elapsed_s'][first]),
            last_elapsed_s=float(raw['elapsed_s'][last]), duration_s=len(opened)*dt,
            maximum_whole_right_robot_contact_count=int(np.max(raw['right_bolt_contact_count'][opened])),
            maximum_bolt_world_contact_count=int(np.max(raw['bolt_world_support_contact_count'][opened])),
            maximum_nonthread_seating_count=int(np.max(raw['nonthread_block_bolt_contact_count'][opened])),
            original_applied_external_drive_zero=bool(np.all(raw['external_drive_zero'][opened]==1)),
            maximum_absolute_hand_upward_load_N=float(np.max(abs(raw['hand_gravity_opposing_force_N'][opened]))),
            maximum_applied_axial_feed_N=float(np.max(abs(raw['right_applied_axial_feed_N'][opened]))),
            maximum_robot_site_axial_wrench_command_N=float(np.max(abs(raw['right_site_axial_wrench_command_N'][opened]))),
            robot_wrench_scope='Free hand can receive finite XYZ position-control motor forces; '
                'zero whole-right-robot contacts prevents those forces from supporting or driving the bolt.'),
        original_saved_window_reconstruction=dict(sample_rows=len(saved), maximum_report_error=max_saved_window_error,
            final_original_open_window=open_reports[-1], final_original_general_window=general_reports[-1]),
        strict_open_ready_original_arithmetic=endpoints(np.asarray(source_ready)),
        strict_open_ready_independent_exact100ms=endpoints(exact_ready),
        strict_valid_streaks=dict(longest_duration_s=(longest[1]-longest[0]+1)*dt,
            longest_first_elapsed_s=float(raw['elapsed_s'][longest[0]]),
            longest_last_elapsed_s=float(raw['elapsed_s'][longest[1]]), final_duration_s=final_streak),
        invalid_original_predicates=invalid,
        fully_open_bolt_motion=dict(maximum_angular_speed_rad_per_s=float(np.max(omega)),
            rms_angular_speed_rad_per_s=float(np.sqrt(np.mean(omega**2))),
            median_p95_p99_angular_speed_rad_per_s=np.quantile(omega,[.5,.95,.99]).tolist(),
            maximum_abs_axial_velocity_m_per_s=float(np.max(abs(raw['relative_bolt_axial_velocity_m_per_s'][opened]))),
            cumulative_peak_axial_drift_m=float(axial_peak[-1]), cumulative_peak_yaw_drift_rad=float(yaw_peak[-1]),
            final_axial_drift_m=float(axial[-1]), final_yaw_drift_rad=float(yaw[-1]),
            mean_cumulative_axial_peak_last100ms_m=float(np.mean(raw['open_drift_peak_axial_m'][w])),
            scope='Cumulative peak, final displacement and mean of a cumulative-peak field are distinct quantities.'),
        final_fully_open_exact100ms_load=dict(duration_s=(w.stop-w.start)*dt,
            mean_signed_thread_weight_fraction=float(np.mean(raw['thread_gravity_opposing_force_N'][w])/weight),
            mean_positive_hand_weight_fraction=float(np.mean(np.maximum(raw['hand_gravity_opposing_force_N'][w],0))/weight),
            loaded_thread_duty=float(np.mean(raw['thread_gravity_opposing_force_N'][w]>.1*weight)),
            all_native_hard_checks_held=bool(np.all(raw['all_checks_held'][w]==1)),
            strict_instantaneous_bolt_angular_gate_all_ticks=bool(np.all(raw['relative_bolt_angular_speed_rad_per_s'][w]<=.01))),
        observed_phase_names=[name for i,name in enumerate(declaration['phase_names']) if np.any(raw['phase_index']==i)],
        actual_regrasp_acquisition=report['physical_closed_turn']['actual_regrasp_acquisition'],
        loaded_actual_conservative_interior_ticks=int(np.sum(raw['loaded_actual_interior_flank_contact_count']>0)),
        full_capture=False, qualified_reset=False, full_trajectory_qualified=False)


if __name__ == '__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('directory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args=parser.parse_args()
    result=audit(args.directory)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ('timing','strict_open_ready_original_arithmetic',
        'strict_valid_streaks','fully_open_bolt_motion','original_producer_failure')},indent=2))
