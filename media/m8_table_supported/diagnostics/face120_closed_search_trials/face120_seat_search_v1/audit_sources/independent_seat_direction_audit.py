"""Original all-step measured stop/drop intervals; diagnostic direction only."""
from pathlib import Path
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def runs(mask):
    changes = np.diff(np.r_[False, mask, False].astype(int))
    return list(zip(np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)))


def audit(path):
    path = Path(path)
    declaration = json.loads((path/'declaration.json').read_text())
    report = json.loads((path/'report.json').read_text())
    xml = ET.fromstring((path/'scene.xml').read_text())
    plugins = {e.get('name'): {c.get('key'): float(c.get('value')) for c in e}
        for e in xml.findall('.//extension/plugin/instance')}
    nominal_offset = plugins['bolt_shape']['length']+plugins['nut_shape']['length']/2
    raw = np.load(path/'original_native_force_ledger.npz', allow_pickle=False)
    events = report['physical_closed_turn']['events']
    references = [row for row in events if 'reference_base_z_m' in row]
    if not references:
        return dict(scope='Direction event only; no qualification.', reference_observed=False)
    reference = references[0]['reference_base_z_m']
    dt = float(np.median(np.diff(raw['elapsed_s'])))
    names = declaration['phase_names']
    stop = raw['phase_index'] == names.index('stop_reverse_seat_1')
    masks = dict(persistent_measured_drop=raw['base_z_m']-reference >= 50e-6,
        actual_linear_speed=abs(raw['relative_bolt_axial_velocity_m_per_s']) <= .0002,
        actual_bolt_angular_speed=raw['relative_bolt_angular_speed_rad_per_s'] <= .01,
        actual_hand_angular_speed=raw['relative_hand_angular_speed_rad_per_s'] <= .01,
        all_original_guards=raw['all_checks_held'] == 1)
    aligned = stop.copy()
    for mask in masks.values():
        aligned &= mask
    intervals = runs(aligned)
    accepted = []
    count = round(.05/dt)
    weight = declaration['bolt_weight_N']
    for start, end in intervals:
        if end-start < count:
            continue
        endpoint = start+count
        window = slice(start, endpoint)
        candidate = dict(start_index=int(start), endpoint_index=int(endpoint-1),
            start_elapsed_s=float(raw['elapsed_s'][start]), endpoint_elapsed_s=float(raw['elapsed_s'][endpoint-1]),
            continuous_interval_duration_s=(end-start)*dt,
            initial_axial_drop_m=float(raw['base_z_m'][start]-reference),
            endpoint_axial_drop_m=float(raw['base_z_m'][endpoint-1]-reference),
            endpoint_nominal_overlap_m=float(raw['base_z_m'][endpoint-1]+nominal_offset),
            original_signed_thread_upward_impulse_Ns=float(raw['thread_gravity_opposing_force_N'][window].sum()*dt),
            mean_signed_thread_weight_fraction=float(raw['thread_gravity_opposing_force_N'][window].mean()/weight),
            mean_positive_hand_upward_weight_fraction=float(np.maximum(raw['hand_gravity_opposing_force_N'][window], 0.).mean()/weight),
            original_upward_thread_loaded_duty=float(np.mean(raw['thread_gravity_opposing_force_N'][window] > .1*weight)),
            maximum_actual_formed_overlap_m=float(raw['formed_flank_overlap_m'][window].max()),
            actual_loaded_interior_substeps=int(np.sum(raw['loaded_actual_interior_flank_contact_count'][window] > 0)))
        if 'thread_summed_normal_force_N' in raw:
            candidate['original_normal_impulse_Ns'] = float(raw['thread_summed_normal_force_N'][window].sum()*dt)
            candidate['original_normal_loaded_duty'] = float(np.mean(raw['thread_summed_normal_force_N'][window] > 1e-5))
        accepted.append(candidate)
    return dict(scope='Original all-step persistent measured drop and actual aligned/slow physical stop. Candidate intervals only, not retroactive passes for the producer criterion or engagement/opening qualification.',
        source_sha256=sha(__file__), ledger_sha256=sha(path/'original_native_force_ledger.npz'),
        report_sha256=sha(path/'report.json'), reference_observed=True, reference_base_z_m=reference,
        stop_original_native_steps=int(stop.sum()),
        failed_stop_ticks_by_independent_condition={name: int(np.sum(stop & ~value)) for name, value in masks.items()},
        stop_condition_failure_counts_overlap=True,
        maximum_continuously_aligned_slow_drop_stop_s=max(((end-start)*dt for start, end in intervals), default=0.),
        candidate_continuous50ms_stops=accepted,
        per_step_original_thread_normal_available='thread_summed_normal_force_N' in raw,
        normal_impulse_scope='Exact per-step normal impulse cannot be recreated when the scalar was omitted. Sparse original samples and world vertical force are not substitutes for original every-step normal forces.',
        producer_original_final_event=report['physical_closed_turn']['final_direction_event'],
        producer_original_termination=report['physical_closed_turn']['termination'],
        capture_qualified=False, full_trajectory_qualified=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = audit(args.directory)
    Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2))
