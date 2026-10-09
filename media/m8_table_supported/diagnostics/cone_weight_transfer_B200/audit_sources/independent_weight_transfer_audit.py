"""Read-only independent load/frame audit of a cold checkpoint diagnostic."""
from pathlib import Path
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[3]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def audit(path):
    path = Path(path)
    declaration = json.loads((path/'declaration.json').read_text())
    producer = json.loads((path/'report.json').read_text())
    raw = np.load(path/'original_native_force_ledger.npz', allow_pickle=False)
    saved = np.load(path/'checkpoint_trace.npz', allow_pickle=False)
    samples = json.loads(str(saved['info_json'].item()))
    parent = ROOT/'outputs/m8_table_supported/full_v2'
    option = ET.fromstring((parent/'scene.xml').read_text()).find('option')
    gravity = np.fromstring(option.attrib['gravity'], sep=' ')
    up = -gravity/np.linalg.norm(gravity)
    dt = float(option.attrib['timestep'])
    time = raw['time_s']
    count = round(.1/dt)
    weight = declaration['bolt_weight_N']
    assert all(np.isfinite(raw[key]).all() and len(raw[key]) == len(time) for key in raw.files)
    assert np.allclose(np.diff(time), dt, rtol=1e-7, atol=1e-10)
    assert abs(time[0]-declaration['initial_saved_state_time_s']-dt) < 1e-10
    wrench_errors, scalar_errors, identities, observed = [], [], set(), 0
    unrecorded_final_force_rows = []
    for row in samples:
        index = round((row['time_s']-time[0])/dt)
        assert abs(time[index]-row['time_s']) < 1e-9
        total = np.zeros(6)
        for contact in row['native_contact_records']:
            frame = np.array(contact['frame'])
            local = np.array(contact['local_force_N_Nm'])
            sign = -1 if contact['geom1'] == 'bolt_thread' else 1
            force = sign*frame.T@local[:3]
            torque = sign*frame.T@local[3:]+np.cross(
                np.array(contact['contact_position_world_m'])-
                np.array(contact['bolt_origin_world_m']), force)
            wrench = np.r_[force, torque]
            wrench_errors.append(float(np.max(abs(wrench-
                np.array(contact['wrench_on_bolt_world_N_Nm'])))))
            total += wrench
        # The producer also saves a final post-state if not a 100-step sample,
        # but does not request local contact records on that extra sample.
        if index % 100 == 0:
            observed += 1
            wrench_errors.append(float(np.max(abs(total-
                np.array(row['thread_wrench_on_bolt_world_N_Nm'])))))
            scalar_errors.append(abs(float(total[:3]@up)-
                float(raw['thread_gravity_opposing_force_N'][index])))
        else:
            unrecorded_final_force_rows.append(index)
        scalar_errors.append(abs(float(np.array(
            row['right_pad_wrench_on_bolt_world_N_Nm'])[:3]@up)-
            float(raw['right_pad_gravity_opposing_force_N'][index])))
        identities.add(row['source_sha256'])

    def rolling(values):
        prefix = np.r_[0., np.cumsum(values)]
        return (prefix[count:]-prefix[:-count])/count

    thread_fraction = rolling(raw['thread_gravity_opposing_force_N'])/weight
    hand_fraction = rolling(np.maximum(raw['hand_gravity_opposing_force_N'], 0.))/weight
    bad = ((abs(raw['relative_bolt_axial_velocity_m_per_s']) > .0002)
        | (raw['radial_offset_m'] > 150e-6) | (raw['bolt_tilt_rad'] > np.deg2rad(2))
        | (raw['external_drive_zero'] != 1) | (raw['bolt_world_support_contact_count'] != 0)
        | (raw['nonthread_block_bolt_contact_count'] != 0) | (raw['all_checks_held'] != 1))
    good = rolling(bad.astype(float)) == 0
    ready = (good & (thread_fraction >= .9) & (hand_fraction <= .1)
        & (raw['thread_gravity_opposing_force_N'][count-1:] > .1*weight))
    source_checks = {key: sha(ROOT/key) == digest
        for key, digest in declaration['parent_source'].items()}
    identity = dict(parent_trace_matches=sha(parent/'insertion_trace.npz') == declaration['parent_trace_sha256'],
        archived_xml_matches=sha(parent/'scene.xml') == declaration['parent_model']['model_xml_sha256'],
        loaded_copied_observer_matches=sha(path/'observer_source.py') == declaration['observer_sha256'],
        all_sample_observer_identities_match=identities == {declaration['observer_sha256']})
    assert all(identity.values()) and all(source_checks.values())
    assert max(wrench_errors, default=0.) < 1e-10 and max(scalar_errors, default=0.) < 1e-10
    ranges = {key: [float(raw[key][-count:].min()), float(raw[key][-count:].max())]
        for key in ['relative_bolt_axial_velocity_m_per_s', 'radial_offset_m', 'bolt_tilt_rad',
            'thread_gravity_opposing_force_N', 'hand_gravity_opposing_force_N', 'native_thread_depth_m']}
    return dict(scope='Original native contact-local forces and frames re-transformed without integration, force solving or post-qpos force reconstruction. Cold-checkpoint diagnostic only; no capture, lead or full-trajectory qualification.',
        auditor_source_sha256=sha(__file__), input_sha256={name: sha(path/name)
            for name in ['declaration.json', 'diagnostic_source.py', 'observer_source.py',
                'report.json', 'original_native_force_ledger.npz', 'checkpoint_trace.npz']},
        identity={**identity, 'source_checks': source_checks}, actual_duration_s=len(time)*dt,
        original_native_steps=len(time), saved_original_sample_rows=len(samples),
        force_frame=dict(original_contact_local_solves_retransformed=observed,
            extra_final_rows_without_local_force_archive=unrecorded_final_force_rows,
            maximum_wrench_retransform_error_N_Nm=max(wrench_errors, default=0.),
            maximum_sampled_force_vs_all_step_ledger_error_N=max(scalar_errors, default=0.),
            scope='Right-pad load recomputed from its original native world wrench; individual pad-local contact forces are not archived in this diagnostic.'),
        independent_exact100ms=dict(duration_s=count*dt, native_steps=count,
            mean_signed_thread_weight_fraction=float(thread_fraction[-1]),
            mean_positive_hand_upward_weight_fraction=float(hand_fraction[-1]),
            mean_signed_hand_weight_fraction=float(raw['hand_gravity_opposing_force_N'][-count:].mean()/weight),
            loaded_thread_gravity_support_duty=float(np.mean(
                raw['thread_gravity_opposing_force_N'][-count:] > .1*weight)),
            geometry_motion_drive_support_checks_contiguous=bool(good[-1]),
            ready=bool(ready[-1]), ever_ready=bool(np.any(ready)), last_window_ranges=ranges),
        producer_original100_05ms_observer=producer['final_weight_window'],
        actual_interior_contact=dict(maximum_formed_full_ring_overlap_m=float(raw['formed_flank_overlap_m'].max()),
            maximum_loaded_interior_flank_count=int(raw['loaded_actual_interior_flank_contact_count'].max()),
            maximum_interior_gravity_opposing_force_N=float(raw['interior_flank_gravity_opposing_force_N'].max()),
            entry_only=bool(np.all(raw['loaded_actual_interior_flank_contact_count'] == 0))),
        all_native_diagnostic_checks_held=bool(np.all(raw['all_checks_held'] == 1)),
        capture_qualified=False)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('directory')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    result = audit(args.directory)
    Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key: result[key] for key in ['force_frame', 'independent_exact100ms',
        'actual_interior_contact', 'all_native_diagnostic_checks_held']}, indent=2))
