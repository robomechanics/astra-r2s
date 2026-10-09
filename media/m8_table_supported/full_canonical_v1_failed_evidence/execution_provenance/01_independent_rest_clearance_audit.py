"""Supplemental read-only native-rest clearance audit; no integration/force solve.

The historical producer proof and its frozen supported auditor are unchanged.
This separate source binds its own SHA and regression evidence. Original
pre-integration tip clearances and saved post-integration poses stay separate.
"""
from pathlib import Path
import argparse
import hashlib
import io
import itertools
import json
import sys

import mujoco
import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'thread_lab/runtime.py').is_file())
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'scripts'))
from audit_m8_insertion_trace import recorded_model
from thread_lab.runtime import require_micron_engine


def digest(value):
    return hashlib.sha256(value).hexdigest()


def fixed_geom_top(model, data, geom):
    """Independent actual world bounds, including tilted cylinder end faces."""
    if int(model.body_weldid[int(model.geom_bodyid[geom])]) != 0:
        raise ValueError('Declared rest/table geometry is not fixed')
    center, rotation = data.geom_xpos[geom], data.geom_xmat[geom].reshape(3, 3)
    size, kind = model.geom_size[geom], int(model.geom_type[geom])
    if kind == int(mujoco.mjtGeom.mjGEOM_BOX):
        corners = np.asarray(list(itertools.product((-1., 1.), repeat=3)))*size
        return float(np.max((corners@rotation.T)[:, 2]+center[2]))
    if kind == int(mujoco.mjtGeom.mjGEOM_CYLINDER):
        radial_projection = np.linalg.norm(rotation[2, :2])*size[0]
        end_projection = abs(rotation[2, 2])*size[1]
        return float(center[2]+radial_projection+end_projection)
    raise ValueError('Supplemental audit supports only actual native box/cylinder rests')


def native_rest_bounds(model, data):
    tops = {model.geom(g).name: fixed_geom_top(model, data, g) for g in range(model.ngeom)
            if model.geom(g).name.startswith('bolt_rest_')}
    if not tops:
        raise ValueError('Exact archived model has no native declared bolt rest')
    return {'maximum_world_z_m': max(tops.values()), 'geom_world_top_z_m': tops}


def transfer_samples(indices, labels, phases):
    """Observe lift completion and every transport tick; an early abort is absent."""
    indices = np.asarray(indices, dtype=int)
    selected = np.array([labels[i] == 'transport_bolt' for i in indices])
    lift = next((p for p in phases if p['phase'] == 'lift_bolt'), None)
    lift_complete = bool(lift and np.isclose(lift['actual_duration_s'],
        lift['maximum_scheduled_duration_s'], rtol=0., atol=1e-10))
    if lift_complete:
        rows = np.flatnonzero([labels[i] == 'lift_bolt' for i in indices])
        if not len(rows):
            raise ValueError('Declared lift completion is absent from original ledger')
        selected[rows[-1]] = True
    return selected


def transfer_stats(clearances, selected, limit):
    values, selected = np.asarray(clearances, dtype=float), np.asarray(selected, dtype=bool)
    if values.shape != selected.shape or not np.isfinite(values).all() or not np.isfinite(limit) or limit <= 0:
        raise ValueError('Invalid original measured clearance observations')
    minimum = float(values[selected].min()) if selected.any() else None
    return {'observed_native_transfer_substeps': int(selected.sum()),
            'minimum_observed_tip_above_rest_m': minimum, 'minimum_allowed_clearance_m': float(limit),
            'passed': bool(minimum is not None and minimum >= limit)}


def audit(path):
    path = Path(path).resolve()
    runtime = require_micron_engine()
    with np.load(path, allow_pickle=False) as saved:
        metadata = json.loads(str(saved['metadata_json']))
        records = json.loads(str(saved['info_json']))
        times, poses = saved['time'].copy(), saved['qpos'].copy()
    model, identity = recorded_model(path, metadata)
    data = mujoco.MjData(model)
    data.qpos[:] = model.qpos0
    mujoco.mj_kinematics(model, data)
    rest = native_rest_bounds(model, data)
    table_top = max(fixed_geom_top(model, data, model.geom(name).id)
                    for name in metadata['table_support_geom_names'])
    source_guard = metadata.get('acceptance_checks', {}).get('whole_shaft_clears_rest_before_lateral_transfer')
    enabled = source_guard is not None or 'pickup_transport_clearance' in metadata
    result = {'kind': 'Supplemental native whole-shaft/rest clearance audit',
        'auditor_source_sha256': digest(Path(__file__).read_bytes()),
        'trajectory_sha256': digest(path.read_bytes()), 'archived_model_identity': identity,
        'audit_runtime': runtime, 'compiled_actual_fixed_rest': rest,
        'compiled_actual_table_top_world_z_m': table_top, 'source_rest_guard_instrumented': enabled,
        'method': 'Exact archived native primitive bounds and original scalar ledger; saved qpos kinematics only; no integration, collision-force solve or force reconstruction',
        'native_producer_source_and_original_report_modified': False}
    if not enabled:
        return {**result, 'passed': None,
                'scope': 'Historical recording predates this rest-transfer guard; absent guard is not retroactively qualified'}
    if source_guard is None or 'pickup_transport_clearance' not in metadata:
        raise ValueError('Partial declaration of the new source rest-clearance guard')
    expected = metadata['pickup_transport_clearance']['native_rest']
    if set(expected['geom_world_top_z_m']) != set(rest['geom_world_top_z_m']):
        raise ValueError('Declared and actual native rest geometry differ')
    for name, top in rest['geom_world_top_z_m'].items():
        if not np.isclose(top, expected['geom_world_top_z_m'][name], rtol=0., atol=1e-12):
            raise ValueError('Declared source rest top differs from compiled native primitive')
    limit = float(metadata['control_config']['minimum_tip_rest_clearance_m'])
    if not np.isclose(source_guard['native_rest_top_world_z_m'], rest['maximum_world_z_m'], rtol=0., atol=1e-12):
        raise ValueError('Original source acceptance uses an incorrect rest top')
    declaration = metadata['table_support_force_history']
    if Path(declaration['filename']).name != declaration['filename']:
        raise ValueError('Native support history must be a sibling filename')
    raw = (path.parent/declaration['filename']).read_bytes()
    if digest(raw) != declaration['sha256']:
        raise ValueError('Original rest-clearance ledger SHA changed')
    with np.load(io.BytesIO(raw), allow_pickle=False) as saved:
        original_times, indices = saved['time'].copy(), saved['phase_index'].copy()
        native_clearance = saved['bolt_tip_rest_top_clearance_m'].copy()
        native_table_clearance = saved['bolt_tip_table_clearance_m'].copy()
        labels = json.loads(str(saved['phase_labels_json']))
        ledger_identity = json.loads(str(saved['metadata_json']))
    for key in ('model_fingerprint', 'controller_sha256', 'runtime'):
        if ledger_identity[key] != metadata[key]:
            raise ValueError('Original rest ledger identity differs from source recording')
    expected_clearance = native_table_clearance+table_top-rest['maximum_world_z_m']
    if not np.allclose(native_clearance, expected_clearance, rtol=0., atol=1e-12):
        raise ValueError('Original measured rest clearances disagree with actual compiled rest/table bounds')
    selected = transfer_samples(indices, labels, metadata['phases'])
    native = transfer_stats(native_clearance, selected, limit)
    if (native['passed'] != bool(source_guard['passed'])
            or (native['minimum_observed_tip_above_rest_m'] is None) != (source_guard['minimum_measured_tip_above_highest_rest_m'] is None)):
        raise ValueError('Original source rest-transfer gate differs from independently selected native observations')
    if native['minimum_observed_tip_above_rest_m'] is not None and not np.isclose(
            native['minimum_observed_tip_above_rest_m'], source_guard['minimum_measured_tip_above_highest_rest_m'], rtol=0., atol=1e-12):
        raise ValueError('Original source minimum transfer clearance disagrees with original raw ledger')
    post_clearance, offsets, post_labels = [], [], []
    bolt = model.body('male_bolt').id
    tip_local = np.asarray(model.site('bolt_tip').pos)
    for time, pose, record in zip(times, poses, records):
        data.qpos[:] = pose
        mujoco.mj_kinematics(model, data)
        tip = data.xpos[bolt]+data.xmat[bolt].reshape(3, 3)@tip_local
        measured = float(tip[2]-rest['maximum_world_z_m'])
        index = int(np.searchsorted(original_times, time))
        if index == len(original_times) or abs(original_times[index]-time) > 1e-10:
            raise ValueError('Saved post-state sample lacks its original native rest observation')
        if not np.isclose(record['bolt_tip_rest_top_clearance_m'], native_clearance[index], rtol=0., atol=1e-12):
            raise ValueError('Saved original rest diagnostic disagrees with raw native ledger')
        post_clearance.append(measured)
        offsets.append(measured-native_clearance[index])
        post_labels.append(record['phase'])
    post_indices = np.arange(len(times))
    post_selected = transfer_samples(post_indices, post_labels, metadata['phases'])
    post = transfer_stats(post_clearance, post_selected, limit)
    return {**result, 'original_native_rest_guard': native,
        'independent_saved_post_state_rest_guard': post,
        'post_minus_original_pre_clearance_range_m': [min(offsets), max(offsets)],
        'timing_scope': 'Original native tip observations are pre-integration at t-dt; replayed saved qpos are post-integration at t. Offsets are explicitly reported, never silently equated.',
        'original_report_passed': metadata['passed'], 'source_gate_passed': source_guard['passed'],
        'passed': bool(native['passed'] and post['passed'])}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.trajectory)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'output': str(args.output), 'passed': result['passed'],
        'source_rest_guard_instrumented': result['source_rest_guard_instrumented']}, allow_nan=False))


if __name__ == '__main__':
    main()
