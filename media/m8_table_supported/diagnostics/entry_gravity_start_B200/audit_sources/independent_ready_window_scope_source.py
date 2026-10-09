"""Supplemental scope of qualifying100ms windows; no release qualification."""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('directory')
args = parser.parse_args()
path = Path(args.directory)
raw = np.load(path/'original_native_force_ledger.npz', allow_pickle=False)
declaration = json.loads((path/'declaration.json').read_text())
dt = float(np.median(np.diff(raw['time_s'])))
assert np.allclose(np.diff(raw['time_s']), dt, rtol=1e-7, atol=1e-10)
count = round(.100/dt)
weight = declaration['bolt_weight_N']


def rolling(values):
    prefix = np.r_[0., np.cumsum(values)]
    return (prefix[count:]-prefix[:-count])/count


bad = ((abs(raw['relative_bolt_axial_velocity_m_per_s']) > .0002)
    | (raw['radial_offset_m'] > 150e-6) | (raw['bolt_tilt_rad'] > np.deg2rad(2))
    | (raw['external_drive_zero'] != 1) | (raw['all_checks_held'] != 1)
    | (raw['bolt_world_support_contact_count'] != 0)
    | (raw['nonthread_block_bolt_contact_count'] != 0))
ready = ((rolling(bad.astype(float)) == 0)
    & (rolling(raw['thread_gravity_opposing_force_N']) >= .90*weight)
    & (rolling(np.maximum(raw['hand_gravity_opposing_force_N'], 0.)) <= .10*weight)
    & (raw['thread_gravity_opposing_force_N'][count-1:] > .1*weight))
indices = np.flatnonzero(ready)+count-1
result = dict(scope='Supplemental new bolt90/10 criterion. Pass timestamps of exact100ms rolling original-force windows; no actual release, interior geometry, lead or capture is implied. Original solved geometry is at logged time minus native timestep.',
    source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    ledger_sha256=hashlib.sha256((path/'original_native_force_ledger.npz').read_bytes()).hexdigest(),
    qualifying_original_steps=len(indices),
    first_pass_elapsed_s=float(raw['elapsed_s'][indices[0]]) if len(indices) else None,
    last_pass_elapsed_s=float(raw['elapsed_s'][indices[-1]]) if len(indices) else None,
    maximum_actual_formed_overlap_at_pass_m=float(raw['formed_flank_overlap_m'][indices].max()) if len(indices) else None,
    maximum_actual_loaded_interior_count_at_pass=int(raw['loaded_actual_interior_flank_contact_count'][indices].max()) if len(indices) else None,
    capture_qualified=False)
(path/'independent_ready_window_scope.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
