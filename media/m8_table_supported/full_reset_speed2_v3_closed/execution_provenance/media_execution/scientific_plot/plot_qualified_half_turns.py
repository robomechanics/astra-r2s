"""Plot original native half-turn geometry; no fit, model, force replay or audit."""
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import time

os.environ.setdefault('MPLCONFIGDIR', '/tmp/astra-m8-qualified-plot-cache')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path('/workspace/astra-r2s')
RUN = ROOT/'outputs/m8_table_supported/full_reset_speed2_v3'
AUDIT = ROOT/'outputs/m8_table_supported/full_reset_speed2_v3_audits'
OUT = Path(__file__).resolve().parent
STRIDE = 25


def sha(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024), b''):
            result.update(block)
    return result.hexdigest()


def stamp():
    return datetime.now(timezone.utc).isoformat()


started, before = stamp(), time.perf_counter()
source_sha = sha(Path(__file__))
after = RUN/'run_publication_identity_after.json'
assert after.is_file() and sha(after) == 'f91f91beaaf7a8ee493fc124b068d469c72fee06be38f9920dce955e53aa2792'
binder_path = AUDIT/'independent_audit_binding.json'
assert sha(binder_path) == '6358e32d170bb9ed7710f76f1b40f51cfdb2143bdc8d5bc47f9f4b60291a51bf'
binder = json.loads(binder_path.read_text())
original_path = RUN/'insertion_validation.json'
official_path = AUDIT/'independent_supported_audit.json'
original = json.loads(original_path.read_text())
official = json.loads(official_path.read_text())
raw = RUN/'table_support_force_history.npz'
bindings = {str(after):sha(after), str(binder_path):sha(binder_path),
            str(original_path):sha(original_path), str(official_path):sha(official_path), str(raw):sha(raw)}
assert bindings[str(raw)] == original['table_support_force_history']['sha256']
assert bindings[str(raw)] == binder['original_run_files_sha256'][raw.name]
assert bindings[str(original_path)] == binder['original_run_files_sha256'][original_path.name]
assert bindings[str(official_path)] == binder['audit_files_sha256'][official_path.name]
assert original['passed'] is True and official['passed'] is True
dt = float(original['scene_config']['base']['thread']['timestep'])
pitch = float(original['scene_config']['base']['thread']['pitch'])
with np.load(raw, allow_pickle=False) as saved:
    times = saved['time']
    phase = saved['phase_index']
    angle = saved['bolt_yaw_unwrapped_rad']
    depth = saved['bolt_base_insertion_m']
    labels = json.loads(str(saved['phase_labels_json']))
official_turns = {p['phase']:p for p in official['original_all_step_bolt_phases']
                  if p['phase'] in ('turn_1', 'turn_2')}
fig, axes = plt.subplots(1, 2, figsize=(12.8, 5.7), constrained_layout=False)
fig.subplots_adjust(left=.077, right=.981, bottom=.26, top=.80, wspace=.27)
colors = ('#2067aa', '#d56a20')
selected, arrays = [], {}
for label, color in zip(('turn_1', 'turn_2'), colors):
    ids = np.flatnonzero(phase == labels.index(label))
    assert len(ids) and ids[0] > 0
    baseline = int(ids[0])-1
    full_ids = np.r_[baseline, ids]
    plotted_ids = np.unique(np.r_[baseline, ids[::STRIDE], ids[-1]])
    relative_angle = angle[full_ids]-angle[baseline]
    advance = depth[full_ids]-depth[baseline]
    residual = advance-pitch*relative_angle/(2*np.pi)
    actual_force_time = times[full_ids]-dt
    baseline_force_time = float(times[baseline]-dt)
    local_time = actual_force_time-baseline_force_time
    plotted = np.searchsorted(full_ids, plotted_ids)
    lead_error = official_turns[label]['independent_lead_fit']['lead_error_percent']
    display = label.replace('_', ' ').capitalize()
    axes[0].plot(np.rad2deg(relative_angle[plotted]), advance[plotted]*1e6,
                 color=color, lw=1.2, label=f'{display}: original measured geometry')
    axes[1].plot(local_time[plotted], residual[plotted]*1e6,
                 color=color, lw=1.15, label=f'{display} (accepted lead-fit error {lead_error:.3f}%)')
    prefix = label+'__'
    arrays.update({prefix+'source_native_indices':full_ids,
                   prefix+'post_label_time_s':times[full_ids],
                   prefix+'original_native_force_geometry_time_s':actual_force_time,
                   prefix+'phase_relative_original_time_s':local_time,
                   prefix+'actual_relative_rotation_rad':relative_angle,
                   prefix+'actual_axial_advance_m':advance,
                   prefix+'nominal_pitch_residual_m':residual,
                   prefix+'plotted_source_native_indices':plotted_ids})
    selected.append({'phase':label, 'phase_id':labels.index(label),
        'actual_phase_native_steps':int(len(ids)), 'baseline_native_index':baseline,
        'baseline_post_label_time_s':float(times[baseline]),
        'baseline_original_force_geometry_time_s':baseline_force_time,
        'baseline_actual_bolt_yaw_rad':float(angle[baseline]),
        'baseline_actual_bolt_base_insertion_m':float(depth[baseline]),
        'full_original_phase_index_range_including_baseline':[int(full_ids[0]),int(full_ids[-1])],
        'full_data_points_retained':len(full_ids), 'display_stride':STRIDE,
        'exact_plotted_source_native_indices':plotted_ids.tolist(),
        'accepted_official_half_turn_result_unchanged':official_turns[label]})
reference_degrees = np.array([0., 180.])
axes[0].plot(reference_degrees, pitch*reference_degrees/360*1e6, ls='--',
             color='#374151', lw=1.3, label='Nominal M8 × 1.25 lead reference')
axes[1].axhline(0, ls='--', color='#374151', lw=1.05)
axes[0].set_xlabel('Actual bolt rotation from original phase baseline (degrees)')
axes[0].set_ylabel('Actual axial advance (µm)')
axes[1].set_xlabel('Actual native geometry time from original baseline (s)')
axes[1].set_ylabel('Measured advance − nominal pitch reference (µm)')
for ax in axes:
    ax.grid(True, alpha=.24)
    ax.spines[['top', 'right']].set_visible(False)
    ax.legend(loc='upper left', fontsize=8.1, framealpha=.95)
fig.suptitle('Two qualified native half-turns', x=.077, ha='left', y=.965,
             fontsize=18, fontweight='bold')
fig.text(.077, .887, 'Original measured motion versus a geometric lead reference; the reference is not a commanded helix.',
         fontsize=10.1, color='#374151')
fig.text(.077, .111, 'Lines connect every 25th original native point; preceding baseline and final point included. Complete phase points retained in NPZ.',
         fontsize=8.9, color='#374151')
fig.text(.077, .071, 'Geometry/force-state time = original post label − 50 µs. Lead-fit annotations reuse the accepted official report; no new fit or audit.',
         fontsize=8.9, color='#374151')
fig.text(.077, .030, 'Within-stroke residuals are visible above; nanometre endpoint residuals do not bound the full stroke or certify material/contact accuracy.',
         fontsize=8.9, color='#374151')
image_path = OUT/'qualified_half_turns.png'
data_path = OUT/'original_qualified_half_turn_plot_data.npz'
manifest_path = OUT/'plot_manifest.json'
assert not any(p.exists() for p in (image_path, data_path, manifest_path))
fig.savefig(image_path, dpi=170, facecolor='white')
plt.close(fig)
np.savez_compressed(data_path, **arrays)
assert sha(Path(__file__)) == source_sha and all(sha(Path(p)) == digest for p, digest in bindings.items())
manifest = {'kind':'Standalone original qualified-half-turn scientific visualization',
    'producer_commit':binder['producer_commit'], 'original_native_child_exit_code':binder['native_child_exit_code'],
    'started_utc':started, 'finished_utc':stamp(), 'wall_seconds':time.perf_counter()-before,
    'plot_source_sha256':source_sha, 'source_and_all_input_bytes_unchanged':True,
    'original_bound_inputs_sha256':bindings,
    'raw_columns':['time', 'phase_index', 'bolt_yaw_unwrapped_rad', 'bolt_base_insertion_m'],
    'original_raw_filename':raw.name, 'original_raw_sha256':bindings[str(raw)],
    'original_timestep_s':dt, 'nominal_pitch_m':pitch,
    'exact_phase_baselines_and_selected_indices':selected,
    'artifacts_sha256':{image_path.name:sha(image_path), data_path.name:sha(data_path), Path(__file__).name:source_sha},
    'timing':'Original raw measured bolt/hole geometry is the retained native contact state at post label minus timestep. Baseline is the original solved row immediately preceding each qualified turn; time, depth and yaw use the same row. Sparse post-qpos and robot command inputs are not used.',
    'scope':'Visualization only, with all original phase points retained and explicit display-stride indices. Nominal pitch reference is descriptive geometry, not a prescribed helix or control input. Reuses the official accepted full-stroke lead-fit annotations unchanged; no fit, audit, physics check, material calibration, native import, model, forward/force solve, integration, FF ledger read or original/audit/render-tree rewrite.'}
with manifest_path.open('x') as stream:
    json.dump(manifest, stream, indent=2, allow_nan=False)
    stream.write('\n')
print(json.dumps({'figure':str(image_path), 'figure_sha256':sha(image_path),
                  'manifest_sha256':sha(manifest_path), 'phase_steps':[p['actual_phase_native_steps'] for p in selected]}))
