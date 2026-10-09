"""Compare original recorded turn geometry; never integrate or infer forces."""
from pathlib import Path
import hashlib
import json
import os

os.environ.setdefault('MPLCONFIGDIR', '/workspace/.cache/matplotlib')
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path('/workspace/astra-r2s')
BRANCH = ROOT/'outputs/m8_table_supported/diagnostics/entry_gravity_start_B200_v1'
PARENT = ROOT/'media/m8_table_supported/failures/cone_release_alignment_abort'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    declaration = json.loads((BRANCH/'declaration.json').read_text())
    report = json.loads((BRANCH/'report.json').read_text())
    phase_audit = json.loads((BRANCH/'independent_phase_depth_audit.json').read_text())
    parent_manifest = json.loads((PARENT/'package_manifest.json').read_text())
    if declaration['parent_trace_sha256'] != parent_manifest['trajectory_sha256']:
        raise ValueError('Cold branch and original parent identities differ')
    # Obtain the timestep from the original fixed-step diagnostic clock.
    with np.load(BRANCH/'original_native_force_ledger.npz', allow_pickle=False) as saved:
        fields = ('time_s','elapsed_s','base_z_m','radial_offset_m','bolt_tilt_rad',
                  'formed_flank_overlap_m','loaded_actual_interior_flank_contact_count')
        data = {key: saved[key].copy() for key in fields}
    dt = float(data['elapsed_s'][0])
    if not np.allclose(np.diff(data['elapsed_s']), dt, rtol=0, atol=2e-12):
        raise ValueError('Unexpected diagnostic native time grid')
    with np.load(PARENT/'trace.npz', allow_pickle=False) as saved:
        parent_rows = json.loads(str(saved['info_json']))
        parent_metadata = json.loads(str(saved['metadata_json']))
    parent_dt = parent_metadata['scene_config']['base']['thread']['timestep']
    rows = [row for row in parent_rows if row['phase'] == 'start_thread_1']
    if not rows:
        raise ValueError('Original parent contains no recorded first starting turn')
    baseline = {
        'time_s': np.asarray([row['time'] for row in rows]),
        'base_z_m': np.asarray([row['bolt_base_insertion_m'] for row in rows]),
        'radial_offset_m': np.asarray([row['radial_offset_m'] for row in rows]),
        'bolt_tilt_rad': np.asarray([row['bolt_tilt_rad'] for row in rows]),
        'formed_flank_overlap_m': np.asarray([row['formed_flank_overlap_m'] for row in rows])}
    parent_start = float(baseline['time_s'][0]-parent_dt)
    if abs(parent_start-declaration['initial_saved_state_time_s']) > 1e-9:
        raise ValueError('The branch does not originate at the original starting-turn checkpoint')
    turn_start_elapsed = float(declaration['ramp_s']+declaration['hold_s'])
    requested_duration = (1.875*abs(declaration['physical_closed_turn_angle_rad']) /
                          declaration['physical_closed_turn_peak_speed_rad_s'])
    native_duration = round(requested_duration/dt)*dt
    geometry_elapsed = data['elapsed_s']-dt
    mask = ((geometry_elapsed >= turn_start_elapsed-1e-12) &
            (geometry_elapsed <= turn_start_elapsed+native_duration+1e-12))
    branch = {key: value[mask] for key,value in data.items()}
    parent_t = baseline['time_s']-parent_dt-parent_start
    branch_t = branch['elapsed_s']-dt-turn_start_elapsed
    for key,value in [*baseline.items(),*branch.items()]:
        if not np.isfinite(value).all():
            raise ValueError(f'Nonfinite recorded geometry: {key}')
    parent_withdrawal = baseline['base_z_m'][0]-baseline['base_z_m']
    branch_withdrawal = branch['base_z_m'][0]-branch['base_z_m']
    summary = {
        'parent_recorded_sample_maximum_withdrawal_m': float(max(0.,np.max(parent_withdrawal))),
        'branch_allstep_maximum_withdrawal_from_first_actual_turn_sample_m': float(max(0.,np.max(branch_withdrawal))),
        'branch_original_producer_report_withdrawal_m': report['physical_closed_turn']['maximum_measured_withdrawal_m'],
        'branch_independent_allstep_withdrawal_from_first_cold_native_row_m': phase_audit['maximum_withdrawal_from_initial_m'],
        'branch_first_actual_turn_native_base_z_m': float(branch['base_z_m'][0]),
        'branch_original_producer_summary_base_z_reference_m': report['physical_closed_turn']['measured_base_z_start_m'],
        'branch_first_cold_native_row_base_z_m': float(data['base_z_m'][0]),
        'branch_allstep_minimum_base_z_in_turn_m': float(np.min(branch['base_z_m'])),
        'branch_final_advance_from_first_actual_turn_sample_m': float(-branch_withdrawal[-1]),
        'parent_recorded_samples': len(rows), 'branch_original_native_steps_in_turn_plot': int(np.count_nonzero(mask)),
        'parent_recorded_maximum_formed_overlap_m': float(np.max(baseline['formed_flank_overlap_m'])),
        'branch_allstep_maximum_formed_overlap_m': float(np.max(branch['formed_flank_overlap_m'])),
        'branch_loaded_actual_interior_contact_substeps': int(np.count_nonzero(branch['loaded_actual_interior_flank_contact_count']))}
    np.savez_compressed(BRANCH/'comparison_data.npz',
        parent_relative_turn_time_s=parent_t, parent_withdrawal_m=parent_withdrawal,
        branch_relative_turn_time_s=branch_t, branch_withdrawal_m=branch_withdrawal,
        **{'parent_'+key: value for key,value in baseline.items()},
        **{'branch_'+key: value for key,value in branch.items()})
    plt.rcParams.update({'font.size':11, 'axes.spines.top':False, 'axes.spines.right':False})
    fig,axes = plt.subplots(2,2,figsize=(13,8),sharex=True)
    colors = ('#d36a21','#2460cc')
    labels = ('Original full_v2 recorded turn samples',
              'Cold gravity-first + B200: every native step')
    ax = axes[0,0]
    for t,v,c,label in zip((parent_t,branch_t),(parent_withdrawal,branch_withdrawal),colors,labels):
        ax.plot(t,v*1000,color=c,lw=1.5,label=label)
    ax.axhline(0,color='#555',lw=.7)
    ax.set_ylabel('Withdrawal from first turn sample (mm)')
    ax.text(.02,.95,f'Parent sampled peak: {summary["parent_recorded_sample_maximum_withdrawal_m"]*1000:.3f} mm\n'
        f'Cold branch all-step peak: {summary["branch_allstep_maximum_withdrawal_from_first_actual_turn_sample_m"]*1e6:.3f} µm',
        transform=ax.transAxes,va='top',fontsize=10)
    ax.legend(loc='center right',fontsize=9)
    ax.set_title('Positive = withdrawal; negative = advance')
    for ax, key,scale,label,note in (
        (axes[0,1],'radial_offset_m',1e6,'Radial offset (µm)','Unchanged radial guard: 150 µm'),
        (axes[1,0],'bolt_tilt_rad',180/np.pi,'Bolt tilt (degrees)','Unchanged tilt guard: 2°'),
        (axes[1,1],'formed_flank_overlap_m',1e6,'Formed full-ring overlap (µm)','No capture / no unsupported reset proof')):
        for t,records,color in ((parent_t,baseline,colors[0]),(branch_t,branch,colors[1])):
            ax.plot(t,records[key]*scale,color=color,lw=1.3)
        ax.set_ylabel(label)
        ax.text(.02,.95,note,transform=ax.transAxes,va='top',fontsize=10)
    axes[1,1].set_ylim(-.05,1.)
    axes[1,1].text(.05,.50,'Both traces: zero formed overlap\nCold branch: zero loaded interior-flank steps',
        transform=axes[1,1].transAxes,fontsize=11)
    for ax in axes.ravel():
        ax.grid(alpha=.18)
        ax.set_xlim(0,native_duration)
    for ax in axes[1]:
        ax.set_xlabel('Time from each actual motor-turn start (s)')
    fig.suptitle('Recorded closed π turns: lower cold-branch withdrawal, zero formed overlap\n'
                 'Separate cold initialization; no original warm starts, trajectory splice or release test',fontsize=16)
    fig.text(.5,.018,'Native geometry clock = recorded solve timestamp − dt; parent is sampled, branch is all-step. '
        'No integration or force reconstruction.',ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.04,1,.91))
    fig.savefig(BRANCH/'entry_gravity_comparison.png',dpi=150)
    fig.savefig(BRANCH/'entry_gravity_comparison.svg')
    plt.close(fig)
    manifest = {
        'scope': 'Scientific comparison of original recorded geometry during each closed starting stroke only. The branch is cold and changes both axial loading and damping. No convergence/capture/full-trajectory qualification or causal isolation is claimed.',
        'source_sha256':sha(__file__),
        'parent_trace_sha256':declaration['parent_trace_sha256'],
        'branch_original_ledger_sha256':sha(BRANCH/'original_native_force_ledger.npz'),
        'branch_original_checkpoint_sha256':sha(BRANCH/'checkpoint_trace.npz'),
        'parent_geometry_clock':'Original row time minus parent timestep; measured retained native geometry',
        'branch_geometry_clock':'Original native ledger time minus diagnostic timestep; measured retained native geometry',
        'parent_motor_turn_start_time_s':parent_start,
        'branch_motor_turn_start_time_s':declaration['initial_saved_state_time_s']+turn_start_elapsed,
        'branch_motor_turn_start_elapsed_s':turn_start_elapsed,
        'branch_native_timestep_s':dt, 'parent_native_timestep_s':parent_dt,
        'plotted_duration_s':native_duration,
        'axial_reference':'Separate first measured native geometry row at actual turn start for each curve. Positive reference_z-current_z means withdrawal; negative means advance.',
        'sample_density_scope':'Parent curve uses only original stored info_json samples (~5 ms). Branch curve contains every original 50 µs native ledger row in the turn window. No samples are interpolated or resimulated.',
        'force_reconstruction':False,
        'numeric_reference_note':'The original producer summary computes its minimum from sparse saved turn_samples, using a raw-ledger pre-turn reference. This plot uses every native ledger row in the actual turn window, from the first actual turn row. Those two reference_z values differ by only ~7.44e-12 m; their withdrawal difference is therefore primarily the sparse versus all-step minimum. The independent phase/depth audit instead uses the first new cold diagnostic native row as its reference and its all-step minimum. All original reports and bindings remain unmodified; each exact value and reference is preserved here.',
        'measured_summary':summary,
        'outputs':{name:{'bytes':(BRANCH/name).stat().st_size,'sha256':sha(BRANCH/name)}
                   for name in ('entry_gravity_comparison.png','entry_gravity_comparison.svg','comparison_data.npz')}}
    (BRANCH/'entry_gravity_comparison_manifest.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    (BRANCH/'entry_gravity_comparison_source.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps(summary,indent=2))


if __name__ == '__main__':
    main()
