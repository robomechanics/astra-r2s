"""Copy closed cold-turn evidence with exact hashes; no simulation or replay."""
from pathlib import Path
import hashlib
import json

ROOT = Path('/workspace/astra-r2s')
SOURCE = ROOT/'outputs/m8_table_supported/diagnostics/entry_gravity_start_B200_v1'
OBSERVER = ROOT/'outputs/m8_table_supported/diagnostics/weight_transfer_observer_v1'
PARENT = ROOT/'media/m8_table_supported/failures/cone_release_alignment_abort'
TARGET = ROOT/'media/m8_table_supported/diagnostics/entry_gravity_start_B200'


def sha(value):
    return hashlib.sha256(value).hexdigest()


def strict(raw):
    return json.loads(raw, parse_constant=lambda value:
        (_ for _ in ()).throw(ValueError(value)))


def main():
    if TARGET.exists():
        raise FileExistsError(TARGET)
    files = {str(p.relative_to(SOURCE)): p.read_bytes()
             for p in sorted(SOURCE.rglob('*')) if p.is_file()}
    for p in sorted(OBSERVER.iterdir()):
        if p.is_file():
            files['frozen_observer/' + p.name] = p.read_bytes()
    declaration = strict(files['declaration.json'])
    report = strict(files['report.json'])
    audit = strict(files['independent_weight_transfer_audit.json'])
    phase = strict(files['independent_phase_depth_audit.json'])
    ready = strict(files['independent_ready_window_scope.json'])
    binding = strict(files['independent_audit_binding.json'])
    precision = strict(files['withdrawal_precision_correction.json'])
    plot = strict(files['entry_gravity_comparison_manifest.json'])
    proof = strict(files['frozen_observer/software_proof.json'])
    phase_proof = strict(files['audit_sources/independent_phase_depth_software_proof.json'])
    parent = strict((PARENT/'package_manifest.json').read_bytes())
    producer = strict((PARENT/'run_publication_identity.json').read_bytes())
    assert declaration['parent_trace_sha256'] == parent['trajectory_sha256']
    assert sha((PARENT/'trace.npz').read_bytes()) == declaration['parent_trace_sha256']
    assert declaration['diagnostic_source_sha256'] == sha(files['diagnostic_source.py'])
    assert declaration['observer_sha256'] == sha(files['observer_source.py'])
    assert files['observer_source.py'] == files['frozen_observer/bolt_weight_transfer_observer.py']
    assert report['ledger_sha256'] == sha(files['original_native_force_ledger.npz'])
    assert report['trace_sha256'] == sha(files['checkpoint_trace.npz'])
    for name, value in audit['input_sha256'].items():
        assert sha(files[name]) == value, name
    for name, value in binding['files_sha256'].items():
        assert sha(files[name]) == value, name
    for name, value in binding['audit_sources_sha256'].items():
        assert sha(files['audit_sources/' + name]) == value, name
    assert precision['original_ledger_sha256'] == report['ledger_sha256']
    assert precision['original_report_sha256'] == sha(files['report.json'])
    assert precision['original_audit_binding_sha256'] == sha(files['independent_audit_binding.json'])
    for name, value in proof['files_sha256'].items():
        assert sha(files['frozen_observer/' + name]) == value, name
    for name, value in phase_proof['source_sha256'].items():
        assert sha(files['audit_sources/' + name]) == value, name
    assert proof['tests']['passed'] == 15 and proof['tests']['failed'] == 0
    assert phase_proof['tests']['passed'] == 4 and phase_proof['tests']['failed'] == 0
    assert plot['source_sha256'] == sha(files['entry_gravity_comparison_source.py'])
    assert plot['parent_trace_sha256'] == declaration['parent_trace_sha256']
    assert plot['branch_original_ledger_sha256'] == report['ledger_sha256']
    assert plot['branch_original_checkpoint_sha256'] == report['trace_sha256']
    for name, value in plot['outputs'].items():
        assert sha(files[name]) == value['sha256'], name
        assert len(files[name]) == value['bytes'], name
    assert report['all_original_guards_held'] and report['aborted'] is None
    assert report['ever_weight_transfer_ready'] is True
    assert report['final_weight_window']['ready_for_diagnostic_release_attempt'] is False
    assert report['partial_formed_loaded_release_allowed'] is False
    assert audit['capture_qualified'] is False and phase['capture_qualified'] is False
    assert ready['capture_qualified'] is False
    assert audit['actual_interior_contact']['maximum_loaded_interior_flank_count'] == 0
    assert phase['actual_positive_full_ring_overlap_substeps'] == 0
    assert ready['maximum_actual_formed_overlap_at_pass_m'] == 0
    assert ready['maximum_actual_loaded_interior_count_at_pass'] == 0
    assert max(map(len, files.values())) < 45_000_000
    for name, raw in files.items():
        if name.endswith('.json'):
            strict(raw)
    files['package_source.py'] = Path(__file__).read_bytes()
    manifest = {
        'status': 'closed_cold_checkpoint_turn_diagnostic_only',
        'parent_package': str(PARENT.relative_to(ROOT)),
        'parent_package_url': 'https://github.com/robomechanics/astra-r2s/tree/main/' + str(PARENT.relative_to(ROOT)),
        'parent_trace_sha256': declaration['parent_trace_sha256'],
        'parent_model_fingerprint': declaration['parent_model']['model_fingerprint'],
        'parent_model_xml_sha256': declaration['parent_model']['model_xml_sha256'],
        'parent_producer_commit': producer['producer_commit'],
        'parent_checkpoint_time_s': declaration['initial_saved_state_time_s'],
        'parent_checkpoint_index': declaration['initial_saved_state_index'],
        'parent_checkpoint_phase': declaration['initial_saved_phase'],
        'parent_checkpoint_qpos_qvel_sha256': declaration['initial_qpos_qvel_sha256'],
        'cold_initialization': declaration['cold_init_note'],
        'warmstarts_or_original_solver_state_restored': False,
        'continuous_full_trajectory': False,
        'checkpoint_spliced_into_parent': False,
        'parent_original_trace_report_sources_modified': False,
        'current_canonical_producer_tests': 213,
        'separate_output_only_observer_tests': proof['tests'],
        'separate_output_only_phase_geometry_tests': phase_proof['tests'],
        'proof_accounting': 'Historical 213-test producer proof, separate 15-test observer proof and separate 4-test pure geometry audit proof are not combined or relabeled.',
        'diagnostic_duration_s': report['actual_duration_s'],
        'diagnostic_wall_seconds': report['wall_seconds'],
        'original_native_steps': audit['original_native_steps'],
        'saved_diagnostic_sample_rows': audit['saved_original_sample_rows'],
        'native_timestep_s': plot['branch_native_timestep_s'],
        'axial_damping_Ns_per_m': declaration['axial_velocity_damping_Ns_per_m'],
        'net_feed_start_N': declaration['net_feed_start_N'],
        'net_feed_final_N': declaration['net_feed_final_N'],
        'original_diagnostic_guards_held': report['all_original_guards_held'],
        'original_abort': report['aborted'],
        'ever_cone_only_weight_transfer_ready': report['ever_weight_transfer_ready'],
        'final_exact100ms_release_ready': audit['independent_exact100ms']['ready'],
        'capture_qualified': False,
        'actual_loaded_interior_flank_contacts': 0,
        'maximum_formed_full_ring_overlap_m': 0,
        'opening_release_or_passive_reset_executed': False,
        'qualified_thread_lead_observed': False,
        'independent_exact100ms': audit['independent_exact100ms'],
        'comparison_geometry_summary': plot['measured_summary'],
        'independent_precision_correction': precision['reference_and_sampling'],
        'comparison_scope': plot['scope'],
        'withdrawal_reference_and_sampling_note': plot['numeric_reference_note'],
        'immutable_audit_precision_note': 'The earlier frozen independent_audit_binding timing_note is preserved verbatim. Its suggestion that a first-physical-turn reference gives the sparse producer 1.31545 micrometre value is superseded by the explicit raw extrema/reference table here and the separate reviewer precision sidecar, when present. The physical-turn all-step value is 1.346192 micrometres; the producer summary additionally uses sparse extrema.',
        'runtime': declaration['runtime'],
        'scope': 'Separate cold diagnostic from the parent feed_to_entry checkpoint. All original guards held during a gravity-feed ramp/hold, physical closed pi turn and post-turn hold. Formed overlap and actual loaded interior-flank contact remain zero. Earlier force-readiness windows are cone-only; final readiness fails with high positive hand support. There is no opening/release, passive reset, qualified lead or completed policy-training trajectory. Neither a warmstarted parent continuation nor a full successful trajectory is claimed.',
        'files': {name: {'bytes': len(raw), 'sha256': sha(raw)}
                  for name, raw in sorted(files.items())}
    }
    files['diagnostic_package_manifest.json'] = (json.dumps(manifest, indent=2, allow_nan=False)+'\n').encode()
    files['README.md'] = make_readme(manifest).encode()
    TARGET.mkdir(parents=True)
    for name, raw in files.items():
        path = TARGET/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(raw)
    (TARGET/'SHA256SUMS').write_text(''.join(f'{sha(raw)}  {name}\n' for name, raw in sorted(files.items())))
    for name, raw in files.items():
        assert (TARGET/name).read_bytes() == raw
    print(json.dumps({'folder': str(TARGET.relative_to(ROOT)), 'files': len(files)+1,
        'bytes': sum(map(len, files.values())), 'maximum_artifact_bytes': max(map(len, files.values())),
        'plot_sha256': sha(files['entry_gravity_comparison.png']),
        'ledger_sha256': report['ledger_sha256'], 'capture_qualified': False}, indent=2))


def make_readme(manifest):
    return '''# Cold gravity-first B200 starting-turn diagnostic

This **closed 7.3905-second diagnostic** starts from the exact parent `full_v2`
checkpoint at **6.75515 s**, saved phase `feed_to_entry`, index 1363. A new
`MjData` receives archived qpos/qvel/ctrl once; original solver state and warm
starts are absent. It is a separate cold branch.

The axial damper is **200 N s/m**. Net feed ramps from **0.05 N** to the
**0.262414 N** bolt weight for 0.5 s, holds for 0.5 s, then a physical closed
**pi turn** runs at peak 1 rad/s for **5.8905 s**, followed by a 0.5 s hold.
All original diagnostic guards hold and there is no abort. There is no imposed
axial position spring or helix/lead constraint.

The scientific plot compares original measured withdrawal, radial alignment,
tilt and formed overlap during the two actual starting turns. Each curve has
its own phase-relative time zero and first measured turn-row axial reference.
The parent curve uses its original recorded samples (~5 ms); the branch uses
every original native turn-window row (50 microseconds). No integration,
interpolation or force reconstruction from saved poses occurs.

![Original measured geometry comparison](entry_gravity_comparison.png)

The original parent has **1.332065 mm sampled peak withdrawal**. The cold
branch has **1.346192 micrometres all-step peak withdrawal** from its first
actual turn row. Both curves have **zero formed overlap**, and the cold branch
has **zero actual loaded interior-flank contact steps**. Axial feed, damping
and the cold solver initialization change together; this comparison does not
isolate which change causes the different response.

The micrometre values below have different references or sample densities:

| Quantity | Withdrawal | Exact reference / minimum |
| --- | ---: | --- |
| Plot: all native turn rows | 1.346192 micrometres | First actual turn row, base_z = -0.022600006480653734 m; all-step minimum = -0.02260135267280573 m |
| Original producer summary | 1.315453 micrometres | Pre-turn raw reference = -0.022600006473209856 m; minimum from sparse saved turn_samples |
| Reviewer correction: all native rows from producer reference | 1.346200 micrometres | Native row 19999 just before the turn; all-step minimum |
| Independent phase/depth audit | 1.315616 micrometres | First new cold native row, base_z = -0.0226000370564958 m; all-step minimum |

The producer reference and plot reference differ by only about 7.44e-12 m;
the producer-versus-plot withdrawal difference is primarily the sparse versus
all-step extrema. Original report/audit/binding bytes are preserved. The
earlier audit-binding suggestion about the physical-turn reference is
superseded by this explicit reference table and the separate reviewer
precision sidecar. The plotted turn-end advance (32.653 micrometres) also has
a different endpoint from the original report's final post-hold advance
(12.475 micrometres); neither is a qualified thread lead.

Some earlier exact 100 ms force-readiness windows pass, but every passing
window remains **cone-only**. The final independent exact 100 ms window fails:
approximately **99.11% mean signed thread reaction** accompanies **94.26% mean
positive right-hand support**, with only **3.55% loaded thread-support duty**.
A signed force mean near bolt weight does not establish unsupported formed
thread support. No opening/release, passive reset, capture or complete policy
trajectory was executed in this branch.

- `declaration.json`, `report.json`, `diagnostic_source.py`, `observer_source.py`:
  exact cold initialization, commands, original outcome and executed sources.
- `original_native_force_ledger.npz`, `checkpoint_trace.npz`: complete original
  native force/geometry ledger and diagnostic saved-state/sample archive.
- `entry_gravity_comparison_source.py`, its manifest, `comparison_data.npz`:
  exact plotting source, input/time/reference identities and selected original
  geometry data, without interpolation or quantization.
- `independent_*`, `audit_sources/`: closed independent original force/frame,
  geometry/depth and cone-only readiness audits with exact frozen sources.
- `withdrawal_precision_correction.json`: separate reviewer correction of
  reference/sampling wording, preserving earlier frozen report/audit bytes.
- `frozen_observer/`: separate 15-test diagnostic observer proof. The separate
  4-test pure geometry proof is in `audit_sources/`. These counts remain
  separate from the historical **213-test parent producer proof**.
- `diagnostic_package_manifest.json`, `SHA256SUMS`: source, runtime, parent,
  state, sample and artifact identities. Every raw original byte is preserved.

The [published failed parent](../../failures/cone_release_alignment_abort/README.md)
contains the original model/source archives, runtime bindings and 213-test
proof. Its trace SHA is
`02a03f0262edfd519a3080bc3d44588f2f2f12ac2da6d40ca44dc379e3acae4c`,
and producer commit is `66276d0dd2188c763ada69e7426e5d74cf64fd29`.
The [later pre-open B200 diagnostic](../cone_weight_transfer_B200/README.md)
is another distinct cold branch and has not been spliced into this trajectory.

Verify this package from its directory:

```sh
sha256sum -c SHA256SUMS
```

To repeat independently, use an isolated checkout of that pinned producer and
the matched native CPU engine in `docs/m8_setup.md`; a stock MuJoCo wheel is
insufficient. Parent source/plugin/engine identities must match the declaration.
Different native builds require explicit comparison before claiming identical
bytes or physics. Restore the parent archive using its reassembly instructions,
then rename its restored `trace.npz` to the original input path
`outputs/m8_table_supported/full_v2/insertion_trace.npz` without changing bytes.
Copy `diagnostic_source.py` to
`outputs/m8_table_supported/diagnostics/load_transfer_probe.py` and copy
`observer_source.py` to
`outputs/m8_table_supported/diagnostics/weight_transfer_observer_v1/bolt_weight_transfer_observer.py`.
Use a fresh output directory:

```sh
scripts/run_m8.sh outputs/m8_table_supported/diagnostics/load_transfer_probe.py --output outputs/m8_table_supported/diagnostics/repeated_entry_B200 --checkpoint 6.75515 --damping 200 --ramp 0.5 --hold 0.5 --turn-angle 3.141592653589793 --turn-speed 1 --post-turn-hold 0.5
```

The frozen diagnostic rejects mismatched parent dependencies. A repeated cold
experiment remains separate from the unspliced original native parent.
'''


if __name__ == '__main__':
    main()
