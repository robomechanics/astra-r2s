"""Publish exact closed cold-checkpoint evidence; no integration or replay."""
from pathlib import Path
import hashlib
import json

ROOT = Path('/workspace/astra-r2s')
SOURCE = ROOT/'outputs/m8_table_supported/diagnostics/stop_transfer_B200_v2'
OBSERVER = ROOT/'outputs/m8_table_supported/diagnostics/weight_transfer_observer_v1'
PARENT = ROOT/'media/m8_table_supported/failures/cone_release_alignment_abort'
TARGET = ROOT/'media/m8_table_supported/diagnostics/cone_weight_transfer_B200'


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
    plot = strict(files['native_load_chatter_binding.json'])
    plot_source = strict(files['native_load_chatter_plot_source_binding.json'])
    proof = strict(files['frozen_observer/software_proof.json'])
    parent = strict((PARENT/'package_manifest.json').read_bytes())
    producer = strict((PARENT/'run_publication_identity.json').read_bytes())
    assert declaration['parent_trace_sha256'] == parent['trajectory_sha256']
    assert declaration['diagnostic_source_sha256'] == sha(files['diagnostic_source.py'])
    assert declaration['observer_sha256'] == sha(files['observer_source.py'])
    assert files['observer_source.py'] == files['frozen_observer/bolt_weight_transfer_observer.py']
    assert report['ledger_sha256'] == sha(files['original_native_force_ledger.npz'])
    assert report['trace_sha256'] == sha(files['checkpoint_trace.npz'])
    for name, value in audit['input_sha256'].items():
        assert sha(files[name]) == value, name
    assert audit['auditor_source_sha256'] == sha(files['audit_sources/independent_weight_transfer_audit.py'])
    for name, value in proof['files_sha256'].items():
        assert sha(files['frozen_observer/' + name]) == value, name
    assert proof['tests']['passed'] == 15 and proof['tests']['failed'] == 0
    assert plot['plot_sha256'] == sha(files['native_load_chatter.png'])
    assert plot['ledger_sha256'] == report['ledger_sha256']
    assert plot_source['source_sha256'] == sha(files['native_load_chatter_plot_source.py'])
    assert plot_source['plot_sha256'] == plot['plot_sha256']
    assert plot_source['original_plot_binding_sha256'] == sha(files['native_load_chatter_binding.json'])
    assert report['ever_weight_transfer_ready'] is False
    assert audit['capture_qualified'] is False
    assert audit['actual_interior_contact']['maximum_loaded_interior_flank_count'] == 0
    for name, raw in files.items():
        if name.endswith('.json'):
            strict(raw)
    files['package_source.py'] = Path(__file__).read_bytes()
    exact = audit['independent_exact100ms']
    manifest = {
        'status': 'cold_checkpoint_diagnostic_only',
        'parent_package': 'media/m8_table_supported/failures/cone_release_alignment_abort',
        'parent_package_url': 'https://github.com/robomechanics/astra-r2s/tree/main/media/m8_table_supported/failures/cone_release_alignment_abort',
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
        'proof_accounting': 'The separate 15 observer tests are not combined with or relabeled as the 213-test native producer proof.',
        'diagnostic_duration_s': report['actual_duration_s'],
        'original_native_steps': audit['original_native_steps'],
        'native_timestep_s': plot['native_timestep_s'],
        'axial_damping_Ns_per_m': declaration['axial_velocity_damping_Ns_per_m'],
        'net_feed_start_N': declaration['net_feed_start_N'],
        'net_feed_final_N': declaration['net_feed_final_N'],
        'original_diagnostic_guards_held': report['all_original_guards_held'],
        'ever_weight_transfer_ready': False,
        'capture_qualified': False,
        'actual_loaded_interior_flank_contacts': 0,
        'independent_exact100ms': exact,
        'signed_force_mean_scope': 'Mean signed thread reaction approximately balances bolt gravity, but periodic alternating right-pad loads remain. Positive hand support is high; a near-zero signed hand mean is not unsupported thread support.',
        'plot_source_provenance': plot_source['scope'],
        'scope': 'One-second cold checkpoint experiment at the pre-open full_v2 stop, with damping B200. Exact archived qpos/qvel/ctrl initialize a new MjData once; original warm starts and solver state are absent. Native entry contacts only, zero actual interior-flank loading, no capture, no passive reset, no qualified lead and no completed policy-training trajectory. Original force records and independent retransformation remain separate from pose replay.',
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
        'bytes': sum(map(len, files.values())), 'plot_sha256': plot['plot_sha256'],
        'ledger_sha256': report['ledger_sha256'], 'capture_qualified': False}, indent=2))


def make_readme(manifest):
    return '''# Cold B200 cone weight-transfer diagnostic

This one-second diagnostic starts from the exact `full_v2` pre-open checkpoint
at **12.76565 s**, saved phase `stop_start_1`. It initializes a new `MjData` from
archived qpos/qvel/ctrl once. Original solver state and warm starts are absent.
It is neither a continuous full trajectory nor a checkpoint splice.

The axial damper is **200 N s/m**, with net feed ramped from 0.05 N to the
0.262414 N bolt weight over 0.5 s, then held for 0.5 s. The original diagnostic
guards hold, but release readiness never passes. The independently evaluated
last exact 100 ms has approximately **99.97% mean signed thread support** and
**67.08% mean positive right-hand support**, with **zero loaded interior-flank
contacts**. Periodic cone/right-pad load chatter explains why mean force balance
does not establish unsupported formed-thread capture.

`native_load_chatter.png` plots original all-step native force records, without
replay, integration or force reconstruction from saved poses. The independent
audit retransforms archived native contact-local forces and frames. The plot
source was saved after its original stdin execution; its separate binding
records that timing, and the earlier plot/data/binding bytes remain unchanged.

The **15 diagnostic observer tests remain separate** from the historical
**213-test full_v2 producer proof**. No test counts are added together. This
package makes no completed trajectory, qualified lead, passive reset, capture,
material calibration or policy-training qualification claim.

- `declaration.json` and `report.json`: exact cold initialization, commands,
  force timing and native diagnostic outcome.
- `original_native_force_ledger.npz` and `checkpoint_trace.npz`: complete
  original diagnostic force ledger and saved diagnostic state/sample archive.
- `observer_source.py`, `diagnostic_source.py` and `frozen_observer/`: actual
  executed diagnostic source and separate versioned 15-test observer proof.
- `independent_weight_transfer_audit.json`, `independent_audit_binding.json`
  and `audit_sources/`: closed independent original-force/frame audit and source.
- `native_load_chatter_plot_source.py` and its binding: recorded scientific
  plotting source, linked to the exact plot and force ledger.
- `diagnostic_package_manifest.json` and `SHA256SUMS`: parent checkpoint/model,
  source/runtime and all artifact identities.

The exact parent trace SHA is
`02a03f0262edfd519a3080bc3d44588f2f2f12ac2da6d40ca44dc379e3acae4c`.
Its [published failed parent package](../../failures/cone_release_alignment_abort/README.md)
preserves source/model/runtime archives and the 213-test proof. Its producer is
`66276d0dd2188c763ada69e7426e5d74cf64fd29`. Later main-branch source changes must
not be silently substituted when trying this frozen diagnostic.

To verify this small package from its directory:

```sh
sha256sum -c SHA256SUMS
```

For an independent repeat, use an isolated checkout of that pinned producer,
the matched native CPU engine from `docs/m8_setup.md`, and the archived parent
model/trace. Restore this diagnostic source into
`outputs/m8_table_supported/diagnostics/` so its original repository-root lookup
resolves correctly; restore the exact observer source at
`outputs/m8_table_supported/diagnostics/weight_transfer_observer_v1/bolt_weight_transfer_observer.py`.
The diagnostic's guarded parent input is
`outputs/m8_table_supported/full_v2/insertion_trace.npz`; the parent reassembly
helper restores it as `trace.npz`, which must be renamed to that original input
name without changing its bytes. Use a new output directory:

```sh
scripts/run_m8.sh outputs/m8_table_supported/diagnostics/load_transfer_probe.py --output outputs/m8_table_supported/diagnostics/repeated_B200 --checkpoint 12.76565 --damping 200 --ramp 0.5 --hold 0.5
```

Copy `diagnostic_source.py` to the `load_transfer_probe.py` path above before
running that command. The helper compares every producer dependency with its
frozen hash and rejects mismatches. A repeat is another cold diagnostic, never
a resume or qualification of the original unspliced native trajectory.
'''


if __name__ == '__main__':
    main()
