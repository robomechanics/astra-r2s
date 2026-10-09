# Cold B200 cone weight-transfer diagnostic

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

For an independent repeat, use an isolated checkout of the pinned producer
and the matched native CPU engine from `docs/m8_setup.md`. This diagnostic
package postdates that producer commit: read its exact diagnostic and observer
sources from a newer review checkout containing the published package, then
copy them into ignored outputs in the isolated producer checkout.

The example uses `/workspace/astra-r2s` as that review checkout and an unused
`/workspace/astra-r2s-cold-B200` as the isolated destination. Reassemble **all**
parent artifacts there, including `scene.xml`, `scene_source.py` and source
archives adjacent to the trace. Copying only the trace is insufficient for
the hash-verified archived-model loader. The parent helper restores the trace
as `trace.npz`; copy it to the diagnostic's guarded `insertion_trace.npz`
input name without changing its bytes:

```sh
supported_review_checkout=/workspace/astra-r2s
supported_cold_checkout=/workspace/astra-r2s-cold-B200
git clone https://github.com/robomechanics/astra-r2s.git "$supported_cold_checkout"
git -C "$supported_cold_checkout" switch --detach 66276d0dd2188c763ada69e7426e5d74cf64fd29
cd "$supported_cold_checkout"
python "$supported_review_checkout/media/m8_table_supported/failures/cone_release_alignment_abort/reassemble_archives.py" --output outputs/m8_table_supported/full_v2
cp outputs/m8_table_supported/full_v2/trace.npz outputs/m8_table_supported/full_v2/insertion_trace.npz
mkdir -p outputs/m8_table_supported/diagnostics/weight_transfer_observer_v1
cp "$supported_review_checkout/media/m8_table_supported/diagnostics/cone_weight_transfer_B200/diagnostic_source.py" outputs/m8_table_supported/diagnostics/load_transfer_probe.py
cp "$supported_review_checkout/media/m8_table_supported/diagnostics/cone_weight_transfer_B200/observer_source.py" outputs/m8_table_supported/diagnostics/weight_transfer_observer_v1/bolt_weight_transfer_observer.py
scripts/run_m8.sh outputs/m8_table_supported/diagnostics/load_transfer_probe.py --output outputs/m8_table_supported/diagnostics/repeated_B200 --checkpoint 12.76565 --damping 200 --ramp 0.5 --hold 0.5
```

Reuse the already verified shared native runtime; set it up from the pinned
checkout first if necessary. Choose a fresh diagnostic output name for each
repeat. The diagnostic and observer must occupy those exact ignored paths so
the original repository-root lookup and frozen observer identity resolve
correctly. The helper compares every producer dependency with its frozen hash
and rejects mismatches. A repeat is another cold diagnostic, never a resume
or qualification of the original unspliced native trajectory.
