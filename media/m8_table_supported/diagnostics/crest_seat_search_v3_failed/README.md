# Closed cold crest-search V3: abrupt-stop radial failure

This is a **separate cold failed diagnostic**, initialized from the exact `full_canonical_v1` saved postintegration checkpoint at **13.194899999975918 s**, local time reset to zero. The cold branch inherits the original closed grasp references and actual settled axial baseline, copies qpos/qvel/ctrl, and copies **no solver warm-start, old native force samples or prior force windows**. Its footage is not spliced into the original full attempt.

The new experimental observer requests a CLOSED stop after a measured return from the actual shallow crest. After a fresh **100 ms** cold support/quiet window, the branch reverses and requests a stop at **1.6191500000012047 s**, original measured return **50.0351879 µm** and desired independent clock **−1.84364899 rad**. The command angular speed switches from **−1.84271420 rad/s to zero** on the next **50 µs** step. Original Cartesian torque reaches its **2 N·m** cap and force reaches its **8 N** cap; radial error grows from **5.53608 µm** at the request to **150.84834 µm** at **1.6254000000012179 s**, violating the unchanged **150 µm** guard. Native wall time was **221.24656 s**, exit **1**. The original report remains **passed=false, partial=true**.

No quiet-stop confirmation, forward scan, formed capture, opening, reset or full fresh trajectory ran. The final invalid radial sample clears the original observer's epoch/current return flags. The earlier valid stop-request metadata remains preserved in `physical_motion_events`; it is a request to decelerate while CLOSED, not a confirmation or engagement result. The raw failure, source, original final cleared flags and earlier event remain separate and unchanged.

The warm-up only delays the undefined rolling table-mean check until enough fresh samples exist. Instantaneous collision/depth, pads, free-workpiece/drive, table contact, tip support, grasp, joint/force/torque guards remain active. After fresh coverage the unchanged rolling table90% / positive-left-up10% /99%duty rule applies. No old window is reused. Perfect-state pose/velocity feedback and bounded Cartesian robot controls are privileged; this is a diagnostic controller, not a learned-policy success result.

The **1,999** initial unavailable rolling table windows and original false table-window acceptance label remain in the raw data/report. The independent reader validates complete fresh local windows separately. It retransforms original archived thread/table/left contact frames, and matches right normals/world wrench between raw ledgers and saved samples. Individual right-pad local contact records were not archived, so that latter comparison is a consistency check with an explicit limitation, not an independent right-contact force reconstruction. Its **10 output-only reader regressions** remain separate from69 observer contracts and402 canonical tests.

## Inspect the actual media

- [Normal1× GIF](render/demo.gif) and [MP4](render/demo.mp4).
- [Original stop-request detail](render/stop_request_detail.png), [radial failure detail](render/abort_detail.png), [full-arm failure view](render/demo.png).
- [Original dense stop-boundary plot](render/scientific_plot/native_stop_boundary.png).
- Original [validation](native_v3/insertion_validation.json), [cold input declaration](native_v3/diagnostic_declaration.json), independent [audit binding](independent_audits/independent_audit_binding.json) and original [execution closure](execution_provenance/crest_seat_search_v3_execution_after.json).

The clip selects **20 exact original native saved states at12fps**, normal1×. MP4 is **1.6666667 s**, GIF centisecond rounding is **1.670 s**, versus actual native **1.6254 s**. The exact endpoint is the last frame; no interpolation occurs. The inherited canonical “1× slow motion” footer means factor1, normal playback. All five stills, frame indices/times, source/state/runtime hashes and originals are bound in `render/render_manifest.json`. Actual saved qpos/qvel are used with **ONLY `mj_kinematics`, `mj_comPos`, `mj_camlight`**: no `mj_forward`, collision discovery, contact-force solve, integration, object posing or additional body hiding. The canonical camera/filter remains unchanged; detail images use real cameras/default visible geometry.

Captions and the scientific plot use original native preintegration solves/retained geometry at row **t−50µs**. Saved qpos/qvel and actual aperture are postintegration **t**. Executed command calibration uses the prior retained solve **t−100µs**, with the first-command initialization exception. The plot compares signed independent-clock command omega with explicitly labelled actual angular-speed **magnitudes**. Its crest-return curve uses the retained prior request reference for diagnosis; it does not restore cleared current flags or pass the failed trial.

## Preserved source, raw data and test scopes

All original native arrays and metadata are preserved byte-for-byte: **32,508 native ticks /47 declared feedback columns**, plus every original left-pad/table row, contact record, phase/event/control declaration, archived XML/assets and source. The largest original archive is about11.38MB, so no chunks are needed here; the general lossless45MB limit and whole/stored SHA bindings remain active. `package_manifest.json` maps native filenames and hashes; `SHA256SUMS` verifies every stored file.

The canonical parent remains producer **`6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1`**, original **402 tests /65 unchanged source files**. Their exact bytes and proof are archived under `producer_sources/` and `original402_software_proof/`. The new uncommitted cold harness **85c485e5…**, observer **1d37ef46…**, builder **6d41c961…** and source-faithful contract test **d1f3347d…** have their own original before/after hashes and execution repository HEAD; they are not part of that402/65 proof. The original outer before manifest records actual **69 synthetic contracts passed in0.63s** (37observer +32fresh-window/bootstrap checks). No standalone contract log was saved; none is synthesized here. Independent output-only audits and reused publisher storage checks have separate sources/counts and do not qualify native threading.

## Verify and restore

From the newer review checkout containing this package:

```sh
python media/m8_table_supported/diagnostics/crest_seat_search_v3_failed/reassemble_archives.py --verify-only
python media/m8_table_supported/diagnostics/crest_seat_search_v3_failed/restore_original_run_layout.py --verify-only
```

Both helpers use standard Python and invoke no simulator. The original-layout helper checks every checksum and reconstructed whole archive, restores the original native relative names from their explicit map, and refuses to overwrite differing bytes. Source-only audit snapshots are provenance, not a standalone Python application/runtime.

The original independent binder bytes are retained. Two byte-identical root aliases of the original outer before/after files preserve its `../crest_seat_search_v3_execution_{before,after}.json` sibling links. Its original absolute paths and execution repository HEAD remain historical provenance. The native/media repeat recipes below do not promise portable reexecution of that path-bound independent reader: it expects the original declared parent path and run-name sibling layout. No auditor portability adapter, source revision or rewritten declaration is supplied here.

## Reproduce in an isolated producer checkout

The newer review checkout contains the published packages; the old producer commit does not. Copy COMPLETE packages from that review checkout into ignored inputs of a fresh isolated producer checkout. Keep all XML/source/report/ledger sidecars and original parent files, not just the NPZ. The following commands do not modify the newer review checkout:

```sh
task_review=/workspace/astra-r2s
task_trial=/workspace/astra-r2s-crest-v3-review
git clone --no-hardlinks "$task_review" "$task_trial"
git -C "$task_trial" checkout --detach 6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1
mkdir -p "$task_trial/outputs/m8_table_supported/cold_inputs"
cp -a "$task_review/media/m8_table_supported/full_canonical_v1_failed_evidence" \
  "$task_trial/outputs/m8_table_supported/cold_inputs/parent_v1_package"
cp -a "$task_review/media/m8_table_supported/diagnostics/crest_seat_search_v3_failed" \
  "$task_trial/outputs/m8_table_supported/cold_inputs/crest_v3_package"
cd "$task_trial"
task_parent_package=outputs/m8_table_supported/cold_inputs/parent_v1_package
task_crest_package=outputs/m8_table_supported/cold_inputs/crest_v3_package
python "$task_parent_package/reassemble_archives.py" --verify-only
python "$task_crest_package/reassemble_archives.py" --verify-only
python "$task_parent_package/restore_original_run_layout.py" \
  --output outputs/m8_table_supported/crest_v3_original_parent
mkdir -p outputs/m8_table_supported/diagnostics
cp "$task_crest_package/source_proof/crest_seat_search_probe_v3.py" \
  outputs/m8_table_supported/diagnostics/crest_seat_search_probe_v3.py
cp "$task_crest_package/source_proof/crest_seat_observer_v3.py" \
  outputs/m8_table_supported/diagnostics/crest_seat_observer_v3.py
```

The frozen harness derives ROOT from `Path(__file__).resolve().parents[3]` and imports its observer from the script directory; those exact placements are required. Prepare the matched native GCC CPU MuJoCo runtime/plugin/bindings using `docs/m8_setup.md`. Stock wheel physics is insufficient. Runtime identities are preserved; a rebuild on another machine must be compared before claiming identical native libraries.

The harness verifies all65 canonical source hashes, original failed parent controller/commit/closure, every required exact parent archive/report/force ledger, exact checkpoint row2651 and settled event/guard at13.1949s, original model/configuration/XML and finite qpos/qvel/ctrl. A source-only `--prepare-only` validates that input/layout without a native rollout; it still loads the matched model/runtime. A fresh native attempt uses:

```sh
scripts/run_m8.sh outputs/m8_table_supported/diagnostics/crest_seat_search_probe_v3.py \
  --parent outputs/m8_table_supported/crest_v3_original_parent/insertion_trace.npz \
  --checkpoint-time 13.1949 \
  --output outputs/m8_table_supported/diagnostics/crest_v3_repeat
```

The destination must not exist. This runs another cold native branch and never changes the original recorded trial. It may reproduce the failure; it does not make this closed trial successful.

For exact recorded-state media replay, the COMPLETE copied trial package already contains adjacent original XML/source/assets and closure sidecars:

```sh
scripts/run_m8.sh "$task_crest_package/render/renderer_sources/render_crest_seat_search_v3.py" \
  --repository-root "$PWD" "$task_crest_package/native_v3" \
  outputs/m8_table_supported/diagnostics/crest_v3_geometry_replay
MPLCONFIGDIR=/tmp/m8-media-matplotlib python \
  "$task_crest_package/render/scientific_plot/plot_crest_seat_search_v3_boundary.py" \
  "$task_crest_package/native_v3" outputs/m8_table_supported/diagnostics/crest_v3_plot_replay
```

The renderer checks all65 canonical producer bytes and original runtime before/after and every original run file after replay. It records newly emitted media hashes and exact original saved-state indices. The separate `render/media_identity_review.json` verifies the published five stills/20frame indices and encoded durations against original data; that read-only check is separate from native qualification. The renderer supports the explicit isolated repository root, never imports the altered diagnostic controller to drive physics and performs no dynamics solve/integration. Plotting needs NumPy/Matplotlib and reads original dense ledgers only.

The original full failed attempt, earlier cold packages and first carried-block demo remain immutable. This new failure evidence does not certify calibrated material properties, broad numerical convergence, full pitch engagement, final seating/preload, passive reset or learned-policy robustness.
