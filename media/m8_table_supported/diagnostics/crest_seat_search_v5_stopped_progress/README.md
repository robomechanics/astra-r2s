# V5 cold search: actual stopped-direction progress snapshot

This is an immutable **historical phase-end progress snapshot**, copied while a separate V5 cold native trial continued. It ends at **1.9062500000018106 s** in `stop_reverse_seat_1`, after the actual150ms C2 brake and a new100ms quiet/load window. The original observer confirms the **closed search direction event**. At this sampled boundary all native hard guards hold, radial offset is **15.316783µm**, tilt **1.026942mrad**, and the inherited original-grasp slip is **244.679519µm**. This is useful progress beyond the failed V4 brake.

**Formed overlap and loaded interior contact count are zero.** The sampled controller window records mean thread support **103.646460% of bolt weight**, positive upward hand support **0.485111%**, and100% loaded support duty over100ms. It describes shallow entry support, not full-flank capture, successful screwing, opening/reset, a fresh end-to-end trajectory or policy qualification. The cold branch initializes from the original full-attempt post-state13.1949s, retaining its measured closed grasp references but copying no solver warm-start or old force windows. Footage is not spliced into another trajectory.

## Actual media

- [Normal1× GIF](render/demo.gif) and [MP4](render/demo.mp4).
- [Clear original stopped endpoint](render/endpoint_detail.png) and [full-arm view](render/demo.png).
- Original [phase progress manifest](native_snapshot/progress_manifest.json), [separate snapshot binding](native_snapshot/snapshot_binding.json), and [cold declaration](native_snapshot/diagnostic_declaration.json).
- [Exact render/state/source identities](render/render_manifest.json).

The clip uses exact saved native qpos/qvel with **only `mj_kinematics`, `mj_comPos`, `mj_camlight`** to refresh geometry. There is no `mj_forward`, collision discovery, force solve, physics integration, interpolation, workpiece posing or extra body hiding. Fixed real cameras show the original head, shaft, block, table and left clamp. The clip shows **23 exact saved states at12fps**, encoded **1.916667s** (GIF1.920s) versus native1.90625s. The original endpoint occupies the last video slot.12fps ceiling and GIF centisecond rounding are presentation quantization; no new state is synthesized. The inherited canonical “1× slow motion” footer denotes factor1 normal playback.

Original native solved contact records/geometry are at saved-row **t−50µs**; qpos/qvel are postintegration **t**. Executed command geometry/calibration and coherent retained arm velocity are from the previous native solve at **t−100µs**, with the first-command initialization exception. FF/PD/capped-total values are **original executed robot command records**, not measured contact forces or forces recomputed during replay. No force is inferred from sparse states.

## Scope and original bytes

`native_snapshot/insertion_trace_partial.npz` is the complete byte-exact copied original prefix, including original sparse force/contact records, inertia command records, state arrays and embedded metadata. No original acceptance fields are rewritten. The original `progress_manifest.json` remains unchanged. An additive `snapshot_model_aliases.json` binds `scene_source.py` as an exact byte alias of the already copied recorded scene module, allowing the existing archive loader to validate the original XML/assets/source.

**The full every-step native force and inertia-command ledgers and final independent audit were pending when this snapshot was captured.** This package makes no dense-force validation or completed-rollout claim. The controller-observed100ms readiness above comes from original saved window metadata; it is not a new independent all-step audit. An eventual closed trial requires its own separate evidence package and cannot change this prefix retrospectively.

The parent producer remains exact **6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1 /402 software checks /65 files**. All65 matching source bytes and original proof are archived. The V5 harness **a84682f4…**, inertia helper **0913f964…** and18-source launch manifest have separate original identities. The active command adds approximate hybrid5 robot-arm inertia feedforward and coherent retained-velocity damping/drag throughout the selected closed phases, while retaining the original8N/2N·m combined Cartesian and native motor caps. Its mass subblock includes downstream finger inertia while treating independent finger acceleration/coupled contact dynamics as an approximation; no axial inversion, bolt inertia drive, prescribed pitch/groove phase or free-object pose changes are introduced.

`source_proof/robot_inertia_feedforward_v5_pure_proof.json` records **67 pure/mocked command tests**, and `source_proof/inertia_rejection_artifacts_v5_pure_proof.json` records **six synthetic rejection-artifact tests**. Their sources/logs are retained and their scopes remain separate from historical69 observer contracts,50 C2 helper tests and402 canonical software checks. None qualifies native threading, release, convergence or policy performance. The18-source launch identities and65 producer files are verified before/after media generation; this is a source/media preservation check, not final native-run closure.

## Verify and replay this recording

```sh
cd media/m8_table_supported/diagnostics/crest_seat_search_v5_stopped_progress
sha256sum -c SHA256SUMS
```

Use a fresh isolated producer checkout and complete copies from the newer review checkout; the older6e7 commit predates this package:

```sh
task_review=/workspace/astra-r2s
task_replay=/workspace/astra-r2s-crest-v5-progress-review
git clone --no-hardlinks "$task_review" "$task_replay"
git -C "$task_replay" checkout --detach 6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1
mkdir -p "$task_replay/outputs/m8_table_supported/cold_inputs"
cp -a "$task_review/media/m8_table_supported/diagnostics/crest_seat_search_v5_stopped_progress" \
  "$task_replay/outputs/m8_table_supported/cold_inputs/v5_progress"
cd "$task_replay"
task_progress=outputs/m8_table_supported/cold_inputs/v5_progress
```

Prepare the matched GCC CPU MuJoCo engine/plugin/bindings following `docs/m8_setup.md`; a stock wheel does not match the recorded micron-contact engine. Runtime/core/plugin identities are preserved. A different-machine rebuild requires comparison before claiming identical libraries. The portable replay command reads only the immutable copied recording:

```sh
scripts/run_m8.sh "$task_progress/render/renderer_sources/render_crest_seat_search_v5.py" \
  --repository-root "$PWD" --mode progress "$task_progress/native_snapshot" \
  outputs/m8_table_supported/diagnostics/v5_progress_geometry_replay
```

It validates all65 producer files, archived model/source/assets and original runtime, checks input/source identities after rendering, and never integrates. The original18 experimental source files are archived for review; this progress package supplies a recording replay, not a qualified native-repeat result. A fresh native diagnostic needs the complete original full-failure parent package restored with its exact34-file layout and the frozen V5 harness/observer/brake/inertia modules directly under `outputs/m8_table_supported/diagnostics/`. That integration is separate from replay and requires its own observed closure/evidence.

The original full failure, V3/V4 failures, older cold packages and first carried-block demo remain immutable.
