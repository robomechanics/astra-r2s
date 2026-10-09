# Fresh C2/inertia run: alignment, entry and weight-transfer progress

This is a historical immutable phase-end prefix of **one fresh native attempt**, beginning with the original separate free table/rest spawns. It preserves the complete original recording through `transfer_bolt_weight` at **13.19459999997592**, with **2652** original saved postintegration states. The prefix includes the actual earlier left stabilization, bolt grasp/lift, high transport, alignment and physical entry; it is never spliced with the earlier pickup package or a cold checkpoint.

The original `feed_to_entry` phase used **6.67465s** of actual entry motion after alignment at4.82s. Its sampled original entry-support observer passed its50ms native impulse/slow/aligned gate. The separate completed transfer phase then passed its own actual100ms quiet/native90–10 weight window. Exact native endpoints, force/window fields and phase events are preserved in the embedded original metadata and original saved rows. The final original controller-observed window covers100.05ms/2001 native samples, with mean thread support100.003432% of bolt weight, positive upward hand support1.383709%, and100% loaded support duty. Formed/interior duration remains zero. These are original saved window fields, not a newly executed independent all-step audit.

**Formed thread overlap and loaded actual interior contact count remain zero at this endpoint.** This progress establishes controller-observed shallow entry and weight transfer only. It does not establish formed full-pitch capture, quiet direction search completion, opening/reset, full trajectory completion or policy robustness. Final every-step force and inertia ledgers and independent full audit were pending at this capture; sparse sampled observations are not a replacement for that audit.

- [Normal1× GIF](render/demo.gif) and [MP4](render/demo.mp4).
- [Actual bolt/block entry detail](render/endpoint_detail.png) and [whole-arm view](render/demo.png).
- Original [snapshot/source identity](native_snapshot/snapshot_binding.json), [phase event/state/media bindings](render/render_manifest.json), and [media identity review](render/media_identity_review.json).

The clip shows **159** exact original states at12fps, encoded **13.250s** (GIF**13250**ms) versus native**13.19459999997592**s. The original endpoint occupies the last slot. Finite presentation timestamp quantization adds no interpolated state. The complete original NPZ, XML/assets, source/dependency snapshots, launch/runtime identities and embedded acceptance metadata are preserved byte-for-byte.

Replay copies actual saved qpos/qvel/time and calls only **`mj_kinematics`, `mj_comPos`, `mj_camlight`** to refresh geometry. It invokes no `mj_forward`, collision discovery, native/contact-force solve, integration, interpolation, workpiece posing or extra body hiding. Real camera poses are recorded. The inherited canonical “1× slow motion” footer means factor1 normal playback.

Original solved contact records/retained geometry are from **t−50µs**, while saved qpos/qvel are postintegration **t**. Executed robot command geometry/calibration and coherent retained arm velocity are from **t−100µs**, with the initialized first-command exception. Robot FF/PD/capped norms are original executed command records, not measured contact forces or replay forces. Disabled pickup/transport records have false inverse-input/Jdot presence flags and zero placeholders; eligible closed head-feedback/axial-force-float records are labeled active only when their original `enabled` field is true. No inverse input is inferred from a disabled placeholder. This original sparse prefix contains2311 disabled and341 active inertia-command records; those counts describe saved samples, not the pending every-step ledger.

This exact producer is **da69a9cd44a8312cc7b97365faf5e09c27a646e2**, with matching **672 software checks /74 unchanged files** and the complete source-bound software packet included under `software_proof/`. The same matched micron-contact engine/core/plugin identities are preserved. This proof is separate from native trajectory qualification and historical cold-trial proofs.

## Verify and replay the recording

```sh
python media/m8_table_supported/full_c2_inertia_v2_progress/entry_transfer/reassemble_archives.py --verify-only
```

An isolated old producer checkout predates this media; restore the complete package from the newer review checkout:

```sh
task_review=/workspace/astra-r2s
task_replay=/workspace/astra-r2s-fresh-c2-entry-review
git clone --no-hardlinks "$task_review" "$task_replay"
git -C "$task_replay" checkout --detach da69a9cd44a8312cc7b97365faf5e09c27a646e2
mkdir -p "$task_replay/outputs/m8_table_supported/progress_inputs"
cp -a "$task_review/media/m8_table_supported/full_c2_inertia_v2_progress/entry_transfer" \
  "$task_replay/outputs/m8_table_supported/progress_inputs/entry_transfer"
cd "$task_replay"
task_progress=outputs/m8_table_supported/progress_inputs/entry_transfer
python "$task_progress/reassemble_archives.py" --verify-only
```

If the manifest contains lossless binary chunks, reassemble into a NEW separate output directory before replay, retaining every original filename/model sidecar. The original whole-file SHA is checked; there is no dropped/quantized state or rewritten metadata:

```sh
python "$task_progress/reassemble_archives.py" --output outputs/m8_table_supported/entry_progress_restored
task_native_snapshot=outputs/m8_table_supported/entry_progress_restored/native_snapshot
```

For an unchunked package, `task_native_snapshot="$task_progress/native_snapshot"` is sufficient. Prepare the matched GCC CPU MuJoCo engine/plugin/bindings following `docs/m8_setup.md`; a stock wheel does not match the recorded micron-contact engine. Compare a different-machine rebuild before claiming identical libraries. This command replays geometry only and never reintegrates the original motion:

```sh
scripts/run_m8.sh "$task_progress/render/renderer_sources/render_full_c2_inertia_v2_progress.py" \
  --repository-root "$PWD" "$task_native_snapshot" \
  outputs/m8_table_supported/fresh_c2_entry_geometry_replay
```

The portable renderer validates all74 producer files, archived model/source/assets and original runtime, original inputs before/after rendering and emitted saved-state/media hashes. This recording replay is distinct from a new native rollout, which needs its own output closure and independent audit. Earlier pickup/cold/full evidence and first carried-block media remain immutable.
