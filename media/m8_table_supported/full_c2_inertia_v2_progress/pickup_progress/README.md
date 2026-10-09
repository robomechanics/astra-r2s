# Fresh C2/inertia run: block stabilization and bolt pickup progress

This immutable historical prefix comes from **one fresh native run starting with separate free table/rest spawns**. It preserves the original recording through completed `lift_bolt` at **3.469999999998582s**,703 saved postintegration states. The same prefix contains actual left stabilization at **1.9000000000017974s** and settled bilateral bolt grasp at **2.820000000000097s**. There is no cold checkpoint, preposed workpiece or splice with an earlier trial.

The left arm secures the horizontal female block while the table bears its weight; the right arm acquires the side bolt and lifts it from the rest. **No entry alignment, formed thread capture, opening/reset or completed assembly is claimed by this package.** Those later stages need their own original observations and independent closure. The150µm entry guard is deliberately inactive while the bolt is beside the hole in pickup/transport; detail captions label that scope.

- [Normal1× GIF](render/demo.gif) and [MP4](render/demo.mp4).
- [Actual left stabilization detail](render/left_stabilized_detail.png), [actual lifted bolt detail](render/endpoint_detail.png), and [whole-arm view](render/demo.png).
- [Immutable source/state snapshot binding](native_snapshot/snapshot_binding.json) and [render identities](render/render_manifest.json).

The clip uses **42 exact original states at12fps**, encoded3.500s versus native3.470s. The exact saved endpoint occupies the last slot. GIF duration is3.500s; display timestamp quantization adds no interpolated state. Six distinct still-state bindings are checked in `render/media_identity_review.json`; `demo.png` and `bolt_lifted.png` intentionally show the same endpoint.

Rendering copies exact original qpos/qvel/time and refreshes only **`mj_kinematics`, `mj_comPos`, `mj_camlight`**. It invokes no `mj_forward`, collision discovery, contact/force solve, physics integration, interpolation, workpiece posing or extra body hiding. One detail camera is fixed on the real table/block; the other follows only the exact saved post-state bolt position. Both real camera poses are recorded per still. The inherited canonical “1× slow motion” footer means factor1 normal playback.

Displayed native force/contact records and retained geometry come from **t−50µs**; qpos/qvel are postintegration **t**. Robot command geometry/calibration and coherent retained arm velocity come from **t−100µs**, with the initialized first-command exception. Robot command norms are original finite control records, not measured contact forces or recomputed replay forces.

**All703 sampled inertia records in this pickup prefix are explicitly disabled.** Their `enabled`, `inertia_inputs_present` and `jacobian_derivatives_present` flags are false. Disabled placeholders are not measured mass, Jacobian-derivative or inverse inputs. The robot uses actual PD/finite motor commands for pickup. The configured approximate arm-inertia feedforward becomes active only in later eligible closed head-feedback phases with axial force float; this prefix makes no claim about its later performance.

The original archive/embedded metadata, XML, mesh ZIP, controller/scene/renderer/dependency snapshots and launch identity are preserved byte-for-byte. **Final every-step force/FF ledgers, full native closure and independent trajectory audit were pending at this capture.** Original sampled guard/load metadata is progress evidence, not a replacement for that final audit. No original acceptance labels are rewritten.

This source is **da69a9cd44a8312cc7b97365faf5e09c27a646e2**, with matching **672 software checks /74 files**. The complete source-bound software packet is included under `software_proof/`; its74 files and proof/runtime match the actual launch. This software proof is separate from native physics/trajectory qualification and all historical cold-trial proofs.

## Verify and replay

```sh
python media/m8_table_supported/full_c2_inertia_v2_progress/pickup_progress/reassemble_archives.py --verify-only
```

Use a fresh isolated producer checkout and copy this complete package from the newer review checkout; the older producer commit predates the media package:

```sh
task_review=/workspace/astra-r2s
task_replay=/workspace/astra-r2s-fresh-c2-pickup-review
git clone --no-hardlinks "$task_review" "$task_replay"
git -C "$task_replay" checkout --detach da69a9cd44a8312cc7b97365faf5e09c27a646e2
mkdir -p "$task_replay/outputs/m8_table_supported/progress_inputs"
cp -a "$task_review/media/m8_table_supported/full_c2_inertia_v2_progress/pickup_progress" \
  "$task_replay/outputs/m8_table_supported/progress_inputs/pickup_progress"
cd "$task_replay"
task_progress=outputs/m8_table_supported/progress_inputs/pickup_progress
python "$task_progress/reassemble_archives.py" --verify-only
scripts/run_m8.sh "$task_progress/render/renderer_sources/render_full_c2_inertia_v2_progress.py" \
  --repository-root "$PWD" "$task_progress/native_snapshot" \
  outputs/m8_table_supported/fresh_c2_pickup_geometry_replay
```

Prepare the matched native GCC CPU MuJoCo engine/plugin/bindings using `docs/m8_setup.md`. Stock-wheel physics does not match the recorded micron-contact engine. Source/runtime/core/plugin identities are preserved; a different-machine rebuild must be compared before claiming identical libraries. The portable renderer checks all74 producer bytes/model/runtime, exact copied files before/after rendering, and emitted original-state/media hashes. It never steps/reset/rebuilds the native trajectory. This package supports recording replay; a fresh full integration is a separate task with its own original outputs and audits.

Earlier carried-block media, cold V3/V4/V5 packages and historical fresh attempts remain immutable. This prefix does not certify material calibration, full-pitch/flank capture, final seating/preload, passive reset or policy robustness.
