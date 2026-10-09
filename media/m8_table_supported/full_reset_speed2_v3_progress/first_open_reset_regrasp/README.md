# Fresh reset-speed2 run: first open reset and quiet regrasp progress

This historical immutable phase-end prefix belongs to one fresh native attempt using producer **9ae1a9fe76968a4013ea6c39e67718026622b9c0** and explicit `--reset-speed 2`. It starts with the original separate free block/table and bolt/rest spawns and preserves the COMPLETE original recording through **`settle_regrip_search_2`** at **24.41380000024869**s, with **4,909** actual saved postintegration states. No earlier pickup/entry/cold footage or checkpoint was spliced into this recording.

The observed milestones are preserved in the original copied completed-phase stdout and embedded original event metadata: **The actual fully-open settle passed after 0.29775 s with zero entire-right-robot/bolt contacts. The actual open reset then executed its full minus-pi tool command over 2.94525 s, with zero right-robot/bolt contacts throughout and at most 0.495080 µm axial / 1.522229 mrad bolt motion during reset. The following 0.12 s open hold also had zero contacts. The 0.25 s regrip transition had 8 contacts, followed by an actual 0.10 s continuous quiet bilateral regrasp acquired at the exact saved endpoint; solved right-pad forces were 15.884799/15.895741 N**. The release transition can contain real right-pad contacts; a zero-contact claim applies only to actual fully-open phases and their original records. The settled regrasp requires the original gate completion and actual continuous100ms quiet bilateral acquisition event. **At the endpoint formed-flank overlap is 20.212215 µm, below the 1.25 mm pitch, and loaded actual interior-contact count remains 0. Shallow starting-geometry support can include partial helical flank contact; interior 0 does not prove plain-cone support. The closed regrasp 100 ms native window measured 91.339723% mg thread support / 8.660251% positive hand support / 100% duty. Full formed-pitch capture remains unqualified**. This observed behavior does not establish a qualified full-pitch capture, successful whole trajectory, qualified reset reward or robust policy.

- [Normal1× GIF](render/demo.gif) and [MP4](render/demo.mp4).
- [Actual fully-open jaws](render/fully_open_detail.png), [actual open reset](render/open_reset_detail.png), and [actual settled regrasp](render/quiet_regrasp_detail.png).
- [Actual endpoint detail](render/endpoint_detail.png) and [whole-arm view](render/demo.png).
- Original [snapshot/source identity](native_snapshot/snapshot_binding.json), [completed-phase stdout](native_snapshot/native_stdout_stderr.log), [exact state/camera/media bindings](render/render_manifest.json), and [media identity review](render/media_identity_review.json).

The clip uses **293** original saved states at12fps, encoded **24.416666666666668**s (GIF**24420**ms) versus original native **24.41380000024869**s. Its last slot contains the exact endpoint. Presentation timestamps are quantized; no state interpolation occurs. Original NPZ states, model XML/assets, source/dependency snapshots, runtime/launch identity and embedded acceptance metadata remain byte-exact. Whole files larger than45,000,000bytes are preserved in lossless fixed-size binary chunks with whole-file and per-chunk hashes; no state is dropped or quantized.

Final dense every-step native contact/force/FF ledgers and independent full audit were pending AT THIS historical capture. Original sampled contact records are genuine original solves, but sparse samples do not replace whole-task validation. Every-step maxima declared in the copied original phase summaries retain their producer scope and await independent final audit.

Replay copies saved qpos/qvel/time and calls only **`mj_kinematics`, `mj_comPos`, `mj_camlight`**. No `mj_forward`, collision discovery, contact/force solve, integration, interpolation, workpiece posing or body hiding occurs. Real camera poses are recorded. The inherited canonical “1× slow motion” footer means factor1 normal playback.

Original solved contact records/retained geometry are from **t−50µs**, saved qpos/qvel are postintegration **t**, and executed command geometry/calibration/coherent retained arm velocity are from **t−100µs**, with the initialized first-command exception. Robot FF/PD/capped norms are original executed commands, not measured contact forces or new replay forces. Inertia FF is DISABLED during opening, free-hand reset and regrasp; false inverse/Jdot presence and zero placeholders are not measured inverse inputs. Eligible CLOSED head-feedback/axial-force-float samples are labeled active only when the original `enabled` field is true.

This exact producer has **672 software checks /74 unchanged files**, with its complete matching software packet under `software_proof/`; proofSHA **9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7** and actual run beginningSHA **1aceb2b906b6201ba715bde99dfe1394bb0880907b0f9914529a9c50adff1ec9**. Software proof is separate from native qualification and older672/402/281 or cold-test packets. Matched micron-contact engine/plugin identities are preserved.

## Verify and replay the original recording

```sh
python media/m8_table_supported/full_reset_speed2_v3_progress/first_open_reset_regrasp/reassemble_archives.py --verify-only
```

The isolated exact producer predates this new media. Copy the COMPLETE new packet from a newer review checkout:

```sh
task_review=/workspace/astra-r2s
task_replay=/workspace/astra-r2s-reset-speed2-progress-review
git clone --no-hardlinks "$task_review" "$task_replay"
git -C "$task_replay" checkout --detach 9ae1a9fe76968a4013ea6c39e67718026622b9c0
mkdir -p "$task_replay/outputs/m8_table_supported/progress_inputs"
cp -a "$task_review/media/m8_table_supported/full_reset_speed2_v3_progress/first_open_reset_regrasp" \
  "$task_replay/outputs/m8_table_supported/progress_inputs/first_open_reset_regrasp"
cd "$task_replay"
task_progress=outputs/m8_table_supported/progress_inputs/first_open_reset_regrasp
python "$task_progress/reassemble_archives.py" --verify-only
python "$task_progress/reassemble_archives.py" --output outputs/m8_table_supported/reset_speed2_progress_restored
task_native_snapshot=outputs/m8_table_supported/reset_speed2_progress_restored/native_snapshot
```

Reassembly verifies original whole-file hashes and restores every original model/source sidecar, including complete chunked NPZ bytes, into a NEW separate directory. Prepare the matched GCC CPU MuJoCo engine/plugin/bindings with `docs/m8_setup.md`; a stock wheel does not match the recorded micron-contact engine. Compare a different-machine build before claiming matching library bytes. The supplied plugin must already exist and match its explicit SHA; this renderer contains no plugin build path. The supplied REPLAY binary anchor is separate from the original native runtime’s plugin SOURCE hash, and does not establish original native plugin-binary equivalence. On another machine provide a matching already-built library by absolute path. This command recreates recorded geometry only:

```sh
scripts/run_m8.sh "$task_progress/render/renderer_sources/render_full_reset_speed2_v3_regrasp_progress_v2.py" \
  --repository-root "$PWD" \
  --plugin-library /workspace/astra-r2s/thread_lab/plugins/libm8_sdf.so \
  --plugin-sha256 53571638b1f6146e1dfd297e8dd5f750bb19c1efc70743180f40b94649489e18 \
  "$task_native_snapshot" \
  outputs/m8_table_supported/reset_speed2_regrasp_geometry_replay
```

The actual renderer verifies all74 source files, original runtime, model/assets/source snapshots, inputs before/after rendering and exact saved-state/media identities. This is distinct from a new native rollout, which needs its own output closure and independent audit. All older media and the first carried-block demonstration remain immutable.

Exact media-stage commands, start/end UTC, exit codes, stdout/stderr and recorder source are preserved under `stage_execution_provenance/`. A media-stage exit is separate from native trajectory qualification. Earlier failed media-stage attempts, if any, retain their original failure records. The archived actual v2 renderer and its portable basename alias are byte-identical, with their hashes bound in the render manifest.
