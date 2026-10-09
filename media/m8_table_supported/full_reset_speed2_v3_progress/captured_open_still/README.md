# Fresh reset-speed2 progress: recorded captured-open state

These two images show one actual fully-open saved state from the fresh native
producer **`9ae1a9fe76968a4013ea6c39e67718026622b9c0`**. The original controller
records a complete **2.94525 s** minus-pi reset with zero whole-right/bolt,
bolt/world-support or head-seating contacts. At the selected state, formed
overlap is **1.271645879905 mm**, and the original final 100 ms / 2,000-tick
OPEN window records **99.999999735%** of bolt weight in thread reaction,
zero positive upward hand support and 100 ms loaded interior duration.

[Whole-arm captured-open state](render/captured_open.png) ·
[Thread and jaw detail](render/captured_open_detail.png).

The state is row **8,749**, `reset_open_1`, at **43.53530000088345 s**. During
the entire recorded contact-free reset, peak bolt axial drift is
**27.909003 nm**, with **143.765332 µrad** yaw drift. The original single
contact solve reports table reaction **3.2611865510843736 N**, left-pad
forces **15.99800775521543 / 15.995647756938666 N**, and right-pad forces
**0 / 0 N**. These are original solved forces, not new forces from replay.
Inertia feedforward is disabled; absent inverse/Jdot inputs remain explicit.
Saved qpos time is **43.53530 s**; original retained command time is
**43.53520 s**. The contact solve/retained native geometry is **43.53525 s**,
preceding the saved state by 50 µs.

The COMPLETE immutable fresh prefix contains **8,847 original states** through
`settle_regrip_1` at **44.005300000899055 s**. The selected image is earlier
than that true endpoint: the prefix is never trimmed or relabeled. It starts
from this run's original independent table/rest spawns, without checkpoint
initialization, previous clips or cold-trajectory stitching. The snapshot
binding SHA is
`9c3deb00a19066c7c8e4d0be99d652257a4da0e98bac4fa451c4758edf9b32f7`;
its complete NPZ SHA is
`c0f1ed6f05001cb7724af308fddbeb9c48109a8b1970d2a41b8dc7553a645829`.

The full native trajectory is still LIVE when this progress packet is
prepared. Dense final ledgers and independent whole-run audits are pending.
Original captured-open flags, window values and recorded phase maxima retain
their producer scope; these stills do not establish final qualified lead,
turn/reset acceptance, whole-task success, a learned policy or hardware
calibration. Do not audit live arrays or substitute this prefix for the final
run; require the untouched original AFTER and confirmed closure first.

## Preserved recording and source scope

The renderer copies one saved qpos/qvel/time and refreshes only
`mj_kinematics`, `mj_comPos`, `mj_camlight`. It performs no `mj_forward`,
collision discovery, force/contact solve, integration, interpolation,
workpiece reposing, body hiding or scene regeneration. No video is generated;
the two PNGs depict the same actual stored state with recorded camera views.

Actual v3 source
`7999ff1a3c97fa8981fcacafce15e50840b0f3df11d4631fd08abc166d4085de`
produced manifest
`ba2c3ec1ba29b333189aa1f85dfae1100ec7041fdec11db929a64bbf3cd608b8`.
Its stage ran **16:44:19–16:44:45 UTC**. The earlier successful v2 source
`2c34…` / manifest `29934…`, **16:40:52–16:41:26 UTC**, remains separately
preserved under `render_attempts/selected_reset_render_v2/`; v3 only corrects
caption layout. Both original stage records and sources are retained; no
native model, pose, geometry or original force byte is changed by that fix.

The [matching 672-test / 74-source proof](../../software_proof_reset_speed2_v3/README.md)
has SHA `9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7`;
the original run BEFORE is
`1aceb2b906b6201ba715bde99dfe1394bb0880907b0f9914529a9c50adff1ec9`.
Software checks and image identity are separate from native acceptance.
The first carried-block demo, earlier failed full run and every older
progress packet remain immutable.

The final package manifest lists its actual artifact/file/checksum counts,
complete byte inventories and original-to-published mappings. It preserves
all **34 original snapshot files**, all 74 producer sources, the primary
render and earlier render attempt, exact stage provenance and publication
dependencies. Files above 45,000,000 bytes use ordered contiguous lossless
chunks, retaining whole-file and per-chunk SHA checks. Restore every model,
source and metadata sidecar, not only the NPZ.

## Verify and restore the complete original prefix

The exact producer predates this media. Copy the COMPLETE newer packet into
ignored inputs of an unused pinned clone; the original-layout helper restores
the exact snapshot filenames from its bound manifest:

```sh
task_review=/workspace/astra-r2s
task_clone=/workspace/astra-r2s-reset2-captured-open-review
git clone --no-hardlinks "$task_review" "$task_clone"
git -C "$task_clone" switch --detach 9ae1a9fe76968a4013ea6c39e67718026622b9c0
mkdir -p "$task_clone/outputs/m8_table_supported/progress_inputs"
cp -a "$task_review/media/m8_table_supported/full_reset_speed2_v3_progress/captured_open_still" "$task_clone/outputs/m8_table_supported/progress_inputs/captured_open_still"
cd "$task_clone"
task_package=outputs/m8_table_supported/progress_inputs/captured_open_still
task_artifacts=outputs/m8_table_supported/captured_open_artifacts
task_snapshot=outputs/m8_table_supported/captured_open_original_snapshot
python "$task_package/reassemble_archives.py" --verify-only
python "$task_package/restore_original_run_layout.py" --verify-only
python "$task_package/reassemble_archives.py" --output "$task_artifacts"
python "$task_package/restore_original_run_layout.py" --output "$task_snapshot"
```

These standard-library byte operations import no engine, verify the complete
snapshot/chunks and refuse differing existing destinations. Use absent
destinations and keep all immutable input/stage files unchanged.

## Geometry-only two-image replay

Reuse the exact matched runtime described in `docs/m8_setup.md`. This renderer
checks the recorded native library paths and SHA bytes; it does not offer a
runtime relocation override. A different host or binary is not assumed
compatible. The supplied plugin must already exist: its explicit
**REPLAY-only** binary SHA below is separate from the original native plugin
SOURCE identity and does not invent an original native binary hash. There is
no plugin build path. Never rebuild a shared runtime during native integration.

The renderer archives the required approved compiler and origin sources.
Supply their actual restored paths explicitly, instead of relying on ignored
helpers being present in the old producer checkout:

```sh
task_sources="$task_artifacts/render/renderer_sources"
scripts/run_m8.sh "$task_sources/render_full_reset_speed2_v3_captured_open_still_v3.py" \
  --repository-root "$PWD" \
  --snapshot-binding-sha256 9c3deb00a19066c7c8e4d0be99d652257a4da0e98bac4fa451c4758edf9b32f7 \
  --expected-phase reset_open_1 \
  --approved-renderer-source "$task_sources/render_full_reset_speed2_v3_regrasp_progress_v2.py" \
  --compiler-origin-source "$task_sources/render_full_reset_speed2_v3_closed.py" \
  --plugin-library /workspace/astra-r2s/thread_lab/plugins/libm8_sdf.so \
  --plugin-sha256 53571638b1f6146e1dfd297e8dd5f750bb19c1efc70743180f40b94649489e18 \
  "$task_snapshot" outputs/m8_table_supported/captured_open_geometry_replay
```

The replay verifies the complete prefix inventory/endpoint, actual selected
row, original phase record, all 74 current producer files, model/source
assets, core/runtime and supplied plugin before and after rendering. This
recreates two recorded-state images; it neither reruns native contact nor
reproduces original force histories. To run a NEW continuous native trajectory,
follow the [pinned fresh-run handoff](../../../../docs/m8_supported_agent_handoff.md#current-producer-fresh-run)
with `--reset-speed 2` and a separate absent output. Its own closure and
acceptance remain independent; do not initialize it from this snapshot.
