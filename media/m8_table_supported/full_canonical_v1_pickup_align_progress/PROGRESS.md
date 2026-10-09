# Fresh tabletop pickup and alignment progress

[Normal-speed video](pickup_align_progress.mp4) · [GIF](pickup_align_progress.gif) · [Screenshot](pickup_align_progress.png)

This is the fresh native prefix of the table-supported M8 attempt produced by
`6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1`. Both workpieces start independently
on the table/rest. The left arm approaches and stabilizes the horizontal free
block on the solid table. The right arm grasps the separate AF20-headed M8 bolt,
lifts it from its rest, carries it, and aligns it over the female M8 × 1.25 bore.
The final saved state is `align_over_hole` at 4.819999999995436 seconds.

This package contains phase-only progress. The full attempt's all-step native
force ledgers were still open when the snapshot was taken. It supplies no full
trajectory audit, loaded-interior capture, qualified turn, passive reset, or
task-success proof. The independent cold search diagnostics are separate trials
and are not spliced into this recording.

The MP4 uses 58 exact recorded states at 12 fps, with no state interpolation or
manual object pose changes. Playback factor 1 means normal speed; the unchanged
canonical renderer footer reads `1× slow motion` for this factor. The 4.833333-second
encoded video duration reflects its integer frame count. A GIF, when present,
is encoded from these same MP4 frames without a new physics render.

The screenshot uses the unchanged canonical overview, bolt/bore close-ups, and
actual right wrist camera. Renderer visibility options are unchanged, including
its existing `geomgroup[3:]` filter; no additional object hiding was applied.
Media replay refreshed only `mj_kinematics`, `mj_comPos`, and `mj_camlight`.
It integrated no physics, discovered no contacts, and solved no forces.
Force overlays are archived original native samples from one timestep before
the displayed post-integration state. Their timing is preserved.

The original partial NPZ, scene XML/ZIP, source/helper archives, media, and
snapshot/render manifests are retained byte for byte. `portable_assets/` holds
all 18 exact file-backed scene meshes. `producer_sources/` holds all 65 source
files bound to the producer's 402-test software proof, copied here under
`original_software_proof/`. Passing software tests is separate from physical
task qualification. `exact_recorded_config.json` preserves the exact scene,
control, and runtime declarations from the saved recording; this includes a
50-microsecond native timestep, independent initial bolt world yaw, +120-degree
hex-face pickup, left 24 mm opening, and right 30 mm opening.

The original executed `render_progress.py` retains its `/workspace/astra-r2s`
ROOT path as provenance. Use the portable helper below on a relocated clone.
It finds the repository from its own location, checks all packaged files and
producer sources, and normalizes mesh paths only in memory. It retains the
original XML bytes, model fingerprint, source-bound canonical renderer, and
geometry-only refresh. Output must be outside this immutable package.

From the repository root:

```sh
cd media/m8_table_supported/full_canonical_v1_pickup_align_progress
sha256sum -c SHA256SUMS
python replay_geometry_only.py --verify-only
cd ../../..
LP_NUM_THREADS=2 scripts/run_m8.sh \
  media/m8_table_supported/full_canonical_v1_pickup_align_progress/replay_geometry_only.py \
  --output outputs/full_canonical_v1_pickup_align_replay
```

Replay requires the matched CPU MuJoCo environment configured by
`scripts/setup.sh` and the recorded native library/geometry plugin identity.
The helper rejects changed producer sources rather than silently using a later
controller or renderer. Running the fresh simulation is a separate operation;
this package's replay commands do not resume or modify the live attempt.
