# Table-supported M8 stabilization progress

This is an actual native four-phase stabilization pilot, not a complete threading demo.
The free block remains on a plain solid table. Left fingers approach and close from
the side. The side bolt remains on its separate rest; the right hand has not picked
it up and the M8 threads have not engaged.

The saved 100 ms load window measures 0.989528 N mean upward table force for
0.998797 N block weight, with only 0.009269 N mean positive upward hand force
and 100% loaded table substep duty. Original solved forces are recorded before
integration; the displayed saved qpos/qvel are immediately postintegration.
Replay uses mj_forward only, with no integration, interpolation, or manual edits
to either free part.

- `demo.png`: exact final saved native state at 1.65 s.
- `demo.gif` and `demo.mp4`: actual pilot at normal 1× playback.
- `trace.npz`: byte-exact original recording.
- `validation_original.json.txt`: byte-exact original report; it contains two Infinity values for unobserved right-transport pad minima.
- `validation.json` and `validation_serialization.json`: strict report with just those unobserved minima represented as null, plus both hashes and exact conversion paths.
- `progress_identity.json`: endpoint, original force sample, model/source/runtime IDs.
- `table_support_force_history.npz`: original native per-substep table/hand force ledger.
- `render_manifest.json` and `renderer_sources/`: rendering/state identities and sources.
- `SHA256SUMS`: every primary and supporting file.

The original validation remains `passed: false`, `partial: true` because this
selected pilot does not attempt later right pickup, capture, reset, or qualified
turns. No completed assembly, hardware calibration, or training readiness is claimed.

The first carried-block demonstration stays preserved at `media/m8_table_pickup/full`.
