# Fresh native bolt pickup and alignment progress

This exact saved state belongs to the fresh, continuing `full_v2` trial. At
4.82 s native physics, the right fingers have picked up the separate bolt,
transported its complete shaft clear of the original three-pin rest, and
aligned it above the M8 female bore. The left fingers stabilize the free block
on the plain solid table. The saved state still has no thread contact or
formed-thread capture. No qualified turn or completed trajectory is claimed.

`demo.png` replays one actual postintegration qpos/qvel row using native
`mj_forward` only, with no integration, interpolation or manual free-body edits.
Its force captions come from the original solved sample immediately before
integration, at saved state time minus timestep. Table/left force ledgers remain
open in the continuing parent trial; this screenshot does not certify whole-task
load retention. The archived 213-test producer proof is separate from physics
qualification and applies to the exact source revision recorded here.

- `trace.npz`: byte-exact, closed-readable parent phase-prefix snapshot.
- `prefix_identity.json`: source/model/controller IDs, endpoint, selected native state
  hashes and original solved force sample. Its original `partial` field describes
  requested phase selection; this copied prefix is incomplete regardless.
- `render_manifest.json`, `renderer_source.py` and `renderer_sources/`: exact replay
  sources, selected state and image hashes, and native runtime identity.
- `scene.xml`, `supported_scene.zip`, `scene_source.py`, `controller_source.py` and
  `recorded_sources/`: exact recorded native model/source dependency snapshots.
- `software_tests.json`, `software_tests.log`, `software_manifest.json` and
  `verify_sources.py`: original historical 213-test producer proof, unchanged.
- `frozen_audit_sources/`: auditor sources frozen for the parent trial; no complete
  independent physical audit is claimed or fabricated in this progress package.
- `SHA256SUMS`: all primary and supporting files.

The original native parent trajectory continues without restoring or splicing this
checkpoint. After it closes, this prefix can be compared against its final rows;
its prefix-file SHA remains distinct from the eventual whole final trace SHA.
The first carried-block demonstration and earlier supported pilot/failure archives
remain unchanged. Replaying requires the matched native CPU engine described in
`docs/m8_setup.md`; stock MuJoCo is insufficient for this M8 evidence.
