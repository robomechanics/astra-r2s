This is an incomplete progress prefix from the fresh table-pickup rollout, not a completed threading demonstration.

It shows the left arm picking up and rolling the free block, the right arm picking up the separately supported bolt, and the first cone-start attempt followed by opening, reset and regrasp. The prefix ends at 14.18785 s before the second starting stroke. Cone contact and this opening/reset sequence do not establish formed-thread support, thread capture or qualified lead.

`demo.mp4` and `demo.gif` replay native recorded states. No physics is integrated, interpolated or posed for these renders. `demo.png` is the actual recorded state at the nearest saved sample after 12.3 s. The original trace metadata's `partial` field reflects requested phase selection rather than completion; `progress_manifest.json` explicitly marks this artifact as incomplete.

`insertion_trace.npz` is a byte-exact, closed snapshot of the running phase-prefix archive. Exact archived scene/controller/observer sources and imported controller helper sources accompany it. Robot mesh assets remain in this repository and their hashes are listed in `model_dependencies.json`. Replay with the command in `progress_manifest.json` using the pinned native runtime selected by `scripts/run_m8.sh`.

This progress package is not an all-substep force audit, a completed acceptance report, a formed-thread support proof, or a training-ready certification. No private input video is included.
