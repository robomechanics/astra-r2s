This is an incomplete progress prefix of the faster native M8 table-pickup rollout, shown at normal 1× playback speed.

Both arms start open with the block and bolt separately supported on the table. The video shows the left arm picking up and rolling the free block, the right arm picking up the free bolt, settled cone/lead-in contact, the first starting stroke at 1 rad/s, and physical release/opening/reset/regrasp recovery. The prefix ends at 19.0268 s after the first regrasp and before the second starting stroke.

Capture remains false throughout this prefix. The final potential formed-flank overlap is about 0.125 mm, below the 1.25 mm pitch threshold. This is a contact-driven starting attempt, not a completed capture or qualified-thread lead demonstration. The original native metadata's `partial` flag records requested phase selection rather than completion; this artifact explicitly remains incomplete.

`demo.mp4` is H264/yuv420p at 12 fps with normal 1× playback. `demo.gif` is an 800 px, 6 fps GitHub preview. `demo.png` shows the saved first-release/recovery state around 17.10 s. All renders select exact native recorded qpos/qvel states and use `mj_forward` for display only, without integration, interpolation, pose editing or a grasp weld.

The closed immutable prefix trace, exact XML, controller/scene/observer and imported controller helper sources are included. `model_dependencies.json` binds repository robot-mesh assets and runtime identity. `render_manifest.json` binds the completed media to this exact trace; all JSON uses strict finite serialization and `checksums.json` lists the final file bytes.

No formed-thread support, capture, qualified pitch, full acceptance or training-ready certification is claimed. No private source video is included.
