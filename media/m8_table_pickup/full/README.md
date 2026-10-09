# Full physical tabletop sequence: strict preload diagnostic failed

The left arm picks up the free block from the table and reorients it; the right arm picks up the separate M8 bolt, physically acquires starting contact, rotates it into formed-flank capture, and performs **two measured qualifying half-turns with real unsupported open resets and regrasp**. The complete 49-phase sequence finishes without an abort.

[Normal-speed video](demo.mp4) · [GIF](demo.gif) · [Final screenshot](demo.png) · [Measured trajectory](trajectory.png)

**Overall acceptance remains FAILED: 22/23 checks.** The sole failing check is continuous bilateral left preload: the original 842,723 post-acquisition physics steps contain **9 /6 isolated 50 µs unloaded steps**. Physical grip retention, unsupported object behavior, native joint limits, thread depth, metric lead, torque and reset checks pass. This is a completed physical sequence with a preserved numerical preload failure; it is not an all-checks-pass certification.

Each actual half-turn advances **0.622591 /0.622897 mm**, with measured rotation **3.141861 /3.141861 rad**. Residuals from the M8×1.25 lead are **−2.462 /−2.156 µm**, within the unchanged tolerance. Candidate capture is recorded at **31.3403 s**; the first 5 ms saved tagged row is 31.34495 s. The captured unsupported resets drift by at most **0.178 /0.424 µm** axially and **0.363 /0.377 mrad** angularly, with zero original hand/world/head-seating contact. Earlier search resets have partial starting flanks below one full pitch and remain unqualified.

Saved native replay finds zero unexpected penetrating contacts and zero nonpad left-jaw/block candidates; maximum reported pad depth is 10.315 µm. Both free objects have zero artificial passive terms or actuators. Only native robot joint motors and finger couplings are present.

`manifest.json` binds exact final states/failed report/XML/controller/helper/observer/scene sources, raw all-step left loads, independent capture audit and closed normal 1× media/chart. `supplemental_manifest.json` binds reset/free-joint/entry-command audits, exact extra sources, original 176-test proof, and published progress-to-final links. `source_identity.json` separates selected-definition and whole-module hashes and preserves the actual loaded renderer **fcf1cb1c…**, rather than subsequent CLI revisions. Current 181-test software proof remains separate.

All local files are bound in `SHA256SUMS`. Native geometry replay requires repository meshes and the verified micron MuJoCo runtime/plugin. Original force inputs are from the pre-integration state; saved qpos/qvel are post-integration. Audits add no dynamics steps or reconstructed-force claims. Numerical contact compliance and idealized thread geometry are not hardware calibration.
