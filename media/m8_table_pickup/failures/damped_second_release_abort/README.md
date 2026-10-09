# Damped tabletop attempt: failed second release

**FAILED / ABORTED.** The unchanged 10 µm depth guard stopped this fresh continuous attempt at **17.32385 s** during `release_search_3`, when the native SDF depth proxy reached **10.129 µm**. The trial never acquired formed-flank capture or qualifying screw turns. `partial=false` describes the requested full-task scope; it does not imply completion.

[Video](demo.mp4) · [GIF](demo.gif) · [Screenshot](demo.png) · [Measured trajectory](trajectory.png)

Both pickups are physical: the left arm grasps the table-supported free block, lifts it by **59.965 mm** and reorients it; the right arm separately picks up the bolt. After their respective lifts both parts remain unsupported by the world. Moving the side bolt to y=-0.22 m eliminates the prior camera interference in every saved native geometry sample. This trial has **zero unexpected sampled penetrating contacts**, **zero nonpad left-jaw/block candidates**, and maximum sampled native pad depth **10.315 µm**.

The native arm feed uses finite axial velocity damping **50 N s/m**. Every one of the **1,308** saved axial-float commands matches `F=-50*v`; the independent command audit explains its pre-integration timing and scope. The controller does not impose axial depth or pitch motion.

One complete open cone-start reset executed with zero hand/world/head-seating contact in the original all-substep phase report and zero unexpected workpiece candidates in all **296** saved poses. Its maximum recorded axial/yaw drift was **0.113 µm / 0.0386 mrad**. The formed-flank overlap remained zero and capture was false throughout: this is lead-in contact stability, not formed-thread self-lock qualification.

The raw left pad archive covers **316,478** post-acquisition physics steps at **50 µs** each. Strict continuous bilateral preload still fails with **9 / 6** isolated one-step gaps, despite the sparse 5 ms force check passing.

`manifest.json` preserves the exact failed trace/validation, archived XML/controller/helper/observer/scene sources, raw force history, independent capture audit and closed media/chart. `source_identity.json` explicitly separates the controller's **selected definition bundle SHA** from its **whole module SHA**. The supplemental manifest binds exact extra audit/render/tooling sources, independent reset/free-joint/raw-force/damping reports, the published progress screenshot crosslink, and the original **167-pass** unchanged-source software proof. Software tests do not qualify this failed physical rollout.

```bash
sha256sum -c media/m8_table_pickup/failures/damped_second_release_abort/SHA256SUMS
bash scripts/run_m8.sh media/m8_table_pickup/failures/damped_second_release_abort/audit_sources/audit_m8_insertion_capture.py media/m8_table_pickup/failures/damped_second_release_abort/trace.npz --output /tmp/damped_capture_audit.json
```

Replay requires repository mesh assets and the verified micron MuJoCo runtime/plugin. Saved geometry audits do not integrate physics or reconstruct original forces. Numerical pad compliance and idealized thread geometry do not constitute hardware calibration.
