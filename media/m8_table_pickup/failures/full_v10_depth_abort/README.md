# Failed full tabletop pickup attempt

**FAILED / ABORTED.** The native thread depth proxy reached **10.562 µm** at **12.15315 s** during `release_search_2`, exceeding the unchanged **10 µm** guard. Formed-flank capture and qualifying turns were never reached. `partial=false` records the requested full-task scope; this attempt stopped before completing it.

[Video](demo.mp4) · [GIF](demo.gif) · [Screenshot](demo.png) · [Measured trajectory](trajectory.png)

The left arm actually approached, grasped and lifted the table-supported free block by **59.965 mm**, then physically reoriented it. The right arm separately picked up the free bolt. After their respective lifts both objects had zero world-support candidates. The saved left block grasp had zero nonpad jaw/block candidates and at most **10.315 µm** native signed pad depth.

Independent saved-pose replay also found **120** native penetrating candidates between `left_rf_down5_collision` and the right D405 camera during right bolt pickup, peaking at **36.610 µm**. These recorded collisions remain explicit failed-attempt evidence.

Every-substep raw left pad loads cover **213,064** steps after acquisition at **50 µs** each. The strict bilateral preload check fails with **9 / 6** isolated one-step gaps, although the sparse 5 ms force check passes. No reset phase was executed, so the empty reset report supplies no reset qualification.

`manifest.json` binds the exact final trace, failed validation, scene, controller/helper/observer sources, raw force history, compact independent capture audit, and closed media/chart. `supplemental_manifest.json` binds the independent reset/free-joint/raw-force audits, failure details, exact extra tooling sources, and the 158-pass software-test snapshot. The software proof does not qualify this physical trial.

```bash
sha256sum -c media/m8_table_pickup/failures/full_v10_depth_abort/SHA256SUMS
bash scripts/run_m8.sh media/m8_table_pickup/failures/full_v10_depth_abort/audit_sources/audit_m8_insertion_capture.py media/m8_table_pickup/failures/full_v10_depth_abort/trace.npz --output /tmp/full_v10_capture_audit.json
```

Native replay requires the repository meshes and the verified micron MuJoCo runtime/plugin. Auditing evaluates saved geometry and original force archives without integrating physics. Native normal compliance is a declared numerical contact assumption, not measured rubber calibration.
