# Failed equivalent convex-mesh pad pilot

This exact bounded table-block acquisition trial uses the same pad box envelopes as the native-box scene, represented by convex meshes. Mass, pose, friction, zero margins and original 0.8ms contact references remain declared in the archived scene. Its original outcome is `passed=false`, `partial=true`; both the sparse sampled and every-physics-step left-pad load checks fail. Full bolt pickup and threading were not attempted.

The raw history has 81,001 post-acquisition steps at 50µs, pad minima [0.0, 0.0]N, and unloaded counts [153, 95]. Longest gaps last one step. Exact per-phase counts and durations are preserved in the compact capture audit and can be independently reaggregated from the raw force history.

`manifest.json` binds the strict primary evidence and exact sources. `supplemental_manifest.json` binds the passive-properties audit and compact notes. `SHA256SUMS` covers both sets.

```sh
scripts/run_m8.sh media/m8_table_pickup/failures/roll_3s_convex_mesh_50us_allsteps/audit_sources/audit_m8_left_pad_force_history.py media/m8_table_pickup/failures/roll_3s_convex_mesh_50us_allsteps/trace.npz --output /tmp/table_pickup_convex_forces.json
scripts/run_m8.sh media/m8_table_pickup/failures/roll_3s_convex_mesh_50us_allsteps/audit_sources/audit_m8_insertion_capture.py media/m8_table_pickup/failures/roll_3s_convex_mesh_50us_allsteps/trace.npz --output /tmp/table_pickup_convex_capture.json
```

Original solved forces belong to the force-evaluation state at recorded time minus one timestep; qpos/qvel are post-integration. Diagnostic aggregation and saved-pose replay perform no integration or independent force reconstruction.
