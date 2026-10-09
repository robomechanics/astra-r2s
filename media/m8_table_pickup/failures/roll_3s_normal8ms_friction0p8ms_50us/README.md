# Failed block-pickup diagnostic: 8ms normal / 0.8ms tangential

This exact bounded trial physically acquires the initially table-supported block with an open left hand, lifts it, and rolls it to the horizontal holding pose. The male bolt remains separately supported on its side rest. Its original outcome is `passed=false`, `partial=true`: full bolt pickup and threading were not attempted.

The new every-physics-step bilateral-load gate fails despite the older 5ms sampled load check passing. The raw 81,001 post-acquisition steps at 50µs contain 26 left-pad and 16 right-pad unilateral load gaps. Each gap lasts one step (50µs); total durations are 1.30ms and 0.80ms. Both pad minima are zero. The pads are never simultaneously unloaded, and the opposite pad has at least 4.816N during a gap. These diagnostics do not negate the strict recorded failure or independently prove wrench balance.

`manifest.json` preserves the strict primary evidence package and binds `trace.npz`, `validation.json`, `left_pad_force_history.npz`, exact scene/controller/observer/helper sources, and compact `independent_capture_audit.json` with its exact auditor sources. `supplemental_manifest.json` separately binds the free-workpiece audit, its source and these notes. `SHA256SUMS` covers the evidence files. Root adds progress visuals through a separate visual manifest; they are a failed pickup diagnostic, not a qualified threading demo.

Regenerate the read-only diagnostics with:

```sh
scripts/run_m8.sh media/m8_table_pickup/failures/roll_3s_normal8ms_friction0p8ms_50us/audit_sources/audit_m8_insertion_capture.py media/m8_table_pickup/failures/roll_3s_normal8ms_friction0p8ms_50us/trace.npz --output /tmp/table_pickup_v8_capture.json
scripts/run_m8.sh media/m8_table_pickup/failures/roll_3s_normal8ms_friction0p8ms_50us/audit_sources/audit_m8_left_pad_force_history.py media/m8_table_pickup/failures/roll_3s_normal8ms_friction0p8ms_50us/trace.npz --output /tmp/table_pickup_v8_force_history.json
scripts/run_m8.sh media/m8_table_pickup/failures/roll_3s_normal8ms_friction0p8ms_50us/audit_sources/audit_m8_free_joint_properties.py media/m8_table_pickup/failures/roll_3s_normal8ms_friction0p8ms_50us/trace.npz --output /tmp/table_pickup_v8_free_joints.json
```

Replayed qpos is post-integration. Original contact forces and contact geometry belong to the force-evaluation state at recorded time minus one timestep. Saved-pose collision replay does not reconstruct those solved forces.
