# Failed direct normal-reference pilot

This exact bounded tabletop block acquisition trial uses native normal acceleration-reference coefficients `solref=-31250 -2500`, with original box pads, impedance, friction coefficient, zero margins and tangential reference of 0.8 ms. Those coefficients are not physical N/m or Ns/m material constants. This isolated numerical compliance choice is not rubber calibration.

The original result remains `passed=false`, `partial=true`. The raw history has 81,001 post-acquisition steps at 50 µs and show unloaded counts [9,6], pad minima [0,0]N, and longest gaps of 50 µs. Full bolt pickup/capture/threading were not attempted.

Saved-pose collision replay finds maximum pad/block signed penetration of 10.315 µm, versus nominal pad inner offset of 500 µm, and zero nonpad left-hand/block candidates. This is sampled geometry only; it does not certify every-substep backing clearance or material deformation.

`manifest.json` binds the primary raw records and exact sources. `supplemental_manifest.json` binds the free-workpiece audit and compact notes; `SHA256SUMS` covers both evidence sets.

```sh
scripts/run_m8.sh media/m8_table_pickup/failures/roll_3s_direct_normal_50us_allsteps/audit_sources/audit_m8_left_pad_force_history.py media/m8_table_pickup/failures/roll_3s_direct_normal_50us_allsteps/trace.npz --output /tmp/table_pickup_direct_normal_forces.json
scripts/run_m8.sh media/m8_table_pickup/failures/roll_3s_direct_normal_50us_allsteps/audit_sources/audit_m8_insertion_capture.py media/m8_table_pickup/failures/roll_3s_direct_normal_50us_allsteps/trace.npz --output /tmp/table_pickup_direct_normal_capture.json
```

Reaggregation and replay perform no integration or force reconstruction. Original solved force/contact data are evaluated at recorded time minus one timestep; saved qpos/qvel are post-integration.
