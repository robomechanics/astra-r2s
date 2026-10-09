# Rejected assumed-compliance pickup pilot

This exact bounded trial preserves `passed=false`, `partial=true`. Native normal acceleration-reference coefficients `solref=-51 -14.2828568570857` were an illustrative compliance assumption, with original box pads and tangential reference of 0.8 ms. They are not hardware rubber calibration. Full bolt pickup and threading were not attempted.

The every-step bilateral-load check fails: unloaded counts [9, 11] over 81,001 post-acquisition steps at 50 µs, with one-step longest gaps and both pad minima of 0 N. Independent saved-pose collision replay also finds 1,720 unexpected penetrating native jaw/block candidates (up to 12 at a saved pose), reaching 18.46 µm penetration. Exact geom names, counts and first occurrences appear in `failure_notes.json`. This backing engagement is a separate geometry failure; it must not be classified as pad-only gripping.

Maximum sampled pad/block contact depth is 318.44 µm. It is below the nominal 500 µm pad inner offset, but that scalar offset does not establish true clearance to every textured native finger component. The observed native contacts reject this assumed-compliance configuration for the full demo.

`manifest.json` binds the strict primary raw evidence and exact sources. `supplemental_manifest.json` binds passive-properties proof and compact failure notes; `SHA256SUMS` covers both sets.

```sh
scripts/run_m8.sh media/m8_table_pickup/failures/roll_3s_assumed_compliance_50us_allsteps/audit_sources/audit_m8_insertion_capture.py media/m8_table_pickup/failures/roll_3s_assumed_compliance_50us_allsteps/trace.npz --output /tmp/table_pickup_assumed_compliance_capture.json
scripts/run_m8.sh media/m8_table_pickup/failures/roll_3s_assumed_compliance_50us_allsteps/audit_sources/audit_m8_left_pad_force_history.py media/m8_table_pickup/failures/roll_3s_assumed_compliance_50us_allsteps/trace.npz --output /tmp/table_pickup_assumed_compliance_forces.json
```

Saved qpos is post-integration. Original solved forces/contact data belong to the force-evaluation state at recorded time minus one timestep. Collision replay is sampled geometry only; no force reconstruction or every-substep backing-clearance claim is made.
