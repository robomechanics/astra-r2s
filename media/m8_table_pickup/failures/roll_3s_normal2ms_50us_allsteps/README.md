# Failed sampled-load / every-step-load comparison

This exact bounded table-block acquisition trial preserves `passed=false`, `partial=true`. The older sparse sampled left-pad load check passes; the stricter every-physics-step check fails. Full bolt pickup and threading were not attempted.

The raw history has 81,001 post-acquisition physics steps at 50µs. Pad minima are [0.0, 0.0]N, with unloaded step counts [77, 46]. The longest gaps last one step. The exact counts, durations and per-phase scope can be independently reaggregated from `left_pad_force_history.npz`; they cannot be inferred from the sparse saved poses. The failed outcome remains unchanged.

`manifest.json` binds the primary evidence and exact archived sources. `supplemental_manifest.json` binds the passive-properties audit and these compact notes. `SHA256SUMS` covers both evidence sets.

```sh
scripts/run_m8.sh media/m8_table_pickup/failures/roll_3s_normal2ms_50us_allsteps/audit_sources/audit_m8_left_pad_force_history.py media/m8_table_pickup/failures/roll_3s_normal2ms_50us_allsteps/trace.npz --output /tmp/roll_3s_normal2ms_50us_allsteps_forces.json
scripts/run_m8.sh media/m8_table_pickup/failures/roll_3s_normal2ms_50us_allsteps/audit_sources/audit_m8_insertion_capture.py media/m8_table_pickup/failures/roll_3s_normal2ms_50us_allsteps/trace.npz --output /tmp/roll_3s_normal2ms_50us_allsteps_capture.json
```

Original native solved forces belong to the force-evaluation state at recorded time minus one timestep; qpos/qvel are post-integration. Reaggregation checks diagnostic integrity and coverage, with no integration or independent force reconstruction.
