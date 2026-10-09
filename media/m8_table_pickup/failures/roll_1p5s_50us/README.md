# Failed table block pickup trial: roll_1p5s_50us

This is the exact recorded bounded trial (`passed=false`, `partial=true`), including the failed left pad-load retention check. It contains physical table acquisition and roll, with no full bolt pickup/thread qualification. The strict packager verified the original trace, report, XML/assets and archived function-source identities.

During the roll, 1 saved sample(s) have one left pad normal load at zero; the other pad remains loaded. The failed check remains unchanged. `failure_notes.json` lists the actual times and preserves the initial table-support count (0). The first saved pose has native table collision candidates; that sampled geometry observation does not change the recorded time-zero support check.

`manifest.json` covers the strict primary package. `supplemental_manifest.json` separately binds this note, the free-workpiece audit and its exact source. `SHA256SUMS` covers both sets. Original numerical outcomes are retained in `validation.json`; compact capture audit details can be regenerated with:

```sh
scripts/run_m8.sh media/m8_table_pickup/failures/roll_1p5s_50us/audit_sources/audit_m8_insertion_capture.py media/m8_table_pickup/failures/roll_1p5s_50us/trace.npz --output /tmp/roll_1p5s_50us_capture_audit.json
scripts/run_m8.sh media/m8_table_pickup/failures/roll_1p5s_50us/audit_sources/audit_m8_free_joint_properties.py media/m8_table_pickup/failures/roll_1p5s_50us/trace.npz --output /tmp/roll_1p5s_50us_free_joint_audit.json
```

The original v1 run did not archive imported helper modules at start. The strict packager verified their exact function blocks against the originally declared controller SHA and archived the matching current helper bytes; it did not claim whole-module start-time capture.

Saved-pose collision replay performs zero integration or force reconstruction. It does not override the original load failure or qualify a policy.
