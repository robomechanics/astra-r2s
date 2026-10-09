# Independent audit of table-supported stabilization progress

This is a supplemental audit of the immutable `../progress_stabilized` package.
Its original media, trace, source archive and checksum ledger have not changed.

The audit validates native table weight sharing and the archived scene,
controller, finite actuation, free workpiece properties, enabled table masks and
saved geometry of the 1.65 s four-phase left-stabilization pilot. Only one
active substep follows verified left acquisition. Whole-task checks apply only
to this selected pilot; they do not establish a longer supported trajectory.

The right arm has not picked up the bolt, and no thread engagement, qualified
turn or unsupported gripper reset was attempted. Original and independent
overall validation remain false.

- `independent_supported_audit.json`: exact closed independent audit.
- `audit_m8_supported_trace.py`: exact auditor source, verified against its reported hash.
- `audit_sources/`: exact common archived-model loader and runtime identity sources.
- `audit_manifest.json`: parent trace/report/source hashes and explicit limited scope.
- `relocated_package_audit.json`: independently repeated audit of the published media archive; all check results match the original run-directory audit.
- `SHA256SUMS`: all supplemental files.

To repeat from the repository using the verified native runtime:

```sh
scripts/run_m8.sh scripts/audit_m8_supported_trace.py media/m8_table_supported/progress_stabilized/trace.npz --output outputs/m8_table_supported/repeated_stabilization_audit.json
```

The command audits the preserved pilot archive. It does not rerun its controller
or qualify the newer controller/source revision. Check the auditor CLI help if
using a different revision.
