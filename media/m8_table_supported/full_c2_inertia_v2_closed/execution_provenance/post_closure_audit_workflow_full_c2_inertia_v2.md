Post-closure workflow for the fresh supported C2/inertia trial

This is prepared while the native run is LIVE. Execute serially only after `outputs/m8_table_supported/full_c2_inertia_v2/run_publication_identity_after.json` exists and root confirms the native child is closed. Do not inspect partial state to infer a completed outcome, rerun integration, alter original files, or stitch cold branches.

The original producer is `da69a9cd44a8312cc7b97365faf5e09c27a646e2`. BEFORE identity is `f98c6ba218d1c5d8503ac3d362a4a501de4fd99a8f1d87e4ee2e11a704f4c9c3`; completed software proof is `71e6d7b7a4688ae8a6e4588424f931b64f824e9552d3d792278ebe472b767be9` (672 passing tests, 74 source/test files). Core is the recorded `58039d439c6504448aafd0a4d5b655c8a0d3bf1d77123ac7e0733cd078367433`. The official supported auditor is frozen `4a8b018a1326bce540a0351d5d715ab58ffb505d2ae44be6d34aa8d8cd68fe3f`; all five frozen audit sources, proof/child log and 74 current/archive bytes must match. Documentation/media commits may advance HEAD while these bytes stay fixed. Historical `e468`/`abb4` compatibility readers and 402/65 proofs do not apply to this producer.

1. Run the standard-library identity check first. It refuses a missing closure, binds exact original proof/source/runtime/log bytes, preserves the integer native exit code, and never interprets identity or exit zero as physics success. It reads no trace or force states. Keep derivative outputs outside the original run; the verifier refuses overwriting its output.

```bash
cd /workspace/astra-r2s
mkdir -p outputs/m8_table_supported/full_c2_inertia_v2_audits
python outputs/m8_table_supported/verify_closed_supported_run_identity_v2.py outputs/m8_table_supported/full_c2_inertia_v2 --repository-root /workspace/astra-r2s --proof-archive media/m8_table_supported/software_proof_c2_inertia_v2 --expected-producer da69a9cd44a8312cc7b97365faf5e09c27a646e2 --expected-before-sha256 f98c6ba218d1c5d8503ac3d362a4a501de4fd99a8f1d87e4ee2e11a704f4c9c3 --expected-proof-sha256 71e6d7b7a4688ae8a6e4588424f931b64f824e9552d3d792278ebe472b767be9 --output outputs/m8_table_supported/full_c2_inertia_v2_audits/source_identity_before.json
```

2. Preserve the original AFTER identity, native log, exit code, report, all raw ledgers, full/partial trace, model/source archives and manifest byte-for-byte. If final report/trace/ledgers are absent, record their absence and publish only an explicitly incomplete raw-evidence bundle; do not synthesize them or silently relabel a partial trace. The five frozen auditor files are provenance snapshots, not a complete executable application package. Use the complete source-matched checkout.

3. When a final `insertion_trace.npz` exists, run the official auditors one at a time through the recorded core. Capture each command, actual exit/stdout/stderr and artifact SHA in the new audit directory. If an auditor raises, retain its exception; do not patch a source-bound reader or original report to improve the result.

```bash
scripts/run_m8.sh scripts/audit_m8_supported_trace.py outputs/m8_table_supported/full_c2_inertia_v2/insertion_trace.npz --output outputs/m8_table_supported/full_c2_inertia_v2_audits/independent_supported_audit.json
scripts/run_m8.sh scripts/audit_m8_left_pad_force_history.py outputs/m8_table_supported/full_c2_inertia_v2/insertion_trace.npz --output outputs/m8_table_supported/full_c2_inertia_v2_audits/independent_left_pad_force_history_audit.json
scripts/run_m8.sh scripts/audit_m8_free_joint_properties.py outputs/m8_table_supported/full_c2_inertia_v2/insertion_trace.npz --output outputs/m8_table_supported/full_c2_inertia_v2_audits/independent_free_joint_properties.json
```

The primary auditor checks original all-step table/feedback/inertia ledgers, source-faithful Crest/Entry/Table observers, independent reverse/first-forward quintics and C2 continuation/bounds, actual event rows and fresh support windows. Saved post-state geometry is separate; no auditor integrates dynamics or reconstructs original solved forces from saved poses. Coordinate additional independent raw force/rest-clearance arithmetic with the physics reviewer after these serial checks; freeze any new supplemental reader/proof separately.

4. Review the evidence rather than CLI completion: original force geometry is at `t−dt`; command scheduling is at `t−dt`, retained M/J/Jdot/calibration/qdot is normally `t−2dt` (initialized first-command exception), and saved qpos/qvel is at `t`. Verify retained qdot row `i≥2` equals original post-qdot row `i−2`. FF is disabled with explicit zero absent-input fields during pickup/transport/OPEN/regrasp. Check total recorded PD+FF under unchanged 8 N/2 Nm caps, then native bias/drag once and motor clipping; recorded FF arithmetic does not independently reconstruct every-tick PD pose error or certify contact dynamics/acceleration tracking.

Initial `settle_bolt` acquisition guards its solved acquisition row; a quiet `settle_regrip_*` acquisition guards the following tick and all subsequent closed ticks. OPEN requires actual bolt quiet and zero WHOLE-right-robot contacts; the detached hand may move. C2 braking cannot accumulate stopped direction readiness. A ≥50 µm actual crest return requests CLOSED deceleration only; the current 100 ms impulse/duty/loaded endpoint plus 90/10 bolt-weight support and both-body/tool quiet must confirm direction before forward scanning. These predicates establish neither full-pitch capture nor a passive open reset.

Keep ≥90% table-weight/≤10% positive left uplift/≥99% duty/current endpoint requirements, strict all-step bilateral pad gaps, zero object drive/passive artificial terms, dynamic unintended collisions, tip/table and rest clearance, loaded full interior contacts, qualified original yaw/depth lead and captured passive resets distinct. Preserve every failed or unexecuted gate; nominal formed overlap, a shallow supporting cone or a rendered image is insufficient for full-task qualification.

5. Repeat the same identity command with `--output outputs/m8_table_supported/full_c2_inertia_v2_audits/source_identity_after.json` after audits. Freeze original and derivative hashes/logs, then preserve every original byte in a separate publication packet. Any large files must use lossless ≤45,000,000-byte chunks with whole/chunk SHA and verified original-layout restoration. Actual-state media and software proofs remain separately labeled; do not splice progress footage or cold diagnostics into the fresh trajectory.
