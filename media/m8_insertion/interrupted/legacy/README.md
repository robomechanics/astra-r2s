This trial was interrupted when the environment runtime restarted. Its original process did not finish and produced no final validation. It is not accepted end-to-end qualification evidence and must not be spliced into another rollout.

The last persisted phase boundary is `settle_regrip_1` at 29.50925 s. Candidate capture at that boundary is `true`. Candidate capture alone does not establish completed engagement: the required qualifying turns and final acceptance report are absent.

`insertion_trace_partial.npz`, `interruption.json`, `controller_source.py`, and `scene.xml` are copied byte for byte from the interrupted run. The window-v1 folder also preserves its exact `engagement_observer_source.py`. The original partial filename is retained; no final trace or validation has been fabricated. `manifest.json` and `SHA256SUMS` record hashes.

`independent_audit_compact.json` retains the independent source/runtime identity checks, phase summaries, and scope. Its provenance records the original full-audit hash and omitted row counts. The saved-sample audit performs no integration or force solve, so it cannot reconstruct between-sample forces or replace the missing final acceptance report.

To regenerate the full independent audit from the repository root with the matched M8 environment:

```bash
./scripts/run_m8.sh scripts/audit_m8_insertion_trace.py media/m8_insertion/interrupted/legacy/insertion_trace_partial.npz --output /tmp/astra-m8-interrupted-legacy-audit.json
```
