# Frozen alternate-head-grip software proof

All 281 software tests passed with unchanged hashes for the 63 recorded source/test files. This proof covers the supported-only 120-degree hex-head grip, native bolt load and measured seat-direction observers, and auditor coverage for future closed weight-transfer/reverse phases. Historical 213/210 and first-demo 181/176 proofs remain unchanged.

The current supported controller has executed a separate pickup-to-entry prefix. Its reverse seating and later full turn/reset schedule are still under development. This software proof does not qualify thread starting, passive capture, numerical convergence, hardware contact calibration or a complete table-supported trajectory.

`software_proof.json` binds every tested source, the launcher command and wrapper. `pytest_all.log` is the original test output; `verification_source.py` is the exact before/after source verifier. Verify this package from its directory using `sha256sum -c SHA256SUMS`.
