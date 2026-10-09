This workflow is prepared while `full_reset_speed2_v3` is LIVE. Execute it only after the parent confirms session 95831 finished and the untouched original `run_publication_identity_after.json` exists. No original state or force data was decoded to prepare it.

The fresh producer is `9ae1a9fe76968a4013ea6c39e67718026622b9c0`, started at `2026-10-09T14:32:53.840320Z`. Its original BEFORE SHA is `1aceb2b906b6201ba715bde99dfe1394bb0880907b0f9914529a9c50adff1ec9`; its completed software proof SHA is `9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7` (672 tests, 74 source/test files). The only producer change from v2 exposes the real reset speed and selects 2 rad/s; the old v2 run, sources, audits and media remain separate.

The frozen generic identity verifier is `verify_closed_supported_run_identity_v2.py`, SHA `7ceed9d55285b4be3c804db9f5443efdb2d2786adcdd100b9faf114050400f3a`. It requires original BEFORE/AFTER/proof, current and archived 74 tested files, frozen five audit sources, native runtime/core, manifest and original child logs to agree. Documentation/media HEAD advancement is permitted only while these bytes remain unchanged. Missing AFTER is refused before any log, trace, runtime, source or proof read.

After confirmed closure, run the new wrapper from the source-bound checkout. Keep the target audit directory absent until this command creates it:

```bash
cd /workspace/astra-r2s
python outputs/m8_table_supported/audit_closed_reset_speed2_v3.py \
  --repository-root /workspace/astra-r2s \
  --run outputs/m8_table_supported/full_reset_speed2_v3 \
  --proof-archive media/m8_table_supported/software_proof_reset_speed2_v3 \
  --output outputs/m8_table_supported/full_reset_speed2_v3_audits \
  --confirmed-closed
```

Optionally add `--expected-after-sha256` with the observed immutable original AFTER SHA. A relocated replay library can be supplied via `--runtime-paths-json` using the original path keys; hashes must still match. Do not invoke the old v2 fixed-producer media gate for this run.

The wrapper runs the official supported, left-pad-history and free-joint-property readers serially against the complete original `insertion_trace.npz`, each through `scripts/run_m8.sh` with positional trajectory and `--output`. Their static parser shapes were checked without executing or importing a model. The official supported reader remains SHA `4a8b018a1326bce540a0351d5d715ab58ffb505d2ae44be6d34aa8d8cd68fe3f`; the other four frozen sources remain original producer/proof bound. Historical e468/abb4 compatibility is not applicable.

The reader execution chain is explicit: `/workspace/.venvs/m8-contact/bin/python`, `/workspace/research/m8-contact-sdk/bindings_verification.json`, and the core path/hash already checked against the original runtime. The wrapper sets the shell's `ASTRA_PYTHON`, `M8_BINDINGS_SDK` and `ASTRA_MUJOCO_LIB` values, removes inherited `PYTHONPATH`/`PYTHONHOME`, and records these effective settings. `run_m8.sh`, the actual Python executable bytes, ABI record and core are hash checked before/after readers; the shell and ABI record are copied into the derivative packet. These extra closed-time execution identities do not extend the original 74-file proof. `--auditor-python` and `--bindings-sdk` support an explicitly matched relocated environment; its ABI record must bind the original core SHA and the shell's existing verification must pass.

The launcher records identity before and after the sequence, original integer native exit of any value, original report `passed`/`partial` fields when present, exact trace/AFTER hashes, commands, timezone-aware start/end stamps, process exit codes, complete stdout/stderr bytes, output JSON SHA, and tested-module bytes before/after every reader. Reader exceptions or missing/malformed outputs remain explicit. Every existing original run file is hash checked unchanged across the sequence; source/trace/AFTER changes prevent later readers. `reader_execution_completed` means only that the three commands finished and wrote parseable outputs; CLI exit 0 never establishes native success or thread capture.

Keep original report, full and partial traces, all raw table/pad/feedback/inertia ledgers, model/assets, source archives, identities, manifest and native/software logs unchanged, including any failure. If complete trace/report evidence is absent, do not substitute a partial trace or synthesize an audit; use a separate explicit raw-only package listing missing artifacts. Preserve unexecuted readers and all exceptions. After the serial official readers, any supplemental original-ledger analysis and actual-state media must use separate source-bound derivatives and original force/state timing.

Acceptance still distinguishes actual crest return/direction stop, entry-only weight support, fully formed loaded thread contact, lead agreement, and genuine zero-right-contact open reset. Keep strict table 90/10/99 load, original pad continuity, finite motor/wrench caps, free-body/no-object-drive, native collision/rest/table guards, initial same-row versus regrasp next-tick reference semantics, bolt-only OPEN quiet, and C2 braking-not-stopped semantics. Do not splice the earlier cold diagnostic into this fresh run or infer capture from shallow formed geometry.

Publication is a later step. Preserve whole original bytes with lossless chunks under 45,000,000 bytes and exact original-layout restoration; archive all derivative reader/source/log identities. This preparation adds only ignored helpers and does not change any tested/native source, earlier proof, runtime, original run, or package.
