# Native table-supported M8 trial

The original recorded sequence did not complete; its original overall validation failed.
Read `validation.json` and `independent_supported_audit.json` for every result
and its scope. Video and stills replay exact native qpos/qvel with the state-refresh method
bound in `render_manifest.json`. They do not integrate, interpolate or manually
pose free workpieces. Force captions use the original solved samples at saved
time minus one native timestep; replay forces are not original evidence.
Playback is normal 1×.

Original CLOSED failed whole fresh native attempt. Pickup/align/entry/weight transfer/reverse search/stopped endpoint executed. Direction gate failed measured19.1214um<50um; formed0/interior0 and no capture/reset/full completion. Canonical fixed-camera saved-state replay uses ONLY mj_kinematics/comPos/camlight; no mj_forward/collision/force solve/integration/interpolation/object posing/splice. Original failure/acceptance flags retained.

The free female block rests on the plain solid table, with left fingers
stabilizing its sides. The separate male bolt starts on its physical side rest.
The table remains solid under the bore; the recorded tip/table clearance guard
limits this to running-thread travel, without a full head-seating claim.

The exact original producer software proof contains **402 tests**
and **65 source hashes**; it remains separate from native
physical acceptance and earlier historical proofs. Every original closed run
artifact is retained, including the complete `native_feedback_force_history.npz`
and its own metadata, control/event declarations, executed finite motor wrenches
and torques, open-contact histories, per-grasp masks, original loaded-interior
observations, all native state arrays, and original before/after provenance.
No feedback column, row, dtype or binary archive byte is dropped or re-encoded.
Extra sibling execution manifests/logs supplied to publication are preserved
under `execution_provenance/` with original identities. The package manifest
lists every original feedback array name and whole-file SHA. Reassembly restores
the exact whole NPZ and standard filenames before audits or replay.

To restore the EXACT ORIGINAL native-run names and directories, use the verified
original-layout helper rather than renaming only the trace:

```sh
python media/m8_table_supported/full_canonical_v1_failed_evidence/restore_original_run_layout.py --verify-only
python media/m8_table_supported/full_canonical_v1_failed_evidence/restore_original_run_layout.py --output outputs/m8_table_supported/original_run_layout
```

It verifies stored/chunk/whole SHA first, then restores all original files from
the manifest's native filename/identity mapping, including `insertion_trace.npz`,
`insertion_validation.json`, `renderer_source.py` and `manifest.json`. It refuses
to overwrite changed destinations. Invoke the original closure verifier and
matched archived/native model auditors on this original layout in the complete
producer checkout. `frozen_audit_sources/` is provenance, not a standalone runtime.

The frozen original supported reader remains `e46808601d7974ffdea1e5e20e07e0f28eb8e0a74a394545b276621a7e856702`; its original exception is preserved. The independently generated report is explicitly an output-only compatibility derivative using `abb4e724de91eeec9dcff9c481d1a89674fba5ea3e3c5c2d9a9b76a074f9cf1e`, with the exact narrow source diff, boundary diagnosis, regression proof and sidecar under `auditor_compatibility/`. Only the initial same-row versus quiet-regrasp next-tick guard boundary is corrected. The original402/65 proof, native source/report and physical acceptance outcomes are unchanged.

Every original trace and force-ledger byte is preserved. Files larger than
45,000,000 bytes are split into consecutive fixed-size binary chunks, with
whole-file size/SHA256 and each chunk offset/size/SHA256 in `package_manifest.json`.
No rows are dropped and no state, force, or contact values are quantized. The
last chunk may be smaller. GitHub may not preview NPZ/chunk files.

## Verify and reassemble

From the repository checkout, use standard Python; no simulator is needed:

```sh
python media/m8_table_supported/full_canonical_v1_failed_evidence/reassemble_archives.py --verify-only
python media/m8_table_supported/full_canonical_v1_failed_evidence/reassemble_archives.py --output outputs/m8_table_supported/reassembled
```

The helper verifies every stored chunk and reconstructed whole file, restores
all artifacts to their original filenames and relative archive directories,
and refuses to overwrite differing files. An existing byte-identical file is
accepted only after the source chunks are verified again.

## Replay and inspect

After preparing the verified native MuJoCo runtime according to
`docs/m8_setup.md`, inspect the archived trajectory independently. This uses the
matched native GCC CPU engine and bindings with the pinned source/engine patch;
the stock MuJoCo wheel is insufficient. Runtime core/plugin SHA identities are
preserved. A rebuild on another machine must be compared before claiming
byte-identical native libraries:

```sh
scripts/run_m8.sh scripts/audit_m8_supported_trace.py outputs/m8_table_supported/reassembled/trace.npz --output outputs/m8_table_supported/repeated_audit.json
scripts/run_m8.sh -m yam_twin.m8_supported_demo --replay outputs/m8_table_supported/reassembled/trace.npz --output outputs/m8_table_supported/replayed --video --fps 12 --slow-motion 1
```

Those generic commands inspect/replay recorded states; they do not regenerate
a controller rollout. The generic supported_demo replay uses mj_forward and
can recompute unused replay forces; it is a NEW replay, not reproduction of the
geometry-only publication render. For actual-render reproduction, follow
REPLAY_ACTUAL_MEDIA.md and its archived renderer source when supplied. Exact production sources are retained in
`controller_source.py`, `scene_source.py`, `recorded_sources/`, and
`frozen_audit_sources/`. `run_publication_identity.json` records the producer
commit. Software proof and its unchanged-source manifest remain separate from
physical qualification. Supplemental audit revisions, when present, retain their
own source hashes and revision record rather than relabeling the original audit.
See `docs/m8_table_supported.md` for fresh rollout
commands, input configuration, and reconstruction limits.

The first carried-block demonstration remains unchanged at
`media/m8_table_pickup/full`. This package does not certify calibrated materials,
full seating/preload, broad numerical convergence, or learned-policy robustness.
