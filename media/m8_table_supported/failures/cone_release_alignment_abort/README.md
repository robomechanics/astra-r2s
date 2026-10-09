# Native table-supported M8 trial

The original recorded sequence did not complete; its original overall validation failed.
Read `validation.json` and `independent_supported_audit.json` for every result
and its scope. Video and stills replay exact native qpos/qvel with `mj_forward`
only. They do not integrate, interpolate, manually pose free workpieces, or
reconstruct original solved forces. Playback is normal 1×.

Fresh unspliced full_v2 native attempt: actual side-bolt pickup, full-shaft rest clearance, high transport, alignment, settled entry and first starting stroke. It aborted at 12.84275 s in release_search_2 when native radial offset 150.129559 µm exceeded the unchanged 150 µm guard. The actual first stroke rotates 3.141634 rad but advances only 8.945825 µm; formed overlap remains zero. No formed capture, qualified turns, unsupported formed-thread reset, complete assembly or full-task pass is claimed. The original failed report, source-bound 213-test proof and all raw force/state bytes remain authoritative. Normal 1× exact saved-state replay; no integration, interpolation or free-body posing.

The free female block rests on the plain solid table, with left fingers
stabilizing its sides. The separate male bolt starts on its physical side rest.
The table remains solid under the bore; the recorded tip/table clearance guard
limits this to running-thread travel, without a full head-seating claim.

Every original trace and force-ledger byte is preserved. Files larger than
45,000,000 bytes are split into consecutive fixed-size binary chunks, with
whole-file size/SHA256 and each chunk offset/size/SHA256 in `package_manifest.json`.
No rows are dropped and no state, force, or contact values are quantized. The
last chunk may be smaller. GitHub may not preview NPZ/chunk files.

## Verify and reassemble

From the repository checkout, use standard Python; no simulator is needed:

```sh
python media/m8_table_supported/failures/cone_release_alignment_abort/reassemble_archives.py --verify-only
python media/m8_table_supported/failures/cone_release_alignment_abort/reassemble_archives.py --output outputs/m8_table_supported/reassembled
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

Those commands inspect/replay recorded states; they do not regenerate a
controller rollout. Exact production sources are retained in
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
