# Reset-speed2 software proof and prospective launcher

**672 software tests passed in104.74 s** (105.51644639499136 s outer wrapper) for the exact74 source/test files archived here. Every selected source and native runtime file remained unchanged through this run. This is a new original proof, separate from earlier672/402 tests and all native trajectory outcomes. The preserved pytest log includes the actual result and two existing Gym unbounded-observation warnings.

The only selected-file change from producer `da69a9cd44a8312cc7b97365faf5e09c27a646e2` is `yam_twin/m8_supported_demo.py`: expose `--reset-speed`, retain the previous default4 rad/s, and pass the selected value through the existing finite-positive control-config validation. The controller, geometry, finite force/motor caps, native runtime, thread contact physics and every physical guard remain byte-identical to that producer. This proof does not certify that a2 rad/s OPEN reset tracks safely or completes the supported trajectory.

The prospective native command explicitly selects `--reset-speed 2`. It retains the original fullXYZ frozen free-hand minus-pi path. The intended duration becomes2.945243112740431 s, versus1.4726215563702154 s at4 rad/s; scheduled velocity halves and scheduled acceleration quarters. Those are command-profile facts, not measured native tracking or collision-clearance results.

## Exact contents and identities

This packet contains80 files: the74 exact tested source/test files under `source_files/`, the new original software proof JSON, original pytest log, exact executed verification wrapper, reviewed versioned prospective launcher, this README and79-entry SHA256SUMS. The source copies are a proof archive, not a standalone application checkout: native mesh/model assets, full repository structure and matched setup must be supplied by the application checkout.

- New original proof SHA256: `9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7`.
- New original pytest log SHA256: `1ecbbb59c74a46232d60b7245b1d4a7fe085e452c3b5d8504c7896bfd5e8dbf6`.
- New selected CLI module SHA256: `42409bb28b5154b853c229360e80cc66c12254eee5233cca46a9a264a7a77b4d`.
- Reviewed new launcher SHA256: `b8abfcc18412aa516ae655d07ab2be27a11ba065fba1b75d63dd9307b7b2ea37`.

The original proof JSON binds the exact verification wrapper, all74 source/test names and SHAs, and native runtime paths/SHAs. The reused verification wrapper is byte-identical; the original new proof and log are distinct from earlier proof artifacts.

A new producer Git commit is not yet assigned when this packet freezes. Do not use `da69a9cd...` as a reset-speed2 producer: that checkout lacks the new CLI flag. The actual published new producer must provide these74 exact source bytes and the corresponding full application assets/setup. The reviewed launcher verifies both the explicitly supplied actual producer HEAD and every tested source before preparing or launching. Its original `run_publication_identity.json` will record that actual new producer commit, exact command, source inventory and runtime at native-run start.

## Verify the packet and application source

From this packet directory, standard Python can verify every checksum without importing the engine or executing tests:

```bash
python - <<'PY'
from pathlib import Path
import hashlib
packet = Path('.')
lines = (packet/'SHA256SUMS').read_text().splitlines()
assert len(lines) == 79
for line in lines:
    expected, name = line.split('  ', 1)
    assert hashlib.sha256((packet/name).read_bytes()).hexdigest() == expected, name
print('Verified79 exact file checksums')
PY
```

After obtaining the published new producer checkout and this packet at `media/m8_table_supported/software_proof_reset_speed2_v3`, verify the actual application source from the repository root:

```bash
python - <<'PY'
from pathlib import Path
import hashlib, json
packet = Path('media/m8_table_supported/software_proof_reset_speed2_v3')
proof = json.loads((packet/'software_proof.json').read_text())
assert proof['passed'] and proof['tests_passed'] == 672
assert proof['source_hashes_unchanged'] and proof['runtime_files_unchanged']
assert len(proof['source_hashes']) == 74
for name, expected in proof['source_hashes'].items():
    assert hashlib.sha256((packet/'source_files'/name).read_bytes()).hexdigest() == expected, name
    assert hashlib.sha256(Path(name).read_bytes()).hexdigest() == expected, name
print('Application and all74 archived proof files match')
PY
```

If a checkout's source differs, do not reinterpret the historical proof as covering it. Pin the proper new producer or make a separately reviewed source change and obtain a new proof. Copying these source snapshots into an unrelated checkout does not prove its assets/setup or Git producer identity.

## Prepare the future fresh run without integration

Use the repository's matched environment setup and binding checks. `scripts/run_m8.sh` validates the matched SDK/core before using its configured Python. This proof's native runtime identities refer to the tested cloud machine; on a different machine, reproduce the pinned matched setup and obtain a new machine-bound proof before native launch. The reviewed launcher rejects a changed runtime rather than silently treating a different engine as tested.

The following is the **prepare-only** recipe from the correct published new producer repository root. It checks the proof, actual producer HEAD, source and runtime; it creates no native output directory and initializes no native model:

```bash
task_producer=$(git rev-parse HEAD)
scripts/run_m8.sh \
  media/m8_table_supported/software_proof_reset_speed2_v3/launch_supported_reset_speed2_v3.py \
  --repository-root "$PWD" \
  --proof media/m8_table_supported/software_proof_reset_speed2_v3 \
  --producer "$task_producer" \
  --output outputs/m8_table_supported/full_reset_speed2_v3 \
  --prepare-only
```

Do not omit `--repository-root`: the launcher also ships as a nested archived artifact, so its surrounding directory is not an application root. Choose a new absent output path; neither prepare nor launch may overwrite a previous native attempt.

A native run remains gated until the current closed evidence/source identities have been preserved and the reviewed new producer is selected. When authorized, omit only `--prepare-only` from that recipe. The launcher executes this exact fresh unspliced native command:

```bash
scripts/run_m8.sh -m yam_twin.m8_supported_demo \
  --output outputs/m8_table_supported/full_reset_speed2_v3 \
  --dt .00005 --starting-angular-speed 1 --angular-speed 2 --reset-speed 2 \
  --maximum-entry-dwell 10 --maximum-starting-strokes 5 \
  --qualifying-strokes 2 --axial-damping 200
```

Both free objects start independently on the table/rest; the arms acquire them in that run. No cold checkpoint, object pose injection, stitched trajectory, carried-block substitution or inherited ready window enters this command. Original force histories, fixed cumulative grasp references, strict table/pad/contact/drift/collision guards and original nonzero failure exits remain intact. The launcher retains complete original before/after source/runtime identities and the native child's actual exit, including failed attempts. A completed software proof never substitutes for actual formed-flank capture, metric lead, unassisted OPEN reset or full-task qualification.
