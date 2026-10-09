# Canonical table-supported feedback software proof

**402 software tests pass** for the exact **65-file source/test map** archived
here: 85.89 s pytest time, 86.63 s wrapper time. The immutable verifier compares
all source hashes before and after the suite; they remain unchanged. Packaging
independently verifies the current frozen bytes against that map. The original
proof, log and verifier are unchanged.

This is **software-only evidence for the new canonical candidate**. It does not
qualify a native full trajectory, thread capture, metric lead, passive reset,
hardware calibration or a trained policy. The older b2b13ff/281 source proof,
213/210 supported proofs and first carried-block181/176 proofs remain separate.
The source includes the finite head-feedback controller and its additive auditor,
30 mm jaw opening, B200 damping and live bounded phase gates. A fresh unspliced
native attempt and its original force/lead/reset acceptance are still required.

`sources/` contains every exact proof-bound file at its repository-relative path.
`manifest.json` binds those bytes plus original proof/log/verifier and the
reviewed output-only launcher snapshot. `launch_source.py` is orchestration
provenance, outside the65-file test map. It requires its original location under
`outputs/m8_table_supported/` when executed; running it directly here resolves
the wrong repository root. Copy it to that location in the complete pinned
producer checkout. It retains the native exit code and binds before/after source
hashes; it is not itself a trajectory qualification result.

From this package directory:

```sh
sha256sum -c SHA256SUMS
```

The commit publishing this immutable packet contains the tested source. In a
review clone, identify it without assuming later main is the same producer:

```sh
git log -1 --format=%H -- media/m8_table_supported/software_proof_feedback_v1
```

Detach a separate complete checkout at that commit before reproducing the
software or upcoming native command. Use the matched GCC MuJoCo CPU setup and
launcher in `docs/m8_setup.md`; stock MuJoCo is a separate runtime. From the
producer checkout, `scripts/run_m8.sh -m pytest -q` repeats the software suite.
Do not combine this count with historical or standalone diagnostic regressions.
