# Agent handoff: table-supported M8 attempt

The [closed failed native package](../media/m8_table_supported/full_canonical_v1_failed_evidence/README.md)
preserves the complete original run, sources, raw force histories and audits.
[Normal 1× GIF](../media/m8_table_supported/full_canonical_v1_failed_evidence/demo.gif) ·
[MP4](../media/m8_table_supported/full_canonical_v1_failed_evidence/demo.mp4) ·
[Actual endpoint](../media/m8_table_supported/full_canonical_v1_failed_evidence/endpoint_detail.png) ·
[Measured direction-gate chart](../media/m8_table_supported/full_canonical_v1_failed_evidence/scientific_plot/native_direction_gate.png).

The completed carried-block demo remains separate; use its unchanged
[first handoff](m8_agent_handoff.md). This handoff concerns the free female-threaded block
stabilized on the table and a separately spawned M8 × 1.25 bolt.

## Recorded outcome and proof scope

Producer **`6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1`** passes **402 software
tests** with **65 exact source/test files** unchanged before/after testing and
the native run. Later documentation/media commits do not replace that source
binding. The original trace SHA is
`c4cd0f891ba6c9af51ced80b4802023f54472135649fab2d41122770a4ffc19c`.

The fresh, unspliced attempt closes at **16.47615 native seconds**, exit **1**,
after **2194.916 s launch wall time** (controller timer: 2156.069 s). Original
`passed=false`, `partial=false`, **19/27 checks pass**. It aborts in
`stop_reverse_seat_1`: the bounded 0.75 s stop fails to confirm the required
absolute 50 µm seat-direction drop; final stopped gain is **19.1214 µm**.
`partial=false` means the full schedule was selected, not completed.
Formed overlap and loaded interior contacts remain zero. No opening,
captured qualified turn or passive reset executes; all eight original task/
no-abort failures remain failures or unexecuted stages.

The independent compatibility audit reads **329,523 original native ticks**
and checks **1,717 original local thread-force/contact frames**. The trajectory
has **3,311 saved states**. The audit passes **14/19 independent checks** and remains
**overall false**: the five required full-task stages are absent. Original
strict left/table retention covers **296,524 active ticks**, with zero gaps;
left pad minima are **15.9485 / 15.8324 N**. Every active table window has at
least **99.046985%** of block weight in mean table reaction, at most **0.953019%**
mean positive left upward support and 100% loaded duty. Original minimum
transport rest clearance is **14.93886 mm**, above the 10 mm guard. These are
measurements of the executed failed prefix, not successful assembly.

## Reproduce the exact fresh attempt

Use an unused COMPLETE clone. Keep first-demo files, original evidence and
any active experiment untouched. Reuse this cloud's verified native runtime;
on a fresh host run setup first from the pinned clone:

```sh
git clone https://github.com/robomechanics/astra-r2s.git /workspace/astra-r2s-supported-reproduce
git -C /workspace/astra-r2s-supported-reproduce switch --detach 6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1
cd /workspace/astra-r2s-supported-reproduce
scripts/setup.sh
scripts/run_m8.sh -m yam_twin.m8_supported_demo --help
scripts/run_m8.sh -m yam_twin.m8_supported_demo --output outputs/m8_table_supported/agent_fresh --dt .00005 --starting-angular-speed 1 --angular-speed 2 --maximum-entry-dwell 10 --maximum-starting-strokes 5 --qualifying-strokes 2 --axial-damping 200
```

The destination must be absent. Omit `--maximum-phases` and `--replay`; this
starts from independent table/rest spawns. The explicit **10 s entry bound**
changes only the timeout configuration. Defaults at this producer give 30 mm
right opening, 18.4 mm closure, 2 N downward left stabilization/B200 and
privileged perfect native pose feedback at 20 kHz through finite robot motors.
Closed axial advance remains contact/load driven, without axial position/pitch
feedback. These exact sources/flags reproduce a failed experiment, not a
completed threading trajectory. The recorded failure took 36.58 min; a longer
bounded attempt can cost 1–2 h or more. The maximum schedule is 80.8969 native
seconds, about 2.7 h at the observed cold V4 cost, not an executed duration.

Setup needs Python 3.12, uv, Git, GCC/G++, network and the pinned lockfiles;
see `docs/m8_setup.md`. Use `scripts/run_m8.sh`: it verifies the matched GCC
MuJoCo 3.15.0 bindings/core and clears `LD_PRELOAD`. Stock MuJoCo/MJLab/Newton
are separate runtimes. Original core SHA:
`58039d439c6504448aafd0a4d5b655c8a0d3bf1d77123ac7e0733cd078367433`;
plugin source SHA:
`1c8b5207c5f6c141cc034983ce76c6c1e9cca4114d16e17a4496b94cb637b42a`.
Exact native audit requires the recorded runtime hashes; a different host's
build is not silently labeled binary-equivalent.

For the same source-bound wrapper used in the original run, instead of the
bare fresh command, restore its required script depth and use a new output:

```sh
mkdir -p outputs/m8_table_supported
cp media/m8_table_supported/software_proof_feedback_v1/launch_source.py outputs/m8_table_supported/launch_continuous_supported_v3.py
python outputs/m8_table_supported/launch_continuous_supported_v3.py --proof media/m8_table_supported/software_proof_feedback_v1 --producer 6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1 --output outputs/m8_table_supported/agent_bound_fresh
```

The wrapper requires exact producer HEAD and all 65 proof-bound bytes. It
retains original native exit/log and before/after source maps; exit 2 denotes
source drift. Its root is `parents[2]`, so do not execute it from the media
folder. A wrapper result is distinct from physical acceptance.

## Observe live; use complete closed evidence for review

During a fresh run, read its `run_publication_identity.json` and
`native_stdout_stderr.log`. Phase-end `insertion_trace_partial.npz` can be
mid-write and contains no complete final force coverage. A copied progress
frame/prefix cannot qualify capture or omitted phases. Do not run native-model
audits while integration is alive.

Require closure identity `run_publication_identity_after.json`, final
`insertion_validation.json`, final `insertion_trace.npz`, original XML/ZIP and
sources, all feedback/table/pad ledgers, logs and SHA bindings. Preserve raw
failures and aborted/unexecuted stages. Do not replace a final trace with a
partial snapshot or concatenate historical cold diagnostics.

## Restore the closed record

The producer pin predates the evidence commit. Copy the COMPLETE packet
from a newer review checkout into the pinned clone's ignored outputs. Do not
copy the NPZ alone or rely on the five-file frozen audit snapshot as a complete
Python package. The final manifest/restorer must preserve all native original
relative names and exact bytes, including ZIP/XML, renderer/controller/imported
sources, original validation, launch manifest and the three dense force ledgers.
Any chunks must reassemble losslessly with ordered offsets/sizes/chunk SHA and
whole-file SHA; no omitted/quantized data are allowed.

The package contains **182 files / 181 checksums**, with SHA256SUMS identity
`47b1c9d7e94d9cd8a00ccba5c478d873cdb9cdce9e9cc78838f08d97893d2f18`.
All 34 original native-run files were losslessly restored and compared
byte-for-byte; all 65 producer files and five frozen audit dependencies match.
From the pinned clone, copy the newer review checkout's complete package into
an absent ignored destination and verify/restore without a simulator:

```sh
python - <<'PY'
from pathlib import Path
import shutil
target=Path('outputs/m8_table_supported/closed_failed_evidence')
target.parent.mkdir(parents=True,exist_ok=True)
shutil.copytree('/workspace/astra-r2s/media/m8_table_supported/full_canonical_v1_failed_evidence',target)
PY
python outputs/m8_table_supported/closed_failed_evidence/restore_original_run_layout.py --verify-only
python outputs/m8_table_supported/closed_failed_evidence/restore_original_run_layout.py --output outputs/m8_table_supported/full_canonical_v1_original_layout
```

The standard-library restorer defaults to adjacent `package_manifest.json`,
verifies packaged/support/chunk and final whole bytes, restores ALL original
run paths and refuses changed existing destinations. Place the archived
helpers at their required original depth, preserving all compatibility
companions and refusing changed existing helper bytes:

```sh
python - <<'PY'
from pathlib import Path
import shutil
p=Path('outputs/m8_table_supported/closed_failed_evidence')
out=Path('outputs/m8_table_supported')
audit=out/'full_canonical_v1_audits'
copies=[]
for archived,name in [('00_verify_closed_supported_run_identity.py','verify_closed_supported_run_identity.py'),('01_independent_rest_clearance_audit.py','independent_rest_clearance_audit.py')]:
    copies.append((p/'execution_provenance'/archived,out/name))
for src in (p/'auditor_compatibility').rglob('*'):
    if src.is_file(): copies.append((src,audit/src.relative_to(p/'auditor_compatibility')))
copies.append((p/'supplemental_audits/independent_original_seat_arithmetic_v1.py',audit/'independent_original_seat_arithmetic_v1.py'))
for src,dst in copies:
    if dst.exists() and dst.read_bytes()!=src.read_bytes(): raise ValueError('Changed existing helper: '+str(dst))
    dst.parent.mkdir(parents=True,exist_ok=True)
    if not dst.exists(): shutil.copy2(src,dst)
PY
```

This preserves the complete compatibility sidecar and its original relative
companions, not an isolated edited reader. Source verifier/rest auditor stay
at their original repository depth; all helpers remain outside canonical code.
Never overwrite `scripts/audit_m8_supported_trace.py` with the compatibility
reader: the original 402-bound source must stay `e4680860…`.

## Replay and audit without integrating

After verified complete restoration and helper placement, use the pinned full
checkout/runtime. All derivative output paths must be new and separate:

```sh
mkdir -p outputs/m8_table_supported/agent_closed_audits
python outputs/m8_table_supported/verify_closed_supported_run_identity.py outputs/m8_table_supported/full_canonical_v1_original_layout --output outputs/m8_table_supported/agent_closed_audits/identity_before.json
scripts/run_m8.sh -m yam_twin.m8_supported_demo --replay outputs/m8_table_supported/full_canonical_v1_original_layout/insertion_trace.npz --output outputs/m8_table_supported/agent_replay --fps 12 --slow-motion 1
scripts/run_m8.sh outputs/m8_table_supported/full_canonical_v1_audits/audit_m8_supported_trace_initial_boundary_v1.py outputs/m8_table_supported/full_canonical_v1_original_layout/insertion_trace.npz --output outputs/m8_table_supported/agent_closed_audits/supported_compatibility.json
scripts/run_m8.sh scripts/audit_m8_left_pad_force_history.py outputs/m8_table_supported/full_canonical_v1_original_layout/insertion_trace.npz --output outputs/m8_table_supported/agent_closed_audits/left_pad.json
scripts/run_m8.sh scripts/audit_m8_free_joint_properties.py outputs/m8_table_supported/full_canonical_v1_original_layout/insertion_trace.npz --output outputs/m8_table_supported/agent_closed_audits/passive_objects.json
scripts/run_m8.sh outputs/m8_table_supported/independent_rest_clearance_audit.py outputs/m8_table_supported/full_canonical_v1_original_layout/insertion_trace.npz --output outputs/m8_table_supported/agent_closed_audits/rest_clearance.json
scripts/run_m8.sh outputs/m8_table_supported/full_canonical_v1_audits/independent_original_seat_arithmetic_v1.py outputs/m8_table_supported/full_canonical_v1_original_layout --output outputs/m8_table_supported/agent_closed_audits/original_seat_arithmetic.json
python outputs/m8_table_supported/verify_closed_supported_run_identity.py outputs/m8_table_supported/full_canonical_v1_original_layout --output outputs/m8_table_supported/agent_closed_audits/identity_after.json
```

Run serially after closure; no native integration occurs. Replay selects actual
saved post-state qpos/qvel at normal 1× playback, with no interpolation or
stitching; it writes `supported_demo.mp4/.png` without `--video`. Geometry
refresh does not recreate historical solved forces. Never use default audit
outputs beside immutable evidence. Reader execution is not acceptance: the
compatibility report must preserve original failed gates and overall false.
The generic replay uses `mj_forward` and may compute unused new forces; the
publication render uses kinematics only. Its exact
[media recipe](../media/m8_table_supported/full_canonical_v1_failed_evidence/REPLAY_ACTUAL_MEDIA.md)
and archived renderer remain separate.

The original `e4680860…` reader raises a preserved initial acquisition-boundary
exception. Separate corrected reader **`abb4e724…`** and sidecar **`36c7add2…`**
change only the initial same-row guard interpretation: `settle_bolt` acquires
its reference before guard computation, so that row is active; later quiet
regrasp acquisition is after cached guard computation, inactive on acquisition
and active next tick. Eight pure standalone regressions verify this correction;
they do not extend the 402-test proof. The original reader/log/diff/diagnosis
and native data stay unchanged. The corrected report SHA is
`48e629f328152d6f827aef9add7f2870ba5f934f548c198575c21dbcf3fc5f13`.

Keep timing distinct: original force/derived geometry at `t−dt`, saved qpos/
actual aperture at `t`, retained previous-solve command calibration at `t−2dt`.
Independently check every expected tick and endpoint, rolling table-load gates
versus strict gaps, positive hand uplift versus downward compression, actual
acquisition references, finite caps, contact masks and zero object drives.
Table force above block weight is extra compression, not a percentage of
weight carried. Capture, qualified full-pitch lead, unsupported open stability
and no head/world seating support are separate required stages; absent stages
remain unexecuted. Historical cold V4 is not spliced into this failed trial.

## Policy-training limits and next experiment

This remains a privileged 20 kHz mechanics demonstrator, with no trained or
perception policy. Existing `YamM8InsertionEnv` is carried-block: its secure-left
condition forbids block/world contact, reset/reward requires pickup and its jaw
mapping is 24/19.4 mm. The supported teacher uses table bearing and 30/18.4 mm;
a separate supported reset/action/reward/termination wrapper is required.

Dense 50 µs forces/motor commands plus mostly 5 ms saved qpos/qvel are not a
complete exact 20 kHz state/action dataset. Export must declare time alignment,
normalization and already-applied bias/damping; adding Gym bias again changes
the action. The 118-vector lacks rolling capture/acquisition histories used by
the teacher, default 30 s horizons do not replay the first 43.636 s expert,
and reset is deterministic without supported randomization. No learning or
robustness result is supplied. Gym success does not enforce the demo reset.

Keep the measured 45.02% contact-depth/search refinement failure visible;
travel/late-lead convergence does not qualify force/depth or whole-robot
convergence. Friction/compliance are uncalibrated numerical assumptions.
Full seating, preload, cross-thread damage, stripping/wear and hardware transfer
remain unqualified.

The proposed output-only crest experiment passes **69 standalone tests**:
the original 37 observer tests plus 32 cold-window checks, in 0.63 s. Observer
SHA is `1d37ef46382c349e29b4206206e9aac2132587b65abc39467342e7e1b0560c0c`;
the separate cold harness/preparation has its own `85c485…` source binding.
Preparation checks are complete; one cold native diagnostic is pending,
with no physical result yet. This code is neither canonical 6e7 nor an extension
of the 402 proof. A local return from an observed
withdrawal crest is a new CLOSED direction heuristic, not the original absolute
50 µm criterion, capture proof or permission to open. Any future changed
controller requires its own source/config binding, native attempt and audits.
