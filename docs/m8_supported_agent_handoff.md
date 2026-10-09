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

## Current producer: fresh run

Current producer **`9ae1a9fe76968a4013ea6c39e67718026622b9c0`** passes
[672 whole-suite tests on 74 unchanged files](../media/m8_table_supported/software_proof_reset_speed2_v3/README.md),
with 104.74 s pytest / 105.516 s proof-wrapper duration and unchanged runtime.
Its new proof SHA is
`9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7`.
The one selected-file change from da69 is CLI reset-speed exposure; the other
73 files, controller, model/thread physics, finite caps and guards are unchanged.
Default 4 rad/s remains; the new native command explicitly selects 2 rad/s.

The fresh **`full_reset_speed2_v3`** attempt starts at **2026-10-09
14:32:53.840320 UTC**, from independent table/rest spawns, with no cold state
or stitched trajectory. Native integration is in progress; no captured-thread,
qualified reset, full-task outcome or closed audit is claimed. BEFORE identity is
`1aceb2b906b6201ba715bde99dfe1394bb0880907b0f9914529a9c50adff1ec9`.
The historical da69 failed run and its 13.1946 s progress clips below remain
separate. They do not show the current 9ae trajectory.

Use an absent complete clone destination and detach to the exact new pin.
The commands below use the recorded runtime; for a different-host build,
follow [the local-proof case](#different-host-local-proof) before launching:

```sh
git clone https://github.com/robomechanics/astra-r2s.git /workspace/astra-r2s-supported-reset2-agent
git -C /workspace/astra-r2s-supported-reset2-agent switch --detach 9ae1a9fe76968a4013ea6c39e67718026622b9c0
cd /workspace/astra-r2s-supported-reset2-agent
scripts/setup.sh
scripts/run_m8.sh -m yam_twin.m8_supported_demo --help
python media/m8_table_supported/software_proof_reset_speed2_v3/launch_supported_reset_speed2_v3.py --repository-root "$PWD" --proof media/m8_table_supported/software_proof_reset_speed2_v3 --producer 9ae1a9fe76968a4013ea6c39e67718026622b9c0 --output outputs/m8_table_supported/agent_reset2_fresh --prepare-only
python media/m8_table_supported/software_proof_reset_speed2_v3/launch_supported_reset_speed2_v3.py --repository-root "$PWD" --proof media/m8_table_supported/software_proof_reset_speed2_v3 --producer 9ae1a9fe76968a4013ea6c39e67718026622b9c0 --output outputs/m8_table_supported/agent_reset2_fresh
```

Reuse the verified cloud runtime if already installed; setup is needed on a
fresh host. Never rebuild shared core/bindings/plugin during another native
integration. Preparation checks exact HEAD, all 74 proof-bound source bytes
and recorded runtime files, without model initialization, native steps or
output creation. `--repository-root` makes the public launcher portable;
it does not require the historical helper depth. The proof is bound to the
recorded binary/runtime hashes listed below. A different machine's rebuild
needs a new local proof/runtime binding, not a claim of identical binaries.
The archived `verification_source.py` reruns the whole suite; it is not a
read-only source verifier. Its original proof/log remain immutable.

### Different-host local proof

A different host's rebuilt binary can differ from the published runtime SHA,
so the published-proof preparation above rejects it by design. After setup,
create a NEW local proof and log, preserving the published packet. The whole
suite includes native tests: run it serially when no other native integration
is active. From the exact detached 9ae checkout:

```sh
python media/m8_table_supported/software_proof_reset_speed2_v3/verification_source.py --output outputs/m8_table_supported/agent_local_software_proof
python - <<'PY_SOURCE_CHECK'
import json
from pathlib import Path
published=json.loads(Path('media/m8_table_supported/software_proof_reset_speed2_v3/software_proof.json').read_text())
local=json.loads(Path('outputs/m8_table_supported/agent_local_software_proof/software_proof.json').read_text())
assert local['passed'] and local['source_hashes_unchanged'] and local['runtime_files_unchanged']
assert len(local['source_hashes']) == 74 and local['source_hashes'] == published['source_hashes']
PY_SOURCE_CHECK
python media/m8_table_supported/software_proof_reset_speed2_v3/launch_supported_reset_speed2_v3.py --repository-root "$PWD" --proof outputs/m8_table_supported/agent_local_software_proof --producer 9ae1a9fe76968a4013ea6c39e67718026622b9c0 --output outputs/m8_table_supported/agent_local_reset2_fresh --prepare-only
```

The proof and trial destinations must be absent. Preparation still takes zero
native steps; start the actual fresh trial by repeating the last command
without `--prepare-only`. This binds a new local runtime/trial with the same
74 source bytes, not binary-equivalent reproduction of the archived trial.
Preserve its new proof/log, runtime identities and native closure independently.

The launcher starts exactly this child; use the launcher OR the bare CLI
for a new run, not both into the same destination:

```sh
scripts/run_m8.sh -m yam_twin.m8_supported_demo --output outputs/m8_table_supported/agent_reset2_fresh --dt .00005 --starting-angular-speed 1 --angular-speed 2 --reset-speed 2 --maximum-entry-dwell 10 --maximum-starting-strokes 5 --qualifying-strokes 2 --axial-damping 200
```

No replay, phase truncation, parent checkpoint or state injection is used.
The explicit 10 s entry timeout is separate from physical readiness. At this
pin, 30 mm actual opening/18.4 mm closure and finite downward left stabilization
remain. The measured local crest return only requests CLOSED yaw deceleration;
150 ms C2 braking plus a fresh 100 ms body/hand quiet, original impulse/current
load and 90/10 bolt-weight window must confirm direction before forward motion.
Approximate arm inertia feedforward controls two transverse translations and
three rotations, excluding axial acceleration/position/lead. Fingers remain
native; their independent acceleration coupling/contact dynamics are not
inverted. Pickup/transport/open/regrasp rows explicitly disable feedforward.
Total PD/feedforward and native motor caps remain finite. Perfect native pose
feedback at 20 kHz is privileged; no learned policy or supported Gym wrapper
is supplied.

The wrapper preserves `run_publication_identity.json`, original native
stdout/stderr/exit and `run_publication_identity_after.json`, all 74 before/after
source hashes and runtime identities. Exit 2 records source/runtime drift;
a native failed exit remains failed. The final native archive must retain
XML/ZIP/imported sources, original validation/trace, all table/pad/native-feedback
ledgers and the complete `robot_inertia_command_history.npz`. Enabled FF rows
need exact retained M/J/Jdot/velocity inputs; disabled rows keep false presence
flags and explicit uncomputed placeholders. Rejected unapplied commands are
separate from actual solved steps/previous controls. Preserve raw timing and
failed/unexecuted stages, and reassemble any large arrays losslessly.

Phase-end partial snapshots/logs are progress only while integration lives.
Require complete closed artifacts/source maps before serial audits; do not
solve/audit live outputs. Record this new attempt's actual native/wall duration
at closure. The pinned auditor source `4a8b018a…` handles the new canonical
ledger; historical e468/abb4 compatibility and 65-file wrappers are for the
older failed run only. Recorded-state geometry replay refreshes saved poses,
not native forces or controller execution; keep a complete matching pinned
checkout/runtime and write derivatives to new paths. A successful shell
preparation or audit execution does not certify physical acceptance. This
new live attempt has no closed outcome. Default reset speed remains 4 rad/s;
the explicit 2 rad/s selection doubles the intended reset schedule to
2.945243112740431 s, halves scheduled velocity and quarters acceleration.
These are command-profile facts, not measured tracking/clearance results.

After closure, a generic received identity verifier must bind THIS producer,
its own original BEFORE/AFTER and the new proof. For this recorded launch the
anchors are producer `9ae1a9fe76968a4013ea6c39e67718026622b9c0`, BEFORE
`1aceb2b906b6201ba715bde99dfe1394bb0880907b0f9914529a9c50adff1ec9`
and proof `9f3617e33a047e1c5755e6fa62a76d490cc16bb73980ded33c2f57caac34d1a7`;
AFTER does not exist until native closure. A new agent's separately launched
trial uses its own BEFORE/AFTER identities. The old fixed da69 closed-media
gate and historical source maps must not be substituted for these anchors.
Require the complete matching helper/source/runtime archive before original-
layout replay/audit; partial milestone bytes do not establish full coverage.

## Historical da69a9c C2/inertia failure

Historical producer **`da69a9cd44a8312cc7b97365faf5e09c27a646e2`** passes
[672 whole-suite tests on 74 unchanged source/test files](../media/m8_table_supported/software_proof_c2_inertia_v2/README.md),
with 88.97 s pytest / 89.685 s proof-wrapper duration and unchanged native
runtime. The source packet archives all 74 files, original proof/log and the
portable launcher. Its software proof SHA is
`71e6d7b7a4688ae8a6e4588424f931b64f824e9552d3d792278ebe472b767be9`.
These tests do not qualify native thread capture or a completed trajectory.

The fresh `full_c2_inertia_v2` attempt starts at **2026-10-09
13:12:50.752963 UTC**, from independent table/rest spawns, and closes failed
at **22.08285000017131 s / 441,657 native ticks**, native exit **1**, after
**3583.482 s / 59.72 min** launch wall time. All 74 source/runtime identities
stay unchanged. Original `passed=false`, `partial=false`, **19/27 checks**
remain; selecting the full schedule does not mean completion. It aborts in
`reset_open_search_2`, after **1.0843 s** of the **1.47262 s** planned motion,
when one right pad recontacts the bolt at **1.487 N**. Capture, qualified
lead/reset and fresh complete assembly remain unachieved.
Its launch-before identity is
`f98c6ba218d1c5d8503ac3d362a4a501de4fd99a8f1d87e4ee2e11a704f4c9c3`;
original closure identity is
`481b930e3dadaddd54240ea9548f5decdb98abfdad9c41db05871c84cf5ccaa3`.
The three official audit commands execute successfully, while the primary
physical audit remains **overall false, 13/19 checks**. Table/left-retention
and passive-property reports pass only their executed scopes. Supplemental
binding `4be80e698c07137d6182dffa297586af18084569d594b4dc49f47ef187f9e605`
is frozen, covering 39 original native files and 36 derivative audit files.
Its ten pure reader regressions are separate from the 672-test producer proof.
The complete failed media/evidence packet is being packaged.
The older failed full run and all cold branches below retain their original
producer/evidence identities; no cold state is stitched into this attempt.

The immutable [pickup progress package](../media/m8_table_supported/full_c2_inertia_v2_progress/pickup_progress/README.md)
contains the complete original **3.47 s / 703-state** prefix and matching
74-source/672-test software packet, with [normal playback](../media/m8_table_supported/full_c2_inertia_v2_progress/pickup_progress/render/demo.gif),
[table stabilization](../media/m8_table_supported/full_c2_inertia_v2_progress/pickup_progress/render/left_stabilized_detail.png)
and [lifted bolt](../media/m8_table_supported/full_c2_inertia_v2_progress/pickup_progress/render/endpoint_detail.png).
Its copied state replay recipe refreshes geometry only. Feedforward is disabled
in all pickup samples; dense ledgers/final audit were pending at capture.
The side-pickup bore-distance guard is inactive; no thread entry is claimed.

The newer [immutable entry/transfer prefix](../media/m8_table_supported/full_c2_inertia_v2_progress/entry_transfer/README.md)
preserves **13.19459999997592 s / 2,652 original post-step states**, from the
same independent spawns through actual alignment, entry and weight transfer.
[Normal 1× GIF](../media/m8_table_supported/full_c2_inertia_v2_progress/entry_transfer/render/demo.gif),
[MP4](../media/m8_table_supported/full_c2_inertia_v2_progress/entry_transfer/render/demo.mp4)
and [entry detail](../media/m8_table_supported/full_c2_inertia_v2_progress/entry_transfer/render/endpoint_detail.png)
select 159 exact states at 12 fps (13.25 s encoded), with the true endpoint
in the last frame and no interpolation. Original transfer-window fields cover
**100.05 ms / 2,001 native samples**, reporting **100.003432%** mean thread
reaction of bolt weight, **1.383709%** mean positive right-hand upward support
and 100% loaded duty. Formed overlap and actual interior contact count stay zero.
Those fields are controller observations; dense force/command ledgers and the
independent full audit were pending at capture. Its 2,311 disabled / 341 enabled
inertia records are sparse saved samples, not every native command.

Package SHA256SUMS is
`28f69cb9769200cb776393d5e495421f199c2acb85aeb991b000fbc4674385f0`.
The **45,367,323-byte** original trace is losslessly split into 45,000,000 and
367,323-byte chunks. Use its [verify/copy/reassemble/replay recipe](../media/m8_table_supported/full_c2_inertia_v2_progress/entry_transfer/README.md#verify-and-replay-the-recording):
copy the COMPLETE newer package to an isolated da69 checkout, reassemble
all original artifacts/sidecars into a new output, and pass the restored
`native_snapshot` to the archived renderer with `--repository-root "$PWD"`.
The 74-source/runtime checks and kinematics-only geometry replay integrate
no motion and reproduce no contact forces. Do not concatenate this prefix
with pickup or cold-trial recordings.

The run later completes **1.4431 s** reverse motion and a **150 ms C2 brake**
within its **0.36275 s** stopped-direction phase. Its actual quiet/load window
reports **103.658724%** thread reaction of bolt weight and **0.458881%** positive
upward hand support, with formed overlap zero. The completed first shallow
forward phase reaches only **21.60 µm** formed overlap and zero interior
contacts. The subsequent **0.29775 s** open-settle phase has zero whole-right/
bolt contacts, with **65.42 nm / 0.162 mrad** maximum drift. The later reset
recontacts and aborts, so no qualified passive reset is completed. These later
events are absent from the historical 13.1946 s prefix; its capture-time scope
is unchanged.

To repeat that historical failed experiment, use a COMPLETE clone detached
to `da69a9cd44a8312cc7b97365faf5e09c27a646e2` and its original 672/74
software packet. Do not use new 9ae source bytes for its archived replay.
The unchanged historical native command is:

```sh
scripts/run_m8.sh -m yam_twin.m8_supported_demo --output outputs/m8_table_supported/agent_historical_da69 --dt .00005 --starting-angular-speed 1 --angular-speed 2 --maximum-entry-dwell 10 --maximum-starting-strokes 5 --qualifying-strokes 2 --axial-damping 200
```

It uses the original default 4 rad/s reset. This repeats a failed-capable
fresh native experiment, not a successful assembly or recorded-state replay.
The complete closed package/portable geometry-only recipe is being prepared;
its exact original source/runtime/audit identities must remain distinct from
the current reset-speed2 launch. No future evidence link is assumed.

## Historical 6e7d0d2 outcome and proof scope

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

## Reproduce the historical failed fresh attempt

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

## Separate cold crest-search diagnostics

The [closed V3 cold failure](../media/m8_table_supported/diagnostics/crest_seat_search_v3_failed/README.md)
starts once from original parent row 2651, post-state **13.1949 s**. It copies
qpos/qvel/ctrl and original cumulative grip references, with no solver warm
starts, old native force samples or previous force windows. A local return
from an observed withdrawal crest is a new CLOSED direction heuristic,
distinct from the frozen full attempt's absolute 50 µm criterion; it is not
capture proof or permission to open. No cold branch is stitched into a full run.

V3 requests a closed stop at **1.61915 s**, after a measured **50.0352 µm**
return. Independent-clock angular command changes abruptly to zero; the robot
reaches its existing 8 N / 2 N·m Cartesian caps. Radial error grows to
**150.848 µm**, above the unchanged 150 µm guard, at **1.6254 s**. Original
exit 1, `passed=false`, `partial=true` remain. There is no quiet-stop readiness,
forward scan, formed/interior capture, opening or qualified reset.

The frozen [independent binding](../media/m8_table_supported/diagnostics/crest_seat_search_v3_failed/independent_audits/independent_audit_binding.json)
(`ae870d0a…`) covers **32,508 original ticks / 47 feedback columns**. Its
**10 reader regressions** are separate from the **69 observer/cold-window
contracts** and **402 canonical tests**. All 1,999 initially unavailable
rolling windows and the raw false table-window label remain; complete fresh
windows are evaluated separately. Original right-pad local records were not
saved, so the raw-ledger/saved-sample right-wrench match is a consistency
check, not independent per-contact reconstruction.

Use the [complete isolated-copy and source-placement recipe](../media/m8_table_supported/diagnostics/crest_seat_search_v3_failed/README.md#reproduce-in-an-isolated-producer-checkout)
before running anything. It restores the WHOLE failed parent into the pinned
6e7 checkout and installs frozen harness `85c485e5…` plus observer `1d37ef46…`
directly under `outputs/m8_table_supported/diagnostics/` (`ROOT=parents[3]`).
Neither experiment belongs to canonical code or extends the 402/65 proof.
The original cold reader requires its declared parent path and sibling layout;
the package does not supply a portability adapter or revised declaration.
Do not substitute the full-run compatibility auditor for this cold reader.

The frozen package contains 145 files / 144 checksums (ledger SHA
`b3fbe85ce75a093051ec389561df559b1d71c0d60173caacd4883ce3469a35d6`).
Its original-layout helper separately verifies/restores all 26 cold native
files, without a simulator, after the complete package-copy steps:

```sh
python outputs/m8_table_supported/cold_inputs/crest_v3_package/restore_original_run_layout.py --verify-only
python outputs/m8_table_supported/cold_inputs/crest_v3_package/restore_original_run_layout.py --output outputs/m8_table_supported/diagnostics/crest_v3_original_layout
```

After the package's complete copy/restore steps, these are two separate actions:

```sh
# Fresh cold native attempt; use an absent output, and retain any new failure.
scripts/run_m8.sh outputs/m8_table_supported/diagnostics/crest_seat_search_probe_v3.py --parent outputs/m8_table_supported/crest_v3_original_parent/insertion_trace.npz --checkpoint-time 13.1949 --output outputs/m8_table_supported/diagnostics/crest_v3_repeat
# Exact recorded-state media replay; no native integration or force solve.
scripts/run_m8.sh outputs/m8_table_supported/cold_inputs/crest_v3_package/render/renderer_sources/render_crest_seat_search_v3.py --repository-root "$PWD" outputs/m8_table_supported/cold_inputs/crest_v3_package/native_v3 outputs/m8_table_supported/diagnostics/crest_v3_geometry_replay
```

The portable renderer checks all 65 pinned source bytes, original runtime and
closed native artifacts before/after. It uses kinematics only, displays
original `t−dt` forces against saved `t` states, and keeps `t−2dt` command
calibration distinct. This replays the original failure; it does not run its
controller or recreate its solve history. The package includes normal 1×
media, every original raw archive and exact state/source hashes.

The separate **crest V4 smooth-brake trial closes failed at 1.7091 s**,
after 260.939 s native wall time. It executes 89.95 ms of the intended 150 ms
brake before radial error reaches **150.011272 µm**, above the unchanged
150 µm guard. Final tilt is 5.641 mrad and grasp slip 247.626 µm; no quiet
direction readiness, forward scan or capture occurs. Its 34,182 ticks and
50 feedback columns are closed, with all 65 canonical, nine experimental
source, nine parent-input and runtime hashes unchanged. The
[frozen V4 failed package](../media/m8_table_supported/diagnostics/crest_seat_search_v4_failed/README.md)
preserves all **28 original native files**, 50 columns and raw failed outcomes:
[normal 1× GIF](../media/m8_table_supported/diagnostics/crest_seat_search_v4_failed/render/demo.gif),
[MP4](../media/m8_table_supported/diagnostics/crest_seat_search_v4_failed/render/demo.mp4),
[actual mid-brake detail](../media/m8_table_supported/diagnostics/crest_seat_search_v4_failed/render/mid_brake_detail.png)
and [six-panel native chart](../media/m8_table_supported/diagnostics/crest_seat_search_v4_failed/render/scientific_plot/native_stop_boundary.png).
Frozen harness SHA is
`aee1c3d5f9f9b8b6b8359b9da92726130acc606862975ab66a8b9ba9174cf74b`;
braking helper SHA is
`4bc45250cbf1aba271dad51df2781971e323735d631349afcc20ab311afa21c1`.
Its actual command uses `crest_seat_brake_probe_v4.py --checkpoint-time 13.1949
--brake-duration .15`, from the same exact original parent, not the failed V3
endpoint. Only the 150 ms C2 braking schedule changes; geometry, motor caps
and physical readiness/capture guards stay unchanged. **50 pure helper tests**
verify the schedule/analytic bounds, separately from the retained 69 contracts
and 402-test proof. Frozen input/argv and execution-before records are local
`crest_seat_search_v4_frozen_inputs.json` / `crest_seat_search_v4_execution_before.json`
under `outputs/m8_table_supported/diagnostics/`; execution-after SHA is
`ca60886564935c746d4f497cd46167b82a5d2cbe889cbb8e1176f2afdb3ddfa8`.
Use its [exact isolated native-repeat recipe](../media/m8_table_supported/diagnostics/crest_seat_search_v4_failed/README.md#native-repeat-in-an-isolated-producer-checkout):
pin complete producer 6e7, copy both newer COMPLETE packages, restore all
34 original parent files and place `crest_seat_brake_probe_v4.py`,
`crest_seat_observer_v3.py` and `reverse_brake_v4.py` directly under
`outputs/m8_table_supported/diagnostics/`. These placements preserve
`ROOT=parents[3]` and adjacent imports. The parent remains original row 2651
at 13.1949 s, never the V3 abort. A fresh native repeat uses the explicit
`--checkpoint-time 13.1949 --brake-duration .15` flags and an absent output.
It integrates another cold branch, not the archived trajectory.

The separate [recorded-state media recipe](../media/m8_table_supported/diagnostics/crest_seat_search_v4_failed/README.md#geometry-only-recorded-state-replay-and-raw-plotting)
uses the archived `render_crest_seat_search_v4.py --repository-root "$PWD"`
with the complete copied `native_v4` directory and a fresh output. It verifies
source/runtime/raw bytes and performs kinematics only; 21 exact saved states
at 12 fps encode 1.75 s, with no interpolated state or native solve. The
historical independent reader remains path-bound to its original parent and
sibling layout; this package supplies no auditor portability adapter.

The [original audit binding](../media/m8_table_supported/diagnostics/crest_seat_search_v4_failed/independent_audits/independent_audit_binding.json)
`d524e6f7…` and [corrected derivative](../media/m8_table_supported/diagnostics/crest_seat_search_v4_failed/independent_audits_v2/independent_audit_binding.json)
`3c45982e…` remain separate. The latter corrects stale V3 prose while preserving V4 numerical
failure; rerunning the same **17 reader/mass regressions** is not 17 additional
tests and does not extend the 50/69/402 proofs. The separate
[19-state mass diagnostic and scope note](../media/m8_table_supported/diagnostics/crest_seat_search_v4_failed/independent_audits_v2/robot_inertia_scope_note.json)
uses geometry/CRB/mass/Jacobians without collision, contact/dynamics solve or
native-force replay. Its right-six-arm subblock holds off-arm finger
accelerations zero; it excludes bolt inertia, Jdot and calibration-derivative
reestimation. It is a prospective capacity estimate, not executed feedforward
or proof of the sole cause of drift. Combined 8 N / 2 N·m and native motor caps
remain required. No V4 readiness, forward scan, capture or full completion is
claimed.

The separate [V5 stopped-direction progress record](../media/m8_table_supported/diagnostics/crest_seat_search_v5_stopped_progress/README.md)
ends at **1.90625 s**, after the actual 150 ms brake and a fresh 100 ms quiet/
load gate. Its [normal 1× clip](../media/m8_table_supported/diagnostics/crest_seat_search_v5_stopped_progress/render/demo.gif)
and [actual endpoint](../media/m8_table_supported/diagnostics/crest_seat_search_v5_stopped_progress/render/endpoint_detail.png)
record 15.317 µm radial error, 1.027 mrad tilt and the original controller's
stopped-direction event. The sampled window reports 103.646% mean thread
reaction of bolt weight, 0.4851% positive hand support and 100% loaded duty;
formed overlap/interior contacts remain zero. This is shallow-entry progress,
not engagement, capture/reset or a fresh complete trajectory.

The [complete closed V5 packet](../media/m8_table_supported/diagnostics/crest_seat_search_v5_closed/README.md)
preserves **7.3566 s / 147,132 native ticks**, exit 0 and no abort, after
**1141.46 s** wall time. Its shallow forward scan advances **552.969644 µm**
over **2.842942425 rad**, with **−12.615781 µm** pitch-reference residual;
this is a partial-motion measurement, not a qualified lead turn. Final
formed overlap is only **21.6147 µm**, with **zero loaded interior contacts
and zero fully-open steps**. Original `passed=false`, `partial=true` remain;
`diagnostic_completed=true` records only the selected bounded cold branch.
Full capture, passive opening/reset and a fresh complete trajectory remain
unqualified. [Normal 1× GIF](../media/m8_table_supported/diagnostics/crest_seat_search_v5_closed/render/demo.gif),
[MP4](../media/m8_table_supported/diagnostics/crest_seat_search_v5_closed/render/demo.mp4),
[actual thread detail](../media/m8_table_supported/diagnostics/crest_seat_search_v5_closed/render/endpoint_detail.png)
and [dense native trajectory](../media/m8_table_supported/diagnostics/crest_seat_search_v5_closed/scientific_plot/native_closed_trajectory.png)
are separate from the immutable stopped-prefix record.

The packet's SHA256SUMS identity is
`ba36ae7d8759dcce7bf68c1835d124d9b92986c5e16ebe891f8d809adae4e2e1`.
Its standard-library restorer preserves **all 31 original native files**, including
all dense force ledgers and the **211,454,565-byte / 35-column** executed
inertia-command ledger. Five contiguous chunks, at most 45 MB each, reconstruct
that ledger losslessly, with whole-file SHA checks; no solver is needed for
byte verification.
From a checkout containing the published packet:

```sh
python media/m8_table_supported/diagnostics/crest_seat_search_v5_closed/restore_original_run_layout.py --verify-only
python media/m8_table_supported/diagnostics/crest_seat_search_v5_closed/restore_original_run_layout.py --output outputs/m8_table_supported/crest_v5_received_original
```

The frozen [independent audit binding](../media/m8_table_supported/diagnostics/crest_seat_search_v5_closed/independent_audits/independent_audit_binding.json)
`3fb45a3725ecd1db732568dec664cfbad678981063a0d0b8abfed6e7d79ff1cc`
verifies all **147,132 commands**, **147,130 exact previous-cache velocity
links plus two initialized rows**, the 50 original force columns, clocks,
stopped windows and events. It reconstructs 1,204 thread, 43,384 table and
11,821 left contact-frame sums. Individual right-pad local records were not
archived, so original aggregate right wrenches/pad loads remain source-bound
consistency evidence. PD wrench is the original recorded input to the mapping
audit; unavailable every-tick target/error terms are not independently recomputed.
Forces/geometry remain at `t−dt`, saved qpos/qvel at `t`, retained command
inputs at `t−2dt` after startup. The equal-model historical passive-property
report binds identical XML/ZIP/runtime only, not a new native force solve.
Its **31 pure reader regressions**, **67+6 experimental controller tests**,
historical **69/50 contracts** and canonical **65-file/402 proof** stay separate.

Use the [exact isolated native-repeat recipe](../media/m8_table_supported/diagnostics/crest_seat_search_v5_closed/README.md#repeat-the-exact-cold-native-branch):
pin complete producer 6e7, copy BOTH newer complete packets, restore all
34 original full-parent files and put `crest_seat_inertia_probe_v5.py`,
`crest_seat_observer_v3.py`, `reverse_brake_v4.py` and
`robot_inertia_feedforward_v5.py` directly under
`outputs/m8_table_supported/diagnostics/` (`ROOT=parents[3]`). The parent is
original full row 2651 at 13.1949 s, with inherited cumulative grip references;
no earlier abort endpoint, solver warmstart or old force windows are used.
The explicit cold flags are `--checkpoint-time 13.1949 --brake-duration .15`
with the restored parent and an absent output. `--prepare-only` checks the
archived model and coherent finite commands without integration; a fresh
native repeat integrates another cold branch; the original cost about 19
minutes here.
It does not prove fresh pickup or continuous task completion.

The [closed recorded-state replay/replot recipe](../media/m8_table_supported/diagnostics/crest_seat_search_v5_closed/README.md#replay-and-replot-preserved-evidence)
uses reassembled archived `render_crest_seat_search_v5_closed.py`,
`--repository-root "$PWD" --mode closed`, restored original native files and
a new output in the pinned complete checkout/runtime. It selects 89 exact
saved states at 12 fps (7.4167 s encoded), with no interpolation, integration,
collision discovery, `mj_forward` or force solve. The historical audit reader's
absolute path/sibling assumptions remain provenance; this portable media
recipe does not claim a relocated auditor adapter. The older
[stopped-prefix replay](../media/m8_table_supported/diagnostics/crest_seat_search_v5_stopped_progress/README.md#verify-and-replay-this-recording)
keeps its separate `--mode progress` and sparse-at-capture scope.
