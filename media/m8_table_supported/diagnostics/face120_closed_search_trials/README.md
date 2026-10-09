# Three closed cold M8 search trials

These are **separate cold local experiments** after the published native
pickup/cone-entry prefix. They do not form a continuous warmstarted trajectory.
Original solver state and warm starts are absent at each declared cold
initialization. Every original guard failure, raw force/state row, log and
source byte is retained. **All three have zero formed overlap and zero actual
loaded interior-flank steps; none tests opening or a passive reset.**

| Trial | Native time | Original outcome | Media |
| --- | ---: | --- | --- |
| Reverse seat v1 | 3.55075 s | Guards hold; measured-drop stop fails the original continuous-load direction criterion; no forward/opening | [MP4](face120_seat_search_v1/render/demo.mp4), [GIF](face120_seat_search_v1/render/demo.gif) |
| Closed forward v1 | 1.26175 s | Aborts table_original_load and active_table_rolling_share; yaw-counter origin issue is preserved with separate correction | [MP4](face120_closed_forward_v1/render/demo.mp4), [GIF](face120_closed_forward_v1/render/demo.gif) |
| Closed forward hold v2 | 2.53060 s | Finite 100-micrometre left-down target and stronger right jaws; aborts bilateral_left_pad_load | [MP4](face120_closed_forward_hold_v2/render/demo.mp4), [GIF](face120_closed_forward_hold_v2/render/demo.gif) |

![Actual shallow cold entry and table clamp](face120_seat_search_v1/render/entry_detail.png)

Each clip uses exact archived qpos/qvel rows, two fixed real camera views and
normal 1x playback. Native `mj_forward` replays geometry only: no integration,
interpolation, manual free-body posing, changed geometry or body hiding.
Caption forces come from original native solve samples. Those solved forces
and retained kinematics occur at logged time minus dt; saved qpos/qvel are the
postintegration state. GIF centisecond durations preserve the 12-fps video
time within one centisecond. Original state-index/time hashes are in each
`render/render_manifest.json`.

The [published native prefix](../../face120_pickup_entry_v1/README.md) is the
canonical **model and original grasp-reference parent** (trace SHA
`069e6f53f97507c1311d69a41ea5cea9a85ee3eba4879b55655c29ea91fbc5c0`).
Reverse-seat v1 initializes from its final saved 6.3983-second state. Both
forward branches instead initialize from the **reverse-seat v1 final state**
at 9.94905 s (checkpoint trace SHA
`0d87e71cc5f85a1210d5b6e3fc43484b6f48d0ecc08ae1e3f941d74acbc5ac3c`).
The prefix's original measured grip references remain unchanged. Cold state
initialization does not redefine the grasp reference or imply that the failed
direction criterion passed. The exact declared qpos/qvel/ctrl, input-state
hashes and reconstruction details are preserved in every branch.

In forward v1, the original instrumentation initialized its unwrapped yaw
counter from the canonical prefix rather than the actual cold input yaw.
That produces a **constant origin offset** in the original counter. Its
original source, reports, logs and ledger remain unchanged. Use
`original_yaw_origin_note.json`,
`independent_native_force_audit_corrected_yaw_scope.json` and
`independent_phase_depth_audit_corrected_yaw.json` in that trial for the
separately verified quaternion-based correction. The initial generic
`independent_native_force_audit.json` and `audit_draft_sources/` are explicitly
superseded provenance. Actual rendered poses come directly from archived
native qpos, so the counter-label issue does not alter their orientation.
Forward v2's raw counter starts from the actual cold quaternion origin; its
independent audit verifies that no correction is applied.

The nominal deepest overlap is about **1.58814 mm** in forward v1 and
**1.61319 mm** in v2, below the independent common formed-span threshold of
**1.992582 mm**. These centroid/coaxial profile quantities are geometric
context, not a full tilted bore-fit proof, an imposed helix, a calibrated lead
or an isolated explanation of the failures. Actual full-ring overlap and
loaded interior contact remain zero.

V2's 100-micrometre downward **robot target** generates real native contact
compression; it does not directly move the free block. Its independently
retransformed final 100 ms has mean signed left-hand upward force **-1.950 N**
and mean table compression **3.227 N**. A table load greater than block weight
denotes extra compression, not a percentage of gravity support. The original
last 50-microsecond tick has one unloaded left pad and triggers the retained
abort. All original native and commanded-motor forces remain distinct.

The canonical app/helper source is pinned to
**b2b13ff39cd47c48afd19b38f83e9a405c9d6e32**, with its unchanged **281-test**
software proof. That proof does not cover these output-only cold harnesses or
qualify native capture/full assembly. Separate output-only auditor arithmetic
proofs remain distinct: their counts are not combined with 281. Independent
audit bindings preserve each original source, raw input and correction lineage.

## Verify and inspect

Every artifact is smaller than 45 MB, so no lossless chunks are needed. From
this package directory:

```sh
sha256sum -c SHA256SUMS
```

Each trial contains complete `original_native_force_ledger.npz`,
`checkpoint_trace.npz`, `declared_cold_initialization.npz`, `scene.xml`,
`supported_scene.zip`, `recorded_sources/`, `diagnostic_source.py`,
`observer_source.py`, original report/declaration/log, independent reports,
bindings and exact auditor sources. The package-level canonical validation and
reference binding archive the original prefix grip references explicitly.
Model/runtime and all media/state/source identities are in the manifests.

## Repeat the exact cold inputs

Use an isolated checkout of the pinned source commit with the matched native
CPU engine from `docs/m8_setup.md`. The stock MuJoCo wheel is insufficient.
Core/plugin hashes, pinned engine patch and exact source snapshots must match;
an independent rebuild requires comparison before claiming identical physics.

The evidence packages were published after the producer commit and are absent
from its checkout. Start with the newer review checkout at
`/workspace/astra-r2s`, containing both complete packages. Clone into an unused
directory and restore those exact packages into ignored outputs; copying only
an NPZ omits the adjacent scene/source files required by the model loader.
Reuse the verified runtime, or run `scripts/setup.sh` in the pinned checkout
first if it is absent:

```sh
git clone https://github.com/robomechanics/astra-r2s.git /workspace/astra-r2s-supported-cold-b2b13ff
git -C /workspace/astra-r2s-supported-cold-b2b13ff switch --detach b2b13ff39cd47c48afd19b38f83e9a405c9d6e32
cd /workspace/astra-r2s-supported-cold-b2b13ff
mkdir -p outputs/m8_table_supported/cold_inputs
cp -a /workspace/astra-r2s/media/m8_table_supported/face120_pickup_entry_v1 outputs/m8_table_supported/cold_inputs/prefix
cp -a /workspace/astra-r2s/media/m8_table_supported/diagnostics/face120_closed_search_trials outputs/m8_table_supported/cold_inputs/trials
(cd outputs/m8_table_supported/cold_inputs/prefix && sha256sum -c SHA256SUMS)
(cd outputs/m8_table_supported/cold_inputs/trials && sha256sum -c SHA256SUMS)
```

The harness repository root lookup is `Path(__file__).resolve().parents[3]`.
Copy each frozen source **directly into outputs/m8_table_supported/diagnostics/**,
not a deeper packaged subdirectory. Run from the pinned repository root:

```sh
mkdir -p outputs/m8_table_supported/diagnostics
cp outputs/m8_table_supported/cold_inputs/trials/face120_seat_search_v1/diagnostic_source.py outputs/m8_table_supported/diagnostics/seat_search_probe.py
cp outputs/m8_table_supported/cold_inputs/trials/face120_closed_forward_v1/diagnostic_source.py outputs/m8_table_supported/diagnostics/closed_forward_probe.py
cp outputs/m8_table_supported/cold_inputs/trials/face120_closed_forward_hold_v2/diagnostic_source.py outputs/m8_table_supported/diagnostics/closed_forward_hold_probe.py
```

Use fresh output paths. The canonical input trace below has its adjacent exact
`scene.xml`, `supported_scene.zip` and original source archives in the restored
prefix directory. Every harness rejects mismatched canonical source
dependencies or helper bytes. The original helper source is
`yam_twin/m8_supported_start.py`, SHA
`ce9c3c2272b475167108ff2d25e568125d6378e64af7a0eadc1fe2219be656f7`.

```sh
scripts/run_m8.sh outputs/m8_table_supported/diagnostics/seat_search_probe.py --parent outputs/m8_table_supported/cold_inputs/prefix/insertion_trace.npz --output outputs/m8_table_supported/diagnostics/repeated_seat_v1
scripts/run_m8.sh outputs/m8_table_supported/diagnostics/closed_forward_probe.py --parent outputs/m8_table_supported/cold_inputs/prefix/insertion_trace.npz --state-source outputs/m8_table_supported/cold_inputs/trials/face120_seat_search_v1/checkpoint_trace.npz --output outputs/m8_table_supported/diagnostics/repeated_forward_v1
scripts/run_m8.sh outputs/m8_table_supported/diagnostics/closed_forward_hold_probe.py --parent outputs/m8_table_supported/cold_inputs/prefix/insertion_trace.npz --state-source outputs/m8_table_supported/cold_inputs/trials/face120_seat_search_v1/checkpoint_trace.npz --output outputs/m8_table_supported/diagnostics/repeated_forward_v2
```

Defaults are the original archived commands: reverse limit 1.9 rad, reverse
peak speed 2 rad/s; forward peak speed 1 rad/s; B200 axial damping. **Only
reverse v1** executes the 0.05 N to 0.262414 N bolt-weight feed ramp over 0.5 s
after 0.15 s centering, with a 0.5 s hold. **Both forward trials hold net feed
at 0.262414 N from their first cold native step**; their retained declaration
fields for 0.05 N starting feed and 0.5 s ramp/hold are legacy values and do
not describe an executed forward feed ramp. V2 additionally uses 0.3 s left
preload with a 100-micrometre target offset, 0.3 s alignment, 0.3 s prehold
and 0.0184 m stronger right aperture. Exact argv, bounds and
phase definitions remain in each declaration and frozen source. Each forward
command intentionally uses the **archived original reverse checkpoint**, not
a newly generated result. A changed input generates another distinct cold
experiment and must retain a new input-state identity. Repeating these commands
does not splice the local branches into a continuous native trajectory.

Live v3 is outside this frozen package. The first carried-block demonstration,
published prefix, earlier failures and other cold diagnostics remain unchanged.
