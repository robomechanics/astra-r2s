# Agent handoff: reproduce the completed M8 pickup trajectory

The preserved first demonstration is a native MuJoCo CPU trajectory: the left
YAM picks up and carries a free female-threaded block; the right YAM picks up a
separate M8 × 1.25 bolt, starts the thread and completes two qualifying
half-turns with real opening, reset and regrasp. Keep its evidence in
[`media/m8_table_pickup/full`](../media/m8_table_pickup/full) unchanged.
The proposed table-supported stabilization trajectory is a separate task and
must use separate source entry points, outputs and evidence directories.

## Start with the preserved result

- [Normal 1× video](../media/m8_table_pickup/full/demo.mp4),
  [GIF](../media/m8_table_pickup/full/demo.gif),
  [screenshot](../media/m8_table_pickup/full/demo.png) and
  [measured trajectory](../media/m8_table_pickup/full/trajectory.png).
- [Original validation](../media/m8_table_pickup/full/validation.json):
  `partial=false`, `aborted=null`, 49 executed phases, 43.63610 simulated seconds,
  **22/23 checks passed and overall `passed=false`**.
- The sole failed check is `all_substep_left_pad_contact_retention`: the
  original 842,723 post-acquisition steps contain 9 / 6 isolated unloaded
  50 µs steps at the left pads, totaling 450 / 300 µs. The coarse sampled pad
  check passes. Preserve both outcomes; they describe different time coverage.
- Actual qualified travel is 1.245488 mm over 1.000085 measured revolutions.
  Half-turn advances are 622.591 / 622.897 µm and signed pitch residuals are
  −2.462 / −2.156 µm. Captured open resets have zero hand/bolt, world-support
  and head-seating contacts, with 0.178 / 0.424 µm peak axial drift.
- The head remains unseated. This is an idealized contact demonstration, not
  calibrated hardware, full tightening/preload or a trained policy result.

The [task description](yam_m8_insertion.md) explains the complete scene,
controls and audit scope. The [physics investigation](m8_insertion_physics.md)
and [status](m8_status.md) retain numerical limitations and failed attempts.

## Pin the evidence and source revision

The publication commit is
`9b1444957427debb2fbdf90962eb3a16f6ab7757`. It contains the closed evidence,
the current **181-test** source proof, the demonstrated 1 rad/s starting-speed
default and the later policy reward correction. Its
[zero-step comparison](../media/m8_table_pickup/default_configuration_match.json)
confirms scene/control equivalence with the completed run; it does not add a
new physics run.

The native run and its renderer actually used the earlier source commit
`37caa12022d245fd3831c8b62397eb1f90eece23`. Every one of the historical
**176-test** proof's 31 source hashes matches that commit. The run explicitly
selected 1 rad/s; that revision's implicit table default was 0.5 rad/s.
Use the historical revision plus explicit parameters for a fresh run with
the exact executed controller source. Use the publication revision for the
complete archived evidence and later policy interface. Do not combine their
software proof scopes.

For a clean checkout, use an unused destination:

```bash
git clone https://github.com/robomechanics/astra-r2s.git /workspace/astra-r2s
cd /workspace/astra-r2s
git switch --detach 9b1444957427debb2fbdf90962eb3a16f6ab7757
```

An existing checkout can replay/audit the evidence directly. Do not switch a
shared checkout while another agent is developing the second trajectory.
If the exact historical source is required, create an independent checkout:

```bash
git clone /workspace/astra-r2s /workspace/astra-r2s-m8-176
git -C /workspace/astra-r2s-m8-176 switch --detach \
  37caa12022d245fd3831c8b62397eb1f90eece23
```

The historical checkout predates the complete evidence directory. The replay
examples below therefore pass an absolute path to the preserved publication
checkout. Both checkouts retain the necessary YAM mesh assets.

## Set up the native runtime

Run from the publication checkout. The supported cloud layout is `/workspace`
with Python 3.12, `uv`, Git, GCC/G++, network access and Mesa EGL. A fresh core
and binding build takes longer than the verified repeat setup.

```bash
cd /workspace/astra-r2s
scripts/setup.sh
scripts/run_m8.sh -c 'import mujoco; print(mujoco.__version__)'
scripts/run_m8.sh -m yam_twin.m8_insertion_demo --help
```

The setup entry point is `scripts/setup.sh`; there is no
`scripts/bootstrap_m8.sh`. It uses the hash-locked dependencies and builds a
matched GCC MuJoCo 3.15.0 core **and Python extensions**. The core is based on
upstream `9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5`, with the declared
100 nm SDF search floor and 2 mm initial search step. The thread plugin
provides geometry, not screw forces or a kinematic helix.

Always launch through `scripts/run_m8.sh`. It selects
`/workspace/.venvs/m8-contact`, verifies the core/extensions and ABI, clears
`LD_PRELOAD`, sets workspace caches and uses EGL. Preloading the GCC core
into the stock PyPI bindings is unsafe because their `MjSpec` C++ ABI differs.
The stock `/workspace/.venvs/astra-r2s` and MuJoCo 3.11 MJLab bridge are separate
legacy runtimes. This trajectory uses neither MJLab nor CUDA nor Newton.
See [setup/provenance](m8_setup.md) for advanced runtime path overrides.

The recorded core SHA-256 is
`58039d439c6504448aafd0a4d5b655c8a0d3bf1d77123ac7e0733cd078367433`.
The recorded plugin **source** SHA-256 is
`1c8b5207c5f6c141cc034983ce76c6c1e9cca4114d16e17a4496b94cb637b42a`.
The latter is not the compiled `.so` hash. A separately rebuilt core can have
a different binary hash; preserve that distinction and inspect the audit's
runtime identity result before claiming the exact recorded runtime.

## Verify the immutable archive

```bash
cd /workspace/astra-r2s/media/m8_table_pickup/full
sha256sum -c SHA256SUMS
cd /workspace/astra-r2s
```

All 45 listed files must verify. `manifest.json` binds the primary evidence;
`supplemental_manifest.json` binds extra audits, exact sources, historical
software proof and progress-to-final links. `source_identity.json` separates
whole-module hashes from the selected controller-definition bundle. Important
archive identities are:

| Artifact/identity | SHA-256 |
| --- | --- |
| `trace.npz` | `c25daaf438f36c8b1617c989c8441b0414305a1c90648b397fb096cab484b9db` |
| `validation.json` | `c2647298f59e7b8d832ac140df243a26d4d403ac62e0de2353ab0760da693a3d` |
| `scene.xml` | `19dd6fcd1081b03ad1c5c584be4a4635b03b32ab5ffa183bbc5b1c71ce2e847e` |
| Compiled model fingerprint | `d8ef2669aff4b4e871d1e09bed2a00e7482649c554bcfce5b7fb234e220bf635` |
| `controller_source.py`, whole module | `199ac49380e95b5849caf98e6bbfda1c0e85d1626fc97e468a421463a03394e8` |
| Selected controller-definition bundle | `dce8a1e1a8bc53d6caf4b717bf846a04fee1c5ebfafac9428b94f8e705385121` |
| `scene_source.py` | `f79d8878e9f3bd661b89bd4472eeac91d994c71bd742f7d544c79fb9ff16680d` |
| `engagement_observer_source.py` | `d9530e53b5570a74c01de52fb8cf5261c0bc7b685427b7965f0d1b8c888ad866` |
| Actual archived renderer module | `fcf1cb1ce92bfcbe1a91777af1a92ee49fa2bd97a6c5f6225cfb40c03a06d6a9` |
| `left_pad_force_history.npz` | `ddaa48aa5ee026aa5c2ef4ef5bdc4b9183c0e79fb72c9340c7873ab1c091150f` |

These hashes identify the preserved archive, not guaranteed byte identities
for a fresh run on a different machine. Mesh hashes are in the manifest;
the XML requires those repository assets and the verified native plugin.
Fresh XML can contain different absolute checkout paths even when its
path-independent model fingerprint is equal.

## Replay quickly, without integrating physics

Use a new ignored output directory. This compiles the exact hash-verified
archived XML, assigns actual saved qpos/qvel and calls `mj_forward` for each
selected frame. It performs no integration, interpolation or object posing.

```bash
cd /workspace/astra-r2s
scripts/run_m8.sh -m yam_twin.m8_insertion_demo \
  --replay media/m8_table_pickup/full/trace.npz \
  --output outputs/m8_table_pickup/agent_replay \
  --fps 12 --slow-motion 1 --still-time 43.6361
```

Expected files are `insertion_demo.mp4` and `insertion_demo.png`. The video
selects 524 actual states at 12 fps, with approximately 43.67 s playback.
For a quick final screenshot, append `--stills-only`. Replaying a failed
validation can exit zero because it completed rendering; that does not change
the original physics result.

For the **exact archived renderer source**, run the same module in the
historical checkout using the publication trace:

```bash
cd /workspace/astra-r2s-m8-176
scripts/run_m8.sh -m yam_twin.m8_insertion_demo \
  --replay /workspace/astra-r2s/media/m8_table_pickup/full/trace.npz \
  --output outputs/m8_table_pickup/agent_archived_replay \
  --fps 12 --slow-motion 1 --still-time 43.6361
```

Do not execute `render_sources/renderer_source.py` directly as a script: its
relative package imports require the module context supplied above.

## Recompute independent audits safely

The archived audit scripts preserve the exact checker source. **Always supply
`--output` outside the canonical package**: their omitted-output defaults
would overwrite an archived audit JSON next to the input trace.

```bash
cd /workspace/astra-r2s
mkdir -p outputs/m8_table_pickup/agent_audits
scripts/run_m8.sh media/m8_table_pickup/full/audit_sources/audit_m8_insertion_capture.py \
  media/m8_table_pickup/full/trace.npz \
  --output outputs/m8_table_pickup/agent_audits/capture.json
scripts/run_m8.sh media/m8_table_pickup/full/audit_sources/audit_m8_insertion_reset_contacts.py \
  media/m8_table_pickup/full/trace.npz \
  --output outputs/m8_table_pickup/agent_audits/reset_contacts.json
scripts/run_m8.sh media/m8_table_pickup/full/audit_sources/audit_m8_free_joint_properties.py \
  media/m8_table_pickup/full/trace.npz \
  --output outputs/m8_table_pickup/agent_audits/free_joint_properties.json
scripts/run_m8.sh media/m8_table_pickup/full/audit_sources/audit_m8_left_pad_force_history.py \
  media/m8_table_pickup/full/trace.npz \
  --output outputs/m8_table_pickup/agent_audits/left_pad_force_history.json
```

Capture auditing also runs saved-pose geometry and raw-force consistency
checks. Reset auditing inspects 296 saved poses in each of four open resets.
The free-body audit checks zero workpiece damping, friction loss, armature,
springs, gravity compensation, fluid terms and actuators. Only 16 native
robot actuators and two finger-coupling equalities are present.

The raw-force audit must report consistency **true** and continuous bilateral
preload **false**, reproducing 9 / 6 gaps. Audit completion/exit status is not
a substitute for reading its JSON outcomes. Saved-pose geometry cannot
certify unsaved substeps or reconstruct original solved forces. Native force
diagnostics belong to the pre-integration state at recorded time minus dt;
saved qpos/qvel are post-integration.

The archived entry-command checker is a run-directory helper with original
filenames and writes its result into that directory. Inspect its source and
the preserved `independent_entry_command_audit.json`; do not point it at the
immutable package. Its normal impulse is not axial weight balance or capture
proof, and sparse poses cannot reaggregate its unsaved native force windows.

## Fresh integration of the first trajectory

The recorded run took **3086.6 s (51.4 min)** on this CPU workspace, alongside
another native trial. Rendering and audits are additional work. A fresh run
can take tens of minutes or longer; a healthy CPU process can remain inside
one long starting stroke without producing a new phase checkpoint.

The following explicit settings reproduce the completed configuration on
either the publication source or the exact historical source. Use a new
output directory, and leave both completed/failure archives untouched.

```bash
cd /workspace/astra-r2s
scripts/run_m8.sh -m yam_twin.m8_insertion_demo \
  --output outputs/m8_table_pickup/agent_fresh_run --dt .00005 \
  --stroke-degrees 180 --angular-speed 2 --starting-angular-speed 1 \
  --maximum-starting-strokes 5 --qualifying-strokes 2 \
  --maximum-entry-dwell 3 --axial-damping 50 \
  --fps 12 --slow-motion 1 --video
```

For exact historical-controller reproduction, replace the first `cd` with
`cd /workspace/astra-r2s-m8-176`. That checkout's launcher uses its own package
source from the working directory and the shared verified runtime; another
setup/editable reinstall is unnecessary. The current source has the same
effective scene/control defaults but different whole-source IDs. The policy
reward correction does not run in this scripted demonstration.

The final CLI exits **1** when the strict preload check fails even if all
49 physical phases complete. Diagnose `insertion_validation.json`:
`partial=false` alone is insufficient; require `aborted=null`, actual capture,
both complete `turn_1`/`turn_2`, qualified metric lead and captured unsupported
resets. A fresh result may differ and must retain its actual failures. Do not
alter guards to recover an expected count.

The run writes exact initial source/XML archives, phase-end
`insertion_trace_partial.npz`, then final `insertion_trace.npz`, original
`insertion_validation.json` and `left_pad_force_history.npz`. Partial phase
checkpoints are progress evidence, not completed acceptance. The completed
configuration has three starting strokes, then two qualified strokes; the
first two search resets have only partial formed flanks below one full pitch.
Candidate full-flank capture occurs at 31.3403 s in the original all-substep
report and 31.34495 s in its first saved tagged row.

Freeze source files while a fresh native run is active. Preserve source IDs,
the failed report and the raw force history before packaging. Recompute the
independent audits against the final closed trace. An overall-failed complete
sequence requires `--evidence-only` packaging into a **new** target; never
replace `media/m8_table_pickup/full`. See
`scripts/package_m8_insertion_demo.py --help` via the verified launcher.

## Software checks and boundaries for further work

```bash
cd /workspace/astra-r2s
scripts/run_m8.sh -m pytest -q
```

At publication commit `9b14449`, expect 181 tests; at historical commit
`37caa12`, expect 176. Later second-trajectory work can increase the current
count. Keep each source hash/log proof attached to its own revision. Software
tests, configuration equality, rendered appearance and physics checks answer
different questions.

The 14-action, 118-observation policy interface uses actual bounded robot
torque/jaw actions and no scripted pickup controller. Its success flag does
not enforce the demonstration's open-release/reset sequence or strict
zero-gap left-pad criterion. No learned policy has been qualified.

Do not add workpiece motors, welds, hidden support, an imposed axial helix,
positive collision margins or relaxed depth/lead/grasp guards to match this
video. Pad compliance remains numerical rather than calibrated rubber.
The isolated thread fixture still fails its peak depth/search refinement
comparison by 45.02%, although its strict travel/lead refinements pass.
The 0.5 rad/s conservative full attempt aborts in its second qualifying turn
at the unchanged 1 mm bolt-grasp slip guard; it is not a successful slower
comparison. Earlier preload, clearance and starting failures remain archived.

For the second approach, real table support and left-arm stabilization need
their own support/load/contact checks. The first trajectory's zero world
support after block lift is a requirement for **this carried-block run**;
keep it intact rather than repurposing its acceptance report for a block that
remains on the table.

## Handoff verification performed

All 45 canonical checksums and all 31 historical source hashes were verified.
The launcher and all four archived audit CLIs accepted the documented flags.
The free-joint audit reproduced zero artificial workpiece terms, and the
raw-force audit reproduced both consistency and the retained 9 / 6 preload
gaps. A final-state still was replayed with both the publication renderer and
the exact historical source in an isolated source tree. Both PNGs matched the
published `demo.png` byte for byte, SHA-256
`672be32ca945a4d67a8889089101d7b968e324b531b25fdc82a2dab49f303576`.
These checks integrated no physics and did not rerun the long trajectory.
All canonical checksums were verified again afterward.
