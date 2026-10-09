# Bimanual YAM M8 contact tasks

**Both robots now physically pick up their workpieces.** The left YAM arm
reaches for the female-threaded block on the table, clamps it, lifts it and
rotates it into the assembly pose. The right arm picks up the separately
supported M8 × 1.25 bolt, finds the thread, releases/regrasps and completes
two qualified half-turns. One measured revolution advances **1.245488 mm**;
the two pitch residuals are **−2.46 / −2.16 µm**. Native MuJoCo CPU contact
drives this motion, with no grasp welds, imposed helix or free-object drive.

[![Complete physical block and bolt pickup, thread capture and turning at normal playback](media/m8_table_pickup/full/demo.gif)](media/m8_table_pickup/full/demo.mp4)

[Normal-speed MP4](media/m8_table_pickup/full/demo.mp4) ·
[Screenshot](media/m8_table_pickup/full/demo.png) ·
[Raw trajectory and original checks](media/m8_table_pickup/full) ·
[Measured motion/contact chart](media/m8_table_pickup/full/trajectory.png) ·
[Controls, audits and limits](docs/yam_m8_insertion.md)

The continuous **43.6361 s** rollout completes all 49 executed phases without
an abort. **22 of 23 checks pass; the original overall result remains false.**
The strict continuous left-pad preload check records 9 / 6 isolated 50 µs
unloaded steps during acquisition/handling, totaling 450 / 300 µs. The raw
842,723-step force history preserves that failure. The 43.67 s video plays
at normal 1× speed; it selects actual recorded states.

The block is physically lifted **59.965 mm** from the table. Both captured
open resets have zero hand/bolt, world-support and head-seating contacts,
with peak axial drift **0.178 / 0.424 µm**. Independent saved-pose audits
check all four resets. The head remains unseated; full tightening/preload,
hardware calibration and broad policy-training fidelity remain unqualified.

**181 current software tests pass.** The completed native trial retains its
separate historical 176-test source proof. Policy success does not enforce
the demo's open-release/reset sequence or strict zero-gap left-pad criterion;
a success flag does not certify those checks.

The demonstrated starting speed is now the table default, **1 rad/s**, with
qualified strokes at 2 rad/s. The explicit 0.5 rad/s
[slower trial](media/m8_table_pickup/failures/conservative_second_turn_grasp_abort)
aborts during its second turn at the unchanged 1 mm grasp-slip guard; it is
failed evidence, not a successful speed comparison. Earlier progress clips
and failed starting/contact pilots remain linked in the task documentation.

The isolated thread fixture passes its strict travel/lead refinements, but
its peak reported depth changes **45.02%** with contact-search refinement.
Older load/search failures also remain. See
[the numerical comparison](media/m8_insertion/refinement/README.md) and
[mechanics status](docs/m8_status.md) before using this for training.

## Earlier left-touching block rollout

The [earlier bolt-pickup demo](media/m8_insertion/full/demo.mp4) starts with
left pads already touching the free block. Its separate raw report passes
all 18 nominal checks: one revolution advances 1.245793 mm. It remains valid
for that initial condition and does not establish tabletop block pickup.
Its 126-test source snapshot and evidence remain archived.

## Verified turning baseline

Two actual YAM arms hold a free mounting block and turn a larger nut on its
short M8 shaft. The nut has a 20 mm outer width and the same M8 × 1.25 bore;
the shaft is 16 mm long. Native joint motors and finite finger contacts drive
the task. See [scene, controls, policy interface and current results](docs/yam_m8.md).

[![Complete recorded physical turning demo](media/yam_m8/demo.gif)](media/yam_m8/demo.mp4)

The [complete three-stroke demo](media/yam_m8/demo.mp4) passes all
15 nominal checks: measured lead errors are **2.9, 3.1 and 3.1 µm**, and both
open resets have zero hand/nut contacts. The left arm lifts the free assembly
**3.829 mm**, with zero world supports or artificial object forces. See the
[raw rollout](media/yam_m8/demo_trace.npz),
[checks](media/yam_m8/demo_validation.json), and
[independent audit](media/yam_m8/demo_independent_audit.json).
Firmer left-pad contact reduces post-clamp block slip to **3.52 µm**;
[the original softer-contact comparison](media/yam_m8/contact_sticking_comparison.json)
retains its results and identical thread/controller settings.
**97 baseline software tests pass**; nominal demo checks do not certify policy-training
fidelity.

```bash
scripts/run_m8.sh -m yam_twin.m8_demo --output outputs/yam_m8/demo \
  --dt .00005 --angular-speed 1 --video
```

## Independent M8 mechanics experiment

A video-informed M8 × 1.25 experiment with a free six-DOF nut, continuous
60° helical collision surfaces, and finite-force frictional fingers. Rotation
and axial travel emerge from contact. The thread plugin supplies geometry only;
the nut has no motor, grasp weld, or prescribed screw joint.

[![Recorded M8 contact demonstration](media/m8_contact.gif)](media/m8_contact.mp4)

**[Watch the two-turn contact demo](media/m8_contact.mp4)** ·
**[Measured results](media/m8_gripper_validation.json)** ·
**[Acceptance status and remaining failures](docs/m8_status.md)**

Download the raw MP4 from GitHub if its file page does not play it. The video
replays recorded simulated states at 3× slow motion; its blue pitch line is a
comparison reference. The nut advances **1.249786 mm and 1.249902 mm** on its
two turns. During the open-hand 360° reset, there are zero hand/nut contacts
and **13.46 nm** of nut drift. This demonstrates contact-driven running and
release/regrasp in the tested condition.

**Physics qualification is incomplete.** The complete load suite passes
13/19 individual diagnostics and retains failures in low-friction local lead,
high-friction speed fluctuation, and some scalar analytical torque comparisons.
The free-body suite also misses its strict contact-search convergence gate.
Read the failed checks before using this for policy training.

The nut starts engaged on a fixed bolt. A proxy parallel hand replaces the
YAM arms for this mechanics experiment. M8 dimensions, steel density, and
μ=0.15 are declared assumptions; the videos do not provide measurements of
tolerances, forces, or friction. This older trace contains no pickup or thread
starting. Calibrated hardware contact, seating/preload and a trained bimanual
policy remain unfinished.

## Run

The tested cloud runtime is Python 3.12 on CPU with Mesa EGL. Setup builds a
pinned MuJoCo 3.15.0 core and matching Python bindings; two documented SDF
search constants are changed for micron-scale geometry. Use the verified
launcher, which checks the engine and C++ ABI before execution.

```bash
cd /workspace/astra-r2s
scripts/setup.sh
scripts/run_m8.sh -m pytest -q
scripts/run_m8.sh -m thread_lab.demo --output outputs/m8/demo
scripts/run_m8.sh -m thread_lab.render outputs/m8/demo/gripper_trace.npz \
  --fps 12 --slow-motion 3 --output outputs/m8/demo/contact.mp4
```

The demo commands finite forces/torques on the hand and permits axial floating
during rotation. It logs per-step validity gates and measured pad forces, and
exits unsuccessfully if the demo checks fail. CPU contact search is expensive;
the recorded demo simulated about 5 seconds. Rendering reuses the trajectory.

```bash
scripts/run_m8.sh scripts/check_m8_distance.py
scripts/run_m8.sh scripts/check_m8_optimization.py
scripts/run_m8.sh -m thread_lab.validate --output outputs/m8/acceptance
scripts/run_m8.sh -m thread_lab.load_benchmark --help
```

`thread_lab.validate` retains strict physics gates and can exit with failure.
The current source passes **181 software tests**. Software tests, numerical geometry checks,
and physics acceptance answer different questions.
[Setup and engine provenance](docs/m8_setup.md) explain the separate runtimes.

## Policy interface and evidence

`thread_lab.env.M8NutEnv` is a Gymnasium interface with seven normalized
hand-wrench/jaw actions, bounded physical actuation, 42 privileged state
observations, and a 25 µs default physics timestep. Invalid depth, alignment,
phase, or solver states terminate with reward −1. Short action stress probes
check these guards; they do not establish long-horizon training stability.
See [hand/controller details](docs/m8_gripper.md).

- [Load tables, failed gates, and raw traces](media/m8_load/README.md)
- [Free-body checks](media/m8_free/validation.json)
- [Thread geometry, analytical predictions, and investigation](docs/thread_mechanics_research.md)
- [MuJoCo contact investigation](docs/mujoco_thread_research.md)
- [Exact geometry optimization and reproducibility](docs/m8_sdf_performance.md)
- [Newton evaluation](docs/newton_thread_research.md): its official thread
  example requires CUDA; this CPU environment did not establish a Newton advantage.
- [Observations from the nut video](docs/nut_video_observations.md)

The original dual-YAM M4 reconstruction is preserved under
[legacy demonstration](docs/legacy_m4.md). It uses an imposed helix and grasp
welds and is insufficient evidence of thread physics. The separate MJLab
bridge also represents that legacy scene; GPU policy training was not run.

Use the existing checkout directly in cloud tasks; no worktree or background
service is needed.
