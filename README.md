# Bimanual YAM M8 contact tasks

The current task starts with **both M8 workpieces set aside**: the female
threaded block rests on the table and the male bolt rests on a separate
low support. The left arm must reach, close its fingers, lift the block,
and rotate it into the assembly pose before the right arm picks up the bolt.
The recorded clip now shows both physical pickups and a first starting/opening/
reset/regrasp sequence. **The complete tabletop threading task remains
unfinished.** The damped full attempt stopped before formed-flank capture.
The measured-entry controller now passes **176 software tests**; fresh
continuous runs at 0.5 and 1 rad/s starting speed are running, with full results
pending. See
[task modes, controls and evidence](docs/yam_m8_insertion.md).

[![Faster physical pickup and starting attempt at normal playback; capture pending](media/m8_table_pickup/progress_faster_start/demo.gif)](media/m8_table_pickup/progress_faster_start/demo.mp4)

[Faster pickup/start MP4](media/m8_table_pickup/progress_faster_start/demo.mp4) ·
[GIF](media/m8_table_pickup/progress_faster_start/demo.gif) ·
[Screenshot](media/m8_table_pickup/progress_faster_start/demo.png) ·
[Exact progress sources and scope](media/m8_table_pickup/progress_faster_start)

This actual recorded prefix uses a **1 rad/s starting command and normal 1×
playback**. It ends at 19.0268 s after the first regrasp, before the second
starting stroke. Capture remains false; its partial formed-flank geometry is
below one pitch. Cone contact and this recovery do not qualify thread lead.
The [actual settled-entry screenshot](media/m8_table_pickup/progress_settled_entry.png)
comes from the separate conservative run and also shows entry, not capture.
The [earlier start/recovery clip](media/m8_table_pickup/progress_first_start)
remains archived with the damped failed attempt.

The [preserved damped failure](media/m8_table_pickup/failures/damped_second_release_abort)
stops at 17.32385 s when the second search opening reaches **10.129 µm**
reported depth, exceeding the unchanged **10 µm** guard. Its saved-pose audit
finds zero unexpected penetrating camera/backing contacts; the strict raw
pad-preload check still fails 9 / 6 isolated 50 µs gaps.

The [preserved failed full attempt](media/m8_table_pickup/failures/full_v10_depth_abort)
physically picks up both workpieces, then aborts at thread entry: its depth
proxy reaches 10.562 µm, exceeding the unchanged 10 µm guard. Capture and
qualified turns were not reached. Camera/jaw collisions and isolated pad-load
gaps remain recorded failures. The later damped trial moved the separate bolt
farther aside and added native axial velocity damping, with no imposed screw motion.

[Failed-attempt MP4](media/m8_table_pickup/failures/full_v10_depth_abort/demo.mp4) ·
[GIF](media/m8_table_pickup/failures/full_v10_depth_abort/demo.gif) ·
[Screenshot](media/m8_table_pickup/failures/full_v10_depth_abort/demo.png)

[Initial table layout](media/m8_table_pickup/preview.png) shows the upright
block and separate bolt. This is a static preview, not a completed rollout.
The [physical block-pickup diagnostic](media/m8_table_pickup/failures/roll_3s_normal8ms_friction0p8ms_50us)
reaches the holding pose but retains a failed pad-load check for isolated
50 µs force gaps. It is partial evidence, with no bolt/thread qualification.

The published **earlier bolt-pickup rollout** starts with the left pads
already touching the free block. The right arm physically picks up the bolt,
starts the thread, releases and regrasps, then performs two qualified 180°
strokes. That recorded rollout
passes all **18 nominal checks**: one verified revolution advances the bolt
**1.24579 mm**, with **0.70 and 3.61 µm** lead residuals. Native MuJoCo CPU
contact drives the threads; the free objects have no grasp welds, imposed
helix, object motors or direct external drive.

[![Earlier bolt pickup, capture and threading; left starts touching block](media/m8_insertion/full/demo.gif)](media/m8_insertion/full/demo.mp4)

[Watch/download the MP4](media/m8_insertion/full/demo.mp4) ·
[Raw trajectory and checks](media/m8_insertion/full) ·
[Measured motion and contact loads](media/m8_insertion/full/trajectory.png)

The short shaft is 16 mm long and its solid graspable head is AF20 × 8 mm.
Both captured open resets have zero hand/bolt, world-support and head-seating
contacts; peak axial drift is **0.253 and 0.083 µm**. This establishes the
nominal bolt-pickup-to-running sequence with that initial left grasp pose.
It does not demonstrate acquiring the block from the table. Full seating,
preload, hardware calibration and policy training remain unqualified.

The separate disengaged contact benchmark measures **1.250052 mm/revolution**
after capture. Without a rotation command, the bolt stops at the thread
entry. These are explicitly ideal-fixture tests; their results do not replace
the full robot checks. Timestep and contact-search refinement pass the strict
2% travel/lead comparisons; the peak reported depth remains search-sensitive.
See [the numerical comparison and raw traces](media/m8_insertion/refinement/README.md).
The [current 176-test measured-entry proof](media/m8_table_pickup/software_tests.json)
and [source-bound manifest](media/m8_table_pickup/software_manifest.json)
are separate from physical acceptance. The previous 167-test proof is
preserved with the damped failed attempt. The earlier bolt-pickup snapshot's
126-test proof remains archived with its own sources.

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
The current measured-entry source passes **176 software tests**. Software tests, numerical geometry checks,
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
