# Contact-driven parallel-jaw end effector

`thread_lab/gripper.py` supplies a small physical end effector for the thread
laboratory. It is a proxy parallel-jaw hand, **not a calibrated YAM gripper or a
complete bimanual arm model**. Its purpose is to isolate nut manipulation and
thread contact before expensive robot-policy experiments.

The hand is a free rigid body, with two independently dynamic slide-joint jaws.
Aperture commands drive finite-force position actuators. Rubber-like pads squeeze
the nut's 13 mm flats; Coulomb contact friction transfers both translational
force and torque. There are no grasp welds, adhesion actuators, nut rotation
commands, axial nut drives, or rotation-to-insertion constraints. Only initial
reset writes hand/jaw coordinates. The controller never reads nut insertion or
orientation to make the nut follow the hand.

## Parameters and scope

| Quantity | Default | Basis |
|---|---:|---|
| Palm mass | 60 g | Assumed proxy end-effector |
| Each jaw mass | 10 g | Assumed proxy end-effector |
| Pad size | 3 × 9 × 5.5 mm | Explicit box contact shape |
| Commandable pad gap | 10–17 mm | Two 3.5 mm travel slide joints |
| Closed command | 12.4 mm | Elastic closure against a 13 mm nut |
| Jaw actuator stiffness | 20,000 N/m | Assumed actuator compliance |
| Jaw actuator force cap | ±20 N per jaw | Hard actuator saturation |
| Pad sliding friction | 0.8 | Uncalibrated rubber-like pad assumption |
| Hand wrench cap | 8 N, 0.08 N m | Euclidean force/torque saturation |
| Contact dimensions | 3 | Normal + two sliding-friction directions |

No torsional-friction or rolling-friction contact terms are added: torque comes
from distributed tangential forces on the pad surfaces. Contact compliance uses
`solref="0.0008 1"` and `solimp="0.95 0.99 0.0001"`; the fixture benchmark has a
0.1 ms timestep. These numerical contact parameters and all the physical
assumptions require sensitivity analysis and hardware calibration for sim-to-real
claims. A softer or coarser solver can alter slip and apparent rigidity.

The palm frame has an open center so the bolt can protrude through it. Palm and
jaw backing shapes also collide with the bolt/nut, using assumed sliding friction
0.3. Their centered baseline leaves clearance around the protruding bolt and
nut; unintended frame contact can obstruct a poor action instead of passing
through the workpiece.

## API

```python
from thread_lab.gripper import add_xml_gripper, ParallelJawController

# root is a mutable MJCF ElementTree, before compilation:
add_xml_gripper(root, initialcenter=(0, 0, .017), nut_geom="nut_geom")

# After compilation and before stepping:
hand = ParallelJawController(model, data)
hand.reset(aperture=.017)  # initializes only the hand and jaw states

# Low-level policy action: world force/torque at the palm body's CoM:
hand.apply_action([Fx, Fy, Fz, Tx, Ty, Tz], aperture_m)

# Optional finite-force Cartesian impedance on the hand:
hand.servo_pose(target_position, target_quaternion_wxyz, aperture_m)
mujoco.mj_step(model, data)

contact_report = hand.contact_wrench_on("nut")
```

`servo_pose` compensates gravity for the hand/jaws, converts rotational error to
world axes, and applies the position spring at the hand's grip origin using the
appropriate wrench transformation. Saturation still applies. Its position target
is an independently prescribed hand target, never an axial nut trajectory.
For thread experiments the z direction can be compliant or externally loaded;
an imposed hand z trajectory synchronized to the pitch would obscure whether
the threads are doing useful work.

`contact_wrench_on` uses `mj_contactForce`, transforms each hand-contact force to
world coordinates, and sums its moment about the nut body origin. It supplies
both the whole-hand `wrench_world` and pad-only `pad_wrench_world`, each pad's
normal force, jaw actuator forces, and the bounded hand wrench. Direct thread/
bolt contact is deliberately excluded from these hand-to-nut measurements.

## Isolated contact checks

Run the built-in benchmark:

```bash
/workspace/.venvs/astra-r2s/bin/python -m thread_lab.gripper
```

The fixture uses a **solid convex hexagonal nut surrogate**, without threads,
gravity, or supports. It isolates the grasp rather than claiming to verify
thread mechanics. After closure settles, a finite-force controller rotates the
hand at 0.8 rad/s for 0.95 s. A separate known external torque on the nut is used
only in the loaded validation case.

| Case | Hand rotation | Nut rotation | Mean pad torque during steady turn |
|---|---:|---:|---:|
| Closed, no resisting load | 0.760 rad | 0.76008 rad | Approximately zero after acceleration |
| Open | 0.760 rad | 0 rad | 0 N m; no pad contacts |
| Closed, resisting load 0.0035 N m | 0.750 rad | 0.74985 rad | 0.003500 N m |

Closed cases have eight pad contact points and about 5.774 N normal force per
pad. The unloaded acceleration transient transfers a peak 0.000129 N m; the
loaded case transfers a peak 0.003574 N m. All cases report zero solver warnings.
The resisting-load check establishes real frictional torque transfer, and the
open check establishes the absence of hidden grasp coupling. These results do
not establish realistic thread engagement, cross-threading, preload, training
throughput, or successful transfer of a learned policy to hardware.

## Policy interface and failure gates

`thread_lab.env.M8NutEnv` supplies a Gymnasium environment with seven normalized
actions: world force (three), world torque (three), and grip closure. Closure -1
opens the 17 mm gap; +1 requests 10 mm, with force capped by the physical jaw
actuators. An explicit, optional hand-only gravity-compensation baseline is added
to the action. The 42-component observation gives relative nut/bolt poses,
relative nut velocity, jaw positions and velocities, pad contact wrench and
normal forces, and world hand orientation and velocity. These are privileged
simulation-state observations, not a deployed vision or force-sensor interface.

Every reset starts **already threaded**. Reset randomization is limited to
±30 µm x/y, ±0.2° roll/pitch and ±0.02 rad yaw. Neither acquisition nor starting
an initially separate nut on the bolt has been validated. No policy is trained
or bundled by this module.

The default environment factory uses a 25 µs physics timestep, matching the
validated slow, 1 N loaded thread-turning benchmark. Supplying a model or explicit
`model_kwargs` can override it for diagnostic experiments; `info.model_timestep_s`
records the actual step. The faster hand demonstration uses its separately
verified 50 µs configuration.

The reward measures actual axial advance, with a small action penalty. Substep
guards abort on invalid states, solver warnings, excessive reported contact
depth, detached/tilted nuts, or large observed pitch residual. An invalid step
gives -1, regardless of apparent progress. A
success additionally requires ten consecutive valid control steps with thread
and pad contacts, radial offset ≤150 µm, tilt ≤2°, and observed pitch residual
≤150 µm. Pitch residual is measured from actual nut motion; it never drives a
joint, hand target, or engagement state.

Thread-contact gap duration remains a diagnostic. It does not establish failure
while the nut traverses ordinary flank backlash. A 0.2 N upward hand-force probe
crossed the fit's approximately 97 µm clearance, reacquired the opposite flank
by 17 ms, and settled to 97.087 µm axial reversal under continued load. The
former 10 ms contact-gap failure incorrectly stopped this valid reversal at
90.74 µm; that duration-only rule was removed. Geometric, finite-state, solver
and reported-depth guards remain active throughout the gap.

The native SDF-SDF `contact.dist` is a **reported solver depth proxy**, not a
certified geometric overlap bound. Planar overlap checks report approximately
half the true overlap. The environment's 10 µm reported-depth cutoff is a
conservative validity gate, and its logs name this quantity accordingly.

The environment requires the project's verified micron-search MuJoCo runtime by
default, including when passed an already compiled model. `allow_stock_engine`
is an explicit diagnostic override. Physical jaw parameters are checked against
the compiled model; thread pitch and dimensions come from the actual plugin
attributes, and a conflicting reward pitch is rejected.

```bash
bash scripts/run_m8.sh - <<'PY'
from thread_lab.env import M8NutEnv
env = M8NutEnv()
observation, info = env.reset(seed=1)
observation, reward, terminated, truncated, info = env.step([0, 0, 0, 0, 0, 0, -1])
env.close()
PY
```

The contact-driven hand demonstration is separate from a policy:

```bash
bash scripts/run_m8.sh -m thread_lab.demo --output outputs/m8/final_demo
```

During turns it removes the hand's axial position spring and axial damping,
applies a constant 0.05 N downward hand load, and uses finite rotational
impedance. During release/reset it holds a z target captured from the **hand's
own pose**, with no nut feedback. It records state trajectories, runtime hashes,
observed per-turn pitch, and open-hand decoupling for independent review.

The measured demonstration completed two turns with physical release and
regrasp between them. The nut advanced 1.249786 mm and 1.249902 mm; each turn's
helix residual was below 0.24 µm, against a 25 µm (2% of pitch) acceptance limit.
The open hand completed a full reset revolution with zero hand–nut contacts at
every physics substep. Nut creep during that reset was 48 µrad and 13.5 nm axial.
The run's maximum reported SDF depth was 1.983 µm, radial offset 0.647 µm and tilt
0.00643°. The nut received no external wrench or position commands. Measured
mean pad torques during the middle of the turns were −0.0476 and −0.0457 mN m.

`gripper_trace.npz` embeds the complete report in `metadata_json`, sampled
contact diagnostics in `info_json`, and time, qpos, qvel and hand-command arrays.
Phase checkpoints store full integration state and verify engine, model and
controller identity for restart. Derived-pose refresh can change the first
resumed control, so bit-identical future trajectories are not claimed. The
completed reported run was uninterrupted; its final checkpoint was reopened
solely to recover JSON serialization, without additional physics steps.

The corrected policy-action probe set covers eight modest actions, eight pure
maximum axial/yaw actions and eight representative force/torque corners for
16 ms each. All modest and pure-axis probes completed without validity guards.
Five corner probes completed; three open-hand corners triggered excessive
reported depth and were terminated with reward −1 and no success. This is
interface and failure-handling evidence, rather than acceptance of broad policy
training fidelity. The earlier 160-case report preserves its original results
and marks the two unsupported contact-duration diagnoses separately.
