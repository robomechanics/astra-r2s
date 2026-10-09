# YAM pickup and M8 thread starting

The female M8 × 1.25 through-hole is in a free aluminum block held by the
left YAM arm. A separate steel bolt has a 16 mm shaft and an AF20 × 8 mm
solid head. The right arm reaches for it, closes its real fingers, lifts it
off a physical three-pin rest, transports it over the hole, and searches for
the thread with finite motor torques and an axially floating hand.

[![Complete recorded pickup, capture and threading](../media/m8_insertion/full/demo.gif)](../media/m8_insertion/full/demo.mp4)

## Completed nominal rollout

The fresh, unspliced **27.35665 s** trajectory passes all **18 nominal gates**.
It starts with the bolt on its rest, performs physical pickup and transport,
searches through three starting strokes, captures complete flanks, opens the
right fingers to reset, regrips and performs two qualified half-turns. Those
two strokes give **1.000085 measured revolutions and 1.245793 mm axial travel**.
The final total axial thread overlap is **4.62465 mm**; the head is unseated.

| Qualified stroke | Measured rotation | Axial advance | Signed pitch residual |
| --- | ---: | ---: | ---: |
| `turn_1` | 180.0154° | 624.350 µm | −0.703 µm |
| `turn_2` | 180.0153° | 621.443 µm | −3.610 µm |

Both strokes pass the declared 2% lead limit. Residuals compare actual axial
motion with M8 × 1.25 pitch times actual rotation; no helix is commanded.

| Captured open reset | All-substep peak axial drift | Peak yaw drift | Hand / world / head-seating contacts |
| --- | ---: | ---: | --- |
| `reset_open_1` | 0.253 µm | 0.669 mrad | 0 / 0 / 0 |
| `reset_open_2` | 0.083 µm | 0.168 mrad | 0 / 0 / 0 |

Earlier resets during thread search are reported separately and do not count
as complete-flank self-locking evidence. The left arm physically lifts the
free block **3.846 mm**. Peak measured grasp translation is **131.08 µm**
at the block and **367.40 µm** at the bolt. Loaded sampled left pad forces
remain at least **15.57 / 14.81 N**. Right pad minima during the declared
transport/alignment/feed/closed-turn samples are **5.70 / 5.50 N**.
Minimum actual native arm-joint margin is **0.03467 rad**. No solver/state
abort, direct object drive or post-pickup world support occurs.

Published evidence:

- [Complete MP4](../media/m8_insertion/full/demo.mp4), [GIF](../media/m8_insertion/full/demo.gif) and [open-reset still](../media/m8_insertion/full/demo.png).
- [Actual trajectory](../media/m8_insertion/full/trace.npz), [all 18 gates and phase metrics](../media/m8_insertion/full/validation.json), [source/runtime manifest](../media/m8_insertion/full/manifest.json) and executed source archives.
- [Independent geometry/capture audit](../media/m8_insertion/full/independent_capture_audit.json), [all-candidate reset-contact audit](../media/m8_insertion/full_reset_contact_audit.json) and [free-joint property audit](../media/m8_insertion/full_free_joint_properties.json).
- [Measured motion/contact chart](../media/m8_insertion/full/trajectory.png) and [126-test software proof](../media/m8_insertion/software_tests.json).

The contact audit recomputes collision candidates at saved poses without
integrating physics. The rollout report supplies all-substep contact and
drift maxima; sampled replay cannot independently reconstruct forces between
samples. The free-joint audit separately checks zero joint damping, friction
loss, armature, springs, gravity compensation and fluid forces on both objects.

The [pickup-only trajectory](../media/m8_insertion/pickup) and
[first entry stroke](../media/m8_insertion/first_start) remain archived scoped
diagnostics. Their full-demo status remains incomplete; neither substitutes
for this completed rollout.

An environment restart interrupted the earlier full trials. Their original
partial traces, executed sources, interruption records and audits are preserved
under [interrupted trials](../media/m8_insertion/interrupted). The legacy
continuous-force trial reached capture and an unsupported reset, then stopped
before its first qualification turn. It has no final acceptance result.
The completed fresh run starts again from the separate bolt's original pickup
state; no trajectory is spliced from those checkpoints.

The separate fixed-female/free-bolt contact experiment starts from 0.5 mm
separation and an arbitrary angular phase. Its conservative full-flank window
measures **1.2500516 mm/revolution**, while the original broad entry-region
fit remains failed. A zero-rotation feed test stops at **1.400317 mm** and
does not push through the threads. These tests use an explicitly declared
bounded ideal fixture wrench; they do not qualify robot force transmission.
See [the physics investigation and limits](m8_insertion_physics.md).

Halving the timestep and doubling the search points both pass the declared
strict 2% total-travel, complete-flank-window travel and lead comparisons.
Doubling search points changes the peak reported contact depth by 45.02%,
so this depth proxy fails its separate 2% comparison. These refinements
qualify motion in the ideal fixture only. The [comparison, raw cases and
preserved failures](../media/m8_insertion/refinement/README.md) are published.

## Geometry and controls

The block is 20 × 120 × 16 mm and weighs 101.814 g after subtracting the real
helical bore. Its collision geometry tiles the rectangular solid around an
AF16 female-thread SDF prism without overlapping volumes or a box covering
the opening. The solid steel head and independently integrated shaft weigh
26.750 g together. Both thread frames point down; the unchanged exact M8
SDF geometry supplies contact surfaces only.

The left arm starts with its pads touching the block and acquires the grip
through finite finger forces. The right arm starts open, 20 mm above the
separate bolt head, then physically reaches and closes. Only actual YAM joint
and finger motors are commanded. The free bolt and block have no actuator,
grasp weld, external wrench, or prescribed screw trajectory.

During thread search/turning, the right controller removes axial position and
velocity servo forces. A constant net 0.05 N axial feed is produced through
bounded arm torques, including the declared bolt-weight compensation. Nut
or bolt yaw is never converted into a commanded axial position. Lateral and
orientation targets follow the measured moving hole frame.

Full engagement is distinct from cone contact. The geometry observer requires
one whole pitch of complete, unchamfered flank overlap, accounting for tilt
and the female exit. It also checks actual loaded contacts inside both
unchamfered axial spans. Qualified closed strokes independently require
measured pitch agreement within 2%. Search resets and qualified unseated
thread resets have separate reports; head/block seating cannot supply a
thread self-locking claim.

The demo and policy environment use the same versioned capture observer:
0.2 s of valid complete-ring
geometry, at least 0.001 N·s of measured interior-contact normal impulse,
at least 0.5 ms of loaded contact, and at most 150 µm helix-phase variation.
Normal impulse establishes loaded contact; it does not establish axial force
balance. Actual lead and unsupported open resets independently qualify
engagement. Every saved sample records the window metrics.

The candidate capture tag occurs at **16.063 s**. Qualification follows from
the measured lead and unsupported resets above, rather than the tag alone.

The earlier observer required 0.2 s of uninterrupted positive contact force.
Zero-margin unilateral contacts have real force gaps even during correct
pitch-following motion. That criterion remains a separate diagnostic, and
the original trial retains its executed source and outcome. The fresh full
run also passes this legacy diagnostic at **22.44105 s**. Contact geometry,
force laws, motor bounds, lead limits and reset limits are unchanged.

## Run and replay

Use the verified native MuJoCo CPU launcher. This is separate from the legacy
MJLab bridge; no GPU or learned policy is used.

```bash
scripts/run_m8.sh -m yam_twin.m8_insertion_demo --preview
scripts/run_m8.sh -m yam_twin.m8_insertion_demo \
  --output outputs/m8_insertion/demo --dt .00005 \
  --stroke-degrees 180 --angular-speed 2 --maximum-starting-strokes 5 --video
scripts/run_m8.sh -m yam_twin.m8_insertion_demo \
  --replay media/m8_insertion/full/trace.npz \
  --output outputs/m8_insertion/full_replay --slow-motion 1.5
scripts/run_m8.sh scripts/audit_m8_insertion_trace.py \
  media/m8_insertion/full/trace.npz
scripts/run_m8.sh scripts/audit_m8_insertion_capture.py \
  media/m8_insertion/full/trace.npz
scripts/run_m8.sh scripts/audit_m8_insertion_reset_contacts.py \
  media/m8_insertion/full/trace.npz
scripts/run_m8.sh scripts/audit_m8_free_joint_properties.py \
  media/m8_insertion/full/trace.npz
scripts/run_m8.sh scripts/probe_m8_thread_start.py \
  --duration 5.5 --output outputs/m8_insertion/geometry_start
```

Physical thread integration can take tens of minutes on CPU. Rendering
replays recorded states without simulating or inventing object motion. The
full robot command can fail when strict gates fail. The fixture probe retains
its original failed broad lead test even when its separate full-flank
diagnostic passes; inspect every result and its scope.

`yam_twin.m8_insertion_env.YamM8InsertionEnv` exposes 14 bounded actual
motor/jaw actions and 118 privileged observations, including pickup history,
grasp slips and moving-thread geometry. Contact impulse and capture-window
diagnostics are available in `info`. Policy steps contain no scripted pickup
controller. Success requires retained,
unsupported grasps and independently measured rotation/advance within 2%
of pitch after full-flank engagement. Actual arm joints outside their native
ranges by more than 10 µrad terminate the episode; current and episode-minimum
margins appear in `info`. The temporal capture-window state is diagnostic
information rather than part of the 118-number observation vector. No policy
has been trained or validated over a useful training horizon.

This task does not yet demonstrate full head seating, tightening preload,
thread damage or calibrated hardware response. Original independent
load/search failures remain in [the mechanics status](m8_status.md).
