# Starting a free M8 bolt in a threaded block

The insertion extension keeps the existing M8 × 1.25 right-hand, 60° thread
geometry, zero-margin contact, specified male/female pitch diameters, and
Coulomb friction. It changes which part is held: the left arm holds a free
aluminum block with a female through-hole, while the right arm picks up a
separate free headed steel bolt. Nothing couples yaw to axial displacement.

## Ends, orientation, and the opening

The existing male SDF spans local `z=0..16 mm`. Its 45° lead-in is at the
`z=16 mm` tip; a solid AF20 × 8 mm head occupies `z=-8..0 mm`. Both male and
female thread frames point local +Z downward in the manipulation scene.
Their fixed phases remain zero. In these common downward coordinates,
matching thread phase gives

`male_base_z = pitch * relative_yaw / (2*pi) + integer*pitch`.

Positive relative yaw around the downward axis is clockwise viewed from
above and advances the bolt down into the block. Flipping only one thread
frame would change this relative geometry and is incorrect.

The female opening has radius 4.399949 mm at the face. The male tip has
radius 2.999949 mm. The 0.956 mm male tip chamfer and female face chamfer
permit genuine entry from a separated pose; they do not prescribe thread
registration. The end chamfers and truncated flanks also permit an initial
axial clearance movement before the bolt begins following its pitch.

The rectangular 20 × 120 × 16 mm block is a non-overlapping solid union:
an AF16 female-thread SDF prism at one end, two side boxes, four convex
trapezoidal prisms around that hex, and the remaining rectangular handle.
These pieces reproduce the rectangle outside the real bore. No full box
covers the cavity, and no coincident solid caps double the seating stiffness.
Aluminum mass and inertia are calculated as the original box minus the
analytically integrated bore once. The independently integrated steel shaft
and exact solid hex head are counted once in the bolt's mass and inertia.

## Capture is distinct from initial contact

Cone contact is possible before a helical flank has captured the bolt.
Consequently, a contact count or axial overlap greater than one pitch alone
does not establish thread engagement. Initial phase can require nearly a
full revolution of compliant clockwise searching. An actual open-hand
reset during this search must retain support through the block contacts;
passive bolt yaw or axial motion must be measured rather than assumed zero.

For a conservative qualification window, the female chamfer can remain
active down to `5H/8 + 0.360 mm = 1.036582 mm` from its face, where
`H=sqrt(3)*pitch/2`. Adding the male's 0.956 mm chamfer and one complete
pitch of shared fully formed flank gives a minimum total overlap of
**3.242582 mm** for coaxial parts. Tilt requires extra depth: the
whole-ring bound adds approximately 146.46 µm at 2°. The shared
[geometry helper](../yam_twin/m8_insertion_mechanics.py) calculates the
complete male-axis interval whose entire major-radius ring lies inside the
female's unchamfered axial region, including its exit. Controller and policy
environment use the same measurement. Lead is assessed independently.
This condition is stricter than a contact-count test near the opening.

The isolated [start probe](../scripts/probe_m8_thread_start.py) explicitly
uses a fixed female and free six-DOF headed male with a bounded ideal fixture
wrench. Its lateral/tilt impedance keeps the fixture coaxial. Its axial input
is constant feed plus velocity drag; its independent angular input tracks a
bounded angular speed. It never commands axial position or uses yaw/pitch
to calculate an axial command. This is a geometry/contact benchmark, not
evidence of YAM pickup or arm-to-bolt force transfer.

An initial 1.5 s probe with 0.5 mm separation and 0.37 rad arbitrary yaw
reached end contacts at 0.38245 s and approximately 1.4 mm insertion, then
continued its phase search. It remained finite with no guard abort and a
maximum reported SDF depth of 3.932 µm. Its original broad one-pitch/contact
lead fit failed by 97.12%; that failure is retained. A separate, stricter
fully formed flank diagnostic is declared before the longer probe. This
avoids reporting cone contacts as completed threading.

With the same controls extended to 5.5 s, the male physically found the
thread phase and advanced into the block. The separate fully formed flank
window spanned 0.24515 revolutions and measured **1.2500516 mm/revolution**,
a 0.004124% lead error, with 0.32468 µm residual range. Maximum reported
depth remained 3.932 µm; radial offset stayed below 14.63 µm and tilt below
0.120°. No numerical guard aborted the run. Total axial travel was
4.0518 mm, including the initial separated gap and thread-finding movement.
The original broad fit still failed by 19.87%; its overall `passed=false`
remains recorded alongside the separate full-flank diagnostic.

Contact force is unilateral and intermittent with zero-margin SDF search.
In the final 0.52 s of this benchmark, complete flank geometry persisted,
while positive contact appeared in 177 of 520 sampled 1 ms rows. A 0.2 s
requirement for positive force on every substep would reject this physically
threading trajectory. A manipulation engagement classifier must distinguish
persistent geometry and repeated loaded contact from an uninterrupted force
signal; completed lead strokes and all penetration/alignment guards remain
separate requirements. This contact intermittency also merits timestep and
contact-search refinement before claiming broad policy-training fidelity.

Reproduce the separated start benchmark:

```bash
scripts/run_m8.sh scripts/probe_m8_thread_start.py \
  --duration 5.5 --output outputs/m8_insertion/start_probe_extended
```

A separate negative control kept the same axial feed and velocity drag but
commanded zero angular speed. It stopped at 1.400317 mm tip insertion,
showed only 0.6013 µm axial motion range during the final 0.4 s, and never
reached formed-flank capture. Net yaw drift was 7.15 µrad; the bolt was
angularly damped, not constrained by a rotation lock. Reported depth stayed
below 0.953 µm with no guard abort. Its positive-threading `passed=false`
is the expected negative result, not a discarded failed experiment.

## Qualification before policy-training claims

The nominal robot trace must show initially separated male/female bodies,
finite finger-contact pickup from the bolt rest, no rest contacts after
lift, approach and contact-driven capture, and subsequent independent
clockwise strokes whose measured lead agrees with 1.25 mm/revolution.
Only actual bounded robot motors may act in that trace. No weld, free-body
actuator, external object wrench, or object state rewrite may assist it.

Essential further checks include arbitrary initial phase, modest radial and
tilt errors, zero-rotation axial-feed obstruction after the end lead-in,
open-grip support during resets, axial-load/friction response, and timestep
and SDF-search refinement. Full head seating requires separate bearing-face
torque and contact/preload checks. Rigid contacts do not establish plastic
cross-thread damage, stripping, wear, or realistic bolt/joint elasticity.
Successful starting and running lead would remain a nominal mechanics proof
of concept rather than a trained or hardware-calibrated policy.
