# M8 nut-on-bolt mechanics: geometry and validation plan

This document specifies a **passive, contact-resolved** M8 thread experiment.
It is a research and verification specification, not a claim that all checks
have passed. The supplied video does not measure thread tolerances, friction,
forces, material properties, or joint telemetry. Those quantities must remain
explicit assumptions until measured on the physical apparatus.

## References inspected

1. Bossard, [Metric ISO Threads: Dimensions and Tolerances](https://www.bossard.com/global-en/knowledge-hub/resources/technical-information/metric-iso-threads/),
   including its [January 2025 technical guide](https://assets.eu.ctfassets.net/0vp0u5uh75zd/2tYqENAuufvdsudjM8Qbrc/b3461eaa59c2203d3a8b9509ac24dacf/096_098_Metric_ISOthreads_Fastening_EN_01_2025.pdf).
   Public engineering tables identify ISO 724 basic dimensions, ISO 965 limits,
   and ISO 262 pitch selection. The M8 coarse row gives the 6g/6H limits below.
   This public guide is a secondary engineering reference, not possession of
   the complete copyrighted ISO standards or certification of a generated part.
2. Alexander Slocum, MIT, [FUNdaMENTALs of Design, Topic 6: Power Transmission Elements II](https://pergatory.mit.edu/resources/FUNdaMENTALs%20Book%20pdf/FUNdaMENTALs%20Topic%206.PDF),
   pp. 6-3 and 6-4. Derives screw force, work, efficiency, thread half-angle, and
   backdrivability; distinguishes thread friction from thrust-bearing friction.
3. Richard T. Barrett, NASA RP-1228, [Fastener Design Manual](https://ntrs.nasa.gov/api/citations/19900009424/downloads/19900009424.pdf),
   printed pp. 15–17. Gives thread-angle and friction-dependent torque relations
   and warns that material, lubricant, coating, and bearing-surface friction
   affect torque. Its whole-joint `T = K F d` formula must not be used as a
   thread-only running-torque law.
4. Bossard, [Preload and Tightening Torques](https://www.bossard.com/global-en/knowledge-hub/resources/technical-information/preload-and-tightening-torque/),
   [January 2025 technical guide](https://assets.eu.ctfassets.net/0vp0u5uh75zd/3S40LEUM235Qk3rJk2phR1/c715f9f45232756c12b59aa681c7fede/060_074_Preload_tightening_torques_Fastening_EN_01_2025.pdf).
   Explains the conditions and uncertainties of tightening-torque tables.
   For its electro-zinc-plated example, thread and bearing friction lie in an
   example interval 0.14–0.24. This does not establish the video's coefficient.
5. [AMES metric thread dimensions calculator](https://amesweb.info/screws/metric-thread-dimensions-calculator.aspx)
   documents the standard basic-profile formulas and external rounded-root
   convention. Used to cross-check formulas; authoritative limits above come
   from the inspected Bossard guide.

## Geometry, with dimensions in millimetres

Use a right-hand, single-start **M8 × 1.25** coarse thread with a 60-degree
included flank angle. Lead equals pitch for one start. Do not replace the
thread with concentric rings, a visual texture, a prescribed screw joint, or
an axial actuator coupled to rotation.

The fundamental triangle height is `H = sqrt(3) P / 2 = 1.082531755`.
Basic pitch diameter is `d2 = D2 = 8 − 3 H / 4 = 7.188101184`.
Basic internal minor diameter is `D1 = 8 − 5 H / 4 = 6.646835307`.
A common ideal external rounded-root profile gives
`d3 = 8 − 17 H / 12 = 6.466413348`; this is not the same quantity as the
basic ISO reference minor diameter. External root contour and its tolerance
must be considered separately.

The public ISO 965 table gives:

| Feature | Minimum | Maximum | Proposed experiment |
|---|---:|---:|---:|
| Bolt major diameter `d`, 6g | 7.760 | 7.972 | 7.911899 |
| Bolt pitch diameter `d2`, 6g | 7.042 | 7.160 | 7.100000 |
| Bolt root radius | 0.156 | — | 0.180422 |
| Nut pitch diameter `D2`, 6H | 7.188 | 7.348 | 7.268000 |
| Nut minor diameter `D1`, 6H | 6.647 | 6.912 | 6.726734 |

The M8 row specifies a medium thread-engagement range of 4–12 mm.
The proposed dimensions are a mutually consistent idealized profile inside
the listed individual diameter bands. They are not a manufactured-part gauge
certificate; finite-length ends, pitch error, coating, eccentricity, and
allowable root-contour details remain relevant.

### A coherent radial profile

Let `u = wrap(z − P theta/(2 pi), −P/2, P/2)` in each body's local frame.
Both bolt outside and nut void use the same helix direction and phase when
engaged. Let `a = abs(u)` and `s = sqrt(3)`. The straight flank radius is
`r_flank = r_pitch + s (P/4 − a)`.

For the bolt, `r_pitch = d2/2 = 3.55` and its crest radius is
`r_crest = r_pitch + 3 H/8 = 3.955949408`, giving a crest flat of axial
width `P/8`. Use a tangent circular external root, not a sharp V:

```text
R = H/6 = 0.180421959
v = P/2 − a
r_circle_center = r_pitch − H/2 + 2 R
if 0 <= v <= sqrt(3) R/2:
    r = r_circle_center − sqrt(R*R − v*v)
else:
    r = min(r_flank, r_crest)
```

The transition is at `a = 3 P/8`; its radial position and slope agree with
the straight flank. The resulting external minimum diameter is
`d3 = d2 − 2 H/3 = 6.378312163`.

For the nut void, `r_pitch = D2/2 = 3.634` and
`r = clip(r_flank, r_pitch − H/4, r_pitch + 3 H/8)`.
The lower limit is the nut's internal crest radius; the upper limit is its
internal root radius. The corresponding void diameters are 6.726734123 and
8.079898816. The internal crest flat has axial width `P/4` and its root
flat has width `P/8`.

Away from truncations, the radial clearance is
`(D2 − d2)/2 = 0.084 mm`. Expected axial backlash from that ideal profile is
approximately `2 (0.084)/sqrt(3) = 0.096995 mm`. Do not judge a reversal's
initial clearance travel as pitch error. Any contact margin, soft-contact
penetration, mesh approximation, or geometric expansion must be reported
against this scale; a millimetre of overlap is mechanically unacceptable.

A radial implicit field is not automatically a Euclidean signed-distance
field. Near a straight flank, normalize its gradient using approximately
`sqrt(1 + s*s + (s P/(2 pi r))**2)`. Rounded roots, truncations, and end
chamfers have different gradients. Incorrect normals or distances can create
torque and penetration artifacts even when a render looks correct.

### Ends and mass

A reasonable explicit lead-in assumption is a 45-degree nut-bore chamfer:
`r_void = max(r_thread, 4.3 − distance_from_face)` on each face, with radii
and distances in mm. A 4.4 mm opening radius is a slightly more generous
alternative. A bolt tip can use a 45-degree outer chamfer,
`r_outer = min(r_thread, 3.0 + depth_from_tip)`.
These are assumed end geometries, not dimensions inferred from the video or
a claim about a particular ISO 4032 nut's countersink angle.

Use a hex nut with 13 mm across flats and 6.5 mm thickness as a representative
M8 part. With density 7850 kg/m³ and the coherent ideal bore above, an
unchamfered, period-averaged geometry gives approximately:

```text
mass = 0.00531094 kg
Ixx = Iyy = 9.89457e-8 kg m^2
Izz = 1.60494e-7 kg m^2
```

Chamfers decrease these values, and a non-integer number of thread periods
can slightly alter the tensor. Prefer mass properties integrated from the
actual solid. Do not inflate mass or inertia to stabilize the solver and
then present the result as a steel M8 nut.

### Independent check of the implemented chamfered nut

The implemented SDF geometry uses a 0.360 mm chamfer measured outward from
the internal major radius, at each nut face. Its opening radius is therefore
4.399949 mm. An independent NumPy integration of the analytical **solid
boundary**, without calling the plugin or importing simulation state, gives:

| Deterministic samples | Mass, g | `Ixx`, kg m² | `Iyy`, kg m² | `Izz`, kg m² |
|---:|---:|---:|---:|---:|
| 262,144 | 5.19581160 | 9.7096064e-8 | 9.7076269e-8 | 1.5880455e-7 |
| 1,048,576 | 5.19580610 | 9.7095962e-8 | 9.7076247e-8 | 1.5880447e-7 |
| 4,194,304 | 5.19580662 | 9.7095960e-8 | 9.7076242e-8 | 1.5880448e-7 |

These are central inertia components. At the finest resolution, the centroid
is about `(-1.3214e-6, 0, 0)` m; the small asymmetric finite helix contributes
`Iyz ≈ -7.315e-11 kg m²`. Remaining product moments are negligible. The
previous unchamfered 5.311 g approximation overestimates this mass by approximately
2.22%; its transverse inertia by about 1.9% and axial inertia by about 1.1%.
The model now uses mass 5.195807 g, the averaged transverse inertia
9.70861e-8 kg m², and axial inertia 1.58804e-7 kg m². It neglects the
micron-scale centroid offset and small off-diagonal moment. This records the
remaining approximation when computing gravity compensation or comparing
very small load/torque measurements.

The deterministic quadrature uses unscrambled Halton coordinates at bases 2
and 3 for angle and axial position. For each sample, independently evaluate
the clipped 60-degree internal profile and the face chamfer:

```python
theta = 2*pi*radical_inverse(sample_index, base=2)
z = height*(radical_inverse(sample_index, base=3) - 0.5)
u = (z - pitch*theta/(2*pi) + pitch/2) % pitch - pitch/2
r = clip(r_pitch + sqrt(3)*(pitch/4-abs(u)),
         r_pitch-H/4, r_pitch+3*H/8)
r = maximum(r, r_pitch+3*H/8+chamfer-(height/2-abs(z)))
```

Integrate the bore analytically in radius and numerically over angle/height.
With domain area `A = 2*pi*height`, hole volume is `A*mean(r²/2)`.
Hole first moments are
`A*mean((cos(theta)*r³/3, sin(theta)*r³/3, z*r²/2))`.
Hole second moments are the means of `cos²(theta)*r⁴/4`,
`sin²(theta)*r⁴/4`, `z²*r²/2`, and their corresponding cross products,
times `A`. Subtract them from an exact solid hexagonal prism with
`area_hex = sqrt(3)*across_flats²/2` and
`polar_area_moment_hex = 5*sqrt(3)*across_flats⁴/72`. Apply the parallel-axis
correction at the computed centroid and multiply by density. The finest two
sample counts differ in mass by roughly 0.00001%; this is a numerical
cross-check of assumed geometry, not a measured nut mass.

## Force, torque, and friction benchmarks

Take a declared nominal sliding coefficient `mu = 0.15`, and sweep at least
`0, 0.05, 0.08, 0.15, 0.25`. These are experimental hypotheses. Do not
infer a coefficient from an uninstrumented video or assume one friction value
transfers across bare, plated, lubricated, and contaminated steel.

For an aligned, thread-only, quasi-static experiment without bearing contact:

```text
alpha = 30 degrees                # thread half-angle
lambda = atan(P / (pi*d2))        # single-start lead angle
mu_eff = mu / cos(alpha)
T_raise / F = (d2/2) * (tan(lambda) + mu_eff) / (1 − mu_eff*tan(lambda))
T_lower / F = (d2/2) * (mu_eff − tan(lambda)) / (1 + mu_eff*tan(lambda))
```

Convert diameters to metres before obtaining Nm/N. `T_raise` moves against
an axial load; `T_lower` is the torque required in the load-assisted direction
when self-locking. Torque signs depend on the selected body and axes.
An axial push that assists insertion must not be compared with the raising
equation. No bearing-face torque is included in these expressions.

For the **basic** `d2 = 7.188101 mm`, `lambda = 3.168295 degrees`:

| `mu` | Raise, Nm/N | Lower, Nm/N |
|---:|---:|---:|
| 0.00 | 0.000198944 | −0.000198944 |
| 0.05 | 0.000407749 | 0.000008532 |
| 0.08 | 0.000533677 | 0.000132384 |
| 0.15 | 0.000829403 | 0.000419542 |
| 0.25 | 0.001256535 | 0.000825380 |

Recompute these targets from the actual experimental pitch diameter. The
basic example at 10 N and `mu = 0.15` gives about 8.29 mNm raising torque
and 4.20 mNm lowering torque. Such running torques are far smaller than the
torque of a seated, preloaded M8 bolted joint. Do not use a tightening table
to set hand-spinning torque.

`thread_lab/theory.py` uses the explicit average of the specified male and
female pitch diameters, 7.184 mm, as its comparison lever arm. It predicts
8.290494 mNm raising and 4.191877 mNm lowering at 10 N and `mu = 0.15`;
the corresponding self-lock threshold is 0.04796500. This is a declared
first-order effective radius, not a constraint on contact location. Real
contact-force integration can reveal different load distribution across the
flank. The independent analytical tests check zero-friction work conservation,
the inclined-plane limit, torque/load proportionality, and the change in
lowering-torque sign at the self-lock threshold.

An ideal V thread is self-locking if `mu_eff > tan(lambda)`. For the basic
M8 geometry this means `mu > 0.0479376`. Thus an axial force alone should
backdrive a frictionless thread and should generally hold at `mu = 0.15`,
after clearance travel. Locking the rotation coordinate or holding it with a
servo cannot demonstrate self-locking.

Set extra torsional and rolling contact friction to zero in the analytic
benchmark. Otherwise extra point-contact moments can confound the comparison.
Real gripper-pad friction is a separate modeled interface.

## Benchmark ladder and falsification checks

1. **Static geometry and overlap.** Verify dimensions, winding sign, face
   normals, internal bore, thread continuity, and initial collision distances.
   Establish a known engaged phase without initial interpenetration. Use
   force/torque sensors and exact initial transforms instead of aligning only
   by eye. Visualize actual collision geometry and its contacts.
2. **Prefit isolated mechanism.** Fix the bolt and use independent axial slide
   and axial rotation DOFs for the nut. This is an aligned mechanical test
   fixture; it does not demonstrate assembly alignment or robotic grasping.
   Start with 2–3 engaged turns. Apply only bounded torque and an explicit axial
   force. No helix constraint, axial target, coordinate overwrite, depth
   integrator, switched weld, or fitted translation-versus-angle controller.
3. **Pitch from contacts.** Turn slowly over several revolutions and regress
   measured axial displacement against unwrapped measured relative angle.
   Expected slope is `P/(2 pi)` with sign fixed by the winding. Start/end
   transients and backlash are separate. Initial target: lead error below 2%
   over at least two revolutions, with reported pitch residual and penetration.
4. **Reverse.** Reverse torque and demonstrate unscrewing. Allow measured
   backlash, then check the same slope and finite contact reaction. A state
   machine permitting only tightening fails this check.
5. **No contact means no feed.** Move the nut completely off the bolt, disable
   gravity and axial forces, then apply pure axial torque. Axial position must
   remain invariant apart from numerical noise. Repeat with collision disabled
   in the engaged scene. Continued pitch-locked translation reveals hidden
   kinematic coupling. Do not add axial push to this null test.
6. **Axial hold and backdrive.** With rotation free and its drive disabled,
   apply modest known axial loads in both directions. Measure clearance
   travel and sustained velocity. At `mu = 0.15`, engaged threads should hold
   after settling; at `mu = 0`, they should backdrive. Report frictionless
   drift and frictional breakaway instead of passing merely because a stiff
   rotation servo held the nut.
7. **Torque versus load.** At steady low angular speed, sweep opposing axial
   loads, e.g. 1, 5, and 10 N. Log actual drive torque, angular velocity,
   axial velocity, contact forces, and acceleration. Compare the torque-load
   slope with the V-thread expression above, separately from drive inertial
   torque and bearing friction. A 10% target is a useful initial engineering
   gate, not a proof of physical calibration. A load-independent torque is
   evidence of an incomplete or wrong contact model.
8. **Friction and pitch ablations.** Friction changes must change running and
   breakaway torque. A separately generated M8 × 1.0 geometry must change
   measured lead to 1.0 mm/revolution. These independent physical predictions
   are stronger than tests that simply assert the controller's chosen depth.
9. **Free-body assembly.** Remove the alignment guide; use a full 6-DOF nut
   and test approach from above the tip. Sweep initial offset and tilt;
   record success, jam, lift-off, and contact forces. Assembly should become
   harder with adverse offsets, not always succeed under a hidden guide.
   A rigid ideal model cannot establish real thread damage or plastic
   cross-threading, but it can expose misalignment, wedging, and escape.
10. **Manipulation and training interface.** Only after the prior checks,
    introduce physically limited gripper contacts and arm controllers.
    Wrench commands applied directly to a nut are a mechanical benchmark,
    not evidence of frictional robot grasping or a trained policy. Observations
    should use actual body states/contacts; rewards may measure insertion,
    but may not drive it. Report action limits, frequency, termination
    conditions, and physical failure cases.

For all runs, keep contact-enabled results separate from the null controls.
Log simulation configuration, geometry checksum, solver, timestep, contact
law, friction, applied wrenches, positions, velocities, contact count,
penetration, thread overlap, and wall-clock cost. A demonstration video and
a passing compiler/smoke test alone do not establish thread fidelity.

## Numerical convergence and energy checks

Begin low-speed CPU validation around `dt = 1e-4 s`; compare `5e-5` and
`2.5e-5 s`. This is a starting experiment, not an established stable timestep.
Contact stiffness/compliance is a physical approximation and should remain
fixed while timestep changes; changing both together does not isolate time
discretization error. Test solver tolerances/iterations separately. Measure
peak and percentile penetration, force spikes, and warning counts.

A useful initial penetration goal is below 10 micrometres relative to the
84 micrometre nominal radial flank clearance. Convergence goals should cover
lead, steady torque, backlash, and the self-lock/backdrive conclusion, e.g.
less than 2% change between the two finest timesteps. If nominal clearance
or contact penetration is deliberately enlarged for speed, quantify that
and rerun the benchmarks; it changes the task distribution.

For meshes, refine circumferential resolution and rounded roots independently.
At radius 4 mm, polygon chord sagitta is about 4.82 micrometres at 64 samples
per revolution and 1.20 micrometres at 128. These errors matter at the stated
clearance. A continuous implicit profile avoids that particular polygon error
but still needs contact-search and field-gradient convergence. SDF-based
contact is not automatically accurate merely because it lacks mesh triangles.

Track actuator work `integral(tau*omega + F dot v) dt`, kinetic/potential
energy changes, and dissipative work. A frictionless, quasi-static raising
test should approach `T = F P/(2 pi)`; friction adds required work. Contact
compliance stores energy and must be handled when evaluating closure. Large
unaccounted positive energy, finite insertion with no contact, or strong
changes under timestep refinement fail the mechanical benchmark.

## Limits of the proof of concept

Contact-resolved rigid-body threads can support studies of alignment, lead,
reversal, running torque, grip slip, and policy sensitivity to contact
parameters. They cannot by themselves validate metal yielding, permanent
cross-threading, burrs, damage, coating wear, stripping, or industrial
tightening preload. Seating a rigid nut against a rigid stop with compliant
solver contacts is not a calibrated model of elastic bolt stretch and
clamped-joint stiffness. Those require separately justified constitutive
models and physical force/torque measurements.

The immediate defensible milestone is an honestly labeled mechanical M8
thread benchmark with passive contact coupling and independently measured
results. A training environment can build on it, but simulator agreement
with analytic ideal-thread predictions is only the first validation layer;
matching measured apparatus behavior remains necessary for policy transfer.

## Recorded load diagnostics, first optimizer variant

`thread_lab/load_benchmark.py` implements an independent two-DOF bearing
fixture, applying a downward axial load and a finite PI yaw-speed controller.
It never controls axial position or velocity, writes body coordinates, or
uses analytical torque feedforward. A force ramp takes up initial clearance
without an impulsive full-load impact. Steady force/torque statistics use
every physics step, because a 1 kHz plotted trace can alias contact chatter.
The controller's zero-speed preload phase is explicitly distinct from
`self_lock_probe`, which applies **zero yaw torque with no controller**.

The initial custom MuJoCo 3.15.0 engine variant lowers the SDF optimizer's
minimum search step to 1e-7 m. On that variant, low-speed frictional results
**fail the torque benchmark**, despite correct pitch, small penetration,
balanced mean wrenches, and no solver warnings:

| Load | μ | Yaw speed, rad/s | dt, µs | Mean torque, mNm | Predicted torque, mNm | Lead, mm/rev | Result |
|---:|---:|---:|---:|---:|---:|---:|---|
| 1 N | 0.15 | +0.2 | 50 | 0.51365 | 0.82905 | 1.24691 | Fails torque and speed fluctuation |
| 1 N | 0.15 | +0.2 | 25 | 0.52969 | 0.82905 | 1.24832 | Fails torque and speed fluctuation |
| 1 N | 0.15 | +0.2 | 10 | 0.53820 | 0.82905 | 1.25093 | Fails torque |
| 1 N | 0.15 | +2.0 | 50 | 0.86499 | 0.82905 | 1.25054 | Passes this load diagnostic |
| 10 N | 0.15 | +2.0 | 50 | 8.08582 | 8.29049 | 1.25281 | Fails speed fluctuation |
| 10 N | 0.15 | −2.0 | 50 | −4.29098 | −4.19188 | 1.24671 | Fails speed fluctuation |
| 1 N | 0 | +2.0 | 50 | 0.19900 | 0.19894 | 1.24978 | Passes this load diagnostic |
| 1 N | 0 | −2.0 | 50 | +0.19797 | +0.19894 | 1.24859 | Passes this load diagnostic |

The contact-time constant remains 0.5 ms in the timestep comparisons.
Increasing the supported friction impedance ratio from 1 to 100 does not
restore the low-speed torque. At 1 N, μ=0.15, the 10 µs run shows saturated
friction cones, a force-weighted normal axial component 0.8647 consistent
with a 60-degree flank, and nominally correct lead. Its normal contact speed
still fluctuates around a weighted absolute value 0.571 mm/s, comparable
with the tangential slip speed 0.835 mm/s. This changes the direction of
frictional force at slow rotation. At 2 rad/s, tangential motion dominates
the contact chatter and the running torque approaches the ideal prediction.
This is a diagnosis supported by measured contact data, not proof that all
solver error has been isolated.

The initial variant's zero-torque tests correctly backdrive at μ=0, but
retain slow yaw creep at μ=0.05–0.25 under 1 N. Thus they do not establish
ideal self-locking. Several-turn lead, free-body assembly, low-speed torque,
self-locking, and numerical convergence remain separate gates. A passing
fast-turning case cannot justify policy training that relies on accurate
slow contact or sticking. Further optimizer variants are evaluated against
these failures without retuning physical friction or adding thread coupling.

Raw results are retained under `outputs/m8/guided_load_ramped`,
`guided_load_dt25`, `guided_load_dt10_diagnostic`,
`guided_load_speed2_diagnostic`, `guided_load10_speed2`,
`guided_friction_speed2`, and `guided_self_lock`. Reports include failed
checks instead of turning these observations into a success-only demo.

## Final zero-margin load sweep and remaining failures

The final aligned-fixture sweep uses the matched GCC MuJoCo 3.15.0 core
`58039d439c6504448aafd0a4d5b655c8a0d3bf1d77123ac7e0733cd078367433`,
SDF source hash `fc1d703d5406b7856d70d6e8ead3c64d3aec1811af58423610ecd167e39989af`,
40 SDF initial points, 30 SDF iterations, a 0.5 ms contact time constant,
elliptic Coulomb friction, and **zero contact margin**. The native SDF
narrowphase [rejects positive contact distances](https://github.com/google-deepmind/mujoco/blob/3.15.0/src/engine/engine_collision_sdf.c#L557)
and does not generate a shell of positive-margin SDF contacts. A positive
solver margin therefore caused repeated contact loss near the assumed shell.
Removing that incompatible margin materially improves slow running torque and
zero-torque holding, without changing physical friction, adding axial control,
or imposing a screw relation.

This improvement does **not** make every original acceptance gate pass. The
complete reports, NPZ traces, failed checks, independent audit, and aggregate
are retained under `outputs/m8/final_load`. Every running case applies a
1 N downward load and commands yaw speed ±0.2 rad/s for 0.5 s with a bounded
PI torque controller. The controller receives yaw speed alone. Its late
steady interval covers only about 0.0055 turns; these are local load
measurements, not several-revolution lead certification.

| μ | dt, µs | Raising torque, mNm | Lowering torque, mNm | Lowering lead, mm/rev | Remaining failed running gates |
|---:|---:|---:|---:|---:|---|
| 0 | 25 | +0.198986 | +0.198901 | 1.250207 | None |
| 0.05 | 25 | +0.412281 | −0.014693 | 1.231335 | Lowering mean-radius torque |
| 0.08 | 25 | +0.540994 | −0.141907 | 1.224033 | Lowering mean-radius torque and 2% lead |
| 0.15 | 25 | +0.847671 | −0.442028 | 1.251183 | Lowering mean-radius torque |
| 0.25 | 25 | +1.298161 | −0.875716 | 1.248063 | Raising/lowering speed fluctuation; lowering mean-radius torque |
| 0.15 | 12.5 | +0.852550 | −0.446587 | 1.250402 | Lowering mean-radius torque |

All running cases have finite states, no solver warnings, mean force/torque
balance within the stated gates, penetration below 2.251 µm, and torque within
the **independently calculated fixed geometry bounds**. The separate
mean-pitch-radius comparison remains failed where shown. At μ=0.15, the
lowering prediction at the declared mean pitch radius is −0.419188 mNm,
whereas the actual flank geometry permits −0.482048 to −0.379718 mNm. The
12.5 µs run's force-weighted measured radius is 3.746664 mm; using that radius
post hoc predicts −0.445897 mNm, within 0.155% of measured torque. This
explains a plausible radius sensitivity; it is explicitly a post hoc diagnostic
and does not replace or erase the original independent failed gate. Likewise,
near μ=0.05's neutral backdrive threshold, small radius differences materially
change the tiny lowering torque. The μ=0.08 short-window lead failure and
μ=0.25 speed fluctuations remain unresolved.

For μ=0.15, refining 25 to 12.5 µs changes mean raising torque by 0.5723%,
raising lead by 0.000105%, lowering torque by 1.0209%, lowering lead by
0.06244%, and worst penetration by 1.7236%, relative to the finer run. Those
particular mean measurements meet the 2% refinement target. Speed chatter
changes substantially, and total axial-load work in the raising case changes
7.442%; this does not establish convergence of every dynamic or energy
quantity. The work residual includes unmeasured contact elastic energy and
is not a verified friction dissipation or complete energy closure.

With the yaw controller absent and applied yaw torque exactly zero, 0.08 s
load tests match the expected distinction across μ=0, 0.05, 0.08, 0.15, and
0.25: μ=0 backdrives by 362.264 µm during the late window, while the frictional
cases show at most 0.001214 µm of late axial drift. A separate **0.5 s**
μ=0.15, 25 µs hold test reports 0.002515 µm drift over its final 0.175 s and
mean yaw velocity −0.0001347 rad/s, with no warnings and worst penetration
3.623 µm. These are finite-duration holding results with tiny measured creep;
they do not prove an exact mathematical static equilibrium.

An independent code/data audit confirms no axial controller, pitch input,
helix coupling, or coordinate overwrite, and reproduces the raising lead fits
from raw NPZ traces. Historical self-lock reports sampled their final kinetic
energy 1 ms before the terminal state; the code now computes terminal energy
from final velocities. Trace poses, velocities and kinetic energies are
pre-step, while contact forces and cumulative work include the following
physics step; the report records this timing convention.

During setup, an identical-core wheel reinstall unlinked the packaged core
inode in some live experiments. The historical runtime parser consequently
left `engine.libraries` empty because `/proc/maps` ended with `(deleted)`.
Those original fields and raw reports are preserved. An annotation records
the successful launch verification and the setup maintainer's matching
pre/post core hashes; it does not invent an old wheel hash or claim that a
later path lookup hashed the live unlinked inode.

The complete load suite remains **not accepted** under its original gates.
It provides contact-driven mechanical evidence and sharply bounded remaining
problems; it is not a calibrated, fully validated policy-training physics
model. Free-body assembly, physical gripper manipulation, hardware calibration,
and GPU implementation remain separate validation requirements.

The failing scalar mean-radius comparisons should be distinguished from the
passing predeclared geometry envelope. They do not by themselves prove an
incorrect contact force law: the ideal scalar prediction chooses a
first-order mean lever arm, while the actual force distribution spans a
finite flank. Conversely, envelope agreement does not certify the entire
model. The short steady travel is about 7 µm; a 2% slope gate then corresponds
to only about 140 nm of differential axial motion. Contact-distance and
trajectory fluctuations can materially affect that local fit. Retain the
reported failed gate while treating the case as a local diagnostic rather
than a multiple-turn lead result. Future load runs capture engine provenance
before their physics loop, preventing a setup reinstall from erasing that
identity at report time.
