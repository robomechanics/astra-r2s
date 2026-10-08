# Contact-resolved metric-thread research

The first-party MuJoCo bolt/nut SDF plugins are not suitable for an accurate M8
assembly without modification: their pitch is fixed to 1/12 m, their flank
profile is a 90-degree triangle, and their heads and nut thickness are fixed.
The compiler explicitly rejects mesh scaling for SDF plugins. These facts are
from the authoritative [bolt implementation](https://github.com/google-deepmind/mujoco/blob/main/plugin/sdf/bolt.cc),
[nut implementation](https://github.com/google-deepmind/mujoco/blob/main/plugin/sdf/nut.cc),
and [mesh compiler](https://github.com/google-deepmind/mujoco/blob/main/src/user/user_mesh.cc).
MuJoCo 3.15.0 already includes first-party SDF support in its official Python
wheel; the custom plugin builds against that installed wheel's headers/library.

## Implemented geometry

`thread_lab/plugins/m8_sdf.cc` registers `astra.m8_thread`, with parameters
`diameter`, `pitch`, `length`, `pitch_diameter`, `af`, `chamfer`, `female`, and
`phase`. Only geometry capability is registered. The plugin defines no force,
actuator, passive torque, screw joint, progress state, or engagement state.
Axial translation and rotation remain independent simulated freedoms.

The default coarse pitch is 1.25 mm with right-hand phase
`u = wrap(z - P atan2(y,x)/(2π), P)`. Male and female threads have straight
60-degree flanks. With `H=√3P/2`, their dimensional form is:

| Feature | Formula | Selected M8 value |
| --- | --- | --- |
| Male pitch diameter | explicit `pitch_diameter` | 7.100 mm |
| Male crest radius | `d2/2 + 3H/8` | 3.955949 mm |
| Male rounded root radius | `R=H/6` | 0.180422 mm |
| Male minor radius | `d2/2 - H/3` | 3.189156 mm |
| Female pitch diameter | explicit `pitch_diameter` | 7.268 mm |
| Female major cavity radius | `D2/2 + 3H/8` | 4.039949 mm |
| Female minor cavity radius | `D2/2 - H/4` | 3.363367 mm |
| Nut across flats | explicit `af` | 13 mm |
| Nut height | explicit female `length` | 6.5 mm |

The selected pitch diameters lie within the proposed 6g/6H fit ranges. The fit
assumes ideal coaxial geometry; no dimensional certification or measured fit
from the video is claimed. Male and female dimensions supply 84 µm of radial
clearance at the pitch line and about 97 µm of axial backlash. Both pieces have
finite length; the nut has a real through-cavity. The bolt's tip and nut entry
have explicitly assumed 45-degree chamfers. The outer nut is a hexagonal prism.

## Distance field and independent geometry validation

The collision field uses closest points on periodic profile line segments and
the male root's circular arc. A local helical metric accounts for the azimuthal
slope, `β=1/√(1+[P/(2πr)]²)`. This is a local approximation to the full 3D
Euclidean signed distance; the helical zero surface is exact. Caps, chamfers,
and the outer hexagon use signed-field constructive geometry, which is exact
in sign but can approximate distance near their intersections.

An independent 3D closest-surface numerical optimization tested 20 probes at
crest, flank, and rounded root with radial offsets ±10 and ±100 µm. Nine
independent optimization starts per probe gave a maximum distance discrepancy
of **0.000827 µm**, with maximum relative discrepancy **1.14×10⁻⁵**, on these
probes. This is an empirical check near threaded contact, not a global bound.
The reproducer and report are in `/workspace/research/thread_sdf/` as
`distance_accuracy.py` and `distance_accuracy.json`.

`tests/test_m8_sdf.py` checks crest/root dimensions, actual closest-point flank
distance, signed cavity distance, 2,000 random aligned male/female probes that
must not report false contact, right-hand helical invariance, finite axial
queries, and entry-chamfer behavior. The sum-of-distances check is important:
naively normalizing a radial implicit field by a different local gradient at
each point can generate spurious contacts away from its zero surface.

## Initial mechanical evidence and unresolved stability

Native MuJoCo CPU research runs used a free 6-DOF nut on a fixed bolt, gravity,
Coulomb friction 0.15, zero torsional/rolling friction, and external torque with
angular damping `1e-5 N·m·s/rad`. There was no axial actuator, weld, equality,
thread joint, or position target. At timestep 50 µs, 40 SDF seeds, 30 SDF
iterations, contact time constant 0.5 ms:

| Case | Measured behavior |
| --- | --- |
| Clockwise −50 µN·m, 1 s | −0.5046 revolutions, −0.6802 mm including initial backlash; fitted pitch 1.25058 mm/rev; worst reported penetration 6.16 µm |
| Reverse +100 µN·m, 0.5 s | +0.4616 revolutions, +0.5306 mm; fitted pitch 1.25552 mm/rev; worst reported penetration 5.01 µm |
| Zero torque, μ=0.15, 0.5 s | About 66 µm initial settling/backlash, then held with very small rotation |
| Zero torque, μ=0, 0.5 s, initial minimalist prototype | **Failed:** prototype used a pyramidal cone and a body-local velocity for a world torque damper; its energy conclusion was confounded |

These early results establish contact-driven axial travel in the loaded
frictional case. They do **not** establish robust free-body physics across
friction, speed, misalignment, or policy exploration. The initial prototype's
damper used a body-local velocity for a world torque, which can add energy after
a large tilt. Later experiments use world velocity and record external work.
Diagnostic
independent slide/hinge cases and smaller-step/more-seed free-body cases are
being used to separate collision sampling, solver stability, and geometric
errors. Full reports are saved in `/workspace/research/thread_sdf/`.

## Diagnosed engine scale issue and transparent correction

The initial instability motivated inspection of the native SDF narrowphase
search's absolute line-search floor. The exact upstream
[MuJoCo 3.15.0 implementation](https://github.com/google-deepmind/mujoco/blob/3.15.0/src/engine/engine_collision_sdf.c#L627)
sets `amin=1e-4`, which can stop spatial line search at about 100 µm with a
unit signed-distance gradient. This exceeds the M8 fit clearance and intended
micron-scale penetration. More seeds or time steps do not remove that search
floor.

As an independent diagnosis, the entire free-body experiment was expressed in
equivalent millimetre units: every length and gravity was multiplied by 1,000,
inertia and torque by 1,000,000, and output converted back to SI. No force
law, shape, friction, or degree of freedom changed. The μ=0 nut then
backdrove under gravity by 0.0790 revolutions in 0.5 s, with fitted pitch
**1.249785 mm/rev**, maximum reported penetration **0.0744 µm**, maximum angular
speed 1.095 rad/s, and **no increase in passive mechanical energy**. This
demonstrates sensitivity to the collider's model-unit search constants. It
does not establish that changing this one constant fixes every instability.
The equivalent SI stock prototype run failed.
The unit-scaled experiment is recorded by `bench_units.py` and
`unit_scaled_benchmark_results.json` in the research directory.

The development implementation keeps SI units and builds an explicitly patched
MuJoCo core from official 3.15.0 commit
`9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5`. The first research patch uses
`amin=1e-7` (0.1 µm). A later candidate sets the initial step to 2 mm;
an isolated nanometre-floor candidate is being evaluated for slow motion and
static friction. Exported markers prove which modified core is actually
loaded. No solver force law, contact Jacobian, friction coefficient, mass,
thread relation, or integration step is changed by these patches. The current
patch is saved in `scripts/patches/mujoco-3.15.0-micron-sdf.patch`; the build
provenance records the exact patch and library SHA-256 hashes.

With the main model's elliptic Coulomb cone and correct world-axis damper,
SI μ=0 at initial nut center 12.5 mm passed: 0.5 s gives −0.07796 revolutions,
−0.14584 mm, fitted lead 1.250004 mm/rev, maximum reported penetration
1.98 µm, and maximum tilt 0.00362 degrees. Rigid-body energy decreases by
7.36 µJ while the damper removes 4.90 µJ. This demonstrates passive free-body
backdrive in a documented configuration distinct from the early prototype.

Build and run using the pinned Python environment:

```bash
python scripts/build_mujoco_thread_engine.py --jobs 4
scripts/run_m8.sh -m thread_lab.benchmark
scripts/run_m8.sh -m pytest tests/test_m8_sdf.py
```

The build requires a C/C++ compiler, CMake, Ninja, and access to the official
GitHub repository and its pinned dependencies. It saves commit, patch, and
library hashes in `thread_engine_provenance.json`. The current wrapper uses a separately built GCC core **and matching GCC Python
bindings**, clears `LD_PRELOAD`, verifies both search markers and package hashes,
and rejects a process mapping multiple MuJoCo cores. The earlier preload research
approach is superseded: mixing the GCC core with the PyPI wheel's C++ ABI corrupts
string-bearing `MjSpec` operations. The final launcher checks that interface with
an edit/ZIP roundtrip before running. See [matched setup](m8_setup.md).

The plugin is a native CPU MuJoCo extension. Its C++ SDF callback is not a
MuJoCo Warp/Newton/MJLab GPU collision implementation; GPU training support
requires a separate implementation and validation. No trained policy or
sim-to-real validity is claimed by geometry and pitch tests.
