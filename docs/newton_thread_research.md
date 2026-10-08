# Newton and real M8 thread contact

This research concerns **contact-generated motion**. No screw joint, pitch equality,
position overwrite, weld between fasteners, or rotation-to-insertion controller is
acceptable as evidence of mechanical thread engagement.

## Recommendation for this machine

Do not switch the proof of concept to Newton merely because its demo looks good.
Newton provides genuinely useful non-convex mesh, signed-distance-field (SDF), and
hydroelastic contact models, but its accelerated SDF and hydroelastic route needs a
CUDA GPU. This cloud machine reports only a CPU. Use a verified CPU-capable thread
contact implementation for the current benchmark; reserve Newton SDF/hydroelastic
for a later GPU comparison using identical thread geometry and validation cases.
Native MuJoCo SDF plugins are a separate capability from Newton's MuJoCo wrapper.
Newton's documentation about convex mesh conversion does **not** rule out a native
MuJoCo SDF plugin implementation.

Newton's live BVH mesh contacts can also preserve concavity on CPU. A diagnostic
probe is included under `outputs/newton_research/`; it is not a validated training
environment. Passing a visual demo or producing finite coordinates alone would not
qualify it as one.

## Source and runtime evidence

Inspected Newton commit
[`2e1e6b17c208b63132727dba53439122617d08ff`](https://github.com/newton-physics/newton/tree/2e1e6b17c208b63132727dba53439122617d08ff),
dated 2026-10-08. Installed it in the isolated venv
`/workspace/.venvs/newton-thread` with verified HTTPS package downloads. No shared
project dependency declaration or existing application source was changed by this
research.

Runtime versions: Newton `1.8.0.dev0`, Warp `1.18.0`, MuJoCo `3.14.0`,
MuJoCo Warp `3.14.0`. `warp.is_cuda_available()` returned `False` and Warp enumerated
only `cpu`. A tetrahedron's `mesh.build_sdf(device="cpu", max_resolution=32)` raised:

```text
RuntimeError: SDF.create_from_mesh requires a CUDA device: the texture SDF
build pipeline uses CUDA kernels and wp.Texture3D.
No CUDA-capable device was detected.
```

The exact runtime record is `outputs/newton_research/capability.json`. This is a
capability limit, not a package installation failure.

Authoritative source:

- [SDF creation explicitly checks CUDA availability](https://github.com/newton-physics/newton/blob/2e1e6b17c208b63132727dba53439122617d08ff/newton/_src/geometry/sdf_utils.py#L450-L460).
- [Collision architecture](https://github.com/newton-physics/newton/blob/2e1e6b17c208b63132727dba53439122617d08ff/docs/concepts/collisions.rst):
  meshes without cooked SDFs use live BVH distance queries; high triangle counts can
  be slow. Hydroelastic contacts require SDFs on both shapes and are unavailable on
  the live BVH path.
- [Newton's MuJoCo solver conversion and external contacts](https://github.com/newton-physics/newton/blob/2e1e6b17c208b63132727dba53439122617d08ff/docs/solvers/mujoco.rst):
  internal mesh contacts convex-hull non-convex MESH geometry. Set
  `use_mujoco_contacts=False` to use Newton contacts with MuJoCo Warp. This is distinct
  from native MuJoCo SDF plugins, which need their own integration and tests.

## What the official nut-and-bolt examples show

Newton has official
[SDF](https://github.com/newton-physics/newton/blob/2e1e6b17c208b63132727dba53439122617d08ff/newton/examples/contacts/example_nut_bolt_sdf.py)
and
[hydroelastic](https://github.com/newton-physics/newton/blob/2e1e6b17c208b63132727dba53439122617d08ff/newton/examples/contacts/example_nut_bolt_hydro.py)
nut-and-bolt examples. They load NVIDIA Factory **M20 loose** threaded CAD meshes.
The nut is a dynamic body and geometry determines contact. There is no helical
equality in these examples.

However, both choose sliding friction `mu=0.01`, zero rolling and torsional
friction, and allow a nut to descend and rotate under gravity. This is a deliberate
low-friction demonstration; it is not a measured M8 steel material calibration or
a robotic finger manipulation task. For a real M8 coarse V-thread with approximately
1.25 mm pitch and 7.19 mm pitch diameter, the lead angle is about 3.17 degrees.
Even modest steel friction generally prevents gravity-only backdriving. A freely
backdriving nut should not be presented as evidence of realistic friction.

The SDF example's final test asks only for more than 0.1 rad of rotation, some
descent, and less than 20 mm bolt displacement. The hydroelastic example asks for
more than 45 degrees of rotation and 5 mm descent. Neither test checks the measured
pitch per revolution, thread flank interpenetration, tightening torque, self-locking,
misalignment, cross-threading, timestep convergence, or robustness under friction
randomization. Their tests are appropriate example smoke tests, not sufficient
acceptance tests for this project's intended policy training.

The hydroelastic example sets `kh=1e11`; its source comments state that the softer
`1e10` setting yielded about a 95 ms relaxation time and a visibly cocked nut under
MuJoCo. XPBD's positional projection can hide this contact-stiffness issue. This is
a specific warning against comparing engines only through appearance.

## Solver differences relevant to threading

| Newton backend | Relevant mechanics | Implication for a thread benchmark |
| --- | --- | --- |
| MuJoCo Warp + Newton contacts | Generalized coordinates, frictional constraints, Newton BVH/SDF/hydroelastic contact input | Candidate GPU engine; gains, cone, timestep and contact-generation resolution still need numerical convergence tests. It is not an independent replacement for MuJoCo's constraint mathematics. |
| XPBD | Maximal coordinates, iterative position/contact projection; uses `mu`, not `ke`/`kd` as a calibrated force-space rigid contact law | Useful exploratory CPU contact route; do not infer physical stiffness or force fidelity from a stable rendered animation. Check force/torque behavior explicitly. |
| Kamino PADMM | Experimental maximal-coordinate hard Coulomb contact and coupled bilateral/unilateral constraint solve | Promising alternative for rigid threading, but Newton's own docs discourage depending on this BETA backend. Measure convergence residuals and contact behavior before adopting it. |
| Kamino DVI | Faster alternating bilateral solve and projected inequality iterations | Docs say inequality constraints are generally solved less accurately than PADMM as active contacts grow. Dense multi-flank thread contact deserves particular caution. |

Source:
[supported features and contact material fields](https://github.com/newton-physics/newton/blob/2e1e6b17c208b63132727dba53439122617d08ff/docs/solvers/index.rst),
[Kamino's limitations and terminal residuals](https://github.com/newton-physics/newton/blob/2e1e6b17c208b63132727dba53439122617d08ff/docs/solvers/kamino.rst).

Hydroelastic contact is a distributed compliant pressure model. It is not automatic
metal plasticity, thread damage, galling, or bolt preload calibration. Those effects
need additional constitutive models and physical measurements in every engine.

## Reproducible CPU diagnostic

`outputs/newton_research/cpu_thread_probe.py` applies an axial force and axial torque
to a free 6-DoF nut contacting a fixed threaded bolt. It uses the two Factory M8 loose
OBJ meshes referenced by Newton's example family, with exact duplicate export
vertices merged, and preserves all 29,478 bolt / 12,434 nut triangles. It uses
live BVH contact, not an imposed helix. The bolt becomes watertight after merging;
the nut still has non-watertight boundaries. This asset issue must be resolved or
the geometry replaced with validated watertight threads before using it as a
high-fidelity reference.

The initial 200-step run generated approximately 5,200 contacts against a 1,024
capacity. The overflow means that run is **invalid**, even though state remained
finite. A rerun with 12,000 capacity did not overflow but started at the bolt-head
runout with up to approximately 0.425 mm of detected initial penetration. Its
transient motion cannot validate threading either.

A 36-phase scan at +5 mm axial offset found no contacts at exactly 30 and 40 degrees.
Interpolating to 35 degrees was **not** safe: a subsequent exact-pose scan revealed
9 contacts with approximately 0.238 mm penetration. The nut mesh has open boundaries
and degenerate faces, and Newton's live BVH runtime uses pseudo-normal signs for
non-watertight meshes. Whatever the detailed geometric cause of these few contacts,
the 35-degree dynamic run is also unsuitable for engagement validation. The filename
`xpbd_clean_start_probe.json` reflects the original hypothesis, not an accepted result.

| Diagnostic | Simulated duration | Wall time (including compile/setup) | Result |
| --- | --- | --- | --- |
| Initial raw contacts, 1,024 capacity | 0.02 s | 58.47 s | Overflow; invalid |
| Raw contacts, 12,000 capacity | 0.10 s | 211.50 s | 6,749 maximum recorded contacts, initial overlap; no valid thread result |
| Contact reduction, 12,000 capacity, initial runout pose | 0.02 s | 44.03 s | About 117 final contacts; initial overlap; no valid thread result |
| Contact reduction, +5 mm / 35-degree pose | 0.10 s | 247.52 s | 143 max contacts/no overflow, -0.797 rad rotation, net +0.107 mm axial change; invalid initial penetration and noisy motion |

Even with contact reduction this CPU live-BVH configuration was approximately
2,475 times slower than real time. That timing is a diagnostic observation for this
configuration, not a general Newton benchmark or a GPU performance prediction.
The transient fitted lead in the last run was 1.187 mm/revolution, but less than
one eighth turn, a wrong net axial response, and initial penetration make it
**unacceptable as a passing pitch test**. No successful physical-thread demo has
been established by these Newton probes.

The probe writes actual transforms, velocities, contact counts, axial displacement,
unwrapped angle and a fitted axial lead when enough angular excursion exists. These
are measurements, not the policy's prescribed trajectory. Its fit threshold is only
a diagnostic convenience; it does not constitute the required multi-turn validation.

```bash
WARP_CACHE_PATH=/workspace/.cache/warp OMP_NUM_THREADS=3 \
  /workspace/.venvs/newton-thread/bin/python -u \
  outputs/newton_research/cpu_thread_probe.py \
  --solver xpbd --steps 1000 --dt 0.0001 --mu 0.1 \
  --force -0.05 --torque -0.0005 --contacts 12000 \
  --output outputs/newton_research/xpbd_full_buffer_probe.json
```

The external meshes are downloaded from the
[IsaacGymEnvs Factory asset directory](https://github.com/isaac-sim/IsaacGymEnvs/tree/main/assets/factory/mesh/factory_nut_bolt)
to `/workspace/research/newton_factory/`. The research script requires that directory.
It is not a distributable Newton environment or an installed root-project feature.

The raw contact capacities, mesh sign behavior, exactly validated initial pose and
stability must be addressed before interpreting a future rollout. Subsequent engine
comparisons should use the project's validated watertight procedural M8 geometry
instead of inheriting this Factory export issue. This investigation has not run the
Kamino or MuJoCo Warp backend with the threaded pair.

## Required numerical gate before choosing an engine

Use the same M8 x 1.25 right-hand profile, measured clearance, chamfer and material
parameters across engines. The nut should remain free in all six coordinates.
Apply wrenches through contact-driven fingers for the manipulation demonstration;
direct nut wrenches are suitable for an isolated fastener mechanics benchmark only.

At minimum, measure:

1. Emergent axial lead close to 1.25 mm/revolution during established engagement;
   rotation reversal must reverse axial motion without an insertion controller.
2. Axial load support and self-locking at appropriate friction. A no-torque load
   test and a below-engagement test must be different because of geometry.
3. No axial advance through locked thread rotation under moderate axial force;
   no thread teleportation, deep interpenetration, or contact-buffer overflow.
4. Tilt and radial offset recovery/jamming, rejected unfavorable thread starts,
   and a cleared/unthreaded-bore ablation that eliminates the helical behavior.
5. Timestep, solver-iteration, contact-resolution and mesh-resolution convergence.
   Record penetration, force/torque response, solver residuals, wall-clock cost and
   success rates over randomized starts. Do not quietly discard failed starts.
6. Hardware measurements before calling the environment validated for sim-to-real:
   dimensions and tolerances, surface friction, grip force, nut torque versus turns,
   and robot/finger compliance. The video alone cannot supply these quantities.

No evidence collected here establishes Newton as more accurate than a verified
MuJoCo SDF implementation for this application. It establishes the relevant
collision capabilities and the CPU limitation, and provides a transparent
diagnostic path for a future engine comparison.
