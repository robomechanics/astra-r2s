# Geometry-preserving M8 SDF acceleration

The production female SDF now skips the helical profile query when a bound
proves that the exterior hex or end face determines the result. This preserves
the original field and its existing finite-difference normals. It changes no
friction, geometry, solver settings, contact search, or dynamic state.

The original source is preserved byte for byte in
[`tests/fixtures/m8_sdf_reference.cc`](../tests/fixtures/m8_sdf_reference.cc),
including its original registration name and `-mjMAXVAL` hex sentinel.

| Source | SHA-256 |
| --- | --- |
| Original reference | `fc1d703d5406b7856d70d6e8ead3c64d3aec1811af58423610ecd167e39989af` |
| Optimized production | `1c8b5207c5f6c141cc034983ce76c6c1e9cca4114d16e17a4496b94cb637b42a` |

## Why the early return preserves the field

For finite physical parameters and positive pitch, every female profile segment
has cylindrical radius at most `major`. For a query at radius `rho > major`,
the distance to any segment is at least `rho - major`. The helical-plane metric
changes only the phase coordinate and cannot weaken this radial lower bound.
The female thread-hole field is negative there, so its value is at most
`-(rho - major)`.

Let `B` be the original exterior body field and `C` its original chamfer-hole
field. The original final result is `max(B, thread_hole, C)`. Define
`U = max(-(rho - major), C)`. If `U <= B`, the final result must equal `B`.
The new branch returns exactly that original body calculation. If the bound
does not prove dominance, the original kernel runs unchanged.

This accelerates the existing helical-plane distance field. It does not improve
that field's approximation to the exact three-dimensional distance; geometric
accuracy and contact fidelity are validated separately.

## Differential and contact checks

[`scripts/check_m8_optimization.py`](../scripts/check_m8_optimization.py)
compiles temporary reference and production copies under distinct plugin names.
The preserved fixture remains unchanged. Both copies use the installed MuJoCo
headers, core, compiler, and production `-O3` flags.

The final production source passed 500,000 comparisons with byte-identical
distances and every component of the original central finite-difference
gradient at ±10 nm. Every value was finite and both maximum errors were zero.
The samples cover the axis, exterior hex, crest/flank/valley transitions,
chamfers, end faces, constructed upper-bound equality points, six geometry
parameter variations, and the unchanged male thread. The nominal female
parameters include the production 360 µm chamfer.

A fresh 0.2-second free-nut probe compared all 4,000 physics steps. The state
trace was byte-identical for position, velocity, acceleration, constraint
forces, contact count, and energy. Every contact distance, frame, friction,
solver parameter, and force also produced an identical digest. The core was
the verified MuJoCo 3.15.0 engine with micron-scale SDF search, SHA-256
`58039d439c6504448aafd0a4d5b655c8a0d3bf1d77123ac7e0733cd078367433`.

The fresh production 0.6-second nominal probe also matched the entire saved
600 × 18 baseline trace byte for byte. Reuse was accepted only after checking
the original source hash, core hash and search parameters, model XML hash,
configuration, and applied loads. The trace SHA-256 for both versions was
`69941c25ae2733d95e2d6d26dbe5f16c7961e9e1863ee173a25b495be719dd99`.

## Measured query cost

The final microbenchmark pinned its own process to one allowed CPU, alternated
reference/production order, and used the median of nine 300,000-query batches.
Ratios below use CPU time.

| Query population | Distance queries | Complete finite-difference gradients |
| --- | ---: | ---: |
| Exterior gripper pad region | 9.59× faster | 9.09× faster |
| Uniform volume | 2.67× faster | 2.97× faster |
| Bore-heavy near-surface sample | 3.51% more CPU time | 2.04% more CPU time |

These are geometry microbenchmarks. They do not establish complete policy
training throughput. The free-nut probe's exterior branch is rarely useful,
and the matched 0.2-second trajectories had similar run times. The expected
benefit is for physical gripper queries near the exterior hex. Historical and
current nominal-probe run times were measured under different machine loads
and must not be used as a speed comparison.

## Reproducing the checks

Run a quick 5,000-point comparison:

```bash
scripts/run_m8.sh scripts/check_m8_optimization.py
```

Run the full differential, timing, and every-step contact comparison:

```bash
scripts/run_m8.sh scripts/check_m8_optimization.py \
  --points 500000 --timing-points 300000 --pin-cpu --trace-duration .2
```

If the original nominal probe is available, add
`--cached-baseline outputs/m8/acceptance/nominal` to compare its fingerprint and
trace. The generated machine-readable reports are `comparison.json`,
`timing.json`, `trajectory.json`, and optional `cached_probe.json` under
`outputs/m8/optimization/`. The comparison exits with failure on any unequal
or nonfinite field, gradient, state, or contact result.

If the thread profile changes later, reevaluate the radial bound and preserve a
new explicit reference. Do not infer unchanged physics from a performance
measurement alone.
