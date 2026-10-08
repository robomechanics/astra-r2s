# Matched M8 contact environment

The M8 contact task runs in `/workspace/.venvs/m8-contact`. Its Python
extensions and patched MuJoCo core are built together with GCC/libstdc++.
MuJoCo 3.15.0's PyPI wheel uses a different C++ ABI for its string-bearing
`MjSpec` interface, so preloading this GCC core into that wheel is unsafe.
`scripts/run_m8.sh` uses the matched package directly and verifies its core
and extension hashes before launching a task.

From the repository checkout, run:

```bash
scripts/setup.sh
scripts/run_m8.sh -m pytest -q
scripts/run_m8.sh scripts/check_m8_distance.py
scripts/run_m8.sh -m thread_lab.benchmark --help
```

Setup requires `uv`, Git, GCC, and G++, plus network access for the pinned
dependencies and upstream source. Python 3.12 is selected by `.python-version`.
The ordinary runtime dependencies are hash-locked in `requirements.lock`;
the GCC binding build tools have their own `requirements-build.lock`.
Cloud cache paths stay under `/workspace/.cache` because the account's home
directory is read-only. Rendering uses the available Mesa EGL backend.

The setup script retains three independent runtimes:

| Runtime | Purpose | MuJoCo |
| --- | --- | --- |
| `/workspace/.venvs/astra-r2s` | Original YAM visualization and stock-core diagnostics | Stock 3.15.0 wheel |
| `/workspace/.venvs/m8-contact` | M8 contact experiments and policy interface | Matched GCC 3.15.0 core and bindings |
| `/workspace/.venvs/mjlab-twin` | Original MJLab CPU bridge smoke test | 3.11.0, as required by the pinned MJLab revision |

The core starts from official tag `3.15.0`, commit
`9ea3cdfcae93bf2cc4dc0e1a1627c5a39a1e06e5`. The checked-in patch changes the
SDF search floor to `1e-7` m and initial search step to `.002` m, and exports
markers so a fresh process can verify those values. It does not alter the
thread geometry, contact parameters, or dynamics. The built core's provenance
is saved under `/workspace/research/mujoco-3.15.0-thread-build`.

`scripts/build_mujoco_bindings.sh` uses `PYTHONHASHSEED=0` for MuJoCo's generated
headers, capped build parallelism, and the separate hash-locked build tools.
It records a fingerprint of the source revision, core hash, Python ABI,
lockfiles, and build helpers. A repeat invocation reuses the installed build
only after checking the fingerprint, every extension hash, the core hash,
both search markers, one mapped core, and an `MjSpec` name/edit/ZIP roundtrip.
The binding provenance and verification records live under
`/workspace/research/m8-contact-sdk`.
When a changed build fingerprint requires wheel assembly, the helper also
compares every package file against the installed wheel and avoids replacing
an identical installation. Ordinary repeated setup takes the earlier verified
cache path, without rebuilding or replacing the loaded core.

The launcher clears `LD_PRELOAD`; it does not splice a research library into
an unrelated wheel. Setting `ASTRA_PYTHON` to the stock runtime fails the hash
and marker checks. Advanced builds can set `M8_BINDINGS_VENV`,
`M8_BINDINGS_STAGE`, `M8_BINDINGS_SDK`, `M8_MUJOCO_SOURCE`, and
`M8_MUJOCO_CORE` on the binding helper; use the corresponding
`ASTRA_PYTHON` and `M8_BINDINGS_SDK` when launching that verified installation.

An environment smoke test establishes installation and rendering readiness.
Thread fidelity is evaluated separately by the mechanics acceptance reports;
a passing import or image is insufficient evidence for policy training.

## Verified in this cloud workspace

On 2026-10-08, the complete setup script and its repeat succeeded, preserving
the separate stock and MJLab runtimes. The repeat took 15.741 s. The matched
core SHA-256 was
`58039d439c6504448aafd0a4d5b655c8a0d3bf1d77123ac7e0733cd078367433`.

- Final `scripts/run_m8.sh -m pytest -q`: **64 passed** in 39.87 s, covering the
  original reconstruction, thread geometry/theory, contact response, and Gym
  interface. Gymnasium emitted two warnings about unbounded observation-space
  limits. This suite was rerun after the exact thread-distance optimization
  and the fit-backlash guard correction. Its full log and runtime provenance
  are in `outputs/m8/environment/pytest_final.log` and `pytest_final.json`.
  The launcher separately verified the `MjSpec` ABI roundtrip. The following
  installation and smoke checks were performed earlier on the same date.
- A repeated `scripts/build_mujoco_bindings.sh`: verified cache reused in
  **0.556 s**, with no compiler or wheel replacement.
- `scripts/run_m8.sh scripts/check_m8_distance.py`: passed its 20 targeted
  male-thread probes against independent 3D closest-surface optimization.
  Maximum discrepancy was **0.0008272 µm**, below the 0.01 µm audit threshold;
  this local audit excludes the constructive end-cap/chamfer fields.
- EGL rendering of the compiled M8 gripper scene produced a finite, nonblank
  **640 × 480** RGB image; pixel standard deviation was 27.16.
- The separate MJLab CPU bridge compiled and advanced **5 steps**, with finite
  state, using MJLab 1.6.0, MuJoCo/MuJoCo-Warp 3.11.0, Warp 1.18.0, and
  PyTorch 2.14.1+cpu. It continues to represent the legacy surrogate scene.
- `ASTRA_PYTHON=/workspace/.venvs/astra-r2s/bin/python scripts/run_m8.sh ...`
  exited before the payload with “The installed core does not match the
  verified build.” This confirms the stock ABI cannot silently enter an M8
  acceptance run through the launcher.

Machine-readable local checks and setup logs are in
`outputs/m8/environment/`; these generated workspace files are not required
to rebuild the environment.
