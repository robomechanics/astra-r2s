# MJLab scene bridge

`yam_twin.mjlab_adapter.make_scene_cfg()` creates a real MJLab `SceneCfg` and
attaches the complete reconstructed station through its supported `spec_fn`
callback. The two YAM arms, freely movable board and screwdriver, welds, screw
hinge/slide coupling, actuators, and mesh assets remain in one MuJoCo model.
The callback adds the prefix `twin/` to every model name.

The scene uses this callback because MJLab's `Entity` abstraction allows at
most one free root. Treating the whole two-arm workstation as a single Entity
would reject its multiple floating objects. This bridge provides batched
physics and direct actuator/state access. It does not provide a manager-based
RL task, rewards, learned policy, or per-arm Entity action terms.

## Installation

The integration targets MJLab source commit
`033ae22a2c7a30a25a6fa77b16c113ed88dd1b55` (version 1.6.0), which requires
MuJoCo and MuJoCo-Warp 3.11.x. Use a separate environment so these requirements
do not disturb another simulator installation. Run from the checkout:

```bash
export UV_CACHE_DIR=/workspace/.cache/uv
export WARP_CACHE_PATH=/workspace/.cache/warp
export MPLCONFIGDIR=/workspace/.cache/matplotlib
export XDG_CACHE_HOME=/workspace/.cache
uv venv /workspace/.venvs/mjlab-twin
uv pip install --python /workspace/.venvs/mjlab-twin/bin/python \
  --index https://download.pytorch.org/whl/cpu 'torch>=2.14.0'
uv pip install --python /workspace/.venvs/mjlab-twin/bin/python --no-sources \
  'mjlab @ git+https://github.com/mujocolab/mjlab@033ae22a2c7a30a25a6fa77b16c113ed88dd1b55'
uv pip install --python /workspace/.venvs/mjlab-twin/bin/python --no-deps -e .
```

`--no-sources` uses the public PyPI distribution of Warp rather than MJLab's
optional NVIDIA package index. TLS and package integrity checks remain enabled.
CPU PyTorch is installed first to avoid downloading GPU runtime libraries on
the current CPU machine. For CUDA execution install the compatible CUDA
PyTorch build recommended by the pinned MJLab documentation in a GPU machine.
The writable cache variables are needed in this cloud environment because
the home directory is read-only.

## Compile, export, and step

```bash
/workspace/.venvs/mjlab-twin/bin/python -m yam_twin.mjlab_adapter \
  --compile-only --export outputs/mjlab-scene \
  --report outputs/mjlab-compile.json
/workspace/.venvs/mjlab-twin/bin/python -m yam_twin.mjlab_adapter \
  --device cpu --steps 5 --report outputs/mjlab-smoke.json
```

The export contains `scene.xml` and its mesh assets, using MJLab's own
`Scene.write()` implementation. The second command initializes a real MJLab
`Simulation`, seeds its state/control from the first scene keyframe when
present, advances MuJoCo-Warp, and checks finite positions and velocities.
Backend errors are reported directly; the smoke command never silently
switches to native MuJoCo. The short smoke run holds the initial actuator
targets. The reconstructed screw-driving trajectory is produced by the
project's replay command, not by this smoke test.

On a suitable GPU machine, the same interface supports `--device cuda:0
--num-envs 64`. Physics worlds are independent; the scene's `env_spacing`
setting concerns visual presentation and does not physically duplicate the
bodies inside a single world. GPU execution and RL training need separate
validation on that hardware.

## Python API

```python
from mjlab.scene import Scene
from yam_twin.mjlab_adapter import make_scene_cfg, initialize_simulation

scene = Scene(make_scene_cfg(num_envs=1), device="cpu")
simulation = initialize_simulation(scene)
# simulation.data.ctrl has shape (num_envs, nu).
# simulation.data.qpos has shape (num_envs, nq).
simulation.step()
```

Use the compiled MuJoCo model for indexing, for example
`simulation.mj_model.actuator("twin/left_servo1")` after checking the actual
actuator names in the exported XML. Positions use MuJoCo's generalized
coordinates, including seven coordinates per free body; they are not only
robot joint angles.

## Validation record

The cloud environment has no NVIDIA driver. MJLab 1.6.0, MuJoCo 3.11.0,
MuJoCo-Warp 3.11.0, Warp 1.18.0, and CPU PyTorch 2.14.1 were installed from
the pinned source and verified distributions. The complete station compiled,
initialized, and passed 5 CPU Warp steps with finite generalized positions
and velocities. Two independent batched worlds also passed 10 steps (0.020 s
per world). Results are in `outputs/mjlab-smoke.json` and
`outputs/mjlab-batched-smoke.json`. The model has 32 position coordinates,
30 velocity coordinates, 15 actuators, 35 bodies, and 5 equalities. After the
final fixture and controller changes, two CPU worlds passed 20 steps (0.040 s),
recorded in `outputs/mjlab-final-smoke.json`; the final export includes 18 meshes.

Warp reports that the capsule/cylinder and cylinder/box collision pairs in
this scene receive at most one contact even with `MULTICCD` enabled. This is
a backend contact-manifold limitation; the smoke result establishes that the
scene initializes and advances, rather than validating detailed screw-contact
accuracy. The screw thread remains the project's helical constraint surrogate.
