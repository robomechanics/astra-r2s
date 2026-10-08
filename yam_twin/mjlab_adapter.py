"""Supported scene-level MJLab bridge for the complete reconstructed workstation.

The scene contains multiple disconnected free bodies and two robot trees. MJLab's
Entity abstraction permits at most one floating root, so the complete MuJoCo
specification is attached through SceneCfg.spec_fn instead. This module creates
an actual MJLab Scene and, when requested, an actual MuJoCo-Warp Simulation.
"""

from __future__ import annotations

import argparse
import importlib.metadata
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

NAMESPACE = "twin/"
MJLAB_SOURCE_SHA = "033ae22a2c7a30a25a6fa77b16c113ed88dd1b55"


def _populate_scene(destination: mujoco.MjSpec) -> None:
    from yam_twin.scene import build_spec

    source = build_spec()
    # MjSpec.attach copies bodies, assets, actuators, equalities and keys, but
    # does not propagate global options. Set them before attaching so the
    # source and destination have no solver configuration conflicts.
    for field in (
        "timestep", "gravity", "integrator", "cone", "solver", "jacobian",
        "iterations", "ls_iterations", "tolerance", "ls_tolerance",
        "impratio", "ccd_iterations", "enableflags", "disableflags",
    ):
        value = getattr(source.option, field)
        setattr(destination.option, field, value.copy() if isinstance(value, np.ndarray) else value)
    destination.njmax = source.njmax
    destination.nconmax = source.nconmax
    destination.attach(
        child=source,
        prefix=NAMESPACE,
        frame=destination.worldbody.add_frame(),
    )


def make_scene_cfg(num_envs: int = 1) -> Any:
    """Return an MJLab SceneCfg containing the entire station specification.

    Names acquire the ``twin/`` prefix. There are no Entity manager wrappers;
    controls are available through ``simulation.data.ctrl`` and joint/object
    state through ``simulation.data.qpos``. MuJoCo-Warp batches every body.
    """
    if num_envs < 1:
        raise ValueError("num_envs must be positive")
    from mjlab.scene import SceneCfg

    return SceneCfg(num_envs=num_envs, env_spacing=2.0, spec_fn=_populate_scene)


def _simulation_cfg(model: mujoco.MjModel) -> Any:
    from mjlab.sim import MujocoCfg, SimulationCfg

    option = model.opt
    integrators = {
        mujoco.mjtIntegrator.mjINT_EULER: "euler",
        mujoco.mjtIntegrator.mjINT_IMPLICITFAST: "implicitfast",
    }
    solvers = {
        mujoco.mjtSolver.mjSOL_NEWTON: "newton",
        mujoco.mjtSolver.mjSOL_CG: "cg",
        mujoco.mjtSolver.mjSOL_PGS: "pgs",
    }
    jacobians = {
        mujoco.mjtJacobian.mjJAC_AUTO: "auto",
        mujoco.mjtJacobian.mjJAC_DENSE: "dense",
        mujoco.mjtJacobian.mjJAC_SPARSE: "sparse",
    }
    if option.integrator not in integrators:
        raise ValueError("MJLab supports Euler or implicitfast for this bridge")

    def flags(enum: Any, prefix: str, mask: int) -> tuple[str, ...]:
        return tuple(
            name.removeprefix(prefix).lower()
            for name in dir(enum)
            if name.startswith(prefix) and int(getattr(enum, name)) & int(mask)
        )

    return SimulationCfg(
        nconmax=256,
        njmax=512,
        mujoco=MujocoCfg(
            timestep=float(option.timestep),
            integrator=integrators[option.integrator],
            gravity=tuple(float(x) for x in option.gravity),
            cone="elliptic" if option.cone == mujoco.mjtCone.mjCONE_ELLIPTIC else "pyramidal",
            solver=solvers[option.solver],
            jacobian=jacobians[option.jacobian],
            iterations=int(option.iterations),
            ls_iterations=int(option.ls_iterations),
            tolerance=float(option.tolerance),
            ls_tolerance=float(option.ls_tolerance),
            impratio=float(option.impratio),
            ccd_iterations=int(option.ccd_iterations),
            disableflags=flags(mujoco.mjtDisableBit, "mjDSBL_", option.disableflags),
            enableflags=flags(mujoco.mjtEnableBit, "mjENBL_", option.enableflags),
        ),
    )


def initialize_simulation(scene: Any) -> Any:
    """Initialize real MJLab/Warp data from the scene's first keyframe.

    Errors from unsupported devices or Warp features propagate to the caller;
    this function never substitutes the native MuJoCo backend.
    """
    import torch
    from mjlab.sim import Simulation

    model = scene.compile()
    simulation = Simulation(
        num_envs=scene.num_envs,
        cfg=_simulation_cfg(model),
        model=model,
        device=scene.device,
    )
    scene.initialize(simulation.mj_model, simulation.model, simulation.data)
    if model.nkey:
        simulation.data.qpos[:] = torch.as_tensor(
            model.key_qpos[0].copy(), dtype=torch.float32, device=scene.device
        )
        simulation.data.ctrl[:] = torch.as_tensor(
            model.key_ctrl[0].copy(), dtype=torch.float32, device=scene.device
        )
        simulation.data.qvel[:] = torch.as_tensor(
            model.key_qvel[0].copy(), dtype=torch.float32, device=scene.device
        )
    simulation.forward()
    return simulation


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="cpu", help="cpu or cuda:0")
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--steps", type=int, default=5)
    parser.add_argument("--compile-only", action="store_true")
    parser.add_argument("--export", type=Path, help="Export MJLab scene.xml and meshes into this directory")
    parser.add_argument("--report", type=Path, help="Save JSON validation report")
    args = parser.parse_args()
    if args.steps < 0:
        parser.error("steps must be nonnegative")

    import torch
    from mjlab.scene import Scene

    scene = Scene(make_scene_cfg(args.num_envs), device=args.device)
    model = scene.compile()
    report = {
        "backend": "mjlab/mujoco-warp",
        "versions": {
            name: importlib.metadata.version(name)
            for name in ("mjlab", "mujoco", "mujoco-warp", "warp-lang", "torch")
        },
        "source_sha": MJLAB_SOURCE_SHA,
        "device": args.device,
        "cuda_available": torch.cuda.is_available(),
        "num_envs": args.num_envs,
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "nbody": model.nbody,
        "neq": model.neq,
        "compiled": True,
        "stepped": False,
        "steps": 0,
    }
    if args.export:
        scene.write(args.export)
        report["export_directory"] = str(args.export.resolve())
    if not args.compile_only:
        simulation = initialize_simulation(scene)
        for _ in range(args.steps):
            simulation.step()
            scene.update(float(model.opt.timestep))
        if not torch.isfinite(simulation.data.qpos).all().item():
            raise RuntimeError("MJLab smoke test produced non-finite joint positions")
        if not torch.isfinite(simulation.data.qvel).all().item():
            raise RuntimeError("MJLab smoke test produced non-finite joint velocities")
        report.update(
            stepped=args.steps > 0,
            steps=args.steps,
            finite=True,
            simulation_time=simulation.data.time.tolist(),
        )
    output = json.dumps(report, indent=2) + "\n"
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(output)
    print(output, end="")


if __name__ == "__main__":
    main()
