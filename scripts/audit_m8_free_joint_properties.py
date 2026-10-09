"""Compile the recorded insertion scene and expose passive workpiece properties.

This read-only audit performs zero integration, controller commands or force
solves. It checks for artificial free-joint drag, stiction, armature, springs,
gravity compensation and object actuators in the exact recorded model.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from audit_m8_insertion_trace import recorded_configuration
from thread_lab.runtime import require_micron_engine
from yam_twin.m8_insertion_scene import build_model, scene_fingerprint


def audit(path):
    runtime = require_micron_engine()
    with np.load(path) as saved:
        metadata = json.loads(str(saved["metadata_json"]))
    config = recorded_configuration(metadata)
    fingerprint = scene_fingerprint(config)
    if fingerprint != metadata["model_fingerprint"]:
        raise ValueError("Recorded scene does not reproduce its portable fingerprint")
    expected = metadata["runtime"]
    runtime_matches = (runtime["mujoco_version"] == expected["mujoco_version"]
        and runtime["thread_plugin_source_sha256"] == expected["thread_plugin_source_sha256"]
        and sorted(lib["sha256"] for lib in runtime["libraries"])
        == sorted(lib["sha256"] for lib in expected["libraries"]))
    if not runtime_matches:
        raise ValueError("Audit core/plugin binaries do not match recorded runtime")
    model = build_model(config)
    properties = []
    for body_name, joint_name in (("fixture_block", "fixture_block_free"), ("male_bolt", "male_bolt_free")):
        body, joint = model.body(body_name).id, model.joint(joint_name).id
        adr = int(model.jnt_dofadr[joint])
        damping = model.dof_damping[adr:adr + 6]
        friction = model.dof_frictionloss[adr:adr + 6]
        armature = model.dof_armature[adr:adr + 6]
        stiffness = float(model.jnt_stiffness[joint])
        actuators = np.flatnonzero((model.actuator_trntype == mujoco.mjtTrn.mjTRN_JOINT)
                                  & (model.actuator_trnid[:, 0] == joint))
        properties.append({"body": body_name, "joint": joint_name,
            "joint_type": int(model.jnt_type[joint]),
            "is_free_joint": bool(model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE),
            "dof_damping": damping.tolist(), "dof_frictionloss": friction.tolist(),
            "dof_armature": armature.tolist(), "joint_stiffness": stiffness,
            "body_gravity_compensation_fraction": float(model.body_gravcomp[body]),
            "joint_actuator_ids": actuators.tolist(),
            "body_mass_kg": float(model.body_mass[body]),
            "principal_inertia_kg_m2": model.body_inertia[body].tolist(),
            "all_tested_passive_artificial_terms_zero": bool(
                not np.any(damping) and not np.any(friction) and not np.any(armature)
                and stiffness == 0 and model.body_gravcomp[body] == 0 and len(actuators) == 0)})
    return {"kind": "Compiled free-workpiece passive-properties audit",
            "method": "Compile exact recorded scene only; zero integration, commands or force solve",
            "scope": "Zero free-joint passive terms excludes inherited joint drag/stiction or extra rotational inertia as the source of open-reset stability. Physical thread/finger contact friction remains enabled; this does not qualify contact-law accuracy.",
            "model_fingerprint": fingerprint, "scene_config": metadata["scene_config"],
            "audit_runtime": runtime, "core_and_plugin_match_recorded": runtime_matches,
            "gravity_m_s2": model.opt.gravity.tolist(),
            "fluid_density": float(model.opt.density), "fluid_viscosity": float(model.opt.viscosity),
            "workpieces": properties, "equality_count": int(model.neq),
            "equality_types": model.eq_type.tolist(),
            "reproduce_command": f"scripts/run_m8.sh scripts/audit_m8_free_joint_properties.py {path}",
            "auditor_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "trajectory": str(path), "trajectory_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit(args.trajectory)
    output = args.output or args.trajectory.parent / "independent_free_joint_properties.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"output": str(output), "workpiece_passive_terms_all_zero": all(
        p["all_tested_passive_artificial_terms_zero"] for p in result["workpieces"])}, indent=2))
