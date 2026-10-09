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

ROOT = next(parent for parent in Path(__file__).resolve().parents
            if (parent / "thread_lab" / "runtime.py").is_file())
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_m8_insertion_trace import recorded_model
from thread_lab.runtime import require_micron_engine


def audit(path):
    runtime = require_micron_engine()
    with np.load(path) as saved:
        metadata = json.loads(str(saved["metadata_json"]))
    model, identity = recorded_model(path, metadata)
    fingerprint = identity["model_fingerprint"]
    expected = metadata["runtime"]
    runtime_matches = (runtime["mujoco_version"] == expected["mujoco_version"]
        and runtime["thread_plugin_source_sha256"] == expected["thread_plugin_source_sha256"]
        and sorted(lib["sha256"] for lib in runtime["libraries"])
        == sorted(lib["sha256"] for lib in expected["libraries"]))
    if not runtime_matches:
        raise ValueError("Audit core/plugin binaries do not match recorded runtime")
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
    equality_rows = []
    for index in range(model.neq):
        kind = int(model.eq_type[index])
        first, second = int(model.eq_obj1id[index]), int(model.eq_obj2id[index])
        equality_rows.append({"id": index, "type": kind, "object_ids": [first, second],
            "object_names": ([model.joint(value).name if value >= 0 else None for value in (first, second)]
                if kind == int(mujoco.mjtEq.mjEQ_JOINT) else
                [model.body(value).name if value >= 0 else None for value in (first, second)]
                if kind in (int(mujoco.mjtEq.mjEQ_CONNECT), int(mujoco.mjtEq.mjEQ_WELD)) else None)})
    only_native_finger_equalities = (len(equality_rows) == 2 and all(
        row["type"] == int(mujoco.mjtEq.mjEQ_JOINT)
        and set(row["object_names"]) == {f"{side}_left_finger", f"{side}_right_finger"}
        for row, side in zip(equality_rows, ("left", "right"))))
    actuator_rows = [{"name": model.actuator(index).name,
        "transmission_type": int(model.actuator_trntype[index]),
        "target_joint": (model.joint(int(model.actuator_trnid[index, 0])).name
            if model.actuator_trntype[index] == mujoco.mjtTrn.mjTRN_JOINT else None)}
        for index in range(model.nu)]
    expected_joint_targets = {f"{side}_joint{index}" for side in ("left", "right") for index in range(1, 7)}
    expected_joint_targets.update(f"{side}_{finger}_finger" for side in ("left", "right")
                                  for finger in ("left", "right"))
    only_native_robot_joint_actuators = (len(actuator_rows) == 16 and
        {row["target_joint"] for row in actuator_rows} == expected_joint_targets and all(
            row["transmission_type"] == int(mujoco.mjtTrn.mjTRN_JOINT) for row in actuator_rows))
    return {"kind": "Compiled free-workpiece passive-properties audit",
            "method": "Compile exact recorded scene only; zero integration, commands or force solve",
            "scope": "Zero free-joint passive terms excludes inherited joint drag/stiction or extra rotational inertia as the source of open-reset stability. Physical thread/finger contact friction remains enabled; this does not qualify contact-law accuracy.",
            "model_fingerprint": fingerprint, "scene_config": metadata["scene_config"],
            "archived_model_identity": identity,
            "audit_runtime": runtime, "core_and_plugin_match_recorded": runtime_matches,
            "gravity_m_s2": model.opt.gravity.tolist(),
            "fluid_density": float(model.opt.density), "fluid_viscosity": float(model.opt.viscosity),
            "workpieces": properties, "equality_count": int(model.neq),
            "equality_types": model.eq_type.tolist(),
            "equalities": equality_rows,
            "only_native_finger_coupling_equalities": only_native_finger_equalities,
            "free_object_welds_or_other_equalities_absent": only_native_finger_equalities,
            "actuators": actuator_rows,
            "only_native_robot_joint_actuators": only_native_robot_joint_actuators,
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
