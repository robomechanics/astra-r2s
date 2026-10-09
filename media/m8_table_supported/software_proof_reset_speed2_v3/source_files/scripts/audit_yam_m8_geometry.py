"""Independent initialization-only audit of the bimanual M8 work poses.

This performs no integration. Nut coordinates are initialized at thirteen
geometrically engaged phases to inspect reachability and clearance; their
prescribed locations do not demonstrate thread lead or grasp retention.
Run with scripts/run_m8.sh scripts/audit_yam_m8_geometry.py.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from thread_lab.runtime import require_micron_engine
from thread_lab.model import ThreadConfig
from yam_twin.kinematics import ArmIK
from yam_twin.m8_scene import (YamM8Config, bolt_rotation, build_model,
                               initial_left_grasp_position, initial_nut_position,
                               jaw_positions, left_grasp_rotation, scene_fingerprint, scene_xml)


def pose_check(ik, position, orientation):
    q = ik.solve(position, orientation, thorough=True)
    p, r = ik.pose()
    margins = np.minimum(q - ik.bounds[0], ik.bounds[1] - q)
    return {
        "joint_positions_rad": q.tolist(),
        "joint_limit_margins_rad": margins.tolist(),
        "minimum_joint_limit_margin_rad": float(margins.min()),
        "position_error_m": float(np.linalg.norm(p - position)),
        "orientation_error_rad": float(np.linalg.norm(
            Rotation.from_matrix(orientation @ r.T).as_rotvec())),
    }


def audit(configuration=None, *, left_orientation=None):
    auditor_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    config = configuration or YamM8Config()
    runtime = require_micron_engine()
    model, data = build_model(config), None
    data = mujoco.MjData(model)
    left, right = ArmIK(model, "left"), ArmIK(model, "right")
    left_position = initial_left_grasp_position(config)
    left_orientation = left_grasp_rotation(config) if left_orientation is None else np.asarray(left_orientation)
    left_result = pose_check(left, left_position, left_orientation)
    data.qpos[left.qadr] = left.q
    # Touching configurations, with no motor preload or simulated retention.
    left_closing_axis_local = bolt_rotation(config).T @ left_orientation[:, 1]
    left_gap = float(np.dot(np.abs(left_closing_axis_local), config.block_size))
    for side, width in (("left", left_gap),
                        ("right", config.thread.nut_across_flats)):
        for finger, q in zip(("left", "right"), jaw_positions(width, config)):
            data.qpos[model.joint(f"{side}_{finger}_finger").qposadr[0]] = q

    bolt_r = bolt_rotation(config)
    axis = bolt_r[:, 2]
    center = initial_nut_position(config)
    orientation0 = (bolt_r @ Rotation.from_euler("z", -90, degrees=True).as_matrix()
                    @ Rotation.from_euler("x", 180, degrees=True).as_matrix())
    nut_address = model.joint("nut_free").qposadr[0]
    samples = []
    for theta in np.linspace(0., -2 * np.pi / 3, 13):
        position = center + axis * config.thread.pitch * theta / (2 * np.pi)
        nut_rotation = bolt_r @ Rotation.from_euler("z", theta).as_matrix()
        data.qpos[nut_address:nut_address + 3] = position
        data.qpos[nut_address + 3:nut_address + 7] = np.roll(
            Rotation.from_matrix(nut_rotation).as_quat(), 1)
        orientation = Rotation.from_rotvec(axis * theta).as_matrix() @ orientation0
        right_result = pose_check(right, position, orientation)
        data.qpos[right.qadr] = right.q
        mujoco.mj_forward(model, data)
        groups = {name: [] for name in ("left_pad_block", "right_pad_nut",
                                        "thread", "unexpected")}
        for contact in data.contact:
            geom_ids = [int(g) for g in contact.geom]
            names = [model.geom(g).name for g in geom_ids]
            pair = set(names)
            if "fixture_block_geom" in pair and any(
                    name.startswith("left_m8_pad_") for name in names):
                group = "left_pad_block"
            elif "nut_thread" in pair and any(
                    name.startswith("right_m8_pad_") for name in names):
                group = "right_pad_nut"
            elif pair == {"bolt_thread", "nut_thread"}:
                group = "thread"
            else:
                group = "unexpected"
            groups[group].append({"geoms": names, "geom_ids": geom_ids,
                                  "bodies": [model.body(int(model.geom_bodyid[g])).name for g in geom_ids],
                                  "signed_distance_m": float(contact.dist),
                                  "position_world_m": contact.pos.tolist()})
        unexpected_penetrations = [c for c in groups["unexpected"]
                                   if c["signed_distance_m"] < 0.]
        samples.append({"angle_rad": float(theta), "angle_degrees": float(np.rad2deg(theta)),
                        "nut_initialized_position_world_m": position.tolist(),
                        "right": right_result, "contacts": groups,
                        "unexpected_penetrating_contact_count": len(unexpected_penetrations),
                        "passed": not unexpected_penetrations})

    mujoco.mj_forward(model, data)
    masses = {name: float(model.body(name).mass[0])
              for name in ("fixture_block", "bolt_frame", "nut")}
    gravity_wrench = np.zeros(6)
    for name, mass in masses.items():
        force = mass * model.opt.gravity
        offset = data.xipos[model.body(name).id] - left_position
        gravity_wrench += np.r_[force, np.cross(offset, force)]
    feed_force = -.05 * axis
    feed_moment = np.cross(center - left_position, feed_force)
    free_names = ("fixture_block_free", "nut_free")
    free_joints = {name: int(model.joint(name).id) for name in free_names}
    actuators = []
    for index in range(model.nu):
        joint = int(model.actuator_trnid[index, 0])
        actuators.append({"name": model.actuator(index).name,
                          "joint": model.joint(joint).name,
                          "control_range": model.actuator_ctrlrange[index].tolist(),
                          "force_range": model.actuator_forcerange[index].tolist(),
                          "force_limited": bool(model.actuator_forcelimited[index])})
    equalities = [{"type": int(model.eq_type[i]),
                   "joint_1": model.joint(int(model.eq_obj1id[i])).name,
                   "joint_2": model.joint(int(model.eq_obj2id[i])).name}
                  for i in range(model.neq)]
    topology_passes = (
        all(model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in free_joints.values())
        and all(a["joint"] not in free_names and a["force_limited"]
                and np.isfinite(a["force_range"]).all()
                and np.isfinite(a["control_range"]).all() for a in actuators)
        and model.neq == 2
        and all(e["type"] == int(mujoco.mjtEq.mjEQ_JOINT)
                and "finger" in e["joint_1"] and "finger" in e["joint_2"] for e in equalities))
    checks = {
        "all_thirteen_poses_have_no_unexpected_penetration": all(s["passed"] for s in samples),
        "joint_limit_margin_above_0_1_rad": min(left_result["minimum_joint_limit_margin_rad"],
            *(s["right"]["minimum_joint_limit_margin_rad"] for s in samples)) > .1,
        "pose_errors_below_2_um_and_20_urad": all(
            p["position_error_m"] < 2e-6 and p["orientation_error_rad"] < 20e-6
            for p in [left_result, *(s["right"] for s in samples)]),
        "free_block_and_nut_only_finite_robot_actuators_and_finger_equalities": bool(topology_passes),
    }
    return {"passed": all(checks.values()), "checks": checks,
            "method": "Independent initialization-only IK and collision audit; zero mj_step calls",
            "scope_limit": "Prescribed nut initialization phases do not validate lead, dynamics, contact forces, or grasp retention",
            "dynamic_steps": 0, "config": asdict(config), "runtime": runtime,
            "model_xml_sha256": hashlib.sha256(scene_xml(config).encode()).hexdigest(),
            "portable_model_fingerprint": scene_fingerprint(config),
            "auditor_sha256": auditor_sha256,
            "left_target_position_world_m": left_position.tolist(),
            "left_target_orientation_world": left_orientation.tolist(),
            "left_initialized_touching_gap_m": left_gap, "left": left_result,
            "masses_kg": masses, "assembly_gravity_wrench_about_left_grip_world": gravity_wrench.tolist(),
            "assumed_right_axial_feed_N": -.05, "right_axial_feed_moment_about_left_grip_world_Nm": feed_moment.tolist(),
            "payload_moment_note": "Gravity wrench uses the last initialized phase; these are load estimates, not measured grip capacity",
            "topology": {"nq": model.nq, "nv": model.nv, "nu": model.nu,
                         "equalities": equalities, "actuators": actuators}, "samples": samples}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/yam_m8/bimanual_geometry_audit.json")
    parser.add_argument("--recorded-trace", type=Path,
                        help="Use and verify the exact portable model configuration recorded in a demo trace")
    args = parser.parse_args()
    configuration, provenance = None, None
    if args.recorded_trace:
        with np.load(args.recorded_trace) as saved:
            metadata = json.loads(str(saved["metadata_json"]))
        values = dict(metadata["scene_config"])
        thread = values.pop("thread")
        for name in ("bolt_position", "bolt_quaternion", "block_size", "bolt_offset", "left_grasp_offset"):
            if name in values:
                values[name] = tuple(values[name])
        configuration = YamM8Config(thread=ThreadConfig(**thread), **values)
        fingerprint = scene_fingerprint(configuration)
        if metadata.get("model_fingerprint") != fingerprint:
            raise ValueError("Recorded model fingerprint does not match the rebuilt scene")
        provenance = {"trace": str(args.recorded_trace),
                      "trace_sha256": hashlib.sha256(args.recorded_trace.read_bytes()).hexdigest(),
                      "recorded_model_fingerprint": metadata["model_fingerprint"]}
    result = audit(configuration)
    if provenance:
        result["recorded_configuration_provenance"] = provenance
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"passed": result["passed"], "checks": result["checks"],
                      "masses_kg": result["masses_kg"], "output": str(args.output)}, indent=2))
    raise SystemExit(0 if result["passed"] else 1)
