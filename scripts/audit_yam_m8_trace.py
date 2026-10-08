"""Recompute bimanual grip and thread motion from an observed saved trajectory.

Forward kinematics and collision queries are replayed; no dynamics or contact-force solve runs.
Forces, contact counts and between-sample drive checks remain evidence reported
by the original physics rollout, not independently reconstructed here.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from thread_lab.build_plugin import build_plugin
from thread_lab.model import ThreadConfig
from thread_lab.runtime import require_micron_engine
from yam_twin.m8_scene import YamM8Config, build_model, scene_fingerprint, scene_xml


def audit(path: Path, scene: Path, *, use_saved_xml=False):
    auditor_sha256 = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    runtime = require_micron_engine()
    with np.load(path) as saved:
        qpos, times = saved["qpos"].copy(), saved["time"].copy()
        controls = saved["controller"].copy() if "controller" in saved else None
        records = json.loads(str(saved["info_json"]))
        metadata = json.loads(str(saved["metadata_json"]))
    portable_fingerprint = None
    if metadata.get("scene_config") and not use_saved_xml:
        values = dict(metadata["scene_config"])
        thread = values.pop("thread", metadata.get("thread_config", {}))
        for key in ("bolt_position", "bolt_quaternion", "block_size", "bolt_offset", "left_grasp_offset"):
            if key in values:
                values[key] = tuple(values[key])
        configuration = YamM8Config(thread=ThreadConfig(**thread), **values)
        portable_fingerprint = scene_fingerprint(configuration)
        expected = metadata.get("model_fingerprint")
        if expected:
            matches = expected == portable_fingerprint
        else:
            matches = metadata.get("model_xml_sha256") == hashlib.sha256(
                scene_xml(configuration).encode()).hexdigest()
        if not matches:
            raise ValueError("Recorded model provenance does not match the rebuilt scene; use --use-saved-xml only for an intentional historical audit")
        model = build_model(configuration)
        model_source = "Rebuilt recorded scene_config, with verified portable fingerprint or legacy XML hash"
    else:
        mujoco.mj_loadPluginLibrary(str(build_plugin()))
        model = mujoco.MjModel.from_xml_path(str(scene.resolve()))
        model_source = "Explicit saved XML; original absolute asset paths may require the original checkout"
    data = mujoco.MjData(model)
    if not len(qpos) or len(qpos) != len(records):
        raise ValueError("A nonempty trajectory must have one diagnostic row per saved pose")
    nut, bolt, block = (model.body(n).id for n in ("nut", "bolt_frame", "fixture_block"))
    left_site = model.site("left_grasp_site").id
    rows = []
    block_translations, block_rotations, yaw_values = [], [], []
    for t, pose, record in zip(times, qpos, records):
        data.qpos[:] = pose
        mujoco.mj_kinematics(model, data)
        mujoco.mj_collision(model, data)
        bolt_r = data.xmat[bolt].reshape(3, 3)
        nut_r = data.xmat[nut].reshape(3, 3)
        left_r = data.site_xmat[left_site].reshape(3, 3)
        block_r = data.xmat[block].reshape(3, 3)
        relative_nut = bolt_r.T @ (data.xpos[nut] - data.xpos[bolt])
        relative_rotation = bolt_r.T @ nut_r
        block_translation = left_r.T @ (data.xpos[block] - data.site_xpos[left_site])
        block_rotation = left_r.T @ block_r
        block_translations.append(block_translation)
        block_rotations.append(block_rotation)
        yaw_values.append(np.arctan2(relative_rotation[1, 0], relative_rotation[0, 0]))
        unexpected_contacts = []
        thread_depth = 0.
        for contact in data.contact:
            geom_ids = [int(g) for g in contact.geom]
            names = [model.geom(g).name for g in geom_ids]
            pair = set(names)
            expected = (pair == {"bolt_thread", "nut_thread"}
                        or ("fixture_block_geom" in pair and any(
                            name.startswith("left_m8_pad_") for name in names))
                        or ("nut_thread" in pair and any(
                            name.startswith("right_m8_pad_") for name in names)))
            if pair == {"bolt_thread", "nut_thread"}:
                thread_depth = max(thread_depth, -float(contact.dist))
            if not expected and contact.dist < 0:
                unexpected_contacts.append({"geoms": names,
                                            "geom_ids": geom_ids,
                                            "bodies": [model.body(int(model.geom_bodyid[g])).name for g in geom_ids],
                                            "signed_distance_m": float(contact.dist)})
        rows.append({"time_s": float(t), "phase": record["phase"],
                     "nut_z_in_current_bolt_frame_m": float(relative_nut[2]),
                     "nut_radial_offset_m": float(np.linalg.norm(relative_nut[:2])),
                     "nut_tilt_relative_to_current_bolt_rad": float(np.arccos(
                         np.clip(relative_rotation[2, 2], -1., 1.))),
                     "block_position_in_left_grip_m": block_translation.tolist(),
                     "block_bottom_world_z_m": float(data.xpos[block, 2] - np.sum(
                         np.abs(block_r[2]) * model.geom("fixture_block_geom").size)),
                     "recorded_reported_sdf_depth_m": record.get("reported_sdf_depth_m"),
                     "recomputed_reported_sdf_depth_m": thread_depth,
                     "unexpected_sampled_penetrating_contacts": unexpected_contacts})
    yaw = np.unwrap(np.asarray(yaw_values))
    pitch = metadata.get("thread_config", metadata.get("scene_config", {}).get("thread", {})).get("pitch")
    if pitch is None:
        raise ValueError("Trajectory metadata must declare its thread pitch")
    for row, theta in zip(rows, yaw):
        row["nut_yaw_in_current_bolt_frame_unwrapped_rad"] = float(theta)
    first_turn = next((i for i, row in enumerate(rows) if row["phase"].startswith("turn_")), None)
    first_phase_end = next((i - 1 for i, row in enumerate(rows)
                            if row["phase"] != rows[0]["phase"]), len(rows) - 1)
    baseline = max(0, first_turn - 1) if first_turn is not None else first_phase_end
    reference_p, reference_r = block_translations[baseline], block_rotations[baseline]
    for row, p, r in zip(rows, block_translations, block_rotations):
        row["block_slip_from_post_clamp_reference_m"] = float(np.linalg.norm(p - reference_p))
        row["block_rotation_from_post_clamp_reference_rad"] = float(np.linalg.norm(
            Rotation.from_matrix(r @ reference_r.T).as_rotvec()))
    phases = []
    start = 0
    while start < len(rows):
        end = start
        while end + 1 < len(rows) and rows[end + 1]["phase"] == rows[start]["phase"]:
            end += 1
        before = max(0, start - 1)
        advance = rows[before]["nut_z_in_current_bolt_frame_m"] - rows[end]["nut_z_in_current_bolt_frame_m"]
        angle = yaw[end] - yaw[before]
        phase_rows = rows[start:end + 1]
        summary = {"phase": rows[start]["phase"], "start_time_s": rows[before]["time_s"],
                   "end_time_s": rows[end]["time_s"], "sampled_axial_advance_m": float(advance),
                   "sampled_relative_nut_rotation_rad": float(angle),
                   "sampled_helix_residual_m": float(advance + pitch * angle / (2 * np.pi)),
                   "maximum_left_relative_block_slip_m": max(
                       r["block_slip_from_post_clamp_reference_m"] for r in phase_rows),
                   "maximum_left_relative_block_rotation_rad": max(
                       r["block_rotation_from_post_clamp_reference_rad"] for r in phase_rows),
                   "maximum_nut_radial_offset_m": max(r["nut_radial_offset_m"] for r in phase_rows),
                   "maximum_nut_relative_tilt_rad": max(
                       r["nut_tilt_relative_to_current_bolt_rad"] for r in phase_rows),
                   "minimum_block_bottom_world_z_m": min(r["block_bottom_world_z_m"] for r in phase_rows)}
        summary["unexpected_sampled_penetrating_contact_count"] = sum(
            len(r["unexpected_sampled_penetrating_contacts"]) for r in phase_rows)
        late_start = next(i for i in range(start, end + 1)
                          if rows[i]["time_s"] >= rows[end]["time_s"] - .1)
        late_dt = rows[end]["time_s"] - rows[late_start]["time_s"]
        late_slip = max(float(np.linalg.norm(block_translations[i] - block_translations[late_start]))
                        for i in range(late_start, end + 1))
        late_angle = max(float(np.linalg.norm(Rotation.from_matrix(
            block_rotations[i] @ block_rotations[late_start].T).as_rotvec()))
                         for i in range(late_start, end + 1))
        summary.update(late_window_duration_s=late_dt,
                       late_window_left_relative_block_translation_range_m=late_slip,
                       late_window_left_relative_block_rotation_range_rad=late_angle,
                       late_window_block_rotation_range_over_duration_rad_s=late_angle / late_dt if late_dt else None)
        left_contacts = [records[i].get("left_contact") for i in range(late_start, end + 1)]
        left_contacts = [c for c in left_contacts if c]
        if left_contacts:
            summary["recorded_late_mean_left_pad_normal_forces_N"] = np.mean(
                [c["pad_normal_force_N"] for c in left_contacts], axis=0).tolist()
            summary["recorded_late_mean_left_pad_wrench_world_about_block"] = np.mean(
                [c["pad_wrench_world"] for c in left_contacts], axis=0).tolist()
        if summary["phase"].startswith("turn_"):
            limit = .02 * pitch * abs(metadata["control_config"]["stroke_angle_rad"]) / (2 * np.pi)
            summary.update(phase_lead_error_limit_m=limit,
                           sampled_phase_lead_passed=abs(summary["sampled_helix_residual_m"]) < limit)
        phases.append(summary)
        start = end + 1
    recorded_z_error, recorded_yaw_error = [], []
    for row, record in zip(rows, records):
        if "nut_z" in record:
            recorded_z_error.append(abs(row["nut_z_in_current_bolt_frame_m"] - record["nut_z"]))
        if "nut_yaw_unwrapped" in record:
            recorded_yaw_error.append(abs(row["nut_yaw_in_current_bolt_frame_unwrapped_rad"] - record["nut_yaw_unwrapped"]))
    finite_caps = None
    maximum_motor_command = None
    if controls is not None and controls.shape == (len(rows), model.nu):
        lower, upper = model.actuator_ctrlrange.T
        finite_caps = bool(np.isfinite(controls).all() and np.all(controls >= lower - 1e-12)
                           and np.all(controls <= upper + 1e-12))
        maximum_motor_command = np.max(np.abs(controls), axis=0).tolist()
    equalities = []
    for i in range(model.neq):
        kind = int(model.eq_type[i])
        resolver = model.joint if kind == int(mujoco.mjtEq.mjEQ_JOINT) else model.body
        equalities.append({"type": kind, "object_1": resolver(int(model.eq_obj1id[i])).name,
                           "object_2": resolver(int(model.eq_obj2id[i])).name})
    free_joints = {name: int(model.joint(name).id) for name in ("nut_free", "fixture_block_free")}
    free_objects_unactuated = all(joint not in model.actuator_trnid[:, 0]
                                  for joint in free_joints.values())
    return {"method": "Independent saved-pose forward kinematics and collision queries; zero integration or contact-force solve",
            "scope_limit": "Sampled poses cannot certify between-sample drive absence, maximum depth, or contact forces",
            "trajectory": str(path), "trajectory_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "saved_scene": str(scene), "saved_scene_sha256": hashlib.sha256(scene.read_bytes()).hexdigest() if scene.exists() else None,
            "model_source": model_source, "rebuilt_portable_model_fingerprint": portable_fingerprint,
            "auditor_sha256": auditor_sha256,
            "audit_runtime": runtime, "recorded_rollout_metadata": metadata,
            "post_clamp_reference_index": baseline, "post_clamp_reference_time_s": rows[baseline]["time_s"],
            "post_clamp_reference_note": "Last saved pose preceding the first turn, or end of the first phase for a clamp-only trace",
            "phase_boundary_note": "Phase deltas use the preceding phase's last sampled pose and the current phase's last sampled pose; original all-substep acceptance gates remain authoritative",
            "derived_pose_note": "A recorded mj_step diagnostic may retain the preceding step's derived pose; replay recomputes kinematics from the saved integrated qpos",
            "initial_to_reference_block_translation_m": float(np.linalg.norm(
                block_translations[baseline] - block_translations[0])),
            "initial_to_reference_block_rotation_rad": float(np.linalg.norm(Rotation.from_matrix(
                block_rotations[baseline] @ block_rotations[0].T).as_rotvec())),
            "post_clamp_maximum_left_relative_block_slip_m": max(
                row["block_slip_from_post_clamp_reference_m"] for row in rows[baseline:]),
            "post_clamp_maximum_left_relative_block_rotation_rad": max(
                row["block_rotation_from_post_clamp_reference_rad"] for row in rows[baseline:]),
            "maximum_recorded_vs_recomputed_bolt_relative_z_error_m": max(recorded_z_error, default=None),
            "maximum_recorded_vs_recomputed_bolt_relative_yaw_error_rad": max(recorded_yaw_error, default=None),
            "sampled_actuator_commands_within_declared_ranges": finite_caps,
            "maximum_absolute_sampled_actuator_command": maximum_motor_command,
            "free_block_and_nut_have_no_actuator": bool(free_objects_unactuated),
            "equalities": equalities,
            "recorded_between_sample_drive_checks": {name: check for name, check in
                metadata.get("acceptance_checks", {}).items() if "drive" in name or "force" in name},
            "drive_check_note": "Applied-force absence cannot be recovered from qpos; reported checks require original rollout instrumentation/source review",
            "phases": phases, "recomputed_rows": rows}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--use-saved-xml", action="store_true", help="Audit a historical saved XML instead of rebuilding recorded configuration")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    scene = args.scene or args.trajectory.parent / "scene.xml"
    output = args.output or args.trajectory.parent / "independent_trace_audit.json"
    result = audit(args.trajectory, scene, use_saved_xml=args.use_saved_xml)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"output": str(output), "phases": result["phases"],
                      "sampled_actuator_commands_within_declared_ranges":
                          result["sampled_actuator_commands_within_declared_ranges"]}, indent=2))
