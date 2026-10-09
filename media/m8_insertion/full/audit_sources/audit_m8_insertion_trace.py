"""Independent saved-geometry audit of M8 pickup or bounded-fixture entry.

No integration or contact-force solve is performed. All-substep force and
support claims require the original rollout instrumentation. Collision
candidates reconstructed here cannot establish their original normal loads.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.util
import inspect
import json
from pathlib import Path
import sys

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from thread_lab.model import ThreadConfig
from thread_lab.runtime import require_micron_engine
from yam_twin.m8_scene import YamM8Config
from yam_twin.m8_insertion_scene import InsertionConfig, build_model, scene_fingerprint


def full_ring_interval(base_z, cosine, sine, thread):
    """Independently bound complete male rings inside both uncut axial spans."""
    H = np.sqrt(3.)*thread.pitch/2
    radius = thread.male_pitch_diameter/2+3*H/8
    female_cut = 5*H/8+.000360
    half = thread.nut_height/2
    if cosine <= 0:
        return 0., 0., 0.
    lower = max(0., (-half+female_cut-base_z+radius*sine)/cosine)
    upper = min(thread.bolt_length-.000956,
                (half-female_cut-base_z-radius*sine)/cosine)
    return float(lower), float(upper), float(max(0., upper-lower))


def lead_fit(z, yaw, mask, pitch):
    if np.count_nonzero(mask) < 3 or np.ptp(yaw[mask]) <= .15:
        return {"turns": 0., "lead_m": None, "lead_error_percent": None, "passed": False}
    lead = float(np.polyfit(yaw[mask], z[mask], 1)[0]*2*np.pi)
    turns = float(np.ptp(yaw[mask])/(2*np.pi))
    error = float(abs(lead/pitch-1)*100)
    return {"turns": turns, "lead_m": lead, "lead_error_percent": error,
            "residual_range_m": float(np.ptp(z[mask]-pitch*yaw[mask]/(2*np.pi))),
            "passed": bool(error < 2. and turns > .1)}


def benchmark_audit(path):
    report_path = path.parent/"report.json"
    original = json.loads(report_path.read_text())
    thread = ThreadConfig(**original["config"])
    with np.load(path) as saved:
        values = saved["trace"].copy()
        fields = list(saved["fields"])
    cols = {name: values[:, i] for i, name in enumerate(fields)}
    yaw = np.unwrap(cols["yaw"])
    overlap = cols["z"]+thread.bolt_length+thread.nut_height/2
    contact = (cols["thread_contacts"] > 0)
    contact_time = original.get("first_thread_contact_time_s")
    ready = np.zeros(len(values), dtype=bool) if contact_time is None else cols["time"] > contact_time+.2
    broad = (overlap > thread.pitch) & contact & ready
    lengths = np.asarray([full_ring_interval(d, np.cos(t), np.sin(t), thread)[2]
                          for d, t in zip(cols["z"], cols["tilt"])])
    complete = (lengths > thread.pitch) & contact & ready
    geometry = (lengths > thread.pitch) & ready
    loaded = cols["normal"] > 1e-5
    active, longest, run = geometry & loaded, 0, 0
    for flag in active:
        run = run+1 if flag else 0
        longest = max(longest, run)
    sample_dt = float(np.median(np.diff(cols["time"]))) if len(values)>1 else 0.
    wrench = np.column_stack([cols[name] for name in ("fx", "fy", "fz", "tx", "ty", "tz")])
    return {"kind": "Fixed-female, direct bounded-wrench benchmark; no robot-pickup qualification",
            "method": "Independent raw sampled positions, yaw and tilt; no dynamics or contact-force reconstruction",
            "original_report": original,
            "original_report_sha256": hashlib.sha256(report_path.read_bytes()).hexdigest(),
            "geometry_note": "Tilt-aware full-ring length is potential flank overlap. Saved total SDF contacts include cones; interior loaded contacts cannot be established from this trace alone.",
            "recomputed_broad_lead_fit": lead_fit(cols["z"], yaw, broad, thread.pitch),
            "recomputed_complete_ring_geometry_lead_fit": lead_fit(cols["z"], yaw, complete, thread.pitch),
            "recomputed_complete_ring_all_sample_lead_fit": lead_fit(cols["z"], yaw, geometry, thread.pitch),
            "complete_ring_geometry_sample_count": int(np.count_nonzero(geometry)),
            "sampled_loaded_total_contact_duty_in_complete_ring_window": float(
                np.mean(loaded[geometry])) if np.any(geometry) else None,
            "longest_consecutive_loaded_sample_run_duration_s": longest*sample_dt,
            "sampled_total_normal_impulse_in_complete_ring_window_N_s": float(np.trapezoid(
                cols["normal"][geometry], cols["time"][geometry])) if np.count_nonzero(geometry)>1 else 0.,
            "contact_window_scope_note": "Duty, run length and impulse are sampled total SDF-contact diagnostics, including possible cone contacts. They do not certify continuous all-substep loaded full-flank contacts.",
            "recorded_fixture_wrench_component_caps_respected": bool(np.isfinite(wrench).all()
                and np.all(np.abs(wrench) <= np.array([2., 2., .2, .02, .02, .002])+1e-12)),
            "maximum_potential_complete_ring_length_m": float(lengths.max()),
            "complete_ring_sample_count": int(np.count_nonzero(complete)),
            "coaxial_one_pitch_nominal_overlap_threshold_m": float(
                .000956+5*np.sqrt(3)*thread.pitch/16+.000360+thread.pitch),
            "initial_gap_m": float(-overlap[0]),
            "recomputed_rows": [{"time_s": float(t), "nominal_overlap_m": float(o),
                                 "potential_complete_ring_length_m": float(length),
                                 "complete_ring_geometry_and_total_contact_window": bool(mask)}
                                for t, o, length, mask in zip(cols["time"], overlap, lengths, complete)]}


def recorded_configuration(metadata):
    values = dict(metadata["scene_config"])
    base = dict(values.pop("base"))
    thread = ThreadConfig(**base.pop("thread"))
    for name in ("bolt_position", "bolt_quaternion", "block_size", "bolt_offset",
                 "left_grasp_offset", "left_block_contact_impedance"):
        if name in base and base[name] is not None:
            base[name] = tuple(base[name])
    for name in ("block_position", "hole_offset", "bolt_head_position"):
        if name in values:
            values[name] = tuple(values[name])
    return InsertionConfig(base=YamM8Config(thread=thread, **base), **values)


def inspected_source_identity(path):
    """Recompute the exact function-source hash declared by a saved module."""
    spec = importlib.util.spec_from_file_location("yam_twin._insertion_audit_source", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    tree = ast.parse(path.read_text())
    candidates = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Dict):
            continue
        for key, value in zip(node.keys, node.values):
            if isinstance(key, ast.Constant) and key.value == "controller_sha256":
                for child in ast.walk(value):
                    if isinstance(child, ast.GeneratorExp) and isinstance(child.generators[0].iter, ast.Tuple):
                        names = child.generators[0].iter.elts
                        if all(isinstance(name, ast.Name) for name in names):
                            candidates.append([name.id for name in names])
    if len(candidates) != 1:
        return {"source_path": str(path), "module_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "controller_sha256": None, "note": "Function hash list could not be resolved"}
    digest = hashlib.sha256("\n".join(inspect.getsource(getattr(module, name))
                                    for name in candidates[0]).encode()).hexdigest()
    return {"source_path": str(path), "module_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "controller_sha256": digest, "hashed_function_names": candidates[0],
            "support_helper_mentions_welded_body_groups": "body_weldid" in inspect.getsource(module._body_contact)}


def robot_audit(path):
    runtime = require_micron_engine()
    with np.load(path) as saved:
        poses, times = saved["qpos"].copy(), saved["time"].copy()
        commands = saved["controller"].copy()
        records = json.loads(str(saved["info_json"]))
        metadata = json.loads(str(saved["metadata_json"]))
    config = recorded_configuration(metadata)
    fingerprint = scene_fingerprint(config)
    if fingerprint != metadata["model_fingerprint"]:
        raise ValueError("Recorded insertion scene does not reproduce its portable fingerprint")
    model, rows = build_model(config), []
    data = mujoco.MjData(model)
    male, female, block = (model.body(name).id for name in ("male_bolt", "female_frame", "fixture_block"))
    left, right = (model.site(f"{side}_grasp_site").id for side in ("left", "right"))
    male_geom, female_geom, head = (model.geom(name).id for name in ("bolt_thread", "female_thread", "bolt_head"))
    thread, yaw_values, block_poses, held_bolt_poses = config.thread, [], [], []
    for time, pose, record in zip(times, poses, records):
        data.qpos[:] = pose
        mujoco.mj_kinematics(model, data)
        mujoco.mj_collision(model, data)
        fr, mr = data.xmat[female].reshape(3, 3), data.xmat[male].reshape(3, 3)
        relative = fr.T@(data.xpos[male]-data.xpos[female])
        rotation = fr.T@mr
        lower, upper, length = full_ring_interval(relative[2], rotation[2, 2],
                                                 np.linalg.norm(rotation[2, :2]), thread)
        yaw_values.append(np.arctan2(rotation[1, 0], rotation[0, 0]))
        lr = data.site_xmat[left].reshape(3, 3)
        block_poses.append((lr.T@(data.xpos[block]-data.site_xpos[left]),
                            lr.T@data.xmat[block].reshape(3, 3)))
        hr = data.site_xmat[right].reshape(3, 3)
        held_bolt_poses.append((hr.T@(data.xpos[male]+mr@[0.,0.,-config.head_height/2]-data.site_xpos[right]),
                                hr.T@mr))
        counters = {"all_thread_candidates": 0, "interior_flank_candidates": 0,
                    "male_world_support_candidates": 0, "block_world_support_candidates": 0,
                    "head_block_seating_candidates": 0, "right_hand_bolt_candidates": 0}
        H = np.sqrt(3)*thread.pitch/2
        unexpected = []
        for contact in data.contact:
            pair = set(map(int, contact.geom))
            bodies = [int(model.geom_bodyid[g]) for g in contact.geom]
            welds = {int(model.body_weldid[b]) for b in bodies}
            if male in welds and 0 in welds:
                counters["male_world_support_candidates"] += 1
            if block in welds and 0 in welds:
                counters["block_world_support_candidates"] += 1
            if head in pair and block in welds:
                counters["head_block_seating_candidates"] += 1
            if male in welds and any(model.body(b).name.startswith("right_") for b in bodies):
                counters["right_hand_bolt_candidates"] += 1
            if pair == {male_geom, female_geom}:
                counters["all_thread_candidates"] += 1
                mz = (mr.T@(contact.pos-data.xpos[male]))[2]
                fz = (fr.T@(contact.pos-data.xpos[female]))[2]
                if 0 < mz < thread.bolt_length-.000956 and abs(fz) < thread.nut_height/2-(5*H/8+.000360):
                    counters["interior_flank_candidates"] += 1
            names = [model.geom(int(g)).name for g in contact.geom]
            expected = (pair == {male_geom, female_geom}
                or (head in pair and any(name.startswith("right_m8_pad_") for name in names))
                or (block in welds and any(name.startswith("left_m8_pad_") for name in names))
                or (head in pair and any(name.startswith("bolt_rest_pin_") for name in names))
                or (head in pair and block in welds))
            if not expected and contact.dist < 0:
                unexpected.append({"geoms": names, "geom_ids": list(map(int,contact.geom)),
                                   "bodies": [model.body(b).name for b in bodies],
                                   "signed_distance_m": float(contact.dist)})
        rows.append({"time_s": float(time), "phase": record["phase"],
                     "male_base_z_in_female_m": float(relative[2]),
                     "radial_base_offset_m": float(np.linalg.norm(relative[:2])),
                     "axis_tilt_rad": float(np.arccos(np.clip(rotation[2, 2], -1., 1.))),
                     "potential_complete_ring_interval_m": [lower, upper],
                     "potential_complete_ring_length_m": length,
                     "recorded_thread_engaged": bool(record["thread_engaged"]),
                     "unexpected_sampled_penetrating_contacts": unexpected, **counters})
    yaw = np.unwrap(yaw_values)
    baseline = next((i-1 for i, row in enumerate(rows) if row["phase"] != rows[0]["phase"]), len(rows)-1)
    reference_p, reference_r = block_poses[max(0, baseline)]
    right_baseline = max((i for i,row in enumerate(rows) if row["phase"]=="settle_bolt"), default=None)
    for row, angle, (position, rotation) in zip(rows, yaw, block_poses):
        row["male_yaw_unwrapped_in_female_rad"] = float(angle)
        row["left_block_post_clamp_slip_m"] = float(np.linalg.norm(position-reference_p))
        row["left_block_post_clamp_rotation_rad"] = float(np.linalg.norm(
            Rotation.from_matrix(rotation@reference_r.T).as_rotvec()))
    grip_reference = None
    grip_references = []
    for index, (row, record, held_pose) in enumerate(zip(rows, records, held_bolt_poses)):
        opening = row["phase"].startswith(("release_", "open_", "reset_open_"))
        if opening:
            grip_reference = None
            continue
        if right_baseline is not None and index == right_baseline:
            grip_reference = held_pose
            grip_references.append(row["time_s"])
        elif grip_reference is None and row["phase"].startswith("regrip_"):
            if np.all(np.asarray(record["contact"]["pad_normal_force_N"]) > .1):
                grip_reference = held_pose
                grip_references.append(row["time_s"])
        if grip_reference is not None:
            position, rotation = held_pose
            rp, rr = grip_reference
            row["right_bolt_post_grasp_slip_m"] = float(np.linalg.norm(position-rp))
            row["right_bolt_post_grasp_rotation_rad"] = float(np.linalg.norm(
                Rotation.from_matrix(rotation@rr.T).as_rotvec()))
    phases, start = [], 0
    while start < len(rows):
        end = start
        while end+1 < len(rows) and rows[end+1]["phase"] == rows[start]["phase"]:
            end += 1
        before, selected = max(0, start-1), rows[start:end+1]
        advance = rows[end]["male_base_z_in_female_m"]-rows[before]["male_base_z_in_female_m"]
        angle = yaw[end]-yaw[before]
        phases.append({"phase": rows[start]["phase"], "sampled_advance_m": advance,
                       "sampled_rotation_rad": float(angle),
                       "sampled_lead_residual_m": float(advance-thread.pitch*angle/(2*np.pi)),
                       "started_with_recorded_engagement": selected[0]["recorded_thread_engaged"],
                       "minimum_potential_complete_ring_length_m": min(r["potential_complete_ring_length_m"] for r in selected),
                       "unexpected_sampled_penetrating_contact_count": sum(
                           len(r["unexpected_sampled_penetrating_contacts"]) for r in selected),
                       **{f"maximum_{key}": max(r[key] for r in selected) for key in counters}})
        start = end+1
    margins = {}
    for side in ("left", "right"):
        joints = [model.joint(f"{side}_joint{i}").id for i in range(1, 7)]
        values = poses[:, model.jnt_qposadr[joints]]
        limits = model.jnt_range[joints]
        margins[side] = float(np.minimum(values-limits[:, 0], limits[:, 1]-values).min())
    saved_source = path.parent/"controller_source.py"
    identity = inspected_source_identity(saved_source if saved_source.exists() else ROOT/"yam_twin/m8_insertion_simulation.py")
    expected_runtime = metadata["runtime"]
    runtime_match = (runtime["mujoco_version"]==expected_runtime["mujoco_version"]
        and runtime["thread_plugin_source_sha256"]==expected_runtime["thread_plugin_source_sha256"]
        and sorted(lib["sha256"] for lib in runtime["libraries"])
            == sorted(lib["sha256"] for lib in expected_runtime["libraries"]))
    return {"kind": "Actual YAM pickup/insertion saved-geometry replay",
            "method": "Independent saved-pose kinematics and collision queries; zero integration or force solve",
            "derived_pose_note": "Original mj_step diagnostics can retain preceding-step derived poses; replay recomputes from integrated qpos",
            "geometry_scope_note": "Potential full-ring overlap and interior collision candidates do not prove loaded thread engagement",
            "model_fingerprint": fingerprint, "recorded_metadata": metadata,
            "audit_runtime": runtime, "inspected_controller_source": identity,
            "controller_source_matches_recorded": identity["controller_sha256"] == metadata["controller_sha256"],
            "runtime_core_and_plugin_identities_match_recorded": runtime_match,
            "post_clamp_reference_time_s": rows[max(0, baseline)]["time_s"],
            "maximum_post_clamp_block_slip_m": max(r["left_block_post_clamp_slip_m"] for r in rows[max(0, baseline):]),
            "maximum_post_clamp_block_rotation_rad": max(r["left_block_post_clamp_rotation_rad"] for r in rows[max(0, baseline):]),
            "right_post_grasp_reference_time_s": rows[right_baseline]["time_s"] if right_baseline is not None else None,
            "right_post_grasp_and_regrasp_reference_times_s": grip_references,
            "right_retention_scope_note": "First reference is end of pickup settle; intentional opening clears it, and each loaded regrasp establishes its own reference. Open hand motion is not classified as grip slip.",
            "maximum_post_grasp_right_bolt_slip_m": max((r.get("right_bolt_post_grasp_slip_m",0.) for r in rows),default=None),
            "maximum_post_grasp_right_bolt_rotation_rad": max((r.get("right_bolt_post_grasp_rotation_rad",0.) for r in rows),default=None),
            "actual_sampled_arm_joint_margins_rad": margins,
            "commands_inside_declared_ranges": bool(np.isfinite(commands).all()
                and np.all(commands >= model.actuator_ctrlrange[:, 0]) and np.all(commands <= model.actuator_ctrlrange[:, 1])),
            "free_objects_unactuated": bool(not {model.joint(name).id for name in
                ("fixture_block_free", "male_bolt_free")}.intersection(model.actuator_trnid[:, 0])),
            "equality_count": int(model.neq), "equality_types": model.eq_type.tolist(),
            "phases": phases, "recomputed_rows": rows}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    with np.load(args.trajectory) as saved:
        robot = "qpos" in saved
    result = robot_audit(args.trajectory) if robot else benchmark_audit(args.trajectory)
    result.update(auditor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  trajectory=str(args.trajectory),
                  trajectory_sha256=hashlib.sha256(args.trajectory.read_bytes()).hexdigest(),
                  force_scope_note="Original loaded-contact and between-sample force/support checks remain the rollout's evidence; they cannot be reconstructed from saved poses")
    output = args.output or args.trajectory.parent/"independent_insertion_audit.json"
    output.write_text(json.dumps(result, indent=2)+"\n")
    print(json.dumps({"output": str(output), "kind": result["kind"]}, indent=2))
