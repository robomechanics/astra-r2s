"""Count every saved-pose workpiece collision candidate during open resets.

Only the true thread pair and left-pad/block contacts are expected during a
reset. Candidates are counted regardless of signed distance. This is a sampled
geometry audit, with zero integration and no force reconstruction.
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
# Prefer the accompanying hash-bound auditor archive when run from a package.
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from audit_m8_insertion_trace import recorded_model
from thread_lab.runtime import require_micron_engine


def audit(path):
    runtime = require_micron_engine()
    with np.load(path) as saved:
        poses = saved["qpos"].copy()
        times = saved["time"].copy()
        records = json.loads(str(saved["info_json"]))
        metadata = json.loads(str(saved["metadata_json"]))
    model, identity = recorded_model(path, metadata)
    fingerprint = identity["model_fingerprint"]
    data = mujoco.MjData(model)
    male, female, block = [model.body(n).id for n in ("male_bolt", "female_frame", "fixture_block")]
    thread_pair = {model.geom(n).id for n in ("bolt_thread", "female_thread")}
    head = model.geom("bolt_head").id
    left_pads = {model.geom(f"left_m8_pad_{side}").id for side in ("left", "right")}
    summaries = {row["phase"]: row for row in metadata.get("phases", [])}
    result_phases = []
    for label in dict.fromkeys(row["phase"] for row in records if row["phase"].startswith("reset_open_")):
        indices = [i for i, row in enumerate(records) if row["phase"] == label]
        before = max(0, indices[0] - 1)
        values, rows = [], []
        for index in [before] + indices:
            data.qpos[:] = poses[index]
            mujoco.mj_kinematics(model, data)
            fr = data.xmat[female].reshape(3, 3)
            mr = data.xmat[male].reshape(3, 3)
            relative = fr.T @ (data.xpos[male] - data.xpos[female])
            rr = fr.T @ mr
            values.append((float(relative[2]), float(np.arctan2(rr[1, 0], rr[0, 0]))))
            if index == before:
                continue
            mujoco.mj_collision(model, data)
            counters = {"expected_thread_candidates": 0, "expected_left_pad_block_candidates": 0,
                        "world_workpiece_candidates": 0, "head_block_seating_candidates": 0,
                        "other_robot_workpiece_candidates": 0, "unrelated_scene_candidates": 0}
            unexpected = []
            for c in data.contact:
                pair = set(map(int, c.geom))
                bodies = [int(model.geom_bodyid[int(g)]) for g in c.geom]
                welds = {int(model.body_weldid[b]) for b in bodies}
                if not ({male, block} & welds):
                    counters["unrelated_scene_candidates"] += 1
                    continue
                expected_thread = pair == thread_pair
                expected_left_grip = block in welds and bool(pair & left_pads)
                counters["expected_thread_candidates"] += expected_thread
                counters["expected_left_pad_block_candidates"] += expected_left_grip
                counters["world_workpiece_candidates"] += 0 in welds
                counters["head_block_seating_candidates"] += head in pair and block in welds
                robot_contact = any(model.body(b).name.startswith(("left_", "right_")) for b in bodies)
                counters["other_robot_workpiece_candidates"] += robot_contact and not expected_left_grip
                if not (expected_thread or expected_left_grip):
                    unexpected.append({"geom_ids": list(map(int, c.geom)),
                                       "geoms": [model.geom(int(g)).name for g in c.geom],
                                       "bodies": [model.body(b).name for b in bodies],
                                       "welded_bodies": [model.body(b).name for b in sorted(welds)],
                                       "signed_distance_m": float(c.dist)})
            rows.append({"time_s": float(times[index]), "unexpected_workpiece_candidates": unexpected,
                         **{name: int(value) for name, value in counters.items()}})
        values = np.asarray(values)
        angles = np.unwrap(values[:, 1])
        recorded_summary = summaries.get(label)
        result_phases.append({
            "phase": label,
            "recorded_all_substep_phase_summary": recorded_summary,
            "phase_started_with_recorded_candidate_capture": (
                bool(recorded_summary["started_engaged"]) if recorded_summary is not None
                else None),
            "first_saved_reset_sample_has_candidate_capture": bool(records[indices[0]]["thread_engaged"]),
            "sample_count": len(rows),
            "sampled_peak_absolute_axial_drift_m": float(np.max(np.abs(values[1:, 0] - values[0, 0]))),
            "sampled_peak_absolute_yaw_drift_rad": float(np.max(np.abs(angles[1:] - angles[0]))),
            "unexpected_workpiece_candidate_count": sum(len(r["unexpected_workpiece_candidates"]) for r in rows),
            "all_sampled_workpiece_candidates_are_thread_or_left_grip": not any(r["unexpected_workpiece_candidates"] for r in rows),
            **{f"maximum_{name}": max(r[name] for r in rows) for name in counters},
            "rows": rows,
        })
    return {
        "kind": "Independent all-candidate workpiece geometry audit during open resets",
        "method": "Saved qpos kinematics and collision queries only; zero integration or force solve",
        "scope": "All workpiece collision candidates are counted, including nonpenetrating candidates. Zero unexpected candidates means neither free workpiece has other sampled geometry contacts. It does not establish zero contacts between saved samples or zero unrelated scene contacts.",
        "expected_contacts": ["True male/female SDF thread pair", "Actual left finger pads gripping the free block"],
        "force_scope": "Original rollout all-substep hand, world-support, head-seating and drift instrumentation remains the acceptance evidence. Candidate geometry does not reconstruct contact forces.",
        "model_fingerprint": fingerprint,
        "archived_model_identity": identity,
        "audit_runtime": runtime,
        "recorded_controller_sha256": metadata["controller_sha256"],
        "recorded_observer_sha256": metadata.get("engagement_observer_source_sha256"),
        "original_trial_partial": metadata["partial"],
        "original_trial_passed": metadata.get("passed"),
        "phases": result_phases,
        "auditor_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "trajectory": str(path),
        "trajectory_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit(args.trajectory)
    output = args.output or args.trajectory.parent / "independent_reset_contact_audit.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"output": str(output), "reset_phases": len(result["phases"]),
                      "unexpected_workpiece_candidate_count": sum(p["unexpected_workpiece_candidate_count"] for p in result["phases"])}, indent=2))
