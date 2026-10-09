"""Compare recorded clamp/lift diagnostics at two timesteps, excluding all turns."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

import numpy as np

PHASES = ("close", "settle", "lift", "hold_lift")


def recorded_case(path):
    with np.load(path) as saved:
        metadata = json.loads(str(saved["metadata_json"]))
        rows = json.loads(str(saved["info_json"]))
    rows = [row for row in rows if row["phase"] in PHASES]
    if tuple(dict.fromkeys(row["phase"] for row in rows)) != PHASES:
        raise ValueError(f"{path} does not contain all four clamp/lift phases")
    normals = np.asarray([row["left_contact"]["pad_normal_force_N"]
                          for row in rows if row["phase"] != "close"])
    return metadata, {
        "trace": str(path), "trace_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "model_fingerprint": metadata["model_fingerprint"],
        "timestep_s": metadata["thread_config"]["timestep"],
        "end_time_s": rows[-1]["time"],
        "end_block_world_position_m": rows[-1]["block_world_position"],
        "end_block_lift_m": rows[-1]["block_lift_m"],
        "sampled_peak_initial_relative_block_slip_m": max(row["block_grip_slip_m"] for row in rows),
        "sampled_peak_initial_relative_block_rotation_rad": max(row["block_grip_rotation_slip_rad"] for row in rows),
        "sampled_peak_nut_radial_m": max(row["radial_offset_m"] for row in rows),
        "sampled_peak_nut_tilt_rad": max(row["nut_tilt_rad"] for row in rows),
        "sampled_peak_reported_sdf_depth_proxy_m": max(row["reported_sdf_depth_m"] for row in rows),
        "sampled_minimum_loaded_left_pad_normals_N": normals.min(axis=0).tolist(),
        "sampled_support_contacts": max(row["world_support_contacts"] for row in rows),
        "sampled_object_drive_flags_all_zero": all(row["nut_external_drive_zero"]
            and row["block_external_drive_zero"] for row in rows),
    }


def compare(first, second):
    metadata, cases, configurations = [], {}, []
    for label, path in (("first", first), ("second", second)):
        record, result = recorded_case(path)
        metadata.append(record)
        cases[label] = result
        configuration = copy.deepcopy(record["scene_config"])
        configuration["thread"].pop("timestep")
        configurations.append(configuration)
    same_scene = configurations[0] == configurations[1]
    same_controller = (metadata[0]["control_config"] == metadata[1]["control_config"]
                       and metadata[0]["controller_sha256"] == metadata[1]["controller_sha256"])
    # Runtime paths are machine-specific; compare the content identities.
    runtime_id = lambda value: (value["mujoco_version"], value["thread_plugin_source_sha256"],
        sorted(library["sha256"] for library in value["libraries"]))
    same_runtime = runtime_id(metadata[0]["runtime"]) == runtime_id(metadata[1]["runtime"])
    if not (same_scene and same_controller and same_runtime):
        raise ValueError("A timestep-only comparison requires matching scene, controller and runtime identities")
    a, b = cases.values()
    return {
        "scope": "Close, settle, lift and hold_lift only. No turn-lead or thread-load timestep-convergence qualification.",
        "method": "Compare original saved diagnostics sampled approximately every 5 ms; no dynamics or independent force reconstruction",
        "comparer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "configuration_differs_only_by_timestep": same_scene,
        "controller_configuration_and_source_match": same_controller,
        "core_and_plugin_identities_match": same_runtime,
        "cases": cases,
        "hold_lift_endpoint_world_position_difference_m": float(np.linalg.norm(
            np.asarray(a["end_block_world_position_m"]) - b["end_block_world_position_m"])),
        "sampled_peak_nut_radial_difference_m": abs(a["sampled_peak_nut_radial_m"] - b["sampled_peak_nut_radial_m"]),
        "sampled_peak_nut_tilt_difference_rad": abs(a["sampled_peak_nut_tilt_rad"] - b["sampled_peak_nut_tilt_rad"]),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("first", type=Path)
    parser.add_argument("second", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = compare(args.first, args.second)
    output = args.output or args.first.parent / "hold_lift_timestep_comparison.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"output": str(output), "endpoint_difference_m":
                      result["hold_lift_endpoint_world_position_difference_m"]}, indent=2))
