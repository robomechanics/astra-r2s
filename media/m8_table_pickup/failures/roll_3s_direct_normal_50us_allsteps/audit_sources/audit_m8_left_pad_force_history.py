"""Recompute retention diagnostics from original every-substep force records.

This checks raw diagnostic integrity, timestep coverage and aggregation. It
performs no integration, collision query, force solve or force reconstruction.
The native solved loads remain evidence from the original recorded rollout.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path

import numpy as np


def aggregate(forces, timestep, threshold, observer):
    counts = np.count_nonzero(forces <= threshold, axis=0)
    longest = []
    for column in range(2):
        flags = forces[:, column] <= threshold
        changes = np.diff(np.r_[False, flags, False].astype(np.int8))
        lengths = np.flatnonzero(changes == -1) - np.flatnonzero(changes == 1)
        longest.append(float(lengths.max(initial=0) * timestep))
    return {"observer": observer, "minimum_loaded_force_N": threshold,
        "observed_physics_steps": len(forces), "observed_duration_s": float(len(forces) * timestep),
        "minimum_normal_force_N": forces.min(axis=0).tolist() if len(forces) else None,
        "unloaded_physics_steps": counts.tolist(),
        "unloaded_duration_s": (counts * timestep).tolist(),
        "maximum_consecutive_unloaded_duration_s": longest,
        "continuously_bilateral_loaded": bool(len(forces) and not counts.any())}


def stats_match(recorded, recomputed):
    """Allow accumulated duration roundoff, never force or step-count changes."""
    if not isinstance(recorded, dict):
        return False
    for name, expected in recomputed.items():
        actual = recorded.get(name)
        if name in {"observed_duration_s", "unloaded_duration_s",
                    "maximum_consecutive_unloaded_duration_s"}:
            if actual is None or not np.allclose(actual, expected, rtol=1e-9, atol=1e-7):
                return False
        elif actual != expected:
            return False
    return True


def audit_recorded_force_history(directory, report, *, raw_bytes=None, final_time_s=None,
                                sampled_force_rows=None):
    """Verify one immutable raw history; absent historical records return None."""
    declaration = report.get("left_pad_force_history")
    if declaration is None:
        if report.get("all_substep_left_pad_loads") is not None:
            raise ValueError("Every-substep pad statistics lack their declared raw force history")
        return None
    name = declaration["filename"]
    if Path(name).name != name:
        raise ValueError("Raw force history must be a filename inside the recording directory")
    path = Path(directory) / name
    value = path.read_bytes() if raw_bytes is None else raw_bytes
    digest = hashlib.sha256(value).hexdigest()
    if digest != declaration["sha256"]:
        raise ValueError("Raw every-substep pad history does not match recorded SHA")
    with np.load(io.BytesIO(value), allow_pickle=False) as saved:
        times = saved["time"].copy()
        forces = saved["pad_normal_force_N"].copy()
        phases = saved["phase_index"].copy()
        labels = json.loads(str(saved["phase_labels_json"]))
        metadata = json.loads(str(saved["metadata_json"]))
    timestep = float(report["scene_config"]["base"]["thread"]["timestep"])
    threshold = float(metadata["minimum_loaded_force_N"])
    if (not np.isfinite(timestep) or timestep <= 0 or not np.isfinite(threshold)
            or threshold != .1 or metadata["timestep_s"] != timestep):
        raise ValueError("Every-substep pad history timestep or force threshold changed")
    if (times.ndim != 1 or forces.shape != (len(times), 2) or phases.shape != times.shape
            or not np.isfinite(times).all() or not np.isfinite(forces).all()
            or not np.issubdtype(phases.dtype, np.integer)
            or not isinstance(labels, list) or len(set(labels)) != len(labels)
            or np.any(phases < 0) or np.any(phases >= len(labels))):
        raise ValueError("Invalid or misaligned every-substep force arrays")
    if len(phases) > 1 and np.any(np.diff(phases.astype(np.int64)) < 0):
        raise ValueError("Force history phase indices are not in integration order")
    if (metadata["model_fingerprint"] != report["model_fingerprint"]
            or metadata["controller_sha256"] != report["controller_sha256"]
            or metadata["runtime"] != report["runtime"]
            or metadata["observer"] != "native-bilateral-pad-load-v1"
            or metadata["scope"] != report["left_pad_history_scope"]
            or metadata["timing"] != report["native_force_recording_note"]):
        raise ValueError("Raw pad history does not bind to the recorded scene/controller/runtime/observer")
    tolerance = max(1e-10, timestep * 1e-6)
    if len(times) > 1 and not np.all(np.abs(np.diff(times) - timestep) <= tolerance):
        raise ValueError("Raw force history does not cover consecutive physics timesteps")
    acquisition = report.get("left_acquisition")
    if len(times):
        if acquisition is None or abs(float(times[0]) - acquisition["time_s"]) > tolerance:
            raise ValueError("Force history does not begin at the declared settled acquisition step")
        if final_time_s is not None and abs(float(times[-1]) - final_time_s) > tolerance:
            raise ValueError("Force history does not reach the final saved physics step")
    elif acquisition is not None:
        raise ValueError("Recorded acquisition has no subsequent raw pad-force observation")
    recomputed = aggregate(forces, timestep, threshold, metadata["observer"])
    phase_checks = []
    for summary in report.get("phases", []):
        label = summary["phase"]
        if label not in labels:
            raise ValueError("Completed phase is absent from raw force history labels")
        selected = forces[phases == labels.index(label)]
        stats = aggregate(selected, timestep, threshold, metadata["observer"])
        phase_checks.append({"phase": label, "recomputed_stats": stats,
            "matches_recorded_phase_stats": stats_match(summary.get("all_substep_left_pad_loads"), stats)})
    gate = report.get("acceptance_checks", {}).get("all_substep_left_pad_contact_retention")
    stats_agree = stats_match(report.get("all_substep_left_pad_loads"), recomputed)
    gate_agrees = bool(isinstance(gate, dict) and stats_match(gate, recomputed)
                       and gate.get("passed") is recomputed["continuously_bilateral_loaded"])
    count_agrees = declaration["observed_physics_steps"] == len(times)
    sampled_count, sampled_agrees = 0, None
    if sampled_force_rows is not None:
        sampled_agrees = True
        for row in sampled_force_rows:
            if not row.get("left_grasp_acquired", False):
                continue
            sampled_count += 1
            index = int(np.searchsorted(times, row["time"]))
            if index == len(times) or abs(float(times[index]) - row["time"]) > tolerance:
                sampled_agrees = False
                continue
            if (labels[int(phases[index])] != row["phase"] or not np.allclose(
                    forces[index], row["left_contact"]["pad_normal_force_N"], rtol=1e-12, atol=1e-12)):
                sampled_agrees = False
    consistent = bool(stats_agree and gate_agrees and count_agrees
                      and sampled_agrees is not False
                      and all(row["matches_recorded_phase_stats"] for row in phase_checks))
    return {"kind": "Original every-substep pad-force diagnostic consistency audit",
        "method": "Raw-array identity, timestep coverage and independent aggregation only; zero integration or force solve",
        "force_scope": "Normal loads originate from the recorded native contact solve. Reaggregation does not independently establish contact-law accuracy or reconstruct forces from saved poses.",
        "timing": metadata["timing"], "scope": metadata["scope"],
        "raw_history_filename": name, "raw_history_sha256": digest,
        "raw_history_metadata": metadata, "raw_force_data_matches_recorded_sha": True,
        "consecutive_physics_step_coverage_verified": True,
        "first_recorded_force_time_s": float(times[0]) if len(times) else None,
        "last_recorded_force_time_s": float(times[-1]) if len(times) else None,
        "coverage_note": "Force evaluation is at recorded time minus timestep; saved qpos/qvel are post-integration. Coverage runs from the acquisition step through the final step; initial ungrasped phases are outside this retention scope.",
        "recomputed_stats": recomputed, "matches_recorded_global_stats": stats_agree,
        "matches_recorded_acceptance_gate": gate_agrees, "matches_declared_step_count": count_agrees,
        "sampled_diagnostic_rows_checked": sampled_count,
        "sampled_diagnostics_match_raw": sampled_agrees,
        "per_phase_stats": phase_checks, "all_consistency_checks_passed": consistent,
        "auditor_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    with np.load(args.trajectory, allow_pickle=False) as saved:
        report = json.loads(str(saved["metadata_json"]))
        final = float(saved["time"][-1])
        rows = json.loads(str(saved["info_json"]))
    audit = audit_recorded_force_history(args.trajectory.parent, report, final_time_s=final,
                                         sampled_force_rows=rows)
    if audit is None:
        raise ValueError("This historical trace has no every-substep force archive")
    audit.update(trajectory=str(args.trajectory),
                 trajectory_sha256=hashlib.sha256(args.trajectory.read_bytes()).hexdigest())
    output = args.output or args.trajectory.parent / "independent_left_pad_force_history_audit.json"
    output.write_text(json.dumps(audit, indent=2) + "\n")
    print(json.dumps({"output": str(output), "all_consistency_checks_passed": audit["all_consistency_checks_passed"],
                      "recomputed_stats": audit["recomputed_stats"]}, indent=2))


if __name__ == "__main__":
    main()
