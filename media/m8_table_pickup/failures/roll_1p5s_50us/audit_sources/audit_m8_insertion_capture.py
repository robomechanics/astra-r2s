"""Audit a versioned capture observer alongside the frozen insertion audit.

Saved rolling-window forces are original rollout diagnostics. This audit does
not reconstruct between-sample impulses or establish axial load balance.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np

ROOT = next(parent for parent in Path(__file__).resolve().parents
            if (parent / "thread_lab" / "runtime.py").is_file())
sys.path.insert(0, str(ROOT / "scripts"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import audit_m8_insertion_trace as geometry_audit


def load_archived_observer(path):
    """Bind controller inspection to the observer actually archived at start."""
    spec = importlib.util.spec_from_file_location("yam_twin.m8_insertion_engagement", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def capture_audit(path):
    with np.load(path) as saved:
        records = json.loads(str(saved["info_json"]))
        metadata = json.loads(str(saved["metadata_json"]))
    archive = path.parent / "engagement_observer_source.py"
    if not archive.exists():
        raise ValueError("Versioned capture audit requires the observer archive from run start")
    archived_module = load_archived_observer(archive)
    archived_sha = hashlib.sha256(archive.read_bytes()).hexdigest()
    if archived_sha != metadata.get("engagement_observer_source_sha256"):
        raise ValueError("Archived observer does not match recorded whole-module SHA")
    if archived_module.LoadedFlankWindow.version != metadata.get("engagement_observer"):
        raise ValueError("Archived observer version does not match recorded version")
    result = geometry_audit.robot_audit(path)
    limits = metadata["engagement_window"]
    duration = limits["duration_s"]
    impulse = limits["minimum_loaded_normal_impulse_Ns"]
    loaded_duration = limits["minimum_loaded_duration_s"]
    phase_range = limits["maximum_helix_phase_range_m"]
    inconsistent = []
    windows = []
    sampled_first_ready = sampled_first_capture = None
    previous_capture = False
    for record in records:
        window = record["engagement_window"]
        expected_ready = bool(
            window["continuous_geometry_elapsed_s"] + 1e-9 >= duration
            and window["normal_impulse_Ns"] >= impulse
            and window["loaded_duration_s"] + 1e-12 >= loaded_duration
            and window["helix_phase_range_m"] <= phase_range)
        if bool(window["ready"]) != expected_ready:
            inconsistent.append({"time_s": record["time"], "issue": "Ready flag disagrees with recorded physical thresholds"})
        if window["version"] != metadata["engagement_observer"]:
            inconsistent.append({"time_s": record["time"], "issue": "Observer version changed within trial"})
        if previous_capture and not record["thread_engaged"]:
            inconsistent.append({"time_s": record["time"], "issue": "Capture latch unexpectedly cleared"})
        previous_capture = bool(record["thread_engaged"])
        if window["ready"] and sampled_first_ready is None:
            sampled_first_ready = float(record["time"])
        if record["thread_engaged"] and sampled_first_capture is None:
            sampled_first_capture = float(record["time"])
        windows.append({"time_s": record["time"], "phase": record["phase"],
                        "recorded_candidate_capture": bool(record["thread_engaged"]),
                        "recorded_window": window,
                        "recorded_loaded_interior_normal_force_N": record["loaded_formed_thread_normal_force_N"],
                        "legacy_consecutive_loaded_contact_streak_s": record["legacy_consecutive_loaded_contact_streak_s"]})
    checks = metadata.get("acceptance_checks", {})
    proof_names = ("completed_qualifying_turns", "observed_metric_lead",
                   "closed_turn_tracking", "contact_torque_turns_bolt",
                   "open_reset_contact_decoupling", "passive_self_locking_during_open_reset")
    final_proof = bool(all(name in checks and checks[name]["passed"] for name in proof_names))
    result["capture_observer_audit"] = {
        "version": metadata["engagement_observer"],
        "archived_source_path": str(archive),
        "archived_source_sha256": archived_sha,
        "archived_source_matches_recorded": True,
        "archived_version_matches_recorded": True,
        "scope": "The window declares candidate full-flank capture. Completed engagement additionally requires coupled metric lead, actual contact torque, and unsupported unseated open resets.",
        "force_scope": "Resolved interior-contact normal impulse is not axial impulse or axial load balance. Rolling diagnostics originate from every-substep instrumentation; sampled qpos cannot independently reconstruct them.",
        "limits": limits,
        "sampled_ready_flags_match_recorded_limits": not inconsistent,
        "sampled_flag_inconsistencies": inconsistent,
        "first_sampled_ready_time_s": sampled_first_ready,
        "first_sampled_candidate_capture_time_s": sampled_first_capture,
        "recorded_final_engagement_proof_checks_present_and_passed": final_proof,
        "recorded_final_proof_checks": {name: checks.get(name) for name in proof_names},
        "original_trial_overall_passed": metadata.get("passed"),
        "legacy_force_streak_outcome": metadata.get("legacy_continuous_loaded_force_criterion"),
        "recorded_window_rows": windows,
    }
    result.update(
        capture_auditor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        geometry_auditor_sha256=hashlib.sha256(Path(geometry_audit.__file__).read_bytes()).hexdigest(),
        trajectory=str(path),
        trajectory_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        force_scope_note="Original loaded-contact and between-sample force/support checks remain the rollout's evidence; this audit performs zero integration or force solve")
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = capture_audit(args.trajectory)
    output = args.output or args.trajectory.parent / "independent_capture_audit.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps({"output": str(output),
                      "controller_source_matches_recorded": result["controller_source_matches_recorded"],
                      "observer_source_matches_recorded": result["capture_observer_audit"]["archived_source_matches_recorded"],
                      "sampled_candidate_capture_time_s": result["capture_observer_audit"]["first_sampled_candidate_capture_time_s"]}, indent=2))
