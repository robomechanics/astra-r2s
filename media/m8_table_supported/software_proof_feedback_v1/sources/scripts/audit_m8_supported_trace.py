"""Independent saved-geometry and original-load audit of supported M8 assembly.

This auditor never integrates dynamics or reconstructs solved contact forces
from saved poses. Native loads come only from the immutable original history;
saved qpos are used for separate kinematics/collision/lead measurements.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = next(parent for parent in Path(__file__).resolve().parents
            if (parent / "thread_lab" / "runtime.py").is_file())
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "scripts"))
from audit_m8_insertion_trace import (recorded_model, inspected_source_identity,
                                     full_ring_interval, lead_fit)
from audit_m8_free_joint_properties import audit as passive_audit
from thread_lab.model import ThreadConfig
from thread_lab.runtime import require_micron_engine


def sha256(value):
    return hashlib.sha256(value).hexdigest()


def strict_original_checks(metadata):
    """Export an early pilot's declared unobserved minima without changing bytes.

    Only the historical positive-infinity transport minimum is an allowed
    absent observation, when transport never executed and its gate is false.
    Nonfinite native load/pose arrays are always rejected elsewhere.
    """
    absent = []
    checks = metadata["acceptance_checks"]
    no_transport = not any(item["phase"] == "transport_bolt" for item in metadata["phases"])
    def walk(value, path):
        if isinstance(value, dict):
            return {key: walk(item, path+[key]) for key, item in value.items()}
        if isinstance(value, list):
            return [walk(item, path+[str(index)]) for index, item in enumerate(value)]
        if isinstance(value, float) and not np.isfinite(value):
            allowed = (no_transport and path[:2] == ["picked_up_free_bolt", "sampled_minimum_closed_transport_pad_normals_N"]
                and value == float("inf") and not checks["picked_up_free_bolt"]["passed"])
            if not allowed:
                raise ValueError("Nonfinite original physical report value: "+"/".join(path))
            absent.append("acceptance_checks/"+"/".join(path))
            return None
        return value
    return walk(checks, []), absent


def contiguous_gaps(flags, dt):
    """Return all gap durations, including a gap at either recording edge."""
    flags = np.asarray(flags, dtype=bool)
    changes = np.diff(np.r_[False, flags, False].astype(np.int8))
    return ((np.flatnonzero(changes == -1)-np.flatnonzero(changes == 1))*dt).tolist()


def rotation_changes(rotations, initial_rotation):
    """Measure physical motion relative to the archived initial yaw, not world I."""
    return Rotation.from_matrix(np.asarray(rotations)@np.asarray(initial_rotation).T).magnitude()


def load_statistics(table_upward_N, left_upward_N, pads_N, dt, weight_N):
    """Independent finite-volume summaries of original every-step loads."""
    table = np.asarray(table_upward_N, dtype=float)
    hand = np.asarray(left_upward_N, dtype=float)
    pads = np.asarray(pads_N, dtype=float)
    if (table.ndim != 1 or hand.shape != table.shape or pads.shape != (len(table), 2)
            or not np.isfinite(table).all() or not np.isfinite(hand).all()
            or not np.isfinite(pads).all() or np.any(pads < 0)
            or not np.isfinite((dt, weight_N)).all() or min(dt, weight_N) <= 0):
        raise ValueError("Invalid original load arrays or positive physical units")
    if not len(table):
        return {"observed_steps": 0, "duration_s": 0., "table_loaded_duty": None,
                "mean_table_weight_fraction": None,
                "mean_positive_hand_upward_weight_fraction": None,
                "continuous_table_load": False, "continuous_bilateral_pad_load": False}
    loaded = table > .1*weight_N
    gaps = contiguous_gaps(~loaded, dt)
    pad_gaps = [contiguous_gaps(pads[:, i] <= .1, dt) for i in range(2)]
    return {"observed_steps": len(table), "duration_s": float(len(table)*dt),
        "loaded_table_threshold_N": float(.1*weight_N),
        "minimum_table_upward_force_N": float(table.min()),
        "maximum_table_upward_force_N": float(table.max()),
        "mean_table_upward_force_N": float(table.mean()),
        "mean_table_weight_fraction": float(table.mean()/weight_N),
        "table_upward_impulse_Ns": float(table.sum()*dt),
        "mean_positive_hand_upward_force_N": float(np.maximum(hand, 0).mean()),
        "mean_positive_hand_upward_weight_fraction": float(np.maximum(hand, 0).mean()/weight_N),
        "positive_hand_upward_impulse_Ns": float(np.maximum(hand, 0).sum()*dt),
        "table_loaded_duty": float(loaded.mean()),
        "table_unloaded_steps": int(np.count_nonzero(~loaded)),
        "table_unloaded_duration_s": float(np.count_nonzero(~loaded)*dt),
        "maximum_consecutive_table_unloaded_duration_s": max(gaps, default=0.),
        "continuous_table_load": bool(loaded.all()),
        "minimum_pad_normal_force_N": pads.min(axis=0).tolist(),
        "pad_unloaded_steps": np.count_nonzero(pads <= .1, axis=0).tolist(),
        "maximum_consecutive_pad_unloaded_duration_s": [max(g, default=0.) for g in pad_gaps],
        "continuous_bilateral_pad_load": bool(np.all(pads > .1))}


def settled_baseline_passes(stats, *, duration_s=.100, minimum_table_fraction=.90,
                            maximum_hand_fraction=.10, minimum_duty=.99):
    """Table bearing and side stabilization cannot be inferred from candidates."""
    return bool(stats["observed_steps"] and stats["duration_s"]+1e-12 >= duration_s
        and stats["mean_table_weight_fraction"] >= minimum_table_fraction
        and stats["mean_positive_hand_upward_weight_fraction"] <= maximum_hand_fraction
        and stats["table_loaded_duty"] >= minimum_duty)


def rolling_load_shares(table_N, hand_N, active, dt, weight_N, *, duration_s=.100,
                       minimum_table_fraction=.90, maximum_hand_fraction=.10,
                       minimum_duty=.99):
    """Measure weight bearing through every active trailing native window."""
    table, hand, active = np.asarray(table_N), np.asarray(hand_N), np.asarray(active, dtype=bool)
    if table.ndim != 1 or hand.shape != table.shape or active.shape != table.shape:
        raise ValueError("Rolling load-share arrays must align")
    length = int(np.ceil(duration_s/dt-1e-9))
    ends = np.flatnonzero(active)+1
    if not len(ends) or np.any(ends < length):
        return {"complete_active_rolling_windows": False, "passed": False}
    means = []
    for values in (table, np.maximum(hand, 0), (table > .1*weight_N).astype(float)):
        sums = np.r_[0., np.cumsum(values)]
        means.append((sums[ends]-sums[ends-length])/length)
    table_fraction, hand_fraction, duty = means[0]/weight_N, means[1]/weight_N, means[2]
    # Match the original TableLoadWindow transition: a settled mean/duty
    # window still fails when its actual final native timestep is unloaded.
    # Preserve that failure separately from continuous all-substep retention.
    final_tick_loaded = table[ends-1] > .1*weight_N
    failed = ((table_fraction < minimum_table_fraction)
        | (hand_fraction > maximum_hand_fraction) | (duty < minimum_duty)
        | ~final_tick_loaded)
    return {"complete_active_rolling_windows": True, "window_substeps": length,
        "window_duration_s": float(length*dt), "checked_active_windows": len(ends),
        "minimum_mean_table_weight_fraction": float(table_fraction.min()),
        "maximum_mean_positive_hand_upward_weight_fraction": float(hand_fraction.max()),
        "minimum_loaded_table_duty": float(duty.min()),
        "unloaded_final_tick_windows": int(np.count_nonzero(~final_tick_loaded)),
        "failed_windows": int(np.count_nonzero(failed)),
        "passed": bool(not np.any(failed))}


def transform_contact_wrench(frame, local_force, position, origin, sign):
    """MuJoCo contact axes are rows; force acts on geom2, opposite on geom1."""
    frame = np.asarray(frame, dtype=float).reshape(3, 3)
    local_force = np.asarray(local_force, dtype=float)
    position, origin = np.asarray(position, dtype=float), np.asarray(origin, dtype=float)
    if (local_force.shape != (6,) or position.shape != (3,) or origin.shape != (3,)
            or not np.isfinite(frame).all() or not np.isfinite(local_force).all()
            or not np.isfinite(position).all() or not np.isfinite(origin).all()
            or sign not in (-1., 1.) or not np.allclose(frame@frame.T, np.eye(3), atol=1e-9)):
        raise ValueError("Invalid original contact frame, local force or action-reaction sign")
    force = sign*(frame.T@local_force[:3])
    return np.r_[force, sign*(frame.T@local_force[3:])+np.cross(position-origin, force)]


def recorded_contact_wrench(records, model, body_id):
    """Re-transform original sampled contact records; do not query a new solve."""
    result = np.zeros(6)
    root = int(model.body_weldid[body_id])
    for record in records:
        geom1, geom2 = (model.geom(record[key]).id for key in ("geom1", "geom2"))
        roots = [int(model.body_weldid[int(model.geom_bodyid[g])]) for g in (geom1, geom2)]
        if roots[0] == root and roots[1] != root:
            sign = -1.
        elif roots[1] == root and roots[0] != root:
            sign = 1.
        else:
            raise ValueError("Original contact does not have exactly one block counterpart")
        wrench = transform_contact_wrench(record.get("contact_frame", record.get("frame")),
            record["local_contact_force_N_Nm"], record["contact_position_world_m"],
            record["block_origin_world_m"], sign)
        declared = record.get("force_on_block_world_N",
                              record.get("signed_force_contribution_on_block_world_N"))
        if declared is not None and not np.allclose(
                wrench[:3], declared, rtol=1e-12, atol=1e-12):
            raise ValueError("Recorded block force has an incorrect contact-frame transform or sign")
        result += wrench
    return result


def original_load_audit(path, metadata, model, records, final_time_s):
    """Verify immutable every-step ledger and reaggregate its actual native loads."""
    declaration = metadata["table_support_force_history"]
    filename = declaration["filename"]
    if Path(filename).name != filename:
        raise ValueError("Original load ledger must be a sibling filename")
    raw = (path.parent/filename).read_bytes()
    if sha256(raw) != declaration["sha256"]:
        raise ValueError("Original support ledger bytes do not match recorded SHA")
    with np.load(io.BytesIO(raw), allow_pickle=False) as saved:
        columns = {name: saved[name].copy() for name in saved.files
                   if name not in {"phase_labels_json", "metadata_json"}}
        labels = json.loads(str(saved["phase_labels_json"]))
        identity = json.loads(str(saved["metadata_json"]))
    times, indices = columns["time"], columns["phase_index"]
    count = len(times)
    if not count or times.ndim != 1 or not np.isfinite(times).all():
        raise ValueError("No consecutive original physics-step support observations")
    dt = float(model.opt.timestep)
    tolerance = max(1e-10, dt*1e-6)
    if (not np.isfinite(dt) or dt <= 0 or abs(times[0]-dt) > tolerance
            or abs(times[-1]-final_time_s) > tolerance
            or np.any(np.abs(np.diff(times)-dt) > tolerance)
            or declaration["observed_physics_steps"] != count):
        raise ValueError("Original support history does not cover initial through final consecutive substeps")
    if (indices.shape != times.shape or not np.issubdtype(indices.dtype, np.integer)
            or np.any(indices < 0) or np.any(indices >= len(labels))
            or np.any(np.diff(indices.astype(np.int64)) < 0)
            or len(labels) != len(set(labels))):
        raise ValueError("Original phase indices are invalid or not in integration order")
    for key in ("model_fingerprint", "controller_sha256", "runtime"):
        if identity[key] != metadata[key]:
            raise ValueError(f"Original force ledger has mismatched {key}")
    if (identity["timestep_s"] != dt
            or identity["force_timing"] != metadata["native_force_recording_note"]
            or identity["observer"] != "native-supported-block-load-window-v1"):
        raise ValueError("Original load ledger timing or observer changed")
    shapes = {"table_wrench_world_at_block_origin_N_Nm": (count, 6),
        "left_hand_wrench_world_at_block_origin_N_Nm": (count, 6),
        "pad_normal_force_N": (count, 2), "block_position_m": (count, 3),
        "block_rotation_matrix": (count, 3, 3)}
    for name, values in columns.items():
        if values.shape != shapes.get(name, (count,)) or not np.isfinite(values).all():
            raise ValueError(f"Invalid original load ledger column {name}")
    for name in ("external_drive_zero", "supported_task_active"):
        if not np.isin(columns[name], [False, True]).all():
            raise ValueError(f"Original {name} flags must be boolean")
    for name in ("unexpected_world_support_count", "bolt_world_support_count",
                 "table_contact_candidates", "table_loaded_contacts",
                 "right_hand_bolt_contact_count", "head_block_seating_contact_count",
                 "loaded_formed_thread_contact_count", "unexpected_native_contact_count"):
        if np.any(columns[name] < 0) or not np.equal(columns[name], np.floor(columns[name])).all():
            raise ValueError(f"Original {name} contact counts are invalid")
    rotations = columns["block_rotation_matrix"]
    if not np.allclose(rotations@np.transpose(rotations, (0, 2, 1)), np.eye(3), atol=1e-9):
        raise ValueError("Original block rotations are not orthogonal")
    block = model.body("fixture_block").id
    weight = float(np.sum(model.body_mass[model.body_weldid == block])*abs(model.opt.gravity[2]))
    if not np.isclose(weight, metadata["known_block_weight_N"], rtol=1e-12, atol=1e-12):
        raise ValueError("Original support threshold does not use compiled block gravity")
    table = columns["table_wrench_world_at_block_origin_N_Nm"][:, 2]
    hand = columns["left_hand_wrench_world_at_block_origin_N_Nm"][:, 2]
    pads = columns["pad_normal_force_N"]
    active = columns["supported_task_active"].astype(bool)
    acquisition = metadata.get("left_acquisition")
    if acquisition is None:
        if active.any():
            raise ValueError("Supported-active samples lack declared stabilization acquisition")
        baseline_stats, baseline_ok = None, False
    else:
        end = int(np.searchsorted(times, acquisition["time_s"]))
        if end == count or abs(times[end]-acquisition["time_s"]) > tolerance:
            raise ValueError("Stabilization acquisition does not correspond to a recorded native substep")
        if not np.array_equal(active, np.arange(count) >= end):
            raise ValueError("Supported history must start exactly at acquisition and remain active")
        control = metadata["control_config"]
        required = float(control["settled_table_window_s"])
        declared_window = acquisition["table_load_window"]
        # The controller's accumulated floating window can retain one extra
        # timestep. Bound that declared extent before independently measuring
        # its actual immutable loads, rather than importing the observer.
        duration = float(declared_window["observed_window_s"])
        length = int(round(duration/dt))
        if (length*dt < required-1e-10 or length*dt > required+dt+1e-10
                or abs(length*dt-duration) > 1e-9):
            raise ValueError("Declared settled baseline has an invalid native window extent")
        start = end+1-length
        if start < 0 or any(labels[int(i)] != "settle_left_block" for i in indices[start:end+1]):
            raise ValueError("Pre-bolt table baseline is not the declared complete settled window")
        baseline_stats = load_statistics(table[start:end+1], hand[start:end+1],
                                        pads[start:end+1], dt, weight)
        baseline_ok = settled_baseline_passes(baseline_stats, duration_s=required,
            minimum_table_fraction=control["minimum_table_weight_fraction"],
            maximum_hand_fraction=control["maximum_hand_upward_weight_fraction"],
            minimum_duty=control["minimum_loaded_table_duty"])
        baseline_ok = bool(baseline_ok and np.all(pads[start:end+1] > .1)
            and not np.any(columns["unexpected_world_support_count"][start:end+1]))
        for ours, theirs in (("mean_table_upward_force_N", "mean_table_upward_force_N"),
                ("mean_positive_hand_upward_force_N", "mean_positive_hand_upward_force_N"),
                ("table_loaded_duty", "loaded_table_substep_duty")):
            if not np.isclose(baseline_stats[ours], declared_window[theirs], rtol=1e-8, atol=1e-9):
                raise ValueError("Original baseline observer is inconsistent with native force ledger")
        if bool(declared_window["ready"]) != baseline_ok:
            raise ValueError("Original table-bearing transition disagrees with recomputed baseline")
    active_stats = load_statistics(table[active], hand[active], pads[active], dt, weight)
    control = metadata["control_config"]
    whole_task_gate = metadata["acceptance_checks"]["table_bears_weight_throughout_active_task"]
    rolling_extent = float(whole_task_gate["final_window"]["observed_window_s"])
    if (active.any() and not (control["settled_table_window_s"]-1e-10
            <= rolling_extent <= control["settled_table_window_s"]+dt+1e-10)):
        raise ValueError("Whole-task trailing load window has an invalid native extent")
    rolling = rolling_load_shares(table, hand, active, dt, weight,
        duration_s=rolling_extent if active.any() else control["settled_table_window_s"],
        minimum_table_fraction=control["minimum_table_weight_fraction"],
        maximum_hand_fraction=control["maximum_hand_upward_weight_fraction"],
        minimum_duty=control["minimum_loaded_table_duty"])
    if active.any():
        for ours, theirs in (("minimum_mean_table_weight_fraction", "minimum_mean_table_weight_fraction"),
                ("maximum_mean_positive_hand_upward_weight_fraction", "maximum_mean_positive_hand_upward_weight_fraction"),
                ("minimum_loaded_table_duty", "minimum_loaded_table_duty")):
            if not np.isclose(rolling[ours], whole_task_gate[theirs], rtol=1e-6, atol=1e-8):
                raise ValueError("Whole-task original load shares differ from independent raw aggregation")
        if (rolling["failed_windows"] != whole_task_gate["failed_active_trailing_windows"]
                or bool(whole_task_gate["passed"]) != rolling["passed"]):
            raise ValueError("Whole-task original load-share gate differs from independent raw aggregation")
    source_gate = metadata["acceptance_checks"]["all_substep_table_load_retention"]
    if (source_gate["observed_physics_steps"] != active_stats["observed_steps"]
            or source_gate["unloaded_physics_steps"] != active_stats.get("table_unloaded_steps", 0)
            or bool(source_gate["passed"]) != active_stats["continuous_table_load"]):
        raise ValueError("Strict original table-load gate differs from independent ledger aggregation")
    checked = 0
    for record in records:
        index = int(np.searchsorted(times, record["time"]))
        if index == count or abs(times[index]-record["time"]) > tolerance:
            raise ValueError("Saved diagnostic time is missing from original support ledger")
        if labels[int(indices[index])] != record["phase"]:
            raise ValueError("Saved native force phase differs from original support ledger")
        declared_table = np.asarray(record["table_support"]["table_wrench_on_block_world"])
        declared_left = np.asarray(record["left_contact"]["wrench_world"])
        if not np.allclose(declared_table, columns["table_wrench_world_at_block_origin_N_Nm"][index], rtol=1e-12, atol=1e-12):
            raise ValueError("Sampled table wrench differs from original every-step ledger")
        if not np.allclose(declared_left, columns["left_hand_wrench_world_at_block_origin_N_Nm"][index], rtol=1e-12, atol=1e-12):
            raise ValueError("Sampled left wrench differs from original every-step ledger")
        transformed_table = recorded_contact_wrench(record["table_support"]["contacts"], model, block)
        transformed_left = recorded_contact_wrench(record["left_contact"]["native_contact_records"], model, block)
        if not np.allclose(transformed_table, declared_table, rtol=1e-10, atol=1e-10):
            raise ValueError("Original table contact frame/action reaction does not reconstruct declared wrench")
        if not np.allclose(transformed_left, declared_left, rtol=1e-10, atol=1e-10):
            raise ValueError("Original hand contact frame/action reaction does not reconstruct declared wrench")
        original_table_contacts = record["table_support"]["contacts"]
        normal_upward = 0.
        for contact in original_table_contacts:
            first = model.geom(contact["geom1"]).id
            sign = -1. if int(model.body_weldid[int(model.geom_bodyid[first])]) == block else 1.
            frame = np.asarray(contact.get("contact_frame", contact.get("frame")))
            normal_upward += sign*frame[0, 2]*contact["local_contact_force_N_Nm"][0]
        if not np.isclose(normal_upward, columns["table_upward_normal_force_N"][index], rtol=1e-12, atol=1e-12):
            raise ValueError("Original normal-only table force disagrees with contact-frame normal projection")
        if (len(original_table_contacts) != columns["table_contact_candidates"][index]
                or sum(contact["local_contact_force_N_Nm"][0] > .005 for contact in original_table_contacts)
                != columns["table_loaded_contacts"][index]):
            raise ValueError("Original table candidate/loaded counts disagree with original contact records")
        checked += 1
    center = np.asarray(metadata["scene_config"]["block_position"])
    position = columns["block_position_m"]
    translation = np.linalg.norm(position-center, axis=1)
    block_joint = model.joint("fixture_block_free").id
    qadr = int(model.jnt_qposadr[block_joint])
    initial_rotation = np.empty(9)
    mujoco.mju_quat2Mat(initial_rotation, model.qpos0[qadr+3:qadr+7])
    rotation = rotation_changes(rotations, initial_rotation.reshape(3, 3))
    lift = position[:, 2]-center[2]
    post_pickup = np.array([labels[int(i)] not in {"settle_table", "reach_left_block", "close_left_block",
        "settle_left_block", "secure_left", "reach_bolt", "close_bolt", "settle_bolt", "lift_bolt"}
        for i in indices])
    return {"method": "Immutable original arrays; independent aggregation and original sampled contact-frame transform only; no force reconstruction or dynamics",
        "timing": identity["force_timing"], "raw_sha256": sha256(raw),
        "identity_and_consecutive_full_step_coverage_verified": True,
        "original_sampled_contact_frame_checks": checked,
        "block_weight_N": weight, "settled_pre_bolt_baseline": baseline_stats,
        "settled_pre_bolt_baseline_passed": baseline_ok,
        "after_stabilization_load_statistics": active_stats,
        "whole_task_active_rolling_weight_bearing": rolling,
        "every_original_substep_external_drive_zero": bool(np.all(columns["external_drive_zero"])),
        "all_step_maximum_block_translation_m": float(translation.max()),
        "all_step_maximum_block_rotation_rad": float(rotation.max()),
        "all_step_maximum_block_lift_m": float(lift.max()),
        "all_step_maximum_unexpected_world_support_count": int(columns["unexpected_world_support_count"].max()),
        "all_step_maximum_unexpected_native_contact_count": int(columns["unexpected_native_contact_count"].max()),
        "all_step_maximum_unexpected_native_contact_depth_m": float(columns["unexpected_native_contact_peak_depth_m"].max()),
        "all_step_minimum_tip_table_clearance_m": float(columns["bolt_tip_table_clearance_m"].min()),
        "all_step_post_pickup_bolt_world_contact_count": int(columns["bolt_world_support_count"][post_pickup].max(initial=0)),
        "source_observer_agrees_with_independent_aggregation": True}, columns, labels


def all_step_bolt_measurements(columns, labels, metadata):
    """Recompute actual qualified lead and passive reset motion from raw rows."""
    indices = columns["phase_index"].astype(int)
    z, yaw = columns["bolt_base_insertion_m"], columns["bolt_yaw_unwrapped_rad"]
    pitch = float(metadata["scene_config"]["base"]["thread"]["pitch"])
    expected_angle = float(metadata["control_config"]["arm"]["stroke_angle_rad"])
    summaries = {item["phase"]: item for item in metadata["phases"]}
    phases = []
    for index, label in enumerate(labels):
        selected = np.flatnonzero(indices == index)
        if not len(selected) or not label.startswith(("turn_", "reset_open_")):
            continue
        before, last = max(0, int(selected[0])-1), int(selected[-1])
        advance, angle = float(z[last]-z[before]), float(yaw[last]-yaw[before])
        values = {"phase": label, "original_substeps": len(selected),
            "advance_m": advance, "rotation_rad": angle,
            "lead_residual_m": float(advance-pitch*angle/(2*np.pi)),
            "minimum_formed_flank_overlap_m": float(columns["formed_flank_overlap_m"][selected].min()),
            "maximum_bolt_world_contacts": int(columns["bolt_world_support_count"][selected].max()),
            "maximum_right_hand_bolt_contacts": int(columns["right_hand_bolt_contact_count"][selected].max()),
            "maximum_head_block_seating_contacts": int(columns["head_block_seating_contact_count"][selected].max()),
            "maximum_axial_drift_m": float(np.max(np.abs(z[selected]-z[before]))),
            "maximum_yaw_drift_rad": float(np.max(np.abs(yaw[selected]-yaw[before]))),
            "started_with_recorded_capture": bool(summaries[label]["started_engaged"])}
        if label.startswith("turn_"):
            mask = np.zeros(len(z), dtype=bool)
            mask[selected] = True
            values["independent_lead_fit"] = lead_fit(z, yaw, mask, pitch)
            values["qualified_stroke_passed"] = bool(
                values["started_with_recorded_capture"]
                and values["minimum_formed_flank_overlap_m"] > pitch
                and abs(angle-expected_angle) < .03
                and abs(values["lead_residual_m"]) < .02*pitch*expected_angle/(2*np.pi)
                and values["independent_lead_fit"]["passed"]
                and not values["maximum_bolt_world_contacts"])
        else:
            values["open_hand_contact_decoupling_passed"] = not values["maximum_right_hand_bolt_contacts"]
            values["captured_passive_reset_passed"] = bool(
                values["started_with_recorded_capture"]
                and values["minimum_formed_flank_overlap_m"] > pitch
                and values["maximum_axial_drift_m"] < 10e-6
                and values["maximum_yaw_drift_rad"] < .02
                and not values["maximum_bolt_world_contacts"]
                and not values["maximum_right_hand_bolt_contacts"]
                and not values["maximum_head_block_seating_contacts"])
        phases.append(values)
    return phases


def feedback_weight_windows(columns, dt, weight, *, fully_open=False, duration_s=.100):
    """Original observer's timestamp-bounded windows, without force solving.

    Recompute sums from immutable arrays. The source removes times at or
    before t-duration, which can retain2000 or2001 native ticks because of
    floating timestamps. Contiguous geometry validity and endpoint load
    remain separate from the mean. An open-hand angular velocity is allowed;
    the fully-open quiet predicate applies to the actual bolt.
    """
    time = np.asarray(columns["time"], dtype=float)
    if not len(time) or not np.isfinite(time).all() or dt <= 0 or weight <= 0 or duration_s <= 0:
        raise ValueError("Feedback windows need positive finite time, step, weight and duration")
    valid_name = "open_observation_valid" if fully_open else "weight_observation_valid"
    original_valid = np.asarray(columns[valid_name])
    if original_valid.shape != time.shape or not np.isin(original_valid, [False, True]).all():
        raise ValueError("Original feedback observer validity flags must be aligned booleans")
    valid = original_valid.astype(bool)
    good = (valid & (columns["external_drive_zero"] == 1)
        & (columns["bolt_world_support_contact_count"] == 0)
        & (columns["nonthread_block_bolt_contact_count"] == 0)
        & (np.abs(columns["relative_bolt_axial_velocity_m_per_s"]) <= .0002)
        & (columns["radial_offset_m"] <= .000150)
        & (columns["bolt_tilt_rad"] <= np.deg2rad(2)))
    if fully_open:
        good &= ((columns["fully_open_unassisted"] == 1)
            & (columns["right_robot_bolt_contact_count"] == 0)
            & (columns["relative_bolt_angular_speed_rad_per_s"] <= .01))
    index = np.arange(len(time))
    previous_bad = np.maximum.accumulate(np.where(good, -1, index))
    left = np.maximum(np.searchsorted(time, time-duration_s, side="right"), previous_bad+1)
    count = index-left+1
    extent = count*dt
    def sums(values):
        prefix = np.r_[0., np.cumsum(values, dtype=float)]
        return prefix[index+1]-prefix[left]
    denominator = np.maximum(count, 1)*weight
    thread = sums(columns["thread_gravity_opposing_force_N"])/denominator
    hand = sums(np.maximum(columns["hand_gravity_opposing_force_N"], 0.))/denominator
    ready = (good & ((index-previous_bad)*dt+1e-9 >= duration_s)
        & (extent+1e-9 >= duration_s) & (thread >= .9) & (hand <= .1)
        & (columns["thread_gravity_opposing_force_N"] > .1*weight))
    return {"valid": good, "ready": ready, "native_samples": count, "observed_window_s": extent,
        "mean_thread_weight_fraction": thread, "mean_positive_hand_upward_weight_fraction": hand,
        "scope": "Gravity-support/quiet search windows only; no formed capture, lead or passive-reset qualification"}


def validate_feedback_columns(columns, original_times, original_indices):
    """Keep the new ledger's scalar/boolean/vector schema independent of legacy loads."""
    count = len(original_times)
    vectors = {"thread_wrench_on_bolt_world_N_Nm", "hand_wrench_on_bolt_world_N_Nm",
        "right_command_wrench_N_Nm", "left_command_wrench_N_Nm",
        "right_motor_torques_Nm", "left_motor_torques_Nm"}
    for name, value in columns.items():
        shape = (count, 6) if name in vectors else (count, 2) if name == "right_pad_normal_force_N" else (count,)
        if value.shape != shape or not np.isfinite(value).all():
            raise ValueError(f"Invalid original feedback ledger column {name}")
    if not np.array_equal(columns["time"], original_times) or not np.array_equal(columns["phase_index"], original_indices):
        raise ValueError("Feedback history must cover exactly the original table-ledger native ticks and phases")
    for name in ("fully_open_unassisted", "right_grasp_guard_active", "right_axial_float_active",
        "weight_window_ready", "open_weight_window_ready", "all_hard_guards_held",
        "external_drive_zero", "weight_observation_valid", "open_observation_valid"):
        if not np.isin(columns[name], [False, True]).all():
            raise ValueError(f"Original feedback {name} flags must be boolean")
    for name in ("right_robot_bolt_contact_count", "loaded_actual_interior_flank_contact_count",
        "bolt_world_support_contact_count", "nonthread_block_bolt_contact_count", "native_thread_contact_count"):
        if np.any(columns[name] < 0) or not np.equal(columns[name], np.floor(columns[name])).all():
            raise ValueError(f"Original feedback {name} counts must be nonnegative integers")


def first_eligible_open_ready_index(ids, dt, minimum_dwell_s, readiness, hand_speed):
    """Consume the first CURRENT ready solve after the declared minimum dwell."""
    eligible = ids[((np.arange(len(ids))+1)*dt+1e-12 >= minimum_dwell_s)
        & readiness[ids] & (hand_speed[ids] <= .01)]
    return int(eligible[0]) if len(eligible) else None


def original_grasp_flag_audit(columns, labels, metadata):
    """Original acquisition row is inactive; its measured guard starts next tick."""
    time, phase = columns["time"], columns["phase_index"].astype(int)
    names = np.asarray(labels, dtype=object)[phase]
    count = len(time)
    starts = np.full(count, -1, dtype=int)
    active = columns["right_grasp_guard_active"].astype(bool)
    for acquisition in metadata["right_grasp_acquisitions"]:
        index = int(np.searchsorted(time, acquisition["time_s"]))
        if index == count or abs(time[index]-acquisition["time_s"]) > 1e-9:
            raise ValueError("Measured grasp acquisition is absent from the original feedback ledger")
        if active[index]:
            raise ValueError("Acquisition row incorrectly claims the new grasp was already guarded")
        if index+1 < count:
            starts[index+1] = index+1
    clear = np.array([name.startswith(("release_", "reset_open_", "open_settle_", "open_hold_", "regrip_"))
                      for name in names])
    last_clear = np.maximum.accumulate(np.where(clear, np.arange(count), -1))
    last_acquired = np.maximum.accumulate(starts)
    plan = {p[0]: p for p in metadata["maximum_phase_plan"]}
    closed_gap = metadata["control_config"]["arm"]["closed_aperture"]
    closed_target = np.array([plan[name][5] == closed_gap for name in names])
    expected = (last_acquired > last_clear) & closed_target
    if not np.array_equal(active, expected):
        raise ValueError("Original right grasp guards do not cover every post-acquisition closed native tick")
    return {"original_acquisition_rows_inactive_next_tick_guarded": True,
        "guarded_native_steps": int(np.sum(active)), "intentional_open_reference_clears_verified": True}


def original_feedback_audit(path, metadata, model, table_columns, labels, records, poses):
    """Bind the additional source/force/control/event ledger to every real native step."""
    version = metadata.get("physical_feedback_controller")
    if version is None:
        return {"observed": False, "passed": True,
            "scope": "Legacy supported recording; no later feedback metadata or observer criteria inferred"}
    if version != "supported-head-feedback-v1":
        raise ValueError("Unknown recorded supported physical-feedback version")
    declaration = metadata["native_feedback_force_history"]
    filename = declaration["filename"]
    if Path(filename).name != filename:
        raise ValueError("Feedback ledger must be an immutable sibling filename")
    raw = (path.parent/filename).read_bytes()
    if sha256(raw) != declaration["sha256"]:
        raise ValueError("Original feedback ledger bytes differ from recorded SHA")
    with np.load(io.BytesIO(raw), allow_pickle=False) as saved:
        columns = {name: saved[name].copy() for name in saved.files
                   if name not in {"phase_labels_json", "metadata_json"}}
        identity = json.loads(str(saved["metadata_json"]))
        actual_labels = json.loads(str(saved["phase_labels_json"]))
    if actual_labels != labels or declaration["columns"] != list(columns):
        raise ValueError("Feedback ledger schema/phase names differ from original declaration")
    for key in ("model_fingerprint", "controller_sha256", "runtime"):
        if identity[key] != metadata[key]:
            raise ValueError(f"Original feedback ledger has mismatched {key}")
    if (identity["timestep_s"] != model.opt.timestep
            or identity["force_timing"] != metadata["native_force_recording_note"]
            or identity["observer"] != "supported-original-native-feedback-ledger-v1"
            or identity["helper_source_sha256"] != metadata["feedback_source_sha256"]
            or declaration["helper_source_sha256"] != metadata["feedback_source_sha256"]
            or declaration["observed_physics_steps"] != len(table_columns["time"])):
        raise ValueError("Feedback timing, helper identity, observer or coverage changed")
    validate_feedback_columns(columns, table_columns["time"], table_columns["phase_index"])
    grasp_flags = original_grasp_flag_audit(columns, labels, metadata)
    dt, time, phase = float(model.opt.timestep), columns["time"], columns["phase_index"].astype(int)
    weight = float(model.body_subtreemass[model.body("male_bolt").id]*np.linalg.norm(model.opt.gravity))
    general = feedback_weight_windows(columns, dt, weight)
    opened = feedback_weight_windows(columns, dt, weight, fully_open=True)
    for name, ours in (("weight_window_ready", general), ("open_weight_window_ready", opened)):
        if not np.array_equal(columns[name].astype(bool), ours["ready"]):
            raise ValueError(f"Original {name} does not match independent original-input aggregation")
    checks = {"all_recorded_hard_guards_held": bool(np.all(columns["all_hard_guards_held"])),
        "every_original_substep_external_drive_zero": bool(np.all(columns["external_drive_zero"]))}
    names = np.asarray(labels, dtype=object)[phase]
    fully_open = np.array([name.startswith(("open_settle_", "reset_open_", "open_hold_")) for name in names])
    if not np.array_equal(fully_open, columns["fully_open_unassisted"].astype(bool)):
        raise ValueError("Original fully-open flags do not cover the actual open phases")
    checks["whole_right_robot_has_zero_contacts_while_fully_open"] = bool(
        np.all(columns["right_robot_bolt_contact_count"][fully_open] == 0))
    checks["open_free_hand_has_no_extra_axial_feed_or_closed_feedback"] = bool(
        np.all(columns["right_applied_axial_feed_N"][fully_open] == 0)
        and not np.any(columns["right_axial_float_active"][fully_open]))
    closed = np.array([name in {"transfer_bolt_weight", "reverse_seat_1", "stop_reverse_seat_1"}
        or name.startswith(("start_thread_", "turn_", "stop_")) for name in names])
    checks["closed_head_feedback_uses_axial_force_float"] = bool(np.all(columns["right_axial_float_active"][closed]))
    active = columns["right_grasp_guard_active"].astype(bool)
    checks["original_all_step_right_grasp_retention"] = bool(
        np.all(columns["right_grip_slip_m"][active] <= .001)
        and np.all(columns["right_grip_rotation_slip_rad"][active] <= np.deg2rad(2))
        and np.all(columns["right_pad_normal_force_N"][active] > .1))
    checks["original_all_step_left_grasp_retention"] = bool(
        np.all(columns["left_grip_slip_m"][table_columns["supported_task_active"].astype(bool)] <= .001)
        and np.all(columns["left_grip_rotation_slip_rad"][table_columns["supported_task_active"].astype(bool)] <= np.deg2rad(2)))
    arm = metadata["control_config"]["arm"]
    motor_checks = []
    for side in ("right", "left"):
        wrench = columns[f"{side}_command_wrench_N_Nm"]
        ids = [model.actuator(f"{side}_servo{i}").id for i in range(1, 7)]
        commands = columns[f"{side}_motor_torques_Nm"]
        motor_checks.append(bool(np.all(np.linalg.norm(wrench[:, :3], axis=1) <= arm["maximum_cartesian_force"]+1e-12)
            and np.all(np.linalg.norm(wrench[:, 3:], axis=1) <= arm["maximum_cartesian_torque"]+1e-12)
            and np.all(commands >= model.actuator_ctrlrange[ids, 0]-1e-12)
            and np.all(commands <= model.actuator_ctrlrange[ids, 1]+1e-12)))
    checks["every_original_finite_motor_and_wrench_command_bounded"] = all(motor_checks)
    saved_at = {float(row["time"]): (i, row) for i, row in enumerate(records)}
    checked_contacts = 0
    for time_s, (saved_index, row) in saved_at.items():
        index = int(np.searchsorted(time, time_s))
        feedback = row["native_feedback"]
        for key in ("thread_wrench_on_bolt_world_N_Nm", "hand_wrench_on_bolt_world_N_Nm",
                    "right_pad_normal_force_N", "thread_gravity_opposing_force_N",
                    "hand_gravity_opposing_force_N", "loaded_actual_interior_flank_contact_count"):
            if not np.allclose(feedback[key], columns[key][index], rtol=1e-12, atol=1e-12):
                raise ValueError("Sampled original feedback force differs from its every-step ledger")
        for key in ("fully_open_unassisted", "right_grasp_guard_active", "right_robot_bolt_contact_count",
                    "right_axial_float_active", "weight_observation_valid", "open_observation_valid"):
            if row[key] != columns[key][index]:
                raise ValueError("Sampled original feedback control mode differs from its native step")
        for field, ours in (("weight_window", general), ("open_weight_window", opened)):
            original = row[field]
            if (original["ready_for_diagnostic_release_attempt"] != bool(ours["ready"][index])
                    or original["original_native_samples"] != int(ours["native_samples"][index])):
                raise ValueError("Sampled original feedback window extent/readiness differs from its raw history")
            for key in ("observed_window_s", "mean_thread_weight_fraction", "mean_positive_hand_upward_weight_fraction"):
                if original[key] is not None and not np.isclose(original[key], ours[key][index], rtol=1e-7, atol=1e-9):
                    raise ValueError("Sampled original feedback observer load differs from independent native inputs")
        wrench = np.zeros(6)
        for contact in feedback["native_contact_records"]:
            if {contact["geom1"], contact["geom2"]} != {"bolt_thread", "female_thread"}:
                raise ValueError("Feedback thread wrench includes a different contact pair")
            wrench += transform_contact_wrench(contact["frame"], contact["local_force_N_Nm"],
                contact["contact_position_world_m"], contact["bolt_origin_world_m"],
                -1. if contact["geom1"] == "bolt_thread" else 1.)
        if not np.allclose(wrench, columns["thread_wrench_on_bolt_world_N_Nm"][index], rtol=1e-10, atol=1e-10):
            raise ValueError("Original thread local solves do not reconstruct their full world wrench")
        checked_contacts += len(feedback["native_contact_records"])
        calibration = row.get("command_calibration")
        if calibration is not None:
            matrix, vector = np.asarray(calibration["head_to_tool_R"]), np.asarray(calibration["head_to_tool_p_m"])
            if (matrix.shape != (3, 3) or vector.shape != (3,) or not np.isfinite(matrix).all()
                    or not np.isfinite(vector).all() or not np.allclose(matrix@matrix.T, np.eye(3), atol=1e-9)
                    or abs(calibration["source_prior_solve_time_s"]-(time_s-2*dt)) > 1e-9):
                raise ValueError("Command calibration lacks its separately timed retained-pose proof")
    phase_counts, events = {}, []
    for label in labels:
        ids = np.flatnonzero(names == label)
        if len(ids):
            phase_counts[label] = {"first_original_time_s": float(time[ids[0]]),
                "last_original_time_s": float(time[ids[-1]]), "native_steps": len(ids), "actual_duration_s": len(ids)*dt}
    for event in metadata["physical_motion_events"]:
        if event["event"] != "Live physical phase readiness consumed":
            continue
        event_time, label = float(event["time_s"]), event["phase"]
        if event_time not in saved_at:
            raise ValueError("Physical readiness event lacks its exact saved original force/post-state row")
        index = int(np.searchsorted(time, event_time))
        ids = np.flatnonzero(names == label)
        if index != ids[-1] or abs(event["force_time_s"]-(event_time-dt)) > 1e-9:
            raise ValueError("Physical phase event is not the last actual original solve of its phase")
        actual_dwell = (index-ids[0]+1)*dt
        if abs(event["actual_dwell_s"]-actual_dwell) > 1e-9:
            raise ValueError("Physical event dwell differs from immutable phase steps")
        if label.startswith("open_settle_"):
            first = first_eligible_open_ready_index(ids, dt,
                metadata["control_config"]["minimum_open_settle_s"], opened["ready"],
                columns["relative_hand_angular_speed_rad_per_s"])
            if first != index:
                raise ValueError("Open phase did not consume its first eligible live original quiet/support window")
        events.append({"phase": label, "original_time_s": event_time,
            "original_force_time_s": event_time-dt, "saved_post_state_time_s": event_time,
            "actual_native_dwell_s": actual_dwell})
    return {"observed": True, "passed": bool(all(checks.values())), "raw_sha256": sha256(raw),
        "original_native_steps": len(time), "original_thread_contact_frame_checks": checked_contacts,
        "helper_source_sha256": identity["helper_source_sha256"], "independent_checks": checks,
        "actual_original_phase_intervals": phase_counts, "actual_saved_physical_events": events,
        "original_grasp_flag_boundaries": grasp_flags,
        "fully_open_original_steps": int(np.sum(fully_open)),
        "loaded_actual_interior_original_steps": int(np.sum(columns["loaded_actual_interior_flank_contact_count"] > 0)),
        "maximum_native_right_slip_during_active_grasps_m": float(columns["right_grip_slip_m"][active].max(initial=0.)),
        "maximum_native_right_rotation_slip_during_active_grasps_rad": float(columns["right_grip_rotation_slip_rad"][active].max(initial=0.)),
        "scope": "Source-bound original all-step forces/controls and actual event clocks. Prior command-calibration t-2dt, original native force/derived pose t-dt, and saved post-qpos t are separate. Interior loads or readiness do not establish a full-pitch capture, qualified lead or reset."}


def saved_grasp_retention(rows, records, held_poses, metadata):
    """Measure every closed phase against its separately replayed grasp reference."""
    acquisitions = metadata["right_grasp_acquisitions"]
    events = {float(event["time_s"]): event for event in acquisitions}
    if len(events) != len(acquisitions):
        raise ValueError("Right native grasp acquisitions must have unique original times")
    reference, replayed_acquisitions = None, []
    missing_reference = False
    maximum_grip_translation = maximum_grip_rotation = 0.
    for row, record, held_pose in zip(rows, records, held_poses):
        label = row["phase"]
        if label.startswith(("release_", "reset_open_", "regrip_", "open_settle_", "open_hold_")):
            reference = None
        event = events.get(row["time_s"])
        if event is not None:
            if (event["phase"] != label or not label.startswith(("settle_bolt", "settle_regrip_"))
                    or not np.all(np.asarray(event["pad_normal_force_N"]) > .1)
                    or np.linalg.norm(event["grasp_relative_bolt_head_position_m"]) >= .001
                    or not np.allclose(event["pad_normal_force_N"],
                        record["contact"]["pad_normal_force_N"], rtol=1e-12, atol=1e-12)):
                raise ValueError("Right acquisition lacks its original settled, centered bilateral native load proof")
            required_regrasp = float(metadata.get("control_config", {}).get("regrasp_window_s", .1))
            if ("continuous_quiet_bilateral_streak_s" in event
                    and event["continuous_quiet_bilateral_streak_s"] < required_regrasp-1e-12):
                raise ValueError("Right regrasp acquisition lacks its declared continuous100ms quiet native window")
            reference = held_pose
            replayed_acquisitions.append({"time_s": row["time_s"], "phase": label,
                "original_pre_integration_head_relative_position_m": event["grasp_relative_bolt_head_position_m"],
                "independent_post_integration_head_relative_position_m": held_pose[0].tolist(),
                "reference_timing_offset_note": "Native contact/reference belongs to t-dt; this independent retention reference is derived from corresponding saved qpos at t"})
        closed_manipulation = (label in {"lift_bolt", "transport_bolt", "align_over_hole",
                                        "feed_to_entry", "transfer_bolt_weight"}
                              or label.startswith(("reverse_seat_", "start_thread_", "turn_", "stop_")))
        feedback = record.get("native_feedback", {})
        if (metadata.get("physical_feedback_controller")
                and "right_grasp_guard_active" not in record
                and "right_grasp_guard_active" not in feedback):
            raise ValueError("A feedback sample lacks its original native right grasp-active flag")
        if "right_grasp_guard_active" in record or "right_grasp_guard_active" in feedback:
            flag = record.get("right_grasp_guard_active", feedback.get("right_grasp_guard_active"))
            if not isinstance(flag, bool):
                raise ValueError("Saved native right grasp-active flags must be boolean")
            # A real quiet acquisition may occur within settle_regrip. Its
            # original row still uses the preceding inactive reference; the
            # next tick guards the newly measured grasp, including that same
            # settle phase. Keep this source boundary separate from saved
            # post-qpos retention instead of forcing acquisition to phase end.
            closed_manipulation = bool(closed_manipulation or flag)
        if closed_manipulation:
            if reference is None:
                missing_reference = True
            else:
                shift = float(np.linalg.norm(held_pose[0]-reference[0]))
                angle = float(Rotation.from_matrix(held_pose[1]@reference[1].T).magnitude())
                maximum_grip_translation = max(maximum_grip_translation, shift)
                maximum_grip_rotation = max(maximum_grip_rotation, angle)
                row["independent_post_grasp_translation_slip_m"] = shift
                row["independent_post_grasp_rotation_slip_rad"] = angle
    if len(replayed_acquisitions) != len(acquisitions):
        raise ValueError("Declared right native grasp acquisition has no corresponding saved pose")
    return {"right_grasp_acquisition_replay": replayed_acquisitions,
        "closed_manipulation_without_measured_grasp_reference": missing_reference,
        "maximum_independent_post_grasp_translation_slip_m": maximum_grip_translation,
        "maximum_independent_post_grasp_rotation_slip_rad": maximum_grip_rotation}


def saved_geometry_audit(model, times, poses, records, metadata):
    """Exact saved qpos kinematics/candidates, explicitly separate from loads."""
    if (poses.shape != (len(times), model.nq) or len(records) != len(times)
            or not np.isfinite(poses).all()):
        raise ValueError("Saved poses and native diagnostic rows are misaligned")
    data = mujoco.MjData(model)
    male, female, block = (model.body(name).id for name in
                          ("male_bolt", "female_frame", "fixture_block"))
    bolt, female_geom, head = (model.geom(name).id for name in
                               ("bolt_thread", "female_thread", "bolt_head"))
    tables = {model.geom(name).id for name in metadata["table_support_geom_names"]}
    left_pads = {model.geom(f"left_m8_pad_{side}").id for side in ("left", "right")}
    right_pads = {model.geom(f"right_m8_pad_{side}").id for side in ("left", "right")}
    thread = ThreadConfig(**metadata["scene_config"]["base"]["thread"])
    rows, angles, held_poses = [], [], []
    for time, pose, record in zip(times, poses, records):
        if abs(float(time)-record["time"]) > 1e-10:
            raise ValueError("Saved qpos timestamps differ from original native diagnostic rows")
        data.qpos[:] = pose
        mujoco.mj_kinematics(model, data)
        mujoco.mj_collision(model, data)
        fr, mr = data.xmat[female].reshape(3, 3), data.xmat[male].reshape(3, 3)
        relative, rotation = fr.T@(data.xpos[male]-data.xpos[female]), fr.T@mr
        angles.append(float(np.arctan2(rotation[1, 0], rotation[0, 0])))
        lower, upper, length = full_ring_interval(relative[2], rotation[2, 2],
                                                 np.linalg.norm(rotation[2, :2]), thread)
        tip = data.xpos[male]+mr@[0., 0., thread.bolt_length]
        hand = model.site("right_grasp_site").id
        hand_r = data.site_xmat[hand].reshape(3, 3)
        head_p = data.xpos[male]+mr@[0., 0., -metadata["scene_config"]["head_height"]/2]
        held_poses.append((hand_r.T@(head_p-data.site_xpos[hand]), hand_r.T@mr))
        # This demo deliberately uses a continuous horizontal box tabletop.
        table_tops = []
        for table in tables:
            if (model.geom_type[table] != mujoco.mjtGeom.mjGEOM_BOX
                    or not np.allclose(data.geom_xmat[table].reshape(3, 3), np.eye(3), atol=1e-12)):
                raise ValueError("Supported limited-depth demo requires the declared horizontal solid box table")
            table_tops.append(data.geom_xpos[table, 2]+model.geom_size[table, 2])
        counters = {"male_world_candidates": 0, "block_table_candidates": 0,
            "head_block_seating_candidates": 0, "right_hand_bolt_candidates": 0,
            "left_nonpad_block_candidates": 0, "thread_candidates": 0,
            "interior_flank_candidates": 0}
        unexpected = []
        for contact in data.contact:
            pair = {int(contact.geom1), int(contact.geom2)}
            bodies = [int(model.geom_bodyid[g]) for g in (contact.geom1, contact.geom2)]
            roots = {int(model.body_weldid[b]) for b in bodies}
            names = [model.geom(int(g)).name for g in (contact.geom1, contact.geom2)]
            if male in roots and 0 in roots:
                counters["male_world_candidates"] += 1
            if block in roots and pair&tables:
                counters["block_table_candidates"] += 1
            if head in pair and block in roots:
                counters["head_block_seating_candidates"] += 1
            if male in roots and any(model.body(b).name.startswith("right_") for b in bodies):
                counters["right_hand_bolt_candidates"] += 1
            if block in roots and any(model.body(b).name.startswith("left_") for b in bodies) and not pair&left_pads:
                counters["left_nonpad_block_candidates"] += 1
            if pair == {bolt, female_geom}:
                counters["thread_candidates"] += 1
                mz, fz = (mr.T@(contact.pos-data.xpos[male]))[2], (fr.T@(contact.pos-data.xpos[female]))[2]
                female_cut = 5*np.sqrt(3)*thread.pitch/16+.000360
                if 0 < mz < thread.bolt_length-.000956 and abs(fz) < thread.nut_height/2-female_cut:
                    counters["interior_flank_candidates"] += 1
            intended = (pair == {bolt, female_geom}
                or head in pair and bool(pair&right_pads)
                or block in roots and bool(pair&left_pads)
                or block in roots and bool(pair&tables)
                or head in pair and any(name.startswith("bolt_rest_pin_") for name in names)
                or head in pair and block in roots)
            if not intended and contact.dist < 0:
                unexpected.append({"geoms": names, "signed_distance_m": float(contact.dist)})
        rows.append({"time_s": float(time), "phase": record["phase"],
            "recorded_capture": bool(record["thread_engaged"]),
            "male_base_z_in_female_m": float(relative[2]),
            "radial_offset_m": float(np.linalg.norm(relative[:2])),
            "tilt_rad": float(np.arccos(np.clip(rotation[2, 2], -1., 1.))),
            "potential_full_ring_interval_m": [lower, upper],
            "potential_full_ring_length_m": length,
            "tip_table_clearance_m": float(tip[2]-max(table_tops)),
            "block_position_m": data.xpos[block].tolist(),
            "unexpected_penetrating_contacts": unexpected, **counters})
    yaw = np.unwrap(angles)
    for row, angle in zip(rows, yaw):
        row["male_yaw_unwrapped_in_female_rad"] = float(angle)
    grip = saved_grasp_retention(rows, records, held_poses, metadata)
    turns = []
    for label in sorted({row["phase"] for row in rows if row["phase"].startswith("turn_")}):
        mask = np.array([row["phase"] == label for row in rows])
        z = np.array([row["male_base_z_in_female_m"] for row in rows])
        turns.append({"phase": label, "fit": lead_fit(z, yaw, mask, thread.pitch),
            "minimum_potential_full_ring_length_m": min(row["potential_full_ring_length_m"] for row in rows if row["phase"] == label)})
    margins = {}
    for side in ("left", "right"):
        joints = [model.joint(f"{side}_joint{i}").id for i in range(1, 7)]
        q = poses[:, model.jnt_qposadr[joints]]
        limits = model.jnt_range[joints]
        margins[side] = float(np.minimum(q-limits[:, 0], limits[:, 1]-q).min())
    return {"method": "Saved post-integration qpos only; native kinematics/collision candidates, zero dynamics integration or force solve",
        "scope": "Candidates do not prove native loads. Original mj_step derived geometry precedes these integrated poses by one step.",
        "sample_count": len(rows), "sampled_native_joint_margins_rad": margins,
        "minimum_sampled_tip_table_clearance_m": min(row["tip_table_clearance_m"] for row in rows),
        **grip,
        "unexpected_sampled_penetration_count": sum(len(row["unexpected_penetrating_contacts"]) for row in rows),
        "qualified_sampled_lead_fits": turns, "rows": rows}


def archive_and_actuation_audit(path, metadata, model, commands, runtime):
    """Exact archived sources, native transmission caps and declared exclusions."""
    source = path.parent/"controller_source.py"
    identity = inspected_source_identity(source)
    if (identity["module_sha256"] != metadata["controller_module_sha256"]
            or identity["controller_sha256"] != metadata["controller_sha256"]):
        raise ValueError("Archived supported controller whole-module/bundle identity mismatch")
    dependencies = metadata["recorded_source_dependencies_sha256"]
    for name, expected in dependencies.items():
        relative = Path(name)
        if relative.is_absolute() or ".." in relative.parts or relative.parts[:2] != ("recorded_sources", "yam_twin"):
            raise ValueError("Invalid archived source dependency path")
        if sha256((path.parent/relative).read_bytes()) != expected:
            raise ValueError(f"Archived source dependency changed: {name}")
    for name, expected in identity["helper_source_sha256"].items():
        if name not in dependencies or dependencies[name] != expected:
            raise ValueError("Inspected controller helper lacks matching immutable whole-source archive")
    if metadata.get("physical_feedback_controller"):
        feedback_sha = metadata["feedback_source_sha256"]
        helper_files = [name for name, digest in identity["helper_source_sha256"].items()
                        if digest == feedback_sha]
        if (len(helper_files) != 1 or dependencies.get(helper_files[0]) != feedback_sha
                or not helper_files[0].endswith(("m8_supported_start.py", "m8_supported_feedback.py"))):
            raise ValueError("Physical feedback helper lacks its selected and whole-source archive identity")
    observer_sha = sha256((path.parent/"engagement_observer_source.py").read_bytes())
    if observer_sha != metadata["engagement_observer_source_sha256"]:
        raise ValueError("Archived full-flank observer bytes differ from recorded observer")
    expected_runtime = metadata["runtime"]
    runtime_ok = (runtime["mujoco_version"] == expected_runtime["mujoco_version"]
        and runtime["thread_plugin_source_sha256"] == expected_runtime["thread_plugin_source_sha256"]
        and sorted(lib["sha256"] for lib in runtime["libraries"])
        == sorted(lib["sha256"] for lib in expected_runtime["libraries"]))
    if not runtime_ok:
        raise ValueError("Independent audit native core/plugin differs from original runtime")
    if commands.shape[1:] != (model.nu,) or not np.isfinite(commands).all():
        raise ValueError("Invalid original native actuator command array")
    finite_caps = bool(np.all(model.actuator_ctrllimited) and np.all(model.actuator_forcelimited)
        and np.isfinite(model.actuator_ctrlrange).all() and np.isfinite(model.actuator_forcerange).all())
    commands_bounded = bool(finite_caps and np.all(commands >= model.actuator_ctrlrange[:, 0]-1e-12)
        and np.all(commands <= model.actuator_ctrlrange[:, 1]+1e-12))
    xml = ET.fromstring((path.parent/"scene.xml").read_bytes())
    thread = metadata["scene_config"]["base"]["thread"]
    if model.opt.timestep != thread["timestep"]:
        raise ValueError("Recorded thread timestep differs from compiled archived dynamics")
    for name, expected in (("bolt_shape", {"pitch": thread["pitch"], "length": thread["bolt_length"],
            "pitch_diameter": thread["male_pitch_diameter"], "female": 0., "chamfer": .000956}),
            ("nut_shape", {"pitch": thread["pitch"], "length": thread["nut_height"],
            "pitch_diameter": thread["female_pitch_diameter"], "female": 1.,
            "af": thread["nut_across_flats"], "chamfer": .000360})):
        instance = xml.find(f"extension/plugin/instance[@name='{name}']")
        values = {} if instance is None else {entry.get("key"): float(entry.get("value")) for entry in instance.findall("config")}
        if any(not np.isclose(values.get(key, np.nan), value, rtol=0., atol=1e-15) for key, value in expected.items()):
            raise ValueError("Recorded geometry dimensions differ from exact archived thread SDF configuration")
    exclusions = [{"body1": node.get("body1"), "body2": node.get("body2")}
                  for node in xml.findall("contact/exclude")]
    expected_exclusions = {tuple(sorted((f"{side}_{first}", f"{side}_{second}")))
        for side in ("left", "right") for first, second in (("link_4", "link_6"),
            ("link_5", "link_left_finger"), ("link_5", "link_right_finger"))}
    only_native_exclusions = (len(exclusions) == len(expected_exclusions)
        and {tuple(sorted((item["body1"], item["body2"]))) for item in exclusions} == expected_exclusions)
    masks = []
    block_root = model.body("fixture_block").id
    male_root = model.body("male_bolt").id
    tables = [model.geom(name).id for name in metadata["table_support_geom_names"]]
    for index in range(model.ngeom):
        root = int(model.body_weldid[int(model.geom_bodyid[index])])
        name = model.geom(index).name
        if root not in {block_root, male_root} and index not in tables and not name.startswith(("left_m8_pad_", "right_m8_pad_")):
            continue
        masks.append({"geom": name, "contype": int(model.geom_contype[index]),
                      "conaffinity": int(model.geom_conaffinity[index])})
    table_masks_ok = all(item["contype"] == 16 and item["conaffinity"] == 14
                         for item in masks if item["geom"] in metadata["table_support_geom_names"])
    interactions_ok = all(bool((model.geom_contype[g]&model.geom_conaffinity[t])
                              or (model.geom_contype[t]&model.geom_conaffinity[g]))
        for t in tables for g in range(model.ngeom)
        if int(model.body_weldid[int(model.geom_bodyid[g])]) in {block_root, male_root}
        or model.geom(g).name.startswith(("left_m8_pad_", "right_m8_pad_")))
    return {"controller_identity": identity, "all_archived_whole_dependencies_match_recorded": True,
        "dependency_sha256": dependencies, "runtime_core_and_plugin_match_recorded": runtime_ok,
        "native_actuator_commands_within_finite_caps": commands_bounded,
        "native_contact_exclusions": exclusions, "only_declared_native_adjacent_chain_exclusions": only_native_exclusions,
        "workpiece_table_pad_collision_masks": masks,
        "plain_table_masks_preserved": table_masks_ok,
        "table_collisions_with_both_workpieces_and_pads_remain_enabled": interactions_ok}


def audit(path):
    path = Path(path).resolve()
    runtime = require_micron_engine()
    with np.load(path, allow_pickle=False) as saved:
        times, poses = saved["time"].copy(), saved["qpos"].copy()
        commands = saved["controller"].copy()
        records = json.loads(str(saved["info_json"]))
        metadata = json.loads(str(saved["metadata_json"]))
    if not metadata.get("supported_task"):
        raise ValueError("This auditor accepts the separate table-supported task only")
    if not len(times) or times.ndim != 1 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError("Saved supported trajectory times must be finite and increasing")
    if commands.shape[0] != len(times):
        raise ValueError("Saved native actuator commands do not align with pose times")
    model, model_identity = recorded_model(path, metadata)
    if not np.allclose(model.opt.gravity, [0., 0., -9.81], rtol=0., atol=1e-12):
        raise ValueError("Supported weight-bearing audit requires declared Earth vertical gravity")
    identity = archive_and_actuation_audit(path, metadata, model, commands, runtime)
    loads, columns, labels = original_load_audit(path, metadata, model, records, float(times[-1]))
    feedback = original_feedback_audit(path, metadata, model, columns, labels, records, poses)
    geometry = saved_geometry_audit(model, times, poses, records, metadata)
    bolt_phases = all_step_bolt_measurements(columns, labels, metadata)
    passive = passive_audit(path)
    original_checks, absent_original = strict_original_checks(metadata)
    turns = [item for item in bolt_phases if item["phase"].startswith("turn_")]
    resets = [item for item in bolt_phases if item["phase"].startswith("reset_open_")]
    captured_resets = [item for item in resets if item["started_with_recorded_capture"]]
    required_turns = int(metadata["control_config"]["qualifying_turns"])
    minimum_clearance = float(metadata["minimum_tip_table_clearance_m"])
    checks = {
        "complete_unaborted_native_sequence": not metadata["partial"] and metadata["aborted"] is None,
        "source_bound_original_feedback_controls_forces_and_events": feedback["passed"],
        "measured_pre_bolt_table_bears_weight": loads["settled_pre_bolt_baseline_passed"],
        "whole_task_table_bears_weight_left_merely_stabilizes": loads["whole_task_active_rolling_weight_bearing"]["passed"],
        "strict_all_substep_table_load_retention": loads["after_stabilization_load_statistics"]["continuous_table_load"],
        "strict_all_substep_bilateral_left_pad_preload": loads["after_stabilization_load_statistics"]["continuous_bilateral_pad_load"],
        "block_has_no_lift_or_unintended_world_support": loads["all_step_maximum_block_translation_m"] <= .001
            and loads["all_step_maximum_block_rotation_rad"] <= np.deg2rad(2)
            and loads["all_step_maximum_block_lift_m"] <= metadata["control_config"]["maximum_block_lift_m"]
            and loads["all_step_maximum_unexpected_world_support_count"] == 0,
        "male_clears_solid_table_in_original_and_replayed_geometry": loads["all_step_minimum_tip_table_clearance_m"] >= minimum_clearance
            and geometry["minimum_sampled_tip_table_clearance_m"] >= minimum_clearance,
        "separately_picked_up_bolt_has_no_later_world_support": loads["all_step_post_pickup_bolt_world_contact_count"] == 0
            and any(item["phase"] == "transport_bolt" for item in metadata["phases"]),
        "native_qualified_lead_and_rotation": len(turns) == required_turns and all(item["qualified_stroke_passed"] for item in turns),
        "independent_saved_pose_qualified_lead": len(geometry["qualified_sampled_lead_fits"]) == required_turns
            and all(item["fit"]["passed"] for item in geometry["qualified_sampled_lead_fits"]),
        "all_open_resets_have_no_hand_bolt_contact": bool(resets) and all(item["open_hand_contact_decoupling_passed"] for item in resets),
        "captured_bolt_resets_are_passive_and_unseated": bool(captured_resets) and all(item["captured_passive_reset_passed"] for item in captured_resets),
        "no_sampled_unintended_penetrations": geometry["unexpected_sampled_penetration_count"] == 0,
        "original_all_substep_native_collision_guard": loads["all_step_maximum_unexpected_native_contact_count"] == 0
            and loads["all_step_maximum_unexpected_native_contact_depth_m"] <= 1e-6,
        "native_joint_limits_and_finite_motor_caps": min(geometry["sampled_native_joint_margins_rad"].values()) >= -1e-5
            and identity["native_actuator_commands_within_finite_caps"],
        "each_right_grasp_is_measured_and_retained": bool(geometry["right_grasp_acquisition_replay"])
            and not geometry["closed_manipulation_without_measured_grasp_reference"]
            and geometry["maximum_independent_post_grasp_translation_slip_m"] < .001
            and geometry["maximum_independent_post_grasp_rotation_slip_rad"] < np.deg2rad(2),
        "zero_original_external_drive_and_freebody_artificial_passive_terms": loads["every_original_substep_external_drive_zero"]
            and all(item["all_tested_passive_artificial_terms_zero"] for item in passive["workpieces"])
            and passive["free_object_welds_or_other_equalities_absent"]
            and passive["only_native_robot_joint_actuators"]
            and passive["fluid_density"] == 0 and passive["fluid_viscosity"] == 0,
        "no_new_task_collision_exclusions_or_disabled_table_masks": identity["only_declared_native_adjacent_chain_exclusions"]
            and identity["plain_table_masks_preserved"]
            and identity["table_collisions_with_both_workpieces_and_pads_remain_enabled"],
    }
    return {"kind": "Independent supported-block original-load and saved-geometry audit",
        "scope": "A limited-depth native tabletop running-thread trial. Original force aggregation and saved-pose geometry remain separate. No material calibration or policy-training robustness certificate.",
        "trajectory": str(path), "trajectory_sha256": sha256(path.read_bytes()),
        "auditor_source_sha256": sha256(Path(__file__).read_bytes()),
        "archived_model_identity": model_identity, "archive_and_actuation": identity,
        "original_load_history": loads, "original_all_step_bolt_phases": bolt_phases,
        "original_physical_feedback_history": feedback,
        "saved_geometry": geometry, "passive_workpiece_properties": passive,
        "original_report_passed": metadata["passed"],
        "original_acceptance_checks": original_checks,
        "original_unobserved_infinity_minima_exported_as_null": absent_original,
        "original_metadata_export_note": "Original bytes and failed gates remain unchanged; named unexecuted legacy minima are exported as null only in this strict JSON derivative",
        "independent_acceptance_checks": {name: {"passed": bool(value)} for name, value in checks.items()},
        "passed": bool(all(checks.values())),
        "reproduce_command": f"scripts/run_m8.sh scripts/audit_m8_supported_trace.py {path}"}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = audit(args.trajectory)
    output = args.output or args.trajectory.parent/"independent_supported_audit.json"
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"output": str(output), "passed": result["passed"],
        "checks": result["independent_acceptance_checks"]}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
