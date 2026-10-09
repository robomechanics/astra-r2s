"""Read original cold V3 ledgers; never import live app or solve/integrate physics."""
from __future__ import annotations
import argparse
import ast
from collections import deque
import hashlib
import inspect
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

VERSION = "independent-cold-crest-native-reader-v1"
HARN = "85c485e5382adad57e83bc8e3c924e0a9ac3b36dfc896ed77712201baeb02bf9"
OBS = "1d37ef46382c349e29b4206206e9aac2132587b65abc39467342e7e1b0560c0c"
HELP = "ffe021f74b879b91e961509d1b2b0071b7aadf8ff64b1329bf179d8f33758f51"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def equal(a, b):
    if isinstance(a, dict):
        return isinstance(b, dict) and a.keys() == b.keys() and all(equal(a[k], b[k]) for k in a)
    if isinstance(a, list):
        return isinstance(b, list) and len(a) == len(b) and all(equal(x, y) for x, y in zip(a, b))
    if isinstance(a, bool) or a is None or isinstance(a, str):
        return type(a) is type(b) and a == b
    return bool(np.isclose(a, b, rtol=1e-9, atol=1e-12))


def source_object(source, name):
    node = next((n for n in ast.parse(source).body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name == name), None)
    if node is None:
        return None
    first = min([node.lineno, *(n.lineno for n in node.decorator_list)])-1
    return "".join(inspect.getblock(source.splitlines(keepends=True)[first:]))


def bundle_identity(path):
    source = path.read_text()
    tree = ast.parse(source)
    tuples = []
    for n in ast.walk(tree):
        if isinstance(n, ast.Dict):
            for k, v in zip(n.keys, n.values):
                if isinstance(k, ast.Constant) and k.value == "controller_sha256":
                    tuples.extend(x.generators[0].iter for x in ast.walk(v) if isinstance(x, ast.GeneratorExp) and isinstance(x.generators[0].iter, ast.Tuple))
    require(len(tuples) == 1, "Ambiguous archived source bundle")
    imports = {a.asname or a.name: (n.module, a.name) for n in tree.body if isinstance(n, ast.ImportFrom) for a in n.names}
    pieces = []
    for n in tuples[0].elts:
        require(isinstance(n, ast.Name), "Malformed archived bundle member")
        piece = source_object(source, n.id)
        if piece is None:
            module, original = imports[n.id]
            helper = path.parent/"recorded_sources"/"yam_twin"/(module+".py")
            require(helper.is_file(), "Missing archived helper; no live fallback allowed")
            piece = source_object(helper.read_text(), original)
        require(piece is not None, "Missing source object")
        pieces.append(piece)
    return hashlib.sha256("\n".join(pieces).encode()).hexdigest()


def archived_classes(run):
    """Compile only pure observer class ASTs from immutable hash-bound source."""
    require(digest(run/"controller_source.py") == HARN, "Wrong cold harness")
    require(digest(run/"experimental_observer_source.py") == OBS, "Wrong crest observer")
    require(digest(run/"recorded_sources/yam_twin/m8_supported_start.py") == HELP, "Wrong native load observer")
    ns = {"np": np, "deque": deque}
    for file, name in (("recorded_sources/yam_twin/m8_insertion_simulation.py", "EntrySupportWindow"),
                       ("recorded_sources/yam_twin/m8_supported_start.py", "BoltWeightTransferWindow"),
                       ("controller_source.py", "TableLoadWindow"),
                       ("experimental_observer_source.py", "CrestSeatDropWindowV3")):
        source = (run/file).read_text()
        node = next(n for n in ast.parse(source).body if isinstance(n, ast.ClassDef) and n.name == name)
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(run/file), "exec"), ns)
    return ns


def contact_wrench(frame, local, position, origin, sign):
    frame, local = np.asarray(frame), np.asarray(local)
    require(frame.shape == (3, 3) and local.shape == (6,), "Bad original contact shape")
    require(np.allclose(frame@frame.T, np.eye(3), atol=1e-9), "Bad contact basis")
    f = sign*frame.T@local[:3]
    return np.r_[f, sign*frame.T@local[3:]+np.cross(np.asarray(position)-origin, f)]


def load_ledger(run, name, metadata):
    path = run/(name+".npz")
    declaration = metadata[name]
    require(digest(path) == declaration["sha256"], "Original ledger hash mismatch")
    with np.load(path, allow_pickle=False) as z:
        columns = {k: z[k].copy() for k in z.files if k not in {"metadata_json", "phase_labels_json"}}
        identity = json.loads(str(z["metadata_json"]))
        labels = json.loads(str(z["phase_labels_json"]))
    require(declaration["observed_physics_steps"] == len(columns["time"]), "Wrong step count")
    for k in ("model_fingerprint", "controller_sha256", "runtime"):
        require(identity[k] == metadata[k], "Ledger identity changed")
    for k, value in columns.items():
        require(value.shape[0] == len(columns["time"]) and np.isfinite(value).all(), "Malformed raw array "+k)
    return columns, labels


def verify_boolean_flags(columns):
    for k in ("fully_open_unassisted", "right_grasp_guard_active", "right_axial_float_active", "weight_window_ready", "open_weight_window_ready", "all_hard_guards_held", "external_drive_zero", "weight_observation_valid", "open_observation_valid", "cold_table_window_warmup"):
        require(columns[k].dtype.kind == "b", "Recorded guard flag must be boolean: "+k)


def validate_shapes(columns):
    count = len(columns["time"])
    vector6 = {"thread_wrench_on_bolt_world_N_Nm", "hand_wrench_on_bolt_world_N_Nm", "right_command_wrench_N_Nm", "left_command_wrench_N_Nm", "right_motor_torques_Nm", "left_motor_torques_Nm"}
    counts = {"phase_index", "right_robot_bolt_contact_count", "loaded_actual_interior_flank_contact_count", "bolt_world_support_contact_count", "nonthread_block_bolt_contact_count", "native_thread_contact_count"}
    for k, v in columns.items():
        shape = (count, 6) if k in vector6 else (count, 2) if k == "right_pad_normal_force_N" else (count,)
        require(v.shape == shape and np.isfinite(v).all(), "Wrong raw feedback shape: "+k)
        if k in counts:
            require(v.dtype.kind in "iu" and np.all(v>=0), "Contact/phase counts must be native nonnegative integers")
    verify_boolean_flags(columns)


def snapshot(i, feedback, table, labels, dt):
    result = {"native_index": int(i), "post_step_time_s": float(feedback["time"][i]),
              "original_force_time_s": float(feedback["time"][i]-dt), "phase": labels[int(feedback["phase_index"][i])]}
    for k in ("desired_independent_clock_rad", "desired_independent_angular_speed_rad_s", "relative_bolt_axial_velocity_m_per_s", "relative_bolt_angular_speed_rad_per_s", "relative_hand_angular_speed_rad_per_s", "radial_offset_m", "bolt_tilt_rad", "thread_gravity_opposing_force_N", "hand_gravity_opposing_force_N", "thread_summed_normal_force_N", "right_grip_slip_m", "right_grip_rotation_slip_rad", "all_hard_guards_held", "weight_window_ready"):
        result[k] = feedback[k][i].item()
    result["bolt_base_insertion_m"] = float(table["bolt_base_insertion_m"][i])
    for side in ("left", "right"):
        w = feedback[side+"_command_wrench_N_Nm"][i]
        result[side+"_command_force_norm_N"] = float(np.linalg.norm(w[:3]))
        result[side+"_command_torque_norm_Nm"] = float(np.linalg.norm(w[3:]))
    return result


def audit(run):
    run = Path(run).resolve()
    outer_path = run.with_name(run.name+"_execution_after.json")
    outer = json.loads(outer_path.read_text())
    require(outer["actual_native_exit_code"] == 1, "This reader binds the original closed failed V3")
    for k in ("source_files_unchanged", "producer_source_65_unchanged", "parent_inputs_unchanged", "runtime_files_unchanged"):
        require(outer[k] is True, "Closure identity failed: "+k)
    for k, v in outer["closed_output_sha256"].items():
        require(digest(run/k) == v, "Closed raw bytes changed: "+k)
    metadata = json.loads((run/"insertion_validation.json").read_text())
    require(metadata["physical_feedback_controller"] == "cold-crest-return-diagnostic-v3" and metadata["diagnostic_only"] is True, "Wrong branch scope")
    require(bundle_identity(run/"controller_source.py") == metadata["controller_sha256"], "Selected source identity differs")
    require(digest(run/"controller_source.py") == metadata["controller_module_sha256"], "Whole controller identity differs")
    for k, v in metadata["recorded_source_dependencies_sha256"].items():
        require(digest(run/k) == v, "Whole dependency identity differs")
    declaration = json.loads((run/"diagnostic_declaration.json").read_text())
    require(declaration == metadata["cold_initialization"], "Cold declaration is not exact inherited metadata")
    require(declaration["native_warm_start_copied"] is False, "Cold branch must not claim solver history")
    parent = Path(declaration["parent"]).parent
    for k, v in declaration["parent_input_sha256"].items():
        require(digest(parent/k) == v, "Parent source/checkpoint changed")
    with np.load(parent/"insertion_trace.npz", allow_pickle=False) as z:
        j = declaration["parent_native_checkpoint_index"]
        parent_metadata = json.loads(str(z["metadata_json"]))
        require(float(z["time"][j]) == declaration["parent_native_checkpoint_time_s"], "Wrong cold checkpoint time")
        for key, name in (("qpos", "qpos"), ("qvel", "qvel"), ("controller", "ctrl")):
            require(hashlib.sha256(z[key][j].tobytes()).hexdigest() == declaration[name+"_sha256"], "Wrong cold initial "+name)
    require(metadata["left_acquisition"] == parent_metadata["left_acquisition"] and metadata["right_grasp_acquisitions"] == [parent_metadata["right_grasp_acquisitions"][-1]], "Original physical grasp references changed")
    with np.load(run/"insertion_trace.npz", allow_pickle=False) as z:
        time_saved, qpos, qvel, controls = (z[k].copy() for k in ("time", "qpos", "qvel", "controller"))
        rows = json.loads(str(z["info_json"]))
        require(json.loads(str(z["metadata_json"])) == metadata, "Trace/report metadata differ")
    feedback, labels = load_ledger(run, "native_feedback_force_history", metadata)
    table, labels_t = load_ledger(run, "table_support_force_history", metadata)
    left, labels_l = load_ledger(run, "left_pad_force_history", metadata)
    require(labels == labels_t == labels_l, "Phase labels differ")
    require(metadata["native_feedback_force_history"]["columns"] == list(feedback) and len(feedback) == 47, "Wrong declared cold schema")
    validate_shapes(feedback)
    dt = metadata["scene_config"]["base"]["thread"]["timestep"]
    require(np.array_equal(feedback["thread_gravity_opposing_force_N"], feedback["thread_wrench_on_bolt_world_N_Nm"][:,2]) and np.array_equal(feedback["hand_gravity_opposing_force_N"], feedback["hand_wrench_on_bolt_world_N_Nm"][:,2]), "Gravity-opposing load changed from native world wrench")
    time = feedback["time"]
    require(np.allclose(np.diff(time), dt, rtol=1e-7, atol=1e-10), "Native history is not contiguous")
    require(abs(time[0]-dt)<1e-12 and time[-1] == metadata["aborted"]["time"], "Wrong original cold endpoints")
    for other in (table, left):
        require(np.array_equal(other["time"], time) and np.array_equal(other["phase_index"], feedback["phase_index"]), "Raw histories differ in timing")
    require(np.array_equal(left["pad_normal_force_N"], table["pad_normal_force_N"]), "Independent left pad recording differs")
    ns = archived_classes(run)
    bolt_weight = metadata["known_bolt_mass_kg"]*abs(metadata["scene_config"]["base"]["thread"]["gravity"])
    weight_win = ns["BoltWeightTransferWindow"](bolt_weight)
    table_win = ns["TableLoadWindow"](.1, metadata["known_block_weight_N"])
    crest = ns["CrestSeatDropWindowV3"](declaration["parent_original_settled_reference_event"]["base_z_m"], bolt_weight)
    saved_indices = {int(np.searchsorted(time, t)): r for t, r in zip(time_saved, rows)}
    require(len(saved_indices) == len(rows) and all(time[i] == r["time"] for i, r in saved_indices.items()), "Saved samples must match exact native ticks")
    table_ready = np.zeros(len(time), bool)
    weight_ready = np.zeros(len(time), bool)
    crest_ready = np.zeros(len(time), bool)
    warmup = np.zeros(len(time), bool)
    first_request = None
    event_reports = {}
    force_checks = {"thread": 0, "table": 0, "left": 0, "right": 0}
    for i, t in enumerate(time):
        label = labels[int(feedback["phase_index"][i])]
        table_ready[i] = table_win.observe(table["table_wrench_world_at_block_origin_N_Nm"][i, 2], table["left_hand_wrench_world_at_block_origin_N_Nm"][i, 2], dt, bool(table["supported_task_active"][i]))
        warmup[i] = label == "cold_window_hold" and table_win.elapsed+1e-12 < .1
        quiet = feedback["relative_bolt_angular_speed_rad_per_s"][i] <= .01 and feedback["relative_hand_angular_speed_rad_per_s"][i] <= .01 and abs(feedback["relative_bolt_axial_velocity_m_per_s"][i]) <= .0002
        valid = bool(feedback["all_hard_guards_held"][i] and quiet)
        require(valid == feedback["weight_observation_valid"][i], "Original general window validity differs")
        sample = {k: v[i] for k, v in feedback.items()}
        wr = weight_win.observe(float(t), dt, sample, valid=valid)
        weight_ready[i] = weight_win.ready
        seat_report = None
        if label in {"reverse_seat_1", "stop_reverse_seat_1"}:
            seat_valid = bool(feedback["all_hard_guards_held"][i] and table["table_wrench_world_at_block_origin_N_Nm"][i, 2] > .1*metadata["known_block_weight_N"] and table_ready[i])
            seat_report = crest.observe(float(t), dt, table["bolt_base_insertion_m"][i], feedback["relative_bolt_axial_velocity_m_per_s"][i], feedback["thread_summed_normal_force_N"][i], actual_relative_angular_speed_rad_per_s=max(feedback["relative_bolt_angular_speed_rad_per_s"][i], feedback["relative_hand_angular_speed_rad_per_s"][i]), valid=seat_valid, physically_stopped=label == "stop_reverse_seat_1")
            crest_ready[i] = crest.ready
            if first_request is None and crest.stop_requested:
                first_request = i
                event_reports[i] = seat_report
        if i not in saved_indices:
            continue
        r = saved_indices[i]
        require(equal(wr, r["weight_window"]), "Saved load window differs from exact archived-class replay")
        require(equal(seat_report, r["seat_direction_event"]), "Saved crest event/epoch differs from exact native replay")
        require(equal(table_win.report(), r["active_table_load_window"]), "Saved table window differs from raw history")
        for k in ("radial_offset_m", "bolt_tilt_rad", "right_pad_normal_force_N", "thread_wrench_on_bolt_world_N_Nm", "hand_wrench_on_bolt_world_N_Nm"):
            require(np.allclose(r["native_feedback"][k], feedback[k][i], rtol=1e-12, atol=1e-12), "Original force/sample mismatch")
        fb = r["native_feedback"]
        total = np.zeros(6)
        normals = 0.
        for c in fb["native_contact_records"]:
            require({c["geom1"], c["geom2"]} == {"bolt_thread", "female_thread"}, "Wrong original thread contact")
            w = contact_wrench(c["frame"], c["local_force_N_Nm"], c["contact_position_world_m"], c["bolt_origin_world_m"], -1 if c["geom1"] == "bolt_thread" else 1)
            require(np.allclose(w, c["wrench_on_bolt_world_N_Nm"], atol=1e-10), "Thread original signed action/reaction mismatch")
            total += w
            normals += c["local_force_N_Nm"][0]
            force_checks["thread"] += 1
        require(np.allclose(total, feedback["thread_wrench_on_bolt_world_N_Nm"][i], atol=1e-10) and np.isclose(normals, feedback["thread_summed_normal_force_N"][i]), "Thread summed original wrench differs")
        total = np.zeros(6)
        for c in r["table_support"]["contacts"]:
            w = contact_wrench(c["frame"], c["local_contact_force_N_Nm"], c["contact_position_world_m"], c["block_origin_world_m"], -1 if c["geom1"] == c["block_geom"] else 1)
            require(np.allclose(w[:3], c["signed_force_contribution_on_block_world_N"], atol=1e-10), "Original table reaction sign differs")
            total += w
            force_checks["table"] += 1
        require(np.allclose(total, table["table_wrench_world_at_block_origin_N_Nm"][i], atol=1e-10), "Original table local solve sum differs")
        for field, side, target in (("left_contact", "left", "block"), ("contact", "right", "bolt")):
            cdata = r[field]
            # Each record includes its exact signed world contribution; original
            # frame reconstruction is performed when local data is archived.
            require(np.allclose(cdata["wrench_world"], table["left_hand_wrench_world_at_block_origin_N_Nm"][i] if side == "left" else feedback["hand_wrench_on_bolt_world_N_Nm"][i], atol=1e-10), "Original native hand/sample sum differs")
            if side == "left":
                total = np.zeros(6)
                for c in cdata["native_contact_records"]:
                    require(c["geom1"].startswith("left_m8_pad_") or c["geom2"].startswith("left_m8_pad_"), "Unexpected original left load path")
                    w = contact_wrench(c["frame"], c["local_contact_force_N_Nm"], c["contact_position_world_m"], c["block_origin_world_m"], 1 if c["geom1"].startswith("left_m8_pad_") else -1)
                    require(np.allclose(w[:3], c["signed_force_contribution_on_block_world_N"], atol=1e-10), "Original left pad reaction sign differs")
                    total += w
                    force_checks["left"] += 1
                require(np.allclose(total, table["left_hand_wrench_world_at_block_origin_N_Nm"][i], atol=1e-10), "Original left local solve sum differs")
        cal = r["command_calibration"]
        require(np.allclose(np.asarray(cal["head_to_tool_R"])@np.asarray(cal["head_to_tool_R"]).T, np.eye(3), atol=1e-9), "Invalid command calibration rotation")
        require(abs(cal["source_prior_solve_time_s"]-(t-2*dt)) < 1e-9, "Command retained-pose timestamp differs")
    require(np.array_equal(weight_ready, feedback["weight_window_ready"]), "Every-step native weight readiness differs")
    require(np.array_equal(warmup, feedback["cold_table_window_warmup"]), "Cold unavailable-history flags differ")
    require(equal(crest.report(), metadata["final_experimental_crest_direction_event"]), "Final cleared epoch differs")
    require(first_request is not None, "No actual original crest-return request")
    events = []
    for event in metadata["physical_motion_events"]:
        if event["event"] != "Live physical phase readiness consumed":
            continue
        i = int(np.searchsorted(time, event["time_s"]))
        ids = np.flatnonzero(feedback["phase_index"] == feedback["phase_index"][i])
        require(i in saved_indices and i == ids[-1] and abs(event["force_time_s"]-(time[i]-dt)) < 1e-9, "Phase event lacks exact source-bound tick")
        require(abs(event["actual_dwell_s"]-len(ids)*dt) < 1e-9, "Event actual dwell differs")
        if event["phase"] == "cold_window_hold":
            eligible = ids[table_ready[ids] & weight_ready[ids] & ((np.arange(len(ids))+1)*dt >= .1-1e-12)]
            require(len(eligible) and i == eligible[0], "Cold hold did not consume first fresh eligible load window")
        elif event["phase"] == "reverse_seat_1":
            require(i == first_request and equal(event["seat_direction_event"], event_reports[i]), "Request not exact first native measured return")
        events.append(snapshot(i, feedback, table, labels, dt))
    tree = ET.parse(run/"scene.xml")
    excludes = [dict(e.attrib) for e in tree.findall("./contact/exclude")]
    require(not any("bolt" in str(e) or "block" in str(e) for e in excludes), "Workpiece collision disabled by exclusion")
    caps_ok = True
    for side in ("left", "right"):
        w = feedback[side+"_command_wrench_N_Nm"]
        bounds = np.array([np.fromstring(tree.find("./actuator/motor[@name='"+side+"_servo"+str(j)+"']").attrib["ctrlrange"], sep=" ") for j in range(1, 7)])
        motor = feedback[side+"_motor_torques_Nm"]
        caps_ok &= bool(np.all(np.linalg.norm(w[:, :3], axis=1)<=8+1e-12) and np.all(np.linalg.norm(w[:, 3:], axis=1)<=2+1e-12) and np.all(motor>=bounds[:, 0]-1e-12) and np.all(motor<=bounds[:, 1]+1e-12))
    require(caps_ok, "Original finite motor/wrench caps violated")
    last = len(time)-1
    stop = np.flatnonzero(feedback["phase_index"] == 2)
    require(len(stop) and stop[0] == first_request+1, "Next closed stop command did not start next native tick")
    boundary = [snapshot(j, feedback, table, labels, dt) for j in (first_request, stop[0], min(stop[0]+19,last), last)]
    require(np.array_equal(np.flatnonzero(~feedback["all_hard_guards_held"]), [last]), "Original radial abort tick count differs")
    require(np.array_equal(np.flatnonzero(feedback["radial_offset_m"]>150e-6), [last]), "Original alignment limit crossing differs")
    require(np.all(feedback["right_grasp_guard_active"]) and np.all(feedback["right_axial_float_active"]) and not np.any(feedback["fully_open_unassisted"]), "Cold held-grasp modes differ from actual phases")
    pads = left["pad_normal_force_N"]
    recorded_pads = metadata["all_substep_left_pad_loads"]
    require(recorded_pads["observed_physics_steps"] == len(time) and recorded_pads["minimum_normal_force_N"] == np.min(pads,axis=0).tolist() and recorded_pads["unloaded_physics_steps"] == np.sum(pads<=.1,axis=0).tolist(), "Original native bilateral statistics differ")
    return {"observer": VERSION, "reader_source_sha256": digest(__file__), "method": "Pure original-array arithmetic and exact archived pure observer AST replay; zero model integration or force recomputation", "original_passed": metadata["passed"], "diagnostic_completed": metadata["diagnostic_completed"], "full_fresh_trajectory_qualified": False, "capture_or_open_reset_qualified": False,
        "closed_original_exit_code": outer["actual_native_exit_code"], "original_aborted": metadata["aborted"],
        "source_and_parent_identity_verified": True, "selected_source_bundle_verified": True,
        "original_native_steps": len(time), "saved_post_state_samples": len(rows), "original_contact_frame_checks": force_checks,
        "right_local_force_frame_scope": "Right hand world wrench and pad normals are original native values matched between samples and all-step arrays; individual right pad local/frame records were not archived, so no independent right-local-frame reconstruction is claimed. Original source bundle includes its signed contact-force function; thread, table and left original local-frame sums are independently reconstructed.",
        "inherited_acceptance_scope": "Original producer labels are preserved verbatim. Its inherited pre-pickup/stabilization baseline is not fresh cold acquisition/pickup evidence; this reader separately verifies fresh local100ms windows and inherited fixed grasp references. Canonical402 tests and e468 reader are not coverage of this new diagnostic tag.",
        "every_step_observer_and_saved_event_consistency": True, "first_fresh_support_event": events[0], "first_moving_closed_stop_request": events[1], "first_request_crest_report": event_reports[first_request], "final_invalid_epoch_report": crest.report(),
        "cold_table_unavailable_steps": int(np.sum(warmup)), "raw_original_table_failed_window_steps": int(np.sum(~table_ready)), "complete_fresh_table_failed_windows": int(np.sum(~table_ready & ~warmup)), "raw_original_table_passed_label": metadata["acceptance_checks"]["table_bears_weight_throughout_active_task"]["passed"],
        "minimum_table_upward_force_N": float(np.min(table["table_wrench_world_at_block_origin_N_Nm"][:,2])), "final_table_upward_force_N": float(table["table_wrench_world_at_block_origin_N_Nm"][-1,2]), "final_left_signed_upward_force_N": float(table["left_hand_wrench_world_at_block_origin_N_Nm"][-1,2]), "maximum_positive_left_upward_force_N": float(np.max(np.maximum(table["left_hand_wrench_world_at_block_origin_N_Nm"][:,2],0))), "minimum_left_pad_normals_N": np.min(left["pad_normal_force_N"],axis=0).tolist(), "minimum_right_pad_normals_N": np.min(feedback["right_pad_normal_force_N"],axis=0).tolist(), "maximum_right_grip_slip_m": float(np.max(feedback["right_grip_slip_m"])), "maximum_right_rotation_slip_rad": float(np.max(feedback["right_grip_rotation_slip_rad"])), "maximum_left_grip_slip_m": float(np.max(feedback["left_grip_slip_m"])), "maximum_left_rotation_slip_rad": float(np.max(feedback["left_grip_rotation_slip_rad"])),
        "finite_motor_and_wrench_caps_held": caps_ok, "original_hard_guard_failure_ticks": np.flatnonzero(~feedback["all_hard_guards_held"]).tolist(), "every_step_objects_unforced": bool(np.all(feedback["external_drive_zero"]) and np.all(table["external_drive_zero"])), "maximum_unexpected_native_contacts": int(np.max(table["unexpected_native_contact_count"])), "maximum_bolt_world_support_contacts": int(np.max(feedback["bolt_world_support_contact_count"])), "maximum_nonthread_bolt_block_contact_count": int(np.max(feedback["nonthread_block_bolt_contact_count"])), "recorded_collision_exclusions": excludes,
        "direction_confirmation_ticks": int(np.sum(crest_ready)), "forward_native_steps": int(np.sum(feedback["phase_index"]==3)), "fully_open_native_steps": int(np.sum(feedback["fully_open_unassisted"])), "maximum_formed_flank_overlap_m": float(np.max(table["formed_flank_overlap_m"])), "maximum_loaded_actual_interior_contacts": int(np.max(feedback["loaded_actual_interior_flank_contact_count"])),
        "closed_braking_boundary": boundary, "stop_boundary_radial_secant_speed_m_per_s": float((feedback["radial_offset_m"][last]-feedback["radial_offset_m"][first_request])/(time[last]-time[first_request])), "max_stop_radial_finite_difference_speed_m_per_s": float(np.max(np.diff(feedback["radial_offset_m"][np.r_[first_request,stop]])/dt)),
        "timing": {"native_original_forces_and_geometry": "post-step label minus dt", "saved_qpos_qvel": "post-step label", "command_calibration": "previous retained native solve at label minus 2dt; first command uses initialized mj_forward pose at time zero (generic numeric logging label is minus dt)", "source_clock_not_retimed": True},
        "scientific_scope": "The moving unloaded 50um crest return only requests closed braking. No stopped direction event, forward action, interior engagement, opening, capture or policy qualification occurred. Final radial failure remains150.848um>150um; abrupt desired angular speed to zero followed by finite torque/force saturation is an observed control transition, not proven sole causality."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.run)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"output":str(args.output),"original_passed":result["original_passed"],"original_native_steps":result["original_native_steps"],"direction_confirmation_ticks":result["direction_confirmation_ticks"]}))
