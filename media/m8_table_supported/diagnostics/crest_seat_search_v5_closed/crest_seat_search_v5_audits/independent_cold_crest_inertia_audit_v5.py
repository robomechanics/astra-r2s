"""Read original cold C2/hybrid-inertia V5 ledgers; never import live app or solve/integrate physics."""
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
from independent_inertia_command_math_v5 import verify_inertia_ledger

VERSION = "independent-cold-crest-c2-inertia-native-reader-v5"
HARN = "a84682f45585e68d599d09c0415de4dc16ebce89859e7ad80edaf27df6229c95"
OBS = "1d37ef46382c349e29b4206206e9aac2132587b65abc39467342e7e1b0560c0c"
HELP = "ffe021f74b879b91e961509d1b2b0071b7aadf8ff64b1329bf179d8f33758f51"
BRAKE = "4bc45250cbf1aba271dad51df2781971e323735d631349afcc20ab311afa21c1"


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


def analytic_brake(theta0, omega0, alpha0, duration, elapsed):
    """Independent power-basis polynomial; no producer helper is executed."""
    require(np.isfinite([theta0,omega0,alpha0,duration]).all() and duration>0, "Invalid independent brake jet")
    elapsed=np.asarray(elapsed, dtype=float)
    require(np.isfinite(elapsed).all(), "Invalid brake time")
    s=np.clip(elapsed/duration,0.,1.)
    a=duration*alpha0
    v=np.array([omega0,a,-3*omega0-2*a,2*omega0+a])
    p=np.r_[theta0,duration*v/np.arange(1,5)]
    omega=np.polynomial.polynomial.polyval(s,v)
    alpha=np.polynomial.polynomial.polyval(s,np.polynomial.polynomial.polyder(v))/duration
    jerk=np.polynomial.polynomial.polyval(s,np.polynomial.polynomial.polyder(v,2))/duration**2
    stationary=elapsed>=duration
    return np.array([np.polynomial.polynomial.polyval(s,p),np.where(stationary,0.,omega),np.where(stationary,0.,alpha),np.where(stationary,0.,jerk)])


def brake_extrema(theta0, omega0, alpha0, duration):
    a=duration*alpha0
    v=np.array([omega0,a,-3*omega0-2*a,2*omega0+a])
    def points(coeff):
        roots=np.polynomial.polynomial.polyroots(coeff)
        return np.r_[0.,1.,[r.real for r in roots if abs(r.imag)<1e-12 and 0<r.real<1]]
    vp=np.polynomial.polynomial.polyder(v)
    vpp=np.polynomial.polynomial.polyder(v,2)
    speeds=np.polynomial.polynomial.polyval(points(vp),v)
    acc=np.polynomial.polynomial.polyval(points(vpp),vp)/duration
    jerks=np.polynomial.polynomial.polyval([0.,1.],vpp)/duration**2
    return {"peak_speed_rad_s":float(np.max(np.abs(speeds))), "peak_acceleration_rad_s2":float(np.max(np.abs(acc))), "peak_left_limit_jerk_rad_s3":float(np.max(np.abs(jerks))), "maximum_signed_reverse_speed_rad_s":float(np.max(speeds)), "complete_curve_endpoint_rad":float(theta0+duration*(omega0/2+a/12))}


def verify_command_curve(feedback, labels, metadata, dt):
    expected=np.zeros((4,len(feedback["time"])))
    expected[0]=metadata["cold_initialization"]["original_independent_clock_anchor_rad"]
    reverse=np.flatnonzero(feedback["phase_index"]==labels.index("reverse_seat_1"))
    brake=np.flatnonzero(feedback["phase_index"]==labels.index("stop_reverse_seat_1"))
    seed=expected[0,0]
    finish=-metadata["control_config"]["maximum_reverse_angle_rad"]
    duration=1.875*abs(finish-seed)/metadata["control_config"]["reverse_angular_speed_rad_s"]
    total_steps=round(duration/dt)
    u=np.arange(1,len(reverse)+1)/total_steps
    delta=finish-seed
    expected[:,reverse]=np.array([seed+delta*(10*u**3-15*u**4+6*u**5), delta*(30*u**2-60*u**3+30*u**4)/duration,delta*(60*u-180*u**2+120*u**3)/duration**2,delta*(60-360*u+360*u**2)/duration**3])
    fields=("desired_independent_clock_rad","desired_independent_angular_speed_rad_s","desired_independent_angular_acceleration_rad_s2","desired_independent_angular_jerk_rad_s3")
    recorded=np.array([feedback[k] for k in fields])
    event=next(e for e in metadata["physical_motion_events"] if e["event"]=="Measured crest return requests C2 CLOSED robot-yaw braking")
    require(feedback["time"][reverse[-1]]==event["time_s"], "Brake request must use actual last reverse tick")
    require(np.allclose(expected[:3,reverse[-1]], [event["initial_theta_rad"],event["initial_omega_rad_s"],event["initial_alpha_rad_s2"]],rtol=1e-11,atol=1e-12), "Brake seed does not match independent preceding scheduled jet")
    T=event["duration_s"]
    expected[:,brake]=analytic_brake(*expected[:3,reverse[-1]],T,np.arange(1,len(brake)+1)*dt)
    forward=np.flatnonzero(feedback["phase_index"]==labels.index("start_thread_1"))
    forward_stop=np.flatnonzero(feedback["phase_index"]==labels.index("stop_start_1"))
    seed_forward=float(analytic_brake(*expected[:3,reverse[-1]],T,np.array([T]))[0,0])
    target_forward=metadata["control_config"]["first_forward_clock_rad"]
    forward_duration=1.875*abs(target_forward-seed_forward)/(metadata["control_config"]["starting_angular_speed_rad_s"] or metadata["control_config"]["arm"]["angular_speed_rad_s"])
    forward_steps=round(forward_duration/dt)
    u=np.arange(1,len(forward)+1)/forward_steps
    d=target_forward-seed_forward
    expected[:,forward]=np.array([seed_forward+d*(10*u**3-15*u**4+6*u**5),d*(30*u**2-60*u**3+30*u**4)/forward_duration,d*(60*u-180*u**2+120*u**3)/forward_duration**2,d*(60-360*u+360*u*u)/forward_duration**3])
    expected[0,forward_stop]=target_forward
    require(np.allclose(expected,recorded,rtol=1e-10,atol=1e-11), "Actual original clock derivative arrays differ from independent schedule")
    flag=np.zeros(len(feedback["time"]),dtype=bool)
    flag[brake]=np.arange(1,len(brake)+1)*dt<T-1e-12
    require(np.array_equal(flag,feedback["robot_yaw_brake_active"]), "Brake-active timing changed")
    ext=brake_extrema(*expected[:3,reverse[-1]],T)
    require(ext["maximum_signed_reverse_speed_rad_s"]<=1e-12 and ext["peak_speed_rad_s"]<=2+1e-12 and -2.7<=ext["complete_curve_endpoint_rad"]<=event["initial_theta_rad"]<=0, "Complete analytic curve exceeded independent workspace/speed bounds")
    require(abs(ext["complete_curve_endpoint_rad"]-event["terminal_theta_rad"])<1e-12, "Declared analytic endpoint differs")
    return {"original_command_rows_checked":len(feedback["time"]),"forward_rows_checked":len(forward),"forward_declared_duration_s":forward_duration,"forward_integer_grid_duration_s":forward_steps*dt,"executed_brake_rows":len(brake),"executed_brake_duration_s":len(brake)*dt,"planned_brake_duration_s":T,"completed_brake":bool(len(brake)*dt>=T-1e-12),"every_recorded_clock_derivative_and_active_flag_matches":True,"maximum_absolute_derivative_residuals":[float(x) for x in np.max(np.abs(expected-recorded),axis=1)],"last_reverse_jet":recorded[:,reverse[-1]].tolist(),"first_brake_jet":recorded[:,brake[0]].tolist(),"final_executed_brake_jet":recorded[:,brake[-1]].tolist(),"complete_analytic_curve":ext,"continuity_scope":"C2 theta/omega/alpha at virtual brake start; first executed command samplesdt. Endpoint source clampsomega/alpha/jerk to zero at/afterT; no C3 continuity claim.","preceding_quintic_step_rounding":{"declared_duration_s":duration,"rounded_steps":total_steps,"rounded_grid_duration_s":total_steps*dt,"scope":"Original normalized progress uses integer step/rounded_steps; derivative denominators use declared duration. Exact source convention retained; tiny rounding difference is not silently retimed."}}


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
    for k in ("fully_open_unassisted", "right_grasp_guard_active", "right_axial_float_active", "weight_window_ready", "open_weight_window_ready", "all_hard_guards_held", "external_drive_zero", "weight_observation_valid", "open_observation_valid", "cold_table_window_warmup", "robot_yaw_brake_active"):
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
    for k in ("desired_independent_clock_rad", "desired_independent_angular_speed_rad_s", "relative_bolt_axial_velocity_m_per_s", "relative_bolt_angular_speed_rad_per_s", "relative_hand_angular_speed_rad_per_s", "radial_offset_m", "bolt_tilt_rad", "thread_gravity_opposing_force_N", "hand_gravity_opposing_force_N", "thread_summed_normal_force_N", "right_grip_slip_m", "right_grip_rotation_slip_rad", "all_hard_guards_held", "weight_window_ready", "desired_independent_angular_acceleration_rad_s2", "desired_independent_angular_jerk_rad_s3", "robot_yaw_brake_active"):
        result[k] = feedback[k][i].item()
    result["bolt_base_insertion_m"] = float(table["bolt_base_insertion_m"][i])
    for side in ("left", "right"):
        w = feedback[side+"_command_wrench_N_Nm"][i]
        result[side+"_command_force_norm_N"] = float(np.linalg.norm(w[:3]))
        result[side+"_command_torque_norm_Nm"] = float(np.linalg.norm(w[3:]))
    return result


def first_eligible_tick(ids, ready, minimum_dwell_s, dt):
    eligible=ids[ready[ids] & ((np.arange(len(ids))+1)*dt>=minimum_dwell_s-1e-12)]
    require(len(eligible)>0, "No original eligible phase tick")
    return int(eligible[0])


def audit(run):
    run = Path(run).resolve()
    outer_path = run.with_name(run.name+"_execution_after.json")
    outer = json.loads(outer_path.read_text())
    before_path=run.with_name(run.name+"_execution_before.json")
    before=json.loads(before_path.read_text())
    require(outer["native_exit_code"] == 0 and outer["all_bindings_unchanged"] is True, "V5 native closure scope differs")
    require(outer["execution_before_sha256"]==digest(before_path), "Outer before identity differs")
    require(digest(run.with_name(run.name+"_frozen_inputs.json"))==before["frozen_inputs_manifest_sha256"]==outer["frozen_inputs_manifest_sha256_after"], "Frozen input manifest differs")
    for initial, final in (("producer_source_65_before","source_65_after"),("source_files_before","source_files_after"),("parent_input_sha256_before","parent_input_sha256_after"),("runtime_file_sha256_before","runtime_file_sha256_after")):
        require(before[initial]==outer[final], "Immutable source/parent/runtime binding changed: "+initial)
    for relative,expected in before["producer_source_65_before"].items():
        require(digest(run.parents[3]/relative)==expected,"Actual frozen producer source changed: "+relative)
    for relative,expected in before["source_files_before"].items():
        require(digest(run.parent/relative)==expected,"Actual frozen experimental source/proof changed: "+relative)
    for absolute,expected in before["runtime_file_sha256_before"].items():
        require(digest(absolute)==expected,"Actual native runtime bytes changed")
    require(digest(run/"insertion_validation.json")==outer["original_validation_sha256"], "Closed original validation changed")
    metadata = json.loads((run/"insertion_validation.json").read_text())
    require(metadata["physical_feedback_controller"] == "cold-crest-return-c2-hybrid-inertia-diagnostic-v5" and metadata["diagnostic_only"] is True, "Wrong branch scope")
    require(bundle_identity(run/"controller_source.py") == metadata["controller_sha256"], "Selected source identity differs")
    require(digest(run/"robot_yaw_brake_source.py")==BRAKE and digest(run/"recorded_sources/yam_twin/reverse_brake_v4.py")==BRAKE, "Wrong archived C2 helper")
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
    require(metadata["native_feedback_force_history"]["columns"] == list(feedback) and len(feedback) == 50, "Wrong declared cold schema")
    validate_shapes(feedback)
    dt = metadata["scene_config"]["base"]["thread"]["timestep"]
    command_curve=verify_command_curve(feedback,labels,metadata,dt)
    require(np.array_equal(feedback["thread_gravity_opposing_force_N"], feedback["thread_wrench_on_bolt_world_N_Nm"][:,2]) and np.array_equal(feedback["hand_gravity_opposing_force_N"], feedback["hand_wrench_on_bolt_world_N_Nm"][:,2]), "Gravity-opposing load changed from native world wrench")
    time = feedback["time"]
    require(np.allclose(np.diff(time), dt, rtol=1e-7, atol=1e-10), "Native history is not contiguous")
    require(abs(time[0]-dt)<1e-12 and metadata["aborted"] is None and time[-1] == time_saved[-1] and abs(time[-1]-sum(x["actual_executed_duration_s"] for x in metadata["phases"]))<1e-9, "Wrong original cold endpoints")
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
    first_confirmation = None
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
            seat_report = crest.observe(float(t), dt, table["bolt_base_insertion_m"][i], feedback["relative_bolt_axial_velocity_m_per_s"][i], feedback["thread_summed_normal_force_N"][i], actual_relative_angular_speed_rad_per_s=max(feedback["relative_bolt_angular_speed_rad_per_s"][i], feedback["relative_hand_angular_speed_rad_per_s"][i]), valid=seat_valid, physically_stopped=(label == "stop_reverse_seat_1" and not feedback["robot_yaw_brake_active"][i]))
            crest_ready[i] = crest.ready
            if first_confirmation is None and crest.ready:
                first_confirmation=i
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
            require(i == first_eligible_tick(ids,table_ready & weight_ready,.1,dt), "Cold hold did not consume first fresh eligible load window")
        elif event["phase"] == "reverse_seat_1":
            require(i == first_request and equal(event["seat_direction_event"], event_reports[i]), "Request not exact first native measured return")
        elif event["phase"] == "stop_reverse_seat_1":
            require(i==first_eligible_tick(ids,crest_ready & weight_ready,0.,dt), "Stopped phase did not consume first original direction/load-ready event")
            require(equal(event["seat_direction_event"], crest.report()), "Stopped direction event differs from final retained observer")
        elif event["phase"] == "stop_start_1":
            require(i==first_eligible_tick(ids,weight_ready,.12,dt), "Forward stop did not consume first original eligible load-ready event")
        else:
            raise ValueError("Unexpected live readiness event")
        require(equal(event["weight_window"],saved_indices[i]["weight_window"]), "Event native weight window differs")
        events.append(snapshot(i, feedback, table, labels, dt))
    tree = ET.parse(run/"scene.xml")
    excludes = [dict(e.attrib) for e in tree.findall("./contact/exclude")]
    require(not any("bolt" in str(e) or "block" in str(e) for e in excludes), "Workpiece collision disabled by exclusion")
    caps_ok = True
    brake_cap_observations={}
    for side in ("left", "right"):
        w = feedback[side+"_command_wrench_N_Nm"]
        bounds = np.array([np.fromstring(tree.find("./actuator/motor[@name='"+side+"_servo"+str(j)+"']").attrib["ctrlrange"], sep=" ") for j in range(1, 7)])
        motor = feedback[side+"_motor_torques_Nm"]
        caps_ok &= bool(np.all(np.linalg.norm(w[:, :3], axis=1)<=8+1e-12) and np.all(np.linalg.norm(w[:, 3:], axis=1)<=2+1e-12) and np.all(motor>=bounds[:, 0]-1e-12) and np.all(motor<=bounds[:, 1]+1e-12))
        mask=feedback["phase_index"]==labels.index("stop_reverse_seat_1")
        brake_cap_observations[side]={"maximum_original_force_norm_N":float(np.max(np.linalg.norm(w[mask,:3],axis=1))),"maximum_original_torque_norm_Nm":float(np.max(np.linalg.norm(w[mask,3:],axis=1))),"maximum_original_absolute_joint_torques_Nm":np.max(np.abs(motor[mask]),axis=0).tolist(),"native_motor_caps_Nm":bounds[:,1].tolist(),"cartesian_force_saturated_ticks":int(np.sum(np.linalg.norm(w[mask,:3],axis=1)>=8.-1e-12)),"cartesian_torque_saturated_ticks":int(np.sum(np.linalg.norm(w[mask,3:],axis=1)>=2.-1e-12)),"any_native_motor_saturated_ticks":int(np.sum(np.any(np.abs(motor[mask])>=bounds[:,1]-1e-12,axis=1)))}
    require(caps_ok, "Original finite motor/wrench caps violated")
    last = len(time)-1
    stop = np.flatnonzero(feedback["phase_index"] == 2)
    require(len(stop) and stop[0] == first_request+1, "Next closed stop command did not start next native tick")
    boundary = [snapshot(j, feedback, table, labels, dt) for j in (first_request, stop[0], min(stop[0]+19,last), last)]
    require(np.all(feedback["all_hard_guards_held"]), "Original diagnostic hard guard failure occurred")
    require(np.all(feedback["radial_offset_m"]<=150e-6), "Original alignment limit violated")
    require(np.all(feedback["right_grasp_guard_active"]) and np.all(feedback["right_axial_float_active"]) and not np.any(feedback["fully_open_unassisted"]), "Cold held-grasp modes differ from actual phases")
    pads = left["pad_normal_force_N"]
    require(np.all(pads>.1) and np.all(feedback["right_pad_normal_force_N"]>.1),"Original bilateral native pad retention failed")
    require(np.all(feedback["right_grip_slip_m"]<=.001) and np.all(feedback["left_grip_slip_m"]<=.001) and np.all(feedback["right_grip_rotation_slip_rad"]<=np.deg2rad(2)) and np.all(feedback["left_grip_rotation_slip_rad"]<=np.deg2rad(2)),"Original fixed grasp slip limits failed")
    require(np.all(table["bolt_tip_table_clearance_m"]>=.001),"Original male/table tip clearance failed")
    recorded_pads = metadata["all_substep_left_pad_loads"]
    require(recorded_pads["observed_physics_steps"] == len(time) and recorded_pads["minimum_normal_force_N"] == np.min(pads,axis=0).tolist() and recorded_pads["unloaded_physics_steps"] == np.sum(pads<=.1,axis=0).tolist(), "Original native bilateral statistics differ")
    forward=np.flatnonzero(feedback["phase_index"]==labels.index("start_thread_1"))
    baseline=forward[0]-1
    advance=float(table["bolt_base_insertion_m"][forward[-1]]-table["bolt_base_insertion_m"][baseline])
    angle=float(table["bolt_yaw_unwrapped_rad"][forward[-1]]-table["bolt_yaw_unwrapped_rad"][baseline])
    pitch=float(metadata["scene_config"]["base"]["thread"]["pitch"])
    summary=next(x for x in metadata["phases"] if x["phase"]=="start_thread_1")
    require(np.isclose(advance,summary["bolt_insertion_advance_m"],rtol=0.,atol=1e-12) and np.isclose(angle,summary["bolt_clockwise_rotation_rad"],rtol=0.,atol=1e-12),"Raw physical forward phase displacement differs from original summary")
    final_window_ids=np.flatnonzero(time>=time[-1]-.1-1e-12)
    quiet_extrema={k:float(np.max(np.abs(feedback[k][final_window_ids]))) for k in ("relative_bolt_axial_velocity_m_per_s","relative_bolt_angular_speed_rad_per_s","relative_hand_angular_speed_rad_per_s")}
    return {"observer": VERSION, "reader_source_sha256": digest(__file__), "method": "Pure original-array arithmetic and exact archived pure observer AST replay; zero model integration or force recomputation", "original_passed": metadata["passed"], "diagnostic_completed": metadata["diagnostic_completed"], "full_fresh_trajectory_qualified": False, "capture_or_open_reset_qualified": False,
        "closed_original_exit_code": outer["native_exit_code"], "original_aborted": metadata["aborted"],
        "source_and_parent_identity_verified": True, "selected_source_bundle_verified": True,"independent_analytic_command_curve":command_curve,
        "original_native_steps": len(time), "saved_post_state_samples": len(rows), "original_contact_frame_checks": force_checks,
        "right_local_force_frame_scope": "Right hand world wrench and pad normals are original native values matched between samples and all-step arrays; individual right pad local/frame records were not archived, so no independent right-local-frame reconstruction is claimed. Original source bundle includes its signed contact-force function; thread, table and left original local-frame sums are independently reconstructed.",
        "inherited_acceptance_scope": "Original producer labels are preserved verbatim. Its inherited pre-pickup/stabilization baseline is not fresh cold acquisition/pickup evidence; this reader separately verifies fresh local100ms windows and inherited fixed grasp references. Canonical402 tests and e468 reader are not coverage of this new C2/inertia diagnostic tag.",
        "every_step_observer_and_saved_event_consistency": True, "first_fresh_support_event": events[0], "first_moving_closed_stop_request": events[1], "first_request_crest_report": event_reports[first_request], "final_original_crest_report": crest.report(),
        "cold_table_unavailable_steps": int(np.sum(warmup)), "raw_original_table_failed_window_steps": int(np.sum(~table_ready)), "complete_fresh_table_failed_windows": int(np.sum(~table_ready & ~warmup)), "raw_original_table_passed_label": metadata["acceptance_checks"]["table_bears_weight_throughout_active_task"]["passed"],
        "minimum_table_upward_force_N": float(np.min(table["table_wrench_world_at_block_origin_N_Nm"][:,2])), "final_table_upward_force_N": float(table["table_wrench_world_at_block_origin_N_Nm"][-1,2]), "final_left_signed_upward_force_N": float(table["left_hand_wrench_world_at_block_origin_N_Nm"][-1,2]), "maximum_positive_left_upward_force_N": float(np.max(np.maximum(table["left_hand_wrench_world_at_block_origin_N_Nm"][:,2],0))), "minimum_left_pad_normals_N": np.min(left["pad_normal_force_N"],axis=0).tolist(), "minimum_right_pad_normals_N": np.min(feedback["right_pad_normal_force_N"],axis=0).tolist(), "maximum_right_grip_slip_m": float(np.max(feedback["right_grip_slip_m"])), "maximum_right_rotation_slip_rad": float(np.max(feedback["right_grip_rotation_slip_rad"])), "maximum_left_grip_slip_m": float(np.max(feedback["left_grip_slip_m"])), "maximum_left_rotation_slip_rad": float(np.max(feedback["left_grip_rotation_slip_rad"])),
        "finite_motor_and_wrench_caps_held": caps_ok,"original_brake_cap_observations":brake_cap_observations, "original_hard_guard_failure_ticks": np.flatnonzero(~feedback["all_hard_guards_held"]).tolist(), "every_step_objects_unforced": bool(np.all(feedback["external_drive_zero"]) and np.all(table["external_drive_zero"])), "maximum_unexpected_native_contacts": int(np.max(table["unexpected_native_contact_count"])), "maximum_bolt_world_support_contacts": int(np.max(feedback["bolt_world_support_contact_count"])), "maximum_nonthread_bolt_block_contact_count": int(np.max(feedback["nonthread_block_bolt_contact_count"])), "recorded_collision_exclusions": excludes,
        "direction_confirmation_ticks": int(np.sum(crest_ready)), "forward_native_steps": int(np.sum(feedback["phase_index"]==3)), "fully_open_native_steps": int(np.sum(feedback["fully_open_unassisted"])), "maximum_formed_flank_overlap_m": float(np.max(table["formed_flank_overlap_m"])), "maximum_loaded_actual_interior_contacts": int(np.max(feedback["loaded_actual_interior_flank_contact_count"])),
        "closed_braking_boundary": boundary, "stop_boundary_radial_secant_speed_m_per_s": float((feedback["radial_offset_m"][last]-feedback["radial_offset_m"][first_request])/(time[last]-time[first_request])), "max_stop_radial_finite_difference_speed_m_per_s": float(np.max(np.diff(feedback["radial_offset_m"][np.r_[first_request,stop]])/dt)),
        "timing": {"native_original_forces_and_geometry": "post-step label minus dt", "saved_qpos_qvel": "post-step label", "command_calibration": "previous retained native solve at label minus 2dt; first command uses initialized mj_forward pose at time zero (generic numeric logging label is minus dt)", "source_clock_not_retimed": True},
        "final_native_weight_window": weight_win.report(), "final_active_table_window":table_win.report(),
        "first_direction_confirmation_native_index":first_confirmation,
        "partial_forward_motion_measurement":{"original_force_baseline_time_s":float(time[baseline]-dt),"original_force_end_time_s":float(time[forward[-1]]-dt),"actual_advance_m":advance,"actual_relative_yaw_rad":angle,"metric_pitch_reference_m":pitch,"pitch_reference_residual_m":advance-pitch*angle/(2*np.pi),"scope":"Original native pre-integration phase baseline matches original producer summary. Shallow partial forward travel does not independently qualify a metric-lead turn, full-pitch capture or reset."},
        "final_original_100ms_quiet_extrema":quiet_extrema,
        "source_original_acceptance_checks":metadata["acceptance_checks"],
        "independent_inertia_commands":verify_inertia_ledger(run, metadata, feedback, labels, table, time_saved, qvel, controls, rows, dt, declaration),
        "scientific_scope": "The moving measured crest return requested CLOSED C2 braking. The executed stop subsequently met the original actual100ms native impulse/angular/axial/90-10 load criteria, allowing the actually executed bounded forward scan. Every original hard guard held. Nominal formed overlap remains approximately21.6um, actual conservative interior contact count zero, no opening/full-pitch capture/reset or full fresh trajectory qualification. Original failed/unexecuted inherited acceptance rows are preserved. Independent arithmetic validates original recorded robot commands, not a full contact inverse dynamics model or sole-cause tracking proof."}



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.run)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    print(json.dumps({"output":str(args.output),"original_passed":result["original_passed"],"original_native_steps":result["original_native_steps"],"direction_confirmation_ticks":result["direction_confirmation_ticks"]}))
