"""Prepare an output-only, source-derived cold diagnostic; does not integrate."""
from pathlib import Path
import hashlib
import json

ROOT = Path(__file__).resolve().parents[3]
SOURCE = ROOT / 'yam_twin/m8_supported_simulation.py'
TARGET = Path(__file__).with_name('crest_seat_search_probe_v3.py')
EXPECTED = '57f430143787317f8dc6f50bd2ea1ff23aab123810284da74da70b53b45a9493'
raw = SOURCE.read_bytes()
assert hashlib.sha256(raw).hexdigest() == EXPECTED
source = raw.decode()
changes = []


def replace_once(old, new, purpose):
    global source
    assert source.count(old) == 1, (purpose, source.count(old))
    source = source.replace(old, new)
    changes.append({'purpose': purpose, 'original': old, 'diagnostic': new})


replace_once('from __future__ import annotations\n', '''from __future__ import annotations
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
__package__ = "yam_twin"
from crest_seat_observer_v3 import CrestSeatDropWindowV3
_COLD_CONTEXT = None

def cold_table_mean_unavailable(label, observed_duration_s, required_duration_s):
    if (not np.isfinite((observed_duration_s, required_duration_s)).all()
            or observed_duration_s < 0 or required_duration_s <= 0):
        raise ValueError("Cold-window durations must be finite and physical")
    return bool(label == "cold_window_hold"
                and observed_duration_s+1e-12 < required_duration_s)
''', 'Standalone ignored diagnostic imports; frozen producer unchanged')
replace_once('    model = build_model(scene)\n',
             '    model = _COLD_CONTEXT["model"]\n',
             'Use exact archived XML/assets/model, not a scene reconstruction')
replace_once('    bolt_id = model.body("male_bolt").id\n', '''    # Declared cold initialization only.  Native contact warm-start/history is
    # deliberately absent; no later workpiece state write is introduced.
    original_block_id = model.body("fixture_block").id
    original_block_p0 = data.xpos[original_block_id].copy()
    original_block_r0 = data.xmat[original_block_id].reshape(3, 3).copy()
    original_hole_r0 = data.xmat[model.body("female_frame").id].reshape(3, 3).copy()
    data.qpos[:] = _COLD_CONTEXT["qpos"]
    data.qvel[:] = _COLD_CONTEXT["qvel"]
    data.ctrl[:] = _COLD_CONTEXT["ctrl"]
    data.time = 0.
    mujoco.mj_forward(model, data)
    bolt_id = model.body("male_bolt").id
''', 'Exact selected original post-integration qpos/qvel/ctrl written once at cold initialization')
replace_once('    phases = supported_phases(control)\n', '''    phases = [("cold_window_hold", .5, 0., 0., control.arm.closed_aperture,
               control.arm.closed_aperture, True)] + [p for p in supported_phases(control) if p[0] in {
        "reverse_seat_1", "stop_reverse_seat_1", "start_thread_1", "stop_start_1"}]
''', 'Bounded reverse, strict stopped observation, first closed forward, final strict stop only')
replace_once('            native_weight_transfer_sample, BoltWeightTransferWindow, ImpulseSeatDropWindow,\n',
             '            native_weight_transfer_sample, BoltWeightTransferWindow, ImpulseSeatDropWindow, CrestSeatDropWindowV3, cold_table_mean_unavailable,\n',
             'Bind executed experimental observer in selected source bundle')
replace_once('                  ArmIK):\n', '                  ArmIK, CrestSeatDropWindowV3):\n',
             'Archive whole executed experimental observer alongside frozen dependencies')
replace_once('    for phase_index, (label, duration, angle0, angle1, gap0, gap1, axial_float) in enumerate(selected):\n', '''    # Preserve the original actually-acquired per-grasp references. Target
    # recalibration never rebases these cumulative physical slip guards.
    left_grip_p = np.asarray(_COLD_CONTEXT["left_acquisition"]["grasp_relative_block_position_m"]).copy()
    left_grip_r = np.asarray(_COLD_CONTEXT["left_acquisition"]["grasp_relative_block_rotation"]).copy()
    left_grasp_acquired = True
    left_acquisition_time = 0.
    grip_reference = (np.asarray(_COLD_CONTEXT["right_acquisition"]["grasp_relative_bolt_head_position_m"]).copy(),
                      np.asarray(_COLD_CONTEXT["right_acquisition"]["grasp_relative_bolt_rotation"]).copy())
    metadata["left_acquisition"] = _COLD_CONTEXT["left_acquisition"]
    metadata["right_grasp_acquisitions"] = [_COLD_CONTEXT["right_acquisition"]]
    block_p0, block_r0 = original_block_p0, original_block_r0
    hand_relative_r = original_hole_r0.T @ right_initial_r
    desired_clock = closed_clock_anchor = _COLD_CONTEXT["desired_clock_rad"]
    closed_yaw_anchor = _COLD_CONTEXT["closed_yaw_anchor_rad"]
    seat_event = CrestSeatDropWindowV3(_COLD_CONTEXT["settled_base_z_m"], bolt_weight)
    metadata["diagnostic_only"] = True
    metadata["cold_initialization"] = _COLD_CONTEXT["declaration"]
    metadata["experimental_crest_observer_source_sha256"] = _COLD_CONTEXT["observer_sha256"]
    metadata["inherited_reference_scope"] = "Original full-run measured left/right acquisition references are retained verbatim for cumulative grip guards. They are not new cold-branch acquisition evidence. Fresh native pad/table/force/motion windows begin with this branch; no old force window or solver warm-start is reused."
    metadata["starts_preengaged"] = False
    metadata["starts_grasp_ready"] = True
    metadata["block_starts_left_touching"] = True
    metadata["bolt_starts_on_declared_fixed_rest"] = False
    metadata["physical_feedback_controller"] = "cold-crest-return-diagnostic-v3"
    metadata["seat_direction_stop_scope"] = "Measured50um return from a valid contiguous actual bolt/female shallow crest requests CLOSED deceleration only. Persistent return, actual max(body,hand) angular quiet, axial slow motion and original EntrySupportWindow impulse/duty/current-load proof over100ms PLUS the unchanged live100ms90/10 native weight-transfer window are required before any forward scan. No capture or jaw opening occurs."
    metadata["cold_window_hold_scope"] = "First actual100ms trailing force mean is undefined after cold initialization: this explicitly unqualified warm-up retains all instantaneous table/pad/grip/collision/depth/limit/drive/cap guards. After complete fresh table-window coverage the unchanged original rolling90/10/99 guard applies every tick. Reverse cannot start before fresh table, bolt-weight90/10, and actual body/tool angular/axial quiet readiness. No original native force or window is reused."
    for phase_index, (label, duration, angle0, angle1, gap0, gap1, axial_float) in enumerate(selected):
''', 'Cold lineage, original cumulative references, original independent yaw clock, fresh observer windows')
replace_once('        feedback_closed = label in {"transfer_bolt_weight", "reverse_seat_1", "stop_reverse_seat_1"}',
             '        feedback_closed = label in {"cold_window_hold", "transfer_bolt_weight", "reverse_seat_1", "stop_reverse_seat_1"}',
             'Warm-up keeps the same closed finite force/pose controller and continuous left press')
replace_once('                task_table_failed_windows += int(not task_table_window.ready)\n',
             '                task_table_failed_windows += int(not task_table_window.ready)\n',
             'Keep unqualified warm-up mean windows visible in legacy full-task diagnostics')
replace_once('            if (thread_depth > 10e-6 or left_slip > .001 or left_angle > np.deg2rad(2)\n', '''            cold_table_window_warmup = cold_table_mean_unavailable(label,
                task_table_window.elapsed, control.settled_table_window_s)
            if (thread_depth > 10e-6 or left_slip > .001 or left_angle > np.deg2rad(2)
''', 'Explicit availability flag for initial fresh cold trailing100ms window; instantaneous guards still apply')
replace_once('or (left_grasp_acquired and (not table_bearing or not task_table_window.ready\n',
             'or (left_grasp_acquired and (not table_bearing or (not task_table_window.ready and not cold_table_window_warmup)\n',
             'Undefined initial cold mean only; after100ms original rolling ready predicate restored without changed values')
replace_once('            if label == "transfer_bolt_weight":\n                after_ramp =', '''            if label == "cold_window_hold":
                gate = bounded_phase_gate(elapsed_phase, control.settled_table_window_s,
                    duration, task_table_window.ready and weight_window.ready and quiet,
                    valid=not aborted)
            elif label == "transfer_bolt_weight":
                after_ramp =''', 'Fresh native table and bolt weight/actual motion window gate before any reverse motion')
replace_once('                ready = seat_event.stop_requested if label == "reverse_seat_1" else seat_event.ready\n', '''                ready = (seat_event.stop_requested if label == "reverse_seat_1"
                         else seat_event.ready and weight_window.ready and quiet)
''', 'Stopped direction additionally requires live original 100ms 90/10 weight and both angular/axial quiet')
replace_once('                grip_slip, right_grip_rotation_slip, left_slip, left_angle))\n',
             '                grip_slip, right_grip_rotation_slip, left_slip, left_angle, cold_table_window_warmup))\n',
             'Archive unavailable initial trailing mean as a separate original all-step scalar')
replace_once('"right_grip_rotation_slip_rad", "left_grip_slip_m", "left_grip_rotation_slip_rad")\n',
             '"right_grip_rotation_slip_rad", "left_grip_slip_m", "left_grip_rotation_slip_rad", "cold_table_window_warmup")\n',
             'Add corresponding raw native feedback ledger column')
replace_once('"all_hard_guards_held": hard_guards_held,\n',
             '"all_hard_guards_held": hard_guards_held, "cold_table_window_warmup": cold_table_window_warmup,\n',
             'Sample availability flag; missing rolling coverage is explicitly unqualified')
replace_once('    serialized_result = json.dumps(result, allow_nan=False)\n', '''    result["passed"] = False
    result["partial"] = True
    result["diagnostic_completed"] = bool(aborted is None and len(summaries) == 5
        and summaries[-1]["phase"] == "stop_start_1"
        and summaries[-1]["live_physical_phase_gate"] == "complete"
        and entry_support.ready and weight_window.ready)
    result["diagnostic_final_native_entry_support"] = entry_support.report()
    result["diagnostic_final_native_weight_support"] = weight_window.report()
    result["full_fresh_trajectory_qualified"] = False
    result["capture_or_open_reset_qualified"] = False
    result["final_experimental_crest_direction_event"] = seat_event.report()
    result["diagnostic_acceptance_scope"] = "Only this new cold branch's observed native guards, closed direction/first-forward motion and original solved forces. Legacy whole-task acceptance rows remain explicitly incomplete; inherited acquisition references are not fresh prefix evidence. No captured thread/open reset/policy/full trajectory qualification."
    serialized_result = json.dumps(result, allow_nan=False)
''', 'Truthful cold-only reporting; cannot become full captured/fresh success')

source += '''

def _sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    import argparse
    import subprocess
    from .m8_supported_scene import SupportedConfig, scene_xml
    from .m8_scene import YamM8Config
    from thread_lab.model import ThreadConfig
    from scripts.audit_m8_insertion_trace import recorded_model
    global _COLD_CONTEXT
    parser = argparse.ArgumentParser(description="Separate cold closed crest-return diagnostic; never a full rollout splice")
    parser.add_argument("--parent", type=Path, required=True)
    parser.add_argument("--checkpoint-time", type=float, default=13.1949)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prepare-only", action="store_true")
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError("Never overwrite a closed or active native diagnostic")
    report_path = args.parent.parent/"insertion_validation.json"
    original = json.loads(report_path.read_text())
    original_after_path = args.parent.parent/"run_publication_identity_after.json"
    original_after = json.loads(original_after_path.read_text())
    if (original_after["producer_commit"] != "6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1"
            or not original_after["source_hashes_unchanged"]
            or original_after["native_exit_code"] != 1
            or original["controller_module_sha256"] != "57f430143787317f8dc6f50bd2ea1ff23aab123810284da74da70b53b45a9493"):
        raise ValueError("Cold parent is not the immutable declared first continuous failure")
    root = Path(__file__).resolve().parents[3]
    before = {name:_sha(root/name) for name in original_after["source_hashes_before"]}
    if len(before) != 65 or before != original_after["source_hashes_before"]:
        raise ValueError("The original 65 frozen source hashes changed")
    observer_path = Path(inspect.getfile(CrestSeatDropWindowV3))
    source_sha, observer_sha = _sha(__file__), _sha(observer_path)
    with np.load(args.parent, allow_pickle=False) as archive:
        metadata = json.loads(str(archive["metadata_json"].item()))
        times = archive["time"]
        candidates = np.flatnonzero(np.abs(times-args.checkpoint_time) <= 1e-9)
        if len(candidates) != 1:
            raise ValueError("Cold checkpoint must be exactly one archived native post-integration row")
        index = int(candidates[0])
        native_time = float(times[index])
        qpos, qvel, ctrl = (archive[key][index].copy() for key in ("qpos", "qvel", "controller"))
    if metadata["controller_module_sha256"] != original["controller_module_sha256"]:
        raise ValueError("Parent trace and validation controller identity disagree")
    reference = next(event for event in original["physical_motion_events"]
                     if event["event"] == "Actual post-ramp settled seat reference")
    if abs(reference["time_s"]-native_time) > 1e-9 or reference["stable_actual_quiet_streak_s"] < .1-1e-12:
        raise ValueError("Chosen checkpoint is not the exact actually settled reference event")
    inputs = {name:_sha(args.parent.parent/name) for name in (
        "insertion_trace.npz", "insertion_validation.json", "scene.xml", "supported_scene.zip",
        "table_support_force_history.npz", "native_feedback_force_history.npz",
        "run_publication_identity.json", "run_publication_identity_after.json")}
    if inputs["scene.xml"] != original["model_xml_sha256"]:
        raise ValueError("Archived original scene XML does not match its native report")
    with np.load(args.parent.parent/"table_support_force_history.npz", allow_pickle=False) as history:
        jj = np.flatnonzero(np.abs(history["time"]-native_time) <= 1e-9)
        if len(jj) != 1 or abs(float(history["bolt_base_insertion_m"][jj[0]])-reference["base_z_m"]) > 1e-12:
            raise ValueError("Settled reference is not bound to its original native force/pose ledger")
    with np.load(args.parent.parent/"native_feedback_force_history.npz", allow_pickle=False) as history:
        jj = np.flatnonzero(np.abs(history["time"]-native_time) <= 1e-9)
        if len(jj) != 1 or not bool(history["all_hard_guards_held"][jj[0]]):
            raise ValueError("Settled reference lacks its original native hard-guard proof")
    def tuple_lists(value):
        if isinstance(value, list):
            return tuple(tuple_lists(v) for v in value)
        if isinstance(value, dict):
            return {k:tuple_lists(v) for k,v in value.items()}
        return value
    scene_values = dict(tuple_lists(original["scene_config"]))
    base_values = dict(scene_values.pop("base"))
    base_values["thread"] = ThreadConfig(**base_values["thread"])
    scene = SupportedConfig(base=YamM8Config(**base_values), **scene_values)
    control_values = dict(original["control_config"])
    control_values["arm"] = YamM8ControlConfig(**control_values["arm"])
    control = SupportedControlConfig(**control_values)
    if hashlib.sha256(scene_xml(scene).encode()).hexdigest() != original["model_xml_sha256"]:
        raise ValueError("Decoded configuration differs from the archived XML")
    runtime = require_micron_engine()
    model, model_identity = recorded_model(args.parent, original)
    if not (qpos.shape == (model.nq,) and qvel.shape == (model.nv,) and ctrl.shape == (model.nu,)
            and np.isfinite(qpos).all() and np.isfinite(qvel).all() and np.isfinite(ctrl).all()):
        raise ValueError("Cold native state is not finite/aligned with exact archived model")
    yaw_anchor = original["initial_bolt_yaw_rad"]
    for phase in original["phases"]:
        if phase["phase"] == "transfer_bolt_weight":
            break
        yaw_anchor += phase["bolt_clockwise_rotation_rad"]
    declaration = {
        "scope":"Separate new cold diagnostic from exact archived post-integration checkpoint, without solver warm-start or original force-window reuse. Never splice into a full trajectory.",
        "parent":str(args.parent.resolve()), "parent_input_sha256":inputs,
        "parent_native_checkpoint_time_s":native_time, "parent_native_checkpoint_index":index,
        "parent_original_settled_reference_event":reference,
        "qpos_sha256":hashlib.sha256(qpos.tobytes()).hexdigest(),
        "qvel_sha256":hashlib.sha256(qvel.tobytes()).hexdigest(),
        "ctrl_sha256":hashlib.sha256(ctrl.tobytes()).hexdigest(),
        "cold_time_origin_s":0., "native_warm_start_copied":False,
        "source_65_before":before, "producer_commit":original_after["producer_commit"],
        "execution_repository_head":subprocess.check_output(["git","rev-parse","HEAD"],cwd=root,text=True).strip(),
        "controller_source_sha256":source_sha, "crest_observer_source_sha256":observer_sha,
        "archived_model_identity":model_identity, "runtime":runtime,
        "original_closed_yaw_anchor_rad":yaw_anchor,
        "original_independent_clock_anchor_rad":reference["desired_clock_rad"],
        "physical_change":"Replace absolute-settled-depth direction observer with measured-crest-return CLOSED stop request. Require persistent return and original100ms native impulse/angular/axial quiet PLUS original100ms90/10 native weight before first closed forward. Original controller command targets, finite caps and every hard physics/grasp/table/support guard are unchanged.",
    }
    if args.prepare_only:
        print(json.dumps({"prepared":True,"declaration":declaration},indent=2,allow_nan=False))
        return
    args.output.mkdir(parents=True)
    (args.output/"diagnostic_declaration.json").write_text(json.dumps(declaration,indent=2,allow_nan=False))
    (args.output/"original_controller_source.py").write_bytes((args.parent.parent/"controller_source.py").read_bytes())
    (args.output/"experimental_observer_source.py").write_bytes(observer_path.read_bytes())
    (args.output/"diagnostic_source.py").write_bytes(Path(__file__).read_bytes())
    (args.output/"supported_scene.zip").write_bytes((args.parent.parent/"supported_scene.zip").read_bytes())
    _COLD_CONTEXT = {"model":model,"qpos":qpos,"qvel":qvel,"ctrl":ctrl,
        "left_acquisition":original["left_acquisition"],
        "right_acquisition":original["right_grasp_acquisitions"][0],
        "desired_clock_rad":reference["desired_clock_rad"],
        "closed_yaw_anchor_rad":yaw_anchor,"settled_base_z_m":reference["base_z_m"],
        "observer_sha256":observer_sha,"declaration":declaration}
    result = run_supported_demo(args.output, scene_config=scene, control_config=control)
    after = {name:_sha(root/name) for name in before}
    closure = {"source_65_after":after,"source_65_unchanged":after==before,
        "harness_sha256_before":source_sha,"harness_sha256_after":_sha(__file__),
        "observer_sha256_before":observer_sha,"observer_sha256_after":_sha(observer_path),
        "parent_input_sha256_before":inputs,
        "parent_input_sha256_after":{name:_sha(args.parent.parent/name) for name in inputs},
        "diagnostic_completed":result["diagnostic_completed"],
        "full_fresh_trajectory_qualified":False,"capture_or_open_reset_qualified":False}
    (args.output/"diagnostic_execution_after.json").write_text(json.dumps(closure,indent=2,allow_nan=False))
    print(json.dumps({"diagnostic_completed":result["diagnostic_completed"],"aborted":result["aborted"],
                      "final_crest_event":result["final_experimental_crest_direction_event"]},indent=2,allow_nan=False))
    if (after != before or source_sha != _sha(__file__) or observer_sha != _sha(observer_path)
            or closure["parent_input_sha256_after"] != inputs):
        raise RuntimeError("Immutable execution or parent source changed")
    raise SystemExit(0 if result["diagnostic_completed"] else 1)


if __name__ == "__main__":
    main()
'''
compile(source, str(TARGET), 'exec')
TARGET.write_text(source)
TARGET.with_suffix('.changes.json').write_text(json.dumps(changes, indent=2))
print(json.dumps({'source':str(TARGET), 'sha256':hashlib.sha256(TARGET.read_bytes()).hexdigest(),
                  'changes':len(changes), 'integration_started':False}))
