"""Output-only reach study: an adjacent hex-face grip, no integration."""
from pathlib import Path
import hashlib
import json
import sys

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = next(p for p in Path(__file__).resolve().parents if (p / "thread_lab/runtime.py").is_file())
sys.path.insert(0, str(ROOT))
from yam_twin.kinematics import ArmIK
from scripts.audit_m8_insertion_trace import recorded_model, archived_source_object
from thread_lab.runtime import require_micron_engine


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    out = ROOT / "outputs/m8_table_supported/diagnostics/kinematic_face120_route_v2"
    out.mkdir(exist_ok=False)
    source = Path(__file__).resolve()
    (out / "diagnostic_source.py").write_bytes(source.read_bytes())
    parent = ROOT / "outputs/m8_table_supported/full_v2/insertion_trace.npz"
    saved = np.load(parent, allow_pickle=False)
    metadata = json.loads(str(saved["metadata_json"].item()))
    rows = json.loads(str(saved["info_json"].item()))
    kinematics = ROOT / "yam_twin/kinematics.py"
    recorded_ik_sha = next(v for k, v in metadata["recorded_source_dependencies_sha256"].items()
                           if k.endswith("yam_twin/kinematics.py"))
    if sha(kinematics) != recorded_ik_sha:
        raise ValueError("The parent IK source changed")
    archived = parent.parent / "controller_source.py"
    if sha(archived) != metadata["controller_module_sha256"]:
        raise ValueError("Parent controller source changed")
    # Pure contact classification from the exact parent controller, independent
    # of any current scene/controller edits made while this study runs.
    classifier_source = archived_source_object(archived.read_text(), "_unexpected_native_contacts")
    namespace = {}
    exec(classifier_source, namespace)
    classifier = namespace["_unexpected_native_contacts"]
    inputs = out / "inputs"
    inputs.mkdir()
    for path in (kinematics, archived, ROOT / "scripts/audit_m8_insertion_trace.py",
                 ROOT / "thread_lab/runtime.py", ROOT / "thread_lab/plugins/m8_sdf.cc"):
        (inputs / path.name).write_bytes(path.read_bytes())
    model, identity = recorded_model(parent, metadata)
    site, bolt, block = (model.site("right_grasp_site").id,
                        model.body("male_bolt").id, model.body("fixture_block").id)
    joints = [model.joint("right_joint" + str(i)).id for i in range(1, 7)]
    qadr = model.jnt_qposadr[joints]
    pads = [frozenset(model.geom(side + "_m8_pad_" + f).id for f in ("left", "right"))
            for side in ("left", "right")]
    tables = [model.geom(n).id for n in metadata["table_support_geom_names"]]
    thread_geoms = tuple(model.geom(n).id for n in ("bolt_thread", "female_thread"))
    head_geom = model.geom("bolt_head").id
    initial = mujoco.MjData(model)
    mujoco.mj_kinematics(model, initial)
    # Select an equivalent flat pair for robot workspace. The bolt's initial
    # world yaw and all recorded free-body states remain unchanged.
    face_offset = 2 * np.pi / 3
    target_r = initial.xmat[bolt].reshape(3, 3) @ Rotation.from_euler("z", np.pi / 2 + face_offset).as_matrix()
    ik = ArmIK(model, "right")
    ik.bounds[0] += .05
    ik.bounds[1] -= .05
    ik.q = saved["qpos"][0][qadr].copy()
    reference = mujoco.MjData(model)
    results, solved_q = [], []
    phase_ends = {}

    def evaluate(goal, orientation, qbase, source_index, *, segment, theta=None, original_phase=None):
        ik.data.qpos[:] = qbase
        ik.solve(goal, orientation, thorough=(not results))
        p, r = ik.pose()
        position_error = float(np.linalg.norm(p - goal))
        rotation_error = float(np.linalg.norm(Rotation.from_matrix(orientation @ r.T).as_rotvec()))
        margin = float(np.min(np.minimum(ik.q - model.jnt_range[joints, 0],
                                        model.jnt_range[joints, 1] - ik.q)))
        mujoco.mj_collision(model, ik.data)
        early = original_phase not in ("transport_bolt", "align_over_hole", "feed_to_entry") if segment == "parent_route" else False
        unexpected = classifier(model, ik.data, block, bolt, tables, *pads,
                                thread_geoms, allow_bolt_rest=early, head_geom=head_geom)
        if segment != "parent_route":
            # Fixed archived bolt does not rotate with a hypothetical hand.
            # This scan proves only robot/other-robot/table/block workspace.
            unexpected = [c for c in unexpected if not {"bolt_head", "bolt_thread"}
                          & {c["geom1"], c["geom2"]}]
        free_indices = np.setdiff1d(np.arange(model.nq), qadr)
        if not np.array_equal(ik.data.qpos[free_indices], qbase[free_indices]):
            raise ValueError("IK changed a non-right-arm coordinate")
        value = {"segment": segment, "source_state_index": source_index,
                 "source_saved_time_s": float(saved["time"][source_index]),
                 "parent_phase": original_phase, "search_theta_rad": theta,
                 "goal_position_world_m": goal.tolist(), "goal_rotation_world": orientation.tolist(),
                 "right_q_rad": ik.q.tolist(), "position_error_m": position_error,
                 "rotation_error_rad": rotation_error, "minimum_joint_margin_rad": margin,
                 "unexpected_native_geometry": unexpected,
                 "state_source_qpos_sha256": hashlib.sha256(qbase.tobytes()).hexdigest(),
                 "passed": bool(position_error <= 50e-6 and rotation_error <= .0005
                                and margin >= .05 - 1e-9 and not unexpected)}
        results.append(value)
        solved_q.append(ik.q.copy())
        return value

    last_index = None
    for index, row in enumerate(rows):
        if row["phase"] == "start_thread_1":
            break
        reference.qpos[:] = saved["qpos"][index]
        mujoco.mj_kinematics(model, reference)
        goal = reference.site_xpos[site].copy()
        value = evaluate(goal, target_r, saved["qpos"][index], index,
                         segment="parent_route", original_phase=row["phase"])
        phase_ends[row["phase"]] = {"time_s": value["source_saved_time_s"],
                                   "right_q_rad": value["right_q_rad"],
                                   "site_position_world_m": goal.tolist()}
        last_index = index
    qbase = saved["qpos"][last_index]
    reference.qpos[:] = qbase
    mujoco.mj_kinematics(model, reference)
    entry_goal = reference.site_xpos[site].copy()
    for segment, start, end in (("closed_reverse_workspace", 0., -1.9),
                               ("closed_forward_workspace", -1.9, 1.0)):
        count = int(np.ceil(abs(end-start) / .025)) + 1
        for theta in np.linspace(start, end, count):
            orientation = Rotation.from_rotvec(np.array([0., 0., -theta])).as_matrix() @ target_r
            evaluate(entry_goal, orientation, qbase, last_index, segment=segment, theta=float(theta))
    summaries = {}
    for segment in sorted({r["segment"] for r in results}):
        selected = [r for r in results if r["segment"] == segment]
        summaries[segment] = {"samples": len(selected), "all_sampled_states_passed": all(r["passed"] for r in selected),
            "peak_position_error_m": max(r["position_error_m"] for r in selected),
            "peak_rotation_error_rad": max(r["rotation_error_rad"] for r in selected),
            "minimum_joint_margin_rad": min(r["minimum_joint_margin_rad"] for r in selected),
            "bad_states": [r for r in selected if not r["passed"]]}
    np.savez_compressed(out / "solved_right_arm_coordinates.npz", right_q=np.asarray(solved_q))
    report = {"scope": "Output-only sampled fixed-pose reach/collision study. No native integration, forces, grip proof, thread proof, policy success or complete continuous-path certification.",
        "operations": ["Archived post-integration non-right-arm coordinates copied unchanged", "right-arm bounded least-squares IK", "mj_kinematics", "mj_collision"],
        "not_performed": ["mj_step", "mj_forward", "mj_contactForce", "constraint solve", "object rotation/phase registration", "canonical source edits"],
        "candidate_rationale": "+120 degrees selects another identical parallel face pair of the native regular hexagonal head and avoids the +60-degree high-transfer joint4 limit. It is chosen for robot workspace, independently of thread groove phase.",
        "face_offset_rad": face_offset, "initial_bolt_world_yaw_rad_unchanged": metadata["scene_config"]["bolt_yaw_rad"],
        "parent_trace_sha256": sha(parent), "model_identity": identity, "runtime": require_micron_engine(),
        "plugin_library_sha256": sha(ROOT / "thread_lab/plugins/libm8_sdf.so"),
        "input_source_sha256": {p.name: sha(p) for p in sorted(inputs.iterdir())},
        "classifier_function_sha256": hashlib.sha256(classifier_source.encode()).hexdigest(),
        "diagnostic_source_sha256": sha(source),
        "coordinates_archive_sha256": sha(out / "solved_right_arm_coordinates.npz"),
        "sampling": "Every archived parent position sample before starting rotation (nominal5ms, including phase endpoints); closed workspace orientation grid<=.025rad at fixed entry height.",
        "route_scope": "Original whole-scene fixed free-body states and finger coordinates retained. +120deg grip has the identical hex-head collision geometry; all original unexpected-pair classification remains active for these poses. Native measured10mm rest transfer guard is not changed by this study, but must be remeasured in a fresh rollout.",
        "search_scope": "Fixed entry bolt does not follow hypothetical hand, so bolt-related contacts are excluded only from this non-workpiece workspace study. Fresh native integration must verify actual grasp, bolt/table/rest/thread contact and all original guards.",
        "forward_limit_note": "From reverse−1.9, fixedπ forward would end+1.2416 and exceed this grip's conservative~+1.13 forward wrist bound. This study ends+1.0; a real controller must stop using actual joint margins. Search drop is not capture, and opening still requires genuine loaded partial formed guidance/support.",
        "phase_endpoints": phase_ends, "summary": summaries, "results": results}
    (out / "study.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    (out / "summary.json").write_text(json.dumps({"scope": report["scope"], "face_offset_rad": face_offset,
        "phase_endpoints": phase_ends, "summary": summaries, "study_sha256": sha(out / "study.json")}, indent=2, allow_nan=False) + "\n")
    print(json.dumps(summaries, allow_nan=False))


if __name__ == "__main__":
    main()
