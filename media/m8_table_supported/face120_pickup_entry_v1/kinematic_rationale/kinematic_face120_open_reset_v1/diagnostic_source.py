"""Output-only conditional open-reset workspace scan, never integration."""
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
    out = ROOT / "outputs/m8_table_supported/diagnostics/kinematic_face120_open_reset_v1"
    out.mkdir(exist_ok=False)
    source = Path(__file__).resolve()
    (out / "diagnostic_source.py").write_bytes(source.read_bytes())
    parent = ROOT / "outputs/m8_table_supported/full_v2/insertion_trace.npz"
    saved = np.load(parent, allow_pickle=False)
    metadata = json.loads(str(saved["metadata_json"].item()))
    prior = ROOT / "outputs/m8_table_supported/diagnostics/kinematic_face120_route_v2/study.json"
    declaration = json.loads(prior.read_text())
    if sha(parent) != declaration["parent_trace_sha256"]:
        raise ValueError("Bound original trace changed")
    index = int(np.argmin(abs(saved["time"] - 6.75515)))
    qbase = saved["qpos"][index].copy()
    model, identity = recorded_model(parent, metadata)
    frozen = parent.parent / "controller_source.py"
    classifier_source = archived_source_object(frozen.read_text(), "_unexpected_native_contacts")
    namespace = {}
    exec(classifier_source, namespace)
    classifier = namespace["_unexpected_native_contacts"]
    site, bolt, block = (model.site("right_grasp_site").id,
                        model.body("male_bolt").id, model.body("fixture_block").id)
    joints = [model.joint("right_joint" + str(i)).id for i in range(1, 7)]
    qadr = model.jnt_qposadr[joints]
    pads = [frozenset(model.geom(side + "_m8_pad_" + f).id for f in ("left", "right"))
            for side in ("left", "right")]
    tables = [model.geom(n).id for n in metadata["table_support_geom_names"]]
    thread_geoms = tuple(model.geom(n).id for n in ("bolt_thread", "female_thread"))
    head_geom = model.geom("bolt_head").id
    ref = mujoco.MjData(model)
    ref.qpos[:] = qbase
    mujoco.mj_kinematics(model, ref)
    entry_p = ref.site_xpos[site].copy()
    initial = mujoco.MjData(model)
    mujoco.mj_kinematics(model, initial)
    base_r = initial.xmat[bolt].reshape(3, 3) @ Rotation.from_euler("z", 7 * np.pi / 6).as_matrix()
    # Explicit kinematic jaw opening only, using the parent jaw_positions law.
    aperture = float(metadata["control_config"]["arm"]["open_aperture"])
    pad_offset = float(metadata["scene_config"]["base"]["pad_inner_offset"])
    opening = (aperture + 2 * pad_offset) / 2
    for name, value in (("right_left_finger", opening), ("right_right_finger", -opening)):
        qbase[int(model.joint(name).qposadr[0])] = value
    results, coordinates = [], []
    for depth in (0., .00125, .0025, .004):
        ik = ArmIK(model, "right")
        ik.bounds[0] += .05
        ik.bounds[1] -= .05
        ik.q = np.asarray(declaration["phase_endpoints"]["feed_to_entry"]["right_q_rad"])
        goal = entry_p.copy()
        goal[2] -= depth
        for theta in np.linspace(1., 1. - np.pi, 127):
            ik.data.qpos[:] = qbase
            orientation = Rotation.from_rotvec(np.array([0., 0., -theta])).as_matrix() @ base_r
            ik.solve(goal, orientation, thorough=not results)
            p, r = ik.pose()
            pe = float(np.linalg.norm(p - goal))
            re = float(np.linalg.norm(Rotation.from_matrix(orientation @ r.T).as_rotvec()))
            margin = float(np.min(np.minimum(ik.q - model.jnt_range[joints, 0],
                                            model.jnt_range[joints, 1] - ik.q)))
            mujoco.mj_collision(model, ik.data)
            all_contacts = classifier(model, ik.data, block, bolt, tables, *pads,
                                      thread_geoms, allow_bolt_rest=False, head_geom=head_geom)
            nonworkpiece = [c for c in all_contacts if not {"bolt_head", "bolt_thread"}
                           & {c["geom1"], c["geom2"]}]
            results.append({"conditional_depth_below_archived_entry_m": depth,
                "reset_theta_rad_relative_pickup": float(theta), "right_q_rad": ik.q.tolist(),
                "goal_position_m": goal.tolist(), "position_error_m": pe,
                "rotation_error_rad": re, "minimum_joint_margin_rad": margin,
                "unexpected_nonworkpiece_geometry": nonworkpiece,
                "passed": bool(pe <= 50e-6 and re <= .0005 and margin >= .05 - 1e-9 and not nonworkpiece)})
            coordinates.append(ik.q.copy())
    summaries = []
    for depth in (0., .00125, .0025, .004):
        selected = [r for r in results if r["conditional_depth_below_archived_entry_m"] == depth]
        summaries.append({"conditional_depth_m": depth, "samples": len(selected),
            "all_sampled_states_passed": all(r["passed"] for r in selected),
            "peak_position_error_m": max(r["position_error_m"] for r in selected),
            "peak_rotation_error_rad": max(r["rotation_error_rad"] for r in selected),
            "minimum_joint_margin_rad": min(r["minimum_joint_margin_rad"] for r in selected),
            "bad_states": [r for r in selected if not r["passed"]]})
    np.savez_compressed(out / "solved_right_arm_coordinates.npz", right_q=np.asarray(coordinates))
    report = {"scope": "Conditional sampled open-hand reindex workspace only. No integration, force solve, actual release, grip/capture or reset proof.",
        "candidate": "+120deg hex face pickup; actual-stop clock+1.0 to+1.0−pi is a physical opposite-flat reindex proposal",
        "kinematic_configuration": "Only right six arm coordinates and right finger aperture varied; all original free-body and left-robot qpos retained. Conditional depths0,1.25,2.5,4mm are assumed hand heights, not observed native insertion.",
        "contact_scope": "Bolt contacts excluded because fixed original bolt cannot stand in for a physically inserted free bolt at another depth. All original non-workpiece unexpected-pair guards retained, including cameras, finger backings, other arm, block and table.",
        "opening_requirement": "A real loaded partial formed-contact/support window must permit opening before this motion. This workspace study supplies no contact-release readiness. A fresh native reset must remeasure actual grip, object support and zero drives.",
        "phase_scope": "Opposite hex flats are geometric symmetry. No bolt yaw/phase is assigned or registered. Actual measured stop clock should be used; this study examines endpoint+1.0 and a127-point pi span.",
        "operations": ["right-arm IK", "kinematic right finger opening", "mj_kinematics", "mj_collision"],
        "not_performed": ["mj_forward", "mj_step", "mj_contactForce", "constraint solve", "free-body state changes", "canonical source edits"],
        "parent_trace_sha256": sha(parent), "parent_entry_state_index": index,
        "parent_entry_qpos_sha256": hashlib.sha256(saved["qpos"][index].tobytes()).hexdigest(),
        "prior_route_study_sha256": sha(prior), "model_identity": identity,
        "runtime": require_micron_engine(), "plugin_library_sha256": sha(ROOT / "thread_lab/plugins/libm8_sdf.so"),
        "input_source_sha256": {str(p.relative_to(ROOT)): sha(p) for p in (
            frozen, ROOT / "yam_twin/kinematics.py", ROOT / "yam_twin/m8_scene.py",
            ROOT / "scripts/audit_m8_insertion_trace.py", ROOT / "thread_lab/plugins/m8_sdf.cc")},
        "diagnostic_source_sha256": sha(source), "coordinates_archive_sha256": sha(out / "solved_right_arm_coordinates.npz"),
        "right_open_aperture_m": aperture, "summary": summaries, "results": results}
    (out / "study.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    (out / "summary.json").write_text(json.dumps(summaries, indent=2, allow_nan=False) + "\n")
    print(json.dumps(summaries, allow_nan=False))


if __name__ == "__main__":
    main()
