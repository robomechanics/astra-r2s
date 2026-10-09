"""Physical support, open-thread geometry and native-arm reach for variant two."""
from dataclasses import replace
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from yam_twin import m8_insertion_scene as insertion
from yam_twin.kinematics import ArmIK, HOME
from yam_twin.m8_simulation import YamCartesianController, YamM8ControlConfig
from yam_twin.m8_supported_scene import (
    SupportedConfig, TABLE_SUPPORT_GEOM_NAMES, TABLE_REST_OVERLAP_M,
    block_rotation, build_model, build_spec, initial_bolt_grasp_position,
    initial_hole_position, initial_left_grasp_position, initial_right_grasp_rotation,
    jaw_positions, left_grasp_rotation, left_touch_aperture,
    maximum_tip_insertion_m, scene_fingerprint, scene_xml, table_top_height,
)


def _set_arm(model, data, side, position, rotation, aperture=.024):
    ik = ArmIK(model, side)
    ik.q = HOME.copy()
    ik.solve(position, rotation, thorough=True)
    data.qpos[ik.qadr] = ik.q
    for finger, q in zip(("left", "right"), jaw_positions(aperture)):
        data.qpos[model.joint(f"{side}_{finger}_finger").qposadr[0]] = q
    return ik


def test_additive_scene_keeps_original_published_models_unchanged():
    assert insertion.scene_fingerprint() == "fb08834cd2d24330b48050be0859cc3c8516c896b36fc804595c74b6b7ea85f4"
    cfg = insertion.InsertionConfig()
    cfg = replace(cfg, base=replace(cfg.base, thread=replace(cfg.thread, timestep=.00005)))
    assert insertion.scene_fingerprint(cfg) == "570387eef1dabf8b6c76c043c6f628f14fbd15cd928bcc123454e49600d69e75"
    assert scene_fingerprint() not in (insertion.scene_fingerprint(), insertion.scene_fingerprint(cfg))
    assert not SupportedConfig().pickup_from_table


def test_real_continuous_table_bears_horizontal_free_block_without_a_fixture():
    cfg = SupportedConfig()
    model = build_model(cfg)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    block, table = model.body("fixture_block").id, model.geom("table").id
    np.testing.assert_allclose(data.xmat[block].reshape(3, 3), block_rotation(cfg), atol=1e-14)
    assert data.xpos[block, 2]-cfg.base.block_size[2]/2 == pytest.approx(-TABLE_REST_OVERLAP_M, abs=1e-14)
    assert table_top_height(cfg) == 0.
    np.testing.assert_allclose(model.geom_size[table], [.46, .49, .018], atol=0)
    assert TABLE_SUPPORT_GEOM_NAMES == ("table",)
    contacts = [c for c in data.contact if table in (c.geom1, c.geom2)
                and block in [model.body_weldid[model.geom_bodyid[g]] for g in (c.geom1, c.geom2)]]
    assert contacts
    root = ET.fromstring(scene_xml(cfg))
    assert not root.findall(".//weld")
    assert not root.findall(".//body[@mocap='true']")
    assert (model.nq, model.nv, model.nu, model.neq) == (30, 28, 16, 2)
    free = [model.joint(name).id for name in ("fixture_block_free", "male_bolt_free")]
    assert all(model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in free)
    assert not set(free).intersection(model.actuator_trnid[:, 0])
    assert all(model.eq_type == mujoco.mjtEq.mjEQ_JOINT)
    for name in ("fixture_block", "male_bolt"):
        body = model.body(name).id
        dof = model.jnt_dofadr[model.body_jntadr[body]]
        assert np.all(model.dof_damping[dof:dof+6] == 0.)
        assert np.all(model.dof_armature[dof:dof+6] == 0.)


def test_tilted_side_clamp_and_limited_depth_bore_are_explicit():
    cfg = SupportedConfig()
    np.testing.assert_allclose(initial_left_grasp_position(cfg), [.255, -.010, .014-TABLE_REST_OVERLAP_M], atol=1e-14)
    rotation = left_grasp_rotation(cfg)
    np.testing.assert_allclose(rotation[:, 1], [0., 1., 0.], atol=1e-14)
    np.testing.assert_allclose(rotation[:, 2], [np.sqrt(3)/2, 0., -.5], atol=1e-14)
    assert np.linalg.det(rotation) == pytest.approx(1.)
    assert left_touch_aperture(cfg) == pytest.approx(.020)
    np.testing.assert_allclose(initial_hole_position(cfg), [.35, -.010, .008-TABLE_REST_OVERLAP_M], atol=1e-14)
    assert maximum_tip_insertion_m(cfg) == pytest.approx(.015-TABLE_REST_OVERLAP_M)
    model = build_model(cfg)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    assert data.site_xpos[model.site("bolt_tip").id, 2] == pytest.approx(.020)
    assert initial_bolt_grasp_position(cfg)[1] == -.22
    for changes in ({"pickup_from_table": True}, {"block_position": (.30, -.01, .01)},
                    {"minimum_tip_table_clearance_m": 0.}, {"minimum_tip_table_clearance_m": .016},
                    {"left_grasp_tilt_rad": -1.}):
        with pytest.raises(ValueError):
            replace(cfg, **changes)


def test_unchanged_native_inertials_finite_motors_and_zero_margin_thread_contact():
    cfg = SupportedConfig()
    model = build_model(cfg)
    original = insertion.build_model()
    for name in ("fixture_block", "male_bolt"):
        np.testing.assert_array_equal(model.body(name).mass, original.body(name).mass)
        np.testing.assert_array_equal(model.body(name).inertia, original.body(name).inertia)
    assert np.all(model.actuator_forcelimited)
    np.testing.assert_array_equal(model.actuator_forcerange, original.actuator_forcerange)
    root = ET.fromstring(scene_xml(cfg))
    pair = next(p for p in root.findall("contact/pair")
                if p.get("geom1") == "bolt_thread" and p.get("geom2") == "female_thread")
    assert float(pair.get("margin")) == 0.
    assert pair.get("friction") == "0.15 0.15 0 0 0"
    assert root.find(".//geom[@name='female_thread']").get("type") == "sdf"


def test_native_hands_clear_table_backings_camera_and_opposite_arm_at_targets():
    cfg = SupportedConfig()
    model = build_model(cfg)
    data = mujoco.MjData(model)
    left_p, left_r = initial_left_grasp_position(cfg), left_grasp_rotation(cfg)
    left_ik = _set_arm(model, data, "left", left_p, left_r, aperture=.020)
    assert np.min(np.minimum(left_ik.q-left_ik.bounds[0], left_ik.bounds[1]-left_ik.q)) > .60
    right_ik = ArmIK(model, "right")
    right_ik.q = HOME.copy()
    rest = initial_bolt_grasp_position(cfg)
    poses = [(rest+[0., 0., dz], initial_right_grasp_rotation(cfg)) for dz in (.05, .025, 0.)]
    # Intermediate transport poses matter: independently reachable endpoints
    # can still send a wrist camera through the other arm's native links.
    hole = initial_hole_position(cfg)
    for u in np.linspace(0., 1., 13):
        xy = (1-u)*rest[:2]+u*hole[:2]
        poses.append((np.r_[xy, .065], initial_right_grasp_rotation(cfg)))
    # The head grasp initially puts the tip 1 mm above the block entry. The
    # lower pose represents 6 mm tip insertion, beyond the planned 4.6 mm run.
    for height in (.037, .030):
        for angle in np.linspace(0., -np.pi, 13):
            c, s = np.cos(angle), np.sin(angle)
            yaw = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
            poses.append((np.r_[initial_hole_position(cfg)[:2], height],
                          yaw @ initial_right_grasp_rotation(cfg)))
    for position, rotation in poses:
        right_ik.solve(position, rotation, thorough=True)
        data.qpos[right_ik.qadr] = right_ik.q
        for finger, q in zip(("left", "right"), jaw_positions(.024, cfg)):
            data.qpos[model.joint(f"right_{finger}_finger").qposadr[0]] = q
        mujoco.mj_forward(model, data)
        unintended = []
        for contact in data.contact:
            names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) or ""
                     for g in (contact.geom1, contact.geom2)]
            if contact.dist < -1e-6 and any(name.startswith(("left_", "right_")) for name in names):
                unintended.append((names, float(contact.dist)))
        assert not unintended
        assert np.min(np.minimum(right_ik.q-right_ik.bounds[0], right_ik.bounds[1]-right_ik.q)) > .02
    for dz in (.04, .02, 0.):
        _set_arm(model, data, "left", left_p+[0., 0., dz], left_r, aperture=.024)
        _set_arm(model, data, "right", rest+[0., 0., .05], initial_right_grasp_rotation(cfg))
        mujoco.mj_forward(model, data)
        for contact in data.contact:
            bodies = [model.body(int(model.geom_bodyid[g])).name for g in (contact.geom1, contact.geom2)]
            assert not (contact.dist < -1e-6 and any(name.startswith(("left_", "right_")) for name in bodies))


def test_native_gravity_settle_establishes_actual_table_load():
    cfg = SupportedConfig()
    cfg = replace(cfg, base=replace(cfg.base, thread=replace(cfg.thread, timestep=.00005)))
    model = build_model(cfg)
    data = mujoco.MjData(model)
    targets = {"left": (initial_left_grasp_position(cfg)+[0., 0., .03], left_grasp_rotation(cfg)),
               "right": (initial_bolt_grasp_position(cfg)+[0., 0., .05], initial_right_grasp_rotation(cfg))}
    for side, (position, rotation) in targets.items():
        _set_arm(model, data, side, position, rotation)
    mujoco.mj_forward(model, data)
    controls = {side: YamCartesianController(model, data, side,
                                            YamM8ControlConfig(), cfg.base) for side in targets}
    block, table = model.body("fixture_block").id, model.geom("table").id
    for _ in range(1000):
        for side, (position, rotation) in targets.items():
            controls[side].command(position, rotation, .024)
        mujoco.mj_step(model, data)
    support_z = 0.
    wrench = np.zeros(6)
    for index, contact in enumerate(data.contact):
        roots = [model.body_weldid[model.geom_bodyid[g]] for g in (contact.geom1, contact.geom2)]
        if table in (contact.geom1, contact.geom2) and block in roots:
            mujoco.mj_contactForce(model, data, index, wrench)
            sign = -1 if roots[0] == block else 1
            support_z += (sign*contact.frame.reshape(3, 3).T @ wrench[:3])[2]
    assert support_z == pytest.approx(model.body_mass[block]*9.81, rel=.01)
    assert np.linalg.norm(data.xpos[block]-cfg.block_position) < .0002
    assert not any(controls[side].contact_wrench_on("fixture_block")["contact_count"] for side in targets)


def test_scene_zip_is_portable_and_keeps_physical_table(tmp_path):
    spec = build_spec()
    path = tmp_path/"yam_m8_supported.zip"
    spec.to_zip(path)
    model = spec.compile()
    restored = mujoco.MjSpec.from_zip(path).compile()
    assert (restored.nq, restored.nv, restored.nu, restored.neq) == (model.nq, model.nv, model.nu, model.neq)
    np.testing.assert_allclose(restored.body_mass, model.body_mass, rtol=5e-6)
    np.testing.assert_array_equal(restored.geom("table").size, model.geom("table").size)
