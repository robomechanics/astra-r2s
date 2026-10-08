"""Mechanical composition checks for real YAM fingers and passive M8 contacts."""
from dataclasses import replace
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from thread_lab.model import ThreadConfig, model_xml
from yam_twin.kinematics import ArmIK, HOME
from yam_twin.m8_scene import (
    PAD_NAMES, YamM8Config, build_model, build_spec, initial_nut_position, scene_xml,
    initial_block_position, initial_left_grasp_position, jaw_positions,
    left_grasp_rotation, left_touch_aperture, nut_mass_properties, bolt_mass_properties,
)


def _set_arm(model, data, side, q, aperture=.017):
    for index, value in enumerate(q, 1):
        data.qpos[model.joint(f"{side}_joint{index}").qposadr[0]] = value
    for finger, q in zip(("left", "right"), jaw_positions(aperture)):
        data.qpos[model.joint(f"{side}_{finger}_finger").qposadr[0]] = q


def test_m8_contact_geometry_options_and_mass_are_reused_verbatim():
    config = YamM8Config()
    scene = ET.fromstring(scene_xml(config))
    reference = ET.fromstring(model_xml(config.thread))
    for path in ("option", "extension", "worldbody/body/inertial", "contact/pair"):
        # The nut itself is translated into the workcell; its local shape and
        # inertial properties, and the bolt/nut contact law, are unchanged.
        scene_path = "worldbody/body[@name='nut']/inertial" if path == "worldbody/body/inertial" else path
        assert ET.tostring(scene.find(scene_path)) == ET.tostring(reference.find(path))
    assert ET.tostring(scene.find("worldbody/body[@name='nut']/geom")) == ET.tostring(
        reference.find("worldbody/body/geom"))
    scene_bolt = scene.find(".//body[@name='bolt_frame']/geom[@name='bolt_thread']")
    scene_bolt.attrib.pop("mass")  # Independent steel inertia is in the rigid bolt body.
    assert ET.tostring(scene_bolt) == ET.tostring(reference.find("worldbody/geom[@name='bolt_thread']"))


def test_nut_is_free_and_only_real_arm_and_finger_joints_are_actuated():
    model = build_model()
    nut_joint = model.joint("nut_free").id
    assert model.jnt_type[nut_joint] == mujoco.mjtJoint.mjJNT_FREE
    block_joint = model.joint("fixture_block_free").id
    assert model.jnt_type[block_joint] == mujoco.mjtJoint.mjJNT_FREE
    assert model.body("bolt_frame").parentid == model.body("fixture_block").id
    assert model.nv == 28  # two 6+2-DOF YAMs plus free nut and free clamping block
    assert model.nu == 16
    assert model.neq == 2
    assert np.all(model.eq_type == mujoco.mjtEq.mjEQ_JOINT)
    assert all(nut_joint != joint for joint in model.actuator_trnid[:, 0])
    assert all(block_joint != joint for joint in model.actuator_trnid[:, 0])
    assert np.all(model.actuator_forcelimited)
    for side in ("left", "right"):
        for index in range(1, 7):
            actuator = model.actuator(f"{side}_servo{index}").id
            np.testing.assert_allclose(model.actuator_gear[actuator], [1, 0, 0, 0, 0, 0])
            assert model.actuator_gaintype[actuator] == mujoco.mjtGain.mjGAIN_FIXED
            assert model.actuator_biastype[actuator] == mujoco.mjtBias.mjBIAS_NONE
            assert model.actuator_gainprm[actuator, 0] == 1
            limit = 28 if index <= 3 else 10
            np.testing.assert_allclose(model.actuator_ctrlrange[actuator], [-limit, limit])
    xml = ET.fromstring(scene_xml())
    assert not xml.findall(".//weld")
    assert not xml.findall(".//velocity")
    for exclusion in xml.findall("contact/exclude"):
        assert "nut" not in exclusion.attrib.values()
        assert "bolt_frame" not in exclusion.attrib.values()


@pytest.mark.parametrize("aperture", [.0184, .0194, .024])
def test_pad_aperture_tracks_the_actual_native_slide_joints(aperture):
    model = build_model()
    data = mujoco.MjData(model)
    _set_arm(model, data, "right", HOME, aperture)
    mujoco.mj_kinematics(model, data)
    site = model.site("right_grasp_site").id
    world_to_grasp = data.site_xmat[site].reshape(3, 3).T
    for name, sign, parent in zip(PAD_NAMES, (1, -1), ("right_lf_down", "right_rf_down")):
        geom = model.geom(name).id
        assert model.geom_bodyid[geom] == model.body(parent).id
        position = world_to_grasp @ (data.geom_xpos[geom] - data.site_xpos[site])
        np.testing.assert_allclose(position, [0, sign * (aperture / 2 + .0015), 0], atol=1e-12)
        np.testing.assert_allclose(model.geom_size[geom], [.0045, .0015, .00275])
        assert model.geom_contype[geom] and model.geom_conaffinity[geom]
    native = model.site("right_native_grasp_site").id
    offset = world_to_grasp @ (data.site_xpos[native] - data.site_xpos[site])
    np.testing.assert_allclose(offset, [-.014, 0, -.0158], atol=1e-12)


def test_reachable_120_degree_stroke_has_no_robot_fixture_collisions():
    model = build_model()
    data = mujoco.MjData(model)
    config = YamM8Config()
    left_ik = ArmIK(model, "left")
    left_q = left_ik.solve(initial_left_grasp_position(config), left_grasp_rotation(config), thorough=True)
    _set_arm(model, data, "left", left_q, left_touch_aperture(config))
    ik = ArmIK(model, "right")
    ik.q = np.array([.76672, 2.15180, 1.97955, -1.39855, 0, -.80407])
    nut_center = initial_nut_position()
    fixtures = {model.geom(name).id for name in ("fixture_block_geom", "table", "mounting_rail")}
    pad_geoms = {model.geom(f"{side}_m8_pad_{finger}").id
                 for side in ("left", "right") for finger in ("left", "right")}
    robot_geoms = {index for index, body in enumerate(model.geom_bodyid)
                   if model.body(int(body)).name.startswith(("left_", "right_"))}
    for angle in np.linspace(90, -30, 13):
        rotation = Rotation.from_euler("z", angle, degrees=True).as_matrix() @ np.diag([-1, 1, -1])
        q = ik.solve(nut_center, rotation, thorough=True)
        _set_arm(model, data, "right", q, config.open_aperture)
        mujoco.mj_forward(model, data)
        site = model.site("right_grasp_site").id
        assert np.linalg.norm(data.site_xpos[site] - nut_center) < 2e-6
        for contact in data.contact:
            pair = {int(contact.geom1), int(contact.geom2)}
            # The left pads are intentionally touching the free block. All
            # backing, table and cross-arm contacts must remain clear.
            assert not (pair & fixtures and pair & (robot_geoms - pad_geoms)), (
                angle, model.geom(contact.geom1).name, model.geom(contact.geom2).name, contact.dist)


def test_scene_zip_contains_both_real_arms_meshes_and_recompiles(tmp_path):
    spec = build_spec()
    model = spec.compile()
    path = tmp_path / "yam_m8.zip"
    spec.to_zip(path)
    with zipfile.ZipFile(path) as archive:
        meshes = [name for name in archive.namelist() if name.endswith(".stl")]
        assert len(meshes) == 18  # shared by both arms
        xml_file = next(name for name in archive.namelist() if name.endswith(".xml"))
        xml = archive.read(xml_file)
        assert b"/workspace" not in xml
        for mesh in ET.fromstring(xml).findall("asset/mesh"):
            file = mesh.get("file")
            if file:
                assert not Path(file).is_absolute()
                assert file in archive.namelist()
    restored = mujoco.MjSpec.from_zip(path).compile()
    assert (restored.nq, restored.nv, restored.nu, restored.neq, restored.nmesh) == (
        model.nq, model.nv, model.nu, model.neq, model.nmesh)
    # MuJoCo's XML writer emits six significant digits for numeric inertials.
    # Native plugin config strings retain the full specified thread dimensions.
    np.testing.assert_allclose(restored.body_mass, model.body_mass, rtol=5e-6)


def test_yam_scene_rejects_proxy_hand_guided_nut_and_invalid_transform():
    with pytest.raises(ValueError):
        YamM8Config(thread=ThreadConfig(with_gripper=True))
    with pytest.raises(ValueError):
        YamM8Config(thread=ThreadConfig(guided=True))
    with pytest.raises(ValueError):
        replace(YamM8Config(), bolt_quaternion=(2, 0, 0, 0))


def test_short_bolt_large_nut_and_free_block_mass_properties():
    config = YamM8Config()
    assert config.thread.pitch == .00125
    assert config.thread.male_pitch_diameter == .007100
    assert config.thread.female_pitch_diameter == .007268
    assert config.thread.bolt_length == .016
    assert config.thread.nut_across_flats == .020
    assert config.thread.nut_height == .008
    assert config.thread.initial_z / config.thread.pitch == pytest.approx(7)
    model = build_model(config)
    assert float(model.body("fixture_block").mass[0]) == pytest.approx(.10368)
    assert .00499 < float(model.body("bolt_frame").mass[0]) < .00500
    assert .01898 < config.thread.nut_mass < .01899
    np.testing.assert_allclose(initial_block_position(config), [.30, -.010, .10325])
    np.testing.assert_allclose(initial_left_grasp_position(config), [.30, -.0194, .10325])
    assert left_touch_aperture(config) == pytest.approx(.016)
    assert left_touch_aperture(replace(config, left_grasp_roll_rad=0)) == pytest.approx(.020)
    np.testing.assert_allclose(left_grasp_rotation(config), [[-1, 0, 0], [0, 0, -1], [0, -1, 0]], atol=1e-15)
    for integrate in (nut_mass_properties, bolt_mass_properties):
        coarse = integrate(config.thread, samples=262_144)
        fine = integrate(config.thread, samples=1_048_576)
        assert abs(coarse["mass_kg"] / fine["mass_kg"] - 1) < 2e-5
        np.testing.assert_allclose(np.diag(coarse["inertia_kg_m2"]),
                                   np.diag(fine["inertia_kg_m2"]), rtol=3e-5)
        assert np.linalg.eigvalsh(np.asarray(fine["inertia_kg_m2"])).min() > 0


def test_left_block_impedance_study_changes_only_the_two_left_block_pad_pairs():
    original = ET.fromstring(scene_xml())
    altered = ET.fromstring(scene_xml(replace(YamM8Config(),
        left_block_contact_impedance=(.999, .9999, .0001))))
    original_pairs, altered_pairs = original.findall("contact/pair"), altered.findall("contact/pair")
    changed = 0
    for old, new in zip(original_pairs, altered_pairs):
        if old.get("geom1") == "fixture_block_geom" and old.get("geom2").startswith("left_"):
            assert old.get("solimp") == "0.95 0.99 0.0001"
            assert new.get("solimp") == "0.999 0.9999 0.0001"
            new.set("solimp", old.get("solimp"))
            changed += 1
    assert changed == 2
    assert ET.tostring(original) == ET.tostring(altered)


@pytest.mark.parametrize("impedance", [(.99999, .99999, .0001), (.95, 1., .0001),
                                       (.00001, .99, .0001), (.999, .99, .0001),
                                       (.95, .99, 0.)])
def test_left_block_impedance_does_not_allow_silent_clamping_or_invalid_width(impedance):
    with pytest.raises(ValueError):
        replace(YamM8Config(), left_block_contact_impedance=impedance)
