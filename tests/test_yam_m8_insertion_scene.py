"""Geometry, free-body, and inertial checks for the unengaged insertion scene."""
import ctypes
from dataclasses import replace
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from thread_lab.build_plugin import build_plugin
from yam_twin.m8_insertion_scene import (
    BLOCK_GEOM_NAMES, InsertionConfig, block_mass_properties, block_outer_prisms,
    block_strip_geometry, bore_mass_properties, build_model, build_spec,
    initial_bolt_grasp_position, initial_bolt_origin_position, initial_bolt_rotation,
    initial_hole_position, male_mass_properties, scene_xml,
)


def _inside_polygon(points, polygon):
    polygon = np.asarray(polygon)
    edges = np.roll(polygon, -1, axis=0)-polygon
    relative = points[:, None, :]-polygon[None, :, :]
    cross = edges[None, :, 0]*relative[:, :, 1]-edges[None, :, 1]*relative[:, :, 0]
    return np.all(cross > 1e-13, axis=1) | np.all(cross < -1e-13, axis=1)


def test_rectangular_outer_block_is_exactly_tiled_without_overlapping_solids():
    cfg = InsertionConfig()
    rng = np.random.default_rng(805)
    points = rng.uniform(-.5, .5, (30_000, 3))*cfg.base.block_size
    ownership = np.zeros(len(points), dtype=int)
    for _, center, size in block_strip_geometry(cfg):
        ownership += np.all(np.abs(points-center) < np.asarray(size)-1e-12, axis=1)
    for _, polygon in block_outer_prisms(cfg):
        ownership += _inside_polygon(points[:, :2], polygon)
    relative = points-np.asarray(cfg.hole_offset)
    normals = np.column_stack((np.cos(np.arange(6)*np.pi/3), np.sin(np.arange(6)*np.pi/3)))
    ownership += np.max(relative[:, :2]@normals.T, axis=1) < cfg.thread.nut_across_flats/2-1e-12
    np.testing.assert_array_equal(ownership, np.ones(len(points), dtype=int))
    # Exact areas prove the same result without relying solely on sampling.
    area = 0.
    for _, _, half in block_strip_geometry(cfg):
        area += 4*half[0]*half[1]
    for _, polygon in block_outer_prisms(cfg):
        p = np.asarray(polygon)
        area += abs(np.sum(p[:, 0]*np.roll(p[:, 1], -1)-np.roll(p[:, 0], -1)*p[:, 1]))/2
    area += np.sqrt(3)*cfg.thread.nut_across_flats**2/2
    assert area == pytest.approx(cfg.base.block_size[0]*cfg.base.block_size[1], abs=1e-16)


def test_real_plugin_bore_remains_open_through_every_block_collision_piece():
    cfg = InsertionConfig()
    library = ctypes.CDLL(str(build_plugin()))
    sdf = library.astra_thread_distance
    sdf.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
    sdf.restype = ctypes.c_double
    t = cfg.thread
    attributes = np.array([.008, t.pitch, t.nut_height, t.female_pitch_diameter,
                           t.nut_across_flats, .000360, 1., 0.], dtype=np.float64)
    for z in np.linspace(-t.nut_height/2, t.nut_height/2, 81):
        point = np.array([0., 0., z], dtype=np.float64)
        distance = sdf(point.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
                       attributes.ctypes.data_as(ctypes.POINTER(ctypes.c_double)))
        assert distance > .003
        block_point = np.asarray(cfg.hole_offset)+point*np.array([1., -1., -1.])
        for _, center, half in block_strip_geometry(cfg):
            assert not np.all(np.abs(block_point-center) < np.asarray(half)+1e-12)
        assert not any(_inside_polygon(block_point[None, :2], polygon)[0]
                       for _, polygon in block_outer_prisms(cfg))
    root = ET.fromstring(scene_xml(cfg))
    female = root.find(".//geom[@name='female_thread']")
    assert female.get("type") == "sdf"
    pair = next(pair for pair in root.findall("contact/pair")
                if pair.get("geom1") == "bolt_thread" and pair.get("geom2") == "female_thread")
    assert float(pair.get("margin")) == 0
    assert pair.get("friction") == "0.15 0.15 0 0 0"


def test_both_objects_are_free_and_only_actual_robot_joints_are_actuated():
    model = build_model()
    assert (model.nq, model.nv, model.nu, model.neq) == (30, 28, 16, 2)
    free_joints = {model.joint(name).id for name in ("fixture_block_free", "male_bolt_free")}
    assert all(model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE for j in free_joints)
    assert not free_joints.intersection(model.actuator_trnid[:, 0])
    assert np.all(model.eq_type == mujoco.mjtEq.mjEQ_JOINT)
    assert np.all(model.actuator_forcelimited)
    assert model.body("female_frame").parentid == model.body("fixture_block").id
    assert model.body("male_bolt").parentid == 0
    root = ET.fromstring(scene_xml())
    assert not root.findall(".//weld")
    assert not any("male_bolt" in p.attrib.values() or "fixture_block" in p.attrib.values()
                   for p in root.findall("contact/exclude"))
    assert set(BLOCK_GEOM_NAMES).issubset({geom.get("name") for geom in root.findall(".//geom")})


def test_downward_axes_put_the_chamfered_tip_first_without_preset_engagement():
    cfg = InsertionConfig()
    model = build_model(cfg)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    female = model.body("female_frame").id
    male = model.body("male_bolt").id
    np.testing.assert_allclose(data.xmat[female].reshape(3, 3)[:, 2], [0., 0., -1.], atol=1e-14)
    np.testing.assert_allclose(data.xmat[male].reshape(3, 3), initial_bolt_rotation(cfg), atol=1e-14)
    np.testing.assert_allclose(data.site_xpos[model.site("bolt_head_grasp").id], initial_bolt_grasp_position(cfg), atol=1e-14)
    np.testing.assert_allclose(data.site_xpos[model.site("bolt_tip").id],
        initial_bolt_origin_position(cfg)+[0., 0., -cfg.thread.bolt_length], atol=1e-14)
    np.testing.assert_allclose(data.site_xpos[model.site("hole_origin").id], initial_hole_position(cfg), atol=1e-14)
    assert np.linalg.norm(initial_bolt_origin_position(cfg)[:2]-initial_hole_position(cfg)[:2]) > .1
    assert cfg.bolt_yaw_rad == .23
    thread_pair = {model.geom("bolt_thread").id, model.geom("female_thread").id}
    assert not any({int(contact.geom1), int(contact.geom2)} == thread_pair for contact in data.contact)


def test_mass_and_inertia_count_bore_removal_and_steel_head_once():
    cfg = InsertionConfig()
    model = build_model(cfg)
    for name, function in (("fixture_block", block_mass_properties), ("male_bolt", male_mass_properties)):
        coarse, fine = function(cfg, 262_144), function(cfg, 1_048_576)
        assert coarse["mass_kg"] == pytest.approx(fine["mass_kg"], rel=2e-5)
        np.testing.assert_allclose(np.diag(coarse["inertia_kg_m2"]), np.diag(fine["inertia_kg_m2"]), rtol=3e-5)
        assert np.linalg.eigvalsh(fine["inertia_kg_m2"]).min() > 0
        body = model.body(name).id
        assert model.body_mass[body] == pytest.approx(fine["mass_kg"], rel=1e-11)
        np.testing.assert_allclose(model.body_ipos[body], fine["centroid_m"], atol=1e-12)
    block = block_mass_properties(cfg)
    assert block["mass_kg"] == pytest.approx(.10368-bore_mass_properties(cfg)["mass_kg"], abs=1e-15)
    assert .10181 < block["mass_kg"] < .10182
    assert .02674 < male_mass_properties(cfg)["mass_kg"] < .02676
    assert block["centroid_m"][1] > 0  # Removing aluminum from the negative-Y end shifts COM.


def test_fixed_rest_supports_head_and_clears_the_complete_threaded_shaft():
    cfg = InsertionConfig()
    model = build_model(cfg)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    root_z = initial_bolt_origin_position(cfg)[2]
    for index in range(3):
        geom = model.geom(f"bolt_rest_pin_{index}").id
        assert model.geom_bodyid[geom] == 0
        assert data.geom_xpos[geom][2]+model.geom_size[geom][1] == pytest.approx(root_z, abs=1e-12)
        radius = np.linalg.norm(data.geom_xpos[geom][:2]-initial_bolt_grasp_position(cfg)[:2])
        assert radius-model.geom_size[geom][0] > .004
    with pytest.raises(ValueError):
        replace(cfg, rest_radius=.004)
    with pytest.raises(ValueError):
        replace(cfg, base=replace(cfg.base, thread=replace(cfg.thread, nut_height=.008)))


def test_scene_zip_recompiles_without_external_paths(tmp_path):
    spec = build_spec()
    path = tmp_path/"yam_m8_insertion.zip"
    spec.to_zip(path)
    original, restored = spec.compile(), mujoco.MjSpec.from_zip(path).compile()
    assert (restored.nq, restored.nv, restored.nu, restored.nmesh) == (
        original.nq, original.nv, original.nu, original.nmesh)
    np.testing.assert_allclose(restored.body_mass, original.body_mass, rtol=5e-6)
