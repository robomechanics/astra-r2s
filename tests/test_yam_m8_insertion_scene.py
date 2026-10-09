"""Geometry, free-body, and inertial checks for the unengaged insertion scene."""
import ctypes
from dataclasses import replace
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

from thread_lab.build_plugin import build_plugin
from yam_twin.m8_insertion_scene import (
    BLOCK_GEOM_NAMES, TABLE_REST_OVERLAP_M, InsertionConfig, block_mass_properties, block_outer_prisms,
    block_strip_geometry, bore_mass_properties, build_model, build_spec,
    initial_bolt_grasp_position, initial_bolt_origin_position, initial_bolt_rotation,
    initial_hole_position, initial_left_grasp_position, left_grasp_rotation,
    holding_block_position, holding_block_rotation, holding_left_grasp_position,
    holding_left_grasp_rotation, jaw_positions, male_mass_properties,
    scene_fingerprint, scene_xml, table_pickup_config,
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


def test_table_pickup_is_opt_in_and_published_scene_remains_portably_identical():
    # The published raw trajectory must keep replaying the exact old model.
    assert not InsertionConfig().pickup_from_table
    assert scene_fingerprint() == "fb08834cd2d24330b48050be0859cc3c8516c896b36fc804595c74b6b7ea85f4"
    cfg = InsertionConfig()
    fifty_us = replace(cfg, base=replace(cfg.base, thread=replace(cfg.thread, timestep=.000050)))
    assert scene_fingerprint(fifty_us) == "570387eef1dabf8b6c76c043c6f628f14fbd15cd928bcc123454e49600d69e75"
    assert scene_fingerprint(table_pickup_config()) != scene_fingerprint()
    with pytest.raises(ValueError, match="boolean"):
        replace(cfg, pickup_from_table=1)
    assert cfg.left_block_direct_normal_solref is None
    assert cfg.left_block_friction_time_constant is None


def test_table_block_rests_on_its_real_end_then_has_the_qualified_held_transform():
    cfg = table_pickup_config()
    model = build_model(cfg)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    block = model.body("fixture_block").id
    table = model.geom("table").id
    expected_rotation = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
    np.testing.assert_allclose(data.xmat[block].reshape(3, 3), expected_rotation, atol=1e-12)
    np.testing.assert_allclose(data.xpos[block], [.25, .15, .060-TABLE_REST_OVERLAP_M], atol=1e-14)
    # Its actual solid reaches z=0; there is no pedestal or hidden constraint.
    corners = np.array([[x, y, z] for x in (-.010, .010)
                       for y in (-.060, .060) for z in (-.008, .008)])
    world_corners = corners @ expected_rotation.T + data.xpos[block]
    assert world_corners[:, 2].min() == pytest.approx(-TABLE_REST_OVERLAP_M, abs=1e-14)
    assert data.geom_xpos[table][2] + model.geom_size[table][2] == pytest.approx(0.)
    supports = [c for c in data.contact if table in (c.geom1, c.geom2)
                and block in (model.body_weldid[model.geom_bodyid[c.geom1]],
                              model.body_weldid[model.geom_bodyid[c.geom2]])]
    assert supports and min(c.dist for c in supports) >= -TABLE_REST_OVERLAP_M-1e-12
    np.testing.assert_allclose(initial_left_grasp_position(cfg), [.25, .15, .0506-TABLE_REST_OVERLAP_M], atol=1e-14)
    np.testing.assert_allclose(holding_block_position(cfg), [.30, -.010, .10325], atol=1e-14)
    np.testing.assert_allclose(holding_block_rotation(cfg), np.eye(3), atol=1e-14)
    np.testing.assert_allclose(holding_left_grasp_position(cfg), [.30, -.0194, .10325], atol=1e-14)
    # Carrying and rolling the gripped block preserves the exact old relative
    # transform; these are geometry targets, not a free-body state overwrite.
    np.testing.assert_allclose(left_grasp_rotation(cfg).T @ expected_rotation,
                              holding_left_grasp_rotation(cfg).T, atol=1e-14)
    initial_relative = left_grasp_rotation(cfg).T @ (data.xpos[block]-initial_left_grasp_position(cfg))
    held_relative = holding_left_grasp_rotation(cfg).T @ (holding_block_position(cfg)-holding_left_grasp_position(cfg))
    np.testing.assert_allclose(initial_relative, held_relative, atol=1e-14)
    original = build_model(InsertionConfig())
    for name in ("fixture_block", "male_bolt"):
        np.testing.assert_allclose(model.body(name).mass, original.body(name).mass, atol=0)
        np.testing.assert_allclose(model.body(name).inertia, original.body(name).inertia, atol=0)
    assert (model.nq, model.nv, model.nu, model.neq) == (30, 28, 16, 2)


def test_table_objects_have_reachable_native_open_grasps_without_table_penetration():
    # Actual native fingers include large backings and a wrist camera. Merely
    # checking the inserted pad boxes would miss their tabletop collisions.
    from yam_twin.kinematics import ArmIK, HOME
    from yam_twin.m8_insertion_scene import initial_right_grasp_rotation
    cfg = table_pickup_config()
    model = build_model(cfg)
    data = mujoco.MjData(model)
    targets = {"left": (initial_left_grasp_position(cfg), left_grasp_rotation(cfg)),
               "right": (initial_bolt_grasp_position(cfg), initial_right_grasp_rotation(cfg))}
    for side, (position, rotation) in targets.items():
        ik = ArmIK(model, side)
        ik.q = np.asarray(HOME).copy()
        ik.solve(position, rotation, thorough=True)
        data.qpos[ik.qadr] = ik.q
        for finger, q in zip(("left", "right"), jaw_positions(.024, cfg)):
            data.qpos[model.joint(f"{side}_{finger}_finger").qposadr[0]] = q
        assert np.min(np.minimum(ik.q-model.jnt_range[ik.joints, 0],
                                 model.jnt_range[ik.joints, 1]-ik.q)) > .02
    mujoco.mj_forward(model, data)
    block = model.body("fixture_block").id
    bolt = model.body("male_bolt").id
    unexpected = []
    robot_ids = {i for i in range(model.nbody)
                 if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) or "").startswith(("left_", "right_"))}
    for c in data.contact:
        roots = [int(model.body_weldid[model.geom_bodyid[g]]) for g in (c.geom1, c.geom2)]
        bodies = [int(model.geom_bodyid[g]) for g in (c.geom1, c.geom2)]
        if c.dist < -1e-6 and any(body in robot_ids for body in bodies):
            unexpected.append((int(c.geom1), int(c.geom2), float(c.dist)))
        assert not (any(body in robot_ids for body in bodies)
                    and any(root in (block, bolt) for root in roots) and c.dist <= 0)
    assert not unexpected
    assert cfg.bolt_head_position[2] == .040
    assert data.site_xpos[model.site("bolt_tip").id][2] == pytest.approx(.020)
    assert np.linalg.norm(initial_bolt_grasp_position(cfg)[:2]-initial_hole_position(cfg)[:2]) > .25


@pytest.mark.parametrize("normal_time_constant", [.004, .008])
def test_optional_left_block_friction_regularization_is_native_and_scoped(normal_time_constant):
    original = replace(table_pickup_config(), left_block_direct_normal_solref=None,
                       left_block_friction_time_constant=None)
    cfg = replace(original,
                  base=replace(original.base, left_block_contact_time_constant=normal_time_constant),
                  left_block_friction_time_constant=.0008)
    root = ET.fromstring(scene_xml(cfg))
    block_names = {*BLOCK_GEOM_NAMES, "female_thread"}
    changed = []
    pairs = root.findall("contact/pair")
    for pair in pairs:
        is_left_block = (pair.get("geom1") in block_names
                         and pair.get("geom2", "").startswith("left_m8_pad_"))
        if is_left_block:
            np.testing.assert_array_equal(np.fromstring(pair.get("solref"), sep=" "),
                                          [normal_time_constant, 1.])
            np.testing.assert_array_equal(np.fromstring(pair.get("solreffriction"), sep=" "), [.0008, 1.])
            assert pair.get("friction") == ".8 .8 0 0 0" or pair.get("friction") == "0.8 0.8 0 0 0"
            assert pair.get("margin") == "0"
            changed.append(pair)
        else:
            assert "solreffriction" not in pair.attrib
    assert len(changed) == 2*len(block_names)
    # Check what the engine actually compiles, not just the serialized options.
    model = build_model(cfg)
    assert model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC
    checked = 0
    for i in range(model.npair):
        name1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, model.pair_geom1[i])
        name2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, model.pair_geom2[i])
        if name2 in block_names:
            name1, name2 = name2, name1
        if name1 in block_names and name2.startswith("left_m8_pad_"):
            np.testing.assert_array_equal(model.pair_solref[i], [normal_time_constant, 1.])
            np.testing.assert_array_equal(model.pair_solreffriction[i], [.0008, 1.])
            assert model.pair_margin[i] == 0
            checked += 1
    assert checked == len(changed)
    assert original.left_block_friction_time_constant is None
    assert all("solreffriction" not in pair.attrib
               for pair in ET.fromstring(scene_xml(original)).findall("contact/pair"))


@pytest.mark.parametrize("time_constant", [np.nan, np.inf, 0., -.001, .000049])
def test_optional_left_block_friction_regularization_must_be_finite_and_resolved(time_constant):
    with pytest.raises(ValueError, match="friction time constant"):
        replace(InsertionConfig(), left_block_friction_time_constant=time_constant)


def test_convex_mesh_pad_keeps_the_exact_physical_box_envelope_and_properties():
    from yam_twin.m8_scene import PAD_HALF_SIZE
    cfg = table_pickup_config()
    alternative = replace(cfg, left_pad_collision_geometry="convex_mesh")
    box_model, mesh_model = build_model(cfg), build_model(alternative)
    box_data, mesh_data = mujoco.MjData(box_model), mujoco.MjData(mesh_model)
    mujoco.mj_forward(box_model, box_data)
    mujoco.mj_forward(mesh_model, mesh_data)
    corners = np.array([[x, y, z] for x in (-PAD_HALF_SIZE[0], PAD_HALF_SIZE[0])
                        for y in (-PAD_HALF_SIZE[1], PAD_HALF_SIZE[1])
                        for z in (-PAD_HALF_SIZE[2], PAD_HALF_SIZE[2])])
    root = ET.fromstring(scene_xml(alternative))
    mesh_asset = root.find("asset/mesh[@name='left_m8_pad_convex_shape']")
    np.testing.assert_array_equal(np.sort(np.fromstring(mesh_asset.get("vertex"), sep=" ").reshape(-1, 3), axis=0),
                                  np.sort(corners, axis=0))
    assert mesh_model.nmesh == box_model.nmesh+1
    assert (mesh_model.nq, mesh_model.nv, mesh_model.nu, mesh_model.neq) == (
        box_model.nq, box_model.nv, box_model.nu, box_model.neq)
    for name in ("left_m8_pad_left", "left_m8_pad_right"):
        box_id, mesh_id = box_model.geom(name).id, mesh_model.geom(name).id
        assert box_model.geom_type[box_id] == mujoco.mjtGeom.mjGEOM_BOX
        assert mesh_model.geom_type[mesh_id] == mujoco.mjtGeom.mjGEOM_MESH
        mesh = int(mesh_model.geom_dataid[mesh_id])
        start, count = int(mesh_model.mesh_vertadr[mesh]), int(mesh_model.mesh_vertnum[mesh])
        vertices = mesh_model.mesh_vert[start:start+count].astype(float)
        # Mesh compilation may translate/rotate its principal frame. Compare
        # the actual reconstructed world vertices, not the MJCF inputs alone.
        actual = vertices @ mesh_data.geom_xmat[mesh_id].reshape(3, 3).T + mesh_data.geom_xpos[mesh_id]
        expected = corners @ box_data.geom_xmat[box_id].reshape(3, 3).T + box_data.geom_xpos[box_id]
        errors = np.linalg.norm(actual[:, None, :]-expected[None, :, :], axis=-1)
        assert actual.shape == expected.shape == (8, 3)
        assert errors.min(axis=0).max() < 1e-9
        assert errors.min(axis=1).max() < 1e-9
        for attr in ("geom_friction", "geom_margin", "geom_gap", "geom_condim", "geom_contype", "geom_conaffinity",
                     "geom_solref", "geom_solimp"):
            np.testing.assert_array_equal(getattr(mesh_model, attr)[mesh_id], getattr(box_model, attr)[box_id])
        assert float(root.find(f".//geom[@name='{name}']").get("mass")) == 0
    for attr in ("qpos0", "body_mass", "body_inertia", "body_ipos", "body_iquat", "jnt_range",
                 "dof_armature", "dof_damping", "actuator_ctrlrange", "actuator_forcerange",
                 "pair_friction", "pair_margin", "pair_solref", "pair_solreffriction", "pair_solimp"):
        np.testing.assert_array_equal(getattr(mesh_model, attr), getattr(box_model, attr))
    assert all(np.array_equal(getattr(mesh_model.opt, attr), getattr(box_model.opt, attr))
               for attr in ("timestep", "cone", "disableflags", "enableflags", "iterations", "tolerance"))
    assert not mesh_model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_NATIVECCD)
    assert not mesh_model.opt.disableflags & int(mujoco.mjtDisableBit.mjDSBL_MULTICCD)


def test_convex_mesh_pad_selects_actual_native_convex_collision_dispatch():
    from thread_lab.runtime import require_micron_engine
    runtime = require_micron_engine()
    library = ctypes.CDLL(runtime["libraries"][0]["path"])
    count = int(mujoco.mjtGeom.mjNGEOMTYPES)
    dispatch = ((ctypes.c_void_p*count)*count).in_dll(library, "mjCOLLISIONFUNC")
    convex_pointer = ctypes.cast(library.mjc_Convex, ctypes.c_void_p).value
    box, mesh = int(mujoco.mjtGeom.mjGEOM_BOX), int(mujoco.mjtGeom.mjGEOM_MESH)
    assert dispatch[box][mesh] == convex_pointer
    assert dispatch[box][box] != convex_pointer
    cfg = replace(table_pickup_config(), left_pad_collision_geometry="convex_mesh")
    model = build_model(cfg)
    for name in ("left_m8_pad_left", "left_m8_pad_right"):
        pad_type = int(model.geom_type[model.geom(name).id])
        block_type = int(model.geom_type[model.geom("fixture_block_geom").id])
        assert dispatch[min(pad_type, block_type)][max(pad_type, block_type)] == convex_pointer
    with pytest.raises(ValueError, match="requires table pickup"):
        replace(InsertionConfig(), left_pad_collision_geometry="convex_mesh")
    with pytest.raises(ValueError, match="must be box or convex_mesh"):
        replace(table_pickup_config(), left_pad_collision_geometry="sphere")


def test_direct_normal_solref_compiles_separately_from_original_friction_regularization():
    original = replace(table_pickup_config(), left_block_direct_normal_solref=None,
                       left_block_friction_time_constant=None)
    cfg = replace(original, left_block_direct_normal_solref=(-31250., -2500.),
                  left_block_friction_time_constant=.0008)
    root = ET.fromstring(scene_xml(cfg))
    original_root = ET.fromstring(scene_xml(original))
    original_pairs = {(p.get("geom1"), p.get("geom2")): dict(p.attrib)
                      for p in original_root.findall("contact/pair")}
    block_names = {*BLOCK_GEOM_NAMES, "female_thread"}
    changed = []
    for pair in root.findall("contact/pair"):
        original_pair = original_pairs[(pair.get("geom1"), pair.get("geom2"))]
        if pair.get("geom1") in block_names and pair.get("geom2", "").startswith("left_m8_pad_"):
            assert pair.get("solref") == "-31250 -2500"
            assert pair.get("solreffriction") == "0.0008 1"
            expected = {**original_pair, "solref": "-31250 -2500", "solreffriction": "0.0008 1"}
            assert pair.attrib == expected
            changed.append(pair)
        else:
            assert pair.attrib == original_pair
    assert len(changed) == 16
    model = build_model(cfg)
    for i in range(model.npair):
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom)
                 for geom in (model.pair_geom1[i], model.pair_geom2[i])]
        if names[1] in block_names:
            names.reverse()
        if names[0] in block_names and names[1].startswith("left_m8_pad_"):
            np.testing.assert_array_equal(model.pair_solref[i], [-31250., -2500.])
            np.testing.assert_array_equal(model.pair_solreffriction[i], [.0008, 1.])
            assert model.pair_margin[i] == 0
    old_model = build_model(original)
    for attr in ("qpos0", "body_mass", "body_inertia", "geom_type", "geom_pos", "geom_quat",
                 "geom_friction", "pair_friction", "pair_solimp", "pair_margin", "actuator_forcerange"):
        np.testing.assert_array_equal(getattr(model, attr), getattr(old_model, attr))
    restored = replace(original, left_block_direct_normal_solref=[-31250., -2500.])
    assert restored.left_block_direct_normal_solref == (-31250., -2500.)
    assert hash(restored)
    assert original.left_block_direct_normal_solref is None
    assert original.left_pad_collision_geometry == "box"


@pytest.mark.parametrize("parameters", [(-1.,), (-1., -2., -3.), (0., -2500.),
                                      (-31250., 0.), (31250., -2500.),
                                      (-31250., np.inf), (np.nan, -2500.)])
def test_direct_normal_solref_requires_native_signed_finite_parameters(parameters):
    with pytest.raises(ValueError, match="two finite strictly negative"):
        replace(table_pickup_config(), left_block_direct_normal_solref=parameters)


def test_table_factory_has_declared_numerical_normal_compliance_and_original_tangent():
    cfg = table_pickup_config()
    assert cfg.left_pad_collision_geometry == "box"
    assert cfg.left_block_direct_normal_solref == (-31250., -2500.)
    assert cfg.left_block_friction_time_constant == .0008
    assert cfg.base.left_block_contact_impedance == (.9999, .9999, .0001)
    assert cfg.base.pad_friction == .8
    assert cfg.base.maximum_jaw_force == 20.
    assert cfg.base.thread == InsertionConfig().base.thread
    model = build_model(cfg)
    block_names = {*BLOCK_GEOM_NAMES, "female_thread"}
    count = 0
    for i in range(model.npair):
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom)
                 for geom in (model.pair_geom1[i], model.pair_geom2[i])]
        if names[1] in block_names:
            names.reverse()
        if names[0] in block_names and names[1].startswith("left_m8_pad_"):
            np.testing.assert_array_equal(model.pair_solref[i], [-31250., -2500.])
            np.testing.assert_array_equal(model.pair_solreffriction[i], [.0008, 1.])
            np.testing.assert_array_equal(model.pair_solimp[i], [.9999, .9999, .0001, .5, 2.])
            assert model.pair_margin[i] == 0
            count += 1
    assert count == 16


def test_separate_bolt_layout_clears_native_camera_through_reach_and_lift():
    """The real wrist camera collided with the held left finger at y=-.15.

    Sweep intended native arm poses rather than testing the spawn constant.
    This is a geometry regression check; it does not integrate a pickup or
    overwrite an object in an actual rollout.
    """
    from yam_twin.kinematics import ArmIK
    from yam_twin.m8_insertion_scene import initial_right_grasp_rotation

    def path_clearance(cfg):
        model = build_model(cfg)
        data = mujoco.MjData(model)
        left = ArmIK(model, "left")
        left.solve(holding_left_grasp_position(cfg)+[0., 0., .004],
                   holding_left_grasp_rotation(cfg), thorough=True)
        data.qpos[left.qadr] = left.q
        for finger, q in zip(("left", "right"), jaw_positions(.016, cfg)):
            data.qpos[model.joint(f"left_{finger}_finger").qposadr[0]] = q
        left_geoms = [i for i in range(model.ngeom) if model.geom_contype[i]
                      and model.body(int(model.geom_bodyid[i])).name.startswith("left_")]
        right_geoms = [i for i in range(model.ngeom) if model.geom_contype[i]
                       and model.body(int(model.geom_bodyid[i])).name.startswith("right_")]
        right = ArmIK(model, "right")
        center = initial_bolt_grasp_position(cfg)
        minimum = .03
        for z in np.linspace(center[2], .15025, 17):
            right.solve([center[0], center[1], z], initial_right_grasp_rotation(cfg), thorough=True)
            data.qpos[right.qadr] = right.q
            assert np.min(np.minimum(right.q-right.bounds[0], right.bounds[1]-right.q)) > .02
            for finger, q in zip(("left", "right"), jaw_positions(.020, cfg)):
                data.qpos[model.joint(f"right_{finger}_finger").qposadr[0]] = q
            mujoco.mj_forward(model, data)
            for first in left_geoms:
                for second in right_geoms:
                    minimum = min(minimum, float(mujoco.mj_geomDistance(
                        model, data, first, second, .03, None)))
        return minimum

    cfg = table_pickup_config()
    assert path_clearance(cfg) > .005
    # Keep the actual failure as a negative control: a regression of the old
    # layout must fail even though both native IK targets remain reachable.
    old_layout = replace(cfg, bolt_head_position=(.36, -.15, .040))
    assert path_clearance(old_layout) < 0
