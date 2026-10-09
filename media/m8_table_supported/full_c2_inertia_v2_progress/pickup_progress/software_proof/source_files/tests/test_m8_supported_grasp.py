"""Grasp selection follows the physical hex head without registering its phase."""
from dataclasses import replace

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from yam_twin.m8_supported_scene import build_model, supported_config
from yam_twin.m8_supported_simulation import (
    SupportedControlConfig, initialize_supported_pose,
)


@pytest.fixture(scope="module")
def supported_model():
    scene = supported_config()
    return scene, build_model(scene)


def _free_slice(model, name):
    joint = model.joint(name).id
    qstart = int(model.jnt_qposadr[joint])
    return slice(qstart, qstart + 7)


def _set_free_pose(model, data, name, position, rotation):
    qslice = _free_slice(model, name)
    data.qpos[qslice] = np.r_[position, np.roll(rotation.as_quat(), 1)]


def _spawn_bolt_rotation(model, data):
    qslice = _free_slice(model, "male_bolt_free")
    return Rotation.from_quat(np.roll(data.qpos[qslice][3:], -1))


@pytest.mark.parametrize("bolt_angles", [(0., 0., .78), (.04, -.06, -.55)])
def test_pickup_follows_moved_rotated_bolt_and_preserves_free_state(
    supported_model, bolt_angles,
):
    scene, model = supported_model
    data = mujoco.MjData(model)
    bolt_rotation = Rotation.from_euler("xyz", bolt_angles) * _spawn_bolt_rotation(model, data)
    bolt_origin = np.array([.365, -.213, .044])
    _set_free_pose(model, data, "male_bolt_free", bolt_origin, bolt_rotation)
    # The initialization contract applies to both existing free objects,
    # including a block pose that differs from its scene configuration.
    _set_free_pose(model, data, "fixture_block_free", [.302, -.011, .012],
                   Rotation.from_euler("xyz", [.02, -.03, 1.21]))
    data.qvel[:] = np.linspace(-.013, .021, model.nv)
    free_slices = [_free_slice(model, name) for name in
                   ("male_bolt_free", "fixture_block_free")]
    free_before = [data.qpos[qslice].copy() for qslice in free_slices]
    velocity_before = data.qvel.copy()
    mujoco.mj_forward(model, data)
    physical_head_center = data.site_xpos[model.site("bolt_head_grasp").id].copy()
    physical_bolt_axis = data.xmat[model.body("male_bolt").id].reshape(3, 3)[:, 2].copy()

    control = replace(SupportedControlConfig(), pickup_grasp_face_offset_rad=2*np.pi/3)
    targets, pickup, grasp_rotation = initialize_supported_pose(model, data, scene, control)

    for qslice, before in zip(free_slices, free_before):
        np.testing.assert_array_equal(data.qpos[qslice], before)
    np.testing.assert_array_equal(data.qvel, velocity_before)
    np.testing.assert_allclose(pickup, physical_head_center, atol=1e-14)
    # Hover follows world up; the grasp axis follows the independently
    # oriented bolt, rather than a configured spawn quaternion.
    np.testing.assert_allclose(targets["right"][0] - physical_head_center,
                               [0., 0., control.pickup_hover_m], atol=1e-14)
    np.testing.assert_allclose(grasp_rotation[:, 2], physical_bolt_axis, atol=1e-14)
    right_site = model.site("right_grasp_site").id
    np.testing.assert_allclose(data.site_xpos[right_site], targets["right"][0], atol=2e-6)
    actual_rotation = data.site_xmat[right_site].reshape(3, 3)
    assert np.linalg.norm(Rotation.from_matrix(grasp_rotation @ actual_rotation.T).as_rotvec()) < 2e-5
    np.testing.assert_array_equal(data.xfrc_applied, 0.)
    np.testing.assert_array_equal(data.qfrc_applied, 0.)


def test_selected_alternate_grasp_normal_matches_opposed_compiled_hex_flats(supported_model):
    scene, model = supported_model
    data = mujoco.MjData(model)
    # Changing the part's own orientation must move the selected physical
    # flats and the robot's target together.
    _set_free_pose(model, data, "male_bolt_free", [.363, -.217, .042],
                   Rotation.from_euler("xyz", [.03, -.04, .81]) * _spawn_bolt_rotation(model, data))
    control = replace(SupportedControlConfig(), pickup_grasp_face_offset_rad=2*np.pi/3)
    _, pickup, grasp_rotation = initialize_supported_pose(model, data, scene, control)
    jaw_axis = grasp_rotation[:, 1]
    bolt_rotation = data.xmat[model.body("male_bolt").id].reshape(3, 3)
    bolt_axis = bolt_rotation[:, 2]
    # The baseline closes on the local +/-X flats. The declared +120-degree
    # choice must select the different flat pair with this local normal;
    # merely returning any valid hex flat (or ignoring the offset) fails.
    np.testing.assert_allclose(bolt_rotation.T @ jaw_axis,
                               [.5, -np.sqrt(3)/2, 0.], atol=1e-14)
    geom = model.geom("bolt_head").id
    mesh = int(model.geom_dataid[geom])
    vstart, vend = int(model.mesh_vertadr[mesh]), int(model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh])
    vertices = model.mesh_vert[vstart:vend]
    geom_rotation = data.geom_xmat[geom].reshape(3, 3)
    world_vertices = vertices @ geom_rotation.T + data.geom_xpos[geom]
    fstart, fend = int(model.mesh_faceadr[mesh]), int(model.mesh_faceadr[mesh] + model.mesh_facenum[mesh])
    triangles = world_vertices[model.mesh_face[fstart:fend]]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    # Use actual collision-mesh triangles, independently of the controller's
    # Euler-angle expression. Both opposed side planes must have that normal.
    selected = (np.abs(normals @ jaw_axis) > 1. - 1e-6) & (np.abs(normals @ bolt_axis) < 1e-6)
    assert np.count_nonzero(selected) >= 4
    plane_coordinates = (triangles[selected].mean(axis=1) - pickup) @ jaw_axis
    assert np.any(plane_coordinates > 0.) and np.any(plane_coordinates < 0.)
    np.testing.assert_allclose(np.abs(plane_coordinates), scene.head_across_flats/2,
                               rtol=0., atol=2e-9)
    projections = (world_vertices - pickup) @ jaw_axis
    assert np.ptp(projections) == pytest.approx(scene.head_across_flats, abs=2e-9)


@pytest.mark.parametrize("offset", [0., np.pi/3, 2*np.pi/3, -np.pi/3, 2*np.pi])
def test_real_hex_flat_pair_offsets_are_accepted(offset):
    assert replace(SupportedControlConfig(), pickup_grasp_face_offset_rad=offset).pickup_grasp_face_offset_rad == offset


@pytest.mark.parametrize("offset", [np.nan, np.inf, -np.inf, np.pi/6, .2, 2*np.pi/3 + 1e-5])
def test_nonfinite_or_nonflat_grasp_offsets_are_rejected(offset):
    with pytest.raises(ValueError, match="hex flat pair"):
        replace(SupportedControlConfig(), pickup_grasp_face_offset_rad=offset)
