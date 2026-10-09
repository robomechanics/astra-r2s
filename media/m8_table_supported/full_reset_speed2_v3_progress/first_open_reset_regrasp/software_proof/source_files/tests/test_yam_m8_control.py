"""Check genuine arm actuation and the independent floating axial command."""
import mujoco
import numpy as np
import pytest

from yam_twin.m8_scene import build_model, jaw_positions, left_touch_aperture
from yam_twin.m8_simulation import (YamCartesianController, YamM8ControlConfig,
                                    demo_phases, initialize_work_pose, smooth_profile)


@pytest.fixture(scope="module")
def model():
    return build_model()


def initialized(model):
    data = mujoco.MjData(model)
    poses = initialize_work_pose(model, data)
    return data, poses, YamCartesianController(model, data)


def test_reset_preserves_free_nut_and_reaches_physical_pad_center(model):
    data = mujoco.MjData(model)
    jid = model.joint("nut_free").id
    addr = int(model.jnt_qposadr[jid])
    original = data.qpos[addr:addr + 7].copy()
    block_addr = int(model.jnt_qposadr[model.joint("fixture_block_free").id])
    block_original = data.qpos[block_addr:block_addr + 7].copy()
    poses = initialize_work_pose(model, data)
    assert np.array_equal(data.qpos[addr:addr + 7], original)
    assert np.array_equal(data.qpos[block_addr:block_addr + 7], block_original)
    assert np.linalg.norm(poses["right"][0] - data.xpos[model.body("nut").id]) < 1e-6
    axis = data.xmat[model.body("bolt_frame").id].reshape(3, 3)[:, 2]
    assert np.dot(poses["right"][1][:, 2], axis) < -.999999


def test_left_ready_grasp_supports_gravity_through_normal_contact(model):
    data, poses, _ = initialized(model)
    assert np.dot(poses["left"][1][:, 1], [0., 0., -1.]) > .999999
    actual = [data.qpos[model.joint(f"left_{finger}_finger").qposadr[0]]
              for finger in ("left", "right")]
    assert np.allclose(actual, jaw_positions(left_touch_aperture()), atol=1e-12, rtol=0)
    commands = [data.ctrl[model.actuator(f"left_grip_{finger}").id]
                for finger in ("left", "right")]
    assert np.allclose(commands, jaw_positions(YamM8ControlConfig().left_closed_aperture), atol=1e-12, rtol=0)
    assert abs(actual[0]) > abs(commands[0])


def test_stroke_profile_has_zero_endpoint_velocity_and_acceleration():
    assert smooth_profile(0.) == (0., 0.)
    assert smooth_profile(1.) == (1., 0.)
    epsilon = 1e-5
    assert abs(smooth_profile(epsilon)[1] / epsilon) < .001
    assert abs(smooth_profile(1. - epsilon)[1] / epsilon) < .001


def test_commands_change_only_motor_controls_and_are_torque_limited(model):
    data, poses, controller = initialized(model)
    before = {name: getattr(data, name).copy() for name in
              ("qpos", "qvel", "xfrc_applied", "qfrc_applied")}
    p, r = poses["right"]
    controller.command(p + np.array([100., -100., 100.]), r, controller.config.closed_aperture,
                       angular_velocity=[100., 100., 100.])
    for name, value in before.items():
        assert np.array_equal(getattr(data, name), value)
    assert np.all(np.abs(data.ctrl[controller.motor_ids]) <= controller.torque_caps)
    assert np.isclose(np.linalg.norm(controller.last_wrench[:3]),
                      controller.config.maximum_cartesian_force)
    assert np.isclose(np.linalg.norm(controller.last_wrench[3:]),
                      controller.config.maximum_cartesian_torque)
    assert np.array_equal(data.ctrl[controller.finger_ids], jaw_positions(controller.config.closed_aperture))


def test_floating_axis_ignores_position_and_velocity_targets_along_bolt(model):
    data, poses, controller = initialized(model)
    p, r = poses["right"]
    axis = data.xmat[model.body("bolt_frame").id].reshape(3, 3)[:, 2]
    baseline = controller.command(p, r, controller.config.closed_aperture, axial_float=True, axis_world=axis)
    wrench = controller.last_wrench.copy()
    altered = controller.command(p + axis * .2, r, controller.config.closed_aperture, linear_velocity=axis * 100.,
                                 axial_float=True, axis_world=axis)
    assert np.allclose(altered, baseline, atol=1e-12, rtol=0)
    assert np.allclose(controller.last_wrench, wrench, atol=1e-12, rtol=0)
    assert np.isclose(np.dot(wrench[:3], axis), controller.config.axial_feed_N)


def test_nut_state_does_not_influence_arm_command(model):
    data, poses, controller = initialized(model)
    p, r = poses["right"]
    baseline = controller.command(p, r, controller.config.closed_aperture, axial_float=True)
    jid = model.joint("nut_free").id
    addr = int(model.jnt_qposadr[jid])
    # Test-only fault injection, not a rollout action: the controller must not
    # synthesize a response from measured nut advance or orientation.
    data.qpos[addr:addr + 3] += [.001, .001, -.003]
    mujoco.mj_forward(model, data)
    changed = controller.command(p, r, controller.config.closed_aperture, axial_float=True)
    assert np.allclose(changed, baseline, atol=1e-12, rtol=0)


def test_viscous_compensation_is_motor_feedforward_inside_the_same_caps(model):
    data, poses, controller = initialized(model)
    data.qvel[controller.dof_indices] = [.01, -.02, .03, -.04, .05, -.06]
    mujoco.mj_forward(model, data)
    mujoco.mj_jacSite(model, data, controller.jp, controller.jr, controller.site_id)
    qv = data.qvel[controller.dof_indices]
    p, r = poses["right"]
    actual_v = controller.jp[:, controller.dof_indices] @ qv
    actual_w = controller.jr[:, controller.dof_indices] @ qv
    expected = data.qfrc_bias[controller.dof_indices] + model.dof_damping[controller.dof_indices] * qv
    controller.command(p, r, controller.config.closed_aperture, linear_velocity=actual_v, angular_velocity=actual_w)
    assert np.allclose(controller.last_motor_torques,
                       np.clip(expected, -controller.torque_caps, controller.torque_caps), atol=1e-10, rtol=0)
    assert np.all(data.xfrc_applied == 0) and np.all(data.qfrc_applied == 0)


def test_strokes_return_hand_open_and_preserve_hex_flat_regrasp():
    config = YamM8ControlConfig()
    phases = demo_phases(config)
    turns = [p for p in phases if p[0].startswith("turn_")]
    resets = [p for p in phases if p[0].startswith("reset_open_")]
    assert len(turns) == 3 and len(resets) == 2
    assert np.isclose(sum(p[2] - p[3] for p in turns), 2 * np.pi)
    assert np.isclose(config.stroke_angle_rad / (np.pi / 3), 2)
    assert all(p[4] == config.open_aperture and p[5] == config.open_aperture
               and not p[6] for p in resets)
