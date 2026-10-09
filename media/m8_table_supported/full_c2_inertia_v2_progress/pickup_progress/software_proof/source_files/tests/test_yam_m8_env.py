"""Short YAM policy-interface checks, not a policy-training validation."""
import mujoco
import numpy as np
import pytest

from yam_twin.m8_env import OBSERVATION_FIELDS, YamM8Env
from yam_twin.m8_scene import (YamM8Config, build_model, initial_left_grasp_position,
                              left_grasp_rotation, left_touch_aperture)


@pytest.fixture(scope="module")
def model():
    return build_model(YamM8Config())


def test_reset_uses_real_arm_pose_and_declared_privileged_state(model):
    env = YamM8Env(model, control_dt=.000025)
    obs, info = env.reset(seed=7)
    qpos = env.data.qpos.copy()
    again, _ = env.reset(seed=7)
    assert np.array_equal(qpos, env.data.qpos)
    assert np.array_equal(obs, again)
    assert obs.shape == (sum(OBSERVATION_FIELDS.values()),)
    assert env.observation_space.contains(obs) and np.isfinite(obs).all()
    assert env.action_space.shape == (14,)
    assert info["actual_yam_joint_actuation"] and info["privileged_state_observations"]
    assert info["starts_preengaged"] and not info["fixed_bolt"]
    assert obs.shape == (98,)
    assert not info["free_block_held_by_left_fingers"]
    assert info["workpiece_world_support_contact_count"] == 0
    assert env.scene_config.left_closed_aperture == .0144
    assert np.isclose(left_touch_aperture(env.scene_config), .016)
    assert np.allclose(info["commanded_apertures_m"], [.0144, .024])
    assert np.allclose(info["measured_apertures_m"], [.016, .024])
    assert info["model_timestep_s"] == .000025
    addr = env.nut_qpos_addr
    assert np.array_equal(env.data.qpos[addr:addr + 7], model.qpos0[addr:addr + 7])
    addr = env.block_qpos_addr
    assert np.array_equal(env.data.qpos[addr:addr + 7], model.qpos0[addr:addr + 7])
    # The same work pose as the demo centers the real right fingers at the nut.
    right_site = model.site("right_grasp_site").id
    assert np.linalg.norm(env.data.site_xpos[right_site] - env.data.xpos[env.nut_id]) < 1e-5
    left_site = model.site("left_grasp_site").id
    assert np.linalg.norm(env.data.site_xpos[left_site] - initial_left_grasp_position(env.scene_config)) < 1e-5
    assert np.allclose(env.data.site_xmat[left_site].reshape(3, 3),
                       left_grasp_rotation(env.scene_config), atol=1e-5)
    info["runtime"]["libraries"][0]["path"] = "consumer modification"
    assert env.runtime_info["libraries"][0]["path"] != "consumer modification"
    env.close()


def test_actions_drive_capped_joint_motors_and_mirrored_native_fingers(model):
    env = YamM8Env(model, gravity_compensation=False, control_dt=.000025)
    env.reset(seed=2)
    action = np.r_[np.tile([2., -2., .5, -.5, 1., -1.], 2), [-1., 1.]]
    _, _, terminated, _, info = env.step(action)
    assert not terminated, info["failure_reasons"]
    expected = np.clip(action[:12], -1., 1.) * env.torque_caps
    assert np.array_equal(env.data.ctrl[env.arm_motor_ids], expected)
    assert np.array_equal(env.data.actuator_force[env.arm_motor_ids], expected)
    assert np.all(np.abs(env.data.actuator_force[env.arm_motor_ids]) <= env.torque_caps)
    assert np.allclose(env.data.ctrl[env.finger_actuator_ids], [.0125, -.0125, .0102, -.0102])
    assert np.all(np.abs(env.data.actuator_force[env.finger_actuator_ids]) <= 20)
    assert np.all(env.data.xfrc_applied[env.nut_id] == 0)
    assert np.all(env.data.qfrc_applied[env.nut_dof_addr:env.nut_dof_addr + 6] == 0)
    assert np.all(env.data.xfrc_applied[[env.block_id, env.bolt_id]] == 0)
    assert np.all(env.data.qfrc_applied[env.block_dof_addr:env.block_dof_addr + 6] == 0)
    with pytest.raises(ValueError, match="fourteen finite"):
        env.step(np.zeros(13))
    env.close()


def test_open_finger_arm_torque_has_no_hidden_nut_drive(model):
    baseline = YamM8Env(model, gravity_compensation=False, control_dt=.0005)
    driven = YamM8Env(model, gravity_compensation=False, control_dt=.0005)
    baseline.reset(seed=3)
    driven.reset(seed=3)
    action = np.r_[np.zeros(12), [-1., -1.]]
    baseline.step(action)
    action[11] = .2
    _, _, terminated, _, info = driven.step(action)
    assert not terminated, info["failure_reasons"]
    assert info["left_pad_contact_count"] == info["right_pad_contact_count"] == 0
    addr = driven.nut_qpos_addr
    assert np.allclose(driven.data.qpos[addr:addr + 7], baseline.data.qpos[addr:addr + 7],
                       atol=1e-10, rtol=0)
    arm_dof = driven.arm_dof_indices[-1]
    assert abs(driven.data.qvel[arm_dof] - baseline.data.qvel[arm_dof]) > 1e-4
    assert np.all(driven.data.xfrc_applied[driven.nut_id] == 0)
    baseline.close()
    driven.close()


def test_invalid_plunge_cannot_earn_positive_progress_reward(model):
    env = YamM8Env(model, control_dt=.001)
    env.reset()
    # This deliberately injected simulator fault is not a policy action.
    env.data.qpos[env.nut_qpos_addr + 2] -= .003
    mujoco.mj_forward(model, env.data)
    _, reward, terminated, _, info = env.step(np.r_[np.zeros(12), [-1., -1.]])
    assert terminated and info["failure"] and not info["success"]
    assert reward == -1 and info["failure_reasons"]
    assert env.data.time <= model.opt.timestep * 1.01
    with pytest.raises(RuntimeError, match="reset"):
        env.step(np.zeros(14))
    env.close()


@pytest.mark.parametrize("fault,reason", [
    ("radial", "nut_radial_offset_above_150_um"),
    ("tilt", "nut_tilt_above_2_degrees"),
])
def test_preengaged_task_stops_outside_validated_alignment_scope(model, fault, reason):
    env = YamM8Env(model, control_dt=.001)
    env.reset()
    # Inject an out-of-scope simulator state, not a command available to a policy.
    addr = env.nut_qpos_addr
    if fault == "radial":
        env.data.qpos[addr] += 200e-6
    else:
        angle = np.deg2rad(2.5)
        env.data.qpos[addr + 3:addr + 7] = [np.cos(angle / 2), np.sin(angle / 2), 0, 0]
    mujoco.mj_forward(model, env.data)
    _, reward, terminated, _, info = env.step(np.r_[np.zeros(12), [-1., -1.]])
    assert terminated and info["failure"] and not info["success"]
    assert reward == -1 and reason in info["failure_reasons"]
    assert env.data.time <= model.opt.timestep * 1.01
    env.close()


def test_moving_bolt_relative_velocity_uses_origin_and_rigid_point_transport(model):
    env = YamM8Env(model)
    data = env.data
    block_addr, nut_addr = env.block_qpos_addr, env.nut_qpos_addr
    block_q = np.empty(4)
    mujoco.mju_euler2Quat(block_q, np.deg2rad([20., -35., 55.]), "xyz")
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, block_q)
    rotation = rotation.reshape(3, 3)
    block_position = np.array([.32, .07, .25])
    nut_local = model.body_pos[env.bolt_id] + [0, 0, env.scene_config.thread.initial_z]
    nut_position = block_position + rotation @ nut_local
    nut_yaw = np.array([np.cos(.31 / 2), 0, 0, np.sin(.31 / 2)])
    nut_q = np.empty(4)
    mujoco.mju_mulQuat(nut_q, block_q, nut_yaw)
    nut_rotation = np.empty(9)
    mujoco.mju_quat2Mat(nut_rotation, nut_q)
    nut_rotation = nut_rotation.reshape(3, 3)
    data.qpos[block_addr:block_addr + 7] = np.r_[block_position, block_q]
    data.qpos[nut_addr:nut_addr + 7] = np.r_[nut_position, nut_q]
    linear_world = np.array([.11, -.04, .03])
    angular_world = np.array([1.2, -.7, .9])
    nut_linear_world = linear_world + np.cross(angular_world, nut_position - block_position)
    data.qvel[env.block_dof_addr:env.block_dof_addr + 6] = np.r_[linear_world, rotation.T @ angular_world]
    data.qvel[env.nut_dof_addr:env.nut_dof_addr + 6] = np.r_[nut_linear_world, nut_rotation.T @ angular_world]
    mujoco.mj_forward(model, data)
    # The independently integrated bolt has a CoM offset: using mjOBJ_BODY's
    # CoM velocity instead of mjOBJ_XBODY's origin velocity would fail this test.
    assert np.linalg.norm(data.xipos[env.bolt_id] - data.xpos[env.bolt_id]) > .005
    expected_bolt_linear = linear_world + np.cross(angular_world, data.xpos[env.bolt_id] - block_position)
    assert np.allclose(env._body_velocity(env.bolt_id), np.r_[angular_world, expected_bolt_linear], atol=2e-14)
    assert np.allclose(env._nut_relative_velocity(), 0, atol=2e-14)
    relative_linear, relative_angular = np.array([.02, -.003, .011]), np.array([.05, -.07, .02])
    data.qvel[env.nut_dof_addr:env.nut_dof_addr + 3] = nut_linear_world + rotation @ relative_linear
    data.qvel[env.nut_dof_addr + 3:env.nut_dof_addr + 6] = nut_rotation.T @ (angular_world + rotation @ relative_angular)
    mujoco.mj_forward(model, data)
    assert np.allclose(env._nut_relative_velocity(), np.r_[relative_linear, relative_angular], atol=2e-14)
    env.close()


def test_grasp_support_rejects_one_pad_world_support_and_block_slip(model):
    env = YamM8Env(model)
    _, info = env.reset()
    assert not info["free_block_held_by_left_fingers"]  # reset is not settled
    assert not env._grasp_support_metrics([1., 0., 1., 1.])["left_block_grasp_secure"]
    assert not env._grasp_support_metrics([1., 1., 1., 0.])["right_nut_grasp_loaded"]
    # Adversarial force evidence cannot convert geometric slip into a secure
    # left-hand grasp, even when both pad normal channels report positive loads.
    env.data.qpos[env.block_qpos_addr] += .0011
    mujoco.mj_forward(model, env.data)
    slip = env._grasp_support_metrics([1., 1., 1., 1.])
    assert slip["block_left_grasp_position_slip_m"] > .001
    assert not slip["left_block_grasp_secure"]
    env.reset()
    angle = np.deg2rad(2.5)
    env.data.qpos[env.block_qpos_addr + 3:env.block_qpos_addr + 7] = [np.cos(angle / 2), 0, 0, np.sin(angle / 2)]
    mujoco.mj_forward(model, env.data)
    slip = env._grasp_support_metrics([1., 1., 1., 1.])
    assert slip["block_left_grasp_rotation_slip_rad"] > np.deg2rad(2.)
    assert not slip["left_block_grasp_secure"]
    # Place the free block directly on the real table without moving the arm.
    env.reset()
    env.data.qpos[env.block_qpos_addr + 2] = env.scene_config.block_size[2] / 2 - 1e-6
    mujoco.mj_forward(model, env.data)
    support = env._grasp_support_metrics([1., 1., 1., 1.])
    assert support["block_world_support_contact_count"] > 0
    assert not support["left_block_grasp_secure"]
    env.close()
