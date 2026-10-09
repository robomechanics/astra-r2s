"""Short acquisition/start interface checks, without a learned policy claim."""
import mujoco
import numpy as np
import pytest

from yam_twin.m8_insertion_env import OBSERVATION_FIELDS, YamM8InsertionEnv
from yam_twin.m8_insertion_scene import InsertionConfig, build_model, table_pickup_config
from yam_twin.m8_insertion_engagement import LoadedFlankWindow


@pytest.fixture(scope="module")
def model():
    return build_model(InsertionConfig())


@pytest.fixture(scope="module")
def table_scene():
    return table_pickup_config()


@pytest.fixture(scope="module")
def table_model(table_scene):
    return build_model(table_scene)


def test_default_task_picks_up_both_supported_workpieces_and_preserves_legacy_explicit_model(model):
    env = YamM8InsertionEnv(control_dt=.000025)
    _, info = env.reset()
    assert info["pickup_from_table"] and env.scene_config.pickup_from_table
    env.close()
    old = YamM8InsertionEnv(model)
    _, old_info = old.reset()
    assert not old_info["pickup_from_table"]
    assert old._left_reference is not None
    assert "Left touching-pad" in old_info["reset_grasp_state"]
    old.close()
    explicit_config = YamM8InsertionEnv(scene_config=InsertionConfig())
    _, explicit_info = explicit_config.reset()
    assert not explicit_info["pickup_from_table"]
    explicit_config.close()


def test_table_reset_has_open_separated_hands_and_no_invented_grasp_reference(table_model, table_scene):
    env = YamM8InsertionEnv(table_model, scene_config=table_scene, control_dt=.000050)
    obs, info = env.reset(seed=3)
    qpos = env.data.qpos.copy()
    repeated, repeated_info = env.reset(seed=3)
    assert np.array_equal(qpos, env.data.qpos) and np.array_equal(obs, repeated)
    assert obs.shape == (118,) and env.action_space.shape == (14,)
    assert env.observation_space.contains(obs)
    # Exact touching placements can have no candidate at t=0 owing to
    # roundoff. Test physical surface placement, then observed support after
    # real gravity/contact substeps; never fabricate reset contact history.
    block_p, block_r = env._free_pose(env.block_qadr)
    corners = np.asarray([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    corners = block_p+(corners*np.asarray(table_scene.base.block_size)/2)@block_r.T
    table = table_model.geom("table").id
    table_top = float(table_model.geom_pos[table, 2]+table_model.geom_size[table, 2])
    assert abs(np.min(corners[:, 2])-table_top) <= 20e-9
    assert table_scene.bolt_head_position[2]-table_scene.head_height/2 == pytest.approx(
        table_model.geom_pos[table_model.geom("bolt_rest_pin_0").id, 2]
        +table_model.geom_size[table_model.geom("bolt_rest_pin_0").id, 1], abs=1e-12)
    assert not info["left_pads_loaded"] and not info["right_pads_loaded"]
    assert not info["left_grasp_reference_observed"] and not info["right_grasp_reference_observed"]
    assert env._left_reference is None and env._right_reference is None
    for key in ("left_pad_acquisition_observed", "right_pad_acquisition_observed", "left_block_pickup_observed", "right_bolt_pickup_observed",
                "pickup_observed", "loaded_flank_engagement_candidate", "thread_started"):
        assert not info[key] and not repeated_info[key]
    assert info["left_pad_acquisition_time_s"] is None and info["left_block_pickup_time_s"] is None
    assert info["right_pad_acquisition_time_s"] is None
    assert info["right_bolt_pickup_time_s"] is None
    assert np.allclose(env.last_apertures, table_scene.base.open_aperture)
    assert np.allclose(env.data.qpos[env.finger_qpos], [.0125, -.0125, .0125, -.0125])
    for address in (env.block_qadr, env.male_qadr):
        assert np.array_equal(env.data.qpos[address:address+7], table_model.qpos0[address:address+7])
    assert np.linalg.norm(env.data.site_xpos[env.grasp_sites[0]]-
                          (env._free_pose(env.block_qadr)[0]+env._free_pose(env.block_qadr)[1]
                           @np.asarray(table_scene.base.left_grasp_offset))) >= .02
    assert np.allclose(env.data.site_xpos[env.grasp_sites[1]]-env._head_position(), [0, 0, .020], atol=2e-6)
    _, reward, terminated, _, stepped = env.step(np.r_[np.zeros(12), [-1., -1.]])
    assert not terminated, stepped["failure_reasons"]
    assert stepped["block_world_support_contacts"] > 0 and stepped["bolt_rest_support_contacts"] > 0
    assert stepped["block_world_support_observed"] and stepped["rest_support_observed"]
    assert reward == 0 and not stepped["pickup_observed"] and not stepped["success"]
    assert not stepped["left_grasp_reference_observed"]
    assert not stepped["right_grasp_reference_observed"]
    env.close()


def test_lifted_block_without_measured_pad_acquisition_cannot_earn_pickup(table_model, table_scene):
    env = YamM8InsertionEnv(table_model, scene_config=table_scene, control_dt=.000025)
    env.reset()
    # Backend fault injection lifts the free block geometrically while both
    # hands remain open and away. Height and loss of support alone are never
    # evidence that a policy acquired the workpiece.
    env.data.qpos[env.block_qadr+2] += .010
    mujoco.mj_forward(table_model, env.data)
    assert env._contacts()["block_world_support_contacts"] == 0
    _, reward, terminated, _, info = env.step(np.r_[np.zeros(12), [-1., -1.]])
    assert not terminated, info["failure_reasons"]
    assert info["block_lift_from_reset_m"] > info["minimum_block_pickup_lift_m"]
    assert not info["left_pads_loaded"] and not info["left_pad_acquisition_observed"]
    assert not info["left_block_pickup_observed"] and not info["left_grasp_reference_observed"]
    assert not info["pickup_observed"] and not info["success"] and reward == 0
    env.close()


@pytest.mark.parametrize("name", ["block", "bolt"])
def test_table_task_detects_external_workpiece_drive_before_any_acquisition(table_model, table_scene, name):
    env = YamM8InsertionEnv(table_model, scene_config=table_scene, control_dt=.001)
    env.reset()
    body = env.block_id if name == "block" else env.male_id
    env.data.xfrc_applied[body, 0] = .01
    _, reward, terminated, _, info = env.step(np.r_[np.zeros(12), [-1., -1.]])
    assert terminated and reward == -1 and not info["success"]
    assert name+"_external_drive_present" in info["failure_reasons"]
    assert not info["pickup_observed"]
    assert env.data.time <= table_model.opt.timestep*1.01
    env.close()


def test_reset_is_unengaged_hover_and_preserves_both_free_workpieces(model):
    env = YamM8InsertionEnv(model, control_dt=.000025)
    obs, info = env.reset(seed=4)
    qpos = env.data.qpos.copy()
    repeated, _ = env.reset(seed=4)
    assert np.array_equal(qpos, env.data.qpos) and np.array_equal(obs, repeated)
    assert obs.shape == (118,) == (sum(OBSERVATION_FIELDS.values()),)
    assert env.observation_space.contains(obs) and np.isfinite(obs).all()
    assert env.action_space.shape == (14,) and info["privileged_state_observations"]
    assert not info["starts_preengaged"] and not info["thread_started"]
    assert not info["pickup_observed"] and info["thread_contact_count"] == 0
    assert not info["right_pads_loaded"]
    for address in (env.block_qadr, env.male_qadr):
        assert np.array_equal(env.data.qpos[address:address+7], model.qpos0[address:address+7])
    right = model.site("right_grasp_site").id
    assert np.allclose(env.data.site_xpos[right]-env._head_position(), [0, 0, .020], atol=2e-6)
    env.close()


def test_actual_motor_and_finger_actions_are_bounded_and_do_not_drive_free_bodies(model):
    env = YamM8InsertionEnv(model, control_dt=.000025, gravity_compensation=False)
    env.reset()
    action = np.r_[np.tile([2., -2., .5, -.5, 1., -1.], 2), [-1., 1.]]
    _, _, terminated, _, info = env.step(action)
    assert not terminated, info["failure_reasons"]
    expected = np.clip(action[:12], -1, 1)*env.torque_caps
    assert np.array_equal(env.data.ctrl[env.arm_motors], expected)
    assert np.array_equal(env.data.actuator_force[env.arm_motors], expected)
    assert np.allclose(env.data.ctrl[env.finger_actuators], [.0125, -.0125, .0102, -.0102])
    assert np.all(np.abs(env.data.actuator_force[env.finger_actuators]) <= 20.)
    assert np.all(env.data.xfrc_applied[[env.block_id, env.male_id, env.female_id]] == 0)
    for address in (env.block_dadr, env.male_dadr):
        assert np.all(env.data.qfrc_applied[address:address+6] == 0)
    with pytest.raises(ValueError, match="fourteen finite"):
        env.step(np.zeros(13))
    env.close()


def test_projected_depth_away_from_hole_is_not_progress_or_success(model):
    env = YamM8InsertionEnv(model, control_dt=.0005)
    _, initial = env.reset()
    # The bolt on its offset rest projects more than a pitch below the entry
    # plane. This must not be confused with insertion into the distant hole.
    assert initial["tip_depth_from_entry_m"] > env.pitch
    assert initial["axis_radial_offset_at_entry_m"] > .05
    _, reward, terminated, _, info = env.step(np.r_[np.zeros(12), [1., -1.]])
    assert not terminated and not info["success"] and not info["thread_started"]
    assert info["thread_contact_count"] == 0 and info["observed_engaged_advance_m"] == 0
    assert reward == 0
    env.close()


def test_entry_cone_contacts_are_not_loaded_full_flank_engagement(model):
    env = YamM8InsertionEnv(model)
    env.reset()
    female_p, female_r = env._female_pose()
    male_q = np.empty(4)
    mujoco.mju_mat2Quat(male_q, female_r.ravel())
    # Test-only geometric fault: a 0.6mm lateral offset at shallow entry creates
    # real cone/chamfer contacts, while no fully formed flank ring can overlap.
    origin_z = .001-env.bolt_length-env.hole_height/2
    env.data.qpos[env.male_qadr:env.male_qadr+7] = np.r_[
        female_p+female_r@np.array([.0006, 0, origin_z]), male_q]
    mujoco.mj_forward(model, env.data)
    contacts = env._contacts()
    assert contacts["thread_count"] > 0
    assert contacts["loaded_full_flank_contact_count"] == 0
    metrics = env._metrics(contacts)
    assert metrics["whole_ring_full_flank_length_m"] == 0
    assert not metrics["thread_started"] and not metrics["loaded_flank_engagement_candidate"]
    env.close()


@pytest.mark.parametrize("name", ["block", "bolt"])
def test_external_workpiece_drive_is_detected_as_simulator_fault(model, name):
    env = YamM8InsertionEnv(model, control_dt=.001)
    env.reset()
    body = env.block_id if name == "block" else env.male_id
    # Deliberate backend fault injection; no policy action exposes this wrench.
    env.data.xfrc_applied[body, 0] = .01
    _, reward, terminated, _, info = env.step(np.r_[np.zeros(12), [1., -1.]])
    assert terminated and reward == -1 and not info["success"]
    assert name+"_external_drive_present" in info["failure_reasons"]
    assert env.data.time <= model.opt.timestep*1.01
    env.close()


def test_relative_velocity_accounts_for_moving_female_frame_and_bolt_com(model):
    env = YamM8InsertionEnv(model)
    d = env.data
    q = np.empty(4)
    mujoco.mju_euler2Quat(q, np.deg2rad([20., -35., 55.]), "xyz")
    r = np.empty(9)
    mujoco.mju_quat2Mat(r, q)
    r = r.reshape(3, 3)
    p = np.array([.3, .05, .35])
    d.qpos[env.block_qadr:env.block_qadr+7] = np.r_[p, q]
    female_p, female_r = env._female_pose()
    male_p = female_p+female_r@np.array([.05, .01, -.03])
    male_q = np.empty(4)
    mujoco.mju_mat2Quat(male_q, female_r.ravel())
    d.qpos[env.male_qadr:env.male_qadr+7] = np.r_[male_p, male_q]
    v, omega = np.array([.11, -.04, .03]), np.array([1.2, -.7, .9])
    male_v = v+np.cross(omega, male_p-p)
    d.qvel[env.block_dadr:env.block_dadr+6] = np.r_[v, r.T@omega]
    d.qvel[env.male_dadr:env.male_dadr+6] = np.r_[male_v, female_r.T@omega]
    mujoco.mj_forward(model, d)
    assert np.linalg.norm(d.xipos[env.male_id]-d.xpos[env.male_id]) > .001
    assert np.allclose(env._relative_velocity(), 0, atol=2e-14)
    dv, dw = np.array([.02, -.003, .011]), np.array([.05, -.07, .02])
    d.qvel[env.male_dadr:env.male_dadr+3] = male_v+female_r@dv
    d.qvel[env.male_dadr+3:env.male_dadr+6] = female_r.T@omega+dw
    mujoco.mj_forward(model, d)
    assert np.allclose(env._relative_velocity(), np.r_[dv, dw], atol=2e-14)
    env.close()


def test_shared_engagement_observer_receives_only_real_physics_substeps(model):
    class RecordedWindow(LoadedFlankWindow):
        def __init__(self):
            super().__init__()
            self.samples = []

        def update(self, *sample):
            self.samples.append(sample)
            return super().update(*sample)

    env = YamM8InsertionEnv(model, control_dt=4*model.opt.timestep)
    _, reset_info = env.reset()
    assert isinstance(env.engagement_observer, LoadedFlankWindow)
    settings = reset_info["engagement_observer"]
    assert settings["version"] == "loaded_full_flank_window_v1"
    assert settings["duration_s"] == .2
    assert settings["minimum_normal_impulse_Ns"] == .001
    assert settings["minimum_loaded_duration_s"] == .0005
    assert settings["maximum_helix_phase_range_m"] == 150e-6
    assert reset_info["engagement_window"]["sample_count"] == 0
    observer = RecordedWindow()
    env.engagement_observer = observer
    _, _, terminated, _, info = env.step(np.r_[np.zeros(12), [1., -1.]])
    assert not terminated and len(observer.samples) == env.frame_skip == 4
    assert np.allclose([s[0] for s in observer.samples], np.arange(1, 5)*model.opt.timestep)
    assert all(s[1] == model.opt.timestep for s in observer.samples)
    assert all(not s[2] and s[3] == 0 and s[4] == 0 for s in observer.samples)
    expected_phase = env._relative_pose()[0][2]-env.pitch*env._yaw_total/(2*np.pi)
    assert observer.samples[-1][5] == expected_phase
    assert not info["loaded_flank_engagement_candidate"] and not info["thread_started"]
    info["engagement_window"]["ready"] = True
    assert not env._engagement_metrics["ready"]
    env.close()


@pytest.mark.parametrize("bound,sign", [(0, -1), (1, 1)])
def test_actual_native_joint_bounds_stop_out_of_range_policy_state(model, bound, sign):
    env = YamM8InsertionEnv(model, control_dt=.001)
    _, info = env.reset()
    assert info["minimum_native_joint_margin_rad"] >= 0
    joint, address = env.arm_joints[-1], env.arm_qpos[-1]
    limit = model.jnt_range[joint, bound]
    # Native soft limits can have tiny solver excursions. The demo and policy
    # interface permit the same 10-microradian tolerance, not an unbounded wrist.
    env.data.qpos[address] = limit+sign*5e-6
    mujoco.mj_forward(model, env.data)
    assert "native_arm_joint_range_violation" not in env._failures(env._contacts())
    assert env._info()["minimum_native_joint_margin_rad"] == pytest.approx(-5e-6)
    # Test-only simulator fault at the actual compiled bound; no policy action
    # assigns joint positions or changes those limits.
    env.data.qpos[address] = limit+sign*1e-4
    mujoco.mj_forward(model, env.data)
    _, reward, terminated, _, info = env.step(np.r_[np.zeros(12), [1., -1.]])
    assert terminated and reward == -1 and not info["success"]
    assert "native_arm_joint_range_violation" in info["failure_reasons"]
    assert info["minimum_native_joint_margin_rad"] < -1e-5
    assert info["episode_minimum_native_joint_margin_rad"] <= info["minimum_native_joint_margin_rad"]
    assert info["native_joint_limit_violation_tolerance_rad"] == 1e-5
    assert env.data.time <= model.opt.timestep*1.01
    env.close()
