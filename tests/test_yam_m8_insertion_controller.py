"""Safety-critical reset and support accounting for actual bolt acquisition."""
import mujoco
import numpy as np
import pytest

from yam_twin.m8_insertion_scene import InsertionConfig, build_model
from yam_twin.m8_insertion_simulation import (
    InsertionControlConfig, initialize_insertion_pose, insertion_phases, _body_contact,
    PadLoadHistory, _pad_normal_forces, _relative_axial_velocity, EntrySupportWindow,
)


def test_pickup_reset_preserves_both_free_body_states():
    scene = InsertionConfig()
    model = build_model(scene)
    data = mujoco.MjData(model)
    free_joints = [model.joint(name).id for name in
                   ("fixture_block_free", "male_bolt_free")]
    q_indices = np.concatenate([np.arange(model.jnt_qposadr[j], model.jnt_qposadr[j]+7)
                                for j in free_joints])
    v_indices = np.concatenate([np.arange(model.jnt_dofadr[j], model.jnt_dofadr[j]+6)
                                for j in free_joints])
    # Nonzero free-body velocity catches reset helpers that silently stabilize
    # the workpiece while initializing the arms.
    data.qvel[v_indices] = np.linspace(-.013, .019, 12)
    q_before, v_before = data.qpos[q_indices].copy(), data.qvel[v_indices].copy()
    targets, pickup, _ = initialize_insertion_pose(model, data, scene, InsertionControlConfig())
    np.testing.assert_array_equal(data.qpos[q_indices], q_before)
    np.testing.assert_array_equal(data.qvel[v_indices], v_before)
    np.testing.assert_allclose(targets["right"][0]-pickup, [0., 0., .020])
    for side in ("left", "right"):
        actual = data.site_xpos[model.site(f"{side}_grasp_site").id]
        np.testing.assert_allclose(actual, targets[side][0], atol=2e-6)
    np.testing.assert_array_equal(data.xfrc_applied, 0.)
    np.testing.assert_array_equal(data.qfrc_applied, 0.)


def test_support_accounting_includes_rigid_child_and_fixed_body_counterpart():
    # The workpiece's collision lives on a rigid frame child, and the fixed
    # support lives on a named body rather than body0. Both must be accounted.
    model = mujoco.MjModel.from_xml_string("""
      <mujoco><worldbody>
        <body name="named_fixed_support"><geom type="plane" size="1 1 .01"/></body>
        <body name="free_block" pos="0 0 .1"><freejoint/>
          <inertial mass=".1" pos="0 0 0" diaginertia=".001 .001 .001"/>
          <body name="female_frame" pos="0 0 -.1">
            <geom type="box" size=".01 .01 .005" mass="0"/>
          </body>
        </body>
      </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    root = _body_contact(model, data, model.body("free_block").id)
    child = _body_contact(model, data, model.body("female_frame").id)
    assert root == child
    assert root["world_support_contacts"] > 0
    assert root["contact_count"] > 0


def test_table_pickup_reset_starts_open_and_preserves_free_body_states():
    from yam_twin.m8_insertion_scene import table_pickup_config, initial_left_grasp_position
    from yam_twin.m8_simulation import YamCartesianController
    scene, control = table_pickup_config(), InsertionControlConfig()
    model = build_model(scene)
    data = mujoco.MjData(model)
    free_joints = [model.joint(name).id for name in
                   ("fixture_block_free", "male_bolt_free")]
    q_indices = np.concatenate([np.arange(model.jnt_qposadr[j], model.jnt_qposadr[j]+7)
                                for j in free_joints])
    v_indices = np.concatenate([np.arange(model.jnt_dofadr[j], model.jnt_dofadr[j]+6)
                                for j in free_joints])
    data.qvel[v_indices] = np.linspace(-.013, .019, 12)
    q_before, v_before = data.qpos[q_indices].copy(), data.qvel[v_indices].copy()
    targets, pickup, _ = initialize_insertion_pose(model, data, scene, control)
    np.testing.assert_array_equal(data.qpos[q_indices], q_before)
    np.testing.assert_array_equal(data.qvel[v_indices], v_before)
    left_r = targets["left"][1]
    np.testing.assert_allclose(targets["left"][0]-initial_left_grasp_position(scene),
                               -left_r[:, 2]*control.left_pickup_hover_m, atol=1e-12)
    np.testing.assert_allclose(targets["right"][0]-pickup, [0., 0., control.pickup_hover_m])
    for side, body in (("left", "fixture_block"), ("right", "male_bolt")):
        controller = YamCartesianController(model, data, side, control.arm, scene.base)
        np.testing.assert_allclose(controller.pose()[0], targets[side][0], atol=2e-6)
        assert _body_contact(model, data, model.body(body).id,
                             controller.hand_geom_ids)["contact_count"] == 0
        np.testing.assert_allclose(data.ctrl[controller.finger_ids],
                                  [(control.arm.open_aperture+2*scene.base.pad_inner_offset)/2,
                                   -(control.arm.open_aperture+2*scene.base.pad_inner_offset)/2])
    assert _body_contact(model, data, model.body("fixture_block").id)["world_support_contacts"] > 0
    np.testing.assert_array_equal(data.xfrc_applied, 0.)
    np.testing.assert_array_equal(data.qfrc_applied, 0.)


def test_table_pickup_schedule_adds_acquisition_before_unchanged_bolt_schedule():
    control = InsertionControlConfig()
    old = insertion_phases(control)
    table = insertion_phases(control, pickup_from_table=True)
    expected = ["settle_table", "reach_left_block", "close_left_block",
                "settle_left_block", "lift_left_block", "transport_left_block",
                "hold_left_block"]
    assert [phase[0] for phase in table[:len(expected)]] == expected
    assert table[len(expected):] == old
    assert all(not phase[-1] for phase in table[:len(expected)])


@pytest.mark.parametrize("field", ["left_pickup_hover_m", "left_lift_clearance_m"])
def test_table_pickup_clearances_require_positive_finite_values(field):
    with pytest.raises(ValueError, match="positive"):
        InsertionControlConfig(**{field: 0.})


def test_all_substep_pad_history_detects_force_gap_between_saved_pose_samples():
    history = PadLoadHistory()
    # The usual saved-pose period is 5 ms. A real 100 us unilateral force gap
    # between snapshots must still fail continuous bilateral retention.
    for step in range(100):
        history.observe([16., 0.] if step in (37, 38) else [16., 16.], .00005)
    report = history.report()
    assert not report["continuously_bilateral_loaded"]
    assert report["unloaded_physics_steps"] == [0, 2]
    np.testing.assert_allclose(report["unloaded_duration_s"], [0., .0001])
    np.testing.assert_allclose(report["maximum_consecutive_unloaded_duration_s"], [0., .0001])
    assert report["minimum_normal_force_N"] == [16., 0.]
    assert report["observed_physics_steps"] == 100
    np.testing.assert_allclose(report["observed_duration_s"], .005)


def test_all_substep_pad_force_sums_rigid_frame_child_contact():
    model = mujoco.MjModel.from_xml_string("""
      <mujoco><worldbody>
        <geom name="left_pad" type="plane" size="1 1 .01"/>
        <geom name="right_pad" type="sphere" pos="0 0 1" size=".01"/>
        <body name="free_block" pos="0 0 .104"><freejoint/>
          <inertial mass=".1" pos="0 0 0" diaginertia=".001 .001 .001"/>
          <body name="female_frame" pos="0 0 -.1">
            <geom type="box" size=".01 .01 .005" mass="0"/>
          </body>
        </body>
      </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    body = model.body("free_block").id
    pads = [model.geom(name).id for name in ("left_pad", "right_pad")]
    forces = _pad_normal_forces(model, data, body, pads)
    assert forces[0] > .1
    assert forces[1] == 0.
    np.testing.assert_allclose(forces[0], _body_contact(model, data, body, {pads[0]})["normal_force_N"])
    observation = _pad_normal_forces(model, data, body, pads, with_compliance=True)
    np.testing.assert_allclose(observation["normal_force_N"], forces)
    np.testing.assert_allclose(observation["loaded_force_weighted_indentation_m"][0], .001)
    np.testing.assert_allclose(observation["effective_pad_force_per_indentation_N_per_m"][0],
                               forces[0]/.001)
    assert observation["loaded_force_weighted_indentation_m"][1] is None


def test_relative_axial_velocity_accounts_for_rotating_and_translating_hole():
    hand = np.array([.3, -.1, .2])
    hole = np.array([.2, -.2, .1])
    hole_velocity = np.array([.7, -.3, .5, .04, -.02, .01])
    axis = np.array([0., 0., -1.])
    comoving = hole_velocity[3:]+np.cross(hole_velocity[:3], hand-hole)
    np.testing.assert_allclose(_relative_axial_velocity(hand, comoving, hole,
                                                       hole_velocity, axis), 0., atol=1e-15)
    for speed in (-.04, .01):
        measured = _relative_axial_velocity(hand, comoving+speed*axis, hole,
                                           hole_velocity, axis)
        np.testing.assert_allclose(measured, speed)
        damping_force = -50.*measured
        assert damping_force*measured < 0.  # Relative axial mechanical power.
        assert np.sign(damping_force) == -np.sign(speed)


@pytest.mark.parametrize("value", [-1., np.inf, np.nan])
def test_axial_velocity_damping_rejects_invalid_coefficients(value):
    assert InsertionControlConfig().axial_velocity_damping_Ns_per_m == 0.
    with pytest.raises(ValueError, match="nonnegative and finite"):
        InsertionControlConfig(axial_velocity_damping_Ns_per_m=value)


def test_damped_feed_uses_finite_arm_motors_without_rewriting_free_objects():
    from yam_twin.m8_insertion_scene import table_pickup_config
    from yam_twin.m8_simulation import YamCartesianController
    scene = table_pickup_config()
    control = InsertionControlConfig(axial_velocity_damping_Ns_per_m=50.)
    model = build_model(scene)
    data = mujoco.MjData(model)
    initialize_insertion_pose(model, data, scene, control)
    controller = YamCartesianController(model, data, "right", control.arm, scene.base)
    data.qvel[controller.dof_indices] = np.linspace(-.1, .1, 6)
    mujoco.mj_forward(model, data)
    free_joints = [model.joint(name).id for name in
                   ("fixture_block_free", "male_bolt_free")]
    q_indices = np.concatenate([np.arange(model.jnt_qposadr[j], model.jnt_qposadr[j]+7)
                                for j in free_joints])
    v_indices = np.concatenate([np.arange(model.jnt_dofadr[j], model.jnt_dofadr[j]+6)
                                for j in free_joints])
    q_before, v_before = data.qpos[q_indices].copy(), data.qvel[v_indices].copy()
    hole_id = model.body("female_frame").id
    axis = data.xmat[hole_id].reshape(3, 3)[:, 2]
    hand_velocity, hole_velocity = np.zeros(6), np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE,
                            controller.site_id, hand_velocity, 0)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY,
                            hole_id, hole_velocity, 0)
    p, r = controller.pose()
    speed = _relative_axial_velocity(p, hand_velocity[3:], data.xpos[hole_id],
                                    hole_velocity, axis)
    # A deliberately over-cap nominal request demonstrates that damping uses
    # the existing bounded motor path, including its unchanged force limits.
    controller.command(p, r, control.arm.closed_aperture, axial_float=True,
                       axis_world=axis, axial_feed_N=100.-50.*speed)
    assert np.linalg.norm(controller.last_wrench[:3]) <= control.arm.maximum_cartesian_force+1e-12
    assert np.all(np.abs(controller.last_motor_torques) <= controller.torque_caps)
    np.testing.assert_array_equal(data.qpos[q_indices], q_before)
    np.testing.assert_array_equal(data.qvel[v_indices], v_before)
    np.testing.assert_array_equal(data.xfrc_applied, 0.)
    np.testing.assert_array_equal(data.qfrc_applied, 0.)


@pytest.mark.parametrize("table, explicit_zero, expected", [(False, False, 0.),
                                                          (True, False, 50.),
                                                          (True, True, 0.)])
def test_demo_default_damping_preserves_legacy_and_explicit_configuration(tmp_path,
                                                                       table, explicit_zero,
                                                                       expected):
    from yam_twin.m8_insertion_scene import table_pickup_config
    from yam_twin.m8_insertion_simulation import run_insertion_demo
    # Persist the resolved configuration/source archive without integrating a
    # physics step. This verifies runtime selection, including explicit zero.
    scene = table_pickup_config() if table else InsertionConfig()
    control = InsertionControlConfig() if explicit_zero else None
    result = run_insertion_demo(tmp_path, scene_config=scene, control_config=control,
                                maximum_phases=0)
    assert result["control_config"]["axial_velocity_damping_Ns_per_m"] == expected
    assert result["control_config"]["maximum_entry_dwell_s"] == (3. if table and not explicit_zero else None)
    assert result["control_config"]["starting_angular_speed_rad_s"] == (.5 if table and not explicit_zero else None)


def test_entry_window_needs_native_load_and_settled_aligned_bolt_motion():
    observer = EntrySupportWindow(.05, .0002, .05)
    for _ in range(100):
        assert not observer.observe(0., 0., .001, True)
    # A cone impact with actual bolt motion must not authorize jaw opening.
    for _ in range(100):
        assert not observer.observe(.2, .001, .001, True)
    for _ in range(100):
        assert not observer.observe(.05, 0., .001, False)
    # Resolved unilateral contact gaps remain explicitly observable. The
    # acquisition window requires load impulse/duration and load at release,
    # without inventing continuous contact or formed-flank engagement.
    for step in range(51):
        ready = observer.observe(0. if step%5 == 1 else .05, .00001, .001, True)
        if step < 49:
            assert not ready
    assert observer.ready
    report = observer.report()
    assert .7 < report["loaded_substep_duty"] < .9
    assert report["normal_impulse_Ns"] >= report["minimum_normal_impulse_Ns"]
    assert report["final_native_normal_force_N"] == .05
    assert report["relative_bolt_axial_velocity_range_m_per_s"] == [.00001, .00001]
    assert not observer.observe(0., .00001, .001, True)
    assert not observer.observe(.05, .001, .001, True)
    assert observer.report()["observed_window_s"] == 0.


@pytest.mark.parametrize("field,value", [("maximum_entry_dwell_s", .01),
                                        ("maximum_entry_dwell_s", np.inf),
                                        ("entry_support_window_s", 0.),
                                        ("entry_support_velocity_limit_m_per_s", np.nan),
                                        ("starting_angular_speed_rad_s", 0.),
                                        ("starting_angular_speed_rad_s", np.inf)])
def test_entry_and_starting_profile_reject_invalid_configuration(field, value):
    with pytest.raises(ValueError):
        InsertionControlConfig(**{field: value})


def test_bounded_entry_holds_closed_jaws_and_starting_speed_matches_yaw_profile():
    from dataclasses import replace
    from yam_twin.m8_simulation import smooth_profile
    control = InsertionControlConfig(arm=replace(InsertionControlConfig().arm,
                                                 angular_speed_rad_s=2., stroke_angle_rad=np.pi),
        maximum_entry_dwell_s=3., starting_angular_speed_rad_s=.5)
    phases = insertion_phases(control, pickup_from_table=True)
    for name, duration, a0, a1, gap0, gap1, floating in phases:
        if name == "feed_to_entry" or name.startswith("stop_start_"):
            assert duration == 3.
            assert gap0 == gap1 == control.arm.closed_aperture
            assert floating
        if name.startswith(("start_thread_", "turn_")):
            expected_speed = .5 if name.startswith("start_thread_") else 2.
            _, peak_derivative = smooth_profile(.5)
            np.testing.assert_allclose(peak_derivative*(a1-a0)/duration, expected_speed)


def test_native_entry_timeout_aborts_before_unsupported_jaw_opening(tmp_path, monkeypatch):
    from dataclasses import replace
    import yam_twin.m8_insertion_simulation as simulation
    # A supported bolt far above the hole cannot acquire native thread load
    # during this short force-only dwell. Exercise the actual motor loop and
    # its failure path, with the next scheduled phase explicitly opening jaws.
    scene = replace(InsertionConfig(), bolt_head_position=(.36, -.15, .14))
    control = InsertionControlConfig(maximum_entry_dwell_s=.12,
                                    axial_velocity_damping_Ns_per_m=50.)

    def short_schedule(config, *, pickup_from_table=False):
        c = config.arm
        return [("feed_to_entry", .12, 0., 0., c.closed_aperture, c.closed_aperture, True),
                ("release_search_2", .15, 0., 0., c.closed_aperture, c.open_aperture, False)]

    monkeypatch.setattr(simulation, "insertion_phases", short_schedule)
    result = simulation.run_insertion_demo(tmp_path, scene_config=scene, control_config=control)
    assert result["aborted"]["phase"] == "feed_to_entry"
    assert "closed grasp retained" in result["aborted"]["reason"]
    assert [p["phase"] for p in result["phases"]] == ["feed_to_entry"]
    assert not result["acceptance_checks"]["native_cone_entry_acquired_before_rotation"]["passed"]
    saved = np.load(tmp_path/"insertion_trace.npz")
    model = build_model(scene)
    fingers = [model.actuator(f"right_grip_{side}").id for side in ("left", "right")]
    from yam_twin.m8_scene import jaw_positions
    np.testing.assert_allclose(saved["controller"][-1, fingers],
                               jaw_positions(control.arm.closed_aperture, scene.base))
    assert result["acceptance_checks"]["free_objects_have_no_external_drive"]["passed"]
