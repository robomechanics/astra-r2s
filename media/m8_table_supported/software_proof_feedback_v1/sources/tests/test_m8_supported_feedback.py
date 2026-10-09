"""Physical head feedback, frozen OPEN calibration, and impulse-seat contracts."""

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from yam_twin import m8_supported_start as candidate


def rotation(angles):
    return Rotation.from_euler("xyz", angles).as_matrix()


@pytest.fixture
def closed_inputs():
    tool_R = rotation([.37, -.29, .18])
    grip_R = rotation([-.21, .16, .31])
    tool_p = np.array([.33, -.07, .041])
    grip_p = np.array([.0007, -.0004, .0021])
    return dict(tool_p=tool_p, tool_R=tool_R,
        head_p=tool_p+tool_R@grip_p, bolt_R=tool_R@grip_R,
        hole_p=np.array([.35, -.01, .008]), hole_R=rotation([.14, -.11, .43]),
        desired_head_yaw_rad=.61, head_anchor_z_m=-.03,
        yaw_speed_rad_s=.7, hole_velocity_6=np.array([.03, -.02, .04, .001, -.002, .0003]))


@pytest.fixture
def open_inputs():
    return dict(hole_p=np.array([.35, -.01, .008]), hole_R=rotation([.14, -.11, .43]),
        hole_velocity_6=np.array([.03, -.02, .04, .001, -.002, .0003]),
        release_relative_tool_p=np.array([.0002, -.0003, -.032]),
        release_relative_tool_R=rotation([.12, -.08, -.63]),
        release_head_tool_p=np.array([.0007, -.0004, .0021]),
        delta_clock_rad=.54, clock_speed_rad_s=.8)


def test_closed_target_recomposes_the_physical_head_pose(closed_inputs):
    inputs = closed_inputs
    output = candidate.closed_head_target(**inputs)
    grip_R = inputs['tool_R'].T@inputs['bolt_R']
    grip_p = inputs['tool_R'].T@(inputs['head_p']-inputs['tool_p'])
    desired_head_R = inputs['hole_R']@Rotation.from_euler('z', inputs['desired_head_yaw_rad']).as_matrix()
    desired_head_p = inputs['hole_p']+inputs['hole_R'][:, 2]*inputs['head_anchor_z_m']
    np.testing.assert_allclose(output['rotation']@grip_R, desired_head_R, atol=1e-14)
    np.testing.assert_allclose(output['position_m']+output['rotation']@grip_p, desired_head_p, atol=1e-14)
    np.testing.assert_allclose(output['measured_head_to_tool_R'], grip_R, atol=1e-14)
    np.testing.assert_allclose(output['measured_head_to_tool_p'], grip_p, atol=1e-14)


def test_explicit_smooth_head_orientation_recomposes_without_forcing_immediate_bore_alignment(closed_inputs):
    inputs = closed_inputs
    scheduled_head_R = rotation([-.19, .23, -.51])
    output = candidate.closed_head_target(**inputs, desired_head_rotation=scheduled_head_R)
    actual_grip_R = inputs['tool_R'].T@inputs['bolt_R']
    actual_grip_p = inputs['tool_R'].T@(inputs['head_p']-inputs['tool_p'])
    np.testing.assert_allclose(output['rotation']@actual_grip_R, scheduled_head_R, atol=1e-14)
    anchor = inputs['hole_p']+inputs['hole_R'][:, 2]*inputs['head_anchor_z_m']
    np.testing.assert_allclose(output['position_m']+output['rotation']@actual_grip_p, anchor, atol=1e-14)
    # During a finite alignment ramp the scheduled physical head can retain
    # tilt. The helper must not silently snap to the bore-aligned yaw frame.
    assert np.linalg.norm(scheduled_head_R[:, 2]-inputs['hole_R'][:, 2]) > .1


def test_explicit_head_orientation_is_covariant_under_rigid_world_frame_change(closed_inputs):
    inputs = closed_inputs
    scheduled_head_R = rotation([-.19, .23, -.51])
    baseline = candidate.closed_head_target(**inputs, desired_head_rotation=scheduled_head_R)
    world_R, translation = rotation([-.36, .24, -.71]), np.array([.4, -.2, .3])
    changed = dict(inputs)
    for name in ('tool_p', 'head_p', 'hole_p'):
        changed[name] = world_R@inputs[name]+translation
    for name in ('tool_R', 'bolt_R', 'hole_R'):
        changed[name] = world_R@inputs[name]
    changed['hole_velocity_6'] = np.r_[world_R@inputs['hole_velocity_6'][:3], world_R@inputs['hole_velocity_6'][3:]]
    output = candidate.closed_head_target(**changed, desired_head_rotation=world_R@scheduled_head_R)
    np.testing.assert_allclose(output['position_m'], world_R@baseline['position_m']+translation, atol=1e-14)
    np.testing.assert_allclose(output['rotation'], world_R@baseline['rotation'], atol=1e-14)
    for name in ('linear_velocity_m_per_s', 'angular_velocity_rad_per_s'):
        np.testing.assert_allclose(output[name], world_R@baseline[name], atol=1e-14)


@pytest.mark.parametrize('bad_rotation', [
    np.full((3, 3), np.nan), np.diag([1., 1., -1.]), np.eye(3)*1.01,
    np.eye(2), np.inf,
])
def test_explicit_head_orientation_requires_a_finite_proper_rotation(closed_inputs, bad_rotation):
    with pytest.raises(ValueError, match='proper orthonormal rotation'):
        candidate.closed_head_target(**closed_inputs, desired_head_rotation=bad_rotation)


def test_closed_target_is_covariant_under_a_rigid_world_frame_change(closed_inputs):
    inputs = closed_inputs
    baseline = candidate.closed_head_target(**inputs)
    world_R, translation = rotation([-.36, .24, -.71]), np.array([.4, -.2, .3])
    changed = dict(inputs)
    for name in ('tool_p', 'head_p', 'hole_p'):
        changed[name] = world_R@inputs[name]+translation
    for name in ('tool_R', 'bolt_R', 'hole_R'):
        changed[name] = world_R@inputs[name]
    changed['hole_velocity_6'] = np.r_[world_R@inputs['hole_velocity_6'][:3], world_R@inputs['hole_velocity_6'][3:]]
    output = candidate.closed_head_target(**changed)
    np.testing.assert_allclose(output['position_m'], world_R@baseline['position_m']+translation, atol=1e-14)
    np.testing.assert_allclose(output['rotation'], world_R@baseline['rotation'], atol=1e-14)
    for name in ('linear_velocity_m_per_s', 'angular_velocity_rad_per_s'):
        np.testing.assert_allclose(output[name], world_R@baseline[name], atol=1e-14)


def test_frozen_open_target_is_covariant_under_a_rigid_world_frame_change(open_inputs):
    inputs = open_inputs
    baseline = candidate.frozen_open_target(**inputs)
    world_R, translation = rotation([-.36, .24, -.71]), np.array([.4, -.2, .3])
    changed = {**inputs, 'hole_p': world_R@inputs['hole_p']+translation,
        'hole_R': world_R@inputs['hole_R'],
        'hole_velocity_6': np.r_[world_R@inputs['hole_velocity_6'][:3], world_R@inputs['hole_velocity_6'][3:]]}
    output = candidate.frozen_open_target(**changed)
    np.testing.assert_allclose(output['position_m'], world_R@baseline['position_m']+translation, atol=1e-14)
    np.testing.assert_allclose(output['rotation'], world_R@baseline['rotation'], atol=1e-14)
    for name in ('linear_velocity_m_per_s', 'angular_velocity_rad_per_s'):
        np.testing.assert_allclose(output[name], world_R@baseline[name], atol=1e-14)


@pytest.mark.parametrize('which', ['closed', 'open'])
def test_scheduled_target_velocity_matches_independent_centered_difference(
    which, closed_inputs, open_inputs,
):
    original = closed_inputs if which == 'closed' else open_inputs
    function = candidate.closed_head_target if which == 'closed' else candidate.frozen_open_target
    clock_name = 'desired_head_yaw_rad' if which == 'closed' else 'delta_clock_rad'
    speed_name = 'yaw_speed_rad_s' if which == 'closed' else 'clock_speed_rad_s'
    hv = original['hole_velocity_6']
    # Only scheduled yaw and the hole move in this derivative check; the
    # measured closed grip/frozen OPEN calibration stays fixed. This does
    # not claim feedforward compensation for changing grasp measurements.
    def at_time(t):
        inputs = dict(original)
        inputs['hole_p'] = original['hole_p']+hv[3:]*t
        inputs['hole_R'] = Rotation.from_rotvec(hv[:3]*t).as_matrix()@original['hole_R']
        inputs[clock_name] = original[clock_name]+original[speed_name]*t
        return function(**inputs)
    epsilon = 1e-6
    baseline, before, after = at_time(0.), at_time(-epsilon), at_time(epsilon)
    linear = (after['position_m']-before['position_m'])/(2*epsilon)
    angular = Rotation.from_matrix(after['rotation']@before['rotation'].T).as_rotvec()/(2*epsilon)
    np.testing.assert_allclose(baseline['linear_velocity_m_per_s'], linear, atol=1e-9)
    np.testing.assert_allclose(baseline['angular_velocity_rad_per_s'], angular, atol=1e-9)


def test_open_release_calibration_preserves_head_anchor_while_tool_rotates(open_inputs):
    inputs = open_inputs
    anchor = inputs['hole_p']+inputs['hole_R']@(
        inputs['release_relative_tool_p']+inputs['release_relative_tool_R']@inputs['release_head_tool_p'])
    initial_tool_p = inputs['hole_p']+inputs['hole_R']@inputs['release_relative_tool_p']
    for clock in (0., .4, 1.7, 2*np.pi, 12*np.pi):
        output = candidate.frozen_open_target(**{**inputs, 'delta_clock_rad': clock})
        np.testing.assert_allclose(output['position_m']+output['rotation']@inputs['release_head_tool_p'], anchor, atol=1e-14)
        assert np.linalg.norm(output['position_m']-initial_tool_p) <= 2*np.linalg.norm(inputs['release_head_tool_p'])+1e-14
        if clock == 0.:
            np.testing.assert_allclose(output['position_m'], initial_tool_p, atol=1e-14)
            np.testing.assert_allclose(output['rotation'], inputs['hole_R']@inputs['release_relative_tool_R'], atol=1e-14)
    # There is no current-head pose input during OPEN. Repeating the same
    # clock with the frozen release geometry must never accumulate drift.
    a = candidate.frozen_open_target(**inputs)
    b = candidate.frozen_open_target(**inputs)
    np.testing.assert_array_equal(a['position_m'], b['position_m'])


def test_feedback_calibration_does_not_replace_the_cumulative_acquisition_reference(closed_inputs):
    inputs = closed_inputs
    original_p = inputs['tool_R'].T@(inputs['head_p']-inputs['tool_p'])
    original_R = inputs['tool_R'].T@inputs['bolt_R']
    original_p.setflags(write=False)
    original_R.setflags(write=False)
    last_output = None
    for number in range(1, 5):
        measured_p = original_p+np.array([number*.0003, 0., 0.])
        measured_R = Rotation.from_euler('z', number*np.deg2rad(.7)).as_matrix()@original_R
        changed = {**inputs, 'head_p': inputs['tool_p']+inputs['tool_R']@measured_p,
                   'bolt_R': inputs['tool_R']@measured_R}
        output = candidate.closed_head_target(**changed)
        np.testing.assert_allclose(output['measured_head_to_tool_p'], measured_p, atol=1e-14)
        head_goal = inputs['hole_p']+inputs['hole_R'][:, 2]*inputs['head_anchor_z_m']
        np.testing.assert_allclose(output['position_m']+output['rotation']@measured_p, head_goal, atol=1e-14)
        last_output = output
    assert np.linalg.norm(last_output['measured_head_to_tool_p']-original_p) > .001
    assert np.linalg.norm(Rotation.from_matrix(last_output['measured_head_to_tool_R']@original_R.T).as_rotvec()) > np.deg2rad(2.)
    # The caller must reject these cumulative original-reference differences
    # even though every feedback calibration produced a centered target.
    np.testing.assert_array_equal(original_p, inputs['tool_R'].T@(inputs['head_p']-inputs['tool_p']))


@pytest.mark.parametrize('which', ['closed', 'open'])
def test_target_helpers_do_not_mutate_readonly_geometry(which, closed_inputs, open_inputs):
    inputs = closed_inputs if which == 'closed' else open_inputs
    snapshots = {}
    for name, value in inputs.items():
        if isinstance(value, np.ndarray):
            snapshots[name] = value.copy()
            value.setflags(write=False)
    function = candidate.closed_head_target if which == 'closed' else candidate.frozen_open_target
    function(**inputs)
    for name, before in snapshots.items():
        np.testing.assert_array_equal(inputs[name], before)


@pytest.mark.parametrize('name,bad', [
    ('tool_p', [np.nan, 0., 0.]), ('head_p', [0., 0.]),
    ('tool_R', np.diag([1., 1., -1.])), ('bolt_R', np.eye(3)*1.01),
    ('hole_R', np.full((3, 3), np.nan)), ('desired_head_yaw_rad', np.inf),
    ('head_anchor_z_m', np.nan), ('yaw_speed_rad_s', np.nan),
    ('hole_velocity_6', [0.]*5),
])
def test_closed_target_rejects_nonphysical_inputs_before_returning_commands(closed_inputs, name, bad):
    with pytest.raises(ValueError):
        candidate.closed_head_target(**{**closed_inputs, name: bad})


@pytest.mark.parametrize('name,bad', [
    ('hole_p', [0., np.inf, 0.]), ('release_relative_tool_p', [0., 0.]),
    ('release_head_tool_p', [np.nan, 0., 0.]),
    ('release_relative_tool_R', np.diag([1., -1., 1.])),
    ('delta_clock_rad', np.inf), ('clock_speed_rad_s', np.nan),
    ('hole_velocity_6', [0., 0., 0., 0., 0., np.nan]),
])
def test_open_target_rejects_nonphysical_inputs_before_returning_commands(open_inputs, name, bad):
    with pytest.raises(ValueError):
        candidate.frozen_open_target(**{**open_inputs, name: bad})


def test_left_force_damping_is_dissipative_with_correct_world_Z_sign():
    for velocity, expected in ((-.003, -1.4), (0., -2.), (.003, -2.6)):
        force = candidate.downward_hold_feed(velocity)
        assert force == pytest.approx(expected)
        assert (force+2.)*velocity == pytest.approx(-200.*velocity**2)
    assert candidate.downward_hold_feed(0., downward_force_N=0.) == 0.
    assert candidate.downward_hold_feed(.003, downward_force_N=0., damping_Ns_per_m=0.) == 0.


@pytest.mark.parametrize('inputs', [
    (np.nan, 2., 200.), (0., np.inf, 200.), (0., 2., np.nan),
    (0., -1., 200.), (0., 2., -1.),
])
def test_left_force_rejects_invalid_or_antidissipative_inputs(inputs):
    with pytest.raises(ValueError):
        candidate.downward_hold_feed(*inputs)


@pytest.fixture(scope='module')
def native_controllers():
    from yam_twin.m8_supported_scene import supported_config, build_model
    from yam_twin.m8_supported_simulation import SupportedControlConfig, initialize_supported_pose
    from yam_twin.m8_simulation import YamCartesianController
    scene, control = supported_config(), SupportedControlConfig()
    model = build_model(scene)
    data = mujoco.MjData(model)
    initialize_supported_pose(model, data, scene, control)
    return model, data, {
        side: YamCartesianController(model, data, side, control.arm, scene.base)
        for side in ('left', 'right')}


def test_native_command_projection_removes_axial_position_and_velocity_feedback(native_controllers):
    _, data, controllers = native_controllers
    right = controllers['right']
    position, orientation = right.pose()
    axis = np.array([.3, .4, np.sqrt(.75)])
    tangent = np.cross(axis, [0., 0., 1.])
    qpos, qvel = data.qpos.copy(), data.qvel.copy()
    right.command(position+1e-5*tangent, orientation, .024,
        linear_velocity=np.zeros(3), axial_float=True, axis_world=axis, axial_feed_N=.05)
    wrench, motors = right.last_wrench.copy(), right.last_motor_torques.copy()
    right.command(position+1e-5*tangent+.5*axis, orientation, .024,
        linear_velocity=.7*axis, axial_float=True, axis_world=axis, axial_feed_N=.05)
    np.testing.assert_allclose(right.last_wrench, wrench, atol=1e-10)
    np.testing.assert_allclose(right.last_motor_torques, motors, atol=1e-10)
    assert np.dot(right.last_wrench[:3], axis) == pytest.approx(.05)
    np.testing.assert_array_equal(data.qpos, qpos)
    np.testing.assert_array_equal(data.qvel, qvel)


def test_explicit_head_alignment_still_leaves_native_axial_position_unconstrained(
    native_controllers, closed_inputs,
):
    _, data, controllers = native_controllers
    right = controllers['right']
    scheduled_head_R = rotation([-.19, .23, -.51])
    inputs = {**closed_inputs, 'hole_velocity_6': np.zeros(6)}
    first = candidate.closed_head_target(**inputs, desired_head_rotation=scheduled_head_R)
    shifted_inputs = {**inputs, 'head_anchor_z_m': inputs['head_anchor_z_m']+.4}
    shifted = candidate.closed_head_target(**shifted_inputs, desired_head_rotation=scheduled_head_R)
    axis = closed_inputs['hole_R'][:, 2]
    np.testing.assert_allclose(shifted['position_m']-first['position_m'], .4*axis, atol=1e-14)
    qpos, qvel = data.qpos.copy(), data.qvel.copy()
    right.command(first['position_m'], first['rotation'], .024,
        linear_velocity=first['linear_velocity_m_per_s'],
        angular_velocity=first['angular_velocity_rad_per_s'],
        axial_float=True, axis_world=axis, axial_feed_N=.05)
    wrench, motor_torques = right.last_wrench.copy(), right.last_motor_torques.copy()
    right.command(shifted['position_m'], shifted['rotation'], .024,
        linear_velocity=shifted['linear_velocity_m_per_s'],
        angular_velocity=shifted['angular_velocity_rad_per_s'],
        axial_float=True, axis_world=axis, axial_feed_N=.05)
    # No moving-hole orbital term is present: changing the numerical axial
    # anchor must leave the physical force-float command unchanged.
    np.testing.assert_allclose(shifted['linear_velocity_m_per_s'], first['linear_velocity_m_per_s'], atol=1e-14)
    np.testing.assert_allclose(right.last_wrench, wrench, atol=1e-10)
    np.testing.assert_allclose(right.last_motor_torques, motor_torques, atol=1e-10)
    np.testing.assert_array_equal(data.qpos, qpos)
    np.testing.assert_array_equal(data.qvel, qvel)


def test_native_left_force_command_respects_combined_wrench_motor_and_jaw_caps(native_controllers):
    model, data, controllers = native_controllers
    left = controllers['left']
    position, orientation = left.pose()
    qpos, qvel = data.qpos.copy(), data.qvel.copy()
    ctrl_before = data.ctrl.copy()
    left.command(position+[.2, -.2, .5], orientation, .0184,
        axial_float=True, axis_world=[0., 0., 1.],
        axial_feed_N=candidate.downward_hold_feed(.1))
    assert np.linalg.norm(left.last_wrench[:3]) <= 8.+1e-12
    assert np.linalg.norm(left.last_wrench[3:]) <= 2.+1e-12
    assert np.all(np.abs(left.last_motor_torques) <= left.torque_caps)
    allowed = np.r_[left.motor_ids, left.finger_ids]
    untouched = np.setdiff1d(np.arange(model.nu), allowed)
    np.testing.assert_array_equal(data.ctrl[untouched], ctrl_before[untouched])
    assert np.all(data.ctrl[left.finger_ids] >= model.actuator_ctrlrange[left.finger_ids, 0])
    assert np.all(data.ctrl[left.finger_ids] <= model.actuator_ctrlrange[left.finger_ids, 1])
    np.testing.assert_array_equal(data.qpos, qpos)
    np.testing.assert_array_equal(data.qvel, qvel)
    np.testing.assert_array_equal(data.qfrc_applied, 0.)
    np.testing.assert_array_equal(data.xfrc_applied, 0.)


def test_initial_native_pad_surfaces_keep_left_pickup_opening_and_widen_only_right(native_controllers):
    from yam_twin.m8_supported_simulation import SupportedControlConfig
    model, data, controllers = native_controllers
    config = SupportedControlConfig()
    measured = {}
    for side in ('left', 'right'):
        jaw_axis = controllers[side].pose()[1][:, 1]
        centers, extents = [], []
        for finger in ('left', 'right'):
            geom = model.geom(f'{side}_m8_pad_{finger}').id
            assert model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_BOX
            centers.append(np.dot(data.geom_xpos[geom], jaw_axis))
            local_axis = data.geom_xmat[geom].reshape(3, 3).T@jaw_axis
            extents.append(np.dot(np.abs(local_axis), model.geom_size[geom]))
        measured[side] = abs(centers[1]-centers[0])-sum(extents)
    assert measured['left'] == pytest.approx(config.left_open_aperture_m, abs=1e-12)
    assert measured['right'] == pytest.approx(config.arm.open_aperture, abs=1e-12)
    assert measured['left'] == pytest.approx(.024)
    assert measured['right'] > measured['left']


def seat_sample(window, tick, *, force=1., drop=60e-6, axial=0., angular=0., valid=True, stopped=True, dt=.001):
    return window.observe(tick*dt, dt, drop, axial, force,
        actual_relative_angular_speed_rad_per_s=angular, valid=valid, physically_stopped=stopped)


def test_impulse_event_accepts_original_loaded_duty_without_inventing_continuous_contact():
    event = candidate.ImpulseSeatDropWindow(0., 1.)
    report = None
    for tick in range(1, 51):
        report = seat_sample(event, tick, force=1. if tick%5 == 0 else 0.)
        if tick < 50:
            assert not report['confirmed_search_direction_event']
    assert report['confirmed_search_direction_event']
    original = report['original_native_impulse_window']
    assert original['observer'] == 'native-settled-entry-support-v1'
    assert original['minimum_loaded_normal_force_N'] == pytest.approx(.1)
    assert original['minimum_normal_impulse_Ns'] == pytest.approx(.005)
    assert original['minimum_loaded_duration_s'] == pytest.approx(.005)
    assert original['loaded_substep_duty'] == pytest.approx(.2)
    assert original['loaded_duration_s'] == pytest.approx(.010)
    assert original['normal_impulse_Ns'] == pytest.approx(.010)
    assert not report['is_engagement_or_release_proof']
    assert 'never jaw-opening' in report['scope']


def test_impulse_event_a_large_short_spike_cannot_replace_loaded_duration():
    event = candidate.ImpulseSeatDropWindow(0., 1.)
    for tick in range(1, 51):
        report = seat_sample(event, tick, force=100. if tick == 50 else 0.)
    original = report['original_native_impulse_window']
    assert original['normal_impulse_Ns'] > original['minimum_normal_impulse_Ns']
    assert original['loaded_duration_s'] < original['minimum_loaded_duration_s']
    assert not report['confirmed_search_direction_event']


def test_impulse_event_loaded_time_cannot_replace_required_impulse():
    event = candidate.ImpulseSeatDropWindow(0., 1.)
    for tick in range(1, 51):
        report = seat_sample(event, tick, force=.11 if tick%5 == 0 else 0.)
    original = report['original_native_impulse_window']
    assert original['loaded_duration_s'] >= original['minimum_loaded_duration_s']
    assert original['normal_impulse_Ns'] < original['minimum_normal_impulse_Ns']
    assert not report['confirmed_search_direction_event']


def test_impulse_event_requires_loaded_original_final_tick():
    event = candidate.ImpulseSeatDropWindow(0., 1.)
    for tick in range(1, 51):
        seat_sample(event, tick)
    assert event.ready
    report = seat_sample(event, 51, force=0.)
    assert report['original_native_impulse_window']['normal_impulse_Ns'] >= .005
    assert not report['confirmed_search_direction_event']
    assert seat_sample(event, 52)['confirmed_search_direction_event']


@pytest.mark.parametrize('bad', [
    dict(drop=49e-6), dict(axial=.00021), dict(axial=-.00021),
    dict(angular=.011), dict(valid=False), dict(stopped=False),
])
def test_impulse_event_unstable_geometry_motion_or_command_state_resets_quiet_window(bad):
    event = candidate.ImpulseSeatDropWindow(0., 1.)
    for tick in range(1, 51):
        seat_sample(event, tick)
    assert event.ready
    report = seat_sample(event, 51, **bad)
    assert not report['confirmed_search_direction_event']
    assert report['original_native_impulse_window']['observed_window_s'] == 0.
    for tick in range(52, 101):
        assert not seat_sample(event, tick)['confirmed_search_direction_event']
    assert seat_sample(event, 101)['confirmed_search_direction_event']


def test_impulse_event_keeps_original_drop_reference_across_moving_contact_impacts():
    event = candidate.ImpulseSeatDropWindow(0., 1.)
    for tick in range(1, 31):
        report = seat_sample(event, tick, force=100., drop=200e-6, axial=.002, angular=.5, stopped=False)
    assert report['stop_requested_from_measured_drop']
    assert report['first_actual_drop_time_s'] == pytest.approx(.001)
    for tick in range(31, 131):
        report = seat_sample(event, tick, force=1., drop=49e-6)
    assert report['actual_axial_drop_m'] == pytest.approx(49e-6)
    assert not report['confirmed_search_direction_event']
    # A transient impact and its old 200 um drop cannot qualify a later
    # below-threshold position; the initial measured reference stays fixed.
    assert event.reference == 0.
    for tick in range(131, 181):
        report = seat_sample(event, tick, drop=50e-6)
    assert report['confirmed_search_direction_event']
    assert report['first_actual_drop_time_s'] == pytest.approx(.001)


def test_impulse_event_quiet_contact_without_actual_drop_never_requests_stop():
    event = candidate.ImpulseSeatDropWindow(0., 1.)
    for tick in range(1, 201):
        report = seat_sample(event, tick, force=100., drop=49e-6)
    assert not report['stop_requested_from_measured_drop']
    assert not report['confirmed_search_direction_event']
    assert report['first_actual_drop_time_s'] is None


@pytest.mark.parametrize('next_tick', [49, 50, 52])
def test_impulse_event_repeated_backward_or_skipped_time_clears_qualified_window(next_tick):
    event = candidate.ImpulseSeatDropWindow(0., 1.)
    for tick in range(1, 51):
        seat_sample(event, tick)
    assert event.ready
    with pytest.raises(ValueError, match='contiguous native'):
        seat_sample(event, next_tick)
    assert not event.ready
    assert event.report()['original_native_impulse_window']['observed_window_s'] == 0.


@pytest.mark.parametrize('bad', [dict(force=-1.), dict(angular=-.01), dict(force=np.nan), dict(axial=np.inf)])
def test_impulse_event_rejects_nonphysical_original_observations(bad):
    event = candidate.ImpulseSeatDropWindow(0., 1.)
    with pytest.raises(ValueError, match='finite physical values'):
        seat_sample(event, 1, **bad)


@pytest.mark.parametrize('field', [
    'minimum_drop_m', 'required_stable_stop_s', 'velocity_limit_m_per_s',
    'angular_velocity_limit_rad_per_s',
])
def test_impulse_event_thresholds_must_be_positive_and_finite(field):
    with pytest.raises(ValueError, match='positive and finite'):
        candidate.ImpulseSeatDropWindow(0., 1., **{field: np.nan})


@pytest.mark.parametrize('reference,feed', [(np.nan, 1.), (np.inf, 1.), (0., 0.), (0., np.inf)])
def test_impulse_event_requires_finite_reference_and_positive_finite_feed(reference, feed):
    with pytest.raises(ValueError):
        candidate.ImpulseSeatDropWindow(reference, feed)


@pytest.mark.parametrize('dt', [0., np.nan])
def test_impulse_event_rejects_nonpositive_or_nonfinite_native_timestep(dt):
    with pytest.raises(ValueError, match='positive timestep'):
        seat_sample(candidate.ImpulseSeatDropWindow(0., 1.), 1, dt=dt)
