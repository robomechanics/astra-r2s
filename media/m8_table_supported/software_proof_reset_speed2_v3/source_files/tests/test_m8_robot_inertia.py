"""Independent robot mechanics and mocked controller checks; no integration.

Mass derives from kinetic energy and task acceleration from actual FK paths.
Passing tests cannot qualify physical motor/contact tracking or threading.
"""
from types import SimpleNamespace
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from yam_twin import m8_robot_inertia as ff


def planar_endpoint(q):
    return .4*np.array([np.cos(q[0]), np.sin(q[0])])+ .3*np.array(
        [np.cos(q.sum()), np.sin(q.sum())])


def planar_kinetic_energy(q, velocity):
    # Two physical COMs and link rotations, independent of operational-space
    # algebra. Link lengths .4/.3m, COMs .2/.15m, masses .6/.8kg.
    tangent1 = np.array([-np.sin(q[0]), np.cos(q[0])])
    tangent2 = np.array([-np.sin(q.sum()), np.cos(q.sum())])
    v1 = .2*velocity[0]*tangent1
    v2 = .4*velocity[0]*tangent1+.15*velocity.sum()*tangent2
    return .5*(.6*v1@v1+.8*v2@v2+.01*velocity[0]**2+.02*velocity.sum()**2)


def planar_mass_from_energy(q):
    basis = np.eye(2)
    diagonal = [2*planar_kinetic_energy(q, item) for item in basis]
    cross = planar_kinetic_energy(q, basis.sum(axis=0))-sum(diagonal)/2
    return np.array([[diagonal[0], cross], [cross, diagonal[1]]])


def planar_jacobian(q):
    h = 2e-5
    return np.column_stack([(planar_endpoint(q+h*item)-planar_endpoint(q-h*item))/(2*h)
                            for item in np.eye(2)])


@pytest.fixture
def mechanics():
    q, qdot, qdd = np.array([.4, .9]), np.array([.3, -.2]), np.array([.6, -.4])
    h = 1e-4
    mass = np.diag([1., 1., .11, .13, .17, .8])
    mass[:2, :2] = planar_mass_from_energy(q)
    jp, jr, jpdot, jrdot = (np.zeros((3, 6)) for _ in range(4))
    jp[:2, :2] = planar_jacobian(q)
    jp[2, 5] = 1.  # An independent axial slide remains outside the task.
    jr[:, 2:5] = np.eye(3)
    jpdot[:2, :2] = (planar_jacobian(q+h*qdot)-planar_jacobian(q-h*qdot))/(2*h)
    endpoint_acceleration = (planar_endpoint(q+h*qdot+.5*h*h*qdd)
        -2*planar_endpoint(q)+planar_endpoint(q-h*qdot+.5*h*h*qdd))/(h*h)
    return dict(mass_aa=mass, jp=jp, jr=jr, jpdot=jpdot, jrdot=jrdot,
        retained_qdot=np.r_[qdot, [.1, -.4, .2, .7]],
        transverse_basis_world=np.eye(3)[:, :2],
        desired_site_acceleration_world=np.r_[endpoint_acceleration, 123.],
        desired_angular_acceleration_world=np.array([.4, -.5, .2]),
        q=q, qdd=qdd)


def run_mechanics(case, **overrides):
    values = {name: value for name, value in case.items() if name not in ('q', 'qdd')}
    values.update(overrides)
    return ff.hybrid5d_feedforward(**values)


def mapped_torque(case, wrench):
    return case['jp'].T@wrench[:3]+case['jr'].T@wrench[3:]


def test_planar2r_feedforward_matches_independently_derived_kinetic_energy_inertia(mechanics):
    result = run_mechanics(mechanics)
    torque = mapped_torque(mechanics, result['wrench_world_N_Nm'])
    expected = np.r_[planar_mass_from_energy(mechanics['q'])@mechanics['qdd'],
                      np.array([.11, .13, .17])*np.array([.4, -.5, .2]), 0.]
    assert torque == pytest.approx(expected, abs=8e-8)
    assert result['wrench_world_N_Nm'][2] == pytest.approx(0., abs=1e-14)


def test_nonzero_velocity_geometric_bias_is_required_for_actual_fk_path_acceleration(mechanics):
    result = run_mechanics(mechanics)
    torque = mapped_torque(mechanics, result['wrench_world_N_Nm'])
    joint_acceleration = np.linalg.solve(mechanics['mass_aa'], torque)
    achieved_world = mechanics['jp']@joint_acceleration+mechanics['jpdot']@mechanics['retained_qdot']
    assert achieved_world[:2] == pytest.approx(mechanics['desired_site_acceleration_world'][:2], abs=2e-14)
    missing_bias = run_mechanics(mechanics, jpdot=np.zeros((3, 6)))
    missing_joint_acceleration = np.linalg.solve(mechanics['mass_aa'],
        mapped_torque(mechanics, missing_bias['wrench_world_N_Nm']))
    actual_with_missing_bias = mechanics['jp']@missing_joint_acceleration+mechanics['jpdot']@mechanics['retained_qdot']
    assert np.linalg.norm(actual_with_missing_bias[:2]-achieved_world[:2]) > .02


@pytest.mark.parametrize('omega, alpha', [(1.7, 3.), (-1.7, 3.), (1.7, -3.), (0., 3.), (1.7, 0.)])
def test_off_center_tool_orbit_acceleration_matches_actual_rotated_trajectory(omega, alpha):
    lever = np.array([.027, -.013, .004])
    axis = np.array([.3, -.4, .5]); axis /= np.linalg.norm(axis)
    h = 2e-5
    def tool_position(t):
        return -Rotation.from_rotvec(axis*(omega*t+.5*alpha*t*t)).apply(lever)
    expected = (tool_position(h)-2*tool_position(0.)+tool_position(-h))/(h*h)
    actual = ff.orbital_site_acceleration(lever, axis*omega, axis*alpha)
    assert actual == pytest.approx(expected, abs=2e-8)


def test_planar_orbit_tangential_and_centripetal_signs_are_physical():
    lever = [.03, 0., 0.]
    assert ff.orbital_site_acceleration(lever, [0., 0., 2.], [0., 0., 3.]) == pytest.approx([.12, -.09, 0.])
    assert ff.orbital_site_acceleration(lever, [0., 0., -2.], [0., 0., 3.]) == pytest.approx([.12, -.09, 0.])
    assert ff.orbital_site_acceleration(lever, [0., 0., 2.], [0., 0., -3.]) == pytest.approx([.12, .09, 0.])


def test_world_frame_rotation_preserves_physical_joint_command_and_rotates_full_wrench(mechanics):
    rotation = Rotation.from_euler('xyz', [.37, -.64, 1.12]).as_matrix()
    result = run_mechanics(mechanics)
    rotated = dict(mechanics)
    for name in ('jp', 'jr', 'jpdot', 'jrdot', 'transverse_basis_world',
                 'desired_site_acceleration_world', 'desired_angular_acceleration_world'):
        rotated[name] = rotation@mechanics[name]
    actual = run_mechanics(rotated)
    expected_wrench = np.r_[rotation@result['wrench_world_N_Nm'][:3],
                             rotation@result['wrench_world_N_Nm'][3:]]
    assert actual['wrench_world_N_Nm'] == pytest.approx(expected_wrench, abs=1e-13)
    assert mapped_torque(rotated, actual['wrench_world_N_Nm']) == pytest.approx(
        mapped_torque(mechanics, result['wrench_world_N_Nm']), abs=2e-14)
    axis = rotation[:, 2]
    assert axis@actual['wrench_world_N_Nm'][:3] == pytest.approx(0., abs=1e-14)


def test_arbitrary_axial_acceleration_target_is_ignored_in_rotated_task(mechanics):
    rotation = Rotation.from_euler('zyx', [.51, -.72, .29]).as_matrix()
    rotated = dict(mechanics)
    for name in ('jp', 'jr', 'jpdot', 'jrdot', 'transverse_basis_world',
                 'desired_site_acceleration_world', 'desired_angular_acceleration_world'):
        rotated[name] = rotation@mechanics[name]
    base = run_mechanics(rotated)
    # A large axial acceleration is never an axial lead, spring, or servo.
    axial = rotated['desired_site_acceleration_world']+2000.*rotation[:, 2]
    altered = run_mechanics(rotated, desired_site_acceleration_world=axial)
    assert altered['wrench_world_N_Nm'] == pytest.approx(base['wrench_world_N_Nm'], abs=2e-10)


def test_free_task_axis_can_accelerate_through_arm_inertia_without_an_axial_wrench():
    mass = np.eye(6); mass[0, 5] = mass[5, 0] = .35
    jp, jr = np.zeros((3, 6)), np.zeros((3, 6))
    jp[0, 0], jp[1, 1], jp[2, 5] = .5, .4, .8
    jr[:, 2:5] = np.eye(3)
    result = ff.hybrid5d_feedforward(mass, jp, jr, np.zeros_like(jp), np.zeros_like(jr),
        np.zeros(6), np.eye(3)[:, :2], [1., .2, 0.], [0., 0., 0.])
    wrench = result['wrench_world_N_Nm']
    qdd = np.linalg.solve(mass, jp.T@wrench[:3]+jr.T@wrench[3:])
    assert wrench[2] == 0.
    assert (jp@qdd)[:2] == pytest.approx([1., .2])
    assert abs((jp@qdd)[2]) > .1
    # A full6D solve followed by dropping axial force fails this physical task.
    full_jac = np.vstack((jp, jr))
    full_wrench = np.linalg.solve(full_jac@np.linalg.solve(mass, full_jac.T), [1., .2, 0., 0., 0., 0.])
    full_wrench[2] = 0.
    wrong_qdd = np.linalg.solve(mass, full_jac.T@full_wrench)
    assert abs((jp@wrong_qdd)[0]-1.) > .1


def test_rotating_hole_basis_projects_world_acceleration_without_spurious_coordinate_bias(mechanics):
    rotation = Rotation.from_euler('xyz', [.4, -.3, .2]).as_matrix()
    basis = rotation[:, :2]
    axis = rotation[:, 2]
    jp = rotation@mechanics['jp']
    jr = rotation@mechanics['jr']
    qdot = mechanics['retained_qdot']
    actual = run_mechanics(mechanics, jp=jp, jr=jr, transverse_basis_world=basis,
        jpdot=np.zeros((3, 6)), jrdot=np.zeros((3, 6)),
        desired_site_acceleration_world=np.zeros(3), desired_angular_acceleration_world=np.zeros(3))
    assert actual['wrench_world_N_Nm'] == pytest.approx(np.zeros(6), abs=1e-14)
    h = 1e-5
    minus = Rotation.from_rotvec(-axis*.8*h).as_matrix()@basis
    plus = Rotation.from_rotvec(axis*.8*h).as_matrix()@basis
    coordinate_derivative = (plus.T@(jp@qdot)-minus.T@(jp@qdot))/(2*h)
    assert np.linalg.norm(coordinate_derivative) > .02
    # This E_dot term belongs to moving-coordinate velocity components, not
    # physical world acceleration; adding it only to Jdot would be wrong.
    assert actual['projected_world_jdot_qdot5'] == pytest.approx(np.zeros(5))


def test_transverse_basis_gauge_changes_components_without_changing_world_wrench(mechanics):
    change = np.array([[np.cos(.63), -np.sin(.63)], [np.sin(.63), np.cos(.63)]])
    result = run_mechanics(mechanics)
    changed = run_mechanics(mechanics,
        transverse_basis_world=mechanics['transverse_basis_world']@change)
    assert changed['wrench_world_N_Nm'] == pytest.approx(result['wrench_world_N_Nm'], abs=1e-13)


def test_geometric_robot_scaling_preserves_mixed_force_and_torque_units(mechanics):
    result = run_mechanics(mechanics)
    length_scale = 3.
    scaled = run_mechanics(mechanics, mass_aa=mechanics['mass_aa']*length_scale**2,
        jp=mechanics['jp']*length_scale, jpdot=mechanics['jpdot']*length_scale,
        desired_site_acceleration_world=mechanics['desired_site_acceleration_world']*length_scale)
    expected = np.r_[result['wrench_world_N_Nm'][:3]*length_scale,
                      result['wrench_world_N_Nm'][3:]*length_scale**2]
    assert scaled['wrench_world_N_Nm'] == pytest.approx(expected, abs=1e-13)


def test_head_to_tool_wrench_translation_preserves_generalized_torque_and_power(mechanics):
    lever, force, head_torque = np.array([.03, -.02, .01]), np.array([2., -1., .5]), np.array([.3, -.2, .4])
    skew = np.array([[0., -lever[2], lever[1]], [lever[2], 0., -lever[0]], [-lever[1], lever[0], 0.]])
    head_jp = mechanics['jp']-skew@mechanics['jr']
    tool_torque = head_torque+np.cross(lever, force)
    zeros, caps = np.zeros(6), np.ones(6)*100.
    head = ff.combine_capped_robot_command(np.r_[force, head_torque], zeros,
        head_jp, mechanics['jr'], zeros, zeros, caps)
    tool = ff.combine_capped_robot_command(np.r_[force, tool_torque], zeros,
        mechanics['jp'], mechanics['jr'], zeros, zeros, caps)
    assert head['motor_torques_Nm'] == pytest.approx(tool['motor_torques_Nm'], abs=2e-15)
    qdot = mechanics['retained_qdot']
    head_power = force@(head_jp@qdot)+head_torque@(mechanics['jr']@qdot)
    tool_power = force@(mechanics['jp']@qdot)+tool_torque@(mechanics['jr']@qdot)
    assert head_power == pytest.approx(tool_power, abs=2e-15)
    assert tool['motor_torques_Nm']@qdot == pytest.approx(tool_power, abs=2e-15)


def test_combined_pd_feed_and_ff_are_capped_after_sum_not_individually(mechanics):
    pd = np.array([5., 0., 1., 0., 1.3, 0.])  # Includes existing axial feed.
    inertia = np.array([5., 0., 0., 0., 1.3, 0.])
    assert np.linalg.norm(pd[:3]) < 8 and np.linalg.norm(inertia[:3]) < 8
    assert np.linalg.norm(pd[3:]) < 2 and np.linalg.norm(inertia[3:]) < 2
    result = ff.combine_capped_robot_command(pd, inertia, mechanics['jp'], mechanics['jr'],
        np.zeros(6), np.zeros(6), np.ones(6)*100.)
    assert result['combined_uncapped_wrench_world_N_Nm'] == pytest.approx([10., 0., 1., 0., 2.6, 0.])
    assert result['capped_wrench_world_N_Nm'] == pytest.approx(np.r_[8*np.array([10., 0., 1.])/np.sqrt(101.), [0., 2., 0.]])
    assert result['cartesian_force_clipped'] and result['cartesian_torque_clipped']
    assert np.linalg.norm(result['capped_wrench_world_N_Nm'][:3]) == pytest.approx(8.)
    assert np.linalg.norm(result['capped_wrench_world_N_Nm'][3:]) == pytest.approx(2.)


def test_native_bias_and_viscous_drag_are_added_once_before_final_motor_clips():
    jp, jr = np.zeros((3, 6)), np.zeros((3, 6))
    jp[0, 0], jr[1, 0] = .4, 1.
    pd, inertia = np.array([1., 0., 0., 0., .2, 0.]), np.array([2., 0., 0., 0., .3, 0.])
    bias, drag = np.array([.5, -.3, .2, 0., 0., 0.]), np.array([.25, -.1, 0., 0., 0., 0.])
    result = ff.combine_capped_robot_command(pd, inertia, jp, jr, bias, drag, np.ones(6)*100.)
    assert result['motor_uncapped_torques_Nm'] == pytest.approx([2.45, -.4, .2, 0., 0., 0.])
    clipped = ff.combine_capped_robot_command(pd, inertia, jp, jr, bias, drag, np.array([1., .2, .1, 1., 1., 1.]))
    assert clipped['motor_torques_Nm'] == pytest.approx([1., -.2, .1, 0., 0., 0.])
    assert clipped['motor_clipped'].tolist() == [True, True, True, False, False, False]


def test_zero_task_wrench_preserves_exactly_one_native_bias_and_drag_command():
    bias, drag = np.linspace(-.3, .4, 6), np.linspace(.2, -.1, 6)
    result = ff.combine_capped_robot_command(np.zeros(6), np.zeros(6), np.zeros((3, 6)),
        np.zeros((3, 6)), bias, drag, np.ones(6)*100.)
    assert result['motor_torques_Nm'] == pytest.approx(bias+drag)


def test_helpers_preserve_all_input_arrays_on_success_and_failure(mechanics):
    before = {name: value.copy() for name, value in mechanics.items() if isinstance(value, np.ndarray)}
    run_mechanics(mechanics)
    with pytest.raises(ff.InertiaFeedforwardError):
        run_mechanics(mechanics, maximum_mobility_condition=1.01)
    for name, expected in before.items():
        assert np.array_equal(mechanics[name], expected)


@pytest.mark.parametrize('which', ['mass_zero', 'mass_negative', 'mass_nonsymmetric', 'rank_missing_rotation', 'rank_missing_translation'])
def test_invalid_or_rank_deficient_plant_fails_closed_without_regularized_motion(mechanics, which):
    case = {name: value.copy() if isinstance(value, np.ndarray) else value for name, value in mechanics.items()}
    if which == 'mass_zero':
        case['mass_aa'][5, 5] = 0.
    elif which == 'mass_negative':
        case['mass_aa'][5, 5] = -.1
    elif which == 'mass_nonsymmetric':
        case['mass_aa'][0, 1] += .01
    elif which == 'rank_missing_rotation':
        case['jr'][2] = 0.
    else:
        case['jp'][0] = 0.
    with pytest.raises(ff.InertiaFeedforwardError):
        run_mechanics(case)


def test_numerically_ill_conditioned_full_rank_task_rejects_before_large_wrench(mechanics):
    jr = mechanics['jr'].copy(); jr[2] *= 1e-7
    with pytest.raises(ff.InertiaFeedforwardError, match='condition bound'):
        run_mechanics(mechanics, jr=jr)


@pytest.mark.parametrize('limit_name', ['maximum_mass_condition', 'maximum_mobility_condition'])
@pytest.mark.parametrize('bad', [0., -1., np.nan, np.inf])
def test_condition_limits_must_be_finite_positive(mechanics, limit_name, bad):
    with pytest.raises(ff.InertiaFeedforwardError):
        run_mechanics(mechanics, **{limit_name: bad})


@pytest.mark.parametrize('name', ['mass_aa', 'jp', 'jr', 'jpdot', 'jrdot', 'retained_qdot',
    'transverse_basis_world', 'desired_site_acceleration_world', 'desired_angular_acceleration_world'])
def test_nonfinite_physical_inputs_reject_before_a_feedforward_command(mechanics, name):
    value = mechanics[name].copy(); value.flat[0] = np.nan
    with pytest.raises(ff.InertiaFeedforwardError):
        run_mechanics(mechanics, **{name: value})


@pytest.mark.parametrize('basis', [np.eye(3)[:, :2]*2., np.zeros((3, 2)), np.eye(3), np.ones((3, 1))])
def test_invalid_transverse_task_basis_rejects(mechanics, basis):
    with pytest.raises(ff.InertiaFeedforwardError):
        run_mechanics(mechanics, transverse_basis_world=basis)


@pytest.mark.parametrize('which', ['lever', 'velocity', 'acceleration'])
def test_orbital_acceleration_rejects_nonfinite_or_nonscalar_vectors(which):
    values = dict(lever_world_m=[.02, 0., 0.], angular_velocity_world_rad_s=[0., 0., 1.],
                  angular_acceleration_world_rad_s2=[0., 0., 1.])
    name = {'lever':'lever_world_m', 'velocity':'angular_velocity_world_rad_s',
            'acceleration':'angular_acceleration_world_rad_s2'}[which]
    values[name] = [0., np.inf, 0.]
    with pytest.raises(ff.InertiaFeedforwardError):
        ff.orbital_site_acceleration(**values)


@pytest.mark.parametrize('name', ['pd_wrench_world', 'ff_wrench_world', 'jp', 'jr',
    'arm_bias_torque', 'native_drag_torque', 'torque_caps'])
def test_command_composition_rejects_nonfinite_inputs(mechanics, name):
    values = dict(pd_wrench_world=np.zeros(6), ff_wrench_world=np.zeros(6),
        jp=mechanics['jp'], jr=mechanics['jr'], arm_bias_torque=np.zeros(6),
        native_drag_torque=np.zeros(6), torque_caps=np.ones(6))
    bad = values[name].copy(); bad.flat[0] = np.nan
    values[name] = bad
    with pytest.raises(ff.InertiaFeedforwardError):
        ff.combine_capped_robot_command(**values)


@pytest.mark.parametrize('name', ['maximum_force_N', 'maximum_torque_Nm'])
@pytest.mark.parametrize('bad', [0., -1., np.inf])
def test_combined_wrench_caps_are_finite_positive(mechanics, name, bad):
    with pytest.raises(ff.InertiaFeedforwardError):
        ff.combine_capped_robot_command(np.zeros(6), np.zeros(6), mechanics['jp'],
            mechanics['jr'], np.zeros(6), np.zeros(6), np.ones(6), **{name:bad})


def analytic_controller(monkeypatch, mechanics):
    # Native reads return declared analytic retained caches; no model compile,
    # mj_forward, contact evaluation, state integration, or engine write runs.
    controller = ff.HybridInertiaController.__new__(ff.HybridInertiaController)
    controller.model = SimpleNamespace(nv=6, dof_damping=np.linspace(.03, .08, 6),
        site_bodyid=np.array([0]), actuator_ctrlrange=np.array([[-100., 100.]]*6+[[0., .037524], [-.037524, 0.]]))
    controller.data = SimpleNamespace(qpos=np.linspace(.1, .6, 6),
        qvel=mechanics['retained_qdot'].copy(), qfrc_bias=np.linspace(.01, .06, 6),
        ctrl=np.zeros(8), time=2., site_xpos=np.zeros((1, 3)), site_xmat=np.eye(3).reshape(1, 9),
        qfrc_applied=np.zeros(6), xfrc_applied=np.zeros((1, 6)))
    controller.side, controller.site_id = 'right', 0
    controller.dof_indices = controller.motor_ids = np.arange(6)
    controller.finger_ids = np.array([6, 7])
    controller.config = SimpleNamespace(position_stiffness=100., position_damping=.7,
        rotation_stiffness=2., rotation_damping=.4, left_rotation_stiffness=2.,
        left_rotation_damping=.4, maximum_cartesian_force=8., maximum_cartesian_torque=2., axial_feed_N=.2)
    controller.scene_config = SimpleNamespace(pad_inner_offset=.0005)
    controller.torque_caps=np.ones(6)*100.
    controller.jp, controller.jr, controller.jpdot, controller.jrdot = (np.zeros((3, 6)) for _ in range(4))
    controller.full_mass=np.empty((6, 6))
    controller.last_wrench=np.zeros(6); controller.last_motor_torques=np.zeros(6)
    controller.last_position_error=np.zeros(3); controller.last_rotation_error=np.zeros(3)
    controller.retained_qdot=controller.retained_time_s=controller.last_inertia_record=None
    def jac_site(model, data, jp, jr, site):
        jp[:], jr[:] = mechanics['jp'], mechanics['jr']
    def full_mass(model, data, mass):
        mass[:] = mechanics['mass_aa']
    def jac_dot(model, data, jp, jr, point, body):
        jp[:], jr[:] = mechanics['jpdot'], mechanics['jrdot']
    monkeypatch.setattr(ff.mujoco, 'mj_jacSite', jac_site)
    monkeypatch.setattr(ff.mujoco, 'mj_fullM', full_mass)
    monkeypatch.setattr(ff.mujoco, 'mj_jacDot', jac_dot)
    return controller


def test_controller_uses_coherent_retained_velocity_for_pd_inertia_and_drag_without_state_writes(monkeypatch, mechanics):
    controller=analytic_controller(monkeypatch, mechanics)
    controller.capture_velocity_before_step()
    retained=controller.retained_qdot.copy()
    controller.data.qvel += np.linspace(3., 8., 6)  # Distinct post-integration velocity.
    controller.data.time += 50e-6
    states={name:getattr(controller.data,name).copy() for name in ('qpos','qvel','qfrc_applied','xfrc_applied')}
    controller.command([0.,0.,100.], np.eye(3), .0184, axial_float=True,
        linear_velocity=mechanics['jp']@retained,
        angular_velocity=mechanics['jr']@retained, axial_feed_N=.2,
        inertia_feedforward_enabled=True,
        transverse_basis_world=np.eye(3)[:,:2],
        desired_site_acceleration_world=mechanics['desired_site_acceleration_world'],
        desired_angular_acceleration_world=mechanics['desired_angular_acceleration_world'])
    record=controller.last_inertia_record
    assert record['retained_native_state_time_s'] == 2.
    assert record['retained_arm_velocity_rad_s'] == pytest.approx(retained)
    assert record['pd_wrench_world_N_Nm'] == pytest.approx([0.,0.,.2,0.,0.,0.], abs=1e-14)
    assert record['native_drag_torque_Nm'] == pytest.approx(controller.model.dof_damping*retained)
    expected=run_mechanics(mechanics)
    assert record['inertia']['wrench_world_N_Nm'] == pytest.approx(expected['wrench_world_N_Nm'])
    for name,before in states.items():
        assert np.array_equal(getattr(controller.data,name),before)
    assert controller.data.ctrl[6:] == pytest.approx([.0097,-.0097])


@pytest.mark.parametrize('failure', ['uncaptured', 'not_float', 'axial_in_basis', 'singular_task'])
def test_controller_partial_task_failure_leaves_real_commands_unwritten(monkeypatch, mechanics, failure):
    controller=analytic_controller(monkeypatch, mechanics)
    if failure != 'uncaptured':
        controller.capture_velocity_before_step()
    if failure == 'singular_task':
        mechanics['jr'][2] = 0.
    basis=np.eye(3)[:,:2] if failure != 'axial_in_basis' else np.eye(3)[:,1:]
    before=controller.data.ctrl.copy()
    with pytest.raises(ff.InertiaFeedforwardError):
        controller.command([0.,0.,0.],np.eye(3),.0184,
            axial_float=failure != 'not_float', inertia_feedforward_enabled=True,
            transverse_basis_world=basis, desired_site_acceleration_world=[0.,0.,0.],
            desired_angular_acceleration_world=[0.,0.,0.])
    assert np.array_equal(controller.data.ctrl,before)
