"""Independent pure command checks; no native physics or V3 source changes."""
from dataclasses import FrozenInstanceError
import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest

SOURCE = Path(__file__).with_name('reverse_brake_v4.py')
SPEC = importlib.util.spec_from_file_location('reverse_brake_v4', SOURCE)
braking = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = braking
SPEC.loader.exec_module(braking)


def preceding_schedule(elapsed_s):
    # Exact scheduled source values from the archived V3 boundary. The source
    # divides the velocity by this duration, not rounded steps*50us. No actual
    # object yaw, speed, axial travel, pitch, or contact state enters the curve.
    duration = 2.531250559159044
    start = 5.964363130916848e-7
    travel = -2.7-start
    u = elapsed_s/duration
    blend = 6*u**5-15*u**4+10*u**3
    derivative = 30*u*u*(1-u)**2
    second = 60*u-180*u*u+120*u**3
    return start+travel*blend, travel*derivative/duration, travel*second/duration**2


@pytest.fixture
def scheduled_boundary():
    return preceding_schedule((30383/50625)*2.531250559159044)


def make_brake(boundary, duration=.15, **changes):
    values = dict(theta0_rad=boundary[0], omega0_rad_s=boundary[1],
                  alpha0_rad_s2=boundary[2], duration_s=duration)
    values.update(changes)
    return braking.C2ReverseBrake(**values)


def test_actual_preceding_scheduled_jet_matches_archived_command_not_object_motion(scheduled_boundary):
    theta, omega, alpha = scheduled_boundary
    assert theta == pytest.approx(-1.843648993692857, abs=2e-15)
    assert omega == pytest.approx(-1.8427141965956917, abs=2e-15)
    assert 1.2 < alpha < 1.3
    brake = make_brake(scheduled_boundary)
    assert brake.sample(0.)[:3] == pytest.approx(scheduled_boundary, abs=2e-15)
    # Actual body speed magnitude in that original native row was 1.8509;
    # matching it would silently replace the independent command schedule.
    assert brake.sample(0.).omega_rad_s != pytest.approx(-1.8509038532368143)


@pytest.mark.parametrize('duration', [.100, .150])
def test_braking_starts_at_scheduled_c2_jet_and_ends_at_stationary_hold(scheduled_boundary, duration):
    brake = make_brake(scheduled_boundary, duration)
    start = brake.sample(0.)
    assert start[:3] == pytest.approx(scheduled_boundary)
    expected_distance = duration*start.omega_rad_s/2+duration**2*start.alpha_rad_s2/12
    end = brake.sample(duration)
    assert end.theta_rad == pytest.approx(start.theta_rad+expected_distance)
    assert end[1:] == (0., 0., 0.)
    assert brake.displacement_rad == pytest.approx(expected_distance)
    left = brake.sample(duration-1e-8)
    assert left.theta_rad == pytest.approx(end.theta_rad, abs=2e-14)
    assert left.omega_rad_s == pytest.approx(0., abs=1e-12)
    assert left.alpha_rad_s2 == pytest.approx(0., abs=2e-5)
    # Jerk may jump at the hold join: this schedule promises C2, not C3.


def test_first_native_command_tick_has_no_old_angular_speed_jump(scheduled_boundary):
    brake = make_brake(scheduled_boundary)
    start = brake.sample(0.)
    first = brake.sample(50e-6)
    assert abs(first.omega_rad_s-start.omega_rad_s) < 1e-4
    assert first.theta_rad < start.theta_rad
    assert first.alpha_rad_s2 == pytest.approx(start.alpha_rad_s2, abs=.04)
    assert abs(first.omega_rad_s) > 1.8
    assert abs(start.omega_rad_s-0.) > 1.8  # Original abrupt-stop comparison.


@pytest.mark.parametrize('duration', [.100, .150])
@pytest.mark.parametrize('omega, alpha', [(-1.8, 1.2), (-1., -3.), (0., 0.)])
def test_returned_velocity_acceleration_and_jerk_are_independent_numerical_derivatives(duration, omega, alpha):
    brake = braking.C2ReverseBrake(-1., omega, alpha, duration)
    h = duration*2e-5
    for fraction in (.13, .42, .81):
        t = duration*fraction
        before, sample, after = (brake.sample(t-h), brake.sample(t), brake.sample(t+h))
        assert (after.theta_rad-before.theta_rad)/(2*h) == pytest.approx(sample.omega_rad_s, abs=2e-8)
        assert (after.omega_rad_s-before.omega_rad_s)/(2*h) == pytest.approx(sample.alpha_rad_s2, abs=2e-7)
        assert (after.alpha_rad_s2-before.alpha_rad_s2)/(2*h) == pytest.approx(sample.jerk_rad_s3, abs=2e-6)


@pytest.mark.parametrize('duration', [.100, .150])
def test_whole_stopping_distance_matches_independent_velocity_quadrature(scheduled_boundary, duration):
    brake = make_brake(scheduled_boundary, duration)
    nodes, weights = np.polynomial.legendre.leggauss(8)
    speeds = np.array([brake.sample(duration*(node+1)/2).omega_rad_s for node in nodes])
    integrated = duration/2*np.dot(weights, speeds)
    assert integrated == pytest.approx(brake.final_theta_rad-brake.theta0_rad, abs=4e-16)
    assert -.14 < integrated < -.09


@pytest.mark.parametrize('duration', [.100, .150])
def test_every_50us_command_tick_stays_monotonic_within_original_workspace_and_speed_cap(scheduled_boundary, duration):
    brake = make_brake(scheduled_boundary, duration)
    samples = np.array([brake.sample(t) for t in np.arange(0., duration+25e-6, 50e-6)])
    assert np.max(np.diff(samples[:, 0])) <= 1e-14
    assert np.min(samples[:, 0]) >= -2.7
    assert np.max(samples[:, 0]) <= 0.
    assert np.max(samples[:, 1]) <= 0.
    assert np.max(abs(samples[:, 1])) <= 2.
    assert brake.peak_speed_rad_s == pytest.approx(np.max(abs(samples[:, 1])), rel=2e-7)
    assert brake.peak_acceleration_rad_s2 == pytest.approx(np.max(abs(samples[:, 2])), rel=2e-6)


def test_150ms_reduces_command_acceleration_and_jerk_while_requiring_more_reverse_room(scheduled_boundary):
    short = make_brake(scheduled_boundary, .100)
    long = make_brake(scheduled_boundary, .150)
    assert long.peak_acceleration_rad_s2 < .70*short.peak_acceleration_rad_s2
    assert long.peak_jerk_rad_s3 < .50*short.peak_jerk_rad_s3
    assert long.final_theta_rad < short.final_theta_rad
    assert long.final_theta_rad > -2.7
    assert long.peak_speed_rad_s == short.peak_speed_rad_s


def test_continuity_matches_prior_reverse_quintic_with_one_sided_derivatives(scheduled_boundary):
    brake = make_brake(scheduled_boundary)
    event = (30383/50625)*2.531250559159044
    h = 2e-5
    p2, p1, p0 = (preceding_schedule(event-k*h) for k in (2, 1, 0))
    b0, b1, b2 = (brake.sample(k*h) for k in (0, 1, 2))
    prior_velocity = (3*p0[0]-4*p1[0]+p2[0])/(2*h)
    next_velocity = (-3*b0.theta_rad+4*b1.theta_rad-b2.theta_rad)/(2*h)
    prior_acceleration = (3*p0[1]-4*p1[1]+p2[1])/(2*h)
    next_acceleration = (-3*b0.omega_rad_s+4*b1.omega_rad_s-b2.omega_rad_s)/(2*h)
    assert next_velocity == pytest.approx(prior_velocity, abs=1e-7)
    assert next_acceleration == pytest.approx(prior_acceleration, abs=3e-6)


def test_cap_validation_accounts_for_interior_peak_speed_not_only_initial_speed():
    brake = braking.C2ReverseBrake(-1., -1.9, -8., .15)
    assert brake.peak_speed_rad_s > 1.9
    with pytest.raises(ValueError, match='angular-speed cap'):
        braking.C2ReverseBrake(-1., -1.9, -8., .15, maximum_speed_rad_s=1.9)


def test_acceleration_cap_checks_interior_braking_extremum(scheduled_boundary):
    brake = make_brake(scheduled_boundary)
    assert brake.peak_acceleration_rad_s2 > 10*abs(brake.alpha0_rad_s2)
    with pytest.raises(ValueError, match='angular-acceleration cap'):
        make_brake(scheduled_boundary, maximum_acceleration_rad_s2=brake.peak_acceleration_rad_s2-.1)
    accepted = make_brake(scheduled_boundary, maximum_acceleration_rad_s2=brake.peak_acceleration_rad_s2+.1)
    assert accepted.sample(.075) == brake.sample(.075)


def test_workspace_rejects_stopping_distance_beyond_bound_even_when_initial_clock_is_legal():
    with pytest.raises(ValueError, match='workspace'):
        braking.C2ReverseBrake(-2.65, -1.8, 1.2, .15)
    brake = braking.C2ReverseBrake(-2.55, -1.8, 1.2, .15)
    assert brake.final_theta_rad > -2.7


@pytest.mark.parametrize('omega, alpha', [(1., 0.), (-1., 21.), (0., 1.)])
def test_brake_rejects_positive_or_hidden_interior_sign_reversal(omega, alpha):
    with pytest.raises(ValueError, match='sign reversal'):
        braking.C2ReverseBrake(-1., omega, alpha, .15)


def test_boundary_monotonic_curve_with_zero_end_bracket_is_valid():
    brake = braking.C2ReverseBrake(-1., -1., 20., .15)
    assert brake.duration_s*brake.alpha0_rad_s2 == -3*brake.omega0_rad_s
    assert all(brake.sample(t).omega_rad_s <= 0. for t in np.linspace(0., .15, 101))


@pytest.mark.parametrize('elapsed', [-1e6, -50e-6])
def test_finite_prestart_queries_clamp_to_preceding_start_jet(scheduled_boundary, elapsed):
    brake = make_brake(scheduled_boundary)
    assert brake.sample(elapsed) == brake.sample(0.)


@pytest.mark.parametrize('elapsed', [.150, .15005, 1e6])
def test_finite_completed_queries_hold_the_same_stationary_clock(scheduled_boundary, elapsed):
    brake = make_brake(scheduled_boundary)
    assert brake.sample(elapsed) == (brake.final_theta_rad, 0., 0., 0.)


@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf, None, [0., 1.]])
def test_nonfinite_or_nonscalar_query_time_is_rejected(scheduled_boundary, bad):
    with pytest.raises(ValueError, match='finite scalar'):
        make_brake(scheduled_boundary).sample(bad)


@pytest.mark.parametrize('name', ['theta0_rad', 'omega0_rad_s', 'alpha0_rad_s2',
    'duration_s', 'minimum_theta_rad', 'maximum_theta_rad', 'maximum_speed_rad_s',
    'maximum_acceleration_rad_s2'])
def test_all_schedule_workspace_and_cap_inputs_must_be_finite(scheduled_boundary, name):
    with pytest.raises(ValueError, match='finite scalar'):
        make_brake(scheduled_boundary, **{name: np.nan})


@pytest.mark.parametrize('changes', [dict(duration_s=0.), dict(duration_s=-.1),
    dict(maximum_speed_rad_s=0.), dict(maximum_acceleration_rad_s2=0.),
    dict(minimum_theta_rad=0.), dict(maximum_theta_rad=-3.)])
def test_nonphysical_duration_caps_or_workspace_are_rejected(scheduled_boundary, changes):
    with pytest.raises(ValueError, match='positive and workspace ordered'):
        make_brake(scheduled_boundary, **changes)


def test_unrepresentable_tiny_duration_is_rejected_as_nonfinite_derived_motion(scheduled_boundary):
    with pytest.raises(ValueError, match='remain finite'):
        make_brake(scheduled_boundary, duration_s=1e-300)


def test_already_stationary_command_is_a_degenerate_constant_schedule():
    brake = braking.C2ReverseBrake(-1., 0., 0., .15)
    assert brake.displacement_rad == 0.
    assert brake.peak_speed_rad_s == brake.peak_acceleration_rad_s2 == brake.peak_jerk_rad_s3 == 0.
    assert all(brake.sample(t) == (-1., 0., 0., 0.) for t in (-1., 0., .025, .15, 1e6))


def test_curve_and_samples_are_immutable_and_only_accept_command_state(scheduled_boundary):
    brake = make_brake(scheduled_boundary)
    with pytest.raises(FrozenInstanceError):
        brake.theta0_rad = 0.
    with pytest.raises(AttributeError):
        brake.sample(.025).theta_rad = 0.
    with pytest.raises(TypeError):
        make_brake(scheduled_boundary, bolt_yaw_rad=0.)
    with pytest.raises(TypeError):
        make_brake(scheduled_boundary, pitch_m=.00125)
