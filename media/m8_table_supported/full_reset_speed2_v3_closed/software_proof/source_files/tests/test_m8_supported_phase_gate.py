"""Live native events end bounded phases at their measured timestamps."""
import numpy as np
import pytest
from dataclasses import replace

from yam_twin.m8_supported_start import ImpulseSeatDropWindow, bounded_phase_gate


def test_fresh_event_between_old_nominal_endpoints_is_consumed_at_its_native_tick():
    event = ImpulseSeatDropWindow(0., 1.)
    timestep = .00005
    first_completion = None
    nominal_endpoint_report = None
    for tick in range(1, 2401):
        actual_time = tick*timestep
        # Earlier measured contacts establish >=10% impulse/load coverage,
        # but the final-tick condition is false through the minimum dwell.
        # A new actual loaded tick occurs between nominal phase endpoints.
        force = 1. if 1400 <= tick < 1580 or tick == 2357 else 0.
        report = event.observe(actual_time, timestep, 60e-6, 0., force,
            actual_relative_angular_speed_rad_per_s=0., valid=True, physically_stopped=True)
        decision = bounded_phase_gate(actual_time, .100, .200,
                                      report['confirmed_search_direction_event'])
        if decision == 'complete' and first_completion is None:
            first_completion = actual_time
        if tick == 2400:
            nominal_endpoint_report = report
    assert first_completion == pytest.approx(.11785, abs=1e-12)
    assert first_completion != .120
    assert not nominal_endpoint_report['confirmed_search_direction_event']
    # An old endpoint-only check would miss this event. The next phase must
    # start after the actual .11785 s observation, without rewriting time.


def test_readiness_before_minimum_dwell_does_not_bypass_it_or_latch():
    assert bounded_phase_gate(.040, .050, .200, True) == 'continue'
    assert bounded_phase_gate(.050, .050, .200, False) == 'continue'
    assert bounded_phase_gate(.07135, .050, .200, True) == 'complete'


def test_current_unloaded_native_tick_cannot_reuse_previous_ready_event():
    event = ImpulseSeatDropWindow(0., 1.)
    for tick in range(1, 51):
        report = event.observe(tick*.001, .001, 60e-6, 0., 1.,
            actual_relative_angular_speed_rad_per_s=0., valid=True, physically_stopped=True)
    assert report['confirmed_search_direction_event']
    assert bounded_phase_gate(.050, .060, .100, report['confirmed_search_direction_event']) == 'continue'
    for tick in range(51, 61):
        report = event.observe(tick*.001, .001, 60e-6, 0., 0.,
            actual_relative_angular_speed_rad_per_s=0., valid=True, physically_stopped=True)
    assert report['original_native_impulse_window']['normal_impulse_Ns'] >= .005
    assert not report['confirmed_search_direction_event']
    assert bounded_phase_gate(.060, .060, .100, report['confirmed_search_direction_event']) == 'continue'


def test_timeout_uses_current_physical_stop_state_even_with_drop_and_large_load():
    event = ImpulseSeatDropWindow(0., 1.)
    decision = None
    for tick in range(1, 101):
        elapsed = tick*.001
        report = event.observe(elapsed, .001, 60e-6, 0., 100.,
            actual_relative_angular_speed_rad_per_s=.02, valid=True, physically_stopped=True)
        decision = bounded_phase_gate(elapsed, .050, .100,
                                      report['confirmed_search_direction_event'])
        if tick < 100:
            assert decision == 'continue'
    assert report['stop_requested_from_measured_drop']
    assert decision == 'timeout'
    assert not report['is_engagement_or_release_proof']


def test_fresh_completion_has_priority_on_the_exact_maximum_tick():
    assert bounded_phase_gate(.100, .050, .100, True) == 'complete'
    assert bounded_phase_gate(.100, .050, .100, False) == 'timeout'


def test_invalid_observation_cannot_authorize_transition_even_at_timeout():
    assert bounded_phase_gate(.100, .050, .100, True, valid=False) == 'invalid'
    assert bounded_phase_gate(.100, .050, .100, False, valid=False) == 'invalid'


def test_offgrid_minimum_waits_for_the_next_actual_observation():
    times = np.arange(1, 5)*.003
    decisions = [bounded_phase_gate(t, .0074, .0127, True) for t in times]
    assert decisions[:3] == ['continue', 'continue', 'complete']
    assert times[2] == pytest.approx(.009)
    assert times[2] != .0074


def test_offgrid_timeout_occurs_on_the_first_actual_tick_crossing_the_bound():
    observed = []
    for tick in range(1, 10):
        actual_time = tick*.003
        decision = bounded_phase_gate(actual_time, .0074, .0127, False)
        observed.append((actual_time, decision))
        if decision == 'timeout':
            break
    assert observed[-1][0] == pytest.approx(.015)
    assert all(decision == 'continue' for _, decision in observed[:-1])
    assert observed[-1][0] != .0127


@pytest.mark.parametrize('elapsed,minimum,maximum', [
    (np.nan, .05, .1), (.05, np.inf, .1), (.05, .05, np.nan),
    (-.001, .05, .1), (.05, -.01, .1), (.05, .05, 0.), (.05, .2, .1),
])
def test_nonphysical_phase_bounds_are_rejected(elapsed, minimum, maximum):
    with pytest.raises(ValueError, match='finite, ordered and physical'):
        bounded_phase_gate(elapsed, minimum, maximum, True)


def test_supported_search_preserves_physical_pickup_prefix_and_uses_bounded_first_forward_clock():
    from yam_twin.m8_supported_simulation import SupportedControlConfig, supported_phases
    config = SupportedControlConfig()
    phases = supported_phases(config)
    assert [phase[0] for phase in phases[:12]] == [
        'settle_table', 'reach_left_block', 'close_left_block', 'settle_left_block',
        'secure_left', 'reach_bolt', 'close_bolt', 'settle_bolt', 'lift_bolt',
        'transport_bolt', 'align_over_hole', 'feed_to_entry',
    ]
    by_name = {phase[0]: phase for phase in phases}
    assert by_name['reverse_seat_1'][3] >= -2.7
    assert 0. < by_name['start_thread_1'][3] <= 1.
    # A first fixed pi stroke from a partial reverse-search event could exceed
    # the checked forward wrist workspace. Its scheduled destination is an
    # absolute bounded clock; subsequent strokes require real open/regrasp.
    assert by_name['start_thread_1'][3] < np.pi
    assert phases.index(by_name['settle_regrip_search_2']) < phases.index(by_name['start_thread_2'])
    assert config.transport_tip_clearance_m >= .035
    assert config.minimum_tip_rest_clearance_m >= .010


@pytest.mark.parametrize('forward_clock', [.3, .7, 1.])
def test_first_forward_plan_can_cover_full_reverse_workspace_without_exceeding_peak_speed(forward_clock):
    from yam_twin.m8_simulation import smooth_profile
    from yam_twin.m8_supported_simulation import SupportedControlConfig, supported_phases
    config = replace(SupportedControlConfig(), first_forward_clock_rad=forward_clock)
    first = next(phase for phase in supported_phases(config) if phase[0] == 'start_thread_1')
    # Evaluate physical clock displacement across the worst allowed actual
    # reverse stop, independently of the declared duration expression.
    origin = -config.maximum_reverse_angle_rad
    span = first[3]-origin
    profiles = np.array([smooth_profile(u) for u in np.linspace(0., 1., 101)])
    clocks = origin+span*profiles[:, 0]
    speeds = span*profiles[:, 1]/first[1]
    assert clocks[0] >= -2.7
    assert clocks[-1] == pytest.approx(forward_clock)
    assert np.max(clocks) <= 1.
    assert np.max(np.abs(speeds)) <= config.starting_angular_speed_rad_s+1e-12
    assert speeds[0] == speeds[-1] == 0.


def test_closed_thread_motion_and_open_reset_use_distinct_physical_axis_control():
    from yam_twin.m8_supported_simulation import SupportedControlConfig, supported_phases
    config = SupportedControlConfig()
    for name, duration, _, _, gap_start, gap_end, axial_float in supported_phases(config):
        assert duration > 0. and np.isfinite(duration)
        if name in ('transfer_bolt_weight', 'reverse_seat_1', 'stop_reverse_seat_1') or name.startswith(('start_thread_', 'stop_start_', 'turn_', 'stop_')):
            assert axial_float
            assert gap_start == gap_end == config.arm.closed_aperture
        if name.startswith(('release_', 'open_settle_', 'reset_open_', 'open_hold_', 'regrip_', 'settle_regrip_')):
            assert not axial_float
        if name.startswith(('open_settle_', 'reset_open_', 'open_hold_')):
            assert gap_start == gap_end == config.arm.open_aperture
        if name.startswith('release_'):
            assert gap_start == config.arm.closed_aperture
            assert gap_end == config.arm.open_aperture
        if name.startswith('regrip_'):
            assert gap_start == config.arm.open_aperture
            assert gap_end == config.arm.closed_aperture


@pytest.mark.parametrize('changes', [
    {'maximum_reverse_angle_rad': 2.700001}, {'first_forward_clock_rad': 1.000001},
    {'first_forward_clock_rad': 0.}, {'first_forward_clock_rad': np.nan},
    {'minimum_open_settle_s': .8, 'maximum_open_settle_s': .75},
    {'left_downward_hold_N': 8.001},
    {'left_open_aperture_m': .0183}, {'left_open_aperture_m': .0401},
    {'regrasp_window_s': .2501},
])
def test_unsupported_search_workspace_or_force_and_inverted_open_dwell_are_rejected(changes):
    from yam_twin.m8_supported_simulation import SupportedControlConfig
    with pytest.raises(ValueError, match='checked workspace/force bounds'):
        replace(SupportedControlConfig(), **changes)


@pytest.mark.parametrize('alignment_duration', [0., -1., np.nan, np.inf])
def test_head_alignment_requires_a_finite_positive_physical_duration(alignment_duration):
    from yam_twin.m8_supported_simulation import SupportedControlConfig
    with pytest.raises(ValueError, match='positive and finite'):
        replace(SupportedControlConfig(), head_alignment_s=alignment_duration)


def test_transfer_bound_covers_physical_press_alignment_centering_and_gravity_stages():
    from yam_twin.m8_supported_simulation import SupportedControlConfig, supported_phases
    for center, align in ((.15, .45), (.7, .3)):
        config = replace(SupportedControlConfig(), head_centering_s=center, head_alignment_s=align)
        phase = next(phase for phase in supported_phases(config) if phase[0] == 'transfer_bolt_weight')
        # Alignment/centering may overlap after pressing. The gravity ramp,
        # required hold and bounded settling must start after both finish.
        after_motion = config.left_hold_ramp_s+max(center, align)
        after_gravity_hold = after_motion+config.gravity_feed_ramp_s+config.gravity_feed_hold_s
        assert phase[1] >= after_gravity_hold+config.maximum_reference_settle_s-1e-12
        assert phase[6]


@pytest.mark.parametrize('stroke', [np.pi/2, 2*np.pi/3])
def test_opposite_flat_regrasp_schedule_rejects_incompatible_stroke_angles(stroke):
    from yam_twin.m8_supported_simulation import SupportedControlConfig
    config = SupportedControlConfig()
    with pytest.raises(ValueError, match='requires pi strokes'):
        replace(config, arm=replace(config.arm, stroke_angle_rad=stroke))
