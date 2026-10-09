"""Measured-crest direction contracts; no native physics qualification."""
import numpy as np
import pytest

from yam_twin import m8_supported_crest as observer
from yam_twin.m8_supported_start import BoltWeightTransferWindow, ImpulseSeatDropWindow


def observe(event, time, *, dt=.001, base=-40e-6, force=1., axial=0., angular=0., valid=True, stopped=True):
    return event.observe(time, dt, base, axial, force,
        actual_relative_angular_speed_rad_per_s=angular,
        valid=valid, physically_stopped=stopped)


def request_from_crest(event):
    observe(event, .001, base=0., stopped=False)
    observe(event, .002, base=-100e-6, axial=-.001, stopped=False)
    report = observe(event, .003, base=-40e-6, force=0., axial=.0014, angular=.5, stopped=False)
    assert report['stop_requested_from_measured_return']
    assert not report['confirmed_search_direction_event']
    return report


def stopped_window(event, *, start=.003, count=100, force=None, **updates):
    report = None
    for tick in range(1, count+1):
        value = force(tick) if callable(force) else 1. if force is None else force
        report = observe(event, start+tick*.001, force=value, **updates)
    return report


def test_valley_crest_return_is_distinct_from_advance_beyond_original_settled_depth():
    # Representative measured amplitudes from the closed original failure,
    # submitted as synthetic contiguous observer inputs. This is an event
    # contract test, never a counterfactual native trajectory qualification.
    reference = -.022599933331474656
    event = observer.CrestSeatDropWindowV3(reference, .26241400403581094)
    old = ImpulseSeatDropWindow(reference, .26241400403581094)
    time = 0.
    def both(base, axial=0., stopped=False, force=0.):
        nonlocal time
        time += .001
        values = dict(actual_relative_angular_speed_rad_per_s=0. if stopped else .5,
                      valid=True, physically_stopped=stopped)
        a = event.observe(time, .001, base, axial, force, **values)
        b = old.observe(time, .001, base, axial, force, **values)
        return a, b
    both(reference)
    crest = reference-653.011e-6
    for base in np.linspace(reference, crest, 660)[1:]:
        both(base, axial=-.001)
    final = reference+19.12141233520459e-6
    for base in np.linspace(crest, final, 674)[1:]:
        both(base, axial=.001)
    for _ in range(100):
        new, original = both(final, stopped=True, force=.35)
    assert new['stop_requested_from_measured_return']
    assert new['confirmed_search_direction_event']
    assert new['actual_crest_return_m'] > 650e-6
    assert new['actual_crest_base_z_m'] == pytest.approx(crest)
    assert new['original_settled_reference_base_z_m'] == reference
    assert original['actual_axial_drop_m'] < 50e-6
    assert not original['stop_requested_from_measured_drop']
    assert not original['confirmed_search_direction_event']
    assert not new['is_engagement_or_release_proof']


def test_unloaded_moving_return_can_only_request_closed_deceleration():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    report = request_from_crest(event)
    assert report['actual_crest_base_z_m'] == pytest.approx(-100e-6)
    assert report['actual_crest_time_s'] == pytest.approx(.002)
    assert report['first_actual_return_time_s'] == pytest.approx(.003)
    report = stopped_window(event, force=0.)
    assert not report['confirmed_search_direction_event']
    assert report['original_native_impulse_window']['normal_impulse_Ns'] == 0.
    assert not report['is_engagement_or_release_proof']


def test_exact_actual_50um_return_keeps_the_threshold_unchanged():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    observe(event, .001, base=0., stopped=False)
    observe(event, .002, base=-50e-6, stopped=False)
    below = observe(event, .003, base=-.5e-6, stopped=False)
    assert not below['stop_requested_from_measured_return']
    exact = observe(event, .004, base=0., force=0., stopped=False)
    assert exact['actual_crest_return_m'] == 50e-6
    assert exact['stop_requested_from_measured_return']
    assert not exact['confirmed_search_direction_event']


def test_no_measured_bolt_return_cannot_be_replaced_by_tool_motion_or_elapsed_time():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    for tick in range(1, 1001):
        # The tool is not an observer input. Even a commanded tool scan does
        # not count unless the actual bolt/female base coordinate returns.
        report = observe(event, tick*.001, base=0., stopped=False, angular=.5)
    assert not report['stop_requested_from_measured_return']
    assert not report['confirmed_search_direction_event']


def test_subthreshold_sawtooth_motion_is_not_summed_into_a_false_drop():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    for tick in range(1, 1001):
        base = -40e-6 if tick % 2 else 0.
        report = observe(event, tick*.001, base=base, stopped=False, angular=.5)
    assert report['actual_crest_return_m'] == pytest.approx(40e-6)
    assert not report['stop_requested_from_measured_return']


def test_requested_crest_is_frozen_and_later_motor_rebound_cannot_ratchet_it():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    request_from_crest(event)
    original_crest = event.report()['actual_crest_base_z_m']
    observe(event, .004, base=-200e-6)
    for tick in range(5, 106):
        report = observe(event, tick*.001, base=-140e-6)
    assert report['actual_crest_base_z_m'] == original_crest
    assert report['actual_crest_return_m'] < 0.
    assert not report['confirmed_search_direction_event']


def test_original_native_impulse_and_duty_allow_real_contact_gaps_but_need_100ms():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    request_from_crest(event)
    for tick in range(1, 101):
        report = observe(event, .003+tick*.001, force=1. if tick%5 == 0 else 0.)
        if tick < 100:
            assert not report['confirmed_search_direction_event']
    assert report['confirmed_search_direction_event']
    window = report['original_native_impulse_window']
    assert window['observer'] == 'native-settled-entry-support-v1'
    assert window['required_window_s'] == pytest.approx(.100)
    assert window['minimum_normal_impulse_Ns'] == pytest.approx(.010)
    assert window['minimum_loaded_duration_s'] == pytest.approx(.010)
    assert window['minimum_loaded_normal_force_N'] == pytest.approx(.1)
    assert window['loaded_duration_s'] == pytest.approx(.020)
    assert window['loaded_substep_duty'] == pytest.approx(.2)
    assert window['normal_impulse_Ns'] == pytest.approx(.020)
    assert not report['is_engagement_or_release_proof']


def test_large_single_contact_spike_cannot_replace_original_loaded_duration():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    request_from_crest(event)
    report = stopped_window(event, force=lambda tick: 100. if tick == 100 else 0.)
    window = report['original_native_impulse_window']
    assert window['normal_impulse_Ns'] > window['minimum_normal_impulse_Ns']
    assert window['loaded_duration_s'] < window['minimum_loaded_duration_s']
    assert not report['confirmed_search_direction_event']


def test_loaded_duration_cannot_replace_required_native_impulse():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    request_from_crest(event)
    report = stopped_window(event, force=lambda tick: .11 if tick%5 == 0 else 0.)
    window = report['original_native_impulse_window']
    assert window['loaded_duration_s'] >= window['minimum_loaded_duration_s']
    assert window['normal_impulse_Ns'] < window['minimum_normal_impulse_Ns']
    assert not report['confirmed_search_direction_event']


def test_current_unloaded_endpoint_cancels_direction_readiness_despite_large_past_impulse():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    request_from_crest(event)
    assert stopped_window(event)['confirmed_search_direction_event']
    report = observe(event, .104, force=0.)
    assert report['original_native_impulse_window']['normal_impulse_Ns'] > .010
    assert not report['confirmed_search_direction_event']
    assert observe(event, .105)['confirmed_search_direction_event']


@pytest.mark.parametrize('bad', [dict(base=-51e-6), dict(axial=.000201),
    dict(axial=-.000201), dict(angular=.011), dict(stopped=False)])
def test_persistent_return_or_actual_quiet_loss_restarts_full_stopped_window(bad):
    event = observer.CrestSeatDropWindowV3(0., 1.)
    request_from_crest(event)
    assert stopped_window(event)['confirmed_search_direction_event']
    invalid = observe(event, .104, **bad)
    assert not invalid['confirmed_search_direction_event']
    assert invalid['original_native_impulse_window']['observed_window_s'] == 0.
    assert not stopped_window(event, start=.104, count=99)['confirmed_search_direction_event']
    assert observe(event, .204)['confirmed_search_direction_event']


def test_commanded_stop_cannot_replace_actual_angular_stop():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    request_from_crest(event)
    report = stopped_window(event, force=100., angular=.03, stopped=True)
    assert not report['confirmed_search_direction_event']
    assert report['original_native_impulse_window']['observed_window_s'] == 0.


def test_invalid_or_noisy_crest_cannot_seed_a_return_in_a_new_valid_epoch():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    observe(event, .001, base=0., stopped=False)
    invalid = observe(event, .002, base=-.001, valid=False, stopped=False)
    assert invalid['actual_crest_base_z_m'] is None
    report = observe(event, .003, base=0., stopped=False)
    assert report['actual_crest_time_s'] == pytest.approx(.003)
    assert report['actual_crest_return_m'] == 0.
    assert not report['stop_requested_from_measured_return']
    assert report['original_settled_reference_base_z_m'] == 0.


@pytest.mark.parametrize('gap_time', [.002, .003, .005])
def test_repeated_backward_or_missing_native_tick_clears_all_crest_and_load_history(gap_time):
    event = observer.CrestSeatDropWindowV3(0., 1.)
    request_from_crest(event)
    with pytest.raises(ValueError, match='contiguous native timestep'):
        observe(event, gap_time)
    report = event.report()
    assert report['actual_crest_base_z_m'] is None
    assert not report['stop_requested_from_measured_return']
    assert not report['confirmed_search_direction_event']
    assert report['original_native_impulse_window']['observed_window_s'] == 0.
    assert report['invalid_or_discontinuous_epoch'] == 1


def test_new_invalid_grasp_epoch_cannot_reuse_a_previous_ready_direction_event():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    request_from_crest(event)
    assert stopped_window(event)['confirmed_search_direction_event']
    observe(event, .104, valid=False)
    report = stopped_window(event, start=.104, count=101)
    assert not report['stop_requested_from_measured_return']
    assert not report['confirmed_search_direction_event']
    assert report['actual_crest_base_z_m'] == pytest.approx(-40e-6)


def test_normal_impulse_direction_confirmation_does_not_replace_separate_axial_weight_support():
    event = observer.CrestSeatDropWindowV3(0., 1.)
    request_from_crest(event)
    assert stopped_window(event)['confirmed_search_direction_event']
    weight = BoltWeightTransferWindow(1.)
    sample = dict(thread_gravity_opposing_force_N=.2,
        hand_gravity_opposing_force_N=.8, relative_bolt_axial_velocity_m_per_s=0.,
        radial_offset_m=0., bolt_tilt_rad=0., bolt_world_support_contact_count=0,
        nonthread_block_bolt_contact_count=0, external_drive_zero=True,
        native_thread_contact_count=4, loaded_actual_interior_flank_contact_count=0)
    for tick in range(1, 102):
        report = weight.observe(tick*.001, .001, sample)
    assert not report['ready_for_diagnostic_release_attempt']
    assert report['entry_only_native_contact_window']
    assert not report['loaded_actual_interior_flank_contact_observed']


@pytest.mark.parametrize('field', ['net_feed_N', 'minimum_drop_m', 'required_stop_s',
    'velocity_limit_m_per_s', 'angular_velocity_limit_rad_per_s'])
@pytest.mark.parametrize('bad', [0., np.nan])
def test_crest_thresholds_are_finite_and_positive(field, bad):
    values = dict(reference_base_z_m=0., net_feed_N=1.)
    values[field] = bad
    with pytest.raises(ValueError, match='finite and positive'):
        observer.CrestSeatDropWindowV3(**values)


@pytest.mark.parametrize('bad', [dict(base=np.nan), dict(force=-1.),
    dict(angular=-.01), dict(axial=np.inf), dict(dt=0.)])
def test_nonphysical_samples_clear_an_already_requested_epoch_before_rejection(bad):
    event = observer.CrestSeatDropWindowV3(0., 1.)
    request_from_crest(event)
    with pytest.raises(ValueError, match='finite native physical values'):
        observe(event, .004, **bad)
    report = event.report()
    assert report['actual_crest_base_z_m'] is None
    assert not report['stop_requested_from_measured_return']
    assert not report['confirmed_search_direction_event']


