"""Pure ignored V3 event tests, not native physics or the frozen 402-test proof."""
import ast
import importlib.util
import inspect
from pathlib import Path
import sys

import numpy as np
import pytest

from yam_twin.m8_supported_start import BoltWeightTransferWindow, ImpulseSeatDropWindow
from yam_twin.m8_supported_simulation import TableLoadWindow

SOURCE = Path(__file__).with_name('crest_seat_observer_v3.py')
SPEC = importlib.util.spec_from_file_location('crest_seat_observer_v3', SOURCE)
observer = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(observer)


@pytest.fixture(scope='module')
def cold_harness():
    """Load definitions only: main and native integration never execute."""
    path = SOURCE.with_name('crest_seat_search_probe_v3.py')
    spec = importlib.util.spec_from_file_location('yam_twin.ignored_crest_probe_test', path)
    module = importlib.util.module_from_spec(spec)
    previous = sys.modules.get('crest_seat_observer_v3')
    sys.modules['crest_seat_observer_v3'] = observer
    sys.modules[spec.name] = module  # dataclasses resolve their defining module.
    try:
        spec.loader.exec_module(module)
        yield module
    finally:
        sys.modules.pop(spec.name, None)
        if previous is None:
            sys.modules.pop('crest_seat_observer_v3', None)
        else:
            sys.modules['crest_seat_observer_v3'] = previous


@pytest.fixture(scope='module')
def actual_cold_table_guard(cold_harness):
    """Evaluate the real caller's load subguard without running its controller."""
    tree = ast.parse(inspect.getsource(cold_harness.run_supported_demo))
    matches = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.BoolOp) or not isinstance(node.op, ast.And):
            continue
        names = {item.id for item in ast.walk(node) if isinstance(item, ast.Name)}
        if names == {'left_grasp_acquired', 'table_bearing', 'task_table_window',
                     'cold_table_window_warmup', 'np', 'native_left_contact'}:
            matches.append(node)
    assert len(matches) == 1, 'The actual table-bearing hard guard must be identifiable'
    expression = compile(ast.Expression(matches[0]), str(SOURCE), 'eval')

    def evaluate(window, warmup, *, actual_table_force, weight=1., acquired=True):
        return eval(expression, {'np': np}, dict(left_grasp_acquired=acquired,
            table_bearing=actual_table_force > .1*weight, task_table_window=window,
            cold_table_window_warmup=warmup,
            native_left_contact={'pad_normal_force_N': np.array([1., 1.])}))
    return evaluate


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


@pytest.mark.parametrize('elapsed, expected', [(0., True), (50e-6, True),
    (.100-50e-6, True), (.100, False), (.100+50e-6, False)])
def test_cold_mean_exception_expires_at_real_100ms_coverage(cold_harness, elapsed, expected):
    assert cold_harness.cold_table_mean_unavailable('cold_window_hold', elapsed, .100) is expected


@pytest.mark.parametrize('phase', ['transfer_bolt_weight', 'reverse_seat_1',
    'stop_reverse_seat_1', 'start_thread_1', 'stop_start_1', 'release_search_2',
    'open_settle_search_2', 'reset_open_search_2', 'regrip_search_2', 'turn_1'])
def test_no_later_phase_can_reuse_a_cold_window_exception(cold_harness, phase):
    for elapsed in (0., 50e-6, .09995, .100):
        assert not cold_harness.cold_table_mean_unavailable(phase, elapsed, .100)


@pytest.mark.parametrize('elapsed, required', [(-50e-6, .1), (np.nan, .1),
    (np.inf, .1), (0., 0.), (0., -.1), (0., np.nan), (0., np.inf)])
def test_cold_mean_availability_rejects_nonphysical_durations(cold_harness, elapsed, required):
    with pytest.raises(ValueError, match='finite and physical'):
        cold_harness.cold_table_mean_unavailable('cold_window_hold', elapsed, required)


def native_table_samples(window, *, ticks=2000, table_force=lambda tick: 1.,
                         hand_force=lambda tick: .05):
    for tick in range(1, ticks+1):
        yield window.observe(table_force(tick), hand_force(tick), 50e-6, True)


def test_fresh_original_table_window_requires_all_100ms_before_mean_is_available(cold_harness):
    # This is the original observer, fed actual per-step force samples rather
    # than prefilled elapsed time, readiness, impulses, or old native history.
    window = TableLoadWindow(.100, 1.)
    assert window.report()['mean_table_upward_force_N'] is None
    for tick, ready in enumerate(native_table_samples(window), 1):
        unavailable = cold_harness.cold_table_mean_unavailable(
            'cold_window_hold', window.elapsed, window.duration)
        assert ready is (tick == 2000)
        assert unavailable is (tick < 2000)
    report = window.report()
    assert report['observer'] == 'native-supported-block-load-window-v1'
    assert report['observed_window_s'] == pytest.approx(.100)
    assert report['mean_table_upward_force_N'] == pytest.approx(1.)
    assert report['mean_positive_hand_upward_force_N'] == pytest.approx(.05)
    assert report['loaded_table_substep_duty'] == pytest.approx(1.)


def test_harness_keeps_original_90_10_99_native_window_contract(cold_harness):
    original = TableLoadWindow(.100, 1.)
    generated = cold_harness.TableLoadWindow(.100, 1.)
    # A genuine small contact interruption is permitted by original 99% duty.
    # Nineteen absent 50us ticks give 99.05%, avoiding float-boundary ambiguity.
    samples = dict(table_force=lambda tick: 0. if tick <= 19 else .95,
                   hand_force=lambda tick: .05)
    for before, actual in zip(native_table_samples(original, **samples),
                              native_table_samples(generated, **samples)):
        assert actual is before
    assert generated.ready
    assert generated.report() == original.report()
    report = original.report()
    assert report['minimum_mean_table_weight_fraction'] == .90
    assert report['maximum_mean_positive_hand_upward_weight_fraction'] == .10
    assert report['minimum_loaded_table_duty'] == .99
    assert report['loaded_table_substep_duty'] == pytest.approx(.9905)


def test_zero_current_table_force_is_an_instantaneous_hard_failure_during_warmup(
        cold_harness, actual_cold_table_guard):
    window = TableLoadWindow(.100, 1.)
    assert not window.observe(0., 0., 50e-6, True)
    unavailable = cold_harness.cold_table_mean_unavailable(
        'cold_window_hold', window.elapsed, window.duration)
    assert unavailable
    # Evaluate the unchanged real controller branch: a unavailable rolling
    # mean never exempts current native table-bearing force from its guard.
    assert actual_cold_table_guard(window, unavailable, actual_table_force=0.)


def test_cold_warmup_exempts_only_unavailable_mean_with_current_table_support(
        cold_harness, actual_cold_table_guard):
    window = TableLoadWindow(.100, 1.)
    window.observe(1., .05, 50e-6, True)
    assert not window.ready
    unavailable = cold_harness.cold_table_mean_unavailable(
        'cold_window_hold', window.elapsed, window.duration)
    assert unavailable
    assert not actual_cold_table_guard(window, unavailable, actual_table_force=1.)
    later = cold_harness.cold_table_mean_unavailable('reverse_seat_1', window.elapsed, window.duration)
    assert not later
    assert actual_cold_table_guard(window, later, actual_table_force=1.)


@pytest.mark.parametrize('table_force, hand_force', [
    (lambda tick: .899, lambda tick: 0.),
    (lambda tick: 1., lambda tick: .101),
    (lambda tick: 0. if tick <= 40 else 1., lambda tick: 0.),
    (lambda tick: 1., lambda tick: .30 if tick <= 1000 else -.30),
])
def test_bad_native_load_after_full_coverage_has_no_warmup_exemption(
        cold_harness, actual_cold_table_guard, table_force, hand_force):
    window = TableLoadWindow(.100, 1.)
    list(native_table_samples(window, table_force=table_force, hand_force=hand_force))
    assert window.elapsed == pytest.approx(.100)
    assert not window.ready
    unavailable = cold_harness.cold_table_mean_unavailable(
        'cold_window_hold', window.elapsed, window.duration)
    assert not unavailable
    assert actual_cold_table_guard(window, unavailable, actual_table_force=table_force(2000))


def test_ready_past_table_mean_cannot_hide_a_current_unloaded_endpoint(
        cold_harness, actual_cold_table_guard):
    window = TableLoadWindow(.100, 1.)
    list(native_table_samples(window))
    assert window.ready
    window.observe(0., 0., 50e-6, True)
    report = window.report()
    assert report['mean_table_upward_force_N'] > .90
    assert report['loaded_table_substep_duty'] > .99
    assert not window.ready
    unavailable = cold_harness.cold_table_mean_unavailable(
        'cold_window_hold', window.elapsed, window.duration)
    assert not unavailable
    assert actual_cold_table_guard(window, unavailable, actual_table_force=0.)


def test_invalid_table_observation_discards_old_readiness_and_requires_fresh_native_coverage(cold_harness):
    window = TableLoadWindow(.100, 1.)
    list(native_table_samples(window))
    assert window.ready
    window.observe(1., .05, 50e-6, False)
    assert not window.ready
    assert window.elapsed == 0.
    assert window.report()['mean_table_upward_force_N'] is None
    assert window.table_impulse == window.hand_upward_impulse == window.loaded_duration == 0.
    list(native_table_samples(window, ticks=1999))
    assert not window.ready
    assert cold_harness.cold_table_mean_unavailable('cold_window_hold', window.elapsed, .100)
    window.observe(1., .05, 50e-6, True)
    assert window.ready
    assert not cold_harness.cold_table_mean_unavailable('cold_window_hold', window.elapsed, .100)
