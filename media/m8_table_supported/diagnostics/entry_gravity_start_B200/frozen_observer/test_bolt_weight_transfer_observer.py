"""Output-only diagnostic observer checks; outside the producer's 213 tests."""
import importlib.util
from pathlib import Path

import mujoco
import numpy as np
import pytest

SOURCE = Path(__file__).with_name('bolt_weight_transfer_observer.py')
spec = importlib.util.spec_from_file_location('output_weight_observer', SOURCE)
observer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(observer)


def sample(thread=.95, hand=.05, interior=0, **updates):
    result = dict(thread_gravity_opposing_force_N=thread,
        hand_gravity_opposing_force_N=hand,
        relative_bolt_axial_velocity_m_per_s=0., radial_offset_m=10e-6,
        bolt_tilt_rad=.001, bolt_world_support_contact_count=0,
        nonthread_block_bolt_contact_count=0, external_drive_zero=True,
        native_thread_contact_count=8,
        loaded_actual_interior_flank_contact_count=interior)
    result.update(updates)
    return result


def fill(window, row, *, start=0., count=101, dt=.001):
    for index in range(count):
        report = window.observe(start+(index+1)*dt, dt, row)
    return report


def test_normal_force_alone_does_not_establish_bolt_weight_transfer():
    # The actual failed v2 pre-open state supports only net feed, while the
    # pads still bear most gravity. High summed normals are not axial support.
    window = observer.BoltWeightTransferWindow(1.)
    report = fill(window, sample(.20, .80, thread_summed_normal_force_N=100.))
    assert not report['ready_for_diagnostic_release_attempt']
    assert report['mean_thread_weight_fraction'] == pytest.approx(.20)
    assert report['mean_positive_hand_upward_weight_fraction'] == pytest.approx(.80)


def test_full_weight_on_entry_cone_remains_explicitly_unqualified_capture():
    report = fill(observer.BoltWeightTransferWindow(1.), sample())
    assert report['ready_for_diagnostic_release_attempt']
    assert report['entry_only_native_contact_window']
    assert not report['loaded_actual_interior_flank_contact_observed']
    assert 'never a full-flank capture' in report['scope']


def test_actual_partial_interior_contacts_are_distinguished_from_entry():
    report = fill(observer.BoltWeightTransferWindow(1.), sample(interior=1))
    assert report['ready_for_diagnostic_release_attempt']
    assert not report['entry_only_native_contact_window']
    assert report['loaded_actual_interior_flank_contact_observed']
    assert report['loaded_actual_interior_flank_duration_s'] >= .1-1e-12


def test_downward_hand_force_cannot_cancel_upward_hand_support():
    window = observer.BoltWeightTransferWindow(1.)
    for index in range(101):
        report = window.observe((index+1)*.001, .001,
            sample(hand=.3 if index % 2 else -.3))
    assert report['mean_positive_hand_upward_weight_fraction'] > .1
    assert not report['ready_for_diagnostic_release_attempt']


@pytest.mark.parametrize('bad', [
    {'radial_offset_m':151e-6}, {'bolt_tilt_rad':np.deg2rad(2.01)},
    {'relative_bolt_axial_velocity_m_per_s':.000201},
    {'bolt_world_support_contact_count':1},
    {'nonthread_block_bolt_contact_count':1}, {'external_drive_zero':False}])
def test_actual_bad_geometry_motion_or_hidden_support_resets_window(bad):
    window = observer.BoltWeightTransferWindow(1.)
    assert fill(window, sample())['ready_for_diagnostic_release_attempt']
    invalid = window.observe(.102, .001, sample(**bad))
    assert not invalid['ready_for_diagnostic_release_attempt']
    assert invalid['original_native_samples'] == 0
    assert not fill(window, sample(), start=.102, count=99)['ready_for_diagnostic_release_attempt']


def test_current_unloaded_endpoint_prevents_release_even_if_mean_is_high():
    window = observer.BoltWeightTransferWindow(1.)
    fill(window, sample(1., 0.))
    report = window.observe(.102, .001, sample(0., 0.))
    assert report['mean_thread_weight_fraction'] >= .9
    assert not report['ready_for_diagnostic_release_attempt']


def test_rolling_o1_impulses_match_independent_direct_array_integrals():
    window = observer.BoltWeightTransferWindow(.262,
        duration_s=.100, minimum_thread_weight_fraction=.1)
    for index in range(2500):
        time = (index+1)*.00005
        row = sample(thread=.25+.04*np.sin(index), hand=.03*np.cos(index),
            interior=index % 3 == 0)
        report = window.observe(time, .00005, row)
    rows = list(window.samples)
    assert report['observed_window_s'] == pytest.approx(sum(r[1] for r in rows), abs=1e-12)
    assert report['signed_thread_gravity_opposing_impulse_Ns'] == pytest.approx(
        sum(dt*row['thread_gravity_opposing_force_N'] for _, dt, row in rows), abs=1e-12)
    assert report['positive_hand_upward_impulse_Ns'] == pytest.approx(
        sum(dt*max(row['hand_gravity_opposing_force_N'], 0.) for _, dt, row in rows), abs=1e-12)


def test_missing_native_step_and_nonfinite_force_are_rejected():
    window = observer.BoltWeightTransferWindow(1.)
    window.observe(.001, .001, sample())
    with pytest.raises(ValueError, match='contiguous'):
        window.observe(.003, .001, sample())
    with pytest.raises(ValueError, match='finite'):
        window.observe(.002, .001, sample(thread=np.nan))


def test_contact_frame_transform_uses_all_force_components_and_action_reaction():
    frame = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
    local = np.array([2., 3., 5., 7., 11., 13.])
    origin, position = np.zeros(3), np.array([.1, -.2, .3])
    expected_force = np.array([3., 5., 2.])
    expected_torque = np.array([11., 13., 7.])+np.cross(position, expected_force)
    result = observer.contact_world_wrench(frame, local, position, origin, 1.)
    np.testing.assert_allclose(result, np.r_[expected_force, expected_torque])
    np.testing.assert_allclose(observer.contact_world_wrench(frame, local, position, origin, -1.), -result)


def test_original_native_ground_contact_reaction_on_dynamic_body_is_upward():
    # A genuine native solve checks MuJoCo's contact sign convention. These
    # primitive test geoms exercise wrench extraction, not M8 thread physics.
    from thread_lab.model import ThreadConfig
    model = mujoco.MjModel.from_xml_string('''<mujoco><option gravity="0 0 -9.81"/>
      <worldbody><body name="female_frame"><geom name="female_thread"
        type="plane" size="1 1 .1"/></body>
      <body name="male_bolt" pos="0 0 .0039"><freejoint/>
        <geom name="bolt_thread" type="sphere" size=".004" mass=".02675"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    measured = observer.resolved_thread_wrench_on_bolt(model, data, ThreadConfig(), with_records=True)
    assert measured['native_thread_contact_count'] > 0
    assert measured['thread_gravity_opposing_force_N'] > 0.
    assert measured['thread_summed_normal_force_N'] > 0.
    record = measured['native_contact_records'][0]
    assert record['wrench_on_bolt_world_N_Nm'][2] > 0.
    assert not record['is_actual_interior_flank_contact']
