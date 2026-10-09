"""Focused arithmetic/contact-frame regressions; no native integration."""
import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('audit', Path(__file__).with_name(
    'independent_archived_native_force_audit.py'))
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


def ledger(thread, hand, *, interior=False):
    thread, hand = np.asarray(thread), np.asarray(hand)
    n = len(thread)
    return dict(time_s=np.arange(1, n+1)*.05,
        thread_gravity_opposing_force_N=thread, hand_gravity_opposing_force_N=hand,
        relative_bolt_axial_velocity_m_per_s=np.zeros(n), radial_offset_m=np.zeros(n),
        bolt_tilt_rad=np.zeros(n), external_drive_zero=np.ones(n),
        bolt_world_support_contact_count=np.zeros(n), nonthread_block_bolt_contact_count=np.zeros(n),
        all_checks_held=np.ones(n), formed_flank_overlap_m=np.full(n, 1e-5 if interior else 0.),
        loaded_actual_interior_flank_contact_count=np.full(n, int(interior)))


def test_signed_cancellation_cannot_hide_positive_hand_support():
    raw = ledger([1., 1., 1.], [.4, -.4, .4])
    result = audit.native_weight_windows(raw, 1., .05)
    np.testing.assert_allclose(result['positive_hand'], [.2, .2])
    assert not result['ready'].any()


def test_unloaded_final_tick_prevents_ready_despite_high_mean_load():
    raw = ledger([2., 0.], [0., 0.])
    result = audit.native_weight_windows(raw, 1., .05)
    assert result['thread'][0] == 1.
    assert not result['ready'][0]


def test_cone_only_weight_transfer_is_separate_from_continuous_formed_support():
    raw = ledger([1., 1., 1.], [0., 0., 0.])
    assert audit.native_weight_windows(raw, 1., .05)['ready'].all()
    assert not audit.native_weight_windows(raw, 1., .05, require_interior=True)['ready'].any()
    raw['formed_flank_overlap_m'][:] = 1e-5
    raw['loaded_actual_interior_flank_contact_count'][:] = 1
    assert audit.native_weight_windows(raw, 1., .05, require_interior=True)['ready'].all()


@pytest.mark.parametrize('field,value', [
    ('relative_bolt_axial_velocity_m_per_s', .000201),
    ('bolt_world_support_contact_count', 1.),
    ('external_drive_zero', 0.),
    ('all_checks_held', 0.),
])
def test_any_bad_native_tick_invalidates_whole_weight_window(field, value):
    raw = ledger([1., 1., 1.], [0., 0., 0.], interior=True)
    raw[field][1] = value
    assert not audit.native_weight_windows(raw, 1., .05, require_interior=True)['ready'].any()


@pytest.mark.parametrize('male_first', [True, False])
def test_original_contact_force_action_reaction_frame_and_lever_arm(male_first):
    sign = -1 if male_first else 1
    wrench = sign*np.array([-3., 2., 4., -.2, -3.9, 2.3])
    row = dict(native_contact_records=[dict(
        geom1='bolt_thread' if male_first else 'female_thread',
        geom2='female_thread' if male_first else 'bolt_thread',
        frame=[[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]],
        local_force_N_Nm=[2., 3., 4., .1, .2, .3],
        contact_position_world_m=[2., 2., 3.], bolt_origin_world_m=[1., 2., 3.],
        wrench_on_bolt_world_N_Nm=wrench.tolist(), is_actual_interior_flank_contact=True)])
    total, interior, loaded, error = audit.transformed_thread_contacts(row)
    np.testing.assert_allclose(total, wrench)
    np.testing.assert_allclose(interior, wrench)
    assert loaded == 1 and error < 1e-14


def test_interior_force_uses_recorded_point_tag_without_changing_full_load():
    c = dict(geom1='female_thread', geom2='bolt_thread', frame=np.eye(3).tolist(),
        local_force_N_Nm=[1., 0., 0., 0., 0., 0.], contact_position_world_m=[0., 0., 0.],
        bolt_origin_world_m=[0., 0., 0.], wrench_on_bolt_world_N_Nm=[1., 0., 0., 0., 0., 0.],
        is_actual_interior_flank_contact=False)
    total, interior, loaded, error = audit.transformed_thread_contacts(dict(native_contact_records=[c]))
    assert total[0] == 1. and loaded == 0 and not interior.any() and error == 0.
