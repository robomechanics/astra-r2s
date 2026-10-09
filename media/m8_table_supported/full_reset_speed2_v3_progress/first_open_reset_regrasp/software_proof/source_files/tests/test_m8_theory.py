"""Independent mechanical invariants for the analytical thread predictions."""
import math

import pytest

from thread_lab.theory import (
    contact_pitch_diameter,
    effective_friction,
    is_self_locking,
    lead_angle,
    lowering_torque,
    raising_torque,
    self_locking_threshold,
)


@pytest.mark.parametrize("diameter", [0.004, 0.007184, 0.016])
@pytest.mark.parametrize("starts", [1, 2, 3])
def test_frictionless_work_equals_load_times_advance(diameter, starts):
    # Conservation of work is independent of the wedge-friction derivation.
    load, pitch = 7.3, 0.00125
    work_per_turn = raising_torque(
        load, 0, pitch=pitch, pitch_diameter=diameter, starts=starts,
    ) * 2 * math.pi
    assert work_per_turn == pytest.approx(load * pitch * starts, rel=1e-13)
    assert lowering_torque(
        load, 0, pitch=pitch, pitch_diameter=diameter, starts=starts,
    ) * 2 * math.pi == pytest.approx(-load * pitch * starts, rel=1e-13)


def test_square_thread_is_equivalent_to_inclined_plane():
    # For a square thread, combine lead and friction angles using tan(a+b).
    diameter, pitch, friction, load = 0.007184, 0.00125, 0.15, 10.0
    lam = math.atan(pitch / (math.pi * diameter))
    phi = math.atan(friction)
    expected_raise = load * diameter / 2 * math.tan(lam + phi)
    expected_lower = load * diameter / 2 * math.tan(phi - lam)
    assert raising_torque(load, friction, half_angle=0) == pytest.approx(expected_raise)
    assert lowering_torque(load, friction, half_angle=0) == pytest.approx(expected_lower)


def test_metric_half_angle_increases_frictional_torque():
    assert effective_friction(0.15) > 0.15
    assert raising_torque(10, 0.15) > raising_torque(10, 0.15, half_angle=0)


def test_self_lock_threshold_agrees_with_lowering_torque_sign():
    threshold = self_locking_threshold()
    # This literal target is derived from the specified M8 geometry, not a
    # simulator state or controller's programmed success value.
    assert threshold == pytest.approx(0.04796500, rel=1e-6)
    assert not is_self_locking(0)
    assert not is_self_locking(threshold)
    assert lowering_torque(10, threshold) == pytest.approx(0, abs=1e-17)
    assert lowering_torque(10, threshold * 0.99) < 0
    assert not is_self_locking(threshold * 0.99)
    assert lowering_torque(10, threshold * 1.01) > 0
    assert is_self_locking(threshold * 1.01)
    assert is_self_locking(0.15)


def test_torque_is_linear_in_load_and_friction_costs_positive_work():
    frictionless = raising_torque(10, 0)
    assert raising_torque(10, 0.15) > frictionless
    for friction in (0.05, 0.08, 0.15, 0.25):
        assert raising_torque(10, friction) == pytest.approx(10 * raising_torque(1, friction))
        assert lowering_torque(10, friction) == pytest.approx(10 * lowering_torque(1, friction))
        # Raise/lower a load through equal distance: net energy is dissipated.
        assert raising_torque(10, friction) + lowering_torque(10, friction) > 0


def test_mean_pitch_radius_and_lead_angle_are_explicit_assumptions():
    assert contact_pitch_diameter() == pytest.approx(0.007184)
    assert contact_pitch_diameter(0.01, 0.012) == pytest.approx(0.011)
    assert lead_angle() == pytest.approx(math.radians(3.17010), rel=2e-6)


@pytest.mark.parametrize("call", [
    lambda: contact_pitch_diameter(-0.001, 0.01),
    lambda: contact_pitch_diameter(0.01, 0.009),
    lambda: raising_torque(-1),
    lambda: lowering_torque(1, -0.1),
    lambda: raising_torque(1, float("nan")),
    lambda: lead_angle(pitch=0),
    lambda: lead_angle(starts=1.5),
    lambda: lead_angle(starts=True),
    lambda: lead_angle(starts=float("inf")),
    lambda: effective_friction(0.1, math.pi / 2),
    lambda: raising_torque(1, 100),
])
def test_invalid_physical_inputs_fail_explicitly(call):
    with pytest.raises(ValueError):
        call()
