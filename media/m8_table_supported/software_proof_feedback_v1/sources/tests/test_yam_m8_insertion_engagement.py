"""Contact-gap, impulse, time-resolution, and phase-drift observer checks."""
import numpy as np

from yam_twin.m8_insertion_engagement import LoadedFlankWindow


def test_real_unilateral_force_gaps_can_form_capture_window():
    window = LoadedFlankWindow()
    dt = .001
    for i in range(201):
        loaded = i % 2 == 0
        result = window.update((i+1)*dt, dt, True, .020 if loaded else 0.,
                               2 if loaded else 0, 40e-6*np.sin(i/20))
    assert result["ready"]
    assert .4 < result["loaded_substep_duty"] < .6
    assert result["normal_impulse_Ns"] >= .001
    assert result["helix_phase_range_m"] < 150e-6


def test_geometry_without_loaded_force_does_not_capture():
    window = LoadedFlankWindow()
    for i in range(250):
        result = window.update((i+1)*.001, .001, True, 0., 0, 0.)
    assert not result["ready"]
    assert result["normal_impulse_Ns"] == 0.


def test_actual_geometry_break_clears_old_impulse_and_duration():
    window = LoadedFlankWindow()
    for i in range(250):
        result = window.update((i+1)*.001, .001, True, .05, 2, 0.)
    assert result["ready"]
    result = window.update(.251, .001, False, .05, 2, 0.)
    assert not result["ready"]
    assert result["normal_impulse_Ns"] == 0.
    assert result["continuous_geometry_elapsed_s"] == 0.


def test_large_unrelated_axial_phase_motion_rejects_capture():
    window = LoadedFlankWindow()
    for i in range(250):
        phase = 200e-6 if i >= 200 else 0.
        result = window.update((i+1)*.001, .001, True, .05, 2, phase)
    assert not result["ready"]
    assert result["helix_phase_range_m"] == 200e-6


def test_loaded_duration_requirement_is_invariant_to_timestep_halving():
    def observed(dt, count):
        window = LoadedFlankWindow()
        steps = round(.20/dt)
        for i in range(steps):
            loaded = i >= steps-count
            result = window.update((i+1)*dt, dt, True, 4. if loaded else 0.,
                                   1 if loaded else 0, 0.)
        return result
    coarse = observed(50e-6, 10)
    half_too_short = observed(25e-6, 10)
    half_equivalent = observed(25e-6, 20)
    assert coarse["ready"]
    assert not half_too_short["ready"]
    assert half_too_short["normal_impulse_Ns"] >= .001
    assert half_equivalent["ready"]
    np.testing.assert_allclose(coarse["loaded_duration_s"], half_equivalent["loaded_duration_s"])
