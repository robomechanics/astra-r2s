import importlib.util
from pathlib import Path

import numpy as np
import pytest

path = Path(__file__).with_name('independent_phase_depth_audit.py')
spec = importlib.util.spec_from_file_location('phase_auditor', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_matching_phase_preserves_declared_radial_pitch_clearance():
    value = module.profile_interference(0., .00125, .0071/2, .007268/2)
    assert value == pytest.approx(-.000084, abs=1e-12)


def test_wrong_phase_produces_actual_profile_interference():
    value = module.profile_interference(.000267, .00125, .0071/2, .007268/2)
    assert value > .0003


def test_profile_is_periodic_and_signed_offset_does_not_bias_fit():
    values = [module.profile_interference(value, .00125, .0071/2, .007268/2)
        for value in [.0001, -.0001, .00135]]
    np.testing.assert_allclose(values, values[0], atol=1e-12)


def test_zero_clearance_matching_profiles_touch_but_do_not_interpenetrate():
    assert module.profile_interference(0., .00125, .0071/2, .0071/2) == pytest.approx(0., abs=1e-12)
