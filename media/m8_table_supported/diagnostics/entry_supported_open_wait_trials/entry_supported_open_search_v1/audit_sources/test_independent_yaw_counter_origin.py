"""Archived counter-origin recognition; forbid guessing unknown conventions."""
from pathlib import Path
import importlib.util

import pytest

spec = importlib.util.spec_from_file_location('origin', Path(__file__).with_name(
    'independent_yaw_counter_origin.py'))
origin = importlib.util.module_from_spec(spec)
spec.loader.exec_module(origin)


def test_retained_canonical_counter_is_distinct_from_native_cold_rotation():
    source = "last_yaw=old['bolt_yaw_unwrapped_rad']\nwrapped_last=float(np.arctan2(hr.T@data.xmat[bolt], 1))\n"
    assert origin.classify_counter_origin(source) == 'canonical_parent_counter'


def test_correct_native_wrapped_counter_requires_actual_frame_initialization():
    source = "wrapped_last=float(np.arctan2(hr.T@data.xmat[bolt], 1))\nlast_yaw=wrapped_last\n"
    assert origin.classify_counter_origin(source) == 'actual_cold_state_wrapped_yaw'


@pytest.mark.parametrize('source', ['last_yaw=0.\n', 'wrapped_last=3.\nlast_yaw=wrapped_last\n'])
def test_unknown_or_literal_counter_origin_is_rejected(source):
    with pytest.raises(ValueError):
        origin.classify_counter_origin(source)
