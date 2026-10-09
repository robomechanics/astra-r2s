"""Supplemental regression proof, separate from historical producer tests."""
from pathlib import Path
import importlib.util

import mujoco
import numpy as np
import pytest

path = Path(__file__).with_name('independent_rest_clearance_audit.py')
spec = importlib.util.spec_from_file_location('independent_rest', path)
rest = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rest)


def test_compiled_fixed_rest_bounds_include_height_and_rotated_end_faces():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="fixed_rest" pos="0 0 .010">
        <geom name="bolt_rest_base" type="box" size=".020 .020 .005"/>
        <geom name="bolt_rest_pin" type="cylinder" pos="0 0 .015" euler="0 45 0" size=".002 .010"/>
      </body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    mujoco.mj_kinematics(model, data)
    bounds = rest.native_rest_bounds(model, data)
    assert bounds['geom_world_top_z_m']['bolt_rest_base'] == pytest.approx(.015)
    assert bounds['geom_world_top_z_m']['bolt_rest_pin'] == pytest.approx(.025+.012/np.sqrt(2))
    assert bounds['maximum_world_z_m'] > .033


def test_native_transfer_guard_rejects_one_low_tip_despite_safe_lift_waypoint():
    labels = ['lift_bolt', 'transport_bolt', 'align_over_hole']
    indices = np.array([0, 0, 1, 1, 1, 2, 2])
    phases = [{'phase': 'lift_bolt', 'actual_duration_s': .9, 'maximum_scheduled_duration_s': .9}]
    selected = rest.transfer_samples(indices, labels, phases)
    np.testing.assert_array_equal(selected, [False, True, True, True, True, False, False])
    clearances = np.array([-.016, .015, .015, .015, .015, .003, .002])
    assert rest.transfer_stats(clearances, selected, .010)['passed']
    clearances[3] = .009
    failed = rest.transfer_stats(clearances, selected, .010)
    assert not failed['passed']
    assert failed['minimum_observed_tip_above_rest_m'] == .009
    assert not rest.transfer_samples(np.array([0, 0]), ['lift_bolt'],
        [{'phase': 'lift_bolt', 'actual_duration_s': .4, 'maximum_scheduled_duration_s': .9}]).any()
