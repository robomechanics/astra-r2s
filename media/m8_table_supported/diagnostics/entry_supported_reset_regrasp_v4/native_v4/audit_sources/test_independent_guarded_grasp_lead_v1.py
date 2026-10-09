"""Regression proof for per-acquisition native raw guard scope."""
import importlib.util
from pathlib import Path
import unittest
import numpy as np

source=Path(__file__).with_name('independent_guarded_grasp_lead_v1.py')
spec=importlib.util.spec_from_file_location('grasp_audit',source)
audit=importlib.util.module_from_spec(spec);spec.loader.exec_module(audit)


class GuardedGraspTests(unittest.TestCase):
    def test_cached_acquisition_row_and_remaining_settle_are_explicit(self):
        t=np.array([1.,2.,3.,4.]);p=['reset_open_search_2','settle_regrip_search_2','settle_regrip_search_2','start_thread_2']
        np.testing.assert_array_equal(audit.expected_grasp_active(t,p,2.),[False,False,True,True])

    def test_intentional_open_global_slip_is_separate(self):
        r={'right_grip_slip_m':np.array([.004,.00002]),'right_grip_rotation_slip_rad':np.array([1.,.001]),
            'right_pad_0_N':np.array([0.,15.]),'right_pad_1_N':np.array([0.,15.])}
        result=audit.measure_guarded_grasp(r,np.array([False,True]));self.assertTrue(result['every_original_guarded_tick_retains_1mm_2deg_loaded_pads'])
        self.assertEqual(result['maximum_original_translation_slip_m'],.00002)

    def test_one_guarded_pad_gap_remains_failure(self):
        r={'right_grip_slip_m':np.array([0.]),'right_grip_rotation_slip_rad':np.array([0.]),
            'right_pad_0_N':np.array([0.]),'right_pad_1_N':np.array([15.])}
        self.assertFalse(audit.measure_guarded_grasp(r,np.array([True]))['every_original_guarded_tick_retains_1mm_2deg_loaded_pads'])

    def test_absent_guard_execution_is_not_vacuous_success(self):
        with self.assertRaises(ValueError):audit.measure_guarded_grasp({},np.array([False]))


if __name__=='__main__':unittest.main(verbosity=2)
