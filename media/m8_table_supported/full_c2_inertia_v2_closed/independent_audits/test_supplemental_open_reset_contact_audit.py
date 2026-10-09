"""Narrow pure failure-reader checks, separate from original672 proof."""
import unittest
import numpy as np
import supplemental_open_reset_contact_audit as a


class OriginalOpenResetArithmeticTests(unittest.TestCase):
    def test_first_native_open_contact_is_not_confused_with_closed_grasp(self):
        self.assertEqual(a.first_open_contact(np.array([False,True,True]),np.array([8,0,1])),2)

    def test_contact_free_prefix_does_not_invent_contact_or_success(self):
        self.assertIsNone(a.first_open_contact(np.ones(3,bool),np.zeros(3,dtype=int)))

    def test_float_contact_count_cannot_claim_native_integer_schema(self):
        with self.assertRaises(ValueError):
            a.first_open_contact(np.ones(2,bool),np.array([0.,1.]))

    def test_numeric_open_flag_cannot_waive_native_boolean(self):
        with self.assertRaises(ValueError):
            a.first_open_contact(np.array([0.,1.]),np.array([0,1]))

    def test_executed_partial_phase_baseline_keeps_force_offset(self):
        r=a.phase_baseline(np.arange(1,6)*.00005,np.array([0,0,1,1,1]),1,.00005)
        self.assertAlmostEqual(r['actual_duration_s'],.00015)
        self.assertAlmostEqual(r['original_force_baseline_time_s'],.00005)
        self.assertAlmostEqual(r['last_original_force_time_s'],.00020)

    def test_unexecuted_regrasp_has_no_fabricated_baseline(self):
        with self.assertRaises(ValueError):
            a.phase_baseline(np.array([.1]),np.array([0]),2,.1)

    def test_finite_cap_preserves_actual_uncapped_direction(self):
        f=np.array([[90.,120.,0.],[0.,0.,0.]])
        v=a.capped(f,8.)
        np.testing.assert_allclose(v[0],[4.8,6.4,0.],atol=1e-12)
        np.testing.assert_array_equal(v[1],0.)

    def test_under_cap_command_is_not_reported_as_force_saturation(self):
        np.testing.assert_array_equal(a.capped(np.array([[1.,2.,3.]]),8.),[[1.,2.,3.]])

    def test_original_PD_decomposition_requires_uncapped_same_command_inputs(self):
        arm=dict(position_stiffness=20000.,position_damping=650.,rotation_stiffness=40.,rotation_damping=6.)
        p=np.array([.004,-.003,.001]);r=np.array([.02,0.,-.01]);v=np.array([.01,.02,-.01]);w=np.array([.1,-.2,.05])
        original=np.r_[20000*p+650*v,40*r+6*w]
        out=a.pd_error_components(original,p,r,arm)
        np.testing.assert_allclose(out['inferred_same_command_linear_velocity_error_m_per_s'],v,atol=1e-12)
        np.testing.assert_allclose(out['inferred_same_command_angular_velocity_error_rad_per_s'],w,atol=1e-12)

    def test_invalid_PD_error_never_produces_a_force_explanation(self):
        with self.assertRaises(ValueError):
            a.pd_error_components(np.zeros(6),[np.nan,0,0],np.zeros(3),{})


if __name__=='__main__':
    unittest.main()
