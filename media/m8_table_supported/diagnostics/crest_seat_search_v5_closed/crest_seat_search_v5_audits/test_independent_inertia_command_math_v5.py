"""Pure archive-reader regressions; independent of historical producer proofs."""
import unittest
import numpy as np
import independent_inertia_command_math_v5 as m


class IndependentRecordedInertiaTests(unittest.TestCase):
    def inputs(self):
        M=np.eye(6)[None]
        Jp=np.eye(6)[None,:3,:]
        Jr=np.eye(6)[None,3:,:]
        return M,Jp,Jr,np.zeros((1,3,6)),np.zeros((1,3,6)),np.zeros((1,6)),np.eye(3)[None,:,:2],np.zeros((1,3)),np.array([[0.,0.,2.]])

    def test_off_diagonal_robot_mass_requires_real_transverse_force(self):
        x=list(self.inputs());x[0][0,0,5]=x[0][0,5,0]=.3
        r=m.independent_solve(*x)
        np.testing.assert_allclose(r['ff_wrench_world_N_Nm'],[[.6,0,0,0,0,2]],atol=1e-12)
        tau=np.einsum('nij,ni->nj',x[1],r['ff_wrench_world_N_Nm'][:,:3])+np.einsum('nij,ni->nj',x[2],r['ff_wrench_world_N_Nm'][:,3:])
        np.testing.assert_allclose(r['hybrid_jacobian5']@np.linalg.solve(x[0],tau[...,None]),r['desired_world_acceleration5'][...,None],atol=1e-12)

    def test_jdot_qdot_is_subtracted_before_solve(self):
        x=list(self.inputs());x[3][0,0,0]=.7;x[5][0,0]=2.
        r=m.independent_solve(*x)
        self.assertAlmostEqual(r['projected_world_jdot_qdot5'][0,0],1.4)
        self.assertAlmostEqual(r['ff_wrench_world_N_Nm'][0,0],-1.4)

    def test_requested_axial_acceleration_is_excluded_from_five_axis_task(self):
        x=list(self.inputs());x[7][0,2]=100.
        r=m.independent_solve(*x)
        self.assertEqual(r['ff_wrench_world_N_Nm'][0,2],0.)
        np.testing.assert_array_equal(r['desired_world_acceleration5'][0,:2],0.)

    def test_invalid_mass_is_not_regularized(self):
        x=list(self.inputs());x[0][0,0,0]=-1.
        with self.assertRaisesRegex(ValueError,'positive definite'):
            m.independent_solve(*x)

    def test_rank_deficient_hybrid_task_is_not_regularized(self):
        x=list(self.inputs());x[2][:,2,:]=0.
        with self.assertRaisesRegex(ValueError,'positive definite/full rank'):
            m.independent_solve(*x)

    def test_condition_limit_is_not_silently_clamped(self):
        x=list(self.inputs());x[0][0,2,2]=1e-14
        with self.assertRaisesRegex(ValueError,'condition bound'):
            m.independent_solve(*x)

    def test_caps_apply_to_combined_force_and_preserve_direction(self):
        pd=np.array([[6.,0.,0.],[0.,0.,0.]])
        ff=np.array([[6.,6.,0.],[0.,0.,0.]])
        actual=m.capped(pd+ff,8.)
        self.assertAlmostEqual(np.linalg.norm(actual[0]),8.)
        self.assertAlmostEqual(actual[0,0]/actual[0,1],2.)
        self.assertFalse(np.allclose(actual,m.capped(pd,8.)+m.capped(ff,8.)))
        np.testing.assert_array_equal(actual[1],0.)

    def cache(self):
        dt=.00005
        initial=np.arange(6.)
        post=np.arange(24.).reshape(4,6)+10
        return {'time':dt*np.arange(1,5),'command_time_s':dt*np.arange(4),
            'retained_native_state_time_s':dt*np.array([0,0,1,2]),
            'retained_arm_velocity_rad_s':np.vstack((initial,initial,post[:2])),
            'postintegration_arm_velocity_rad_s':post},initial,dt

    def test_two_initial_rows_then_exact_i_minus_two_cache(self):
        a,initial,dt=self.cache();r=m.verify_cache(a,initial,dt)
        self.assertEqual(r['cross_row_velocity_links_checked'],2)
        self.assertEqual(r['initialized_rows_checked'],2)

    def test_latest_post_velocity_cannot_replace_coherent_cache(self):
        a,initial,dt=self.cache();a['retained_arm_velocity_rad_s'][2]=a['postintegration_arm_velocity_rad_s'][1]
        with self.assertRaisesRegex(ValueError,'preprevious'):
            m.verify_cache(a,initial,dt)

    def test_retained_timestamp_cannot_claim_current_force_time(self):
        a,initial,dt=self.cache();a['retained_native_state_time_s']=a['time']-dt
        with self.assertRaisesRegex(ValueError,'timestamp'):
            m.verify_cache(a,initial,dt)

    def test_float_flag_cannot_claim_real_boolean_execution(self):
        a={k:np.zeros((1,)+v,dtype=(bool if k in {'enabled','cartesian_force_clipped','cartesian_torque_clipped','motor_clipped'} else np.int64 if k=='phase_index' else float)) for k,v in m.SHAPES.items()}
        m.validate_schema(a);a['enabled']=np.ones(1)
        with self.assertRaisesRegex(ValueError,'actual booleans'):
            m.validate_schema(a)

    def test_unarchived_or_missing_command_column_is_rejected(self):
        with self.assertRaisesRegex(ValueError,'exact original schema'):
            m.validate_schema({'time':np.zeros(1)})


if __name__=='__main__':
    unittest.main()
