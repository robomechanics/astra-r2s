"""Reward-free output-only reader checks, separate from producer402 tests."""
from pathlib import Path
import unittest
import numpy as np
import independent_cold_crest_brake_audit_v1 as reader

RUN = Path(__file__).resolve().parent.parent/"crest_seat_search_v4"


class OriginalReaderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ns = reader.archived_classes(RUN)

    def test_world_contact_action_reaction_and_moment(self):
        world = reader.contact_wrench(np.eye(3), [2,3,4,0,0,0], [1,0,0], np.zeros(3), 1)
        np.testing.assert_allclose(world, [2,3,4,0,-4,3])
        np.testing.assert_allclose(reader.contact_wrench(np.eye(3), [2,3,4,0,0,0], [1,0,0], np.zeros(3), -1), -world)

    def test_bad_contact_basis_rejected(self):
        with self.assertRaises(ValueError):
            reader.contact_wrench(np.zeros((3,3)), np.zeros(6), np.zeros(3), np.zeros(3), 1)

    def test_float_flags_cannot_claim_native_boolean_schema(self):
        keys = ("fully_open_unassisted", "right_grasp_guard_active", "right_axial_float_active", "weight_window_ready", "open_weight_window_ready", "all_hard_guards_held", "external_drive_zero", "weight_observation_valid", "open_observation_valid", "cold_table_window_warmup", "robot_yaw_brake_active")
        cols = {k:np.ones(2,dtype=bool) for k in keys}
        reader.verify_boolean_flags(cols)
        cols["all_hard_guards_held"] = np.ones(2)
        with self.assertRaises(ValueError):
            reader.verify_boolean_flags(cols)

    def test_scalar_cannot_replace_six_axis_original_wrench(self):
        cols = {"time":np.arange(2.), "thread_wrench_on_bolt_world_N_Nm":np.zeros(2)}
        with self.assertRaisesRegex(ValueError, "Wrong raw feedback shape"):
            reader.validate_shapes(cols)

    def test_table_window_mean_never_waives_unloaded_endpoint(self):
        w = self.ns["TableLoadWindow"](.1, 1.)
        for _ in range(100):
            w.observe(1., -2., .001, True)
        self.assertTrue(w.ready)
        w.observe(0., -2., .001, True)
        self.assertFalse(w.ready)
        self.assertGreater(w.report()["mean_table_upward_force_N"], .9)

    def test_moving_unloaded_crest_return_only_requests_closed_braking(self):
        w = self.ns["CrestSeatDropWindowV3"](.0, .2624)
        w.observe(.001,.001,-.00065,.0014,0.,actual_relative_angular_speed_rad_per_s=1.85,valid=True,physically_stopped=False)
        r = w.observe(.002,.001,-.000599,.0014,0.,actual_relative_angular_speed_rad_per_s=1.85,valid=True,physically_stopped=False)
        self.assertTrue(r["stop_requested_from_measured_return"])
        self.assertFalse(r["confirmed_search_direction_event"])
        self.assertFalse(r["is_engagement_or_release_proof"])
        self.assertEqual(r["original_native_impulse_window"]["normal_impulse_Ns"], 0.)

    def test_invalid_epoch_clears_current_report_without_rewriting_saved_event(self):
        w = self.ns["CrestSeatDropWindowV3"](0., .2624)
        w.observe(.001,.001,-.00065,0.,0.,actual_relative_angular_speed_rad_per_s=1.,valid=True,physically_stopped=False)
        event = w.observe(.002,.001,-.000599,0.,0.,actual_relative_angular_speed_rad_per_s=1.,valid=True,physically_stopped=False)
        final = w.observe(.003,.001,-.000590,0.,0.,actual_relative_angular_speed_rad_per_s=1.,valid=False,physically_stopped=True)
        self.assertTrue(event["stop_requested_from_measured_return"])
        self.assertFalse(final["stop_requested_from_measured_return"])
        self.assertIsNone(final["actual_crest_base_z_m"])
        self.assertEqual(final["invalid_or_discontinuous_epoch"], 1)

    def test_declared_source_bundle_binds_whole_new_observer(self):
        import json
        metadata=json.loads((RUN/"insertion_validation.json").read_text())
        self.assertEqual(reader.bundle_identity(RUN/"controller_source.py"), metadata["controller_sha256"])
        self.assertEqual(reader.digest(RUN/"experimental_observer_source.py"), reader.OBS)

    def test_warm_history_never_qualifies_before_fresh_100ms(self):
        w = self.ns["TableLoadWindow"](.1,1.)
        for _ in range(1999):
            w.observe(3.,-2.,.00005,True)
        self.assertFalse(w.ready)
        self.assertLess(w.elapsed+1e-12,.1)
        w.observe(3.,-2.,.00005,True)
        self.assertTrue(w.ready)

    def test_positive_hand_load_not_signed_cancellation(self):
        w = self.ns["BoltWeightTransferWindow"](1.)
        sample={"thread_gravity_opposing_force_N":1.,"hand_gravity_opposing_force_N":0.,"relative_bolt_axial_velocity_m_per_s":0.,"radial_offset_m":0.,"bolt_tilt_rad":0.,"native_thread_contact_count":1,"loaded_actual_interior_flank_contact_count":0,"bolt_world_support_contact_count":0,"nonthread_block_bolt_contact_count":0,"external_drive_zero":True}
        for i in range(200):
            sample["hand_gravity_opposing_force_N"] = 1. if i%2==0 else -1.
            w.observe((i+1)*.001,.001,sample,valid=True)
        self.assertFalse(w.ready)
        self.assertGreater(w.report()["mean_positive_hand_upward_weight_fraction"], .45)




class BrakeReaderTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import json
        cls.metadata=json.loads((RUN/'insertion_validation.json').read_text())
        with np.load(RUN/'native_feedback_force_history.npz',allow_pickle=False) as z:
            cls.raw={k:z[k].copy() for k in z.files if k not in ('metadata_json','phase_labels_json')}
            cls.labels=json.loads(str(z['phase_labels_json']))
        cls.dt=.00005

    def test_start_jet_and_first_dt_are_continuous_without_instant_stop(self):
        seed=(-1.843648993692857,-1.8427141965956917,1.215386997053501)
        a=reader.analytic_brake(*seed,.15,[0.,.00005])
        np.testing.assert_allclose(a[:3,0],seed,rtol=1e-12)
        self.assertLess(a[1,1],-1.84)
        self.assertLess(abs(a[0,1]-a[0,0]),.0001)

    def test_stationary_endpoint_is_C2_not_C3(self):
        a=reader.analytic_brake(-1.8,-1.85,1.2,.15,[.15-1e-10,.15,.20])
        np.testing.assert_allclose(a[1:3,1:],0.,atol=0.)
        self.assertGreater(abs(a[3,0]),100.)
        np.testing.assert_allclose(a[3,1:],0.,atol=0.)
        np.testing.assert_allclose(a[0,1],a[0,2],atol=0.)

    def test_entire_curve_peak_acceleration_inside_interval(self):
        e=reader.brake_extrema(-1.8,-1.85,1.2,.15)
        self.assertGreater(e['peak_acceleration_rad_s2'],18.)
        self.assertLessEqual(e['maximum_signed_reverse_speed_rad_s'],1e-12)
        self.assertAlmostEqual(e['complete_curve_endpoint_rad'],-1.9365,places=10)

    def test_all_actual_brake_jets_match_independent_coefficients(self):
        r=reader.verify_command_curve(self.raw,self.labels,self.metadata,self.dt)
        self.assertEqual(r['executed_brake_rows'],1799)
        self.assertFalse(r['completed_brake'])
        self.assertLess(max(r['maximum_absolute_derivative_residuals']),1e-10)

    def test_mutated_acceleration_is_rejected(self):
        cols={k:v.copy() for k,v in self.raw.items()}
        cols['desired_independent_angular_acceleration_rad_s2'][-1]+=.01
        with self.assertRaisesRegex(ValueError,'clock derivative'):
            reader.verify_command_curve(cols,self.labels,self.metadata,self.dt)

    def test_premature_brake_completed_flag_is_rejected(self):
        cols={k:v.copy() for k,v in self.raw.items()}
        cols['robot_yaw_brake_active'][-1]=False
        with self.assertRaisesRegex(ValueError,'Brake-active'):
            reader.verify_command_curve(cols,self.labels,self.metadata,self.dt)

    def test_hybrid_mass_coupling_force_is_real_and_axial_row_absent(self):
        import ast
        source=(Path(__file__).parent/'independent_robot_inertia_v1.py').read_text()
        n=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='hybrid_inertia_feedforward')
        ns={'np':np};exec(compile(ast.Module(body=[n],type_ignores=[]),'inertia_pure','exec'),ns)
        M=np.eye(6);M[0,5]=M[5,0]=.3
        J=np.eye(6)[[0,1,3,4,5]]
        a=np.array([0.,0.,0.,0.,2.])
        lam,w,tau=ns['hybrid_inertia_feedforward'](M,J,a)
        np.testing.assert_allclose(w,[.6,0.,0.,0.,2.],atol=1e-12)
        np.testing.assert_allclose(J@np.linalg.solve(M,tau),a,atol=1e-12)
        self.assertEqual(tau[2],0.)

if __name__ == '__main__':
    unittest.main()
