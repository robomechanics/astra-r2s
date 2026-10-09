"""Reward-free output-only reader checks, separate from producer402 tests."""
from pathlib import Path
import unittest
import numpy as np
import independent_cold_crest_audit_v1 as reader

RUN = Path(__file__).resolve().parent.parent/"crest_seat_search_v3"


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
        keys = ("fully_open_unassisted", "right_grasp_guard_active", "right_axial_float_active", "weight_window_ready", "open_weight_window_ready", "all_hard_guards_held", "external_drive_zero", "weight_observation_valid", "open_observation_valid", "cold_table_window_warmup")
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


if __name__ == "__main__":
    unittest.main()
