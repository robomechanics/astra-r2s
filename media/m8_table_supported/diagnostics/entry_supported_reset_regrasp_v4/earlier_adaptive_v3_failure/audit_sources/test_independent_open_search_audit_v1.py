"""Arithmetic-only regressions; separate from canonical application proof."""
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np

SOURCE = Path(__file__).with_name('independent_open_search_audit_v1.py')
SPEC = importlib.util.spec_from_file_location('open_audit', SOURCE)
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def ledger(n=500):
    data = {k: np.zeros(n) for k in ('right_bolt_contact_count',
        'relative_bolt_angular_speed_rad_per_s','relative_bolt_axial_velocity_m_per_s',
        'bolt_world_support_contact_count','nonthread_block_bolt_contact_count',
        'radial_offset_m','bolt_tilt_rad','hand_gravity_opposing_force_N')}
    for key in ('fully_open_unassisted','all_checks_held','external_drive_zero',
                'thread_gravity_opposing_force_N'):
        data[key] = np.ones(n)
    return data


class OpeningWindowTests(unittest.TestCase):
    def test_live_strict_event_precedes_later_endpoint_timeout(self):
        raw = ledger()
        raw['relative_bolt_angular_speed_rad_per_s'][470] = .02
        good, ready = AUDIT.exact_readiness(raw, 1., .001)
        self.assertTrue(ready[99])
        self.assertTrue(ready[469])
        self.assertFalse(ready[-1])
        self.assertEqual(AUDIT.runs(good)[-1], (471,499))

    def test_whole_right_candidate_contact_rejects_force_free_tick(self):
        raw = ledger(100)
        raw['right_bolt_contact_count'][50] = 1
        self.assertTrue(np.all(raw['hand_gravity_opposing_force_N'] == 0))
        good, ready = AUDIT.exact_readiness(raw, 1., .001)
        self.assertFalse(good[50])
        self.assertFalse(ready[-1])

    def test_positive_hand_up_cannot_be_cancelled_by_downward_load(self):
        raw = ledger(100)
        raw['hand_gravity_opposing_force_N'][::2] = .3
        raw['hand_gravity_opposing_force_N'][1::2] = -.3
        self.assertAlmostEqual(np.mean(raw['hand_gravity_opposing_force_N']), 0.)
        self.assertFalse(AUDIT.exact_readiness(raw, 1., .001)[1][-1])

    def test_noncontact_robot_speed_is_not_added_to_original_bolt_gate(self):
        raw = ledger(100)
        raw['relative_hand_angular_speed_rad_per_s'] = np.full(100, 10.)
        self.assertTrue(AUDIT.exact_readiness(raw, 1., .001)[1][-1])

    def test_loaded_mean_does_not_cover_unloaded_endpoint(self):
        raw = ledger(100)
        raw['thread_gravity_opposing_force_N'][-1] = 0.
        self.assertGreater(np.mean(raw['thread_gravity_opposing_force_N']), .9)
        self.assertFalse(AUDIT.exact_readiness(raw, 1., .001)[1][-1])

    def test_source_predicate_changes_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'source.py'
            path.write_text('open_window.observe(t,dt,sample,valid=bool(fully_open and '
                'right_bolt_contacts==0 and all(checks.values()) and '
                'sample["relative_hand_angular_speed_rad_per_s"]<=.01))')
            with self.assertRaises(AssertionError):
                AUDIT.source_open_predicate(path)


if __name__ == '__main__':
    unittest.main(verbosity=2)
