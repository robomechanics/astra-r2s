"""Event provenance regressions, separate from canonical rollout qualification."""
import copy
import importlib.util
from pathlib import Path
import unittest

import numpy as np

SOURCE=Path(__file__).with_name('independent_adaptive_open_audit_v1.py')
SPEC=importlib.util.spec_from_file_location('adaptive_open_audit',SOURCE)
AUDIT=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(AUDIT)


def fixture():
    dt=.01
    elapsed=np.arange(1,12)*dt
    raw={'elapsed_s':elapsed,'time_s':100.+elapsed,
        'relative_bolt_angular_speed_rad_per_s':np.zeros(11),
        'relative_bolt_axial_velocity_m_per_s':np.zeros(11)}
    event={'elapsed_s':float(elapsed[3]),'time_s':float(raw['time_s'][3]),
        'force_time_s':float(raw['time_s'][3]-dt),
        'first_reset_command_elapsed_s':float(elapsed[3]+dt),
        'actual_open_settle_s':float(elapsed[3]-.01),
        'open_weight_window':{'ready_for_diagnostic_release_attempt':True}}
    planned=[['release_search_2',.01],['open_settle_search_2',.08],['reset_open_search_2',.1]]
    actual=copy.deepcopy(planned);actual[1][1]=event['actual_open_settle_s']
    ends=np.cumsum([s[1]for s in actual]);raw['phase_index']=np.searchsorted(ends,elapsed,side='left')
    declaration={'adaptive_open_settle':True,'phase_schedule':planned,
        'minimum_open_settle_s':.02,'maximum_open_settle_s':.08}
    metadata={'actual_adaptive_open_readiness_event':event,'actual_shifted_phase_schedule':actual,
        'actual_executed_native_duration_s':len(elapsed)*dt,'actual_shifted_planned_duration_s':float(ends[-1])}
    report={'physical_closed_turn':{'adaptive_open_readiness_event':event,'actual_shifted_phase_schedule':actual}}
    row={'elapsed_s':event['elapsed_s'],'time_s':event['time_s'],'adaptive_open_ready_event':event,
        'open_weight_window':event['open_weight_window'],'physical_phase':'open_settle_search_2',
        'right_bolt_contact_count':0,'all_checks_held':True,
        'relative_bolt_angular_speed_rad_per_s':0.,'relative_bolt_axial_velocity_m_per_s':0.}
    return raw,declaration,metadata,report,[row],dt


class AdaptiveEventTests(unittest.TestCase):
    def test_native_event_shifts_next_tick_and_preserves_maximum_plan(self):
        values=fixture();result=AUDIT.check_event(*values)
        self.assertEqual(result['first_reset_native_index'],4)
        self.assertNotEqual(result['actual_shifted_schedule'],result['declared_maximum_schedule'])
        self.assertEqual(values[1]['phase_schedule'][1][1],.08)

    def test_old_maximum_phase_clock_cannot_describe_actual_rows(self):
        values=fixture()
        raw,declaration=values[:2]
        raw['phase_index']=np.searchsorted(np.cumsum([p[1]for p in declaration['phase_schedule']]),
            raw['elapsed_s'],side='left')
        with self.assertRaises(AssertionError):AUDIT.check_event(*values)

    def test_missing_off_grid_event_state_is_not_reconstructed(self):
        values=list(fixture());values[4]=[]
        with self.assertRaises(AssertionError):AUDIT.check_event(*values)

    def test_event_forces_cannot_be_retimestamped_to_post_state(self):
        values=fixture();event=values[3]['physical_closed_turn']['adaptive_open_readiness_event']
        event['force_time_s']=event['time_s']
        with self.assertRaises(AssertionError):AUDIT.check_event(*values)


if __name__=='__main__':unittest.main(verbosity=2)
