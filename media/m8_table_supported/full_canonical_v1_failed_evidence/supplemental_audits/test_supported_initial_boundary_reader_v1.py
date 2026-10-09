"""Pure reader-boundary regressions; separate from immutable402 producer proof."""
import ast
import copy
import difflib
import hashlib
from pathlib import Path
import unittest
import numpy as np

P=Path(__file__).resolve().parent
OLD='e46808601d7974ffdea1e5e20e07e0f28eb8e0a74a394545b276621a7e856702'
source=P/'audit_m8_supported_trace_initial_boundary_v1.py'
# Execute only the corrected pure function AST, avoiding application/native imports.
node=next(n for n in ast.parse(source.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='original_grasp_flag_audit')
namespace={'np':np};exec(compile(ast.Module(body=[node],type_ignores=[]),str(source),'exec'),namespace)
audit=namespace['original_grasp_flag_audit']


def fixture():
    labels=['settle_bolt','lift_bolt','release_1','settle_regrip_1','turn_1']
    columns={'time':np.arange(1.,9.),'phase_index':np.array([0,0,1,2,3,3,3,4]),
        'right_grasp_guard_active':np.array([False,True,True,False,False,False,True,True])}
    metadata={'right_grasp_acquisitions':[{'phase':'settle_bolt','time_s':2.},
        {'phase':'settle_regrip_1','time_s':6.,'continuous_quiet_bilateral_streak_s':.1}],
        'maximum_phase_plan':[(label,.1,None,None,None,.03 if label=='release_1'else.02)for label in labels],
        'control_config':{'arm':{'closed_aperture':.02},'regrasp_window_s':.1}}
    return columns,labels,metadata


class SourceSpecificBoundaryTests(unittest.TestCase):
    def test_original_initial_active_and_quiet_next_tick_both_verified(self):
        result=audit(*fixture());self.assertTrue(result['source_specific_acquisition_guard_boundaries_verified'])
        self.assertEqual(result['initial_same_row_acquisition_count'],1);self.assertEqual(result['quiet_next_tick_acquisition_count'],1)
        self.assertEqual(result['guarded_native_steps'],4)

    def test_initial_inactive_flag_is_not_waived(self):
        values=fixture();values[0]['right_grasp_guard_active'][1]=False
        with self.assertRaises(ValueError):audit(*values)

    def test_quiet_same_row_active_flag_is_not_waived(self):
        values=fixture();values[0]['right_grasp_guard_active'][5]=True
        with self.assertRaises(ValueError):audit(*values)

    def test_initial_acquisition_must_be_final_settle_tick(self):
        values=fixture();values[2]['right_grasp_acquisitions'][0]['time_s']=1.
        values[0]['right_grasp_guard_active'][0]=True
        with self.assertRaises(ValueError):audit(*values)

    def test_unknown_or_mismatched_acquisition_phase_is_rejected(self):
        values=fixture();values[2]['right_grasp_acquisitions'][0]['phase']='lift_bolt'
        with self.assertRaises(ValueError):audit(*values)

    def test_quiet_reference_without_original_continuous_streak_is_rejected(self):
        values=fixture();del values[2]['right_grasp_acquisitions'][1]['continuous_quiet_bilateral_streak_s']
        with self.assertRaises(ValueError):audit(*values)

    def test_post_acquisition_settle_gap_remains_rejected(self):
        values=fixture();values[0]['right_grasp_guard_active'][6]=False
        with self.assertRaises(ValueError):audit(*values)

    def test_source_binding_only_the_pure_boundary_function_changes(self):
        original=P/'original_e468_auditor.py'
        self.assertEqual(hashlib.sha256(original.read_bytes()).hexdigest(),OLD)
        old_tree,new_tree=ast.parse(original.read_text()),ast.parse(source.read_text())
        self.assertEqual(len(old_tree.body),len(new_tree.body));changed=[]
        for old,new in zip(old_tree.body,new_tree.body):
            if ast.dump(old,include_attributes=False)!=ast.dump(new,include_attributes=False):changed.append(getattr(old,'name',None))
        self.assertEqual(changed,['original_grasp_flag_audit'])
        patch=''.join(difflib.unified_diff(original.read_text().splitlines(True),source.read_text().splitlines(True),fromfile=original.name,tofile=source.name))
        self.assertEqual((P/'supported_auditor_initial_boundary_v1.diff').read_text(),patch)


if __name__=='__main__':unittest.main(verbosity=2)
