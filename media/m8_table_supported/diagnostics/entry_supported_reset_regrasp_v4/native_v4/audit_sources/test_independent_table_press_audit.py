"""Original table/left action-reaction and full moment arithmetic."""
from pathlib import Path
import importlib.util

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('press', Path(__file__).with_name('independent_table_press_audit.py'))
press = importlib.util.module_from_spec(spec)
spec.loader.exec_module(press)


@pytest.mark.parametrize('block_first', [True, False])
def test_full_tangent_and_lever_arm_wrench_handles_both_geom_orders(block_first):
    sign = -1 if block_first else 1
    force = sign*np.array([-3., 2., 4.])
    c = dict(geom1='block_outer' if block_first else 'left_m8_pad_left',
        geom2='left_m8_pad_left' if block_first else 'block_outer',
        frame=[[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]],
        local_contact_force_N_Nm=[2., 3., 4., .1, .2, .3],
        contact_position_world_m=[2., 2., 3.], block_origin_world_m=[1., 2., 3.],
        signed_force_contribution_on_block_world_N=force.tolist())
    np.testing.assert_allclose(press.block_wrench(c, {'block_outer'}),
        sign*np.array([-3., 2., 4., -.2, -3.9, 2.3]))


def test_contact_without_exactly_one_block_geom_cannot_supply_stabilization_wrench():
    with pytest.raises(AssertionError):
        press.block_wrench(dict(geom1='left_pad', geom2='table'), {'block_outer'})
