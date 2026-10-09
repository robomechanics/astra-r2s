"""Distinguish actual helical contact normals from an axisymmetric cone."""
from pathlib import Path
import importlib.util

import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('entry',Path(__file__).with_name('independent_formed_entry_audit.py'))
entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)


def test_helical_sixty_degree_flank_normal_recovers_pitch_without_using_axial_motion():
    radius,pitch=.0035,.00125
    normal=np.array([.5,-np.sqrt(3)/2*pitch/(2*np.pi*radius),np.sqrt(3)/2])
    normal/=np.linalg.norm(normal)
    c=dict(contact_position_world_m=[radius,0.,.01],bolt_origin_world_m=[0.,0.,0.],frame=[normal.tolist(),[0.,1.,0.],[1.,0.,0.]])
    result=entry.normal_pitch_world_z(c)
    assert result['estimated_pitch_from_normal_m']==pytest.approx(pitch)
    assert abs(result['normal_azimuth_component_world_Z'])>.04


def test_axisymmetric_forty_five_degree_cone_has_no_helical_normal_slope():
    c=dict(contact_position_world_m=[.0035,0.,.01],bolt_origin_world_m=[0.,0.,0.],
        frame=[[2**-.5,0.,2**-.5],[0.,1.,0.],[0.,0.,1.]])
    result=entry.normal_pitch_world_z(c)
    assert result['normal_azimuth_component_world_Z']==0.
    assert result['estimated_pitch_from_normal_m']==0.
