"""Independent geometry checks for the shared thread-entry classification."""
import numpy as np
import pytest

from thread_lab.model import ThreadConfig
from yam_twin.m8_insertion_mechanics import (
    contact_is_on_full_flanks, fully_formed_flank_interval, thread_end_bounds,
)


def config():
    return ThreadConfig(bolt_length=.016, nut_height=.016, nut_across_flats=.016)


def test_coaxial_full_pitch_threshold_excludes_both_lead_ins():
    c = config()
    # Independent endpoint relation: female full flank begins 1.036582 mm
    # from its face; male full flank ends .956 mm before its tip.
    threshold = .000956 + .000360 + 5*np.sqrt(3)*c.pitch/16 + c.pitch
    d = threshold-c.bolt_length-c.nut_height/2
    measured = fully_formed_flank_interval([0, 0, d], np.eye(3), c)
    assert measured["length_m"] == pytest.approx(c.pitch, abs=1e-17)
    assert measured["minimum_nominal_overlap_for_one_pitch_m"] == pytest.approx(threshold, abs=1e-17)
    assert fully_formed_flank_interval([0, 0, d-1e-6], np.eye(3), c)["one_pitch_available"] is False
    assert fully_formed_flank_interval([0, 0, d+1e-6], np.eye(3), c)["one_pitch_available"] is True


def test_tilted_ring_corners_meet_the_female_entry_plane():
    c = config()
    tilt = np.deg2rad(2)
    co, si = np.cos(tilt), np.sin(tilt)
    rotation = np.array([[co, 0, si], [0, 1, 0], [-si, 0, co]])
    ends = thread_end_bounds(c)
    # Pick a displacement directly from the worst ring corner at the shallow
    # edge of a one-pitch segment, then independently check that corner.
    shaft_start = c.bolt_length-.000956-c.pitch
    d = ends["female_start_m"]-shaft_start*co+ends["male_major_radius_m"]*si
    result = fully_formed_flank_interval([0, 0, d], rotation, c)
    assert result["length_m"] == pytest.approx(c.pitch, abs=1e-17)
    shallow_corner = np.array([0, 0, d])+rotation@np.array([ends["male_major_radius_m"], 0, shaft_start])
    assert shallow_corner[2] == pytest.approx(ends["female_start_m"], abs=1e-17)
    coaxial = .000956+ends["female_chamfer_extent_m"]+c.pitch
    assert result["minimum_nominal_overlap_for_one_pitch_m"]-coaxial == pytest.approx(.000146463, abs=1e-9)


def test_female_exit_limits_overlap_and_reversed_axis_never_passes():
    c = config()
    ends = thread_end_bounds(c)
    result = fully_formed_flank_interval([0, 0, .001], np.eye(3), c)
    assert result["end_m"] == pytest.approx(ends["female_end_m"]-.001)
    reversed_axis = np.diag([1, -1, -1])
    assert fully_formed_flank_interval([0, 0, -.015], reversed_axis, c)["length_m"] == 0


def test_only_points_on_both_full_axial_spans_count_as_flank_contacts():
    c = config()
    assert contact_is_on_full_flanks([.0035, 0, 0], [.0035, 0, .010], c)
    assert not contact_is_on_full_flanks([.0035, 0, -.0078], [.0035, 0, .010], c)
    assert not contact_is_on_full_flanks([.0035, 0, 0], [.0035, 0, .0155], c)
    assert not contact_is_on_full_flanks([.0035, 0, 0], [.0035, 0, 0], c)
    assert not contact_is_on_full_flanks([.0035, 0, 0], [.0035, 0, 1e-6], c, axial_buffer_m=2e-6)


def test_invalid_coordinates_and_buffer_are_rejected():
    c = config()
    with pytest.raises(ValueError):
        fully_formed_flank_interval([0, 0, np.nan], np.eye(3), c)
    with pytest.raises(ValueError):
        contact_is_on_full_flanks([0, 0, 0], [0, 0, .01], c, axial_buffer_m=-1e-6)
