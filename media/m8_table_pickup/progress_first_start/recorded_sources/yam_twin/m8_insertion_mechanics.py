"""Shared geometric measurements for passive M8 thread entry.

These functions classify measured geometry; they contain no force law,
control command, engagement constraint, or object-state mutation.
"""
from __future__ import annotations

import numpy as np


def thread_end_bounds(thread_config):
    """Conservative axial bounds beyond which end chamfers cannot dominate."""
    H = np.sqrt(3.)*thread_config.pitch/2
    female_chamfer_extent = 5*H/8 + .000360
    male_major_radius = thread_config.male_pitch_diameter/2 + 3*H/8
    return {"male_start_m": 0., "male_end_m": float(thread_config.bolt_length-.000956),
            "female_start_m": float(-thread_config.nut_height/2+female_chamfer_extent),
            "female_end_m": float(thread_config.nut_height/2-female_chamfer_extent),
            "female_chamfer_extent_m": float(female_chamfer_extent),
            "male_tip_chamfer_m": .000956,
            "male_major_radius_m": float(male_major_radius)}


def fully_formed_flank_interval(relative_base_position, relative_rotation, thread_config):
    """Return the complete-ring male-axis interval inside full female flanks.

    Position and rotation transform male coordinates into the female thread
    frame. Both thread frames point along insertion (+Z). Every point on a
    conservative male-major-radius ring must lie inside the female's axial
    unchamfered interval. This accounts for tilt at both entry and exit; it
    measures potential complete flank overlap, not force-bearing engagement.
    Radial fit, actual positive contacts, and lead remain separate checks.
    """
    position = np.asarray(relative_base_position, dtype=float)
    rotation = np.asarray(relative_rotation, dtype=float)
    if position.shape != (3,) or rotation.shape != (3, 3) or not (
            np.isfinite(position).all() and np.isfinite(rotation).all()):
        raise ValueError("Relative thread pose must be finite 3-vector and 3x3 rotation")
    bounds = thread_end_bounds(thread_config)
    cosine = float(np.clip(rotation[2, 2], -1., 1.))
    sine = float(np.sqrt(max(0., 1-cosine*cosine)))
    tilt = float(np.arccos(cosine))
    ring_extent = bounds["male_major_radius_m"]*sine
    nominal_overlap = float(position[2]+thread_config.bolt_length+thread_config.nut_height/2)
    minimum_nominal_overlap = float(thread_config.bolt_length
        -(bounds["male_end_m"]-thread_config.pitch)*cosine
        + bounds["female_chamfer_extent_m"]+ring_extent)
    if cosine <= 0:
        start, end, length = 0., 0., 0.
    else:
        start = max(bounds["male_start_m"],
                    (bounds["female_start_m"]-position[2]+ring_extent)/cosine)
        end = min(bounds["male_end_m"],
                  (bounds["female_end_m"]-position[2]-ring_extent)/cosine)
        length = max(0., end-start)
    return {"start_m": float(start), "end_m": float(end), "length_m": float(length),
            "tilt_rad": tilt, "ring_axial_extent_m": float(ring_extent),
            "nominal_total_overlap_m": nominal_overlap,
            "minimum_nominal_overlap_for_one_pitch_m": minimum_nominal_overlap,
            "one_pitch_available": bool(length >= thread_config.pitch)}


def contact_is_on_full_flanks(point_in_female, point_in_male, thread_config,
                              axial_buffer_m=0.):
    """Classify an actual contact point inside both unchamfered axial spans.

    Caller must check that the contact belongs to the male/female SDF pair
    and carries positive measured normal force. A positive axial buffer can
    exclude ambiguous contact points near the axial boundary.
    """
    female = np.asarray(point_in_female, dtype=float)
    male = np.asarray(point_in_male, dtype=float)
    if female.shape != (3,) or male.shape != (3,) or not (
            np.isfinite(female).all() and np.isfinite(male).all()):
        raise ValueError("Contact coordinates must be finite three-vectors")
    if not np.isfinite(axial_buffer_m) or axial_buffer_m < 0:
        raise ValueError("Contact axial buffer must be finite and nonnegative")
    bounds = thread_end_bounds(thread_config)
    return bool(bounds["female_start_m"]+axial_buffer_m < female[2]
        < bounds["female_end_m"]-axial_buffer_m
        and bounds["male_start_m"]+axial_buffer_m < male[2]
        < bounds["male_end_m"]-axial_buffer_m)
