"""Quasi-static V-thread predictions, independent of the contact simulator.

All lengths are metres, forces newtons, torques newton metres, and angles
radians. These relations describe aligned, thread-only sliding contact.
Bearing friction, drive inertia, misalignment, damage, and bolt preload are
outside this model. They are diagnostic predictions, never state updates.
"""
from __future__ import annotations

import math


M8_PITCH = 0.00125
M8_MALE_PITCH_DIAMETER = 0.007100
M8_FEMALE_PITCH_DIAMETER = 0.007268
# The contact radius spans a flank. A mean of the two specified pitch
# diameters is an explicit first-order analytical comparison radius.
M8_CONTACT_PITCH_DIAMETER = (
    M8_MALE_PITCH_DIAMETER + M8_FEMALE_PITCH_DIAMETER
) / 2
METRIC_HALF_ANGLE = math.pi / 6


def _positive(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _nonnegative(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be finite and nonnegative")
    return value


def _angle(half_angle: float) -> float:
    half_angle = float(half_angle)
    if not math.isfinite(half_angle) or not 0 <= half_angle < math.pi / 2:
        raise ValueError("half_angle must be finite and in [0, pi/2)")
    return half_angle


def contact_pitch_diameter(
    male: float = M8_MALE_PITCH_DIAMETER,
    female: float = M8_FEMALE_PITCH_DIAMETER,
) -> float:
    """Return the declared mean pitch diameter for an ideal clearanced fit.

    This approximates the effective friction lever arm, not a substitute for
    integrating measured contact forces across the actual flank.
    """
    male = _positive(male, "male pitch diameter")
    female = _positive(female, "female pitch diameter")
    if female < male:
        raise ValueError("female pitch diameter must not be smaller than male")
    return (male + female) / 2


def lead_angle(
    pitch: float = M8_PITCH,
    pitch_diameter: float = M8_CONTACT_PITCH_DIAMETER,
    starts: int = 1,
) -> float:
    """Return atan(lead / pitch circumference); lead = pitch * starts."""
    pitch = _positive(pitch, "pitch")
    pitch_diameter = _positive(pitch_diameter, "pitch diameter")
    start_count = float(starts)
    if (isinstance(starts, bool) or not math.isfinite(start_count)
            or not start_count.is_integer() or start_count < 1):
        raise ValueError("starts must be a positive integer")
    return math.atan(pitch * start_count / (math.pi * pitch_diameter))


def effective_friction(
    friction: float, half_angle: float = METRIC_HALF_ANGLE,
) -> float:
    """Equivalent unwrapped screw friction, mu / cos(thread half-angle)."""
    return _nonnegative(friction, "friction") / math.cos(_angle(half_angle))


def self_locking_threshold(
    pitch: float = M8_PITCH,
    pitch_diameter: float = M8_CONTACT_PITCH_DIAMETER,
    half_angle: float = METRIC_HALF_ANGLE,
    starts: int = 1,
) -> float:
    """Coulomb coefficient at neutral backdrive: cos(alpha) * tan(lambda)."""
    return math.cos(_angle(half_angle)) * math.tan(
        lead_angle(pitch, pitch_diameter, starts)
    )


def is_self_locking(
    friction: float,
    *,
    pitch: float = M8_PITCH,
    pitch_diameter: float = M8_CONTACT_PITCH_DIAMETER,
    half_angle: float = METRIC_HALF_ANGLE,
    starts: int = 1,
) -> bool:
    """True strictly above the neutral threshold, without a holding servo."""
    return _nonnegative(friction, "friction") > self_locking_threshold(
        pitch, pitch_diameter, half_angle, starts
    )


def raising_torque(
    axial_load: float,
    friction: float = 0.15,
    *,
    pitch: float = M8_PITCH,
    pitch_diameter: float = M8_CONTACT_PITCH_DIAMETER,
    half_angle: float = METRIC_HALF_ANGLE,
    starts: int = 1,
) -> float:
    """Positive torque magnitude required to move against an axial load.

    A push assisting insertion belongs in ``lowering_torque`` instead.
    This excludes torque at a seated bearing surface.
    """
    axial_load = _nonnegative(axial_load, "axial load")
    pitch_diameter = _positive(pitch_diameter, "pitch diameter")
    tangent = math.tan(lead_angle(pitch, pitch_diameter, starts))
    mu = effective_friction(friction, half_angle)
    denominator = 1 - mu * tangent
    if denominator <= 0:
        raise ValueError("raising torque is singular for this friction/lead combination")
    return axial_load * pitch_diameter / 2 * (tangent + mu) / denominator


def lowering_torque(
    axial_load: float,
    friction: float = 0.15,
    *,
    pitch: float = M8_PITCH,
    pitch_diameter: float = M8_CONTACT_PITCH_DIAMETER,
    half_angle: float = METRIC_HALF_ANGLE,
    starts: int = 1,
) -> float:
    """Torque in the load-assisted direction; negative means braking is needed.

    Zero is neutral backdrive. At zero friction the load drives motion, so
    the required torque is negative rather than a claimed self-lock.
    """
    axial_load = _nonnegative(axial_load, "axial load")
    pitch_diameter = _positive(pitch_diameter, "pitch diameter")
    tangent = math.tan(lead_angle(pitch, pitch_diameter, starts))
    mu = effective_friction(friction, half_angle)
    return axial_load * pitch_diameter / 2 * (mu - tangent) / (1 + mu * tangent)
