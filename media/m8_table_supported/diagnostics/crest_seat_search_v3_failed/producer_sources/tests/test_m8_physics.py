"""Physical responses that an imposed helix cannot satisfy.

Run through scripts/run_m8.sh; stock SI-scale SDF search is too coarse for
this contact model, and failing that runtime check is intentional.
"""
from dataclasses import replace

import mujoco
import numpy as np

from thread_lab.benchmark import probe
from thread_lab.model import ThreadConfig, make_model
from thread_lab.runtime import require_micron_engine


def test_free_nut_and_finite_hand_without_engagement_constraints():
    require_micron_engine()
    m = make_model(ThreadConfig(with_gripper=True))
    assert m.neq == 0
    assert m.jnt_type[m.joint("nut_free").id] == mujoco.mjtJoint.mjJNT_FREE
    assert m.nu == 2  # jaw motors only
    assert all(m.actuator_trnid[:, 0] != m.joint("nut_free").id)
    assert np.all(m.actuator_forcelimited)


def test_contact_torque_emerges_as_metric_lead():
    require_micron_engine()
    report, _ = probe(duration=.5)
    assert report["turns"] < -.1
    assert report["axial_travel_mm"] < -.1
    assert report["lead_error_percent"] < 2
    assert report["worst_penetration_um"] < 10
    assert not report["warnings"]


def test_frictionless_backdrive_releases_gravitational_energy():
    require_micron_engine()
    report, _ = probe(ThreadConfig(friction=0), duration=.4, torque=0)
    assert report["turns"] < -.025
    assert report["axial_travel_mm"] < -.04
    assert report["lead_error_percent"] < 2
    assert report["worst_penetration_um"] < 10
    assert report["rigid_body_energy_change_J"] < 0
    assert report["external_work_J"] <= 0
    assert not report["warnings"]


def test_rotation_without_bolt_has_no_insertion():
    require_micron_engine()
    report, _ = probe(ThreadConfig(with_bolt=False, gravity=0), duration=.2)
    assert report["turns"] < -.05
    assert abs(report["axial_travel_mm"]) < 1e-9
    assert report["max_contact_normal_N"] == 0
