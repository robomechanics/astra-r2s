"""Independent checks reject hidden lifting and unsampled support-load gaps."""
from pathlib import Path
import sys

import mujoco
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from audit_m8_supported_trace import (load_statistics, settled_baseline_passes,
                                     transform_contact_wrench, recorded_contact_wrench,
                                     rolling_load_shares, all_step_bolt_measurements,
                                     strict_original_checks, rotation_changes)
from scipy.spatial.transform import Rotation


def test_table_baseline_rejects_a_half_carried_block_and_signed_load_reversal():
    count, dt, weight = 2000, .00005, 1.
    pads = np.full((count, 2), 16.)
    supported = load_statistics(np.full(count, weight), np.zeros(count), pads, dt, weight)
    assert settled_baseline_passes(supported)
    half_carried = load_statistics(np.full(count, .5), np.full(count, .5), pads, dt, weight)
    assert not settled_baseline_passes(half_carried)
    downward = load_statistics(np.full(count, -1.), np.zeros(count), pads, dt, weight)
    assert not settled_baseline_passes(downward)


def test_positive_upward_hand_contribution_cannot_cancel_with_downward_samples():
    count, dt, weight = 2000, .00005, 1.
    hand = np.tile([.4, -.4], count//2)
    measured = load_statistics(np.ones(count), hand, np.ones((count, 2)), dt, weight)
    assert measured["mean_positive_hand_upward_weight_fraction"] == pytest.approx(.2)
    assert not settled_baseline_passes(measured)


def test_whole_task_weight_bearing_rejects_lifting_after_valid_initial_settle():
    table, hand, active = np.ones(6000), np.zeros(6000), np.arange(6000) >= 2000
    assert rolling_load_shares(table, hand, active, .00005, 1.)["passed"]
    table[4000:], hand[4000:] = .5, .5
    result = rolling_load_shares(table, hand, active, .00005, 1.)
    assert not result["passed"]
    assert result["minimum_mean_table_weight_fraction"] == pytest.approx(.5)
    assert result["maximum_mean_positive_hand_upward_weight_fraction"] == pytest.approx(.5)


def test_one_hidden_physics_step_preserves_strict_table_and_pad_failures():
    count, dt = 2000, .00005
    table, hand, pads = np.ones(count), np.zeros(count), np.ones((count, 2))
    table[57], pads[103, 0] = 0., 0.
    measured = load_statistics(table, hand, pads, dt, 1.)
    # A valid time-averaged resting baseline is distinct from strict continuity.
    assert settled_baseline_passes(measured)
    assert not measured["continuous_table_load"]
    assert not measured["continuous_bilateral_pad_load"]
    assert measured["table_unloaded_steps"] == 1
    assert measured["pad_unloaded_steps"] == [1, 0]
    assert measured["maximum_consecutive_table_unloaded_duration_s"] == dt


def test_contact_row_axes_and_action_reaction_include_tangents_and_lever_arm():
    frame = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
    local = [2., 3., 4., 5., 6., 7.]
    wrench = transform_contact_wrench(frame, local, [1., 0., 0.], [0., 0., 0.], 1.)
    np.testing.assert_allclose(wrench, [3., 4., 2., 6., 5., 9.])
    np.testing.assert_array_equal(transform_contact_wrench(
        frame, local, [1., 0., 0.], [0., 0., 0.], -1.), -wrench)


def test_native_contact_force_sign_and_rigid_child_accounting():
    model = mujoco.MjModel.from_xml_string("""
      <mujoco><option gravity="0 0 -9.81"/><worldbody>
        <body name="fixed_support"><geom name="floor" type="plane" size="1 1 .01"/></body>
        <body name="free_block" pos="0 0 .004"><freejoint/>
          <inertial mass=".1" pos="0 0 0" diaginertia=".001 .001 .001"/>
          <body name="rigid_child"><geom name="block_collision" type="box" size=".01 .01 .005" mass="0"/></body>
        </body>
      </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    records = []
    block = model.body("free_block").id
    for index, contact in enumerate(data.contact):
        local = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, local)
        records.append({"geom1": model.geom(int(contact.geom1)).name,
            "geom2": model.geom(int(contact.geom2)).name,
            "contact_frame": contact.frame.reshape(3, 3).tolist(),
            "local_contact_force_N_Nm": local.tolist(),
            "contact_position_world_m": contact.pos.tolist(),
            "block_origin_world_m": data.xpos[block].tolist()})
    assert records
    wrench = recorded_contact_wrench(records, model, block)
    assert wrench[2] > .1*9.81
    # The rigid child has no free joint and must map to the same parent root.
    np.testing.assert_allclose(recorded_contact_wrench(
        records, model, model.body("rigid_child").id), wrench)
    assert np.linalg.norm(wrench[[0, 1, 3, 4, 5]]) < 1e-10


def test_nonfinite_or_nonorthogonal_contact_records_are_rejected():
    with pytest.raises(ValueError):
        transform_contact_wrench(np.ones((3, 3)), np.zeros(6), np.zeros(3), np.zeros(3), 1.)
    with pytest.raises(ValueError):
        load_statistics([float("nan")], [0.], [[1., 1.]], .00005, 1.)


def _qualified_turn_and_reset_columns():
    n = 100
    columns = {"phase_index": np.r_[np.zeros(n, dtype=int), np.ones(n, dtype=int)],
        "bolt_base_insertion_m": np.r_[np.linspace(0, .000625, n), np.full(n, .000625)],
        "bolt_yaw_unwrapped_rad": np.r_[np.linspace(0, np.pi, n), np.full(n, np.pi)],
        "formed_flank_overlap_m": np.full(2*n, .0015),
        "bolt_world_support_count": np.zeros(2*n),
        "right_hand_bolt_contact_count": np.r_[np.ones(n), np.zeros(n)],
        "head_block_seating_contact_count": np.zeros(2*n)}
    metadata = {"scene_config": {"base": {"thread": {"pitch": .00125}}},
        "control_config": {"arm": {"stroke_angle_rad": np.pi}},
        "phases": [{"phase": "turn_1", "started_engaged": True},
                   {"phase": "reset_open_1", "started_engaged": True}]}
    return columns, metadata


def test_qualified_lead_uses_measured_yaw_and_rejects_axial_motion_without_rotation():
    columns, metadata = _qualified_turn_and_reset_columns()
    result = all_step_bolt_measurements(columns, ["turn_1", "reset_open_1"], metadata)
    assert result[0]["qualified_stroke_passed"]
    columns["bolt_yaw_unwrapped_rad"][:100] = 0.
    result = all_step_bolt_measurements(columns, ["turn_1", "reset_open_1"], metadata)
    assert not result[0]["qualified_stroke_passed"]


def test_passive_reset_rejects_hidden_one_substep_motion_contact_or_floor_support():
    columns, metadata = _qualified_turn_and_reset_columns()
    assert all_step_bolt_measurements(columns, ["turn_1", "reset_open_1"], metadata)[1]["captured_passive_reset_passed"]
    for field, bad in (("bolt_base_insertion_m", .000645),
                       ("right_hand_bolt_contact_count", 1.),
                       ("head_block_seating_contact_count", 1.),
                       ("bolt_world_support_count", 1.)):
        changed = {key: value.copy() for key, value in columns.items()}
        changed[field][157] = bad
        result = all_step_bolt_measurements(changed, ["turn_1", "reset_open_1"], metadata)
        assert not result[1]["captured_passive_reset_passed"]


def test_absent_legacy_minima_export_preserves_failed_gate_and_rejects_physical_nan():
    original = {"phases": [], "acceptance_checks": {"picked_up_free_bolt": {
        "passed": False, "sampled_minimum_closed_transport_pad_normals_N": [float("inf"), float("inf")]}}}
    exported, absent = strict_original_checks(original)
    assert exported["picked_up_free_bolt"]["passed"] is False
    assert exported["picked_up_free_bolt"]["sampled_minimum_closed_transport_pad_normals_N"] == [None, None]
    assert len(absent) == 2
    assert np.isinf(original["acceptance_checks"]["picked_up_free_bolt"]["sampled_minimum_closed_transport_pad_normals_N"]).all()
    original["phases"] = [{"phase": "transport_bolt"}]
    with pytest.raises(ValueError, match="Nonfinite original physical"):
        strict_original_checks(original)


def test_rotated_initial_block_is_not_mislabeled_as_tabletop_motion():
    initial = Rotation.from_euler("z", np.pi/2).as_matrix()
    later = Rotation.from_euler("x", .01).as_matrix()@initial
    result = rotation_changes(np.array([initial, later]), initial)
    np.testing.assert_allclose(result, [0., .01], atol=1e-12)
