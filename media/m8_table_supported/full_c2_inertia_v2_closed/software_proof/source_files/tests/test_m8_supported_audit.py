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
                                     strict_original_checks, rotation_changes,
                                     saved_grasp_retention)
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


def test_isolated_unloaded_endpoint_preserves_original_rolling_gate_failure():
    from yam_twin.m8_supported_simulation import TableLoadWindow
    dt, weight, count, acquisition = .00005, 1., 5000, 2500
    table, hand = np.ones(count), np.zeros(count)
    table[3500] = 0.
    active = np.arange(count) >= acquisition
    original = TableLoadWindow(.100, weight)
    source_failures, duration = 0, None
    for index in range(count):
        original.observe(table[index], hand[index], dt, True)
        if active[index]:
            source_failures += int(not original.ready)
        duration = original.report()["observed_window_s"]
    audited = rolling_load_shares(table, hand, active, dt, weight,
                                       duration_s=duration)
    assert audited["minimum_mean_table_weight_fraction"] >= .90
    assert audited["minimum_loaded_table_duty"] >= .99
    assert audited["maximum_mean_positive_hand_upward_weight_fraction"] <= .10
    assert source_failures == audited["failed_windows"] == 1
    assert audited["unloaded_final_tick_windows"] == 1
    assert not audited["passed"]
    # The strict all-step criterion independently retains the same real gap.
    strict = load_statistics(table[active], hand[active],
        np.ones((np.count_nonzero(active), 2)), dt, weight)
    assert not strict["continuous_table_load"]
    assert strict["table_unloaded_steps"] == 1


def test_qualified_closed_stops_detect_grasp_slip_and_missing_measured_reference():
    rows = [{"phase": "settle_bolt", "time_s": 1.}, {"phase": "stop_1", "time_s": 1.12}]
    records = [{"contact": {"pad_normal_force_N": [16., 16.]}}, {}]
    poses = [(np.zeros(3), np.eye(3)),
             (np.array([.0015, 0., 0.]), Rotation.from_euler("z", .05).as_matrix())]
    metadata = {"right_grasp_acquisitions": [{"phase": "settle_bolt", "time_s": 1.,
        "pad_normal_force_N": [16., 16.], "grasp_relative_bolt_head_position_m": [0., 0., 0.]}]}
    measured = saved_grasp_retention(rows, records, poses, metadata)
    assert not measured["closed_manipulation_without_measured_grasp_reference"]
    assert measured["maximum_independent_post_grasp_translation_slip_m"] > .001
    assert measured["maximum_independent_post_grasp_rotation_slip_rad"] > np.deg2rad(2)
    assert rows[1]["independent_post_grasp_translation_slip_m"] == pytest.approx(.0015)
    missing = saved_grasp_retention([{"phase": "stop_2", "time_s": 2.}], [{}],
        [(np.zeros(3), np.eye(3))], {"right_grasp_acquisitions": []})
    assert missing["closed_manipulation_without_measured_grasp_reference"]


@pytest.mark.parametrize("phase", ["transfer_bolt_weight", "reverse_seat_1"])
def test_real_closed_weight_transfer_and_reverse_seating_detect_grasp_slip(phase):
    rows = [{"phase": "settle_bolt", "time_s": 1.}, {"phase": phase, "time_s": 1.2}]
    records = [{"contact": {"pad_normal_force_N": [16., 16.]}}, {}]
    held = [(np.zeros(3), np.eye(3)),
            (np.array([0., .0015, 0.]), Rotation.from_euler("x", .05).as_matrix())]
    metadata = {"right_grasp_acquisitions": [{"phase": "settle_bolt", "time_s": 1.,
        "pad_normal_force_N": [16., 16.], "grasp_relative_bolt_head_position_m": [0., 0., 0.]}]}
    result = saved_grasp_retention(rows, records, held, metadata)
    assert not result["closed_manipulation_without_measured_grasp_reference"]
    assert result["maximum_independent_post_grasp_translation_slip_m"] > .001
    assert result["maximum_independent_post_grasp_rotation_slip_rad"] > np.deg2rad(2)
    assert rows[1]["independent_post_grasp_translation_slip_m"] == pytest.approx(.0015)
    # A real reverse motion must also fail coverage when no measured native
    # pickup/regrasp reference exists; phase names cannot exempt its grip.
    missing = saved_grasp_retention([{"phase": phase, "time_s": 2.}], [{}],
        [(np.zeros(3), np.eye(3))], {"right_grasp_acquisitions": []})
    assert missing["closed_manipulation_without_measured_grasp_reference"]


def test_early_quiet_regrasp_guards_remaining_settle_ticks_with_its_new_reference():
    rows = [{"phase": "settle_regrip_search_2", "time_s": 2.1},
            {"phase": "settle_regrip_search_2", "time_s": 2.105}]
    records = [{"contact": {"pad_normal_force_N": [16., 16.]}, "right_grasp_guard_active": False},
               {"right_grasp_guard_active": True}]
    held = [(np.zeros(3), np.eye(3)), (np.array([.0015, 0., 0.]), np.eye(3))]
    metadata = {"right_grasp_acquisitions": [{"phase": "settle_regrip_search_2", "time_s": 2.1,
        "pad_normal_force_N": [16., 16.], "grasp_relative_bolt_head_position_m": [0., 0., 0.],
        "continuous_quiet_bilateral_streak_s": .1}]}
    measured = saved_grasp_retention(rows, records, held, metadata)
    assert not measured["closed_manipulation_without_measured_grasp_reference"]
    assert measured["maximum_independent_post_grasp_translation_slip_m"] == pytest.approx(.0015)
    assert "independent_post_grasp_translation_slip_m" not in rows[0]
    assert rows[1]["independent_post_grasp_translation_slip_m"] == pytest.approx(.0015)


@pytest.mark.parametrize("phase", ["open_settle_search_2", "open_hold_search_2"])
def test_fully_open_holds_cannot_reuse_a_previous_closed_grasp_reference(phase):
    rows = [{"phase": "settle_bolt", "time_s": 1.}, {"phase": phase, "time_s": 1.2},
            {"phase": "start_thread_2", "time_s": 1.4}]
    records = [{"contact": {"pad_normal_force_N": [16., 16.]}},
               {"right_grasp_guard_active": False}, {"right_grasp_guard_active": True}]
    held = [(np.zeros(3), np.eye(3)) for _ in rows]
    metadata = {"right_grasp_acquisitions": [{"phase": "settle_bolt", "time_s": 1.,
        "pad_normal_force_N": [16., 16.], "grasp_relative_bolt_head_position_m": [0., 0., 0.]}]}
    measured = saved_grasp_retention(rows, records, held, metadata)
    assert measured["closed_manipulation_without_measured_grasp_reference"]


def test_a_regrasp_cannot_declare_a_short_quiet_acquisition_as_100ms():
    rows = [{"phase": "settle_regrip_search_2", "time_s": 2.}]
    records = [{"contact": {"pad_normal_force_N": [16., 16.]}, "right_grasp_guard_active": False}]
    metadata = {"right_grasp_acquisitions": [{"phase": "settle_regrip_search_2", "time_s": 2.,
        "pad_normal_force_N": [16., 16.], "grasp_relative_bolt_head_position_m": [0., 0., 0.],
        "continuous_quiet_bilateral_streak_s": .05}]}
    with pytest.raises(ValueError, match="continuous100ms"):
        saved_grasp_retention(rows, records, [(np.zeros(3), np.eye(3))], metadata)


def test_an_inactive_flag_cannot_waive_an_original_closed_phase_grasp_check():
    rows = [{"phase": "turn_1", "time_s": 2.}]
    records = [{"right_grasp_guard_active": False}]
    measured = saved_grasp_retention(rows, records, [(np.zeros(3), np.eye(3))],
        {"right_grasp_acquisitions": []})
    assert measured["closed_manipulation_without_measured_grasp_reference"]


def test_feedback_sample_guard_flag_covers_closed_settle_and_cannot_be_omitted():
    rows = [{"phase": "settle_regrip_search_2", "time_s": 2.}]
    metadata = {"right_grasp_acquisitions": [], "physical_feedback_controller": "supported-head-feedback-v1"}
    held = [(np.zeros(3), np.eye(3))]
    with pytest.raises(ValueError, match="lacks.*grasp-active"):
        saved_grasp_retention(rows, [{}], held, metadata)
    measured = saved_grasp_retention(rows, [{"native_feedback": {"right_grasp_guard_active": True}}],
        held, metadata)
    assert measured["closed_manipulation_without_measured_grasp_reference"]


def _feedback_window_columns(count=500, dt=.001):
    columns = {"time": np.arange(1, count+1)*dt}
    for name in ("weight_observation_valid", "open_observation_valid", "external_drive_zero", "fully_open_unassisted",
                 "thread_gravity_opposing_force_N"):
        columns[name] = np.ones(count)
    for name in ("bolt_world_support_contact_count", "nonthread_block_bolt_contact_count",
                 "relative_bolt_axial_velocity_m_per_s", "radial_offset_m", "bolt_tilt_rad",
                 "right_robot_bolt_contact_count", "relative_bolt_angular_speed_rad_per_s",
                 "hand_gravity_opposing_force_N"):
        columns[name] = np.zeros(count)
    return columns


def test_live_feedback_open_readiness_precedes_a_later_endpoint_failure():
    from audit_m8_supported_trace import feedback_weight_windows
    columns = _feedback_window_columns()
    columns["relative_bolt_angular_speed_rad_per_s"][470] = .02
    measured = feedback_weight_windows(columns, .001, 1., fully_open=True)
    assert measured["ready"][150]
    assert measured["ready"][469]
    assert not measured["ready"][-1]
    # General support stays valid: the added quiet gate must not relabel a
    # perfectly supported noncontact bolt as having lost its gravity load.
    assert feedback_weight_windows(columns, .001, 1.)["ready"][-1]


def test_feedback_open_window_rejects_whole_robot_contact_even_with_zero_hand_force():
    from audit_m8_supported_trace import feedback_weight_windows
    columns = _feedback_window_columns(100)
    columns["right_robot_bolt_contact_count"][50] = 1.
    assert np.all(columns["hand_gravity_opposing_force_N"] == 0)
    assert not feedback_weight_windows(columns, .001, 1., fully_open=True)["ready"][-1]


def test_feedback_weight_window_preserves_positive_hand_and_native_endpoint_conditions():
    from audit_m8_supported_trace import feedback_weight_windows
    columns = _feedback_window_columns(100)
    columns["hand_gravity_opposing_force_N"] = np.tile([.3, -.3], 50)
    assert np.mean(columns["hand_gravity_opposing_force_N"]) == pytest.approx(0.)
    assert not feedback_weight_windows(columns, .001, 1.)["ready"][-1]
    columns["hand_gravity_opposing_force_N"][:] = 0.
    columns["thread_gravity_opposing_force_N"][-1] = 0.
    assert not feedback_weight_windows(columns, .001, 1.)["ready"][-1]


def test_feedback_open_window_allows_an_independently_moving_noncontact_hand():
    from audit_m8_supported_trace import feedback_weight_windows
    columns = _feedback_window_columns(100)
    columns["relative_hand_angular_speed_rad_per_s"] = np.full(100, 4.)
    assert feedback_weight_windows(columns, .001, 1., fully_open=True)["ready"][-1]


def test_feedback_open_window_uses_its_own_original_validity_and_rejects_malformed_flags():
    from audit_m8_supported_trace import feedback_weight_windows
    columns = _feedback_window_columns(100)
    columns["weight_observation_valid"][:] = 0.
    assert not feedback_weight_windows(columns, .001, 1.)["ready"][-1]
    assert feedback_weight_windows(columns, .001, 1., fully_open=True)["ready"][-1]
    columns["open_observation_valid"][50] = 2.
    with pytest.raises(ValueError, match="validity flags"):
        feedback_weight_windows(columns, .001, 1., fully_open=True)


def test_live_open_event_waits_for_minimum_dwell_after_an_earlier_ready_window():
    from audit_m8_supported_trace import first_eligible_open_ready_index
    ids = np.arange(500)
    ready = np.arange(500) >= 99
    first = first_eligible_open_ready_index(ids, .001, .12, ready, np.zeros(500))
    assert first == 119
    assert first != np.flatnonzero(ready)[0]
    # A new spike at the eligible boundary resets readiness; a historical
    # ready observation cannot stand in for the current event solve.
    ready[119:219] = False
    assert first_eligible_open_ready_index(ids, .001, .12, ready, np.zeros(500)) == 219


def test_native_grasp_flags_start_after_acquisition_and_cannot_omit_later_settle_ticks():
    from audit_m8_supported_trace import original_grasp_flag_audit
    columns = {"time": np.arange(1, 6)*.01, "phase_index": np.zeros(5, dtype=int),
        "right_grasp_guard_active": np.array([False, False, False, True, True])}
    metadata = {"right_grasp_acquisitions": [{"time_s": .03}],
        "maximum_phase_plan": [["settle_regrip_search_2", .05, 0., 0., .0184, .0184, False]],
        "control_config": {"arm": {"closed_aperture": .0184}}}
    assert original_grasp_flag_audit(columns, ["settle_regrip_search_2"], metadata)["guarded_native_steps"] == 2
    columns["right_grasp_guard_active"][2] = True
    with pytest.raises(ValueError, match="already guarded"):
        original_grasp_flag_audit(columns, ["settle_regrip_search_2"], metadata)
    columns["right_grasp_guard_active"][2] = False
    columns["right_grasp_guard_active"][4] = False
    with pytest.raises(ValueError, match="every post-acquisition"):
        original_grasp_flag_audit(columns, ["settle_regrip_search_2"], metadata)


def test_original_feedback_schema_rejects_nonboolean_native_control_flags():
    from audit_m8_supported_trace import validate_feedback_columns
    columns = _feedback_window_columns(100)
    columns["phase_index"] = np.zeros(100, dtype=int)
    for name in ("right_grasp_guard_active", "right_axial_float_active", "weight_window_ready",
                 "open_weight_window_ready", "all_hard_guards_held"):
        columns[name] = np.zeros(100)
    columns["loaded_actual_interior_flank_contact_count"] = np.zeros(100)
    columns["native_thread_contact_count"] = np.zeros(100)
    validate_feedback_columns(columns, columns["time"], columns["phase_index"])
    columns["right_grasp_guard_active"][57] = 2.
    with pytest.raises(ValueError, match="flags must be boolean"):
        validate_feedback_columns(columns, columns["time"], columns["phase_index"])
