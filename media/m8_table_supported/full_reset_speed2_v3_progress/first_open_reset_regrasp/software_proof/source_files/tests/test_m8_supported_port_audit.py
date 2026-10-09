"""Pure original-ledger regressions for the supported controller/auditor port.

Execute only the auditor's real pure function definitions. Application imports,
model construction, native contact queries, and integration are never invoked.
"""
import ast
import copy
from collections import deque
from pathlib import Path

import numpy as np
import pytest


SOURCE = Path(__file__).resolve().parents[1] / "scripts" / "audit_m8_supported_trace.py"
PURE_NAMES = {
    "feedback_weight_windows",
    "first_eligible_open_ready_index",
    "original_grasp_flag_audit",
    "inertia_control_rows_audit",
    "source_bound_search_event_audit",
}
nodes = [node for node in ast.parse(SOURCE.read_text()).body
         if isinstance(node, ast.FunctionDef) and node.name in PURE_NAMES]
assert {node.name for node in nodes} == PURE_NAMES and len(nodes) == len(PURE_NAMES)
namespace = {"np": np}
exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), "exec"), namespace)
weight_windows = namespace["feedback_weight_windows"]
first_open_ready = namespace["first_eligible_open_ready_index"]
grasp_flags = namespace["original_grasp_flag_audit"]
inertia_rows = namespace["inertia_control_rows_audit"]
inertia_node = next(node for node in nodes if node.name == "inertia_control_rows_audit")
shape_node = next(node for node in inertia_node.body if isinstance(node, ast.Assign)
                  and any(isinstance(target, ast.Name) and target.id == "shapes" for target in node.targets))
INERTIA_SHAPES = ast.literal_eval(shape_node.value)
search_events_audit = namespace["source_bound_search_event_audit"]


def pure_class(relative_source, name):
    source = SOURCE.parents[1] / relative_source
    node = next(node for node in ast.parse(source.read_text()).body
                if isinstance(node, ast.ClassDef) and node.name == name)
    namespace["deque"] = deque
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(source), "exec"), namespace)
    return namespace[name]


EntryWindow = pure_class("yam_twin/m8_insertion_simulation.py", "EntrySupportWindow")
CrestWindow = pure_class("yam_twin/m8_supported_crest.py", "CrestSeatDropWindowV3")
TableWindow = pure_class("yam_twin/m8_supported_simulation.py", "TableLoadWindow")


def grasp_ledger():
    labels = ["settle_bolt", "lift_bolt", "release_search_1", "open_settle_search_1",
              "reset_open_search_1", "regrip_search_2", "settle_regrip_search_2",
              "start_thread_search_2"]
    phase = np.repeat(np.arange(len(labels)), [3, 2, 2, 3, 3, 2, 5, 2])
    time = np.arange(1, len(phase) + 1) * .05
    active = np.zeros(len(phase), dtype=bool)
    active[2:5] = True              # Initial acquisition guards its solved row.
    active[17:] = True              # Quiet acquisition is row16, then guarded.
    columns = {"time": time, "phase_index": phase, "right_grasp_guard_active": active}
    closed, opened = .0184, .030
    metadata = {
        "physical_feedback_controller": "supported-crest-c2-hybrid-inertia-v2",
        "right_grasp_acquisitions": [
            {"phase": "settle_bolt", "time_s": float(time[2])},
            {"phase": "settle_regrip_search_2", "time_s": float(time[16]),
             "continuous_quiet_bilateral_streak_s": .100},
        ],
        "maximum_phase_plan": [
            [label, .25, 0., 0., closed,
             opened if label.startswith(("release_", "open_settle_", "reset_open_")) else closed,
             False] for label in labels
        ],
        "control_config": {"arm": {"closed_aperture": closed}, "regrasp_window_s": .100},
    }
    return columns, labels, metadata


def open_ledger(count=260, dt=.001):
    columns = {"time": np.arange(1, count + 1) * dt}
    for name in ("open_observation_valid", "external_drive_zero", "fully_open_unassisted",
                 "thread_gravity_opposing_force_N"):
        columns[name] = np.ones(count)
    for name in ("weight_observation_valid", "bolt_world_support_contact_count",
                 "nonthread_block_bolt_contact_count", "relative_bolt_axial_velocity_m_per_s",
                 "radial_offset_m", "bolt_tilt_rad", "right_robot_bolt_contact_count",
                 "relative_bolt_angular_speed_rad_per_s", "hand_gravity_opposing_force_N"):
        columns[name] = np.zeros(count)
    # General closed quiet is intentionally unavailable while the free hand
    # moves. OPEN uses its own actual-bolt quiet and zero-whole-right-contact.
    columns["relative_hand_angular_speed_rad_per_s"] = np.linspace(.5, 4., count)
    return columns


def eligible(columns, *, ids=None, minimum=.120, dt=.001, legacy_hand_argument=True):
    opened = weight_windows(columns, dt, 1., fully_open=True)
    if ids is None:
        ids = np.arange(len(columns["time"]))
    if legacy_hand_argument:
        first = first_open_ready(ids, dt, minimum, opened["ready"],
                                columns["relative_hand_angular_speed_rad_per_s"])
    else:
        first = first_open_ready(ids, dt, minimum, opened["ready"])
    return first, opened


def test_initial_same_row_and_quiet_next_row_acquisitions_preserve_every_guard_tick():
    columns, labels, metadata = grasp_ledger()
    original = copy.deepcopy(metadata)
    for value in columns.values():
        value.setflags(write=False)
    result = grasp_flags(columns, labels, metadata)
    assert result["source_specific_acquisition_guard_boundaries_verified"]
    assert result["initial_same_row_acquisition_count"] == 1
    assert result["quiet_next_tick_acquisition_count"] == 1
    assert result["guarded_native_steps"] == 8
    assert result["intentional_open_reference_clears_verified"]
    assert metadata == original


@pytest.mark.parametrize("index,wrong", [(2, False), (16, True)])
def test_initial_and_quiet_acquisition_guard_boundaries_cannot_be_interchanged(index, wrong):
    columns, labels, metadata = grasp_ledger()
    columns["right_grasp_guard_active"][index] = wrong
    with pytest.raises(ValueError):
        grasp_flags(columns, labels, metadata)


def test_initial_acquisition_cannot_move_to_an_earlier_active_settle_tick():
    columns, labels, metadata = grasp_ledger()
    metadata["right_grasp_acquisitions"][0]["time_s"] = float(columns["time"][1])
    columns["right_grasp_guard_active"][1] = True
    with pytest.raises(ValueError, match="final solved row"):
        grasp_flags(columns, labels, metadata)


@pytest.mark.parametrize("version", ["supported-head-feedback-v1", "supported-crest-c2-hybrid-inertia-v2"])
@pytest.mark.parametrize("acquisition", [0, 1])
def test_real_feedback_metadata_cannot_use_the_legacy_missing_phase_shorthand(version, acquisition):
    columns, labels, metadata = grasp_ledger()
    metadata["physical_feedback_controller"] = version
    del metadata["right_grasp_acquisitions"][acquisition]["phase"]
    with pytest.raises(ValueError, match="acquisition phase"):
        grasp_flags(columns, labels, metadata)


@pytest.mark.parametrize("malformed", ["wrong_phase", "unknown_phase", "absent_time", "missing_reference"])
def test_missing_or_incorrect_native_acquisition_record_is_rejected(malformed):
    columns, labels, metadata = grasp_ledger()
    if malformed == "wrong_phase":
        metadata["right_grasp_acquisitions"][1]["phase"] = "settle_regrip_search_3"
    elif malformed == "unknown_phase":
        metadata["right_grasp_acquisitions"][0] = {"phase": "lift_bolt", "time_s": float(columns["time"][3])}
    elif malformed == "absent_time":
        metadata["right_grasp_acquisitions"][1]["time_s"] += .001
    else:
        metadata["right_grasp_acquisitions"].pop(1)
    with pytest.raises(ValueError):
        grasp_flags(columns, labels, metadata)


@pytest.mark.parametrize("streak", [None, .099, -1., np.nan, np.inf, ".100", True])
def test_quiet_regrasp_requires_a_real_finite_continuous_streak(streak):
    columns, labels, metadata = grasp_ledger()
    if streak is None:
        del metadata["right_grasp_acquisitions"][1]["continuous_quiet_bilateral_streak_s"]
    else:
        metadata["right_grasp_acquisitions"][1]["continuous_quiet_bilateral_streak_s"] = streak
    with pytest.raises(ValueError, match="continuous native streak"):
        grasp_flags(columns, labels, metadata)


@pytest.mark.parametrize("index", [4, 17, 19, 20])
def test_postacquisition_guard_gap_remains_a_failure_in_transport_settle_and_turn(index):
    columns, labels, metadata = grasp_ledger()
    columns["right_grasp_guard_active"][index] = False
    with pytest.raises(ValueError, match="every post-acquisition closed native tick"):
        grasp_flags(columns, labels, metadata)


@pytest.mark.parametrize("hand_speed", [0., 4., 25.])
def test_actual_bolt_quiet_open_support_allows_an_independently_moving_free_hand(hand_speed):
    columns = open_ledger()
    columns["relative_hand_angular_speed_rad_per_s"][:] = hand_speed
    first, opened = eligible(columns)
    assert first == 119
    assert opened["ready"][first]
    assert np.all(columns["right_robot_bolt_contact_count"] == 0)
    assert not weight_windows(columns, .001, 1.)["ready"].any()
    assert eligible(columns, legacy_hand_argument=False)[0] == first


def test_open_eligibility_minimum_dwell_uses_local_phase_steps():
    columns = open_ledger(400)
    ids = np.arange(100, 360)
    first, opened = eligible(columns, ids=ids)
    assert opened["ready"][ids[0]]
    assert first == 219              # 120 native solves in this OPEN phase.


def test_actual_bolt_angular_speed_limit_stays_binding_while_hand_moves():
    columns = open_ledger()
    columns["relative_bolt_angular_speed_rad_per_s"][:] = .010001
    first, opened = eligible(columns)
    assert first is None
    assert not opened["ready"].any()
    columns["relative_bolt_angular_speed_rad_per_s"][:] = .01
    assert eligible(columns)[0] == 119


@pytest.mark.parametrize("field,bad", [
    ("relative_bolt_angular_speed_rad_per_s", .02),
    ("right_robot_bolt_contact_count", 1.),
    ("fully_open_unassisted", 0.),
    ("open_observation_valid", 0.),
    ("external_drive_zero", 0.),
    ("bolt_world_support_contact_count", 1.),
    ("nonthread_block_bolt_contact_count", 1.),
    ("relative_bolt_axial_velocity_m_per_s", .000201),
])
def test_one_invalid_open_solve_at_eligible_boundary_requires_a_fresh_contiguous_window(field, bad):
    columns = open_ledger()
    columns[field][119] = bad
    first, opened = eligible(columns)
    assert opened["ready"][99]       # Earlier readiness cannot be latched.
    assert not opened["ready"][119]
    assert not opened["ready"][218]
    assert opened["ready"][219]
    assert first == 219


def test_short_clean_tail_after_right_robot_contact_gap_cannot_qualify_open():
    columns = open_ledger(200)
    columns["right_robot_bolt_contact_count"][119] = 1.
    assert np.all(columns["hand_gravity_opposing_force_N"] == 0.)
    first, opened = eligible(columns)
    assert opened["ready"][118]
    assert not opened["ready"][-1]
    assert first is None


def remap_fixture_wrenches(columns, feedback, arm, torque_ranges):
    """Independent fixture map for Jp=[I,0], Jr=[0,I]; no audited solver."""
    for index in range(len(columns["time"])):
        total = columns["pd_wrench_world_N_Nm"][index] + columns["ff_wrench_world_N_Nm"][index]
        force_size = sum(float(value) ** 2 for value in total[:3]) ** .5
        torque_size = sum(float(value) ** 2 for value in total[3:]) ** .5
        force_factor = min(1., arm["maximum_cartesian_force"] / force_size) if force_size else 1.
        torque_factor = min(1., arm["maximum_cartesian_torque"] / torque_size) if torque_size else 1.
        capped = np.r_[total[:3] * force_factor, total[3:] * torque_factor]
        # The analytically specified coordinate-block Jacobians make J^T
        # simply concatenate the translation force and rotation torque.
        motor = capped + columns["arm_bias_torque_Nm"][index] + columns["native_drag_torque_Nm"][index]
        caps = [min(abs(float(lower)), abs(float(upper))) for lower, upper in torque_ranges]
        clipped = np.asarray([max(-cap, min(cap, float(value))) for value, cap in zip(motor, caps)])
        columns["combined_uncapped_wrench_world_N_Nm"][index] = total
        columns["capped_wrench_world_N_Nm"][index] = capped
        columns["cartesian_force_clipped"][index] = force_size > arm["maximum_cartesian_force"]
        columns["cartesian_torque_clipped"][index] = torque_size > arm["maximum_cartesian_torque"]
        columns["motor_uncapped_torques_Nm"][index] = motor
        columns["motor_torques_Nm"][index] = clipped
        columns["motor_clipped"][index] = [abs(float(value)) > cap for value, cap in zip(motor, caps)]
        feedback["right_command_wrench_N_Nm"][index] = capped
        feedback["right_motor_torques_Nm"][index] = clipped


def inertia_ledger():
    """Nonzero retained robot arithmetic; no model, workpiece, or integration.

    The five-coordinate task selects x/y translation and all rotations from
    six coordinates with diagonal positive mass. Its inverse is solved here
    analytically, not by calling or reproducing the auditor's matrix solver.
    """
    boolean = {"enabled", "inertia_inputs_present", "jacobian_derivatives_present",
               "cartesian_force_clipped", "cartesian_torque_clipped", "motor_clipped"}
    columns = {name: np.zeros((2,) + shape, dtype=bool if name in boolean else float)
               for name, shape in INERTIA_SHAPES.items()}
    dt = .00005
    columns["time"][:] = [dt, 2 * dt]
    columns["command_time_s"][:] = [0., dt]
    columns["retained_native_state_time_s"][:] = [0., 0.]  # Initial, then t-2dt.
    columns["phase_index"][:] = [0, 1]
    labels = ["turn_1", "reset_open_1"]
    for name in ("enabled", "inertia_inputs_present", "jacobian_derivatives_present"):
        columns[name][:] = [True, False]
    columns["arm_jacobian_position"][:, :, :3] = np.eye(3)
    columns["arm_jacobian_rotation"][:, :, 3:] = np.eye(3)
    qdot = np.array([.2, -.3, .4, .5, -.6, .7])
    columns["retained_arm_velocity_rad_s"][:] = qdot
    columns["postintegration_arm_velocity_rad_s"][:] = [qdot + .01, qdot - .02]
    columns["arm_bias_torque_Nm"][:] = [.10, -.20, .30, -.40, .50, -.60]
    columns["native_drag_torque_Nm"][:] = [.02, .03, -.01, .04, -.05, .06]
    columns["pd_wrench_world_N_Nm"][:] = [[6.9, 3., 1., 1., .2, .3], [2., -1., .5, .4, -.2, .1]]

    mass_diagonal = np.array([2., 3., 4., 5., 6., 7.])
    columns["mass_aa"][0] = np.diag(mass_diagonal)
    columns["transverse_basis_world"][0] = [[1., 0.], [0., 1.], [0., 0.]]
    columns["hybrid_jacobian5"][0] = np.eye(6)[[0, 1, 3, 4, 5]]
    mobility_diagonal = np.array([1/2, 1/3, 1/5, 1/6, 1/7])
    columns["mobility5"][0] = np.diag(mobility_diagonal)
    columns["mass_condition"][0] = 7 / 2
    columns["mobility_condition"][0] = 7 / 2
    columns["minimum_mobility_eigenvalue"][0] = 1 / 7
    columns["arm_jacobian_position_derivative"][0, :, :3] = np.diag([.1, .2, .3])
    columns["arm_jacobian_rotation_derivative"][0, :, 3:] = np.diag([.4, .5, .6])
    columns["projected_world_jdot_qdot5"][0] = [.02, -.06, .2, -.3, .42]
    # omega=2 about Z and alpha=.5 about Z; the independent orbital
    # derivative of lever (.03,.04,.02) is (.14,.145,0).
    columns["scheduled_lever_world_m"][0] = [.03, .04, .02]
    columns["scheduled_angular_velocity_world_rad_s"][0] = [0., 0., 2.]
    columns["requested_site_acceleration_world_m_s2"][0] = [.14, .145, 0.]
    columns["requested_angular_acceleration_world_rad_s2"][0] = [0., 0., .5]
    columns["desired_world_acceleration5"][0] = [.14, .145, 0., 0., .5]
    # residual=(.12,.205,-.2,.3,.08), so multiplying by the selected
    # masses gives wrench5=(.24,.615,-1,1.8,.56). Axial force stays zero.
    columns["ff_wrench_world_N_Nm"][0] = [.24, .615, 0., -1., 1.8, .56]
    feedback = {
        "time": columns["time"].copy(),
        "phase_index": columns["phase_index"].copy(),
        "right_axial_float_active": np.array([True, False]),
        "right_command_wrench_N_Nm": np.zeros((2, 6)),
        "right_motor_torques_Nm": np.zeros((2, 6)),
        "desired_independent_angular_acceleration_rad_s2": np.array([.5, 0.]),
        "desired_independent_angular_speed_rad_s": np.array([2., 4.]),
    }
    arm = {"maximum_cartesian_force": 8., "maximum_cartesian_torque": 2.}
    torque_ranges = np.array([[-4., 5.], [-3., 6.], [-5., 7.], [-.6, 1.], [-1.5, 2.], [-.4, .7]])
    remap_fixture_wrenches(columns, feedback, arm, torque_ranges)
    return columns, feedback, labels, dt, arm, torque_ranges


def test_retained_five_dimensional_robot_solve_and_disabled_open_row_are_verified():
    columns, feedback, labels, dt, arm, ranges = inertia_ledger()
    assert np.count_nonzero(columns["mass_aa"][0]) == 6
    assert np.count_nonzero(columns["hybrid_jacobian5"][0]) == 5
    assert np.all(columns["projected_world_jdot_qdot5"][0] != 0.)
    assert np.linalg.norm(columns["pd_wrench_world_N_Nm"][0, :3]) < 8.
    assert np.linalg.norm(columns["combined_uncapped_wrench_world_N_Nm"][0, :3]) > 8.
    assert np.linalg.norm(columns["pd_wrench_world_N_Nm"][0, 3:]) < 2.
    assert np.linalg.norm(columns["combined_uncapped_wrench_world_N_Nm"][0, 3:]) > 2.
    assert columns["motor_clipped"].any()
    for value in [*columns.values(), *feedback.values()]:
        value.setflags(write=False)
    result = inertia_rows(columns, feedback, labels, dt, arm, ranges)
    assert result["passed"] and result["observed"]
    assert result["original_native_commands"] == 2
    assert result["enabled_closed_native_commands"] == 1
    assert result["disabled_native_commands"] == 1
    assert result["maximum_absolute_axial_ff_force_N"] == 0.


@pytest.mark.parametrize("invalid", ["enabled_open", "closed_without_axial_float"])
def test_inertia_enable_mode_cannot_escape_closed_axial_force_float(invalid):
    values = inertia_ledger()
    columns, feedback = values[:2]
    if invalid == "enabled_open":
        columns["enabled"][1] = True
    else:
        feedback["right_axial_float_active"][0] = False
    with pytest.raises(ValueError, match="only in declared CLOSED axial-float"):
        inertia_rows(*values)


def test_disabled_open_command_cannot_be_omitted_from_original_inertia_history():
    values = inertia_ledger()
    columns = {name: value[:1].copy() for name, value in values[0].items()}
    with pytest.raises(ValueError, match="Invalid original inertia command column"):
        inertia_rows(columns, *values[1:])


@pytest.mark.parametrize("column", ["arm_jacobian_position_derivative", "mass_aa",
                                    "requested_site_acceleration_world_m_s2", "ff_wrench_world_N_Nm"])
def test_disabled_open_rows_must_clear_every_absent_inverse_derivative_or_ff_input(column):
    values = inertia_ledger()
    values[0][column][1].flat[0] = .1
    with pytest.raises(ValueError, match="explicit zero absent-input placeholders"):
        inertia_rows(*values)


@pytest.mark.parametrize("column,index", [("inertia_inputs_present", 0),
                                           ("jacobian_derivatives_present", 0),
                                           ("inertia_inputs_present", 1)])
def test_inertia_input_presence_must_match_actual_enabled_closed_rows(column, index):
    values = inertia_ledger()
    values[0][column][index] = not values[0][column][index]
    with pytest.raises(ValueError, match="presence flags differ"):
        inertia_rows(*values)


@pytest.mark.parametrize("column", ["time", "command_time_s", "retained_native_state_time_s"])
def test_inertia_input_command_and_original_force_clocks_cannot_be_shifted(column):
    values = inertia_ledger()
    values[0][column][1] += values[3]
    with pytest.raises(ValueError, match="clocks are inconsistent|exactly every original force tick"):
        inertia_rows(*values)


def test_hybrid_inertia_cannot_add_axial_ff_even_if_caps_and_motors_are_self_consistent():
    values = inertia_ledger()
    columns, feedback, _, _, arm, ranges = values
    columns["ff_wrench_world_N_Nm"][0, 2] = .2
    remap_fixture_wrenches(columns, feedback, arm, ranges)
    with pytest.raises(ValueError, match="not introduce axial acceleration force"):
        inertia_rows(*values)


def test_incorrect_five_dimensional_solve_cannot_hide_behind_self_consistent_motor_commands():
    values = inertia_ledger()
    columns, feedback, _, _, arm, ranges = values
    columns["ff_wrench_world_N_Nm"][0, 3] += .2
    remap_fixture_wrenches(columns, feedback, arm, ranges)
    with pytest.raises(ValueError, match="does not solve the declared five-dimensional task"):
        inertia_rows(*values)


def test_inertia_jdot_must_use_retained_velocity_rather_than_saved_postintegration_velocity():
    values = inertia_ledger()
    columns = values[0]
    post = columns["postintegration_arm_velocity_rad_s"][0]
    # The recorded retained input remains correct; only the alleged Jdot
    # product is wrongly evaluated on the newer saved postintegration state.
    columns["projected_world_jdot_qdot5"][0] = [
        .1 * post[0], .2 * post[1], .4 * post[3], .5 * post[4], .6 * post[5],
    ]
    with pytest.raises(ValueError, match="Jdot product must use coherent retained velocity"):
        inertia_rows(*values)


def test_retained_velocity_history_cannot_be_replaced_with_the_newer_postintegration_state():
    values = inertia_ledger()
    values[0]["retained_arm_velocity_rad_s"][1] = values[0]["postintegration_arm_velocity_rad_s"][0]
    with pytest.raises(ValueError, match="Retained robot velocity differs from the original prior pre-solve state history"):
        inertia_rows(*values)


def test_pd_only_clipping_cannot_leave_total_pd_plus_ff_outside_the_cartesian_caps():
    values = inertia_ledger()
    columns = values[0]
    # Both PD blocks are already below their caps. Clipping PD alone and
    # then adding FF therefore leaves the combined demand unbounded.
    columns["capped_wrench_world_N_Nm"][0] = columns["combined_uncapped_wrench_world_N_Nm"][0]
    with pytest.raises(ValueError, match="PD plus FF must share the original Cartesian caps"):
        inertia_rows(*values)


def test_native_bias_torque_cannot_be_added_twice_in_the_recorded_motor_map():
    values = inertia_ledger()
    values[0]["motor_uncapped_torques_Nm"][0] += values[0]["arm_bias_torque_Nm"][0]
    with pytest.raises(ValueError, match="bias and drag exactly once"):
        inertia_rows(*values)


@pytest.mark.parametrize("field", ["right_command_wrench_N_Nm", "right_motor_torques_Nm"])
def test_correct_inertia_arithmetic_must_still_match_original_executed_native_commands(field):
    values = inertia_ledger()
    values[1][field][0, 0] += .01
    with pytest.raises(ValueError, match="differ[s]? from original executed native command"):
        inertia_rows(*values)


@pytest.mark.parametrize("invalid", ["shape", "enabled_flag", "motor_flag", "nonfinite"])
def test_original_inertia_schema_rejects_malformed_shapes_flags_and_nonfinite_values(invalid):
    values = inertia_ledger()
    columns = values[0]
    if invalid == "shape":
        columns["mass_aa"] = np.zeros((2, 5, 5))
    elif invalid == "enabled_flag":
        columns["enabled"] = np.array([2., 0.])
    elif invalid == "motor_flag":
        columns["motor_clipped"] = columns["motor_clipped"].astype(float)
        columns["motor_clipped"][0, 0] = 2.
    else:
        columns["arm_bias_torque_Nm"][0, 0] = np.nan
    with pytest.raises(ValueError, match="must be boolean|Invalid original inertia command column"):
        inertia_rows(*values)


def four_row_inertia_ledger():
    columns, feedback, labels, dt, arm, ranges = inertia_ledger()
    columns = {name: np.concatenate((value, value[[0, 0]]), axis=0) for name, value in columns.items()}
    feedback = {name: np.concatenate((value, value[[0, 0]]), axis=0) for name, value in feedback.items()}
    columns["time"][:] = np.arange(1, 5) * dt
    feedback["time"][:] = columns["time"]
    columns["command_time_s"][:] = np.arange(4) * dt
    columns["retained_native_state_time_s"][:] = [0., 0., dt, 2 * dt]
    initial = columns["retained_arm_velocity_rad_s"][0].copy()
    columns["postintegration_arm_velocity_rad_s"][:] = initial + np.array([.01, -.02, .03, -.04])[:, None]
    columns["retained_arm_velocity_rad_s"][2:] = columns["postintegration_arm_velocity_rad_s"][:2]
    selected_mass = np.array([2., 3., 5., 6., 7.])
    for index in (2, 3):
        qdot = columns["retained_arm_velocity_rad_s"][index]
        product = np.array([.1*qdot[0], .2*qdot[1], .4*qdot[3], .5*qdot[4], .6*qdot[5]])
        columns["projected_world_jdot_qdot5"][index] = product
        wrench5 = selected_mass * (np.array([.14, .145, 0., 0., .5]) - product)
        columns["ff_wrench_world_N_Nm"][index] = [wrench5[0], wrench5[1], 0., *wrench5[2:]]
    remap_fixture_wrenches(columns, feedback, arm, ranges)
    return columns, feedback, labels, dt, arm, ranges


def test_four_row_nonconstant_velocity_history_preserves_the_two_tick_retained_ff_inputs():
    values = four_row_inertia_ledger()
    columns = values[0]
    np.testing.assert_array_equal(columns["retained_arm_velocity_rad_s"][2:], columns["postintegration_arm_velocity_rad_s"][:-2])
    assert not np.array_equal(columns["ff_wrench_world_N_Nm"][0], columns["ff_wrench_world_N_Nm"][2])
    assert not np.array_equal(columns["ff_wrench_world_N_Nm"][2], columns["ff_wrench_world_N_Nm"][3])
    result = inertia_rows(*values)
    assert result["original_native_commands"] == 4
    assert result["enabled_closed_native_commands"] == 3
    assert result["disabled_native_commands"] == 1


@pytest.mark.parametrize("row,wrong_post", [(2, 1), (3, 0)])
def test_correct_timestamps_cannot_hide_a_wrong_two_tick_retained_velocity_source(row, wrong_post):
    values = four_row_inertia_ledger()
    values[0]["retained_arm_velocity_rad_s"][row] = values[0]["postintegration_arm_velocity_rad_s"][wrong_post]
    with pytest.raises(ValueError, match="Retained robot velocity differs from the original prior pre-solve state history"):
        inertia_rows(*values)


def quintic_schedule(angle0, angle1, speed, count, dt):
    duration = (15 / 8) * abs(angle1 - angle0) / speed
    total_steps = round(duration / dt)
    u = np.arange(1, count + 1) / total_steps
    delta = angle1 - angle0
    # Independent expanded minimum-jerk position polynomial and its three
    # analytic derivatives, evaluated on the declared native step grid.
    return (
        angle0 + delta * (6*u**5 - 15*u**4 + 10*u**3),
        delta * (30*u**4 - 60*u**3 + 30*u**2) / duration,
        delta * (120*u**3 - 180*u**2 + 60*u) / duration**2,
        delta * (360*u**2 - 360*u + 60) / duration**3,
    )


def replay_fixture_search_reports(columns, table, labels, metadata, dt):
    """Record only genuine pure observer reports on the synthetic raw inputs."""
    names = np.asarray(labels)[columns["phase_index"].astype(int)]
    observer = CrestWindow(.0014, 1.)
    table_window = TableWindow(.1, 1.)
    reports = {}
    for index, time in enumerate(columns["time"]):
        table_window.observe(table["table_wrench_world_at_block_origin_N_Nm"][index, 2], 0., dt, True)
        if names[index] not in {"reverse_seat_1", "stop_reverse_seat_1"}:
            continue
        reports[index] = observer.observe(float(time), dt, float(table["bolt_base_insertion_m"][index]),
            float(columns["relative_bolt_axial_velocity_m_per_s"][index]), 1.,
            actual_relative_angular_speed_rad_per_s=max(float(columns["relative_bolt_angular_speed_rad_per_s"][index]),
                                                       float(columns["relative_hand_angular_speed_rad_per_s"][index])),
            valid=bool(columns["all_hard_guards_held"][index] and table_window.ready),
            physically_stopped=bool(names[index] == "stop_reverse_seat_1" and not columns["robot_yaw_brake_active"][index]))
    return reports


def search_ledger():
    dt = .001
    transfer_count, reverse_count, stop_count = 200, 240, 249
    labels = ["transfer_bolt_weight", "reverse_seat_1", "stop_reverse_seat_1", "start_thread_1"]
    phase = np.repeat(np.arange(4), [transfer_count, reverse_count, stop_count, 1])
    count = len(phase)
    columns = open_ledger(count, dt)
    columns["phase_index"] = phase
    columns["fully_open_unassisted"][:] = 0.
    columns["open_observation_valid"][:] = 0.
    columns["all_hard_guards_held"] = np.ones(count, dtype=bool)
    columns["thread_summed_normal_force_N"] = np.ones(count)
    columns["robot_yaw_brake_active"] = np.zeros(count, dtype=bool)
    clock_fields = ("desired_independent_clock_rad", "desired_independent_angular_speed_rad_s",
                    "desired_independent_angular_acceleration_rad_s2", "desired_independent_angular_jerk_rad_s3")
    for name in clock_fields:
        columns[name] = np.zeros(count)
    reverse = np.flatnonzero(phase == 1)
    stop = np.flatnonzero(phase == 2)
    reverse_curve = quintic_schedule(0., -2.7, 2., reverse_count, dt)
    for field, value in zip(clock_fields, reverse_curve):
        columns[field][reverse] = value
    theta0, w0, a0 = (float(columns[field][reverse[-1]]) for field in clock_fields[:3])
    duration = .150
    scaled = duration * a0
    s = np.minimum(np.arange(1, stop_count + 1) * dt / duration, 1.)
    # Integrate the cubic angular velocity matching the original theta/omega/
    # alpha jet and ending with zero omega/alpha. Jerk is its next derivative.
    quadratic, cubic = -3*w0 - 2*scaled, 2*w0 + scaled
    stop_curve = (
        theta0 + duration*(w0*s + scaled*s*s/2 + quadratic*s**3/3 + cubic*s**4/4),
        w0 + scaled*s + quadratic*s*s + cubic*s**3,
        (scaled + 2*quadratic*s + 3*cubic*s*s) / duration,
        (2*quadratic + 6*cubic*s) / duration**2,
    )
    for field, value in zip(clock_fields, stop_curve):
        columns[field][stop] = value
    finished = np.arange(1, stop_count + 1) * dt >= duration
    for field in clock_fields[1:]:
        columns[field][stop[finished]] = 0.
    columns["robot_yaw_brake_active"][stop] = np.arange(1, stop_count + 1) * dt < duration - 1e-12
    forward = np.flatnonzero(phase == 3)
    for field, value in zip(clock_fields, quintic_schedule(float(columns[clock_fields[0]][stop[-1]]), 1., 1., 1, dt)):
        columns[field][forward] = value
    speed = np.abs(columns[clock_fields[1]])
    columns["relative_bolt_angular_speed_rad_per_s"][:] = speed
    columns["relative_hand_angular_speed_rad_per_s"][:] = speed
    base = np.full(count, .0014)
    # The measured body withdraws to a shallower crest, then returns. Its
    # final 60um return qualifies while still 40um above the transfer depth;
    # a comparison only to the transfer depth would incorrectly miss it.
    base[reverse[:120]] = np.linspace(.0014, .0013, 120)
    base[reverse[120:-1]] = .0013 + np.linspace(0., 49e-6, reverse_count - 121)
    base[reverse[-1]:] = .00136
    columns["relative_bolt_axial_velocity_m_per_s"][:] = np.r_[0., np.diff(base) / dt]
    columns["weight_observation_valid"][:] = ((speed <= .01)
        & (np.abs(columns["relative_bolt_axial_velocity_m_per_s"]) <= .0002))
    columns["weight_window_ready"] = weight_windows(columns, dt, 1.)["ready"]
    table = {
        "bolt_base_insertion_m": base,
        "table_wrench_world_at_block_origin_N_Nm": np.tile([0., 0., 1., 0., 0., 0.], (count, 1)),
        "left_hand_wrench_world_at_block_origin_N_Nm": np.zeros((count, 6)),
        "unexpected_world_support_count": np.zeros(count),
        "block_position_m": np.tile([0., 0., .1], (count, 1)),
    }
    metadata = {
        "scene_config": {"block_position": [0., 0., .1]},
        "control_config": {"settled_table_window_s": .1, "minimum_table_weight_fraction": .9,
            "maximum_hand_upward_weight_fraction": .1, "minimum_loaded_table_duty": .99,
            "maximum_block_lift_m": .0005, "reverse_brake_duration_s": duration,
            "maximum_search_stop_s": .5, "maximum_reverse_angle_rad": 2.7,
            "reverse_angular_speed_rad_s": 2., "first_forward_clock_rad": 1.,
            "starting_angular_speed_rad_s": 1., "arm": {"angular_speed_rad_s": 2.}},
        "robot_yaw_brake": {"duration_s": duration},
        "maximum_phase_plan": [[label, .650 if label == "stop_reverse_seat_1" else 3.] for label in labels],
        "physical_motion_events": [{"event": "Actual post-ramp settled seat reference",
            "time_s": float(columns["time"][transfer_count-1]), "base_z_m": .0014, "desired_clock_rad": 0.}],
    }
    reports = replay_fixture_search_reports(columns, table, labels, metadata, dt)
    reverse_last, stop_last = reverse[-1], stop[-1]
    assert reports[reverse_last]["stop_requested_from_measured_return"]
    assert not reports[reverse_last]["confirmed_search_direction_event"]
    assert not reports[stop_last-1]["confirmed_search_direction_event"]
    assert reports[stop_last]["confirmed_search_direction_event"]
    assert columns["weight_window_ready"][stop_last]
    for index, label in ((reverse_last, "reverse_seat_1"), (stop_last, "stop_reverse_seat_1")):
        metadata["physical_motion_events"].append({"event": "Live physical phase readiness consumed",
            "phase": label, "time_s": float(columns["time"][index]),
            "seat_direction_event": reports[index]})
    metadata["physical_motion_events"].append({
        "event": "Measured crest return requests C2 CLOSED robot-yaw braking",
        "time_s": float(columns["time"][reverse_last]), "force_time_s": float(columns["time"][reverse_last] - dt),
        "duration_s": duration, "initial_theta_rad": theta0, "initial_omega_rad_s": w0,
        "initial_alpha_rad_s2": a0, "terminal_theta_rad": theta0 + duration*(w0/2 + scaled/12),
    })
    return columns, table, labels, metadata, 1., 1., dt, CrestWindow, TableWindow


def test_actual_crest_return_c2_brake_and_first_quiet_event_validate_on_source_bound_inputs():
    values = search_ledger()
    result = search_events_audit(*values)
    assert result["passed"] and result["observed"]
    assert result["consumed_direction_requests"] == 1
    assert result["consumed_stopped_events"] == 1
    assert result["source_bound_observer_replayed_native_steps"] == 489
    assert result["c2_original_braking_native_steps"] == 149


@pytest.mark.parametrize("field", ["desired_independent_angular_speed_rad_s",
    "desired_independent_angular_acceleration_rad_s2", "desired_independent_angular_jerk_rad_s3",
    "robot_yaw_brake_active"])
def test_c2_brake_schedule_and_active_interval_cannot_be_relabelled(field):
    values = search_ledger()
    stop = np.flatnonzero(values[0]["phase_index"] == 2)
    if field == "robot_yaw_brake_active":
        values[0][field][stop[50]] = False
    else:
        values[0][field][stop[50]] += .01
    with pytest.raises(ValueError):
        search_events_audit(*values)


@pytest.mark.parametrize("phase", [1, 3])
def test_reverse_and_first_forward_jets_must_follow_the_declared_independent_quintic(phase):
    values = search_ledger()
    index = np.flatnonzero(values[0]["phase_index"] == phase)[0]
    values[0]["desired_independent_angular_jerk_rad_s3"][index] += .01
    with pytest.raises(ValueError, match="declared independent quintic"):
        search_events_audit(*values)


@pytest.mark.parametrize("malformed", ["earlier_reference", "wrong_reference_depth", "phase_budget", "forward_without_stop"])
def test_original_search_boundaries_and_finite_stop_budget_cannot_be_fabricated(malformed):
    values = search_ledger()
    metadata = values[3]
    if malformed == "earlier_reference":
        metadata["physical_motion_events"][0]["time_s"] -= values[6]
    elif malformed == "wrong_reference_depth":
        metadata["physical_motion_events"][0]["base_z_m"] += 1e-6
    elif malformed == "phase_budget":
        metadata["maximum_phase_plan"][2][1] -= .01
    else:
        metadata["physical_motion_events"] = [event for event in metadata["physical_motion_events"]
            if event.get("phase") != "stop_reverse_seat_1"]
    with pytest.raises(ValueError):
        search_events_audit(*values)


@pytest.mark.parametrize("invalid", ["lost_return", "hard_guard_gap"])
def test_one_invalid_original_stop_solve_resets_the_persistent_return_or_valid_epoch(invalid):
    values = search_ledger()
    columns, table = values[:2]
    stop = np.flatnonzero(columns["phase_index"] == 2)
    if invalid == "lost_return":
        table["bolt_base_insertion_m"][stop[180]] = .00134
    else:
        columns["all_hard_guards_held"][stop[180]] = False
    with pytest.raises(ValueError, match="source-bound original pure observer replay"):
        search_events_audit(*values)


@pytest.mark.parametrize("early_or_late", ["early", "late"])
def test_stopped_event_must_consume_the_first_current_ready_native_solve(early_or_late):
    values = search_ledger()
    columns, table, labels, metadata = values[:4]
    event = next(event for event in metadata["physical_motion_events"] if event.get("phase") == "stop_reverse_seat_1")
    last = np.flatnonzero(columns["phase_index"] == 2)[-1]
    if early_or_late == "early":
        selected = last - 1
    else:
        selected = last + 1
        columns["phase_index"][selected] = 2
        for field in ("desired_independent_clock_rad", "desired_independent_angular_speed_rad_s",
                      "desired_independent_angular_acceleration_rad_s2", "desired_independent_angular_jerk_rad_s3"):
            columns[field][selected] = columns[field][last]
    reports = replay_fixture_search_reports(columns, table, labels, metadata, values[6])
    event["time_s"] = float(columns["time"][selected])
    event["seat_direction_event"] = reports[selected]
    with pytest.raises(ValueError, match="first current observer/weight/both-quiet"):
        search_events_audit(*values)


def test_ready_weight_and_actual_quiet_during_active_braking_cannot_qualify_a_stopped_event():
    values = search_ledger()
    columns, table, labels, metadata = values[:4]
    stop = np.flatnonzero(columns["phase_index"] == 2)
    columns["relative_bolt_angular_speed_rad_per_s"][stop] = 0.
    columns["relative_hand_angular_speed_rad_per_s"][stop] = 0.
    columns["weight_window_ready"][stop] = True
    selected = stop[99]
    assert columns["robot_yaw_brake_active"][selected]
    reports = replay_fixture_search_reports(columns, table, labels, metadata, values[6])
    assert not reports[selected]["confirmed_search_direction_event"]
    event = next(event for event in metadata["physical_motion_events"] if event.get("phase") == "stop_reverse_seat_1")
    event["time_s"] = float(columns["time"][selected])
    event["seat_direction_event"] = reports[selected]
    with pytest.raises(ValueError, match="first current observer/weight/both-quiet"):
        search_events_audit(*values)
