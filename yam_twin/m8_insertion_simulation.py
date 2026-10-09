"""Actual YAM acquisition and thread starting for a separately resting M8 bolt.

The only writes during integration are native arm and finger motor controls.
The free bolt starts on a declared rest. The female thread moves with the free
block held by the left hand. Thread engagement has an axial force command, with
no depth, pitch, or bolt-position servo along the insertion axis.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import inspect
import json
from pathlib import Path
import re
import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from thread_lab.runtime import require_micron_engine
from .kinematics import ArmIK, HOME
from .m8_simulation import YamCartesianController, YamM8ControlConfig, smooth_profile
from .m8_insertion_mechanics import (fully_formed_flank_interval,
                                     contact_is_on_full_flanks, thread_end_bounds)
from .m8_insertion_engagement import LoadedFlankWindow


@dataclass(frozen=True)
class InsertionControlConfig:
    arm: YamM8ControlConfig = YamM8ControlConfig()
    pickup_hover_m: float = .020
    transport_tip_clearance_m: float = .015
    entry_tip_clearance_m: float = .00075
    block_lift_m: float = .004
    net_axial_feed_N: float = .050
    maximum_starting_strokes: int = 8
    qualifying_turns: int = 2
    sample_period_s: float = .005
    full_flank_contact_streak_s: float = .20

    def __post_init__(self):
        values = (self.pickup_hover_m, self.transport_tip_clearance_m,
                  self.entry_tip_clearance_m, self.block_lift_m,
                  self.net_axial_feed_N, self.sample_period_s,
                  self.full_flank_contact_streak_s)
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError("Insertion clearances, load, and sample interval must be positive")
        if self.qualifying_turns < 1 or int(self.qualifying_turns) != self.qualifying_turns:
            raise ValueError("At least one complete lead-qualification turn is required")
        if self.maximum_starting_strokes < 1 or int(self.maximum_starting_strokes) != self.maximum_starting_strokes:
            raise ValueError("Starting stroke count must be a positive integer")


def insertion_phases(config: InsertionControlConfig):
    """Named phases, durations, hand stroke endpoints, jaw endpoints, and float."""
    c = config.arm
    stroke = c.stroke_angle_rad
    turn_duration = 1.875 * stroke / c.angular_speed_rad_s
    reset_duration = 1.875 * stroke / c.reset_speed_rad_s
    phases = [
        ("secure_left", .25, 0., 0., c.open_aperture, c.open_aperture, False),
        ("reach_bolt", .55, 0., 0., c.open_aperture, c.open_aperture, False),
        ("close_bolt", .25, 0., 0., c.open_aperture, c.closed_aperture, False),
        ("settle_bolt", .12, 0., 0., c.closed_aperture, c.closed_aperture, False),
        ("lift_bolt", .65, 0., 0., c.closed_aperture, c.closed_aperture, False),
        ("transport_bolt", .90, 0., 0., c.closed_aperture, c.closed_aperture, False),
        ("align_over_hole", .45, 0., 0., c.closed_aperture, c.closed_aperture, False),
        ("feed_to_entry", .25, 0., 0., c.closed_aperture, c.closed_aperture, True),
        ("start_thread_1", turn_duration, 0., stroke, c.closed_aperture, c.closed_aperture, True),
        ("stop_start_1", .12, stroke, stroke, c.closed_aperture, c.closed_aperture, True),
    ]
    for tag, turn_label, stop_label in [
            (f"search_{i}", f"start_thread_{i}", f"stop_start_{i}")
            for i in range(2, config.maximum_starting_strokes + 1)] + [
            (str(i), f"turn_{i}", f"stop_{i}") for i in range(1, config.qualifying_turns + 1)]:
        phases.extend([
            (f"release_{tag}", .15, stroke, stroke, c.closed_aperture, c.open_aperture, False),
            (f"open_settle_{tag}", .08, stroke, stroke, c.open_aperture, c.open_aperture, False),
            (f"reset_open_{tag}", reset_duration, stroke, 0., c.open_aperture, c.open_aperture, False),
            (f"open_hold_{tag}", .08, 0., 0., c.open_aperture, c.open_aperture, False),
            (f"regrip_{tag}", .25, 0., 0., c.open_aperture, c.closed_aperture, False),
            (f"settle_regrip_{tag}", .12, 0., 0., c.closed_aperture, c.closed_aperture, True),
            (turn_label, turn_duration, 0., stroke, c.closed_aperture, c.closed_aperture, True),
            (stop_label, .12, stroke, stroke, c.closed_aperture, c.closed_aperture, True),
        ])
    return phases


def initialize_insertion_pose(model, data, scene, control):
    """Initialize native arms around unmodified block and separately resting bolt."""
    from .m8_scene import jaw_positions, left_touch_aperture
    from .m8_insertion_scene import initial_left_grasp_position, left_grasp_rotation
    base = scene.base
    for side in ("left", "right"):
        ids = [model.joint(f"{side}_joint{i}").id for i in range(1, 7)]
        data.qpos[model.jnt_qposadr[ids]] = HOME
        aperture = left_touch_aperture(base) if side == "left" else control.arm.open_aperture
        requested = control.arm.left_closed_aperture if side == "left" else aperture
        for finger, q, command in zip(("left", "right"), jaw_positions(aperture, base),
                                      jaw_positions(requested, base)):
            data.qpos[model.joint(f"{side}_{finger}_finger").qposadr[0]] = q
            data.ctrl[model.actuator(f"{side}_grip_{finger}").id] = command
    mujoco.mj_forward(model, data)
    bolt = model.body("male_bolt").id
    bolt_r = data.xmat[bolt].reshape(3, 3).copy()
    pickup = data.xpos[bolt] + bolt_r @ [0., 0., -scene.head_height / 2]
    right_r = bolt_r @ Rotation.from_euler("z", np.pi / 2).as_matrix()
    targets = {"left": (initial_left_grasp_position(scene), left_grasp_rotation(scene)),
               "right": (pickup + [0., 0., control.pickup_hover_m], right_r)}
    for side, (p, r) in targets.items():
        ik = ArmIK(model, side)
        ik.solve(p, r, thorough=True)
        data.qpos[ik.qadr] = ik.q
    mujoco.mj_forward(model, data)
    return targets, pickup, right_r


def _body_contact(model, data, body, other_geoms=None):
    """Contacts for the complete rigid subtree, including welded frame children."""
    count = 0
    world = 0
    normal = 0.
    force = np.zeros(6)
    welded_root = int(model.body_weldid[body])
    for i in range(data.ncon):
        c = data.contact[i]
        g1, g2 = int(c.geom1), int(c.geom2)
        b1, b2 = int(model.geom_bodyid[g1]), int(model.geom_bodyid[g2])
        w1, w2 = int(model.body_weldid[b1]), int(model.body_weldid[b2])
        if w1 != welded_root and w2 != welded_root:
            continue
        other_g = g2 if w1 == welded_root else g1
        other_welded = w2 if w1 == welded_root else w1
        if other_welded == welded_root:
            continue
        world += other_welded == 0
        if other_geoms is None or other_g in other_geoms:
            mujoco.mj_contactForce(model, data, i, force)
            count += 1
            normal += float(force[0])
    return {"contact_count": int(count), "normal_force_N": normal,
            "world_support_contacts": int(world)}


def formed_flank_overlap(relative, relative_r, thread):
    """Shared geometric full-ring length; never used as a position command."""
    return fully_formed_flank_interval(relative, relative_r, thread)["length_m"]


def run_insertion_demo(output="outputs/m8_insertion", *, scene_config=None,
                       control_config=None, maximum_phases=None):
    """Acquire, carry, align, start, and turn using actual bounded robot motors."""
    from .m8_insertion_scene import InsertionConfig, build_model, scene_xml, scene_fingerprint
    scene = scene_config or InsertionConfig()
    control = control_config or InsertionControlConfig()
    runtime = require_micron_engine()
    model = build_model(scene)
    data = mujoco.MjData(model)
    controllers = {s: YamCartesianController(model, data, s, control.arm, scene.base)
                   for s in ("left", "right")}
    initial, pickup, right_initial_r = initialize_insertion_pose(model, data, scene, control)
    bolt_id = model.body("male_bolt").id
    block_id = model.body("fixture_block").id
    hole_id = model.body("female_frame").id
    bolt_dof = int(model.jnt_dofadr[model.joint("male_bolt_free").id])
    block_dof = int(model.jnt_dofadr[model.joint("fixture_block_free").id])
    bolt_geom = model.geom("bolt_thread").id
    female_geom = model.geom("female_thread").id
    head_geom = model.geom("bolt_head").id
    right, left = controllers["right"], controllers["left"]
    hole_initial_r = data.xmat[hole_id].reshape(3, 3).copy()
    hole_initial_p = data.xpos[hole_id].copy()
    hand_relative_r = hole_initial_r.T @ right_initial_r
    left_p0, left_r0 = initial["left"]
    block_p0 = data.xpos[block_id].copy()
    left_grip_p = left_r0.T @ (data.xpos[block_id] - left_p0)
    left_grip_r = left_r0.T @ data.xmat[block_id].reshape(3, 3)
    mass = float(model.body_mass[bolt_id])
    thread = scene.base.thread
    female_half_height = scene.base.block_size[2] / 2
    shaft_and_half_head = thread.bolt_length + scene.head_height / 2
    # Bounds at which every circumferential phase has cleared both lead-ins.
    # The female chamfer reaches the smallest female thread radius after
    # 5H/8 + .360 mm; the male tip bevel is .956 mm. One additional full
    # pitch gives actual formed-flank overlap, rather than cone-only contact.
    thread_H = np.sqrt(3) * thread.pitch / 2
    female_full_profile_chamfer_m = 5 * thread_H / 8 + .000360
    male_tip_chamfer_m = .000956
    full_flank_overlap_m = male_tip_chamfer_m + female_full_profile_chamfer_m + thread.pitch
    pickup_lift_goal = pickup.copy()
    pickup_lift_goal[2] = (hole_initial_p[2] + control.block_lift_m + female_half_height +
                           shaft_and_half_head + control.transport_tip_clearance_m)
    tip_local = np.array([0., 0., thread.bolt_length])
    phases = insertion_phases(control)
    selected = phases if maximum_phases is None else phases[:maximum_phases]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    start_wall = time.perf_counter()
    rows, times, positions, velocities, controls, summaries = [], [], [], [], [], []
    metadata = {
        "description": "Actual YAM picks up a free bolt, carries it over a female M8 block, and starts the thread",
        "scene_config": asdict(scene), "control_config": asdict(control), "runtime": runtime,
        "starts_preengaged": False, "starts_grasp_ready": False,
        "block_starts_left_touching": True, "bolt_starts_on_declared_fixed_rest": True,
        "model_fingerprint": scene_fingerprint(scene),
        "model_xml_sha256": hashlib.sha256(scene_xml(scene).encode()).hexdigest(),
        "controller_sha256": hashlib.sha256("\n".join(inspect.getsource(v) for v in (
            InsertionControlConfig, insertion_phases, initialize_insertion_pose,
            _body_contact, formed_flank_overlap, run_insertion_demo,
            fully_formed_flank_interval, contact_is_on_full_flanks, thread_end_bounds,
            LoadedFlankWindow,
            YamCartesianController, YamM8ControlConfig, smooth_profile)).encode()).hexdigest(),
        "known_bolt_mass_kg": mass,
        "full_flank_overlap_threshold_m": full_flank_overlap_m,
        "male_tip_chamfer_m": male_tip_chamfer_m,
        "female_full_profile_chamfer_bound_m": female_full_profile_chamfer_m,
        "minimum_engagement_contact_normal_force_N": 1e-5,
        "engagement_observer": LoadedFlankWindow.version,
        "engagement_observer_source_sha256": hashlib.sha256(
            Path(inspect.getfile(LoadedFlankWindow)).read_bytes()).hexdigest(),
        "engagement_observer_scope": "Candidate full-flank capture; actual coupled lead and unsupported reset checks establish completed engagement",
        "engagement_window": {"duration_s": control.full_flank_contact_streak_s,
            "minimum_loaded_normal_impulse_Ns": .1*control.net_axial_feed_N*control.full_flank_contact_streak_s,
            "minimum_loaded_duration_s": .0005,
            "maximum_helix_phase_range_m": 150e-6},
        "legacy_engagement_note": "Consecutive positive force for 0.2 s retained as a diagnostic; unilateral contact may have valid resolved gaps",
        "axial_command_note": "Gravity compensation of known bolt weight plus constant net axial feed; no axial motion servo during entry or turns",
        "partial": True,
    }
    sample_steps = max(1, round(control.sample_period_s / model.opt.timestep))
    block_lift = 0.
    peak_depth = peak_left_slip = peak_left_angle = peak_grip_slip = 0.
    peak_alignment_radial = peak_alignment_tilt = 0.
    maximum_support_after_pickup = maximum_left_support = 0
    minimum_joint_margin = np.inf
    minimum_lift_pad_normals = np.full(2, np.inf)
    minimum_loaded_left_pad_normals = np.full(2, np.inf)
    grip_reference = None
    engaged = False
    engagement_streak = 0.
    engagement_time = None
    legacy_engagement_time = None
    engagement_observer = LoadedFlankWindow(duration_s=control.full_flank_contact_streak_s,
        minimum_normal_impulse_Ns=.1*control.net_axial_feed_N*control.full_flank_contact_streak_s)
    last_yaw = float(np.arctan2((hole_initial_r.T @ data.xmat[bolt_id].reshape(3, 3))[1, 0],
                               (hole_initial_r.T @ data.xmat[bolt_id].reshape(3, 3))[0, 0]))
    yaw_total = last_yaw
    metadata["initial_bolt_yaw_rad"] = last_yaw
    unforced = True
    aborted = None
    hole_velocity = np.zeros(6)
    # Persist the exact controller and model texts used for this integration.
    # Later source revisions cannot silently change how a partial trace is read.
    (output / "controller_source.py").write_text(Path(__file__).read_text())
    (output / "engagement_observer_source.py").write_text(Path(inspect.getfile(LoadedFlankWindow)).read_text())
    (output / "scene.xml").write_text(scene_xml(scene))
    search_skip = {1: False}
    for label, duration, angle0, angle1, gap0, gap1, axial_float in selected:
        match = re.search(r"(?:search_|start_thread_|stop_start_)(\d+)$", label)
        if match:
            group = int(match.group(1))
            if group not in search_skip:
                search_skip[group] = engaged
            if search_skip[group]:
                continue
        if label == "turn_1" and not engaged:
            aborted = {"phase": label, "time": float(data.time),
                       "reason": "No full-flank engagement after all allowed physical starting strokes"}
            break
        phase_p0 = right.pose()[0]
        hole_r = data.xmat[hole_id].reshape(3, 3).copy()
        hole_p = data.xpos[hole_id].copy()
        phase_start_depth = float((hole_r.T @ (data.xpos[bolt_id] - hole_p))[2])
        phase_start_yaw = yaw_total
        held_relative_z = float((hole_r.T @ (phase_p0 - hole_p))[2])
        phase_max_hand_contacts = phase_max_support = 0
        phase_max_head_block_contacts = 0
        phase_peak_pad_torque = 0.
        phase_peak_rotation_drift = phase_peak_axial_drift = 0.
        phase_started_engaged = engaged
        steps = round(duration / model.opt.timestep)
        for step in range(steps):
            u = (step + 1) / steps
            blend, derivative = smooth_profile(u)
            hole_p = data.xpos[hole_id].copy()
            hole_r = data.xmat[hole_id].reshape(3, 3).copy()
            down = hole_r[:, 2]
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY,
                                    hole_id, hole_velocity, 0)
            if label in ("secure_left", "reach_bolt", "close_bolt", "settle_bolt"):
                goal = pickup if label != "secure_left" else initial["right"][0]
            elif label == "lift_bolt":
                goal = pickup_lift_goal
            elif label == "transport_bolt":
                goal = hole_p - down * (female_half_height + shaft_and_half_head +
                                       control.transport_tip_clearance_m)
            elif label == "align_over_hole":
                goal = hole_p - down * (female_half_height + shaft_and_half_head +
                                       control.entry_tip_clearance_m)
            else:
                goal = hole_p + down * held_relative_z
            if label in ("secure_left", "reach_bolt", "close_bolt", "settle_bolt", "lift_bolt", "transport_bolt", "align_over_hole"):
                target_p = phase_p0 + blend * (goal - phase_p0)
                target_v = derivative * (goal - phase_p0) / duration
            else:
                target_p = goal
                target_v = hole_velocity[3:] + np.cross(hole_velocity[:3], goal - hole_p)
            theta = angle0 + blend * (angle1 - angle0)
            omega = derivative * (angle1 - angle0) / duration
            target_r = hole_r @ Rotation.from_euler("z", theta).as_matrix() @ hand_relative_r
            feed = control.net_axial_feed_N + mass * float(np.dot(model.opt.gravity, down)) * -1
            right.command(target_p, target_r, gap0 + blend * (gap1 - gap0),
                          linear_velocity=target_v,
                          angular_velocity=hole_velocity[:3] + down * omega,
                          axial_float=axial_float, axis_world=down, axial_feed_N=feed)
            if label == "lift_bolt":
                block_lift = control.block_lift_m * blend
                lift_velocity = control.block_lift_m * derivative / duration
            else:
                lift_velocity = 0.
            left.command(left_p0 + [0., 0., block_lift], left_r0,
                         control.arm.left_closed_aperture,
                         linear_velocity=[0., 0., lift_velocity])
            mujoco.mj_step(model, data)
            hole_p = data.xpos[hole_id].copy()
            hole_r = data.xmat[hole_id].reshape(3, 3).copy()
            down = hole_r[:, 2]
            bolt_p = data.xpos[bolt_id].copy()
            bolt_r = data.xmat[bolt_id].reshape(3, 3).copy()
            relative = hole_r.T @ (bolt_p - hole_p)
            relative_r = hole_r.T @ bolt_r
            yaw = float(np.arctan2(relative_r[1, 0], relative_r[0, 0]))
            yaw_total += float((yaw - last_yaw + np.pi) % (2*np.pi) - np.pi)
            last_yaw = yaw
            phase_peak_rotation_drift = max(phase_peak_rotation_drift, abs(yaw_total-phase_start_yaw))
            phase_peak_axial_drift = max(phase_peak_axial_drift, abs(float(relative[2])-phase_start_depth))
            overlap = (min(relative[2] + thread.bolt_length * relative_r[2,2], female_half_height)
                       - max(relative[2], -female_half_height))
            formed_overlap = formed_flank_overlap(relative, relative_r, thread)
            radial = float(np.linalg.norm(relative[:2]))
            tilt = float(np.arccos(np.clip(relative_r[2, 2], -1., 1.)))
            thread_depth = 0.
            thread_count = 0
            loaded_formed_contacts = 0
            loaded_formed_normal_force = 0.
            head_block_contacts = 0
            thread_force = np.zeros(6)
            for contact_index, c in enumerate(data.contact):
                c_geoms = {int(c.geom1), int(c.geom2)}
                if head_geom in c_geoms and any(int(model.body_weldid[
                        int(model.geom_bodyid[g])]) == block_id for g in c_geoms):
                    head_block_contacts += 1
                if {int(c.geom1), int(c.geom2)} == {bolt_geom, female_geom}:
                    thread_count += 1
                    thread_depth = max(thread_depth, -float(c.dist))
                    if contact_is_on_full_flanks(hole_r.T @ (c.pos-hole_p),
                                               bolt_r.T @ (c.pos-bolt_p), thread):
                        mujoco.mj_contactForce(model, data, contact_index, thread_force)
                        if float(thread_force[0]) > 1e-5:
                            loaded_formed_contacts += 1
                            loaded_formed_normal_force += float(thread_force[0])
            peak_depth = max(peak_depth, thread_depth)
            phase_max_head_block_contacts = max(phase_max_head_block_contacts, head_block_contacts)
            # Initial cone contact can stall near 1.4 mm without matching the
            # groove. Exclude both entire chamfers plus require a formed pitch.
            engagement_streak = (engagement_streak + model.opt.timestep
                                 if formed_overlap > thread.pitch and loaded_formed_contacts else 0.)
            if legacy_engagement_time is None and engagement_streak >= control.full_flank_contact_streak_s:
                legacy_engagement_time = float(data.time)
            left_p, left_r = left.pose()
            left_slip = float(np.linalg.norm(left_r.T @ (data.xpos[block_id] - left_p) - left_grip_p))
            left_angle = float(np.linalg.norm(Rotation.from_matrix(
                left_r.T @ data.xmat[block_id].reshape(3, 3) @ left_grip_r.T).as_rotvec()))
            peak_left_slip = max(peak_left_slip, left_slip)
            peak_left_angle = max(peak_left_angle, left_angle)
            hand_p, hand_r = right.pose()
            bolt_hand_p = hand_r.T @ (bolt_p + bolt_r @ [0., 0., -scene.head_height/2] - hand_p)
            bolt_hand_r = hand_r.T @ bolt_r
            if label == "lift_bolt" and grip_reference is None:
                grip_reference = (bolt_hand_p.copy(), bolt_hand_r.copy())
            grip_slip = (float(np.linalg.norm(bolt_hand_p - grip_reference[0]))
                         if grip_reference is not None and gap1 == control.arm.closed_aperture else 0.)
            peak_grip_slip = max(peak_grip_slip, grip_slip)
            support = _body_contact(model, data, bolt_id)["world_support_contacts"]
            left_support = _body_contact(model, data, block_id)["world_support_contacts"]
            maximum_left_support = max(maximum_left_support, left_support)
            phase_max_support = max(phase_max_support, support)
            if label not in ("secure_left", "reach_bolt", "close_bolt", "settle_bolt", "lift_bolt"):
                maximum_support_after_pickup = max(maximum_support_after_pickup, support)
            hand_contacts = _body_contact(model, data, bolt_id, right.hand_geom_ids)["contact_count"]
            phase_max_hand_contacts = max(phase_max_hand_contacts, hand_contacts)
            unforced = unforced and bool(np.all(data.xfrc_applied[[bolt_id, block_id, hole_id]] == 0)
                and np.all(data.qfrc_applied[bolt_dof:bolt_dof+6] == 0)
                and np.all(data.qfrc_applied[block_dof:block_dof+6] == 0))
            warnings = int(sum(w.number for w in data.warning))
            joint_margin = min(float(np.min(np.minimum(
                data.qpos[c.qpos_indices]-model.jnt_range[c.joint_ids, 0],
                model.jnt_range[c.joint_ids, 1]-data.qpos[c.qpos_indices])))
                for c in controllers.values())
            minimum_joint_margin = min(minimum_joint_margin, joint_margin)
            alignment_guard = label not in ("secure_left", "reach_bolt", "close_bolt", "settle_bolt", "lift_bolt", "transport_bolt", "align_over_hole") and overlap > 0
            if alignment_guard:
                peak_alignment_radial = max(peak_alignment_radial, radial)
                peak_alignment_tilt = max(peak_alignment_tilt, tilt)
            window = engagement_observer.update(float(data.time), model.opt.timestep,
                formed_overlap > thread.pitch and radial <= 150e-6 and tilt <= np.deg2rad(2)
                and left_slip < .001 and left_angle < np.deg2rad(2) and unforced
                and not left_support and not support and not head_block_contacts,
                loaded_formed_normal_force, loaded_formed_contacts,
                float(relative[2])-thread.pitch*yaw_total/(2*np.pi))
            if not engaged and window["ready"]:
                engaged = True
                engagement_time = float(data.time)
            if (thread_depth > 10e-6 or left_slip > .001 or left_angle > np.deg2rad(2)
                    or grip_slip > .001 or left_support or warnings or not unforced
                    or joint_margin < -1e-5
                    or (alignment_guard and (radial > 150e-6 or tilt > np.deg2rad(2)))
                    or not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all()):
                aborted = {"phase": label, "time": float(data.time), "reported_sdf_depth_m": thread_depth,
                           "radial_offset_m": radial, "bolt_tilt_rad": tilt,
                           "block_grip_slip_m": left_slip, "block_grip_rotation_slip_rad": left_angle,
                           "bolt_grip_slip_m": grip_slip, "world_block_support_contacts": left_support,
                           "minimum_native_joint_margin_rad": joint_margin,
                           "warnings": warnings, "external_drive_zero": unforced}
            if step % sample_steps == 0 or step == steps-1 or aborted:
                contact = right.contact_wrench_on("male_bolt")
                left_contact = left.contact_wrench_on("fixture_block")
                if label != "secure_left":
                    minimum_loaded_left_pad_normals = np.minimum(minimum_loaded_left_pad_normals,
                                                                left_contact["pad_normal_force_N"])
                if label in ("transport_bolt", "align_over_hole", "feed_to_entry") or label.startswith(("start_thread_", "turn_")):
                    minimum_lift_pad_normals = np.minimum(minimum_lift_pad_normals,
                                                         contact["pad_normal_force_N"])
                phase_peak_pad_torque = max(phase_peak_pad_torque,
                    abs(float(np.dot(contact["pad_wrench_world"][3:], down))))
                row = {"time": float(data.time), "phase": label,
                       "bolt_base_insertion_m": float(relative[2]), "bolt_yaw_unwrapped_rad": yaw_total,
                       "clockwise_turns": (yaw_total-metadata["initial_bolt_yaw_rad"])/(2*np.pi),
                       "thread_overlap_m": float(overlap), "thread_contacts": thread_count,
                       "loaded_formed_thread_contacts": int(loaded_formed_contacts),
                       "loaded_formed_thread_normal_force_N": loaded_formed_normal_force,
                       "engagement_window": window,
                       "legacy_consecutive_loaded_contact_streak_s": engagement_streak,
                       "head_block_seating_contacts": head_block_contacts,
                       "formed_flank_overlap_m": formed_overlap,
                       "thread_engaged": engaged, "radial_offset_m": radial,
                       "bolt_tilt_rad": tilt, "reported_sdf_depth_m": thread_depth,
                       "bolt_world_position": bolt_p.tolist(), "hole_world_position": hole_p.tolist(),
                       "bolt_tip_world_position": (bolt_p + bolt_r @ tip_local).tolist(),
                       "block_lift_m": float(data.xpos[block_id, 2] - block_p0[2]),
                       "block_grip_slip_m": left_slip, "block_grip_rotation_slip_rad": left_angle,
                       "bolt_grip_slip_m": grip_slip, "bolt_world_support_contacts": support,
                       "block_world_support_contacts": left_support, "external_drive_zero": unforced,
                       "minimum_native_joint_margin_rad": joint_margin,
                       "contact": contact, "left_contact": left_contact,
                       "axial_float": axial_float, "axial_feed_command_N": feed if axial_float else None,
                       "right_position_error_m": right.last_position_error.tolist(),
                       "right_rotation_error_rad": right.last_rotation_error.tolist()}
                rows.append(row); times.append(float(data.time)); positions.append(data.qpos.copy())
                velocities.append(data.qvel.copy()); controls.append(data.ctrl.copy())
            if aborted:
                break
        end_depth = float((hole_r.T @ (data.xpos[bolt_id] - hole_p))[2])
        rotation = yaw_total-phase_start_yaw
        advance = end_depth-phase_start_depth
        summary = {"phase": label, "bolt_insertion_advance_m": advance,
                   "bolt_clockwise_rotation_rad": rotation,
                   "observed_helix_residual_m": advance-thread.pitch*rotation/(2*np.pi),
                   "maximum_hand_bolt_contacts": phase_max_hand_contacts,
                   "maximum_bolt_world_support_contacts": phase_max_support,
                   "maximum_head_block_seating_contacts": phase_max_head_block_contacts,
                   "sampled_peak_pad_contact_torque_Nm": phase_peak_pad_torque,
                   "maximum_absolute_bolt_rotation_from_phase_start_rad": phase_peak_rotation_drift,
                   "maximum_absolute_bolt_axial_motion_from_phase_start_m": phase_peak_axial_drift,
                   "started_engaged": phase_started_engaged, "ended_engaged": engaged,
                   "final_bolt_world_support_contacts": support,
                   "final_thread_overlap_m": float(overlap),
                   "final_formed_flank_overlap_m": formed_overlap}
        summaries.append(summary)
        print(json.dumps(summary), flush=True)
        np.savez_compressed(output / "insertion_trace_partial.npz", time=times, qpos=positions,
            qvel=velocities, controller=controls, info_json=np.asarray(json.dumps(rows)),
            metadata_json=np.asarray(json.dumps(metadata)))
        if aborted:
            break
    turns = [s for s in summaries if s["phase"].startswith("turn_")]
    resets = [s for s in summaries if s["phase"].startswith("reset_open_")]
    expected_resets = sum(s["phase"].startswith(("start_thread_", "turn_")) for s in summaries)-1
    checks = {
        "completed_qualifying_turns": {"passed": len(turns)==control.qualifying_turns},
        "picked_up_free_bolt": {"passed": any(s["phase"]=="transport_bolt" for s in summaries)
            and maximum_support_after_pickup==0 and np.all(minimum_lift_pad_normals>.1),
            "maximum_world_support_after_pickup": maximum_support_after_pickup,
            "sampled_minimum_closed_transport_pad_normals_N": minimum_lift_pad_normals.tolist()},
        "started_previously_separate_threads": {"passed": engagement_time is not None,
                                               "engagement_time_s": engagement_time},
        "observed_metric_lead": {"passed": bool(turns and all(s["started_engaged"] and
            abs(s["observed_helix_residual_m"])<.02*thread.pitch*control.arm.stroke_angle_rad/(2*np.pi) for s in turns))},
        "closed_turn_tracking": {"passed": bool(turns and all(abs(s["bolt_clockwise_rotation_rad"]-
            control.arm.stroke_angle_rad)<.03 for s in turns))},
        "contact_torque_turns_bolt": {"passed": bool(turns and all(s["sampled_peak_pad_contact_torque_Nm"]>1e-6 for s in turns))},
        "open_reset_contact_decoupling": {"passed": len(resets)==expected_resets and
            all(s["maximum_hand_bolt_contacts"]==0 for s in resets)},
        "passive_self_locking_during_open_reset": {"passed": len(resets)==expected_resets and
            bool([s for s in resets if s["started_engaged"]]) and
            all(s["maximum_absolute_bolt_rotation_from_phase_start_rad"]<.02
                and s["maximum_absolute_bolt_axial_motion_from_phase_start_m"]<10e-6
                and s["maximum_head_block_seating_contacts"]==0
                and s["maximum_bolt_world_support_contacts"]==0 for s in resets if s["started_engaged"]),
            "scope": "All-substep maximum yaw/axial drift only after declared formed-flank engagement; starting-cone resets are reported separately",
            "angular_drift_limit_rad": .02, "axial_drift_limit_m":10e-6},
        "reported_sdf_depth_proxy": {"passed": peak_depth<=10e-6,"observed_peak_m":peak_depth},
        "entry_alignment": {"passed": peak_alignment_radial<=150e-6 and peak_alignment_tilt<=np.deg2rad(2),
                            "peak_radial_m":peak_alignment_radial,"peak_tilt_rad":peak_alignment_tilt},
        "left_grasp_retention": {"passed": peak_left_slip<.001 and peak_left_angle<np.deg2rad(2),
                                  "peak_translation_m":peak_left_slip,"peak_rotation_rad":peak_left_angle},
        "left_pad_contact_retention": {"passed": np.all(minimum_loaded_left_pad_normals>.1),
            "sampled_minimum_loaded_pad_normals_N": minimum_loaded_left_pad_normals.tolist()},
        "left_arm_physically_lifts_block": {"passed": float(data.xpos[block_id,2]-block_p0[2])>.003,
            "observed_lift_m":float(data.xpos[block_id,2]-block_p0[2])},
        "right_bolt_grasp_retention": {"passed": peak_grip_slip<.001,"peak_translation_m":peak_grip_slip},
        "left_block_has_no_world_support": {"passed": maximum_left_support==0},
        "free_objects_have_no_external_drive": {"passed": unforced},
        "native_joint_limits": {"passed": minimum_joint_margin>=-1e-5,
                                "minimum_observed_margin_rad": minimum_joint_margin},
        "no_solver_or_state_abort": {"passed": aborted is None},
    }
    checks = {k:{**v,"passed":bool(v["passed"])} for k,v in checks.items()}
    result = {**metadata,"partial":len(selected)<len(phases),"passed":all(v["passed"] for v in checks.values()),
              "phases":summaries,"acceptance_checks":checks,"aborted":aborted,
              "wall_seconds":time.perf_counter()-start_wall,
              "legacy_continuous_loaded_force_criterion": {"would_have_tagged_engagement":legacy_engagement_time is not None,
                                                           "first_pass_time_s":legacy_engagement_time,
                                                           "required_streak_s":control.full_flank_contact_streak_s},
              "reported_sdf_depth_note":"Native contact dist proxy; not an independent overlap certificate"}
    np.savez_compressed(output / "insertion_trace.npz", time=times, qpos=positions,
        qvel=velocities, controller=controls, info_json=np.asarray(json.dumps(rows)),
        metadata_json=np.asarray(json.dumps(result)))
    (output / "insertion_validation.json").write_text(json.dumps(result,indent=2))
    return result
