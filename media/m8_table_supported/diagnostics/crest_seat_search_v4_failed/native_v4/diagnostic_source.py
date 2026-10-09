"""Native side-bolt pickup with a left-stabilized, table-supported M8 block.

This separate controller preserves the carried-block trajectory. The block is
a free body; its weight is carried by declared native table contacts. Only
bounded arm and finger controls are written during integration.
"""
from __future__ import annotations
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
__package__ = "yam_twin"
from crest_seat_observer_v3 import CrestSeatDropWindowV3
from reverse_brake_v4 import C2ReverseBrake, BrakeSample, _finite_scalar as _brake_finite_scalar
_COLD_CONTEXT = None

def cold_table_mean_unavailable(label, observed_duration_s, required_duration_s):
    if (not np.isfinite((observed_duration_s, required_duration_s)).all()
            or observed_duration_s < 0 or required_duration_s <= 0):
        raise ValueError("Cold-window durations must be finite and physical")
    return bool(label == "cold_window_hold"
                and observed_duration_s+1e-12 < required_duration_s)

from dataclasses import asdict, dataclass
from collections import deque
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
from .m8_insertion_simulation import (InsertionControlConfig, insertion_phases,
    PadLoadHistory, _pad_normal_forces, _relative_axial_velocity,
    EntrySupportWindow, _body_contact, formed_flank_overlap)
from .m8_supported_start import (contact_world_wrench, resolved_thread_wrench_on_bolt,
    native_weight_transfer_sample, BoltWeightTransferWindow, ImpulseSeatDropWindow,
    _finite_vector, _proper_rotation, _rz, closed_head_target, frozen_open_target,
    downward_hold_feed, bounded_phase_gate)


@dataclass(frozen=True)
class SupportedControlConfig(InsertionControlConfig):
    """Reuse contact/arm limits; add explicit table-bearing observations."""

    arm: YamM8ControlConfig = YamM8ControlConfig(left_closed_aperture=.0184,
        closed_aperture=.0184, open_aperture=.030,
        stroke_angle_rad=np.pi, angular_speed_rad_s=2.)
    axial_velocity_damping_Ns_per_m: float = 200.
    maximum_entry_dwell_s: float | None = 3.
    starting_angular_speed_rad_s: float | None = 1.
    maximum_starting_strokes: int = 5
    # A low bore must not leave the shaft between the head-support pins
    # while the arm begins its lateral transfer.
    transport_tip_clearance_m: float = .035
    minimum_tip_rest_clearance_m: float = .010
    # Pick a different real hex-head flat pair to center the available wrist
    # travel. This rotates the robot's grasp frame, never the spawned bolt.
    pickup_grasp_face_offset_rad: float = 2*np.pi/3
    settled_table_window_s: float = .100
    minimum_table_weight_fraction: float = .90
    maximum_hand_upward_weight_fraction: float = .10
    minimum_loaded_table_duty: float = .99
    maximum_block_lift_m: float = .0005
    head_centering_s: float = .15
    head_alignment_s: float = .30
    gravity_feed_ramp_s: float = .50
    gravity_feed_hold_s: float = .50
    maximum_reference_settle_s: float = 1.
    maximum_reverse_angle_rad: float = 2.7
    reverse_angular_speed_rad_s: float = 2.
    first_forward_clock_rad: float = 1.
    maximum_search_stop_s: float = .75
    minimum_open_settle_s: float = .12
    maximum_open_settle_s: float = .75
    regrasp_window_s: float = .10
    left_open_aperture_m: float = .024
    left_downward_hold_N: float = 2.
    left_axial_velocity_damping_Ns_per_m: float = 200.
    left_hold_ramp_s: float = .30

    def __post_init__(self):
        super().__post_init__()
        values = (self.settled_table_window_s, self.minimum_table_weight_fraction,
                  self.maximum_hand_upward_weight_fraction,
                  self.minimum_loaded_table_duty, self.maximum_block_lift_m,
                  self.minimum_tip_rest_clearance_m)
        if not np.isfinite(values).all() or min(values) <= 0:
            raise ValueError("Supported-block thresholds must be finite and positive")
        if any(v > 1. for v in (self.minimum_table_weight_fraction,
                self.maximum_hand_upward_weight_fraction, self.minimum_loaded_table_duty)):
            raise ValueError("Supported-block load fractions must not exceed one")
        face = self.pickup_grasp_face_offset_rad
        if not np.isfinite(face) or not np.isclose(face/(np.pi/3),
                np.rint(face/(np.pi/3)), rtol=0., atol=1e-10):
            raise ValueError("Pickup grasp must select an actual regular-hex flat pair")
        motion = (self.head_centering_s, self.head_alignment_s, self.gravity_feed_ramp_s, self.gravity_feed_hold_s,
            self.maximum_reference_settle_s, self.maximum_reverse_angle_rad,
            self.reverse_angular_speed_rad_s, self.maximum_search_stop_s,
            self.minimum_open_settle_s, self.maximum_open_settle_s, self.regrasp_window_s,
            self.left_open_aperture_m,
            self.left_downward_hold_N, self.left_axial_velocity_damping_Ns_per_m,
            self.left_hold_ramp_s)
        if not np.isfinite(motion).all() or min(motion) <= 0:
            raise ValueError("Physical feedback/search parameters must be positive and finite")
        if (not np.isfinite(self.first_forward_clock_rad)
                or not 0 < self.first_forward_clock_rad <= 1.
                or self.maximum_reverse_angle_rad > 2.7
                or self.minimum_open_settle_s > self.maximum_open_settle_s
                or self.regrasp_window_s > .25
                or not self.arm.left_closed_aperture <= self.left_open_aperture_m <= .040
                or self.left_downward_hold_N > self.arm.maximum_cartesian_force):
            raise ValueError("Physical search must remain within declared checked workspace/force bounds")
        if not np.isclose(self.arm.stroke_angle_rad, np.pi, rtol=0., atol=1e-10):
            raise ValueError("Supported opposite-flat search/qualification requires pi strokes")


class TableLoadWindow:
    """Original solved load window, distinct from contact-candidate geometry."""

    version = "native-supported-block-load-window-v1"

    def __init__(self, duration_s, block_weight_N, *, minimum_table_fraction=.90,
                 maximum_hand_upward_fraction=.10, minimum_loaded_duty=.99):
        if not np.isfinite([duration_s, block_weight_N]).all() or min(duration_s, block_weight_N) <= 0:
            raise ValueError("Table load observation requires positive finite duration and weight")
        fractions = (minimum_table_fraction, maximum_hand_upward_fraction, minimum_loaded_duty)
        if not np.isfinite(fractions).all() or min(fractions) <= 0 or max(fractions) > 1:
            raise ValueError("Table load fractions must be finite in (0, 1]")
        self.duration = float(duration_s)
        self.weight = float(block_weight_N)
        self.minimum_table_fraction = float(minimum_table_fraction)
        self.maximum_hand_fraction = float(maximum_hand_upward_fraction)
        self.minimum_duty = float(minimum_loaded_duty)
        self.samples = deque()
        self.elapsed = self.table_impulse = self.hand_upward_impulse = self.loaded_duration = 0.
        self.last_table_force = 0.
        self.ready = False

    def observe(self, table_upward_force_N, hand_upward_force_N, dt, valid):
        values = [table_upward_force_N, hand_upward_force_N, dt]
        if not np.isfinite(values).all() or dt <= 0:
            raise ValueError("Table observations must be finite and timestep positive")
        self.last_table_force = float(table_upward_force_N)
        if not valid:
            self.samples.clear()
            self.elapsed = self.table_impulse = self.hand_upward_impulse = self.loaded_duration = 0.
            self.ready = False
            return False
        loaded = table_upward_force_N > .1*self.weight
        positive_hand = max(float(hand_upward_force_N), 0.)
        self.samples.append((float(dt), float(table_upward_force_N), positive_hand, loaded))
        self.elapsed += dt
        self.table_impulse += table_upward_force_N*dt
        self.hand_upward_impulse += positive_hand*dt
        self.loaded_duration += loaded*dt
        while self.samples and self.elapsed-self.samples[0][0] >= self.duration:
            old_dt, table, hand, old_loaded = self.samples.popleft()
            self.elapsed -= old_dt
            self.table_impulse -= table*old_dt
            self.hand_upward_impulse -= hand*old_dt
            self.loaded_duration -= old_loaded*old_dt
        self.ready = bool(self.elapsed+1e-12 >= self.duration and loaded
            and self.table_impulse >= self.minimum_table_fraction*self.weight*self.elapsed
            and self.hand_upward_impulse <= self.maximum_hand_fraction*self.weight*self.elapsed
            and self.loaded_duration >= self.minimum_duty*self.elapsed)
        return self.ready

    def report(self):
        return {"observer": self.version, "ready": self.ready,
            "required_window_s": self.duration, "observed_window_s": self.elapsed,
            "block_weight_N": self.weight,
            "minimum_mean_table_weight_fraction": self.minimum_table_fraction,
            "maximum_mean_positive_hand_upward_weight_fraction": self.maximum_hand_fraction,
            "minimum_loaded_table_duty": self.minimum_duty,
            "mean_table_upward_force_N": self.table_impulse/self.elapsed if self.elapsed else None,
            "mean_positive_hand_upward_force_N": self.hand_upward_impulse/self.elapsed if self.elapsed else None,
            "loaded_table_substep_duty": self.loaded_duration/self.elapsed if self.elapsed else 0.,
            "final_table_upward_force_N": self.last_table_force,
            "scope": "No-bolt settled stabilization: original native table force bears block weight; contact candidates alone do not establish support"}


def supported_phases(config):
    """Maximum phase plan; native events shorten bounded acquisition/dwells."""
    c = config.arm
    start = [("settle_table", .20), ("reach_left_block", .90),
             ("close_left_block", .25), ("settle_left_block", .30)]
    phases = [(name, duration, 0., 0., c.open_aperture, c.open_aperture, False)
              for name, duration in start]
    for phase in insertion_phases(config):
        phases.append(phase)
        if phase[0] == "feed_to_entry":
            break
    close = c.closed_aperture
    starting_speed = config.starting_angular_speed_rad_s or c.angular_speed_rad_s
    first_duration = 1.875*(config.maximum_reverse_angle_rad+config.first_forward_clock_rad)/starting_speed
    phases.extend([
        ("transfer_bolt_weight", config.left_hold_ramp_s+max(config.head_centering_s, config.head_alignment_s)+config.gravity_feed_ramp_s+
         config.gravity_feed_hold_s+config.maximum_reference_settle_s, 0., 0., close, close, True),
        ("reverse_seat_1", 1.875*config.maximum_reverse_angle_rad/config.reverse_angular_speed_rad_s,
         0., -config.maximum_reverse_angle_rad, close, close, True),
        ("stop_reverse_seat_1", config.maximum_search_stop_s, 0., 0., close, close, True),
        ("start_thread_1", first_duration, 0., config.first_forward_clock_rad, close, close, True),
        ("stop_start_1", config.maximum_search_stop_s, 0., 0., close, close, True)])
    for tag, turn, stop in [(f"search_{i}", f"start_thread_{i}", f"stop_start_{i}")
            for i in range(2, config.maximum_starting_strokes+1)] + [
            (str(i), f"turn_{i}", f"stop_{i}") for i in range(1, config.qualifying_turns+1)]:
        speed = starting_speed if turn.startswith("start_thread_") else c.angular_speed_rad_s
        phases.extend([
            (f"release_{tag}", .25, 0., 0., close, c.open_aperture, False),
            (f"open_settle_{tag}", config.maximum_open_settle_s, 0., 0., c.open_aperture, c.open_aperture, False),
            (f"reset_open_{tag}", 1.875*np.pi/c.reset_speed_rad_s, 0., -np.pi, c.open_aperture, c.open_aperture, False),
            (f"open_hold_{tag}", .12, 0., 0., c.open_aperture, c.open_aperture, False),
            (f"regrip_{tag}", .25, 0., 0., c.open_aperture, close, False),
            (f"settle_regrip_{tag}", .25, 0., 0., close, close, False),
            (turn, 1.875*np.pi/speed, 0., np.pi, close, close, True),
            (stop, config.maximum_search_stop_s, 0., 0., close, close, True)])
    return phases


def initialize_supported_pose(model, data, scene, control):
    """Initialize arms/jaws only; preserve all free-body poses and velocities."""
    from .m8_scene import jaw_positions
    from .m8_supported_scene import initial_left_grasp_position, left_grasp_rotation
    for side in ("left", "right"):
        ids = [model.joint(f"{side}_joint{i}").id for i in range(1, 7)]
        data.qpos[model.jnt_qposadr[ids]] = HOME
        aperture = control.left_open_aperture_m if side == "left" else control.arm.open_aperture
        for finger, q in zip(("left", "right"), jaw_positions(aperture, scene.base)):
            data.qpos[model.joint(f"{side}_{finger}_finger").qposadr[0]] = q
            data.ctrl[model.actuator(f"{side}_grip_{finger}").id] = q
    mujoco.mj_forward(model, data)
    bolt = model.body("male_bolt").id
    bolt_r = data.xmat[bolt].reshape(3, 3).copy()
    pickup = data.xpos[bolt] + bolt_r @ [0., 0., -scene.head_height/2]
    right_r = bolt_r @ Rotation.from_euler("z",
        np.pi/2+control.pickup_grasp_face_offset_rad).as_matrix()
    left_r = left_grasp_rotation(scene)
    left_p = initial_left_grasp_position(scene)-left_r[:, 2]*control.left_pickup_hover_m
    targets = {"left": (left_p, left_r),
               "right": (pickup+[0., 0., control.pickup_hover_m], right_r)}
    for side, (p, r) in targets.items():
        ik = ArmIK(model, side)
        ik.solve(p, r, thorough=True)
        data.qpos[ik.qadr] = ik.q
    mujoco.mj_forward(model, data)
    return targets, pickup, right_r


def _table_support_state(model, data, body, allowed_support_geoms, *, with_contacts=False):
    """Resolved table force/wrench on the entire free rigid block subtree."""
    root = int(model.body_weldid[body])
    allowed = {int(g) for g in allowed_support_geoms}
    wrench = np.zeros(6)
    normal_upward = 0.
    count = loaded_count = unexpected = 0
    force = np.zeros(6)
    details = []
    for index in range(data.ncon):
        c = data.contact[index]
        g1, g2 = int(c.geom1), int(c.geom2)
        r1, r2 = (int(model.body_weldid[int(model.geom_bodyid[g])]) for g in (g1, g2))
        if r1 == root and r2 == 0:
            other, sign = g2, -1.
        elif r2 == root and r1 == 0:
            other, sign = g1, 1.
        else:
            continue
        if other not in allowed:
            unexpected += 1
            continue
        mujoco.mj_contactForce(model, data, index, force)
        frame = c.frame.reshape(3, 3)
        f_world = sign*(frame.T @ force[:3])
        torque_world = sign*(frame.T @ force[3:])
        wrench[:3] += f_world
        wrench[3:] += torque_world+np.cross(c.pos-data.xpos[body], f_world)
        normal_upward += sign*frame[0, 2]*float(force[0])
        count += 1
        loaded_count += float(force[0]) > .005
        if with_contacts:
            details.append({"table_geom": model.geom(other).name,
            "block_geom": model.geom(g1 if r1 == root else g2).name,
            "signed_distance_m": float(c.dist), "normal_force_N": float(force[0]),
            "geom1": model.geom(g1).name, "geom2": model.geom(g2).name,
            "frame": frame.tolist(), "local_contact_force_N_Nm": force.tolist(),
            "contact_position_world_m": c.pos.tolist(),
            "block_origin_world_m": data.xpos[body].tolist(),
            "signed_force_contribution_on_block_world_N": f_world.tolist()})
    return {"contact_candidates": count, "loaded_contacts": loaded_count,
            "unexpected_world_contact_candidates": unexpected,
            "table_upward_normal_force_N": float(normal_upward),
            "table_upward_force_N": float(wrench[2]),
            "table_wrench_on_block_world": wrench.tolist(), "contacts": details}


def _left_hand_contact_state(model, data, block_id, controller, *, with_contacts=False):
    """Native full-rigid-subtree hand wrench, with optional raw contact rows."""
    root = int(model.body_weldid[block_id])
    total, pad_total = np.zeros(6), np.zeros(6)
    counts = [0, 0]
    normals = {int(g): 0. for g in controller.pad_geom_ids}
    contact_force = np.zeros(6)
    details = []
    for index in range(data.ncon):
        c = data.contact[index]
        g1, g2 = int(c.geom1), int(c.geom2)
        r1, r2 = (int(model.body_weldid[int(model.geom_bodyid[g])]) for g in (g1, g2))
        if r1 == root and g2 in controller.hand_geom_ids:
            sign, hand_geom = -1., g2
        elif r2 == root and g1 in controller.hand_geom_ids:
            sign, hand_geom = 1., g1
        else:
            continue
        mujoco.mj_contactForce(model, data, index, contact_force)
        frame = c.frame.reshape(3, 3)
        force = sign*(frame.T @ contact_force[:3])
        torque = sign*(frame.T @ contact_force[3:])+np.cross(c.pos-data.xpos[block_id], force)
        wrench = np.r_[force, torque]
        total += wrench
        counts[0] += 1
        if hand_geom in normals:
            pad_total += wrench
            normals[hand_geom] += float(contact_force[0])
            counts[1] += 1
        if with_contacts:
            details.append({"geom1": model.geom(g1).name, "geom2": model.geom(g2).name,
                "frame": frame.tolist(), "local_contact_force_N_Nm": contact_force.tolist(),
                "contact_position_world_m": c.pos.tolist(),
                "block_origin_world_m": data.xpos[block_id].tolist(),
                "signed_force_contribution_on_block_world_N": force.tolist()})
    return {"contact_count": counts[0], "pad_contact_count": counts[1],
            "wrench_world": total.tolist(), "pad_wrench_world": pad_total.tolist(),
            "pad_normal_force_N": [normals[model.geom(f"left_m8_pad_{s}").id]
                                    for s in ("left", "right")],
            "jaw_actuator_force": data.actuator_force[controller.finger_ids].tolist(),
            "arm_motor_torques_Nm": controller.last_motor_torques.tolist(),
            "hand_wrench_world": controller.last_wrench.tolist(),
            "native_contact_records": details}


def _unexpected_native_contacts(model, data, block_id, bolt_id, table_geoms,
                                left_pads, right_pads, thread_geoms, *, allow_bolt_rest,
                                head_geom=None):
    """Keep native camera/backing/interarm collisions visible and guarded."""
    failures = []
    for c in data.contact:
        g1, g2 = int(c.geom1), int(c.geom2)
        roots = [int(model.body_weldid[int(model.geom_bodyid[g])]) for g in (g1, g2)]
        if -float(c.dist) <= 1e-6:
            continue
        pairs = ((g1, roots[0], g2, roots[1]), (g2, roots[1], g1, roots[0]))
        intended = ({g1, g2} == set(thread_geoms)
            or any(g in table_geoms and other_root == block_id for g, _, _, other_root in pairs)
            or any(g in left_pads and other_root == block_id for g, _, _, other_root in pairs)
            or any(g in right_pads and other == head_geom for g, _, other, _ in pairs)
            or (allow_bolt_rest and any(root == 0 and other_root == bolt_id
                and model.geom(g).name.startswith("bolt_rest_")
                for g, root, _, other_root in pairs)))
        if not intended:
            failures.append({"geom1": model.geom(g1).name, "geom2": model.geom(g2).name,
                "native_signed_distance_m": float(c.dist),
                "scope": "Native solved contact geometry, distinct from thread SDF depth proxy"})
    return failures


def _declared_bolt_rest_top(model, data):
    """World-Z upper bound of the actual fixed rest boxes and cylinders."""
    tops = {}
    for geom in range(model.ngeom):
        name = model.geom(geom).name
        if not name.startswith("bolt_rest_"):
            continue
        if int(model.body_weldid[int(model.geom_bodyid[geom])]) != 0:
            raise ValueError("The declared bolt rest must be fixed")
        r = data.geom_xmat[geom].reshape(3, 3)
        size = model.geom_size[geom]
        kind = int(model.geom_type[geom])
        if kind == mujoco.mjtGeom.mjGEOM_BOX:
            extent = float(np.dot(np.abs(r[2]), size))
        elif kind == mujoco.mjtGeom.mjGEOM_CYLINDER:
            extent = float(abs(r[2, 2])*size[1]+size[0]*np.sqrt(max(0., 1.-r[2, 2]**2)))
        else:
            raise ValueError("Unsupported native bolt-rest geometry")
        tops[name] = float(data.geom_xpos[geom, 2]+extent)
    if not tops:
        raise ValueError("Native scene lacks its declared bolt rest")
    return {"maximum_world_z_m": max(tops.values()), "geom_world_top_z_m": tops,
            "method": "Exact world-Z bounds of declared native fixed box/cylinder collision geometry"}


def run_supported_demo(output="outputs/m8_supported", *, scene_config=None,
                       control_config=None, maximum_phases=None):
    """Acquire the side bolt, stabilize the block, and turn with native motors."""
    from .m8_supported_scene import (SupportedConfig, supported_config, build_model, scene_xml,
        scene_fingerprint, initial_left_grasp_position, TABLE_SUPPORT_GEOM_NAMES, table_top_height)
    scene = scene_config or supported_config()
    # This flag selects real left acquisition/history, never carried-block motion.
    table_pickup = True
    control = control_config if control_config is not None else SupportedControlConfig()
    runtime = require_micron_engine()
    model = _COLD_CONTEXT["model"]
    data = mujoco.MjData(model)
    controllers = {s: YamCartesianController(model, data, s, control.arm, scene.base)
                   for s in ("left", "right")}
    initial, pickup, right_initial_r = initialize_supported_pose(model, data, scene, control)
    # Declared cold initialization only.  Native contact warm-start/history is
    # deliberately absent; no later workpiece state write is introduced.
    original_block_id = model.body("fixture_block").id
    original_block_p0 = data.xpos[original_block_id].copy()
    original_block_r0 = data.xmat[original_block_id].reshape(3, 3).copy()
    original_hole_r0 = data.xmat[model.body("female_frame").id].reshape(3, 3).copy()
    data.qpos[:] = _COLD_CONTEXT["qpos"]
    data.qvel[:] = _COLD_CONTEXT["qvel"]
    data.ctrl[:] = _COLD_CONTEXT["ctrl"]
    data.time = 0.
    mujoco.mj_forward(model, data)
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
    held_block_p = data.xpos[block_id].copy()
    held_block_r = data.xmat[block_id].reshape(3, 3).copy()
    held_hole_p = held_block_p + held_block_r @ np.asarray(scene.hole_offset)
    held_hole_r = held_block_r @ np.diag([1., -1., -1.])
    hand_relative_r = (held_hole_r if table_pickup else hole_initial_r).T @ right_initial_r
    left_p0, left_r0 = initial["left"]
    block_p0 = data.xpos[block_id].copy()
    left_grip_p = left_r0.T @ (data.xpos[block_id] - left_p0)
    left_grip_r = left_r0.T @ data.xmat[block_id].reshape(3, 3)
    left_pickup_p = initial_left_grasp_position(scene)
    left_hold_p = left_pickup_p.copy()
    left_hold_r = left_r0.copy()
    left_grasp_acquired = False
    left_acquisition_time = None
    left_acquisition_pad_normals = None
    initial_left_block_contacts = _body_contact(model, data, block_id, left.hand_geom_ids)
    initial_right_bolt_contacts = _body_contact(model, data, bolt_id, right.hand_geom_ids)
    initial_block_support = _body_contact(model, data, block_id)["world_support_contacts"]
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
    pickup_lift_goal[2] = (held_hole_p[2]
                           + female_half_height +
                           shaft_and_half_head + control.transport_tip_clearance_m)
    rest_top = _declared_bolt_rest_top(model, data)
    rest_top_z = rest_top["maximum_world_z_m"]
    planned_tip_clearance = pickup_lift_goal[2]-shaft_and_half_head-rest_top_z
    if planned_tip_clearance < control.minimum_tip_rest_clearance_m:
        raise ValueError("Pickup/transfer waypoint does not clear the whole shaft above its native rest")
    tip_local = np.array([0., 0., thread.bolt_length])
    phases = [("cold_window_hold", .5, 0., 0., control.arm.closed_aperture,
               control.arm.closed_aperture, True)] + [
        (label, duration+(_COLD_CONTEXT["brake_duration_s"] if label == "stop_reverse_seat_1" else 0.), *rest)
        for label, duration, *rest in supported_phases(control) if label in {
        "reverse_seat_1", "stop_reverse_seat_1", "start_thread_1", "stop_start_1"}]
    selected = phases if maximum_phases is None else phases[:maximum_phases]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    start_wall = time.perf_counter()
    rows, times, positions, velocities, controls, summaries = [], [], [], [], [], []
    metadata = {
        "description": "Native right pickup and M8 threading into a free table-supported block stabilized by native left pads",
        "supported_task": True,
        "support_scope": "Plain continuous tabletop; bounded running-thread insertion, no through-table bolt travel or head-seating proof",
        "scene_config": asdict(scene), "control_config": asdict(control), "runtime": runtime,
        "starts_preengaged": False, "starts_grasp_ready": False,
        "block_starts_left_touching": False,
        "block_starts_on_table": True,
        "initial_left_block_hand_contacts": initial_left_block_contacts,
        "initial_right_bolt_hand_contacts": initial_right_bolt_contacts,
        "initial_block_world_support_contacts": initial_block_support,
        "left_acquisition_reference_scope": ("Frozen at end of settle_left_block after actual bilateral loaded pad contact"
                                             if table_pickup else "Initial touching grasp pose"),
        "bolt_starts_on_declared_fixed_rest": True,
        "model_fingerprint": scene_fingerprint(scene),
        "model_xml_sha256": hashlib.sha256(scene_xml(scene).encode()).hexdigest(),
        "controller_sha256": hashlib.sha256("\n".join(inspect.getsource(v) for v in (
            PadLoadHistory, _pad_normal_forces, _relative_axial_velocity, EntrySupportWindow,
            InsertionControlConfig, insertion_phases, SupportedControlConfig, TableLoadWindow,
            supported_phases, initialize_supported_pose, _table_support_state,
            _left_hand_contact_state, _unexpected_native_contacts,
            _declared_bolt_rest_top,
            _body_contact, formed_flank_overlap, run_supported_demo,
            fully_formed_flank_interval, contact_is_on_full_flanks, thread_end_bounds,
            contact_world_wrench, resolved_thread_wrench_on_bolt,
            native_weight_transfer_sample, BoltWeightTransferWindow, ImpulseSeatDropWindow, CrestSeatDropWindowV3, cold_table_mean_unavailable, C2ReverseBrake, BrakeSample, _brake_finite_scalar,
            _finite_vector, _proper_rotation, _rz, closed_head_target, frozen_open_target,
            downward_hold_feed, bounded_phase_gate,
            LoadedFlankWindow,
            YamCartesianController, YamM8ControlConfig, smooth_profile)).encode()).hexdigest(),
        "known_bolt_mass_kg": mass,
        "pickup_grasp_face": {"offset_rad": control.pickup_grasp_face_offset_rad,
            "scope": "Actual regular-hex head flat pair selected for robot wrist travel; spawned bolt/block poses and thread phase are unchanged"},
        "pickup_transport_clearance": {"native_rest": rest_top,
            "commanded_lift_grasp_position_m": pickup_lift_goal.tolist(),
            "planned_bolt_tip_world_z_m": float(pickup_lift_goal[2]-shaft_and_half_head),
            "planned_tip_above_highest_rest_m": float(planned_tip_clearance),
            "minimum_measured_tip_above_rest_during_transfer_m": control.minimum_tip_rest_clearance_m,
            "scope": "Lift the whole shaft above the declared head-support rest before lateral transfer; descend over the bore only after transfer"},
        "full_flank_overlap_threshold_m": full_flank_overlap_m,
        "male_tip_chamfer_m": male_tip_chamfer_m,
        "female_full_profile_chamfer_bound_m": female_full_profile_chamfer_m,
        "minimum_engagement_contact_normal_force_N": 1e-5,
        "engagement_observer": LoadedFlankWindow.version,
        "engagement_observer_source_sha256": hashlib.sha256(
            Path(inspect.getfile(LoadedFlankWindow)).read_bytes()).hexdigest(),
        "engagement_observer_scope": "Candidate full-flank capture; actual coupled lead and open-hand unseated bolt resets with a table-supported block establish completed engagement",
        "engagement_window": {"duration_s": control.full_flank_contact_streak_s,
            "minimum_loaded_normal_impulse_Ns": .1*control.net_axial_feed_N*control.full_flank_contact_streak_s,
            "minimum_loaded_duration_s": .0005,
            "maximum_helix_phase_range_m": 150e-6},
        "legacy_engagement_note": "Consecutive positive force for 0.2 s retained as a diagnostic; unilateral contact may have valid resolved gaps",
        "axial_command_note": ("Known bolt-weight compensation plus constant net axial feed, with declared dissipative relative hand/hole velocity damping through finite native arm motors; no axial position or pitch spring"
                               if control.axial_velocity_damping_Ns_per_m else
                               "Gravity compensation of known bolt weight plus constant net axial feed; no axial motion servo during entry or turns"),
        "axial_velocity_reference_note": "Actual hand SITE linear velocity minus hole rigid-frame velocity evaluated at the hand point; pre-command native kinematics retained from the preceding mj_step's pre-integration state (the initialized state on the first command), with requested feed subject to the existing Cartesian-force and native motor-torque caps",
        "entry_support_note": "Optional bounded constant-force entry/starting-stop dwell requires settled actual bolt velocity and measured native SDF-pair load before rotation or searching release; cone support is separate from formed-flank capture, and acquisition timeout retains the closed grasp",
        "entry_support_events": [],
        "physical_feedback_controller": "supported-head-feedback-v1",
        "perfect_state_feedback_scope": "Perfect native head/tool poses at each physics step through finite arm/finger motors; no perception claim, object-state write, object force or prescribed helix",
        "closed_feedback_scope": "Measured tool/head transform calibrates head alignment and transverse centering; desired head yaw is an actual initial anchor plus independent stroke. Axial force-float removes every axial position/lead spring. Original per-grasp references remain fixed for cumulative slip guards.",
        "open_feedback_scope": "Freeze actual release tool/head transform before opening. Explicit fullXYZ free-hand trajectory rotates that actual tool by minus pi about bore; no disconnected dynamic head feedback. Fullyopen requires zero entire-right-robot/bolt contacts and actual unsupported10um/.02rad drift.",
        "left_hold_scope": "After entry acquisition, bounded worldZ downward2N feed with dissipative actual-site velocity damping through existing8NCartesian/motor caps. No block force or lift; solved pads/table retain weight-bearing guards, table may exceedmg from physical clampdown.",
        "feedback_source_sha256": hashlib.sha256(Path(inspect.getfile(closed_head_target)).read_bytes()).hexdigest(),
        "maximum_phase_plan": selected,
        "physical_motion_events": [],
        "physical_phase_timing_scope": "Native event consumes a live original solve; following phase starts on next native timestep. Maximum durations differ from actual executed durations; saved time/state/force never retimestamped.",
        "seat_direction_stop_scope": "Actual measured50um drop holds the last independently commanded HEAD-yaw increment. The body and hand then settle through finite motors; continuous actual max(body,hand) angular speed and axial speed plus original native impulse/duty/endpoint load confirm a stopped direction event. No numeric object/tool clock reset or capture claim.",
        "command_calibration_timing": "Measured tool/head target calibration is copied before mj_step from retained previous-solve transforms (time minus2dt after the subsequent step, except initialized first command). Original force observer and its kinematics are from the current native solve at time minusdt. Saved qpos/qvel and explicitly labeled actual jaw aperture are post-integration at time.",
        "native_force_recording_note": "mj_step contact forces and contact geometry are from the pre-integration state at recorded time minus timestep; saved qpos/qvel are the post-integration state",
        "left_pad_history_scope": "Every physics substep after end-settle stabilization acquisition; separately from 5 ms saved-pose sampling",
        "partial": True,
    }
    sample_steps = max(1, round(control.sample_period_s / model.opt.timestep))
    peak_depth = peak_left_slip = peak_left_angle = peak_grip_slip = 0.
    peak_alignment_radial = peak_alignment_tilt = 0.
    maximum_support_after_pickup = maximum_left_support = 0
    minimum_joint_margin = np.inf
    minimum_lift_pad_normals = np.full(2, np.inf)
    minimum_loaded_left_pad_normals = np.full(2, np.inf)
    grip_reference = None
    peak_right_grip_rotation_slip = 0.
    metadata["right_grasp_acquisitions"] = []
    left_pad_geoms = [model.geom(f"left_m8_pad_{side}").id for side in ("left", "right")]
    left_pad_history = PadLoadHistory()
    left_pad_times, left_pad_forces, left_pad_phase_indices = [], [], []
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
    right_site_velocity = np.zeros(6)
    bolt_velocity = np.zeros(6)
    entry_support_enabled = control.maximum_entry_dwell_s is not None
    entry_support = EntrySupportWindow(control.entry_support_window_s,
        control.entry_support_velocity_limit_m_per_s, control.net_axial_feed_N)
    entry_acquisition_time = None
    entry_support_timeout = None
    bolt_weight = mass*float(np.linalg.norm(model.opt.gravity))
    weight_window = BoltWeightTransferWindow(bolt_weight_N=bolt_weight)
    open_weight_window = BoltWeightTransferWindow(bolt_weight_N=bolt_weight)
    reference_window = BoltWeightTransferWindow(bolt_weight_N=bolt_weight)
    settled_reference_streak = 0.
    seat_event = None
    desired_clock = 0.
    closed_yaw_anchor = yaw_total
    closed_clock_anchor = 0.
    open_calibration = None
    open_reference = None
    open_peak_axial_drift = open_peak_yaw_drift = 0.
    regrasp_streak = 0.
    feedback_rows = []
    left_site_velocity = np.zeros(6)
    # Persist the exact controller and model texts used for this integration.
    # Later source revisions cannot silently change how a partial trace is read.
    (output / "controller_source.py").write_text(Path(__file__).read_text())
    metadata["controller_module_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    (output / "engagement_observer_source.py").write_text(Path(inspect.getfile(LoadedFlankWindow)).read_text())
    (output / "scene.xml").write_text(scene_xml(scene))
    scene_source = Path(inspect.getfile(SupportedConfig)).read_text()
    (output / "scene_source.py").write_text(scene_source)
    metadata["scene_source_sha256"] = hashlib.sha256(scene_source.encode()).hexdigest()
    recorded_helpers = output / "recorded_sources" / "yam_twin"
    recorded_helpers.mkdir(parents=True, exist_ok=True)
    for value in (YamCartesianController, fully_formed_flank_interval, InsertionControlConfig,
                  ArmIK, CrestSeatDropWindowV3, C2ReverseBrake):
        source_path = Path(inspect.getfile(value))
        (recorded_helpers / source_path.name).write_bytes(source_path.read_bytes())
    from . import (m8_supported_scene, m8_insertion_scene, m8_scene,
                   m8_insertion_engagement, m8_insertion_simulation, m8_supported_start)
    for module in (m8_supported_scene, m8_insertion_scene, m8_scene,
                   m8_insertion_engagement, m8_insertion_simulation, m8_supported_start):
        source_path = Path(module.__file__)
        (recorded_helpers / source_path.name).write_bytes(source_path.read_bytes())
    metadata["recorded_source_dependencies_sha256"] = {
        str(path.relative_to(output)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(recorded_helpers.glob("*.py"))}
    search_skip = {1: False}
    left_pickup_phases = {"settle_table", "reach_left_block", "close_left_block", "settle_left_block"}
    table_geoms = [model.geom(name).id for name in TABLE_SUPPORT_GEOM_NAMES]
    block_weight = float(np.sum(model.body_mass[model.body_weldid == block_id]))*abs(float(model.opt.gravity[2]))
    table_window = TableLoadWindow(control.settled_table_window_s, block_weight,
        minimum_table_fraction=control.minimum_table_weight_fraction,
        maximum_hand_upward_fraction=control.maximum_hand_upward_weight_fraction,
        minimum_loaded_duty=control.minimum_loaded_table_duty)
    task_table_window = TableLoadWindow(control.settled_table_window_s, block_weight,
        minimum_table_fraction=control.minimum_table_weight_fraction,
        maximum_hand_upward_fraction=control.maximum_hand_upward_weight_fraction,
        minimum_loaded_duty=control.minimum_loaded_table_duty)
    metadata["known_block_weight_N"] = block_weight
    metadata["table_support_geom_names"] = list(TABLE_SUPPORT_GEOM_NAMES)
    metadata["minimum_tip_table_clearance_m"] = scene.minimum_tip_table_clearance_m
    metadata["support_force_timing"] = metadata["native_force_recording_note"]
    metadata["unused_carried_block_control_fields"] = ["left_lift_clearance_m", "block_lift_m"]
    support_rows = []
    block_r0 = data.xmat[block_id].reshape(3, 3).copy()
    peak_block_translation = peak_block_rotation = peak_block_lift = 0.
    minimum_tip_table_clearance = np.inf
    minimum_tip_rest_transfer_clearance = np.inf
    support_gap = maximum_support_gap = 0.
    minimum_active_table_force = np.inf
    active_table_unloaded_steps = active_table_observations = 0
    peak_unexpected_table_contacts = 0
    peak_unexpected_native_depth = 0.
    task_table_failed_windows = 0
    task_table_minimum_mean_fraction = np.inf
    task_hand_maximum_mean_positive_fraction = 0.
    task_table_minimum_loaded_duty = 1.
    # Preserve the original actually-acquired per-grasp references. Target
    # recalibration never rebases these cumulative physical slip guards.
    left_grip_p = np.asarray(_COLD_CONTEXT["left_acquisition"]["grasp_relative_block_position_m"]).copy()
    left_grip_r = np.asarray(_COLD_CONTEXT["left_acquisition"]["grasp_relative_block_rotation"]).copy()
    left_grasp_acquired = True
    left_acquisition_time = 0.
    grip_reference = (np.asarray(_COLD_CONTEXT["right_acquisition"]["grasp_relative_bolt_head_position_m"]).copy(),
                      np.asarray(_COLD_CONTEXT["right_acquisition"]["grasp_relative_bolt_rotation"]).copy())
    metadata["left_acquisition"] = _COLD_CONTEXT["left_acquisition"]
    metadata["right_grasp_acquisitions"] = [_COLD_CONTEXT["right_acquisition"]]
    block_p0, block_r0 = original_block_p0, original_block_r0
    hand_relative_r = original_hole_r0.T @ right_initial_r
    desired_clock = closed_clock_anchor = _COLD_CONTEXT["desired_clock_rad"]
    closed_yaw_anchor = _COLD_CONTEXT["closed_yaw_anchor_rad"]
    seat_event = CrestSeatDropWindowV3(_COLD_CONTEXT["settled_base_z_m"], bolt_weight)
    metadata["diagnostic_only"] = True
    metadata["cold_initialization"] = _COLD_CONTEXT["declaration"]
    metadata["experimental_crest_observer_source_sha256"] = _COLD_CONTEXT["observer_sha256"]
    metadata["inherited_reference_scope"] = "Original full-run measured left/right acquisition references are retained verbatim for cumulative grip guards. They are not new cold-branch acquisition evidence. Fresh native pad/table/force/motion windows begin with this branch; no old force window or solver warm-start is reused."
    metadata["starts_preengaged"] = False
    metadata["starts_grasp_ready"] = True
    metadata["block_starts_left_touching"] = True
    metadata["bolt_starts_on_declared_fixed_rest"] = False
    metadata["physical_feedback_controller"] = "cold-crest-return-c2-brake-diagnostic-v4"
    metadata["seat_direction_stop_scope"] = "Measured50um return from a valid contiguous actual bolt/female shallow crest requests CLOSED deceleration only. Persistent return, actual max(body,hand) angular quiet, axial slow motion and original EntrySupportWindow impulse/duty/current-load proof over100ms PLUS the unchanged live100ms90/10 native weight-transfer window are required before any forward scan. No capture or jaw opening occurs."
    metadata["cold_window_hold_scope"] = "First actual100ms trailing force mean is undefined after cold initialization: this explicitly unqualified warm-up retains all instantaneous table/pad/grip/collision/depth/limit/drive/cap guards. After complete fresh table-window coverage the unchanged original rolling90/10/99 guard applies every tick. Reverse cannot start before fresh table, bolt-weight90/10, and actual body/tool angular/axial quiet readiness. No original native force or window is reused."
    reverse_brake = None
    metadata["robot_yaw_brake"] = {"duration_s": _COLD_CONTEXT["brake_duration_s"],
        "source_sha256": _COLD_CONTEXT["brake_sha256"],
        "scope": "Actual measured crest return requests CLOSED robot-yaw deceleration. C2 target-clock continuation uses only the preceding independent scheduled theta/omega/alpha; no object/pitch phase or pose is imposed. Actual native hard guards remain active during braking; original100ms quiet/impulse/90–10 support is required afterwards."}
    for phase_index, (label, duration, angle0, angle1, gap0, gap1, axial_float) in enumerate(selected):
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
        if label.startswith(("release_", "reset_open_")):
            live_window = open_weight_window if label.startswith("reset_open_") else weight_window
            if (not live_window.ready or native_feedback["relative_bolt_angular_speed_rad_per_s"] > .01
                    or native_feedback["relative_hand_angular_speed_rad_per_s"] > .01):
                entry_support_timeout = {"phase": label, "time": float(data.time),
                    "reason": "No live100ms native bolt-weight/quiet support before jaw opening or open reset",
                    "radial_offset_m": radial, "bolt_tilt_rad": tilt,
                    "entry_support": live_window.report()}
                aborted = entry_support_timeout
                break
            metadata["entry_support_events"].append({"phase": label, "time_s": float(data.time),
                "event": "Measured native starting support permits searching release/reset",
                "radial_offset_m": radial, "bolt_tilt_rad": tilt,
                **live_window.report()})
        entry_dwell_phase = entry_support_enabled and label == "feed_to_entry"
        if entry_dwell_phase:
            entry_support = EntrySupportWindow(control.entry_support_window_s,
                control.entry_support_velocity_limit_m_per_s, control.net_axial_feed_N)
        if label.startswith(("release_", "open_settle_", "reset_open_", "open_hold_", "regrip_")):
            # An intentional open reset ends this measured grasp. The next
            # closed grasp gets its own reference after actual loaded settle.
            grip_reference = None
        phase_p0 = right.pose()[0]
        phase_left_p0, phase_left_r0 = left.pose()
        hole_r = data.xmat[hole_id].reshape(3, 3).copy()
        hole_p = data.xpos[hole_id].copy()
        phase_start_depth = float((hole_r.T @ (data.xpos[bolt_id] - hole_p))[2])
        phase_start_yaw = yaw_total
        held_relative_z = float((hole_r.T @ (phase_p0 - hole_p))[2])
        if label == "transfer_bolt_weight":
            closed_yaw_anchor = yaw_total
            clock = hole_r.T @ right.pose()[1] @ hand_relative_r.T
            desired_clock = closed_clock_anchor = float(np.arctan2(clock[1, 0], clock[0, 0]))
            settled_reference_streak = 0.
            transfer_tool_r0 = right.pose()[1].copy()
            transfer_bolt_r0 = data.xmat[bolt_id].reshape(3, 3).copy()
            alignment_rotvec = Rotation.from_matrix(
                hole_r @ _rz(closed_yaw_anchor) @ transfer_bolt_r0.T).as_rotvec()
        if label == "reverse_seat_1":
            angle0, angle1 = desired_clock, -control.maximum_reverse_angle_rad
            duration = 1.875*abs(angle1-angle0)/control.reverse_angular_speed_rad_s
        elif label == "stop_reverse_seat_1":
            if reverse_brake is None:
                raise RuntimeError("No preceding independently scheduled reverse clock for braking")
        elif label == "start_thread_1":
            if seat_event is None or not seat_event.ready:
                aborted = {"phase": label, "time": float(data.time),
                    "reason": "No confirmed actual-drop/native-impulse stopped search direction"}
                break
            angle0, angle1 = desired_clock, control.first_forward_clock_rad
            duration = 1.875*abs(angle1-angle0)/(control.starting_angular_speed_rad_s or control.arm.angular_speed_rad_s)
        elif label.startswith(("start_thread_", "turn_")):
            angle0, angle1 = desired_clock, desired_clock+np.pi
        elif label.startswith("release_"):
            release_p, release_r = right.pose()
            release_bolt_r = data.xmat[bolt_id].reshape(3, 3).copy()
            head = data.xpos[bolt_id]+release_bolt_r @ [0., 0., -scene.head_height/2]
            clock = hole_r.T @ release_r @ hand_relative_r.T
            desired_clock = float(np.arctan2(clock[1, 0], clock[0, 0]))
            open_calibration = {"tool_position_relative_hole_m": hole_r.T @ (release_p-hole_p),
                "tool_rotation_relative_hole": hole_r.T @ release_r,
                "head_offset_in_tool_m": release_r.T @ (head-release_p),
                "release_clock_rad": desired_clock, "reset_clock_rad": desired_clock-np.pi}
            open_reference = None
            open_peak_axial_drift = open_peak_yaw_drift = 0.
            metadata["physical_motion_events"].append({"event": "Frozen actual release calibration",
                "phase": label, "time_s": float(data.time),
                "calibration": {k: v.tolist() if isinstance(v, np.ndarray) else v
                                for k, v in open_calibration.items()},
                "scope": "Actual head/tool frame before jaw opening; no selected thread phase"})
            angle0 = angle1 = desired_clock
        elif label.startswith("reset_open_"):
            angle0, angle1 = open_calibration["release_clock_rad"], open_calibration["reset_clock_rad"]
        else:
            angle0 = angle1 = desired_clock
        if label.startswith("settle_regrip_"):
            regrasp_streak = 0.
        feedback_closed = label in {"cold_window_hold", "transfer_bolt_weight", "reverse_seat_1", "stop_reverse_seat_1"} or label.startswith(("start_thread_", "stop_start_", "turn_", "stop_"))
        fully_open_phase = label.startswith(("open_settle_", "reset_open_", "open_hold_"))
        frozen_open_phase = label.startswith(("release_", "open_settle_", "reset_open_", "open_hold_", "regrip_", "settle_regrip_"))
        phase_max_hand_contacts = phase_max_support = 0
        phase_max_left_support = 0
        phase_max_head_block_contacts = 0
        phase_peak_pad_torque = 0.
        phase_peak_rotation_drift = phase_peak_axial_drift = 0.
        phase_started_engaged = engaged
        phase_left_pad_history = PadLoadHistory()
        steps = round(duration / model.opt.timestep)
        for step in range(steps):
            u = (step + 1) / steps
            blend, derivative = smooth_profile(u)
            hole_p = data.xpos[hole_id].copy()
            hole_r = data.xmat[hole_id].reshape(3, 3).copy()
            down = hole_r[:, 2]
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY,
                                    hole_id, hole_velocity, 0)
            if label in left_pickup_phases:
                goal = initial["right"][0]
            elif label in ("secure_left", "reach_bolt", "close_bolt", "settle_bolt"):
                goal = pickup if label != "secure_left" else initial["right"][0]
            elif label == "lift_bolt":
                goal = pickup_lift_goal
            elif label == "transport_bolt":
                goal = hole_p - down * (female_half_height + shaft_and_half_head +
                                       control.transport_tip_clearance_m)
                # Lower only in align_over_hole, after lateral transfer.
                goal[2] = max(float(goal[2]), float(pickup_lift_goal[2]))
            elif label == "align_over_hole":
                goal = hole_p - down * (female_half_height + shaft_and_half_head +
                                       control.entry_tip_clearance_m)
            else:
                goal = hole_p + down * held_relative_z
            if label in left_pickup_phases or label in ("secure_left", "reach_bolt", "close_bolt", "settle_bolt", "lift_bolt", "transport_bolt", "align_over_hole"):
                target_p = phase_p0 + blend * (goal - phase_p0)
                target_v = derivative * (goal - phase_p0) / duration
            else:
                target_p = goal
                target_v = hole_velocity[3:] + np.cross(hole_velocity[:3], goal - hole_p)
            theta = angle0 + blend * (angle1 - angle0)
            omega = derivative * (angle1 - angle0) / duration
            alpha = (60*u-180*u*u+120*u*u*u)*(angle1-angle0)/(duration*duration)
            jerk = (60-360*u+360*u*u)*(angle1-angle0)/(duration*duration*duration)
            robot_yaw_brake_active = False
            if label == "stop_reverse_seat_1":
                brake_sample = reverse_brake.sample((step+1)*model.opt.timestep)
                theta, omega = brake_sample.theta_rad, brake_sample.omega_rad_s
                alpha, jerk = brake_sample.alpha_rad_s2, brake_sample.jerk_rad_s3
                robot_yaw_brake_active = (step+1)*model.opt.timestep < reverse_brake.duration_s-1e-12
            target_r = (right_initial_r if label in left_pickup_phases else
                        hole_r @ Rotation.from_euler("z", theta).as_matrix() @ hand_relative_r)
            feed = control.net_axial_feed_N + mass * float(np.dot(model.opt.gravity, down)) * -1
            actual_net_feed = control.net_axial_feed_N
            if feedback_closed:
                actual_net_feed = bolt_weight
                if label == "transfer_bolt_weight":
                    feedback_begin = control.left_hold_ramp_s
                    gravity_begin = feedback_begin+max(control.head_centering_s, control.head_alignment_s)
                    feed_blend, _ = smooth_profile(np.clip(((step+1)*model.opt.timestep-gravity_begin)/control.gravity_feed_ramp_s, 0., 1.))
                    actual_net_feed = control.net_axial_feed_N+feed_blend*(bolt_weight-control.net_axial_feed_N)
                feed = actual_net_feed-mass*float(np.dot(model.opt.gravity, down))
            axial_velocity = None
            axial_damping_force = 0.
            if axial_float and (control.axial_velocity_damping_Ns_per_m or entry_support_enabled):
                mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE,
                                        right.site_id, right_site_velocity, 0)
                axial_velocity = _relative_axial_velocity(data.site_xpos[right.site_id],
                    right_site_velocity[3:], hole_p, hole_velocity, down)
                axial_damping_force = -control.axial_velocity_damping_Ns_per_m*axial_velocity
                feed += axial_damping_force
            if feedback_closed:
                actual_tool_p, actual_tool_r = right.pose()
                actual_bolt_r = data.xmat[bolt_id].reshape(3, 3)
                actual_head_p = data.xpos[bolt_id]+actual_bolt_r @ [0., 0., -scene.head_height/2]
                scheduled_head_r = None
                if label == "transfer_bolt_weight":
                    align_blend, _ = smooth_profile(np.clip(((step+1)*model.opt.timestep-control.left_hold_ramp_s)/control.head_alignment_s, 0., 1.))
                    scheduled_head_r = Rotation.from_rotvec(align_blend*alignment_rotvec).as_matrix() @ transfer_bolt_r0
                target = closed_head_target(actual_tool_p, actual_tool_r, actual_head_p,
                    actual_bolt_r, hole_p, hole_r, closed_yaw_anchor+theta-closed_clock_anchor,
                    held_relative_z, omega, hole_velocity, desired_head_rotation=scheduled_head_r)
                target_p, target_r = target["position_m"], target["rotation"]
                target_v = target["linear_velocity_m_per_s"]
                target_w = target["angular_velocity_rad_per_s"]
                if label == "transfer_bolt_weight":
                    center_blend, _ = smooth_profile(np.clip(((step+1)*model.opt.timestep-control.left_hold_ramp_s)/control.head_centering_s, 0., 1.))
                    target_p = phase_p0+center_blend*(target_p-phase_p0)
                    if (step+1)*model.opt.timestep <= control.left_hold_ramp_s:
                        target_r = transfer_tool_r0
                        target_v, target_w = np.zeros(3), np.zeros(3)
            elif frozen_open_phase:
                target = frozen_open_target(hole_p, hole_r, hole_velocity,
                    open_calibration["tool_position_relative_hole_m"],
                    open_calibration["tool_rotation_relative_hole"],
                    open_calibration["head_offset_in_tool_m"],
                    theta-open_calibration["release_clock_rad"], omega)
                target_p, target_r = target["position_m"], target["rotation"]
                target_v, target_w = target["linear_velocity_m_per_s"], target["angular_velocity_rad_per_s"]
            else:
                target_w = np.zeros(3) if label in left_pickup_phases else hole_velocity[:3]+down*omega
            right.command(target_p, target_r, gap0 + blend * (gap1 - gap0),
                          linear_velocity=target_v,
                          angular_velocity=target_w,
                          axial_float=axial_float, axis_world=down, axial_feed_N=feed)
            if table_pickup:
                if label == "settle_table":
                    left_target_p, left_target_r = left_p0, left_r0
                    left_target_v, left_target_w = np.zeros(3), np.zeros(3)
                    left_gap = control.left_open_aperture_m
                elif label == "reach_left_block":
                    left_target_p = phase_left_p0 + blend*(left_pickup_p-phase_left_p0)
                    left_target_v = derivative*(left_pickup_p-phase_left_p0)/duration
                    left_target_r, left_target_w = left_r0, np.zeros(3)
                    left_gap = control.left_open_aperture_m
                elif label in ("close_left_block", "settle_left_block"):
                    left_target_p, left_target_r = left_pickup_p, left_r0
                    left_target_v, left_target_w = np.zeros(3), np.zeros(3)
                    left_gap = (control.left_open_aperture_m + blend*(control.arm.left_closed_aperture-control.left_open_aperture_m)
                                if label == "close_left_block" else control.arm.left_closed_aperture)
                else:
                    left_target_p, left_target_r = left_hold_p, left_hold_r
                    left_target_v, left_target_w = np.zeros(3), np.zeros(3)
                    left_gap = control.arm.left_closed_aperture
                left_force_active = bool(open_calibration is not None or feedback_closed)
                left_feed = 0.
                if left_force_active:
                    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_SITE,
                                            left.site_id, left_site_velocity, 0)
                    press_blend = 1.
                    if label == "transfer_bolt_weight":
                        press_blend, _ = smooth_profile(min((step+1)*model.opt.timestep/control.left_hold_ramp_s, 1.))
                    left_feed = downward_hold_feed(left_site_velocity[5],
                        control.left_downward_hold_N*press_blend,
                        control.left_axial_velocity_damping_Ns_per_m)
                left.command(left_target_p, left_target_r, left_gap,
                             linear_velocity=left_target_v, angular_velocity=left_target_w,
                             axial_float=left_force_active, axis_world=[0., 0., 1.],
                             axial_feed_N=left_feed)
            mujoco.mj_step(model, data)
            sampled_native_contacts = step % sample_steps == 0 or step == steps-1
            native_feedback = native_weight_transfer_sample(model, data, right, thread,
                                                            with_records=sampled_native_contacts)
            table_state = _table_support_state(model, data, block_id, table_geoms,
                                               with_contacts=sampled_native_contacts)
            native_left_contact = _left_hand_contact_state(model, data, block_id, left,
                                                          with_contacts=sampled_native_contacts)
            block_translation = float(np.linalg.norm(data.xpos[block_id]-block_p0))
            block_rotation = float(np.linalg.norm(Rotation.from_matrix(
                data.xmat[block_id].reshape(3, 3) @ block_r0.T).as_rotvec()))
            block_lift_now = float(data.xpos[block_id, 2]-block_p0[2])
            peak_block_translation = max(peak_block_translation, block_translation)
            peak_block_rotation = max(peak_block_rotation, block_rotation)
            peak_block_lift = max(peak_block_lift, block_lift_now)
            peak_unexpected_table_contacts = max(peak_unexpected_table_contacts,
                table_state["unexpected_world_contact_candidates"])
            table_bearing = bool(table_state["table_upward_force_N"] > .1*block_weight
                and not table_state["unexpected_world_contact_candidates"]
                and block_lift_now <= control.maximum_block_lift_m)
            table_window.observe(table_state["table_upward_force_N"],
                native_left_contact["wrench_world"][2], model.opt.timestep,
                label == "settle_left_block" and table_bearing
                and np.all(np.asarray(native_left_contact["pad_normal_force_N"]) > .1))
            # Include pre-acquisition original loads so the first active
            # trailing window is complete. Do not reset on a load failure:
            # transient sharing remains visible to the whole-task gate.
            task_table_window.observe(table_state["table_upward_force_N"],
                native_left_contact["wrench_world"][2], model.opt.timestep, True)
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
            thread_normal_force = 0.
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
                    if entry_support_enabled:
                        mujoco.mj_contactForce(model, data, contact_index, thread_force)
                        thread_normal_force += float(thread_force[0])
                    if contact_is_on_full_flanks(hole_r.T @ (c.pos-hole_p),
                                               bolt_r.T @ (c.pos-bolt_p), thread):
                        if not entry_support_enabled:
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
            if table_pickup and label == "settle_left_block" and step == steps-1:
                acquisition_contact = native_left_contact
                left_acquisition_pad_normals = acquisition_contact["pad_normal_force_N"]
                if np.all(np.asarray(left_acquisition_pad_normals) > .1) and table_window.ready:
                    left_grasp_acquired = True
                    left_acquisition_time = float(data.time)
                    left_grip_p = left_r.T @ (data.xpos[block_id] - left_p)
                    left_grip_r = left_r.T @ data.xmat[block_id].reshape(3, 3)
                    metadata["left_acquisition"] = {
                        "time_s": left_acquisition_time,
                        "phase": label,
                        "pad_normal_force_N": left_acquisition_pad_normals,
                        "block_world_support_contacts": _body_contact(model, data, block_id)["world_support_contacts"],
                        "grasp_relative_block_position_m": left_grip_p.tolist(),
                        "grasp_relative_block_rotation": left_grip_r.tolist(),
                        "pad_contact_compliance_observation": _pad_normal_forces(
                            model, data, block_id, left_pad_geoms, with_compliance=True),
                        "block_free_velocity": data.qvel[block_dof:block_dof+6].tolist(),
                        "table_load_window": table_window.report(),
                        "table_support": table_state,
                        "left_world_wrench_on_block_N_Nm": native_left_contact["wrench_world"],
                        "jaw_velocity_m_s": [float(data.qvel[
                            model.joint(f"left_{side}_finger").dofadr[0]])
                            for side in ("left", "right")],
                    }
                else:
                    aborted = {"phase": label, "time": float(data.time),
                               "reason": "Left stabilization lacks bilateral loaded pads or measured table weight-bearing",
                               "left_pad_normal_force_N": left_acquisition_pad_normals,
                               "table_load_window": table_window.report()}
            left_slip = (float(np.linalg.norm(left_r.T @ (data.xpos[block_id] - left_p) - left_grip_p))
                         if left_grasp_acquired else 0.)
            left_angle = (float(np.linalg.norm(Rotation.from_matrix(
                left_r.T @ data.xmat[block_id].reshape(3, 3) @ left_grip_r.T).as_rotvec()))
                          if left_grasp_acquired else 0.)
            if table_pickup and left_grasp_acquired:
                substep_pad_normals = _pad_normal_forces(model, data, block_id, left_pad_geoms)
                left_pad_history.observe(substep_pad_normals, model.opt.timestep)
                phase_left_pad_history.observe(substep_pad_normals, model.opt.timestep)
                left_pad_times.append(float(data.time))
                left_pad_forces.append(substep_pad_normals.copy())
                left_pad_phase_indices.append(phase_index)
            peak_left_slip = max(peak_left_slip, left_slip)
            peak_left_angle = max(peak_left_angle, left_angle)
            hand_p, hand_r = right.pose()
            bolt_hand_p = hand_r.T @ (bolt_p + bolt_r @ [0., 0., -scene.head_height/2] - hand_p)
            bolt_hand_r = hand_r.T @ bolt_r
            if label == "settle_bolt" and step == steps-1:
                right_acquisition = right.contact_wrench_on("male_bolt")
                if (np.all(np.asarray(right_acquisition["pad_normal_force_N"]) > .1)
                        and np.linalg.norm(bolt_hand_p) < .001):
                    grip_reference = (bolt_hand_p.copy(), bolt_hand_r.copy())
                    metadata["right_grasp_acquisitions"].append({"phase": label, "time_s": float(data.time),
                        "pad_normal_force_N": right_acquisition["pad_normal_force_N"],
                        "grasp_relative_bolt_head_position_m": bolt_hand_p.tolist(),
                        "grasp_relative_bolt_rotation": bolt_hand_r.tolist()})
                else:
                    aborted = {"phase": label, "time": float(data.time),
                        "reason": "Right closed grasp lacks bilateral native loaded pads or centered bolt head",
                        "right_pad_normal_force_N": right_acquisition["pad_normal_force_N"],
                        "bolt_head_relative_hand_position_m": bolt_hand_p.tolist()}
            grasp_guard_active = bool(grip_reference is not None and gap1 == control.arm.closed_aperture)
            grip_slip = (float(np.linalg.norm(bolt_hand_p - grip_reference[0]))
                         if grasp_guard_active else 0.)
            right_grip_rotation_slip = (float(np.linalg.norm(Rotation.from_matrix(
                bolt_hand_r @ grip_reference[1].T).as_rotvec()))
                if grasp_guard_active else 0.)
            peak_grip_slip = max(peak_grip_slip, grip_slip)
            peak_right_grip_rotation_slip = max(peak_right_grip_rotation_slip, right_grip_rotation_slip)
            support = _body_contact(model, data, bolt_id)["world_support_contacts"]
            left_support = _body_contact(model, data, block_id)["world_support_contacts"]
            maximum_left_support = max(maximum_left_support, left_support)
            phase_max_left_support = max(phase_max_left_support, left_support)
            phase_max_support = max(phase_max_support, support)
            if label not in left_pickup_phases and label not in ("secure_left", "reach_bolt", "close_bolt", "settle_bolt", "lift_bolt"):
                maximum_support_after_pickup = max(maximum_support_after_pickup, support)
            left_lift_m = float(data.xpos[block_id, 2]-block_p0[2])
            if left_grasp_acquired:
                active_table_observations += 1
                minimum_active_table_force = min(minimum_active_table_force, table_state["table_upward_force_N"])
                unloaded_table = table_state["table_upward_force_N"] <= .1*block_weight
                active_table_unloaded_steps += int(unloaded_table)
                support_gap = support_gap+model.opt.timestep if unloaded_table else 0.
                maximum_support_gap = max(maximum_support_gap, support_gap)
                task_table_failed_windows += int(not task_table_window.ready)
                task_table_minimum_mean_fraction = min(task_table_minimum_mean_fraction,
                    task_table_window.table_impulse/(task_table_window.elapsed*block_weight))
                task_hand_maximum_mean_positive_fraction = max(task_hand_maximum_mean_positive_fraction,
                    task_table_window.hand_upward_impulse/(task_table_window.elapsed*block_weight))
                task_table_minimum_loaded_duty = min(task_table_minimum_loaded_duty,
                    task_table_window.loaded_duration/task_table_window.elapsed)
            hand_contacts = _body_contact(model, data, bolt_id, right.hand_geom_ids)["contact_count"]
            right_robot_contacts = sum(1 for c in data.contact
                if any(int(model.body_weldid[int(model.geom_bodyid[int(g)])]) == bolt_id
                       for g in (c.geom1, c.geom2))
                and any((model.geom(int(g)).name or "").startswith("right_")
                        or (model.body(int(model.geom_bodyid[int(g)])).name or "").startswith("right_")
                        for g in (c.geom1, c.geom2)))
            phase_max_hand_contacts = max(phase_max_hand_contacts, hand_contacts)
            unforced = unforced and bool(np.all(data.xfrc_applied[[bolt_id, block_id, hole_id]] == 0)
                and np.all(data.qfrc_applied[bolt_dof:bolt_dof+6] == 0)
                and np.all(data.qfrc_applied[block_dof:block_dof+6] == 0))
            warnings = int(sum(w.number for w in data.warning))
            unexpected_native_contacts = _unexpected_native_contacts(model, data,
                block_id, bolt_id, table_geoms, left.pad_geom_ids, right.pad_geom_ids,
                (bolt_geom, female_geom), allow_bolt_rest=(label in left_pickup_phases
                or label in ("secure_left", "reach_bolt", "close_bolt", "settle_bolt", "lift_bolt")),
                head_geom=head_geom)
            bolt_support_forbidden = bool(support and label not in left_pickup_phases
                and label not in ("secure_left", "reach_bolt", "close_bolt", "settle_bolt", "lift_bolt"))
            peak_unexpected_native_depth = max(peak_unexpected_native_depth,
                max([-c["native_signed_distance_m"] for c in unexpected_native_contacts], default=0.))
            joint_margin = min(float(np.min(np.minimum(
                data.qpos[c.qpos_indices]-model.jnt_range[c.joint_ids, 0],
                model.jnt_range[c.joint_ids, 1]-data.qpos[c.qpos_indices])))
                for c in controllers.values())
            minimum_joint_margin = min(minimum_joint_margin, joint_margin)
            alignment_guard = label not in left_pickup_phases and label not in ("secure_left", "reach_bolt", "close_bolt", "settle_bolt", "lift_bolt", "transport_bolt", "align_over_hole") and overlap > 0
            if alignment_guard:
                peak_alignment_radial = max(peak_alignment_radial, radial)
                peak_alignment_tilt = max(peak_alignment_tilt, tilt)
            window = engagement_observer.update(float(data.time), model.opt.timestep,
                formed_overlap > thread.pitch and radial <= 150e-6 and tilt <= np.deg2rad(2)
                and left_slip < .001 and left_angle < np.deg2rad(2) and unforced
                and table_bearing and task_table_window.ready
                and not support and not head_block_contacts,
                loaded_formed_normal_force, loaded_formed_contacts,
                float(relative[2])-thread.pitch*yaw_total/(2*np.pi))
            if not engaged and window["ready"]:
                engaged = True
                engagement_time = float(data.time)
            cold_table_window_warmup = cold_table_mean_unavailable(label,
                task_table_window.elapsed, control.settled_table_window_s)
            if (thread_depth > 10e-6 or left_slip > .001 or left_angle > np.deg2rad(2)
                    or grip_slip > .001
                    or right_grip_rotation_slip > np.deg2rad(2)
                    or table_state["unexpected_world_contact_candidates"]
                    or block_lift_now > control.maximum_block_lift_m
                    or block_translation > .001 or block_rotation > np.deg2rad(2)
                    or (left_grasp_acquired and support_gap > .005)
                    or (left_grasp_acquired and (not table_bearing or (not task_table_window.ready and not cold_table_window_warmup)
                        or not np.all(np.asarray(native_left_contact["pad_normal_force_N"]) > .1)))
                    or (grasp_guard_active
                        and not np.all(np.asarray(native_feedback["right_pad_normal_force_N"]) > .1))
                    or (fully_open_phase and right_robot_contacts != 0)
                    or any(np.linalg.norm(c.last_wrench[:3]) > 8.+1e-12
                        or np.linalg.norm(c.last_wrench[3:]) > 2.+1e-12
                        or np.any(np.abs(c.last_motor_torques) > c.torque_caps+1e-12)
                        for c in controllers.values())
                    or warnings or not unforced
                    or unexpected_native_contacts
                    or bolt_support_forbidden
                    or joint_margin < -1e-5
                    or (alignment_guard and (radial > 150e-6 or tilt > np.deg2rad(2)))
                    or not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all()):
                aborted = {"phase": label, "time": float(data.time), "reported_sdf_depth_m": thread_depth,
                           "radial_offset_m": radial, "bolt_tilt_rad": tilt,
                           "block_grip_slip_m": left_slip, "block_grip_rotation_slip_rad": left_angle,
                           "bolt_grip_slip_m": grip_slip, "world_block_support_contacts": left_support,
                           "minimum_native_joint_margin_rad": joint_margin,
                           "warnings": warnings, "external_drive_zero": unforced}
                if unexpected_native_contacts:
                    aborted["unexpected_native_contacts"] = unexpected_native_contacts
            if fully_open_phase:
                if open_reference is None and right_robot_contacts == 0:
                    open_reference = {"time_s": float(data.time), "base_z_m": float(relative[2]),
                        "yaw_unwrapped_rad": yaw_total}
                    metadata["physical_motion_events"].append({"event": "First fully-open zero-right-contact state",
                        "phase": label, "reference": open_reference.copy()})
                if open_reference is not None:
                    open_peak_axial_drift = max(open_peak_axial_drift,
                        abs(float(relative[2])-open_reference["base_z_m"]))
                    open_peak_yaw_drift = max(open_peak_yaw_drift,
                        abs(yaw_total-open_reference["yaw_unwrapped_rad"]))
                    if open_peak_axial_drift > 10e-6 or open_peak_yaw_drift > .02:
                        aborted = {"phase": label, "time": float(data.time),
                            "reason": "Unassisted open search drift exceeded original10um/.02rad guards",
                            "peak_axial_drift_m": open_peak_axial_drift,
                            "peak_yaw_drift_rad": open_peak_yaw_drift}
            hard_guards_held = not bool(aborted)
            quiet = (native_feedback["relative_bolt_angular_speed_rad_per_s"] <= .01
                and native_feedback["relative_hand_angular_speed_rad_per_s"] <= .01
                and abs(native_feedback["relative_bolt_axial_velocity_m_per_s"]) <= .0002)
            bolt_open_quiet = (native_feedback["relative_bolt_angular_speed_rad_per_s"] <= .01
                and abs(native_feedback["relative_bolt_axial_velocity_m_per_s"]) <= .0002)
            weight_observation_valid = bool(not aborted and quiet)
            open_observation_valid = bool(fully_open_phase and right_robot_contacts == 0
                                          and not aborted and bolt_open_quiet)
            weight_report = weight_window.observe(float(data.time), model.opt.timestep,
                native_feedback, valid=weight_observation_valid)
            open_report = open_weight_window.observe(float(data.time), model.opt.timestep,
                native_feedback, valid=open_observation_valid)
            reference_report = reference_window.observe(float(data.time), model.opt.timestep,
                native_feedback, valid=bool(label == "transfer_bolt_weight" and not aborted and quiet))
            if label.startswith("settle_regrip_"):
                quiet_grasp = (not aborted and quiet and np.linalg.norm(bolt_hand_p) < .001
                    and np.all(np.asarray(native_feedback["right_pad_normal_force_N"]) > .1))
                regrasp_streak = regrasp_streak+model.opt.timestep if quiet_grasp else 0.
                if grip_reference is None and regrasp_streak+1e-12 >= control.regrasp_window_s:
                    grip_reference = (bolt_hand_p.copy(), bolt_hand_r.copy())
                    reference = {"phase": label, "time_s": float(data.time),
                        "continuous_quiet_bilateral_streak_s": regrasp_streak,
                        "pad_normal_force_N": native_feedback["right_pad_normal_force_N"],
                        "grasp_relative_bolt_head_position_m": bolt_hand_p.tolist(),
                        "grasp_relative_bolt_rotation": bolt_hand_r.tolist()}
                    metadata["right_grasp_acquisitions"].append(reference)
                    metadata["physical_motion_events"].append({"event": "Actual quiet bilateral regrasp acquired",
                        "phase": label, "reference": reference})
                    closed_yaw_anchor, closed_clock_anchor = yaw_total, theta
            feedback_phase_completed = False
            elapsed_phase = (step+1)*model.opt.timestep
            gate = "continue"
            if label == "cold_window_hold":
                gate = bounded_phase_gate(elapsed_phase, control.settled_table_window_s,
                    duration, task_table_window.ready and weight_window.ready and quiet,
                    valid=not aborted)
            elif label == "transfer_bolt_weight":
                after_ramp = elapsed_phase >= (control.left_hold_ramp_s+max(control.head_centering_s, control.head_alignment_s)+control.gravity_feed_ramp_s+
                                              control.gravity_feed_hold_s)
                settled_reference_streak = (settled_reference_streak+model.opt.timestep
                    if after_ramp and quiet and not aborted else 0.)
                gate = bounded_phase_gate(elapsed_phase,
                    control.left_hold_ramp_s+max(control.head_centering_s, control.head_alignment_s)+control.gravity_feed_ramp_s+control.gravity_feed_hold_s,
                    duration, settled_reference_streak >= .1-1e-12,
                    valid=not aborted)
                if gate == "complete":
                    seat_event = ImpulseSeatDropWindow(float(relative[2]), bolt_weight)
                    metadata["physical_motion_events"].append({"event": "Actual post-ramp settled seat reference",
                        "phase": label, "time_s": float(data.time), "actual_dwell_s": elapsed_phase,
                        "base_z_m": float(relative[2]), "desired_clock_rad": theta,
                        "stable_actual_quiet_streak_s": settled_reference_streak,
                        "weight_window": reference_report,
                        "scope": "Actual100ms aligned/slow/angular-stopped reference after complete load ramp; direction baseline only"})
            elif label in ("reverse_seat_1", "stop_reverse_seat_1"):
                if seat_event is None:
                    raise RuntimeError("Reverse seat motion lacks a measured settled reference")
                seat_report = seat_event.observe(float(data.time), model.opt.timestep, float(relative[2]),
                    native_feedback["relative_bolt_axial_velocity_m_per_s"],
                    native_feedback["thread_summed_normal_force_N"],
                    actual_relative_angular_speed_rad_per_s=max(
                        native_feedback["relative_bolt_angular_speed_rad_per_s"],
                        native_feedback["relative_hand_angular_speed_rad_per_s"]),
                    valid=bool(not aborted and table_bearing and task_table_window.ready),
                    physically_stopped=(label == "stop_reverse_seat_1" and not robot_yaw_brake_active))
                ready = (seat_event.stop_requested if label == "reverse_seat_1"
                         else seat_event.ready and weight_window.ready and quiet)
                gate = bounded_phase_gate(elapsed_phase, 0., duration, ready, valid=not aborted)
            elif label.startswith(("stop_start_", "stop_")):
                gate = bounded_phase_gate(elapsed_phase, .12, duration,
                    weight_window.ready and quiet, valid=not aborted)
            elif label.startswith("open_settle_"):
                gate = bounded_phase_gate(elapsed_phase, control.minimum_open_settle_s,
                    duration, open_weight_window.ready and bolt_open_quiet and right_robot_contacts == 0,
                    valid=not aborted)
            elif label.startswith("settle_regrip_"):
                gate = bounded_phase_gate(elapsed_phase, control.regrasp_window_s,
                    duration, grip_reference is not None and quiet, valid=not aborted)
            if gate == "complete":
                feedback_phase_completed = True
                if label == "reverse_seat_1":
                    try:
                        reverse_brake = C2ReverseBrake(theta, omega, alpha,
                            _COLD_CONTEXT["brake_duration_s"],
                            minimum_theta_rad=-control.maximum_reverse_angle_rad,
                            maximum_theta_rad=0., maximum_speed_rad_s=control.reverse_angular_speed_rad_s)
                    except ValueError as error:
                        reverse_brake = None
                        feedback_phase_completed = False
                        gate = "invalid"
                        aborted = {"phase": label, "time": float(data.time),
                            "reason": "Independent C2 robot-yaw braking plan rejected without relaxing bounds",
                            "brake_validation_error": str(error),
                            "scheduled_theta_rad": theta, "scheduled_omega_rad_s": omega,
                            "scheduled_alpha_rad_s2": alpha}
                    if reverse_brake is not None:
                        metadata["physical_motion_events"].append({
                            "event": "Measured crest return requests C2 CLOSED robot-yaw braking",
                            "time_s": float(data.time), "force_time_s": float(data.time)-model.opt.timestep,
                            "initial_theta_rad": theta, "initial_omega_rad_s": omega,
                            "initial_alpha_rad_s2": alpha, "duration_s": reverse_brake.duration_s,
                            "terminal_theta_rad": reverse_brake.sample(reverse_brake.duration_s).theta_rad,
                            "scope": "Direction-search deceleration request only; actual original load/quiet windows still unqualified."})
                metadata["physical_motion_events"].append({"event": ("Live physical phase readiness consumed" if feedback_phase_completed
                              else "Live direction request could not form a bounded braking plan"),
                    "phase": label, "time_s": float(data.time), "force_time_s": float(data.time)-model.opt.timestep,
                    "actual_dwell_s": elapsed_phase, "maximum_dwell_s": duration,
                    "desired_clock_rad": theta, "weight_window": weight_report,
                    "open_weight_window": open_report,
                    "seat_direction_event": seat_event.report() if label in ("reverse_seat_1", "stop_reverse_seat_1") else None})
            elif gate == "timeout":
                aborted = {"phase": label, "time": float(data.time),
                    "reason": "Bounded physical feedback phase lacked live measured readiness",
                    "weight_window": weight_report, "open_weight_window": open_report,
                    "seat_direction_event": seat_event.report() if seat_event else None}
            desired_clock = theta
            entry_dwell_completed = False
            if entry_support_enabled:
                mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY,
                                        hole_id, hole_velocity, 0)
                mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY,
                                        bolt_id, bolt_velocity, 0)
                bolt_axial_velocity = _relative_axial_velocity(bolt_p, bolt_velocity[3:],
                                                               hole_p, hole_velocity, down)
                entry_support.observe(thread_normal_force, bolt_axial_velocity, model.opt.timestep,
                    radial <= 150e-6 and tilt <= np.deg2rad(2) and not support
                    and table_bearing and task_table_window.ready
                    and unforced and left_slip < .001 and left_angle < np.deg2rad(2))
                if entry_dwell_phase and not aborted:
                    minimum_stop_s = .12 if label.startswith("stop_start_") else 0.
                    entry_dwell_completed = ((step+1)*model.opt.timestep >= minimum_stop_s
                        and (entry_support.ready or (label.startswith("stop_start_") and engaged)))
                    if entry_dwell_completed:
                        event = {"phase": label, "time_s": float(data.time),
                            "actual_dwell_s": (step+1)*model.opt.timestep,
                            "maximum_dwell_s": duration, "radial_offset_m": radial,
                            "bolt_tilt_rad": tilt, "formed_flank_capture_observed": engaged,
                            **entry_support.report()}
                        metadata["entry_support_events"].append(event)
                        if label == "feed_to_entry":
                            entry_acquisition_time = float(data.time)
                    elif step == steps-1:
                        entry_support_timeout = {"phase": label, "time": float(data.time),
                            "reason": "Bounded native starting acquisition timed out; closed grasp retained",
                            "actual_dwell_s": duration, "radial_offset_m": radial,
                            "bolt_tilt_rad": tilt, "entry_support": entry_support.report()}
                        aborted = entry_support_timeout
            tip_table_clearance = float((bolt_p+bolt_r @ tip_local)[2]-table_top_height(scene))
            tip_rest_clearance = float((bolt_p+bolt_r @ tip_local)[2]-rest_top_z)
            if (label == "lift_bolt" and step == steps-1) or label == "transport_bolt":
                minimum_tip_rest_transfer_clearance = min(minimum_tip_rest_transfer_clearance,
                                                          tip_rest_clearance)
                if tip_rest_clearance < control.minimum_tip_rest_clearance_m:
                    aborted = {"phase": label, "time": float(data.time),
                        "reason": "The full male shaft did not clear its native head-support rest before lateral transfer",
                        "bolt_tip_above_highest_rest_m": tip_rest_clearance,
                        "minimum_allowed_clearance_m": control.minimum_tip_rest_clearance_m}
            minimum_tip_table_clearance = min(minimum_tip_table_clearance, tip_table_clearance)
            if tip_table_clearance < scene.minimum_tip_table_clearance_m:
                aborted = {"phase": label, "time": float(data.time),
                    "reason": "Male tip violated plain-table clearance guard",
                    "bolt_tip_table_clearance_m": tip_table_clearance,
                    "minimum_allowed_clearance_m": scene.minimum_tip_table_clearance_m}
            support_rows.append((float(data.time), phase_index,
                table_state["table_wrench_on_block_world"], native_left_contact["wrench_world"],
                native_left_contact["pad_normal_force_N"], data.xpos[block_id].copy(),
                data.xmat[block_id].reshape(3, 3).copy(), unforced,
                table_state["unexpected_world_contact_candidates"], left_grasp_acquired, support,
                table_state["table_upward_normal_force_N"], table_state["contact_candidates"],
                table_state["loaded_contacts"], tip_table_clearance,
                float(relative[2]), yaw_total, hand_contacts, head_block_contacts,
                formed_overlap, loaded_formed_normal_force, loaded_formed_contacts))
            support_rows[-1] += (len(unexpected_native_contacts),
                max([-c["native_signed_distance_m"] for c in unexpected_native_contacts], default=0.),
                tip_rest_clearance)
            feedback_rows.append((float(data.time), phase_index,
                native_feedback["thread_wrench_on_bolt_world_N_Nm"],
                native_feedback["hand_wrench_on_bolt_world_N_Nm"],
                native_feedback["right_pad_normal_force_N"],
                native_feedback["thread_gravity_opposing_force_N"],
                native_feedback["hand_gravity_opposing_force_N"],
                native_feedback["thread_summed_normal_force_N"],
                native_feedback["relative_bolt_axial_velocity_m_per_s"],
                native_feedback["relative_bolt_angular_speed_rad_per_s"],
                native_feedback["relative_hand_angular_speed_rad_per_s"],
                right_robot_contacts, fully_open_phase,
                grasp_guard_active,
                axial_float, feed if axial_float else 0., gap0+blend*(gap1-gap0), theta, omega,
                open_peak_axial_drift, open_peak_yaw_drift, weight_window.ready,
                open_weight_window.ready, regrasp_streak, hard_guards_held,
                native_feedback["loaded_actual_interior_flank_contact_count"],
                native_feedback["interior_flank_gravity_opposing_force_N"],
                native_feedback["interior_flank_summed_normal_force_N"],
                left_feed, right.last_wrench.copy(), left.last_wrench.copy(),
                right.last_motor_torques.copy(), left.last_motor_torques.copy(),
                float(data.qpos[model.joint("right_left_finger").qposadr[0]]-
                    data.qpos[model.joint("right_right_finger").qposadr[0]]-2*scene.base.pad_inner_offset),
                native_feedback["external_drive_zero"],
                native_feedback["bolt_world_support_contact_count"],
                native_feedback["nonthread_block_bolt_contact_count"],
                native_feedback["radial_offset_m"], native_feedback["bolt_tilt_rad"],
                native_feedback["native_thread_contact_count"],
                weight_observation_valid, open_observation_valid,
                grip_slip, right_grip_rotation_slip, left_slip, left_angle, cold_table_window_warmup, robot_yaw_brake_active, alpha, jerk))
            if step % sample_steps == 0 or step == steps-1 or aborted or entry_dwell_completed or feedback_phase_completed:
                if not sampled_native_contacts:
                    # Re-read the original solved contacts before any forward
                    # call; saved early-exit rows need the same raw force proof.
                    table_state = _table_support_state(model, data, block_id, table_geoms,
                                                       with_contacts=True)
                    native_left_contact = _left_hand_contact_state(model, data, block_id, left,
                                                                  with_contacts=True)
                    native_feedback = native_weight_transfer_sample(model, data, right, thread,
                                                                    with_records=True)
                contact = right.contact_wrench_on("male_bolt")
                left_contact = native_left_contact
                if (left_grasp_acquired if table_pickup else label != "secure_left"):
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
                       "native_thread_pair_normal_force_N": thread_normal_force if entry_support_enabled else None,
                       "entry_support": entry_support.report() if entry_support_enabled else None,
                       "engagement_window": window,
                       "legacy_consecutive_loaded_contact_streak_s": engagement_streak,
                       "head_block_seating_contacts": head_block_contacts,
                       "formed_flank_overlap_m": formed_overlap,
                       "thread_engaged": engaged, "radial_offset_m": radial,
                       "bolt_tilt_rad": tilt, "reported_sdf_depth_m": thread_depth,
                       "bolt_world_position": bolt_p.tolist(), "hole_world_position": hole_p.tolist(),
                       "bolt_tip_world_position": (bolt_p + bolt_r @ tip_local).tolist(),
                       "block_lift_m": float(data.xpos[block_id, 2] - block_p0[2]),
                       "left_grasp_acquired": left_grasp_acquired,
                       "left_stabilization_verified": left_grasp_acquired,
                       "block_grip_slip_m": left_slip, "block_grip_rotation_slip_rad": left_angle,
                       "bolt_grip_slip_m": grip_slip, "bolt_world_support_contacts": support,
                       "bolt_grip_rotation_slip_rad": right_grip_rotation_slip,
                       "block_world_support_contacts": left_support, "external_drive_zero": unforced,
                       "table_support": table_state,
                       "known_block_weight_N": block_weight,
                       "positive_left_hand_upward_force_N": max(native_left_contact["wrench_world"][2], 0.),
                       "settled_table_load_window": metadata.get("left_acquisition", {}).get("table_load_window"),
                       "active_table_load_window": task_table_window.report(),
                       "block_translation_from_initial_m": block_translation,
                       "block_rotation_from_initial_rad": block_rotation,
                       "bolt_tip_table_clearance_m": float((bolt_p+bolt_r @ tip_local)[2]-table_top_height(scene)),
                       "bolt_tip_rest_top_clearance_m": tip_rest_clearance,
                       "unexpected_native_contacts": unexpected_native_contacts,
                       "minimum_native_joint_margin_rad": joint_margin,
                       "contact": contact, "left_contact": left_contact,
                       "axial_float": axial_float, "axial_feed_command_N": feed if axial_float else None,
                       "right_relative_axial_velocity_m_per_s": axial_velocity,
                       "axial_velocity_damping_force_N": axial_damping_force if axial_float else None,
                       "right_commanded_stroke_angular_speed_rad_s": omega,
                       "right_position_error_m": right.last_position_error.tolist(),
                       "right_rotation_error_rad": right.last_rotation_error.tolist()}
                row.update({"native_feedback": native_feedback,
                    "weight_window": weight_report, "open_weight_window": open_report,
                    "seat_direction_event": seat_event.report() if label in ("reverse_seat_1", "stop_reverse_seat_1") else None,
                    "right_robot_bolt_contact_count": right_robot_contacts,
                    "fully_open_unassisted": fully_open_phase,
                    "right_grasp_guard_active": grasp_guard_active,
                    "right_axial_float_active": axial_float,
                    "right_applied_axial_feed_N": feed if axial_float else 0.,
                    "right_commanded_aperture_m": gap0+blend*(gap1-gap0),
                    "desired_independent_clock_rad": theta,
                    "open_peak_axial_drift_m": open_peak_axial_drift,
                    "open_peak_yaw_drift_rad": open_peak_yaw_drift,
                    "actual_quiet_regrasp_streak_s": regrasp_streak,
                    "live_physical_phase_gate": gate,
                    "right_command_wrench_N_Nm": right.last_wrench.tolist(),
                    "left_command_wrench_N_Nm": left.last_wrench.tolist(),
                    "left_downward_feed_N": left_feed,
                    "net_axial_feed_N": actual_net_feed,
                    "weight_observation_valid": weight_observation_valid,
                    "open_observation_valid": open_observation_valid,
                    "all_hard_guards_held": hard_guards_held, "cold_table_window_warmup": cold_table_window_warmup,
                    "robot_yaw_brake_active": robot_yaw_brake_active,
                    "desired_independent_angular_acceleration_rad_s2": alpha,
                    "desired_independent_angular_jerk_rad_s3": jerk,
                    "command_calibration": ({"head_to_tool_p_m": target["measured_head_to_tool_p"].tolist(),
                        "head_to_tool_R": target["measured_head_to_tool_R"].tolist(),
                        "source_prior_solve_time_s": float(data.time)-2*model.opt.timestep,
                        "scope": "Copied pre-command retained previous native solve; independent from this row's original force-time transforms"}
                        if feedback_closed else None)})
                rows.append(row); times.append(float(data.time)); positions.append(data.qpos.copy())
                velocities.append(data.qvel.copy()); controls.append(data.ctrl.copy())
            if aborted or entry_dwell_completed or feedback_phase_completed:
                break
        end_depth = float((hole_r.T @ (data.xpos[bolt_id] - hole_p))[2])
        rotation = yaw_total-phase_start_yaw
        advance = end_depth-phase_start_depth
        summary = {"phase": label,
                   "maximum_planned_duration_s": duration,
                   "actual_executed_duration_s": (step+1)*model.opt.timestep,
                   "actual_final_desired_clock_rad": desired_clock,
                   "live_physical_phase_gate": gate,
                   "final_native_weight_window": weight_report,
                   "final_native_open_weight_window": open_report,
                   "final_table_support": table_state,
                   "maximum_block_translation_from_initial_m": peak_block_translation,
                   "maximum_block_rotation_from_initial_rad": peak_block_rotation, "bolt_insertion_advance_m": advance,
                   "bolt_clockwise_rotation_rad": rotation,
                   "observed_helix_residual_m": advance-thread.pitch*rotation/(2*np.pi),
                   "maximum_hand_bolt_contacts": phase_max_hand_contacts,
                   "maximum_bolt_world_support_contacts": phase_max_support,
                   "maximum_block_world_support_contacts": phase_max_left_support,
                   "maximum_head_block_seating_contacts": phase_max_head_block_contacts,
                   "sampled_peak_pad_contact_torque_Nm": phase_peak_pad_torque,
                   "maximum_absolute_bolt_rotation_from_phase_start_rad": phase_peak_rotation_drift,
                   "maximum_absolute_bolt_axial_motion_from_phase_start_m": phase_peak_axial_drift,
                   "started_engaged": phase_started_engaged, "ended_engaged": engaged,
                   "final_bolt_world_support_contacts": support,
                   "final_block_world_support_contacts": left_support,
                   "final_block_lift_m": left_lift_m,
                   "left_grasp_acquired": left_grasp_acquired,
                   "left_stabilization_verified": left_grasp_acquired,
                   "all_substep_left_pad_loads": phase_left_pad_history.report() if table_pickup else None,
                   "final_thread_overlap_m": float(overlap),
                   "final_formed_flank_overlap_m": formed_overlap}
        if entry_support_enabled:
            summary["actual_duration_s"] = (step+1)*model.opt.timestep
            summary["maximum_scheduled_duration_s"] = duration
            summary["entry_support"] = entry_support.report()
        summaries.append(summary)
        if table_pickup:
            metadata["all_substep_left_pad_loads"] = left_pad_history.report()
        print(json.dumps(summary), flush=True)
        np.savez_compressed(output / "insertion_trace_partial.npz", time=times, qpos=positions,
            qvel=velocities, controller=controls, info_json=np.asarray(json.dumps(rows)),
            metadata_json=np.asarray(json.dumps(metadata)))
        if aborted:
            break
    turns = [s for s in summaries if s["phase"].startswith("turn_")]
    resets = [s for s in summaries if s["phase"].startswith("reset_open_")]
    expected_resets = sum(s["phase"].startswith(("start_thread_", "turn_")) for s in summaries)-1
    if table_pickup:
        force_history_path = output / "left_pad_force_history.npz"
        np.savez_compressed(force_history_path, time=np.asarray(left_pad_times),
            pad_normal_force_N=np.asarray(left_pad_forces, dtype=float).reshape(-1, 2),
            phase_index=np.asarray(left_pad_phase_indices, dtype=np.int16),
            phase_labels_json=np.asarray(json.dumps([p[0] for p in selected])),
            metadata_json=np.asarray(json.dumps({"model_fingerprint": metadata["model_fingerprint"],
                "controller_sha256": metadata["controller_sha256"], "runtime": runtime,
                "observer": PadLoadHistory.version, "timestep_s": model.opt.timestep,
                "minimum_loaded_force_N": left_pad_history.threshold,
                "scope": metadata["left_pad_history_scope"],
                "timing": metadata["native_force_recording_note"]})))
        metadata["left_pad_force_history"] = {
            "filename": force_history_path.name,
            "sha256": hashlib.sha256(force_history_path.read_bytes()).hexdigest(),
            "observed_physics_steps": left_pad_history.observations}
    ledger_path = output / "table_support_force_history.npz"
    ledger_names = ("time", "phase_index", "table_wrench_world_at_block_origin_N_Nm",
        "left_hand_wrench_world_at_block_origin_N_Nm", "pad_normal_force_N",
        "block_position_m", "block_rotation_matrix", "external_drive_zero",
        "unexpected_world_support_count", "supported_task_active", "bolt_world_support_count",
        "table_upward_normal_force_N", "table_contact_candidates", "table_loaded_contacts",
        "bolt_tip_table_clearance_m", "bolt_base_insertion_m", "bolt_yaw_unwrapped_rad",
        "right_hand_bolt_contact_count", "head_block_seating_contact_count",
        "formed_flank_overlap_m", "loaded_formed_thread_normal_force_N",
        "loaded_formed_thread_contact_count", "unexpected_native_contact_count",
        "unexpected_native_contact_peak_depth_m", "bolt_tip_rest_top_clearance_m")
    ledger_values = {name: np.asarray([row[i] for row in support_rows])
                     for i, name in enumerate(ledger_names)}
    np.savez_compressed(ledger_path, **ledger_values,
        phase_labels_json=np.asarray(json.dumps([p[0] for p in selected])),
        metadata_json=np.asarray(json.dumps({"model_fingerprint": metadata["model_fingerprint"],
            "controller_sha256": metadata["controller_sha256"], "runtime": runtime,
            "timestep_s": model.opt.timestep, "force_timing": metadata["native_force_recording_note"],
            "observer": TableLoadWindow.version,
            "scope": "Original all-step native force aggregates and pre-integration body kinematics; no pose replay force reconstruction"})))
    metadata["table_support_force_history"] = {"filename": ledger_path.name,
        "sha256": hashlib.sha256(ledger_path.read_bytes()).hexdigest(),
        "observed_physics_steps": len(support_rows)}
    feedback_path = output / "native_feedback_force_history.npz"
    feedback_names = ("time", "phase_index", "thread_wrench_on_bolt_world_N_Nm",
        "hand_wrench_on_bolt_world_N_Nm", "right_pad_normal_force_N",
        "thread_gravity_opposing_force_N", "hand_gravity_opposing_force_N",
        "thread_summed_normal_force_N", "relative_bolt_axial_velocity_m_per_s",
        "relative_bolt_angular_speed_rad_per_s", "relative_hand_angular_speed_rad_per_s",
        "right_robot_bolt_contact_count", "fully_open_unassisted", "right_grasp_guard_active",
        "right_axial_float_active", "right_applied_axial_feed_N", "right_commanded_aperture_m",
        "desired_independent_clock_rad", "desired_independent_angular_speed_rad_s",
        "open_peak_axial_drift_m", "open_peak_yaw_drift_rad", "weight_window_ready",
        "open_weight_window_ready", "actual_quiet_regrasp_streak_s", "all_hard_guards_held",
        "loaded_actual_interior_flank_contact_count", "interior_flank_gravity_opposing_force_N",
        "interior_flank_summed_normal_force_N", "left_downward_feed_N",
        "right_command_wrench_N_Nm", "left_command_wrench_N_Nm",
        "right_motor_torques_Nm", "left_motor_torques_Nm", "right_actual_aperture_postintegration_m",
        "external_drive_zero", "bolt_world_support_contact_count", "nonthread_block_bolt_contact_count",
        "radial_offset_m", "bolt_tilt_rad", "native_thread_contact_count",
        "weight_observation_valid", "open_observation_valid", "right_grip_slip_m",
        "right_grip_rotation_slip_rad", "left_grip_slip_m", "left_grip_rotation_slip_rad", "cold_table_window_warmup", "robot_yaw_brake_active", "desired_independent_angular_acceleration_rad_s2", "desired_independent_angular_jerk_rad_s3")
    feedback_values = {name: np.asarray([row[i] for row in feedback_rows])
                       for i, name in enumerate(feedback_names)}
    np.savez_compressed(feedback_path, **feedback_values,
        phase_labels_json=np.asarray(json.dumps([p[0] for p in selected])),
        metadata_json=np.asarray(json.dumps({"model_fingerprint": metadata["model_fingerprint"],
            "controller_sha256": metadata["controller_sha256"], "runtime": runtime,
            "timestep_s": model.opt.timestep, "force_timing": metadata["native_force_recording_note"],
            "helper_source_sha256": metadata["feedback_source_sha256"],
            "observer": "supported-original-native-feedback-ledger-v1",
            "scope": "Every original native force/kinematic observation and executed finite motor command. Actual aperture explicitly uses post-integration qpos. No saved-pose force reconstruction."})))
    metadata["native_feedback_force_history"] = {"filename": feedback_path.name,
        "sha256": hashlib.sha256(feedback_path.read_bytes()).hexdigest(),
        "observed_physics_steps": len(feedback_rows), "columns": list(feedback_names),
        "helper_source_sha256": metadata["feedback_source_sha256"]}
    checks = {
        "completed_qualifying_turns": {"passed": len(turns)==control.qualifying_turns},
        "picked_up_free_bolt": {"passed": any(s["phase"]=="transport_bolt" for s in summaries)
            and maximum_support_after_pickup==0 and np.all(minimum_lift_pad_normals>.1),
            "maximum_world_support_after_pickup": maximum_support_after_pickup,
            "sampled_minimum_closed_transport_pad_normals_N": [float(v) if np.isfinite(v) else None
                                                               for v in minimum_lift_pad_normals]},
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
        "left_grasp_retention": {"passed": left_grasp_acquired
            and peak_left_slip<.001 and peak_left_angle<np.deg2rad(2),
                                  "peak_translation_m":peak_left_slip,"peak_rotation_rad":peak_left_angle},
        "left_pad_contact_retention": {"passed": left_grasp_acquired
            and np.all(minimum_loaded_left_pad_normals>.1),
            "sampled_minimum_loaded_pad_normals_N": [float(v) if np.isfinite(v) else None
                                                    for v in minimum_loaded_left_pad_normals]},
        "right_bolt_grasp_retention": {"passed": bool(metadata["right_grasp_acquisitions"])
            and peak_grip_slip<.001
            and peak_right_grip_rotation_slip < np.deg2rad(2),
            "peak_translation_m":peak_grip_slip,
            "peak_rotation_slip_rad": peak_right_grip_rotation_slip,
            "scope": "Within each original bilateral loaded closed grasp; references end on intentional opening and are acquired again after real regrasp"},
        "block_stays_on_table": {"passed": peak_block_lift <= control.maximum_block_lift_m
            and peak_block_translation <= .001 and peak_block_rotation <= np.deg2rad(2)
            and peak_unexpected_table_contacts == 0,
            "maximum_upward_lift_m": peak_block_lift,
            "maximum_translation_from_initial_m": peak_block_translation,
            "maximum_rotation_from_initial_rad": peak_block_rotation,
            "unexpected_world_contact_candidates": peak_unexpected_table_contacts},
        "male_tip_clears_plain_table": {"passed": bool(times)
            and minimum_tip_table_clearance >= scene.minimum_tip_table_clearance_m,
            "minimum_observed_clearance_m": float(minimum_tip_table_clearance)
                if np.isfinite(minimum_tip_table_clearance) else None,
            "minimum_allowed_clearance_m": scene.minimum_tip_table_clearance_m},
        "whole_shaft_clears_rest_before_lateral_transfer": {
            "passed": np.isfinite(minimum_tip_rest_transfer_clearance)
                and minimum_tip_rest_transfer_clearance >= control.minimum_tip_rest_clearance_m,
            "minimum_measured_tip_above_highest_rest_m": float(minimum_tip_rest_transfer_clearance)
                if np.isfinite(minimum_tip_rest_transfer_clearance) else None,
            "minimum_allowed_clearance_m": control.minimum_tip_rest_clearance_m,
            "native_rest_top_world_z_m": rest_top_z,
            "scope": "Lift completion and every native transport substep; no extra body actuation"},
        "native_unexpected_collision_clearance": {"passed": peak_unexpected_native_depth <= 1e-6,
            "peak_native_signed_penetration_m": peak_unexpected_native_depth,
            "native_depth_limit_m": 1e-6,
            "scope": "Every native solved substep; intended table/block, pads/workpieces, thread and pre-pickup bolt-rest contacts are declared separately"},
        "free_objects_have_no_external_drive": {"passed": unforced},
        "native_joint_limits": {"passed": bool(times) and minimum_joint_margin>=-1e-5,
                                "minimum_observed_margin_rad": float(minimum_joint_margin)
                                    if np.isfinite(minimum_joint_margin) else None},
        "no_solver_or_state_abort": {"passed": aborted is None},
    }
    if table_pickup:
        checks["all_substep_left_pad_contact_retention"] = {
            "passed": left_pad_history.report()["continuously_bilateral_loaded"],
            **left_pad_history.report(),
            "scope": "Every physics substep at and after the end of settle_left_block; no lift or physical roll is commanded"}
        checks["both_hands_start_separate_from_workpieces"] = {
            "passed": initial_left_block_contacts["contact_count"] == 0
            and initial_right_bolt_contacts["contact_count"] == 0,
            "initial_left_block_hand_contacts": initial_left_block_contacts["contact_count"],
            "initial_right_bolt_hand_contacts": initial_right_bolt_contacts["contact_count"]}
        checks["table_bears_block_weight_before_bolt_pickup"] = {
            "passed": left_acquisition_time is not None,
            "left_stabilization_time_s": left_acquisition_time,
            "settled_table_load_window": metadata.get("left_acquisition", {}).get("table_load_window"),
            "scope": "Pre-bolt no-lift stabilization: >=90% mean block weight on table, <=10% mean positive hand upward load, >=99% loaded table duty over >=100 ms"}
        checks["all_substep_table_load_retention"] = {
            "passed": active_table_observations > 0 and active_table_unloaded_steps == 0,
            "observed_physics_steps": active_table_observations,
            "minimum_table_upward_force_N": minimum_active_table_force if active_table_observations else None,
            "minimum_loaded_table_upward_force_N": .1*block_weight,
            "unloaded_physics_steps": active_table_unloaded_steps,
            "unloaded_duration_s": active_table_unloaded_steps*model.opt.timestep,
            "maximum_consecutive_unloaded_duration_s": maximum_support_gap,
            "scope": "Every original native substep after stabilization acquisition; brief gaps remain failures"}
        checks["table_bears_weight_throughout_active_task"] = {
            "passed": active_table_observations > 0 and task_table_failed_windows == 0,
            "observed_active_windows": active_table_observations,
            "failed_active_trailing_windows": task_table_failed_windows,
            "trailing_window_s": control.settled_table_window_s,
            "minimum_mean_table_weight_fraction": task_table_minimum_mean_fraction if active_table_observations else None,
            "maximum_mean_positive_hand_upward_weight_fraction": task_hand_maximum_mean_positive_fraction,
            "minimum_loaded_table_duty": task_table_minimum_loaded_duty,
            "final_window": task_table_window.report(),
            "scope": "Every active trailing original-force window, including pickup/entry/turn/reset/regrasp; same90%table/10%positive-hand-up/99%duty thresholds as stabilization, without hiding transient load sharing"}
    if entry_support_enabled:
        checks["native_cone_entry_acquired_before_rotation"] = {
            "passed": entry_acquisition_time is not None, "acquisition_time_s": entry_acquisition_time,
            "scope": "Settled native lead-in load only; does not qualify formed-flank capture"}
        checks["native_starting_support_before_searching_open"] = {
            "passed": entry_support_timeout is None and entry_acquisition_time is not None,
            "maximum_dwell_s": control.maximum_entry_dwell_s, "failed_acquisition": entry_support_timeout,
            "scope": "Every searching release/open reset must have measured settled native thread-pair support; pre-release acquisition timeout retains closed jaws"}
    checks = {k:{**v,"passed":bool(v["passed"])} for k,v in checks.items()}
    result = {**metadata,"partial":len(selected)<len(phases),"passed":all(v["passed"] for v in checks.values()),
              "phases":summaries,"acceptance_checks":checks,"aborted":aborted,
              "wall_seconds":time.perf_counter()-start_wall,
              "all_substep_left_pad_loads": left_pad_history.report() if table_pickup else None,
              "legacy_continuous_loaded_force_criterion": {"would_have_tagged_engagement":legacy_engagement_time is not None,
                                                           "first_pass_time_s":legacy_engagement_time,
                                                           "required_streak_s":control.full_flank_contact_streak_s},
              "reported_sdf_depth_note":"Native contact dist proxy; not an independent overlap certificate"}
    result["passed"] = False
    result["partial"] = True
    result["diagnostic_completed"] = bool(aborted is None and len(summaries) == 5
        and summaries[-1]["phase"] == "stop_start_1"
        and summaries[-1]["live_physical_phase_gate"] == "complete"
        and entry_support.ready and weight_window.ready)
    result["diagnostic_final_native_entry_support"] = entry_support.report()
    result["diagnostic_final_native_weight_support"] = weight_window.report()
    result["full_fresh_trajectory_qualified"] = False
    result["capture_or_open_reset_qualified"] = False
    result["final_experimental_crest_direction_event"] = seat_event.report()
    result["diagnostic_acceptance_scope"] = "Only this new cold branch's observed native guards, closed direction/first-forward motion and original solved forces. Legacy whole-task acceptance rows remain explicitly incomplete; inherited acquisition references are not fresh prefix evidence. No captured thread/open reset/policy/full trajectory qualification."
    serialized_result = json.dumps(result, allow_nan=False)
    np.savez_compressed(output / "insertion_trace.npz", time=times, qpos=positions,
        qvel=velocities, controller=controls, info_json=np.asarray(json.dumps(rows)),
        metadata_json=np.asarray(serialized_result))
    (output / "insertion_validation.json").write_text(json.dumps(result,indent=2,allow_nan=False))
    return result


def _sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024*1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    import argparse
    import subprocess
    from .m8_supported_scene import SupportedConfig, scene_xml
    from .m8_scene import YamM8Config
    from thread_lab.model import ThreadConfig
    from scripts.audit_m8_insertion_trace import recorded_model
    global _COLD_CONTEXT
    parser = argparse.ArgumentParser(description="Separate cold closed crest-return diagnostic; never a full rollout splice")
    parser.add_argument("--parent", type=Path, required=True)
    parser.add_argument("--checkpoint-time", type=float, default=13.1949)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--prepare-only", action="store_true")
    parser.add_argument("--brake-duration", type=float, default=.15)
    args = parser.parse_args()
    if not np.isfinite(args.brake_duration) or args.brake_duration <= 0:
        raise ValueError("Brake duration must be finite and positive")
    if args.output.exists():
        raise FileExistsError("Never overwrite a closed or active native diagnostic")
    report_path = args.parent.parent/"insertion_validation.json"
    original = json.loads(report_path.read_text())
    original_after_path = args.parent.parent/"run_publication_identity_after.json"
    original_after = json.loads(original_after_path.read_text())
    if (original_after["producer_commit"] != "6e7d0d2ac3d28ff2538e122a10d7ffb2febf83b1"
            or not original_after["source_hashes_unchanged"]
            or original_after["native_exit_code"] != 1
            or original["controller_module_sha256"] != "57f430143787317f8dc6f50bd2ea1ff23aab123810284da74da70b53b45a9493"):
        raise ValueError("Cold parent is not the immutable declared first continuous failure")
    root = Path(__file__).resolve().parents[3]
    before = {name:_sha(root/name) for name in original_after["source_hashes_before"]}
    if len(before) != 65 or before != original_after["source_hashes_before"]:
        raise ValueError("The original 65 frozen source hashes changed")
    observer_path = Path(inspect.getfile(CrestSeatDropWindowV3))
    source_sha, observer_sha = _sha(__file__), _sha(observer_path)
    brake_path = Path(inspect.getfile(C2ReverseBrake))
    brake_sha = _sha(brake_path)
    with np.load(args.parent, allow_pickle=False) as archive:
        metadata = json.loads(str(archive["metadata_json"].item()))
        times = archive["time"]
        candidates = np.flatnonzero(np.abs(times-args.checkpoint_time) <= 1e-9)
        if len(candidates) != 1:
            raise ValueError("Cold checkpoint must be exactly one archived native post-integration row")
        index = int(candidates[0])
        native_time = float(times[index])
        qpos, qvel, ctrl = (archive[key][index].copy() for key in ("qpos", "qvel", "controller"))
    if metadata["controller_module_sha256"] != original["controller_module_sha256"]:
        raise ValueError("Parent trace and validation controller identity disagree")
    reference = next(event for event in original["physical_motion_events"]
                     if event["event"] == "Actual post-ramp settled seat reference")
    if abs(reference["time_s"]-native_time) > 1e-9 or reference["stable_actual_quiet_streak_s"] < .1-1e-12:
        raise ValueError("Chosen checkpoint is not the exact actually settled reference event")
    inputs = {name:_sha(args.parent.parent/name) for name in (
        "insertion_trace.npz", "insertion_validation.json", "scene.xml", "supported_scene.zip",
        "table_support_force_history.npz", "native_feedback_force_history.npz",
        "run_publication_identity.json", "run_publication_identity_after.json")}
    if inputs["scene.xml"] != original["model_xml_sha256"]:
        raise ValueError("Archived original scene XML does not match its native report")
    with np.load(args.parent.parent/"table_support_force_history.npz", allow_pickle=False) as history:
        jj = np.flatnonzero(np.abs(history["time"]-native_time) <= 1e-9)
        if len(jj) != 1 or abs(float(history["bolt_base_insertion_m"][jj[0]])-reference["base_z_m"]) > 1e-12:
            raise ValueError("Settled reference is not bound to its original native force/pose ledger")
    with np.load(args.parent.parent/"native_feedback_force_history.npz", allow_pickle=False) as history:
        jj = np.flatnonzero(np.abs(history["time"]-native_time) <= 1e-9)
        if len(jj) != 1 or not bool(history["all_hard_guards_held"][jj[0]]):
            raise ValueError("Settled reference lacks its original native hard-guard proof")
    def tuple_lists(value):
        if isinstance(value, list):
            return tuple(tuple_lists(v) for v in value)
        if isinstance(value, dict):
            return {k:tuple_lists(v) for k,v in value.items()}
        return value
    scene_values = dict(tuple_lists(original["scene_config"]))
    base_values = dict(scene_values.pop("base"))
    base_values["thread"] = ThreadConfig(**base_values["thread"])
    scene = SupportedConfig(base=YamM8Config(**base_values), **scene_values)
    control_values = dict(original["control_config"])
    control_values["arm"] = YamM8ControlConfig(**control_values["arm"])
    control = SupportedControlConfig(**control_values)
    if hashlib.sha256(scene_xml(scene).encode()).hexdigest() != original["model_xml_sha256"]:
        raise ValueError("Decoded configuration differs from the archived XML")
    runtime = require_micron_engine()
    model, model_identity = recorded_model(args.parent, original)
    if not (qpos.shape == (model.nq,) and qvel.shape == (model.nv,) and ctrl.shape == (model.nu,)
            and np.isfinite(qpos).all() and np.isfinite(qvel).all() and np.isfinite(ctrl).all()):
        raise ValueError("Cold native state is not finite/aligned with exact archived model")
    yaw_anchor = original["initial_bolt_yaw_rad"]
    for phase in original["phases"]:
        if phase["phase"] == "transfer_bolt_weight":
            break
        yaw_anchor += phase["bolt_clockwise_rotation_rad"]
    declaration = {
        "scope":"Separate new cold diagnostic from exact archived post-integration checkpoint, without solver warm-start or original force-window reuse. Never splice into a full trajectory.",
        "parent":str(args.parent.resolve()), "parent_input_sha256":inputs,
        "parent_native_checkpoint_time_s":native_time, "parent_native_checkpoint_index":index,
        "parent_original_settled_reference_event":reference,
        "qpos_sha256":hashlib.sha256(qpos.tobytes()).hexdigest(),
        "qvel_sha256":hashlib.sha256(qvel.tobytes()).hexdigest(),
        "ctrl_sha256":hashlib.sha256(ctrl.tobytes()).hexdigest(),
        "cold_time_origin_s":0., "native_warm_start_copied":False,
        "source_65_before":before, "producer_commit":original_after["producer_commit"],
        "execution_repository_head":subprocess.check_output(["git","rev-parse","HEAD"],cwd=root,text=True).strip(),
        "controller_source_sha256":source_sha, "crest_observer_source_sha256":observer_sha,
        "robot_yaw_brake_source_sha256":brake_sha, "robot_yaw_brake_duration_s":args.brake_duration,
        "archived_model_identity":model_identity, "runtime":runtime,
        "original_closed_yaw_anchor_rad":yaw_anchor,
        "original_independent_clock_anchor_rad":reference["desired_clock_rad"],
        "physical_change":"Same V3 measured-crest-return observer, original cold input and hard guards. Replace abrupt yaw-stop with explicit C2 CLOSED robot-yaw braking from independently scheduled preceding theta/omega/alpha, then original750ms stop observation bound. Actual persistent return and fresh100ms native impulse/angular/axial quiet PLUS unchanged100ms90/10 native weight are required before first closed forward. No object/pitch/helix drive or state write after initialization.",
    }
    if args.prepare_only:
        print(json.dumps({"prepared":True,"declaration":declaration},indent=2,allow_nan=False))
        return
    args.output.mkdir(parents=True)
    (args.output/"diagnostic_declaration.json").write_text(json.dumps(declaration,indent=2,allow_nan=False))
    (args.output/"original_controller_source.py").write_bytes((args.parent.parent/"controller_source.py").read_bytes())
    (args.output/"experimental_observer_source.py").write_bytes(observer_path.read_bytes())
    (args.output/"robot_yaw_brake_source.py").write_bytes(brake_path.read_bytes())
    (args.output/"diagnostic_source.py").write_bytes(Path(__file__).read_bytes())
    (args.output/"supported_scene.zip").write_bytes((args.parent.parent/"supported_scene.zip").read_bytes())
    _COLD_CONTEXT = {"model":model,"qpos":qpos,"qvel":qvel,"ctrl":ctrl,
        "left_acquisition":original["left_acquisition"],
        "right_acquisition":original["right_grasp_acquisitions"][0],
        "desired_clock_rad":reference["desired_clock_rad"],
        "closed_yaw_anchor_rad":yaw_anchor,"settled_base_z_m":reference["base_z_m"],
        "observer_sha256":observer_sha,"declaration":declaration,
        "brake_sha256":brake_sha, "brake_duration_s":args.brake_duration}
    result = run_supported_demo(args.output, scene_config=scene, control_config=control)
    after = {name:_sha(root/name) for name in before}
    closure = {"source_65_after":after,"source_65_unchanged":after==before,
        "harness_sha256_before":source_sha,"harness_sha256_after":_sha(__file__),
        "observer_sha256_before":observer_sha,"observer_sha256_after":_sha(observer_path),
        "brake_sha256_before":brake_sha,"brake_sha256_after":_sha(brake_path),
        "parent_input_sha256_before":inputs,
        "parent_input_sha256_after":{name:_sha(args.parent.parent/name) for name in inputs},
        "diagnostic_completed":result["diagnostic_completed"],
        "full_fresh_trajectory_qualified":False,"capture_or_open_reset_qualified":False}
    (args.output/"diagnostic_execution_after.json").write_text(json.dumps(closure,indent=2,allow_nan=False))
    print(json.dumps({"diagnostic_completed":result["diagnostic_completed"],"aborted":result["aborted"],
                      "final_crest_event":result["final_experimental_crest_direction_event"]},indent=2,allow_nan=False))
    if (after != before or source_sha != _sha(__file__) or observer_sha != _sha(observer_path)
            or brake_sha != _sha(brake_path)
            or closure["parent_input_sha256_after"] != inputs):
        raise RuntimeError("Immutable execution or parent source changed")
    raise SystemExit(0 if result["diagnostic_completed"] else 1)


if __name__ == "__main__":
    main()
