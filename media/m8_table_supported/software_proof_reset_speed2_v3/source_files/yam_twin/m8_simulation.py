"""Contact-driven M8 demonstration actuated through the actual YAM joints.

The nut starts engaged on a fixture-mounted bolt. Arm motors and native finger
motors are the only controls. During closed turns the hand has no axial spring
or damper: a small constant axial load is mapped through the arm Jacobian.
Measured nut motion is used for diagnostics, never for a motion command.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import inspect
import json
from pathlib import Path
import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from thread_lab.model import ThreadConfig
from thread_lab.runtime import require_micron_engine
from .kinematics import ArmIK, HOME


@dataclass(frozen=True)
class YamM8ControlConfig:
    position_stiffness: float = 20000.
    position_damping: float = 650.
    rotation_stiffness: float = 40.
    rotation_damping: float = 6.
    left_rotation_stiffness: float = 80.
    left_rotation_damping: float = 10.
    maximum_cartesian_force: float = 8.
    maximum_cartesian_torque: float = 2.
    axial_feed_N: float = -.05
    open_aperture: float = .024
    closed_aperture: float = .0194
    left_closed_aperture: float = .0144
    stroke_angle_rad: float = 2 * np.pi / 3
    angular_speed_rad_s: float = 1.
    reset_speed_rad_s: float = 4.
    strokes: int = 3
    sample_period_s: float = .005

    def __post_init__(self):
        if any(not np.isfinite(v) for v in asdict(self).values()):
            raise ValueError("Controller settings must be finite")
        if not .010 <= min(self.closed_aperture, self.left_closed_aperture) <= max(
                self.closed_aperture, self.left_closed_aperture) <= self.open_aperture <= .040:
            raise ValueError("Invalid jaw aperture")
        if self.strokes < 1 or int(self.strokes) != self.strokes:
            raise ValueError("Strokes must be a positive integer")
        for field in ("position_stiffness", "position_damping", "rotation_stiffness",
                      "rotation_damping", "left_rotation_stiffness", "left_rotation_damping", "maximum_cartesian_force",
                      "maximum_cartesian_torque", "stroke_angle_rad",
                      "angular_speed_rad_s", "reset_speed_rad_s", "sample_period_s"):
            if getattr(self, field) <= 0:
                raise ValueError(f"{field} must be positive")


def bounded_vector(value, limit):
    value = np.asarray(value, dtype=float).copy()
    size = float(np.linalg.norm(value))
    return value * min(1., limit / size) if size else value


def smooth_profile(u):
    """C2 quintic interpolation: endpoint velocity and acceleration are zero."""
    u = float(np.clip(u, 0., 1.))
    return u ** 3 * (10. + u * (-15. + 6. * u)), 30. * u ** 2 * (1. - u) ** 2


class YamCartesianController:
    """Finite arm-joint torques, with world-oriented site impedance.

    Bias compensation is applied to arm DOFs only and included inside each
    motor's torque cap. The controller neither reads nor writes nut state.
    """

    def __init__(self, model, data, side="right", config=None, scene_config=None):
        from .m8_scene import YamM8Config
        self.model, self.data = model, data
        self.side, self.config = side, config or YamM8ControlConfig()
        self.scene_config = scene_config or YamM8Config()
        self.site_id = model.site(f"{side}_grasp_site").id
        self.joint_ids = [model.joint(f"{side}_joint{i}").id for i in range(1, 7)]
        self.qpos_indices = model.jnt_qposadr[self.joint_ids]
        self.dof_indices = model.jnt_dofadr[self.joint_ids]
        self.motor_ids = [model.actuator(f"{side}_servo{i}").id for i in range(1, 7)]
        self.torque_caps = np.minimum(np.abs(model.actuator_ctrlrange[self.motor_ids, 0]),
                                      np.abs(model.actuator_ctrlrange[self.motor_ids, 1]))
        self.finger_ids = [model.actuator(f"{side}_grip_{s}").id for s in ("left", "right")]
        self.pad_geom_ids = frozenset(model.geom(f"{side}_m8_pad_{s}").id for s in ("left", "right"))
        self.hand_geom_ids = frozenset(int(i) for i, bid in enumerate(model.geom_bodyid)
            if model.body(int(bid)).name.startswith(f"{side}_")
            and any(word in model.body(int(bid)).name for word in
                    ("link_6", "finger", "lf_", "rf_", "camera")))
        self.jp = np.zeros((3, model.nv))
        self.jr = np.zeros((3, model.nv))
        self.last_wrench = np.zeros(6)
        self.last_motor_torques = np.zeros(6)
        self.last_position_error = np.zeros(3)
        self.last_rotation_error = np.zeros(3)

    def pose(self):
        return (self.data.site_xpos[self.site_id].copy(),
                self.data.site_xmat[self.site_id].reshape(3, 3).copy())

    def command(self, position, orientation, aperture, *, linear_velocity=None,
                angular_velocity=None, axial_float=False, axis_world=(0., 0., 1.),
                axial_feed_N=None):
        cfg = self.config
        mujoco.mj_jacSite(self.model, self.data, self.jp, self.jr, self.site_id)
        jp, jr = self.jp[:, self.dof_indices], self.jr[:, self.dof_indices]
        joint_velocity = self.data.qvel[self.dof_indices]
        p, r = self.pose()
        axis = np.asarray(axis_world, dtype=float)
        axis = axis / np.linalg.norm(axis)
        projection = np.eye(3) - np.outer(axis, axis) if axial_float else np.eye(3)
        target_v = np.zeros(3) if linear_velocity is None else np.asarray(linear_velocity)
        target_w = np.zeros(3) if angular_velocity is None else np.asarray(angular_velocity)
        self.last_position_error = np.asarray(position) - p
        self.last_rotation_error = Rotation.from_matrix(np.asarray(orientation) @ r.T).as_rotvec()
        force = projection @ (cfg.position_stiffness * self.last_position_error
                              + cfg.position_damping * (target_v - jp @ joint_velocity))
        if axial_float:
            feed = cfg.axial_feed_N if axial_feed_N is None else float(axial_feed_N)
            force += feed * axis
        kr = cfg.left_rotation_stiffness if self.side == "left" else cfg.rotation_stiffness
        dr = cfg.left_rotation_damping if self.side == "left" else cfg.rotation_damping
        torque = kr * self.last_rotation_error
        torque += dr * (target_w - jr @ joint_velocity)
        force = bounded_vector(force, cfg.maximum_cartesian_force)
        torque = bounded_vector(torque, cfg.maximum_cartesian_torque)
        self.last_wrench[:] = np.r_[force, torque]
        # Feed forward native arm viscous drag through the same finite motors.
        # qfrc_bias excludes passive damping. Compensate that known linear term
        # only; joint-limit, spring, and contact forces remain physical.
        damping_compensation = self.model.dof_damping[self.dof_indices] * joint_velocity
        arm_torques = (self.data.qfrc_bias[self.dof_indices] + damping_compensation
                       + jp.T @ force + jr.T @ torque)
        self.last_motor_torques[:] = np.clip(arm_torques, -self.torque_caps, self.torque_caps)
        self.data.ctrl[self.motor_ids] = self.last_motor_torques
        # Native positive-left / negative-right finger coordinates OPEN the jaw.
        from .m8_scene import jaw_positions
        finger_targets = jaw_positions(aperture, self.scene_config)
        limits = self.model.actuator_ctrlrange[self.finger_ids]
        self.data.ctrl[self.finger_ids] = np.clip(finger_targets, limits[:, 0], limits[:, 1])
        return self.last_motor_torques.copy()

    def contact_wrench_on(self, body_name="nut"):
        body_id = self.model.body(body_name).id
        total, pad_total = np.zeros(6), np.zeros(6)
        counts = [0, 0]
        normal = {g: 0. for g in self.pad_geom_ids}
        contact_force = np.zeros(6)
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            b1, b2 = int(self.model.geom_bodyid[g1]), int(self.model.geom_bodyid[g2])
            if b1 == body_id and g2 in self.hand_geom_ids:
                sign, hand_geom = -1, g2
            elif b2 == body_id and g1 in self.hand_geom_ids:
                sign, hand_geom = 1, g1
            else:
                continue
            mujoco.mj_contactForce(self.model, self.data, i, contact_force)
            frame = c.frame.reshape(3, 3)
            force = sign * frame.T @ contact_force[:3]
            torque = sign * frame.T @ contact_force[3:]
            wrench = np.r_[force, torque + np.cross(c.pos - self.data.xpos[body_id], force)]
            total += wrench
            counts[0] += 1
            if hand_geom in self.pad_geom_ids:
                pad_total += wrench
                normal[hand_geom] += float(contact_force[0])
                counts[1] += 1
        pad_names = [f"{self.side}_m8_pad_{s}" for s in ("left", "right")]
        return {"contact_count": counts[0], "pad_contact_count": counts[1],
                "wrench_world": total.tolist(), "pad_wrench_world": pad_total.tolist(),
                "pad_normal_force_N": [normal[self.model.geom(n).id] for n in pad_names],
                "jaw_actuator_force": self.data.actuator_force[self.finger_ids].tolist(),
                "arm_motor_torques_Nm": self.last_motor_torques.tolist(),
                "hand_wrench_world": self.last_wrench.tolist()}


def demo_phases(config):
    """Open-loop hand motion schedule. Hex-compatible strokes permit regrasp."""
    stroke = config.stroke_angle_rad
    turn_time = 1.875 * stroke / config.angular_speed_rad_s
    reset_time = 1.875 * stroke / config.reset_speed_rad_s
    phases = [("close", .25, 0., 0., config.open_aperture, config.closed_aperture, True),
              ("settle", .12, 0., 0., config.closed_aperture, config.closed_aperture, True),
              ("lift", .75, 0., 0., config.closed_aperture, config.closed_aperture, True),
              ("hold_lift", .20, 0., 0., config.closed_aperture, config.closed_aperture, True)]
    for i in range(config.strokes):
        phases.extend([(f"turn_{i + 1}", turn_time, 0., -stroke,
                        config.closed_aperture, config.closed_aperture, True),
                       (f"stop_{i + 1}", .12, -stroke, -stroke,
                        config.closed_aperture, config.closed_aperture, True)])
        if i + 1 < config.strokes:
            phases.extend([(f"release_{i + 1}", .15, -stroke, -stroke,
                            config.closed_aperture, config.open_aperture, False),
                           (f"open_settle_{i + 1}", .08, -stroke, -stroke,
                            config.open_aperture, config.open_aperture, False),
                           (f"reset_open_{i + 1}", reset_time, -stroke, 0.,
                            config.open_aperture, config.open_aperture, False),
                           (f"open_hold_{i + 1}", .08, 0., 0.,
                            config.open_aperture, config.open_aperture, False),
                           (f"regrip_{i + 1}", .25, 0., 0.,
                            config.open_aperture, config.closed_aperture, False),
                           (f"settle_regrip_{i + 1}", .12, 0., 0.,
                            config.closed_aperture, config.closed_aperture, True)])
    return phases


def initialize_work_pose(model, data, *, control_config=None, scene_config=None):
    """Initialize arm/finger state around the scene's unmodified engaged nut.

    This helper is for reset only. It never assigns nut state and does not apply
    forces. Returned poses are world-oriented site targets, suitable for a
    separate controller or an independent policy environment.
    """
    from .m8_scene import YamM8Config, jaw_positions, left_touch_aperture, left_grasp_rotation
    control = control_config or YamM8ControlConfig()
    scene = scene_config or YamM8Config()
    for side in ("left", "right"):
        ids = [model.joint(f"{side}_joint{i}").id for i in range(1, 7)]
        data.qpos[model.jnt_qposadr[ids]] = HOME
        # The block starts between touching left pads; actual finite finger
        # actuators acquire compression when integration begins. No free-body
        # state is assigned here, and no grasp constraint supplies preload.
        aperture = left_touch_aperture(scene) if side == "left" else control.open_aperture
        requested = control.left_closed_aperture if side == "left" else control.open_aperture
        for finger, q, command in zip(("left", "right"), jaw_positions(aperture, scene), jaw_positions(requested, scene)):
            data.qpos[model.joint(f"{side}_{finger}_finger").qposadr[0]] = q
            data.ctrl[model.actuator(f"{side}_grip_{finger}").id] = command
    mujoco.mj_forward(model, data)
    bolt_r = data.xmat[model.body("bolt_frame").id].reshape(3, 3).copy()
    center = data.xpos[model.body("nut").id].copy()
    initial_r = (bolt_r @ Rotation.from_euler("z", -90, degrees=True).as_matrix()
                 @ Rotation.from_euler("x", 180, degrees=True).as_matrix())
    block = model.body("fixture_block").id
    block_r = data.xmat[block].reshape(3, 3)
    left_p = data.xpos[block] + block_r @ np.asarray(scene.left_grasp_offset)
    left_r = left_grasp_rotation(scene)
    for side, target_p, target_r in (("left", left_p, left_r), ("right", center, initial_r)):
        ik = ArmIK(model, side)
        ik.solve(target_p, target_r, thorough=True)
        data.qpos[ik.qadr] = ik.q
    mujoco.mj_forward(model, data)
    poses = {}
    for side in ("left", "right"):
        sid = model.site(f"{side}_grasp_site").id
        poses[side] = (data.site_xpos[sid].copy(), data.site_xmat[sid].reshape(3, 3).copy())
    return poses


def run_demo(output="outputs/yam_m8", *, config=None, scene_config=None,
             control_config=None, maximum_phases=None):
    """Integrate genuine YAM joint actuation and save the observed trajectory."""
    from dataclasses import replace
    from .m8_scene import build_model, scene_xml, scene_fingerprint, YamM8Config
    runtime = require_micron_engine()
    scene = scene_config or YamM8Config()
    if config is not None:
        scene = replace(scene, thread=config)
    thread = scene.thread
    control = control_config or YamM8ControlConfig()
    model = build_model(scene)
    data = mujoco.MjData(model)
    ctrls = {side: YamCartesianController(model, data, side, control, scene) for side in ("left", "right")}
    # State assignments occur solely in initialization. The nut retains the
    # scene's pre-engaged free-body state; neither its pose nor velocity is set.
    initial_poses = initialize_work_pose(model, data, control_config=control, scene_config=scene)
    nut_id, bolt_id = model.body("nut").id, model.body("bolt_frame").id
    block_id = model.body("fixture_block").id
    block_jid = model.joint("fixture_block_free").id
    block_dof = int(model.jnt_dofadr[block_jid])
    nut_jid = model.joint("nut_free").id
    nut_dof = int(model.jnt_dofadr[nut_jid])
    bolt_origin, bolt_r = data.xpos[bolt_id].copy(), data.xmat[bolt_id].reshape(3, 3).copy()
    axis = bolt_r[:, 2]
    grip_center = data.xpos[nut_id].copy()
    initial_r = initial_poses["right"][1]
    initial_r_relative = bolt_r.T @ initial_r
    ik = ArmIK(model, "right")
    ik.q = data.qpos[ctrls["right"].qpos_indices].copy()
    left_pose = initial_poses["left"]
    block_initial_position = data.xpos[block_id].copy()
    block_grip_initial_p = left_pose[1].T @ (data.xpos[block_id] - left_pose[0])
    block_grip_initial_r = left_pose[1].T @ data.xmat[block_id].reshape(3, 3)
    # Fail before dynamics if the complete wrist stroke is outside native limits.
    for theta in np.linspace(0., -control.stroke_angle_rad, 13):
        r = Rotation.from_rotvec(axis * theta).as_matrix() @ initial_r
        ik.solve(grip_center, r, thorough=True)
    phases = demo_phases(control)
    if maximum_phases is not None:
        phases = phases[:maximum_phases]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    wall_start = time.perf_counter()
    records, poses, velocities, motor_controls, times, summaries = [], [], [], [], [], []
    last_yaw, yaw_total = 0., 0.
    initial_rel = bolt_r.T @ (data.xpos[nut_id] - bolt_origin)
    initial_z = float(initial_rel[2])
    peak_depth = peak_radial = peak_tilt = 0.
    nut_unforced = block_unforced = True
    peak_block_slip = peak_block_angle = 0.
    maximum_support_contacts = 0
    minimum_loaded_left_normals = np.full(2, np.inf)
    aborted = None
    sample_steps = max(1, round(control.sample_period_s / model.opt.timestep))
    metadata = {"thread_config": thread.as_dict(), "scene_config": asdict(scene),
                "control_config": asdict(control), "runtime": runtime,
                "starts_preengaged": True, "partial": True,
                "starts_grasp_ready": True, "block_supported_by_world": False,
                "controller": "Actual YAM joint torque impedance; floating hand axial DOF during closed turns",
                "initial_z_m": initial_z,
                "model_fingerprint": scene_fingerprint(scene),
                "model_xml_sha256": hashlib.sha256(scene_xml(scene).encode()).hexdigest(),
                "controller_sha256": hashlib.sha256("\n".join(inspect.getsource(value) for value in
                    (YamM8ControlConfig, YamCartesianController, bounded_vector, smooth_profile,
                     demo_phases, initialize_work_pose, run_demo)).encode()).hexdigest()}
    bolt_geom, nut_geom = model.geom("bolt_thread").id, model.geom("nut_thread").id
    right = ctrls["right"]
    lift_offset = 0.
    bolt_velocity = np.zeros(6)
    for label, duration, angle0, angle1, gap0, gap1, axial_float in phases:
        phase_start_z = float((bolt_r.T @ (data.xpos[nut_id] - bolt_origin))[2])
        phase_start_yaw = yaw_total
        held_position = right.pose()[0]
        held_relative_z = float(np.dot(held_position - data.xpos[bolt_id],
                                      data.xmat[bolt_id].reshape(3, 3)[:, 2]))
        phase_max_contacts = 0
        phase_peak_pad_torque = 0.
        steps = round(duration / model.opt.timestep)
        for step in range(steps):
            bolt_origin = data.xpos[bolt_id].copy()
            bolt_r = data.xmat[bolt_id].reshape(3, 3).copy()
            axis = bolt_r[:, 2]
            u = (step + 1) / steps
            blend, deriv = smooth_profile(u)
            theta = angle0 + blend * (angle1 - angle0)
            omega = deriv * (angle1 - angle0) / duration
            target_r = bolt_r @ Rotation.from_euler("z", theta).as_matrix() @ initial_r_relative
            target_p = bolt_origin + axis * held_relative_z
            # Open phases capture z from the hand itself; closed phases project
            # the entire axial spring and damper out of the Cartesian wrench.
            aperture = gap0 + blend * (gap1 - gap0)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY, bolt_id, bolt_velocity, 0)
            right.command(target_p, target_r, aperture,
                          linear_velocity=bolt_velocity[3:] + np.cross(bolt_velocity[:3], target_p - bolt_origin),
                          angular_velocity=bolt_velocity[:3] + axis * omega,
                          axial_float=axial_float, axis_world=axis,
                          axial_feed_N=0. if label == "close" else control.axial_feed_N)
            if label == "lift":
                lift_offset = .004 * blend
                lift_speed = .004 * deriv / duration
            else:
                lift_speed = 0.
            ctrls["left"].command(left_pose[0] + [0., 0., lift_offset], left_pose[1],
                                  control.left_closed_aperture, linear_velocity=[0., 0., lift_speed])
            mujoco.mj_step(model, data)
            bolt_origin = data.xpos[bolt_id].copy()
            bolt_r = data.xmat[bolt_id].reshape(3, 3).copy()
            axis = bolt_r[:, 2]
            rel = bolt_r.T @ (data.xpos[nut_id] - bolt_origin)
            rel_r = bolt_r.T @ data.xmat[nut_id].reshape(3, 3)
            yaw = float(np.arctan2(rel_r[1, 0], rel_r[0, 0]))
            yaw_total += float((yaw - last_yaw + np.pi) % (2 * np.pi) - np.pi)
            last_yaw = yaw
            radial = float(np.linalg.norm(rel[:2]))
            tilt = float(np.arccos(np.clip(rel_r[2, 2], -1., 1.)))
            depth = 0.
            hand_nut_contacts = 0
            support_contacts = 0
            left_p, left_r = ctrls["left"].pose()
            block_rel_p = left_r.T @ (data.xpos[block_id] - left_p)
            block_rel_r = left_r.T @ data.xmat[block_id].reshape(3, 3)
            block_slip = float(np.linalg.norm(block_rel_p - block_grip_initial_p))
            block_angle = float(np.linalg.norm(Rotation.from_matrix(block_rel_r @ block_grip_initial_r.T).as_rotvec()))
            peak_block_slip = max(peak_block_slip, block_slip)
            peak_block_angle = max(peak_block_angle, block_angle)
            for ci in range(data.ncon):
                c = data.contact[ci]
                geoms = {int(c.geom1), int(c.geom2)}
                if geoms == {bolt_geom, nut_geom}:
                    depth = max(depth, -float(c.dist))
                if nut_geom in geoms and any(g in right.hand_geom_ids for g in geoms):
                    hand_nut_contacts += 1
                bodies = [int(model.geom_bodyid[int(c.geom1)]), int(model.geom_bodyid[int(c.geom2)])]
                if 0 in bodies and (block_id in bodies or bolt_id in bodies):
                    support_contacts += 1
            maximum_support_contacts = max(maximum_support_contacts, support_contacts)
            peak_depth, peak_radial, peak_tilt = max(peak_depth, depth), max(peak_radial, radial), max(peak_tilt, tilt)
            phase_max_contacts = max(phase_max_contacts, hand_nut_contacts)
            nut_unforced = nut_unforced and bool(np.all(data.xfrc_applied[nut_id] == 0)
                                                and np.all(data.qfrc_applied[nut_dof:nut_dof + 6] == 0))
            block_unforced = block_unforced and bool(np.all(data.xfrc_applied[[block_id, bolt_id]] == 0)
                                                and np.all(data.qfrc_applied[block_dof:block_dof + 6] == 0))
            warnings = int(sum(w.number for w in data.warning))
            residual = initial_z - float(rel[2]) + thread.pitch * yaw_total / (2 * np.pi)
            if (depth > 10e-6 or radial > 150e-6 or tilt > np.deg2rad(2.)
                    or abs(residual) > 150e-6 or warnings or not nut_unforced or not block_unforced
                    or block_slip > .001 or block_angle > np.deg2rad(2.) or support_contacts
                    or not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all()):
                aborted = {"phase": label, "time": float(data.time), "reported_sdf_depth_m": depth,
                           "radial_offset_m": radial, "nut_tilt_rad": tilt,
                           "helix_residual_m": residual, "warnings": warnings,
                           "nut_external_drive_zero": nut_unforced, "block_external_drive_zero": block_unforced,
                           "block_grip_slip_m": block_slip, "block_grip_rotation_slip_rad": block_angle,
                           "world_support_contacts": support_contacts}
            if step % sample_steps == 0 or step == steps - 1 or aborted:
                contact = right.contact_wrench_on()
                left_contact = ctrls["left"].contact_wrench_on("fixture_block")
                if label != "close":
                    minimum_loaded_left_normals = np.minimum(minimum_loaded_left_normals,
                                                              left_contact["pad_normal_force_N"])
                phase_peak_pad_torque = max(phase_peak_pad_torque, abs(float(np.dot(contact["pad_wrench_world"][3:], axis))))
                hand_p, hand_r = right.pose()
                row = {"time": float(data.time), "phase": label, "nut_z": float(rel[2]),
                       "nut_yaw_unwrapped": yaw_total, "axial_advance_mm": (initial_z - float(rel[2])) * 1000,
                       "clockwise_turns": -yaw_total / (2 * np.pi), "radial_offset_m": radial,
                       "nut_tilt_rad": tilt, "reported_sdf_depth_m": depth,
                       "hand_z": float(np.dot(hand_p - bolt_origin, axis)),
                       "hand_yaw": float(np.arctan2((bolt_r.T @ hand_r)[1, 0], (bolt_r.T @ hand_r)[0, 0])),
                       "helix_residual_m": residual, "contact": contact,
                       "left_contact": left_contact, "block_grip_slip_m": block_slip,
                       "block_grip_rotation_slip_rad": block_angle, "world_support_contacts": support_contacts,
                       "block_world_position": data.xpos[block_id].copy().tolist(),
                       "block_lift_m": float(data.xpos[block_id, 2] - block_initial_position[2]),
                       "nut_external_drive_zero": nut_unforced, "block_external_drive_zero": block_unforced,
                       "right_position_error_m": right.last_position_error.tolist(),
                       "right_rotation_error_rad": right.last_rotation_error.tolist()}
                records.append(row)
                times.append(float(data.time))
                poses.append(data.qpos.copy())
                velocities.append(data.qvel.copy())
                motor_controls.append(data.ctrl.copy())
            if aborted:
                break
        end_z = float((bolt_r.T @ (data.xpos[nut_id] - bolt_origin))[2])
        rotation = yaw_total - phase_start_yaw
        advance = phase_start_z - end_z
        summary = {"phase": label, "axial_advance_m": advance, "nut_rotation_rad": rotation,
                   "observed_helix_residual_m": advance + thread.pitch * rotation / (2 * np.pi),
                   "maximum_hand_nut_contacts": phase_max_contacts,
                   "sampled_peak_pad_contact_torque_Nm": phase_peak_pad_torque}
        summaries.append(summary)
        print(json.dumps(summary), flush=True)
        np.savez_compressed(output / "yam_m8_trace_partial.npz", time=times, qpos=poses, qvel=velocities,
                            controller=motor_controls, info_json=np.asarray(json.dumps(records)),
                            metadata_json=np.asarray(json.dumps(metadata)))
        if aborted:
            break
    turns = [s for s in summaries if s["phase"].startswith("turn_")]
    resets = [s for s in summaries if s["phase"].startswith("reset_open_")]
    checks = {
        "all_requested_strokes_completed": {"passed": len(turns) == control.strokes, "observed": len(turns), "expected": control.strokes},
        "closed_turn_tracking": {"passed": bool(turns and all(abs(s["nut_rotation_rad"] + control.stroke_angle_rad) < .03 for s in turns)), "angular_error_limit_rad": .03},
        "observed_metric_lead": {"passed": bool(turns and all(abs(s["observed_helix_residual_m"]) < .02 * thread.pitch * control.stroke_angle_rad / (2 * np.pi) for s in turns))},
        "reported_sdf_depth_proxy": {"passed": peak_depth <= 10e-6, "observed_peak_m": peak_depth, "limit_m": 10e-6},
        "free_nut_alignment": {"passed": peak_radial <= 150e-6 and peak_tilt <= np.deg2rad(2.), "peak_radial_m": peak_radial, "peak_tilt_rad": peak_tilt},
        "no_solver_or_state_abort": {"passed": aborted is None},
        "nut_has_no_external_drive": {"passed": nut_unforced},
        "block_and_bolt_have_no_external_drive": {"passed": block_unforced},
        "left_block_contact_retention": {"passed": bool(np.all(minimum_loaded_left_normals > .1)),
                                          "sampled_minimum_loaded_pad_normals_N": minimum_loaded_left_normals.tolist()},
        "left_block_grip_slip": {"passed": peak_block_slip < .001 and peak_block_angle < np.deg2rad(2.),
                                  "peak_translation_slip_m": peak_block_slip, "peak_rotation_slip_rad": peak_block_angle},
        "block_has_no_world_support": {"passed": maximum_support_contacts == 0,
                                        "maximum_support_contacts": maximum_support_contacts},
        "left_arm_physically_lifts_block": {"passed": float(data.xpos[block_id, 2] - block_initial_position[2]) > .003,
                                             "observed_lift_m": float(data.xpos[block_id, 2] - block_initial_position[2])},
        "closed_turn_contact_torque": {"passed": bool(turns and all(s["sampled_peak_pad_contact_torque_Nm"] > 1e-6 for s in turns))},
        "open_reset_contact_decoupling": {"passed": bool(len(resets) == control.strokes - 1 and all(s["maximum_hand_nut_contacts"] == 0 for s in resets))},
        "passive_self_locking_during_reset": {"passed": bool(len(resets) == control.strokes - 1 and all(abs(s["nut_rotation_rad"]) < .02 for s in resets))},
    }
    checks = {name: {**values, "passed": bool(values["passed"])} for name, values in checks.items()}
    partial = maximum_phases is not None and maximum_phases < len(demo_phases(control))
    result = {**metadata, "partial": partial, "passed": all(c["passed"] for c in checks.values()),
              "description": "Actual bimanual YAM joint torque actuation; left fingers hold a free block and bolt, right fingers turn a pre-engaged nut",
              "phases": summaries, "acceptance_checks": checks, "aborted": aborted,
              "clockwise_turns": -yaw_total / (2 * np.pi),
              "axial_advance_mm": (initial_z - float((bolt_r.T @ (data.xpos[nut_id] - bolt_origin))[2])) * 1000,
              "wall_seconds": time.perf_counter() - wall_start,
              "reported_sdf_depth_note": "Native contact dist proxy; not an independent overlap certificate",
              "warnings": {str(i): int(w.number) for i, w in enumerate(data.warning) if w.number},
              "trajectory": str(output / "yam_m8_trace.npz")}
    np.savez_compressed(output / "yam_m8_trace.npz", time=times, qpos=poses, qvel=velocities,
                        controller=motor_controls, info_json=np.asarray(json.dumps(records)),
                        metadata_json=np.asarray(json.dumps(result)))
    (output / "yam_m8_validation.json").write_text(json.dumps(result, indent=2))
    return result
