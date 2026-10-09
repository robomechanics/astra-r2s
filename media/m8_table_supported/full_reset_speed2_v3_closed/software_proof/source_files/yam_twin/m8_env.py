"""Joint-actuated Gymnasium interface for the pre-engaged dual-YAM M8 scene.

The policy controls the actual arm motors and mirrored native fingers. It does
not command a Cartesian proxy hand, a nut trajectory, or a scripted phase. State
observations are privileged simulator measurements, not a camera policy input.
This interface does not validate thread starting or certify training fidelity.
"""
from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
from typing import Any

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

from thread_lab.env import _thread_attributes
from thread_lab.runtime import require_micron_engine
from .m8_scene import YamM8Config, build_model, jaw_positions


OBSERVATION_FIELDS = OrderedDict([
    ("arm_joint_position_rad", 12),
    ("arm_joint_velocity_rad_s", 12),
    ("finger_position_m", 4),
    ("finger_velocity_m_s", 4),
    ("nut_position_bolt_m", 3),
    ("nut_quaternion_bolt_wxyz", 4),
    ("nut_velocity_bolt_linear_angular", 6),
    ("bolt_pose_world_position_quaternion", 7),
    ("bolt_velocity_world_linear_angular", 6),
    ("block_pose_world_position_quaternion", 7),
    ("block_velocity_world_linear_angular", 6),
    ("left_grasp_pose_bolt_position_quaternion", 7),
    ("right_grasp_pose_bolt_position_quaternion", 7),
    ("thread_contact_count_depth_radial_tilt", 4),
    ("left_block_right_nut_pad_contact_counts_then_four_normal_forces", 6),
    ("axial_advance_rotation_helix_residual", 3),
])


def _quaternion(matrix):
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, np.asarray(matrix).reshape(9))
    if quat[0] < 0:
        quat *= -1
    return quat


class YamM8Env(gym.Env):
    """Two six-joint arms, two mirrored finger-aperture actions, and a free nut.

    Actions 0..5 and 6..11 are normalized left/right arm motor torques. Each is
    scaled by its compiled motor limit (28 Nm for joints 1..3 and 10 Nm for
    joints 4..6). Optional arm bias compensation is added *inside* those limits.
    Actions 12 and 13 request left/right jaw apertures: -1 is open, +1 closed.
    The four native position actuators retain their compiled finite force caps.

    Reset places the robots at the demonstration's grasp-ready poses. The left
    pads touch the block and request finite compression; the right pads are open.
    The nut starts engaged on the short bolt mounted to that free block. Reset
    does not claim a settled grasp: held/support status comes from actual forces.
    No scripted motion controller runs during a policy step. All progress is
    measured relative to the moving bolt and requires secure physical grasps.
    The 98 privileged observation entries are declared in OBSERVATION_FIELDS.
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 25}

    def __init__(self, model: mujoco.MjModel | None = None, *,
                 scene_config: YamM8Config | None = None,
                 control_dt: float = .01, horizon_seconds: float = 6.,
                 target_advance: float = .00125,
                 gravity_compensation: bool = True,
                 render_mode: str | None = None):
        super().__init__()
        self.runtime_info = require_micron_engine()
        self.scene_config = scene_config or YamM8Config()
        self.model = model if model is not None else build_model(self.scene_config)
        self.data = mujoco.MjData(self.model)
        if not np.isfinite([control_dt, horizon_seconds, target_advance]).all() or min(
                control_dt, horizon_seconds, target_advance) <= 0:
            raise ValueError("Control timing, horizon and target advance must be positive and finite")
        if render_mode not in (None, "rgb_array"):
            raise ValueError("Only rgb_array rendering is supported")
        self.render_mode = render_mode
        self.gravity_compensation = bool(gravity_compensation)
        self.frame_skip = max(1, round(control_dt / self.model.opt.timestep))
        self.control_dt = float(self.frame_skip * self.model.opt.timestep)
        self.max_steps = max(1, round(horizon_seconds / self.control_dt))
        self.target_advance = float(target_advance)
        self.arm_joint_ids = np.asarray([self.model.joint(f"{side}_joint{i}").id
            for side in ("left", "right") for i in range(1, 7)])
        self.arm_qpos_indices = self.model.jnt_qposadr[self.arm_joint_ids]
        self.arm_dof_indices = self.model.jnt_dofadr[self.arm_joint_ids]
        self.arm_motor_ids = np.asarray([self.model.actuator(f"{side}_servo{i}").id
            for side in ("left", "right") for i in range(1, 7)])
        for aid, jid in zip(self.arm_motor_ids, self.arm_joint_ids):
            if (self.model.actuator_trnid[aid, 0] != jid
                    or not np.array_equal(self.model.actuator_gear[aid], [1, 0, 0, 0, 0, 0])
                    or self.model.actuator_gaintype[aid] != mujoco.mjtGain.mjGAIN_FIXED
                    or self.model.actuator_gainprm[aid, 0] != 1
                    or np.any(self.model.actuator_biasprm[aid])
                    or not self.model.actuator_ctrllimited[aid]
                    or not self.model.actuator_forcelimited[aid]):
                raise ValueError("YAM policy actions require finite unit-gear unbiased joint motors")
        self.motor_limits = np.stack([
            np.maximum(self.model.actuator_ctrlrange[self.arm_motor_ids, 0],
                       self.model.actuator_forcerange[self.arm_motor_ids, 0]),
            np.minimum(self.model.actuator_ctrlrange[self.arm_motor_ids, 1],
                       self.model.actuator_forcerange[self.arm_motor_ids, 1]),
        ], axis=1)
        if np.any(self.motor_limits[:, 0] >= 0) or np.any(self.motor_limits[:, 1] <= 0):
            raise ValueError("Motor limits must permit positive and negative joint torques")
        self.torque_caps = np.minimum(-self.motor_limits[:, 0], self.motor_limits[:, 1])
        self.finger_joint_ids = np.asarray([self.model.joint(f"{side}_{finger}_finger").id
            for side in ("left", "right") for finger in ("left", "right")])
        self.finger_qpos_indices = self.model.jnt_qposadr[self.finger_joint_ids]
        self.finger_dof_indices = self.model.jnt_dofadr[self.finger_joint_ids]
        self.finger_actuator_ids = np.asarray([self.model.actuator(f"{side}_grip_{finger}").id
            for side in ("left", "right") for finger in ("left", "right")])
        self.pad_geom_ids = np.asarray([self.model.geom(f"{side}_m8_pad_{finger}").id
            for side in ("left", "right") for finger in ("left", "right")])
        self.grasp_site_ids = [self.model.site(f"{side}_grasp_site").id for side in ("left", "right")]
        self.nut_id, self.bolt_id = self.model.body("nut").id, self.model.body("bolt_frame").id
        self.block_id = self.model.body("fixture_block").id
        self.block_geom_id = self.model.geom("fixture_block_geom").id
        self.block_joint_id = self.model.joint("fixture_block_free").id
        if (self.model.jnt_type[self.block_joint_id] != mujoco.mjtJoint.mjJNT_FREE
                or self.model.body_parentid[self.bolt_id] != self.block_id):
            raise ValueError("The policy task requires a free block carrying its rigid bolt")
        self.block_qpos_addr = int(self.model.jnt_qposadr[self.block_joint_id])
        self.block_dof_addr = int(self.model.jnt_dofadr[self.block_joint_id])
        self.nut_joint_id = self.model.joint("nut_free").id
        if self.model.jnt_type[self.nut_joint_id] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError("The policy task requires a free six-DOF nut")
        self.nut_qpos_addr = int(self.model.jnt_qposadr[self.nut_joint_id])
        self.nut_dof_addr = int(self.model.jnt_dofadr[self.nut_joint_id])
        self.bolt_geom_id, self.nut_geom_id = (self.model.geom(name).id
                                              for name in ("bolt_thread", "nut_thread"))
        bolt_attrs = _thread_attributes(self.model, self.bolt_geom_id)
        nut_attrs = _thread_attributes(self.model, self.nut_geom_id)
        self.pitch = float(bolt_attrs["pitch"])
        self.bolt_length, self.nut_height = float(bolt_attrs["length"]), float(nut_attrs["length"])
        if not np.isclose(self.pitch, float(nut_attrs["pitch"]), rtol=0, atol=1e-12):
            raise ValueError("Compiled nut and bolt pitches disagree")
        self.action_space = spaces.Box(-1., 1., shape=(14,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf,
            shape=(sum(OBSERVATION_FIELDS.values()),), dtype=np.float32)
        self._renderer = None
        self._active = False
        self._steps = self._recent_valid_contact_steps = 0
        self._initial_axial = self._last_advance = self._unwrapped_yaw = self._last_yaw = 0.
        self._initial_warnings = 0
        self._peak_depth = 0.
        self._initial_block_in_left_position = np.zeros(3)
        self._initial_block_in_left_rotation = np.eye(3)
        self.last_motor_torques = np.zeros(12)
        self.last_commanded_apertures = np.full(2, self.scene_config.open_aperture)

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        if options:
            raise ValueError("This pre-engaged proof-of-concept currently has a deterministic work-pose reset")
        from .m8_simulation import initialize_work_pose, YamM8ControlConfig
        mujoco.mj_resetData(self.model, self.data)
        initialize_work_pose(self.model, self.data, control_config=YamM8ControlConfig(
            open_aperture=self.scene_config.open_aperture,
            closed_aperture=self.scene_config.closed_aperture,
            left_closed_aperture=self.scene_config.left_closed_aperture),
            scene_config=self.scene_config)
        mujoco.mj_forward(self.model, self.data)
        self._initial_block_in_left_position, self._initial_block_in_left_rotation = self._block_in_left_grasp()
        self._initial_axial = float(self._nut_relative_pose()[0][2])
        self._last_yaw = self._nut_yaw()
        self._unwrapped_yaw = self._last_advance = self._peak_depth = 0.
        self._steps = self._recent_valid_contact_steps = 0
        self._initial_warnings = int(sum(w.number for w in self.data.warning))
        self.last_motor_torques.fill(0)
        self.last_commanded_apertures[:] = [self.scene_config.left_closed_aperture,
                                            self.scene_config.open_aperture]
        self._active = True
        info = self._info()
        info["reset_grasp_state"] = "Left pads touch with finite preload request; right pads open; no physical settling"
        return self._observation(), info

    def step(self, action):
        if not self._active:
            raise RuntimeError("Call reset before step, including after an episode ends")
        action = np.asarray(action, dtype=float)
        if action.shape != (14,) or not np.isfinite(action).all():
            raise ValueError("Action must contain fourteen finite entries")
        action = np.clip(action, -1., 1.)
        cfg = self.scene_config
        closed_apertures = np.array([cfg.left_closed_aperture, cfg.closed_aperture])
        apertures = cfg.open_aperture - (action[12:] + 1) / 2 * (cfg.open_aperture - closed_apertures)
        self.last_commanded_apertures[:] = apertures
        failure_reasons = []
        for _ in range(self.frame_skip):
            torques = action[:12] * self.torque_caps
            if self.gravity_compensation:
                torques += self.data.qfrc_bias[self.arm_dof_indices]
            self.last_motor_torques[:] = np.clip(torques, self.motor_limits[:, 0], self.motor_limits[:, 1])
            self.data.ctrl[self.arm_motor_ids] = self.last_motor_torques
            self.data.ctrl[self.finger_actuator_ids] = np.ravel([jaw_positions(gap, cfg) for gap in apertures])
            # These are the only rollout assignments: joint motors and fingers.
            # Neither free workpiece receives an external wrench or state command.
            mujoco.mj_step(self.model, self.data)
            yaw = self._nut_yaw()
            self._unwrapped_yaw += float((yaw - self._last_yaw + np.pi) % (2 * np.pi) - np.pi)
            self._last_yaw = yaw
            self._peak_depth = max(self._peak_depth, self._contact_metrics()[1])
            failure_reasons = self._failure_reasons()
            if failure_reasons:
                break
        # Refresh derived poses, velocities and contacts at the returned state.
        # mj_step integrates qpos/qvel after its force/contact evaluation. This
        # forward pass advances no time and applies no scripted robot command.
        if np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all():
            mujoco.mj_forward(self.model, self.data)
            self._peak_depth = max(self._peak_depth, self._contact_metrics()[1])
            failure_reasons = list(dict.fromkeys(failure_reasons + self._failure_reasons()))
        self._steps += 1
        info = self._info()
        advance = info["axial_advance_m"]
        progress = (advance - self._last_advance) / self.pitch
        self._last_advance = advance
        failure = bool(failure_reasons)
        valid_contact = (not failure and info["bolt_nut_contact_count"] > 0
            and info["left_block_grasp_secure"] and info["right_nut_grasp_loaded"]
            and info["workpiece_world_support_contact_count"] == 0
            and info["radial_offset_m"] <= 150e-6
            and info["nut_tilt_rad"] <= np.deg2rad(2.)
            and abs(info["observed_helix_residual_m"]) <= 150e-6)
        self._recent_valid_contact_steps = self._recent_valid_contact_steps + 1 if valid_contact else 0
        success = bool(advance >= self.target_advance and self._recent_valid_contact_steps >= 10)
        reward = -1. if failure else float(progress - .002 * np.dot(action[:12], action[:12]))
        if success:
            reward += 1.
        terminated = bool(success or failure)
        truncated = bool(self._steps >= self.max_steps and not terminated)
        info.update(success=success, failure=failure, failure_reasons=failure_reasons,
            recent_valid_contact_steps=self._recent_valid_contact_steps,
            action_normalized=action.tolist(), commanded_arm_motor_torques_Nm=self.last_motor_torques.tolist(),
            commanded_apertures_m=apertures.tolist())
        if terminated or truncated:
            self._active = False
        return self._observation(), float(reward), terminated, truncated, info

    def _bolt_pose(self):
        # Read the integrated free block directly: derived world body frames
        # otherwise lag mj_step by one substep until the control-boundary refresh.
        state = self.data.qpos[self.block_qpos_addr:self.block_qpos_addr + 7]
        block_rotation, local_rotation = np.empty(9), np.empty(9)
        mujoco.mju_quat2Mat(block_rotation, state[3:])
        mujoco.mju_quat2Mat(local_rotation, self.model.body_quat[self.bolt_id])
        block_rotation = block_rotation.reshape(3, 3)
        return (state[:3] + block_rotation @ self.model.body_pos[self.bolt_id],
                block_rotation @ local_rotation.reshape(3, 3))

    def _nut_relative_pose(self):
        bolt_position, bolt_rotation = self._bolt_pose()
        rotate = bolt_rotation.T
        # The free-joint state is current immediately after integration. Derived
        # xmat/xpos arrays are refreshed at the control boundary above.
        state = self.data.qpos[self.nut_qpos_addr:self.nut_qpos_addr + 7]
        matrix = np.empty(9)
        mujoco.mju_quat2Mat(matrix, state[3:])
        return (rotate @ (state[:3] - bolt_position),
                rotate @ matrix.reshape(3, 3))

    def _nut_yaw(self):
        matrix = self._nut_relative_pose()[1]
        return float(np.arctan2(matrix[1, 0], matrix[0, 0]))

    def _contact_metrics(self):
        thread_count, depth = 0, 0.
        counts, normals = np.zeros(2, dtype=int), np.zeros(4)
        force = np.zeros(6)
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            geoms = {int(contact.geom1), int(contact.geom2)}
            if geoms == {self.bolt_geom_id, self.nut_geom_id}:
                thread_count += 1
                depth = max(depth, -float(contact.dist))
            if self.nut_geom_id in geoms or self.block_geom_id in geoms:
                for index, gid in enumerate(self.pad_geom_ids):
                    held_geom = self.block_geom_id if index < 2 else self.nut_geom_id
                    if gid in geoms and held_geom in geoms:
                        counts[index // 2] += 1
                        mujoco.mj_contactForce(self.model, self.data, i, force)
                        normals[index] += float(force[0])
        return thread_count, max(0., depth), counts, normals

    def _block_in_left_grasp(self):
        site = self.grasp_site_ids[0]
        world_to_left = self.data.site_xmat[site].reshape(3, 3).T
        return (world_to_left @ (self.data.xpos[self.block_id] - self.data.site_xpos[site]),
                world_to_left @ self.data.xmat[self.block_id].reshape(3, 3))

    def _grasp_support_metrics(self, normals):
        """Measure support and retention; contact intent is not a held grasp."""
        block_position, block_rotation = self._block_in_left_grasp()
        position_slip = float(np.linalg.norm(block_position - self._initial_block_in_left_position))
        relative_rotation = block_rotation @ self._initial_block_in_left_rotation.T
        rotation_slip = float(np.arccos(np.clip((np.trace(relative_rotation) - 1) / 2, -1., 1.)))
        block_support, nut_support = 0, 0
        block_geoms = {self.block_geom_id, self.bolt_geom_id}
        for contact in self.data.contact:
            g1, g2 = int(contact.geom1), int(contact.geom2)
            for own, other in ((g1, g2), (g2, g1)):
                other_body = int(self.model.geom_bodyid[other])
                # weldid 0 includes the table and every rigid world-fixed body.
                # Dynamic robot fingers and links have nonzero weld IDs.
                if self.model.body_weldid[other_body] != 0:
                    continue
                if own in block_geoms:
                    block_support += 1
                elif own == self.nut_geom_id:
                    nut_support += 1
        left_loaded = bool(np.all(np.asarray(normals[:2]) > .1))
        right_loaded = bool(np.all(np.asarray(normals[2:]) > .1))
        retained = bool(position_slip < .001 and rotation_slip < np.deg2rad(2.))
        secure = bool(left_loaded and retained and block_support == 0)
        return {"free_block_held_by_left_fingers": secure,
            "left_block_grasp_secure": secure, "right_nut_grasp_loaded": right_loaded,
            "left_pad_normals_loaded": left_loaded, "right_pad_normals_loaded": right_loaded,
            "minimum_pad_normal_force_for_success_N": .1,
            "block_left_grasp_position_slip_m": position_slip,
            "block_left_grasp_rotation_slip_rad": rotation_slip,
            "block_left_grasp_retained": retained,
            "block_world_support_contact_count": block_support,
            "nut_world_support_contact_count": nut_support,
            "workpiece_world_support_contact_count": block_support + nut_support}

    def _failure_reasons(self):
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            return ["nonfinite_state"]
        position, rotation = self._nut_relative_pose()
        residual = self._initial_axial - position[2] + self.pitch * self._unwrapped_yaw / (2 * np.pi)
        reasons = []
        if self._contact_metrics()[1] > 10e-6:
            reasons.append("reported_sdf_depth_above_10_um")
        # This task starts engaged and stays inside the demonstrated alignment
        # envelope. Exploration of thread starting or larger offsets needs its
        # own physics validation before those limits can be relaxed.
        if np.linalg.norm(position[:2]) > 150e-6:
            reasons.append("nut_radial_offset_above_150_um")
        if rotation[2, 2] < np.cos(np.deg2rad(2.)):
            reasons.append("nut_tilt_above_2_degrees")
        if not -self.nut_height / 2 < position[2] < self.bolt_length + self.nut_height / 2:
            reasons.append("nut_outside_bolt_axially")
        if abs(residual) > 150e-6:
            reasons.append("observed_helix_residual_above_150_um")
        if sum(w.number for w in self.data.warning) > self._initial_warnings:
            reasons.append("solver_warning")
        if np.any(self.data.xfrc_applied[self.nut_id]) or np.any(
                self.data.qfrc_applied[self.nut_dof_addr:self.nut_dof_addr + 6]):
            reasons.append("nut_external_drive_present")
        if np.any(self.data.xfrc_applied[[self.block_id, self.bolt_id]]) or np.any(
                self.data.qfrc_applied[self.block_dof_addr:self.block_dof_addr + 6]):
            reasons.append("block_external_drive_present")
        return reasons

    def _body_velocity(self, body_id):
        velocity = np.empty(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, body_id, velocity, 0)
        return velocity

    def _nut_relative_velocity(self):
        # A body-origin twist is needed here, not a COM velocity. The rotating
        # bolt frame transports its velocity to the nut origin before subtraction.
        bolt_rotation = self.data.xmat[self.bolt_id].reshape(3, 3)
        nut_velocity, bolt_velocity = self._body_velocity(self.nut_id), self._body_velocity(self.bolt_id)
        delta = self.data.xpos[self.nut_id] - self.data.xpos[self.bolt_id]
        linear = nut_velocity[3:] - bolt_velocity[3:] - np.cross(bolt_velocity[:3], delta)
        angular = nut_velocity[:3] - bolt_velocity[:3]
        return np.r_[bolt_rotation.T @ linear, bolt_rotation.T @ angular]

    def _observation(self):
        data, model = self.data, self.model
        position, rotation = self._nut_relative_pose()
        bolt_rotation = data.xmat[self.bolt_id].reshape(3, 3)
        bolt_velocity, block_velocity = self._body_velocity(self.bolt_id), self._body_velocity(self.block_id)
        poses = []
        for site in self.grasp_site_ids:
            poses.extend([bolt_rotation.T @ (data.site_xpos[site] - data.xpos[self.bolt_id]),
                          _quaternion(bolt_rotation.T @ data.site_xmat[site].reshape(3, 3))])
        count, depth, pad_counts, normals = self._contact_metrics()
        advance = self._initial_axial - position[2]
        observation = np.concatenate([
            data.qpos[self.arm_qpos_indices], data.qvel[self.arm_dof_indices],
            data.qpos[self.finger_qpos_indices], data.qvel[self.finger_dof_indices],
            position, _quaternion(rotation), self._nut_relative_velocity(),
            data.xpos[self.bolt_id], data.xquat[self.bolt_id], bolt_velocity[3:], bolt_velocity[:3],
            data.xpos[self.block_id], data.xquat[self.block_id], block_velocity[3:], block_velocity[:3], *poses,
            [count, depth, np.linalg.norm(position[:2]), np.arccos(np.clip(rotation[2, 2], -1., 1.))],
            pad_counts, normals, [advance, self._unwrapped_yaw,
                                advance + self.pitch * self._unwrapped_yaw / (2 * np.pi)],
        ])
        return observation.astype(np.float32)

    def _info(self):
        position, rotation = self._nut_relative_pose()
        count, depth, pad_counts, normals = self._contact_metrics()
        advance = float(self._initial_axial - position[2])
        return {"starts_preengaged": True, "fixed_bolt": False,
            **self._grasp_support_metrics(normals),
            "actual_yam_joint_actuation": True, "privileged_state_observations": True,
            "gravity_bias_compensation": self.gravity_compensation,
            "axial_advance_m": advance, "nut_unwrapped_rotation_rad": self._unwrapped_yaw,
            "observed_helix_residual_m": float(advance + self.pitch * self._unwrapped_yaw / (2 * np.pi)),
            "radial_offset_m": float(np.linalg.norm(position[:2])),
            "nut_tilt_rad": float(np.arccos(np.clip(rotation[2, 2], -1., 1.))),
            "bolt_nut_contact_count": count, "worst_reported_sdf_depth_m": depth,
            "episode_peak_reported_sdf_depth_m": self._peak_depth,
            "left_pad_contact_count": int(pad_counts[0]), "right_pad_contact_count": int(pad_counts[1]),
            "left_block_pad_contact_count": int(pad_counts[0]), "right_nut_pad_contact_count": int(pad_counts[1]),
            "pad_normal_forces_N": normals.tolist(),
            "commanded_apertures_m": self.last_commanded_apertures.tolist(),
            "measured_apertures_m": (self.data.qpos[self.finger_qpos_indices].reshape(2, 2)
                @ np.array([1., -1.]) - 2*self.scene_config.pad_inner_offset).tolist(),
            "reported_sdf_depth_note": "Native contact dist proxy; not an independent overlap certificate",
            "elapsed_seconds": float(self.data.time), "control_dt": self.control_dt,
            "model_timestep_s": float(self.model.opt.timestep),
            "observation_fields": dict(OBSERVATION_FIELDS), "runtime": deepcopy(self.runtime_info),
            "physics_scope": "Rigid-body M8 contact; actual YAM motors; pre-engaged nut; free block and rigidly attached short bolt; assumed pad inserts; no thread-start or training validation"}

    def render(self):
        if self.render_mode != "rgb_array":
            return None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=480, width=640)
        self._renderer.update_scene(self.data, camera="overview")
        return self._renderer.render().copy()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None
