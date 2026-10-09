"""Actual joint-action interface for unengaged M8 pickup and thread starting.

The default task begins with the block on the table and the bolt on its
three-pin rest, separately from the female thread. No scripted controller runs
in policy steps. The observations are
privileged simulator state; this module supplies neither a trained policy nor
thread-start qualification. Depth, rotation and support gates only measure the
result of rigid-body contacts, never command a helix.
"""
from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
import hashlib
import inspect
from pathlib import Path

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

from thread_lab.env import _thread_attributes
from thread_lab.runtime import require_micron_engine
from .m8_env import _quaternion
from .m8_insertion_scene import InsertionConfig, build_model, jaw_positions
from .m8_insertion_mechanics import fully_formed_flank_interval, contact_is_on_full_flanks
from .m8_insertion_engagement import LoadedFlankWindow


OBSERVATION_FIELDS = OrderedDict([
    ("arm_joint_position_rad", 12), ("arm_joint_velocity_rad_s", 12),
    ("finger_position_m", 4), ("finger_velocity_m_s", 4),
    ("block_pose_world_position_quaternion", 7), ("block_velocity_world_linear_angular", 6),
    ("male_pose_world_position_quaternion", 7), ("male_velocity_world_linear_angular", 6),
    ("female_pose_world_position_quaternion", 7), ("female_velocity_world_linear_angular", 6),
    ("male_pose_in_female_position_quaternion", 7), ("male_relative_velocity_in_female_linear_angular", 6),
    ("block_pose_in_left_grasp_position_quaternion", 7), ("head_pose_in_right_grasp_position_quaternion", 7),
    ("thread_contact_count_reported_depth", 2), ("four_pad_normal_forces_N", 4),
    ("block_bolt_world_support_contact_counts", 2), ("tip_depth_radial_offset_tilt", 3),
    ("left_right_grasp_position_and_rotation_slip", 4),
    ("rest_seen_pickup_seen_loaded_candidate_advance_observed_lead_residual", 5),
])


def _rotation(quat):
    result = np.empty(9)
    mujoco.mju_quat2Mat(result, quat)
    return result.reshape(3, 3)


def _rotation_error(current, reference):
    return float(np.arccos(np.clip((np.trace(current @ reference.T) - 1) / 2, -1., 1.)))


class YamM8InsertionEnv(gym.Env):
    """Fourteen real joint/finger actions and 118 privileged observations.

    Actions 0..11 are normalized left/right arm motor torques. Optional arm bias
    compensation is included inside each compiled motor's finite torque limit.
    Actions 12/13 request mirrored left/right native finger apertures (-1 open,
    +1 closed). No Cartesian hand, free-body wrench, pose assignment or scripted
    pickup is an action. The default reset opens both hands away from the two
    separately supported workpieces. Explicit legacy models/configurations
    preserve the original left touching-pad reset. Neither reset changes a
    workpiece pose. Pickup history is exposed in info; a recurrent policy or
    observation history is needed to use that history with the 118-vector.

    Acquisition, thread-contact and completion flags require measured forces.
    Projected tip depth while away from the hole earns no insertion reward.
    The provisional thread guard begins after a 0.2-second valid geometry/grasp
    window with measured normal impulse inside both unchamfered regions and one
    pitch of whole-ring flank overlap. Unilateral contact gaps are allowed.
    Capture uses the demo's versioned observer: at least 0.001 N s of normal
    impulse, 0.5 ms of loaded contact and at most 150 micrometers of phase range.
    Independent observed lead is still required for the thread-start marker and
    completion. Starting physics must be qualified separately.
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 25}

    def __init__(self, model=None, *, scene_config=None, control_dt=.01,
                 horizon_seconds=30., target_tip_depth=None,
                 gravity_compensation=True, render_mode=None):
        super().__init__()
        self.runtime_info = require_micron_engine()
        if model is None and scene_config is None:
            from .m8_insertion_scene import table_pickup_config
            scene_config = table_pickup_config()
        self.scene_config = scene_config or InsertionConfig()
        self.pickup_from_table = bool(getattr(self.scene_config, "pickup_from_table", False))
        self.model = model if model is not None else build_model(self.scene_config)
        self.data = mujoco.MjData(self.model)
        if not np.isfinite([control_dt, horizon_seconds]).all() or min(control_dt, horizon_seconds) <= 0:
            raise ValueError("Control timing and horizon must be positive and finite")
        if render_mode not in (None, "rgb_array"):
            raise ValueError("Only rgb_array rendering is supported")
        self.render_mode, self.gravity_compensation = render_mode, bool(gravity_compensation)
        self.frame_skip = max(1, round(control_dt / self.model.opt.timestep))
        self.control_dt = float(self.frame_skip * self.model.opt.timestep)
        self.max_steps = max(1, round(horizon_seconds / self.control_dt))
        self.arm_joints = np.asarray([self.model.joint(f"{s}_joint{i}").id
            for s in ("left", "right") for i in range(1, 7)])
        self.arm_qpos = self.model.jnt_qposadr[self.arm_joints]
        self.arm_dofs = self.model.jnt_dofadr[self.arm_joints]
        self.arm_motors = np.asarray([self.model.actuator(f"{s}_servo{i}").id
            for s in ("left", "right") for i in range(1, 7)])
        for aid, jid in zip(self.arm_motors, self.arm_joints):
            if (self.model.actuator_trnid[aid, 0] != jid
                    or not np.array_equal(self.model.actuator_gear[aid], [1, 0, 0, 0, 0, 0])
                    or self.model.actuator_gaintype[aid] != mujoco.mjtGain.mjGAIN_FIXED
                    or self.model.actuator_gainprm[aid, 0] != 1
                    or np.any(self.model.actuator_biasprm[aid])
                    or not self.model.actuator_ctrllimited[aid]
                    or not self.model.actuator_forcelimited[aid]):
                raise ValueError("Insertion actions require finite unit-gear unbiased joint motors")
        self.motor_limits = np.c_[np.maximum(self.model.actuator_ctrlrange[self.arm_motors, 0],
                                              self.model.actuator_forcerange[self.arm_motors, 0]),
                                  np.minimum(self.model.actuator_ctrlrange[self.arm_motors, 1],
                                             self.model.actuator_forcerange[self.arm_motors, 1])]
        self.torque_caps = np.minimum(-self.motor_limits[:, 0], self.motor_limits[:, 1])
        if np.any(self.torque_caps <= 0) or not np.isfinite(self.torque_caps).all():
            raise ValueError("Motor limits must permit positive and negative finite torques")
        self.finger_joints = np.asarray([self.model.joint(f"{s}_{f}_finger").id
            for s in ("left", "right") for f in ("left", "right")])
        self.finger_qpos = self.model.jnt_qposadr[self.finger_joints]
        self.finger_dofs = self.model.jnt_dofadr[self.finger_joints]
        self.finger_actuators = np.asarray([self.model.actuator(f"{s}_grip_{f}").id
            for s in ("left", "right") for f in ("left", "right")])
        self.pad_ids = np.asarray([self.model.geom(f"{s}_m8_pad_{f}").id
            for s in ("left", "right") for f in ("left", "right")])
        self.grasp_sites = [self.model.site(f"{s}_grasp_site").id for s in ("left", "right")]
        self.block_id, self.male_id, self.female_id = [self.model.body(n).id
            for n in ("fixture_block", "male_bolt", "female_frame")]
        self.block_joint, self.male_joint = [self.model.joint(n).id
            for n in ("fixture_block_free", "male_bolt_free")]
        if any(self.model.jnt_type[j] != mujoco.mjtJoint.mjJNT_FREE for j in (self.block_joint, self.male_joint)):
            raise ValueError("Insertion requires free block and male bolt bodies")
        self.block_qadr, self.male_qadr = [int(self.model.jnt_qposadr[j]) for j in (self.block_joint, self.male_joint)]
        self.block_dadr, self.male_dadr = [int(self.model.jnt_dofadr[j]) for j in (self.block_joint, self.male_joint)]
        self.male_geom, self.female_geom, self.head_geom = [self.model.geom(n).id
            for n in ("bolt_thread", "female_thread", "bolt_head")]
        self.block_geoms = {int(i) for i, b in enumerate(self.model.geom_bodyid)
                            if self.model.body_weldid[b] == self.block_id}
        self.male_geoms = {int(i) for i, b in enumerate(self.model.geom_bodyid) if b == self.male_id}
        self.rest_geoms = {i for i in range(self.model.ngeom)
                           if (self.model.geom(i).name or "").startswith("bolt_rest_")}
        male_attrs, female_attrs = (_thread_attributes(self.model, g) for g in (self.male_geom, self.female_geom))
        self.pitch, self.bolt_length = float(male_attrs["pitch"]), float(male_attrs["length"])
        self.hole_height = float(female_attrs["length"])
        if not np.isclose(self.pitch, float(female_attrs["pitch"]), rtol=0, atol=1e-12):
            raise ValueError("Male and female thread pitches disagree")
        self.target_tip_depth = float(target_tip_depth if target_tip_depth is not None else self.bolt_length-.001)
        if not np.isfinite(self.target_tip_depth) or not self.pitch < self.target_tip_depth <= self.bolt_length:
            raise ValueError("Target tip depth must exceed one pitch and fit the threaded shaft")
        self.action_space = spaces.Box(-1., 1., shape=(14,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf,
            shape=(sum(OBSERVATION_FIELDS.values()),), dtype=np.float32)
        self._renderer, self._active = None, False
        self.last_motor_torques = np.zeros(12)
        self.last_apertures = np.array([self.scene_config.base.left_closed_aperture,
                                        self.scene_config.base.open_aperture])
        self.engagement_observer_source_sha256 = hashlib.sha256(
            Path(inspect.getfile(LoadedFlankWindow)).read_bytes()).hexdigest()
        self._clear_episode()

    def _clear_episode(self):
        self._steps = self._valid_contact_controls = 0
        self._warnings = 0
        self._yaw_total = self._last_yaw = self._peak_thread_depth = 0.
        self._start_depth = self._start_yaw = self._last_scored_depth = None
        self._pickup_observed = self._ever_rest_support = False
        self._left_acquisition_observed = self._left_pickup_observed = False
        self._right_pickup_observed = self._ever_block_world_support = False
        self._left_acquisition_time = self._right_acquisition_time = None
        self._left_pickup_time = self._right_pickup_time = None
        self._left_loaded_last = False
        self._right_loaded_last = False
        self._right_reference = None
        self._left_reference = None
        self._initial_block_height = 0.
        self._initial_head_height = 0.
        self._minimum_joint_margin = self._native_joint_margin()
        self.engagement_observer = LoadedFlankWindow()
        self.engagement_sustain_s = self.engagement_observer.duration_s
        # No synthetic observer sample is submitted at reset. These are the
        # unobserved initial diagnostics, replaced after a real physics substep.
        self._engagement_metrics = {"version": self.engagement_observer.version,
            "ready": False, "continuous_geometry_elapsed_s": 0.,
            "window_duration_s": self.engagement_sustain_s, "normal_impulse_Ns": 0.,
            "loaded_substeps": 0, "loaded_contact_count": 0, "loaded_duration_s": 0.,
            "sample_count": 0, "loaded_substep_duty": 0., "helix_phase_range_m": 0.}

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if options:
            raise ValueError("Insertion currently uses a deterministic, unengaged pickup reset")
        from .m8_insertion_simulation import InsertionControlConfig, initialize_insertion_pose
        from .m8_simulation import YamM8ControlConfig
        cfg = self.scene_config.base
        control = InsertionControlConfig(arm=YamM8ControlConfig(open_aperture=cfg.open_aperture,
            closed_aperture=cfg.closed_aperture, left_closed_aperture=cfg.left_closed_aperture))
        mujoco.mj_resetData(self.model, self.data)
        initialize_insertion_pose(self.model, self.data, self.scene_config, control)
        mujoco.mj_forward(self.model, self.data)
        self._clear_episode()
        # The new task has no initial grasp reference: that can only arise
        # from bilateral pad forces after an actual physics substep. The old
        # touching-pad task retains its archived reset behavior.
        if not self.pickup_from_table:
            self._left_reference = self._held_pose("left")
        self._last_yaw = self._relative_yaw()
        self._initial_block_height = float(self._free_pose(self.block_qadr)[0][2])
        self._initial_head_height = float(self._head_position()[2])
        self._warnings = int(sum(w.number for w in self.data.warning))
        contacts = self._contacts()
        self._ever_rest_support = contacts["bolt_rest_support_contacts"] > 0
        self._ever_block_world_support = contacts["block_world_support_contacts"] > 0
        self.last_motor_torques.fill(0)
        self.last_apertures[:] = [cfg.open_aperture if self.pickup_from_table else cfg.left_closed_aperture,
                                  cfg.open_aperture]
        self._active = True
        info = self._info()
        info["reset_grasp_state"] = (
            "Both hands open and separated; free block on table and free bolt on rest; no settling"
            if self.pickup_from_table else
            "Left touching-pad preload request; right open at pickup hover; free bolt on rest; no settling")
        return self._observation(), info

    def _free_pose(self, address):
        state = self.data.qpos[address:address+7]
        return state[:3].copy(), _rotation(state[3:])

    def _female_pose(self):
        p, r = self._free_pose(self.block_qadr)
        return (p+r@self.model.body_pos[self.female_id],
                r@_rotation(self.model.body_quat[self.female_id]))

    def _relative_pose(self):
        fp, fr = self._female_pose()
        mp, mr = self._free_pose(self.male_qadr)
        return fr.T@(mp-fp), fr.T@mr

    def _head_position(self):
        p, r = self._free_pose(self.male_qadr)
        return p+r@np.array([0., 0., -self.scene_config.head_height/2])

    def _relative_yaw(self):
        r = self._relative_pose()[1]
        return float(np.arctan2(r[1, 0], r[0, 0]))

    def _held_pose(self, side):
        index = 0 if side == "left" else 1
        site = self.grasp_sites[index]
        sr = self.data.site_xmat[site].reshape(3, 3)
        if side == "left":
            p, r = self._free_pose(self.block_qadr)
        else:
            p, r = self._head_position(), self._free_pose(self.male_qadr)[1]
        return sr.T@(p-self.data.site_xpos[site]), sr.T@r

    def _contacts(self):
        result = {"thread_count": 0, "thread_depth": 0., "loaded_full_flank_contact_count": 0,
                  "loaded_full_flank_normal_force_N": 0., "rigid_workpiece_depth": 0.,
                  "block_world_support_contacts": 0, "bolt_world_support_contacts": 0,
                  "bolt_rest_support_contacts": 0, "pad_normals": np.zeros(4)}
        force = np.zeros(6)
        female_p, female_r = self._female_pose()
        male_p, male_r = self._free_pose(self.male_qadr)
        for i, c in enumerate(self.data.contact):
            geoms = {int(c.geom1), int(c.geom2)}
            if geoms == {self.male_geom, self.female_geom}:
                result["thread_count"] += 1
                result["thread_depth"] = max(result["thread_depth"], -float(c.dist))
                mujoco.mj_contactForce(self.model, self.data, i, force)
                if force[0] > 1e-5 and contact_is_on_full_flanks(
                        female_r.T@(c.pos-female_p), male_r.T@(c.pos-male_p), self.scene_config.thread):
                    result["loaded_full_flank_contact_count"] += 1
                    result["loaded_full_flank_normal_force_N"] += float(force[0])
            elif geoms & self.male_geoms and geoms & self.block_geoms:
                result["rigid_workpiece_depth"] = max(result["rigid_workpiece_depth"], -float(c.dist))
            for own, other in ((int(c.geom1), int(c.geom2)), (int(c.geom2), int(c.geom1))):
                other_body = int(self.model.geom_bodyid[other])
                if self.model.body_weldid[other_body] == 0:
                    if own in self.block_geoms:
                        result["block_world_support_contacts"] += 1
                    elif own in self.male_geoms:
                        result["bolt_world_support_contacts"] += 1
                        result["bolt_rest_support_contacts"] += other in self.rest_geoms
            for index, pad in enumerate(self.pad_ids):
                targets = self.block_geoms if index < 2 else {self.head_geom}
                if pad in geoms and geoms & targets:
                    mujoco.mj_contactForce(self.model, self.data, i, force)
                    result["pad_normals"][index] += float(force[0])
        return result

    def _metrics(self, contacts=None):
        contacts = self._contacts() if contacts is None else contacts
        position, rotation = self._relative_pose()
        tip = position+rotation@np.array([0., 0., self.bolt_length])
        depth = float(tip[2]+self.hole_height/2)
        axis = rotation[:, 2]
        # Axis offset at the female entry is meaningful even before a tip enters.
        entry_axis = position+axis*((-self.hole_height/2-position[2])/axis[2]) if abs(axis[2]) > 1e-6 else tip
        radial = float(np.linalg.norm(entry_axis[:2]))
        tilt = float(np.arccos(np.clip(axis[2], -1., 1.)))
        left_pose, right_pose = self._held_pose("left"), self._held_pose("right")
        if self._left_reference is None:
            left_slip, left_rotation = 0., 0.
            left_retained = False
        else:
            left_slip = float(np.linalg.norm(left_pose[0]-self._left_reference[0]))
            left_rotation = _rotation_error(left_pose[1], self._left_reference[1])
            left_retained = left_slip < .001 and left_rotation < np.deg2rad(2.)
        normals = contacts["pad_normals"]
        left_loaded, right_loaded = bool(np.all(normals[:2] > .1)), bool(np.all(normals[2:] > .1))
        if self._right_reference is None:
            right_slip, right_rotation = 0., 0.
            right_retained = False
        else:
            right_slip = float(np.linalg.norm(right_pose[0]-self._right_reference[0]))
            right_rotation = _rotation_error(right_pose[1], self._right_reference[1])
            right_retained = right_slip < .001 and right_rotation < np.deg2rad(2.)
        left_secure = bool(left_loaded and left_retained
                           and contacts["block_world_support_contacts"] == 0)
        right_secure = bool(right_loaded and right_retained and contacts["bolt_world_support_contacts"] == 0)
        advance = 0. if self._start_depth is None else depth-self._start_depth
        rotation_since_start = 0. if self._start_yaw is None else self._yaw_total-self._start_yaw
        residual = advance-self.pitch*rotation_since_start/(2*np.pi)
        interval = fully_formed_flank_interval(position, rotation, self.scene_config.thread)
        expected_lead_advance = self.pitch*rotation_since_start/(2*np.pi)
        lead_relative_error = abs(residual/expected_lead_advance) if abs(expected_lead_advance) > 1e-12 else 0.
        lead_observed = bool(self._start_depth is not None and advance >= .98*self.pitch
            and rotation_since_start >= 2*np.pi-.03 and lead_relative_error <= .02)
        return {"tip_depth_from_entry_m": depth, "axis_radial_offset_at_entry_m": radial, "axis_tilt_rad": tilt,
            "left_grasp_position_slip_m": left_slip, "left_grasp_rotation_slip_rad": left_rotation,
            "right_grasp_position_slip_m": right_slip, "right_grasp_rotation_slip_rad": right_rotation,
            "left_block_grasp_secure": left_secure, "right_bolt_grasp_secure": right_secure,
            "left_pads_loaded": left_loaded, "right_pads_loaded": right_loaded,
            "left_grasp_reference_observed": self._left_reference is not None,
            "right_grasp_reference_observed": self._right_reference is not None,
            "loaded_flank_engagement_candidate": self._start_depth is not None,
            "thread_started": lead_observed, "one_turn_lead_observed": lead_observed,
            "whole_ring_full_flank_length_m": interval["length_m"],
            "whole_ring_interval_in_male_m": [interval["start_m"], interval["end_m"]],
            "observed_engaged_advance_m": advance, "relative_rotation_since_engagement_rad": rotation_since_start,
            "observed_lead_residual_m": residual, "observed_lead_relative_error": lead_relative_error}

    def _observe_task_events(self, contacts):
        self._ever_rest_support |= contacts["bolt_rest_support_contacts"] > 0
        self._ever_block_world_support |= contacts["block_world_support_contacts"] > 0
        left_loaded = bool(np.all(contacts["pad_normals"][:2] > .1))
        if self.pickup_from_table and left_loaded and not self._left_loaded_last:
            # Before lift, renewed bilateral contact can establish an
            # acquisition reference. After observed pickup it is frozen, so
            # opening, dropping, or slipping cannot erase measured retention.
            if not self._left_pickup_observed:
                self._left_reference = self._held_pose("left")
            if not self._left_acquisition_observed:
                self._left_acquisition_observed = True
                self._left_acquisition_time = float(self.data.time)
        self._left_loaded_last = left_loaded
        right_loaded = bool(np.all(contacts["pad_normals"][2:] > .1))
        if right_loaded and not self._right_loaded_last:
            self._right_reference = self._held_pose("right")
            if self._right_acquisition_time is None:
                self._right_acquisition_time = float(self.data.time)
        self._right_loaded_last = right_loaded
        metrics = self._metrics(contacts)
        if (self.pickup_from_table and not self._left_pickup_observed
                and self._ever_block_world_support and self._left_acquisition_observed
                and metrics["left_block_grasp_secure"]
                and self._free_pose(self.block_qadr)[0][2]-self._initial_block_height >= .003):
            self._left_pickup_observed = True
            self._left_pickup_time = float(self.data.time)
            # Final retention reference belongs to an observed, unsupported
            # pickup, never to the open hand at initialization.
            self._left_reference = self._held_pose("left")
        if (not self._right_pickup_observed and self._ever_rest_support
                and metrics["right_bolt_grasp_secure"]
                and self._head_position()[2]-self._initial_head_height >= .005):
            self._right_pickup_observed = True
            self._right_pickup_time = float(self.data.time)
        left_ready = self._left_pickup_observed if self.pickup_from_table else True
        if (left_ready and self._right_pickup_observed and metrics["left_block_grasp_secure"]
                and metrics["right_bolt_grasp_secure"]
                and (self.pickup_from_table or self._head_position()[2]-self._initial_head_height >= .005)):
            self._pickup_observed = True
        valid_geometry = (self._pickup_observed and metrics["left_block_grasp_secure"]
            and metrics["right_bolt_grasp_secure"]
            and metrics["whole_ring_full_flank_length_m"] > self.pitch
            and metrics["axis_radial_offset_at_entry_m"] <= 150e-6 and metrics["axis_tilt_rad"] <= np.deg2rad(2.))
        # Use the same base-position helix phase and rolling observer as the
        # frozen demo. Normal impulse is measured load, not axial force balance.
        relative_position = self._relative_pose()[0]
        self._engagement_metrics = self.engagement_observer.update(float(self.data.time),
            self.model.opt.timestep, valid_geometry, contacts["loaded_full_flank_normal_force_N"],
            contacts["loaded_full_flank_contact_count"],
            float(relative_position[2])-self.pitch*self._yaw_total/(2*np.pi))
        if self._start_depth is None and self._engagement_metrics["ready"]:
            self._start_depth, self._start_yaw = metrics["tip_depth_from_entry_m"], self._yaw_total
            self._last_scored_depth = self._start_depth

    def _failures(self, contacts):
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            return ["nonfinite_state"]
        reasons = []
        joint_margin = self._native_joint_margin()
        self._minimum_joint_margin = min(self._minimum_joint_margin, joint_margin)
        if joint_margin < -1e-5:
            reasons.append("native_arm_joint_range_violation")
        if contacts["thread_depth"] > 10e-6:
            reasons.append("reported_thread_sdf_depth_above_10_um")
        if contacts["rigid_workpiece_depth"] > 50e-6:
            reasons.append("rigid_workpiece_contact_depth_above_50_um")
        if sum(w.number for w in self.data.warning) > self._warnings:
            reasons.append("solver_warning")
        for name, body, address in (("block", self.block_id, self.block_dadr), ("bolt", self.male_id, self.male_dadr)):
            if np.any(self.data.xfrc_applied[body]) or np.any(self.data.qfrc_applied[address:address+6]):
                reasons.append(name+"_external_drive_present")
            p = self._free_pose(self.block_qadr if name == "block" else self.male_qadr)[0]
            if not (-.2 <= p[0] <= 1. and -.75 <= p[1] <= .75 and -.03 <= p[2] <= 1.):
                reasons.append(name+"_outside_workspace")
        if np.any(self.data.xfrc_applied[self.female_id]):
            reasons.append("female_frame_external_drive_present")
        if self.pickup_from_table:
            if self._left_pickup_observed and contacts["block_world_support_contacts"]:
                reasons.append("block_world_support_after_pickup")
            if self._right_pickup_observed and contacts["bolt_world_support_contacts"]:
                reasons.append("bolt_world_support_after_pickup")
        if self._start_depth is not None:
            metrics = self._metrics(contacts)
            if metrics["axis_radial_offset_at_entry_m"] > 150e-6:
                reasons.append("engaged_radial_offset_above_150_um")
            if metrics["axis_tilt_rad"] > np.deg2rad(2.):
                reasons.append("engaged_tilt_above_2_degrees")
            if abs(metrics["observed_lead_residual_m"]) > 150e-6:
                reasons.append("observed_lead_residual_above_150_um")
        return reasons

    def _native_joint_margin(self):
        positions = self.data.qpos[self.arm_qpos]
        ranges = self.model.jnt_range[self.arm_joints]
        return float(np.min(np.minimum(positions-ranges[:, 0], ranges[:, 1]-positions)))

    def step(self, action):
        if not self._active:
            raise RuntimeError("Call reset before step, including after an episode ends")
        action = np.asarray(action, dtype=float)
        if action.shape != (14,) or not np.isfinite(action).all():
            raise ValueError("Action must contain fourteen finite entries")
        action = np.clip(action, -1., 1.)
        cfg = self.scene_config.base
        closed = np.array([cfg.left_closed_aperture, cfg.closed_aperture])
        self.last_apertures[:] = cfg.open_aperture-(action[12:]+1)/2*(cfg.open_aperture-closed)
        picked_before, started_before = self._pickup_observed, self._start_depth is not None
        reasons = []
        for _ in range(self.frame_skip):
            torques = action[:12]*self.torque_caps
            if self.gravity_compensation:
                torques += self.data.qfrc_bias[self.arm_dofs]
            self.last_motor_torques[:] = np.clip(torques, self.motor_limits[:, 0], self.motor_limits[:, 1])
            self.data.ctrl[self.arm_motors] = self.last_motor_torques
            self.data.ctrl[self.finger_actuators] = np.ravel([jaw_positions(g, self.scene_config) for g in self.last_apertures])
            # Policy rollouts write only real motor controls, never free bodies.
            mujoco.mj_step(self.model, self.data)
            yaw = self._relative_yaw()
            self._yaw_total += float((yaw-self._last_yaw+np.pi) % (2*np.pi)-np.pi)
            self._last_yaw = yaw
            contacts = self._contacts()
            self._peak_thread_depth = max(self._peak_thread_depth, contacts["thread_depth"])
            self._observe_task_events(contacts)
            reasons = self._failures(contacts)
            if reasons:
                break
        if np.isfinite(self.data.qpos).all() and np.isfinite(self.data.qvel).all():
            mujoco.mj_forward(self.model, self.data)
            contacts = self._contacts()
            self._peak_thread_depth = max(self._peak_thread_depth, contacts["thread_depth"])
            reasons = list(dict.fromkeys(reasons+self._failures(contacts)))
        self._steps += 1
        info = self._info()
        valid = (not reasons and info["pickup_observed"] and info["engagement_window"]["ready"]
                 and info["whole_ring_full_flank_length_m"] > self.pitch and info["loaded_flank_engagement_candidate"]
                 and info["left_block_grasp_secure"] and info["right_bolt_grasp_secure"]
                 and info["axis_radial_offset_at_entry_m"] <= 150e-6 and info["axis_tilt_rad"] <= np.deg2rad(2.))
        self._valid_contact_controls = self._valid_contact_controls+1 if valid else 0
        success = bool(valid and self._valid_contact_controls >= 10
            and info["tip_depth_from_entry_m"] >= self.target_tip_depth
            and info["observed_engaged_advance_m"] >= self.pitch
            and info["relative_rotation_since_engagement_rad"] >= 2*np.pi
            and info["one_turn_lead_observed"])
        reward = -.0002*float(np.dot(action[:12], action[:12]))
        if valid and self._last_scored_depth is not None:
            reward += (info["tip_depth_from_entry_m"]-self._last_scored_depth)/self.pitch
        if self._start_depth is not None:
            self._last_scored_depth = info["tip_depth_from_entry_m"]
        reward += .25*int(self._pickup_observed and not picked_before)+.25*int(info["loaded_flank_engagement_candidate"] and not started_before)
        if reasons:
            reward = -1.
        elif success:
            reward += 1.
        terminated, truncated = bool(reasons or success), bool(self._steps >= self.max_steps and not (reasons or success))
        info.update(success=success, failure=bool(reasons), failure_reasons=reasons,
            target_tip_depth_m=self.target_tip_depth, valid_contact_control_steps=self._valid_contact_controls,
            commanded_arm_motor_torques_Nm=self.last_motor_torques.tolist(), commanded_apertures_m=self.last_apertures.tolist())
        if terminated or truncated:
            self._active = False
        return self._observation(), float(reward), terminated, truncated, info

    def _velocity(self, body):
        result = np.empty(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, body, result, 0)
        return result

    def _relative_velocity(self):
        mv, fv = self._velocity(self.male_id), self._velocity(self.female_id)
        fr = self.data.xmat[self.female_id].reshape(3, 3)
        delta = self.data.xpos[self.male_id]-self.data.xpos[self.female_id]
        return np.r_[fr.T@(mv[3:]-fv[3:]-np.cross(fv[:3], delta)), fr.T@(mv[:3]-fv[:3])]

    def _info(self):
        c = self._contacts()
        return {"starts_preengaged": False, "pickup_observed": self._pickup_observed,
            "pickup_from_table": self.pickup_from_table,
            "left_pad_acquisition_observed": self._left_acquisition_observed,
            "right_pad_acquisition_observed": self._right_acquisition_time is not None,
            "left_block_pickup_observed": self._left_pickup_observed,
            "right_bolt_pickup_observed": self._right_pickup_observed,
            "block_world_support_observed": self._ever_block_world_support,
            "left_pad_acquisition_time_s": self._left_acquisition_time,
            "right_pad_acquisition_time_s": self._right_acquisition_time,
            "left_block_pickup_time_s": self._left_pickup_time,
            "right_bolt_pickup_time_s": self._right_pickup_time,
            "left_grasp_reference_frozen_after_pickup": self.pickup_from_table and self._left_pickup_observed,
            "minimum_block_pickup_lift_m": .003,
            "minimum_bolt_pickup_lift_m": .005,
            "block_lift_from_reset_m": float(self._free_pose(self.block_qadr)[0][2]-self._initial_block_height),
            "bolt_head_lift_from_reset_m": float(self._head_position()[2]-self._initial_head_height),
            "rest_support_observed": self._ever_rest_support, **self._metrics(c),
            "thread_contact_count": c["thread_count"], "worst_reported_thread_sdf_depth_m": c["thread_depth"],
            "loaded_full_flank_contact_count": c["loaded_full_flank_contact_count"],
            "loaded_full_flank_normal_force_N": c["loaded_full_flank_normal_force_N"],
            "engagement_candidate_sustain_seconds": self.engagement_sustain_s,
            "engagement_observer": {"version": self.engagement_observer.version,
                "source_sha256": self.engagement_observer_source_sha256,
                "duration_s": self.engagement_observer.duration_s,
                "minimum_normal_impulse_Ns": self.engagement_observer.minimum_normal_impulse_Ns,
                "minimum_loaded_duration_s": self.engagement_observer.minimum_loaded_duration_s,
                "maximum_helix_phase_range_m": self.engagement_observer.maximum_helix_phase_range_m},
            "engagement_window": deepcopy(self._engagement_metrics),
            "current_valid_flank_geometry_window_seconds": self._engagement_metrics["continuous_geometry_elapsed_s"],
            "window_loaded_full_flank_substeps": self._engagement_metrics["loaded_substeps"],
            "window_full_flank_normal_impulse_Ns": self._engagement_metrics["normal_impulse_Ns"],
            "window_helix_phase_range_m": self._engagement_metrics["helix_phase_range_m"],
            "recent_full_flank_normal_impulse_Ns": self._engagement_metrics["normal_impulse_Ns"],
            "normal_impulse_note": "Resolved normal-contact impulse; not net axial support or momentum balance",
            "episode_peak_reported_thread_sdf_depth_m": self._peak_thread_depth,
            "block_world_support_contacts": c["block_world_support_contacts"],
            "bolt_world_support_contacts": c["bolt_world_support_contacts"],
            "bolt_rest_support_contacts": c["bolt_rest_support_contacts"],
            "pad_normal_forces_N": c["pad_normals"].tolist(),
            "elapsed_seconds": float(self.data.time), "control_dt": self.control_dt,
            "model_timestep_s": float(self.model.opt.timestep),
            "minimum_native_joint_margin_rad": self._native_joint_margin(),
            "episode_minimum_native_joint_margin_rad": self._minimum_joint_margin,
            "native_joint_limit_violation_tolerance_rad": 1e-5,
            "gravity_bias_compensation": self.gravity_compensation,
            "privileged_state_observations": True, "observation_fields": dict(OBSERVATION_FIELDS),
            "pickup_history_observation_note": (
                "Separate left/right pickup and acquisition history is in info; "
                "the unchanged 118-vector contains their combined pickup marker, "
                "so a policy needs observation history to recover the separate history"),
            "runtime": deepcopy(self.runtime_info),
            "reported_sdf_depth_note": "Native distance proxy; not an independent geometric overlap certificate",
            "physics_scope": "Unengaged rigid-body pickup/start task; actual YAM motors; no trained policy or thread-start qualification"}

    def _observation(self):
        c, metrics = self._contacts(), self._metrics()
        block_p, block_r = self._free_pose(self.block_qadr)
        male_p, male_r = self._free_pose(self.male_qadr)
        female_p, female_r = self._female_pose()
        poses = []
        for body, p, r in ((self.block_id, block_p, block_r), (self.male_id, male_p, male_r),
                           (self.female_id, female_p, female_r)):
            v = self._velocity(body)
            poses.extend([p, _quaternion(r), v[3:], v[:3]])
        rp, rr = self._relative_pose()
        lp, lr = self._held_pose("left")
        hp, hr = self._held_pose("right")
        obs = np.concatenate([self.data.qpos[self.arm_qpos], self.data.qvel[self.arm_dofs],
            self.data.qpos[self.finger_qpos], self.data.qvel[self.finger_dofs], *poses,
            rp, _quaternion(rr), self._relative_velocity(), lp, _quaternion(lr), hp, _quaternion(hr),
            [c["thread_count"], c["thread_depth"]], c["pad_normals"],
            [c["block_world_support_contacts"], c["bolt_world_support_contacts"]],
            [metrics["tip_depth_from_entry_m"], metrics["axis_radial_offset_at_entry_m"], metrics["axis_tilt_rad"]],
            [metrics["left_grasp_position_slip_m"], metrics["left_grasp_rotation_slip_rad"],
             metrics["right_grasp_position_slip_m"], metrics["right_grasp_rotation_slip_rad"]],
            [float(self._ever_rest_support), float(self._pickup_observed),
             float(metrics["loaded_flank_engagement_candidate"]), metrics["observed_engaged_advance_m"], metrics["observed_lead_residual_m"]]])
        return obs.astype(np.float32)

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
