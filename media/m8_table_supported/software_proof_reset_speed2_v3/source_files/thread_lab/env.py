"""Gymnasium task for a genuinely contact-driven, *pre-engaged* M8 nut.

This environment does not establish that a policy can find or start a thread.
There is no learned policy bundled with it. It supplies bounded hand-wrench and
dynamic-jaw actions, state observations, and a transparent geometric task reward.
"""

from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
from dataclasses import replace
from typing import Any

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

from .gripper import GripperConfig, ParallelJawController


OBSERVATION_FIELDS = OrderedDict([
    ("nut_position_in_hand", 3),
    ("nut_quaternion_in_hand_wxyz", 4),
    ("nut_relative_velocity_in_hand_linear_angular", 6),
    ("bolt_position_in_hand", 3),
    ("bolt_quaternion_in_hand_wxyz", 4),
    ("jaw_positions", 2),
    ("jaw_velocities", 2),
    ("pad_wrench_on_nut_in_hand_force_torque", 6),
    ("pad_normal_forces", 2),
    ("hand_quaternion_world_wxyz", 4),
    ("hand_velocity_world_linear_angular", 6),
])


def _relative_quat(child, parent):
    inverse = np.zeros(4)
    relative = np.zeros(4)
    mujoco.mju_negQuat(inverse, parent)
    mujoco.mju_mulQuat(relative, inverse, child)
    if relative[0] < 0:
        relative *= -1
    return relative


def _thread_attributes(model, geom_id):
    """Read the packed attributes of this project's fixed eight-field SDF ABI."""
    plugin_id = int(model.geom_plugin[geom_id])
    if plugin_id < 0:
        raise ValueError("The policy task requires the project's thread SDF geometry")
    begin = int(model.plugin_attradr[plugin_id])
    end = int(model.plugin_attradr[plugin_id + 1]) if plugin_id + 1 < model.nplugin else model.npluginattr
    fields = bytes(model.plugin_attr[begin:end]).split(b"\0")[:8]
    keys = ("diameter", "pitch", "length", "pitch_diameter", "af", "chamfer", "female", "phase")
    if len(fields) != len(keys):
        raise ValueError("Unexpected thread plugin attribute layout")
    return dict(zip(keys, (x.decode("ascii") for x in fields)))


def _compiled_gripper_config(model, requested):
    """Prevent controller configuration from disagreeing with compiled jaws."""
    from .gripper import GripNames
    names = GripNames()
    first_pad = model.geom(names.pad_geoms[0]).id
    first_act = model.actuator(names.jaw_actuators[0]).id
    inferred = replace(requested or GripperConfig(),
                       pad_friction=float(model.geom_friction[first_pad, 0]),
                       maximum_jaw_force=float(model.actuator_forcerange[first_act, 1]),
                       jaw_stiffness=float(model.actuator_gainprm[first_act, 0]),
                       jaw_damping=float(-model.actuator_biasprm[first_act, 2]))
    if requested is not None:
        for name in ("pad_friction", "maximum_jaw_force", "jaw_stiffness", "jaw_damping"):
            if not np.isclose(getattr(requested, name), getattr(inferred, name)):
                raise ValueError(f"gripper_config.{name} disagrees with the compiled model")
    for pad, actuator in zip(names.pad_geoms, names.jaw_actuators):
        gid, aid = model.geom(pad).id, model.actuator(actuator).id
        values = [model.geom_friction[gid, 0], model.actuator_forcerange[aid, 1],
                  model.actuator_gainprm[aid, 0], -model.actuator_biasprm[aid, 2]]
        expected = [inferred.pad_friction, inferred.maximum_jaw_force,
                    inferred.jaw_stiffness, inferred.jaw_damping]
        if not np.allclose(values, expected):
            raise ValueError("This environment requires matching physical parameters for both jaws")
    return inferred


class M8NutEnv(gym.Env):
    """CPU MuJoCo single-environment policy interface.

    Action: seven normalized entries in [-1, 1]. Entries 0..2 are world force,
    3..5 world torque, and 6 grip closure (-1 fully open, +1 fully closed).
    Gravity compensation is an optional, explicit hand-only wrench baseline.
    Forces/torques act at the hand body's CoM and retain controller saturation.

    A reset starts the nut already inside an existing thread engagement. Small
    alignment randomizations challenge turning and grip; they are not an
    end-to-end nut acquisition/thread-start benchmark.
    The default factory uses a 25 microsecond physics step: this is the
    resolution validated by the slow 1 N loaded thread-turning benchmarks.
    Supplying a model or model_kwargs can override it for explicit experiments.
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 50}

    def __init__(self, model: mujoco.MjModel | None = None, *,
                 control_dt: float = .02, horizon_seconds: float = 6.,
                 target_advance: float = .0025, pitch: float | None = None,
                 alignment_randomization: bool = True,
                 gravity_compensation: bool = True,
                 gripper_config: GripperConfig | None = None,
                 render_mode: str | None = None, model_kwargs: dict | None = None,
                 allow_stock_engine: bool = False):
        super().__init__()
        from .runtime import engine_info, require_micron_engine
        self.runtime_info = engine_info() if allow_stock_engine else require_micron_engine()
        if model is None:
            # Root model's public factory is used only when no model is supplied.
            from .model import ThreadConfig, make_model
            kwargs = {"with_gripper": True, "timestep": .000025, **(model_kwargs or {})}
            model = make_model(ThreadConfig(**kwargs), gripper_config=gripper_config)
        if control_dt <= 0 or horizon_seconds <= 0 or target_advance <= 0:
            raise ValueError("Timing, pitch and target advance must be positive")
        if render_mode not in (None, "rgb_array"):
            raise ValueError("Only rgb_array rendering is supported")
        self.model = model
        self.data = mujoco.MjData(model)
        self.hand = ParallelJawController(model, self.data, _compiled_gripper_config(model, gripper_config))
        self.nut_id = model.body("nut").id
        self.bolt_geom_id = model.geom("bolt_thread").id
        self.nut_geom_id = model.geom("nut_thread").id
        self.bolt_id = int(model.geom_bodyid[self.bolt_geom_id])
        nut_joints = np.flatnonzero(model.jnt_bodyid == self.nut_id)
        free_joints = [int(j) for j in nut_joints if model.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE]
        if len(free_joints) != 1:
            raise ValueError("Policy task requires exactly one free 6-DOF nut joint")
        self.nut_joint_id = free_joints[0]
        self.nut_qpos_addr = int(model.jnt_qposadr[self.nut_joint_id])
        self.nut_dof_addr = int(model.jnt_dofadr[self.nut_joint_id])
        self.frame_skip = max(1, round(control_dt / model.opt.timestep))
        self.control_dt = float(self.frame_skip * model.opt.timestep)
        self.max_steps = max(1, round(horizon_seconds / self.control_dt))
        bolt_attrs = _thread_attributes(model, self.bolt_geom_id)
        nut_attrs = _thread_attributes(model, self.nut_geom_id)
        actual_pitch = float(bolt_attrs["pitch"])
        if not np.isclose(actual_pitch, float(nut_attrs["pitch"]), rtol=0, atol=1e-12):
            raise ValueError("Nut and bolt pitch must agree for this task")
        if pitch is not None and not np.isclose(pitch, actual_pitch, rtol=0, atol=1e-12):
            raise ValueError("Reward pitch disagrees with the actual thread geometry")
        self.target_advance, self.pitch = float(target_advance), actual_pitch
        self.bolt_length, self.nut_height = float(bolt_attrs["length"]), float(nut_attrs["length"])
        self.alignment_randomization = alignment_randomization
        self.gravity_compensation = gravity_compensation
        self.render_mode = render_mode
        self.action_space = spaces.Box(-1., 1., shape=(7,), dtype=np.float32)
        self.observation_space = spaces.Box(-np.inf, np.inf,
                                           shape=(sum(OBSERVATION_FIELDS.values()),), dtype=np.float32)
        self._renderer = None
        self._steps = 0
        self._initial_nut_z = 0.
        self._last_advance = 0.
        self._initial_warnings = 0
        self._active = False
        self._peak_thread_penetration = 0.
        self._unwrapped_yaw = 0.
        self._last_yaw = 0.
        self._recent_valid_contact_steps = 0
        self._ever_thread_contact = False
        self._thread_gap_substeps = 0

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        options = options or {}
        mujoco.mj_resetData(self.model, self.data)
        addr = self.nut_qpos_addr
        randomize = options.get("alignment_randomization", self.alignment_randomization)
        offsets = np.zeros(2)
        angles = np.zeros(3)
        if randomize:
            offsets = self.np_random.uniform(-30e-6, 30e-6, size=2)
            angles[:2] = self.np_random.uniform(-np.deg2rad(.2), np.deg2rad(.2), size=2)
            angles[2] = self.np_random.uniform(-.02, .02)
            self.data.qpos[addr:addr + 2] += offsets
            qperturb = np.zeros(4)
            mujoco.mju_euler2Quat(qperturb, angles, "xyz")
            qinitial = self.data.qpos[addr + 3:addr + 7].copy()
            mujoco.mju_mulQuat(self.data.qpos[addr + 3:addr + 7], qperturb, qinitial)
        # Hand reset initializes only hand and jaw states. It leaves the nut
        # perturbation intact and begins with a genuinely open contact grasp.
        self.hand.reset(aperture=self.hand.config.open_aperture)
        mujoco.mj_forward(self.model, self.data)
        self._initial_nut_z = float(self.data.xpos[self.nut_id, 2])
        self._last_advance = 0.
        self._steps = 0
        self._initial_warnings = int(sum(w.number for w in self.data.warning))
        self._peak_thread_penetration = 0.
        self._unwrapped_yaw = 0.
        self._last_yaw = self._nut_yaw()
        self._recent_valid_contact_steps = 0
        self._ever_thread_contact = False
        self._thread_gap_substeps = 0
        self._active = True
        info = self._info()
        info["reset_randomization"] = {"xy_offset_m": offsets.tolist(), "xyz_rotation_rad": angles.tolist()}
        info["starts_preengaged"] = True
        return self._observation(), info

    def _gravity_wrench(self):
        result = np.zeros(6)
        if self.gravity_compensation:
            result[:3] = -self.hand.total_mass * self.model.opt.gravity
            for name in self.hand.names.jaw_bodies:
                jid = self.model.body(name).id
                arm = self.data.xipos[jid] - self.data.xipos[self.hand.body_id]
                result[3:] -= np.cross(arm, self.model.body_mass[jid] * self.model.opt.gravity)
        return result

    def step(self, action):
        if not self._active:
            raise RuntimeError("Call reset before step, including after a terminated episode")
        action = np.asarray(action, dtype=float)
        if action.shape != (7,) or not np.all(np.isfinite(action)):
            raise ValueError("Action must have seven finite entries")
        action = np.clip(action, -1, 1)
        cfg = self.hand.config
        wrench = np.r_[action[:3] * cfg.maximum_force,
                       action[3:6] * cfg.maximum_torque]
        # Normalized +1 requests10mm; an M8 nut stops the jaws before that and
        # their force caps retain finite force. -1 requests the17mm open gap.
        aperture = .017 - .0035 * (action[6] + 1)
        failure_reasons = []
        for _ in range(self.frame_skip):
            self.hand.apply_action(wrench + self._gravity_wrench(), aperture)
            # No nut qpos/qvel/ctrl/xfrc assignments occur in a rollout.
            mujoco.mj_step(self.model, self.data)
            now_yaw = self._nut_yaw()
            self._unwrapped_yaw += float((now_yaw - self._last_yaw + np.pi) % (2 * np.pi) - np.pi)
            self._last_yaw = now_yaw
            penetration = self._thread_penetration()
            self._peak_thread_penetration = max(self._peak_thread_penetration, penetration)
            geom = self.data.contact.geom
            has_thread_contact = np.any(((geom[:, 0] == self.bolt_geom_id) & (geom[:, 1] == self.nut_geom_id))
                                        | ((geom[:, 1] == self.bolt_geom_id) & (geom[:, 0] == self.nut_geom_id)))
            if has_thread_contact:
                self._ever_thread_contact = True
                self._thread_gap_substeps = 0
            elif self._ever_thread_contact:
                self._thread_gap_substeps += 1
            failure_reasons = self._failure_reasons(penetration)
            if failure_reasons:
                break
        self._steps += 1
        info = self._info()
        advance = info["axial_advance_m"]
        progress = (advance - self._last_advance) / self.pitch
        self._last_advance = advance
        failure = bool(failure_reasons)
        valid_contact = (not failure and info["bolt_nut_contact_count"] > 0
                         and info["pad_contact"]["pad_contact_count"] > 0
                         and info["radial_offset_m"] <= 150e-6
                         and info["nut_tilt_rad"] <= np.deg2rad(2)
                         and abs(info["observed_helix_residual_m"]) <= 150e-6)
        self._recent_valid_contact_steps = self._recent_valid_contact_steps + 1 if valid_contact else 0
        success = bool(advance >= self.target_advance and valid_contact
                       and self._recent_valid_contact_steps >= 10)
        # Invalid physics cannot yield a large positive plunge-progress reward.
        reward = -1. if failure else progress - .002 * float(np.dot(action[:6], action[:6]))
        terminated = bool(success or failure)
        truncated = bool(self._steps >= self.max_steps and not terminated)
        if success:
            reward += 1.
        info.update(success=bool(success), failure=bool(failure),
                    failure_reasons=failure_reasons,
                    recent_valid_contact_steps=self._recent_valid_contact_steps,
                    target_advance_m=self.target_advance,
                    action_world_wrench=self.hand.last_wrench.tolist(),
                    commanded_aperture_m=float(aperture))
        if terminated or truncated:
            self._active = False
        return self._observation(), float(reward), terminated, truncated, info

    def _nut_yaw(self):
        w, x, y, z = self.data.qpos[self.nut_qpos_addr + 3:self.nut_qpos_addr + 7]
        return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

    def _thread_penetration(self):
        geom, dist = self.data.contact.geom, self.data.contact.dist
        mask = (((geom[:, 0] == self.bolt_geom_id) & (geom[:, 1] == self.nut_geom_id))
                | ((geom[:, 1] == self.bolt_geom_id) & (geom[:, 0] == self.nut_geom_id)))
        return float(max(0., -np.min(dist[mask]))) if np.any(mask) else 0.

    def _failure_reasons(self, penetration):
        reasons = []
        if not (np.all(np.isfinite(self.data.qpos)) and np.all(np.isfinite(self.data.qvel))):
            return ["nonfinite_state"]
        # Native SDF-SDF contact dist is a solver depth proxy, not a certified
        # geometric overlap bound; planar overlap tests report half the overlap.
        if penetration > 10e-6:
            reasons.append("reported_sdf_depth_above_10_um")
        nut = self.data.qpos[self.nut_qpos_addr:self.nut_qpos_addr + 3]
        bolt = self.data.geom_xpos[self.bolt_geom_id]
        nut_axis = self.data.xmat[self.nut_id].reshape(3, 3)[:, 2]
        bolt_axis = self.data.geom_xmat[self.bolt_geom_id].reshape(3, 3)[:, 2]
        axial = float(np.dot(nut - bolt, bolt_axis))
        if np.linalg.norm((nut - bolt)[:2]) > .002:
            reasons.append("nut_detached_radially")
        if np.dot(nut_axis, bolt_axis) < np.cos(np.deg2rad(15)):
            reasons.append("nut_tilt_above_15_degrees")
        if not (-self.nut_height / 2 < axial < self.bolt_length + self.nut_height / 2):
            reasons.append("nut_outside_bolt_axially")
        if sum(w.number for w in self.data.warning) > self._initial_warnings:
            reasons.append("solver_warning")
        advance = self._initial_nut_z - nut[2]
        helix_residual = advance + self.pitch * self._unwrapped_yaw / (2 * np.pi)
        if abs(helix_residual) > 150e-6:
            reasons.append("observed_helix_residual_above_150_um")
        # With the specified fit, changing axial load can traverse about 97µm
        # of backlash without a loaded flank contact. A duration-only contact
        # gap does not establish a physics failure. Geometric phase, alignment,
        # depth, solver and finite-state gates remain active; success still
        # requires consecutive actual thread and pad contacts.
        return reasons

    def _velocity(self, body_id):
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY, body_id, vel, 0)
        return vel

    def _observation(self):
        data = self.data
        hid, nid = self.hand.body_id, self.nut_id
        rotate = data.xmat[hid].reshape(3, 3).T
        delta = data.xpos[nid] - data.xpos[hid]
        hv, nv = self._velocity(hid), self._velocity(nid)
        relative_v = rotate @ (nv[3:] - hv[3:] - np.cross(hv[:3], delta))
        relative_w = rotate @ (nv[:3] - hv[:3])
        jaw_q = [data.qpos[self.model.jnt_qposadr[j]] for j in self.hand.jaw_joint_ids]
        jaw_v = [data.qvel[self.model.jnt_dofadr[j]] for j in self.hand.jaw_joint_ids]
        contact = self.hand.contact_wrench_on("nut")
        wrench = np.asarray(contact["pad_wrench_world"])
        bolt_q = np.zeros(4)
        mujoco.mju_mat2Quat(bolt_q, data.geom_xmat[self.bolt_geom_id])
        obs = np.concatenate([
            rotate @ delta, _relative_quat(data.xquat[nid], data.xquat[hid]),
            relative_v, relative_w,
            rotate @ (data.geom_xpos[self.bolt_geom_id] - data.xpos[hid]),
            _relative_quat(bolt_q, data.xquat[hid]),
            jaw_q, jaw_v, rotate @ wrench[:3], rotate @ wrench[3:],
            contact["pad_normal_force_N"], data.xquat[hid], hv[3:], hv[:3],
        ])
        return obs.astype(np.float32)

    def _info(self):
        nut = self.data.xpos[self.nut_id]
        bolt = self.data.geom_xpos[self.bolt_geom_id]
        nut_axis = self.data.xmat[self.nut_id].reshape(3, 3)[:, 2]
        bolt_axis = self.data.geom_xmat[self.bolt_geom_id].reshape(3, 3)[:, 2]
        bolt_contacts = 0
        worst_penetration = 0.
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            geoms = {int(c.geom1), int(c.geom2)}
            if self.bolt_geom_id in geoms:
                other = int(c.geom2) if int(c.geom1) == self.bolt_geom_id else int(c.geom1)
                if self.model.geom_bodyid[other] == self.nut_id:
                    bolt_contacts += 1
                    worst_penetration = max(worst_penetration, -float(c.dist))
        return {
            "starts_preengaged": True,
            "axial_advance_m": float(self._initial_nut_z - nut[2]),
            "nut_unwrapped_rotation_rad": self._unwrapped_yaw,
            "observed_helix_residual_m": float(self._initial_nut_z - nut[2] + self.pitch * self._unwrapped_yaw / (2 * np.pi)),
            "radial_offset_m": float(np.linalg.norm((nut - bolt)[:2])),
            "nut_tilt_rad": float(np.arccos(np.clip(np.dot(nut_axis, bolt_axis), -1, 1))),
            "nut_center_along_bolt_m": float(np.dot(nut - bolt, bolt_axis)),
            "bolt_nut_contact_count": int(bolt_contacts),
            "worst_reported_sdf_depth_m": max(0., worst_penetration),
            "episode_peak_reported_sdf_depth_m": self._peak_thread_penetration,
            "reported_sdf_depth_note": "Native contact dist proxy; not a geometric overlap certificate",
            "thread_contact_gap_seconds": float(self._thread_gap_substeps * self.model.opt.timestep),
            "pad_contact": self.hand.contact_wrench_on("nut"),
            "solver_warning_count": int(sum(w.number for w in self.data.warning)),
            "elapsed_seconds": float(self.data.time),
            "control_dt": self.control_dt,
            "model_timestep_s": float(self.model.opt.timestep),
            "observation_fields": dict(OBSERVATION_FIELDS),
            "physics_scope": "Rigid-body SDF thread contact; pre-engaged nut; proxy frictional pads; no thread-start validation",
            "runtime": deepcopy(self.runtime_info),
        }

    def render(self):
        if self.render_mode != "rgb_array":
            return None
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, height=480, width=640)
        camera = mujoco.MjvCamera()
        camera.lookat[:] = self.data.xpos[self.nut_id]
        camera.distance, camera.azimuth, camera.elevation = .085, 125, -22
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render().copy()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


def policy_action_stress(output="outputs/m8/policy_action_stress.json", *, subset=None):
    """Short bounded action probes; not a validation of training fidelity.

    Reuse a physically settled open/closed state as an independent reset fixture
    to avoid repeating expensive settling. No state is changed during a pulse.
    Corner cases may legitimately fail the physics gates; invalid progress must
    then earn -1, stop immediately, and never report a success.
    """
    import itertools
    import json
    from pathlib import Path
    import time

    env = M8NutEnv(alignment_randomization=False, control_dt=.001,
                   horizon_seconds=.5)
    wall_start = time.perf_counter()
    env.reset(seed=1)
    # Let the passive nut settle into a flank under gravity first.
    preparation_failure = None
    for _ in range(100):
        _, _, terminated, _, info = env.step([0, 0, 0, 0, 0, 0, -1])
        if terminated:
            preparation_failure = info
            break
    open_state = mujoco.MjData(env.model)
    mujoco.mj_copyData(open_state, env.model, env.data)
    close_action = 2 * (.017 - .0124) / .007 - 1
    if preparation_failure is None:
        for i in range(150):
            grip = -1 + min(1., (i + 1) / 100) * (close_action + 1)
            _, _, terminated, truncated, info = env.step([0, 0, 0, 0, 0, 0, grip])
            if terminated or truncated:
                preparation_failure = info
                break
    closed_state = mujoco.MjData(env.model)
    mujoco.mj_copyData(closed_state, env.model, env.data)
    prepared_closed_contact = env.hand.contact_wrench_on("nut")
    closed_grasp_prepared = (prepared_closed_contact["pad_contact_count"] >= 2
                             and all(x > 1e-4 for x in prepared_closed_contact["pad_normal_force_N"]))
    cases = []
    requested = []
    if preparation_failure is None:
        for mode in ("open", "closed"):
            grip = -1 if mode == "open" else close_action
            for direction in (-1, 1):
                requested.append((f"{mode}_modest_axial_{direction}", mode,
                                  np.array([0, 0, direction * .2 / 8, 0, 0, 0, grip])))
                requested.append((f"{mode}_modest_yaw_{direction}", mode,
                                  np.array([0, 0, 0, 0, 0, direction * .0002 / .08, grip])))
            # Corners saturate the vector norm, so their individual components
            # cannot exercise pure-axis maximum wrenches. Probe those too.
            for axis in range(6):
                for direction in (-1, 1):
                    action = np.zeros(7)
                    action[axis], action[6] = direction, grip
                    requested.append((f"{mode}_maximum_axis_{axis}_{direction}", mode, action))
            for signs in itertools.product((-1., 1.), repeat=6):
                requested.append((f"{mode}_corner_{''.join('+' if x > 0 else '-' for x in signs)}",
                                  mode, np.array([*signs, grip])))
        if subset == "backlash":
            patterns = ("------", "++++++", "---+++", "+++---")
            requested = [(label, mode, action) for label, mode, action in requested
                         if ("modest" in label or "maximum_axis_2_" in label
                             or "maximum_axis_5_" in label
                             or any(label.endswith("corner_" + value) for value in patterns))]
        elif subset is not None:
            raise ValueError("Unknown policy stress subset")
        for index, (label, mode, action) in enumerate(requested):
            env.reset(seed=1)
            mujoco.mj_copyData(env.data, env.model, open_state if mode == "open" else closed_state)
            env._initial_nut_z = float(env.data.qpos[env.nut_qpos_addr + 2])
            env._last_yaw, env._unwrapped_yaw = env._nut_yaw(), 0.
            env._last_advance, env._steps = 0., 0
            env._peak_thread_penetration, env._recent_valid_contact_steps = 0., 0
            env._ever_thread_contact, env._thread_gap_substeps = False, 0
            initial_time = float(env.data.time)
            rewards = []
            terminated = truncated = False
            capped_force, capped_torque = 0., 0.
            # Exceeds the 10 ms lost-thread-contact guard horizon.
            for _ in range(16):
                _, reward, terminated, truncated, info = env.step(action)
                rewards.append(reward)
                capped_force = max(capped_force, float(np.linalg.norm(env.hand.last_wrench[:3])))
                capped_torque = max(capped_torque, float(np.linalg.norm(env.hand.last_wrench[3:])))
                if terminated or truncated:
                    break
            valid = not info["failure"]
            invalid_safely_stopped = (valid or (terminated and not info["success"] and rewards[-1] == -1))
            nut_unforced = bool(np.all(env.data.xfrc_applied[env.nut_id] == 0))
            result = {"case": label, "action": action.tolist(),
                      "status": "physics_gates_not_triggered" if valid else "controlled_failure",
                      "controls_executed": len(rewards),
                      "pulse_simulated_seconds": float(env.data.time - initial_time),
                      "last_reward": float(rewards[-1]), "reward_sum": float(sum(rewards)),
                      "invalid_safely_stopped": bool(invalid_safely_stopped),
                      "success": bool(info["success"]), "failure_reasons": info["failure_reasons"],
                      "maximum_hand_force_N": capped_force, "maximum_hand_torque_Nm": capped_torque,
                      "episode_peak_reported_sdf_depth_m": info["episode_peak_reported_sdf_depth_m"],
                      "axial_advance_m": info["axial_advance_m"],
                      "observed_helix_residual_m": info["observed_helix_residual_m"],
                      "nut_external_wrench_zero": nut_unforced}
            cases.append(result)
            if index % 16 == 0:
                print(json.dumps({"completed": index + 1, "total": len(requested), "latest": result["status"]}), flush=True)
    report = {"interface_safety_checks_passed": bool(preparation_failure is None and closed_grasp_prepared and all(
                  c["invalid_safely_stopped"] and c["nut_external_wrench_zero"]
                  and not c["success"] and c["maximum_hand_force_N"] <= 8.00000001
                  and c["maximum_hand_torque_Nm"] <= .080000001 for c in cases)),
              "description": "Short policy-action stress; valid gates or controlled failure, not training-fidelity acceptance",
              "runtime": env.runtime_info, "starts_preengaged": True,
              "compiled_model": {"timestep": float(env.model.opt.timestep),
                    "thread_contact_margin_m": float(env.model.pair_margin[0]),
                    "friction_impedance_ratio": float(env.model.opt.impratio),
                    "sdf_initpoints": int(env.model.opt.sdf_initpoints),
                    "sdf_iterations": int(env.model.opt.sdf_iterations)},
              "requested_cases": len(requested), "maximum_pulse_seconds": .016,
              "subset": subset or "all_corners_and_axes",
              "guard_semantics": "Contact-gap duration is diagnostic within fit backlash; geometry/depth/solver gates define failures",
              "prepared_closed_grasp_has_two_loaded_pads": bool(closed_grasp_prepared),
              "prepared_closed_state_pad_contact_count": prepared_closed_contact["pad_contact_count"],
              "prepared_closed_state_pad_normal_force_N": prepared_closed_contact["pad_normal_force_N"],
              "preparation_failure": preparation_failure,
              "physics_gates_not_triggered_cases": sum(c["status"] == "physics_gates_not_triggered" for c in cases),
              "controlled_failure_cases": sum(c["status"] == "controlled_failure" for c in cases),
              "wall_seconds": time.perf_counter() - wall_start, "cases": cases}
    target = Path(output)
    target.parent.mkdir(exist_ok=True, parents=True)
    target.write_text(json.dumps(report, indent=2))
    env.close()
    return report


if __name__ == "__main__":
    import argparse
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stress", action="store_true")
    parser.add_argument("--output", default="outputs/m8/policy_action_stress.json")
    parser.add_argument("--subset", choices=["backlash"], default=None)
    args = parser.parse_args()
    if not args.stress:
        parser.error("Use --stress to run bounded policy-action checks")
    report = policy_action_stress(args.output, subset=args.subset)
    print(json.dumps({k: v for k, v in report.items() if k != "cases"}, indent=2))
    raise SystemExit(0 if report["interface_safety_checks_passed"] else 1)
