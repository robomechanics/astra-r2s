"""A finite-force parallel-jaw end effector with contact-only nut coupling.

The hand is a free rigid body, not a mocap body. A Cartesian impedance controller
may push *that body*. Both jaw joints and the nut remain dynamic. There are no
grasp constraints, nut position commands, or rotation-to-insertion rules here.
"""

from __future__ import annotations

from dataclasses import dataclass
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


@dataclass(frozen=True)
class GripperConfig:
    pad_friction: float = 0.8
    maximum_jaw_force: float = 20.0
    maximum_force: float = 8.0
    maximum_torque: float = 0.08
    jaw_stiffness: float = 20_000.0
    jaw_damping: float = 8.0
    position_stiffness: float = 800.0
    position_damping: float = 14.0
    rotation_stiffness: float = 0.35
    rotation_damping: float = 0.012
    open_aperture: float = 0.017
    closed_aperture: float = 0.0124

    def __post_init__(self):
        for name, value in self.__dict__.items():
            if not np.isfinite(value):
                raise ValueError(f"{name} must be finite")
        for name in ("pad_friction", "maximum_jaw_force", "maximum_force", "maximum_torque",
                     "jaw_damping", "position_damping", "rotation_damping"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} must be nonnegative")
        for name in ("jaw_stiffness", "position_stiffness", "rotation_stiffness"):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if not .010 <= self.closed_aperture <= self.open_aperture <= .017:
            raise ValueError("Apertures must satisfy 10mm <= closed <= open <= 17mm")


@dataclass(frozen=True)
class GripNames:
    body: str = "contact_hand"
    joint: str = "contact_hand_free"
    jaw_bodies: tuple[str, str] = ("contact_jaw_left", "contact_jaw_right")
    jaw_joints: tuple[str, str] = ("contact_jaw_slide_left", "contact_jaw_slide_right")
    pad_geoms: tuple[str, str] = ("contact_pad_left", "contact_pad_right")
    jaw_actuators: tuple[str, str] = ("contact_grip_left", "contact_grip_right")


def _section(root: ET.Element, tag: str) -> ET.Element:
    found = root.find(tag)
    return found if found is not None else ET.SubElement(root, tag)


def add_xml_gripper(
    root: ET.Element,
    initialcenter: tuple[float, float, float] = (0.0, 0.0, 0.017),
    config: GripperConfig | None = None,
    *,
    collision_mask: int = 1,
    nut_geom: str | None = None,
) -> GripNames:
    """Add hand/jaws to an MJCF ElementTree before compiling the model.

    Pad geoms have contype 4 and the supplied conaffinity, default 1. Supplying
    ``nut_geom`` adds explicit nut/pad collision pairs and bypasses mask ambiguity.
    Palm and jaw backings are visual geometry; only pads collide. The hand origin
    is the grip center, with the palm 13 mm above that point. Initial slider q=0
    leaves a 17 mm gap. The actuator later closes the dynamic jaws.
    """
    cfg = config or GripperConfig()
    names = GripNames()
    world = _section(root, "worldbody")
    hand = ET.SubElement(world, "body", name=names.body,
                         pos=" ".join(f"{v:.9g}" for v in initialcenter))
    ET.SubElement(hand, "freejoint", name=names.joint)
    ET.SubElement(hand, "inertial", pos="0 0 0.010", mass="0.06",
                  diaginertia="0.000006 0.000009 0.000008")
    # Open-center palm gives the protruding bolt room to pass through, rather
    # than hiding a solid visual bar through the bolt. All solid backing/frame
    # pieces collide too; the centered baseline leaves them clear of the nut.
    for i, (pos, size) in enumerate((("0 .008 .013", ".013 .002 .003"),
                                     ("0 -.008 .013", ".013 .002 .003"),
                                     (".011 0 .013", ".002 .006 .003"),
                                     ("-.011 0 .013", ".002 .006 .003"))):
        ET.SubElement(hand, "geom", name=f"contact_hand_palm_{i}", type="box",
                      pos=pos, size=size, rgba="0.22 0.25 0.29 1",
                      contype="4", conaffinity="3", mass="0", condim="3",
                      friction="0.3 0 0", solref="0.0008 1", solimp="0.95 0.99 0.0001")
    actuators = _section(root, "actuator")
    for side, sign in enumerate((-1, 1)):
        jaw = ET.SubElement(hand, "body", name=names.jaw_bodies[side],
                            pos=f"{sign * 0.01:.6f} 0 0")
        ET.SubElement(jaw, "inertial", pos="0 0 0.003", mass="0.01",
                      diaginertia="0.00000012 0.00000010 0.00000010")
        ET.SubElement(jaw, "joint", name=names.jaw_joints[side], type="slide",
                      axis=f"{-sign} 0 0", range="0 0.0035", limited="true",
                      armature="0.0001", damping="0.02")
        ET.SubElement(jaw, "geom", name=names.pad_geoms[side], type="box",
                      size="0.0015 0.0045 0.00275", mass="0",
                      rgba="0.08 0.09 0.10 1", contype="4", conaffinity=str(collision_mask),
                      condim="3", friction=f"{cfg.pad_friction} 0 0", margin="0",
                      solref="0.0008 1", solimp="0.95 0.99 0.0001")
        ET.SubElement(jaw, "geom", name=f"contact_backing_{side}", type="box", pos="0 0 0.0079",
                      size="0.0015 0.0045 0.0045", rgba="0.38 0.40 0.44 1",
                      contype="4", conaffinity="3", mass="0", condim="3",
                      friction="0.3 0 0", solref="0.0008 1", solimp="0.95 0.99 0.0001")
        ET.SubElement(actuators, "position", name=names.jaw_actuators[side],
                      joint=names.jaw_joints[side], kp=str(cfg.jaw_stiffness),
                      kv=str(cfg.jaw_damping), ctrllimited="true", ctrlrange="0 0.0035",
                      forcelimited="true", forcerange=f"{-cfg.maximum_jaw_force} {cfg.maximum_jaw_force}")
        if nut_geom:
            ET.SubElement(_section(root, "contact"), "pair", geom1=nut_geom,
                          geom2=names.pad_geoms[side], condim="3",
                          friction=f"{cfg.pad_friction} {cfg.pad_friction} 0 0 0",
                          solref="0.0008 1", solimp="0.95 0.99 0.0001")
    return names


class ParallelJawController:
    """World-wrench actions and optional hand-only Cartesian impedance.

    The public action is [Fx, Fy, Fz, Tx, Ty, Tz] plus the requested aperture in
    meters. Wrenches act at the palm body's center of mass and are capped by
    Euclidean force/torque magnitude. Cartesian
    pose commands become finite forces, never qpos writes during a rollout.
    """

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData,
                 config: GripperConfig | None = None, names: GripNames | None = None):
        self.model, self.data = model, data
        self.config, self.names = config or GripperConfig(), names or GripNames()
        self.body_id = model.body(self.names.body).id
        self.joint_id = model.joint(self.names.joint).id
        self.qpos_addr = int(model.jnt_qposadr[self.joint_id])
        self.dof_addr = int(model.jnt_dofadr[self.joint_id])
        self.jaw_joint_ids = tuple(model.joint(n).id for n in self.names.jaw_joints)
        self.actuator_ids = tuple(model.actuator(n).id for n in self.names.jaw_actuators)
        self.pad_geom_ids = frozenset(model.geom(n).id for n in self.names.pad_geoms)
        hand_bodies = {self.body_id, *(model.body(n).id for n in self.names.jaw_bodies)}
        self.hand_geom_ids = frozenset(int(g) for g, b in enumerate(model.geom_bodyid) if int(b) in hand_bodies)
        self.total_mass = float(model.body_mass[self.body_id] + sum(
            model.body_mass[model.body(n).id] for n in self.names.jaw_bodies))
        self.last_wrench = np.zeros(6)

    def reset(self, position=None, quaternion=None, aperture=None):
        """Initialize hand state only, before simulation; leave the nut untouched."""
        a = self.qpos_addr
        if position is not None:
            position = np.asarray(position, dtype=float)
            if position.shape != (3,) or not np.all(np.isfinite(position)):
                raise ValueError("Hand position must have three finite values")
            self.data.qpos[a:a + 3] = position
        if quaternion is not None:
            quaternion = np.asarray(quaternion, dtype=float)
            if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)) or np.linalg.norm(quaternion) == 0:
                raise ValueError("Hand quaternion must have four finite values and nonzero norm")
            self.data.qpos[a + 3:a + 7] = quaternion / np.linalg.norm(quaternion)
        self.data.qvel[self.dof_addr:self.dof_addr + 6] = 0
        aperture = self.config.open_aperture if aperture is None else aperture
        closure = self._closure(aperture)
        for jid, aid in zip(self.jaw_joint_ids, self.actuator_ids):
            self.data.qpos[self.model.jnt_qposadr[jid]] = closure
            self.data.qvel[self.model.jnt_dofadr[jid]] = 0
            self.data.ctrl[aid] = closure
        self.data.xfrc_applied[self.body_id] = 0
        mujoco.mj_forward(self.model, self.data)

    @staticmethod
    def _closure(aperture):
        # At q=0 the two inner pad faces are x=+/-8.5 mm.
        return float(np.clip((0.017 - aperture) / 2, 0, 0.0035))

    def apply_action(self, wrench_world, aperture):
        wrench = np.asarray(wrench_world, dtype=float).copy()
        if wrench.shape != (6,) or not np.all(np.isfinite(wrench)):
            raise ValueError("Hand wrench must contain six finite values")
        if not np.isfinite(aperture):
            raise ValueError("Aperture must be finite")
        for sl, cap in ((slice(0, 3), self.config.maximum_force),
                        (slice(3, 6), self.config.maximum_torque)):
            mag = np.linalg.norm(wrench[sl])
            if mag > cap:
                wrench[sl] *= cap / mag
        self.data.xfrc_applied[self.body_id] = wrench
        self.last_wrench[:] = wrench
        self.data.ctrl[list(self.actuator_ids)] = self._closure(aperture)
        return wrench.copy()

    def servo_pose(self, position, quaternion, aperture, *, linear_velocity=None,
                   angular_velocity=None, feedforward_wrench=None):
        """Impedance servo on the hand, including its gravity compensation.

        Nut state is deliberately not read here. In particular, the hand's z
        target must be independent of measured nut insertion for an honest test.
        The rigid palm's center of mass is displaced 10 mm in hand-local z.
        """
        cfg = self.config
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_XBODY,
                                self.body_id, vel, 0)
        target_q = np.asarray(quaternion, dtype=float)
        target_q = target_q / np.linalg.norm(target_q)
        rot_error = np.zeros(3)
        mujoco.mju_subQuat(rot_error, target_q, self.data.xquat[self.body_id])
        # mju_subQuat is in the current body's local axes; the wrench and the
        # angular velocity below are world-oriented.
        rot_error = self.data.xmat[self.body_id].reshape(3, 3) @ rot_error
        target_v = np.zeros(3) if linear_velocity is None else np.asarray(linear_velocity)
        target_w = np.zeros(3) if angular_velocity is None else np.asarray(angular_velocity)
        tracking_force = cfg.position_stiffness * (np.asarray(position) - self.data.xpos[self.body_id])
        tracking_force += cfg.position_damping * (target_v - vel[3:])
        force = tracking_force - self.total_mass * self.model.opt.gravity
        torque = cfg.rotation_stiffness * rot_error + cfg.rotation_damping * (target_w - vel[:3])
        # Apply the translational spring at the grip origin rather than at the
        # elevated palm CoM; this is a wrench conversion, not a nut constraint.
        torque -= np.cross(self.data.xipos[self.body_id] - self.data.xpos[self.body_id], tracking_force)
        # xfrc_applied acts at the palm body's CoM. Cancel the gravitational
        # moment of the child jaws about that point, including current closure.
        for name in self.names.jaw_bodies:
            jid = self.model.body(name).id
            arm = self.data.xipos[jid] - self.data.xipos[self.body_id]
            torque -= np.cross(arm, self.model.body_mass[jid] * self.model.opt.gravity)
        wrench = np.r_[force, torque]
        if feedforward_wrench is not None:
            wrench += np.asarray(feedforward_wrench)
        return self.apply_action(wrench, aperture)

    def contact_wrench_on(self, body_name="nut"):
        """Sum hand contacts, with pad-only wrench separately, about object origin."""
        body_id = self.model.body(body_name).id
        total = np.zeros(6)
        pad_total = np.zeros(6)
        count = 0
        pad_count = 0
        normal_forces = {int(g): 0. for g in self.pad_geom_ids}
        contact_force = np.zeros(6)
        for i in range(self.data.ncon):
            contact = self.data.contact[i]
            g1, g2 = int(contact.geom1), int(contact.geom2)
            b1, b2 = self.model.geom_bodyid[g1], self.model.geom_bodyid[g2]
            if (b1 == body_id and g2 in self.hand_geom_ids):
                sign = -1
            elif (b2 == body_id and g1 in self.hand_geom_ids):
                sign = 1
            else:
                continue
            mujoco.mj_contactForce(self.model, self.data, i, contact_force)
            hand_geom = g1 if g1 in self.hand_geom_ids else g2
            frame = contact.frame.reshape(3, 3)
            force = sign * (frame.T @ contact_force[:3])
            torque = sign * (frame.T @ contact_force[3:])
            moment = torque + np.cross(contact.pos - self.data.xpos[body_id], force)
            total[:3] += force
            total[3:] += moment
            if hand_geom in self.pad_geom_ids:
                normal_forces[hand_geom] += float(contact_force[0])
                pad_total[:3] += force
                pad_total[3:] += moment
                pad_count += 1
            count += 1
        return {"contact_count": count, "wrench_world": total.tolist(),
                "pad_contact_count": pad_count, "pad_wrench_world": pad_total.tolist(),
                "pad_normal_force_N": [normal_forces[self.model.geom(n).id] for n in self.names.pad_geoms],
                "jaw_actuator_force": self.data.actuator_force[list(self.actuator_ids)].tolist(),
                "hand_wrench_world": self.last_wrench.tolist()}


def fixture_xml(config: GripperConfig | None = None):
    """Thread-free hex nut used to isolate the grasp physics in the benchmark."""
    root = ET.Element("mujoco", model="m8_physical_grip_fixture")
    ET.SubElement(root, "compiler", angle="radian")
    ET.SubElement(root, "option", timestep="0.0001", gravity="0 0 0",
                  solver="Newton", iterations="60", integrator="implicitfast",
                  cone="elliptic")
    vertices = []
    r = .0065 / np.cos(np.pi / 6)
    for z in (-.00325, .00325):
        for k in range(6):
            t = np.pi / 6 + k * np.pi / 3
            vertices.extend([r * np.cos(t), r * np.sin(t), z])
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "mesh", name="nut_hex", vertex=" ".join(map(str, vertices)))
    body = ET.SubElement(_section(root, "worldbody"), "body", name="nut", pos="0 0 .017")
    ET.SubElement(body, "freejoint", name="nut_free")
    ET.SubElement(body, "geom", name="nut_hex_geom", type="mesh", mesh="nut_hex",
                  mass=".0055", contype="1", conaffinity="4", friction=".8 0 0", condim="3")
    add_xml_gripper(root, config=config, nut_geom="nut_hex_geom")
    return ET.tostring(root, encoding="unicode")


def grip_benchmark(duration=1.4):
    """Closed jaws must transfer torque; open jaws must leave the nut still.

    This benchmark intentionally removes threads, gravity and support surfaces.
    It isolates contact coupling and cannot validate the bolt/nut threading model.
    """
    cases = {}
    for label, closed, resistance in (("closed", True, 0.), ("open", False, 0.),
                                      ("loaded_closed", True, .0035)):
        model = mujoco.MjModel.from_xml_string(fixture_xml())
        data = mujoco.MjData(model)
        ctrl = ParallelJawController(model, data)
        ctrl.reset()
        nut_joint = model.joint("nut_free").id
        nut_q = int(model.jnt_qposadr[nut_joint])
        nut_body = model.body("nut").id
        angle_start = 0.
        peak_contact_torque = 0.
        max_contacts = 0
        measured_torques = []
        normal_samples = []
        for step in range(round(duration / model.opt.timestep)):
            t = step * model.opt.timestep
            # Slow closure first; rotate hand in the plane only after settling.
            closure_fraction = np.clip((t - .05) / .2, 0, 1) if closed else 0.
            aperture = (1 - closure_fraction) * .017 + closure_fraction * ctrl.config.closed_aperture
            theta = max(0., t - .45) * .8
            q = [np.cos(theta / 2), 0, 0, np.sin(theta / 2)]
            ctrl.servo_pose([0, 0, .017], q, aperture,
                            angular_velocity=[0, 0, .8 if t > .45 else 0.])
            # A known, independent resisting torque tests force transmission.
            # This disturbance is confined to the benchmark, not the controller.
            data.xfrc_applied[nut_body, 5] = -resistance if t > .45 else 0.
            mujoco.mj_step(model, data)
            report = ctrl.contact_wrench_on()
            peak_contact_torque = max(peak_contact_torque, abs(report["wrench_world"][5]))
            max_contacts = max(max_contacts, report["contact_count"])
            if t > .8:
                measured_torques.append(report["wrench_world"][5])
                normal_samples.append(report["pad_normal_force_N"])
        nq = data.qpos[nut_q + 3:nut_q + 7]
        angle = 2 * np.arctan2(nq[3], nq[0]) - angle_start
        hand_angle = 2 * np.arctan2(data.xquat[ctrl.body_id, 3], data.xquat[ctrl.body_id, 0])
        cases[label] = {
            "nut_rotation_rad": float(angle), "hand_rotation_rad": float(hand_angle),
            "peak_pad_contact_torque_Nm": float(peak_contact_torque),
            "maximum_pad_contacts": max_contacts,
            "resisting_torque_Nm": resistance,
            "mean_pad_contact_torque_Nm": float(np.mean(measured_torques)),
            "mean_pad_normal_forces_N": np.mean(normal_samples, axis=0).tolist(),
            "nut_displacement_m": float(np.linalg.norm(data.qpos[nut_q:nut_q + 3] - [0, 0, .017])),
            "warnings": int(sum(w.number for w in data.warning)),
        }
    passed = (cases["closed"]["nut_rotation_rad"] > .5
              and cases["closed"]["peak_pad_contact_torque_Nm"] > 1e-5
              and abs(cases["open"]["nut_rotation_rad"]) < 1e-6
              and cases["open"]["maximum_pad_contacts"] == 0
              and cases["loaded_closed"]["nut_rotation_rad"] > .5
              and abs(cases["loaded_closed"]["mean_pad_contact_torque_Nm"] - .0035) < .0005
              and all(c["warnings"] == 0 for c in cases.values()))
    result = {"passed": bool(passed), "description": "Isolated frictional hex grip; no thread geometry", "cases": cases}
    return result


def threaded_grip_benchmark(output="outputs/m8", *, regrasp=True, release_reset=True,
                            angular_speed=4., config=None, resume=False):
    """Pre-engaged contact demo: turn, release, reset open hand, and turn again.

    The hand's axial impedance is disabled during turns; constant 0.05 N axial
    load replaces any pitch-synchronized motion command. While open, a z target
    is captured from the hand's own pose and held independently of the nut. Only
    initial reset writes state. Acceptance uses observed pitch, penetration and
    open-hand decoupling; these diagnostics do not influence the controls.
    """
    import json
    import hashlib
    import inspect
    from pathlib import Path
    import time
    from dataclasses import asdict, replace
    from .model import make_model, model_xml, ThreadConfig
    from .runtime import require_micron_engine

    runtime = require_micron_engine()
    config = replace(config or ThreadConfig(), with_gripper=True)
    if not config.with_bolt:
        raise ValueError("The thread-contact demonstration requires a bolt")
    model = make_model(config)
    data = mujoco.MjData(model)
    hand = ParallelJawController(model, data)
    controller_identity = hashlib.sha256("\n".join(inspect.getsource(value) for value in
        (GripperConfig, add_xml_gripper, ParallelJawController)).encode()).hexdigest()
    model_identity = hashlib.sha256(model_xml(config).encode()).hexdigest()
    hand.reset(aperture=.017)
    nut_jid = model.joint("nut_free").id
    nut_addr = int(model.jnt_qposadr[nut_jid])
    nut_body = model.body("nut").id
    bolt_geom, nut_geom = model.geom("bolt_thread").id, model.geom("nut_thread").id
    output = Path(output)
    output.mkdir(exist_ok=True, parents=True)
    if angular_speed <= 0:
        raise ValueError("Angular speed must be positive")
    full_turn = 2 * np.pi / angular_speed
    phases = [("close", .25, 0., 0., .017, .0124, True, 0.),
              ("settle", .10, 0., 0., .0124, .0124, True, -.05),
              ("turn_1", full_turn, 0., -2 * np.pi, .0124, .0124, True, -.05),
              ("stop_1", .12, -2 * np.pi, -2 * np.pi, .0124, .0124, True, -.05)]
    if release_reset or regrasp:
        phases += [("release", .12, -2 * np.pi, -2 * np.pi, .0124, .017, False, 0.),
                   ("open_settle", .06, -2 * np.pi, -2 * np.pi, .017, .017, False, 0.),
                   ("reset_open", full_turn / 2, -2 * np.pi, 0., .017, .017, False, 0.),
                   ("open_hold", .06, 0., 0., .017, .017, False, 0.)]
    if regrasp:
        phases += [("regrip", .20, 0., 0., .017, .0124, False, 0.),
                   ("turn_2", full_turn, 0., -2 * np.pi, .0124, .0124, True, -.05),
                   ("stop_2", .15, -2 * np.pi, -2 * np.pi, .0124, .0124, True, -.05)]
    metadata = {"config": config.as_dict(), "runtime": runtime,
                "gripper_config": asdict(hand.config), "starts_preengaged": True,
                "controller": "Finite hand impedance; no axial impedance during turns; constant 0.05N axial hand load",
                "partial": True}
    times, poses, velocities, controls, rows = [], [], [], [], []
    summaries = []
    vel = np.zeros(6)
    unwrapped_yaw, last_yaw, peak_penetration = 0., 0., 0.
    aborted = None
    wall_start = time.perf_counter()
    hand_geom_mask = np.isin(np.arange(model.ngeom), list(hand.hand_geom_ids))
    nut_geom_mask = model.geom_bodyid == nut_body
    maximum_radial_offset, maximum_tilt = 0., 0.
    nut_unforced = True
    state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
    checkpoint_path = output / "gripper_checkpoint.npz"
    completed_phases = 0
    prior_wall_seconds = 0.
    run_settings = {"regrasp": regrasp, "release_reset": release_reset,
                    "angular_speed": angular_speed}
    if resume:
        checkpoint = np.load(checkpoint_path)
        saved = json.loads(str(checkpoint["checkpoint_json"]))
        # Absolute library paths can change when an identical verified engine
        # is installed into the Python wheel; hashes and search constants are
        # the engine identity, and remain mandatory.
        def runtime_identity(value):
            return {**value, "libraries": [{k: v for k, v in entry.items() if k != "path"}
                                           for entry in value["libraries"]]}
        if (saved["config"] != config.as_dict() or runtime_identity(saved["runtime"]) != runtime_identity(runtime)
                or saved["run_settings"] != run_settings):
            raise ValueError("Checkpoint configuration, engine/plugin hashes, or demo settings changed")
        if saved.get("aborted"):
            raise ValueError("A failed-physics checkpoint cannot be resumed")
        if (saved.get("gripper_config") != asdict(hand.config)
                or saved.get("controller_sha256") != controller_identity
                or saved.get("model_xml_sha256") != model_identity):
            raise ValueError("Checkpoint jaw configuration or controller/model identity changed, "
                             "or the checkpoint predates these provenance fields")
        mujoco.mj_setState(model, data, checkpoint["integration_state"], state_spec)
        mujoco.mj_forward(model, data)
        times, poses = list(checkpoint["time"]), list(checkpoint["qpos"])
        velocities, controls = list(checkpoint["qvel"]), list(checkpoint["controller"])
        rows = json.loads(str(checkpoint["info_json"]))
        summaries = saved["phases"]
        completed_phases = len(summaries)
        unwrapped_yaw, last_yaw = saved["unwrapped_yaw"], saved["last_yaw"]
        peak_penetration = saved["peak_penetration"]
        maximum_radial_offset, maximum_tilt = saved["maximum_radial_offset"], saved["maximum_tilt"]
        nut_unforced, prior_wall_seconds = saved["nut_unforced"], saved["wall_seconds"]
    for label, duration, angle0, angle1, aperture0, aperture1, axial_float, axial_load in phases[completed_phases:]:
        phase_start_z = float(data.qpos[nut_addr + 2])
        phase_start_yaw = unwrapped_yaw
        held_z = float(data.xpos[hand.body_id, 2])
        phase_contacts = 0
        peak_pad_torque = 0.
        steps = round(duration / model.opt.timestep)
        for i in range(steps):
            u = min(1., (i + 1) / steps)
            theta = (1 - u) * angle0 + u * angle1
            aperture = (1 - u) * aperture0 + u * aperture1
            q = [np.cos(theta / 2), 0., 0., np.sin(theta / 2)]
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY, hand.body_id, vel, 0)
            target_z = float(data.xpos[hand.body_id, 2]) if axial_float else held_z
            target_vz = float(vel[5]) if axial_float else 0.
            hand.servo_pose([0., 0., target_z], q, aperture,
                            linear_velocity=[0., 0., target_vz],
                            angular_velocity=[0., 0., (angle1 - angle0) / duration],
                            feedforward_wrench=[0., 0., axial_load, 0., 0., 0.])
            mujoco.mj_step(model, data)
            w, x, y, z = data.qpos[nut_addr + 3:nut_addr + 7]
            yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            unwrapped_yaw += float((yaw - last_yaw + np.pi) % (2 * np.pi) - np.pi)
            last_yaw = yaw
            geom, dist = data.contact.geom, data.contact.dist
            hand_nut_mask = ((nut_geom_mask[geom[:, 0]] & hand_geom_mask[geom[:, 1]])
                             | (nut_geom_mask[geom[:, 1]] & hand_geom_mask[geom[:, 0]]))
            # Count every physics substep, including short contacts between
            # trajectory samples; the open-reset gate uses this exact count.
            phase_contacts = max(phase_contacts, int(np.sum(hand_nut_mask)))
            mask = (((geom[:, 0] == bolt_geom) & (geom[:, 1] == nut_geom))
                    | ((geom[:, 1] == bolt_geom) & (geom[:, 0] == nut_geom)))
            penetration = float(max(0., -np.min(dist[mask]))) if np.any(mask) else 0.
            peak_penetration = max(peak_penetration, penetration)
            radial_offset = float(np.linalg.norm(data.qpos[nut_addr:nut_addr + 2]))
            nut_axis = data.xmat[nut_body].reshape(3, 3)[:, 2]
            nut_tilt = float(np.arccos(np.clip(nut_axis[2], -1, 1)))
            maximum_radial_offset = max(maximum_radial_offset, radial_offset)
            maximum_tilt = max(maximum_tilt, nut_tilt)
            nut_unforced = nut_unforced and bool(np.all(data.xfrc_applied[nut_body] == 0))
            warnings = int(sum(w.number for w in data.warning))
            if (penetration > 10e-6 or warnings or not np.all(np.isfinite(data.qpos))
                    or not np.all(np.isfinite(data.qvel)) or radial_offset > 150e-6
                    or nut_tilt > np.deg2rad(2) or not nut_unforced):
                aborted = {"phase": label, "time": float(data.time),
                           "reported_sdf_depth_m": penetration, "solver_warnings": warnings,
                           "radial_offset_m": radial_offset, "nut_tilt_rad": nut_tilt,
                           "nut_external_wrench_zero": nut_unforced}
            if i % 100 == 0 or i == steps - 1 or aborted:
                report = hand.contact_wrench_on()
                peak_pad_torque = max(peak_pad_torque, abs(report["pad_wrench_world"][5]))
                row = {"phase": label, "time": float(data.time),
                       "nut_z": float(data.qpos[nut_addr + 2]), "nut_yaw_unwrapped": unwrapped_yaw,
                       "hand_z": float(data.xpos[hand.body_id, 2]),
                       "radial_offset_m": radial_offset, "nut_tilt_rad": nut_tilt,
                       "reported_sdf_depth_m": penetration, "contact": report}
                rows.append(row)
                times.append(data.time)
                poses.append(data.qpos.copy())
                velocities.append(data.qvel.copy())
                controls.append([theta, aperture, *hand.last_wrench])
            if aborted:
                break
        advance = phase_start_z - float(data.qpos[nut_addr + 2])
        rotation = unwrapped_yaw - phase_start_yaw
        summary = {"phase": label, "axial_advance_m": advance,
                   "nut_rotation_rad": rotation, "observed_helix_residual_m": advance + config.pitch * rotation / (2 * np.pi),
                   "maximum_hand_nut_contacts": phase_contacts,
                   "sampled_peak_pad_contact_torque_Nm": peak_pad_torque}
        summaries.append(summary)
        np.savez_compressed(output / "gripper_trace_partial.npz", time=times, qpos=poses, qvel=velocities,
                            controller=controls, info_json=np.asarray(json.dumps(rows)),
                            metadata_json=np.asarray(json.dumps(metadata)))
        state = np.empty(mujoco.mj_stateSize(model, state_spec))
        mujoco.mj_getState(model, data, state, state_spec)
        checkpoint_metadata = {"config": config.as_dict(), "runtime": runtime,
            "run_settings": run_settings, "phases": summaries, "aborted": aborted,
            "gripper_config": asdict(hand.config), "controller_sha256": controller_identity,
            "model_xml_sha256": model_identity,
            "unwrapped_yaw": unwrapped_yaw, "last_yaw": last_yaw,
            "peak_penetration": peak_penetration, "maximum_radial_offset": maximum_radial_offset,
            "maximum_tilt": maximum_tilt, "nut_unforced": nut_unforced,
            "wall_seconds": prior_wall_seconds + time.perf_counter() - wall_start}
        temporary = output / "gripper_checkpoint.tmp.npz"
        np.savez_compressed(temporary, integration_state=state, time=times, qpos=poses,
                            qvel=velocities, controller=controls,
                            info_json=np.asarray(json.dumps(rows)),
                            checkpoint_json=np.asarray(json.dumps(checkpoint_metadata)))
        temporary.replace(checkpoint_path)
        print(json.dumps(summary), flush=True)
        if aborted:
            break
    turns = [s for s in summaries if s["phase"].startswith("turn_")]
    expected_turns = 2 if regrasp else 1
    checks = {
        "all_requested_turns_completed": {"passed": len(turns) == expected_turns,
                                           "expected": expected_turns, "observed": len(turns)},
        "observed_metric_lead": {"passed": bool(turns and all(abs(s["observed_helix_residual_m"]) <= .02 * config.pitch for s in turns)),
                                 "residual_limit_m": .02 * config.pitch,
                                 "fraction_of_pitch_limit": .02},
        "closed_turn_tracking": {"passed": bool(turns and all(abs(s["nut_rotation_rad"] + 2 * np.pi) <= .02 for s in turns)),
                                 "angular_error_limit_rad": .02},
        "reported_sdf_depth_proxy": {"passed": peak_penetration <= 10e-6,
                                     "reported_depth_limit_m": 10e-6,
                                     "note": "Not a geometric overlap certificate"},
        "no_solver_or_state_abort": {"passed": not aborted},
        "free_nut_alignment": {"passed": bool(maximum_radial_offset <= 150e-6 and maximum_tilt <= np.deg2rad(2)),
                               "maximum_radial_offset_m": maximum_radial_offset,
                               "maximum_tilt_rad": maximum_tilt},
        "nut_has_no_external_drive": {"passed": nut_unforced},
        "closed_turn_contact_torque": {"passed": bool(turns and all(s["sampled_peak_pad_contact_torque_Nm"] > 1e-6 for s in turns)),
                                      "sampled_peak_torque_floor_Nm": 1e-6},
    }
    if release_reset or regrasp:
        resets = [s for s in summaries if s["phase"] == "reset_open"]
        checks["open_reset_contact_decoupling"] = {"passed": bool(resets and resets[0]["maximum_hand_nut_contacts"] == 0),
                                                    "expected_hand_nut_contacts": 0}
        checks["passive_self_locking_during_reset"] = {"passed": bool(resets and abs(resets[0]["nut_rotation_rad"]) < .02),
                                                      "maximum_nut_creep_rad": .02,
                                                      "observed_nut_creep_rad": resets[0]["nut_rotation_rad"] if resets else None}
    passed = all(c["passed"] for c in checks.values())
    result = {"passed": bool(passed), "starts_preengaged": True,
              "description": "Contact-driven proxy hand, floating axial pose during turns; no learned policy",
              "runtime": runtime, "config": config.as_dict(), "phases": summaries,
              "gripper_config": asdict(hand.config), "acceptance_checks": checks,
              "episode_peak_reported_sdf_depth_m": peak_penetration,
              "reported_sdf_depth_note": "Native contact dist proxy; not a certified geometric overlap bound",
              "aborted": aborted, "wall_seconds": prior_wall_seconds + time.perf_counter() - wall_start,
              "resumed_from_checkpoint": bool(resume),
              "report_only_checkpoint_recovery": bool(resume and completed_phases == len(phases)),
              "controller_sha256": controller_identity, "model_xml_sha256": model_identity,
              "checkpoint_note": "Restores integrated state; derived-pose refresh can change the first resumed control, so identical continuation is not claimed",
              "trajectory": str(output / "gripper_trace.npz")}
    np.savez_compressed(output / "gripper_trace.npz", time=times, qpos=poses, qvel=velocities,
                        controller=controls, info_json=np.asarray(json.dumps(rows)),
                        metadata_json=np.asarray(json.dumps(result)))
    (output / "gripper_validation.json").write_text(json.dumps(result, indent=2))
    return result


if __name__ == "__main__":
    import argparse
    import json
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--threaded", action="store_true")
    parser.add_argument("--regrasp", action="store_true")
    parser.add_argument("--output", default="outputs/m8")
    parser.add_argument("--angular-speed", type=float, default=4.)
    args = parser.parse_args()
    result = (threaded_grip_benchmark(args.output, regrasp=args.regrasp,
                                    angular_speed=args.angular_speed) if args.threaded else grip_benchmark())
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 1)
