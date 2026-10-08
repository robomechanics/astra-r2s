"""Two real YAM arms, a free clamping block, and passive M8 thread contacts.

The short bolt is rigidly mounted on a dynamic aluminum block. The left arm
holds that block through finger friction; the right arm turns a larger steel
nut. Both block and nut remain free rigid bodies, without grasp constraints.
The M8 bore, pitch, flank clearance, and material friction are unchanged.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass, field
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.stats import qmc

from thread_lab.build_plugin import build_plugin
from thread_lab.model import ThreadConfig, model_xml
from thread_lab.runtime import require_micron_engine

ROOT = Path(__file__).resolve().parents[1]
YAM_XML = ROOT / "assets/yam/mjlab_reference/xmls/yam.xml"
PAD_HALF_SIZE = (0.0045, 0.0015, 0.00275)
PAD_NAMES = ("right_m8_pad_left", "right_m8_pad_right")
_loaded = False


def _radial_mass_properties(radius, theta, z, domain_area, *, density=7850., hex_af=None, height=None):
    """Independent exact radial integration, with Halton angle/height quadrature."""
    c, s = np.cos(theta), np.sin(theta)
    volume = domain_area * np.mean(radius ** 2 / 2)
    first = domain_area * np.array([np.mean(c * radius ** 3 / 3),
                                    np.mean(s * radius ** 3 / 3),
                                    np.mean(z * radius ** 2 / 2)])
    second = domain_area * np.array([
        [np.mean(c*c * radius**4/4), np.mean(c*s * radius**4/4), np.mean(z*c * radius**3/3)],
        [np.mean(c*s * radius**4/4), np.mean(s*s * radius**4/4), np.mean(z*s * radius**3/3)],
        [np.mean(z*c * radius**3/3), np.mean(z*s * radius**3/3), np.mean(z*z * radius**2/2)],
    ])
    if hex_af is not None:
        area = np.sqrt(3) * hex_af ** 2 / 2
        polar = 5 * np.sqrt(3) * hex_af ** 4 / 72
        volume = area * height - volume
        first = -first
        second = np.diag([polar * height / 2, polar * height / 2,
                          area * height ** 3 / 12]) - second
    centroid = first / volume
    central = second - volume * np.outer(centroid, centroid)
    inertia = density * (np.trace(central) * np.eye(3) - central)
    return {"mass_kg": float(density * volume), "centroid_m": centroid.tolist(),
            "inertia_kg_m2": inertia.tolist(), "density_kg_m3": density,
            "samples": len(radius)}


@lru_cache(maxsize=16)
def nut_mass_properties(config: ThreadConfig, samples=1_048_576):
    """Integrate the declared steel nut independently of MuJoCo or its plugin."""
    uv = qmc.Halton(2, scramble=False).random(samples)
    theta = 2 * np.pi * uv[:, 0]
    z = config.nut_height * (uv[:, 1] - .5)
    H = np.sqrt(3) * config.pitch / 2
    r2 = config.female_pitch_diameter / 2
    phase = (z - config.pitch * theta / (2*np.pi) + config.pitch/2) % config.pitch - config.pitch/2
    radius = np.clip(r2 + np.sqrt(3)*(config.pitch/4 - np.abs(phase)), r2-H/4, r2+3*H/8)
    radius = np.maximum(radius, r2 + 3*H/8 + .000360 - (config.nut_height/2 - np.abs(z)))
    return _radial_mass_properties(radius, theta, z, 2*np.pi*config.nut_height,
                                  hex_af=config.nut_across_flats, height=config.nut_height)


@lru_cache(maxsize=16)
def bolt_mass_properties(config: ThreadConfig, samples=1_048_576):
    """Steel short-bolt inertia from its rounded external profile and tip bevel."""
    uv = qmc.Halton(2, scramble=False).random(samples)
    theta, z = 2*np.pi*uv[:, 0], config.bolt_length*uv[:, 1]
    p, r2 = config.pitch, config.male_pitch_diameter / 2
    H = np.sqrt(3)*p/2
    phase = np.abs((z - p*theta/(2*np.pi) + p/2) % p - p/2)
    major, root_radius = r2 + 3*H/8, H/6
    radius = r2 + np.sqrt(3)*(p/4-phase)
    radius = np.where(phase <= p/16, major, radius)
    v = p/2-phase
    root = r2-H/2 + 2*root_radius - np.sqrt(np.maximum(root_radius**2-v**2, 0))
    radius = np.where(phase >= 3*p/8, root, radius)
    radius = np.minimum(radius, major + config.bolt_length - .000956 - z)
    return _radial_mass_properties(radius, theta, z, 2*np.pi*config.bolt_length)


def default_thread_config():
    # Outer dimensions are deliberate easy-grasp geometry; the M8 bore and its
    # clearances remain the independently specified contact-lab dimensions.
    prototype = ThreadConfig(timestep=.000025, bolt_length=.016, nut_height=.008,
                             nut_across_flats=.020, initial_z=.00875)
    properties = nut_mass_properties(prototype)
    inertia = np.asarray(properties["inertia_kg_m2"])
    return ThreadConfig(**{**prototype.as_dict(), "nut_mass": properties["mass_kg"],
        "nut_transverse_inertia": float((inertia[0, 0]+inertia[1, 1])/2),
        "nut_axial_inertia": float(inertia[2, 2])})


@dataclass(frozen=True)
class YamM8Config:
    thread: ThreadConfig = field(default_factory=default_thread_config)
    bolt_position: tuple[float, float, float] = (0.30, -0.06, 0.11125)
    bolt_quaternion: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    pad_friction: float = 0.8
    jaw_stiffness: float = 20_000.0
    jaw_damping: float = 8.0
    maximum_jaw_force: float = 20.0
    open_aperture: float = 0.024
    closed_aperture: float = 0.0194
    left_closed_aperture: float = 0.0144
    left_grasp_roll_rad: float = -np.pi/2
    # Only the left finger/block contact uses this setting. It permits explicit
    # convergence studies of MuJoCo's tangential contact regularization, without
    # changing the M8 contact law, friction coefficient, or right-hand contact.
    left_block_contact_impedance: tuple[float, float, float] = (.95, .99, .0001)
    left_block_contact_time_constant: float = .0008
    pad_inner_offset: float = 0.0005
    block_size: tuple[float, float, float] = (0.020, 0.120, 0.016)
    block_density: float = 2700.
    bolt_offset: tuple[float, float, float] = (0., -0.050, 0.008)
    left_grasp_offset: tuple[float, float, float] = (0., -0.0094, 0.)

    def __post_init__(self):
        if self.thread.with_gripper or self.thread.guided:
            raise ValueError("The YAM task needs a free nut and the actual YAM fingers")
        if not self.thread.with_bolt:
            raise ValueError("The YAM M8 task requires its physical bolt")
        p, q = np.asarray(self.bolt_position), np.asarray(self.bolt_quaternion)
        if p.shape != (3,) or q.shape != (4,) or not np.isfinite(p).all() or not np.isfinite(q).all():
            raise ValueError("Bolt position and quaternion must be finite 3- and 4-vectors")
        if not np.isclose(np.linalg.norm(q), 1, atol=1e-10):
            raise ValueError("Bolt quaternion must be normalized")
        scalars = (self.pad_friction, self.jaw_stiffness, self.jaw_damping,
                   self.maximum_jaw_force, self.open_aperture, self.closed_aperture,
                   self.left_closed_aperture, self.pad_inner_offset, self.block_density)
        if not np.isfinite(self.left_grasp_roll_rad):
            raise ValueError("Left grasp roll must be finite")
        if not np.isfinite(scalars).all() or self.pad_friction < 0 or min(
                self.jaw_stiffness, self.jaw_damping, self.maximum_jaw_force) <= 0:
            raise ValueError("Pad and jaw parameters must be finite with positive force and gains")
        if not .010 <= min(self.closed_aperture, self.left_closed_aperture) <= max(
                self.closed_aperture, self.left_closed_aperture) <= self.open_aperture <= .040:
            raise ValueError("Apertures must satisfy 10mm <= closed <= open <= 40mm")
        if np.asarray(self.block_size).shape != (3,) or not np.isfinite(self.block_size).all() or min(self.block_size) <= 0:
            raise ValueError("Block dimensions must be three positive finite values")
        if self.block_density <= 0 or not 0 <= self.pad_inner_offset <= .001:
            raise ValueError("Block density and pad offset are outside their valid ranges")
        impedance = np.asarray(self.left_block_contact_impedance, dtype=float)
        if impedance.shape != (3,) or not np.isfinite(impedance).all() or not (
                0.0001 <= impedance[0] <= impedance[1] <= .9999 and impedance[2] > 0):
            raise ValueError("Left block impedance needs 0.0001 <= d0 <= dmax <= 0.9999 and positive width")
        if not np.isfinite(self.left_block_contact_time_constant) or self.left_block_contact_time_constant < 2*self.thread.timestep:
            raise ValueError("Resolve the left block contact time constant with at least two steps")


def _numbers(values):
    return " ".join(f"{float(value):.12g}" for value in values)


def _add(parent, tag, **attributes):
    return ET.SubElement(parent, tag, {key: str(value) for key, value in attributes.items()})


def bolt_rotation(config: YamM8Config | None = None) -> np.ndarray:
    cfg = config or YamM8Config()
    flat = np.empty(9)
    mujoco.mju_quat2Mat(flat, np.asarray(cfg.bolt_quaternion, dtype=float))
    return flat.reshape(3, 3)


def initial_nut_position(config: YamM8Config | None = None) -> np.ndarray:
    cfg = config or YamM8Config()
    return np.asarray(cfg.bolt_position) + bolt_rotation(cfg)[:, 2] * cfg.thread.initial_z


def initial_block_position(config: YamM8Config | None = None) -> np.ndarray:
    cfg = config or YamM8Config()
    return np.asarray(cfg.bolt_position) - bolt_rotation(cfg) @ np.asarray(cfg.bolt_offset)


def initial_left_grasp_position(config: YamM8Config | None = None) -> np.ndarray:
    cfg = config or YamM8Config()
    return initial_block_position(cfg) + bolt_rotation(cfg) @ np.asarray(cfg.left_grasp_offset)


def left_grasp_rotation(config: YamM8Config | None = None) -> np.ndarray:
    cfg = config or YamM8Config()
    unrolled = np.array([[0., 1., 0.], [0., 0., -1.], [-1., 0., 0.]])
    # Roll about the tool's own approach Z. The default puts jaw gap Y along
    # world -Z: block weight is supported by normal contact and the approach
    # remains toward -Y. This also clears the wrist camera near assembly COM.
    c, s = np.cos(cfg.left_grasp_roll_rad), np.sin(cfg.left_grasp_roll_rad)
    roll = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    return bolt_rotation(cfg) @ unrolled @ roll


def left_touch_aperture(config: YamM8Config | None = None) -> float:
    cfg = config or YamM8Config()
    # Support function of the rectangular block along the rolled jaw normal.
    # At default -90 degrees it returns block height 16 mm; at zero it returns
    # block width 20 mm. Intermediate rolls fit the physically rotated corners.
    return float(abs(np.cos(cfg.left_grasp_roll_rad))*cfg.block_size[0]
                 + abs(np.sin(cfg.left_grasp_roll_rad))*cfg.block_size[2])


def jaw_positions(aperture: float, config: YamM8Config | None = None) -> tuple[float, float]:
    cfg = config or YamM8Config()
    if not np.isfinite(aperture) or not 0 <= aperture <= .074:
        raise ValueError("Aperture must be finite and fit within the physical finger stroke")
    q = (aperture + 2*cfg.pad_inner_offset) / 2
    return q, -q


def scene_xml(config: YamM8Config | None = None) -> str:
    cfg = config or YamM8Config()
    # Reuse the validated plugin instances, nut mass, free joint, thread contact
    # pair, and numerical options verbatim. No independent geometry approximation.
    root = ET.fromstring(model_xml(cfg.thread))
    root.set("model", "dual_yam_m8_contact")
    original = ET.parse(YAM_XML).getroot()
    defaults = root.find("default")
    yam_defaults = copy.deepcopy(original.find("default/default"))
    defaults.append(yam_defaults)
    for geom in yam_defaults.findall(".//geom"):
        if geom.get("group") in ("3", "4"):
            geom.attrib.update(contype="4", conaffinity="15", friction=".7 0 0",
                               condim="3", margin="0", solref=".004 1",
                               solimp=".95 .99 .001")
    asset = root.find("asset")
    for entry in original.find("asset"):
        entry = copy.deepcopy(entry)
        if entry.tag == "mesh":
            path = YAM_XML.parent / "assets" / entry.get("file")
            entry.set("name", entry.get("name", path.stem))
            entry.set("file", str(path.resolve()))
        asset.append(entry)
    _add(asset, "material", name="m8_pad_rubber", rgba=".045 .052 .060 1")
    _add(asset, "material", name="m8_fixture_metal", rgba=".20 .30 .38 1")
    visual = root.find("visual")
    _add(visual, "headlight", ambient=".25 .25 .25", diffuse=".35 .35 .35", specular=".2 .2 .2")
    root.find("statistic").attrib.update(center=".28 0 .24", extent=".85")

    old_world = root.find("worldbody")
    nut = copy.deepcopy(old_world.find("body[@name='nut']"))
    bolt = copy.deepcopy(old_world.find("geom[@name='bolt_thread']"))
    root.remove(old_world)
    world = _add(root, "worldbody")
    _add(world, "light", name="work_key", pos=".15 -.3 1.4", dir=".1 .2 -1", diffuse=".8 .8 .8")
    _add(world, "light", name="work_fill", pos=".7 .5 .8", dir="-.5 -.4 -1", diffuse=".35 .35 .4", castshadow="false")
    _add(world, "camera", name="overview", pos="-.48 -.70 .95", xyaxes=".77 -.64 0 .38 .46 .80", fovy="47")
    # Thread closeup is defined relative to the bolt frame, so config transforms
    # also move its camera without changing the physical contact geometry.
    _add(world, "geom", name="table", type="box", size=".46 .49 .018", pos=".32 0 -.018",
         rgba=".72 .74 .73 1", contype="16", conaffinity="14", friction=".8 0 0",
         solref=".004 1", solimp=".95 .99 .001")
    _add(world, "geom", name="mounting_rail", type="box", pos="0 0 .014", size=".035 .395 .014",
         rgba=".40 .44 .47 1", contype="16", conaffinity="14", solref=".004 1")
    block = _add(world, "body", name="fixture_block", pos=_numbers(initial_block_position(cfg)),
                 quat=_numbers(cfg.bolt_quaternion))
    _add(block, "freejoint", name="fixture_block_free")
    block_mass = float(np.prod(cfg.block_size) * cfg.block_density)
    size = np.asarray(cfg.block_size)
    block_inertia = block_mass / 12 * np.array([size[1]**2+size[2]**2,
        size[0]**2+size[2]**2, size[0]**2+size[1]**2])
    _add(block, "inertial", pos="0 0 0", mass=f"{block_mass:.16g}", diaginertia=_numbers(block_inertia))
    _add(block, "geom", name="fixture_block_geom", type="box", size=_numbers(size/2),
         mass="0", material="m8_fixture_metal", contype="8", conaffinity="22",
         friction=".5 0 0", condim="3", solref=".0008 1", solimp=".95 .99 .0001")
    bolt_frame = _add(block, "body", name="bolt_frame", pos=_numbers(cfg.bolt_offset))
    bolt_properties = bolt_mass_properties(cfg.thread)
    bolt_inertia = np.asarray(bolt_properties["inertia_kg_m2"])
    _add(bolt_frame, "inertial", pos=_numbers(bolt_properties["centroid_m"]),
         mass=f"{bolt_properties['mass_kg']:.16g}", fullinertia=_numbers((bolt_inertia[0, 0],
             bolt_inertia[1, 1], bolt_inertia[2, 2], bolt_inertia[0, 1], bolt_inertia[0, 2], bolt_inertia[1, 2])))
    # Its independently integrated steel inertia above avoids estimating a
    # threaded bolt's mass from the coarse visual tessellation or convex hull.
    bolt.set("mass", "0")
    bolt_frame.append(bolt)
    _add(bolt_frame, "site", name="bolt_origin", pos="0 0 0", size=".001", rgba="0 0 0 0")
    _add(bolt_frame, "camera", name="threadcloseup", pos=".035 -.052 .046",
         xyaxes=".829 .558 0 -.229 .34 .912", fovy="35")
    nut.set("pos", _numbers(initial_nut_position(cfg)))
    nut.set("quat", _numbers(cfg.bolt_quaternion))
    world.append(nut)

    equality = _add(root, "equality")
    actuators = _add(root, "actuator")
    contact = root.find("contact")
    for side, y in (("left", .305), ("right", -.305)):
        arm = copy.deepcopy(original.find("worldbody/body"))
        for entry in arm.iter():
            if "name" in entry.attrib:
                entry.set("name", f"{side}_{entry.get('name')}")
        arm.set("pos", _numbers((0, y, .028)))
        world.append(arm)
        # Keep the upstream site for audit; use a task site centered on the
        # measured collision backings and inserted pads, 14 mm from native TCP.
        native_site = arm.find(f".//site[@name='{side}_grasp_site']")
        audit_site = copy.deepcopy(native_site)
        audit_site.set("name", f"{side}_native_grasp_site")
        wrist = arm.find(f".//body[@name='{side}_link_6']")
        wrist.append(audit_site)
        native_site.set("pos", "0 -.044 .1405")
        for index in range(1, 7):
            joint = arm.find(f".//joint[@name='{side}_joint{index}']")
            joint.set("damping", ".6" if index <= 3 else ".08")
            joint.set("armature", ".032" if index <= 3 else ".0018")
            limit = 28 if index <= 3 else 10
            _add(actuators, "motor", name=f"{side}_servo{index}", joint=f"{side}_joint{index}",
                 ctrllimited="true", ctrlrange=f"{-limit} {limit}",
                 forcelimited="true", forcerange=f"{-limit} {limit}")
        _add(equality, "joint", name=f"{side}_finger_coupling", joint1=f"{side}_left_finger",
             joint2=f"{side}_right_finger", polycoef="0 -1 0 0 0")
        for finger, sign, body_name in (("left", 1, "lf_down"), ("right", -1, "rf_down")):
            joint_name = f"{side}_{finger}_finger"
            arm.find(f".//joint[@name='{joint_name}']").set("damping", "2")
            _add(actuators, "position", name=f"{side}_grip_{finger}", joint=joint_name,
                 kp=cfg.jaw_stiffness, kv=cfg.jaw_damping, ctrllimited="true",
                 ctrlrange="0 .037524" if sign == 1 else "-.037524 0",
                 forcelimited="true", forcerange=f"{-cfg.maximum_jaw_force} {cfg.maximum_jaw_force}")
            jaw = arm.find(f".//body[@name='{side}_{body_name}']")
            pad_name = f"{side}_m8_pad_{finger}"
            pad_y = cfg.pad_inner_offset - PAD_HALF_SIZE[1]
            _add(jaw, "geom", name=pad_name, type="box", pos=_numbers((0, pad_y, .080)),
                 size=_numbers(PAD_HALF_SIZE), mass="0", material="m8_pad_rubber", group="1",
                 contype="4", conaffinity="15", condim="3", friction=f"{cfg.pad_friction} 0 0",
                 margin="0", solref=".0008 1", solimp=".95 .99 .0001")
            _add(contact, "pair", geom1="nut_thread", geom2=pad_name, condim="3",
                 friction=f"{cfg.pad_friction} {cfg.pad_friction} 0 0 0", margin="0",
                 solref=".0008 1", solimp=".95 .99 .0001")
            _add(contact, "pair", geom1="fixture_block_geom", geom2=pad_name, condim="3",
                 friction=f"{cfg.pad_friction} {cfg.pad_friction} 0 0 0", margin="0",
                 solref=f"{cfg.left_block_contact_time_constant} 1" if side == "left" else ".0008 1",
                 solimp=_numbers(cfg.left_block_contact_impedance) if side == "left" else ".95 .99 .0001")
        # These are native adjacent-chain exclusions, not nut/arm exclusions.
        for first, second in (("link_4", "link_6"), ("link_5", "link_left_finger"),
                              ("link_5", "link_right_finger")):
            _add(contact, "exclude", body1=f"{side}_{first}", body2=f"{side}_{second}")
    return ET.tostring(root, encoding="unicode")


def build_spec(config: YamM8Config | None = None) -> mujoco.MjSpec:
    global _loaded
    require_micron_engine()
    if not _loaded:
        mujoco.mj_loadPluginLibrary(str(build_plugin()))
        _loaded = True
    root = ET.fromstring(scene_xml(config))
    assets = {}
    for mesh in root.findall("asset/mesh"):
        file = mesh.get("file")
        if file is not None:
            path = Path(file)
            assets[path.name] = path.read_bytes()
            mesh.set("file", path.name)
    return mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"), assets=assets)


def build_model(config: YamM8Config | None = None) -> mujoco.MjModel:
    return build_spec(config).compile()


make_model = build_model


def scene_fingerprint(config: YamM8Config | None = None) -> str:
    """Hash every scene setting and mesh byte independently of checkout paths."""
    root = ET.fromstring(scene_xml(config))
    mesh_hashes = {}
    for mesh in root.findall("asset/mesh"):
        filename = mesh.get("file")
        if filename is None:
            continue
        path = Path(filename)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if path.name in mesh_hashes and mesh_hashes[path.name] != digest:
            raise ValueError("Distinct mesh assets share a filename")
        mesh_hashes[path.name] = digest
        mesh.set("file", path.name)
    canonical = {"xml": ET.tostring(root, encoding="unicode"), "mesh_sha256": mesh_hashes}
    return hashlib.sha256(json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode()).hexdigest()
