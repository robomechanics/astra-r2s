"""Unengaged M8 bolt pickup and a genuinely open threaded aluminum block.

This is a separate scene from the qualified nut-on-bolt demonstration. The
existing exact thread SDF is reused without modifying its plugin or force law.
The rectangular block is tiled by an internal-thread hex prism, three boxes,
and four convex trapezoidal prisms. Their volumes do not overlap. Explicit
independently integrated inertials count the true rectangle minus bore once.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from scipy.stats import qmc

from thread_lab.model import ThreadConfig
from yam_twin import m8_scene as baseline

DOWN_QUATERNION = (0., 1., 0., 0.)
TABLE_REST_OVERLAP_M = 1e-8
BLOCK_GEOM_NAMES = ("fixture_block_geom", "block_hole_x_plus",
                    "block_hole_x_minus", "block_hole_top_left", "block_hole_top_right",
                    "block_hole_bottom_left", "block_hole_bottom_right")


def _thread_config():
    return replace(baseline.default_thread_config(), nut_height=.016,
                   nut_across_flats=.016)


def _base_config():
    return baseline.YamM8Config(thread=_thread_config(),
        left_block_contact_impedance=(.9999, .9999, .0001))


@dataclass(frozen=True)
class InsertionConfig:
    base: baseline.YamM8Config = field(default_factory=_base_config)
    block_position: tuple[float, float, float] = (.30, -.010, .10325)
    hole_offset: tuple[float, float, float] = (0., -.050, 0.)
    bolt_head_position: tuple[float, float, float] = (.36, -.15, .12)
    bolt_yaw_rad: float = .23
    head_across_flats: float = .020
    head_height: float = .008
    steel_density: float = 7850.
    rest_radius: float = .007
    rest_pin_radius: float = .001
    rest_base_height: float = .010
    # Legacy defaults remain byte-identical for replay of the published run.
    # In the opt-in acquisition scene the block rests upright on its short end
    # and is physically carried and rolled flat by the native left hand.
    pickup_from_table: bool = False
    held_block_position: tuple[float, float, float] = (.30, -.010, .10325)
    # Optional native elliptic-contact tangential regularization. None keeps
    # the historical MuJoCo fallback to the pair's normal solref verbatim.
    left_block_friction_time_constant: float | None = None
    # Exact-envelope collision diagnostic for acquisition. The convex mesh
    # chooses native GJK/EPA rather than the specialized box/box manifold.
    left_pad_collision_geometry: str = "box"
    # Native signed direct-format (negative stiffness, negative damping).
    # These are acceleration-reference parameters, not material N/m or Ns/m.
    left_block_direct_normal_solref: tuple[float, float] | None = None

    @property
    def thread(self):
        return self.base.thread

    def __post_init__(self):
        for vector in (self.block_position, self.hole_offset, self.bolt_head_position,
                       self.held_block_position):
            if np.asarray(vector).shape != (3,) or not np.isfinite(vector).all():
                raise ValueError("Insertion positions must be finite three-vectors")
        if not isinstance(self.pickup_from_table, bool):
            raise ValueError("Table pickup must be a boolean mode")
        if self.left_pad_collision_geometry not in ("box", "convex_mesh"):
            raise ValueError("Left pad collision geometry must be box or convex_mesh")
        if self.left_pad_collision_geometry != "box" and not self.pickup_from_table:
            raise ValueError("Alternative left pad collision geometry requires table pickup")
        if self.left_block_direct_normal_solref is not None:
            values = np.asarray(self.left_block_direct_normal_solref, dtype=float)
            if values.shape != (2,) or not np.isfinite(values).all() or not np.all(values < 0):
                raise ValueError("Direct normal solref requires two finite strictly negative parameters")
            # Retain a hashable configuration after JSON restores a list.
            object.__setattr__(self, "left_block_direct_normal_solref",
                               tuple(float(value) for value in values))
        if self.left_block_friction_time_constant is not None:
            tau = self.left_block_friction_time_constant
            if not np.isfinite(tau) or tau <= 0 or tau < 2*self.thread.timestep:
                raise ValueError("Resolve the left block friction time constant with at least two steps")
        scalars = (self.head_across_flats, self.head_height, self.steel_density,
                   self.rest_radius, self.rest_pin_radius, self.rest_base_height)
        if not np.isfinite(scalars).all() or min(scalars) <= 0 or not np.isfinite(self.bolt_yaw_rad):
            raise ValueError("Head and rest dimensions must be positive and finite")
        sx, sy, sz = self.base.block_size
        hx, hy, hz = self.hole_offset
        af, height = self.thread.nut_across_flats, self.thread.nut_height
        if hx != 0 or hz != 0 or not np.isclose(height, sz):
            raise ValueError("This decomposition requires a centered through-hole across block thickness")
        end_half = hy + sy/2
        if not np.isclose(end_half, sx/2) or hy >= 0:
            raise ValueError("Hole must lie at the center of a square block-end section")
        cavity_face_radius = self.thread.female_pitch_diameter/2 + 3*np.sqrt(3)*self.thread.pitch/16 + .000360
        if not cavity_face_radius < af/2:
            raise ValueError("Female hex must contain the complete chamfered cavity")
        if not af/np.sqrt(3) < min(sx/2, end_half):
            raise ValueError("Female hex prism must lie strictly within the rectangular block")
        if self.head_across_flats <= self.thread.male_pitch_diameter:
            raise ValueError("Head must be wider than its M8 threaded shaft")
        if self.rest_radius-self.rest_pin_radius <= self.thread.male_pitch_diameter/2 + 3*np.sqrt(3)*self.thread.pitch/16:
            raise ValueError("Support pins must clear the complete male shaft")
        if self.rest_radius+self.rest_pin_radius >= self.head_across_flats/2:
            raise ValueError("All support pins must lie underneath the head")
        if self.bolt_head_position[2]-self.head_height/2 <= self.rest_base_height:
            raise ValueError("Bolt support surface must lie above the rest base")


def table_pickup_config():
    """Physical tabletop acquisition with a separate, low head-support rest.

    The 20 x 120 x 16 mm block initially stands on its 20 x 16 mm end. This
    leaves the actual native fingers and their collision backings clear of the
    table while keeping the qualified hand-to-block transform. The controller
    must lift and physically roll the block to its horizontal assembly pose.
    """
    # A declared 10 nm initial overlap reliably creates the native tabletop
    # candidate despite quaternion/mesh roundoff. Gravity establishes the
    # resting load during settle_table; no constraint or state correction does.
    # Numerical normal contact compliance is specified separately from the
    # original tangential regularization. These native reference coefficients
    # do not define a physical rubber material model or hardware calibration.
    return replace(InsertionConfig(), pickup_from_table=True,
                   block_position=(.25, .15, .060-TABLE_REST_OVERLAP_M),
                   bolt_head_position=(.36, -.22, .040),
                   left_block_direct_normal_solref=(-31250., -2500.),
                   left_block_friction_time_constant=.0008)


def holding_block_position(config: InsertionConfig | None = None):
    cfg = config or InsertionConfig()
    return np.asarray(cfg.held_block_position if cfg.pickup_from_table else cfg.block_position)


def holding_block_rotation(config: InsertionConfig | None = None):
    return baseline.bolt_rotation((config or InsertionConfig()).base)


def block_rotation(config: InsertionConfig | None = None):
    cfg = config or InsertionConfig()
    rotation = holding_block_rotation(cfg)
    if cfg.pickup_from_table:
        # Rx(+90 degrees): the long local Y axis stands along world +Z.
        return rotation @ np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
    return rotation


def female_rotation(config: InsertionConfig | None = None):
    return block_rotation(config) @ np.diag([1., -1., -1.])


def initial_hole_position(config: InsertionConfig | None = None):
    cfg = config or InsertionConfig()
    return np.asarray(cfg.block_position) + block_rotation(cfg) @ np.asarray(cfg.hole_offset)


def initial_bolt_grasp_position(config: InsertionConfig | None = None):
    return np.asarray((config or InsertionConfig()).bolt_head_position)


def initial_bolt_rotation(config: InsertionConfig | None = None):
    cfg = config or InsertionConfig()
    # This parameter is world +Z yaw. The independent fixed .23-radian yaw
    # prevents prescribing an initially registered male/female thread phase.
    c, s = np.cos(cfg.bolt_yaw_rad), np.sin(cfg.bolt_yaw_rad)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]) @ np.diag([1., -1., -1.])


def initial_bolt_origin_position(config: InsertionConfig | None = None):
    cfg = config or InsertionConfig()
    return initial_bolt_grasp_position(cfg) + initial_bolt_rotation(cfg)[:, 2]*cfg.head_height/2


def initial_left_grasp_position(config: InsertionConfig | None = None):
    cfg = config or InsertionConfig()
    return np.asarray(cfg.block_position) + block_rotation(cfg) @ np.asarray(cfg.base.left_grasp_offset)


def left_grasp_rotation(config: InsertionConfig | None = None):
    cfg = config or InsertionConfig()
    rotation = baseline.left_grasp_rotation(cfg.base)
    if cfg.pickup_from_table:
        return block_rotation(cfg) @ holding_block_rotation(cfg).T @ rotation
    return rotation


def holding_left_grasp_position(config: InsertionConfig | None = None):
    cfg = config or InsertionConfig()
    return holding_block_position(cfg) + holding_block_rotation(cfg) @ np.asarray(cfg.base.left_grasp_offset)


def holding_left_grasp_rotation(config: InsertionConfig | None = None):
    return baseline.left_grasp_rotation((config or InsertionConfig()).base)


def left_touch_aperture(config: InsertionConfig | None = None):
    return baseline.left_touch_aperture((config or InsertionConfig()).base)


def jaw_positions(aperture, config: InsertionConfig | None = None):
    return baseline.jaw_positions(aperture, (config or InsertionConfig()).base)


def initial_right_grasp_rotation(config: InsertionConfig | None = None):
    # Local jaw Y follows a head flat normal; tool Z and the male axis both
    # point down. Native fingers approach the head from above.
    return initial_bolt_rotation(config) @ np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])


@lru_cache(maxsize=16)
def bore_mass_properties(config: InsertionConfig, samples=1_048_576):
    """Integrate removed aluminum, using the real helical bore and chamfers."""
    t = config.thread
    uv = qmc.Halton(2, scramble=False).random(samples)
    theta = 2*np.pi*uv[:, 0]
    z = t.nut_height*(uv[:, 1]-.5)
    H, r2 = np.sqrt(3)*t.pitch/2, t.female_pitch_diameter/2
    phase = (z-t.pitch*theta/(2*np.pi)+t.pitch/2) % t.pitch-t.pitch/2
    radius = np.clip(r2 + np.sqrt(3)*(t.pitch/4-np.abs(phase)), r2-H/4, r2+3*H/8)
    radius = np.maximum(radius, r2+3*H/8+.000360-(t.nut_height/2-np.abs(z)))
    return baseline._radial_mass_properties(radius, theta, z, 2*np.pi*t.nut_height,
                                            density=config.base.block_density)


def _combine(parts):
    """Signed rigid-solid mass combination, including the parallel-axis terms."""
    total_mass = sum(sign*float(prop["mass_kg"]) for sign, prop, _, _ in parts)
    mass_moment, origin_inertia = np.zeros(3), np.zeros((3, 3))
    for sign, prop, translation, rotation in parts:
        mass = sign*float(prop["mass_kg"])
        center = np.asarray(translation) + rotation @ np.asarray(prop["centroid_m"])
        inertia = sign*rotation @ np.asarray(prop["inertia_kg_m2"]) @ rotation.T
        mass_moment += mass*center
        origin_inertia += inertia + mass*(np.dot(center, center)*np.eye(3)-np.outer(center, center))
    center = mass_moment/total_mass
    inertia = origin_inertia-total_mass*(np.dot(center, center)*np.eye(3)-np.outer(center, center))
    return {"mass_kg": float(total_mass), "centroid_m": center.tolist(),
            "inertia_kg_m2": inertia.tolist()}


@lru_cache(maxsize=16)
def block_mass_properties(config: InsertionConfig, samples=1_048_576):
    size = np.asarray(config.base.block_size)
    mass = float(np.prod(size)*config.base.block_density)
    inertia = mass/12*np.array([size[1]**2+size[2]**2,
                                size[0]**2+size[2]**2, size[0]**2+size[1]**2])
    whole = {"mass_kg": mass, "centroid_m": [0., 0., 0.], "inertia_kg_m2": np.diag(inertia).tolist()}
    removed = bore_mass_properties(config, samples)
    result = _combine(((1., whole, np.zeros(3), np.eye(3)),
                       (-1., removed, np.asarray(config.hole_offset), np.diag([1., -1., -1.]))))
    result.update(density_kg_m3=config.base.block_density, samples=samples,
                  full_box_mass_kg=mass, removed_bore_mass_kg=removed["mass_kg"])
    return result


@lru_cache(maxsize=16)
def male_mass_properties(config: InsertionConfig, samples=1_048_576):
    af, height, density = config.head_across_flats, config.head_height, config.steel_density
    area, polar = np.sqrt(3)*af**2/2, 5*np.sqrt(3)*af**4/72
    head = {"mass_kg": float(density*area*height), "centroid_m": [0., 0., -height/2],
        "inertia_kg_m2": np.diag([density*(polar*height/2+area*height**3/12),
             density*(polar*height/2+area*height**3/12), density*polar*height]).tolist()}
    shaft = baseline.bolt_mass_properties(config.thread, samples)
    if density != shaft["density_kg_m3"]:
        ratio = density/shaft["density_kg_m3"]
        shaft = {**shaft, "mass_kg": shaft["mass_kg"]*ratio,
                 "inertia_kg_m2": (np.asarray(shaft["inertia_kg_m2"])*ratio).tolist()}
    result = _combine(((1., head, np.zeros(3), np.eye(3)),
                       (1., shaft, np.zeros(3), np.eye(3))))
    result.update(density_kg_m3=density, samples=samples,
                  head_mass_kg=head["mass_kg"], shaft_mass_kg=shaft["mass_kg"])
    return result


def _set_inertial(body, properties):
    old = body.find("inertial")
    if old is not None:
        body.remove(old)
    I = np.asarray(properties["inertia_kg_m2"])
    baseline._add(body, "inertial", pos=baseline._numbers(properties["centroid_m"]),
        mass=f"{properties['mass_kg']:.16g}", fullinertia=baseline._numbers(
            (I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2])))


def _quat(rotation):
    result = np.empty(4)
    mujoco.mju_mat2Quat(result, np.asarray(rotation).ravel())
    return baseline._numbers(result)


def block_strip_geometry(config: InsertionConfig | None = None):
    """Main handle and side strips outside the female hexagonal prism."""
    cfg = config or InsertionConfig()
    x, y, z = np.asarray(cfg.base.block_size)/2
    _, hole_y, _ = cfg.hole_offset
    hex_half = cfg.thread.nut_across_flats/2
    end_upper = hole_y+x
    main_center = (end_upper+y)/2
    main_half = (y-end_upper)/2
    return (
        (BLOCK_GEOM_NAMES[0], (0., main_center, 0.), (x, main_half, z)),
        (BLOCK_GEOM_NAMES[1], ((x+hex_half)/2, hole_y, 0.), ((x-hex_half)/2, x, z)),
        (BLOCK_GEOM_NAMES[2], (-(x+hex_half)/2, hole_y, 0.), ((x-hex_half)/2, x, z)),
    )


def block_outer_prisms(config: InsertionConfig | None = None):
    """Four convex tiles complete the square around the hex without overlap."""
    cfg = config or InsertionConfig()
    half = cfg.thread.nut_across_flats/2
    outer = cfg.base.block_size[0]/2
    peak, corner = 2*half/np.sqrt(3), half/np.sqrt(3)
    left = ((-half, corner), (-half, outer), (0., outer), (0., peak))
    right = ((0., peak), (0., outer), (half, outer), (half, corner))
    return tuple((name, tuple((x, cfg.hole_offset[1]+sign*y) for x, y in polygon))
        for name, polygon, sign in (
            (BLOCK_GEOM_NAMES[3], left, 1), (BLOCK_GEOM_NAMES[4], right, 1),
            (BLOCK_GEOM_NAMES[5], left, -1), (BLOCK_GEOM_NAMES[6], right, -1)))


def _add_prism_mesh(asset, name, polygon, lower, upper):
    polygon = list(polygon)
    area2 = sum(polygon[k][0]*polygon[(k+1)%len(polygon)][1]
                -polygon[(k+1)%len(polygon)][0]*polygon[k][1] for k in range(len(polygon)))
    if area2 < 0:
        polygon.reverse()
    vertices = [(x, y, z) for z in (lower, upper) for x, y in polygon]
    # MuJoCo compiles the exact convex hull; explicit triangulation keeps the
    # mesh portable and supplies all flat outer contact surfaces.
    n = len(polygon)
    faces = []
    for k in range(1, n-1):
        faces.extend(((0, k+1, k), (n, n+k, n+k+1)))
    for k in range(n):
        j = (k+1)%n
        faces.extend(((k, j, n+j), (k, n+j, n+k)))
    return baseline._add(asset, "mesh", name=name, vertex=baseline._numbers(np.ravel(vertices)),
                  face=" ".join(str(v) for v in np.ravel(faces)))


def scene_xml(config: InsertionConfig | None = None):
    cfg = config or InsertionConfig()
    root = ET.fromstring(baseline.scene_xml(cfg.base))
    root.set("model", "dual_yam_m8_bolt_insertion")
    world, asset, contact = root.find("worldbody"), root.find("asset"), root.find("contact")
    if cfg.left_pad_collision_geometry == "convex_mesh":
        x, y, z = baseline.PAD_HALF_SIZE
        mesh_name = "left_m8_pad_convex_shape"
        _add_prism_mesh(asset, mesh_name,
                        ((-x, -y), (x, -y), (x, y), (-x, y)), -z, z)
        for name in ("left_m8_pad_left", "left_m8_pad_right"):
            pad = root.find(f".//geom[@name='{name}']")
            pad.attrib.update(type="mesh", mesh=mesh_name)
            pad.attrib.pop("size")
    old_nut = world.find("body[@name='nut']")
    female_geom = old_nut.find("geom")
    old_nut.remove(female_geom)
    world.remove(old_nut)
    block = world.find("body[@name='fixture_block']")
    old_bolt = block.find("body[@name='bolt_frame']")
    bolt_geom = old_bolt.find("geom[@name='bolt_thread']")
    old_bolt.remove(bolt_geom)
    block.remove(old_bolt)
    old_block_geom = block.find("geom[@name='fixture_block_geom']")
    block.remove(old_block_geom)
    block.set("pos", baseline._numbers(cfg.block_position))
    if cfg.pickup_from_table:
        block.set("quat", _quat(block_rotation(cfg)))
    _set_inertial(block, block_mass_properties(cfg))
    for name, position, size in block_strip_geometry(cfg):
        baseline._add(block, "geom", name=name, type="box", pos=baseline._numbers(position),
            size=baseline._numbers(size), mass="0", material="m8_fixture_metal",
            contype="8", conaffinity="22", friction=".5 0 0", condim="3", margin="0",
            solref=".0008 1", solimp=".95 .99 .0001")
    for name, polygon in block_outer_prisms(cfg):
        mesh_name = name+"_shape"
        _add_prism_mesh(asset, mesh_name, polygon, -cfg.base.block_size[2]/2, cfg.base.block_size[2]/2)
        baseline._add(block, "geom", name=name, type="mesh", mesh=mesh_name, mass="0",
            material="m8_fixture_metal", contype="8", conaffinity="22", friction=".5 0 0",
            condim="3", margin="0", solref=".0008 1", solimp=".95 .99 .0001")
    female = baseline._add(block, "body", name="female_frame",
                          pos=baseline._numbers(cfg.hole_offset), quat=baseline._numbers(DOWN_QUATERNION))
    female_geom.attrib.update(name="female_thread", mass="0", material="m8_fixture_metal",
                             contype="8", conaffinity="22")
    female_geom.attrib.pop("rgba", None)
    female.append(female_geom)
    baseline._add(female, "site", name="hole_origin", pos="0 0 0", size=".001", rgba="0 0 0 0")
    baseline._add(female, "site", name="hole_entry", pos=f"0 0 {-cfg.thread.nut_height/2}",
                  size=".001", rgba="0 0 0 0")
    baseline._add(female, "camera", name="threadcloseup", pos=".035 -.052 -.046",
                  xyaxes=".829 .558 0 .229 -.34 .912", fovy="35")

    male = baseline._add(world, "body", name="male_bolt",
        pos=baseline._numbers(initial_bolt_origin_position(cfg)), quat=_quat(initial_bolt_rotation(cfg)))
    baseline._add(male, "freejoint", name="male_bolt_free")
    _set_inertial(male, male_mass_properties(cfg))
    bolt_geom.attrib.update(mass="0", contype="2", conaffinity="28")
    male.append(bolt_geom)
    # The head is an exact convex regular hexagonal prism, not a visual proxy.
    radius = cfg.head_across_flats/np.sqrt(3)
    polygon = [(radius*np.cos(np.pi/6+k*np.pi/3), radius*np.sin(np.pi/6+k*np.pi/3)) for k in range(6)]
    _add_prism_mesh(asset, "bolt_head_shape", polygon, -cfg.head_height, 0.)
    baseline._add(male, "geom", name="bolt_head", type="mesh", mesh="bolt_head_shape", mass="0",
        rgba=".58 .62 .66 1", contype="2", conaffinity="28", condim="3", margin="0",
        friction=".5 0 0", solref=".0008 1", solimp=".95 .99 .0001")
    baseline._add(male, "site", name="bolt_origin", pos="0 0 0", size=".001", rgba="0 0 0 0")
    baseline._add(male, "site", name="bolt_tip", pos=f"0 0 {cfg.thread.bolt_length}", size=".001", rgba="0 0 0 0")
    baseline._add(male, "site", name="bolt_head_grasp", pos=f"0 0 {-cfg.head_height/2}", size=".001", rgba="0 0 0 0")
    # Three pins support the head underside. Their open center clears the
    # suspended shaft and permits a real upward pickup without a constraint.
    px, py, pz = cfg.bolt_head_position
    top = pz-cfg.head_height/2
    baseline._add(world, "geom", name="bolt_rest_base", type="box", pos=baseline._numbers((px, py, cfg.rest_base_height/2)),
        size=baseline._numbers((.018, .018, cfg.rest_base_height/2)), contype="16", conaffinity="14",
        rgba=".22 .29 .34 1", friction=".5 0 0", margin="0", solref=".0008 1", solimp=".95 .99 .0001")
    for k in range(3):
        angle = k*2*np.pi/3
        baseline._add(world, "geom", name=f"bolt_rest_pin_{k}", type="cylinder",
            pos=baseline._numbers((px+cfg.rest_radius*np.cos(angle), py+cfg.rest_radius*np.sin(angle),
                                   (cfg.rest_base_height+top)/2)),
            size=baseline._numbers((cfg.rest_pin_radius, (top-cfg.rest_base_height)/2)),
            contype="16", conaffinity="14", rgba=".24 .30 .34 1", friction=".5 0 0", margin="0",
            solref=".0008 1", solimp=".95 .99 .0001")
    # Explicit zero-margin thread pair preserves the exact force law. No
    # object actuation or equality is added, and head/block seating collides.
    for pair in list(contact.findall("pair")):
        if pair.get("geom2") == "nut_thread":
            pair.set("geom2", "female_thread")
            continue
        if pair.get("geom1") == "nut_thread":
            pair.set("geom1", "bolt_head")
        elif pair.get("geom1") == "fixture_block_geom":
            if pair.get("geom2", "").startswith("left_m8_pad_"):
                if cfg.left_block_direct_normal_solref is not None:
                    pair.set("solref", baseline._numbers(cfg.left_block_direct_normal_solref))
                if cfg.left_block_friction_time_constant is not None:
                    pair.set("solreffriction", f"{cfg.left_block_friction_time_constant} 1")
            for name in (*BLOCK_GEOM_NAMES[1:], "female_thread"):
                new = ET.fromstring(ET.tostring(pair, encoding="unicode"))
                new.set("geom1", name)
                contact.append(new)
    return ET.tostring(root, encoding="unicode")


def build_spec(config: InsertionConfig | None = None):
    baseline.require_micron_engine()
    if not baseline._loaded:
        mujoco.mj_loadPluginLibrary(str(baseline.build_plugin()))
        baseline._loaded = True
    root = ET.fromstring(scene_xml(config))
    assets = {}
    for mesh in root.findall("asset/mesh"):
        file = mesh.get("file")
        if file:
            path = Path(file)
            assets[path.name] = path.read_bytes()
            mesh.set("file", path.name)
    return mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"), assets=assets)


def build_model(config: InsertionConfig | None = None):
    return build_spec(config).compile()


def scene_fingerprint(config: InsertionConfig | None = None):
    root = ET.fromstring(scene_xml(config))
    mesh_hashes = {}
    for mesh in root.findall("asset/mesh"):
        file = mesh.get("file")
        if file:
            path = Path(file)
            mesh_hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
            mesh.set("file", path.name)
    value = {"xml": ET.tostring(root, encoding="unicode"), "mesh_sha256": mesh_hashes}
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


make_model = build_model
