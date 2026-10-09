"""A free M8 female block borne by a real table and stabilized from its sides.

This additive scene keeps the published pickup/carry scene byte-identical. The
same open female thread, short male bolt, inertials, native YAM fingers and
finite motors are reused. The continuous tabletop remains solid below the
through-bore: this is a limited-depth running-thread demonstration, with an
explicit tip/table clearance guard. The block has no attachment to the table.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from . import m8_insertion_scene as insertion
from . import m8_scene as baseline

TABLE_SUPPORT_GEOM_NAMES = ("table",)
BLOCK_GEOM_NAMES = insertion.BLOCK_GEOM_NAMES
TABLE_REST_OVERLAP_M = insertion.TABLE_REST_OVERLAP_M
TABLE_CENTER = (.32, 0., -.018)
TABLE_HALF_SIZE = (.46, .49, .018)


def _supported_base():
    # Rotate the long block in the table plane to separate the two native arm
    # chains and their wrist cameras at the low assembly pose. Its side clamp
    # closes on the 20 mm width, without placing a finger below the block.
    return replace(insertion.InsertionConfig().base, left_grasp_roll_rad=0.,
                   bolt_quaternion=(np.sqrt(.5), 0., 0., np.sqrt(.5)),
                   left_grasp_offset=(0., .045, .006),
                   left_closed_aperture=.0184)


@dataclass(frozen=True)
class SupportedConfig(insertion.InsertionConfig):
    """Physical tabletop assembly, with no lift/carry or preloaded engagement.

    The small declared initial overlap merely creates a native table contact
    candidate; gravity establishes the resting load during the initial settle.
    Direct normal contact coefficients retain the numerical compliance of the
    first demonstration. They are not a calibrated rubber material model.
    """
    base: baseline.YamM8Config = field(default_factory=_supported_base)
    block_position: tuple[float, float, float] = (.30, -.010, .008-TABLE_REST_OVERLAP_M)
    bolt_head_position: tuple[float, float, float] = (.36, -.22, .040)
    held_block_position: tuple[float, float, float] = (.30, -.010, .008-TABLE_REST_OVERLAP_M)
    left_block_friction_time_constant: float | None = .0008
    left_block_direct_normal_solref: tuple[float, float] | None = (-31250., -2500.)
    minimum_tip_table_clearance_m: float = .001
    left_grasp_tilt_rad: float = np.pi/6

    def __post_init__(self):
        super().__post_init__()
        if self.pickup_from_table:
            raise ValueError("The supported scene stabilizes its block without pickup")
        rotation = baseline.bolt_rotation(self.base)
        if not np.allclose(rotation[:, 2], [0., 0., 1.], atol=1e-14):
            raise ValueError("The supported block must rest horizontally")
        if not np.isclose(self.block_position[2], self.base.block_size[2]/2-TABLE_REST_OVERLAP_M,
                          rtol=0., atol=1e-14):
            raise ValueError("The supported block bottom must meet the real table")
        if self.base.left_grasp_roll_rad != 0.:
            raise ValueError("The supported clamp closes on the block's side faces")
        if not np.isfinite(self.left_grasp_tilt_rad) or not 0 <= self.left_grasp_tilt_rad <= np.pi/2:
            raise ValueError("The side-clamp approach tilt must lie between horizontal and downward")
        clearance = self.minimum_tip_table_clearance_m
        if not np.isfinite(clearance) or not 0 < clearance < self.base.block_size[2]:
            raise ValueError("Tip/table clearance must lie strictly within the block thickness")
        lower, upper = np.asarray(TABLE_CENTER[:2])-TABLE_HALF_SIZE[:2], np.asarray(TABLE_CENTER[:2])+TABLE_HALF_SIZE[:2]
        center = np.asarray(self.block_position)
        extent = np.abs(rotation) @ (np.asarray(self.base.block_size)/2)
        if np.any(center[:2]-extent[:2] <= lower) or np.any(center[:2]+extent[:2] >= upper):
            raise ValueError("The supported block must lie strictly inside the tabletop")


def supported_config():
    return SupportedConfig()


def block_rotation(config: SupportedConfig | None = None):
    return baseline.bolt_rotation((config or SupportedConfig()).base)


def holding_block_position(config: SupportedConfig | None = None):
    return np.asarray((config or SupportedConfig()).block_position)


def holding_block_rotation(config: SupportedConfig | None = None):
    return block_rotation(config)


def initial_hole_position(config: SupportedConfig | None = None):
    cfg = config or SupportedConfig()
    return np.asarray(cfg.block_position)+block_rotation(cfg) @ np.asarray(cfg.hole_offset)


def initial_left_grasp_position(config: SupportedConfig | None = None):
    cfg = config or SupportedConfig()
    return np.asarray(cfg.block_position)+block_rotation(cfg) @ np.asarray(cfg.base.left_grasp_offset)


def left_grasp_rotation(config: SupportedConfig | None = None):
    cfg = config or SupportedConfig()
    s, c = np.sin(cfg.left_grasp_tilt_rad), np.cos(cfg.left_grasp_tilt_rad)
    # The jaw normal follows the block's local X side faces. Tilting the side
    # approach down keeps the native finger backings above the real tabletop.
    return block_rotation(cfg) @ np.array([[0., 1., 0.], [s, 0., -c], [-c, 0., -s]])


holding_left_grasp_position = initial_left_grasp_position
holding_left_grasp_rotation = left_grasp_rotation
female_rotation = insertion.female_rotation
initial_bolt_grasp_position = insertion.initial_bolt_grasp_position
initial_bolt_origin_position = insertion.initial_bolt_origin_position
initial_bolt_rotation = insertion.initial_bolt_rotation
initial_right_grasp_rotation = insertion.initial_right_grasp_rotation
left_touch_aperture = insertion.left_touch_aperture
jaw_positions = insertion.jaw_positions
block_mass_properties = insertion.block_mass_properties
male_mass_properties = insertion.male_mass_properties


def table_top_height(config: SupportedConfig | None = None):
    return TABLE_CENTER[2]+TABLE_HALF_SIZE[2]


def maximum_tip_insertion_m(config: SupportedConfig | None = None):
    """Geometric upper bound; a controller must still check the measured tip."""
    cfg = config or SupportedConfig()
    return (initial_hole_position(cfg)[2]+cfg.thread.nut_height/2
            -table_top_height(cfg)-cfg.minimum_tip_table_clearance_m)


def scene_xml(config: SupportedConfig | None = None):
    cfg = config or SupportedConfig()
    root = ET.fromstring(insertion.scene_xml(cfg))
    root.set("model", "dual_yam_m8_table_supported")
    return ET.tostring(root, encoding="unicode")


def build_spec(config: SupportedConfig | None = None):
    baseline.require_micron_engine()
    if not baseline._loaded:
        mujoco.mj_loadPluginLibrary(str(baseline.build_plugin()))
        baseline._loaded = True
    root = ET.fromstring(scene_xml(config))
    assets = {}
    for mesh in root.findall("asset/mesh"):
        filename = mesh.get("file")
        if filename:
            path = Path(filename)
            assets[path.name] = path.read_bytes()
            mesh.set("file", path.name)
    return mujoco.MjSpec.from_string(ET.tostring(root, encoding="unicode"), assets=assets)


def build_model(config: SupportedConfig | None = None):
    return build_spec(config).compile()


def scene_fingerprint(config: SupportedConfig | None = None):
    root = ET.fromstring(scene_xml(config))
    hashes = {}
    for mesh in root.findall("asset/mesh"):
        filename = mesh.get("file")
        if filename:
            path = Path(filename)
            hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
            mesh.set("file", path.name)
    canonical = {"xml": ET.tostring(root, encoding="unicode"), "mesh_sha256": hashes}
    return hashlib.sha256(json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


make_model = build_model
