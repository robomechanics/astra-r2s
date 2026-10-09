"""SI-unit contact model; the bolt is fixed and the nut is dynamic."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import xml.etree.ElementTree as ET

import mujoco

from .build_plugin import build_plugin

_loaded = False


@dataclass(frozen=True)
class ThreadConfig:
    timestep: float = 0.00005
    friction: float = 0.15
    contact_time_constant: float = 0.0005
    # MuJoCo's SDF collider rejects positive distances and ignores positive
    # margins. A positive solver margin makes contacts repeatedly disappear.
    contact_margin: float = 0.
    contact_impratio: float = 1.
    sdf_initpoints: int = 40
    sdf_iterations: int = 30
    bolt_length: float = 0.030
    pitch: float = 0.00125
    male_pitch_diameter: float = 0.007100
    female_pitch_diameter: float = 0.007268
    nut_height: float = 0.0065
    nut_across_flats: float = 0.013
    nut_mass: float = 0.005195807
    nut_transverse_inertia: float = 9.70861e-8
    nut_axial_inertia: float = 1.58804e-7
    initial_z: float = 0.0175
    gravity: float = -9.81
    with_gripper: bool = False
    with_bolt: bool = True
    guided: bool = False

    def __post_init__(self):
        if self.timestep <= 0 or self.friction < 0:
            raise ValueError("Positive timestep and nonnegative friction required")
        if self.contact_time_constant < 2 * self.timestep:
            raise ValueError("Resolve the contact time constant with at least two steps")
        if self.female_pitch_diameter <= self.male_pitch_diameter:
            raise ValueError("The specified fit must have positive flank clearance")
        if self.guided and self.with_gripper:
            raise ValueError("The hand task requires a free 6-DOF nut")

    def as_dict(self):
        return asdict(self)


def model_xml(config: ThreadConfig | None = None, *, gripper_config=None) -> str:
    c = config or ThreadConfig()
    root = ET.Element("mujoco", model="m8_contact_lab")
    ET.SubElement(root, "compiler", angle="radian", autolimits="true")
    option = ET.SubElement(root, "option", timestep=str(c.timestep),
                  gravity=f"0 0 {c.gravity}", integrator="implicitfast",
                  impratio=str(c.contact_impratio),
                  solver="Newton", cone="elliptic", iterations="100",
                  tolerance="1e-10", sdf_iterations=str(c.sdf_iterations),
                  sdf_initpoints=str(c.sdf_initpoints))
    ET.SubElement(option, "flag", energy="enable")
    ET.SubElement(root, "size", memory="50M")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="1280", offheight="720")
    ET.SubElement(visual, "quality", shadowsize="2048")
    ET.SubElement(visual, "map", znear="0.001", zfar="20")
    ET.SubElement(root, "statistic", center="0 0 .015", extent=".09")
    default = ET.SubElement(root, "default")
    contact = dict(solref=f"{c.contact_time_constant} 1",
                   solimp=".99 .999 .00005", margin=str(c.contact_margin),
                   condim="3", friction=f"{c.friction} 0 0")
    ET.SubElement(default, "geom", **contact)
    extension = ET.SubElement(root, "extension")
    plugin = ET.SubElement(extension, "plugin", plugin="astra.m8_thread")
    asset = ET.SubElement(root, "asset")
    for name, params in (
        ("bolt_shape", dict(diameter=".008", pitch=c.pitch, length=c.bolt_length,
                            pitch_diameter=c.male_pitch_diameter, female=0,
                            chamfer=".000956")),
        ("nut_shape", dict(diameter=".008", pitch=c.pitch, length=c.nut_height,
                           pitch_diameter=c.female_pitch_diameter, female=1,
                           af=c.nut_across_flats, chamfer=".000360")),
    ):
        instance = ET.SubElement(plugin, "instance", name=name)
        for key, value in params.items():
            ET.SubElement(instance, "config", key=key, value=str(value))
        mesh = ET.SubElement(asset, "mesh", name=name)
        ET.SubElement(mesh, "plugin", instance=name)
    ET.SubElement(asset, "texture", name="floor_grid", type="2d", builtin="checker",
                  rgb1=".22 .25 .29", rgb2=".26 .29 .34", width="256", height="256")
    ET.SubElement(asset, "material", name="floor_mat", texture="floor_grid",
                  texrepeat="8 8", reflectance=".05")
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos=".04 -.04 .1", dir="-.4 .4 -1", diffuse=".8 .8 .8")
    ET.SubElement(world, "light", pos="-.06 .04 .07", dir=".4 -.2 -1", diffuse=".5 .5 .6")
    ET.SubElement(world, "geom", name="floor", type="plane", pos="0 0 -.004",
                  size=".2 .2 .001", material="floor_mat", contype="0", conaffinity="0")
    ET.SubElement(world, "geom", name="fixture", type="box", pos="0 0 -.002",
                  size=".024 .020 .002", rgba=".14 .25 .36 1", contype="0", conaffinity="0")
    for x in (-.017, .017):
        for y in (-.014, .014):
            ET.SubElement(world, "geom", type="cylinder", pos=f"{x} {y} .0002",
                          size=".002 .0002", rgba=".12 .14 .17 1", contype="0", conaffinity="0")
    if c.with_bolt:
        bolt = ET.SubElement(world, "geom", name="bolt_thread", type="sdf",
                             mesh="bolt_shape", rgba=".57 .62 .68 1", contype="1", conaffinity="2")
        ET.SubElement(bolt, "plugin", instance="bolt_shape")
    nut = ET.SubElement(world, "body", name="nut", pos=f"0 0 {c.initial_z}")
    if c.guided:
        # These independent DOFs are a bearing fixture for analytical checks.
        # There is deliberately no equality coupling the slide and hinge.
        ET.SubElement(nut, "joint", name="nut_z", type="slide", axis="0 0 1")
        ET.SubElement(nut, "joint", name="nut_yaw", type="hinge", axis="0 0 1")
    else:
        ET.SubElement(nut, "freejoint", name="nut_free")
    # Independent steel-volume integration includes the two bore chamfers.
    # Micron-scale center offset / small off-diagonal terms are neglected.
    ET.SubElement(nut, "inertial", pos="0 0 0", mass=str(c.nut_mass),
                  diaginertia=f"{c.nut_transverse_inertia} {c.nut_transverse_inertia} {c.nut_axial_inertia}")
    geom = ET.SubElement(nut, "geom", name="nut_thread", type="sdf",
                         mesh="nut_shape", rgba=".80 .65 .30 1", contype="2", conaffinity="5")
    ET.SubElement(geom, "plugin", instance="nut_shape")
    if c.with_bolt:
        pairs = ET.SubElement(root, "contact")
        ET.SubElement(pairs, "pair", geom1="bolt_thread", geom2="nut_thread",
                      **{k:v for k,v in contact.items() if k != "friction"},
                      friction=f"{c.friction} {c.friction} 0 0 0")
    if c.with_gripper:
        from .gripper import add_xml_gripper
        add_xml_gripper(root, initialcenter=(0, 0, c.initial_z), nut_geom="nut_thread", config=gripper_config)
    return ET.tostring(root, encoding="unicode")


def make_model(config: ThreadConfig | None = None, *, gripper_config=None) -> mujoco.MjModel:
    global _loaded
    if not _loaded:
        mujoco.mj_loadPluginLibrary(str(build_plugin()))
        _loaded = True
    return mujoco.MjModel.from_xml_string(model_xml(config, gripper_config=gripper_config))
