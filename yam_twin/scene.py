"""Compose two unmodified mjlab YAM assets and the reconstructed workcell.

SI units throughout. Grasp welds and the thread guide are deliberate reduced
models; they do not claim to simulate frictional grasping or thread teeth.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
YAM_XML = ROOT / "assets/yam/mjlab_reference/xmls/yam.xml"
PITCH = 0.0007  # M4 coarse thread, not measured from video
INSERTION = 0.0084
TURNS = INSERTION / PITCH
PLATE_START = np.array([0.33, 0.035, 0.055])
DRIVER_START = np.array([0.33, -0.15, 0.060])
SCREW_HEAD_Z = 0.003 + INSERTION + 0.002
DRIVER_TIP_Z = 0.145  # tool coordinates: grip centre to Phillips tip


def numbers(v) -> str:
    return " ".join(f"{float(x):.10g}" for x in v)


def add(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})


def scene_xml() -> str:
    original = ET.parse(YAM_XML).getroot()
    root = ET.Element("mujoco", model="dual_yam_screw_twin")
    add(root, "compiler", angle="radian", autolimits="true")
    add(root, "option", timestep="0.002", integrator="implicitfast", iterations="80", gravity="0 0 -9.81")
    add(root, "size", njmax="4000", nconmax="1000")
    visual = add(root, "visual")
    add(visual, "global", offwidth="1280", offheight="960")
    add(visual, "quality", shadowsize="1024")
    add(visual, "headlight", ambient=".20 .20 .20", diffuse=".30 .30 .30", specular=".12 .12 .12")
    add(visual, "rgba", haze=".86 .89 .93 1")
    defaults = copy.deepcopy(original.find("default"))
    root.append(defaults)
    # Capsule collisions are hidden in rendered frames; original meshes retain
    # their white/black materials. Contacts remain enabled with the tabletop.
    for geom in defaults.iter("geom"):
        if geom.get("group") in ("3", "4"):
            geom.set("friction", ".7 .01 .001")
            geom.set("solref", ".008 1")
    assets = copy.deepcopy(original.find("asset"))
    root.append(assets)
    add(assets, "texture", name="sky", type="skybox", builtin="gradient", rgb1=".18 .22 .28", rgb2=".30 .34 .39", width="256", height="256")
    for mesh in assets.findall("mesh"):
        file = mesh.get("file")
        mesh.set("name", mesh.get("name", Path(file).stem))
        mesh.set("file", str((YAM_XML.parent / "assets" / file).resolve()))
    for name, rgba in [("silver", ".60 .64 .68 1"), ("yellow", "1 .65 .035 1"),
                       ("tool_black", ".035 .04 .045 1"), ("rail", ".55 .60 .63 1")]:
        add(assets, "material", name=name, rgba=rgba, specular=".5", shininess=".45")
    world = add(root, "worldbody")
    add(world, "light", name="key", pos=".2 0 1.6", dir="0 0 -1", diffuse=".55 .55 .55", castshadow="true")
    add(world, "light", name="fill", pos="-.6 .6 .9", dir="1 -1 -1", diffuse=".2 .2 .2", castshadow="false")
    # y+ is image-left, x+ is away from the front mounting rail.
    add(world, "camera", name="overview", pos="-.48 0 1.22", xyaxes="0 -1 0 .83 0 .56", fovy="49")
    add(world, "camera", name="closeup", pos=".00 -.31 .72", xyaxes=".68 -.73 0 .40 .37 .84", fovy="42")
    add(world, "geom", name="table", type="box", size=".46 .49 .018", pos=".32 0 -.018", rgba=".78 .79 .76 1", friction=".9 .02 .002")
    add(world, "geom", name="rear_wall", type="box", size=".015 .48 .27", pos=".77 0 .25", rgba=".86 .87 .87 1", contype="0", conaffinity="0")
    for y in [-.48, .48]:
        add(world, "geom", type="box", size=".45 .009 .27", pos=numbers([.32,y,.25]), rgba=".86 .87 .87 1", contype="0", conaffinity="0")
    # Aluminum extrusion with visible grooves and four base bolts.
    add(world, "geom", type="box", name="mounting_rail", pos="0 0 .014", size=".035 .395 .014", material="rail")
    for x in [-.022, -.01, .01, .022]:
        add(world, "geom", type="box", pos=numbers([x,0,.0284]), size=".0014 .395 .0005", rgba=".18 .22 .25 1", contype="0", conaffinity="0")
    # Small estimated pickup supports keep the jaw tips clear of the tabletop.
    add(world, "geom", name="plate_rest", type="box", pos=numbers([PLATE_START[0],PLATE_START[1],(PLATE_START[2]-.003)/2]), size=numbers([.045,.025,(PLATE_START[2]-.003)/2]), rgba=".66 .68 .67 1")
    # Open-topped, inclined tool cradle. Side fences prevent a round handle from
    # rolling away; the front gap clears the shaft when returning the tool.
    incline = math.atan(.2)
    rest = add(world, "body", name="driver_rest", pos=numbers(DRIVER_START-np.array([math.sin(incline)*.014,0,math.cos(incline)*.014])), euler=numbers([0,incline,0]))
    add(rest, "geom", type="box", pos="0 0 -.017", size=".051 .025 .017", rgba=".17 .18 .18 1", friction="1 .02 .002")
    for y in [-.021,.021]:
        add(rest, "geom", type="box", pos=numbers([0,y,.007]), size=".051 .004 .008", rgba=".17 .18 .18 1")
    add(rest, "geom", type="box", pos="-.050 0 .007", size=".004 .025 .010", rgba=".17 .18 .18 1")
    for y in [-.014,.014]:
        add(rest, "geom", type="box", pos=numbers([.047,y,.007]), size=".004 .006 .010", rgba=".17 .18 .18 1")
    eq = add(root, "equality")
    contact = add(root, "contact")
    actuator = add(root, "actuator")
    for side, y in [("left", .305), ("right", -.305)]:
        body = copy.deepcopy(original.find("worldbody/body"))
        # Prefix frame/joint/site names, preserving shared mesh and material names.
        for elem in body.iter():
            if "name" in elem.attrib:
                elem.set("name", side + "_" + elem.get("name"))
        body.set("pos", numbers([0, y, .028]))
        world.append(body)
        cam = body.find(".//camera")
        if cam is not None:
            # The native camera has sensor intrinsics; retain them for wrist views.
            pass
        for i in range(1,7):
            joint = body.find(f".//joint[@name='{side}_joint{i}']")
            joint.set("damping", ".6" if i <= 3 else ".08")
            joint.set("armature", ".032" if i <= 3 else ".0018")
            # High but finite PD gains; torque limits from mjlab YAM constants.
            add(actuator, "position", name=f"{side}_servo{i}", joint=f"{side}_joint{i}", kp="450" if i <= 3 else "100", kv="22" if i <= 3 else "1.5", forcerange="-28 28" if i <= 3 else "-10 10")
        for finger in ["left_finger", "right_finger"]:
            body.find(f".//joint[@name='{side}_{finger}']").set("damping", "2")
        add(eq, "joint", name=f"{side}_finger_coupling", joint1=f"{side}_left_finger", joint2=f"{side}_right_finger", polycoef="0 -1 0 0 0")
        add(actuator, "position", name=f"{side}_grip", joint=f"{side}_left_finger", kp="500", kv="8", forcerange="-12 12")
        # Self-adjacent link exclusions; physical collision bodies are otherwise retained.
        for a,b in [("link_4","link_6"),("link_5","link_left_finger"),("link_5","link_right_finger")]:
            add(contact, "exclude", body1=f"{side}_{a}", body2=f"{side}_{b}")
        # Keep held objects free of duplicate contact forces at the weld grasp.
        for held in ["plate", "screwdriver"]:
            for link in ["arm","link_1","link_2","link_3","link_4","link_5","link_6", "camera_d405", "link_left_finger", "link_right_finger", "lf_rot", "rf_rot", "lf_down", "rf_down"]:
                add(contact, "exclude", body1=f"{side}_{link}", body2=held)
        for link in ["link_5","link_6","camera_d405","link_left_finger","link_right_finger","lf_rot","rf_rot","lf_down","rf_down"]:
            add(contact, "exclude", body1=f"{side}_{link}", body2="driver_rest")

    board = add(world, "body", name="plate", pos=numbers(PLATE_START))
    add(board, "freejoint", name="plate_free")
    add(board, "inertial", pos="0 0 0", mass=".14", diaginertia=".00008 .00014 .0002")
    add(board, "geom", name="plate_solid", type="box", size=".06 .042 .003", material="silver", friction=".8 .01 .001")
    # Hole markings; the centre active hole uses the explicit thread guide below.
    for x in [-.038,0,.038]:
        for y in [-.025,0,.025]:
            add(board, "geom", type="cylinder", pos=numbers([x,y,.00315]), size=".0027 .00015", material="tool_black", contype="0", conaffinity="0", mass="0")
    add(board, "site", name="plate_grasp", pos="0 .043 0", size=".003", rgba="0 0 0 0")
    screw = add(board, "body", name="screw", pos=numbers([0,0,SCREW_HEAD_Z]))
    add(screw, "joint", name="screw_depth", type="slide", axis="0 0 1", range=numbers([-INSERTION,0]), damping=".2")
    add(screw, "joint", name="screw_angle", type="hinge", axis="0 0 -1", range=numbers([0, TURNS*2*math.pi]), damping=".002", armature=".00002")
    add(screw, "geom", name="screw_shank", type="cylinder", pos="0 0 -.005", size=".002 .005", material="silver", mass=".002", contype="0", conaffinity="0")
    add(screw, "geom", name="screw_head", type="cylinder", size=".0038 .002", material="silver", mass=".001", contype="0", conaffinity="0")
    for sz in [".0028 .0005 .00012", ".0005 .0028 .00012"]:
        add(screw, "geom", type="box", pos="0 0 .00205", size=sz, material="tool_black", contype="0", conaffinity="0", mass="0")
    # Visible thread rings are illustrations; equality below supplies the helix.
    for z in np.arange(-.009, -.001, PITCH):
        add(screw, "geom", type="cylinder", pos=numbers([0,0,z]), size=".00225 .00012", material="silver", mass="0", contype="0", conaffinity="0")
    add(screw, "site", name="screw_head_site", pos="0 0 .0021", size=".001", rgba="0 0 0 0")
    add(eq, "joint", name="thread_helix", joint1="screw_depth", joint2="screw_angle", polycoef=numbers([0,-PITCH/(2*math.pi),0,0,0]), solref=".004 1", solimp=".9999 .9999 .000001 .5 2")
    add(actuator, "velocity", name="thread_drive", joint="screw_angle", kv=".012", forcerange="-.04 .04")

    driver = add(world, "body", name="screwdriver", pos=numbers(DRIVER_START), quat=".63398891 0 .77334214 0")
    add(driver, "freejoint", name="driver_free")
    add(driver, "inertial", pos="0 0 .02", mass=".085", diaginertia=".00015 .00015 .000015")
    add(driver, "geom", name="driver_handle", type="cylinder", size=".014 .044", material="yellow", friction="1 .02 .002")
    add(driver, "geom", type="cylinder", pos="0 0 -.041", size=".0148 .007", material="tool_black")
    add(driver, "geom", type="capsule", fromto="0 0 .045 0 0 .141", size=".0021", material="silver")
    for i in range(8):
        angle = i*2*math.pi/8
        add(driver, "geom", type="capsule", fromto=numbers([.013*math.cos(angle),.013*math.sin(angle),-.026,.013*math.cos(angle),.013*math.sin(angle),.026]), size=".0017", material="tool_black", contype="0", conaffinity="0", mass="0")
    for size in [".0018 .0004 .002", ".0004 .0018 .002"]:
        add(driver, "geom", type="box", pos="0 0 .143", size=size, material="silver", contype="0", conaffinity="0")
    add(driver, "site", name="driver_tip", pos=numbers([0,0,DRIVER_TIP_Z]), size=".001", rgba="0 0 0 0")
    add(driver, "site", name="driver_grasp", pos="0 0 0", size=".002", rgba="0 0 0 0")
    # Weld relposes are populated from measured simulation poses when grasping.
    add(eq, "weld", name="plate_grip", body1="left_link_6", body2="plate", active="false", solref=".008 1")
    add(eq, "weld", name="driver_grip", body1="right_link_6", body2="screwdriver", active="false", solref=".008 1")
    # Prevent shaft/plate contacts competing with the analytic engagement model.
    add(contact, "exclude", body1="plate", body2="screwdriver")
    return ET.tostring(root, encoding="unicode")


def build_spec() -> mujoco.MjSpec:
    root = ET.fromstring(scene_xml())
    assets = {}
    for mesh in root.find("asset").findall("mesh"):
        path = Path(mesh.get("file"))
        assets[path.name] = path.read_bytes()
        mesh.set("file",path.name)
    # Supply mesh bytes through MuJoCo's VFS, so MjSpec.to_zip and MJLab's
    # Scene.write actually carry the assets rather than absolute machine paths.
    return mujoco.MjSpec.from_string(ET.tostring(root,encoding="unicode"),assets=assets)


def build_model() -> mujoco.MjModel:
    return build_spec().compile()


def generate_scene(output_path: str | Path) -> Path:
    path = Path(output_path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(scene_xml())
    return path
