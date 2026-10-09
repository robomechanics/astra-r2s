"""Physical invariants of the reconstructed workcell and thread surrogate.

The short controller tests remove gravity and contacts to isolate engagement;
the helix test keeps gravity and fixes the board to expose constraint compliance.
No long choreography replay is needed to check these invariants.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET
import zipfile

import mujoco
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from yam_twin.kinematics import ArmIK, HOME
from yam_twin.scene import DRIVER_TIP_Z, INSERTION, PITCH, ROOT, TURNS, build_spec, scene_xml
from yam_twin.simulation import TwinSimulation, engagement


@pytest.mark.parametrize("directory", [ROOT / "assets/yam", ROOT / "assets/yam/mjlab_reference"])
def test_vendored_assets_match_pinned_provenance(directory: Path):
    provenance = json.loads((directory / "PROVENANCE.json").read_text())
    assert re.fullmatch(r"[0-9a-f]{40}", provenance["upstream_commit"])
    assert provenance["copied_files_unchanged"] is True
    assert provenance["files"]
    for entry in provenance["files"]:
        path = directory / entry["path"]
        assert path.is_file(), f"Missing upstream asset: {path}"
        assert hashlib.sha256(path.read_bytes()).hexdigest() == entry["sha256"], path


@pytest.mark.parametrize("relative", [
    "assets/yam/arm/yam/v1/yam.xml",
    "assets/yam/station/yam_station_crank_4310_d405/yam_station_crank_4310_d405.xml",
    "assets/yam/station/yam_station_linear_4310_d405/yam_station_linear_4310_d405.xml",
    "assets/yam/mjlab_reference/xmls/yam.xml",
])
def test_unmodified_source_models_compile(relative: str):
    model = mujoco.MjModel.from_xml_path(str(ROOT / relative))
    assert model.nmesh > 0 and model.nv >= 6
    assert np.isfinite(model.body_mass).all()
    assert (model.body_mass[1:] > 0).any()


def test_export_is_self_contained_and_recompiles(tmp_path):
    path = tmp_path / "dual_yam.zip"
    original = build_spec()
    original_model = original.compile()
    original.to_zip(path)
    with zipfile.ZipFile(path) as archive:
        assert len([name for name in archive.namelist() if name.endswith(".stl")]) == 18
        xml_names = [name for name in archive.namelist() if name.endswith(".xml")]
        assert len(xml_names) == 1
        xml = archive.read(xml_names[0])
        assert b"/workspace" not in xml
        for mesh in ET.fromstring(xml).findall("asset/mesh"):
            assert not Path(mesh.get("file")).is_absolute()
            assert mesh.get("file") in archive.namelist()
    restored_model = mujoco.MjSpec.from_zip(path).compile()
    assert restored_model.nmesh == 18
    assert (restored_model.nq, restored_model.nv, restored_model.nu) == (
        original_model.nq, original_model.nv, original_model.nu)
    np.testing.assert_allclose(restored_model.body_mass, original_model.body_mass)
    np.testing.assert_allclose(restored_model.mesh_vert, original_model.mesh_vert)


@pytest.mark.parametrize("side", ["left", "right"])
def test_ik_recovers_reachable_pose_with_real_joint_limits(side: str):
    model = TwinSimulation().model
    ik = ArmIK(model, side)
    known_configuration = np.array([.2, 1.3, 1.2, -1., .4, -.3])
    position, rotation = ik.pose(known_configuration)
    solution = ik.solve(position, rotation, thorough=True)
    recovered_position, recovered_rotation = ik.pose(solution)
    assert np.all(solution >= ik.bounds[0])
    assert np.all(solution <= ik.bounds[1])
    assert np.linalg.norm(position - recovered_position) < .002
    assert np.linalg.norm(Rotation.from_matrix(rotation @ recovered_rotation.T).as_rotvec()) < .02


@pytest.mark.parametrize("offset,tilt,turning,expected", [
    ([0, 0, 0], 0, True, True),
    ([.010, 0, 0], 0, True, False),
    ([0, 0, .010], 0, True, False),
    ([0, 0, 0], 30, True, False),
    ([0, 0, 0], 0, False, False),
])
def test_engagement_depends_on_relative_geometry(offset, tilt, turning, expected):
    # Rotate and translate the entire scene: engagement must be frame independent.
    world = Rotation.from_euler("xyz", [31, -47, 62], degrees=True).as_matrix()
    head = np.array([.13, -.04, .27])
    normal = world[:, 2]
    tool_axis = world @ Rotation.from_euler("x", tilt, degrees=True).apply([0, 0, -1])
    actual = engagement(head + world @ offset, head, tool_axis, normal, turning)
    assert actual[0] is expected


@pytest.fixture
def isolated_controller():
    sim = TwinSimulation()
    sim.model.opt.gravity[:] = 0
    sim.model.geom_contype[:] = 0
    sim.model.geom_conaffinity[:] = 0
    sim.data.qvel[:] = 0
    board_qadr = sim.model.joint("plate_free").qposadr[0]
    sim.data.qpos[board_qadr:board_qadr + 7] = [.35, 0, .5, 1, 0, 0, 0]
    sim.data.qpos[sim.thread_q] = 2 * math.pi
    sim.data.qpos[sim.depth_q] = -PITCH
    for side, ik in sim.ik.items():
        sim.data.qpos[ik.qadr] = HOME
        sim.last_command[side] = HOME.copy()
    mujoco.mj_forward(sim.model, sim.data)
    head, _ = sim.site_pose("screw_head_site")
    driver = sim.model.joint("driver_free")
    driver_qadr = driver.qposadr[0]
    sim.data.qpos[driver_qadr:driver_qadr + 7] = [*(head + [0, 0, DRIVER_TIP_Z]), 0, 1, 0, 0]
    # MuJoCo free-joint rotation velocity is local. The flipped driver axis
    # maps positive local Z to clockwise world -Z.
    sim.data.qvel[driver.dofadr[0] + 3:driver.dofadr[0] + 6] = [0, 0, 4]
    sim.turning = True
    mujoco.mj_forward(sim.model, sim.data)
    return sim


@pytest.mark.parametrize("condition", ["idle", "lateral", "axial", "tilted", "reverse"])
def test_thread_cannot_advance_without_an_engaged_clockwise_turn(isolated_controller, condition: str):
    sim = isolated_controller
    driver = sim.model.joint("driver_free")
    qadr, dadr = driver.qposadr[0], driver.dofadr[0]
    if condition == "idle":
        sim.turning = False
    elif condition == "lateral":
        sim.data.qpos[qadr] += .010
    elif condition == "axial":
        sim.data.qpos[qadr + 2] += .010
    elif condition == "tilted":
        sim.data.qpos[qadr + 3:qadr + 7] = np.roll(Rotation.from_euler("x", 150, degrees=True).as_quat(), 1)
    elif condition == "reverse":
        sim.data.qvel[dadr + 5] = -4
    mujoco.mj_forward(sim.model, sim.data)
    start = sim.data.qpos[[sim.thread_q, sim.depth_q]].copy()
    for _ in range(100):
        sim._physics_step()
    assert abs(sim.data.qpos[sim.thread_q] - start[0]) < 1e-6
    assert abs(sim.data.qpos[sim.depth_q] - start[1]) < 1e-7


def test_engaged_turn_inserts_screw_with_the_m4_pitch(isolated_controller):
    sim = isolated_controller
    start_angle = sim.data.qpos[sim.thread_q]
    start_depth = sim.data.qpos[sim.depth_q]
    for _ in range(100):
        sim._physics_step()
    angle = sim.data.qpos[sim.thread_q] - start_angle
    insertion = start_depth - sim.data.qpos[sim.depth_q]
    assert angle > .3
    assert insertion > .00003
    assert PITCH == pytest.approx(.0007)  # ISO M4 coarse thread assumption.
    assert abs(insertion - angle * .0007 / (2 * math.pi)) < .000015


def test_co_rotating_board_and_driver_do_not_turn_the_screw(isolated_controller):
    sim = isolated_controller
    board = sim.model.joint("plate_free")
    sim.data.qvel[board.dofadr[0] + 3:board.dofadr[0] + 6] = [0, 0, -4]
    mujoco.mj_forward(sim.model, sim.data)
    start_angle = sim.data.qpos[sim.thread_q]
    for _ in range(100):
        sim._physics_step()
    assert abs(sim.data.qpos[sim.thread_q] - start_angle) < .001


def test_thread_seats_without_overinsertion_at_excessive_driver_speed(isolated_controller):
    sim = isolated_controller
    sim.data.qpos[sim.thread_q] = TURNS * 2 * math.pi - .5
    sim.data.qpos[sim.depth_q] = -sim.data.qpos[sim.thread_q] * PITCH / (2 * math.pi)
    mujoco.mj_forward(sim.model, sim.data)
    head, _ = sim.site_pose("screw_head_site")
    driver = sim.model.joint("driver_free")
    qadr, dadr = driver.qposadr[0], driver.dofadr[0]
    sim.data.qpos[qadr:qadr + 3] = head + [0, 0, DRIVER_TIP_Z]
    sim.data.qvel[dadr + 5] = 40  # Much faster than the intended wrist stroke.
    mujoco.mj_forward(sim.model, sim.data)
    for _ in range(300):
        sim._physics_step()
        assert abs(sim.data.actuator_force[sim.thread_ctrl]) <= .0400001
        assert -sim.data.qpos[sim.depth_q] <= INSERTION + .00001
    assert -sim.data.qpos[sim.depth_q] >= INSERTION - .00001
    assert sim.data.ctrl[sim.thread_ctrl] == 0
    assert not any(w.number for w in sim.data.warning)


def test_helix_holds_under_gravity_and_turn_stop_transients():
    # Fix the board in its tilted working orientation. Unlike free fall, this
    # puts the screw's weight through its thread guide and exposes soft equality.
    xml = ET.fromstring(scene_xml())
    plate = xml.find("worldbody/body[@name='plate']")
    plate.remove(plate.find("freejoint"))
    plate.set("pos", "0 0 .5")
    quat = np.roll(Rotation.from_euler("y", -60, degrees=True).as_quat(), 1)
    plate.set("quat", " ".join(str(x) for x in quat))
    model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
    model.geom_contype[:] = 0
    model.geom_conaffinity[:] = 0
    data = mujoco.MjData(model)
    angle = model.joint("screw_angle").qposadr[0]
    depth = model.joint("screw_depth").qposadr[0]
    drive = model.actuator("thread_drive").id
    max_error = 0.
    for step in range(1000):
        data.ctrl[drive] = 8. if 100 <= step < 800 else 0.
        mujoco.mj_step(model, data)
        error = abs(data.qpos[depth] + data.qpos[angle] * .0007 / (2 * math.pi))
        max_error = max(max_error, error)
    assert data.qpos[angle] > 5
    assert -data.qpos[depth] > .0005
    assert max_error < .000015, f"Thread pitch violated by {max_error * 1000:.4f} mm"
    assert not any(w.number for w in data.warning)


def test_arm_bias_compensation_remains_inside_torque_limits(isolated_controller):
    sim = isolated_controller
    # A deliberately impossible target exercises saturation instead of merely
    # observing a benign posture with naturally small actuator forces.
    for side in sim.ik:
        sim.last_command[side] = np.full(6, 1000.)
    sim._physics_step()
    for side, ik in sim.ik.items():
        for i, dof in enumerate(ik.dadr, 1):
            actuator = sim.model.actuator(f"{side}_servo{i}").id
            lower, upper = sim.model.actuator_forcerange[actuator]
            assert lower < 0 < upper
            assert lower <= sim.data.actuator_force[actuator] <= upper
            assert sim.data.qfrc_applied[dof] == 0
            assert lower <= sim.data.qfrc_actuator[dof] <= upper
    assert np.isfinite(sim.data.qpos).all()
    assert np.isfinite(sim.data.qvel).all()
