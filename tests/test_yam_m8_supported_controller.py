"""Native load proofs and free-body preservation for the supported task."""
import hashlib
import json

import mujoco
import numpy as np
import pytest

from yam_twin.m8_supported_simulation import (
    SupportedControlConfig, TableLoadWindow, initialize_supported_pose,
    supported_phases, _table_support_state, _unexpected_native_contacts,
)


def test_stabilization_requires_table_weight_and_limits_upward_hand_load():
    observer = TableLoadWindow(.1, 1.)
    # A candidate with no solved table force cannot qualify. A hand carrying
    # 40% also cannot qualify even if the table still has actual positive load.
    for _ in range(200):
        assert not observer.observe(0., 0., .001, True)
    for _ in range(200):
        assert not observer.observe(.60, .40, .001, True)
    for _ in range(100):
        observer.observe(.95, .05, .001, True)
    assert observer.ready
    report = observer.report()
    np.testing.assert_allclose(report["mean_table_upward_force_N"], .95)
    np.testing.assert_allclose(report["mean_positive_hand_upward_force_N"], .05)
    assert report["loaded_table_substep_duty"] == pytest.approx(1.)
    assert not observer.observe(.95, .05, .001, False)


def test_table_window_reports_resolved_gap_duty_and_accepts_downward_stabilization():
    observer = TableLoadWindow(.1, 1.)
    for tick in range(100):
        observer.observe(0. if tick in (40, 41) else 1., -.2, .001, True)
    assert not observer.ready
    assert observer.report()["loaded_table_substep_duty"] == pytest.approx(.98)
    for _ in range(200):
        observer.observe(1.2, -.2, .001, True)
    assert observer.ready
    assert observer.report()["mean_positive_hand_upward_force_N"] == 0.


@pytest.mark.parametrize("value", [0., -1., np.inf, np.nan, 1.1])
def test_invalid_table_load_fraction_is_rejected(value):
    with pytest.raises(ValueError, match="fractions"):
        TableLoadWindow(.1, 1., minimum_table_fraction=value)


def test_native_table_wrench_sums_welded_child_and_excludes_other_world_support():
    model = mujoco.MjModel.from_xml_string("""
      <mujoco><worldbody>
        <geom name="table" type="plane" size="1 1 .01"/>
        <geom name="other_world" type="plane" pos="0 0 -.005" size="1 1 .01"/>
        <body name="free_block" pos="0 0 .104"><freejoint/>
          <inertial mass=".1" pos="0 0 0" diaginertia=".001 .001 .001"/>
          <body name="female_frame" pos="0 0 -.1">
            <geom name="child_box" type="box" size=".01 .01 .005" mass="0"/>
          </body>
        </body>
      </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    state = _table_support_state(model, data, model.body("free_block").id,
        [model.geom("table").id], with_contacts=True)
    assert state["contact_candidates"] > 0
    assert state["table_upward_force_N"] > .1
    assert state["table_upward_normal_force_N"] == pytest.approx(state["table_upward_force_N"])
    assert all(c["block_geom"] == "child_box" for c in state["contacts"])
    world_forces = []
    for c in state["contacts"]:
        sign = -1. if c["geom1"] == "child_box" else 1.
        f = sign*np.asarray(c["frame"]).T @ np.asarray(c["local_contact_force_N_Nm"])[:3]
        np.testing.assert_allclose(f, c["signed_force_contribution_on_block_world_N"])
        world_forces.append(f)
    np.testing.assert_allclose(np.sum(world_forces, axis=0), state["table_wrench_on_block_world"][:3])
    forbidden = _table_support_state(model, data, model.body("free_block").id, [])
    assert forbidden["unexpected_world_contact_candidates"] > 0
    assert forbidden["table_upward_force_N"] == 0.


def test_supported_initialization_leaves_free_parts_unchanged_and_starts_open():
    from yam_twin.m8_supported_scene import supported_config, build_model
    from yam_twin.m8_insertion_simulation import _body_contact
    from yam_twin.m8_simulation import YamCartesianController
    scene, control = supported_config(), SupportedControlConfig()
    model, data = build_model(scene), None
    data = mujoco.MjData(model)
    qindices, vindices = [], []
    for name in ("fixture_block_free", "male_bolt_free"):
        joint = model.joint(name).id
        qindices.extend(range(model.jnt_qposadr[joint], model.jnt_qposadr[joint]+7))
        vindices.extend(range(model.jnt_dofadr[joint], model.jnt_dofadr[joint]+6))
    data.qvel[vindices] = np.linspace(-.01, .02, 12)
    qbefore, vbefore = data.qpos[qindices].copy(), data.qvel[vindices].copy()
    targets, _, _ = initialize_supported_pose(model, data, scene, control)
    np.testing.assert_array_equal(data.qpos[qindices], qbefore)
    np.testing.assert_array_equal(data.qvel[vindices], vbefore)
    for side, body in (("left", "fixture_block"), ("right", "male_bolt")):
        c = YamCartesianController(model, data, side, control.arm, scene.base)
        np.testing.assert_allclose(c.pose()[0], targets[side][0], atol=2e-6)
        assert _body_contact(model, data, model.body(body).id, c.hand_geom_ids)["contact_count"] == 0
    np.testing.assert_array_equal(data.xfrc_applied, 0.)
    np.testing.assert_array_equal(data.qfrc_applied, 0.)
    assert control.arm.left_closed_aperture == pytest.approx(.0184)
    assert not any("lift_left" in p[0] or "transport_left" in p[0] for p in supported_phases(control))


def test_unexpected_camera_block_collision_is_not_hidden_by_allowed_table_contact():
    model = mujoco.MjModel.from_xml_string("""
      <mujoco><worldbody>
        <geom name="table" type="plane" size="1 1 .01"/>
        <geom name="right_camera" type="box" pos="0 0 .01" size=".01 .01 .01"/>
        <body name="free_block" pos="0 0 .014"><freejoint/>
          <geom name="block" type="box" size=".005 .005 .005" mass=".1"/>
        </body>
        <body name="male_bolt" pos="1 1 1"><freejoint/>
          <geom name="bolt" type="sphere" size=".005" mass=".1"/>
        </body>
      </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    bad = _unexpected_native_contacts(model, data, model.body("free_block").id,
        model.body("male_bolt").id, [model.geom("table").id], [], [], [-1, -2], allow_bolt_rest=False)
    assert any("right_camera" in (c["geom1"], c["geom2"]) for c in bad)


def test_snapshot_binds_all_reused_controller_and_scene_sources(tmp_path):
    from yam_twin.m8_supported_simulation import run_supported_demo
    from scripts.audit_m8_insertion_trace import archived_controller_identity
    report = run_supported_demo(tmp_path, maximum_phases=0)
    assert report["supported_task"]
    digest, _, helpers = archived_controller_identity(tmp_path/"controller_source.py")
    assert digest == report["controller_sha256"]
    assert helpers
    assert hashlib.sha256((tmp_path/"controller_source.py").read_bytes()).hexdigest() == report["controller_module_sha256"]
    dependencies = report["recorded_source_dependencies_sha256"]
    for name in ("m8_insertion_simulation.py", "m8_insertion_scene.py", "m8_scene.py", "m8_supported_scene.py"):
        assert f"recorded_sources/yam_twin/{name}" in dependencies
    for path, digest in dependencies.items():
        assert hashlib.sha256((tmp_path/path).read_bytes()).hexdigest() == digest
