"""Safety-critical reset and support accounting for actual bolt acquisition."""
import mujoco
import numpy as np

from yam_twin.m8_insertion_scene import InsertionConfig, build_model
from yam_twin.m8_insertion_simulation import (
    InsertionControlConfig, initialize_insertion_pose, _body_contact,
)


def test_pickup_reset_preserves_both_free_body_states():
    scene = InsertionConfig()
    model = build_model(scene)
    data = mujoco.MjData(model)
    free_joints = [model.joint(name).id for name in
                   ("fixture_block_free", "male_bolt_free")]
    q_indices = np.concatenate([np.arange(model.jnt_qposadr[j], model.jnt_qposadr[j]+7)
                                for j in free_joints])
    v_indices = np.concatenate([np.arange(model.jnt_dofadr[j], model.jnt_dofadr[j]+6)
                                for j in free_joints])
    # Nonzero free-body velocity catches reset helpers that silently stabilize
    # the workpiece while initializing the arms.
    data.qvel[v_indices] = np.linspace(-.013, .019, 12)
    q_before, v_before = data.qpos[q_indices].copy(), data.qvel[v_indices].copy()
    targets, pickup, _ = initialize_insertion_pose(model, data, scene, InsertionControlConfig())
    np.testing.assert_array_equal(data.qpos[q_indices], q_before)
    np.testing.assert_array_equal(data.qvel[v_indices], v_before)
    np.testing.assert_allclose(targets["right"][0]-pickup, [0., 0., .020])
    for side in ("left", "right"):
        actual = data.site_xpos[model.site(f"{side}_grasp_site").id]
        np.testing.assert_allclose(actual, targets[side][0], atol=2e-6)
    np.testing.assert_array_equal(data.xfrc_applied, 0.)
    np.testing.assert_array_equal(data.qfrc_applied, 0.)


def test_support_accounting_includes_rigid_child_and_fixed_body_counterpart():
    # The workpiece's collision lives on a rigid frame child, and the fixed
    # support lives on a named body rather than body0. Both must be accounted.
    model = mujoco.MjModel.from_xml_string("""
      <mujoco><worldbody>
        <body name="named_fixed_support"><geom type="plane" size="1 1 .01"/></body>
        <body name="free_block" pos="0 0 .1"><freejoint/>
          <inertial mass=".1" pos="0 0 0" diaginertia=".001 .001 .001"/>
          <body name="female_frame" pos="0 0 -.1">
            <geom type="box" size=".01 .01 .005" mass="0"/>
          </body>
        </body>
      </worldbody></mujoco>""")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    root = _body_contact(model, data, model.body("free_block").id)
    child = _body_contact(model, data, model.body("female_frame").id)
    assert root == child
    assert root["world_support_contacts"] > 0
    assert root["contact_count"] > 0
