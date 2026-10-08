"""Policy-interface checks and defenses against invalid-physics reward exploits."""
import mujoco
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

from thread_lab.env import M8NutEnv
from thread_lab.gripper import GripperConfig
from thread_lab.model import ThreadConfig, make_model
from thread_lab.runtime import require_micron_engine


def test_gym_contract_seed_and_geometry_configuration():
    require_micron_engine()
    env = M8NutEnv(control_dt=.001, horizon_seconds=.02)
    check_env(env, skip_render_check=True)
    obs, info = env.reset(seed=7)
    qpos = env.data.qpos.copy()
    again, info_again = env.reset(seed=7)
    assert np.array_equal(qpos, env.data.qpos)
    assert np.array_equal(obs, again)
    assert env.observation_space.contains(obs)
    assert info["starts_preengaged"]
    assert env.pitch == .00125 and env.bolt_length == .03 and env.nut_height == .0065
    info_again["runtime"]["libraries"][0]["path"] = "modified consumer data"
    assert env.runtime_info["libraries"][0]["path"] != "modified consumer data"
    with pytest.raises(ValueError, match="Reward pitch"):
        M8NutEnv(env.model, pitch=.001)
    with pytest.raises(ValueError, match="compiled model"):
        M8NutEnv(env.model, gripper_config=GripperConfig(pad_friction=.3))
    env.close()


def test_invalid_plunge_is_failure_with_negative_reward_before_next_control():
    require_micron_engine()
    env = M8NutEnv(alignment_randomization=False, control_dt=.02)
    env.reset()
    # Inject an invalid geometric state as an adversarial simulator fault.
    # This is not a policy action or a controller command.
    env.data.qpos[env.nut_qpos_addr + 2] -= .003
    mujoco.mj_forward(env.model, env.data)
    _, reward, terminated, _, info = env.step([0, 0, 0, 0, 0, 0, -1])
    assert terminated and info["failure"] and not info["success"]
    assert reward == -1
    assert info["failure_reasons"]
    assert env.data.time <= env.model.opt.timestep * 1.01
    with pytest.raises(RuntimeError, match="reset"):
        env.step(np.zeros(7))
    env.close()


def test_open_hand_wrench_cannot_turn_nut_through_hidden_coupling():
    require_micron_engine()
    model = make_model(ThreadConfig(with_gripper=True))
    baseline = M8NutEnv(model, alignment_randomization=False, control_dt=.005)
    driven = M8NutEnv(model, alignment_randomization=False, control_dt=.005)
    baseline.reset(seed=2)
    driven.reset(seed=2)
    baseline.step([0, 0, 0, 0, 0, 0, -1])
    _, _, terminated, _, info = driven.step([0, 0, 0, 0, 0, .1, -1])
    a = driven.nut_qpos_addr
    assert not terminated
    assert info["pad_contact"]["contact_count"] == 0
    assert np.allclose(driven.data.qpos[a:a + 7], baseline.data.qpos[a:a + 7], atol=1e-8, rtol=0)
    hand_a = driven.hand.qpos_addr
    assert abs(driven.data.qpos[hand_a + 6] - baseline.data.qpos[hand_a + 6]) > 1e-4
    assert np.all(driven.data.xfrc_applied[driven.nut_id] == 0)
    baseline.close()
    driven.close()
