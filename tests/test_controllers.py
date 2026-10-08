"""Headless regression checks; no viewer or long training run is started."""
import importlib.util
from pathlib import Path
import unittest

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load_controller(kind):
    name = f"Rotary_Inverted_Pendulum_{kind}"
    spec = importlib.util.spec_from_file_location(name, ROOT / name / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class ControllerTests(unittest.TestCase):
    def tearDown(self):
        mj.set_mjcb_control(None)

    def test_pid_rejects_invalid_timestep_and_resets_accumulator(self):
        pid = load_controller("PID")
        pid.model = mj.MjModel.from_xml_path(pid.xml_path)
        pid.data = mj.MjData(pid.model)
        with self.assertRaises(ValueError):
            pid.pid_control(np.zeros(2), np.ones(2), 0)
        pid.pid_control(np.zeros(2), np.ones(2), 0.01)
        pid.keyboard(None, pid.glfw.KEY_BACKSPACE, 0, pid.glfw.PRESS, 0)
        np.testing.assert_array_equal(pid.integral, np.zeros(2))
        np.testing.assert_array_equal(pid.previous_error, np.zeros(2))

    @unittest.skipUnless(importlib.util.find_spec("control"), "Install control for LQR checks")
    def test_lqr_dynamics_preserve_live_state_and_gain_is_finite(self):
        lqr = load_controller("LQR")
        lqr.model = mj.MjModel.from_xml_path(lqr.xml_path)
        lqr.data = mj.MjData(lqr.model)
        lqr.data.qpos[:] = [0.1, -0.2]
        initial = lqr.data.qpos.copy()
        derivative = lqr.f(np.zeros(4), np.zeros(1))
        self.assertEqual(derivative.shape, (4,))
        np.testing.assert_array_equal(lqr.data.qpos, initial)
        lqr.init_controller(lqr.model, lqr.data)
        self.assertEqual(lqr.K.shape, (1, 4))
        self.assertTrue(np.isfinite(lqr.K).all())
        for _ in range(20):
            lqr.controller(lqr.model, lqr.data)
            mj.mj_step(lqr.model, lqr.data)
        self.assertTrue(np.isfinite(lqr.data.qpos).all())

    @unittest.skipUnless(importlib.util.find_spec("torch"), "Install torch for RL checks")
    def test_one_step_reinforce_keeps_parameters_finite(self):
        rl = load_controller("RL")
        model = mj.MjModel.from_xml_path(rl.xml_path)
        env = rl.MuJoCoEnv(model, mj.MjData(model))
        self.assertEqual(env.reset().shape, (model.nq + model.nv,))
        policy = rl.Policy(model.nq + model.nv, model.nu, 8).to(rl.device)
        optimizer = rl.optim.Adam(policy.parameters(), lr=0.001)
        scores, averages = rl.reinforce(env, policy, optimizer, 1, 1, 0.99, 1)
        self.assertTrue(np.isfinite(scores + averages).all())
        self.assertTrue(all(rl.torch.isfinite(p).all() for p in policy.parameters()))


if __name__ == "__main__":
    unittest.main()
