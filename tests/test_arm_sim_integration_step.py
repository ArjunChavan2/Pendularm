"""Raw-socket integration tests for /arm_sim/integration_step, the Project 2
checkpoint service, exercised over the real gateway (per spec/ROSBRIDGE_PROTOCOL.md).
"""
import asyncio
import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, os.path.dirname(__file__))

import arm_sim_node  # noqa: E402
from gateway import Gateway  # noqa: E402
from registry import Registry  # noqa: E402

from client_helper import Client  # noqa: E402


class _ServerThread:
    def __init__(self) -> None:
        self.loop = asyncio.new_event_loop()
        self.registry = Registry()
        arm_sim_node.register(self.registry)
        self.gateway = Gateway(self.registry)
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.gateway.start())
        self.loop.run_forever()

    def start(self) -> None:
        self._thread.start()
        time.sleep(0.2)

    def stop(self) -> None:
        fut = asyncio.run_coroutine_threadsafe(self.gateway.stop(), self.loop)
        try:
            fut.result(timeout=2)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)
        self._thread.join(timeout=2)


class TestIntegrationStepService(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.server = _ServerThread()
        cls.server.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.stop()

    def test_constant_forcing_matches_closed_form(self):
        with Client() as client:
            resp = client.call_service("/arm_sim/integration_step", {
                "function": "3", "x0": 0.0, "xdot0": 1.0, "dt": 0.1, "steps": 10, "integrator": "rk4",
            })
        self.assertTrue(resp["result"])
        values = resp["values"]
        self.assertEqual(values["times"][0], 0.0)
        self.assertEqual(values["positions"][0], 0.0)
        self.assertEqual(values["velocities"][0], 1.0)
        self.assertEqual(len(values["times"]), 11)  # steps + the t=0 starting point
        T = 1.0
        self.assertAlmostEqual(values["positions"][-1], 0.0 + 1.0 * T + 0.5 * 3.0 * T * T, places=6)
        self.assertAlmostEqual(values["velocities"][-1], 1.0 + 3.0 * T, places=6)

    def test_all_four_integrators_are_selectable(self):
        with Client() as client:
            for name in ("euler", "midpoint", "verlet", "rk4"):
                resp = client.call_service("/arm_sim/integration_step", {
                    "function": "sin(t)", "x0": 0.0, "dt": 0.05, "steps": 5, "integrator": name,
                })
                self.assertTrue(resp["result"], f"integrator {name} failed: {resp}")

    def test_xdot0_defaults_to_zero(self):
        with Client() as client:
            resp = client.call_service("/arm_sim/integration_step", {
                "function": "0", "x0": 5.0, "dt": 0.1, "steps": 1, "integrator": "euler",
            })
        self.assertTrue(resp["result"])
        self.assertEqual(resp["values"]["velocities"][0], 0.0)

    def test_malformed_function_rejected_cleanly(self):
        with Client() as client:
            resp = client.call_service("/arm_sim/integration_step", {
                "function": "t +", "x0": 0.0, "dt": 0.1, "steps": 1, "integrator": "euler",
            })
        self.assertFalse(resp["result"])
        self.assertTrue(resp["status"])

    def test_unknown_integrator_rejected_cleanly(self):
        with Client() as client:
            resp = client.call_service("/arm_sim/integration_step", {
                "function": "t", "x0": 0.0, "dt": 0.1, "steps": 1, "integrator": "leapfrog",
            })
        self.assertFalse(resp["result"])

    def test_non_positive_dt_rejected_cleanly(self):
        with Client() as client:
            resp = client.call_service("/arm_sim/integration_step", {
                "function": "t", "x0": 0.0, "dt": 0.0, "steps": 1, "integrator": "euler",
            })
        self.assertFalse(resp["result"])

    def test_zero_steps_rejected_cleanly(self):
        with Client() as client:
            resp = client.call_service("/arm_sim/integration_step", {
                "function": "t", "x0": 0.0, "dt": 0.1, "steps": 0, "integrator": "euler",
            })
        self.assertFalse(resp["result"])

    def test_runtime_stays_responsive_after_a_bad_request(self):
        with Client() as client:
            bad = client.call_service("/arm_sim/integration_step", {
                "function": "((", "x0": 0.0, "dt": 0.1, "steps": 1, "integrator": "euler",
            })
            self.assertFalse(bad["result"])
            good = client.call_service("/arm_sim/integration_step", {
                "function": "t", "x0": 0.0, "dt": 0.1, "steps": 1, "integrator": "euler",
            })
            self.assertTrue(good["result"])


if __name__ == "__main__":
    unittest.main()
