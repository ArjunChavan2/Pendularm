"""Wire-level integration tests for /arm_sim/set_params, /arm_sim/set_integrator,
and /arm_sim/pause -- the state-storage-only stubs in arm_sim_node.py.
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
    def __init__(self, links: int = 2) -> None:
        self.loop = asyncio.new_event_loop()
        self.registry = Registry()
        arm_sim_node.register(self.registry, links)
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


class TestSetParams(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.server = _ServerThread(links=2)
        cls.server.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.stop()

    def test_empty_request_queries_current_values(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/set_params", {})
        self.assertTrue(resp["result"])
        self.assertEqual(resp["values"], {"gravity": 9.81, "masses": [1.0, 1.0], "lengths": [1.0, 1.0]})

    def test_updates_apply_and_are_echoed(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/set_params", {"gravity": 3.0, "masses": [2.0, 4.0]})
        self.assertTrue(resp["result"])
        self.assertEqual(resp["values"]["gravity"], 3.0)
        self.assertEqual(resp["values"]["masses"], [2.0, 4.0])
        # a field not present in this request should retain its previous value
        self.assertEqual(resp["values"]["lengths"], [1.0, 1.0])

    def test_invalid_field_rejected_without_touching_valid_fields_in_same_request(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/set_params", {"gravity": 5.0, "lengths": [1.0]})  # wrong length
        self.assertFalse(resp["result"])
        # gravity (valid) should still have applied
        self.assertEqual(resp["values"]["gravity"], 5.0)
        # lengths (invalid) should be unchanged from before this call
        self.assertEqual(resp["values"]["lengths"], [1.0, 1.0])

    def test_negative_gravity_rejected(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/set_params", {"gravity": -1.0})
        self.assertFalse(resp["result"])

    def test_non_positive_mass_rejected(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/set_params", {"masses": [1.0, 0.0]})
        self.assertFalse(resp["result"])


class TestSetIntegrator(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.server = _ServerThread(links=2)
        cls.server.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.stop()

    def test_empty_request_queries_current_values(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/set_integrator", {})
        self.assertTrue(resp["result"])
        self.assertEqual(resp["values"], {"method": "euler", "timestep": 0.01})

    def test_valid_update_applies_and_is_echoed(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/set_integrator", {"method": "rk4", "timestep": 0.05})
        self.assertTrue(resp["result"])
        self.assertEqual(resp["values"], {"method": "rk4", "timestep": 0.05})

    def test_unrecognized_method_rejected(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/set_integrator", {"method": "leapfrog"})
        self.assertFalse(resp["result"])

    def test_non_positive_timestep_rejected(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/set_integrator", {"timestep": 0.0})
        self.assertFalse(resp["result"])


class TestPause(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.server = _ServerThread(links=2)
        cls.server.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.stop()

    def test_starts_unpaused(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/pause", {})
        self.assertTrue(resp["result"])
        self.assertEqual(resp["values"], {"data": False})

    def test_pause_and_resume(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/pause", {"data": True})
            self.assertTrue(resp["result"])
            self.assertEqual(resp["values"], {"data": True})

            resp = c.call_service("/arm_sim/pause", {})  # query without changing
            self.assertEqual(resp["values"], {"data": True})

            resp = c.call_service("/arm_sim/pause", {"data": False})
            self.assertEqual(resp["values"], {"data": False})

    def test_non_bool_data_rejected(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/pause", {"data": "yes"})
        self.assertFalse(resp["result"])


class TestThreeLinkArm(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.server = _ServerThread(links=3)
        cls.server.start()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.stop()

    def test_defaults_have_three_entries(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/set_params", {})
        self.assertEqual(resp["values"]["masses"], [1.0, 1.0, 1.0])
        self.assertEqual(resp["values"]["lengths"], [1.0, 1.0, 1.0])

    def test_two_link_length_array_rejected_for_three_link_arm(self):
        with Client() as c:
            resp = c.call_service("/arm_sim/set_params", {"lengths": [1.0, 2.0]})
        self.assertFalse(resp["result"])


if __name__ == "__main__":
    unittest.main()
