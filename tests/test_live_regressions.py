"""Live-process regressions for the pre-resubmission audit findings.

Each test launches its own real runtime (`python3 src/main.py`, 3 links) and
talks to it over the real TCP/JSON gateway, exactly like the autograder:
  - SIGTERM must exit promptly even while a client is still connected
    (Python >= 3.12.1's Server.wait_closed() waits on open connections).
  - /joint_trajectory publishes are honored without a prior `advertise`.
  - The physics loop keeps sim time locked to wall time.
  - /joint_states' stamp nanosec stays in [0, 1e9).
  - A line over asyncio's default 64 KiB limit (but under the protocol's
    4 MiB) doesn't drop the connection.
  - With default gains/timestep, enabling PID and commanding a setpoint
    converges quickly, with steady-state effort ~= G(q).
"""
from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

import arm_dynamics  # noqa: E402
from test_live_process_e2e import HOST, PORT, REPO_ROOT, _RawClient, _wait_for_port  # noqa: E402


def _start(links: int = 3) -> subprocess.Popen:
    env = dict(os.environ, ARM_SIM_LINKS=str(links))
    proc = subprocess.Popen([sys.executable, "src/main.py"], cwd=REPO_ROOT, env=env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if not _wait_for_port(HOST, PORT, timeout=10.0):
        proc.kill()
        raise RuntimeError("runtime did not start listening within 10s")
    return proc


def _stop(proc: subprocess.Popen) -> None:
    if proc.poll() is None:
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)


@unittest.skipUnless(os.environ.get("PENDULARM_SKIP_E2E") != "1", "PENDULARM_SKIP_E2E=1")
class TestLiveRegressions(unittest.TestCase):
    def setUp(self) -> None:
        self.proc = _start(3)
        self.c = _RawClient()

    def tearDown(self) -> None:
        try:
            self.c.close()
        except OSError:
            pass
        _stop(self.proc)

    def _states(self, seconds: float) -> list[dict]:
        self.c.subscribe("/joint_states")
        out, t0 = [], time.monotonic()
        while time.monotonic() - t0 < seconds:
            out.append(self.c.recv_publish("/joint_states")["msg"])
        self.c.send({"op": "unsubscribe", "topic": "/joint_states"})
        return out

    def test_sigterm_exits_promptly_with_client_still_connected(self):
        self.c.subscribe("/joint_states")
        self.c.recv_publish("/joint_states")
        t0 = time.monotonic()
        self.proc.send_signal(signal.SIGTERM)
        self.proc.wait(timeout=5)  # raises TimeoutExpired (-> test error) if it hangs
        self.assertLess(time.monotonic() - t0, 3.0)

    def test_sim_time_tracks_wall_time_and_stamp_is_valid(self):
        sim = lambda m: m["header"]["stamp"]["sec"] + m["header"]["stamp"]["nanosec"] * 1e-9
        self.c.subscribe("/joint_states")
        first = self.c.recv_publish("/joint_states")["msg"]
        w0, count = time.monotonic(), 0
        while time.monotonic() - w0 < 4.0:
            last = self.c.recv_publish("/joint_states")["msg"]
            self.assertTrue(0 <= last["header"]["stamp"]["nanosec"] < 1_000_000_000, last["header"])
            count += 1
        ratio = (sim(last) - sim(first)) / (time.monotonic() - w0)
        self.assertGreater(count, 100)
        self.assertGreater(ratio, 0.95, f"sim/wall ratio {ratio:.3f}")

    def test_trajectory_without_advertise_converges_with_default_gains(self):
        setpoint = [0.6, -0.4, 0.3]
        self.assertTrue(self.c.call_service("/pid_controller/enable", {"data": True})["result"])
        self.c.send({"op": "publish", "topic": "/joint_trajectory", "msg": {
            "joint_names": ["joint1", "joint2", "joint3"],
            "points": [{"positions": setpoint, "velocities": [0, 0, 0]}]}})
        t0 = time.monotonic()
        settled_at = None
        self.c.subscribe("/joint_states")
        while time.monotonic() - t0 < 8.0:
            m = self.c.recv_publish("/joint_states")["msg"]
            err = max(abs(a - b) for a, b in zip(m["position"], setpoint))
            if err < 0.05:
                settled_at = settled_at or time.monotonic()
                if time.monotonic() - settled_at > 1.0:
                    break
            else:
                settled_at = None
        self.assertIsNotNone(settled_at, f"did not converge within 8 s wall (last position {m['position']})")
        # Hold a little longer, then steady-state effort should match G(q).
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            m = self.c.recv_publish("/joint_states")["msg"]
            if max(abs(v) for v in m["velocity"]) < 0.01:
                break
        p = self.c.call_service("/arm_sim/set_params", {})["values"]
        G = arm_dynamics.G(m["position"], p["gravity"], p["masses"], p["lengths"])
        for eff, g in zip(m["effort"], G):
            self.assertAlmostEqual(eff, g, delta=0.05 * max(1.0, abs(g)))

    def test_long_line_does_not_drop_connection(self):
        big = {"joint_names": ["joint1", "joint2", "joint3"],
               "points": [{"positions": [0, 0, 0], "velocities": [0, 0, 0]}] * 12000}
        line = json.dumps({"op": "publish", "topic": "/joint_trajectory", "msg": big})
        self.assertGreater(len(line), 64 * 1024)
        self.c.send(json.loads(line))
        resp = self.c.call_service("/arm_sim/set_params", {})
        self.assertTrue(resp["result"])


if __name__ == "__main__":
    unittest.main()
