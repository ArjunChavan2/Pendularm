"""Genuine end-to-end regression test: launches `make run` as a REAL separate
OS subprocess (not an in-process asyncio thread, unlike the other
test_*.py modules in this directory), talks to it over a real TCP socket
from this separate test process, and verifies clean SIGTERM shutdown.

This exists specifically to catch anything that only manifests through the
actual `make build && make run` path (per spec/PROJECT2_PENDULARM.md's
grading description: "runs the Make targets ... launches your runtime, and
connects externally ... using the documented TCP/JSON protocol"), and to
independently re-confirm (not just trust) agent-notes/AUDIT.md's findings
about expr.py's `^` operator and trig functions are actually fixed, against
a real running gateway rather than expr.py in isolation.

Slower and heavier than the rest of the suite (spawns a real subprocess,
real TCP), so kept in its own module.
"""
from __future__ import annotations

import json
import math
import os
import signal
import socket
import subprocess
import sys
import time
import unittest

REPO_ROOT = os.path.join(os.path.dirname(__file__), "..")
HOST, PORT = "127.0.0.1", 9095


def _wait_for_port(host: str, port: int, timeout: float = 10.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            s = socket.create_connection((host, port), timeout=0.5)
            s.close()
            return True
        except OSError:
            time.sleep(0.1)
    return False


class _RawClient:
    """Deliberately independent of tests/client_helper.py -- this module
    wants its own minimal, from-scratch wire implementation so a bug shared
    between arm_sim_node.py and client_helper.py can't hide from both."""

    def __init__(self, host: str = HOST, port: int = PORT, timeout: float = 5.0) -> None:
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.buf = b""
        self._id = 0

    def close(self) -> None:
        self.sock.close()

    def send(self, msg: dict) -> None:
        self.sock.sendall(json.dumps(msg).encode() + b"\n")

    def recv_json(self, timeout: float = 5.0) -> dict:
        self.sock.settimeout(timeout)
        while b"\n" not in self.buf:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError("connection closed")
            self.buf += chunk
        line, self.buf = self.buf.split(b"\n", 1)
        return json.loads(line.decode())

    def call_service(self, service: str, args, timeout: float = 5.0) -> dict:
        self._id += 1
        cid = f"e2e-{self._id}"
        self.send({"op": "call_service", "service": service, "id": cid, "args": args})
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"no service_response for {service} id={cid} within {timeout}s")
            msg = self.recv_json(timeout=remaining)
            if msg.get("op") == "service_response" and msg.get("id") == cid:
                return msg

    def subscribe(self, topic: str) -> None:
        self._id += 1
        self.send({"op": "subscribe", "topic": topic, "type": "std_msgs/Any", "id": f"e2e-sub-{self._id}"})

    def recv_publish(self, topic: str, timeout: float = 5.0) -> dict:
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(f"no publish on {topic} within {timeout}s")
            msg = self.recv_json(timeout=remaining)
            if msg.get("op") == "publish" and msg.get("topic") == topic:
                return msg


@unittest.skipUnless(
    os.environ.get("PENDULARM_SKIP_E2E") != "1",
    "set PENDULARM_SKIP_E2E=1 to skip the real make-run subprocess test",
)
class TestLiveMakeRunProcess(unittest.TestCase):
    """Spawns a real `make run` subprocess against a real 3-link runtime."""

    @classmethod
    def setUpClass(cls) -> None:
        build = subprocess.run(["make", "build"], cwd=REPO_ROOT, capture_output=True, text=True)
        if build.returncode != 0:
            raise RuntimeError(f"make build failed: {build.stderr}")

        env = dict(os.environ)
        env["ARM_SIM_LINKS"] = "3"
        cls.proc = subprocess.Popen(
            ["make", "run"], cwd=REPO_ROOT, env=env,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        if not _wait_for_port(HOST, PORT, timeout=10.0):
            cls.proc.kill()
            raise RuntimeError("gateway did not start listening within 10s of `make run`")

    @classmethod
    def tearDownClass(cls) -> None:
        if cls.proc.poll() is None:
            cls.proc.send_signal(signal.SIGTERM)
            try:
                cls.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                cls.proc.kill()
                cls.proc.wait(timeout=5)

    def test_negative_base_fractional_power_does_not_hang_over_real_gateway(self):
        # AUDIT.md Finding 1: (-4)^0.5 used to hang the whole request (complex
        # value slipping past JSON serialization). Confirm, over a real
        # subprocess and real socket, that this now returns promptly.
        c = _RawClient()
        try:
            t0 = time.monotonic()
            resp = c.call_service("/arm_sim/integration_step", {
                "function": "(-4)^0.5", "x0": 0.0, "xdot0": 0.0, "dt": 0.1, "steps": 3,
                "integrator": "euler",
            }, timeout=3.0)
            elapsed = time.monotonic() - t0
            self.assertLess(elapsed, 2.0, "response took too long -- may indicate a hang")
            self.assertTrue(resp["result"])
            self.assertTrue(any(isinstance(v, float) and math.isnan(v) for v in resp["values"]["positions"][1:]))
        finally:
            c.close()

    def test_zero_to_negative_power_and_overflow_do_not_reject_over_real_gateway(self):
        # AUDIT.md Finding 2.
        c = _RawClient()
        try:
            resp = c.call_service("/arm_sim/integration_step", {
                "function": "t^-1", "x0": 0.0, "xdot0": 0.0, "dt": 1.0, "steps": 1, "integrator": "euler",
            })
            self.assertTrue(resp["result"], resp)

            resp = c.call_service("/arm_sim/integration_step", {
                "function": "t^1000", "x0": 0.0, "xdot0": 0.0, "dt": 500.0, "steps": 1, "integrator": "euler",
            })
            self.assertTrue(resp["result"], resp)
        finally:
            c.close()

    def test_trig_of_infinite_input_does_not_reject_over_real_gateway(self):
        # AUDIT.md Finding 3.
        c = _RawClient()
        try:
            resp = c.call_service("/arm_sim/integration_step", {
                "function": "sin(1/t)", "x0": 0.0, "xdot0": 0.0, "dt": 1.0, "steps": 1, "integrator": "euler",
            })
            self.assertTrue(resp["result"], resp)
        finally:
            c.close()

    def test_full_rejection_set_and_responsiveness_after_each(self):
        c = _RawClient()
        try:
            cases = {
                "unparseable function": {"function": "1 2", "x0": 0.0, "dt": 0.1, "steps": 1, "integrator": "euler"},
                "unknown integrator": {"function": "t", "x0": 0.0, "dt": 0.1, "steps": 1, "integrator": "bogus"},
                "dt<=0": {"function": "t", "x0": 0.0, "dt": -1.0, "steps": 1, "integrator": "euler"},
                "steps==0": {"function": "t", "x0": 0.0, "dt": 0.1, "steps": 0, "integrator": "euler"},
            }
            for label, args in cases.items():
                resp = c.call_service("/arm_sim/integration_step", args)
                self.assertFalse(resp["result"], f"{label} should be rejected: {resp}")
                self.assertTrue(resp["status"], f"{label} should carry a non-empty status: {resp}")
                good = c.call_service("/arm_sim/integration_step", {
                    "function": "t", "x0": 0.0, "dt": 0.1, "steps": 1, "integrator": "euler",
                })
                self.assertTrue(good["result"], f"runtime not responsive after {label}: {good}")
        finally:
            c.close()

    def test_response_shape_invariants_all_integrators_real_gateway(self):
        c = _RawClient()
        try:
            for integ in ("euler", "midpoint", "verlet", "rk4"):
                resp = c.call_service("/arm_sim/integration_step", {
                    "function": "t^2 - 1", "x0": 2.5, "xdot0": -1.5, "dt": 0.2, "steps": 4, "integrator": integ,
                })
                self.assertTrue(resp["result"], resp)
                v = resp["values"]
                self.assertEqual(v["times"][0], 0.0)
                self.assertEqual(v["positions"][0], 2.5)
                self.assertEqual(v["velocities"][0], -1.5)
                self.assertEqual(len(v["times"]), 5)
                self.assertEqual(len(v["positions"]), 5)
                self.assertEqual(len(v["velocities"]), 5)
        finally:
            c.close()

    def test_state_services_on_real_three_link_runtime(self):
        c = _RawClient()
        try:
            resp = c.call_service("/arm_sim/set_params", {})
            self.assertTrue(resp["result"])
            self.assertEqual(len(resp["values"]["masses"]), 3)
            self.assertEqual(len(resp["values"]["lengths"]), 3)

            # Per-field independence: a valid gravity update alongside an
            # invalid (wrong-length) masses array in the SAME request --
            # gravity must still apply.
            resp = c.call_service("/arm_sim/set_params", {"gravity": 2.0, "masses": [1.0, 2.0]})
            self.assertFalse(resp["result"])
            self.assertEqual(resp["values"]["gravity"], 2.0)

            resp = c.call_service("/arm_sim/set_integrator", {"method": "rk4", "timestep": 0.02})
            self.assertTrue(resp["result"])
            self.assertEqual(resp["values"], {"method": "rk4", "timestep": 0.02})

            resp = c.call_service("/arm_sim/pause", {"data": True})
            self.assertTrue(resp["result"])
            self.assertTrue(resp["values"]["data"])
            # restore for any later test ordering
            c.call_service("/arm_sim/pause", {"data": False})
        finally:
            c.close()

    def test_joint_states_publishes_and_runtime_stays_responsive_with_unimplemented_arm_dynamics(self):
        # src/arm_dynamics.py ships as an intentional stub (raises
        # NotImplementedError every physics tick) until its owner
        # hand-implements it -- confirms physics_loop's per-tick try/except
        # (see arm_sim_node.py) keeps the real subprocess's gateway, other
        # services, and /joint_states publication fully working despite
        # that, rather than the live-arm addition silently regressing the
        # already-passing checkpoint behavior or crashing the process.
        c = _RawClient()
        try:
            c.subscribe("/joint_states")
            # ack for the subscribe request itself, then several published
            # /joint_states messages.
            first = c.recv_publish("/joint_states", timeout=3.0)
            second = c.recv_publish("/joint_states", timeout=3.0)

            for msg in (first, second):
                body = msg["msg"]
                self.assertEqual(body["name"], ["joint1", "joint2", "joint3"])
                self.assertEqual(len(body["position"]), 3)
                self.assertEqual(len(body["velocity"]), 3)
                self.assertEqual(len(body["effort"]), 3)
                # arm_dynamics.forward_dynamics raises every tick, so the
                # physics loop never successfully advances -- position and
                # velocity stay frozen at the reset pose, not NaN/garbage
                # and not silently missing.
                self.assertEqual(body["position"], [0.0, 0.0, 0.0])
                self.assertEqual(body["velocity"], [0.0, 0.0, 0.0])

            # The rest of the runtime must stay fully responsive throughout.
            resp = c.call_service("/arm_sim/integration_step", {
                "function": "t", "x0": 0.0, "xdot0": 0.0, "dt": 0.1, "steps": 1, "integrator": "euler",
            })
            self.assertTrue(resp["result"], resp)

            resp = c.call_service("/arm_sim/set_params", {})
            self.assertTrue(resp["result"], resp)
        finally:
            c.close()

    def test_zz_clean_sigterm_shutdown_of_underlying_python_process(self):
        # Named "zz_" so unittest's alphabetical method ordering runs this
        # LAST within the class -- it kills the shared server subprocess, so
        # every other test in this class must run before it.
        # Sending SIGTERM to the `make run` process itself is confounded by
        # `make`'s own signal disposition (it terminates itself with the
        # same signal after forwarding it, a standard POSIX/Make
        # convention -- confirmed separately during manual verification, not
        # exercised here to keep this test deterministic). This test instead
        # confirms the actual production code's signal handling is clean by
        # sending SIGTERM directly to the python3 src/main.py child.
        children = subprocess.run(
            ["pgrep", "-P", str(self.proc.pid)], capture_output=True, text=True,
        ).stdout.split()
        self.assertTrue(children, "expected `make run` to have spawned a python3 child process")
        child_pid = int(children[0])

        os.kill(child_pid, signal.SIGTERM)
        deadline = time.monotonic() + 5.0
        exited = False
        while time.monotonic() < deadline:
            try:
                os.kill(child_pid, 0)
            except ProcessLookupError:
                exited = True
                break
            time.sleep(0.1)
        self.assertTrue(exited, "python3 src/main.py did not exit within 5s of SIGTERM")

        # port must be released
        time.sleep(0.3)
        released = True
        try:
            s = socket.create_connection((HOST, PORT), timeout=0.5)
            s.close()
            released = False
        except OSError:
            released = True
        self.assertTrue(released, "port 9095 still accepting connections after shutdown")

        # `make run`'s own process may still be alive (it was waiting on the
        # now-dead child); clean it up so tearDownClass's SIGTERM to it is a
        # no-op wait rather than a hang.
        if self.proc.poll() is None:
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass


if __name__ == "__main__":
    unittest.main()
