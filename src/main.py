"""Project 2 runtime entry point (`make run`).

Currently wires the Registry, TCP gateway, and the /arm_sim/integration_step
checkpoint service. The live n-link arm (dynamics, PID, IK nodes) will be
added here once arm_dynamics.py, pid.py, and kinematics.py exist.
"""
from __future__ import annotations

import asyncio
import os
import signal

import arm_sim_node
from gateway import Gateway, log
from registry import Registry


async def run() -> None:
    links_str = os.environ.get("ARM_SIM_LINKS", "2")
    if links_str not in ("2", "3"):
        log(f"ARM_SIM_LINKS={links_str!r} is not '2' or '3'; defaulting to 2")
        links_str = "2"
    links = int(links_str)
    log(f"ARM_SIM_LINKS={links}")

    registry = Registry()
    arm_sim_node.register(registry, links)

    gateway = Gateway(registry)
    await gateway.start()

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    def _request_stop() -> None:
        log("shutting down")
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _request_stop)

    await stop_event.wait()
    await gateway.stop()


def main() -> None:
    asyncio.run(run())


if __name__ == "__main__":
    main()
