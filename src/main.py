"""Project 2 runtime entry point (`make run`).

Wires the Registry, TCP gateway, the /arm_sim/integration_step checkpoint
service and /arm_sim/* state services, and the live n-link arm's two
background tasks (arm_sim_node.physics_loop, arm_sim_node.publish_loop),
running concurrently with the gateway's own per-connection coroutines on the
same asyncio event loop. PID and IK nodes will be added here once pid.py and
kinematics.py exist.
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
    state = arm_sim_node.register(registry, links)

    gateway = Gateway(registry)
    await gateway.start()

    physics_task = asyncio.create_task(arm_sim_node.physics_loop(state))
    publish_task = asyncio.create_task(arm_sim_node.publish_loop(registry, state))

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    def _request_stop() -> None:
        log("shutting down")
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _request_stop)

    await stop_event.wait()
    await gateway.stop()

    for task in (physics_task, publish_task):
        task.cancel()
    await asyncio.gather(physics_task, publish_task, return_exceptions=True)


def main() -> None:
    asyncio.run(run())


if __name__ == "__main__":
    main()
