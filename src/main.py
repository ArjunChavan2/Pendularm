"""Project 2 runtime entry point (`make run`).

Wires the Registry, TCP gateway, the /arm_sim/integration_step checkpoint
service, the /arm_sim/* and /pid_controller/* services (all registered by
arm_sim_node.register), the /ik/*, /ik_action/*, and /ik_trial/* services
(ik_node.register, after arm_sim_node since /ik/solve queries
/arm_sim/set_params), and the runtime's background tasks
(arm_sim_node.physics_loop, arm_sim_node.publish_loop,
ik_node.trial_status_loop), running concurrently with the gateway's own
per-connection coroutines on the same asyncio event loop.
"""
from __future__ import annotations

import asyncio
import os
import signal

import arm_sim_node
import ik_node
from gateway import Gateway, log
from registry import Registry


async def run() -> None:
    links_str = os.environ.get("ARM_SIM_LINKS", "2")
    # Spec: "2" or "3", defaulting to 2. A 1-link arm is a dev-only extra for
    # tools/visualizer.py, never enabled unless explicitly opted into.
    allowed = ("1", "2", "3") if os.environ.get("PENDULARM_DEV_ALLOW_1LINK") == "1" else ("2", "3")
    if links_str not in allowed:
        log(f"ARM_SIM_LINKS={links_str!r} is not '2' or '3'; defaulting to 2")
        links_str = "2"
    links = int(links_str)
    log(f"ARM_SIM_LINKS={links}")

    registry = Registry()
    state = arm_sim_node.register(registry, links)
    _, trial = ik_node.register(registry)

    gateway = Gateway(registry)
    await gateway.start()

    physics_task = asyncio.create_task(arm_sim_node.physics_loop(state))
    publish_task = asyncio.create_task(arm_sim_node.publish_loop(registry, state))
    trial_task = asyncio.create_task(ik_node.trial_status_loop(trial))

    loop = asyncio.get_running_loop()
    stop_event = asyncio.Event()

    def _request_stop() -> None:
        log("shutting down")
        stop_event.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, _request_stop)

    await stop_event.wait()
    await gateway.stop()

    tasks = (physics_task, publish_task, trial_task)
    for task in tasks:
        task.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


def main() -> None:
    asyncio.run(run())


if __name__ == "__main__":
    main()
