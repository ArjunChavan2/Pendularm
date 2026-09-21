"""In-process pub/sub + service registry shared by the TCP gateway and internal nodes.

This module has no networking in it. A "connection" is any object with a
``send(message: dict) -> None`` method (a real TCP connection, or a lightweight
in-process stand-in used by internal nodes such as the heap/map/A* services).
Because the gateway and all internal nodes talk to the same Registry instance
through this same API, topic isolation, service routing, and disconnect
cleanup only need to be implemented once.

Intended to run entirely on a single asyncio event loop (no threads), so no
locking is used -- callers must not call across threads.
"""
from __future__ import annotations

import itertools
from typing import Any, Callable, Protocol


class Connection(Protocol):
    def send(self, message: dict) -> None: ...


ServiceHandler = Callable[[Any], Any]
"""A service handler receives ``args`` and returns ``(result: bool, values: dict, status: str)``."""


class Registry:
    def __init__(self) -> None:
        self._subscribers: dict[str, set[Connection]] = {}
        self._publishers: dict[str, set[Connection]] = {}
        self._service_providers: dict[str, Connection] = {}
        # Handlers registered in-process (used by internal nodes instead of
        # advertising a real provider connection over the wire).
        self._service_handlers: dict[str, ServiceHandler] = {}
        self._call_id_counter = itertools.count(1)

    # -- topics -----------------------------------------------------------

    def advertise(self, conn: Connection, topic: str) -> None:
        self._publishers.setdefault(topic, set()).add(conn)

    def unadvertise(self, conn: Connection, topic: str) -> None:
        self._publishers.get(topic, set()).discard(conn)

    def is_advertised(self, conn: Connection, topic: str) -> bool:
        """Whether ``conn`` has an active advertisement on ``topic``.

        Only the external wire ``publish`` op is gated by this (see
        gateway.py) -- internal nodes that call ``publish()`` directly (e.g.
        plan_path_service.py publishing /path) bypass the wire dispatch layer
        entirely and are unaffected.
        """
        return conn in self._publishers.get(topic, set())

    def subscribe(self, conn: Connection, topic: str) -> None:
        self._subscribers.setdefault(topic, set()).add(conn)

    def unsubscribe(self, conn: Connection, topic: str) -> None:
        self._subscribers.get(topic, set()).discard(conn)

    def publish(self, topic: str, msg: Any) -> None:
        message = {"op": "publish", "topic": topic, "msg": msg}
        for sub in list(self._subscribers.get(topic, set())):
            sub.send(message)

    # -- services -----------------------------------------------------------

    def advertise_service(self, conn: Connection, service: str) -> None:
        self._service_providers[service] = conn

    def unadvertise_service(self, conn: Connection, service: str) -> None:
        if self._service_providers.get(service) is conn:
            del self._service_providers[service]

    def register_handler(self, service: str, handler: ServiceHandler) -> None:
        """Register an in-process handler (used by internal nodes)."""
        self._service_handlers[service] = handler

    def call_service_sync(self, service: str, args: Any) -> tuple[bool, Any, str]:
        """Call a service registered via ``register_handler`` directly (no wire round-trip).

        Used internally by nodes (e.g. A* calling /heap_sort logic, or tests)
        that want the result without going through a TCP connection.
        """
        handler = self._service_handlers.get(service)
        if handler is None:
            return False, {}, f"no provider for {service}"
        return handler(args)

    def has_in_process_handler(self, service: str) -> bool:
        return service in self._service_handlers

    def get_provider(self, service: str) -> Connection | None:
        return self._service_providers.get(service)

    def has_provider(self, service: str) -> bool:
        return service in self._service_providers or service in self._service_handlers

    # -- cleanup -----------------------------------------------------------

    def remove_connection(self, conn: Connection) -> None:
        """Withdraw every registration owned by ``conn`` (call on disconnect)."""
        for subs in self._subscribers.values():
            subs.discard(conn)
        for pubs in self._publishers.values():
            pubs.discard(conn)
        for service, provider in list(self._service_providers.items()):
            if provider is conn:
                del self._service_providers[service]

    def next_call_id(self) -> str:
        return f"gw-{next(self._call_id_counter)}"
