"""Rosbridge-style TCP/JSON gateway.

Implements the external protocol documented in spec/ROSBRIDGE_PROTOCOL.md:
newline-delimited JSON over TCP on 127.0.0.1:9095, multiple concurrent
clients, topics (advertise/publish/subscribe/unadvertise/unsubscribe) and
services (advertise_service/call_service/service_response/unadvertise_service).

This module only speaks the wire protocol and delegates all pub/sub and
service-provider bookkeeping to registry.Registry. Service calls routed to a
real TCP provider connection are tracked here (in ``_pending``) because only
this module has access to asyncio timers/event loop for the required bounded
timeout on service calls.
"""
from __future__ import annotations

import asyncio
import json
import sys
from dataclasses import dataclass, field
from typing import Any

from registry import Registry

HOST = "127.0.0.1"
PORT = 9095

SERVICE_CALL_TIMEOUT_S = 5.0


def log(*args: Any) -> None:
    """Diagnostic output, kept off the TCP/JSON protocol stream (spec requirement)."""
    print(*args, file=sys.stderr, flush=True)


class Connection:
    """Wraps one TCP client's writer with the send(dict) API Registry expects."""

    def __init__(self, writer: asyncio.StreamWriter) -> None:
        self._writer = writer
        self.advertised_services: set[str] = set()

    def send(self, message: dict) -> None:
        try:
            line = json.dumps(message, separators=(",", ":")) + "\n"
            self._writer.write(line.encode("utf-8"))
        except (ConnectionError, RuntimeError):
            pass  # best-effort delivery; a dead peer is cleaned up by its read loop


@dataclass
class _PendingCall:
    caller_conn: Connection
    caller_id: Any
    provider_conn: Connection
    service: str
    timer: asyncio.TimerHandle = field(default=None)


class Gateway:
    def __init__(self, registry: Registry) -> None:
        self.registry = registry
        self._pending: dict[str, _PendingCall] = {}
        self._server: asyncio.base_events.Server | None = None

    async def start(self) -> None:
        self._server = await asyncio.start_server(self._handle_client, HOST, PORT)
        log(f"gateway listening on {HOST}:{PORT}")

    async def stop(self) -> None:
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()

    # -- per-connection loop -------------------------------------------------

    async def _handle_client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        conn = Connection(writer)
        try:
            while True:
                line = await reader.readline()
                if not line:
                    break
                line = line.strip()
                if not line:
                    continue
                try:
                    message = json.loads(line.decode("utf-8"))
                except (UnicodeDecodeError, json.JSONDecodeError):
                    conn.send({"op": "status", "level": "error", "msg": "malformed JSON"})
                    continue
                if not isinstance(message, dict) or "op" not in message:
                    conn.send({"op": "status", "level": "error", "msg": "missing op"})
                    continue
                self._dispatch(conn, message)
        except (ConnectionError, asyncio.IncompleteReadError):
            pass
        finally:
            self._cleanup_connection(conn)
            try:
                writer.close()
            except Exception:
                pass

    def _cleanup_connection(self, conn: Connection) -> None:
        self.registry.remove_connection(conn)
        # Any calls still pending against a provider that just disconnected
        # must resolve with a clean failure, not hang.
        for call_id, pending in list(self._pending.items()):
            if pending.provider_conn is conn:
                self._resolve_pending(call_id, result=False, values={}, status="provider disconnected")

    # -- op dispatch -------------------------------------------------------

    def _dispatch(self, conn: Connection, message: dict) -> None:
        op = message.get("op")
        req_id = message.get("id")
        try:
            if op == "advertise":
                self.registry.advertise(conn, message["topic"])
            elif op == "unadvertise":
                self.registry.unadvertise(conn, message["topic"])
            elif op == "subscribe":
                self.registry.subscribe(conn, message["topic"])
                conn.send({"op": "status", "level": "info", "msg": f"subscribed to {message['topic']}", "id": req_id})
            elif op == "unsubscribe":
                self.registry.unsubscribe(conn, message["topic"])
            elif op == "publish":
                topic = message["topic"]
                # A connection may publish on a topic only while it currently
                # holds an advertisement there; unadvertise immediately
                # revokes it. Silently dropped, matching this project's other
                # silent-reject cases (e.g. a malformed /map) -- publish has
                # no acknowledgement in this protocol either way.
                if self.registry.is_advertised(conn, topic):
                    self.registry.publish(topic, message.get("msg"))
            elif op == "advertise_service":
                service = message["service"]
                self.registry.advertise_service(conn, service)
                conn.advertised_services.add(service)
                conn.send({"op": "status", "level": "info", "msg": f"providing {service}", "id": req_id})
            elif op == "unadvertise_service":
                self.registry.unadvertise_service(conn, message["service"])
                conn.advertised_services.discard(message["service"])
            elif op == "call_service":
                self._handle_call_service(conn, message)
            elif op == "service_response":
                self._handle_service_response(conn, message)
            else:
                conn.send({"op": "status", "level": "error", "msg": f"unknown op {op!r}", "id": req_id})
        except KeyError as exc:
            conn.send({"op": "status", "level": "error", "msg": f"missing field {exc}", "id": req_id})
        except Exception as exc:  # malformed requests must not crash the gateway (spec requirement)
            log(f"error handling op {op!r}: {exc!r}")
            conn.send({"op": "status", "level": "error", "msg": f"internal error: {exc}", "id": req_id})

    # -- services ------------------------------------------------------------

    def _handle_call_service(self, caller_conn: Connection, message: dict) -> None:
        service = message["service"]
        caller_id = message["id"]
        args = message.get("args")

        # Prefer an in-process handler (used by this project's own nodes):
        # answered synchronously, no wire round-trip or timeout needed.
        if self.registry.has_in_process_handler(service):
            try:
                result, values, status = self.registry.call_service_sync(service, args)
            except Exception as exc:  # a buggy/unimplemented handler must not crash the gateway
                log(f"service handler for {service} raised: {exc!r}")
                result, values, status = False, {}, f"internal error: {exc}"
            caller_conn.send({
                "op": "service_response", "service": service, "id": caller_id,
                "values": values, "result": result, "status": status,
            })
            return

        provider_conn = self.registry.get_provider(service)
        if provider_conn is None:
            caller_conn.send({
                "op": "service_response", "service": service, "id": caller_id,
                "values": {}, "result": False, "status": "no provider",
            })
            return

        provider_call_id = self.registry.next_call_id()
        loop = asyncio.get_event_loop()
        timer = loop.call_later(
            SERVICE_CALL_TIMEOUT_S, self._resolve_pending, provider_call_id, False, {}, "timeout",
        )
        self._pending[provider_call_id] = _PendingCall(caller_conn, caller_id, provider_conn, service, timer)
        provider_conn.send({"op": "call_service", "service": service, "id": provider_call_id, "args": args})

    def _handle_service_response(self, provider_conn: Connection, message: dict) -> None:
        provider_call_id = message.get("id")
        pending = self._pending.get(provider_call_id)
        if pending is None or pending.provider_conn is not provider_conn:
            # Unknown/stale response id: ignored per spec (must never complete
            # another caller's request).
            return
        self._resolve_pending(
            provider_call_id,
            result=bool(message.get("result")),
            values=message.get("values", {}),
            status=message.get("status", ""),
        )

    def _resolve_pending(self, provider_call_id: str, result: bool, values: Any, status: str) -> None:
        pending = self._pending.pop(provider_call_id, None)
        if pending is None:
            return
        if pending.timer is not None:
            pending.timer.cancel()
        pending.caller_conn.send({
            "op": "service_response", "service": pending.service, "id": pending.caller_id,
            "values": values, "result": result, "status": status,
        })
