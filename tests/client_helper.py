"""Small reusable TCP/JSON test client for the rosbridge-style gateway."""
from __future__ import annotations

import itertools
import json
import socket
import time
from typing import Any

HOST = "127.0.0.1"
PORT = 9095


class Client:
    def __init__(self, host: str = HOST, port: int = PORT, timeout: float = 5.0) -> None:
        self._sock = socket.create_connection((host, port), timeout=timeout)
        self._buf = b""
        self._ids = itertools.count(1)

    def close(self) -> None:
        self._sock.close()

    def __enter__(self) -> "Client":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    def send(self, message: dict) -> None:
        self._sock.sendall(json.dumps(message, separators=(",", ":")).encode("utf-8") + b"\n")

    def recv_line(self, timeout: float = 5.0) -> dict:
        self._sock.settimeout(timeout)
        while b"\n" not in self._buf:
            chunk = self._sock.recv(65536)
            if not chunk:
                raise ConnectionError("connection closed")
            self._buf += chunk
        line, self._buf = self._buf.split(b"\n", 1)
        return json.loads(line.decode("utf-8"))

    def recv_matching(self, predicate, timeout: float = 5.0, max_messages: int = 50) -> dict:
        """Read lines until one satisfies predicate(msg), skipping unrelated ones."""
        deadline = time.monotonic() + timeout
        for _ in range(max_messages):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            msg = self.recv_line(timeout=remaining)
            if predicate(msg):
                return msg
        raise TimeoutError("no matching message received")

    def call_service(self, service: str, args: Any, timeout: float = 5.0) -> dict:
        call_id = f"call-{next(self._ids)}"
        self.send({"op": "call_service", "service": service, "id": call_id, "args": args})
        return self.recv_matching(
            lambda m: m.get("op") == "service_response" and m.get("id") == call_id, timeout=timeout,
        )

    def subscribe(self, topic: str) -> None:
        sub_id = f"sub-{next(self._ids)}"
        self.send({"op": "subscribe", "topic": topic, "type": "std_msgs/Any", "id": sub_id})

    def advertise(self, topic: str) -> None:
        self.send({"op": "advertise", "topic": topic, "type": "std_msgs/Any"})

    def unadvertise(self, topic: str) -> None:
        self.send({"op": "unadvertise", "topic": topic})

    def publish(self, topic: str, msg: Any) -> None:
        self.send({"op": "publish", "topic": topic, "msg": msg})
