"""Live Pendularm visualizer on localhost: click a target and watch the arm move there.

Dev tool only -- not part of the submission (lives outside src/ and tests/).
Browsers can't open the simulator's raw TCP socket, so this is a small bridge:

    browser  <-- HTTP (page, Server-Sent Events, POST /call) -->  this server
    this server  <-- newline-delimited JSON over TCP -->  simulator (127.0.0.1:9095)

What you see is the real runtime: /ik_action/send_goal -> /ik/solve ->
/joint_trajectory -> PID -> forward dynamics -> integrator -> /joint_states.

Usage (two terminals, from the repo root):
    ARM_SIM_LINKS=3 make run          # or 2
    python3 tools/visualizer.py       # then open http://localhost:8367

Stdlib only.
"""
from __future__ import annotations

import json
import queue
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

SIM_HOST, SIM_PORT = "127.0.0.1", 9095
WEB_PORT = 8367
TOPICS = ("/joint_states", "/ik_action/feedback", "/ik_action/result")
PAGE = Path(__file__).with_name("visualizer.html")


class SimBridge:
    """One upstream TCP connection to the simulator, shared by every browser tab.

    A reader thread fans publishes out to per-tab queues and routes
    service_responses back to the HTTP request waiting on them. Reconnects
    automatically if the simulator restarts.
    """

    def __init__(self) -> None:
        self.sock: socket.socket | None = None
        self._send_lock = threading.Lock()
        self._subscribers: set[queue.Queue] = set()
        self._subs_lock = threading.Lock()
        self._pending: dict[str, queue.Queue] = {}
        self._next_id = 0
        threading.Thread(target=self._run, daemon=True).start()

    @property
    def connected(self) -> bool:
        return self.sock is not None

    def _run(self) -> None:
        while True:
            try:
                sock = socket.create_connection((SIM_HOST, SIM_PORT), timeout=2.0)
                sock.settimeout(None)
            except OSError:
                time.sleep(1.0)
                continue
            self.sock = sock
            self._broadcast({"op": "bridge", "connected": True})
            try:
                for topic in TOPICS:
                    self._send({"op": "subscribe", "topic": topic, "type": "std_msgs/Any", "id": f"sub-{topic}"})
                self._read(sock)
            except OSError:
                pass
            self.sock = None
            self._broadcast({"op": "bridge", "connected": False})
            time.sleep(1.0)

    def _read(self, sock: socket.socket) -> None:
        buf = b""
        while True:
            chunk = sock.recv(65536)
            if not chunk:
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    msg = json.loads(line.decode())
                except ValueError:
                    continue
                if msg.get("op") == "publish":
                    self._broadcast(msg)
                elif msg.get("op") == "service_response":
                    waiter = self._pending.pop(msg.get("id"), None)
                    if waiter:
                        waiter.put(msg)

    def _send(self, msg: dict) -> None:
        with self._send_lock:
            if self.sock is None:
                raise OSError("simulator not connected")
            self.sock.sendall(json.dumps(msg).encode() + b"\n")

    def _broadcast(self, msg: dict) -> None:
        with self._subs_lock:
            for q in self._subscribers:
                q.put(msg)

    def subscribe(self) -> queue.Queue:
        q: queue.Queue = queue.Queue(maxsize=1000)
        with self._subs_lock:
            self._subscribers.add(q)
        return q

    def unsubscribe(self, q: queue.Queue) -> None:
        with self._subs_lock:
            self._subscribers.discard(q)

    def call(self, service: str, args: dict, timeout: float = 5.0) -> dict:
        self._next_id += 1
        cid = f"web-{self._next_id}"
        waiter: queue.Queue = queue.Queue()
        self._pending[cid] = waiter
        try:
            self._send({"op": "call_service", "service": service, "id": cid, "args": args})
            return waiter.get(timeout=timeout)
        except (OSError, queue.Empty) as e:
            self._pending.pop(cid, None)
            return {"result": False, "status": f"bridge: {e or 'timeout'}", "values": {}}


BRIDGE = SimBridge()
ALLOWED_SERVICES = {"/ik_action/send_goal", "/ik_action/cancel_goal", "/pid_controller/enable",
                    "/pid_controller/set_gains", "/arm_sim/reset", "/arm_sim/set_params",
                    "/arm_sim/set_integrator", "/ik/solve"}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args) -> None:  # keep the terminal quiet
        pass

    def do_GET(self) -> None:
        if self.path in ("/", "/index.html"):
            body = PAGE.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        elif self.path == "/events":
            self._stream_events()
        else:
            self.send_error(404)

    def _stream_events(self) -> None:
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        q = BRIDGE.subscribe()
        try:
            self._write_event({"op": "bridge", "connected": BRIDGE.connected})
            while True:
                try:
                    self._write_event(q.get(timeout=15.0))
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
        except OSError:  # browser tab closed
            pass
        finally:
            BRIDGE.unsubscribe(q)

    def _write_event(self, msg: dict) -> None:
        self.wfile.write(b"data: " + json.dumps(msg).encode() + b"\n\n")
        self.wfile.flush()

    def do_POST(self) -> None:
        if self.path != "/call":
            self.send_error(404)
            return
        try:
            req = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))))
            service, args = req["service"], req.get("args", {})
        except (ValueError, KeyError, TypeError):
            self.send_error(400)
            return
        if service not in ALLOWED_SERVICES:
            self.send_error(403)
            return
        body = json.dumps(BRIDGE.call(service, args)).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


if __name__ == "__main__":
    server = ThreadingHTTPServer(("127.0.0.1", WEB_PORT), Handler)
    server.daemon_threads = True
    print(f"Pendularm visualizer: http://localhost:{WEB_PORT}  (simulator expected on {SIM_HOST}:{SIM_PORT})")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
