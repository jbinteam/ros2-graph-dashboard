# Copyright 2026 JB
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""serve — local web view of the static ROS graph on 127.0.0.1:8091.

Endpoints:
  GET /            the rqt_graph-style view (vendored vis-network, offline)
  GET /api/graph   graph JSON, RE-SCANNED on every request so a browser
                   reload always reflects the current source tree — no
                   stale graph.json coordination needed.
  GET /api/ego?id=<element>   signed-distance ego-graph of one element
                   (negative = upstream inputs, positive = downstream
                   outputs) for the focus panel; recomputed per request
                   from a fresh scan like /api/graph.
  GET /api/tap?id=topic:<name>   on-demand live tap of ONE topic (Hz,
                   bandwidth, latest message rendered as thumbnail or field
                   tree). Poll-driven lifecycle: the first poll creates the
                   tap, and a tap unpolled for ~5 s is dropped — no explicit
                   close needed, so a closed browser tab releases the
                   subscription by itself. At most 3 concurrent taps.
  GET /api/live    snapshot of the RUNNING ROS graph, sampled every ~2 s by
                   a background rclpy probe node. Degrades honestly: where
                   rclpy is not importable (ROS not sourced) the payload
                   says {"available": false, "reason": ...} and the static
                   view keeps working untouched — rclpy is imported lazily
                   inside the probe thread, never at module import time.

Port 8091 deliberately: 8090 belongs to the frozen webui/. Binds loopback
only — this is a lab-local introspection tool, not a service.
"""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import sys
import threading
import time
import urllib.parse

from graph_dashboard.scanner import _find_src_root, compute_ego, scan_workspace

_PROBE_NODE_NAME = "graph_dashboard_probe"
_PROBE_PERIOD_S = 2.0


class LiveProber:
    """Background sampler of the live DDS graph via a throwaway rclpy node.

    Read-only introspection: the probe node publishes nothing and commands
    nothing. All rclpy use lives inside the daemon thread so the HTTP
    server works identically where ROS is absent.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._state = {"available": False, "reason": "probe starting"}
        self._thread = threading.Thread(target=self._run, daemon=True, name="live-probe")
        self._thread.start()

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def _set(self, state):
        with self._lock:
            self._state = state

    def _run(self):
        try:
            import rclpy
        except Exception as exc:  # ROS not sourced — static view must survive
            self._set({"available": False, "reason": "rclpy not importable: {}".format(exc)})
            return
        try:
            rclpy.init()
            node = rclpy.create_node(_PROBE_NODE_NAME)
        except Exception as exc:
            self._set({"available": False, "reason": "rclpy init failed: {}".format(exc)})
            return
        try:
            while True:
                time.sleep(_PROBE_PERIOD_S)
                nodes = []
                for name, ns in node.get_node_names_and_namespaces():
                    if name == _PROBE_NODE_NAME:
                        continue
                    full = (ns.rstrip("/") or "") + "/" + name
                    nodes.append({"name": name, "namespace": ns, "full": full})
                topics = []
                for tname, ttypes in node.get_topic_names_and_types():
                    topics.append(
                        {
                            "name": tname,
                            "types": ttypes,
                            "pub_count": node.count_publishers(tname),
                            "sub_count": node.count_subscribers(tname),
                        }
                    )
                self._set(
                    {
                        "available": True,
                        "sampled_at": time.time(),
                        "nodes": sorted(nodes, key=lambda n: n["full"]),
                        "topics": sorted(topics, key=lambda t: t["name"]),
                    }
                )
        except Exception as exc:
            self._set({"available": False, "reason": "probe died: {}".format(exc)})


class _Tap:
    """One tapped topic.

    Counters are written by the DDS callback; everything else is
    read/written only at poll time.
    """

    def __init__(self, topic):
        self.topic = topic
        self.sub = None
        self.msg_class = None
        self.msg_type = None
        self.last_poll = time.monotonic()
        self.count = 0
        self.bytes = 0
        self.latest_raw = None
        self.latest_mono = None
        self.prev = None  # (count, bytes, monotonic) at previous poll
        self.render_cache = None
        self.render_mono = 0.0


def _truncate_tree(obj, max_items=32):
    """Bound a message dict for display: long arrays elided, bytes summarized."""
    if isinstance(obj, dict):
        return {k: _truncate_tree(v, max_items) for k, v in obj.items()}
    if isinstance(obj, (bytes, bytearray)):
        return "bytes[{}]".format(len(obj))
    if isinstance(obj, (list, tuple)) or (hasattr(obj, "tolist") and hasattr(obj, "__len__")):
        seq = list(obj)
        out = [_truncate_tree(v, max_items) for v in seq[:max_items]]
        if len(seq) > max_items:
            out.append("… {} total".format(len(seq)))
        return out
    if isinstance(obj, (bool, int, str)) or obj is None:
        return obj
    if isinstance(obj, float):
        return round(obj, 6)
    try:
        return round(float(obj), 6)  # numpy scalars
    except (TypeError, ValueError):
        return str(obj)


def _render_message(msg, msg_type):
    """Render a deserialized message for the tap strip.

    JPEG thumbnail for images, truncated field tree otherwise. Runs at
    poll time (<= ~2 Hz), never in the DDS callback.
    """
    if msg_type == "sensor_msgs/msg/Image" and msg.encoding in ("rgb8", "bgr8"):
        import base64

        import cv2
        import numpy as np

        arr = np.frombuffer(bytes(msg.data), np.uint8)
        if msg.step == msg.width * 3:
            arr = arr.reshape(msg.height, msg.width, 3)
        else:  # row padding: cut each row down to the pixel payload
            arr = arr.reshape(msg.height, msg.step)[:, : msg.width * 3]
            arr = arr.reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        if msg.width > 480:
            scale = 480.0 / msg.width
            arr = cv2.resize(arr, (480, max(1, int(msg.height * scale))))
        ok, buf = cv2.imencode(".jpg", arr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if ok:
            return {
                "kind": "image",
                "jpeg_b64": base64.b64encode(buf.tobytes()).decode("ascii"),
                "source_size": "{}x{} {}".format(msg.width, msg.height, msg.encoding),
            }
        return {"kind": "note", "note": "jpeg encode failed"}
    from rosidl_runtime_py.convert import message_to_ordereddict

    return {"kind": "fields", "tree": _truncate_tree(message_to_ordereddict(msg))}


class TapManager:
    """On-demand single-topic live taps (oscilloscope-probe principle).

    Safety rules baked in: best-effort KEEP_LAST(1) raw subscription; the
    DDS callback only counts bytes and stashes the newest serialized
    buffer; deserialization + rendering happen at poll time (browser polls
    ~2 Hz) with a rate guard. Byte counts are exact wire payload sizes
    (raw=True serialized length). Taps expire ~5 s after the last poll.
    """

    MAX_TAPS = 3
    EXPIRE_S = 5.0
    RENDER_MIN_INTERVAL_S = 0.45

    def __init__(self):
        self._lock = threading.Lock()
        self._taps = {}
        self._node = None
        self._executor = None
        self._thread = None

    def _ensure_spinning(self):
        """Create the tap node + executor thread lazily; None if rclpy is down."""
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
        except Exception:
            return None
        if not rclpy.ok():
            return None
        with self._lock:
            if self._node is None:
                self._node = rclpy.create_node("graph_dashboard_tap")
                self._executor = SingleThreadedExecutor()
                self._executor.add_node(self._node)
                self._thread = threading.Thread(
                    target=self._spin, daemon=True, name="tap-spin"
                )
                self._thread.start()
            return self._node

    def _spin(self):
        while True:
            try:
                self._executor.spin_once(timeout_sec=0.2)
            except Exception:
                time.sleep(0.2)
            self._reap()

    def _reap(self):
        now = time.monotonic()
        with self._lock:
            for topic, tap in list(self._taps.items()):
                if now - tap.last_poll > self.EXPIRE_S:
                    if tap.sub is not None:
                        try:
                            self._node.destroy_subscription(tap.sub)
                        except Exception:
                            pass
                    del self._taps[topic]

    def _try_subscribe(self, tap, live):
        """Resolve the live type and attach the raw subscription.

        Returns a status string on failure, None on success.
        """
        entry = next((t for t in live.get("topics", []) if t["name"] == tap.topic), None)
        if entry is None or not entry.get("types"):
            return "type_unavailable"
        node = self._ensure_spinning()
        if node is None:
            return "live_unavailable"
        from rclpy.qos import (
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from rosidl_runtime_py.utilities import get_message

        tap.msg_type = entry["types"][0]
        try:
            tap.msg_class = get_message(tap.msg_type)
        except Exception as exc:
            return "type_unavailable: {}".format(exc)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        def _cb(raw, tap=tap):
            # Oscilloscope probe: count, measure, stash the reference. Zero
            # deserialization or copying here.
            tap.count += 1
            tap.bytes += len(raw)
            tap.latest_raw = raw
            tap.latest_mono = time.monotonic()

        tap.sub = node.create_subscription(tap.msg_class, tap.topic, _cb, qos, raw=True)
        return None

    def poll(self, topic, live):
        now = time.monotonic()
        with self._lock:
            tap = self._taps.get(topic)
            if tap is None:
                if len(self._taps) >= self.MAX_TAPS:
                    return {
                        "status": "tap_limit",
                        "detail": "at most {} concurrent taps — close another "
                                  "focus panel first".format(self.MAX_TAPS),
                    }
                tap = _Tap(topic)
                self._taps[topic] = tap
            tap.last_poll = now
        if not live.get("available"):
            return {"status": "live_unavailable", "reason": live.get("reason", "")}
        if tap.sub is None:
            failure = self._try_subscribe(tap, live)
            if failure:
                return {"status": failure, "topic": topic}

        count, nbytes = tap.count, tap.bytes
        hz = bw = 0.0
        if tap.prev is not None:
            dt = now - tap.prev[2]
            if dt > 0.05:
                hz = (count - tap.prev[0]) / dt
                bw = (nbytes - tap.prev[1]) / dt
        tap.prev = (count, nbytes, now)

        if count == 0:
            entry = next((t for t in live.get("topics", []) if t["name"] == topic), None)
            if entry is not None and entry.get("pub_count", 0) == 0:
                return {"status": "no_live_publisher", "topic": topic,
                        "msg_type": tap.msg_type}

        render = tap.render_cache
        raw = tap.latest_raw
        if raw is not None and now - tap.render_mono >= self.RENDER_MIN_INTERVAL_S:
            from rclpy.serialization import deserialize_message

            try:
                msg = deserialize_message(raw, tap.msg_class)
                render = _render_message(msg, tap.msg_type)
            except Exception as exc:
                render = {"kind": "note", "note": "render failed: {}".format(exc)}
            tap.render_cache = render
            tap.render_mono = now
        age = None if tap.latest_mono is None else round(now - tap.latest_mono, 2)
        return {
            "status": "ok",
            "topic": topic,
            "msg_type": tap.msg_type,
            "hz": round(hz, 2),
            "bandwidth_bps": round(bw, 1),
            "msgs_total": count,
            "last_msg_age_s": age,
            "render": render,
        }


_WEB_DIR = Path(__file__).parent / "web"


def _raise_keyboard_interrupt(signum, frame):
    raise KeyboardInterrupt


class _Handler(BaseHTTPRequestHandler):
    src_root = None  # set by main() before serving
    prober = None  # set by main() before serving
    tap_manager = None  # set by main() before serving

    def _send(self, status, content_type, body):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):  # noqa: N802 — http.server API name
        if self.path == "/api/live":
            body = json.dumps(self.prober.snapshot()).encode("utf-8")
            self._send(200, "application/json", body)
        elif self.path.startswith("/api/tap"):
            query = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            elem = (query.get("id") or [""])[0]
            graph = scan_workspace(self.src_root)
            known = {"topic:" + t["name"] for t in graph["topics"]}
            if elem not in known:
                self._send(404, "application/json",
                           json.dumps({"error": "unknown topic", "id": elem}).encode("utf-8"))
            else:
                result = self.tap_manager.poll(elem[len("topic:"):], self.prober.snapshot())
                self._send(200, "application/json", json.dumps(result).encode("utf-8"))
        elif self.path.startswith("/api/ego"):
            query = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            elem = (query.get("id") or [""])[0]
            graph = scan_workspace(self.src_root)
            known = {n["id"] for n in graph["nodes"]}
            known.update("topic:" + t["name"] for t in graph["topics"])
            if elem not in known:
                self._send(404, "application/json",
                           json.dumps({"error": "unknown element", "id": elem}).encode("utf-8"))
            else:
                # Advisor-scoped radius: direct neighborhood only. A node
                # shows its topics (±1) and the before/after nodes (±2);
                # a topic shows just its publishers/subscribers (±1).
                depth = 1 if elem.startswith("topic:") else 2
                body = json.dumps(
                    compute_ego(graph["edges"], elem, max_depth=depth)
                ).encode("utf-8")
                self._send(200, "application/json", body)
        elif self.path.split("?", 1)[0] in ("/", "/index.html"):
            body = (_WEB_DIR / "index.html").read_bytes()
            self._send(200, "text/html; charset=utf-8", body)
        elif self.path == "/vendor/vis-network.min.js":
            body = (_WEB_DIR / "vis-network.min.js").read_bytes()
            self._send(200, "application/javascript", body)
        elif self.path == "/api/graph":
            graph = scan_workspace(self.src_root)
            body = json.dumps(graph).encode("utf-8")
            self._send(200, "application/json", body)
        else:
            self._send(404, "text/plain; charset=utf-8", b"not found\n")

    def log_message(self, fmt, *args):
        # One quiet line per request instead of BaseHTTPRequestHandler's
        # stderr chatter with timestamps.
        print("  {} {}".format(self.address_string(), fmt % args))


def main(argv=None):
    parser = argparse.ArgumentParser(description="Serve the static ROS-graph dashboard.")
    parser.add_argument(
        "--src",
        type=Path,
        default=None,
        help="workspace src directory (default: auto-detect a src/ with packages upward from cwd)",
    )
    parser.add_argument("--host", default="127.0.0.1", help="bind address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8091, help="port (default: 8091)")
    args = parser.parse_args(argv)

    src_root = args.src or _find_src_root(Path.cwd())
    if src_root is None or not Path(src_root).is_dir():
        print(
            "error: could not find a colcon src/ directory with packages by "
            "walking up from the current directory; pass --src /path/to/ws/src",
            file=sys.stderr,
        )
        return 2

    _Handler.src_root = Path(src_root).resolve()
    _Handler.prober = LiveProber()
    _Handler.tap_manager = TapManager()
    server = ThreadingHTTPServer((args.host, args.port), _Handler)

    # SIGTERM shuts down as cleanly as Ctrl-C: raising KeyboardInterrupt in
    # the main thread breaks serve_forever() and runs the same finally path
    # (matters for scripted/supervised runs, where SIGINT may be ignored).
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)
    print(
        "graph_dashboard: serving {} on http://{}:{}/ (Ctrl-C to stop)".format(
            _Handler.src_root, args.host, args.port
        )
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\ngraph_dashboard: shutting down")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
