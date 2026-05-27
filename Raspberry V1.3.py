#!/usr/bin/env python3
# =====================================================================
#   PLANT CARE PI CONTROLLER  -  v1.3
#   Raspberry Pi supervisor for the Mega-based plant care robot.
#
#   v1.3 — Bug fixes & hardening (this revision):
#     • os.getlogin() crash fixed (use pwd.getpwuid; safe in non-tty)
#     • XSS in events list fixed (textContent instead of innerHTML)
#     • DB writer drains queue on shutdown (no more lost rows)
#     • Silent command loss logged as 'command_failed' event
#     • HTTP_HOST defaults to 127.0.0.1 (PLANT_HTTP_HOST overrides)
#     • Same-origin CSRF check on /api/command
#     • Optional API key now also protects /snapshot.jpg
#     • DB writer keeps one persistent SQLite connection
#     • Camera init runs on background thread (no startup block)
#     • Bucket-average downsampling (preserves trends; not decimation)
#     • Per-zone rate limit on water commands
#     • /api/command rejects 503 when Mega is offline (was silent)
#     • State.snapshot returns shallow copy (defensive)
#     • Cleaned up dialout-membership check logic
#     • Documented sentinel values in _clean()
#
#   v1.2 — earlier audit fixes (carried forward).
#   v1.1 — auto-detection of Arduino serial port.
#
#   Setup on Raspberry Pi OS (Bookworm 3.11+ or Bullseye 3.9+):
#     sudo apt update
#     sudo apt install -y python3-pip python3-venv python3-opencv
#     sudo usermod -aG dialout $USER     # then log out and back in
#     python3 -m venv .venv
#     source .venv/bin/activate
#     pip install -r requirements.txt
#     # Optional: enable API auth for command endpoint
#     # export PLANT_API_KEY="some-long-random-string"
#     # Optional: bind LAN-wide (default is loopback only)
#     # export PLANT_HTTP_HOST="0.0.0.0"
#     python3 plant_controller.py
#
#   For long-running deployment, prefer waitress over the dev server:
#     pip install waitress
#     # then edit main() to use serve(app, host=..., port=..., threads=4)
# =====================================================================

from __future__ import annotations   # PEP 563: `str | None` works on 3.9

import grp
import json
import logging
import os
import queue
import signal
import sqlite3
import sys
import threading
import time
from collections import deque
from pathlib import Path
from urllib.parse import urlparse

import serial
from serial.tools import list_ports
from flask import Flask, Response, jsonify, render_template_string, request

# pwd is POSIX-only; we only use it in a try/except so this import is
# also guarded.
try:
    import pwd
    PWD_AVAILABLE = True
except ImportError:
    PWD_AVAILABLE = False

# OpenCV is optional - the dashboard works fine without it
try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# ======================== CONFIGURATION ==============================
# Set to None for auto-detection, or hard-code a path to force one port.
SERIAL_PORT_OVERRIDE: str | None = None

# Known USB-Serial chip VIDs found in Arduino Mega boards (genuine + clones)
KNOWN_USB_SERIAL_VIDS = {
    0x2341,   # Arduino LLC (genuine Mega, Uno, etc.)
    0x2A03,   # Arduino SRL
    0x1A86,   # WCH CH340/CH341 (most clones)
    0x0403,   # FTDI (older clones)
    0x10C4,   # Silicon Labs CP210x
}

SERIAL_BAUD              = 115200
SERIAL_TIMEOUT_S         = 1.0
RECONNECT_BACKOFF_MAX_S  = 10

# How fresh must the latest status be to count the Mega as "connected"
CONNECTED_TIMEOUT_S      = 15
# Don't write status rows to DB more than once every N seconds (saves SD)
DB_STATUS_THROTTLE_S     = 30
# Cap history endpoint response to this many points (downsampling)
HISTORY_MAX_POINTS       = 500
# DB writer queue size (drops oldest if full)
DB_EVENT_QUEUE_SIZE      = 500
# Command queue size; rejects new commands when full
COMMAND_QUEUE_SIZE       = 32
# Minimum seconds between accepting water commands for the same zone
ZONE_COMMAND_COOLDOWN_S  = 10

CAMERA_INDEX             = 0
CAMERA_WIDTH             = 1280
CAMERA_HEIGHT            = 720
CAMERA_JPEG_Q            = 80
CAMERA_CACHE_S           = 1.0

DB_PATH                  = Path(__file__).resolve().parent / "plant_history.db"
LOG_RETAIN_DAYS          = 30

# Default to loopback for safety. The dashboard has no auth by default,
# and listening on 0.0.0.0 exposes pump control to the whole LAN.
# To enable LAN access, either set PLANT_API_KEY *and* PLANT_HTTP_HOST,
# or accept the risk and set PLANT_HTTP_HOST="0.0.0.0".
HTTP_HOST                = os.environ.get("PLANT_HTTP_HOST", "127.0.0.1")
HTTP_PORT                = int(os.environ.get("PLANT_HTTP_PORT", "5000"))

# Optional API key auth for /api/command and /snapshot.jpg.
API_KEY: str | None      = os.environ.get("PLANT_API_KEY") or None

# Friendly names for the 3 zones.
ZONE_NAMES = ["Low-Light Herbs (Mint)",
              "Bright-Light Fruit (Cherry Tomato)",
              "Balanced Herbs (Basil)"]

# ======================== LOGGING ====================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
log = logging.getLogger("plant-pi")
# Silence Werkzeug's per-request log line (would print every 3s from polling)
logging.getLogger("werkzeug").setLevel(logging.WARNING)

# ======================== UTILITIES ==================================
def _pad3(arr, fill=None) -> list:
    """Coerce *arr* to a list of exactly 3 items, padding/truncating.

    Defensive helper: the Mega is supposed to send 3-element arrays for
    moisture/fails/alarms, but a firmware glitch or future addition
    could change that. Indexing arr[2] blindly used to crash the whole
    Serial thread; this prevents that.
    """
    out = list(arr) if isinstance(arr, (list, tuple)) else []
    while len(out) < 3:
        out.append(fill)
    return out[:3]


def _clean(v):
    """Convert sentinel values to None so charts aren't polluted.

    Sentinel conventions used by the Mega firmware:
      • moisture (m0..m2):  -1  means sensor disconnected / open circuit
      • light    (l):       -1  means no reading available
      • temp:               -99 means DHT11 read failed
      • humidity (hum):     -1  means DHT11 read failed
    Returning None makes SQLite store NULL, which Chart.js skips
    (spanGaps: true) instead of drawing a downward spike to -99.
    """
    if v is None:
        return None
    if v == -1 or v == -99:
        return None
    if isinstance(v, float) and v < -50:   # guard against -99.0 drift
        return None
    return v


# ======================== SHARED STATE ===============================
class State:
    """Thread-safe shared state between the serial thread and Flask."""

    def __init__(self) -> None:
        self._lock         = threading.Lock()
        self.last_status   = None
        self.last_seen_mono = 0.0          # monotonic clock (NTP-safe)
        self.last_seen_wall = 0.0          # wall clock (for display)
        self.mega_log      = deque(maxlen=200)
        self.startup_mono  = time.monotonic()
        self.startup_wall  = time.time()

    def update_status(self, data: dict) -> None:
        with self._lock:
            self.last_status    = data
            self.last_seen_mono = time.monotonic()
            self.last_seen_wall = time.time()

    def append_log(self, line: str) -> None:
        with self._lock:
            self.mega_log.append((time.time(), line))

    def snapshot(self) -> dict:
        with self._lock:
            # Shallow copy: the Mega payload is flat (no nested dicts), so
            # dict() is enough to prevent HTTP threads from accidentally
            # mutating our cached state.
            status_copy = dict(self.last_status) if self.last_status else None
            connected = (
                self.last_seen_mono > 0
                and (time.monotonic() - self.last_seen_mono) < CONNECTED_TIMEOUT_S
            )
            return {
                "status":     status_copy,
                "last_seen":  self.last_seen_wall,
                "connected":  connected,
                "uptime_s":   int(time.monotonic() - self.startup_mono),
                "recent_log": list(self.mega_log)[-20:],
            }


state = State()
command_queue:   "queue.Queue[dict]"  = queue.Queue(maxsize=COMMAND_QUEUE_SIZE)
db_event_queue:  "queue.Queue[tuple]" = queue.Queue(maxsize=DB_EVENT_QUEUE_SIZE)
shutdown_event = threading.Event()

# Zone rate-limit bookkeeping (monotonic timestamps of last water cmd)
_zone_last_water_mono: dict[int, float] = {}
_zone_rate_lock = threading.Lock()


def _check_zone_rate(zone: int) -> bool:
    """Return True if it's been long enough to allow another water cmd."""
    with _zone_rate_lock:
        now = time.monotonic()
        last = _zone_last_water_mono.get(zone, 0.0)
        if now - last < ZONE_COMMAND_COOLDOWN_S:
            return False
        _zone_last_water_mono[zone] = now
        return True


# ======================== DATABASE ===================================
def init_db() -> None:
    conn = sqlite3.connect(str(DB_PATH))
    cur  = conn.cursor()
    # WAL: readers don't block writers; better behaviour under load.
    # synchronous=NORMAL: safe with WAL; faster than FULL on SD cards.
    # busy_timeout: wait up to 5s on lock contention before raising.
    cur.execute("PRAGMA journal_mode=WAL;")
    cur.execute("PRAGMA synchronous=NORMAL;")
    cur.execute("PRAGMA busy_timeout=5000;")
    cur.executescript("""
        CREATE TABLE IF NOT EXISTS status_log (
            ts        INTEGER PRIMARY KEY,
            state     TEXT,
            zone      INTEGER,
            m0        INTEGER, m1        INTEGER, m2        INTEGER,
            light     INTEGER,
            temp      REAL,
            humidity  REAL,
            fails_0   INTEGER, fails_1   INTEGER, fails_2   INTEGER,
            alarms_0  INTEGER, alarms_1  INTEGER, alarms_2  INTEGER
        );
        CREATE TABLE IF NOT EXISTS events (
            id       INTEGER PRIMARY KEY AUTOINCREMENT,
            ts       INTEGER NOT NULL,
            kind     TEXT    NOT NULL,
            zone     INTEGER,
            details  TEXT
        );
        CREATE INDEX IF NOT EXISTS idx_events_ts ON events(ts DESC);
    """)
    conn.commit()
    conn.close()
    log.info(f"Database ready at {DB_PATH}")


def _db_insert_status_conn(conn: sqlite3.Connection, data: dict) -> None:
    """Insert one status row using an existing connection."""
    m = _pad3(data.get("m"))
    f = _pad3(data.get("fails"), 0)
    a = _pad3(data.get("alarms"), 0)
    conn.execute(
        "INSERT OR REPLACE INTO status_log VALUES "
        "(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
        (int(time.time()), data.get("state"), data.get("zone", -1),
         _clean(m[0]), _clean(m[1]), _clean(m[2]),
         _clean(data.get("l")),
         _clean(data.get("temp")), _clean(data.get("hum")),
         f[0] or 0, f[1] or 0, f[2] or 0,
         a[0] or 0, a[1] or 0, a[2] or 0))
    conn.commit()


def _db_insert_event_conn(conn: sqlite3.Connection,
                          kind: str,
                          zone: int | None = None,
                          details: str = "") -> None:
    conn.execute(
        "INSERT INTO events(ts,kind,zone,details) VALUES (?,?,?,?)",
        (int(time.time()), kind, zone, details))
    conn.commit()


def db_history(hours: int = 24) -> list[dict]:
    try:
        cutoff = int(time.time()) - hours * 3600
        conn = sqlite3.connect(str(DB_PATH))
        conn.row_factory = sqlite3.Row
        rows = conn.execute(
            "SELECT * FROM status_log WHERE ts >= ? ORDER BY ts ASC", (cutoff,)
        ).fetchall()
        conn.close()
        return [dict(r) for r in rows]
    except Exception as e:
        log.error(f"db_history failed: {e}")
        return []


def db_events(limit: int = 30) -> list[dict]:
    try:
        conn = sqlite3.connect(str(DB_PATH))
        conn.row_factory = sqlite3.Row
        rows = conn.execute(
            "SELECT * FROM events ORDER BY ts DESC LIMIT ?", (limit,)
        ).fetchall()
        conn.close()
        return [dict(r) for r in rows]
    except Exception as e:
        log.error(f"db_events failed: {e}")
        return []


def db_prune() -> None:
    try:
        cutoff = int(time.time()) - LOG_RETAIN_DAYS * 86400
        conn = sqlite3.connect(str(DB_PATH))
        conn.execute("DELETE FROM status_log WHERE ts < ?", (cutoff,))
        conn.execute("DELETE FROM events     WHERE ts < ?", (cutoff,))
        conn.commit()
        conn.close()
    except Exception as e:
        log.warning(f"db_prune failed: {e}")


# ---- async DB writer -------------------------------------------------
# The Serial thread should NEVER block on disk I/O (a slow SD write
# could mean missing a Mega status frame). The writer holds one
# persistent connection — far cheaper than open/close per row — and
# drains the queue before exiting on shutdown.

def enqueue_db_status(data: dict) -> None:
    try:
        db_event_queue.put_nowait(("status", (data,)))
    except queue.Full:
        # Drop on overflow rather than blocking. Status rows are
        # already throttled to one per 30s, so this is essentially
        # impossible in practice unless the DB is completely stuck.
        log.warning("DB queue full; dropped a status row")


def enqueue_db_event(kind: str, zone: int | None = None,
                     details: str = "") -> None:
    try:
        db_event_queue.put_nowait(("event", (kind, zone, details)))
    except queue.Full:
        log.warning(f"DB queue full; dropped event {kind}")


def db_writer_loop() -> None:
    """Persistent-connection DB writer that drains on shutdown.

    Unlike v1.2, the loop does NOT exit while items remain in the queue;
    it only exits when shutdown_event is set AND the queue is empty.
    """
    conn = sqlite3.connect(str(DB_PATH))
    conn.execute("PRAGMA journal_mode=WAL;")
    conn.execute("PRAGMA synchronous=NORMAL;")
    conn.execute("PRAGMA busy_timeout=5000;")
    log.info("DB writer started")
    try:
        while True:
            try:
                kind, args = db_event_queue.get(timeout=0.5)
            except queue.Empty:
                if shutdown_event.is_set():
                    log.info("DB writer: queue empty, exiting")
                    return
                continue
            try:
                if kind == "status":
                    _db_insert_status_conn(conn, *args)
                elif kind == "event":
                    _db_insert_event_conn(conn, *args)
            except Exception as e:
                log.error(f"db_writer error processing {kind}: {e}")
    finally:
        try:
            conn.close()
        except Exception:
            pass


# ======================== DOWNSAMPLING ===============================
def downsample_status(rows: list[dict], max_points: int) -> list[dict]:
    """Bucket-average downsampling.

    Unlike `rows[::step]` (which decimates and can hide brief spikes),
    this averages each numeric column across the bucket and keeps the
    middle row's text/integer fields as a representative. Cheaper than
    LTTB, plenty good enough for a 24-hour moisture chart.
    """
    if len(rows) <= max_points:
        return rows
    bucket = len(rows) / max_points
    out: list[dict] = []
    NUMERIC     = ("m0", "m1", "m2", "light", "temp", "humidity")
    PASSTHROUGH = ("state", "zone",
                   "fails_0", "fails_1", "fails_2",
                   "alarms_0", "alarms_1", "alarms_2")
    for i in range(max_points):
        start = int(i * bucket)
        end   = max(int((i + 1) * bucket), start + 1)
        chunk = rows[start:end]
        if not chunk:
            continue
        rep = chunk[len(chunk) // 2]
        row_out: dict = {"ts": rep["ts"]}
        for k in NUMERIC:
            vals = [r[k] for r in chunk if r.get(k) is not None]
            row_out[k] = (sum(vals) / len(vals)) if vals else None
        for k in PASSTHROUGH:
            row_out[k] = rep.get(k)
        out.append(row_out)
    return out


# ======================== SERIAL PORT DETECTION ======================
def detect_serial_port() -> str | None:
    """Auto-detect an Arduino-compatible USB-Serial port."""
    if SERIAL_PORT_OVERRIDE:
        return SERIAL_PORT_OVERRIDE

    ports = list(list_ports.comports())
    if not ports:
        return None

    # Pass 1: match by known USB VID (most reliable signal)
    for p in ports:
        if p.vid is not None and p.vid in KNOWN_USB_SERIAL_VIDS:
            pid_str = f"{p.pid:04x}" if p.pid is not None else "????"
            log.info(f"Auto-detected Arduino at {p.device} "
                     f"(VID:PID {p.vid:04x}:{pid_str} · {p.description})")
            return p.device

    # Pass 2: name-pattern fallback
    for p in ports:
        if "ttyACM" in p.device or "ttyUSB" in p.device:
            log.info(f"Auto-detected serial device at {p.device} "
                     f"(no VID match · {p.description})")
            return p.device

    return None


# ======================== SERIAL BRIDGE ==============================
class SerialBridge(threading.Thread):
    """Maintains the link with the Mega: reads status, writes commands."""

    def __init__(self) -> None:
        super().__init__(daemon=True, name="SerialBridge")
        self.ser              = None
        self.current_port     = None
        self.prev_state       = None
        self.prev_alarms      = [0, 0, 0]
        self.last_db_log_mono = 0.0       # monotonic clock

    def open(self) -> bool:
        port = detect_serial_port()
        if port is None:
            log.warning("No Arduino-compatible serial port found. "
                        "Check the USB cable, or run: ls /dev/tty*")
            self.ser = None
            self.current_port = None
            return False
        try:
            self.ser = serial.Serial(port, SERIAL_BAUD,
                                     timeout=SERIAL_TIMEOUT_S)
            self.current_port = port
            log.info(f"Opened {port} @ {SERIAL_BAUD}")
            enqueue_db_event("connect", details=f"Connected to {port}")
            return True
        except (serial.SerialException, OSError) as e:
            log.warning(f"Could not open {port}: {e}")
            self.ser = None
            self.current_port = None
            return False

    def close(self) -> None:
        if self.ser:
            try: self.ser.close()
            except Exception: pass
        self.ser = None
        self.current_port = None

    def send(self, cmd: dict) -> bool:
        if not self.ser or not self.ser.is_open:
            return False
        try:
            payload = (json.dumps(cmd, separators=(",", ":")) + "\n").encode()
            self.ser.write(payload)
            self.ser.flush()
            log.info(f"-> Mega: {cmd}")
            enqueue_db_event("command", zone=cmd.get("zone"),
                             details=json.dumps(cmd))
            return True
        except Exception as e:
            log.error(f"Serial write failed: {e}")
            enqueue_db_event("disconnect", details=f"write failed: {e}")
            self.close()
            return False

    def _handle_line(self, line: str) -> None:
        line = line.strip()
        if not line:
            return

        if line.startswith("{"):
            try:
                data = json.loads(line)
            except json.JSONDecodeError:
                state.append_log(f"[bad-json] {line}")
                return
            if not isinstance(data, dict):
                state.append_log(f"[not-object] {line}")
                return

            state.update_status(data)

            # Throttle history writes
            now_mono = time.monotonic()
            if now_mono - self.last_db_log_mono >= DB_STATUS_THROTTLE_S:
                enqueue_db_status(data)
                self.last_db_log_mono = now_mono

            # State change detection
            new_state  = data.get("state")
            new_alarms = _pad3(data.get("alarms"), 0)

            if new_state != self.prev_state and self.prev_state is not None:
                enqueue_db_event("state_change", zone=data.get("zone"),
                                 details=f"{self.prev_state} -> {new_state}")
            self.prev_state = new_state

            # Edge-triggered alarm detection
            for i, (old, new) in enumerate(zip(self.prev_alarms, new_alarms)):
                old = old or 0
                new = new or 0
                if old == 0 and new == 1:
                    enqueue_db_event("alarm_set", zone=i,
                                     details=f"Zone {i+1} entered alarm")
                elif old == 1 and new == 0:
                    enqueue_db_event("alarm_clear", zone=i,
                                     details=f"Zone {i+1} alarm cleared")
            self.prev_alarms = new_alarms

        else:
            state.append_log(line)

    def run(self) -> None:
        backoff = 1.0
        while not shutdown_event.is_set():
            if not self.ser:
                if not self.open():
                    if shutdown_event.wait(min(backoff, RECONNECT_BACKOFF_MAX_S)):
                        break
                    backoff = min(backoff * 1.5, RECONNECT_BACKOFF_MAX_S)
                    continue
                backoff = 1.0

            try:
                line = self.ser.readline().decode("utf-8", errors="replace")
                if line:
                    self._handle_line(line)
            except Exception as e:
                log.warning(f"Serial read error on {self.current_port}: {e}")
                self.close()
                enqueue_db_event("disconnect", details=str(e))
                continue

            # Drain outbound command queue. If a write fails the port is
            # already closed by send(); log every command we couldn't
            # deliver so the operator sees it in the events feed instead
            # of silently losing it.
            try:
                while True:
                    cmd = command_queue.get_nowait()
                    if not self.send(cmd):
                        enqueue_db_event(
                            "command_failed", zone=cmd.get("zone"),
                            details=f"send failed: {json.dumps(cmd)}")
                        # Port is closed; stop draining and let the outer
                        # loop reconnect. Remaining commands stay queued.
                        break
            except queue.Empty:
                pass


# ======================== CAMERA =====================================
class Camera:
    """Thin OpenCV wrapper. Returns latest JPEG bytes on demand.

    VideoCapture(0) can stall for several seconds when no device is
    present, so initialization happens on a background thread to keep
    startup snappy.
    """

    def __init__(self) -> None:
        self.cap          = None
        self.lock         = threading.Lock()
        self.last_jpeg    = None
        self.last_capture = 0.0
        self.enabled      = CV2_AVAILABLE
        self._init_done   = threading.Event()
        if self.enabled:
            threading.Thread(target=self._try_open, daemon=True,
                             name="CameraInit").start()
        else:
            self._init_done.set()

    def _try_open(self) -> None:
        try:
            cap = cv2.VideoCapture(CAMERA_INDEX)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            if not cap.isOpened():
                log.warning("Camera failed to open; video features disabled.")
                try: cap.release()
                except Exception: pass
                return
            with self.lock:
                self.cap = cap
            log.info("Camera initialized")
        except Exception as e:
            log.warning(f"Camera init error: {e}")
        finally:
            self._init_done.set()

    def is_ready(self) -> bool:
        return self.enabled and self.cap is not None

    def snapshot(self) -> bytes | None:
        if not self.is_ready():
            return None
        with self.lock:
            if self.last_jpeg and (time.monotonic() - self.last_capture) < CAMERA_CACHE_S:
                return self.last_jpeg
            ok, frame = self.cap.read()
            if not ok:
                try: self.cap.release()
                except Exception: pass
                self.cap = None
                # Re-attempt open in the background so we don't block here.
                threading.Thread(target=self._try_open, daemon=True,
                                 name="CameraReopen").start()
                return self.last_jpeg
            ok, buf = cv2.imencode(".jpg", frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, CAMERA_JPEG_Q])
            if not ok:
                return self.last_jpeg
            self.last_jpeg    = buf.tobytes()
            self.last_capture = time.monotonic()
            return self.last_jpeg


camera = Camera()


# ======================== FLASK APP ==================================
app = Flask(__name__)


def _same_origin() -> bool:
    """Reject obvious cross-origin POSTs (CSRF defense).

    Browsers always send Origin on cross-origin POST. If Origin is
    missing, the request is either same-origin (per HTML spec for some
    fetch modes) or non-browser (curl, scripts).
    """
    origin = request.headers.get("Origin")
    if not origin:
        return True
    try:
        o = urlparse(origin)
    except Exception:
        return False
    if o.netloc == request.host:
        return True
    if o.hostname in ("localhost", "127.0.0.1", "::1"):
        return True
    return False


def _check_api_key() -> bool:
    if not API_KEY:
        return True
    key = request.headers.get("X-API-Key") or request.args.get("api_key")
    return key == API_KEY


@app.before_request
def _auth_gate():
    """Auth + CSRF for sensitive endpoints."""
    if request.path == "/api/command":
        if not _same_origin():
            return jsonify(error="cross-origin request rejected"), 403
        if not _check_api_key():
            return jsonify(error="unauthorized; X-API-Key required"), 401
    elif request.path == "/snapshot.jpg":
        # Camera output is private if a key is set.
        if not _check_api_key():
            return Response(status=401)
    return None


@app.route("/")
def index():
    return render_template_string(
        DASHBOARD_HTML,
        zones=ZONE_NAMES,
        api_required=bool(API_KEY),
    )


@app.route("/api/status")
def api_status():
    snap = state.snapshot()
    snap["zone_names"]     = ZONE_NAMES
    snap["camera_enabled"] = camera.is_ready()
    snap["now"]            = time.time()
    return jsonify(snap)


@app.route("/api/history")
def api_history():
    # Flask coerces type; falls back to default on parse failure.
    hours = request.args.get("hours", default=24, type=int) or 24
    hours = max(1, min(hours, 720))
    rows  = db_history(hours)
    rows  = downsample_status(rows, HISTORY_MAX_POINTS)
    return jsonify(rows)


@app.route("/api/events")
def api_events():
    limit = request.args.get("limit", default=30, type=int) or 30
    limit = max(1, min(limit, 200))
    return jsonify(db_events(limit))


@app.route("/api/command", methods=["POST"])
def api_command():
    # Refuse to queue blind commands when we know the Mega is offline.
    snap = state.snapshot()
    if not snap["connected"]:
        return jsonify(error="Mega is offline; not accepting commands"), 503

    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify(error="JSON object body required"), 400

    cmd = payload.get("cmd")

    def queue_cmd(c: dict):
        try:
            command_queue.put_nowait(c)
            return jsonify(queued=True)
        except queue.Full:
            return jsonify(error="command queue full; try again shortly"), 503

    if cmd == "water":
        try:
            zone = int(payload.get("zone", -1))
            ms   = int(payload.get("ms", 3000))
        except (TypeError, ValueError):
            return jsonify(error="zone/ms must be integers"), 400
        if not 0 <= zone <= 2:
            return jsonify(error="zone must be 0..2"), 400
        if not 500 <= ms <= 8000:
            return jsonify(error="ms must be 500..8000"), 400
        if not _check_zone_rate(zone):
            return jsonify(
                error=f"zone {zone} on cooldown "
                      f"({ZONE_COMMAND_COOLDOWN_S}s between waterings)"
            ), 429
        return queue_cmd({"cmd": "water", "zone": zone, "ms": ms})

    if cmd == "clear_alarm":
        try:
            zone = int(payload.get("zone", -1))
        except (TypeError, ValueError):
            return jsonify(error="zone must be integer"), 400
        if not 0 <= zone <= 2:
            return jsonify(error="zone must be 0..2"), 400
        return queue_cmd({"cmd": "clear_alarm", "zone": zone})

    if cmd == "set_servo":
        try:
            angle = int(payload.get("angle", 0))
        except (TypeError, ValueError):
            return jsonify(error="angle must be integer"), 400
        if not 0 <= angle <= 180:
            return jsonify(error="angle must be 0..180"), 400
        return queue_cmd({"cmd": "set_servo", "angle": angle})

    return jsonify(error=f"unknown cmd: {cmd}"), 400


@app.route("/snapshot.jpg")
def snapshot():
    img = camera.snapshot()
    if img is None:
        # 204 No Content is more correct than an empty 503 JPEG.
        return Response(status=204)
    return Response(img, mimetype="image/jpeg",
                    headers={"Cache-Control": "no-store"})


# ======================== DASHBOARD HTML =============================
# Botanical field journal aesthetic. Changes since v1.2:
#  - Events list now built with createElement/textContent (no XSS)
#  - Snapshot URL carries api_key in query string when auth required
#  - Misc small JS robustness fixes

DASHBOARD_HTML = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Hortus Vigilis — Live Garden Monitor</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Cormorant+Garamond:ital,wght@0,400;0,500;0,600;0,700;1,400;1,600&family=Crimson+Pro:wght@400;500;600&family=JetBrains+Mono:wght@400;500;600&display=swap">
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/chartjs-adapter-date-fns@3.0.0/dist/chartjs-adapter-date-fns.bundle.min.js"></script>
<style>
  :root {
    --paper:    #f3ecdb;
    --paper-2:  #eae2cc;
    --paper-3:  #ddd1b3;
    --ink:      #2a1f12;
    --ink-2:    #4a3d2a;
    --ink-soft: #786648;
    --moss:     #3d5a3d;
    --moss-2:   #2a4029;
    --terra:    #a85d3c;
    --terra-2:  #8a4729;
    --gold:     #b08d57;
    --rule:     #b8a884;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  html { background: var(--paper); }
  body {
    font-family: 'Crimson Pro', Georgia, serif;
    color: var(--ink); line-height: 1.5; min-height: 100vh;
    background:
      radial-gradient(ellipse at 20% 10%, rgba(176,141,87,.07) 0%, transparent 50%),
      radial-gradient(ellipse at 80% 80%, rgba(61,90,61,.06) 0%, transparent 55%),
      var(--paper);
    background-attachment: fixed;
  }
  body::before {
    content: ""; position: fixed; inset: 0; pointer-events: none; z-index: 0;
    background-image:
      url("data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='180' height='180'><filter id='n'><feTurbulence baseFrequency='0.85' numOctaves='2' seed='5'/><feColorMatrix values='0 0 0 0 .15 0 0 0 0 .12 0 0 0 0 .08 0 0 0 .06 0'/></filter><rect width='100%' height='100%' filter='url(%23n)'/></svg>");
    opacity: 0.55;
  }
  .wrap { max-width: 1180px; margin: 0 auto; padding: 36px 28px 80px; position: relative; z-index: 1; }
  header { text-align: center; margin-bottom: 28px; position: relative; }
  .conn {
    position: absolute; top: 4px; right: 0; display: inline-flex; align-items: center; gap: 8px;
    font-family: 'JetBrains Mono', monospace; font-size: 11px; letter-spacing: .12em;
    text-transform: uppercase; color: var(--ink-soft);
  }
  .conn .dot { width: 8px; height: 8px; border-radius: 50%;
    background: var(--terra); box-shadow: 0 0 0 0 rgba(168,93,60,.6);
    animation: pulse 2s infinite; }
  .conn.online .dot { background: var(--moss); }
  @keyframes pulse {
    0%, 100% { box-shadow: 0 0 0 0 rgba(61,90,61,.5); }
    50%      { box-shadow: 0 0 0 6px rgba(61,90,61,0); }
  }
  h1 { font-family: 'Cormorant Garamond', serif; font-weight: 500;
       font-size: clamp(40px, 6vw, 64px); letter-spacing: 0.02em;
       line-height: 1; color: var(--ink); }
  h1 em { color: var(--moss); font-style: italic; font-weight: 400; }
  .subtitle { font-family: 'Cormorant Garamond', serif; font-style: italic;
              font-weight: 400; font-size: 18px; color: var(--ink-soft); margin-top: 4px; }
  .meta-bar { margin-top: 14px; font-family: 'JetBrains Mono', monospace;
              font-size: 11px; letter-spacing: .14em; color: var(--ink-soft);
              text-transform: uppercase; }
  .meta-bar span { margin: 0 14px; }
  .divider { display: flex; align-items: center; justify-content: center;
             gap: 16px; margin: 30px 0 36px; color: var(--rule); }
  .divider::before, .divider::after {
    content: ""; flex: 1; max-width: 240px;
    border-top: 1px solid var(--rule); border-bottom: 1px solid var(--rule); height: 3px;
  }
  .divider svg { width: 18px; height: 18px; color: var(--gold); }
  .row { display: grid; gap: 22px; }
  .zones { grid-template-columns: repeat(3, 1fr); }
  .two-up { grid-template-columns: 1.4fr 1fr; margin-top: 24px; }
  @media (max-width: 880px) {
    .zones, .two-up { grid-template-columns: 1fr; }
  }
  .card { background: var(--paper-2); border: 1px solid var(--rule);
          padding: 22px 24px 20px; position: relative;
          box-shadow: 0 1px 0 var(--paper-3) inset, 0 8px 24px -16px rgba(42,31,18,.25); }
  .card::before, .card::after { content: ""; position: absolute; left: 8px; right: 8px;
                                height: 1px; background: var(--rule); }
  .card::before { top: 4px; } .card::after { bottom: 4px; }
  .card-label { font-family: 'JetBrains Mono', monospace; font-size: 10px;
                letter-spacing: .22em; color: var(--ink-soft); text-transform: uppercase;
                margin-bottom: 8px; }
  .card h2 { font-family: 'Cormorant Garamond', serif; font-weight: 500;
             font-size: 22px; line-height: 1.15; margin-bottom: 18px; color: var(--ink); }
  .card h2 em { font-style: italic; color: var(--moss); font-weight: 400; }
  .moisture { font-family: 'Cormorant Garamond', serif; font-weight: 500;
              font-size: 76px; line-height: 1; color: var(--ink);
              display: flex; align-items: baseline; gap: 4px; }
  .moisture .pct { font-size: 26px; font-weight: 400; color: var(--ink-soft); margin-left: 2px; }
  .moisture.err { color: var(--terra); font-size: 32px; font-style: italic; }
  .status-tag { display: inline-block; margin-top: 12px; padding: 4px 12px;
                font-family: 'JetBrains Mono', monospace; font-size: 10px;
                letter-spacing: .18em; text-transform: uppercase;
                border: 1px solid var(--rule); background: var(--paper); color: var(--ink-2); }
  .status-tag.busy   { background: var(--moss);  color: var(--paper); border-color: var(--moss-2); }
  .status-tag.alarm  { background: var(--terra); color: var(--paper); border-color: var(--terra-2);
                       animation: blink 1.2s infinite; }
  @keyframes blink { 50% { opacity: 0.65; } }
  .zone-meta { margin-top: 14px; padding-top: 12px; border-top: 1px solid var(--rule);
               font-family: 'JetBrains Mono', monospace; font-size: 11px;
               color: var(--ink-soft); letter-spacing: .04em;
               display: flex; justify-content: space-between; }
  .actions { display: flex; gap: 8px; margin-top: 14px; }
  button { flex: 1; background: var(--paper); color: var(--ink);
           border: 1px solid var(--rule); padding: 9px 12px;
           font-family: 'Cormorant Garamond', serif; font-weight: 500;
           font-size: 15px; font-style: italic; letter-spacing: .02em;
           cursor: pointer; transition: all .15s ease; }
  button:hover { background: var(--moss); color: var(--paper); border-color: var(--moss-2); }
  button.warn { color: var(--terra); }
  button.warn:hover { background: var(--terra); color: var(--paper); border-color: var(--terra-2); }
  button:disabled { opacity: 0.4; cursor: not-allowed; }
  button:disabled:hover { background: var(--paper); color: var(--ink); border-color: var(--rule); }
  .env-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 18px 24px; }
  .env-item .label { font-family: 'JetBrains Mono', monospace; font-size: 10px;
                     letter-spacing: .2em; color: var(--ink-soft); text-transform: uppercase; }
  .env-item .value { font-family: 'Cormorant Garamond', serif; font-weight: 500;
                     font-size: 34px; line-height: 1.1; color: var(--ink); margin-top: 4px; }
  .env-item .value .unit { font-size: 14px; color: var(--ink-soft); font-weight: 400; margin-left: 2px; }
  .servo-row { margin-top: 18px; padding-top: 18px; border-top: 1px solid var(--rule); }
  .servo-row label { display: flex; justify-content: space-between; align-items: baseline;
                     font-family: 'JetBrains Mono', monospace; font-size: 10px;
                     letter-spacing: .2em; color: var(--ink-soft); text-transform: uppercase;
                     margin-bottom: 8px; }
  .servo-row .v { font-family: 'Cormorant Garamond', serif; font-size: 20px;
                  color: var(--ink); font-style: italic; letter-spacing: 0; }
  input[type=range] { width: 100%; -webkit-appearance: none; appearance: none; background: transparent; }
  input[type=range]::-webkit-slider-runnable-track { height: 4px; background: var(--rule); }
  input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; appearance: none;
                                            width: 14px; height: 14px; border-radius: 50%;
                                            background: var(--moss); border: 2px solid var(--paper);
                                            margin-top: -5px; cursor: pointer; }
  input[type=range]::-moz-range-track { height: 4px; background: var(--rule); }
  input[type=range]::-moz-range-thumb { width: 14px; height: 14px; border-radius: 50%;
                                        background: var(--moss); border: 2px solid var(--paper);
                                        cursor: pointer; }
  .camera-card { padding: 18px; }
  .camera-frame { width: 100%; aspect-ratio: 16/9; background: var(--paper-3);
                  border: 1px solid var(--rule); display: flex;
                  align-items: center; justify-content: center;
                  overflow: hidden; position: relative; }
  .camera-frame img { width: 100%; height: 100%; object-fit: cover; }
  .camera-frame .none { font-family: 'Cormorant Garamond', serif; font-style: italic;
                        color: var(--ink-soft); font-size: 16px; }
  .camera-caption { margin-top: 10px; font-family: 'Cormorant Garamond', serif;
                    font-style: italic; color: var(--ink-soft); font-size: 14px;
                    text-align: center; }
  .chart-card { padding: 24px 28px 26px; }
  #chart { width: 100% !important; height: 280px !important; }
  .events { list-style: none; padding: 0; max-height: 260px; overflow-y: auto; }
  .events li { padding: 8px 0; border-bottom: 1px dashed var(--rule);
               display: grid; grid-template-columns: 80px 100px 1fr;
               gap: 14px; align-items: baseline;
               font-family: 'Crimson Pro', serif; font-size: 14px; }
  .events li:last-child { border-bottom: none; }
  .events .t { font-family: 'JetBrains Mono', monospace; font-size: 10px;
               color: var(--ink-soft); letter-spacing: .1em; }
  .events .k { font-family: 'JetBrains Mono', monospace; font-size: 10px;
               text-transform: uppercase; letter-spacing: .14em; color: var(--moss); }
  .events .k.alarm_set, .events .k.disconnect, .events .k.command_failed { color: var(--terra); }
  .events .d { color: var(--ink-2); font-style: italic; }
  footer { margin-top: 50px; padding-top: 24px; text-align: center; color: var(--ink-soft);
           font-family: 'Cormorant Garamond', serif; font-style: italic; font-size: 14px; }
  footer .orn { color: var(--gold); margin: 0 10px; }
  .stale { opacity: 0.45; transition: opacity .3s; }
</style>
</head>
<body>
<div class="wrap">

  <header>
    <div class="conn" id="conn">
      <span class="dot"></span><span id="connText">connecting</span>
    </div>
    <h1>Hortus <em>Vigilis</em></h1>
    <div class="subtitle">A live record of the small circular garden</div>
    <div class="meta-bar">
      <span id="dateLabel">—</span>·<span id="clockLabel">—:—:—</span>·<span id="uptimeLabel">—</span>
    </div>
  </header>

  <div class="divider">
    <span></span>
    <svg viewBox="0 0 24 24" fill="currentColor"><path d="M12 2c-1 4-4 6-8 6 0 6 4 12 8 14 4-2 8-8 8-14-4 0-7-2-8-6z"/></svg>
    <span></span>
  </div>

  <div class="row zones" id="zoneCards">
    {% for z in zones %}
    <article class="card zone" data-zone="{{ loop.index0 }}">
      <div class="card-label">Specimen № {{ loop.index }}</div>
      <h2>{{ z }}</h2>
      <div class="moisture"><span class="val">—</span><span class="pct">%</span></div>
      <span class="status-tag">tranquil</span>
      <div class="zone-meta">
        <span class="failtxt">fails · 0</span>
        <span class="lastpump">awaiting first cycle</span>
      </div>
      <div class="actions">
        <button class="btn-water">Irrigate</button>
        <button class="btn-clear warn" disabled>Clear Alarm</button>
      </div>
    </article>
    {% endfor %}
  </div>

  <div class="row two-up">
    <article class="card">
      <div class="card-label">Atmospheric Conditions</div>
      <h2>The <em>Air</em> About Them</h2>
      <div class="env-grid">
        <div class="env-item">
          <div class="label">Temperature</div>
          <div class="value"><span id="envTemp">—</span><span class="unit">°C</span></div>
        </div>
        <div class="env-item">
          <div class="label">Humidity</div>
          <div class="value"><span id="envHum">—</span><span class="unit">%</span></div>
        </div>
        <div class="env-item">
          <div class="label">Luminosity</div>
          <div class="value"><span id="envLight">—</span><span class="unit">%</span></div>
        </div>
        <div class="env-item">
          <div class="label">Shading Sheet</div>
          <div class="value"><span id="envServo">—</span><span class="unit">°</span></div>
        </div>
      </div>
      <div class="servo-row">
        <label>Manual Shade Override <span class="v" id="servoLbl">—</span></label>
        <input type="range" id="servoSlider" min="0" max="180" value="0">
      </div>
    </article>

    <article class="card camera-card">
      <div class="card-label">Visual Record</div>
      <h2 style="margin-bottom: 14px;">The <em>Glasshouse</em></h2>
      <div class="camera-frame">
        <img id="camImg" alt="" style="display:none">
        <div id="camNone" class="none">No camera detected</div>
      </div>
      <div class="camera-caption" id="camCaption">—</div>
    </article>
  </div>

  <article class="card chart-card" style="margin-top: 24px;">
    <div class="card-label">Moisture Chronicle · Last 24 Hours</div>
    <canvas id="chart"></canvas>
  </article>

  <article class="card" style="margin-top: 24px; padding: 22px 26px;">
    <div class="card-label">Journal of Events</div>
    <h2 style="margin-bottom: 14px;">Recent <em>Happenings</em></h2>
    <ul class="events" id="events">
      <li><span class="t">—</span><span class="k">—</span><span class="d">waiting for the first transmission</span></li>
    </ul>
  </article>

  <footer>
    <span class="orn">❦</span> Drawn in living ink by the orchard's instruments <span class="orn">❦</span>
  </footer>

</div>

<script>
const ZONES        = {{ zones | tojson }};
const API_REQUIRED = {{ api_required | tojson }};
let chart = null;

const $  = (q, root=document) => root.querySelector(q);
const $$ = (q, root=document) => Array.from(root.querySelectorAll(q));

function getApiKey() {
  if (!API_REQUIRED) return null;
  let k = localStorage.getItem('plant_api_key');
  if (!k) {
    k = window.prompt('This dashboard requires an API key. Enter it now:');
    if (k) localStorage.setItem('plant_api_key', k);
  }
  return k;
}

function fmtTime(epoch_s) {
  if (!epoch_s) return "—";
  const d = new Date(epoch_s * 1000);
  return d.toLocaleTimeString('en-GB', { hour: '2-digit', minute: '2-digit' });
}
function setText(el, v) { if (el && el.textContent !== String(v)) el.textContent = v; }

function updateClock() {
  const d = new Date();
  setText($('#clockLabel'), d.toLocaleTimeString('en-GB'));
  setText($('#dateLabel'),
    d.toLocaleDateString('en-GB', { weekday:'long', day:'numeric', month:'long', year:'numeric' }));
}
setInterval(updateClock, 1000); updateClock();

async function sendCommand(body) {
  const headers = {'Content-Type': 'application/json'};
  const key = getApiKey();
  if (key) headers['X-API-Key'] = key;
  try {
    const r = await fetch('/api/command', {
      method: 'POST', headers, body: JSON.stringify(body)
    });
    if (r.status === 401) {
      localStorage.removeItem('plant_api_key');
      window.alert('API key rejected. You will be prompted again on next action.');
    } else if (r.status === 429) {
      const j = await r.json().catch(() => ({}));
      window.alert(j.error || 'Cooldown active; please wait.');
    } else if (r.status === 503) {
      const j = await r.json().catch(() => ({}));
      window.alert(j.error || 'Robot is offline; command not queued.');
    } else if (!r.ok) {
      console.warn('command failed', await r.text());
    }
  } catch (e) { console.error(e); }
  setTimeout(fetchStatus, 250);
}

document.addEventListener('click', e => {
  const btn = e.target.closest('button'); if (!btn) return;
  const card = btn.closest('.zone'); if (!card) return;
  const zone = Number(card.dataset.zone);
  if (btn.classList.contains('btn-water'))  sendCommand({cmd:'water', zone, ms:3000});
  if (btn.classList.contains('btn-clear'))  sendCommand({cmd:'clear_alarm', zone});
});

let servoTimer = null;
$('#servoSlider').addEventListener('input', e => {
  const ang = Number(e.target.value);
  setText($('#servoLbl'), ang + '°');
  clearTimeout(servoTimer);
  servoTimer = setTimeout(() => sendCommand({cmd:'set_servo', angle: ang}), 250);
});

async function fetchStatus() {
  try {
    const r = await fetch('/api/status');
    if (!r.ok) throw new Error(r.statusText);
    applyStatus(await r.json());
  } catch (e) {
    setConn(false, 'offline');
    document.body.classList.add('stale');
  }
}

function setConn(online, text) {
  const c = $('#conn');
  c.classList.toggle('online', online);
  setText($('#connText'), text);
}

function applyStatus(d) {
  document.body.classList.remove('stale');
  setConn(d.connected, d.connected ? 'live · Mega online' : 'mega silent');
  const u = d.uptime_s;
  let upt = u < 60 ? u+'s' : u < 3600 ? Math.floor(u/60)+'m '+(u%60)+'s'
                                      : Math.floor(u/3600)+'h '+Math.floor((u%3600)/60)+'m';
  setText($('#uptimeLabel'), 'pi uptime ' + upt);

  const s = d.status; if (!s) return;
  $$('.zone').forEach((card, i) => {
    const m  = (s.m      && s.m[i]      != null) ? s.m[i]      : null;
    const al = (s.alarms && s.alarms[i] != null) ? s.alarms[i] : 0;
    const fl = (s.fails  && s.fails[i]  != null) ? s.fails[i]  : 0;
    const active = (s.zone === i);
    const moistEl = $('.moisture', card);
    const valEl   = $('.val', card);
    const tag     = $('.status-tag', card);
    const fails   = $('.failtxt', card);
    const clrBtn  = $('.btn-clear', card);
    const wtrBtn  = $('.btn-water', card);

    if (m === null || m === -1) {
      moistEl.classList.add('err'); valEl.textContent = 'sensor lost';
      $('.pct', card).style.display = 'none';
    } else {
      moistEl.classList.remove('err'); valEl.textContent = m;
      $('.pct', card).style.display = '';
    }
    tag.classList.remove('busy', 'alarm');
    if (al) { tag.classList.add('alarm'); tag.textContent = 'alarmed'; }
    else if (active && s.state && s.state !== 'IDLE') {
      tag.classList.add('busy');
      tag.textContent = (s.state === 'RUNNING') ? 'irrigating'
                       : (s.state === 'VERIFYING') ? 'verifying'
                       : String(s.state).toLowerCase().replace('_',' ');
    } else { tag.textContent = 'tranquil'; }
    setText(fails, 'fails · ' + fl);
    clrBtn.disabled = !al;
    wtrBtn.disabled = (s.state !== 'IDLE') || al;
  });

  setText($('#envTemp'),  (s.temp  != null && s.temp  !== -99) ? Number(s.temp).toFixed(1) : '—');
  setText($('#envHum'),   (s.hum   != null && s.hum   !== -1)  ? s.hum  : '—');
  setText($('#envLight'), (s.l     != null) ? s.l    : '—');
  const sv = $('#servoSlider').value;
  setText($('#envServo'), sv);
  setText($('#servoLbl'), sv + '°');
}

// ----- Events list -----
// IMPORTANT: build with createElement / textContent. Never interpolate
// user/device-supplied strings into innerHTML — fields like e.details
// can contain text echoed from the Mega or exception messages, and a
// past version of this dashboard was vulnerable to stored XSS.
async function fetchEvents() {
  try {
    const r  = await fetch('/api/events?limit=25');
    const ev = await r.json();
    const ul = $('#events');
    if (!ev || !ev.length) return;
    ul.replaceChildren();
    for (const e of ev) {
      const li = document.createElement('li');

      const t = document.createElement('span');
      t.className = 't';
      t.textContent = fmtTime(e.ts);

      const k = document.createElement('span');
      // Whitelist the kind name to safe characters before using it as a
      // CSS class (so a malicious value can't break out of the class attr).
      const safeKind = String(e.kind || '').replace(/[^a-z0-9_]/gi, '');
      k.className = 'k ' + safeKind;
      k.textContent = String(e.kind || '').replace('_', ' ');

      const d = document.createElement('span');
      d.className = 'd';
      let dtxt = String(e.details || '');
      if (e.zone != null) dtxt += ' · Z' + (Number(e.zone) + 1);
      d.textContent = dtxt;

      li.append(t, k, d);
      ul.append(li);
    }
  } catch (e) { /* keep last good list */ }
}

async function fetchHistory() {
  try {
    const r = await fetch('/api/history?hours=24');
    const rows = await r.json();
    if (!rows.length) return;
    const labels = rows.map(r => new Date(r.ts*1000));
    const ds = (k, color) => ({
      label: k, data: rows.map(r => r[k]),
      borderColor: color, backgroundColor: color + '22',
      tension: 0.35, pointRadius: 0, borderWidth: 2, spanGaps: true,
    });
    if (chart) {
      chart.data.labels = labels;
      chart.data.datasets.forEach((d,i) => d.data = rows.map(r => r['m'+i]));
      chart.update('none');
      return;
    }
    chart = new Chart($('#chart'), {
      type: 'line',
      data: { labels,
        datasets: [
          ds('m0', '#3d5a3d'),
          ds('m1', '#a85d3c'),
          ds('m2', '#b08d57'),
        ].map((d,i) => Object.assign(d, { label: ZONES[i] }))
      },
      options: {
        responsive: true, maintainAspectRatio: false,
        plugins: { legend: { position: 'bottom',
          labels: { color: '#4a3d2a',
                    font: { family: 'Crimson Pro', size: 13, style: 'italic' },
                    boxWidth: 22, boxHeight: 2 } } },
        scales: {
          x: { type: 'time', time: { unit: 'hour', displayFormats: { hour: 'HH:mm' } },
               grid: { color: '#d9cba6', lineWidth: 0.5 },
               ticks: { color: '#786648', font: { family: 'JetBrains Mono', size: 10 } } },
          y: { min: 0, max: 100,
               grid: { color: '#d9cba6', lineWidth: 0.5 },
               ticks: { color: '#786648', font: { family: 'JetBrains Mono', size: 10 },
                        callback: v => v + '%' } }
        }
      }
    });
  } catch (e) { console.warn('chart', e); }
}

function snapshotUrl() {
  let url = '/snapshot.jpg?t=' + Date.now();
  if (API_REQUIRED) {
    const k = getApiKey();
    if (k) url += '&api_key=' + encodeURIComponent(k);
  }
  return url;
}

function refreshCamera() {
  const img = $('#camImg'); const none = $('#camNone');
  const url = snapshotUrl();
  const probe = new Image();
  probe.onload = () => {
    img.src = url; img.style.display = ''; none.style.display = 'none';
    setText($('#camCaption'), 'Captured ' + new Date().toLocaleTimeString('en-GB'));
  };
  probe.onerror = () => {
    img.style.display = 'none'; none.style.display = '';
    setText($('#camCaption'), '—');
  };
  probe.src = url;
}

fetchStatus();    setInterval(fetchStatus,  3000);
fetchEvents();    setInterval(fetchEvents, 10000);
fetchHistory();   setInterval(fetchHistory, 60000);
refreshCamera();  setInterval(refreshCamera, 15000);
</script>
</body>
</html>"""


# ======================== STARTUP / SHUTDOWN =========================
def check_dialout_membership() -> None:
    """Warn (don't crash) if the current user can't open serial ports.

    Uses pwd.getpwuid instead of os.getlogin(), which raises OSError in
    non-tty contexts (systemd service, cron, ssh without controlling
    terminal). Only catches the specific errors we expect.
    """
    if not PWD_AVAILABLE:
        return
    try:
        uid = os.getuid()
        if uid == 0:
            return  # root has access regardless
        username = pwd.getpwuid(uid).pw_name
        my_groups = {grp.getgrgid(g).gr_name for g in os.getgroups()}
        if "dialout" not in my_groups:
            log.warning(f"User '{username}' is not in the 'dialout' group; "
                        "opening /dev/ttyACM* may fail with Permission denied.")
            log.warning(f"Fix:  sudo usermod -aG dialout {username}  "
                        "(then log out and back in)")
    except KeyError:
        # pwd entry missing for current uid (unusual but possible)
        log.debug("Could not resolve current user; skipping dialout check.")
    except OSError as e:
        log.debug(f"OS error during dialout check: {e}")


def graceful_shutdown(signum, frame) -> None:
    log.info(f"Signal {signum} received; flushing and shutting down.")
    shutdown_event.set()
    # The DB writer drains its own queue when shutdown_event is set.
    # Wait briefly for it to finish, but no longer than 3s so SIGTERM
    # under systemd doesn't escalate to SIGKILL.
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not db_event_queue.empty():
        time.sleep(0.05)
    sys.exit(0)


def periodic_pruner() -> None:
    while not shutdown_event.is_set():
        if shutdown_event.wait(3600):
            break
        db_prune()


def main() -> None:
    if sys.version_info < (3, 9):
        print("Python 3.9+ required.", file=sys.stderr)
        sys.exit(1)

    signal.signal(signal.SIGINT,  graceful_shutdown)
    signal.signal(signal.SIGTERM, graceful_shutdown)

    check_dialout_membership()
    init_db()

    # Start the DB writer FIRST so the boot event lands cleanly.
    threading.Thread(target=db_writer_loop,  daemon=True, name="DBWriter").start()
    enqueue_db_event("boot", details="Pi controller started")

    available = list(list_ports.comports())
    if available:
        log.info(f"Found {len(available)} serial port(s):")
        for p in available:
            vid = f"{p.vid:04x}" if p.vid is not None else "----"
            pid = f"{p.pid:04x}" if p.pid is not None else "----"
            log.info(f"  · {p.device:20s} VID:PID {vid}:{pid}  {p.description}")
    else:
        log.warning("No serial ports detected at startup. "
                    "Make sure the Arduino is plugged in.")

    threading.Thread(target=periodic_pruner, daemon=True, name="Pruner").start()
    SerialBridge().start()

    log.info(f"Dashboard: http://{HTTP_HOST}:{HTTP_PORT}")
    if HTTP_HOST == "0.0.0.0":
        log.warning("Bound to 0.0.0.0 (LAN-accessible). "
                    "Make sure PLANT_API_KEY is set if exposed.")
    log.info(f"Camera:    {'enabled (init in background)' if camera.enabled else 'disabled (OpenCV not installed)'}")
    if API_KEY:
        log.info("API auth:  ENABLED (X-API-Key required for /api/command "
                 "and /snapshot.jpg)")
    else:
        log.info("API auth:  disabled  (set PLANT_API_KEY env var to enable)")

    app.run(host=HTTP_HOST, port=HTTP_PORT,
            debug=False, use_reloader=False, threaded=True)


if __name__ == "__main__":
    main()
