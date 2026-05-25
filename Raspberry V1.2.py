#!/usr/bin/env python3
# =====================================================================
#   PLANT CARE PI CONTROLLER  -  v1.2
#   Raspberry Pi supervisor for the Mega-based plant care robot.
#
#   v1.2 — Audit fixes:
#     • Defensive array indexing (no IndexError on short/missing arrays)
#     • Robust API input validation (400 instead of 500 on bad types)
#     • Non-blocking command queue (HTTP can't hang)
#     • Responsive shutdown (shutdown_event.wait instead of time.sleep)
#     • SQLite WAL mode + busy_timeout for concurrency
#     • Dedicated DB writer thread (Serial thread no longer blocks on disk)
#     • Optional API key auth (env: PLANT_API_KEY)
#     • time.monotonic() for elapsed-time math (NTP-safe)
#     • History endpoint downsampling
#     • try/except on all DB read paths
#     • Dialout group sanity check at startup
#     • from __future__ import annotations  -> works on Python 3.9
#     • Werkzeug request log silenced
#     • Camera sepia filter removed (was misleading for leaf diagnostics)
#
#   v1.1 — Auto-detection of Arduino serial port.
#
#   Setup on Raspberry Pi OS (Bookworm, Python 3.11+) or Bullseye (3.9+):
#     sudo apt update
#     sudo apt install -y python3-pip python3-venv python3-opencv
#     sudo usermod -aG dialout $USER     # then log out and back in
#     python3 -m venv .venv
#     source .venv/bin/activate
#     pip install -r requirements.txt
#     # Optional: enable API auth for command endpoint
#     # export PLANT_API_KEY="some-long-random-string"
#     python3 plant_controller.py
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

import serial
from serial.tools import list_ports
from flask import Flask, Response, jsonify, render_template_string, request

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

CAMERA_INDEX             = 0
CAMERA_WIDTH             = 1280
CAMERA_HEIGHT            = 720
CAMERA_JPEG_Q            = 80
CAMERA_CACHE_S           = 1.0

DB_PATH                  = Path(__file__).resolve().parent / "plant_history.db"
LOG_RETAIN_DAYS          = 30
HTTP_HOST                = "0.0.0.0"
HTTP_PORT                = 5000

# Optional API key auth for /api/command. Disabled if env var not set.
# To enable:   export PLANT_API_KEY="some-long-random-string"
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
    """Convert sentinel values (-1, -99, or large negatives) to None.

    The Mega uses -1 / -99 to flag a missing/broken sensor. Storing
    those as numbers would distort charts; turning them into NULL
    keeps the time series honest.
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
            connected = (
                self.last_seen_mono > 0
                and (time.monotonic() - self.last_seen_mono) < CONNECTED_TIMEOUT_S
            )
            return {
                "status":     self.last_status,
                "last_seen":  self.last_seen_wall,
                "connected":  connected,
                "uptime_s":   int(time.monotonic() - self.startup_mono),
                "recent_log": list(self.mega_log)[-20:],
            }


state = State()
command_queue:   "queue.Queue[dict]"  = queue.Queue(maxsize=COMMAND_QUEUE_SIZE)
db_event_queue:  "queue.Queue[tuple]" = queue.Queue(maxsize=DB_EVENT_QUEUE_SIZE)
shutdown_event = threading.Event()


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


def db_insert_status(data: dict) -> None:
    try:
        m = _pad3(data.get("m"))
        f = _pad3(data.get("fails"), 0)
        a = _pad3(data.get("alarms"), 0)
        conn = sqlite3.connect(str(DB_PATH))
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
        conn.close()
    except Exception as e:
        log.error(f"db_insert_status failed: {e}")


def db_insert_event(kind: str, zone: int | None = None,
                    details: str = "") -> None:
    try:
        conn = sqlite3.connect(str(DB_PATH))
        conn.execute(
            "INSERT INTO events(ts,kind,zone,details) VALUES (?,?,?,?)",
            (int(time.time()), kind, zone, details))
        conn.commit()
        conn.close()
    except Exception as e:
        log.error(f"db_insert_event failed: {e}")


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
# could mean missing a Mega status frame). Instead it enqueues, and a
# dedicated thread drains the queue.

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
    """Consume the DB queue. One write at a time, serialized."""
    while not shutdown_event.is_set():
        try:
            kind, args = db_event_queue.get(timeout=1.0)
        except queue.Empty:
            continue
        try:
            if kind == "status":
                db_insert_status(*args)
            elif kind == "event":
                db_insert_event(*args)
        except Exception as e:
            log.error(f"db_writer_loop error processing {kind}: {e}")


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
            # Bug #5 fix: log the disconnect that send() induces.
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
                    # Bug #4 fix: wait() lets us exit fast on shutdown
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

            # Drain outbound command queue
            try:
                while True:
                    cmd = command_queue.get_nowait()
                    self.send(cmd)
            except queue.Empty:
                pass


# ======================== CAMERA =====================================
class Camera:
    """Thin OpenCV wrapper. Returns latest JPEG bytes on demand."""

    def __init__(self) -> None:
        self.cap          = None
        self.lock         = threading.Lock()
        self.last_jpeg    = None
        self.last_capture = 0.0
        self.enabled      = CV2_AVAILABLE
        if self.enabled:
            self._try_open()

    def _try_open(self) -> None:
        try:
            self.cap = cv2.VideoCapture(CAMERA_INDEX)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            if not self.cap.isOpened():
                log.warning("Camera failed to open; disabling video features.")
                self.cap = None
        except Exception as e:
            log.warning(f"Camera init error: {e}")
            self.cap = None

    def snapshot(self) -> bytes | None:
        if not self.enabled or self.cap is None:
            return None
        with self.lock:
            if self.last_jpeg and (time.monotonic() - self.last_capture) < CAMERA_CACHE_S:
                return self.last_jpeg
            ok, frame = self.cap.read()
            if not ok:
                try: self.cap.release()
                except Exception: pass
                self._try_open()
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


@app.before_request
def _auth_gate():
    """Optional API key check, only on the mutating command endpoint."""
    if not API_KEY:
        return None
    if request.path != "/api/command":
        return None
    key = request.headers.get("X-API-Key") or request.args.get("api_key")
    if key != API_KEY:
        return jsonify(error="unauthorized; X-API-Key header required"), 401
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
    snap["camera_enabled"] = camera.enabled and camera.cap is not None
    snap["now"]            = time.time()
    return jsonify(snap)


@app.route("/api/history")
def api_history():
    # Flask coerces type; falls back to default on parse failure.
    hours = request.args.get("hours", default=24, type=int) or 24
    hours = max(1, min(hours, 720))
    rows  = db_history(hours)
    # Downsample so 30-day requests don't drown the browser.
    if len(rows) > HISTORY_MAX_POINTS:
        step = max(1, len(rows) // HISTORY_MAX_POINTS)
        rows = rows[::step]
    return jsonify(rows)


@app.route("/api/events")
def api_events():
    limit = request.args.get("limit", default=30, type=int) or 30
    limit = max(1, min(limit, 200))
    return jsonify(db_events(limit))


@app.route("/api/command", methods=["POST"])
def api_command():
    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        return jsonify(error="JSON object body required"), 400

    cmd = payload.get("cmd")

    def queue_cmd(c: dict):
        try:
            command_queue.put_nowait(c)
            return jsonify(queued=True)
        except queue.Full:
            return jsonify(error="command queue full; Mega may be offline"), 503

    # Bug #2 fix: typed parsing wrapped in try/except so bad input -> 400
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
# Botanical field journal aesthetic. Same as v1.1, except:
#  - sepia camera filter removed (don't mislead leaf-colour diagnosis)
#  - sends X-API-Key header on commands when api_required is True
#  - prompts once for the key and remembers it in localStorage

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
  /* sepia filter removed in v1.2 — was distorting leaf colour diagnosis */
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
  .events .k.alarm_set, .events .k.disconnect { color: var(--terra); }
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
      localStorage.removeItem('plant_api_key');  // bad key -> forget it
      window.alert('API key rejected. You will be prompted again on next action.');
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
                       : s.state.toLowerCase().replace('_',' ');
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

async function fetchEvents() {
  try {
    const r  = await fetch('/api/events?limit=25');
    const ev = await r.json();
    const ul = $('#events');
    if (!ev.length) return;
    ul.innerHTML = ev.map(e => `
      <li>
        <span class="t">${fmtTime(e.ts)}</span>
        <span class="k ${e.kind}">${e.kind.replace('_',' ')}</span>
        <span class="d">${e.details || ''} ${e.zone != null ? '· Z'+(e.zone+1) : ''}</span>
      </li>`).join('');
  } catch (e) { }
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

function refreshCamera() {
  const img = $('#camImg'); const none = $('#camNone');
  const url = '/snapshot.jpg?t=' + Date.now();
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
    """Warn (don't crash) if the current user can't open serial ports."""
    try:
        uid = os.getuid()
        my_groups = {grp.getgrgid(g).gr_name for g in os.getgroups()}
        # Also look up groups that list this user explicitly
        for g in grp.getgrall():
            if os.getlogin() in g.gr_mem or uid == 0:
                my_groups.add(g.gr_name)
        if "dialout" not in my_groups and uid != 0:
            log.warning("User not in 'dialout' group; opening /dev/ttyACM* "
                        "may fail with Permission denied.")
            log.warning("Fix:  sudo usermod -aG dialout $USER  "
                        "(then log out and back in)")
    except Exception:
        # Best-effort only; not all platforms expose grp the same way.
        pass


def graceful_shutdown(signum, frame) -> None:
    log.info(f"Signal {signum} received; flushing and shutting down.")
    shutdown_event.set()
    # Brief grace period to let the DB writer drain its queue.
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline and not db_event_queue.empty():
        time.sleep(0.05)
    sys.exit(0)


def periodic_pruner() -> None:
    while not shutdown_event.is_set():
        # wait() returns True if event got set; we exit immediately then.
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

    # Background workers
    threading.Thread(target=db_writer_loop,  daemon=True, name="DBWriter").start()
    threading.Thread(target=periodic_pruner, daemon=True, name="Pruner").start()
    SerialBridge().start()

    log.info(f"Dashboard: http://{HTTP_HOST}:{HTTP_PORT}")
    log.info(f"Camera:    {'enabled' if camera.enabled else 'disabled (OpenCV not installed)'}")
    if API_KEY:
        log.info("API auth:  ENABLED (X-API-Key required for /api/command)")
    else:
        log.info("API auth:  disabled  (set PLANT_API_KEY env var to enable)")

    app.run(host=HTTP_HOST, port=HTTP_PORT,
            debug=False, use_reloader=False, threaded=True)


if __name__ == "__main__":
    main()
