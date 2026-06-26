#!/usr/bin/env python3
# =====================================================================
#   PLANT CARE PI CONTROLLER  -  v1.6
#   Raspberry Pi supervisor for the Mega-based plant care robot.
#
#   v1.6 — Fourth-pass fixes (this revision):
#     • Zone rate-limit rollback uses a (prev, ours) token and only
#       rolls back if our exact stamp is still in the dict. v1.5's
#       `current > prev` check was a latent footgun: if anyone ever
#       grew the path between acquire and rollback to span COOLDOWN,
#       it could clobber a legitimate later reservation.
#     • SerialBridge.open() suppresses the "no port found" warning
#       after the first instance, re-logging only on state transition.
#       Avoids ~8000 redundant log lines per day when the Mega is
#       unplugged under systemd/journald.
#
#   v1.5 — third-pass: atomic rate-limit reserve+rollback, /api/*
#          and /snapshot.jpg auth, periodic camera retry, blob-URL
#          delayed revoke, refreshCamera 401 handling, longer
#          shutdown join, _spawn_reopen enabled-guard.
#   v1.4 — second-pass: graceful shutdown via join(), tentative
#          rate-limit, light sentinel filter, env-var tolerance,
#          snapshot via X-API-Key + blob, type="button", wal_checkpoint.
#   v1.3 — first-pass: pwd.getpwuid for non-tty contexts, XSS fix in
#          events list, DB writer drain-on-shutdown, command_failed
#          events, loopback default HTTP host, CSRF check, persistent
#          DB writer connection, background camera init, bucket
#          downsampling, per-zone water cooldown, 503 when Mega offline.
#   v1.2 — earlier audit fixes.
#   v1.1 — auto-detection of Arduino serial port.
#
#   Setup on Raspberry Pi OS (Bookworm 3.11+ or Bullseye 3.9+):
#     sudo apt update
#     sudo apt install -y python3-pip python3-venv python3-opencv
#     sudo usermod -aG dialout $USER     # then log out and back in
#     python3 -m venv .venv
#     source .venv/bin/activate
#     pip install -r requirements.txt
#     # Optional: enable API auth for all data endpoints
#     # export PLANT_API_KEY="some-long-random-string"
#     # Optional: bind LAN-wide (default is loopback only)
#     # export PLANT_HTTP_HOST="0.0.0.0"
#     python3 plant_controller.py
#
#   For long-running deployment, prefer waitress over the dev server:
#     pip install waitress
#     # then edit main() to use serve(app, host=..., port=..., threads=4)
# =====================================================================

from __future__ import annotations

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

try:
    import pwd
    PWD_AVAILABLE = True
except ImportError:
    PWD_AVAILABLE = False

try:
    import cv2
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False

# Species + health classifier (TFLite model with OpenCV heuristic fallback).
# Lives in plant_classifier.py alongside this file. If that module is missing
# or fails to import, the controller still boots and classification is disabled.
try:
    from plant_classifier import build_classifier
except Exception:
    build_classifier = None

# ======================== CONFIGURATION ==============================
SERIAL_PORT_OVERRIDE: str | None = None

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

CONNECTED_TIMEOUT_S      = 15
DB_STATUS_THROTTLE_S     = 30
HISTORY_MAX_POINTS       = 500
DB_EVENT_QUEUE_SIZE      = 500
COMMAND_QUEUE_SIZE       = 32
ZONE_COMMAND_COOLDOWN_S  = 10

SHUTDOWN_JOIN_TIMEOUT_S  = 6.0
CAMERA_RETRY_INTERVAL_S  = 60

CAMERA_INDEX             = 0
CAMERA_WIDTH             = 1280
CAMERA_HEIGHT            = 720
CAMERA_JPEG_Q            = 80
CAMERA_CACHE_S           = 1.0

DB_PATH                  = Path(__file__).resolve().parent / "plant_history.db"
LOG_RETAIN_DAYS          = 30


def _env_str(name: str, default: str) -> str:
    return os.environ.get(name) or default


def _env_port(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if not raw:
        return default
    try:
        v = int(raw)
        if not 1 <= v <= 65535:
            raise ValueError("out of range 1..65535")
        return v
    except ValueError as e:
        print(f"[WARN] Invalid {name}={raw!r} ({e}); using {default}",
              file=sys.stderr)
        return default


HTTP_HOST                = _env_str("PLANT_HTTP_HOST", "127.0.0.1")
HTTP_PORT                = _env_port("PLANT_HTTP_PORT", 5000)

API_KEY: str | None      = os.environ.get("PLANT_API_KEY") or None

ZONE_NAMES = ["Low-Light Herbs (Mint)",
              "Bright-Light Fruit (Cherry Tomato)",
              "Balanced Herbs (Basil)"]

# ---- Plant classifier (species + health) ----------------------------
# Drop a TFLite image-classification model + a labels file next to this
# script (or point the env vars elsewhere). PlantVillage MobileNet works
# well; labels are one-per-line in model-output order, e.g.
#   Tomato___Late_blight
#   Tomato___healthy
# With no model present, the classifier falls back to a colour-based
# foliage-health estimate (no species name). Set PLANT_MODEL_NORM=signed
# if your model expects [-1,1] inputs instead of the default [0,1].
CLASSIFIER_MODEL_PATH   = _env_str("PLANT_MODEL_PATH",
                                   str(Path(__file__).resolve().parent / "model.tflite"))
CLASSIFIER_LABELS_PATH  = _env_str("PLANT_LABELS_PATH",
                                   str(Path(__file__).resolve().parent / "labels.txt"))
CLASSIFIER_NORM         = _env_str("PLANT_MODEL_NORM", "unit")   # "unit" (/255) or "signed" (/127.5-1)
CLASSIFY_MIN_INTERVAL_S = 4      # guard the Pi CPU against rapid re-triggers

# ======================== LOGGING ====================================
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
log = logging.getLogger("plant-pi")
logging.getLogger("werkzeug").setLevel(logging.WARNING)

# ======================== UTILITIES ==================================
def _pad3(arr, fill=None) -> list:
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
    """
    if v is None:
        return None
    if v == -1 or v == -99:
        return None
    if isinstance(v, float) and v < -50:
        return None
    return v


# ======================== SHARED STATE ===============================
class State:
    """Thread-safe shared state between the serial thread and Flask."""

    def __init__(self) -> None:
        self._lock         = threading.Lock()
        self.last_status   = None
        self.last_seen_mono = 0.0
        self.last_seen_wall = 0.0
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

db_writer_thread: threading.Thread | None = None


# ---- Zone rate-limit (atomic reserve + tokenised rollback) -----------
# Evolution of this section across revisions:
#   v1.3: atomic check-and-record under one lock. Safe against
#         concurrent abuse, but a queue-full on enqueue locked the
#         user out for COOLDOWN seconds.
#   v1.4: split check/record. Fixed the lockout, opened a TOCTOU race
#         where two HTTP threads could both pass the gate concurrently.
#   v1.5: reserve + rollback under separate locks. Better, but the
#         rollback condition was `current > prev`, which doesn't
#         actually prove "our reservation is still the latest".
#         No symptom under current code paths (acquire→rollback is
#         microseconds), but a footgun for future changes.
#   v1.6 (here): rollback receives a token that contains OUR exact
#         stamp. It only undoes the reservation if that stamp is
#         still there — a later legitimate reservation by another
#         thread is left untouched.
_zone_last_water_mono: dict[int, float] = {}
_zone_rate_lock = threading.Lock()


def _zone_rate_try_acquire(zone: int) -> tuple[float, float] | None:
    """Atomically reserve a watering slot for *zone*.

    Returns a (prev, ours) token on success — caller passes this back
    to _zone_rate_rollback() if the downstream enqueue failed.
    Returns None if the zone is still on cooldown.
    """
    with _zone_rate_lock:
        now = time.monotonic()
        prev = _zone_last_water_mono.get(zone, 0.0)
        if now - prev < ZONE_COMMAND_COOLDOWN_S:
            return None
        _zone_last_water_mono[zone] = now
        return (prev, now)


def _zone_rate_rollback(zone: int, token: tuple[float, float]) -> None:
    """Undo a reservation if and only if it is still ours.

    The token from try_acquire carries the exact monotonic timestamp
    we wrote. We compare against the dict — if it doesn't match, a
    later request has already legitimately reserved the slot and we
    must not clobber it. Equality on float monotonic stamps is safe
    here because we put the value in ourselves; nothing else can
    produce the same float.
    """
    prev, ours = token
    with _zone_rate_lock:
        if _zone_last_water_mono.get(zone) == ours:
            _zone_last_water_mono[zone] = prev


# ======================== DATABASE ===================================
def init_db() -> None:
    conn = sqlite3.connect(str(DB_PATH))
    cur  = conn.cursor()
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
        CREATE TABLE IF NOT EXISTS classifications (
            id           INTEGER PRIMARY KEY AUTOINCREMENT,
            ts           INTEGER NOT NULL,
            zone         INTEGER,
            species      TEXT,
            condition    TEXT,
            health       TEXT,
            health_score INTEGER,
            confidence   REAL,
            source       TEXT,
            details      TEXT
        );
        CREATE INDEX IF NOT EXISTS idx_class_ts ON classifications(ts DESC);
    """)
    conn.commit()
    conn.close()
    log.info(f"Database ready at {DB_PATH}")


def _db_insert_status_conn(conn: sqlite3.Connection, data: dict) -> None:
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


def _db_insert_classification_conn(conn: sqlite3.Connection,
                                   result: dict) -> None:
    conn.execute(
        "INSERT INTO classifications"
        "(ts,zone,species,condition,health,health_score,confidence,source,details) "
        "VALUES (?,?,?,?,?,?,?,?,?)",
        (int(result.get("ts", time.time())), result.get("zone"),
         result.get("species"), result.get("condition"),
         result.get("health"), result.get("health_score"),
         result.get("confidence"), result.get("source"),
         json.dumps(result.get("detail", {}))))
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


def db_classifications(limit: int = 10) -> list[dict]:
    try:
        conn = sqlite3.connect(str(DB_PATH))
        conn.row_factory = sqlite3.Row
        rows = conn.execute(
            "SELECT * FROM classifications ORDER BY ts DESC LIMIT ?", (limit,)
        ).fetchall()
        conn.close()
        return [dict(r) for r in rows]
    except Exception as e:
        log.error(f"db_classifications failed: {e}")
        return []


def db_prune() -> None:
    try:
        cutoff = int(time.time()) - LOG_RETAIN_DAYS * 86400
        conn = sqlite3.connect(str(DB_PATH))
        conn.execute("DELETE FROM status_log WHERE ts < ?", (cutoff,))
        conn.execute("DELETE FROM events     WHERE ts < ?", (cutoff,))
        conn.commit()
        try:
            conn.execute("PRAGMA wal_checkpoint(TRUNCATE);")
        except sqlite3.Error as e:
            log.debug(f"wal_checkpoint failed: {e}")
        conn.close()
    except Exception as e:
        log.warning(f"db_prune failed: {e}")


def enqueue_db_status(data: dict) -> None:
    try:
        db_event_queue.put_nowait(("status", (data,)))
    except queue.Full:
        log.warning("DB queue full; dropped a status row")


def enqueue_db_event(kind: str, zone: int | None = None,
                     details: str = "") -> None:
    try:
        db_event_queue.put_nowait(("event", (kind, zone, details)))
    except queue.Full:
        log.warning(f"DB queue full; dropped event {kind}")


def enqueue_db_classification(result: dict) -> None:
    try:
        db_event_queue.put_nowait(("classify", (result,)))
    except queue.Full:
        log.warning("DB queue full; dropped a classification")


def db_writer_loop() -> None:
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
                    log.info("DB writer: queue drained, exiting")
                    return
                continue
            try:
                if kind == "status":
                    _db_insert_status_conn(conn, *args)
                elif kind == "event":
                    _db_insert_event_conn(conn, *args)
                elif kind == "classify":
                    _db_insert_classification_conn(conn, *args)
            except Exception as e:
                log.error(f"db_writer error processing {kind}: {e}")
    finally:
        try:
            conn.close()
        except Exception:
            pass


# ======================== DOWNSAMPLING ===============================
def downsample_status(rows: list[dict], max_points: int) -> list[dict]:
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
    if SERIAL_PORT_OVERRIDE:
        return SERIAL_PORT_OVERRIDE

    ports = list(list_ports.comports())
    if not ports:
        return None

    for p in ports:
        if p.vid is not None and p.vid in KNOWN_USB_SERIAL_VIDS:
            pid_str = f"{p.pid:04x}" if p.pid is not None else "????"
            log.info(f"Auto-detected Arduino at {p.device} "
                     f"(VID:PID {p.vid:04x}:{pid_str} · {p.description})")
            return p.device

    for p in ports:
        if "ttyACM" in p.device or "ttyUSB" in p.device:
            log.info(f"Auto-detected serial device at {p.device} "
                     f"(no VID match · {p.description})")
            return p.device

    return None


# ======================== SERIAL BRIDGE ==============================
class SerialBridge(threading.Thread):
    def __init__(self) -> None:
        super().__init__(daemon=True, name="SerialBridge")
        self.ser              = None
        self.current_port     = None
        self.prev_state       = None
        self.prev_alarms      = [0, 0, 0]
        self.last_db_log_mono = 0.0
        # v1.6: throttle the "no port found" warning. The serial loop
        # retries every 1-10s; if the Arduino is unplugged for a day,
        # that's thousands of identical lines in the journal. We log
        # only on state transitions: once when the port disappears,
        # again when it comes back.
        self._no_port_warned  = False

    def open(self) -> bool:
        port = detect_serial_port()
        if port is None:
            if not self._no_port_warned:
                log.warning("No Arduino-compatible serial port found. "
                            "Check the USB cable, or run: ls /dev/tty*")
                self._no_port_warned = True
            self.ser = None
            self.current_port = None
            return False
        # Port is back — reset the throttle so the next disappearance
        # logs again. (detect_serial_port itself logs the "found at
        # ..." line, so no need to repeat it here.)
        self._no_port_warned = False
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

            now_mono = time.monotonic()
            if now_mono - self.last_db_log_mono >= DB_STATUS_THROTTLE_S:
                enqueue_db_status(data)
                self.last_db_log_mono = now_mono

            new_state  = data.get("state")
            new_alarms = _pad3(data.get("alarms"), 0)

            if new_state != self.prev_state and self.prev_state is not None:
                enqueue_db_event("state_change", zone=data.get("zone"),
                                 details=f"{self.prev_state} -> {new_state}")
            self.prev_state = new_state

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

            try:
                while True:
                    cmd = command_queue.get_nowait()
                    if not self.send(cmd):
                        enqueue_db_event(
                            "command_failed", zone=cmd.get("zone"),
                            details=f"send failed: {json.dumps(cmd)}")
                        break
            except queue.Empty:
                pass


# ======================== CAMERA =====================================
class Camera:
    def __init__(self) -> None:
        self.cap             = None
        self.lock            = threading.Lock()
        self.last_jpeg       = None
        self.last_capture    = 0.0
        self.enabled         = CV2_AVAILABLE
        self._reopen_pending = False
        if self.enabled:
            self._spawn_reopen()

    def _spawn_reopen(self) -> None:
        if not self.enabled:
            return
        with self.lock:
            if self._reopen_pending or self.cap is not None:
                return
            self._reopen_pending = True
        threading.Thread(target=self._try_open, daemon=True,
                         name="CameraOpen").start()

    def _try_open(self) -> None:
        new_cap = None
        try:
            new_cap = cv2.VideoCapture(CAMERA_INDEX)
            new_cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
            new_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            if not new_cap.isOpened():
                log.debug("Camera not available; will retry.")
                try: new_cap.release()
                except Exception: pass
                new_cap = None
                return
            with self.lock:
                if self.cap is not None:
                    try: new_cap.release()
                    except Exception: pass
                    new_cap = None
                    return
                self.cap = new_cap
                new_cap = None
            log.info("Camera initialized")
        except Exception as e:
            log.warning(f"Camera init error: {e}")
            if new_cap is not None:
                try: new_cap.release()
                except Exception: pass
        finally:
            with self.lock:
                self._reopen_pending = False

    def is_ready(self) -> bool:
        return self.enabled and self.cap is not None

    def snapshot(self) -> bytes | None:
        if not self.enabled:
            return None
        stale = None
        spawn = False
        with self.lock:
            if self.cap is None:
                return self.last_jpeg
            if self.last_jpeg and (time.monotonic() - self.last_capture) < CAMERA_CACHE_S:
                return self.last_jpeg
            ok, frame = self.cap.read()
            if not ok:
                try: self.cap.release()
                except Exception: pass
                self.cap = None
                stale = self.last_jpeg
                spawn = True
            else:
                ok2, buf = cv2.imencode(".jpg", frame,
                                        [cv2.IMWRITE_JPEG_QUALITY, CAMERA_JPEG_Q])
                if ok2:
                    self.last_jpeg    = buf.tobytes()
                    self.last_capture = time.monotonic()
                return self.last_jpeg
        if spawn:
            self._spawn_reopen()
        return stale


camera = Camera()

# Build the classifier once at import time. warm_up() (inside
# build_classifier) loads the TFLite model in a background thread, so this
# does not block startup; it is a no-op if no model file is present.
classifier = (build_classifier(CLASSIFIER_MODEL_PATH, CLASSIFIER_LABELS_PATH,
                               norm=CLASSIFIER_NORM)
              if build_classifier else None)


def periodic_camera_retry() -> None:
    while not shutdown_event.is_set():
        if shutdown_event.wait(CAMERA_RETRY_INTERVAL_S):
            break
        if camera.enabled and camera.cap is None:
            camera._spawn_reopen()


# ======================== FLASK APP ==================================
app = Flask(__name__)


def _same_origin() -> bool:
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
    path = request.path
    # State-changing POSTs get same-origin (CSRF) protection in addition
    # to the API-key check below.
    if path in ("/api/command", "/api/classify"):
        if not _same_origin():
            return jsonify(error="cross-origin request rejected"), 403
    if path.startswith("/api/") or path == "/snapshot.jpg":
        if not _check_api_key():
            if path == "/snapshot.jpg":
                return Response(status=401)
            return jsonify(error="unauthorized; X-API-Key required"), 401
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
            return jsonify(queued=True), 200
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

        # v1.6: atomic reserve. The token carries our exact stamp,
        # so rollback (if enqueue fails) won't clobber any later
        # legitimate reservation by another thread.
        token = _zone_rate_try_acquire(zone)
        if token is None:
            return jsonify(
                error=f"zone {zone} on cooldown "
                      f"({ZONE_COMMAND_COOLDOWN_S}s between waterings)"
            ), 429
        resp, status = queue_cmd({"cmd": "water", "zone": zone, "ms": ms})
        if status != 200:
            _zone_rate_rollback(zone, token)
        return resp, status

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


# ---- Classification (on-demand) -------------------------------------
_classify_lock = threading.Lock()
_last_classify_mono = 0.0


def _push_diagnosis_to_mega(result: dict) -> None:
    """Send a compact diagnosis to the Mega for its LCD (display only).

    The Mega cannot run image classification and has no camera; it simply
    shows what the Pi found. Health is sent as a small integer code:
        0 = unknown, 1 = healthy, 2 = stressed, 3 = diseased
    plus an optional zone, foliage score, and a short (<=11 char) label.
    Best-effort: if the queue is full we silently drop it (the dashboard
    still has the full result).
    """
    health = (result.get("health") or "").lower()
    if "disease" in health:
        h = 3
    elif health == "healthy":
        h = 1
    elif health in ("", "unknown"):
        h = 0
    else:
        h = 2

    cond    = result.get("condition")
    species = result.get("species")
    label   = cond if (cond and cond.lower() != "healthy") else (species or "")
    if label.lower() in ("", "unknown"):
        label = ""

    cmd: dict = {"cmd": "diag", "h": h}
    zone = result.get("zone")
    if isinstance(zone, int):
        cmd["zone"] = zone
    score = result.get("health_score")
    if isinstance(score, int):
        cmd["score"] = score
    if label:
        cmd["label"] = label[:11]   # the Mega's LCD buffer truncates anyway

    try:
        command_queue.put_nowait(cmd)
    except queue.Full:
        pass


@app.route("/api/classify", methods=["POST"])
def api_classify():
    global _last_classify_mono
    if classifier is None or not classifier.available:
        detail = classifier.status if classifier else "classifier module not loaded"
        return jsonify(error="classifier unavailable", detail=detail), 503

    payload = request.get_json(silent=True) or {}
    zone = payload.get("zone")
    if zone is not None:
        try:
            zone = int(zone)
            if not 0 <= zone <= 2:
                zone = None
        except (TypeError, ValueError):
            zone = None

    # Run one inference at a time on the Pi; a second concurrent request
    # gets 429 immediately rather than queueing behind the first.
    if not _classify_lock.acquire(blocking=False):
        return jsonify(error="an examination is already in progress"), 429
    try:
        now = time.monotonic()
        if now - _last_classify_mono < CLASSIFY_MIN_INTERVAL_S:
            return jsonify(error="please wait a moment between examinations"), 429
        jpeg = camera.snapshot()
        if not jpeg:
            return jsonify(error="no camera frame available"), 503
        result = classifier.classify(jpeg, zone=zone)
        _last_classify_mono = time.monotonic()
    finally:
        _classify_lock.release()

    enqueue_db_classification(result)
    if result.get("source") == "tflite":
        detail = (f"{result.get('species')} · {result.get('condition')} "
                  f"({result.get('confidence', 0):.0%})")
    else:
        hs = result.get("health_score")
        detail = (f"health {result.get('health')}"
                  + (f" · foliage {hs}%" if hs is not None else ""))
    enqueue_db_event("classify", zone=zone, details=detail)
    # Mirror the result onto the Mega's LCD, but only if it's online.
    # Classification itself never depends on the Mega being connected.
    if state.snapshot()["connected"]:
        _push_diagnosis_to_mega(result)
    return jsonify(result), 200


@app.route("/api/classifications")
def api_classifications():
    limit = request.args.get("limit", default=10, type=int) or 10
    limit = max(1, min(limit, 100))
    return jsonify(db_classifications(limit))


@app.route("/snapshot.jpg")
def snapshot():
    img = camera.snapshot()
    if img is None:
        return Response(status=204)
    return Response(img, mimetype="image/jpeg",
                    headers={"Cache-Control": "no-store"})


# ======================== DASHBOARD HTML =============================
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
  .diag-controls { display: flex; gap: 10px; margin-bottom: 16px; }
  .diag-controls select { flex: 1; background: var(--paper); color: var(--ink);
      border: 1px solid var(--rule); padding: 9px 12px;
      font-family: 'Cormorant Garamond', serif; font-size: 15px; font-style: italic; }
  .diag-controls button { flex: 0 0 auto; min-width: 200px; }
  .diag-result { border: 1px solid var(--rule); background: var(--paper);
      padding: 18px 20px; min-height: 84px; }
  .diag-empty { font-family: 'Cormorant Garamond', serif; font-style: italic; color: var(--ink-soft); }
  .diag-species { font-family: 'Cormorant Garamond', serif; font-size: 30px; color: var(--ink); line-height: 1.1; }
  .diag-sub { font-family: 'JetBrains Mono', monospace; font-size: 11px;
      letter-spacing: .08em; color: var(--ink-soft); margin-top: 4px; }
  .diag-badges { display: flex; gap: 8px; margin-top: 12px; flex-wrap: wrap; align-items: center; }
  .diag-badge { font-family: 'JetBrains Mono', monospace; font-size: 10px;
      letter-spacing: .16em; text-transform: uppercase; padding: 4px 12px;
      border: 1px solid var(--rule); background: var(--paper-2); color: var(--ink-2); }
  .diag-badge.good { background: var(--moss); color: var(--paper); border-color: var(--moss-2); }
  .diag-badge.bad  { background: var(--terra); color: var(--paper); border-color: var(--terra-2); }
  .diag-badge.warn { background: var(--gold); color: #2a1f12; border-color: #93733f; }
  .diag-topk { margin-top: 12px; font-family: 'JetBrains Mono', monospace;
      font-size: 10px; color: var(--ink-soft); letter-spacing: .04em; }
  #diagHistory { margin-top: 16px; }
  #diagHistory li { grid-template-columns: 80px 1fr 90px; }
  #diagHistory .k.good { color: var(--moss); }
  #diagHistory .k.bad  { color: var(--terra); }
  #diagHistory .k.warn { color: var(--gold); }
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
        <button type="button" class="btn-water">Irrigate</button>
        <button type="button" class="btn-clear warn" disabled>Clear Alarm</button>
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

  <article class="card" style="margin-top: 24px; padding: 22px 26px;">
    <div class="card-label">Botanical Identification & Vigour</div>
    <h2 style="margin-bottom: 16px;">Examine the <em>Specimen</em></h2>
    <div class="diag-controls">
      <select id="diagZone" aria-label="Specimen in view">
        <option value="">— whole view —</option>
        {% for z in zones %}
        <option value="{{ loop.index0 }}">{{ z }}</option>
        {% endfor %}
      </select>
      <button type="button" id="diagBtn">Analyse Current View</button>
    </div>
    <div class="diag-result" id="diagResult">
      <div class="diag-empty">No examination yet — aim the camera and press analyse.</div>
    </div>
    <ul class="events" id="diagHistory"></ul>
  </article>

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

function authHeaders() {
  const h = {};
  const key = getApiKey();
  if (key) h['X-API-Key'] = key;
  return h;
}

function handle401(r) {
  if (r.status !== 401) return false;
  if (localStorage.getItem('plant_api_key')) {
    localStorage.removeItem('plant_api_key');
  }
  return true;
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
  const headers = {'Content-Type': 'application/json', ...authHeaders()};
  try {
    const r = await fetch('/api/command', {
      method: 'POST', headers, body: JSON.stringify(body)
    });
    if (handle401(r)) {
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
    const r = await fetch('/api/status', { headers: authHeaders() });
    if (handle401(r)) throw new Error('unauthorized');
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

  setText($('#envTemp'),  (s.temp != null && s.temp !== -99) ? Number(s.temp).toFixed(1) : '—');
  setText($('#envHum'),   (s.hum  != null && s.hum  !== -1)  ? s.hum : '—');
  setText($('#envLight'), (s.l    != null && s.l    !== -1)  ? s.l   : '—');
  const sv = $('#servoSlider').value;
  setText($('#envServo'), sv);
  setText($('#servoLbl'), sv + '°');
}

async function fetchEvents() {
  try {
    const r  = await fetch('/api/events?limit=25', { headers: authHeaders() });
    if (handle401(r)) return;
    if (!r.ok) return;
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
    const r = await fetch('/api/history?hours=24', { headers: authHeaders() });
    if (handle401(r)) return;
    if (!r.ok) return;
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

let lastBlobUrl = null;
const BLOB_REVOKE_DELAY_MS = 2000;

async function refreshCamera() {
  const img = $('#camImg');
  const none = $('#camNone');
  try {
    const r = await fetch('/snapshot.jpg', {
      headers: authHeaders(),
      cache: 'no-store',
    });
    if (handle401(r)) throw new Error('unauthorized');
    if (r.status === 204) throw new Error('no camera');
    if (!r.ok) throw new Error('http ' + r.status);
    const blob = await r.blob();
    const newUrl = URL.createObjectURL(blob);
    if (lastBlobUrl) {
      const toRevoke = lastBlobUrl;
      setTimeout(() => URL.revokeObjectURL(toRevoke), BLOB_REVOKE_DELAY_MS);
    }
    lastBlobUrl = newUrl;
    img.src = newUrl;
    img.style.display = '';
    none.style.display = 'none';
    setText($('#camCaption'), 'Captured ' + new Date().toLocaleTimeString('en-GB'));
  } catch (e) {
    img.style.display = 'none';
    none.style.display = '';
    setText($('#camCaption'), '—');
  }
}

function healthBadgeClass(h) {
  if (!h) return '';
  const s = String(h).toLowerCase();
  if (s.includes('healthy') || s === 'thriving') return 'good';
  if (s.includes('disease') || s.includes('poor') || s.includes('blight') || s.includes('rot')) return 'bad';
  if (s.includes('stress') || s.includes('mild') || s.includes('moderate')) return 'warn';
  return '';
}

function renderDiagnosis(r) {
  const box = $('#diagResult');
  box.replaceChildren();

  const species = document.createElement('div');
  species.className = 'diag-species';
  species.textContent = r.species || 'Unknown';
  box.append(species);

  const sub = document.createElement('div');
  sub.className = 'diag-sub';
  const when = new Date((r.ts || Date.now() / 1000) * 1000).toLocaleTimeString('en-GB');
  sub.textContent = (r.source === 'tflite' ? 'model classification' : 'colour-based estimate') + ' · ' + when;
  box.append(sub);

  const badges = document.createElement('div');
  badges.className = 'diag-badges';

  const hb = document.createElement('span');
  hb.className = 'diag-badge ' + healthBadgeClass(r.health);
  let hbText = String(r.health || 'unknown');
  if (r.condition && r.source === 'tflite' && r.condition.toLowerCase() !== 'healthy') hbText = r.condition;
  hb.textContent = hbText;
  badges.append(hb);

  if (r.source === 'tflite' && typeof r.confidence === 'number') {
    const cb = document.createElement('span');
    cb.className = 'diag-badge';
    cb.textContent = 'conf ' + Math.round(r.confidence * 100) + '%';
    badges.append(cb);
  }
  if (r.health_score != null) {
    const sb = document.createElement('span');
    sb.className = 'diag-badge';
    sb.textContent = 'foliage ' + r.health_score + '%';
    badges.append(sb);
  }
  box.append(badges);

  if (r.topk && r.topk.length > 1) {
    const tk = document.createElement('div');
    tk.className = 'diag-topk';
    tk.textContent = 'also: ' + r.topk.slice(1, 3)
        .map(t => t.label + ' (' + Math.round(t.p * 100) + '%)').join(' · ');
    box.append(tk);
  }
}

async function runDiagnosis() {
  const btn = $('#diagBtn');
  const zoneSel = $('#diagZone');
  const zoneVal = zoneSel.value === '' ? null : Number(zoneSel.value);
  btn.disabled = true;
  const old = btn.textContent;
  btn.textContent = 'Examining…';
  try {
    const headers = {'Content-Type': 'application/json', ...authHeaders()};
    const r = await fetch('/api/classify', {
      method: 'POST', headers,
      body: JSON.stringify(zoneVal === null ? {} : { zone: zoneVal }),
    });
    if (handle401(r)) {
      window.alert('API key rejected. You will be prompted again on next action.');
    } else if (r.status === 503) {
      const j = await r.json().catch(() => ({}));
      window.alert(j.error || 'Classifier unavailable.');
    } else if (r.status === 429) {
      window.alert('The examiner is still working; try again in a moment.');
    } else if (r.ok) {
      renderDiagnosis(await r.json());
      fetchDiagHistory();
    } else {
      console.warn('classify failed', await r.text());
    }
  } catch (e) { console.error(e); }
  finally { btn.disabled = false; btn.textContent = old; }
}

async function fetchDiagHistory() {
  try {
    const r = await fetch('/api/classifications?limit=8', { headers: authHeaders() });
    if (handle401(r)) return;
    if (!r.ok) return;
    const rows = await r.json();
    const ul = $('#diagHistory');
    if (!rows || !rows.length) return;
    ul.replaceChildren();
    for (const c of rows) {
      const li = document.createElement('li');
      const t = document.createElement('span'); t.className = 't';
      t.textContent = fmtTime(c.ts);
      const d = document.createElement('span'); d.className = 'd';
      let txt = (c.species && c.species !== 'Unknown') ? c.species : 'health check';
      if (c.condition && c.condition.toLowerCase() !== 'healthy') txt += ' — ' + c.condition;
      if (c.zone != null) txt += ' · Z' + (Number(c.zone) + 1);
      d.textContent = txt;
      const s = document.createElement('span'); s.className = 'k ' + healthBadgeClass(c.health);
      s.style.textAlign = 'right';
      s.textContent = c.health || '—';
      li.append(t, d, s);
      ul.append(li);
    }
  } catch (e) { /* keep last */ }
}

$('#diagBtn').addEventListener('click', runDiagnosis);
fetchDiagHistory();

fetchStatus();    setInterval(fetchStatus,  3000);
fetchEvents();    setInterval(fetchEvents, 10000);
fetchHistory();   setInterval(fetchHistory, 60000);
refreshCamera();  setInterval(refreshCamera, 15000);
</script>
</body>
</html>"""


# ======================== STARTUP / SHUTDOWN =========================
def check_dialout_membership() -> None:
    if not PWD_AVAILABLE:
        return
    try:
        uid = os.getuid()
        if uid == 0:
            return
        username = pwd.getpwuid(uid).pw_name
        my_groups = {grp.getgrgid(g).gr_name for g in os.getgroups()}
        if "dialout" not in my_groups:
            log.warning(f"User '{username}' is not in the 'dialout' group; "
                        "opening /dev/ttyACM* may fail with Permission denied.")
            log.warning(f"Fix:  sudo usermod -aG dialout {username}  "
                        "(then log out and back in)")
    except KeyError:
        log.debug("Could not resolve current user; skipping dialout check.")
    except OSError as e:
        log.debug(f"OS error during dialout check: {e}")


def graceful_shutdown(signum, frame) -> None:
    log.info(f"Signal {signum} received; flushing and shutting down.")
    shutdown_event.set()
    if db_writer_thread is not None:
        db_writer_thread.join(timeout=SHUTDOWN_JOIN_TIMEOUT_S)
        if db_writer_thread.is_alive():
            log.warning(
                f"DB writer did not finish within {SHUTDOWN_JOIN_TIMEOUT_S}s; "
                f"{db_event_queue.qsize()} event(s) may be lost.")
    sys.exit(0)


def periodic_pruner() -> None:
    while not shutdown_event.is_set():
        if shutdown_event.wait(3600):
            break
        db_prune()


def main() -> None:
    global db_writer_thread

    if sys.version_info < (3, 9):
        print("Python 3.9+ required.", file=sys.stderr)
        sys.exit(1)

    signal.signal(signal.SIGINT,  graceful_shutdown)
    signal.signal(signal.SIGTERM, graceful_shutdown)

    check_dialout_membership()
    init_db()

    db_writer_thread = threading.Thread(target=db_writer_loop,
                                        daemon=True, name="DBWriter")
    db_writer_thread.start()
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

    threading.Thread(target=periodic_pruner,       daemon=True, name="Pruner").start()
    threading.Thread(target=periodic_camera_retry, daemon=True, name="CameraRetry").start()
    SerialBridge().start()

    log.info(f"Dashboard: http://{HTTP_HOST}:{HTTP_PORT}")
    if HTTP_HOST == "0.0.0.0":
        log.warning("Bound to 0.0.0.0 (LAN-accessible). "
                    "Set PLANT_API_KEY to require authentication.")
    log.info(f"Camera:    {'enabled (init in background)' if camera.enabled else 'disabled (OpenCV not installed)'}")
    if classifier is not None:
        log.info(f"Classifier: {classifier.status}")
    else:
        log.info("Classifier: disabled (plant_classifier.py not importable)")
    if API_KEY:
        log.info("API auth:  ENABLED (X-API-Key required for /api/* and /snapshot.jpg)")
    else:
        log.info("API auth:  disabled  (set PLANT_API_KEY env var to enable)")

    app.run(host=HTTP_HOST, port=HTTP_PORT,
            debug=False, use_reloader=False, threaded=True)


if __name__ == "__main__":
    main()
