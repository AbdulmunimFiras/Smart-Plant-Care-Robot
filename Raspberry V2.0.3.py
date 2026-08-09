#!/usr/bin/env python3
# =====================================================================
#   PLANT CARE PI CONTROLLER  -  v2.0.3
#   Raspberry Pi supervisor for the Mega-based plant care robot.
#
#   v2.0.3 — third audit pass (this revision):
#     • The dashboard trusted the shape of every reading it was handed.
#       A garbled-but-parseable serial line — the kind a long or noisy
#       USB run produces — could arrive as {"m":"nope","alarms":"broken"}
#       and the page would print NaN% and light an alarm badge on all
#       three beds, because "broken"[0] is a truthy character. Readings
#       are now coerced on arrival: non-numeric values read as "sensor
#       lost", and an alarm is only raised by a number that isn't zero.
#     • The serial log pane assumed recent_log was a list of pairs and
#       would walk a string character by character. It now ignores
#       anything that isn't shaped like a log line.
#     • Moisture, humidity and light are rounded for display, so an
#       averaged or fractional reading can't print 42.66666666666667%.
#     • Startup warns if ZONE_NAMES has been edited to anything other
#       than three entries — the schema, the firmware and the API all
#       assume three, and the mismatch was previously silent.
#
#   v2.0.2 — second audit pass:
#     • Watering was refused on every zone for the first
#       ZONE_COMMAND_COOLDOWN_S seconds of system uptime. The rate
#       limiter defaulted "never watered" to 0.0 and compared it against
#       time.monotonic(), which counts from boot — so a controller
#       started by systemd at boot answered "on cooldown" to its own
#       first commands. Never-watered is now a missing key, not 0.0.
#     • /api/history read the whole range into Python: 30 days of rows
#       peaked at 70 MB and 1.7s per request, and the dashboard re-reads
#       history on a timer. It now counts first and asks SQLite for every
#       Nth row, so at most HISTORY_QUERY_MAX_ROWS reach Python (30 days:
#       1.7 MB, 0.13s). The drawn curve is unchanged. Falls back to a
#       full read on SQLite older than 3.25 (no window functions).
#     • No ceiling existed on request bodies, so anyone who could reach
#       the port — and the default install has no API key — could make
#       the Pi buffer megabytes per request. Capped at MAX_REQUEST_BYTES
#       with a JSON 413.
#     • Read connections now set busy_timeout like the writer does.
#     • Dashboard: a 7-day chart no longer re-reads a week of history
#       every 60s; wide ranges refresh every 5 minutes instead.
#
#   v2.0.1 — first audit pass over v2.0:
#     • DB writer rolls back after a failed insert instead of leaving a
#       half-open transaction for the next row to trip over.
#     • The live-view route asks Camera.stream_ready(), which reads cap
#       and last_jpeg under the camera lock, instead of touching those
#       attributes from the request thread.
#     • Dashboard: a still frame arriving after the viewer switched back
#       to the live stream is discarded rather than replacing the stream
#       in the same <img> and stalling it. "Take a frame" re-syncs the
#       stream instead of killing it, and a refused stream no longer
#       overwrites the saved live/still preference.
#     • Dashboard: the serial log keeps your scroll position instead of
#       snapping to the bottom on every poll.
#     • Dashboard: the chart says why it is empty — no rows in range, or
#       Chart.js blocked on a Pi with no route out — instead of showing
#       a blank rectangle.
#     • Dashboard: losing the Pi disables the pump buttons; they used to
#       stay live because the last good status was still on screen.
#     • Dashboard: a frame vanishing while the fullscreen viewer is open
#       closes the viewer instead of leaving an empty stage.
#
#   v2.0 — Dashboard rebuild + full remote control:
#     • New dashboard visual direction: "grow room at night". Deep
#       violet ground lit by the two horticultural LED spectra
#       (450nm blue + 660nm red), Fraunces / Space Grotesk / Space
#       Mono type, glass panels. Night is the default; a frosted
#       daylight theme is one tap away and is remembered.
#     • Signature interaction: the shade-sheet slider physically dims
#       the page's ambient grow-light aura, so closing the shade over
#       the garden closes it over the interface too.
#     • The camera is now the hero: a large edge-lit pane with an
#       MJPEG live stream (/camera/stream.mjpg), a fullscreen viewer
#       with wheel/pinch zoom and drag panning, and an automatic
#       fallback to still snapshots if the stream is unavailable.
#     • Full remote control surface: per-zone watering duration,
#       live cooldown countdowns, servo presets, clear-all-alarms,
#       adjustable poll rate, chart range/metric switching, an
#       in-page API-key manager (no more window.prompt), toasts
#       instead of alerts, and the Mega's serial chatter on screen.
#     • Optional raw-command console for firmware commands this
#       controller doesn't know about yet — off unless you set
#       PLANT_ALLOW_RAW_CMD=1. Raw `water` is still refused so the
#       pump cooldown can never be bypassed.
#     • Python control paths (threading, rate-limit tokens, DB
#       writer, serial bridge) are unchanged from v1.6.
#
#   v1.6 — Fourth-pass fixes:
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
#     python3 plant_controller.py
#
#   Reaching the garden from another room (phone, laptop):
#     export PLANT_API_KEY="$(python3 -c 'import secrets;print(secrets.token_urlsafe(32))')"
#     export PLANT_HTTP_HOST="0.0.0.0"      # listen on the LAN
#     python3 plant_controller.py
#     # then open http://<pi-ip>:5000 and paste the key when asked.
#
#   Reaching it from outside the house: do NOT forward port 5000.
#   Put it behind Tailscale/WireGuard, or a reverse proxy that
#   terminates HTTPS — the API key travels in plain text otherwise.
#
#   Optional extras:
#     export PLANT_ALLOW_RAW_CMD=1          # enable the raw console
#
#   For long-running deployment, prefer waitress over the dev server:
#     pip install waitress
#     # then edit main() to use serve(app, host=..., port=..., threads=8)
#     # (threads=8 leaves room for the MJPEG stream connections)
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
HISTORY_QUERY_MAX_ROWS   = 2000   # ceiling on rows pulled into Python per request
DB_EVENT_QUEUE_SIZE      = 500
COMMAND_QUEUE_SIZE       = 32
ZONE_COMMAND_COOLDOWN_S  = 10

WATER_MS_MIN             = 500
WATER_MS_MAX             = 8000
WATER_MS_DEFAULT         = 3000

SHUTDOWN_JOIN_TIMEOUT_S  = 6.0
CAMERA_RETRY_INTERVAL_S  = 60

CAMERA_INDEX             = 0
CAMERA_WIDTH             = 1280
CAMERA_HEIGHT            = 720
CAMERA_JPEG_Q            = 80
CAMERA_CACHE_S           = 1.0

# MJPEG live view. Each viewer holds a worker thread for as long as the
# tab is open, so the number of simultaneous streams is capped and every
# stream self-terminates after STREAM_MAX_SECONDS (the browser silently
# reconnects). Frames are served straight from the snapshot cache, so a
# stream costs no extra camera reads beyond CAMERA_CACHE_S.
STREAM_MAX_CLIENTS       = 4
STREAM_FPS               = 2
STREAM_MAX_SECONDS       = 600
STREAM_FIRST_FRAME_WAIT_S = 5

DB_PATH                  = Path(__file__).resolve().parent / "plant_history.db"
LOG_RETAIN_DAYS          = 30


def _env_str(name: str, default: str) -> str:
    return os.environ.get(name) or default


def _env_bool(name: str, default: bool = False) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


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

# Raw passthrough lets the dashboard send commands this controller has
# no validator for (handy while the Mega firmware grows). Off by default;
# still behind the API key and the same-origin check when on.
ALLOW_RAW_CMD            = _env_bool("PLANT_ALLOW_RAW_CMD", False)
RAW_CMD_MAX_BYTES        = 512
MAX_REQUEST_BYTES        = 64 * 1024   # ceiling on any request body

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
#   v1.6: rollback receives a token that contains OUR exact stamp.
#         It only undoes the reservation if that stamp is still
#         there — a later legitimate reservation by another thread
#         is left untouched.
#   v2.0: unchanged. The new raw-command path deliberately refuses
#         `water`, so there is still exactly one door to the pumps.
_zone_last_water_mono: dict[int, float] = {}
_zone_rate_lock = threading.Lock()


def _zone_rate_try_acquire(zone: int) -> tuple[float | None, float] | None:
    """Atomically reserve a watering slot for *zone*.

    Returns a (prev, ours) token on success — caller passes this back
    to _zone_rate_rollback() if the downstream enqueue failed.
    Returns None if the zone is still on cooldown.

    v2.0.2: "never watered" is a missing key, not 0.0. time.monotonic()
    counts from system boot, so with a 0.0 default every zone reported
    itself on cooldown for the first ZONE_COMMAND_COOLDOWN_S seconds of
    uptime — precisely when the controller starts under systemd at boot.
    """
    with _zone_rate_lock:
        now = time.monotonic()
        prev = _zone_last_water_mono.get(zone)
        if prev is not None and now - prev < ZONE_COMMAND_COOLDOWN_S:
            return None
        _zone_last_water_mono[zone] = now
        return (prev, now)


def _zone_rate_rollback(zone: int, token: tuple[float | None, float]) -> None:
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
            if prev is None:
                _zone_last_water_mono.pop(zone, None)   # back to never-watered
            else:
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
    """Rows for the last *hours*, thinned in SQL before they reach Python.

    v2.0.2: 30 days at one row per 30s is ~86k rows; materialising all of
    them peaked at 70 MB and 1.7s on every poll — painful on a Pi, and the
    dashboard re-reads history on a timer. We now count first and, when
    needed, ask SQLite for every Nth row, so at most HISTORY_QUERY_MAX_ROWS
    ever cross into Python. downsample_status() still averages on top of
    that, so the drawn curve is unchanged to the eye.
    """
    conn = None
    try:
        cutoff = int(time.time()) - hours * 3600
        conn = sqlite3.connect(str(DB_PATH))
        conn.execute("PRAGMA busy_timeout=5000;")
        conn.row_factory = sqlite3.Row
        total = conn.execute(
            "SELECT COUNT(*) FROM status_log WHERE ts >= ?", (cutoff,)
        ).fetchone()[0]
        stride = max(1, total // HISTORY_QUERY_MAX_ROWS)
        rows = None
        if stride > 1:
            try:
                rows = conn.execute(
                    "SELECT * FROM ("
                    "  SELECT *, ROW_NUMBER() OVER (ORDER BY ts) AS _rn"
                    "  FROM status_log WHERE ts >= ?"
                    ") WHERE _rn % ? = 0 ORDER BY ts ASC", (cutoff, stride)
                ).fetchall()
            except sqlite3.OperationalError as e:
                # SQLite older than 3.25 has no window functions; take the
                # slow path rather than dropping the chart entirely.
                log.debug(f"history stride query unavailable ({e}); reading in full")
        if rows is None:
            rows = conn.execute(
                "SELECT * FROM status_log WHERE ts >= ? ORDER BY ts ASC", (cutoff,)
            ).fetchall()
        out = []
        for r in rows:
            d = dict(r)
            d.pop("_rn", None)
            out.append(d)
        return out
    except Exception as e:
        log.error(f"db_history failed: {e}")
        return []
    finally:
        if conn is not None:
            try: conn.close()
            except Exception: pass


def db_events(limit: int = 30) -> list[dict]:
    try:
        conn = sqlite3.connect(str(DB_PATH))
        conn.execute("PRAGMA busy_timeout=5000;")
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
        conn.execute("PRAGMA busy_timeout=5000;")
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
        conn.execute("DELETE FROM status_log      WHERE ts < ?", (cutoff,))
        conn.execute("DELETE FROM events           WHERE ts < ?", (cutoff,))
        conn.execute("DELETE FROM classifications  WHERE ts < ?", (cutoff,))
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
                try:
                    conn.rollback()     # never leave a half-open transaction
                except Exception:
                    pass
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

    def stream_ready(self) -> bool:
        """True if a live stream could produce at least one frame.

        Checked under the lock: the camera thread may be swapping self.cap
        at any moment, and a stale frame is still worth streaming while a
        reopen is in flight.
        """
        if not self.enabled:
            return False
        with self.lock:
            return self.cap is not None or self.last_jpeg is not None

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

# Nothing here is an upload: the largest legitimate body is a raw command,
# capped at RAW_CMD_MAX_BYTES. Without a ceiling, anyone who can reach the
# port (the default install has no API key) could make the Pi buffer
# megabytes per request.
app.config["MAX_CONTENT_LENGTH"] = MAX_REQUEST_BYTES

AUTHED_EXACT_PATHS = ("/snapshot.jpg", "/camera/stream.mjpg")


@app.errorhandler(413)
def _too_large(_e):
    return jsonify(error=f"request body too large "
                         f"(limit {MAX_REQUEST_BYTES} bytes)"), 413


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
    if path.startswith("/api/") or path in AUTHED_EXACT_PATHS:
        if not _check_api_key():
            # <img> tags can't set headers, so the image endpoints take
            # ?api_key= instead and answer with a bare 401.
            if path in AUTHED_EXACT_PATHS:
                return Response(status=401)
            return jsonify(error="unauthorized; X-API-Key required"), 401
    return None


@app.route("/")
def index():
    return render_template_string(
        DASHBOARD_HTML,
        zones=ZONE_NAMES,
        api_required=bool(API_KEY),
        cooldown=ZONE_COMMAND_COOLDOWN_S,
        raw_enabled=ALLOW_RAW_CMD,
        ms_min=WATER_MS_MIN,
        ms_max=WATER_MS_MAX,
        ms_default=WATER_MS_DEFAULT,
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
            ms   = int(payload.get("ms", WATER_MS_DEFAULT))
        except (TypeError, ValueError):
            return jsonify(error="zone/ms must be integers"), 400
        if not 0 <= zone <= 2:
            return jsonify(error="zone must be 0..2"), 400
        if not WATER_MS_MIN <= ms <= WATER_MS_MAX:
            return jsonify(
                error=f"ms must be {WATER_MS_MIN}..{WATER_MS_MAX}"), 400

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

    if cmd == "raw":
        # Escape hatch for firmware commands this controller has no
        # validator for. Opt-in, size-capped, and it refuses `water` so
        # the pump cooldown above stays the only way to run a pump.
        if not ALLOW_RAW_CMD:
            return jsonify(
                error="raw commands are disabled; start with "
                      "PLANT_ALLOW_RAW_CMD=1 to enable them"), 403
        inner = payload.get("payload")
        if not isinstance(inner, dict) or not isinstance(inner.get("cmd"), str):
            return jsonify(
                error="raw payload must be an object with a string 'cmd'"), 400
        if inner.get("cmd") == "water":
            return jsonify(
                error="send watering as cmd=water so the pump cooldown "
                      "still applies"), 400
        try:
            encoded = json.dumps(inner, separators=(",", ":"))
        except (TypeError, ValueError):
            return jsonify(error="raw payload is not JSON-serialisable"), 400
        if len(encoded) > RAW_CMD_MAX_BYTES:
            return jsonify(
                error=f"raw payload too large "
                      f"({len(encoded)} > {RAW_CMD_MAX_BYTES} bytes)"), 400
        return queue_cmd(inner)

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

    payload = request.get_json(silent=True)
    if not isinstance(payload, dict):
        payload = {}        # tolerate empty/array/scalar bodies as "no zone"
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


# ---- Live view (MJPEG) ----------------------------------------------
_stream_slots = threading.Semaphore(STREAM_MAX_CLIENTS)


@app.route("/camera/stream.mjpg")
def camera_stream():
    """multipart/x-mixed-replace live view, served from the frame cache.

    Returns 503 when the camera is off or all viewer slots are taken; the
    dashboard falls back to polling /snapshot.jpg on its own.
    """
    # 503 immediately (rather than an empty stream the browser waits on)
    # when there is no capture and not even a stale frame to show.
    if not camera.stream_ready():
        return Response(status=503)
    if not _stream_slots.acquire(blocking=False):
        return Response(status=503)

    # The slot is taken before the generator runs, so it must also be
    # released if the client vanishes before a single frame is pulled.
    # Both paths funnel through release(), which only fires once.
    released = threading.Event()

    def release() -> None:
        if not released.is_set():
            released.set()
            _stream_slots.release()

    def gen():
        started = time.monotonic()
        last = None
        try:
            while not shutdown_event.is_set():
                elapsed = time.monotonic() - started
                if elapsed > STREAM_MAX_SECONDS:
                    return                      # browser reconnects
                frame = camera.snapshot()
                if frame is None:
                    if last is None and elapsed > STREAM_FIRST_FRAME_WAIT_S:
                        return                  # never came up; let JS fall back
                elif frame is not last:
                    last = frame
                    yield (b"--hortusframe\r\n"
                           b"Content-Type: image/jpeg\r\n"
                           b"Content-Length: " + str(len(frame)).encode() +
                           b"\r\n\r\n" + frame + b"\r\n")
                time.sleep(max(0.05, 1.0 / STREAM_FPS))
        finally:
            release()

    resp = Response(
        gen(),
        mimetype="multipart/x-mixed-replace; boundary=hortusframe",
        headers={"Cache-Control": "no-store", "X-Accel-Buffering": "no"})
    resp.call_on_close(release)
    return resp


# ======================== DASHBOARD HTML =============================
# Visual direction: "grow room at night". The page is lit by the two
# horticultural LED spectra a real grow light emits — 450nm blue and
# 660nm red — bleeding across dark glass. The shade-sheet slider dims
# that light, so closing the shade over the garden closes it over the
# interface too. A frosted daylight theme is available and remembered.
DASHBOARD_HTML = r"""<!doctype html>
<html lang="en" data-theme="night">
<head>
<meta charset="utf-8">
<title>Hortus Vigilis — the garden, live</title>
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<meta name="color-scheme" content="dark light">
<meta id="themeColor" name="theme-color" content="#0C0A16">
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link rel="stylesheet" href="https://fonts.googleapis.com/css2?family=Fraunces:ital,opsz,wght@0,9..144,300..700;1,9..144,400..600&family=Space+Grotesk:wght@300..700&family=Space+Mono:wght@400;700&display=swap">
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/chartjs-adapter-date-fns@3.0.0/dist/chartjs-adapter-date-fns.bundle.min.js"></script>
<style>
/* ---------- tokens ---------------------------------------------- */
html[data-theme=night]{
  --ground:#0C0A16; --ground-2:#141029;
  --pane:rgba(25,20,50,.62); --pane-2:rgba(31,25,62,.5);
  --edge:#33285E; --edge-soft:rgba(130,110,205,.20);
  --text:#EDE6FF; --text-2:#CFC4EE; --muted:#9C8FC6;
  --bloom:#E85D9F; --beam:#6C7BF7; --leaf:#4FD69C; --amber:#F5A742;
  --shadow:0 30px 70px -40px rgba(0,0,0,.95);
  --grain:.30; --aura:.85; --track:rgba(160,140,230,.16);
}
html[data-theme=day]{
  --ground:#E7EDEA; --ground-2:#F4F7F5;
  --pane:rgba(255,255,255,.78); --pane-2:rgba(255,255,255,.6);
  --edge:#C7D3CD; --edge-soft:rgba(60,90,80,.14);
  --text:#152019; --text-2:#324038; --muted:#5F7168;
  --bloom:#BE2F7C; --beam:#3B4FC7; --leaf:#127A51; --amber:#A96708;
  --shadow:0 26px 60px -42px rgba(21,32,25,.55);
  --grain:.16; --aura:.45; --track:rgba(21,32,25,.10);
}
*{box-sizing:border-box;margin:0;padding:0}
html{background:var(--ground);-webkit-text-size-adjust:100%}
body{
  font-family:'Space Grotesk',system-ui,sans-serif;
  color:var(--text);background:var(--ground);
  line-height:1.5;min-height:100vh;overflow-x:hidden;
  transition:background .6s ease,color .6s ease;
}
:focus-visible{outline:2px solid var(--beam);outline-offset:3px;border-radius:2px}
::selection{background:var(--bloom);color:#fff}

/* ---------- the grow light -------------------------------------- */
/* --shade (1 = sheet open, dim = sheet drawn) is written by the
   servo slider, so the room's light follows the shade sheet.        */
.room{position:fixed;inset:0;z-index:0;pointer-events:none;overflow:hidden;
      opacity:calc(.30 + .70 * var(--shade,1));transition:opacity .6s ease}
.lamp{position:absolute;border-radius:50%;filter:blur(70px);
      animation:breathe 26s ease-in-out infinite}
.lamp.red{width:70vw;height:70vw;left:-18vw;top:-24vw;
  background:radial-gradient(circle,var(--bloom) 0%,transparent 66%);opacity:calc(.30*var(--aura))}
.lamp.blue{width:76vw;height:76vw;right:-24vw;top:16vh;
  background:radial-gradient(circle,var(--beam) 0%,transparent 66%);opacity:calc(.26*var(--aura));
  animation-duration:34s;animation-direction:reverse}
.lamp.grow{width:54vw;height:54vw;left:26vw;bottom:-24vw;
  background:radial-gradient(circle,var(--leaf) 0%,transparent 70%);opacity:calc(.16*var(--aura));
  animation-duration:44s}
@keyframes breathe{
  0%,100%{transform:translate3d(0,0,0) scale(1)}
  50%{transform:translate3d(2vw,2vh,0) scale(1.12)}
}
.grain{position:fixed;inset:0;z-index:1;pointer-events:none;opacity:var(--grain);
  background-image:url("data:image/svg+xml;utf8,<svg xmlns='http://www.w3.org/2000/svg' width='160' height='160'><filter id='n'><feTurbulence baseFrequency='.9' numOctaves='2' seed='7'/><feColorMatrix values='0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 0 0 0 .07 0'/></filter><rect width='100%' height='100%' filter='url(%23n)'/></svg>")}
.motes{position:fixed;inset:0;z-index:1;pointer-events:none;overflow:hidden}
.mote{position:absolute;width:3px;height:3px;border-radius:50%;
  background:var(--text);opacity:.10;animation:rise-mote linear infinite}
@keyframes rise-mote{
  0%{transform:translate3d(0,10vh,0);opacity:0}
  12%{opacity:.16}
  100%{transform:translate3d(3vw,-105vh,0);opacity:0}
}

/* ---------- shell ------------------------------------------------ */
.shell{position:relative;z-index:2;max-width:1240px;margin:0 auto;
  padding:0 22px 90px}
.rail{position:sticky;top:0;z-index:20;display:flex;align-items:center;
  gap:16px;padding:14px 0 13px;margin-bottom:26px;
  border-bottom:1px solid var(--edge-soft);
  background:linear-gradient(to bottom,var(--ground) 62%,transparent);
  backdrop-filter:blur(10px)}
.mark{display:flex;align-items:center;gap:11px;min-width:0}
.mark svg{width:26px;height:26px;color:var(--leaf);flex:none}
.mark .name{font-family:'Fraunces',Georgia,serif;font-weight:600;
  font-size:19px;letter-spacing:-.01em;white-space:nowrap}
.mark .name em{font-style:italic;font-weight:400;color:var(--bloom)}
.rail .spacer{flex:1}
.pill{display:inline-flex;align-items:center;gap:8px;padding:6px 13px;
  border:1px solid var(--edge);border-radius:999px;background:var(--pane-2);
  font-family:'Space Mono',monospace;font-size:10.5px;letter-spacing:.14em;
  text-transform:uppercase;color:var(--muted);white-space:nowrap}
.pill .dot{width:7px;height:7px;border-radius:50%;background:var(--amber);
  box-shadow:0 0 0 0 currentColor;animation:beat 2.4s infinite}
.pill.online{color:var(--leaf);border-color:color-mix(in srgb,var(--leaf) 45%,transparent)}
.pill.online .dot{background:var(--leaf)}
.pill.down{color:var(--bloom);border-color:color-mix(in srgb,var(--bloom) 45%,transparent)}
.pill.down .dot{background:var(--bloom)}
@keyframes beat{0%,100%{box-shadow:0 0 0 0 currentColor;opacity:1}
                50%{box-shadow:0 0 0 5px transparent;opacity:.55}}
.icon-btn{width:36px;height:36px;display:grid;place-items:center;
  border:1px solid var(--edge);border-radius:10px;background:var(--pane-2);
  color:var(--text-2);cursor:pointer;font-size:15px;
  transition:border-color .2s,color .2s,transform .12s}
.icon-btn:hover{color:var(--text);border-color:var(--beam)}
.icon-btn:active{transform:scale(.94)}

/* ---------- headline --------------------------------------------- */
.hero-type{margin:8px 0 22px}
.hero-type h1{font-family:'Fraunces',Georgia,serif;font-weight:300;
  font-size:clamp(38px,7vw,74px);line-height:.98;letter-spacing:-.025em;
  font-variation-settings:'opsz' 120}
.hero-type h1 em{font-style:italic;font-weight:600;
  background:linear-gradient(96deg,var(--bloom),var(--beam));
  -webkit-background-clip:text;background-clip:text;color:transparent}
.hero-type p{margin-top:10px;max-width:52ch;color:var(--muted);font-size:15px}
.readout{display:flex;flex-wrap:wrap;gap:10px 26px;margin-top:16px;
  font-family:'Space Mono',monospace;font-size:11px;letter-spacing:.1em;
  text-transform:uppercase;color:var(--muted)}
.readout b{color:var(--text-2);font-weight:400}

/* ---------- panels ----------------------------------------------- */
.panel{position:relative;background:var(--pane);border:1px solid var(--edge);
  border-radius:18px;padding:22px 24px;box-shadow:var(--shadow);
  backdrop-filter:blur(14px);
  transition:border-color .3s,background .6s,transform .3s}
.panel:hover{border-color:color-mix(in srgb,var(--beam) 35%,var(--edge))}
.eyebrow{font-family:'Space Mono',monospace;font-size:10px;letter-spacing:.22em;
  text-transform:uppercase;color:var(--muted)}
.panel h2{font-family:'Fraunces',Georgia,serif;font-weight:400;font-size:26px;
  line-height:1.1;letter-spacing:-.015em;margin:6px 0 18px}
.panel h2 em{font-style:italic;color:var(--bloom);font-weight:500}
.panel-head{display:flex;align-items:flex-start;justify-content:space-between;
  gap:14px;flex-wrap:wrap}
.grid{display:grid;gap:20px}
.beds{grid-template-columns:repeat(3,1fr);margin-top:20px}
.split{grid-template-columns:1.25fr 1fr;margin-top:20px}
.reveal{opacity:0;transform:translateY(16px);
  animation:rise .75s cubic-bezier(.16,.84,.34,1) forwards}
@keyframes rise{to{opacity:1;transform:none}}

/* ---------- the pane (camera hero) -------------------------------- */
.pane-card{padding:18px 18px 16px;overflow:hidden}
.pane-card h2{margin-bottom:0}
.cam-tools{display:flex;gap:8px;flex-wrap:wrap}
.chip{display:inline-flex;align-items:center;gap:7px;padding:7px 13px;
  border:1px solid var(--edge);border-radius:999px;background:var(--pane-2);
  color:var(--text-2);font-family:'Space Mono',monospace;font-size:10.5px;
  letter-spacing:.12em;text-transform:uppercase;cursor:pointer;
  transition:border-color .2s,color .2s,background .2s}
.chip:hover{color:var(--text);border-color:var(--beam)}
.chip.on{color:var(--leaf);border-color:color-mix(in srgb,var(--leaf) 50%,transparent);
  background:color-mix(in srgb,var(--leaf) 12%,transparent)}
.cam-frame{position:relative;margin-top:16px;border-radius:14px;overflow:hidden;
  aspect-ratio:16/9;max-height:66vh;cursor:zoom-in;
  background:radial-gradient(120% 120% at 50% 0%,var(--ground-2),var(--ground));
  border:1px solid var(--edge);
  box-shadow:0 0 0 1px var(--edge-soft) inset,
             0 0 90px -30px var(--bloom) inset,
             0 0 70px -30px var(--beam)}
.cam-frame img{width:100%;height:100%;object-fit:cover;display:block;
  transform-origin:center center}
.cam-none{position:absolute;inset:0;display:grid;place-items:center;
  padding:20px;text-align:center;color:var(--muted);font-size:14px}
.cam-none b{display:block;font-family:'Fraunces',serif;font-size:20px;
  font-weight:400;color:var(--text-2);margin-bottom:6px}
.cam-hud{position:absolute;top:12px;left:12px;display:flex;gap:8px}
.hud{padding:5px 11px;border-radius:999px;font-family:'Space Mono',monospace;
  font-size:10px;letter-spacing:.16em;text-transform:uppercase;
  background:rgba(8,6,18,.55);color:#fff;backdrop-filter:blur(6px);
  display:inline-flex;align-items:center;gap:7px}
.hud .dot{width:6px;height:6px;border-radius:50%;background:var(--bloom);
  animation:beat 1.8s infinite}
.cam-bar{position:absolute;left:0;right:0;bottom:0;display:flex;
  justify-content:space-between;gap:12px;padding:10px 14px;
  font-family:'Space Mono',monospace;font-size:10px;letter-spacing:.1em;
  text-transform:uppercase;color:rgba(255,255,255,.82);
  background:linear-gradient(to top,rgba(8,6,18,.75),transparent)}

/* ---------- beds -------------------------------------------------- */
.bed{display:flex;flex-direction:column}
.bed-head{display:flex;justify-content:space-between;align-items:center;gap:10px}
.tag{font-family:'Space Mono',monospace;font-size:9.5px;letter-spacing:.16em;
  text-transform:uppercase;padding:4px 10px;border-radius:999px;
  border:1px solid var(--edge);color:var(--muted);white-space:nowrap}
.tag.busy{color:var(--leaf);border-color:var(--leaf);
  background:color-mix(in srgb,var(--leaf) 14%,transparent)}
.tag.alarm{color:#fff;background:var(--bloom);border-color:var(--bloom);
  animation:beat 1.4s infinite}
.bed h3{font-family:'Fraunces',Georgia,serif;font-weight:400;font-size:19px;
  line-height:1.2;margin:10px 0 4px;letter-spacing:-.01em}
.ring-wrap{position:relative;width:100%;max-width:200px;margin:12px auto 6px;
  aspect-ratio:1}
.ring{width:100%;height:100%;transform:rotate(-90deg)}
.ring circle{fill:none;stroke-width:7;stroke-linecap:round}
.ring .ring-track{stroke:var(--track)}
.ring .ring-fill{stroke:var(--leaf);stroke-dasharray:326.7;stroke-dashoffset:326.7;
  transition:stroke-dashoffset 1s cubic-bezier(.16,.84,.34,1),stroke .5s;
  filter:drop-shadow(0 0 6px color-mix(in srgb,var(--leaf) 55%,transparent))}
.bed.watering .ring .ring-fill{animation:sip 1.6s ease-in-out infinite}
@keyframes sip{50%{opacity:.45}}
.ring-read{position:absolute;inset:0;display:flex;flex-direction:column;
  align-items:center;justify-content:center;gap:2px}
.ring-read .n{font-family:'Fraunces',Georgia,serif;font-weight:300;font-size:44px;
  line-height:1;letter-spacing:-.03em;font-variation-settings:'opsz' 100}
.ring-read .n.lost{font-size:17px;font-style:italic;color:var(--bloom)}
.ring-read .u{font-family:'Space Mono',monospace;font-size:9.5px;
  letter-spacing:.2em;text-transform:uppercase;color:var(--muted)}
.bed-stats{display:flex;justify-content:space-between;gap:10px;margin-top:8px;
  padding-top:12px;border-top:1px solid var(--edge-soft);
  font-family:'Space Mono',monospace;font-size:10px;letter-spacing:.06em;
  color:var(--muted)}
.dose{display:block;margin-top:14px}
.dose .lab{display:flex;justify-content:space-between;align-items:baseline;
  font-family:'Space Mono',monospace;font-size:10px;letter-spacing:.14em;
  text-transform:uppercase;color:var(--muted);margin-bottom:8px}
.dose .lab b{font-family:'Fraunces',serif;font-size:16px;font-weight:400;
  letter-spacing:0;color:var(--text);text-transform:none}
input[type=range]{width:100%;-webkit-appearance:none;appearance:none;
  background:transparent;cursor:pointer}
input[type=range]::-webkit-slider-runnable-track{height:3px;border-radius:3px;
  background:var(--track)}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;appearance:none;
  width:16px;height:16px;border-radius:50%;background:var(--leaf);
  border:3px solid var(--ground);margin-top:-6.5px;
  box-shadow:0 0 12px color-mix(in srgb,var(--leaf) 70%,transparent)}
input[type=range]::-moz-range-track{height:3px;border-radius:3px;background:var(--track)}
input[type=range]::-moz-range-thumb{width:16px;height:16px;border-radius:50%;
  background:var(--leaf);border:3px solid var(--ground)}
.acts{display:flex;gap:8px;margin-top:14px}
.btn{flex:1;padding:11px 12px;border-radius:11px;cursor:pointer;
  font-family:'Space Grotesk',sans-serif;font-size:13.5px;font-weight:500;
  letter-spacing:.01em;border:1px solid var(--edge);background:var(--pane-2);
  color:var(--text);transition:background .2s,border-color .2s,transform .12s,opacity .2s}
.btn:hover{border-color:var(--leaf)}
.btn:active{transform:scale(.975)}
.btn.primary{background:var(--leaf);background:linear-gradient(135deg,var(--leaf),color-mix(in srgb,var(--beam) 55%,var(--leaf)));
  border-color:transparent;color:#08130E;font-weight:600}
html[data-theme=day] .btn.primary{color:#fff}
.btn.primary:hover{filter:brightness(1.08)}
.btn.warn{color:var(--bloom);border-color:color-mix(in srgb,var(--bloom) 45%,transparent)}
.btn.warn:hover{background:var(--bloom);color:#fff;border-color:var(--bloom)}
.btn:disabled{opacity:.35;cursor:not-allowed;transform:none;filter:none}
.btn:disabled:hover{border-color:var(--edge);background:var(--pane-2);color:var(--text)}
.btn.primary:disabled:hover{background:linear-gradient(135deg,var(--leaf),var(--beam));color:#08130E}

/* ---------- climate + shade --------------------------------------- */
.climate{display:grid;grid-template-columns:1fr 1fr;gap:18px}
.metric .k{font-family:'Space Mono',monospace;font-size:10px;letter-spacing:.18em;
  text-transform:uppercase;color:var(--muted)}
.metric .v{font-family:'Fraunces',Georgia,serif;font-weight:300;font-size:34px;
  line-height:1.1;margin-top:2px;letter-spacing:-.02em}
.metric .v span{font-family:'Space Grotesk',sans-serif;font-size:13px;
  color:var(--muted);margin-left:3px;font-weight:400}
.bar{height:3px;border-radius:3px;background:var(--track);margin-top:9px;overflow:hidden}
.bar i{display:block;height:100%;width:0;border-radius:3px;
  background:linear-gradient(90deg,var(--beam),var(--bloom));
  transition:width .8s cubic-bezier(.16,.84,.34,1)}
.shade{margin-top:20px;padding-top:18px;border-top:1px solid var(--edge-soft)}
.shade-dial{width:100%;max-width:230px;height:auto;display:block;margin:0 auto 6px}
.shade-dial .arc{fill:none;stroke:var(--track);stroke-width:5;stroke-linecap:round}
.shade-dial .arc-lit{fill:none;stroke:var(--amber);stroke-width:5;stroke-linecap:round;
  stroke-dasharray:251;stroke-dashoffset:251;transition:stroke-dashoffset .5s ease}
.shade-dial .arm{fill:var(--text);transition:transform .5s cubic-bezier(.16,.84,.34,1)}
.shade-dial .hub{fill:var(--text)}
.shade-presets{display:flex;gap:6px;margin-top:12px;flex-wrap:wrap}
.shade-presets .chip{flex:1;justify-content:center;min-width:56px}

/* ---------- deck (remote control) --------------------------------- */
.deck .line{display:flex;align-items:center;justify-content:space-between;
  gap:12px;padding:13px 0;border-bottom:1px solid var(--edge-soft)}
.deck .line:last-of-type{border-bottom:none}
.deck .line .t{font-size:13.5px;color:var(--text-2)}
.deck .line .t small{display:block;color:var(--muted);font-size:11.5px;margin-top:2px}
select{padding:8px 11px;border-radius:9px;border:1px solid var(--edge);
  background:var(--pane-2);color:var(--text);font-family:'Space Grotesk',sans-serif;
  font-size:13px;cursor:pointer}
.deck .row-btns{display:flex;gap:8px;margin-top:16px}
.raw{margin-top:16px;padding-top:16px;border-top:1px solid var(--edge-soft)}
.raw textarea{width:100%;min-height:76px;resize:vertical;padding:11px 13px;
  border-radius:11px;border:1px solid var(--edge);background:var(--ground-2);
  color:var(--text);font-family:'Space Mono',monospace;font-size:12px;line-height:1.5}

/* ---------- examine ------------------------------------------------ */
.exam-controls{display:flex;gap:10px;flex-wrap:wrap}
.exam-controls select{flex:1;min-width:190px}
.exam-controls .btn{flex:0 0 auto;min-width:190px}
.exam-out{margin-top:16px;border:1px solid var(--edge);border-radius:14px;
  background:var(--pane-2);padding:18px 20px;min-height:92px}
.exam-empty{color:var(--muted);font-size:14px}
.exam-species{font-family:'Fraunces',Georgia,serif;font-weight:400;font-size:28px;
  line-height:1.15;letter-spacing:-.015em}
.exam-sub{font-family:'Space Mono',monospace;font-size:10px;letter-spacing:.12em;
  text-transform:uppercase;color:var(--muted);margin-top:5px}
.badges{display:flex;flex-wrap:wrap;gap:8px;margin-top:14px}
.badge{font-family:'Space Mono',monospace;font-size:10px;letter-spacing:.14em;
  text-transform:uppercase;padding:5px 12px;border-radius:999px;
  border:1px solid var(--edge);color:var(--text-2)}
.badge.good{background:var(--leaf);border-color:var(--leaf);color:#08130E}
.badge.bad{background:var(--bloom);border-color:var(--bloom);color:#fff}
.badge.warn{background:var(--amber);border-color:var(--amber);color:#1A1206}
.exam-topk{margin-top:12px;font-family:'Space Mono',monospace;font-size:10.5px;
  color:var(--muted);letter-spacing:.04em}

/* ---------- chart --------------------------------------------------- */
.seg{display:inline-flex;border:1px solid var(--edge);border-radius:999px;
  overflow:hidden;background:var(--pane-2)}
.seg button{padding:7px 13px;border:none;background:transparent;color:var(--muted);
  font-family:'Space Mono',monospace;font-size:10px;letter-spacing:.12em;
  text-transform:uppercase;cursor:pointer;transition:color .2s,background .2s}
.seg button.on{color:var(--text);background:color-mix(in srgb,var(--beam) 22%,transparent)}
.chart-box{position:relative;height:300px;margin-top:18px}
.chart-note{position:absolute;inset:0;display:grid;place-items:center;text-align:center;
  padding:0 26px;color:var(--muted);font-size:13.5px;line-height:1.6}
#chart{width:100% !important;height:300px !important}

/* ---------- lists --------------------------------------------------- */
.feed{list-style:none;max-height:290px;overflow-y:auto;margin-top:4px;
  scrollbar-width:thin}
.feed li{display:grid;grid-template-columns:58px 92px 1fr;gap:12px;
  align-items:baseline;padding:9px 0;border-bottom:1px solid var(--edge-soft);
  font-size:13.5px}
.feed li:last-child{border-bottom:none}
.feed .t{font-family:'Space Mono',monospace;font-size:10px;color:var(--muted)}
.feed .k{font-family:'Space Mono',monospace;font-size:9.5px;letter-spacing:.14em;
  text-transform:uppercase;color:var(--leaf)}
.feed .k.alarm_set,.feed .k.disconnect,.feed .k.command_failed{color:var(--bloom)}
.feed .k.connect,.feed .k.alarm_clear{color:var(--beam)}
.feed .k.boot,.feed .k.classify{color:var(--amber)}
.feed .d{color:var(--text-2)}
#examFeed li{grid-template-columns:58px 1fr 84px}
#examFeed .k{text-align:right}
#examFeed .k.good{color:var(--leaf)}
#examFeed .k.bad{color:var(--bloom)}
#examFeed .k.warn{color:var(--amber)}
.chatter{font-family:'Space Mono',monospace;font-size:11.5px;line-height:1.75;
  max-height:290px;overflow-y:auto;color:var(--text-2);
  white-space:pre-wrap;word-break:break-word}
.chatter .ts{color:var(--muted)}
.chatter .empty{color:var(--muted);font-style:italic}

/* ---------- viewer (fullscreen camera) ------------------------------ */
.viewer{position:fixed;inset:0;z-index:60;display:none;
  background:rgba(6,4,14,.94);backdrop-filter:blur(8px)}
.viewer.show{display:flex;flex-direction:column;animation:fade .25s ease}
@keyframes fade{from{opacity:0}to{opacity:1}}
.viewer-bar{display:flex;align-items:center;gap:10px;padding:14px 18px;
  color:#fff;flex-wrap:wrap}
.viewer-bar .grow{flex:1}
.viewer .chip{background:rgba(255,255,255,.08);border-color:rgba(255,255,255,.22);
  color:#fff}
.viewer .chip:hover{border-color:#fff}
.stage{flex:1;overflow:hidden;display:grid;place-items:center;cursor:grab;
  touch-action:none;padding:0 12px 18px}
.stage.dragging{cursor:grabbing}
.stage img{max-width:100%;max-height:100%;object-fit:contain;
  border-radius:10px;transition:transform .12s ease-out;will-change:transform}

/* ---------- key modal ------------------------------------------------ */
.modal{position:fixed;inset:0;z-index:70;display:none;place-items:center;
  padding:22px;background:rgba(6,4,14,.7);backdrop-filter:blur(6px)}
.modal.show{display:grid;animation:fade .2s ease}
.modal .box{width:min(430px,100%);background:var(--ground-2);
  border:1px solid var(--edge);border-radius:18px;padding:24px;
  box-shadow:var(--shadow)}
.modal h3{font-family:'Fraunces',serif;font-weight:400;font-size:24px;margin-bottom:8px}
.modal p{color:var(--muted);font-size:13.5px;margin-bottom:16px}
.modal input{width:100%;padding:12px 14px;border-radius:11px;
  border:1px solid var(--edge);background:var(--pane-2);color:var(--text);
  font-family:'Space Mono',monospace;font-size:13px}
.modal .row-btns{display:flex;gap:8px;margin-top:16px}

/* ---------- toasts ---------------------------------------------------- */
.toasts{position:fixed;left:50%;bottom:24px;transform:translateX(-50%);
  z-index:80;display:flex;flex-direction:column;gap:8px;align-items:center;
  width:min(440px,calc(100vw - 32px));pointer-events:none}
.toast{width:100%;padding:12px 16px;border-radius:12px;font-size:13.5px;
  background:var(--ground-2);border:1px solid var(--edge);color:var(--text);
  box-shadow:var(--shadow);animation:toast-in .3s cubic-bezier(.16,.84,.34,1)}
.toast.good{border-color:color-mix(in srgb,var(--leaf) 55%,transparent)}
.toast.warn{border-color:color-mix(in srgb,var(--amber) 55%,transparent)}
.toast.bad{border-color:color-mix(in srgb,var(--bloom) 55%,transparent)}
.toast.out{animation:toast-out .35s ease forwards}
@keyframes toast-in{from{opacity:0;transform:translateY(14px)}to{opacity:1;transform:none}}
@keyframes toast-out{to{opacity:0;transform:translateY(10px)}}

/* ---------- footer ------------------------------------------------------ */
footer{margin-top:56px;padding-top:22px;border-top:1px solid var(--edge-soft);
  display:flex;justify-content:space-between;gap:16px;flex-wrap:wrap;
  font-family:'Space Mono',monospace;font-size:10.5px;letter-spacing:.12em;
  text-transform:uppercase;color:var(--muted)}

/* ---------- responsive --------------------------------------------------- */
@media (max-width:960px){
  .beds{grid-template-columns:1fr;gap:16px}
  .split{grid-template-columns:1fr}
  .ring-wrap{max-width:170px}
}
@media (max-width:620px){
  .shell{padding:0 14px 70px}
  .cam-frame{aspect-ratio:4/3;max-height:none}
  .climate{grid-template-columns:1fr 1fr;gap:14px}
  .exam-controls .btn,.exam-controls select{min-width:100%}
  .rail .mark .name{font-size:16px}
  .feed li{grid-template-columns:52px 1fr;gap:6px}
  .feed .k{grid-column:2}
  .feed .d{grid-column:2}
}
@media (prefers-reduced-motion:reduce){
  *,*::before,*::after{animation-duration:.001ms !important;
    animation-iteration-count:1 !important;transition-duration:.001ms !important}
  .reveal{opacity:1;transform:none}
}
</style>
</head>
<body>

<div class="room" aria-hidden="true">
  <div class="lamp red"></div>
  <div class="lamp blue"></div>
  <div class="lamp grow"></div>
</div>
<div class="grain" aria-hidden="true"></div>
<div class="motes" id="motes" aria-hidden="true"></div>

<div class="shell">

  <div class="rail">
    <div class="mark">
      <svg viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
        <path d="M12 2c-1 4-4 6-8 6 0 6 4 12 8 14 4-2 8-8 8-14-4 0-7-2-8-6z"/>
      </svg>
      <span class="name">Hortus <em>Vigilis</em></span>
    </div>
    <div class="spacer"></div>
    <span class="pill" id="conn"><span class="dot"></span><span id="connText">connecting</span></span>
    <button type="button" class="icon-btn" id="themeBtn" title="Switch to daylight" aria-label="Switch theme">☀</button>
    <button type="button" class="icon-btn" id="keyBtn" title="Access key" aria-label="Access key">⚿</button>
  </div>

  <section class="hero-type reveal">
    <h1>Three beds,<br>one <em>watchful</em> eye.</h1>
    <p>Soil, air and foliage from the small circular garden — read every few seconds,
       and controllable from wherever you are standing.</p>
    <div class="readout">
      <span id="dateLabel">—</span>
      <span><b id="clockLabel">—:—:—</b> local</span>
      <span>pi up <b id="uptimeLabel">—</b></span>
      <span>heard from mega <b id="seenLabel">—</b></span>
    </div>
  </section>

  <article class="panel pane-card reveal" style="animation-delay:.06s">
    <div class="panel-head">
      <div>
        <div class="eyebrow">The pane</div>
        <h2>What the camera <em>sees</em></h2>
      </div>
      <div class="cam-tools">
        <button type="button" class="chip" id="camLiveBtn">live view</button>
        <button type="button" class="chip" id="camShotBtn">take a frame</button>
        <button type="button" class="chip" id="camZoomBtn">open large</button>
      </div>
    </div>
    <div class="cam-frame" id="camFrame">
      <img id="camImg" alt="Live view of the garden" style="display:none">
      <div class="cam-none" id="camNone">
        <div><b>No picture yet</b>Plug in the USB camera, or install python3-opencv on the Pi.</div>
      </div>
      <div class="cam-hud"><span class="hud" id="camHud"><span class="dot"></span><span id="camHudText">waiting</span></span></div>
      <div class="cam-bar">
        <span id="camCaption">—</span>
        <span>tap to open · scroll to zoom</span>
      </div>
    </div>
  </article>

  <div class="grid beds" id="beds">
    {% for z in zones %}
    <article class="panel bed reveal" data-zone="{{ loop.index0 }}"
             style="animation-delay:{{ '%.2f'|format(0.12 + 0.06 * loop.index0) }}s">
      <div class="bed-head">
        <span class="eyebrow">Bed {{ loop.index }}</span>
        <span class="tag" data-role="tag">idle</span>
      </div>
      <h3>{{ z }}</h3>
      <div class="ring-wrap">
        <svg class="ring" viewBox="0 0 120 120" aria-hidden="true">
          <circle class="ring-track" cx="60" cy="60" r="52"></circle>
          <circle class="ring-fill" cx="60" cy="60" r="52"></circle>
        </svg>
        <div class="ring-read">
          <span class="n" data-role="moist">—</span>
          <span class="u">soil moisture</span>
        </div>
      </div>
      <div class="bed-stats">
        <span data-role="fails">0 failed cycles</span>
        <span data-role="sensor">sensor idle</span>
      </div>
      <div class="dose">
        <div class="lab"><span>pour for</span><b data-role="mslabel">3.0s</b></div>
        <input type="range" class="ms" min="{{ ms_min }}" max="{{ ms_max }}" step="100"
               value="{{ ms_default }}" aria-label="Watering duration">
      </div>
      <div class="acts">
        <button type="button" class="btn primary" data-act="water">Water now</button>
        <button type="button" class="btn warn" data-act="clear" disabled>Clear alarm</button>
      </div>
    </article>
    {% endfor %}
  </div>

  <div class="grid split">
    <article class="panel reveal" style="animation-delay:.3s">
      <div class="eyebrow">Air &amp; shade</div>
      <h2>The <em>weather</em> under glass</h2>
      <div class="climate">
        <div class="metric">
          <div class="k">Temperature</div>
          <div class="v" id="mTemp">—<span>°C</span></div>
          <div class="bar"><i id="bTemp"></i></div>
        </div>
        <div class="metric">
          <div class="k">Humidity</div>
          <div class="v" id="mHum">—<span>%</span></div>
          <div class="bar"><i id="bHum"></i></div>
        </div>
        <div class="metric">
          <div class="k">Light</div>
          <div class="v" id="mLight">—<span>%</span></div>
          <div class="bar"><i id="bLight"></i></div>
        </div>
        <div class="metric">
          <div class="k">Shade sheet</div>
          <div class="v" id="mServo">—<span>°</span></div>
          <div class="bar"><i id="bServo"></i></div>
        </div>
      </div>
      <div class="shade">
        <svg class="shade-dial" viewBox="0 0 200 122" aria-hidden="true">
          <path class="arc" d="M20 105 A80 80 0 0 1 180 105"></path>
          <path class="arc-lit" id="shadeArc" d="M20 105 A80 80 0 0 1 180 105"></path>
          <g id="shadeArm" class="arm" transform="rotate(-90 100 105)">
            <rect x="97" y="34" width="6" height="68" rx="3"></rect>
          </g>
          <circle class="hub" cx="100" cy="105" r="5"></circle>
        </svg>
        <div class="lab" style="display:flex;justify-content:space-between;
             font-family:'Space Mono',monospace;font-size:10px;letter-spacing:.16em;
             text-transform:uppercase;color:var(--muted);margin-bottom:8px">
          <span>drag to draw the sheet</span><b id="servoLbl" style="color:var(--text)">0°</b>
        </div>
        <input type="range" id="servo" min="0" max="180" value="0" aria-label="Shade sheet angle">
        <div class="shade-presets">
          <button type="button" class="chip" data-servo="0">open</button>
          <button type="button" class="chip" data-servo="45">45°</button>
          <button type="button" class="chip" data-servo="90">half</button>
          <button type="button" class="chip" data-servo="135">135°</button>
          <button type="button" class="chip" data-servo="180">shut</button>
        </div>
      </div>
    </article>

    <article class="panel deck reveal" style="animation-delay:.36s">
      <div class="eyebrow">From here</div>
      <h2>Remote <em>controls</em></h2>
      <div class="line">
        <div class="t">Reading rate<small>How often this page asks the Pi</small></div>
        <select id="pollSel">
          <option value="1000">every 1s</option>
          <option value="3000" selected>every 3s</option>
          <option value="10000">every 10s</option>
          <option value="30000">every 30s</option>
        </select>
      </div>
      <div class="line">
        <div class="t">Still-frame rate<small>Used when the live view is off</small></div>
        <select id="camSel">
          <option value="3000">every 3s</option>
          <option value="10000" selected>every 10s</option>
          <option value="30000">every 30s</option>
          <option value="0">only on request</option>
        </select>
      </div>
      <div class="line">
        <div class="t">Access key<small id="keyState">not required</small></div>
        <button type="button" class="chip" id="keyEditBtn">manage</button>
      </div>
      <div class="row-btns">
        <button type="button" class="btn warn" id="clearAllBtn">Clear every alarm</button>
        <button type="button" class="btn" id="refreshBtn">Refresh everything</button>
      </div>
      {% if raw_enabled %}
      <div class="raw">
        <div class="eyebrow" style="margin-bottom:8px">Direct to the Mega</div>
        <textarea id="rawBox" spellcheck="false"
          placeholder='{"cmd":"set_servo","angle":90}'></textarea>
        <div class="row-btns">
          <button type="button" class="btn" id="rawBtn">Send this line</button>
        </div>
        <div class="eyebrow" style="margin-top:10px;text-transform:none;letter-spacing:.04em">
          Watering is not accepted here — use the beds above so the pump cooldown holds.
        </div>
      </div>
      {% endif %}
    </article>
  </div>

  <article class="panel reveal" style="margin-top:20px;animation-delay:.42s">
    <div class="panel-head">
      <div>
        <div class="eyebrow">Examination</div>
        <h2>Name it, and <em>judge its vigour</em></h2>
      </div>
    </div>
    <div class="exam-controls">
      <select id="examZone" aria-label="Which bed is in view">
        <option value="">whole view</option>
        {% for z in zones %}
        <option value="{{ loop.index0 }}">{{ z }}</option>
        {% endfor %}
      </select>
      <button type="button" class="btn primary" id="examBtn">Examine what's in view</button>
    </div>
    <div class="exam-out" id="examOut">
      <div class="exam-empty">Point the camera at one plant and press examine. The result also
        goes to the Mega's screen.</div>
    </div>
    <ul class="feed" id="examFeed" style="margin-top:14px"></ul>
  </article>

  <article class="panel reveal" style="margin-top:20px;animation-delay:.48s">
    <div class="panel-head">
      <div>
        <div class="eyebrow">Chronicle</div>
        <h2>How the soil has <em>run dry</em></h2>
      </div>
      <div style="display:flex;gap:8px;flex-wrap:wrap">
        <div class="seg" id="rangeSeg">
          <button type="button" data-h="6">6h</button>
          <button type="button" data-h="24" class="on">24h</button>
          <button type="button" data-h="72">3d</button>
          <button type="button" data-h="168">7d</button>
        </div>
        <div class="seg" id="metricSeg">
          <button type="button" data-m="soil" class="on">soil</button>
          <button type="button" data-m="air">air</button>
        </div>
      </div>
    </div>
    <div class="chart-box">
      <canvas id="chart"></canvas>
      <div class="chart-note" id="chartEmpty" style="display:none"></div>
    </div>
  </article>

  <div class="grid split">
    <article class="panel reveal" style="animation-delay:.54s">
      <div class="eyebrow">Journal</div>
      <h2>What has <em>happened</em></h2>
      <ul class="feed" id="events">
        <li><span class="t">—</span><span class="k">—</span><span class="d">nothing recorded yet</span></li>
      </ul>
    </article>
    <article class="panel reveal" style="animation-delay:.6s">
      <div class="eyebrow">Instrument chatter</div>
      <h2>The Mega, <em>talking</em></h2>
      <div class="chatter" id="chatter"><span class="empty">no lines yet</span></div>
    </article>
  </div>

  <footer>
    <span>Hortus Vigilis · plant care controller</span>
    <span id="footHint">reading the garden</span>
  </footer>
</div>

<div class="viewer" id="viewer" role="dialog" aria-modal="true" aria-label="Enlarged garden view">
  <div class="viewer-bar">
    <span class="hud"><span class="dot"></span><span id="vwHud">live</span></span>
    <span class="grow"></span>
    <button type="button" class="chip" id="vwOut">−</button>
    <button type="button" class="chip" id="vwLevel">100%</button>
    <button type="button" class="chip" id="vwIn">+</button>
    <button type="button" class="chip" id="vwClose">close</button>
  </div>
  <div class="stage" id="stage"></div>
</div>

<div class="modal" id="keyModal" role="dialog" aria-modal="true" aria-label="Access key">
  <div class="box">
    <h3>Access key</h3>
    <p id="keyMsg">This garden is locked. Paste the key you set in PLANT_API_KEY.</p>
    <input id="keyInput" type="password" autocomplete="off" spellcheck="false" placeholder="key">
    <div class="row-btns">
      <button type="button" class="btn primary" id="keySave">Save key</button>
      <button type="button" class="btn" id="keyForget">Forget it</button>
      <button type="button" class="btn" id="keyCancel">Cancel</button>
    </div>
  </div>
</div>

<div class="toasts" id="toasts" aria-live="polite"></div>

<script>
const ZONES        = {{ zones | tojson }};
const API_REQUIRED = {{ api_required | tojson }};
const COOLDOWN_S   = {{ cooldown | tojson }};
const RAW_ENABLED  = {{ raw_enabled | tojson }};
const MS_DEFAULT   = {{ ms_default | tojson }};
const RING_C       = 326.7;
let chart = null;          // declared early: applyTheme() may rebuild it

const $  = (q, r) => (r || document).querySelector(q);
const $$ = (q, r) => Array.from((r || document).querySelectorAll(q));
const LS = window.localStorage;
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

// A garbled-but-parseable serial line reaches us as-is, so nothing below
// trusts the shape of a reading. num() rejects NaN/Infinity/objects,
// triple() guarantees three slots, alarmOn() refuses to raise an alarm on
// a value that isn't really a number.
const num = v => {
  if (typeof v === 'number') return isFinite(v) ? v : null;
  if (typeof v === 'string' && v.trim() !== '' && isFinite(Number(v))) return Number(v);
  return null;
};
const triple  = v => Array.isArray(v) ? [v[0], v[1], v[2]] : [null, null, null];
const alarmOn = v => (typeof v === 'number' && isFinite(v)) ? v !== 0 : v === true;
const count   = v => (typeof v === 'number' && isFinite(v) && v > 0) ? Math.round(v) : 0;

function normStatus(s){
  if (!s || typeof s !== 'object' || Array.isArray(s)) return null;
  return {
    state:  (s.state == null) ? null : String(s.state),
    zone:   (typeof s.zone === 'number' && isFinite(s.zone)) ? s.zone : -1,
    m:      triple(s.m).map(num),
    alarms: triple(s.alarms).map(alarmOn),
    fails:  triple(s.fails).map(count),
    temp:   num(s.temp), hum: num(s.hum), l: num(s.l),
  };
}
const setText = (el, v) => { if (el && el.textContent !== String(v)) el.textContent = String(v); };

/* ---------- ambient motes ---------- */
(function motes(){
  const box = $('#motes');
  for (let i = 0; i < 9; i++){
    const m = document.createElement('span');
    m.className = 'mote';
    m.style.left = (Math.random() * 100).toFixed(1) + 'vw';
    m.style.bottom = '-4vh';
    m.style.animationDuration = (38 + Math.random() * 34).toFixed(0) + 's';
    m.style.animationDelay = (-Math.random() * 40).toFixed(0) + 's';
    m.style.transform = 'scale(' + (0.6 + Math.random() * 1.4).toFixed(2) + ')';
    box.append(m);
  }
})();

/* ---------- toasts ---------- */
function toast(msg, kind, ms){
  const t = document.createElement('div');
  t.className = 'toast ' + (kind || '');
  t.textContent = msg;
  $('#toasts').append(t);
  setTimeout(() => { t.classList.add('out'); setTimeout(() => t.remove(), 400); }, ms || 3600);
}

/* ---------- theme ---------- */
function applyTheme(t){
  document.documentElement.dataset.theme = t;
  LS.setItem('hv_theme', t);
  const btn = $('#themeBtn');
  btn.textContent = (t === 'night') ? '☀' : '☾';
  btn.title = (t === 'night') ? 'Switch to daylight' : 'Switch to night';
  const c = getComputedStyle(document.documentElement).getPropertyValue('--ground').trim();
  $('#themeColor').setAttribute('content', c || '#0C0A16');
  if (chart){ chart.destroy(); chart = null; fetchHistory(); }
}
applyTheme(LS.getItem('hv_theme') || 'night');
$('#themeBtn').addEventListener('click', () =>
  applyTheme(document.documentElement.dataset.theme === 'night' ? 'day' : 'night'));

/* ---------- access key ---------- */
let keyPrompted = false;
function apiKey(){ return API_REQUIRED ? (LS.getItem('hv_key') || null) : null; }
function authHeaders(){ const k = apiKey(); return k ? { 'X-API-Key': k } : {}; }
function apiUrl(path, extra){
  const p = new URLSearchParams(extra || {});
  const k = apiKey();
  if (k) p.set('api_key', k);
  const q = p.toString();
  return q ? path + '?' + q : path;
}
function openKeyModal(msg){
  $('#keyMsg').textContent = msg || 'This garden is locked. Paste the key you set in PLANT_API_KEY.';
  $('#keyInput').value = LS.getItem('hv_key') || '';
  $('#keyModal').classList.add('show');
  setTimeout(() => $('#keyInput').focus(), 60);
}
function closeKeyModal(){ $('#keyModal').classList.remove('show'); }
function paintKeyState(){
  setText($('#keyState'), !API_REQUIRED ? 'not required on this Pi'
        : (apiKey() ? 'saved in this browser' : 'required — none saved'));
}
function handle401(r){
  if (r.status !== 401) return false;
  LS.removeItem('hv_key');
  paintKeyState();
  if (!keyPrompted){ keyPrompted = true; openKeyModal('That key was refused. Try another.'); }
  return true;
}
$('#keySave').addEventListener('click', () => {
  const v = $('#keyInput').value.trim();
  if (v) LS.setItem('hv_key', v); else LS.removeItem('hv_key');
  keyPrompted = false; closeKeyModal(); paintKeyState();
  toast(v ? 'Key saved. Reconnecting.' : 'Key cleared.', 'good');
  refreshAll();
});
$('#keyForget').addEventListener('click', () => {
  LS.removeItem('hv_key'); closeKeyModal(); paintKeyState(); toast('Key forgotten on this device.');
});
$('#keyCancel').addEventListener('click', closeKeyModal);
$('#keyBtn').addEventListener('click', () => openKeyModal(''));
$('#keyEditBtn').addEventListener('click', () => openKeyModal(''));
$('#keyInput').addEventListener('keydown', e => { if (e.key === 'Enter') $('#keySave').click(); });
paintKeyState();
if (API_REQUIRED && !apiKey()){ keyPrompted = true; openKeyModal(''); }

/* ---------- clock ---------- */
function tickClock(){
  const d = new Date();
  setText($('#clockLabel'), d.toLocaleTimeString('en-GB'));
  setText($('#dateLabel'),
    d.toLocaleDateString('en-GB', { weekday:'long', day:'numeric', month:'long' }));
}
setInterval(tickClock, 1000); tickClock();

function fmtTime(epoch){
  if (!epoch) return '—';
  return new Date(epoch * 1000).toLocaleTimeString('en-GB', { hour:'2-digit', minute:'2-digit' });
}
function ago(epoch){
  if (!epoch) return 'never';
  const s = Math.max(0, Math.floor(Date.now()/1000 - epoch));
  if (s < 60) return s + 's ago';
  if (s < 3600) return Math.floor(s/60) + 'm ago';
  return Math.floor(s/3600) + 'h ago';
}

/* ---------- commands ---------- */
async function sendCommand(body, label){
  try {
    const r = await fetch('/api/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json', ...authHeaders() },
      body: JSON.stringify(body),
    });
    if (handle401(r)) return false;
    let j = {};
    try { j = await r.json(); } catch (e) {}
    if (r.ok){
      if (label) toast(label, 'good', 2600);
      setTimeout(fetchStatus, 300);
      setTimeout(fetchEvents, 1200);
      return true;
    }
    toast(j.error || ('Refused (' + r.status + ')'), r.status === 429 ? 'warn' : 'bad', 5000);
    return false;
  } catch (e){
    toast('No answer from the Pi. Check the connection.', 'bad', 5000);
    return false;
  }
}

/* ---------- beds ---------- */
const beds = $$('.bed');
const cool = ZONES.map(() => ({ left: 0, timer: null }));
let lastSnap = null, piUp = true;

beds.forEach((card, i) => {
  const ms = $('.ms', card);
  const saved = Number(LS.getItem('hv_ms_' + i));
  ms.value = (saved >= Number(ms.min) && saved <= Number(ms.max)) ? saved : MS_DEFAULT;
  const paint = () => setText($('[data-role=mslabel]', card), (ms.value / 1000).toFixed(1) + 's');
  paint();
  ms.addEventListener('input', () => { paint(); LS.setItem('hv_ms_' + i, ms.value); });

  $('[data-act=water]', card).addEventListener('click', async () => {
    const dur = Number(ms.value);
    const ok = await sendCommand({ cmd:'water', zone:i, ms:dur },
      'Watering bed ' + (i + 1) + ' for ' + (dur / 1000).toFixed(1) + 's');
    if (ok) startCooldown(i);
  });
  $('[data-act=clear]', card).addEventListener('click', () =>
    sendCommand({ cmd:'clear_alarm', zone:i }, 'Alarm cleared on bed ' + (i + 1)));
});

function startCooldown(i){
  const c = cool[i];
  c.left = COOLDOWN_S;
  if (c.timer) clearInterval(c.timer);
  c.timer = setInterval(() => {
    c.left -= 1;
    if (c.left <= 0){ clearInterval(c.timer); c.timer = null; c.left = 0; }
    paintBeds();
  }, 1000);
  paintBeds();
}

function paintBeds(){
  const s = lastSnap && lastSnap.status;
  const online = piUp && !!(lastSnap && lastSnap.connected);
  beds.forEach((card, i) => {
    const wtr = $('[data-act=water]', card);
    const clr = $('[data-act=clear]', card);
    const alarm = !!(s && s.alarms && s.alarms[i]);
    const busy  = !!(s && s.state && s.state !== 'IDLE');
    if (cool[i].left > 0){
      wtr.disabled = true;
      setText(wtr, 'ready in ' + cool[i].left + 's');
    } else {
      wtr.disabled = !online || alarm || busy;
      setText(wtr, 'Water now');
    }
    clr.disabled = !online || !alarm;
  });
}

/* ---------- shade sheet: it dims the room ---------- */
const servo = $('#servo');
let servoTimer = null;
function paintShade(angle){
  const frac = angle / 180;
  document.documentElement.style.setProperty('--shade', (1 - frac * 0.8).toFixed(3));
  $('#shadeArm').setAttribute('transform', 'rotate(' + (angle - 90) + ' 100 105)');
  $('#shadeArc').style.strokeDashoffset = String(251 - 251 * frac);
  setText($('#servoLbl'), angle + '°');
  setText($('#mServo').firstChild, angle);
  $('#bServo').style.width = (frac * 100) + '%';
}
servo.addEventListener('input', () => {
  const a = Number(servo.value);
  paintShade(a);
  clearTimeout(servoTimer);
  servoTimer = setTimeout(() => sendCommand({ cmd:'set_servo', angle:a }, 'Shade sheet at ' + a + '°'), 260);
});
$$('.shade-presets .chip').forEach(b => b.addEventListener('click', () => {
  const a = Number(b.dataset.servo);
  servo.value = a; paintShade(a);
  sendCommand({ cmd:'set_servo', angle:a }, 'Shade sheet at ' + a + '°');
}));
paintShade(Number(servo.value));

/* ---------- status ---------- */
function setConn(cls, text){
  const p = $('#conn');
  p.classList.remove('online', 'down');
  if (cls) p.classList.add(cls);
  setText($('#connText'), text);
}

function ringColour(pct){
  const cs = getComputedStyle(document.documentElement);
  if (pct < 30) return cs.getPropertyValue('--bloom').trim();
  if (pct < 55) return cs.getPropertyValue('--amber').trim();
  return cs.getPropertyValue('--leaf').trim();
}

function applyStatus(d){
  d.status = normStatus(d.status);   // one predictable shape from here down
  lastSnap = d; piUp = true;
  setConn(d.connected ? 'online' : 'down', d.connected ? 'mega online' : 'mega silent');
  const u = d.uptime_s || 0;
  setText($('#uptimeLabel'), u < 60 ? u + 's'
    : u < 3600 ? Math.floor(u/60) + 'm'
    : Math.floor(u/3600) + 'h ' + Math.floor((u%3600)/60) + 'm');
  setText($('#seenLabel'), ago(d.last_seen));
  setText($('#footHint'), d.camera_enabled ? 'camera live · pi reachable' : 'pi reachable · no camera');

  renderChatter(d.recent_log || []);

  const s = d.status;
  if (!s){ paintBeds(); return; }

  beds.forEach((card, i) => {
    const raw = s.m[i];
    const lost = (raw === null || raw === -1);
    const pct = lost ? 0 : clamp(raw, 0, 100);
    const numEl  = $('[data-role=moist]', card);
    const fill   = $('.ring-fill', card);
    const tag    = $('[data-role=tag]', card);
    const alarm  = s.alarms[i];
    const active = (s.zone === i) && !!s.state && s.state !== 'IDLE';

    numEl.classList.toggle('lost', lost);
    setText(numEl, lost ? 'sensor lost' : Math.round(pct) + '%');
    fill.style.strokeDashoffset = String(RING_C * (1 - (lost ? 0 : pct / 100)));
    fill.style.stroke = lost ? getComputedStyle(document.documentElement)
      .getPropertyValue('--bloom').trim() : ringColour(pct);

    tag.classList.remove('busy', 'alarm');
    if (alarm){ tag.classList.add('alarm'); setText(tag, 'alarm'); }
    else if (active){
      tag.classList.add('busy');
      setText(tag, s.state === 'RUNNING' ? 'watering'
             : s.state === 'VERIFYING' ? 'verifying'
             : String(s.state).toLowerCase().replace(/_/g, ' '));
    } else setText(tag, 'idle');

    card.classList.toggle('watering', active && s.state === 'RUNNING');
    const f = s.fails[i];
    setText($('[data-role=fails]', card), f + (f === 1 ? ' failed cycle' : ' failed cycles'));
    setText($('[data-role=sensor]', card), lost ? 'check the probe' : 'probe reading');
  });

  const temp = (s.temp !== null && s.temp !== -99) ? s.temp : null;
  const hum  = (s.hum  !== null && s.hum  !== -1)  ? s.hum  : null;
  const lux  = (s.l    !== null && s.l    !== -1)  ? s.l    : null;
  setText($('#mTemp').firstChild, temp === null ? '—' : temp.toFixed(1));
  setText($('#mHum').firstChild,  hum  === null ? '—' : Math.round(hum));
  setText($('#mLight').firstChild, lux === null ? '—' : Math.round(lux));
  $('#bTemp').style.width  = temp === null ? '0%' : clamp(temp / 45 * 100, 0, 100) + '%';
  $('#bHum').style.width   = hum  === null ? '0%' : clamp(hum, 0, 100) + '%';
  $('#bLight').style.width = lux  === null ? '0%' : clamp(lux, 0, 100) + '%';

  paintBeds();
}

async function fetchStatus(){
  try {
    const r = await fetch('/api/status', { headers: authHeaders() });
    if (handle401(r)) return;
    if (!r.ok) throw new Error(r.statusText);
    applyStatus(await r.json());
  } catch (e){
    piUp = false;
    setConn('down', 'pi unreachable');
    setText($('#footHint'), 'cannot reach the pi');
    paintBeds();          // nothing can be commanded through a dead link
  }
}

/* ---------- chatter ---------- */
function renderChatter(lines){
  const box = $('#chatter');
  if (!Array.isArray(lines)) lines = [];
  if (!lines.length){
    if (box.dataset.sig !== 'empty'){
      box.replaceChildren();
      const s = document.createElement('span'); s.className = 'empty';
      s.textContent = 'no lines yet'; box.append(s); box.dataset.sig = 'empty';
    }
    return;
  }
  const tail = lines[lines.length - 1] || [];
  const sig = lines.length + '|' + tail[0] + '|' + tail[1];
  if (box.dataset.sig === sig) return;      // nothing new arrived; leave it alone
  // Follow the tail only if the reader is already at the bottom, so scrolling
  // back through the log isn't yanked away every poll.
  const atEnd = (box.scrollHeight - box.scrollTop - box.clientHeight) < 24;
  box.dataset.sig = sig;
  box.replaceChildren();
  for (const item of lines){
    if (!Array.isArray(item)) continue;
    const ts = document.createElement('span'); ts.className = 'ts';
    ts.textContent = fmtTime(item[0]) + '  ';
    const tx = document.createTextNode(String(item[1]) + '\n');
    box.append(ts, tx);
  }
  if (atEnd) box.scrollTop = box.scrollHeight;
}

/* ---------- journal ---------- */
async function fetchEvents(){
  try {
    const r = await fetch('/api/events?limit=25', { headers: authHeaders() });
    if (handle401(r) || !r.ok) return;
    const ev = await r.json();
    if (!ev || !ev.length) return;
    const ul = $('#events');
    ul.replaceChildren();
    for (const e of ev){
      const li = document.createElement('li');
      const t = document.createElement('span'); t.className = 't'; t.textContent = fmtTime(e.ts);
      const k = document.createElement('span');
      k.className = 'k ' + String(e.kind || '').replace(/[^a-z0-9_]/gi, '');
      k.textContent = String(e.kind || '').replace(/_/g, ' ');
      const d = document.createElement('span'); d.className = 'd';
      let txt = String(e.details || '');
      if (e.zone != null) txt += ' · bed ' + (Number(e.zone) + 1);
      d.textContent = txt;
      li.append(t, k, d);
      ul.append(li);
    }
  } catch (e){ /* keep the last good list */ }
}

/* ---------- chronicle ---------- */
let chartHours = 24, chartMetric = 'soil';

function cssVar(n){ return getComputedStyle(document.documentElement).getPropertyValue(n).trim(); }

function chartNote(msg){
  const el = $('#chartEmpty');
  if (msg){ el.textContent = msg; el.style.display = ''; }
  else el.style.display = 'none';
}

async function fetchHistory(){
  if (typeof Chart === 'undefined'){
    chartNote('The chart library did not load. This Pi may have no route to the internet — '
            + 'everything else on this page still works.');
    return;
  }
  try {
    const r = await fetch('/api/history?hours=' + chartHours, { headers: authHeaders() });
    if (handle401(r) || !r.ok) return;
    const rows = await r.json();
    if (!rows.length){
      if (chart){ chart.destroy(); chart = null; }
      chartNote('Nothing recorded in this window yet. Readings land in the database every '
              + 'half minute or so — try a wider range.');
      return;
    }
    chartNote('');
    const labels = rows.map(x => new Date(x.ts * 1000));
    const line = (key, colour, name, axis) => ({
      label: name, data: rows.map(x => x[key]), yAxisID: axis || 'y',
      borderColor: colour, backgroundColor: colour + '1f',
      borderWidth: 2, tension: .35, pointRadius: 0, spanGaps: true, fill: true,
    });
    const sets = (chartMetric === 'soil')
      ? [ line('m0', cssVar('--leaf'),  ZONES[0]),
          line('m1', cssVar('--bloom'), ZONES[1]),
          line('m2', cssVar('--beam'),  ZONES[2]) ]
      : [ line('temp',     cssVar('--amber'), 'temperature °C', 'y2'),
          line('humidity', cssVar('--beam'),  'humidity %'),
          line('light',    cssVar('--leaf'),  'light %') ];

    if (chart){
      chart.data.labels = labels;
      chart.data.datasets.forEach((d, i) => { d.data = sets[i].data; });
      chart.update('none');
      return;
    }
    const grid = cssVar('--edge-soft') || 'rgba(120,110,160,.2)';
    const tick = cssVar('--muted');
    const axes = {
      x: { type:'time', time:{ unit: chartHours > 48 ? 'day' : 'hour',
             displayFormats:{ hour:'HH:mm', day:'d MMM' } },
           grid:{ color: grid }, border:{ display:false },
           ticks:{ color: tick, font:{ family:'Space Mono', size:10 }, maxRotation:0 } },
      y: { min:0, max:100, grid:{ color: grid }, border:{ display:false },
           ticks:{ color: tick, font:{ family:'Space Mono', size:10 },
                   callback: v => v + '%' } },
    };
    if (chartMetric === 'air'){
      axes.y2 = { position:'right', min:0, max:50, grid:{ display:false },
                  border:{ display:false },
                  ticks:{ color: tick, font:{ family:'Space Mono', size:10 },
                          callback: v => v + '°' } };
    }
    chart = new Chart($('#chart'), {
      type:'line',
      data:{ labels, datasets: sets },
      options:{
        responsive:true, maintainAspectRatio:false,
        interaction:{ mode:'index', intersect:false },
        plugins:{
          legend:{ position:'bottom',
            labels:{ color: cssVar('--text-2'), boxWidth:14, boxHeight:2,
                     padding:16, font:{ family:'Space Grotesk', size:12 } } },
          tooltip:{ backgroundColor: cssVar('--ground-2'), borderColor: cssVar('--edge'),
                    borderWidth:1, titleColor: cssVar('--text'),
                    bodyColor: cssVar('--text-2'), padding:10, displayColors:true },
        },
        scales: axes,
      },
    });
  } catch (e){ if (!chart) chartNote('The chart could not be drawn.'); }
}

$$('#rangeSeg button').forEach(b => b.addEventListener('click', () => {
  $$('#rangeSeg button').forEach(x => x.classList.remove('on'));
  b.classList.add('on');
  chartHours = Number(b.dataset.h);
  if (chart){ chart.destroy(); chart = null; }
  fetchHistory();
  schedule();          // wider ranges get a slower refresh clock
}));
$$('#metricSeg button').forEach(b => b.addEventListener('click', () => {
  $$('#metricSeg button').forEach(x => x.classList.remove('on'));
  b.classList.add('on');
  chartMetric = b.dataset.m;
  if (chart){ chart.destroy(); chart = null; }
  fetchHistory();
}));

/* ---------- the camera ---------- */
let camWantLive = LS.getItem('hv_cam_live') !== '0';   // what the person chose
let camLive = camWantLive;                             // what we can manage right now
let camTimer = null, streamTimer = null, lastBlob = null, streamGaveUp = false;
let camGen = 0;      // bumped on every mode change; voids still-frames in flight
const BLOB_REVOKE_MS = 2000;
const STREAM_CYCLE_MS = 280000;

function camIdle(msg){
  $('#camImg').style.display = 'none';
  $('#camNone').style.display = '';
  setText($('#camHudText'), 'no signal');
  setText($('#camCaption'), msg || '—');
  // don't leave anyone staring at an empty fullscreen stage
  if ($('#viewer').classList.contains('show')) closeViewer();
}

function stopCamera(){
  camGen++;
  if (camTimer){ clearInterval(camTimer); camTimer = null; }
  if (streamTimer){ clearInterval(streamTimer); streamTimer = null; }
  const img = $('#camImg');
  img.onerror = null; img.onload = null;
}

function startStream(){
  const img = $('#camImg');
  img.onload = () => {
    img.style.display = ''; $('#camNone').style.display = 'none';
    setText($('#camHudText'), 'live'); setText($('#vwHud'), 'live');
    setText($('#camCaption'), 'streaming');
  };
  img.onerror = () => {
    if (streamGaveUp) return;
    streamGaveUp = true;
    camLive = false;        // this session only — the saved choice stays as it was
    paintCamButtons();
    // Don't nag when the Pi simply has no camera; the frame says so already.
    if (lastSnap && lastSnap.camera_enabled)
      toast('Live view refused — falling back to still frames.', 'warn', 5000);
    startCamera();
  };
  img.src = apiUrl('/camera/stream.mjpg', { t: Date.now() });
  streamTimer = setInterval(() => {
    img.src = apiUrl('/camera/stream.mjpg', { t: Date.now() });
  }, STREAM_CYCLE_MS);
}

async function grabStill(quiet){
  const gen = camGen;
  const img = $('#camImg');
  try {
    const r = await fetch('/snapshot.jpg', { headers: authHeaders(), cache:'no-store' });
    if (handle401(r)) throw new Error('unauthorized');
    if (r.status === 204) throw new Error('no frame');
    if (!r.ok) throw new Error('http ' + r.status);
    const blob = await r.blob();
    if (gen !== camGen) return;   // we switched to the live stream while waiting
    const url = URL.createObjectURL(blob);
    if (lastBlob){ const old = lastBlob; setTimeout(() => URL.revokeObjectURL(old), BLOB_REVOKE_MS); }
    lastBlob = url;
    img.src = url;
    img.style.display = ''; $('#camNone').style.display = 'none';
    setText($('#camHudText'), 'still'); setText($('#vwHud'), 'still');
    setText($('#camCaption'), 'taken ' + new Date().toLocaleTimeString('en-GB'));
  } catch (e){
    if (gen !== camGen) return;
    camIdle(quiet ? '—' : 'no frame available');
  }
}

function startCamera(){
  stopCamera();
  if (camLive){ startStream(); return; }
  grabStill(true);
  const every = Number(LS.getItem('hv_cam_ms') || 10000);
  if (every > 0) camTimer = setInterval(() => grabStill(true), every);
}

function paintCamButtons(){
  $('#camLiveBtn').classList.toggle('on', camLive);
  setText($('#camLiveBtn'), camLive ? 'live view on' : 'live view off');
}

$('#camLiveBtn').addEventListener('click', () => {
  camWantLive = !camLive; camLive = camWantLive; streamGaveUp = false;
  LS.setItem('hv_cam_live', camLive ? '1' : '0');
  paintCamButtons();
  startCamera();
  toast(camLive ? 'Live view on.' : 'Live view off — polling still frames.');
});
$('#camShotBtn').addEventListener('click', () => {
  // A still would replace the stream in the same <img> and stall the live view,
  // so while it is running this button re-syncs the stream instead.
  if (camLive){ startCamera(); toast('Live view re-synced.'); }
  else grabStill(false);
});
$('#camSel').addEventListener('change', e => {
  LS.setItem('hv_cam_ms', e.target.value);
  if (!camLive) startCamera();
});
(function initCamSel(){
  const v = LS.getItem('hv_cam_ms');
  if (v && $('#camSel').querySelector('option[value="' + v + '"]')) $('#camSel').value = v;
})();
paintCamButtons();

/* ---------- enlarged view: zoom + pan ---------- */
const viewer = $('#viewer'), stage = $('#stage');
let zoom = 1, panX = 0, panY = 0, drag = null;
const pointers = new Map();
let pinchStart = 0, pinchZoom = 1;

function applyTransform(){
  const img = $('#camImg');
  img.style.transform = 'translate(' + panX + 'px,' + panY + 'px) scale(' + zoom + ')';
  setText($('#vwLevel'), Math.round(zoom * 100) + '%');
}
function resetZoom(){ zoom = 1; panX = 0; panY = 0; applyTransform(); }
function setZoom(z){ zoom = clamp(z, 1, 6); if (zoom === 1){ panX = 0; panY = 0; } applyTransform(); }

function openViewer(){
  const img = $('#camImg');
  if (img.style.display === 'none') { toast('There is no picture to enlarge yet.', 'warn'); return; }
  stage.append(img);
  viewer.classList.add('show');
  document.body.style.overflow = 'hidden';
  resetZoom();
}
function closeViewer(){
  const img = $('#camImg');
  img.style.transform = '';
  $('#camFrame').prepend(img);
  viewer.classList.remove('show');
  document.body.style.overflow = '';
}
$('#camZoomBtn').addEventListener('click', openViewer);
$('#camFrame').addEventListener('click', e => { if (e.target.closest('.cam-tools')) return; openViewer(); });
$('#vwClose').addEventListener('click', closeViewer);
$('#vwIn').addEventListener('click', () => setZoom(zoom * 1.4));
$('#vwOut').addEventListener('click', () => setZoom(zoom / 1.4));
$('#vwLevel').addEventListener('click', resetZoom);
document.addEventListener('keydown', e => {
  if (e.key === 'Escape'){
    if (viewer.classList.contains('show')) closeViewer();
    else if ($('#keyModal').classList.contains('show')) closeKeyModal();
  }
  if (!viewer.classList.contains('show')) return;
  if (e.key === '+' || e.key === '=') setZoom(zoom * 1.4);
  if (e.key === '-') setZoom(zoom / 1.4);
  if (e.key === '0') resetZoom();
});
stage.addEventListener('wheel', e => {
  e.preventDefault();
  setZoom(zoom * (e.deltaY < 0 ? 1.12 : 1 / 1.12));
}, { passive:false });
stage.addEventListener('dblclick', () => setZoom(zoom > 1.05 ? 1 : 2.5));
stage.addEventListener('pointerdown', e => {
  stage.setPointerCapture(e.pointerId);
  pointers.set(e.pointerId, { x:e.clientX, y:e.clientY });
  if (pointers.size === 2){
    const [a, b] = Array.from(pointers.values());
    pinchStart = Math.hypot(a.x - b.x, a.y - b.y);
    pinchZoom = zoom;
    drag = null;
  } else if (pointers.size === 1 && zoom > 1){
    drag = { x:e.clientX - panX, y:e.clientY - panY };
    stage.classList.add('dragging');
  }
});
stage.addEventListener('pointermove', e => {
  if (!pointers.has(e.pointerId)) return;
  pointers.set(e.pointerId, { x:e.clientX, y:e.clientY });
  if (pointers.size === 2 && pinchStart > 0){
    const [a, b] = Array.from(pointers.values());
    setZoom(pinchZoom * (Math.hypot(a.x - b.x, a.y - b.y) / pinchStart));
  } else if (drag){
    panX = e.clientX - drag.x; panY = e.clientY - drag.y;
    applyTransform();
  }
});
function endPointer(e){
  pointers.delete(e.pointerId);
  if (pointers.size < 2) pinchStart = 0;
  if (pointers.size === 0){ drag = null; stage.classList.remove('dragging'); }
}
stage.addEventListener('pointerup', endPointer);
stage.addEventListener('pointercancel', endPointer);

/* ---------- examination ---------- */
function healthClass(h){
  const s = String(h || '').toLowerCase();
  if (s.includes('healthy') || s === 'thriving') return 'good';
  if (s.includes('disease') || s.includes('poor') || s.includes('blight') || s.includes('rot')) return 'bad';
  if (s.includes('stress') || s.includes('mild') || s.includes('moderate')) return 'warn';
  return '';
}

function renderExam(r){
  const box = $('#examOut');
  box.replaceChildren();
  const sp = document.createElement('div');
  sp.className = 'exam-species';
  sp.textContent = r.species || 'Species not identified';
  box.append(sp);

  const sub = document.createElement('div');
  sub.className = 'exam-sub';
  sub.textContent = (r.source === 'tflite' ? 'model reading' : 'colour estimate') + ' · '
    + new Date((r.ts || Date.now() / 1000) * 1000).toLocaleTimeString('en-GB');
  box.append(sub);

  const bs = document.createElement('div'); bs.className = 'badges';
  const hb = document.createElement('span');
  hb.className = 'badge ' + healthClass(r.health);
  hb.textContent = (r.condition && r.source === 'tflite'
    && String(r.condition).toLowerCase() !== 'healthy') ? r.condition : String(r.health || 'unknown');
  bs.append(hb);
  if (r.source === 'tflite' && typeof r.confidence === 'number'){
    const c = document.createElement('span'); c.className = 'badge';
    c.textContent = Math.round(r.confidence * 100) + '% sure'; bs.append(c);
  }
  if (r.health_score != null){
    const f = document.createElement('span'); f.className = 'badge';
    f.textContent = 'foliage ' + r.health_score + '%'; bs.append(f);
  }
  box.append(bs);

  if (r.topk && r.topk.length > 1){
    const tk = document.createElement('div'); tk.className = 'exam-topk';
    tk.textContent = 'also considered: ' + r.topk.slice(1, 3)
      .map(t => t.label + ' (' + Math.round(t.p * 100) + '%)').join(' · ');
    box.append(tk);
  }
}

async function runExam(){
  const btn = $('#examBtn');
  const zv = $('#examZone').value;
  const old = btn.textContent;
  btn.disabled = true; btn.textContent = 'Looking…';
  try {
    const r = await fetch('/api/classify', {
      method:'POST',
      headers:{ 'Content-Type':'application/json', ...authHeaders() },
      body: JSON.stringify(zv === '' ? {} : { zone: Number(zv) }),
    });
    if (handle401(r)){ /* modal already open */ }
    else if (r.ok){ renderExam(await r.json()); fetchExamFeed(); }
    else {
      const j = await r.json().catch(() => ({}));
      toast(j.error || 'The examiner could not answer.', r.status === 429 ? 'warn' : 'bad', 5000);
    }
  } catch (e){ toast('No answer from the Pi.', 'bad'); }
  finally { btn.disabled = false; btn.textContent = old; }
}
$('#examBtn').addEventListener('click', runExam);

async function fetchExamFeed(){
  try {
    const r = await fetch('/api/classifications?limit=8', { headers: authHeaders() });
    if (handle401(r) || !r.ok) return;
    const rows = await r.json();
    if (!rows || !rows.length) return;
    const ul = $('#examFeed');
    ul.replaceChildren();
    for (const c of rows){
      const li = document.createElement('li');
      const t = document.createElement('span'); t.className = 't'; t.textContent = fmtTime(c.ts);
      const d = document.createElement('span'); d.className = 'd';
      let txt = (c.species && c.species !== 'Unknown') ? c.species : 'health check';
      if (c.condition && String(c.condition).toLowerCase() !== 'healthy') txt += ' — ' + c.condition;
      if (c.zone != null) txt += ' · bed ' + (Number(c.zone) + 1);
      d.textContent = txt;
      const k = document.createElement('span');
      k.className = 'k ' + healthClass(c.health);
      k.textContent = c.health || '—';
      li.append(t, d, k);
      ul.append(li);
    }
  } catch (e){ /* keep the last list */ }
}

/* ---------- deck actions ---------- */
$('#clearAllBtn').addEventListener('click', async () => {
  const s = lastSnap && lastSnap.status;
  const armed = ZONES.map((_, i) => i).filter(i => s && s.alarms && s.alarms[i]);
  if (!armed.length){ toast('No alarms are ringing.'); return; }
  for (const i of armed) await sendCommand({ cmd:'clear_alarm', zone:i });
  toast('Cleared ' + armed.length + (armed.length === 1 ? ' alarm.' : ' alarms.'), 'good');
});
$('#refreshBtn').addEventListener('click', () => { refreshAll(); toast('Reading everything again.'); });

if (RAW_ENABLED){
  $('#rawBtn').addEventListener('click', async () => {
    const txt = $('#rawBox').value.trim();
    if (!txt){ toast('Type a JSON line first.', 'warn'); return; }
    let obj;
    try { obj = JSON.parse(txt); }
    catch (e){ toast('That is not valid JSON.', 'bad'); return; }
    await sendCommand({ cmd:'raw', payload: obj }, 'Sent to the Mega.');
  });
}

/* ---------- polling ---------- */
let timers = {};
function schedule(){
  Object.values(timers).forEach(clearInterval);
  const p = Number($('#pollSel').value);
  // A week of history is a far bigger read on the Pi than a status poll,
  // so wide ranges refresh on a slower clock.
  const histEvery = chartHours <= 24 ? 60000 : 300000;
  timers = {
    status: setInterval(fetchStatus, p),
    events: setInterval(fetchEvents, Math.max(p * 3, 10000)),
    hist:   setInterval(fetchHistory, histEvery),
  };
  LS.setItem('hv_poll', String(p));
}
(function initPoll(){
  const v = LS.getItem('hv_poll');
  if (v && $('#pollSel').querySelector('option[value="' + v + '"]')) $('#pollSel').value = v;
})();
$('#pollSel').addEventListener('change', () => { schedule(); toast('Reading every ' + (Number($('#pollSel').value) / 1000) + 's.'); });

function refreshAll(){
  fetchStatus(); fetchEvents(); fetchHistory(); fetchExamFeed();
  streamGaveUp = false; camLive = camWantLive; paintCamButtons(); startCamera();
}

document.addEventListener('visibilitychange', () => {
  if (document.visibilityState === 'visible') refreshAll();
});

refreshAll();
schedule();
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

    if len(ZONE_NAMES) != 3:
        log.warning(f"ZONE_NAMES has {len(ZONE_NAMES)} entries, but the database "
                    "columns, the Mega firmware and the API all assume 3. "
                    "Rename the three zones freely; adding or removing one "
                    "needs changes on both sides.")

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
    if camera.enabled:
        log.info(f"Live view: /camera/stream.mjpg "
                 f"({STREAM_FPS} fps, up to {STREAM_MAX_CLIENTS} viewers)")
    if classifier is not None:
        log.info(f"Classifier: {classifier.status}")
    else:
        log.info("Classifier: disabled (plant_classifier.py not importable)")
    if API_KEY:
        log.info("API auth:  ENABLED (X-API-Key required for /api/*, "
                 "/snapshot.jpg and the live view)")
    else:
        log.info("API auth:  disabled  (set PLANT_API_KEY env var to enable)")
    if ALLOW_RAW_CMD:
        log.warning("Raw command console ENABLED (PLANT_ALLOW_RAW_CMD). "
                    "Anything you type there goes straight to the Mega.")

    app.run(host=HTTP_HOST, port=HTTP_PORT,
            debug=False, use_reloader=False, threaded=True)


if __name__ == "__main__":
    main()
