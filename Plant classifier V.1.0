#!/usr/bin/env python3
# =====================================================================
#   PLANT CLASSIFIER  —  species + health for the Hortus Vigilis pi
#
#   Two backends, used together:
#
#     1. TFLiteClassifier — real CNN inference (e.g. a MobileNet /
#        EfficientNet trained on PlantVillage). Needs a .tflite model
#        + a labels.txt. Loads lazily in a background thread and is
#        guarded by a lock (TFLite interpreters are NOT reentrant).
#        PlantVillage labels look like "Tomato___Late_blight" /
#        "Tomato___healthy" and are parsed into species + condition.
#
#     2. analyze_health_heuristic — pure OpenCV colour analysis. No
#        model, no extra deps (OpenCV is already required). Segments
#        foliage (green + yellow + brown) and reports the yellow/brown
#        fraction (chlorosis / necrosis) as a coarse vigour score.
#        Cannot name a species, but always works.
#
#   Public entry point:
#       clf = build_classifier(model_path, labels_path, norm="unit")
#       result = clf.classify(jpeg_bytes, zone=0)   # -> dict
#
#   Result dict shape:
#     { ts, zone, species, condition, health, health_score (0..100),
#       confidence (0..1), source ("tflite"|"heuristic"), label,
#       topk: [{label, p}, ...], detail: {...} }
# =====================================================================

from __future__ import annotations

import logging
import threading
import time
from pathlib import Path

log = logging.getLogger("plant-pi.classifier")

try:
    import numpy as np
    import cv2
    _CV_OK = True
except Exception:                       # pragma: no cover
    _CV_OK = False


# --------------------------------------------------------------------- #
#  TFLite runtime loading — tolerate the three common packagings.
# --------------------------------------------------------------------- #
def _load_interpreter(model_path: str):
    errors = []
    try:
        from ai_edge_litert.interpreter import Interpreter   # current name
        return Interpreter(model_path=model_path)
    except Exception as e:
        errors.append(f"ai_edge_litert: {e}")
    try:
        from tflite_runtime.interpreter import Interpreter   # classic name
        return Interpreter(model_path=model_path)
    except Exception as e:
        errors.append(f"tflite_runtime: {e}")
    try:
        import tensorflow as tf                              # full TF fallback
        return tf.lite.Interpreter(model_path=model_path)
    except Exception as e:
        errors.append(f"tensorflow: {e}")
    raise RuntimeError("no TFLite runtime found — " + " | ".join(errors))


def _humanize(token: str) -> str:
    return token.replace("_", " ").strip().title()


def _parse_plantvillage(label: str):
    """'Tomato___Late_blight' -> ('Tomato', 'Late Blight', False).
       'Tomato___healthy'     -> ('Tomato', 'Healthy', True)."""
    raw = label.strip()
    if "___" in raw:
        sp, cond = raw.split("___", 1)
    elif "__" in raw:
        sp, cond = raw.split("__", 1)
    else:
        sp, cond = raw, ""
    species = _humanize(sp)
    healthy = cond.lower() in ("healthy", "health", "normal") or cond == ""
    condition = "Healthy" if healthy else _humanize(cond)
    return species, condition, healthy


def _decode(jpeg: bytes):
    arr = np.frombuffer(jpeg, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)   # BGR or None


# --------------------------------------------------------------------- #
#  Heuristic, model-free health estimate (colour segmentation).
# --------------------------------------------------------------------- #
def analyze_health_heuristic(jpeg: bytes) -> dict:
    if not _CV_OK:
        return {"health": "unknown", "health_score": None,
                "detail": {"reason": "opencv/numpy unavailable"}}
    img = _decode(jpeg)
    if img is None:
        return {"health": "unknown", "health_score": None,
                "detail": {"reason": "could not decode frame"}}

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H, S, V = hsv[..., 0], hsv[..., 1], hsv[..., 2]   # OpenCV hue is 0..179

    green  = (H >= 35) & (H <= 85) & (S >= 45) & (V >= 40)
    yellow = (H >= 20) & (H <  35) & (S >= 45) & (V >= 70)
    brown  = (H >=  8) & (H <  20) & (S >= 35) & (V >= 25) & (V <= 175)

    veg = green | yellow | brown
    veg_px   = int(veg.sum())
    total_px = img.shape[0] * img.shape[1]
    veg_frac = veg_px / max(total_px, 1)

    if veg_px < 0.02 * total_px:            # under 2% foliage in frame
        return {"health": "unknown", "health_score": None,
                "detail": {"reason": "little/no foliage in frame",
                           "veg_fraction": round(veg_frac, 4)}}

    stressed_px   = int((yellow | brown).sum())
    stressed_frac = stressed_px / max(veg_px, 1)
    score = max(0, min(100, int(round(100 * (1.0 - stressed_frac)))))

    if   score >= 85: health = "healthy"
    elif score >= 65: health = "mild stress"
    elif score >= 45: health = "moderate stress"
    else:             health = "poor"

    return {"health": health, "health_score": score,
            "detail": {"veg_fraction": round(veg_frac, 4),
                       "stressed_fraction": round(stressed_frac, 4)}}


# --------------------------------------------------------------------- #
#  TFLite classifier (lazy, thread-safe).
# --------------------------------------------------------------------- #
class TFLiteClassifier:
    def __init__(self, model_path: str, labels_path: str,
                 norm: str = "unit", top_k: int = 3):
        self.model_path  = model_path
        self.labels_path = labels_path
        self.norm        = norm          # "unit" -> /255 ; "signed" -> /127.5-1
        self.top_k       = top_k
        self._lock       = threading.Lock()
        self._load_lock  = threading.Lock()
        self._interp     = None
        self._in         = None
        self._out        = None
        self._hw         = (224, 224)
        self._labels: list[str] = []
        self._loaded      = False
        self._load_failed = False
        self.status       = "not loaded"

    @property
    def available(self) -> bool:
        if self._loaded:
            return True
        if self._load_failed:
            return False
        return (_CV_OK
                and Path(self.model_path).is_file()
                and Path(self.labels_path).is_file())

    def _ensure_loaded(self) -> bool:
        if self._loaded:
            return True
        if self._load_failed:
            return False
        # Serialize loading: the background warm-up thread and the first
        # classify() call must not both build an interpreter (a double load
        # wastes time and can briefly double memory — an OOM risk on a small
        # Pi). Re-check the flags after acquiring the lock.
        with self._load_lock:
            if self._loaded:
                return True
            if self._load_failed:
                return False
            if not (Path(self.model_path).is_file()
                    and Path(self.labels_path).is_file()):
                self.status = "model/labels file missing"
                return False
            try:
                interp = _load_interpreter(self.model_path)
                interp.allocate_tensors()
                inp = interp.get_input_details()[0]
                out = interp.get_output_details()[0]
                _, h, w, _ = inp["shape"]
                with open(self.labels_path, "r", encoding="utf-8") as fh:
                    labels = [ln.strip() for ln in fh if ln.strip()]
                self._interp, self._in, self._out = interp, inp, out
                self._hw = (int(h), int(w))
                self._labels = labels
                self._loaded = True
                self.status = (f"loaded ({len(labels)} classes, {w}x{h}, "
                               f"{getattr(inp['dtype'], '__name__', inp['dtype'])})")
                log.info("Classifier model loaded: %s", self.status)
                return True
            except Exception as e:
                self._load_failed = True
                self.status = f"load failed: {e}"
                log.warning("Classifier load failed: %s", e)
                return False

    def warm_up_async(self) -> None:
        threading.Thread(target=self._ensure_loaded, daemon=True,
                         name="ClassifierLoad").start()

    def _preprocess(self, img):
        h, w = self._hw
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        dt = self._in["dtype"]
        if dt == np.float32:
            x = img.astype(np.float32)
            x = (x / 127.5 - 1.0) if self.norm == "signed" else (x / 255.0)
        elif dt == np.int8:
            # int8 image models expect pixels shifted into [-128, 127]
            # (input zero-point ~128). A plain astype(int8) would wrap
            # values 128..255 into negatives and corrupt the input.
            x = (img.astype(np.int16) - 128).astype(np.int8)
        else:                              # uint8 (or other) — raw 0..255
            x = img.astype(dt)
        return np.expand_dims(x, 0)

    def classify(self, jpeg: bytes) -> dict | None:
        if not self._ensure_loaded():
            return None
        img = _decode(jpeg)
        if img is None:
            return None
        x = self._preprocess(img)
        with self._lock:
            self._interp.set_tensor(self._in["index"], x)
            self._interp.invoke()
            raw = self._interp.get_tensor(self._out["index"])[0]
        raw = raw.astype(np.float32)

        scale, zp = self._out.get("quantization", (0.0, 0))
        if self._out["dtype"] != np.float32 and scale:
            raw = scale * (raw - zp)

        if raw.min() < 0 or raw.sum() <= 0 or raw.max() > 1.0001:
            e = np.exp(raw - raw.max())          # treat as logits
            probs = e / e.sum()
        else:
            probs = raw / max(raw.sum(), 1e-9)   # already probabilities

        order = np.argsort(probs)[::-1][:self.top_k]
        top = [{"label": self._labels[i] if i < len(self._labels) else str(i),
                "p": float(probs[i])} for i in order]
        best = top[0]
        species, condition, healthy = _parse_plantvillage(best["label"])
        return {"species": species, "condition": condition, "healthy": healthy,
                "label": best["label"], "confidence": best["p"], "topk": top}


# --------------------------------------------------------------------- #
#  Facade: TFLite species/health + heuristic vigour cross-check.
# --------------------------------------------------------------------- #
class Classifier:
    def __init__(self, model_path: str, labels_path: str, **kw):
        self._tfl = TFLiteClassifier(model_path, labels_path, **kw)

    @property
    def available(self) -> bool:
        # Heuristic fallback only needs OpenCV, so we're "available" whenever
        # OpenCV is present — even with no model file on disk.
        return _CV_OK

    @property
    def status(self) -> str:
        if not _CV_OK:
            return "unavailable (OpenCV/numpy missing)"
        return f"model: {self._tfl.status}; heuristic: ready"

    def warm_up(self) -> None:
        self._tfl.warm_up_async()

    def classify(self, jpeg: bytes, zone: int | None = None) -> dict:
        ts = int(time.time())
        heur  = analyze_health_heuristic(jpeg)
        try:
            model = self._tfl.classify(jpeg) if self._tfl.available else None
        except Exception as e:
            # A loaded-but-faulty model (bad shape, corrupt op, etc.) must not
            # take the endpoint down — fall back to the colour heuristic.
            log.warning("model inference failed; using heuristic: %s", e)
            model = None

        if model:
            health = "healthy" if model["healthy"] else "diseased"
            return {
                "ts": ts, "zone": zone,
                "species": model["species"],
                "condition": model["condition"],
                "health": health,
                "health_score": heur.get("health_score"),
                "confidence": round(model["confidence"], 4),
                "source": "tflite",
                "label": model["label"],
                "topk": model["topk"],
                "detail": {**heur.get("detail", {}),
                           "heuristic_health": heur.get("health")},
            }

        # heuristic-only: no species, vigour from colour
        return {
            "ts": ts, "zone": zone,
            "species": "Unknown",
            "condition": None,
            "health": heur.get("health", "unknown"),
            "health_score": heur.get("health_score"),
            "confidence": 0.0,
            "source": "heuristic",
            "label": None,
            "topk": [],
            "detail": {**heur.get("detail", {}), "note": self._tfl.status},
        }


def build_classifier(model_path: str, labels_path: str,
                     norm: str = "unit") -> Classifier:
    clf = Classifier(model_path, labels_path, norm=norm)
    clf.warm_up()        # background load; no-op if files are absent
    return clf
