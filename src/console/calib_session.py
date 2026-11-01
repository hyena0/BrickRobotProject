"""Calibration session persistence for the 5C console.

Goal: stop repeating J1-J4 sensor homing + J5/J6 manual marking every
session. The brick program (Pybricks, ``reset_angle=False``) keeps its
accumulated encoder values while powered, so PC-side offsets stay
valid until a brick reboot. Reboot is detected with the ``uptime_ms``
field of the brick snapshot: uptime only grows within one boot, so a
current uptime below the saved one means the brick rebooted.

Session file: ``runs/calib/session.json`` (PC clock + boot markers +
offsets + park pose + human confirmation). Source of joint values is
always ``config/calibration.local.yaml`` via the console; this module
stores copies, never invents them.
"""

from __future__ import annotations

import datetime
import json
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[2]
SESSION_PATH = ROOT / "runs" / "calib" / "session.json"

# Park pose: 6.0 verified rest (arm out of the virtual frame, clear of
# the pedestal). Source: runs/vla/6.0/report.json rest_q_deg.
PARK_Q_DEG = {"J1": 25.58, "J2": 20.93, "J3": 17.17,
              "J4": -43.58, "J5": -87.46, "J6": 60.0}
PARK_LABEL = "park"
PARK_SPEED_DPS = 15.0
CONFIRM_TOL_DEG = 3.0


def _resolve_path(path) -> pathlib.Path:
    # NOTE: default arg would freeze SESSION_PATH at import time and
    # break monkeypatched test paths; resolve on every call instead.
    return pathlib.Path(SESSION_PATH if path is None else path)


def load_session(path: str | pathlib.Path | None = None) -> dict:
    """Last saved session, {} when none exists yet."""
    try:
        return json.loads(_resolve_path(path).read_text(
            encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def save_session(record: dict,
                path: str | pathlib.Path | None = None) -> dict:
    """Persist one session record (JSON-serializable)."""
    path = _resolve_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rec = dict(record)
    rec["saved_at"] = datetime.datetime.now().isoformat(timespec="seconds")
    path.write_text(json.dumps(rec, indent=1, ensure_ascii=False),
                    encoding="utf-8")
    return rec


def read_boot(handler) -> dict:
    """Current uptime_ms per brick; None when a brick doesn't answer."""
    out = {}
    for brick in ("left", "right"):
        try:
            state = handler.transport.send(brick, "read")
            out[brick] = (state or {}).get("uptime_ms")
        except Exception:
            out[brick] = None
    return out


def boot_status(saved_boot: dict, now_boot: dict) -> str:
    """same: both uptimes grew (no reboot). rebooted: any uptime went
    backwards, or any brick is silent now (cannot prove continuity).
    unknown: nothing saved or nothing answers now."""
    saved = dict(saved_boot or {})
    now = dict(now_boot or {})
    if not saved or not any(v is not None for v in now.values()):
        return "unknown"
    if any(v is None for v in now.values()):
        return "rebooted"
    for brick, was in saved.items():
        cur = now.get(brick)
        if was is None or cur is None:
            return "rebooted"
        try:
            if float(cur) < float(was):
                return "rebooted"
        except (TypeError, ValueError):
            return "rebooted"
    return "same"


def session_status(handler, saved: dict | None = None) -> dict:
    """One status object for the calibration page."""
    saved = load_session() if saved is None else saved
    now = read_boot(handler)
    return {"ok": True,
            "boot_now": now,
            "boot_saved": saved.get("boot", {}),
            "continuity": boot_status(saved.get("boot", {}), now),
            "has_offsets": bool(saved.get("offsets")),
            "park_q": saved.get("park_q", dict(PARK_Q_DEG)),
            "confirmed": bool(saved.get("confirmed")),
            "confirmed_at": saved.get("confirmed_at"),
            "saved_at": saved.get("saved_at")}
