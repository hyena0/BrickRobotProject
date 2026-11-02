"""Digital twin: slot colours -> virtual image -> hold pose.

The console calls :func:`run_twin` with the real slot layout typed in
by the user. It renders the fixed virtual camera frame, asks the VLA
policy for one chunk (or the expert IK pipeline when no checkpoint is
available — always labelled), and returns the PNG bytes, the chosen
target and the hold pose for the safety check + low-speed send.
"""

from __future__ import annotations

import json
import pathlib
import struct
import zlib

import numpy as np

import src.vla.demo as D
import src.vla.scene as S

ROOT = pathlib.Path(__file__).resolve().parents[2]
REST_Q = [25.58, 20.93, 17.17, -43.58, -87.46, 60.0]  # 6.0 rest_q_deg
TWIN_SLOTS = ROOT / "runs" / "vla" / "twin_slots.json"
DEFAULT_CKPT = ROOT / "runs" / "vla" / "ckpt-last"


def load_slots() -> dict:
    """Last typed slot colours, {} when never saved."""
    try:
        return json.loads(TWIN_SLOTS.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def save_slots(slot_colours: dict) -> dict:
    """Persist validated slot colours; raises ValueError on bad input."""
    clean = validate_slots(slot_colours)
    TWIN_SLOTS.parent.mkdir(parents=True, exist_ok=True)
    TWIN_SLOTS.write_text(json.dumps(clean, indent=1, ensure_ascii=False),
                          encoding="utf-8")
    return clean


def validate_slots(slot_colours: dict) -> dict:
    """Keep only S1..S6 with known colours; target implied, not stored."""
    ids = set(S.SLOT_IDS.values())
    pal = set(S.COLORS)
    clean = {}
    for sid, colour in dict(slot_colours or {}).items():
        if sid not in ids:
            raise ValueError("unknown slot: " + str(sid))
        if colour not in pal:
            raise ValueError("unknown colour: " + str(colour))
        clean[sid] = colour
    if not clean:
        raise ValueError("no slots given")
    return clean


def scene_from_slots(slot_colours: dict, target: str) -> dict:
    """Deterministic scene: blocks axis-aligned on the typed slots."""
    clean = validate_slots(slot_colours)
    if target not in clean.values():
        raise ValueError("target colour not on any slot")
    # SLOT_IDS maps (x, y) -> "S1"; invert once to id -> (x, y).
    inv = {sid: xy for xy, sid in S.SLOT_IDS.items()}
    blocks = []
    for sid in sorted(clean, key=lambda s: int(s[1:])):
        (x, y) = inv[sid]
        blocks.append({"colour": clean[sid], "x": x, "y": y,
                       "yaw_deg": 0.0, "slot": sid})
    return {"blocks": blocks, "target": target}


def png_bytes(px: np.ndarray) -> bytes:
    """Minimal RGB PNG encoder (same layout as scene.write_png)."""
    arr = np.clip(np.asarray(px), 0, 255).astype(np.uint8)
    h, w, _ = arr.shape
    raw = bytearray()
    for y in range(h):
        raw.append(0)
        raw.extend(arr[y].tobytes())

    def chunk(t: bytes, d: bytes) -> bytes:
        c = struct.pack(">I", len(d)) + t + d
        return c + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)

    ihdr = struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
            + chunk(b"IEND", b""))


def run_twin(slot_colours: dict, target: str,
             ckpt_dir: str | None = None) -> dict:
    """Full twin pass. Never moves hardware; motion is the console's job.

    Returns ``{"png": bytes, "target": ..., "slots": ...,
    "hold_q": [...], "policy": "vla"|"expert-fallback", "rec": {...}}``.
    """
    from src.kin.model import load_model

    scene = scene_from_slots(slot_colours, target)
    doc = load_model(str(ROOT / "models" / "robot_inferred.json"))
    tdoc = S.tip_doc(doc)
    cam = S.load_camera(str(ROOT / "config" / "sim_camera.yaml"))
    rest = list(REST_Q)
    policy = "expert-fallback"
    rec = {"status": "fallback"}
    hold_q = None
    ckpt = str(ckpt_dir or DEFAULT_CKPT)
    if pathlib.Path(ckpt).is_dir():
        try:
            from src.vla import local_infer as LI

            img = S.render_frame(scene, rest, doc, cam, None)
            out = LI.predict_chunk(
                ckpt, img, rest,
                f"Point to the {target} block.")
            assert len(out["actions"]) == 50, out.get("n_steps")
            hold_q = [float(v) for v in out["actions"][-1]]
            policy = "vla"
            rec = {"status": "pass", "load_s": out["load_s"],
                   "infer_s": out["infer_s"],
                   "state_source": out.get("state_source"),
                   "n_steps": out.get("n_steps")}
        except RuntimeError:
            pass
    if hold_q is None:
        slots = S.load_slot_seeds()
        mat = slots
        seeds = [p["seed"] for p in slots]
        tb = next(b for b in scene["blocks"]
                  if b["colour"] == target)
        seed = S.nearest_mat_seed(tb["x"], tb["y"], seeds, mat)
        rec, traj = D.run_demo(0, scene, rest, tdoc, seed)
        if traj is None:
            raise ValueError("twin IK failed: " + rec.get("status", "?"))
        hold_q = [float(v) for v in traj[-1]]
    img = S.render_frame(scene, rest, doc, cam, None)
    save_slots({b["slot"]: b["colour"] for b in scene["blocks"]})
    return {"png": png_bytes(img), "target": target,
            "slots": scene["blocks"], "hold_q": hold_q,
            "policy": policy, "rec": rec}
