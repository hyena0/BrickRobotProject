"""6.0/6.1 shared scene: pedestal plane, tip TCP, 5C shared range.

Priority: docs/stage/Korean/stage-6-addendum-sim-vla.ko.md over stage-6 B.

- Range/TCP source of truth: :mod:`src.console.control` (``stage_table``,
  ``tip_offset``, ``check_pose_against_ranges``) plus
  ``packages/robot_inferred`` and the measured-ranges file. No second
  range definition lives here.
- Coordinates: model frame == mat frame (x right, y forward, z up,
  z=0 at the pedestal top = table + 220.8). ``mat_points.json`` points
  are used directly as model coordinates.
- Rendering: dependency-free NumPy z-buffer rasterizer (256x256), the
  same approach as ``src/verify/sim_check`` (MuJoCo offscreen GL is
  unavailable in this sandbox). Only primitive shapes are rendered.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import struct
import zlib

import numpy as np

from src.console.control import stage_table, tip_offset
from src.kin.ik import _coupled_ok
from src.kin.model import joint_arrays, load_model

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6"]
COLORS = {
    "red": (0.85, 0.15, 0.12),
    "blue": (0.10, 0.30, 0.90),
    "green": (0.10, 0.65, 0.20),
    "yellow": (0.95, 0.80, 0.10),
}
BLOCK_SIZE = (32.0, 16.0, 11.0)  # stage-6 B.1, mm
MIN_SEP_MM = 80.0
VIA_H_MM = 120.0   # addendum 4/6.1: waypoint above the block
POINT_H_MM = 60.0  # addendum 3: above the *pedestal* top
HOLD_Z_LO, HOLD_Z_HI = 40.0, 90.0  # addendum 3, pedestal-relative
H_FLAT_MM = 30.0   # success: horizontal error within 30 mm
IMG = 256
MEASURED_RANGES = "runs/stage5c/2026-10-31/ranges.json"
MAT_POINTS = "assets/private/mat_points.json"
# Public slot table (derived): 6 slot coords + nearest measured IK
# seeds. Remote (Modal) code must use this, never MAT_POINTS, so no
# private file leaves the Mac. Provenance: assets/private/mat_points.json.
SLOT_SEEDS = "src/vla/slot_seeds.json"
LIGHT = (-0.4, -0.6, 0.75)


def load_package() -> dict:
    with open("packages/robot_inferred/robot_package.json",
              encoding="utf-8") as fh:
        return json.load(fh)


def adopted_ranges() -> dict:
    """Same rule as control.ranges_payload: narrower wins +5deg margin.

    Degenerate measured captures (lo == hi, e.g. single-point M5 probes)
    are ignored, exactly like the console does.
    """
    pkg = load_package()
    sim = {j["id"]: list(j["limits_deg"]) for j in pkg.get("joints", [])}
    try:
        measured = json.load(open(MEASURED_RANGES,
                                  encoding="utf-8")).get("measured", {})
    except (OSError, ValueError):
        measured = {}
    out = {}
    for jid, (lo_sim, hi_sim) in sim.items():
        m = measured.get(jid, {}) or {}
        lo_m, hi_m = m.get("lo"), m.get("hi")
        if lo_m is None or hi_m is None or not lo_m < hi_m:
            lo_m, hi_m = None, None
        if lo_m is None:
            out[jid] = [lo_sim, hi_sim]
        else:
            out[jid] = [max(lo_sim, lo_m) + 5.0,
                        min(hi_sim, hi_m) - 5.0]
    return out


def tip_doc(doc: dict) -> dict:
    """Doc copy whose TCP is the gripper tip (same as console tip_doc)."""
    out = dict(doc)
    out["end_effector"] = {"link": "D6", "point_mm": list(tip_offset())}
    return out


def limits_doc(pin_j6: float | None = None) -> dict:
    pkg = load_package()
    lims = {jid: list(v) for jid, v in adopted_ranges().items()}
    if pin_j6 is not None:  # addendum 3: J6 fixed for position IK
        lims["J6"] = [pin_j6, pin_j6]
    return {"limits_deg": lims,
            "coupled_limits": pkg.get("coupled_limits", [])}


def load_camera(path: str = "config/sim_camera.yaml") -> dict:
    import yaml
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def cam_frame(cam: dict):
    """Orthonormal pinhole basis: u ~ +x, image-up ~ +y (away)."""
    eye = np.array(cam["position_mm"], float)
    tgt = np.array(cam["look_at_mm"], float)
    fwd = tgt - eye
    fwd = fwd / np.linalg.norm(fwd)
    cz = -fwd
    cx = np.cross(np.array([0.0, 0.0, 1.0]), cz)
    n = np.linalg.norm(cx)
    if n < 1e-9:  # straight-down fallback (unused by sim_camera.yaml)
        cx = np.array([1.0, 0.0, 0.0])
    else:
        cx = cx / n
    cy = np.cross(cz, cx)
    f = (cam["image_px"][0] / 2.0) / math.tan(
        math.radians(cam["fov_deg"] / 2.0))
    return eye, cx, cy, fwd, f


def mat_bbox():
    pts = json.load(open(MAT_POINTS, encoding="utf-8"))["points"]
    xs = [p["x"] for p in pts]
    ys = [p["y"] for p in pts]
    return (min(xs), max(xs), min(ys), max(ys)), pts


def load_slot_seeds(path: str = SLOT_SEEDS) -> list:
    """Public slot table: [{slot, x, y, seed}]. No private content."""
    return json.load(open(path, encoding="utf-8"))


def sample_scene(rng: random.Random) -> dict:
    """3-4 blocks, >=80mm apart, random yaw; target color uniform.

    Pointing area = left part of the mat (lattice x -200..-120,
    y 150..310): with the 22-deg framing in config/sim_camera.yaml
    this keeps every block corner inside the frame while the robot
    base (x >= -13) stays out.
    """
    n = rng.choice([3, 4])
    colors = rng.sample(sorted(COLORS), n)
    # Lattice with 80 mm pitch inside the pointing area (centers
    # x -200..-120, y 150..310): any subset keeps the separation.
    slots = list(LATTICE_SLOTS)  # order frozen: rng stream stable
    placed = [(x, y, rng.uniform(-180.0, 180.0))
              for x, y in rng.sample(slots, n)]
    # NOTE: no rng use below this line (keeps the stream stable).
    blocks = [{"colour": c, "x": x, "y": y, "yaw_deg": w,
               "slot": SLOT_IDS[(x, y)]}
              for c, (x, y, w) in zip(colors, placed)]
    return {"blocks": blocks, "target": rng.choice(colors)}


def joint_world_points(q_deg: list[float], doc: dict):
    """World positions of J1..J6 plus (R, t) of the tip frame."""
    arr = joint_arrays(doc)
    R = np.eye(3)
    t = np.zeros(3)
    pts = []
    for qi, w, p in zip(q_deg, arr["axes"], arr["points"]):
        wn = np.array(w, float)
        wn = wn / np.linalg.norm(wn)
        pn = np.array(p, float)
        th = math.radians(qi)
        K = np.array([[0, -wn[2], wn[1]], [wn[2], 0, -wn[0]],
                      [-wn[1], wn[0], 0]])
        dR = (np.eye(3) + math.sin(th) * K
              + (1 - math.cos(th)) * (K @ K))
        pts.append(R @ pn + t)
        t = R @ ((np.eye(3) - dR) @ pn) + t
        R = R @ dR
    return pts, R, t


def tip_world(q_deg: list[float], tdoc: dict):
    from src.kin.model import ee_point
    pts, R, t = joint_world_points(q_deg, tdoc)
    return R @ np.array(ee_point(tdoc), float) + t, pts


def _box_faces(pts, rgb):
    # pts: [---,+--,++-, -+-, --+, +-+, +++, -++] (double-sided render)
    F = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
         (0, 1, 5), (0, 5, 4), (2, 3, 7), (2, 7, 6),
         (0, 4, 7), (0, 7, 3), (1, 2, 6), (1, 6, 5)]
    return [(np.array([pts[a_], pts[b_], pts[c_]], float), rgb)
            for a_, b_, c_ in F]


def box_tris(cx, cy, cz, sx, sy, sz, yaw_deg, rgb):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    pts = []
    for dx, dy, dz in [(-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
                       (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1)]:
        pts.append((cx + (dx * sx / 2) * c - (dy * sy / 2) * s,
                    cy + (dx * sx / 2) * s + (dy * sy / 2) * c,
                    cz + dz * sz / 2))
    return _box_faces(pts, rgb)


def seg_tris(a, b, r, rgb):
    """Link capsule approximated as a square-section box."""
    a, b = np.array(a, float), np.array(b, float)
    d = b - a
    n = np.linalg.norm(d)
    if n < 1e-9:
        return []
    u = d / n
    h = np.array([0.0, 0.0, 1.0]) if abs(u[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    w1 = np.cross(u, h)
    w1 = w1 / np.linalg.norm(w1)
    w2 = np.cross(u, w1)
    pts = []
    for end in (a, b):
        for s1 in (-1, 1):
            for s2 in (-1, 1):
                pts.append(end + s1 * r * w1 + s2 * r * w2)
    # order per end: --,-+,+-,++ ; ends: a(0-3), b(4-7). Remap to
    # [---,+--,++-, -+-, --+, +-+, +++, -++] with axis u = +z.
    order = [0, 2, 3, 1, 4, 6, 7, 5]
    return _box_faces([pts[i] for i in order], rgb)


def scene_static_tris():
    """Table + pedestal (background, not the arm)."""
    tris = []
    H = 800.0  # table quad half-size
    z = stage_table()["table_z_mm"]
    tris.append((np.array([[-H, -H, z], [H, -H, z], [H, H, z]],
                          float), (0.72, 0.72, 0.70)))
    tris.append((np.array([[-H, -H, z], [H, H, z], [-H, H, z]],
                          float), (0.72, 0.72, 0.70)))
    tris += box_tris(-110.0, 230.0, -110.4, 300.0, 260.0, 220.8, 0.0,
                     (0.88, 0.87, 0.84))  # pedestal block, top z=0
    return tris


def arm_tris(q_deg, doc, rgb=(0.25, 0.25, 0.28)):
    """Base column + link boxes for a pose (used for the out-of-view check)."""
    tdoc = tip_doc(doc)
    tip, jpts = tip_world(q_deg, tdoc)
    j1 = np.array(jpts[0])
    tris = []
    tris += box_tris(float(j1[0]), float(j1[1]), -180.0, 70.0, 70.0,
                     82.0, 0.0, rgb)  # base column table -> near J1
    chain = [j1] + [np.array(p) for p in jpts[1:]] + [np.array(tip)]
    for a, b in zip(chain[:-1], chain[1:]):
        tris += seg_tris(a, b, 8.0, rgb)
    return tris, tip, jpts


def block_tris(block):
    rgb = COLORS[block["colour"]]
    return box_tris(block["x"], block["y"], BLOCK_SIZE[2] / 2,
                    BLOCK_SIZE[0], BLOCK_SIZE[1], BLOCK_SIZE[2],
                    block["yaw_deg"], rgb)


def project_points(pts, eye, right, up, fwd, f, cx=128.0, cy=128.0):
    rel = np.asarray(pts, float) - eye
    xc = rel @ right
    yc = rel @ up
    zc = rel @ fwd
    with np.errstate(divide="ignore", invalid="ignore"):
        u = cx + f * xc / zc
        v = cy - f * yc / zc
    return u, v, zc


def rasterize(tris, cam, gain=1.0):
    """Z-buffer render of (tri, rgb) list. Returns float IMGxIMGx3."""
    W, H = cam["image_px"]
    eye, right, up, fwd, f = cam_frame(cam)
    img = np.zeros((H, W, 3))
    zbuf = np.full((H, W), np.inf)
    lx, ly, lz = np.array(LIGHT, float)
    ln = math.sqrt(lx * lx + ly * ly + lz * lz)
    lx, ly, lz = lx / ln, ly / ln, lz / ln
    for tri, rgb in tris:
        P = np.asarray(tri, float)
        rel = P - eye
        zc = rel @ fwd
        if (zc <= 1.0).any():
            continue
        u, v, _ = project_points(P, eye, right, up, fwd, f)
        n = np.cross(P[1] - P[0], P[2] - P[0])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        shade = 0.35 + 0.65 * abs(float(n @ np.array([lx, ly, lz])) / nn)
        x0, x1 = max(int(u.min()), 0), min(int(u.max()) + 1, W - 1)
        y0, y1 = max(int(v.min()), 0), min(int(v.max()) + 1, H - 1)
        if x1 <= x0 or y1 <= y0:
            continue
        ys, xs = np.mgrid[y0:y1 + 1, x0:x1 + 1]
        den = ((v[1] - v[2]) * (u[0] - u[2])
               + (u[2] - u[1]) * (v[0] - v[2]))
        if abs(den) < 1e-9:
            continue
        l0 = (((v[1] - v[2]) * (xs - u[2]) + (u[2] - u[1]) * (ys - v[2]))
              / den)
        l1 = (((v[2] - v[0]) * (xs - u[2]) + (u[0] - u[2]) * (ys - v[2]))
              / den)
        l2 = 1.0 - l0 - l1
        m = (l0 >= 0) & (l1 >= 0) & (l2 >= 0)
        if not m.any():
            continue
        zd = l0 * zc[0] + l1 * zc[1] + l2 * zc[2]
        upd = m & (zd < zbuf[y0:y1 + 1, x0:x1 + 1])
        if not upd.any():
            continue
        zbuf[y0:y1 + 1, x0:x1 + 1][upd] = zd[upd]
        shaded = np.clip(np.array(rgb) * shade * gain + 0.06, 0, 1)
        for k in range(3):
            img[y0:y1 + 1, x0:x1 + 1, k][upd] = shaded[k]
    drawn = np.isfinite(zbuf)
    img[~drawn] = (0.16, 0.17, 0.20)  # background (never hit: table covers)
    return img


def write_png(path, px):
    H, W, _ = px.shape
    raw = bytearray()
    for y in range(H):
        raw.append(0)
        raw.extend(px[y].tobytes())

    def chunk(t, d):
        return (struct.pack(">I", len(d)) + t + d
                + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff))
    ihdr = struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
           + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
           + chunk(b"IEND", b""))
    with open(path, "wb") as fh:
        fh.write(png)


def frame_checks(scene, q_deg, doc, cam):
    """6.0 gate: all blocks visible, arm fully out of frame."""
    eye, right, up, fwd, f = cam_frame(cam)
    W, H = cam["image_px"]
    blocks_ok = True
    for b in scene["blocks"]:
        corners = np.array([t.reshape(-1, 3)
                            for t, _ in block_tris(b)]).reshape(-1, 3)
        u, v, zc = project_points(corners, eye, right, up, fwd, f)
        if not bool(((zc > 1.0) & (u > 2) & (u < W - 2)
                     & (v > 2) & (v < H - 2)).all()):
            blocks_ok = False
    atris, _, _ = arm_tris(q_deg, doc)
    worst = 0.0
    for tri, _ in atris:
        P = np.asarray(tri, float).reshape(-1, 3)
        u, v, zc = project_points(P, eye, right, up, fwd, f)
        pxr = 8.0 * f / np.maximum(zc, 1.0) + 3.0
        inside = ((zc > 1.0) & (u > -pxr) & (u < W + pxr)
                  & (v > -pxr) & (v < H + pxr))
        if inside.any():
            worst = max(worst, float(np.sum(inside)) / len(inside))
    return {"blocks_visible": blocks_ok,
            "arm_out": worst == 0.0,
            "arm_overlap_frac": round(worst, 3)}


def _arm_overlap(q, doc, cam, extra=0.0):
    """Max over link tris of the in-frame vertex fraction (0 = out).

    ``extra`` expands the frame (px) for a safety margin during search;
    the 6.0 gate itself uses extra=0.
    """
    atris, _, _ = arm_tris(q, doc)
    eye, right, up, fwd, f = cam_frame(cam)
    W, H = cam["image_px"]
    worst = 0.0
    for tri, _ in atris:
        P = np.asarray(tri, float).reshape(-1, 3)
        u, v, zc = project_points(P, eye, right, up, fwd, f)
        pxr = 8.0 * f / np.maximum(zc, 1.0) + 3.0 + extra
        inside = ((zc > 1.0) & (u > -pxr) & (u < W + pxr)
                  & (v > -pxr) & (v < H + pxr))
        if inside.any():
            worst = max(worst, float(np.sum(inside)) / len(inside))
    return worst


LATTICE_SLOTS = [(-200.0, 150.0), (-200.0, 230.0), (-200.0, 310.0),
                 (-120.0, 150.0), (-120.0, 230.0), (-120.0, 310.0)]
# Physical slot numbers (front row near the robot first). Real blocks
# go on these slots only; sim scenes record the slot per block.
SLOT_IDS = {(-200.0, 150.0): "S1", (-120.0, 150.0): "S2",
            (-200.0, 230.0): "S3", (-120.0, 230.0): "S4",
            (-200.0, 310.0): "S5", (-120.0, 310.0): "S6"}
REST_J6 = 60.0  # pinned J6 shared by rest, via and point (addendum 3)


def _max_joint_dist(a, b):
    return max(abs(x - y) for x, y in zip(a, b))


def nearest_mat_seed(x, y, mat_seeds, pts):
    best, bd = None, None
    for s, p in zip(mat_seeds, pts):
        d = math.hypot(x - p["x"], y - p["y"])
        if bd is None or d < bd:
            best, bd = s, d
    return list(best)


def find_rest_pose(rng, doc, tdoc, lims, cam, tries=600):
    """Parked pose: out of frame AND near the pointing region.

    Expert demos must fit 50 steps at <=5 deg/step, so the rest pose
    has to be close (in joint space) to the pointing poses. Hideout
    tips south/east of the mat are solved with IK seeded from pointing
    solutions (continuity), keeping the coupled limits; the winner
    minimizes the joint distance to the nearest pointing reference.
    """
    from src.kin.ik import ik
    pkg = load_package()
    coupled = pkg.get("coupled_limits", [])
    slots = load_slot_seeds()
    mat_seeds = [s["seed"] for s in slots]
    mat = slots
    lim_pin = limits_doc(pin_j6=REST_J6)
    refs = []
    for si, (sx, sy) in enumerate(LATTICE_SLOTS):
        seed = nearest_mat_seed(sx, sy, mat_seeds, mat)
        seed[5] = REST_J6
        sol = ik([sx, sy, 60.0], None, tdoc, q_init_deg=seed,
                 position_only=True, limits_doc=lim_pin, seed=si,
                 max_restarts=32)
        if sol["q_deg"] is not None and sol["pos_err_mm"] <= 5.0:
            q = [float(v) for v in sol["q_deg"]]
            if _coupled_ok(q, JOINTS, coupled):
                refs.append(q)
    if not refs:
        raise RuntimeError("no pointing reference solved")
    # Margin: the hideout must stay out under the +-10 mm camera
    # jitter used for domain randomization.
    jrng = random.Random(7)
    cams = [cam] + [jitter_cam(cam, jrng) for _ in range(4)]

    def clear(q):
        return all(_arm_overlap(q, doc, c, extra=15.0) == 0.0
                   for c in cams)

    best, best_q, t = 1e9, None, 0
    for hx in (-180.0, -100.0, -20.0, 50.0, 90.0):
        for hy in (-60.0, -20.0, 40.0):
            for hz in (80.0, 140.0):
                for r in refs:
                    t += 1
                    try:
                        sol = ik([hx, hy, hz], None, tdoc,
                                 q_init_deg=r, position_only=True,
                                 limits_doc=lim_pin, seed=t,
                                 max_restarts=8)
                    except Exception:
                        continue
                    if sol["q_deg"] is None or sol["pos_err_mm"] > 5.0:
                        continue
                    q = [float(v) for v in sol["q_deg"]]
                    if not _coupled_ok(q, JOINTS, coupled):
                        continue
                    if not clear(q):
                        continue
                    d = min(_max_joint_dist(q, r) for r in refs)
                    if d < best:
                        best, best_q = d, list(q)
    if best_q is None:
        raise RuntimeError("no out-of-frame hideout near the work area")
    q = best_q
    step = 4.0
    for _ in range(300):
        cand = [min(max(v + rng.uniform(-step, step), lims[j][0]),
                    lims[j][1]) for v, j in zip(q, JOINTS)]
        cand[5] = REST_J6
        if not _coupled_ok(cand, JOINTS, coupled):
            continue
        if not clear(cand):
            step *= 0.997
            continue
        d = min(_max_joint_dist(cand, r) for r in refs)
        if d < best:
            best, q = d, cand
        else:
            step *= 0.997
    return q, t


def render_frame(scene, q_deg, doc, cam, rng=None):
    """Full frame: static + blocks + arm. Light jitter keeps B.1 range."""
    gain = rng.uniform(0.9, 1.1) if rng else 1.0
    tris = scene_static_tris()
    for b in scene["blocks"]:
        tris += block_tris(b)
    atris, _, _ = arm_tris(q_deg, doc)
    tris += atris
    img = rasterize(tris, cam, gain=gain)
    return (np.clip(img, 0, 1) * 255 + 0.5).astype(np.uint8)


def jitter_cam(cam, rng):
    """Camera +-1cm per addendum 3 (light domain randomization)."""
    out = dict(cam)
    out["position_mm"] = [cam["position_mm"][0] + rng.uniform(-10, 10),
                          cam["position_mm"][1] + rng.uniform(-10, 10),
                          cam["position_mm"][2]]
    return out


def run_60(out="runs/vla/6.0", seed=60):
    """6.0: 9 example first-frames (3x3), blocks in view, arm out."""
    os.makedirs(out, exist_ok=True)
    rng = random.Random(seed)
    cam = load_camera()
    doc = load_model("models/robot_inferred.json")
    tdoc = tip_doc(doc)
    lims = adopted_ranges()
    rest_q, rest_tries = find_rest_pose(rng, doc, tdoc, lims, cam)
    rows = []
    for i in range(9):
        scene = sample_scene(rng)
        cami = jitter_cam(cam, rng)
        checks = frame_checks(scene, rest_q, doc, cami)
        img = render_frame(scene, rest_q, doc, cami, rng)
        write_png(os.path.join(out, f"frame_{i:02d}.png"), img)
        rows.append({"id": f"frame_{i:02d}", "blocks": scene["blocks"],
                     "target": scene["target"], **checks})
    grid = np.zeros((3 * IMG, 3 * IMG, 3), np.uint8)
    for i in range(9):
        tile = _read_png_back(os.path.join(out, f"frame_{i:02d}.png"))
        grid[(i // 3) * IMG:(i // 3 + 1) * IMG,
             (i % 3) * IMG:(i % 3 + 1) * IMG] = tile
    write_png(os.path.join(out, "grid_3x3.png"), grid)
    ok_blocks = sum(1 for r in rows if r["blocks_visible"])
    ok_arm = sum(1 for r in rows if r["arm_out"])
    report = {"seed": seed, "camera": "config/sim_camera.yaml",
              "rest_q_deg": [round(v, 2) for v in rest_q],
              "rest_search_tries": rest_tries,
              "frames": rows,
              "gate": {"blocks_visible": f"{ok_blocks}/9",
                       "arm_out": f"{ok_arm}/9",
                       "pass": ok_blocks == 9 and ok_arm == 9}}
    json.dump(report, open(os.path.join(out, "report.json"), "w"),
              indent=1)
    print(f"6.0: blocks {ok_blocks}/9 visible, arm out {ok_arm}/9")
    print("PASS" if report["gate"]["pass"] else "FAIL", "->", out)
    return report


def _read_png_back(path):
    """Minimal PNG reader for the grid (undoes our own write_png)."""
    raw = open(path, "rb").read()
    pos = 8
    W = H = 0
    idat = bytearray()
    while pos < len(raw):
        ln = struct.unpack(">I", raw[pos:pos + 4])[0]
        typ = raw[pos + 4:pos + 8]
        data = raw[pos + 8:pos + 8 + ln]
        if typ == b"IHDR":
            W, H = struct.unpack(">II", data[:8])
        elif typ == b"IDAT":
            idat += data
        pos += 12 + ln
    px = zlib.decompress(bytes(idat))
    img = np.zeros((H, W, 3), np.uint8)
    p = 0
    for y in range(H):
        p += 1  # filter byte 0
        img[y] = np.frombuffer(px[p:p + W * 3],
                               dtype=np.uint8).reshape(W, 3)
        p += W * 3
    return img


def main(argv=None):
    ap = argparse.ArgumentParser(description="6.0 scene renders")
    ap.add_argument("--out", default="runs/vla/6.0")
    ap.add_argument("--seed", type=int, default=60)
    args = ap.parse_args(argv)
    run_60(args.out, args.seed)


if __name__ == "__main__":
    main()
