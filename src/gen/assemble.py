"""Stage 5D.4: deterministic module-chain assembler.

Places library modules and straight Technic beams along a serial chain,
snaps to the stud grid, and writes ``robot.ldr`` plus the known-answer
``expected_kinematics.json`` (``docs/schema.md`` frame: base link, Z up,
mm; joint axes/points are base-frame-at-zero space screws, matching
``src/kin/fk.py``).

Chain rule: module 1 sits at the world origin. Each beam runs from a
module output mount along its +Z by the beam hole span; the next module
input mount is that output frame translated by the span. Mount +Z points
base-to-tip (see ``assets/modules/README.md``).
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np
import yaml

from src.gen import ldraw as ldraw_io
from src.gen.inventory import load_catalog_merged
from src.gen.modules import load_module, validate_module
from src.gen.spec import load_spec, validate_spec

LINK_TOL_MM = 8.0  # one hole pitch
GRID_WARN_MM = 0.1  # warn if assembly lands off the stud grid
AABB_MARGIN_MM = 8.0  # hole-bbox margin (heuristic pre-check)
TOUCH_MM = 2.0  # AABB penetration below this counts as touching
FALLBACK_HALF_MM = {"motor": 32.0, "sensor": 16.0}
DEFAULT_HALF_MM = 10.0
MOTOR_PORTS = ["A", "B", "C", "D"]
SEGMENT_COLOURS = ["4", "1", "14", "2", "5", "6", "7"]


# ---------------------------------------------------------------- beams


def _hole_points_mm(entry: dict) -> list[list[float]]:
    pts = []
    for hole in entry.get("holes") or []:
        x, y, z = (float(v) for v in hole["pos"])
        pts.append([x * 0.4, z * 0.4, -y * 0.4])
    return pts


PITCH_LDU = 20.0  # one hole pitch
STRAIGHT_RATIO = 4.0  # straight beams are >= this long-vs-thick


def _straight_beam(entry: dict) -> tuple[int, float] | None:
    """(hole count, span_mm) for straight beams, else None.

    Catalog holes are mesh-derived connection points, not exact hole
    centres, so straightness is judged by proportions: the long spread
    must dominate the perpendicular spread (validated on the catalog:
    straight beams are >= 4x, bent ones <= 3x). Hole count comes from
    the span over the 20 LDU hole pitch.
    """
    pts = [[float(v) for v in hole["pos"]]
           for hole in (entry.get("holes") or [])]
    if len(pts) < 2:
        return None
    spread = [max(p[ax] for p in pts) - min(p[ax] for p in pts)
              for ax in range(3)]
    order = sorted(range(3), key=lambda ax: spread[ax])
    long_ax = order[2]
    perp_max = max(spread[order[0]], spread[order[1]])
    if spread[long_ax] < STRAIGHT_RATIO * perp_max:
        return None
    pitches = spread[long_ax] / PITCH_LDU
    holes = int(round(pitches)) + 1
    if abs(pitches - (holes - 1)) > 0.25:
        return None
    if not 3 <= holes <= 15:
        return None
    if len({p[long_ax] for p in pts}) < 3:
        return None
    return holes, spread[long_ax] * 0.4


def list_beams(catalog: dict, inventory: dict | None = None) -> list[dict]:
    """Straight 3-15 hole beams in stock, sorted by span."""
    stocked = None
    if inventory is not None:
        stocked = {pid: row.get("total", 0)
                   for pid, row in (inventory.get("parts") or {}).items()}
    beams = []
    for part_id, entry in catalog.items():
        if (entry or {}).get("category") != "beam":
            continue
        straight = _straight_beam(entry)
        if straight is None:
            continue
        holes, span_mm = straight
        if stocked is not None and stocked.get(part_id, 0) <= 0:
            continue
        beams.append({"part_id": part_id, "holes": holes,
                      "span_mm": span_mm})
    beams.sort(key=lambda b: (b["span_mm"], b["part_id"]))
    return beams


def _beam_mount(entry: dict) -> tuple[np.ndarray, np.ndarray, float]:
    """(axis permutation P, min-hole mm point, span_mm) for a beam.

    The min-hole point is centred: perpendicular mesh noise is removed
    by snapping the centreline to the 10 LDU grid, so the chain does
    not drift sideways beam after beam.
    """
    ldu = np.array([[float(v) for v in hole["pos"]]
                    for hole in (entry.get("holes") or [])])
    mm = np.array(_hole_points_mm(entry))
    spread = mm.max(axis=0) - mm.min(axis=0)
    order = list(np.argsort(spread, kind="stable"))  # thin, mid, long
    perm = np.zeros((3, 3))
    for mount_ax, local_ax in enumerate(order):
        perm[local_ax, mount_ax] = 1.0
    if np.linalg.det(perm) < 0:
        perm[:, 0] *= -1.0
    ldu_spread = ldu.max(axis=0) - ldu.min(axis=0)
    long_ldu = int(np.argmax(ldu_spread))
    centre = [0.0, 0.0, 0.0]
    for ax in range(3):
        if ax == long_ldu:
            centre[ax] = float(ldu[:, ax].min())
        else:
            centre[ax] = round(float(ldu[:, ax].mean()) / 10.0) * 10.0
    cx, cy, cz = centre
    hmin = np.array([cx * 0.4, cz * 0.4, -cy * 0.4])
    return perm, hmin, float(ldu_spread[long_ldu]) * 0.4


# --------------------------------------------------- mount interfaces


def _transform_points(pose: tuple[np.ndarray, np.ndarray],
                      points: np.ndarray) -> np.ndarray:
    rot, pos = pose
    return points @ rot.T + pos


ELEMENT_PIN = "2780.dat"  # most stocked friction pin (fits any bore)
ELEMENT_AXLE = "32062.dat"  # shortest stocked 16 mm axle (mesh-verified)
MIN_GAP_MM = 24.0  # shortest mount-to-mount span exact-fit beams cover


def mount_points_world(module: dict, rot_w: np.ndarray,
                       pos_w: np.ndarray) -> dict:
    """Mount holes in world mm (converted frame)."""
    pts = {}
    for key in ("mount_in", "mount_out"):
        rows = []
        for hole in module[key]["holes"]:
            world = rot_w @ (np.array(hole["pos_ldu"], dtype=float)
                             * 0.4) + pos_w
            rows.append({"point_mm": [float(v) for v in world],
                         "type": hole["type"]})
        pts[key] = rows
    return pts


# Beam-local axes in the mount frame: row along mount +Z (the chain),
# hole tunnels along mount +X (the pin line).
P_BEAM = np.array([[0.0, 0.0, 1.0],
                   [1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0]])


def _check_elements(catalog: dict) -> None:
    from src.gen.meshbox import local_box_mm

    for pid, need in ((ELEMENT_PIN, 12.0), (ELEMENT_AXLE, 12.0)):
        entry = catalog.get(pid)
        if entry is None:
            raise ValueError(f"mount element {pid} missing from catalog")
        box = local_box_mm(pid)
        if box is not None:
            long = max(box[1][i] - box[0][i] for i in range(3))
            if long < need:
                raise ValueError(
                    f"mount element {pid} spans {long:.1f} mm "
                    f"(need >= {need:.0f})")


def solve_gap(out_module: dict, out_frame: dict, in_module: dict | None,
              in_frame: dict | None, beam_id: str,
              catalog: dict) -> dict:
    """Place a connector beam over gap mount holes (loud on no fit).

    Returns the beam world transform, deduped hole points, per-point
    kinds and the mount element per point (``pin`` unless every bore
    at the point is a cross bore, which takes an ``axle``).
    """
    from src.gen import mounts as _mounts

    _check_elements(catalog)
    r_mount = out_frame["rot"] @ np.array(
        out_module["mount_out"]["frame"], dtype=float)[:, :3]
    r_beam = r_mount @ P_BEAM
    out_pts = mount_points_world(
        out_module, out_frame["rot"], out_frame["pos"])["mount_out"]
    in_pts = []
    if in_module is not None:
        assert in_frame is not None
        in_pts = mount_points_world(
            in_module, in_frame["rot"], in_frame["pos"])["mount_in"]
    # Dedup shared points (short spans reuse one hole pair).
    points: list[dict] = []
    for row in out_pts + in_pts:
        key = tuple(round(v, 3) for v in row["point_mm"])
        found = next((p for p in points if p["key"] == key), None)
        if found is None:
            points.append({"key": key, "point_mm": row["point_mm"],
                           "types": [row["type"]]})
        else:
            found["types"].append(row["type"])
    entry = catalog.get(beam_id)
    if entry is None:
        raise ValueError(f"beam {beam_id} missing from catalog")
    solved = _mounts.solve_beam_origin(
        entry, r_beam.tolist(), [p["point_mm"] for p in points])
    origin_mm = [v * 0.4 for v in solved["origin_ldu"]]
    elements = []
    for point, kind in zip(points, solved["kinds"]):
        bore_axle = all(t == "axle" for t in point["types"])
        elements.append(ELEMENT_AXLE if bore_axle and kind == "axle"
                        else ELEMENT_PIN)
    return {"part_id": beam_id, "origin_mm": origin_mm,
            "rotation": r_beam.tolist(),
            "points_mm": [p["point_mm"] for p in points],
            "beam_kinds": solved["kinds"], "elements": elements}


def compose_frames(modules: list[dict],
                   spans_mm: list[float]) -> dict:
    """Module input frames + joint frames along the chain (no parts).

    Module 1 sits at the world origin; each span runs from a module
    output mount along its +Z, and the next module input frame lands
    on the span end. Returns frames (rot/pos ndarrays), joints
    (axis/point lists), ee and link_lengths.
    """
    n = len(modules)
    frames: list[dict] = []
    joints: list[dict] = []
    rot_w, pos_w = np.eye(3), np.zeros(3)
    for i, (module, span) in enumerate(zip(modules, spans_mm)):
        joint = module["joint"]
        axis = rot_w @ np.array(joint["axis"], dtype=float)
        axis = axis / np.linalg.norm(axis)
        point = rot_w @ np.array(joint["point_mm"], dtype=float) + pos_w
        frames.append({"rot": np.array(rot_w), "pos": np.array(pos_w)})
        joints.append({"axis": [float(v) for v in axis],
                       "point": [float(v) for v in point]})
        out = module["output_mount"]
        r_out = rot_w @ np.array(out["rotation"], dtype=float)
        p_out = rot_w @ np.array(out["position_mm"], dtype=float) + pos_w
        end = p_out + span * r_out[:, 2]
        if i < n - 1:
            nxt = modules[i + 1]["input_mount"]
            r_in = np.array(nxt["rotation"], dtype=float)
            p_in = np.array(nxt["position_mm"], dtype=float)
            rot_w = r_out @ r_in.T
            pos_w = end - rot_w @ p_in
        else:
            ee = end
    link_lengths = []
    prev = np.zeros(3)
    for joint in joints:
        cur = np.array(joint["point"])
        link_lengths.append(float(np.linalg.norm(cur - prev)))
        prev = cur
    return {"frames": frames, "joints": joints,
            "ee": [float(v) for v in ee], "link_lengths": link_lengths}


def place_chain(modules: list[dict], geometries: list[list[dict]],
                beams: list[dict], catalog: dict) -> dict:
    """Compose module frames along the chain. Returns assembly data."""
    n = len(modules)
    beam_mounts = [_beam_mount(catalog[beam["part_id"]])
                   for beam in beams]
    spans = [mount[2] for mount in beam_mounts]
    comp = compose_frames(modules, spans)
    seg_parts: list[list[dict]] = []
    beam_rows: list[dict] = []
    for i, (module, geometry, beam, frame, mount) in enumerate(
            zip(modules, geometries, beams, comp["frames"],
                beam_mounts)):
        rot_w, pos_w = frame["rot"], frame["pos"]
        perm, hmin, span = mount
        seg: list[dict] = []
        for part in geometry:
            moved = _transform_points(
                (rot_w, pos_w), np.array([part["position_mm"]]))[0]
            seg.append({**part,
                        "position_mm": [float(v) for v in moved],
                        "rotation": (rot_w @ np.array(
                            part["rotation"])).tolist(),
                        "_segment": i})
        out = module["output_mount"]
        r_out = rot_w @ np.array(out["rotation"], dtype=float)
        p_out = rot_w @ np.array(out["position_mm"], dtype=float) + pos_w
        r_beam = r_out @ perm
        t_beam = p_out - r_beam @ hmin
        seg.append({"uid": "", "part_id": beam["part_id"],
                    "colour": SEGMENT_COLOURS[i % len(SEGMENT_COLOURS)],
                    "position_mm": [float(v) for v in t_beam],
                    "rotation": r_beam.tolist(), "_segment": i,
                    "_beam": True})
        seg_parts.append(seg)
        beam_rows.append({"part_id": beam["part_id"], "span_mm": span})
    parts = [p for seg in seg_parts for p in seg]
    for k, part in enumerate(parts, 1):
        part["uid"] = f"p{k:06d}"
    return {"parts": parts, "joints": comp["joints"], "ee": comp["ee"],
            "link_lengths": comp["link_lengths"], "beams": beam_rows}


# ------------------------------------------------- precheck: poses


def _part_aabb(part: dict, catalog: dict) -> tuple[np.ndarray, np.ndarray]:
    entry = catalog.get(part["part_id"]) or {}
    rot = np.array(part["rotation"], dtype=float)
    pos = np.array(part["position_mm"], dtype=float)
    pts = _hole_points_mm(entry)
    if pts:
        corners = _transform_points((rot, pos), np.array(pts))
        lo = corners.min(axis=0) - AABB_MARGIN_MM
        hi = corners.max(axis=0) + AABB_MARGIN_MM
    else:
        half = FALLBACK_HALF_MM.get(entry.get("category"),
                                    DEFAULT_HALF_MM)
        lo = pos - half
        hi = pos + half
    return lo, hi


def _local_box(part: dict, catalog: dict, mesh_root: str | None
               ) -> tuple[np.ndarray, np.ndarray, bool]:
    """Converted-local box + mesh-exact flag (loud fallback)."""
    if mesh_root is not None:
        from src.gen.meshbox import local_box_mm

        box = local_box_mm(part["part_id"], mesh_root)
        if box is not None:
            return (np.array(box[0]), np.array(box[1]), True)
    entry = catalog.get(part["part_id"]) or {}
    pts = _hole_points_mm(entry)
    if pts:
        arr = np.array(pts, dtype=float)
        return (arr.min(axis=0) - AABB_MARGIN_MM,
                arr.max(axis=0) + AABB_MARGIN_MM, False)
    half = FALLBACK_HALF_MM.get(entry.get("category"), DEFAULT_HALF_MM)
    return -np.full(3, half), np.full(3, half), False


def _obb_overlap(box_a: tuple, box_b: tuple) -> float:
    """Separating-axis overlap of two OBBs (<= 0: clear).

    Boxes are ``(center, axes(3x3 columns), half(3))``; returns the
    minimum axial overlap (a penetration estimate when positive).
    """
    (c0, a0, h0), (c1, a1, h1) = box_a, box_b
    delta = np.asarray(c1, dtype=float) - np.asarray(c0, dtype=float)
    a0 = np.asarray(a0, dtype=float)
    a1 = np.asarray(a1, dtype=float)
    axes = [a0[:, k] for k in range(3)] + [a1[:, k] for k in range(3)]
    for i in range(3):
        for j in range(3):
            axes.append(np.cross(a0[:, i], a1[:, j]))
    worst = float("inf")
    for axis in axes:
        norm = float(np.linalg.norm(axis))
        if norm < 1e-9:
            continue
        axis = axis / norm
        r0 = sum(h0[k] * abs(float(axis @ a0[:, k])) for k in range(3))
        r1 = sum(h1[k] * abs(float(axis @ a1[:, k])) for k in range(3))
        worst = min(worst, r0 + r1 - abs(float(axis @ delta)))
    return worst


class _BVH:
    """Median-split AABB tree over part boxes."""

    def __init__(self, boxes: list, leaf: int = 4):
        self.boxes = boxes
        self.root = self._build(list(range(len(boxes))), leaf)

    def _build(self, ids: list[int], leaf: int):
        lo = np.min([self.boxes[i][0] for i in ids], axis=0)
        hi = np.max([self.boxes[i][1] for i in ids], axis=0)
        if len(ids) <= leaf:
            return {"lo": lo, "hi": hi, "ids": ids}
        span = hi - lo
        ax = int(np.argmax(span))
        ids.sort(key=lambda i: (self.boxes[i][0][ax]
                                + self.boxes[i][1][ax]) / 2.0)
        mid = len(ids) // 2
        return {"lo": lo, "hi": hi,
                "left": self._build(ids[:mid], leaf),
                "right": self._build(ids[mid:], leaf)}

    @staticmethod
    def _hit(a, b) -> bool:
        return bool(np.all(a["hi"] > b["lo"]) and np.all(b["hi"] > a["lo"]))

    def overlaps(self, segments: list,
                   mated: set[tuple[int, int]] | None = None,
                   obbs: list | None = None
                   ) -> list[tuple[int, int, float]]:
        mated = mated or set()
        found: list[tuple[int, int, float]] = []

        def visit(a, b) -> None:
            if not self._hit(a, b):
                return
            if "ids" in a and "ids" in b:
                for i in a["ids"]:
                    for j in b["ids"]:
                        if j <= i or (i, j) in mated:
                            continue
                        if segments[i] == segments[j]:
                            continue  # same rigid module: trusted
                        lo = np.maximum(self.boxes[i][0],
                                        self.boxes[j][0])
                        hi = np.minimum(self.boxes[i][1],
                                        self.boxes[j][1])
                        if np.all(hi > lo):
                            pen = float(np.min(hi - lo))
                            if pen <= TOUCH_MM:
                                continue
                            if obbs is not None:
                                pen = _obb_overlap(obbs[i], obbs[j])
                                if pen <= TOUCH_MM:
                                    continue
                            found.append((i, j, pen))
                return
            if "ids" in a:
                visit(a, b["left"])
                visit(a, b["right"])
            elif "ids" in b:
                visit(a["left"], b)
                visit(a["right"], b)
            else:
                visit(a["left"], b["left"])
                visit(a["left"], b["right"])
                visit(a["right"], b["left"])
                visit(a["right"], b["right"])

        visit(self.root, self.root)
        return found


def _rot_about_axis(axis, angle_deg: float) -> np.ndarray:
    import math

    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    theta = math.radians(angle_deg)
    kx, ky, kz = axis
    c, s = math.cos(theta), math.sin(theta)
    return np.array([
        [c + kx * kx * (1 - c), kx * ky * (1 - c) - kz * s,
         kx * kz * (1 - c) + ky * s],
        [ky * kx * (1 - c) + kz * s, c + ky * ky * (1 - c),
         ky * kz * (1 - c) - kx * s],
        [kz * kx * (1 - c) - ky * s, kz * ky * (1 - c) + kx * s,
         c + kz * kz * (1 - c)]])


def check_pre2_interference(parts: list[dict], joints: list[dict],
                            design_ranges: list[list[float]],
                            catalog: dict,
                            mated: set[tuple[str, str]],
                            mesh_root: str | None = None,
                            bolted: set[tuple[str, str]] | None = None
                            ) -> dict:
    """BVH clearance at zero + every range-end combination.

    Downstream subtrees rotate rigidly about the (nested) joint
    frames; module subfiles stay rigid (mates excluded, same-module
    pairs trusted). Bolted pairs are checked at zero but skipped
    once articulated (remote-pivot sweep artifact). Boxes are
    mesh-exact; parts without mesh fall back loudly (listed, never
    silent).
    """
    import itertools

    by_uid = {p["uid"]: i for i, p in enumerate(parts)}
    mate_idx = {(by_uid[a], by_uid[b]) for a, b in mated
                if a in by_uid and b in by_uid}
    mate_idx |= {(b, a) for a, b in mate_idx}
    bolt_idx = {(by_uid[a], by_uid[b]) for a, b in (bolted or set())
                if a in by_uid and b in by_uid}
    bolt_idx |= {(b, a) for a, b in bolt_idx}
    segs = [p.get("_segment", 0) for p in parts]
    poses = list(itertools.product(
        *[[0.0, r[0], r[1]] for r in design_ranges]))
    worst: list[dict] = []
    fallback: set[str] = set()
    for pose in poses:
        articulated = any(v != 0.0 for v in pose)
        skip = mate_idx | (bolt_idx if articulated else set())
        moved = [dict(p) for p in parts]
        axes = [np.array(j["axis"], dtype=float) for j in joints]
        points = [np.array(j["point"], dtype=float) for j in joints]
        for i, angle in enumerate(pose):
            if angle == 0.0:
                continue
            rot = _rot_about_axis(axes[i], angle)
            pt = points[i]
            for k, part in enumerate(moved):
                if segs[k] > i:
                    pos = np.array(part["position_mm"], dtype=float)
                    part["position_mm"] = (rot @ (pos - pt) + pt).tolist()
                    part["rotation"] = (
                        rot @ np.array(part["rotation"])).tolist()
            for j in range(i + 1, len(joints)):
                axes[j] = rot @ axes[j]
                points[j] = rot @ (points[j] - pt) + pt
        boxes = []
        obbs = []
        for part in moved:
            lo, hi, exact = _local_box(part, catalog, mesh_root)
            rot = np.array(part["rotation"], dtype=float)
            pos = np.array(part["position_mm"], dtype=float)
            mid, half = (lo + hi) / 2.0, (hi - lo) / 2.0
            obbs.append((rot @ mid + pos, rot, half))
            corners = np.stack(np.meshgrid(*zip(lo, hi)),
                               axis=-1).reshape(-1, 3)
            world = pos + corners @ rot.T
            boxes.append((world.min(axis=0), world.max(axis=0)))
            if not exact:
                fallback.add(part["part_id"])
        for a, b, pen in _BVH(boxes).overlaps(segs, skip, obbs):
            worst.append({"pose_deg": list(pose),
                          "uid_a": moved[a]["uid"],
                          "uid_b": moved[b]["uid"],
                          "part_a": moved[a]["part_id"],
                          "part_b": moved[b]["part_id"],
                          "penetration_mm": round(pen, 2)})
            if len(worst) >= 12:
                break
        if len(worst) >= 12:
            break
    return {"ok": not worst, "poses": len(poses), "pairs": worst,
            "fallback_parts": sorted(fallback),
            "mesh": mesh_root is not None}


# ------------------------------------------- precheck: connectivity


def check_pre1_connectivity(gaps: list[dict], modules: list[dict],
                            catalog: dict) -> dict:
    """Hole coincidence + pin counts + single part-graph component.

    Recomputes beam hole worlds from the solved transforms (never
    trusting the solver's own arithmetic) and requires mount-hole
    coincidence within 0.1 LDU, >= 2 elements per beamed interface,
    and one connected component over the declared matings.
    """
    from src.gen import mounts as _mounts

    worst_ldu = 0.0
    details = []
    for gi, gap in enumerate(gaps):
        entry = catalog[gap["part_id"]]
        rot = np.array(gap["rotation"], dtype=float)
        org = np.array(gap["origin_mm"], dtype=float)
        beam_holes = []
        for centre in _mounts.hole_centres(entry):
            local = np.array(_mounts.conv_ldu_to_mm([0, 0, centre["z"]]))
            beam_holes.append(org + rot @ local)
        for pi, point in enumerate(gap["points_mm"]):
            dists = [float(np.linalg.norm(h - np.array(point)))
                     for h in beam_holes]
            best = min(dists) / 0.4 if dists else float("inf")
            worst_ldu = max(worst_ldu, best)
            details.append({"gap": gi, "point_mm": point,
                            "gap_ldu": round(best, 4),
                            "pass": best <= _mounts.MATCH_LDU})
    per_interface: dict[tuple[int, str], int] = {}
    for gi, gap in enumerate(gaps):
        per_interface[(gi, "out")] = gap["n_out"]
        if gap["has_next"]:
            per_interface[(gi, "in")] = gap["n_in"]
    counts_ok = all(n >= 2 for n in per_interface.values())
    # Declared mating graph: element--module and element--beam edges.
    parent: dict[str, str] = {}

    def find(x: str) -> str:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    nodes = set()
    for i in range(len(modules)):
        nodes.add(f"m{i}")
    for gi, gap in enumerate(gaps):
        nodes.add(f"b{gi}")
        for ei in range(len(gap["elements"])):
            nodes.add(f"e{gi}.{ei}")
    for node in nodes:
        parent[node] = node
    for gi, gap in enumerate(gaps):
        for ei in range(len(gap["elements"])):
            for other in gap["mates"][ei]:
                a, b = f"e{gi}.{ei}", other
                parent[find(a)] = find(b)
    components = len({find(n) for n in nodes})
    ok = (all(d["pass"] for d in details) and counts_ok
          and components == 1)
    return {"ok": ok, "worst_gap_ldu": round(worst_ldu, 4),
            "points": details,
            "pins_per_interface": {f"{g}{s}": n for (g, s), n in
                                   sorted(per_interface.items())},
            "counts_ok": counts_ok, "components": components}


def audit_element_matings(parts: list[dict], catalog: dict,
                          element_uids: set[str],
                          beam_of: dict[str, str] | None = None,
                          beam_uids: set[str] | None = None) -> dict:
    """Same-tool audit: every element must join module + gap-beam parts.

    Uses ``src.tools.geometry.find_connections`` on the parsed flat
    robot: each element needs line-tier neighbours including its own
    gap beam (``beam_of``) plus at least one module part (a neighbour
    that is neither a gap beam nor an element). Cross-articulation
    segments are NOT required: an in-side pin fastens its gap beam
    to the downstream module inside one rigid group by design.
    """
    from src.tools.geometry import find_connections

    beam_of = beam_of or {}
    beam_uids = beam_uids or set(beam_of.values())
    rows = []
    for part in parts:
        if part["uid"] not in element_uids:
            continue
        conns = [c for c in find_connections(part["uid"], parts, catalog)
                 if c["connection"] in ("pin_friction",
                                        "pin_frictionless",
                                        "axle_in_axle_hole",
                                        "axle_in_round_hole")]
        by_uid = {p["uid"]: p for p in parts}
        segs = {by_uid[c["uid"]].get("_segment", 0) for c in conns}
        want = beam_of.get(part["uid"])
        has_beam = want is not None and any(
            c["uid"] == want for c in conns)
        has_module = any(c["uid"] not in beam_uids
                         and c["uid"] not in element_uids
                         for c in conns)
        ok = has_beam and has_module
        rows.append({"uid": part["uid"], "part_id": part["part_id"],
                     "neighbours": len(conns),
                     "segments": sorted(segs), "gap_beam": has_beam,
                     "module_part": has_module, "pass": ok})
    return {"ok": all(r["pass"] for r in rows), "elements": rows}


# ----------------------------------------------- precheck: frame fidelity


def check_pre3_frames(robot_text: str, modules: list[dict],
                      names: list[str], spec: dict,
                      joints: list[dict]) -> dict:
    """Re-extracted joint frames vs skeleton and composer (exact).

    Parses module reference transforms back out of the emitted MPD
    main block (never reusing composer memory) and requires the
    re-extracted joint axes/points to match the spec-pure skeleton
    and the composer within 0.01 deg / 0.01 mm.
    """
    import math

    from src.gen import skeleton as _skeleton
    from src.parse.ldr import parse_type1

    reflist: list[dict] = []
    for line in robot_text.splitlines():
        s = line.strip()
        if not s.startswith("1 ") or not s.lower().endswith(".ldr"):
            continue
        part = parse_type1(s, "ref")
        if part is not None:
            reflist.append(part)
    want_frames = _skeleton.joint_frames(spec)
    rows = []
    seen: dict[str, int] = {}
    for i, (module, name) in enumerate(zip(modules, names)):
        key = f"{name}.ldr".lower()
        # Same module may drive several joints: take the k-th
        # same-named ref in emission order (dicts would collide).
        k = seen.get(key, 0)
        seen[key] = k + 1
        same = [r for r in reflist
                if r["part_id"].lower() == key]
        if len(same) <= k:
            rows.append({"joint": f"J{i + 1}", "pass": False,
                         "note": f"missing subfile ref {key} #{k}"})
            continue
        ref = same[k]
        rot = np.array(ref["rotation"], dtype=float)
        pos = np.array(ref["position_mm"], dtype=float)
        axis = rot @ np.array(module["joint"]["axis"], dtype=float)
        axis = axis / np.linalg.norm(axis)
        point = rot @ np.array(module["joint"]["point_mm"],
                               dtype=float) + pos
        exp = want_frames[i]
        dot = abs(float(np.dot(axis, np.array(exp["axis"]))))
        angle = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
        dist = float(np.linalg.norm(point - np.array(exp["point_mm"])))
        cdot = abs(float(np.dot(
            axis, np.array(joints[i]["axis"]) /
            np.linalg.norm(joints[i]["axis"]))))
        cangle = math.degrees(math.acos(max(-1.0, min(1.0, cdot))))
        cdist = float(np.linalg.norm(
            point - np.array(joints[i]["point"])))
        ok = angle <= 0.01 and dist <= 0.01 and cangle <= 0.01 \
            and cdist <= 0.01
        rows.append({"joint": f"J{i + 1}", "pass": ok,
                       "skel_angle_deg": round(angle, 6),
                       "skel_dist_mm": round(dist, 6),
                       "composer_angle_deg": round(cangle, 6),
                       "composer_dist_mm": round(cdist, 6)})
    return {"ok": all(r["pass"] for r in rows), "joints": rows}


# -------------------------------------------------------------- evaluate


def _design_ranges(spec: dict, modules: list[dict]
                   ) -> tuple[list[list[float]], list[str]]:
    ranges, errors = [], []
    for i, (module, span) in enumerate(
            zip(modules, spec["ranges_deg"]), 1):
        mspan = module["joint"]["range_deg"]
        lo, hi = max(span[0], mspan[0]), min(span[1], mspan[1])
        if lo > hi:
            errors.append(f"J{i}: spec {span} misses module {mspan}")
        ranges.append([float(lo), float(hi)])
    return ranges, errors


def build_expected_kinematics(modules: list[dict], assembly: dict,
                              design_ranges: list[list[float]]) -> dict:
    n = len(modules)
    if n > len(MOTOR_PORTS):
        raise ValueError("single EV3 brick supports at most "
                         f"{len(MOTOR_PORTS)} motors, got {n}")
    joints = []
    for i, (module, joint) in enumerate(zip(modules, assembly["joints"]), 1):
        joints.append({
            "id": f"J{i}", "type": "revolute",
            "parent": f"L{i - 1}", "child": f"L{i}",
            "axis": joint["axis"], "point_mm": joint["point"],
            "limits_deg": design_ranges[i - 1],
            "zero": "pose in the source file",
        })
    eye = [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]
    for i, module in enumerate(modules):
        ratio = module["joint"]["joint_deg_per_motor_deg"]
        for j in range(n):
            eye[i][j] *= ratio
    return {
        "frame": "base link, Z up, mm",
        "links": [{"id": f"L{i}"} for i in range(n + 1)],
        "joints": joints,
        "actuators": [{
            "id": f"M{i}", "brick": "left", "port": MOTOR_PORTS[i - 1],
            "motor": module["motor"]["kind"],
        } for i, module in enumerate(modules, 1)],
        "transmission": {
            "coupling": "q = C m, q in joint degrees, m in motor degrees",
            "C": eye,
        },
        "end_effector": {"link": f"L{n}", "point_mm": assembly["ee"]},
    }


def check_bom(bom: Counter, inventory: dict | None) -> dict:
    if inventory is None:
        return {"ok": True, "shortfalls": [], "note": "no inventory given"}
    stocked = {pid: row.get("total", 0)
               for pid, row in (inventory.get("parts") or {}).items()}
    shortfalls = [{"part_id": pid, "need": need,
                   "own": stocked.get(pid, 0)}
                  for pid, need in sorted(bom.items())
                  if need > stocked.get(pid, 0)]
    return {"ok": not shortfalls, "shortfalls": shortfalls}


class PrecheckError(ValueError):
    """A precheck failed; carries the report fragment (no output)."""

    def __init__(self, message: str, fragment: dict):
        super().__init__(message)
        self.fragment = fragment


def _hole_match_uids(points_mm: list[list[float]], geometry: list[dict],
                     frame: dict, catalog: dict) -> list[set[str]]:
    """Module part uids with a hole at each point (<= 0.1 LDU)."""
    from src.gen import mounts as _mounts

    rot_w, pos_w = frame["rot"], frame["pos"]
    matched = []
    for point in points_mm:
        want = np.array(point, dtype=float)
        uids = set()
        for part in geometry:
            entry = catalog.get(part["part_id"]) or {}
            if entry.get("category") != "beam":
                continue
            rot = np.array(part["rotation"], dtype=float)
            org = np.array(part["position_mm"], dtype=float)
            for centre in _mounts.hole_centres(entry):
                local = rot @ np.array(_mounts.conv_ldu_to_mm(
                    [0, 0, centre["z"]])) + org
                world = rot_w @ local + pos_w
                if float(np.linalg.norm(world - want)) <= 0.04:
                    uids.add(part["uid"])
                    break
        matched.append(uids)
    return matched


def assemble_design(design: dict, spec: dict, modules_dir: str | Path,
                    catalog: dict, inventory: dict | None = None,
                    mesh_root: str | None = None) -> dict:
    """Assemble with skeleton gate + 3 prechecks (raises on error)."""
    from src.gen import skeleton as _skeleton

    if spec.get("gripper"):
        raise ValueError("gripper arms are not supported in Part 1")
    if len(design.get("joints", [])) != len(spec["joints"]):
        raise ValueError("design joints must match spec joints")
    skel = _skeleton.check_skeleton(
        spec, _skeleton.joint_frames(spec))
    if not skel["overall_pass"]:
        raise PrecheckError(
            f"skeleton red, refusing assembly: {skel}",
            {"failed": "skeleton", "skeleton": skel})
    modules_dir = Path(modules_dir)
    modules, geometries, names = [], [], []
    for i, pick in enumerate(design["joints"], 1):
        module_path = modules_dir / pick["module"] / "module.json"
        module = load_module(module_path)
        errors = validate_module(module, module_path.parent, catalog)
        if errors:
            raise ValueError(f"{pick['module']}: {errors[0]}")
        geometry = ldraw_io.read_module_geometry(module_path.parent, module)
        counts = Counter(p["part_id"] for p in geometry)
        if dict(counts) != dict(module["bom"]):
            raise ValueError(
                f"{pick['module']}: geometry counts {dict(counts)} "
                f"miss module bom {module['bom']}")
        if "span_mm" not in pick:
            raise ValueError(f"J{i}: pick needs span_mm (enumerate it)")
        modules.append(module)
        geometries.append(geometry)
        names.append(pick["module"])

    design_ranges, range_errors = _design_ranges(spec, modules)
    if range_errors:
        raise ValueError("; ".join(range_errors))
    from src.gen.templates import mounts_compatible

    for i in range(len(modules) - 1):
        ok, reason = mounts_compatible(modules[i]["output_mount"],
                                       modules[i + 1]["input_mount"])
        if not ok:
            raise ValueError(f"J{i + 1} -> J{i + 2}: {reason}")
    spans = [float(pick["span_mm"]) for pick in design["joints"]]
    comp = compose_frames(modules, spans)
    link_errors = [abs(got - want) for got, want in
                   zip(comp["link_lengths"], spec["link_lengths_mm"])]
    link_ok = all(e <= LINK_TOL_MM for e in link_errors)

    # Flat placed parts (module geometries + gap beams + elements).
    parts: list[dict] = []
    counter = 0

    def _next_uid() -> str:
        nonlocal counter
        counter += 1
        return f"p{counter:06d}"

    for i, (geometry, frame) in enumerate(zip(geometries,
                                              comp["frames"])):
        for part in geometry:
            moved = _transform_points(
                (frame["rot"], frame["pos"]),
                np.array([part["position_mm"]]))[0]
            parts.append({**part, "uid": _next_uid(),
                          "position_mm": [float(v) for v in moved],
                          "rotation": (frame["rot"] @ np.array(
                              part["rotation"])).tolist(),
                          "_segment": i})
    gaps = []
    for i, pick in enumerate(design["joints"]):
        has_next = i < len(modules) - 1
        try:
            solved = solve_gap(
                modules[i], comp["frames"][i],
                modules[i + 1] if has_next else None,
                comp["frames"][i + 1] if has_next else None,
                pick["beam"], catalog)
        except ValueError as exc:
            raise PrecheckError(
                f"gap {i}: beam {pick['beam']} misfits: {exc}",
                {"failed": "pre1-solve",
                 "gap": i, "beam": pick["beam"]}) from exc
        beam_uid = _next_uid()
        parts.append({"uid": beam_uid, "part_id": pick["beam"],
                      "colour": SEGMENT_COLOURS[i % len(SEGMENT_COLOURS)],
                      "position_mm": solved["origin_mm"],
                      "rotation": solved["rotation"], "_segment": i + 1,
                      "_beam": True})
        r_mount = comp["frames"][i]["rot"] @ np.array(
            modules[i]["mount_out"]["frame"], dtype=float)[:, :3]
        element_uids = []
        for point, element in zip(solved["points_mm"],
                                  solved["elements"]):
            uid = _next_uid()
            element_uids.append(uid)
            # Elements seat upstream (static about the spanned joint);
            # beams ride downstream (they rotate about it).
            parts.append({"uid": uid, "part_id": element, "colour": "7",
                          "position_mm": list(point),
                          "rotation": r_mount.tolist(),
                          "_segment": i, "_element": True})
        out_holes = [h["point_mm"] for h in mount_points_world(
            modules[i], comp["frames"][i]["rot"],
            comp["frames"][i]["pos"])["mount_out"]]
        n_out = sum(any(float(np.linalg.norm(
            np.array(p) - np.array(h))) <= 0.04 for h in out_holes)
            for p in solved["points_mm"])
        if has_next:
            in_holes = [h["point_mm"] for h in mount_points_world(
                modules[i + 1], comp["frames"][i + 1]["rot"],
                comp["frames"][i + 1]["pos"])["mount_in"]]
            n_in = sum(any(float(np.linalg.norm(
                np.array(p) - np.array(h))) <= 0.04 for h in in_holes)
                for p in solved["points_mm"])
        else:
            n_in = 0
        out_matched = _hole_match_uids(
            solved["points_mm"], geometries[i], comp["frames"][i],
            catalog)
        if has_next:
            in_matched = _hole_match_uids(
                solved["points_mm"], geometries[i + 1],
                comp["frames"][i + 1], catalog)
        else:
            in_matched = [set() for _ in solved["points_mm"]]
        mates = []
        namespaced = []
        for k, (out_set, in_set) in enumerate(zip(out_matched,
                                                 in_matched)):
            edge = [f"b{i}"]
            here = set()
            if out_set:
                edge.append(f"m{i}")
                here.update(f"m{i}:{u}" for u in out_set)
            if in_set:
                edge.append(f"m{i + 1}")
                here.update(f"m{i + 1}:{u}" for u in in_set)
            mates.append(edge)
            namespaced.append(here)
        gaps.append({"part_id": pick["beam"],
                     "origin_mm": solved["origin_mm"],
                     "rotation": solved["rotation"],
                     "points_mm": solved["points_mm"],
                     "beam_kinds": solved["beam_kinds"],
                     "elements": solved["elements"],
                     "element_uids": element_uids, "beam_uid": beam_uid,
                     "n_out": n_out, "n_in": n_in, "has_next": has_next,
                     "mates": mates, "matched": namespaced})

    pre1 = check_pre1_connectivity(gaps, modules, catalog)
    if not pre1["ok"]:
        raise PrecheckError(f"pre1 connectivity red: {pre1}",
                            {"failed": "pre1", "pre1": pre1})
    mated: set[tuple[str, str]] = set()
    for gi, gap in enumerate(gaps):
        for uid, matched in zip(gap["element_uids"], gap["matched"]):
            clique = [uid, gap["beam_uid"]] + sorted(matched)
            for a in clique:
                for b in clique:
                    if a != b:
                        mated.add((a, b))
    # Remap module-local uids to placed uids (emission order).
    local_to_placed: dict[str, str] = {}
    cursor = 0
    for i, geometry in enumerate(geometries):
        for part in geometry:
            local_to_placed[f"m{i}:{part['uid']}"] = \
                parts[cursor]["uid"]
            cursor += 1
    remapped = set()
    for a, b in mated:
        ra = local_to_placed.get(a, a)
        rb = local_to_placed.get(b, b)
        remapped.add((ra, rb))
    # Bolted pairs (gap beam vs its upstream module) are checked at
    # the zero pose but skipped once articulated: the bolted beam
    # swinging about the remote joint pivot is a virtual-model
    # artifact, not a mechanism motion.
    bolted = set()
    module_uids: list[set[str]] = []
    for i in range(len(geometries)):
        uids = {p["uid"] for p in parts
                if p.get("_segment") == i and not p.get("_element")
                and not p.get("_beam")}
        module_uids.append(uids)
    for gi, gap in enumerate(gaps):
        for uid in module_uids[gi]:
            bolted.add((gap["beam_uid"], uid))
            bolted.add((uid, gap["beam_uid"]))
    pre2 = check_pre2_interference(
        parts, comp["joints"], design_ranges, catalog, remapped,
        mesh_root, bolted)
    if not pre2["ok"]:
        raise PrecheckError(f"pre2 interference red: {pre2}",
                            {"failed": "pre2", "pre2": pre2})

    bom: Counter = Counter()
    for module in modules:
        bom.update(module["bom"])
    for gap in gaps:
        bom[gap["part_id"]] += 1
        bom.update(gap["elements"])
    bom_check = check_bom(bom, inventory)
    score = (round(sum(link_errors), 6), sum(bom.values()))
    kinematics = build_expected_kinematics(
        modules, {"joints": comp["joints"], "ee": comp["ee"]},
        design_ranges)
    report = {
        "ok": bool(link_ok and bom_check["ok"]),
        "score": list(score),
        "link_lengths": comp["link_lengths"],
        "link_errors_mm": link_errors,
        "link_ok": link_ok,
        "reach_spec_mm": float(sum(spec["link_lengths_mm"])),
        "reach_actual_mm": float(sum(comp["link_lengths"])),
        "design_ranges_deg": design_ranges,
        "pre1": pre1,
        "pre2": pre2,
        "bom": dict(sorted(bom.items())),
        "bom_check": bom_check,
        "expected_kinematics": kinematics,
        "parts": parts,
        "_gaps": gaps,
        "_names": names,
    }
    from src.gen import ev3_gate

    report["ev3_gate"] = ev3_gate.check_ev3(report, catalog, inventory)
    return report


def render_mpd(report: dict, modules_dir: str | Path,
               name: str = "robot") -> tuple[str, float]:
    """Module refs + beams + elements over FILE blocks (MPD text)."""
    modules_dir = Path(modules_dir)
    lines = [f"0 5D generated robot {name}", "0 Name: robot.ldr",
             f"0 Author: 5D generator ({name})"]
    max_snap = 0.0
    seen: set[str] = set()
    blocks: list[str] = []
    for part in report["parts"]:
        if part.get("_beam") or part.get("_element"):
            line, snap = ldraw_io.format_type1(part)
            lines.append(line)
            max_snap = max(max_snap, snap)
    # Refs need module frames: recover from placed module parts by
    # re-reading local geometry (never composer memory).
    ref_lines: list[str] = []
    for i, mod_name in enumerate(report["_names"]):
        local = ldraw_io.read_module_geometry(
            modules_dir / mod_name, {"geometry": "module.ldr"})
        placed = [p for p in report["parts"]
                  if p.get("_segment") == i and not p.get("_beam")
                  and not p.get("_element")]
        if len(placed) != len(local):
            raise ValueError(f"{mod_name}: placed {len(placed)} != "
                             f"local {len(local)}")
        # Frame from the first part (rigid subfile, exact).
        lp, pp = local[0], placed[0]
        rot = np.array(pp["rotation"]) @ np.array(
            lp["rotation"]).T
        pos = np.array(pp["position_mm"]) - rot @ np.array(
            lp["position_mm"])
        line, snap = ldraw_io.format_type1(
            {"position_mm": [float(v) for v in pos],
             "rotation": rot.tolist(), "part_id": f"{mod_name}.ldr",
             "colour": "16"})
        ref_lines.append(line)
        max_snap = max(max_snap, snap)
        if mod_name not in seen:
            seen.add(mod_name)
            text = (modules_dir / mod_name / "module.ldr").read_text(
                encoding="utf-8")
            blocks.append(f"0 FILE {mod_name}.ldr")
            blocks += [ln for ln in text.splitlines() if ln.strip()]
            blocks.append("0 NOFILE")
    lines[3:3] = ref_lines
    return "\n".join(lines + blocks) + "\n", max_snap


def write_outputs(report: dict, out_dir: str | Path,
                  modules_dir: str | Path,
                  name: str = "robot") -> dict:
    """Write robot.ldr (MPD), expected_kinematics.json, report.json."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    segments = {p["uid"]: p.get("_segment", 0) for p in report["parts"]}
    beams = [{"uid": p["uid"], "part_id": p["part_id"],
              "segment": p.get("_segment", 0)}
             for p in report["parts"] if p.get("_beam")]
    elements = [{"uid": p["uid"], "part_id": p["part_id"],
                 "segment": p.get("_segment", 0),
                 "point_mm": p["position_mm"]}
                for p in report["parts"] if p.get("_element")]
    text, max_snap = render_mpd(report, modules_dir, name)
    report["_segmap"] = segments
    for part in report["parts"]:
        part.pop("_segment", None)
        part.pop("_beam", None)
        part.pop("_element", None)
    (out_dir / "robot.ldr").write_text(text, encoding="utf-8")
    (out_dir / "expected_kinematics.json").write_text(
        json.dumps(report["expected_kinematics"], indent=2), encoding="utf-8")
    summary = {k: v for k, v in report.items()
               if k not in ("parts", "expected_kinematics", "_gaps",
                            "_segmap")}
    summary["segments"] = segments
    summary["beams"] = beams
    summary["elements"] = elements
    summary["ldraw"] = {"n_parts": len(report["parts"]),
                        "max_snap_mm": max_snap}
    (out_dir / "report.json").write_text(json.dumps(summary, indent=2),
                                         encoding="utf-8")
    return {"robot.ldr": summary["ldraw"], "dir": str(out_dir),
            "text": text}


def assemble_ranked(candidates: list[dict], spec: dict,
                    modules_dir: str | Path, catalog: dict,
                    inventory: dict | None = None,
                    mesh_root: str | None = None) -> tuple[dict, list]:
    """Try ranked candidates in order; first 3-green wins (loud)."""
    tries = []
    for k, cand in enumerate(candidates):
        design = cand["design"] if "design" in cand else cand
        try:
            report = assemble_design(design, spec, modules_dir,
                                     catalog, inventory, mesh_root)
        except (PrecheckError, ValueError) as exc:
            fragment = getattr(exc, "fragment", {"error": str(exc)})
            tries.append({"rank": k, "score": cand.get("score"),
                          "ok": False, "error": str(exc)[:500],
                          "fragment": fragment})
            continue
        if not report["ok"]:
            tries.append({"rank": k, "score": cand.get("score"),
                          "ok": False, "error": "report not ok",
                          "fragment": {
                              "link_ok": report["link_ok"],
                              "bom_check": report["bom_check"]}})
            continue
        tries.append({"rank": k, "score": cand.get("score"), "ok": True})
        return report, tries
    raise PrecheckError(f"all {len(candidates)} candidates failed",
                        {"failed": "ranked", "tries": tries})


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Assemble a 5D design.")
    parser.add_argument("--design", required=True)
    parser.add_argument("--candidates", default=None,
                        help="ranked candidates.json (try in order)")
    parser.add_argument("--spec", required=True)
    parser.add_argument("--modules", default="assets/modules")
    parser.add_argument("--out", required=True)
    parser.add_argument("--inventory", default="config/inventory.local.yaml")
    parser.add_argument("--mesh", default=None,
                        help="Studio ldraw root for mesh BVH "
                        "(None: hole-box fallback, loud)")
    parser.add_argument("--name", default="robot")
    args = parser.parse_args(argv)

    with open(args.design, encoding="utf-8") as fh:
        design_doc = json.load(fh)
    design = design_doc.get("design", design_doc)
    spec = load_spec(args.spec)
    spec_errors = validate_spec(spec)
    if spec_errors:
        print(f"{args.spec}: INVALID: {spec_errors[0]}")
        return 1
    catalog = load_catalog_merged()
    inventory = None
    inventory_path = Path(args.inventory)
    if inventory_path.exists():
        with open(args.inventory, encoding="utf-8") as fh:
            inventory = yaml.safe_load(fh) or {}
    else:
        print(f"note: {args.inventory} missing, skipping stock check")
    mesh_root = args.mesh
    if mesh_root is None:
        from src.gen.meshbox import STUDIO_LDRAW, library_present

        if library_present(STUDIO_LDRAW):
            mesh_root = STUDIO_LDRAW
        else:
            print("note: Studio library missing, BVH uses hole-box "
                  "fallback (listed in pre2)")
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.candidates:
        with open(args.candidates, encoding="utf-8") as fh:
            candidates = json.load(fh)
    else:
        candidates = [{"design": design, "score": None}]
    try:
        report, tries = assemble_ranked(
            candidates, spec, args.modules, catalog, inventory,
            mesh_root)
    except (PrecheckError, ValueError) as exc:
        fragment = getattr(exc, "fragment", {"error": str(exc)})
        (out_dir / "assemble_report.json").write_text(
            json.dumps({"ok": False, "error": str(exc)[:2000],
                        **fragment}, indent=2),
            encoding="utf-8")
        print(f"assembly failed: {exc}")
        return 1
    written = write_outputs(report, out_dir, args.modules, args.name)
    # Pre3 + tool audit run against the emitted artifact text.
    names = report["_names"]
    modules = [load_module(Path(args.modules) / n / "module.json")
               for n in names]
    comp_joints = [{"axis": j["axis"], "point": j["point"]}
                   for j in comp_joints_of(report)]
    pre3 = check_pre3_frames(written["text"], modules, names, spec,
                             comp_joints)
    audit = audit_of_text(written["text"], report, catalog)
    report["pre3"] = pre3
    report["audit"] = audit
    summary_path = out_dir / "report.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    summary["pre3"] = pre3
    summary["audit"] = audit
    summary["tries"] = tries
    summary_path.write_text(json.dumps(summary, indent=2),
                            encoding="utf-8")
    (out_dir / "assemble_report.json").write_text(
        json.dumps({"ok": True, "tries": tries,
                    "winner": tries[-1]["rank"]}, indent=2),
        encoding="utf-8")
    if not pre3["ok"] or not audit["ok"]:
        print(f"post-emission checks red: pre3={pre3['ok']} "
              f"audit={audit['ok']} (see report.json)")
        (out_dir / "robot.ldr").unlink(missing_ok=True)
        (out_dir / "expected_kinematics.json").unlink(missing_ok=True)
        (out_dir / "report.json").unlink(missing_ok=True)
        fail = {"ok": False, "failed": "post-emission", "pre3": pre3,
                "audit": audit, "tries": tries}
        (out_dir / "assemble_report.json").write_text(
            json.dumps(fail, indent=2), encoding="utf-8")
        return 1
    snap = written["robot.ldr"]["max_snap_mm"]
    print(f"wrote {args.out}: ok={report['ok']} "
          f"score={report['score']} snap_mm={snap:.4f}")
    if snap > GRID_WARN_MM:
        print(f"warning: assembly is off the stud grid by {snap:.3f} mm")
        return 2
    return 0 if report["ok"] else 2


def comp_joints_of(report: dict) -> list[dict]:
    """Composer joints out of expected_kinematics (artifact order)."""
    return [{"axis": j["axis"], "point": j["point_mm"]}
            for j in report["expected_kinematics"]["joints"]]


def audit_of_text(text: str, report: dict, catalog: dict) -> dict:
    """Remap parsed artifact uids, then audit element matings."""
    from src.parse import parse_text

    parsed = parse_text(text)
    mine = report["parts"]
    if len(parsed) != len(mine):
        return {"ok": False,
                "error": f"parsed {len(parsed)} != placed {len(mine)}"}
    used = [False] * len(parsed)
    mapping: dict[str, str] = {}
    for mine_part in mine:
        best, best_i = None, -1
        for i, cand in enumerate(parsed):
            if used[i] or cand["part_id"] != mine_part["part_id"]:
                continue
            dist = float(np.linalg.norm(
                np.array(cand["position_mm"])
                - np.array(mine_part["position_mm"])))
            if best is None or dist < best:
                best, best_i = dist, i
        if best is None or best > 0.01:
            return {"ok": False,
                    "error": f"unmatched placed part {mine_part}"}
        used[best_i] = True
        mapping[mine_part["uid"]] = parsed[best_i]["uid"]
    segments = report.get("_segmap") or {
        p["uid"]: p.get("_segment", 0) for p in mine}
    for cand in parsed:
        mine_uid = next(m for m, c in mapping.items() if c == cand["uid"])
        cand["_segment"] = segments[mine_uid]
    element_mine = set()
    beam_of_mine: dict[str, str] = {}
    for gap in report.get("_gaps", []):
        element_mine.update(gap["element_uids"])
        for uid in gap["element_uids"]:
            beam_of_mine[uid] = gap["beam_uid"]
    parsed_elements = {mapping[u] for u in element_mine}
    beam_of = {mapping[u]: mapping[b]
               for u, b in beam_of_mine.items()}
    beam_uids = set(beam_of.values())
    return audit_element_matings(
        parsed, catalog, parsed_elements, beam_of, beam_uids)


if __name__ == "__main__":
    raise SystemExit(main())
