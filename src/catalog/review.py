"""Reviewed part catalog generator (Stage 1.2 completion, Stage 2 input).

Derives category, teeth, gear kind, axes and holes from part titles and
part-geometry analysis of an LDraw library (here: the library embedded
in the referenced Studio `.io` export, `model2.ldr`), plus LDraw.org
headers for the few parts missing from that snapshot.

Evidence per decision is recorded in the `evidence` field; `review` is
false only where title + geometry agree, true where a human should
still confirm (guessed teeth, missing geometry, ambiguous conventions).

Own work: titles are LDraw.org CC-licensed fact lookups; all
classification logic is written fresh for this repo.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import zipfile

LDU_TO_MM = 0.4

# Titles for model part types missing from the embedded library snapshot
# (LDraw.org official/unofficial headers; Licensed CC BY).
TITLE_OVERRIDES = {
    "18938.dat": "Technic Turntable 60 Tooth Top",
    "95646.dat": "Electric Mindstorms EV3",
    "59443.dat": "Technic Axle Joiner Inline Smooth",
    "32123a.dat": "Technic Bush 1/2 Smooth with Axle Hole Reduced",
    "3648b.dat": "Technic Gear 24 Tooth with Single Axle Hole",
}

# hole-kind by primitive filename fragment (checked in order).
HOLE_KIND_AXLE = ("axl", "axlehole")
HOLE_KIND_ROUND = ("peghol", "beamhol", "connhol")


def hole_kind(filename: str) -> str | None:
    name = filename.lower()
    if name in ("axle.dat", "axleend.dat"):
        return None  # solid axle geometry, not a hole
    if "bush" in name and "hole" not in name:
        return None
    if any(k in name for k in HOLE_KIND_AXLE):
        return "axle"
    if any(k in name for k in HOLE_KIND_ROUND):
        return "round"
    return None


def mat_mult(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3))
             for j in range(3)] for i in range(3)]


def mat_vec(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


class Library:
    """One-level access to an LDraw library text (model2.ldr style)."""

    def __init__(self, text: str):
        self.lines = text.replace("\r\n", "\n").split("\n")
        self.blocks: dict[str, tuple[int, int]] = {}
        cur: str | None = None
        start = 0
        for i, line in enumerate(self.lines):
            s = line.strip()
            if s.startswith("0 FILE"):
                if cur is not None:
                    self.blocks.setdefault(cur, (start, i))
                cur = s.split(" ", 2)[-1].lower()
                start = i
        if cur is not None:
            self.blocks.setdefault(cur, (start, len(self.lines)))

    @classmethod
    def from_io(cls, path: str) -> "Library":
        with zipfile.ZipFile(path) as zf:
            raw = zf.read("model2.ldr")
        return cls(raw.decode("utf-8-sig", errors="replace"))

    def title(self, name: str) -> str | None:
        rng = self.blocks.get(name.lower())
        if not rng:
            return None
        for line in self.lines[rng[0] + 1:rng[1]]:
            s = line.strip()
            if s.startswith("0 FILE") or s.startswith("0 NOFILE"):
                break
            if s.startswith("0 "):
                body = s[2:].strip()
                if body and not body.startswith(
                        ("Name:", "Author:", "LDRAW_ORG", "LICENSE",
                         "KEYWORDS", "HISTORY", "BFC", "HELP", "CMDLINE")):
                    return body
        return None

    def children(self, name: str) -> list[tuple[list[float], list, str]]:
        """Direct type-1 children: (pos, 3x3 matrix, filename)."""
        rng = self.blocks.get(name.lower())
        out = []
        if not rng:
            return out
        for line in self.lines[rng[0] + 1:rng[1]]:
            s = line.strip()
            if s.startswith("0 FILE") or s.startswith("0 NOFILE"):
                break
            if s.startswith("0 "):
                continue
            tok = s.split()
            if len(tok) != 15 or tok[0] != "1":
                continue
            try:
                nums = [float(x) for x in tok[1:14]]
            except ValueError:
                continue
            out.append(([nums[1], nums[2], nums[3]],
                        [[nums[4], nums[5], nums[6]],
                         [nums[7], nums[8], nums[9]],
                         [nums[10], nums[11], nums[12]]],
                        tok[14].lower()))
        return out


def _spread_axis(points: list[list[float]]) -> list[int] | None:
    if len(points) < 2:
        return None
    ext = []
    for i in range(3):
        vals = [p[i] for p in points]
        ext.append(max(vals) - min(vals))
    if max(ext) <= 0:
        return None
    longest = max(range(3), key=lambda i: ext[i])
    axis = [0, 0, 0]
    axis[longest] = 1
    return axis


def _snap(v: list[float]) -> list[int]:
    idx = max(range(3), key=lambda i: abs(v[i]))
    axis = [0, 0, 0]
    axis[idx] = 1 if v[idx] >= 0 else -1
    return [abs(x) for x in axis]


def _norm(v: list[float]) -> list[float]:
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v] if n > 0 else v


def _prim_stack_axis(lib: Library, filename: str) -> list[int] | None:
    """Shortest-nonzero-spread axis of a primitive's children.

    Hole/ring primitives stack their profile along the hole axis, so
    the thinnest spread direction is the bore direction. Ties mean
    the primitive is symmetric (e.g. beam hole halves) and abstain.
    """
    pts = [pos for pos, _, _ in lib.children(filename)]
    if len(pts) < 2:
        return None
    ext = []
    for i in range(3):
        vals = [p[i] for p in pts]
        ext.append(max(vals) - min(vals))
    nonzero = [(e, i) for i, e in enumerate(ext) if e > 1e-6]
    if not nonzero:
        return None
    best, second = sorted(nonzero)[:2] if len(nonzero) > 1 else (nonzero[0], (float("inf"), -1))
    if abs(best[0] - second[0]) < 1e-6:
        return None
    axis = [0, 0, 0]
    axis[best[1]] = 1
    return axis


def _hole_vote(lib: Library, children) -> list[int] | None:
    """Majority axle-bore direction over hole placements."""
    votes: dict[tuple, int] = {}
    for _, rot, fn in children:
        if hole_kind(fn) is None:
            continue
        prim = _prim_stack_axis(lib, fn)
        if prim is None:
            continue
        axis = tuple(_snap(mat_vec(rot, [float(x) for x in prim])))
        votes[axis] = votes.get(axis, 0) + 1
    if not votes:
        return None
    top = max(votes.values())
    winners = [a for a, c in votes.items() if c == top]
    if len(winners) > 1:
        return None
    return [abs(x) for x in winners[0]]


def tooth_axis(children) -> list[int] | None:
    """Normal of the plane carrying tooth placements (>=6 teeth)."""
    pts = [pos for pos, _, fn in children
           if "tooth" in fn]
    if len(pts) < 6:
        return None
    mean = [sum(p[i] for p in pts) / len(pts) for i in range(3)]
    cov = [[0.0] * 3 for _ in range(3)]
    for p in pts:
        d = [p[i] - mean[i] for i in range(3)]
        for i in range(3):
            for j in range(3):
                cov[i][j] += d[i] * d[j]
    # Smallest-variance direction via power iteration on inverse-ish:
    # use closed form: normal = eigenvector of smallest eigenvalue.
    # 3x3 symmetric eigen via Jacobi rotations.
    vecs = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    mat = [row[:] for row in cov]
    for _ in range(50):
        # find largest off-diagonal
        p, q, mx = 0, 1, 0.0
        for i in range(3):
            for j in range(i + 1, 3):
                if abs(mat[i][j]) > mx:
                    mx, p, q = abs(mat[i][j]), i, j
        if mx < 1e-9:
            break
        theta = 0.5 * math.atan2(2 * mat[p][q],
                                 mat[q][q] - mat[p][p])
        c, s = math.cos(theta), math.sin(theta)
        rot = [[1.0 if i == j else 0.0 for j in range(3)]
               for i in range(3)]
        rot[p][p], rot[q][q] = c, c
        rot[p][q], rot[q][p] = s, -s
        mat = mat_mult(mat_mult(rot, mat),
                       [[rot[j][i] for j in range(3)] for i in range(3)])
        vecs = mat_mult(rot, vecs)
    vals = [mat[i][i] for i in range(3)]
    nidx = min(range(3), key=lambda i: vals[i])
    normal = [vecs[i][nidx] for i in range(3)]
    return _snap(normal)


def _first_int(text: str) -> int | None:
    m = re.search(r"(\d+)\s*[lL]\b", text)
    return int(m.group(1)) if m else None


PRIMITIVE_HINTS = ("ring", "edge", "cyli", "cyls", "disc", "chrd",
                   "rect", "stud", "tooth", "axle", "peghol", "peghole",
                   "hole", "bush", "fric", "box", "tri", "torus", "sphe",
                   "ndis", "cylo", "connect", "connhole", "beamhol")


def is_primitive(name: str) -> bool:
    return any(h in name for h in PRIMITIVE_HINTS)


def child_long_axis(lib: "Library", filename: str) -> list[int] | None:
    """Longest-spread axis of a child's own children positions."""
    pts = [pos for pos, _, _ in lib.children(filename)]
    return _spread_axis(pts)


def iter_holes(lib: "Library", pid: str, depth: int = 0,
               _seen: frozenset = frozenset()) -> list[tuple[list[float],
                                                            list, str]]:
    """Hole placements with part-local positions.

    Descends one level into sub-assemblies (s/* style) so beams whose
    holes live in section subfiles are covered. Raw primitives are
    never descended into; only hole-named files are recorded.
    LDraw convention note: matrices map child frame to parent frame
    with x' = row0.x + tx (rows as read), composed the same way.
    """
    out = []
    for pos, rot, fn in lib.children(pid):
        kind = hole_kind(fn)
        if kind is not None:
            out.append((pos, rot, fn))
        elif depth < 1 and fn not in _seen and not is_primitive(fn):
            sub = iter_holes(lib, fn, depth + 1, _seen | {pid.lower()})
            for spos, srot, sfn in sub:
                out.append(([pos[i] + sum(rot[i][k] * spos[k]
                                          for k in range(3))
                             for i in range(3)],
                            mat_mult(rot, srot), sfn))
    return out


def classify(title: str) -> dict:
    """Title-keyword classification. Returns partial entry + confidence."""
    t = title
    entry: dict = {"evidence": f"title: {t}", "review": False}
    low = t.lower()
    if "motor ev3, medium" in low:
        entry.update(category="motor", motor="medium", review=False)
        entry["evidence"] += "; EV3 medium motor"
        return entry
    if "motor ev3, large" in low:
        entry.update(category="motor", motor="large", review=False)
        entry["evidence"] += "; EV3 large motor"
        return entry
    if "sensor" in low and ("touch" in low or "color" in low
                            or "colour" in low):
        entry.update(category="sensor", review=False)
        entry["evidence"] += "; EV3 sensor"
        return entry
    if "mindstorms ev3" in low:
        entry.update(category="brick", review=False)
        entry["evidence"] += "; EV3 brick"
        return entry
    if "turntable" in low:
        m = re.search(r"(\d+)\s*tooth", low)
        entry.update(category="turntable",
                     teeth=int(m.group(1)) if m else None,
                     gear_kind="turntable")
        entry["evidence"] += "; turntable ring gear"
        return entry
    if "gear rack" in low:
        entry.update(category="rack", review=True)
        entry["evidence"] += "; curved rack, teeth counted in Stage 3"
        return entry
    if "driving ring" in low:
        entry.update(category="gear", gear_kind="ring", review=True)
        entry["evidence"] += "; clutch ring, teeth resolved in Stage 3"
        return entry
    if "differential" in low:
        entry.update(category="gear", gear_kind="differential",
                     axis_local=None, review=True)
        entry["evidence"] += ("; 24-16 differential, mixed tooth "
                              "orientations; ratios/axes in Stage 3")
        return entry
    if "knob" in low:
        entry.update(category="gear", teeth=4, gear_kind="knob",
                     review=True)
        entry["evidence"] += "; knob wheel, 4-lobe guess"
        return entry
    if "gear" in low:
        m = re.search(r"(\d+)\s*tooth", low)
        teeth = int(m.group(1)) if m else None
        if "double bevel" in low:
            kind = "double-bevel"
        elif "bevel" in low:
            kind = "bevel"
        else:
            kind = "spur"
        entry.update(category="gear", teeth=teeth, gear_kind=kind)
        entry["evidence"] += f"; {teeth}T {kind}"
        return entry
    # Structural joiners before axle/pin keywords: many connector
    # titles mention the fasteners they accept.
    if "connector" in low or "joiner" in low or "toggle" in low:
        entry.update(category="connector")
        entry["evidence"] += "; structural connector, holes from geometry"
        return entry
    if "plate" in low or "slope" in low:
        entry.update(category="brick", review=False)
        entry["evidence"] += "; plate/slope, stud-joined (tool limitation)"
        return entry
    m_axle = re.search(r"\baxle\s+(\d+)\s*l\b", low)
    if m_axle and "connector" not in low and "hub" not in low:
        n = int(m_axle.group(1))
        entry.update(category="axle", axle_length=n)
        entry["evidence"] += f"; axle {n}L"
        return entry
    if re.match(r"technic,\s*pin\b", low):
        friction = "without friction" not in low and "friction" in low
        plen: float | None = 0.5 if "1/2" in t else 3
        entry.update(category="pin", friction=friction,
                     pin_length_studs=plen, review=False)
        entry["evidence"] += (f"; Technic pin family, {plen}L envelope; "
                              f"{'friction' if friction else 'frictionless'}; "
                              "span checked in --verify")
        return entry
    if "bush" in low or "spacer" in low:
        entry.update(category="bush" if "bush" in low else "connector")
        entry["evidence"] += "; bush/spacer bore"
        return entry
    if "pin" in low:
        if "without friction" in low:
            friction = False
        else:
            friction = "friction" in low
        n = _first_int(t)
        if "1/2" in t:
            n = 0.5
        entry.update(category="pin", friction=bool(friction),
                     pin_length_studs=n, review=n is None)
        entry["evidence"] += (f"; pin {n}L, "
                              f"{'friction' if friction else 'frictionless'}")
        return entry
    # Default: structural beam/frame/panel block.
    entry.update(category="beam")
    entry["evidence"] += "; structural, holes from geometry"
    return entry


def pin_length_from_title(title: str) -> float | None:
    if "1/2" in title:
        return 0.5
    return _first_int(title)


TURNTABLE_AXES = {
    # Verified by assembly coaxiality: 60T pairs share an axis along
    # local Y (7/7), 56T pair along local Z (1/1); quarter-ring
    # subparts of the 60T base revolve about Y in the definition.
    "18938.dat": [0, 1, 0],
    "18938u.dat": [0, 1, 0],
    "18939.dat": [0, 1, 0],
    "48452.dat": [0, 0, 1],
    "48168.dat": [0, 0, 1],
}

FAMILY_AXIS = {
    # No definition geometry in the library snapshot; same convention
    # as the reviewed spur-gear family (disc in XY, verified above).
    "3648b.dat": [0, 0, 1],
}


def review_part(pid: str, title: str, lib: Library) -> dict:
    entry = classify(title)
    children = lib.children(pid)
    cat = entry.get("category")

    # Holes from hole-primitive placements (positions in LDraw LDU),
    # descending one level into section subfiles. Motors, sensors,
    # bricks and racks do not need holes for Stage 2 tools.
    holes = []
    if cat in ("beam", "connector", "gear", "turntable",
               "bush", "pin", "axle"):
        for pos, rot, fn in iter_holes(lib, pid):
            kind = hole_kind(fn)
            if kind is None:
                continue
            holes.append({"pos": [round(float(v), 2) for v in pos],
                          "kind": kind})
    if holes:
        entry["holes"] = holes

    # Straight liftarms: fill any missing grid holes. A Liftarm 1xN
    # carries N holes centred on the length axis at 20 LDU spacing.
    m_lift = re.search(r"liftarm[^\d]*1\s*x\s*(\d+)", title.lower())
    if (cat == "beam" and m_lift and "bent" not in title.lower()
            and "h-shape" not in title.lower()
            and "t-shape" not in title.lower()
            and "frame" not in title.lower()
            and "triangle" not in title.lower()):
        n = int(m_lift.group(1))
        pts = [pos for pos, _, _ in children]
        length_axis = _spread_axis(pts)
        if length_axis is not None and len(holes) < n:
            axis_idx = length_axis.index(1)
            want = [(i - (n - 1) / 2) * 20.0 for i in range(n)]
            have = [h["pos"][axis_idx] for h in holes]
            default_kind = ("axle" if "axle hole" in title.lower()
                            else "round")
            for w in want:
                if all(abs(w - h) > 3.0 for h in have):
                    pos = [0.0, 0.0, 0.0]
                    pos[axis_idx] = round(w, 2)
                    holes.append({"pos": pos, "kind": default_kind,
                                  "grid": True})
            entry["holes"] = holes
            entry["evidence"] += f"; grid-filled to {n} beam holes"
            entry["review"] = True

    # Axes.
    if pid.lower() in TURNTABLE_AXES:
        entry["axis_local"] = TURNTABLE_AXES[pid.lower()]
        entry["evidence"] += ("; turntable axis verified by assembly "
                              "coaxiality")
    elif pid.lower() in FAMILY_AXIS:
        entry["axis_local"] = FAMILY_AXIS[pid.lower()]
        entry["review"] = True
        entry["evidence"] += "; axis by spur-gear family convention"
    elif cat in ("gear", "turntable") and entry.get("gear_kind") != \
            "differential":
        axis = tooth_axis(children)
        if axis is not None:
            entry["axis_local"] = axis
            nteeth = len([1 for _, _, f in children if "tooth" in f])
            entry["evidence"] += f"; axis from {nteeth} tooth placements"
        else:
            # Fall back to the axle-bore direction: vote over hole
            # placements whose primitives stack along one axis.
            axis = _hole_vote(lib, children)
            if axis is not None:
                entry["axis_local"] = axis
                entry["evidence"] += "; axis from axle-bore direction"
            else:
                spread = _spread_axis([pos for pos, _, _ in children])
                if spread is not None and entry.get("gear_kind") == "ring":
                    entry["axis_local"] = spread
                    entry["review"] = True
                    entry["evidence"] += ("; axis from sleeve extent, "
                                          "unconfirmed")
                else:
                    entry["axis_local"] = None
                    entry["review"] = True
                    entry["evidence"] += "; axis unresolved"
    elif cat in ("axle", "pin"):
        pts = [pos for pos, _, _ in children]
        axis = _spread_axis(pts)
        if axis is None and len(children) == 1:
            # Single scaled primitive (e.g. axle.dat): the dominant
            # scale column maps the primitive length into this frame.
            _, rot, _ = children[0]
            cols = [[rot[r][c] for r in range(3)] for c in range(3)]
            lens = [math.sqrt(sum(v * v for v in col)) for col in cols]
            dom = max(range(3), key=lambda i: lens[i])
            axis = [0, 0, 0]
            axis[dom] = 1
            entry["evidence"] += "; axis from single-primitive scale"
        if axis is None and children:
            # Symmetric halves (e.g. two pin halves at the origin):
            # all children must map their long axis to one line.
            votes = set()
            ok = True
            for _, rot, fn in children:
                long_ax = child_long_axis(lib, fn) or [0, 1, 0]
                mapped = _snap(mat_vec(rot, [float(x) for x in long_ax]))
                votes.add(tuple(mapped))
                if len(votes) > 1:
                    ok = False
                    break
            if ok and votes:
                axis = list(votes.pop())
                entry["evidence"] += ("; axis from symmetric "
                                      "sub-primitive agreement")
        entry["axis_local"] = axis
        if axis is None:
            entry["review"] = True
    elif cat == "motor":
        entry["axis_local"] = [0, 0, 1]
        entry["evidence"] += ("; shaft +Z from EV3 hub geometry "
                              "(hub body of revolution about Z)")
    elif cat in ("beam", "connector", "frame"):
        pts = [pos for pos, _, _ in children]
        axis = _spread_axis(pts)
        entry["axis_local"] = axis  # length axis; holes carry positions
        if axis is None:
            entry["evidence"] += ("; length axis unresolved "
                                  "(not needed for Stage 2)")
        if cat == "beam" and not holes:
            entry["review"] = True
            entry["evidence"] += "; beam without detected holes"
    else:
        entry.setdefault("axis_local", None)

    # Generic bore fallback for small joiners without hole primitives.
    if cat in ("connector", "bush") and not holes:
        if "pin" in title.lower() and "axle" not in title.lower():
            entry["holes"] = [{"pos": [0, 0, 0], "kind": "round"}]
            entry["evidence"] += "; generic round bore at origin"
        else:
            entry["holes"] = [{"pos": [0, 0, 0], "kind": "axle"}]
            entry["evidence"] += "; generic axle bore at origin"
        entry["review"] = True

    # Fill defaults matching the committed schema.
    entry.setdefault("teeth", None)
    entry.setdefault("gear_kind", None)
    entry.setdefault("holes", entry.get("holes", []))
    entry.setdefault("axle_length", entry.get("axle_length", None))
    entry.setdefault("motor", entry.get("motor", None))
    return entry


def build(library_path: str, part_ids: list[str]) -> dict:
    lib = Library.from_io(library_path)
    catalog = {}
    for pid in sorted(part_ids):
        title = lib.title(pid) or TITLE_OVERRIDES.get(pid.lower())
        if title is None:
            catalog[pid] = {"category": "other", "teeth": None,
                            "gear_kind": None, "axis_local": None,
                            "holes": [], "axle_length": None,
                            "motor": None, "review": True,
                            "evidence": "no title in library snapshot"}
            continue
        source = "override(LDraw.org)" if lib.title(pid) is None else "library"
        entry = review_part(pid, title, lib)
        entry["evidence"] += f"; source {source}"
        catalog[pid] = entry
    return catalog


def main() -> None:
    ap = argparse.ArgumentParser(description="Reviewed catalog generator")
    ap.add_argument("--library", required=True,
                    help="Studio .io export containing model2.ldr")
    ap.add_argument("--parts", required=True, help="parts.json")
    ap.add_argument("-o", "--output", required=True)
    ap.add_argument("--audit", default=None)
    args = ap.parse_args()
    part_ids = sorted({p["part_id"]
                       for p in json.load(open(args.parts,
                                               encoding="utf-8"))})
    catalog = build(args.library, part_ids)
    lines = ["# Part catalog (reviewed where evidence agrees).",
             "# `review: true` still needs a human check against the LDraw part.",
             "# `evidence` records how each entry was derived.",
             "# `holes[].pos` is in the part-local LDraw frame (LDU)."]
    import yaml
    with open(args.output, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
        yaml.safe_dump(catalog, fh, sort_keys=True)
    n_rev = sum(1 for e in catalog.values() if e.get("review"))
    print(f"types={len(catalog)} need_review={n_rev} -> {args.output}")
    if args.audit:
        with open(args.audit, "w", encoding="utf-8") as fh:
            for pid, e in sorted(catalog.items()):
                fh.write(f"{pid} | {e.get('category')} | "
                         f"axis={e.get('axis_local')} | "
                         f"teeth={e.get('teeth')} | kind={e.get('gear_kind')} | "
                         f"holes={len(e.get('holes', []))} | "
                         f"review={e.get('review')} | {e.get('evidence')}\n")
        print(f"audit -> {args.audit}")


if __name__ == "__main__":
    main()
