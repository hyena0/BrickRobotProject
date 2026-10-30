"""Stage 5D skeleton: spec-pure joint frames ahead of the generator.

Reads ``specs/<name>.json`` and emits per-joint 4x4 frames plus a
``skeleton.ldr`` (one 10-LDU cube and one axis stub per joint, raw
LDraw triangles for eyeball checks). Deliberately imports nothing
from the assembler: instruction-4 check (3) compares these frames
against frames re-extracted from the assembled artifact, so the two
paths must stay independent.

Checks: (a) adjacent axis relations per joint kinds, (b) joint gaps
against ``link_lengths_mm`` (+-0.5 mm), (c) chain end against the
spec FK value.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from src.gen.spec import load_spec, validate_spec

AXIS_OF = {"yaw": [0.0, 0.0, 1.0], "pitch": [0.0, 1.0, 0.0],
           "roll": [0.0, 0.0, 1.0]}
ORTHO_TOL = 0.01
PARA_TOL = 0.999
LINK_TOL_MM = 0.5
COLOURS = ["4", "1", "14", "2", "5", "6", "7"]


def joint_frames(spec: dict) -> list[dict]:
    """Per-joint frames: axis (spec kinds) + stacked points."""
    frames = []
    z = 0.0
    for i, (kind, length) in enumerate(
            zip(spec["joints"], spec["link_lengths_mm"]), 1):
        z += float(length) if i > 1 else float(length)
        axis = list(AXIS_OF[kind])
        rot = [[1.0 if a == b else 0.0 for b in range(3)]
               for a in range(3)]
        mat = [row + [[0.0, 0.0, z][a]] for a, row in enumerate(rot)]
        frames.append({"id": f"J{i}", "kind": kind, "axis": axis,
                       "point_mm": [0.0, 0.0, z],
                       "matrix": mat + [[0.0, 0.0, 0.0, 1.0]]})
    return frames


def _pair_ok(kind_a: str, axis_a, kind_b: str, axis_b) -> tuple[bool, str]:
    na = math.sqrt(sum(a * a for a in axis_a)) or 1.0
    nb = math.sqrt(sum(b * b for b in axis_b)) or 1.0
    dot = abs(sum(a * b for a, b in zip(axis_a, axis_b)) / (na * nb))
    same_chain = {kind_a, kind_b} <= {"yaw", "roll"}
    if kind_a == kind_b or same_chain:
        ok = dot > PARA_TOL
        return ok, (f"{kind_a}->{kind_b} parallel (|dot|={dot:.4f})"
                    f"{'' if ok else ' FAIL'}")
    ok = dot < ORTHO_TOL
    return ok, (f"{kind_a}->{kind_b} orthogonal (|dot|={dot:.4f})"
                f"{'' if ok else ' FAIL'}")


def check_skeleton(spec: dict, frames: list[dict]) -> dict:
    """Run checks (a)(b)(c); returns the verdict (never raises)."""
    rows_a = []
    for i in range(len(frames) - 1):
        ok, note = _pair_ok(frames[i]["kind"], frames[i]["axis"],
                            frames[i + 1]["kind"], frames[i + 1]["axis"])
        rows_a.append({"pair": f"J{i + 1}->J{i + 2}", "pass": ok,
                       "note": note})
    rows_b = []
    prev = [0.0, 0.0, 0.0]
    for frame, want in zip(frames, spec["link_lengths_mm"]):
        got = math.dist(frame["point_mm"], prev)
        err = abs(got - want)
        rows_b.append({"joint": frame["id"], "want_mm": want,
                       "got_mm": round(got, 3), "err_mm": round(err, 3),
                       "pass": err <= LINK_TOL_MM})
        prev = frame["point_mm"]
    want_end = [0.0, 0.0, sum(spec["link_lengths_mm"])]
    got_end = frames[-1]["point_mm"] if frames else [0.0, 0.0, 0.0]
    err_end = math.dist(got_end, want_end)
    check_c = {"want_mm": want_end,
               "got_mm": [round(v, 3) for v in got_end],
               "err_mm": round(err_end, 3),
               "pass": err_end <= LINK_TOL_MM}
    ok_a = all(r["pass"] for r in rows_a)
    ok_b = all(r["pass"] for r in rows_b)
    return {"a_axes": {"pass": ok_a, "pairs": rows_a},
            "b_links": {"pass": ok_b, "links": rows_b},
            "c_fk": {"pass": check_c["pass"], **check_c},
            "overall_pass": ok_a and ok_b and check_c["pass"]}


def _box_tris(cx, cy, cz, hx, hy, hz, colour: str) -> list[str]:
    x0, x1 = cx - hx, cx + hx
    y0, y1 = cy - hy, cy + hy
    z0, z1 = cz - hz, cz + hz
    quads = [
        [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)],
        [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],
        [(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)],
        [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],
        [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],
        [(x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)],
    ]
    lines = []
    for quad in quads:
        for tri in ((quad[0], quad[1], quad[2]),
                    (quad[0], quad[2], quad[3])):
            nums = " ".join(f"{v:g}" for pt in tri for v in pt)
            lines.append(f"4 {colour} {nums}")
    return lines


def write_skeleton_ldr(frames: list[dict], path: str | Path) -> Path:
    """10-LDU cubes + axis stubs (LDraw units, -Y up) for eyeballs."""
    lines = ["0 5D skeleton (eyeball only, not a part)",
             "0 Name: skeleton.ldr", "0 Author: 5D skeleton"]
    for i, frame in enumerate(frames):
        colour = COLOURS[i % len(COLOURS)]
        x, y, z = frame["point_mm"]
        # Module mm (Z up) into LDraw LDU: (x, y, z) -> (x, -z, y)/0.4.
        cx, cy, cz = x / 0.4, -z / 0.4, y / 0.4
        ax, ay, az = frame["axis"]
        lax, lay, laz = ax, -az, ay
        lines.append(f"0 // joint {frame['id']} ({frame['kind']}) "
                     f"axis [{ax:g}, {ay:g}, {az:g}]")
        lines += _box_tris(cx, cy, cz, 5, 5, 5, colour)
        ex, ey, ez = cx + lax * 20, cy + lay * 20, cz + laz * 20
        mx, my, mz = (cx + ex) / 2, (cy + ey) / 2, (cz + ez) / 2
        half = [2 + 18 * abs(v) for v in (lax, lay, laz)]
        lines += _box_tris(mx, my, mz, half[0], half[1], half[2],
                           colour)
    path = Path(path)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def run_skeleton(spec_path: str | Path, out_dir: str | Path) -> dict:
    """Validate the spec, emit frames + ldr, return the report."""
    spec = load_spec(spec_path)
    spec_errors = validate_spec(spec)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if spec_errors:
        report = {"ok": False, "spec_errors": spec_errors,
                  "overall_pass": False}
    else:
        frames = joint_frames(spec)
        checks = check_skeleton(spec, frames)
        (out_dir / "frames.json").write_text(
            json.dumps({"frames": frames, "checks": checks}, indent=2),
            encoding="utf-8")
        write_skeleton_ldr(frames, out_dir / "skeleton.ldr")
        report = {"ok": True, "frames": frames, "checks": checks,
                  "overall_pass": checks["overall_pass"]}
    (out_dir / "skeleton_report.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8")
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="5D skeleton check.")
    parser.add_argument("--spec", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    report = run_skeleton(args.spec, args.out)
    if not report["ok"]:
        print(f"{args.spec}: INVALID: {report['spec_errors'][0]}")
        return 1
    checks = report["checks"]
    print(f"skeleton {args.spec}: "
          f"a={'pass' if checks['a_axes']['pass'] else 'FAIL'} "
          f"b={'pass' if checks['b_links']['pass'] else 'FAIL'} "
          f"c={'pass' if checks['c_fk']['pass'] else 'FAIL'}")
    return 0 if report["overall_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
