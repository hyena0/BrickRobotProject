"""Stage 5D.1: joint module description loader and validator.

A module is a human-designed (Studio) single-joint unit built from owned
parts only. Each module lives in its own directory::

    assets/modules/<name>/
        module.json   # frames, joint, motor, BOM (this format)
        module.ldr    # LDraw geometry in the module frame

The module frame is shared by both files: the LDraw file uses standard
LDraw convention (LDU, -Y up) and ``module.json`` uses the same frame in
millimetres with Z up (the ``docs/schema.md`` LDraw conversion maps one
to the other).
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import yaml

from src.gen.inventory import load_catalog_merged

MOTOR_KINDS = ("large", "medium")
SENSOR_KINDS = ("touch", "color", "ultrasonic", "gyro", "infrared")
HOLE_TYPES = ("pin", "axle")
ROT_TOL = 1e-6
GRID_LDU = 10.0
MATCH_LDU = 0.1


def load_module(path: str | Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _is_num(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _check_vec3(value, what: str, errors: list[str]) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 3 \
            or not all(_is_num(v) for v in value):
        errors.append(f"{what} must be a list of 3 numbers")
        return None
    return [float(v) for v in value]


def _check_rotation(value, what: str, errors: list[str]) -> None:
    if not isinstance(value, list) or len(value) != 3 or any(
            not isinstance(row, list) or len(row) != 3
            or not all(_is_num(v) for v in row) for row in value):
        errors.append(f"{what} must be a 3x3 list of numbers")
        return
    rows = [[float(v) for v in row] for row in value]
    dots = [sum(a * b for a, b in zip(rows[i], rows[j]))
            for i in range(3) for j in range(i, 3)]
    want = [1.0, 0.0, 0.0, 1.0, 0.0, 1.0]
    if any(abs(got - exp) > ROT_TOL for got, exp in zip(dots, want)):
        errors.append(f"{what} must be orthonormal")
        return
    det = (rows[0][0] * (rows[1][1] * rows[2][2] - rows[1][2] * rows[2][1])
           - rows[0][1] * (rows[1][0] * rows[2][2] - rows[1][2] * rows[2][0])
           + rows[0][2] * (rows[1][0] * rows[2][1] - rows[1][1] * rows[2][0]))
    if abs(det - 1.0) > ROT_TOL:
        errors.append(f"{what} must have determinant +1")


def _check_mount(value, what: str, errors: list[str]) -> None:
    if not isinstance(value, dict):
        errors.append(f"{what} must be an object")
        return
    _check_vec3(value.get("position_mm"), f"{what}.position_mm", errors)
    _check_rotation(value.get("rotation"), f"{what}.rotation", errors)


def _check_hole(row, what: str, errors: list[str]) -> dict | None:
    if not isinstance(row, dict):
        errors.append(f"{what} must be an object")
        return None
    pos = row.get("pos_ldu")
    if not isinstance(pos, list) or len(pos) != 3 \
            or not all(_is_num(v) for v in pos):
        errors.append(f"{what}.pos_ldu must be a list of 3 numbers")
        return None
    for v in pos:
        if abs(v / GRID_LDU - round(v / GRID_LDU)) > 1e-6:
            errors.append(f"{what}.pos_ldu {pos} is off the "
                          f"{GRID_LDU:g}-LDU grid")
            break
    if row.get("type") not in HOLE_TYPES:
        errors.append(f"{what}.type must be one of {HOLE_TYPES}")
        return None
    return {"pos_ldu": [float(v) for v in pos], "type": row["type"]}


def _check_mount_block(value, legacy: dict | None, what: str,
                       errors: list[str]) -> dict | None:
    """Validate a mount_in/mount_out block (frame + holes)."""
    if not isinstance(value, dict):
        errors.append(f"{what} must be an object")
        return None
    frame = value.get("frame")
    rot = None
    if not isinstance(frame, list) or len(frame) != 3 or any(
            not isinstance(row, list) or len(row) != 4
            or not all(_is_num(v) for v in row) for row in frame):
        errors.append(f"{what}.frame must be a 3x4 list of numbers")
    else:
        rot = [row[:3] for row in frame]
        _check_rotation(rot, f"{what}.frame rotation", errors)
    holes = value.get("holes")
    rows: list[dict] = []
    if not isinstance(holes, list) or len(holes) < 2:
        errors.append(f"{what}.holes needs >= 2 holes (one hole "
                      "leaves a free rotation)")
    else:
        for i, row in enumerate(holes):
            checked = _check_hole(row, f"{what}.holes[{i}]", errors)
            if checked is not None:
                rows.append(checked)
    if legacy is not None and rot is not None \
            and isinstance(frame, list) and len(frame) == 3:
        pos = [frame[i][3] for i in range(3)]
        want_p = (legacy.get("position_mm") or []) if isinstance(
            legacy, dict) else []
        want_r = (legacy.get("rotation") or []) if isinstance(
            legacy, dict) else []
        if len(want_p) == 3 and all(
                abs(a - b) <= 1e-9 for a, b in zip(pos, want_p)) \
                and len(want_r) == 3 and all(
                abs(a - b) <= 1e-9
                for ra, rb in zip(rot, want_r) for a, b in zip(ra, rb)):
            pass
        else:
            errors.append(f"{what}.frame must match the legacy mount")
    if rot is not None and len(rows) >= 2:
        # Pin lines run along the mount frame +X: the holes must span
        # at least two distinct lines (coaxial holes spin freely).
        direction = [rot[0][0], rot[1][0], rot[2][0]]
        seen = set()
        for row in rows:
            p = row["pos_ldu"]
            along = sum(p[i] * direction[i] for i in range(3))
            perp = tuple(round(p[i] - along * direction[i], 3)
                         for i in range(3))
            seen.add(perp)
        if len(seen) < 2:
            errors.append(f"{what}.holes are coaxial (rotation "
                          "unconstrained)")
    if rot is None or not rows:
        return None
    return {"frame": frame, "holes": rows}


def _beam_centres_mm(module_dir: Path, module: dict,
                     catalog: dict) -> list[tuple]:
    """(centre mm, kind, part_id) over beam parts in the subfile."""
    from src.gen.mounts import (centre_kind, conv_ldu_to_mm,
                                hole_centres)
    from src.parse import parse_text

    text = (module_dir / module["geometry"]).read_text(encoding="utf-8")
    parts = parse_text(text)
    out = []
    for part in parts:
        entry = catalog.get(part["part_id"]) or {}
        if entry.get("category") != "beam":
            continue
        rot = part["rotation"]
        org = part["position_mm"]
        for centre in hole_centres(entry):
            local = conv_ldu_to_mm([0, 0, centre["z"]])
            world = [org[i] + sum(rot[i][k] * local[k] for k in range(3))
                     for i in range(3)]
            out.append((world, centre_kind(centre), part["part_id"]))
    return out


def validate_module(module: dict, module_dir: str | Path,
                    catalog: dict | None = None,
                    inventory: dict | None = None) -> list[str]:
    """Return a list of validation errors (empty means valid)."""
    errors: list[str] = []
    module_dir = Path(module_dir)

    for key in ("name", "frame", "geometry", "joint", "motor",
                "input_mount", "output_mount", "mount_in", "mount_out",
                "size_mm", "bom"):
        if key not in module:
            errors.append(f"missing required key: {key}")

    if "geometry" in module:
        geometry = module_dir / module["geometry"]
        if not geometry.is_file():
            errors.append(f"geometry file missing: {geometry}")

    joint = module.get("joint")
    if joint is None:
        pass
    elif not isinstance(joint, dict):
        errors.append("joint must be an object")
    else:
        axis = _check_vec3(joint.get("axis"), "joint.axis", errors)
        if axis is not None:
            norm = math.sqrt(sum(v * v for v in axis))
            if norm <= 0:
                errors.append("joint.axis must be non-zero")
            elif abs(norm - 1.0) > 1e-6:
                errors.append("joint.axis must be a unit vector")
        _check_vec3(joint.get("point_mm"), "joint.point_mm", errors)
        span = joint.get("range_deg")
        if not isinstance(span, list) or len(span) != 2 \
                or not all(_is_num(v) for v in span):
            errors.append("joint.range_deg must be [lo, hi] numbers")
        elif not span[0] < span[1]:
            errors.append("joint.range_deg needs lo < hi")
        ratio = joint.get("joint_deg_per_motor_deg")
        if not _is_num(ratio) or ratio <= 0:
            errors.append("joint.joint_deg_per_motor_deg must be > 0")

    motor = module.get("motor")
    if motor is not None and (
            not isinstance(motor, dict)
            or motor.get("kind") not in MOTOR_KINDS):
        errors.append(f"motor.kind must be one of {MOTOR_KINDS}")

    sensor = module.get("sensor_mount")
    if sensor is not None:
        if not isinstance(sensor, dict) \
                or sensor.get("kind") not in SENSOR_KINDS:
            errors.append(f"sensor_mount.kind must be one of {SENSOR_KINDS}")
        else:
            _check_vec3(sensor.get("position_mm"),
                        "sensor_mount.position_mm", errors)

    if "input_mount" in module:
        _check_mount(module["input_mount"], "input_mount", errors)
    if "output_mount" in module:
        _check_mount(module["output_mount"], "output_mount", errors)

    blocks = {}
    if "mount_in" in module:
        blocks["mount_in"] = _check_mount_block(
            module["mount_in"], module.get("input_mount"), "mount_in",
            errors)
    if "mount_out" in module:
        blocks["mount_out"] = _check_mount_block(
            module["mount_out"], module.get("output_mount"), "mount_out",
            errors)
    if blocks and catalog is not None and "geometry" in module \
            and (module_dir / module["geometry"]).is_file():
        centres = _beam_centres_mm(module_dir, module, catalog)
        for what, block in blocks.items():
            if block is None:
                continue
            for i, row in enumerate(block["holes"]):
                want = [v * 0.4 for v in row["pos_ldu"]]
                best = None
                for centre, kind, pid in centres:
                    dist = sum((a - b) ** 2
                               for a, b in zip(centre, want)) ** 0.5
                    if best is None or dist < best[0]:
                        best = (dist, kind, pid, centre)
                if best is None or best[0] > MATCH_LDU * 0.4:
                    got = ("none" if best is None
                           else f"{best[3]} at {best[0] / 0.4:.2f} LDU")
                    errors.append(
                        f"{what}.holes[{i}] {row['pos_ldu']} matches no "
                        f"subfile beam hole (nearest: {got})")
                elif row["type"] == "axle" and best[1] != "axle":
                    errors.append(
                        f"{what}.holes[{i}] needs an axle bore, but "
                        f"{best[2]} has a {best[1]} hole there")

    size = module.get("size_mm")
    if size is not None and (
            not isinstance(size, list) or len(size) != 3
            or not all(_is_num(v) and v > 0 for v in size)):
        errors.append("size_mm must be 3 positive numbers")

    bom = module.get("bom")
    if not isinstance(bom, dict) or not bom:
        errors.append("bom must be a non-empty {part_id: count} object")
    else:
        for part_id, count in bom.items():
            if not isinstance(count, int) or isinstance(count, bool) \
                    or count <= 0:
                errors.append(f"bom[{part_id}] must be a positive int")
            elif catalog is not None and part_id not in catalog:
                errors.append(f"bom[{part_id}] is not in the catalog")

    if inventory is not None and isinstance(bom, dict) and bom:
        stocked = {pid: row.get("total", 0)
                   for pid, row in (inventory.get("parts") or {}).items()}
        for part_id, count in bom.items():
            if isinstance(count, int) and count > stocked.get(part_id, 0):
                errors.append(
                    f"bom[{part_id}]: need {count}, own "
                    f"{stocked.get(part_id, 0)}")
    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Validate a 5D module description.")
    parser.add_argument("module_json")
    parser.add_argument("--inventory", default="config/inventory.local.yaml")
    args = parser.parse_args(argv)

    module_path = Path(args.module_json)
    catalog = load_catalog_merged()
    inventory = None
    inventory_path = Path(args.inventory)
    if inventory_path.exists():
        with open(inventory_path, encoding="utf-8") as fh:
            inventory = yaml.safe_load(fh) or {}
    else:
        print(f"note: {args.inventory} missing, skipping stock check")
    errors = validate_module(load_module(module_path), module_path.parent,
                             catalog=catalog, inventory=inventory)
    if errors:
        print(f"{module_path}: INVALID")
        for error in errors:
            print(f"  - {error}")
        return 1
    print(f"{module_path}: valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
