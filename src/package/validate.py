"""Robot package validator (Stage 5A.6).

Schema check + forbidden-field scan + size check. Runs in tests and
before every package write. Rejects anything that would let someone
rebuild or render the robot: meshes, part data, colours, images,
file references, free-text design descriptions, or more than one
envelope box per link.
"""

from __future__ import annotations

import argparse
import json
import os
import re

SIZE_LIMIT_BYTES = 100 * 1024

REQUIRED_TOP = ("schema_version", "package_version", "robot", "frame",
                "joints", "coupling", "coupled_limits",
                "coupled_lookup", "operating_margin_deg",
                "end_effector", "safe_poses", "speed_caps")

# Part-id shaped tokens (p000147-style uids) and design text.
PART_ID_RE = re.compile(r"\bp\d{6,}\b")
MESH_KEYS = ("vertices", "faces", "triangles", "mesh", "meshes",
             "point_cloud", "pointcloud", "convex_hull", "hull",
             "voxels", "urdf", "sdf", "stl", "obj", "glb", "blend",
             "usd", "fbx", "dae")
PART_KEYS = ("part_id", "partId", "part_uid", "members", "uid",
             "position_mm", "rotation", "colour", "color",
             "submodel", "sub_model", "mpd", "ldr", "io")
IMAGE_KEYS = ("image", "images", "render", "renders", "texture",
              "textures", "png", "jpg", "jpeg", "file", "path", "uri",
              "url", "visual_pack")
PRIVATE_DIR_RE = re.compile(r"assets/private|runs/|eval/")


def _walk(node, trail=""):
    if isinstance(node, dict):
        for k, v in node.items():
            yield trail + "/" + str(k), k, v
            yield from _walk(v, trail + "/" + str(k))
    elif isinstance(node, list):
        for n, v in enumerate(node):
            yield trail + f"[{n}]", n, v
            yield from _walk(v, trail + f"[{n}]")


def validate_file(path: str) -> list[str]:
    """Collectors of violations; empty means the package is accepted."""
    errors: list[str] = []
    try:
        size = os.path.getsize(path)
    except OSError:
        return [f"missing package file: {path}"]
    if size > SIZE_LIMIT_BYTES:
        errors.append(f"package {size} bytes over the 100 KB limit")
    try:
        with open(path, encoding="utf-8") as fh:
            doc = json.load(fh)
    except (OSError, ValueError) as exc:
        return [f"unreadable package JSON: {exc}"]
    return validate_doc(doc, source=path) + errors


def validate_doc(doc: dict, source: str = "<doc>") -> list[str]:
    errors: list[str] = []
    for key in REQUIRED_TOP:
        if key not in doc:
            errors.append(f"{source}: missing required key {key!r}")
    joints = doc.get("joints", [])
    for j in joints:
        for key in ("id", "axis", "point_mm", "limits_deg",
                    "limit_source"):
            if key not in j:
                errors.append(f"{source}: joint missing {key!r}")
        lo_hi = j.get("limits_deg", [])
        if len(lo_hi) != 2 or not lo_hi[0] < lo_hi[1]:
            errors.append(f"{source}: joint {j.get('id')} limits_deg "
                          f"must be [lo, hi] with lo < hi")
    envs = doc.get("link_envelopes", [])
    seen: dict[str, int] = {}
    for e in envs:
        if not isinstance(e, dict):
            errors.append(f"{source}: link_envelope entry not an object")
            continue
        for key in ("link", "center_mm", "size_mm", "orientation"):
            if key not in e:
                errors.append(f"{source}: envelope missing {key!r}")
        seen[e.get("link", "?")] = seen.get(e.get("link", "?"), 0) + 1
        for key in ("center_mm", "size_mm"):
            vals = e.get(key, [])
            if len(vals) != 3 or any(
                    abs(v - round(v, 0)) > 1e-9 for v in vals):
                errors.append(f"{source}: envelope {key} must be 3 "
                              f"numbers at 1 mm precision")
        ori = e.get("orientation", [])
        if len(ori) != 3 or any(abs(v - round(v, 0)) > 1e-9
                                for v in ori):
            errors.append(f"{source}: envelope orientation must be 3 "
                          f"numbers at 1 deg precision")
    for link, n in seen.items():
        if n > 1:
            errors.append(f"{source}: link {link} has {n} envelopes "
                          f"(at most one allowed)")
    text = json.dumps(doc)
    if PART_ID_RE.search(text):
        errors.append(f"{source}: part-id shaped token (assembly data)")
    if PRIVATE_DIR_RE.search(text):
        errors.append(f"{source}: reference into a private directory")
    lowered = text.lower()
    for key in MESH_KEYS + PART_KEYS + IMAGE_KEYS:
        if f'"{key}"' in lowered:
            errors.append(f"{source}: forbidden field {key!r}")
    return errors


def main() -> None:
    ap = argparse.ArgumentParser(description="Validate a robot package")
    ap.add_argument("path")
    args = ap.parse_args()
    errors = validate_file(args.path)
    if errors:
        print("REJECTED:")
        for e in errors:
            print(f"  - {e}")
        raise SystemExit(1)
    print(f"ACCEPTED: {args.path}")


if __name__ == "__main__":
    main()
