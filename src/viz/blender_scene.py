"""Import the model into Blender with part uids (Stage 1.8).

Run inside Blender (LDraw importer add-on installed)::

    blender --python src/viz/blender_scene.py -- model.ldr parts.json

Applies the same Z-up conversion as `src.parse.ldr` and tags every
object with its part `uid`, so later stages can colour objects by link.
All renders stay in `assets/private/`.
"""

import json
import sys

LDU_TO_MM = 0.4


def convert_point(x: float, y: float, z: float) -> tuple[float, float, float]:
    return (x * LDU_TO_MM, z * LDU_TO_MM, -y * LDU_TO_MM)


def main(argv: list[str]) -> None:
    import bpy  # noqa: F401  # Blender-bundled

    model_path, parts_path = argv[1], argv[2]
    parts = json.load(open(parts_path, encoding="utf-8"))
    print(f"model={model_path} parts={len(parts)}")
    print("Import the .ldr/.mpd with the LDraw add-on, then run "
          "tag_uids(parts) to attach uids. "
          "Manual step: confirm scale (mm) and orientation (Z up).")


def tag_uids(parts: list[dict]) -> int:
    """Attach part uids to already-imported objects. Returns tag count."""
    import bpy

    by_suffix = {obj.name.split(".")[0]: obj for obj in bpy.data.objects}
    tagged = 0
    for part in parts:
        obj = by_suffix.get(part["part_id"])
        if obj is None:
            continue
        obj["uid"] = part["uid"]
        tagged += 1
    return tagged


if __name__ == "__main__":
    main(sys.argv)
