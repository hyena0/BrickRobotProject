"""Link/axis overlay data for Blender (Stage 2.8).

Headless-safe: writes a JSON overlay (part uid -> link id, joint axes
as arrows) that `blender --python` or the Blender MCP session consumes
for colouring and rendering. Renders stay in `assets/private/renders/`.
"""

from __future__ import annotations

import argparse
import json
import os


def build_overlay(pred_path: str) -> dict:
    pred = json.load(open(pred_path, encoding="utf-8"))
    colours = {}
    for i, link in enumerate(pred.get("links", [])):
        for uid in link.get("members", []):
            colours[uid] = {"link": link["id"], "slot": i}
    arrows = [{"id": j["id"], "axis": j["axis"], "point_mm": j["point_mm"],
               "parent": j["parent"], "child": j["child"],
               "confidence": j.get("confidence"),
               "style": (j.get("status", "selected")
                         if j.get("status", "selected")
                         in ("selected", "dropped") else "rejected")}
              for j in pred.get("joints", [])]
    return {"colours": colours, "arrows": arrows,
            "base": pred.get("base"), "end_effector": pred.get("end_effector")}


def main() -> None:
    ap = argparse.ArgumentParser(description="Build Blender overlay JSON")
    ap.add_argument("--pred", required=True,
                    help="runs/<id>/links_joints.json")
    ap.add_argument("-o", "--output", required=True)
    args = ap.parse_args()
    overlay = build_overlay(args.pred)
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as fh:
        json.dump(overlay, fh)
    print(f"parts={len(overlay['colours'])} "
          f"arrows={len(overlay['arrows'])} -> {args.output}")


if __name__ == "__main__":
    main()
