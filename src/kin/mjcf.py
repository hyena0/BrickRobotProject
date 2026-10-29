"""Primitive-geometry MJCF generator (Stage 4.1).

Builds a MuJoCo model from an abstract kinematic model JSON
(docs/schema.md): nested bodies, one hinge per joint with the given
axis and limits, capsule links between consecutive joint points, and
a site marking the end effector. Geometry is visual only; the file
carries no part IDs or appearance data, so the inferred output is
public. The same generator builds the private ground-truth MJCF.
"""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET

from .model import chain_order, ee_point, load_model, norm

LINK_RADIUS_MM = 8.0
BASE_STUB_MM = 40.0
TOOL_STUB_MM = 30.0


def _fmt(nums) -> str:
    return " ".join(f"{v:.4f}" for v in nums)


def _sub(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: str(v) for k, v in
                                       attrs.items()})


def build_mjcf(doc: dict) -> str:
    """Generate an MJCF string from a validated model dict."""
    joints = chain_order(doc)
    ee = ee_point(doc)
    root = ET.Element("mujoco")
    _sub(root, "compiler", angle="degree")
    world = _sub(root, "worldbody")
    parent_el = world
    parent_pt = [0.0, 0.0, 0.0]
    last = joints[-1]
    for i, joint in enumerate(joints):
        axis = norm(joint["axis"])
        pt = [float(x) for x in joint["point_mm"]]
        rel = [pt[k] - parent_pt[k] for k in range(3)]
        body = _sub(parent_el, "body", name=joint["child"],
                    pos=_fmt(rel))
        lo, hi = joint["limits_deg"]
        _sub(body, "joint", name=joint["id"], type="hinge",
             axis=_fmt(axis), range=f"{lo} {hi}")
        if i < len(joints) - 1:
            nxt = [float(x) for x in joints[i + 1]["point_mm"]]
            seg = [nxt[k] - pt[k] for k in range(3)]
        else:
            seg = [ee[k] - pt[k] for k in range(3)]
            if math.sqrt(sum(x * x for x in seg)) < 1e-6:
                seg = [axis[k] * TOOL_STUB_MM for k in range(3)]
        _sub(body, "geom", type="capsule",
             fromto=_fmt([0, 0, 0] + seg), size=f"{LINK_RADIUS_MM}")
        if i == 0:
            base = _sub(world, "body", name="base_stub",
                        pos=_fmt(pt))
            _sub(base, "geom", type="capsule",
                 fromto=_fmt([0, 0, 0] +
                             [-axis[k] * BASE_STUB_MM for k in range(3)]),
                 size=f"{LINK_RADIUS_MM}")
        if joint["id"] == last["id"]:
            local = [ee[k] - pt[k] for k in range(3)]
            _sub(body, "site", name="ee", pos=_fmt(local))
        parent_el, parent_pt = body, pt
    ET.indent(root)
    return ET.tostring(root, encoding="unicode")


def main() -> None:
    ap = argparse.ArgumentParser(description="Generate primitive MJCF")
    ap.add_argument("--model", required=True, help="model JSON")
    ap.add_argument("--out", required=True, help="output XML path")
    args = ap.parse_args()
    xml = build_mjcf(load_model(args.model))
    with open(args.out, "w", encoding="utf-8") as f:
        f.write(xml)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
