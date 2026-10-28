"""Tier-1 chain joint runner (Stage 2.10, deterministic).

Usage:
    .venv/bin/python -m src.agent.chain_run --parts runs/stage1/parts.json \\
        --catalog src/catalog/parts.yaml \\
        --div runs/stage2-div/links_joints.json --run-id stage2-chain

No ground truth, no model calls. Refuses to overwrite an existing run.
"""

from __future__ import annotations

import argparse
import json
import os

from src.agent.schema import validate_links_joints
from src.catalog.review import Library
from src.tools import chain_joints as chain
from src.tools import geometry as geom

STUDIO_LDRAW = "/Applications/Studio 2.0/ldraw"


def main() -> None:
    ap = argparse.ArgumentParser(description="Tier-1 chain joints")
    ap.add_argument("--parts", required=True)
    ap.add_argument("--catalog", required=True)
    ap.add_argument("--div", required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--io", default="3D models/Robot_6axis_integration_Comb.io")
    ap.add_argument("--no-lib", action="store_true",
                    help="legacy catalog-axis path (flags fallback)")
    args = ap.parse_args()

    out_dir = os.path.join("runs", args.run_id)
    if os.path.exists(os.path.join(out_dir, "links_joints.json")):
        raise SystemExit(
            f"refusing to overwrite {out_dir}/links_joints.json; "
            f"use a new --run-id")
    with open(args.parts, encoding="utf-8") as fh:
        parts = json.load(fh)
    catalog = geom.load_catalog(args.catalog)
    lib = None
    if not args.no_lib:
        # Primary geometry from the Studio ldraw/ tree; the .io
        # embedded snapshot only fills custom types Studio lacks
        # (never the reverse), mirroring src/viz/build_blend.py.
        lib = Library.from_dir(STUDIO_LDRAW,
                               sorted({p["part_id"] for p in parts}))
        lib.merge_missing(Library.from_io(args.io))
    doc = chain.infer_chain(args.div, parts, catalog, lib)
    errors = validate_links_joints(doc)
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "links_joints.json"), "w",
              encoding="utf-8") as fh:
        json.dump(doc, fh, indent=2)
    print(f"joints={len(doc['joints'])} "
          f"suspects={len(doc['boundary_suspects'])} "
          f"deficits={len(doc['deficits'])} "
          f"div_issues={len(doc['div_issues'])}")
    for j in doc["joints"]:
        ver = j.get("verify", {})
        print(f"  {j['id']} {j['parent']}->{j['child']} "
              f"pt={j['point_mm']} ax={j['axis']} "
              f"anchor={j.get('anchor_kind')} "
              f"verify={ver.get('verdict')} "
              f"worst={ver.get('worst_lateral_mm')}")
    for s in doc["boundary_suspects"]:
        print(f"  SUSPECT {s.get('joint', s.get('pair'))}: "
              f"[{s.get('class')}] {s.get('note')}")
    for d in doc["deficits"]:
        print(f"  DEFICIT {d.get('joint')}: {d.get('note')}")
    if errors:
        print("schema errors:", errors[:5])


if __name__ == "__main__":
    main()
