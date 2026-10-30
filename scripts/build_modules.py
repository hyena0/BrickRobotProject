"""Build the assets/modules/ library from joint templates.

Allocates mount beams globally from owned stock (deterministic:
uniform per type when stock allows, mixed fill otherwise), then
emits ``M_base_yaw`` and ``M_pitch`` (module.json + module.ldr,
validated, stock-checked).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, ".")

import yaml

from src.gen import mounts as _mounts
from src.gen import templates as _templates
from src.gen.inventory import load_catalog_merged
from src.gen.meshbox import local_box_mm
from src.gen.templates import write_module_files

TYPES = (
    # (module dir, template, axis, H, ratio, motor, instances)
    ("M_base_yaw", "base_yaw", [0, 0, 1], 52.0, 1.0, "Medium", 1),
    ("M_pitch", "pitch", [0, 1, 0], 64.0, 0.6, "Large", 2),
)
MOUNTS_PER_TYPE = 4


def mount_candidates(catalog: dict,
                     inventory: dict | None) -> list[dict]:
    """Mount-eligible beams, deterministic preference order."""
    stock = {}
    if inventory is not None:
        stock = {pid: row.get("total", 0)
                 for pid, row in (inventory.get("parts") or {}).items()}
    cands = []
    for part_id in sorted(catalog):
        entry = catalog[part_id]
        if (entry or {}).get("category") != "beam":
            continue
        try:
            if _mounts.row_axis_converted(entry) != 1:
                continue
            if _mounts.hole_dir_converted(entry) != (0, 0, 1):
                continue
            _templates._pick_mount_centre(catalog, part_id)
        except ValueError:
            continue
        thick = True
        box = local_box_mm(part_id)
        if box is not None:
            thick = (box[1][2] - box[0][2]) >= 7.5
        centres = _mounts.hole_centres(entry)
        cands.append({"part_id": part_id, "thick": thick,
                      "holes": len(centres),
                      "stock": stock.get(part_id, 10 ** 9)})
    cands.sort(key=lambda c: (not c["thick"], c["holes"],
                              -c["stock"], c["part_id"]))
    return cands


def allocate(cands: list[dict], need: int, instances: int) -> str:
    """Mount beam part for one type (uniform; loud on shortage)."""
    for cand in cands:
        if cand["stock"] >= need * instances:
            cand["stock"] -= need * instances
            return cand["part_id"]
    raise ValueError("stock cannot cover uniform mount beams "
                     f"(need {need} x{instances}; mixed mounts are "
                     "not supported)")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Build 5D modules.")
    parser.add_argument("--out", default="assets/modules")
    parser.add_argument("--inventory",
                        default="config/inventory.local.yaml")
    args = parser.parse_args(argv)

    catalog = load_catalog_merged()
    inventory = None
    if Path(args.inventory).exists():
        with open(args.inventory, encoding="utf-8") as fh:
            inventory = yaml.safe_load(fh) or {}
    else:
        print(f"note: {args.inventory} missing, skipping stock check")
    cands = mount_candidates(catalog, inventory)
    print("mount candidates:",
          [(c["part_id"], c["stock"]) for c in cands[:6]])
    ident = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    for name, template, axis, height, ratio, motor, instances in TYPES:
        mount_beam = allocate(cands, MOUNTS_PER_TYPE, instances)
        builder = _templates.TEMPLATES[template]
        module, parts = builder(
            axis, {"position_mm": [0, 0, 0], "rotation": ident},
            {"position_mm": [0, 0, height], "rotation": ident},
            ratio, motor, catalog=catalog, mount_beam=mount_beam)
        module["name"] = name
        out = write_module_files(args.out, name, module, parts,
                                 catalog=catalog, inventory=inventory)
        print(f"wrote {out} (mounts {mount_beam} x{MOUNTS_PER_TYPE}, "
              f"x{instances} instance(s))")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
