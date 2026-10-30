"""Stage 5D.0: owned-parts inventory builder.

Reads the parsed MR JOS part list (private stage 1 output) and merges it
with hand-counted surplus stock into ``config/inventory.local.yaml``.

The output contains MR JOS part counts, so it stays a local-only file and
is never committed (``config/*.local.yaml`` is git-ignored).
"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
PARTS_YAML = REPO_ROOT / "src" / "catalog" / "parts.yaml"
PARTS_5D_YAML = REPO_ROOT / "src" / "catalog" / "parts_5d.yaml"

LARGE_GEAR_TEETH = 24  # gears with at least this many teeth are precious


def load_catalog_merged(parts_yaml: str | Path = PARTS_YAML,
                        parts_5d_yaml: str | Path = PARTS_5D_YAML) -> dict:
    """Load the part catalog with the 5D overlay applied on top."""
    with open(parts_yaml, encoding="utf-8") as fh:
        catalog = yaml.safe_load(fh) or {}
    overlay_path = Path(parts_5d_yaml)
    if overlay_path.exists():
        with open(overlay_path, encoding="utf-8") as fh:
            overlay = yaml.safe_load(fh) or {}
        catalog = {**catalog, **overlay}
    return catalog


def short_label(part_id: str, entry: dict | None) -> str:
    """Human-readable label from the catalog evidence title."""
    evidence = (entry or {}).get("evidence") or ""
    title = evidence.split(";")[0].strip()
    if title.lower().startswith("title:"):
        title = title[len("title:"):].strip()
    return title or part_id


def sensor_kind(label: str) -> str:
    low = label.lower()
    for kind in ("touch", "color", "ultrasonic", "gyro", "infrared"):
        if kind in low:
            return kind
    return "other"


def is_precious(category: str, entry: dict, label: str) -> bool:
    """Precious parts (motors, sensors, brick, turntables, large gears)."""
    if category in ("motor", "sensor", "turntable"):
        return True
    if category == "gear" and (entry.get("teeth") or 0) >= LARGE_GEAR_TEETH:
        return True
    if category == "brick" and "ev3" in label.lower():
        return True
    return False


def count_parts_list(parts: list[dict]) -> Counter:
    return Counter(p["part_id"] for p in parts)


def load_surplus(path: str | Path) -> tuple[dict, list]:
    """Load hand-counted surplus: ({part_id: count}, [extra entries])."""
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    surplus = data.get("parts") or {}
    extra = data.get("_extra") or []
    for part_id, count in surplus.items():
        if not isinstance(count, int) or isinstance(count, bool) \
                or count < 0:
            raise ValueError(
                f"surplus count for {part_id} must be a non-negative int, "
                f"got {count!r}")
    for item in extra:
        for key in ("part_id", "label", "category", "count"):
            if key not in item:
                raise ValueError(f"surplus _extra entry misses {key}: {item!r}")
        if not isinstance(item["count"], int) or item["count"] < 0:
            raise ValueError(f"surplus _extra count invalid: {item!r}")
    return surplus, extra


def build_inventory(counts: dict, catalog: dict,
                    surplus: dict | None = None,
                    extra: list | None = None,
                    source: str = "") -> dict:
    """Merge robot part counts with surplus stock into an inventory dict."""
    surplus = surplus or {}
    extra = extra or []
    unknown = [pid for pid in surplus
               if pid not in catalog and pid not in counts]
    if unknown:
        raise ValueError(
            f"surplus has part ids outside the catalog: {sorted(unknown)}; "
            "add them under _extra with label/category instead")

    parts: dict[str, dict] = {}
    for part_id in sorted(set(counts) | set(surplus)):
        entry = catalog.get(part_id) or {}
        category = entry.get("category", "uncatalogued")
        label = short_label(part_id, entry)
        n_robot = counts.get(part_id, 0)
        n_surplus = surplus.get(part_id, 0)
        parts[part_id] = {
            "label": label,
            "category": category,
            "mrjos": n_robot,
            "surplus": n_surplus,
            "total": n_robot + n_surplus,
            "precious": is_precious(category, entry, label),
        }
    for item in extra:
        part_id = item["part_id"]
        if part_id in parts:
            raise ValueError(f"_extra duplicates {part_id}; use parts instead")
        parts[part_id] = {
            "label": item["label"],
            "category": item["category"],
            "mrjos": 0,
            "surplus": item["count"],
            "total": item["count"],
            "precious": is_precious(item["category"], {}, item["label"]),
        }

    summary: dict = {
        "motors": {},
        "sensors": {},
        "turntables": 0,
        "gears_by_teeth": {},
        "beams": {},
        "axles_by_length": {},
        "pins": {},
        "other": {},
    }
    for part_id, row in parts.items():
        total = row["total"]
        if total == 0:
            continue
        entry = catalog.get(part_id) or {}
        category = row["category"]
        if category == "motor":
            key = entry.get("motor") or "unknown"
            summary["motors"][key] = summary["motors"].get(key, 0) + total
        elif category == "sensor":
            key = sensor_kind(row["label"])
            summary["sensors"][key] = summary["sensors"].get(key, 0) + total
        elif category == "turntable":
            summary["turntables"] += total
        elif category == "gear":
            key = entry.get("teeth") or 0
            summary["gears_by_teeth"][key] = \
                summary["gears_by_teeth"].get(key, 0) + total
        elif category == "beam":
            summary["beams"][part_id] = total
        elif category == "axle":
            key = entry.get("axle_length")
            key = key if key is not None else part_id
            summary["axles_by_length"][key] = \
                summary["axles_by_length"].get(key, 0) + total
        elif category == "pin":
            summary["pins"][part_id] = total
        else:
            summary["other"][part_id] = total

    return {
        "source": source,
        "n_robot_parts": sum(counts.values()),
        "n_kinds": len(parts),
        "parts": parts,
        "precious": sorted(pid for pid, row in parts.items()
                           if row["precious"] and row["total"] > 0),
        "summary": summary,
    }


def write_surplus_template(path: str | Path, counts: dict,
                           catalog: dict) -> None:
    """Write a hand-count template grouped by category (all zeros)."""
    by_category: dict[str, list[str]] = {}
    for part_id in sorted(set(counts) | set(catalog)):
        entry = catalog.get(part_id) or {}
        by_category.setdefault(entry.get("category", "uncatalogued"),
                               []).append(part_id)
    lines = [
        "# Hand-counted surplus stock: parts NOT in the MR JOS robot.",
        "# Fill in counts keyed by LDraw part id, then rebuild with:",
        "#   PYTHONPATH=. .venv/bin/python -m src.gen.inventory \\",
        "#       --parts runs/stage1/parts.json \\",
        "#       --surplus config/surplus.local.yaml \\",
        "#       --out config/inventory.local.yaml",
        "# Parts missing from the catalog (EV3 brick, cables, ...) go",
        "# under `_extra` with an explicit label and category.",
        "parts:",
    ]
    for category in sorted(by_category):
        lines.append(f"  # --- {category} ---")
        for part_id in by_category[category]:
            label = short_label(part_id, catalog.get(part_id))
            lines.append(f"  {part_id}: 0  # {label} "
                         f"(robot has {counts.get(part_id, 0)})")
    lines += ["_extra: []",
              "# - {part_id: ..., label: ..., category: ..., count: 0}"]
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Build config/inventory.local.yaml (private).")
    parser.add_argument("--parts", default="runs/stage1/parts.json")
    parser.add_argument("--surplus", default="config/surplus.local.yaml")
    parser.add_argument("--out", default="config/inventory.local.yaml")
    parser.add_argument("--write-surplus-template", default=None,
                        help="write a hand-count template and exit")
    args = parser.parse_args(argv)

    catalog = load_catalog_merged()
    with open(args.parts, encoding="utf-8") as fh:
        counts = count_parts_list(json.load(fh))

    if args.write_surplus_template:
        write_surplus_template(args.write_surplus_template, counts, catalog)
        print(f"wrote {args.write_surplus_template}")
        return 0

    surplus_path = Path(args.surplus)
    if surplus_path.exists():
        surplus, extra = load_surplus(surplus_path)
    else:
        surplus, extra = {}, []
        print(f"note: {args.surplus} missing, assuming zero surplus")
    inventory = build_inventory(counts, catalog, surplus, extra,
                                source=args.parts)
    header = ("# PRIVATE - never commit. Regenerate with "
              "`python -m src.gen.inventory` (see module docstring).\n")
    Path(args.out).write_text(
        header + yaml.safe_dump(inventory, sort_keys=False,
                                allow_unicode=True),
        encoding="utf-8")
    n_precious = len(inventory["precious"])
    print(f"wrote {args.out}: {inventory['n_robot_parts']} robot parts, "
          f"{inventory['n_kinds']} kinds, {n_precious} precious kinds")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
