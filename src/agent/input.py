"""Agent input summary for Stage 2.2.

Contains only: part counts by category, motor/sensor positions, and the
overall bounding box. Sub-model names, colours-as-hints, and anything
from `eval/ground_truth/` are never included.
"""

from __future__ import annotations

import json

from src.tools import geometry as geom


def build_summary(parts: list[dict], catalog: dict,
                  max_motors_listed: int = 12) -> dict:
    counts: dict[str, int] = {}
    for part in parts:
        cat = (catalog.get(part["part_id"]) or {}).get("category")
        counts[cat or "unknown"] = counts.get(cat or "unknown", 0) + 1
    motors = geom.get_parts(parts, catalog, category="motor")
    sensors = geom.get_parts(parts, catalog, category="sensor")
    listed = [
        {"uid": m["uid"], "part_id": m["part_id"],
         "position_mm": m["position_mm"]}
        for m in (motors + sensors)[:max_motors_listed]
    ]
    xs = [p["position_mm"][0] for p in parts]
    ys = [p["position_mm"][1] for p in parts]
    zs = [p["position_mm"][2] for p in parts]
    return {
        "part_count": len(parts),
        "counts_by_category": counts,
        "motor_count": len(motors),
        "sensor_count": len(sensors),
        "motors_sensors": listed,
        "bbox_min_mm": [min(xs), min(ys), min(zs)],
        "bbox_max_mm": [max(xs), max(ys), max(zs)],
    }


def summary_text(summary: dict) -> str:
    return json.dumps(summary, indent=2)
