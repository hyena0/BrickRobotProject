"""Generate a draft `parts.yaml` covering every part type in parts.json.

Every entry starts as `category: other` with `review: true`: a human
must confirm category, teeth, axes and holes against the LDraw part
(Stage 1.2). Beams get generated hole rows from their length once the
human confirms the part is a beam.
"""

from __future__ import annotations

import argparse
import json

TEMPLATE = """\
# Part catalog (Stage 1.2). One entry per part type in the model.
# `review: true` means a human has NOT yet confirmed this entry.
# Fields: category, teeth, gear_kind, axis_local, holes, axle_length, motor.
"""


def draft(parts_path: str, out_path: str) -> tuple[int, int]:
    with open(parts_path, encoding="utf-8") as fh:
        parts = json.load(fh)
    counts: dict[str, int] = {}
    for part in parts:
        pid = part["part_id"]
        counts[pid] = counts.get(pid, 0) + 1
    lines = [TEMPLATE]
    for pid in sorted(counts):
        lines.append(f'"{pid}":')
        lines.append("  category: other  # gear|axle|pin|beam|frame|... ")
        lines.append("  teeth: null")
        lines.append("  gear_kind: null")
        lines.append("  axis_local: null  # e.g. [0, 0, 1]")
        lines.append("  holes: []")
        lines.append("  axle_length: null  # studs, for axles")
        lines.append("  motor: null  # large|medium")
        lines.append("  review: true")
    with open(out_path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")
    return len(counts), sum(counts.values())


def main() -> None:
    ap = argparse.ArgumentParser(description="Draft parts.yaml from parts.json")
    ap.add_argument("parts_json", help="parser output")
    ap.add_argument("-o", "--output", required=True, help="output parts.yaml")
    args = ap.parse_args()
    n_types, n_parts = draft(args.parts_json, args.output)
    print(f"types={n_types} parts={n_parts} -> {args.output}")


if __name__ == "__main__":
    main()
