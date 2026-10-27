"""Parse LDraw type-1 lines into parts with mm Z-up poses.

Input: Studio-exported `.ldr`/`.mpd` (LDU units, `-Y` up).
Output: list of dicts with `uid`, `part_id`, `colour`,
`position_mm` and `rotation` (3x3, right-handed Z-up frame).

Coordinate conversion (documented in `docs/schema.md`):
LDraw `(x, y, z)` maps to `(x, z, -y)`, scaled by 0.4 mm/LDU.
Up in LDraw is `-Y`, which becomes `+Z`. The map is a proper
rotation, so handedness is preserved.
"""

from __future__ import annotations

import json

LDU_TO_MM = 0.4


def _convert_point(x: float, y: float, z: float) -> list[float]:
    return [x * LDU_TO_MM, z * LDU_TO_MM, -y * LDU_TO_MM]


def _convert_matrix(a: list[float]) -> list[list[float]]:
    # True world rotation R = C @ L @ C^-1 with C the point remap
    # (x, y, z) -> (x, z, -y). With remap(r) = (r0, r2, -r1):
    # R rows are remap(L row 0), remap(L row 2), -remap(L row 1).
    # A row-only remap (R = L @ S) is wrong for rotated parts: it
    # maps converted-local vectors back into the LDraw frame.
    it = iter(a)
    rows = [[next(it), next(it), next(it)] for _ in range(3)]
    remapped = [[r[0], r[2], -r[1]] for r in rows]
    return [remapped[0], remapped[2],
            [-v for v in remapped[1]]]


def parse_type1(line: str, uid: str) -> dict | None:
    """Parse one type-1 line. Returns None for non-type-1 lines."""
    tokens = line.split()
    if len(tokens) != 15 or tokens[0] != "1":
        return None
    try:
        nums = [float(t) for t in tokens[1:14]]
    except ValueError:
        return None
    colour = tokens[1] if "." not in tokens[1] and "e" not in tokens[1].lower() \
        else str(int(nums[0]))
    x, y, z = nums[1], nums[2], nums[3]
    return {
        "uid": uid,
        "part_id": tokens[14].lower(),
        "colour": colour,
        "position_mm": _convert_point(x, y, z),
        "rotation": _convert_matrix(nums[4:13]),
    }


def _split_submodels(lines: list[str]) -> dict[str, list[str]]:
    """Split MPD content into named blocks. Plain `.ldr` is one block."""
    blocks: dict[str, list[str]] = {}
    current: list[str] = []
    name = "main"
    for line in lines:
        s = line.strip()
        if not s:
            continue
        if s.startswith("0 FILE"):
            if current:
                blocks.setdefault(name, []).extend(current)
                current = []
            name = s[len("0 FILE"):].strip().lower() or "main"
        elif s == "0 NOFILE":
            blocks.setdefault(name, []).extend(current)
            current = []
            name = "main"
        else:
            current.append(line)
    if current:
        blocks.setdefault(name, []).extend(current)
    return blocks


def _compose(parent: dict, child: dict) -> dict:
    """Compose a child part pose into the parent sub-model frame."""
    r1, r2 = parent["rotation"], child["rotation"]
    rot = [[sum(r1[i][k] * r2[k][j] for k in range(3))
            for j in range(3)] for i in range(3)]
    p1, p2 = parent["position_mm"], child["position_mm"]
    pos = [p1[i] + sum(r1[i][k] * p2[k] for k in range(3))
           for i in range(3)]
    return {**child, "position_mm": pos, "rotation": rot}


def parse_text(text: str) -> list[dict]:
    """Parse LDraw/MPD text. Sub-model FILE names never enter output."""
    lines = text.replace("\r\n", "\n").replace("\r", "\n").split("\n")
    blocks = _split_submodels(lines)
    parts: list[dict] = []
    counter = 0

    def emit(block: list[str], stack: list[str]) -> None:
        nonlocal counter
        for line in block:
            s = line.strip()
            if not s.startswith("1 "):
                continue
            tokens = s.split()
            if len(tokens) != 15:
                continue
            ref = tokens[14].lower()
            if ref in blocks and ref not in stack:
                # Sub-model reference: recurse with composed transform.
                counter += 1
                holder = parse_type1(s, f"p{counter:06d}")
                if holder is None:
                    continue
                before = len(parts)
                emit(blocks[ref], stack + [ref])
                for part in parts[before:]:
                    composed = _compose(holder, part)
                    part.update(composed)
            else:
                counter += 1
                part = parse_type1(s, f"p{counter:06d}")
                if part is not None:
                    parts.append(part)

    main = blocks.get("main", [])
    # A bare model file whose only block is the file itself: fall back to
    # the block holding the most type-1 lines.
    if not any(line.strip().startswith("1 ") for line in main):
        main = max(blocks.values(),
                   key=lambda b: sum(l.strip().startswith("1 ") for l in b))
    emit(main, [])
    # Renumber uids sequentially (recursion consumed placeholder numbers).
    for i, part in enumerate(parts, 1):
        part["uid"] = f"p{i:06d}"
    return parts


def parse_file(path: str) -> list[dict]:
    with open(path, encoding="utf-8-sig") as fh:
        return parse_text(fh.read())


def main() -> None:
    import argparse

    ap = argparse.ArgumentParser(description="Parse LDraw to parts.json")
    ap.add_argument("input", help="input .ldr/.mpd file")
    ap.add_argument("-o", "--output", required=True, help="output parts.json")
    args = ap.parse_args()
    parts = parse_file(args.input)
    with open(args.output, "w", encoding="utf-8") as fh:
        json.dump(parts, fh)
    kinds: dict[str, int] = {}
    for part in parts:
        kinds[part["part_id"]] = kinds.get(part["part_id"], 0) + 1
    print(f"parts={len(parts)} distinct_types={len(kinds)} -> {args.output}")


if __name__ == "__main__":
    main()
