"""Stage 5D.2: build-spec loader and validator.

A spec states the desired arm kinematics; the design agent (5D.3) maps it
onto library modules. Specs are our own design input and are committed.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

JOINT_KINDS = ("yaw", "pitch", "roll")


def load_spec(path: str | Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _is_num(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def validate_spec(spec: dict) -> list[str]:
    """Return a list of validation errors (empty means valid)."""
    errors: list[str] = []
    for key in ("joints", "link_lengths_mm", "ranges_deg", "gripper"):
        if key not in spec:
            errors.append(f"missing required key: {key}")
    if errors:
        return errors
    joints = spec["joints"]
    lengths = spec["link_lengths_mm"]
    ranges = spec["ranges_deg"]
    if not isinstance(joints, list) or not joints \
            or any(j not in JOINT_KINDS for j in joints):
        errors.append(f"joints must be a non-empty list of {JOINT_KINDS}")
        return errors
    n = len(joints)
    if not isinstance(lengths, list) or len(lengths) != n \
            or not all(_is_num(v) and v >= 0 for v in lengths):
        errors.append("link_lengths_mm must be n non-negative numbers")
    if not isinstance(ranges, list) or len(ranges) != n:
        errors.append("ranges_deg must be n [lo, hi] pairs")
    else:
        for i, span in enumerate(ranges):
            if not isinstance(span, list) or len(span) != 2 \
                    or not all(_is_num(v) for v in span):
                errors.append(f"ranges_deg[{i}] must be [lo, hi] numbers")
            elif not span[0] < span[1]:
                errors.append(f"ranges_deg[{i}] needs lo < hi")
    if not isinstance(spec["gripper"], bool):
        errors.append("gripper must be a boolean")
    return errors


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Validate a 5D spec.")
    parser.add_argument("spec_json")
    args = parser.parse_args(argv)
    errors = validate_spec(load_spec(args.spec_json))
    if errors:
        print(f"{args.spec_json}: INVALID")
        for error in errors:
            print(f"  - {error}")
        return 1
    print(f"{args.spec_json}: valid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
