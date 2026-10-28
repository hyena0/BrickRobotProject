"""Stage 2 joint re-identification runner (Stage 2.9).

Usage:
    .venv/bin/python -m src.agent.joints_run --parts runs/stage1/parts.json \\
        --catalog src/catalog/parts.yaml --stage3 runs/stage3/transmission.json \\
        --run-id stage2-joints

Motor path terminals come from the Stage 3 transmission report
(motor_paths chains); ground truth is never read.
"""

from __future__ import annotations

import argparse
import json
import os

import numpy as np

from src.agent import joints_spark
from src.agent.schema import validate_links_joints


def _load_json(path: str):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _terminals(stage3: dict, parts: list[dict]) -> dict[str, list]:
    """Motor uid -> last-gear position of its longest drive chain."""
    by_uid = {p["uid"]: p for p in parts}
    out = {}
    for motor, joints in (stage3.get("report") or {}).get(
            "motor_paths", {}).items():
        best = None
        for _jid, chains in (joints or {}).items():
            if chains and (best is None or len(chains) > len(best)):
                best = chains
        if best:
            triple = best[-1]
            last = triple[1] if isinstance(triple, list) else triple
            if last in by_uid:
                out[motor] = list(by_uid[last]["position_mm"])
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Stage 2 joint re-id")
    ap.add_argument("--parts", required=True)
    ap.add_argument("--catalog", required=True)
    ap.add_argument("--stage3", required=True)
    ap.add_argument("--run-id", required=True)
    args = ap.parse_args()

    from src.tools import geometry as geom
    parts = _load_json(args.parts)
    catalog = geom.load_catalog(args.catalog)
    stage3 = _load_json(args.stage3)
    terminals = _terminals(stage3, parts)

    from openai import OpenAI
    client = OpenAI(
        base_url="https://api.meta.ai/v1",
        api_key=os.environ.get("META_API_KEY")
        or os.environ.get("MODEL_API_KEY"))
    result = joints_spark.run_selection(client, parts, catalog, terminals)
    doc = result["doc"]
    errors = validate_links_joints(doc)
    out_dir = os.path.join("runs", args.run_id)
    if os.path.exists(os.path.join(out_dir, "links_joints.json")):
        raise SystemExit(
            f"refusing to overwrite {out_dir}/links_joints.json; "
            f"use a new --run-id")
    os.makedirs(out_dir, exist_ok=True)
    with open(os.path.join(out_dir, "links_joints.json"), "w",
              encoding="utf-8") as fh:
        json.dump(doc, fh, indent=2)
    for snap in result.get("attempt_docs", []):
        with open(os.path.join(
                out_dir, f"links_joints_attempt{snap['attempt']}.json"),
                "w", encoding="utf-8") as fh:
            json.dump(snap["doc"], fh, indent=2)
    print("counts: " + ", ".join(
        f"{k}={v}" for k, v in result["counts"].items()))
    print(f"joints={len([j for j in doc['joints'] if j.get('status') == 'selected'])} "
          f"dropped={len([j for j in doc['joints'] if j.get('status') == 'dropped'])} "
          f"attempts={result['attempts']}")
    if result["errors"]:
        print("INCONSISTENT:")
        for r in result["errors"]:
            print(f"  - {r}")
    else:
        print("consistency: OK")
    if result["gaps"]:
        print("drive gaps (recorded, not retried):")
        for r in result["gaps"]:
            print(f"  - {r}")
    if errors:
        print("schema errors:", errors[:5])


if __name__ == "__main__":
    main()
