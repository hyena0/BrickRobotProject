"""Live-agent roundtrip verdict for arm_4dof (needs a model API key).

Parses generated_robots/arm_4dof/robot.ldr, runs the Stage 2 Spark agent
(same tools, same agent as the main pipeline; no hints), then runs
the 5D roundtrip gates (count, axes, links, ratios incl. signs, IK)
and writes generated_robots/arm_4dof/roundtrip_live.json.

Fails loudly without META_API_KEY/MODEL_API_KEY (no silent heuristic
fallback: a live verdict needs the live agent).

Usage: META_API_KEY=... PYTHONPATH=. .venv/bin/python \\
           scripts/live_roundtrip_arm_4dof.py [--run-id ID] [--max-calls N]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src.agent import run as agent_run  # noqa: E402
from src.gen import roundtrip as rt  # noqa: E402
from src.gen.inventory import load_catalog_merged  # noqa: E402
from src.parse import parse_file  # noqa: E402

ROBOT = ROOT / "generated_robots" / "arm_4dof"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Live roundtrip for arm_4dof.")
    ap.add_argument("--run-id", default="stage5d-live")
    ap.add_argument("--max-calls", type=int, default=60)
    args = ap.parse_args(argv)
    if not (os.environ.get("META_API_KEY")
            or os.environ.get("MODEL_API_KEY")):
        print("error: live verdict needs META_API_KEY or MODEL_API_KEY",
              file=sys.stderr)
        return 2
    parts = parse_file(str(ROBOT / "robot.ldr"))
    catalog = load_catalog_merged()
    run_dir = ROOT / "runs" / args.run_id
    run_dir.mkdir(parents=True, exist_ok=True)
    (run_dir / "parts.json").write_text(
        json.dumps(parts, indent=1), encoding="utf-8")
    doc = agent_run.run_live(parts, catalog, args.run_id, args.max_calls)
    agent_run.write_outputs(doc, args.run_id)
    print(f"agent: joints={len(doc['joints'])} links={len(doc['links'])}")
    report = rt.run_roundtrip(ROBOT, infer_fn=lambda p, c: doc,
                              mode="live")
    (ROBOT / "roundtrip_live.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8")
    print(f"overall: {report['overall_pass']}")
    for key, gate in report["gates"].items():
        print(f"  {key}: {gate['pass']}")
    return 0 if report["overall_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
