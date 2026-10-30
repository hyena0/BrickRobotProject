"""Headless driver for the template combine panel (Stage 5D).

Runs the full panel flow with plain Python: combine templates ->
oracle roundtrip gates -> headless-Blender convert (.blend) ->
assembly docs. Run OUTSIDE the sandbox (Blender segfaults inside it):

    PYTHONPATH=. .venv/bin/python scripts/panel_combine.py \\
        --modules generated_robots/arm_4dof/modules \\
        --design base_yaw:32316.dat,pitch:32316.dat,pitch:6632.dat,wrist_roll:6632.dat \\
        --out generated_robots/panel_bot --name panel_bot

Exits 0 when assembly and roundtrip gates pass, 1 otherwise. The EV3
gate is advisory (warns, recorded in report.json). Use --no-render to
skip the Blender convert leg.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src.gen.inventory import load_catalog_merged  # noqa: E402
from src.gen.spec import load_spec  # noqa: E402
from src.render import combine_panel as panel  # noqa: E402


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Template combine flow.")
    ap.add_argument("--modules", required=True)
    ap.add_argument("--design", required=True,
                    help="module:beam,module:beam,...")
    ap.add_argument("--spec", default="specs/arm_4dof.json")
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="panel_bot")
    ap.add_argument("--inventory", default="config/inventory.local.yaml")
    ap.add_argument("--render", dest="render", action="store_true",
                    default=True)
    ap.add_argument("--no-render", dest="render", action="store_false")
    ap.add_argument("--blender-bin", default=None)
    ap.add_argument("--studio", default=None)
    args = ap.parse_args(argv)
    picks = []
    for entry in args.design.split(","):
        module, _, beam = entry.partition(":")
        module, beam = module.strip(), beam.strip()
        if not module or not beam:
            print(f"bad design entry {entry!r} (want module:beam,...)")
            return 2
        picks.append({"module": module, "beam": beam})
    import yaml

    catalog = load_catalog_merged()
    inv_path = Path(args.inventory)
    inventory = yaml.safe_load(inv_path.read_text(
        encoding="utf-8")) if inv_path.exists() else None
    if inventory is None:
        print(f"note: {args.inventory} missing, stock unknown")
    spec = load_spec(args.spec)
    try:
        out = panel.run_flow(args.modules, picks, spec, args.out,
                             args.name, catalog, inventory,
                             render=args.render,
                             blender_bin=args.blender_bin,
                             studio_ldraw=args.studio)
    except (ValueError, RuntimeError, OSError) as exc:
        print(f"panel flow failed: {exc}")
        return 1
    print("artifacts:")
    for key, path in out["paths"].items():
        print(f"  {key}: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
