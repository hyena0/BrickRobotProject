"""Brick-local config loader (Stage 5B).

The brick reads the hub address from a local YAML file that is never
committed (see ``config/brick.local.yaml.example``). The broker takes
anonymous clients, so there is no account or password. Moving the hub
changes only ``hub_address``; topics, payloads and brick code stay the
same. The brick knows ports only.
"""

from __future__ import annotations

from pathlib import Path

DEFAULT_PATH = "config/brick.local.yaml"


def load_brick_config(path: str = DEFAULT_PATH) -> dict:
    """Load and validate the brick-local config.

    Returns a dict with ``brick``, ``hub_address``, ``hub_port``,
    ``motors``, ``sensors`` and ``master_bt_addr``. Raises
    ``FileNotFoundError`` / ``KeyError`` / ``ValueError`` with a clear
    message when the file is missing or incomplete.
    """
    import yaml  # type: ignore

    cfg_path = Path(path)
    if not cfg_path.exists():
        raise FileNotFoundError(
            f"brick config not found: {path} "
            "(copy config/brick.local.yaml.example and edit it)")
    with open(cfg_path, encoding="utf-8") as fh:
        cfg = yaml.safe_load(fh) or {}
    brick = str(cfg.get("brick", "left"))
    if brick not in ("left", "right"):
        raise ValueError(f"brick must be 'left' or 'right', got {brick!r}")
    hub_address = cfg.get("hub_address") or cfg.get("broker")
    if not hub_address:
        raise KeyError("brick config needs `hub_address`")
    return {
        "brick": brick,
        "hub_address": str(hub_address),
        "hub_port": int(cfg.get("hub_port", cfg.get("broker_port", 1883))),
        "motors": list(cfg.get("motors", [])),
        "sensors": list(cfg.get("sensors", [])),
        "master_bt_addr": str(cfg.get("master_bt_addr", "")),
    }


def build_core_from_wiring(brick: str, wiring: dict):
    """Create a :class:`BrickCore` from ports-only wiring lists."""
    from .core import BrickCore
    from .motors import open_motor
    from .sensors import open_sensor

    motors = {m["port"]: open_motor(m["port"], m.get("type", "medium"))
              for m in wiring.get("motors", [])}
    sensors = {s["port"]: open_sensor(s["port"], s.get("type", "touch"))
               for s in wiring.get("sensors", [])}
    return BrickCore(
        brick, motors, sensors,
        {m["port"]: m.get("type", "medium")
         for m in wiring.get("motors", [])},
        {s["port"]: s.get("type", "touch")
         for s in wiring.get("sensors", [])})
