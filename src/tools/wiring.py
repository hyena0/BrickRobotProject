"""Hardware-layer wiring check (Stage 1.7).

Compares `config/ports.local.yaml` against `parts.json` using the
catalog: every motor/sensor part in the model needs exactly one port,
and every port from Stage 0 `info` needs exactly one part. No joint
names or sensor meanings are allowed in the hardware layer.
"""

from __future__ import annotations

import argparse
import json

import yaml

FORBIDDEN_KEYS = {"joint", "name", "meaning", "homes", "drives"}


def check(parts_path: str, catalog_path: str,
          ports_path: str) -> list[str]:
    errors: list[str] = []
    parts = json.load(open(parts_path, encoding="utf-8"))
    catalog = yaml.safe_load(open(catalog_path, encoding="utf-8")) or {}
    ports = yaml.safe_load(open(ports_path, encoding="utf-8")) or {}
    model_ms = [p["uid"] for p in parts
                if (catalog.get(p["part_id"]) or {}).get("category")
                in ("motor", "sensor")]
    wired = [e.get("uid") for e in (ports.get("motors") or [])
             + (ports.get("sensors") or [])]
    for uid in model_ms:
        if wired.count(uid) != 1:
            errors.append(f"model part {uid} wired {wired.count(uid)}x")
    for uid in wired:
        if uid not in {p["uid"] for p in parts}:
            errors.append(f"wired uid {uid} not in model")
    for section in ("motors", "sensors"):
        for entry in ports.get(section) or []:
            if not isinstance(entry, dict):
                errors.append(f"{section} entry is not a mapping: {entry!r}")
                continue
            for key in FORBIDDEN_KEYS:
                if key in {k.lower() for k in entry}:
                    errors.append(f"semantic key '{key}' forbidden "
                                  f"in hardware layer ({section})")
    if not model_ms:
        errors.append("no motor/sensor in catalog: review pending "
                      "(Stage 1.2), wiring cannot be completed yet")
    return errors


def main() -> None:
    ap = argparse.ArgumentParser(description="Check hardware wiring layer")
    ap.add_argument("--parts", required=True)
    ap.add_argument("--catalog", required=True)
    ap.add_argument("--ports", required=True)
    args = ap.parse_args()
    errors = check(args.parts, args.catalog, args.ports)
    if errors:
        print("WIRING ISSUES:")
        for err in errors:
            print(f"  - {err}")
        raise SystemExit(1)
    print("wiring OK")


if __name__ == "__main__":
    main()
