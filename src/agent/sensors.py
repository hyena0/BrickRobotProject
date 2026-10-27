"""Sensor-to-joint association (Stage 2.7).

For each wired sensor, the agent infers which joint it homes: the link
the sensor is mounted on, and which joint's motion brings a moving
part into contact (touch) or into view (colour).

Homing *direction* needs joint kinematics (Stage 4): until then it is
reported as "unknown". Output per sensor: joint id, homing direction,
confidence. Evaluated against `wiring_semantic.yaml` (verifier only).
"""

from __future__ import annotations

import argparse
import json
import math
import os

import yaml


def load_ports(path: str) -> dict:
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _dist(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def associate(sensors: list[dict], parts: list[dict],
              links: list[dict], joints: list[dict]) -> list[dict]:
    """Map each wired sensor to its homing joint.

    Mounting link: the link containing the sensor uid. Candidate
    joint: the nearest joint (by axis-point distance) whose parent
    or child link differs from the mounting link, i.e. motion across
    the joint moves parts relative to the sensor.
    """
    by_uid = {p["uid"]: p for p in parts}
    link_of = {}
    for link in links:
        for uid in link.get("members", []):
            link_of[uid] = link["id"]
    out = []
    for sensor in sensors:
        uid = sensor["uid"]
        mount = link_of.get(uid)
        spos = by_uid[uid]["position_mm"] if uid in by_uid else None
        best = None
        if spos is not None:
            for joint in joints:
                if mount is not None and mount not in (joint["parent"],
                                                       joint["child"]):
                    continue
                d = _dist(spos, joint["point_mm"])
                if best is None or d < best[0]:
                    best = (d, joint["id"])
        out.append({
            "uid": uid,
            "brick": sensor.get("brick"),
            "port": sensor.get("port"),
            "mount_link": mount,
            "joint": best[1] if best else None,
            "homing_direction": "unknown",
            "confidence": round(0.5 if best else 0.1, 2),
            "note": ("direction needs Stage 4 kinematics; "
                     "mount link from segmentation members"),
        })
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Sensor-joint association")
    ap.add_argument("--parts", required=True)
    ap.add_argument("--links", required=True,
                    help="runs/<id>/links_joints.json")
    ap.add_argument("--ports", required=True,
                    help="config/ports.local.yaml (hardware layer)")
    ap.add_argument("-o", "--output", required=True)
    args = ap.parse_args()
    parts = json.load(open(args.parts, encoding="utf-8"))
    doc = json.load(open(args.links, encoding="utf-8"))
    ports = load_ports(args.ports)
    sensors = ports.get("sensors") or []
    if not sensors:
        print("no wired sensors in hardware layer; "
              "sensor association pending human wiring")
        raise SystemExit(2)
    result = associate(sensors, parts,
                       doc.get("links", []), doc.get("joints", []))
    os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=2)
    mapped = sum(1 for r in result if r["joint"] is not None)
    print(f"sensors={len(result)} mapped={mapped} -> {args.output}")


if __name__ == "__main__":
    main()
