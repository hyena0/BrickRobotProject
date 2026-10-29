"""Assemble the public inferred model (Stage 4.0).

Merges the R17 joint set, the normalized coupling matrix from
runs/stage3/pred_C.json and the hardware-layer port wiring into one
schema.md model JSON. Output carries abstract kinematics only
(axes, points, limits, C, brick/port + motor type): public.
"""

from __future__ import annotations

import argparse
import json

import yaml


def assemble(joints_path: str, coupling_path: str, ports_path: str,
             parts_path: str, catalog_path: str) -> dict:
    joints_doc = json.load(open(joints_path, encoding="utf-8"))
    pred = json.load(open(coupling_path, encoding="utf-8"))
    ports = yaml.safe_load(open(ports_path, encoding="utf-8")) or {}
    parts = json.load(open(parts_path, encoding="utf-8"))
    catalog = yaml.safe_load(open(catalog_path, encoding="utf-8")) or {}

    type_by_uid = {}
    for p in parts:
        entry = catalog.get(p["part_id"]) or {}
        if entry.get("category") == "motor":
            type_by_uid[p["uid"]] = entry.get("motor", "medium")

    port_by_uid = {}
    brick_by_uid = {}
    for entry in ports.get("motors", []) or []:
        port_by_uid[entry["uid"]] = entry["port"]
        brick_by_uid[entry["uid"]] = entry["brick"]

    motor_ids = list(pred["motor_ids"])
    actuators = [{"id": uid, "brick": brick_by_uid[uid],
                  "port": port_by_uid[uid],
                  "motor": type_by_uid.get(uid, "medium")}
                 for uid in motor_ids]

    joints = []
    for j in joints_doc["joints"]:
        joints.append({"id": j["id"], "type": "revolute",
                       "parent": j["parent"], "child": j["child"],
                       "axis": [float(x) for x in j["axis"]],
                       "point_mm": [float(x) for x in j["point_mm"]],
                       "limits_deg": [float(x) for x in
                                      j["limits_deg"]],
                       "zero": "pose in the source file"})

    last_pt = joints[-1]["point_mm"]
    return {"frame": joints_doc.get("frame", "base link, Z up, mm"),
            "links": joints_doc["links"], "joints": joints,
            "actuators": actuators,
            "transmission": {
                "coupling": "q = C m, q in joint degrees, "
                            "m in motor degrees",
                "C": [[float(x) for x in row] for row in pred["C"]]},
            "end_effector": {"link": joints_doc["end_effector"]["link"]
                             if isinstance(joints_doc.get("end_effector"),
                                           dict) else "D6",
                             "point_mm": last_pt}}


def main() -> None:
    ap = argparse.ArgumentParser(description="Assemble model JSON")
    ap.add_argument("--joints", required=True)
    ap.add_argument("--coupling", required=True)
    ap.add_argument("--ports", required=True)
    ap.add_argument("--parts", required=True)
    ap.add_argument("--catalog", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    doc = assemble(args.joints, args.coupling, args.ports,
                   args.parts, args.catalog)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=1)
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
