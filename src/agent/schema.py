"""Structured output schema for Stage 2 (links and joints).

Public projection carries abstract kinematics only: no part IDs, no part
poses. The private file additionally records link membership (part uids
per link) for the verifier; it lives under `runs/` (git-ignored).
"""

from __future__ import annotations


def _is_vec3(value: object) -> bool:
    return (isinstance(value, list) and len(value) == 3
            and all(isinstance(v, (int, float)) for v in value))


def validate_links_joints(doc: dict) -> list[str]:
    """Return a list of errors; empty means valid."""
    errors: list[str] = []
    if not isinstance(doc, dict):
        return ["top level must be an object"]
    for key in ("links", "joints", "base", "end_effector"):
        if key not in doc:
            errors.append(f"missing key: {key}")
    links = doc.get("links", [])
    joints = doc.get("joints", [])
    if not isinstance(links, list) or not links:
        errors.append("links must be a non-empty list")
        links = []
    if not isinstance(joints, list):
        errors.append("joints must be a list")
        joints = []
    link_ids = set()
    for i, link in enumerate(links):
        if not isinstance(link, dict) or not link.get("id"):
            errors.append(f"links[{i}] must have an id")
            continue
        link_ids.add(link["id"])
    for i, joint in enumerate(joints):
        if not isinstance(joint, dict):
            errors.append(f"joints[{i}] must be an object")
            continue
        for key in ("id", "type", "parent", "child", "axis",
                    "point_mm", "confidence", "reason"):
            if key not in joint:
                errors.append(f"joints[{i}] missing key: {key}")
        if joint.get("type") != "revolute":
            errors.append(f"joints[{i}].type must be 'revolute'")
        if joint.get("parent") not in link_ids:
            errors.append(f"joints[{i}].parent is not a known link")
        if joint.get("child") not in link_ids:
            errors.append(f"joints[{i}].child is not a known link")
        if "axis" in joint and not _is_vec3(joint["axis"]):
            errors.append(f"joints[{i}].axis must be a 3-vector")
        if "point_mm" in joint and not _is_vec3(joint["point_mm"]):
            errors.append(f"joints[{i}].point_mm must be a 3-vector")
        conf = joint.get("confidence")
        if conf is not None and not (isinstance(conf, (int, float))
                                     and 0.0 <= conf <= 1.0):
            errors.append(f"joints[{i}].confidence must be in [0, 1]")
    base = doc.get("base")
    if isinstance(base, dict):
        if base.get("link") not in link_ids:
            errors.append("base.link is not a known link")
    ee = doc.get("end_effector")
    if isinstance(ee, dict):
        if ee.get("link") not in link_ids:
            errors.append("end_effector.link is not a known link")
    for sensor in doc.get("sensors", []) or []:
        if not isinstance(sensor, dict):
            errors.append("sensors entries must be objects")
            continue
        for key in ("uid", "joint", "homing_direction", "confidence"):
            if key not in sensor:
                errors.append(f"sensors entry missing key: {key}")
    return errors


def to_public_projection(doc: dict) -> dict:
    """Strip part membership, keep abstract kinematics (schema format)."""
    joints = []
    for joint in doc.get("joints", []):
        joints.append({
            "id": joint["id"],
            "type": joint["type"],
            "parent": joint["parent"],
            "child": joint["child"],
            "axis": list(joint["axis"]),
            "point_mm": list(joint["point_mm"]),
            "limits_deg": list(joint.get("limits_deg", [-180, 180])),
            "zero": "pose in the source file",
        })
    return {
        "frame": "base link, Z up, mm",
        "links": [{"id": link["id"]} for link in doc.get("links", [])],
        "joints": joints,
        "actuators": list(doc.get("actuators", [])),
        "transmission": doc.get("transmission", {
            "coupling": "q = C · m, q in joint degrees, m in motor degrees",
            "C": [],
        }),
        "end_effector": dict(doc.get("end_effector", {})),
    }
