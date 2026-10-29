"""Convention alignment for model comparison (Stage 4.5).

Two abstract kinematic models (docs/schema.md) can describe the same
physical robot under different arbitrary conventions:

- Joint axis sign: rotating about -w by -q equals rotating about +w
  by +q, so the sign of a listed axis is a pure convention. The guard
  maps each joint to s_i = sign(dot(pred_axis, gt_axis)) and flips the
  reference side where s_i is -1. A |dot| below 0.5 means the axes do
  not even agree as lines, and alignment refuses loudly.
- End-effector roll gauge: the EE point along the roll (tool) axis is
  a definition choice (tool length), the same unobservable gauge that
  PRED_SPEC section 5 grants joint points along their own axes. The
  gauge aligns the evaluated model's EE along its own roll axis to the
  reference EE, then compares full 3D error.

Both transforms are rigid reparameterizations of identical physics,
computed in-process from models passed as CLI arguments. Only the
sign map (+1/-1 per joint) and the removed gauge offset (one scalar)
are reported, never model values.
"""

from __future__ import annotations

import copy
import math

from src.kin.model import chain_order, ee_point, joint_arrays, norm

MIN_ABS_DOT = 0.5


def sign_map(pred_doc: dict, gt_doc: dict) -> dict[str, int]:
    """Per-joint sign s_i = sign(dot(pred_axis, gt_axis)), by ID."""
    pred = {j["id"]: norm(j["axis"]) for j in chain_order(pred_doc)}
    gt = {j["id"]: norm(j["axis"]) for j in chain_order(gt_doc)}
    if set(pred) != set(gt):
        raise ValueError("joint ID sets differ; cannot map signs")
    out = {}
    for jid, w in pred.items():
        v = gt[jid]
        if math.sqrt(sum(x * x for x in v)) == 0.0:
            raise ValueError(f"GT joint {jid} axis is zero")
        d = sum(a * b for a, b in zip(w, v))
        if abs(d) < MIN_ABS_DOT:
            raise ValueError(
                f"joint {jid} axes disagree as lines (|dot|={abs(d):.3f})")
        out[jid] = 1 if d > 0 else -1
    return out


def apply_signs(doc: dict, signs: dict[str, int]) -> dict:
    """Copy of doc with each joint axis multiplied by its sign."""
    out = copy.deepcopy(doc)
    for j in chain_order(out):
        s = signs[j["id"]]
        j["axis"] = [s * float(x) for x in j["axis"]]
    return out


def roll_joint_id(doc: dict) -> str:
    """Joint driving the EE link (its axis is the roll gauge)."""
    ee_link = doc.get("end_effector", {}).get("link")
    for j in chain_order(doc):
        if j["child"] == ee_link:
            return j["id"]
    return chain_order(doc)[-1]["id"]


def align_ee_gauge(pred_doc: dict, gt_doc: dict) -> tuple[dict, float]:
    """Shift the pred EE along its own roll axis to the GT EE level.

    Returns (aligned pred copy, signed offset in mm). Only the
    along-axis component is removed; lateral differences stay.
    """
    rid = roll_joint_id(pred_doc)
    w = norm(next(j["axis"] for j in chain_order(pred_doc)
                  if j["id"] == rid))
    pe, ge = ee_point(pred_doc), ee_point(gt_doc)
    offset = sum((g - p) * a for g, p, a in zip(ge, pe, w))
    out = copy.deepcopy(pred_doc)
    out.setdefault("end_effector", {})["point_mm"] = [
        p + offset * a for p, a in zip(pe, w)]
    return out, offset


def align_for_eval(pred_doc: dict,
                   gt_doc: dict) -> tuple[dict, dict, dict]:
    """Align the reference side to the pred conventions.

    Returns (pred_aligned, gt_aligned, report) where report holds the
    sign map and the removed EE gauge offset. Sampling then uses
    q_aligned_i = s_i * q_i with the aligned GT axes, which reproduces
    the identical physical poses.
    """
    signs = sign_map(pred_doc, gt_doc)
    gt_aligned = apply_signs(gt_doc, signs)
    pred_aligned, offset = align_ee_gauge(pred_doc, gt_aligned)
    return pred_aligned, gt_aligned, {"signs": signs,
                                      "ee_roll_gauge_mm": offset}
