"""Stage 5D.5: roundtrip verification for generated robots.

Runs ``robot.ldr`` through generic tools plus the 5D minimal driver
with no hints (inference never sees ``expected_kinematics.json``),
then compares. No single-robot stage-3 code runs here; see the
protocol notes below.

- joint count (exact) and per-joint axes (Gate 1: lateral <= 5 mm,
  angle <= 5 deg, sign-invariant),
- link lengths (consecutive joint distance, <= 8 mm),
- per-joint drive ratios (stage-3 gates on magnitudes: <= 1.0 % and
  <= 0.02 abs; signs are gated too: the predicted sign must match
  the declared sign on sign-known paths, and unknown-sign paths
  fail loudly),
- IK end error (Gate 2: success >= 95 %, median <= 2 mm).

Three documented protocol choices keep the comparison fair. First,
the inferred kinematics cannot know the tool tip, so the IK leg
evaluates both sides at the wrist centre (last joint point); the tip
offset is reported, not gated. Second, arms with fewer than 6 joints
cannot control full orientation, so their IK leg is position-only
(same 95 % / 2 mm gate). Third, the stage-3 topology builder is MR
JOS-specific, so ratios come from a small generic driver (motor to
joint gear-train paths over ``gear_graph``); coupled or ambiguous
trains fail loudly instead of guessing.
"""

from __future__ import annotations

import argparse
import copy
import datetime
import json
import os
import time
from collections import deque
from pathlib import Path

import yaml

from src.agent.schema import to_public_projection, validate_links_joints
from src.agent.transmission import joint_gears
from src.gen.inventory import load_catalog_merged
from src.kin.model import chain_order, load_model
from src.parse import parse_file
from src.tools.transmission import gear_graph
from src.verify import ik_eval
from src.verify.joints import (AXIS_TOL_DEG, POS_TOL_MM, axis_angle_deg,
                               axis_line_distance, score_joint)
from src.verify.transmission import max_abs_diff

LINK_TOL_MM = 8.0
RATIO_PCT_TOL = 1.0
C_ABS_TOL = 0.02
IK_N = 500
MATCH_ANGLE_MAX_DEG = 45.0
MODULE_AXIS_TOL_DEG = 1.0


# --------------------------------------------------------------- matching


def match_joints(pred_joints: list[dict],
                 exp_joints: list[dict]) -> list[tuple[int, int]]:
    """Greedy bijective pairing by joint-line distance.

    Returns (pred_index, exp_index) pairs in expected chain order.
    Raises ValueError on count mismatch or implausible pairs.
    """
    if len(pred_joints) != len(exp_joints):
        raise ValueError(f"joint count differs: pred {len(pred_joints)} "
                         f"vs expected {len(exp_joints)}")
    costs = []
    for i, pred in enumerate(pred_joints):
        for j, exp in enumerate(exp_joints):
            dist = axis_line_distance(pred["axis"], pred["point_mm"],
                                      exp["axis"], exp["point_mm"])
            angle = axis_angle_deg(pred["axis"], exp["axis"])
            costs.append((dist, i, j, angle))
    costs.sort()
    pairs: list[tuple[int, int]] = []
    used_pred, used_exp = set(), set()
    for dist, i, j, angle in costs:
        if i in used_pred or j in used_exp:
            continue
        if angle > MATCH_ANGLE_MAX_DEG:
            continue  # crossing lines can be near; angle disambiguates
        pairs.append((i, j))
        used_pred.add(i)
        used_exp.add(j)
    if len(pairs) != len(exp_joints):
        raise ValueError("could not pair all joints within "
                         f"{MATCH_ANGLE_MAX_DEG} deg")
    pairs.sort(key=lambda pair: pair[1])
    return pairs


def link_lengths(joints: list[dict]) -> list[float]:
    """Consecutive joint-point distances (base origin first)."""
    import math

    lengths = []
    prev = [0.0, 0.0, 0.0]
    for joint in joints:
        cur = [float(v) for v in joint["point_mm"]]
        lengths.append(math.sqrt(sum((a - b) ** 2 for a, b in zip(cur, prev))))
        prev = cur
    return lengths


# ------------------------------------------------- transmission driver


def _bfs_path(adj: dict[str, list[tuple[str, dict]]],
              start: str, goals: set[str]) -> dict[str, list[dict]]:
    """Shortest edge-path from start to each reachable goal."""
    prev: dict[str, tuple[str, dict] | None] = {start: None}
    queue = deque([start])
    while queue:
        node = queue.popleft()
        for nxt, edge in adj.get(node, []):
            if nxt not in prev:
                prev[nxt] = (node, edge)
                queue.append(nxt)
    paths = {}
    for goal in goals:
        if goal not in prev:
            continue
        edges: list[dict] = []
        node = goal
        while prev[node] is not None:
            node, edge = prev[node]
            edges.append(edge)
        edges.reverse()
        paths[goal] = edges
    return paths


def _path_ratio(edges: list[dict], start: str
                ) -> tuple[float, bool, list[str]]:
    """Speed ratio along a motor-to-joint path (magnitude, sign_known)."""
    ratio, sign_known, kinds = 1.0, True, []
    node = start
    for edge in edges:
        if edge["a"] == node:
            ta, tb, node = edge["ta"], edge["tb"], edge["b"]
        else:
            ta, tb, node = edge["tb"], edge["ta"], edge["a"]
        if ta is None or tb is None:
            raise ValueError(f"mesh {edge['gear_a']}/{edge['gear_b']} "
                             "has unknown teeth")
        ratio *= ta / tb
        kind = edge.get("kind")
        kinds.append(f"{kind}:{ta}/{tb}")
        if kind == "external":
            ratio = -ratio
        elif kind in ("internal", "compound"):
            pass
        else:
            sign_known = False
    return ratio, sign_known, kinds


def infer_transmission(parts: list[dict], catalog: dict,
                       links_doc: dict) -> dict:
    """Infer per-joint drive ratios from gear-train paths.

    Returns {C, joint_ids, motor_ids, trains}. Raises ValueError when
    the topology leaves the simple-train envelope (no motors, missing
    output gear, missing path, coupled or ambiguous drives).
    """
    members = all("members" in link for link in links_doc.get("links", []))
    group_of = None
    if members:
        group_of = {}
        for link in links_doc["links"]:
            for uid in link["members"]:
                group_of[uid] = link["id"]
    graph = gear_graph(parts, catalog, link_of=group_of)
    if graph["intra_meshes"]:
        bad = graph["intra_meshes"][0]
        raise ValueError(f"intra-node mesh at {bad['node']}: locked "
                         "gearbox or mesh-rule false positive")
    motors = sorted(p["uid"] for p in parts
                    if (catalog.get(p["part_id"]) or {}).get("motor"))
    if not motors:
        raise ValueError("no motors found in parts")
    drive = graph["motor_drive"]
    missing = [m for m in motors if not drive.get(m)]
    if missing:
        raise ValueError(f"motors without drive node: {missing}")

    node_of_gear: dict[str, str] = {}
    for node in graph["nodes"]:
        for gear in node["gears"]:
            node_of_gear[gear["uid"]] = node["id"]
    joints = links_doc.get("joints", [])
    out_node: dict[str, str] = {}
    for joint in joints:
        scope = group_of if group_of is not None else {
            p["uid"]: joint["child"] for p in parts}
        gears = joint_gears(joint, parts, catalog, scope)
        if not gears:
            raise ValueError(f"{joint['id']}: no coaxial output gear")
        node = node_of_gear.get(gears[0]["uid"])
        if node is None:
            raise ValueError(f"{joint['id']}: output gear outside graph")
        out_node[joint["id"]] = node

    return solve_trains(graph, joints, motors, drive, out_node)


def solve_trains(graph: dict, joints: list[dict], motors: list[str],
                 drive: dict[str, str],
                 out_node: dict[str, str]) -> dict:
    """Ratios from motor-to-joint graph paths (pure function)."""
    adj: dict[str, list[tuple[str, dict]]] = {}
    for edge in graph["edges"]:
        adj.setdefault(edge["a"], []).append((edge["b"], edge))
        adj.setdefault(edge["b"], []).append((edge["a"], edge))
    goals = set(out_node.values())
    joint_of_node = {node: jid for jid, node in out_node.items()}
    trains: dict[str, dict] = {}
    for motor in motors:
        paths = _bfs_path(adj, drive[motor], goals)
        if len(paths) != 1:
            found = sorted(joint_of_node[g] for g in paths)
            raise ValueError(
                f"motor {motor} reaches {len(paths)} joints {found}; "
                "coupled or disconnected drives are unsupported")
        node, edges = next(iter(paths.items()))
        jid = joint_of_node[node]
        if jid in trains:
            raise ValueError(f"{jid} driven by two motors; ambiguous")
        ratio, sign_known, kinds = _path_ratio(edges, drive[motor])
        trains[jid] = {"motor": motor, "ratio": ratio,
                       "sign_known": sign_known, "path": kinds}
    if set(trains) != {j["id"] for j in joints}:
        raise ValueError("undriven joints remain")
    order = [j["id"] for j in joints]
    matrix = [[0.0 for _ in motors] for _ in order]
    for i, jid in enumerate(order):
        matrix[i][motors.index(trains[jid]["motor"])] = \
            trains[jid]["ratio"]
    return {"C": matrix, "joint_ids": order, "motor_ids": motors,
            "trains": trains}


# -------------------------------------------------- module roundtrip


def infer_module_output(parts: list[dict], catalog: dict) -> dict:
    """Blind module output extraction (no module.json hints).

    Same tools as the robot leg (parse + ``gear_graph`` + train walk):
    the single motor's drive tree must be linear with one sink node,
    whose first gear's axle is the output axis. Returns
    ``{motor, axis, point_mm, ratio_frac, ratio, sign, path}`` with an
    exact ``Fraction`` ratio. Raises ValueError on anything outside
    the one-motor linear-train envelope (coupled, disconnected or
    gearless outputs fail loudly instead of guessing).
    """
    from fractions import Fraction

    from src.tools import geometry as geom
    from src.tools.transmission import gear_graph

    graph = gear_graph(parts, catalog)
    if graph["intra_meshes"]:
        bad = graph["intra_meshes"][0]
        raise ValueError(f"intra-node mesh at {bad['node']}: locked "
                         "gearbox or mesh-rule false positive")
    motors = sorted(p["uid"] for p in parts
                    if (catalog.get(p["part_id"]) or {}).get("motor"))
    if len(motors) != 1:
        raise ValueError(f"module needs exactly 1 motor, found "
                         f"{len(motors)}: {motors}")
    motor = motors[0]
    drive = graph["motor_drive"]
    if motor not in drive:
        raise ValueError(f"motor {motor} has no drive node")
    start = drive[motor]
    adj: dict[str, list[dict]] = {}
    for edge in graph["edges"]:
        adj.setdefault(edge["a"], []).append(edge)
        adj.setdefault(edge["b"], []).append(edge)
    prev: dict[str, dict | None] = {start: None}
    queue = deque([start])
    while queue:
        node = queue.popleft()
        for edge in adj.get(node, []):
            nxt = edge["b"] if edge["a"] == node else edge["a"]
            if nxt not in prev:
                prev[nxt] = edge
                queue.append(nxt)
    geared = {n["id"] for n in graph["nodes"] if n["gears"]}
    unreached = sorted(geared - set(prev))
    if unreached:
        raise ValueError(f"gears disconnected from motor {motor}: "
                         f"{unreached}")
    reached = set(prev) & geared

    def _other(edge: dict, node: str) -> str:
        return edge["b"] if edge["a"] == node else edge["a"]

    children = {
        n: [_other(e, n) for e in adj.get(n, [])
            if _other(e, n) in reached and _other(e, n) != n
            and prev.get(_other(e, n)) is e]
        for n in reached}
    sinks = sorted(n for n in reached if not children[n])
    if len(sinks) != 1:
        raise ValueError(f"motor {motor} drive tree has {len(sinks)} "
                         f"sinks {sinks}; linear trains only")
    sink = sinks[0]
    gears = [g for n in graph["nodes"] if n["id"] == sink
             for g in n["gears"]]
    if not gears:
        raise ValueError(f"output node {sink} holds no gear")
    by_uid = {p["uid"]: p for p in parts}
    axis = geom.axis_world(by_uid[gears[0]["uid"]], catalog)
    if axis is None:
        raise ValueError(f"output gear {gears[0]['uid']} has no "
                         "catalog axis")
    chain: list[dict] = []
    node = sink
    while prev[node] is not None:
        edge = prev[node]
        chain.append(edge)
        node = _other(edge, node)
    chain.reverse()
    # Forward walk (same orientation convention as the 5D driver).
    frac = Fraction(1, 1)
    kinds: list[str] = []
    node = start
    for edge in chain:
        nxt = _other(edge, node)
        if edge["a"] == node:
            t_cur, t_nxt = edge["ta"], edge["tb"]
        else:
            t_cur, t_nxt = edge["tb"], edge["ta"]
        if t_cur is None or t_nxt is None:
            raise ValueError(f"mesh {edge['gear_a']}/{edge['gear_b']} "
                             "has unknown teeth")
        frac *= Fraction(t_cur, t_nxt)
        kinds.append(f"{edge.get('kind')}:{t_cur}/{t_nxt}")
        if edge.get("kind") == "external":
            frac = -frac
        elif edge.get("kind") not in ("internal", "compound"):
            raise ValueError(f"mesh {edge['gear_a']}/{edge['gear_b']} "
                             f"kind {edge.get('kind')!r}: sign unknown, "
                             "refusing to guess")
        node = nxt
    return {"motor": motor, "output_node": sink,
            "output_gear": gears[0]["uid"],
            "axis": [float(v) for v in axis],
            "point_mm": list(by_uid[gears[0]["uid"]]["position_mm"]),
            "ratio_frac": str(frac),
            "ratio": float(frac), "path": kinds}


def run_module_roundtrip(module_dir: str | Path) -> dict:
    """Module subfile vs module.json (axis < 1 deg, ratio exact).

    Inference is blind (parse + gear tools only); the declared values
    are used solely for the comparison. On mismatch the report names
    the side-agnostic numbers for a human to judge -- module.json is
    never edited here.
    """
    from src.gen import ldraw as ldraw_io
    from src.gen.modules import load_module

    module_dir = Path(module_dir)
    module = load_module(module_dir / "module.json")
    catalog = load_catalog_merged()
    parts = ldraw_io.read_module_geometry(module_dir, module)
    inf = infer_module_output(parts, catalog)
    decl_axis = [float(v) for v in module["joint"]["axis"]]
    angle = axis_angle_deg(inf["axis"], decl_axis)
    decl_ratio = float(module["joint"]["joint_deg_per_motor_deg"])
    ratio_ok = inf["ratio"] == decl_ratio
    axis_ok = angle < MODULE_AXIS_TOL_DEG
    return {
        "module": module_dir.name,
        "axis": {"inferred": [round(v, 5) for v in inf["axis"]],
                 "declared": decl_axis,
                 "angle_deg": round(angle, 5),
                 "tol_deg": MODULE_AXIS_TOL_DEG,
                 "pass": bool(axis_ok)},
        "ratio": {"inferred": inf["ratio"],
                  "inferred_frac": inf["ratio_frac"],
                  "declared": decl_ratio,
                  "exact": bool(ratio_ok),
                  "pass": bool(ratio_ok)},
        "output_node": inf["output_node"],
        "output_gear": inf["output_gear"],
        "motor": inf["motor"],
        "path": inf["path"],
        "overall_pass": bool(axis_ok and ratio_ok),
    }


# -------------------------------------------------------------- compare


def compare_joints(pred_joints: list[dict], exp_joints: list[dict],
                   pairs: list[tuple[int, int]]) -> dict:
    rows = []
    for i, j in pairs:
        pred, exp = pred_joints[i], exp_joints[j]
        row = score_joint(pred["axis"], pred["point_mm"],
                          {"axis": exp["axis"], "point": exp["point_mm"]})
        row["pred"], row["expected"] = pred["id"], exp["id"]
        row["line_dist_mm"] = round(axis_line_distance(
            pred["axis"], pred["point_mm"], exp["axis"],
            exp["point_mm"]), 3)
        rows.append(row)
    return {"pass": all(r["pass"] for r in rows), "joints": rows,
            "pos_tol_mm": POS_TOL_MM, "axis_tol_deg": AXIS_TOL_DEG}


def compare_links(pred_joints: list[dict], exp_joints: list[dict],
                  pairs: list[tuple[int, int]]) -> dict:
    ordered = [pred_joints[i] for i, _ in
               sorted(pairs, key=lambda pair: pair[1])]
    pred_len = link_lengths(ordered)
    exp_len = link_lengths(exp_joints)
    errors = [abs(a - b) for a, b in zip(pred_len, exp_len)]
    return {"pass": all(e <= LINK_TOL_MM for e in errors),
            "pred_mm": [round(v, 2) for v in pred_len],
            "exp_mm": [round(v, 2) for v in exp_len],
            "errors_mm": [round(v, 2) for v in errors],
            "tol_mm": LINK_TOL_MM}


def compare_ratios(trains: dict, exp_doc: dict,
                   pairs: list[tuple[int, int]],
                   pred_ids: list[str]) -> dict:
    exp_order = [j["id"] for j in chain_order(exp_doc)]
    exp_c = exp_doc["transmission"]["C"]
    rows = []
    for i, j in pairs:
        pred_ratio = trains[pred_ids[i]]["ratio"]
        exp_ratio = exp_c[j][j]
        denom = abs(exp_ratio)
        err_pct = (abs(abs(pred_ratio) - denom) / denom * 100.0
                   if denom else None)
        abs_diff = abs(abs(pred_ratio) - denom)
        sign_known = bool(trains[pred_ids[i]]["sign_known"])
        sign_ok = (sign_known and (pred_ratio >= 0) == (exp_ratio >= 0)
                   and pred_ratio != 0)
        rows.append({
            "joint": exp_order[j],
            "pred_ratio": round(pred_ratio, 5),
            "exp_ratio": exp_ratio,
            "motor": trains[pred_ids[i]]["motor"],
            "sign_known": sign_known,
            "sign_ok": sign_ok,
            "err_pct": round(err_pct, 3) if err_pct is not None else None,
            "abs_diff": round(abs_diff, 5),
            "pass": bool(err_pct is not None and err_pct <= RATIO_PCT_TOL
                         and abs_diff <= C_ABS_TOL and sign_ok),
        })
    pred_diag = [[abs(trains[pred_ids[i]]["ratio"]) if i == j else 0.0
                  for j in range(len(pairs))] for i in range(len(pairs))]
    exp_diag = [[abs(exp_c[j][k]) for k in range(len(exp_c))]
                for j in range(len(exp_c))]
    c_diff = max_abs_diff(pred_diag, exp_diag)
    ok = all(r["pass"] for r in rows) and c_diff is not None \
        and c_diff <= C_ABS_TOL
    return {"pass": bool(ok), "joints": rows, "c_max_abs_diff": c_diff,
            "ratio_pct_tol": RATIO_PCT_TOL, "c_abs_tol": C_ABS_TOL}


def _wrist_doc(doc: dict) -> dict:
    """Copy with the EE point at the last joint point (wrist centre)."""
    out = copy.deepcopy(doc)
    last = chain_order(out)[-1]["point_mm"]
    out.setdefault("end_effector", {})["point_mm"] = list(last)
    return out


def build_pred_doc(pred_joints: list[dict],
                   pairs: list[tuple[int, int]]) -> dict:
    """Pred joints re-chained in expected order (link names abstract)."""
    ordered = [pred_joints[i] for i, _ in
               sorted(pairs, key=lambda pair: pair[1])]
    n = len(ordered)
    joints = []
    for k, pred in enumerate(ordered, 1):
        joints.append({
            "id": f"J{k}", "type": "revolute",
            "parent": f"L{k - 1}", "child": f"L{k}",
            "axis": list(pred["axis"]),
            "point_mm": list(pred["point_mm"]),
            "limits_deg": list(pred.get("limits_deg", [-180, 180])),
            "zero": "pose in the source file",
        })
    return {
        "frame": "base link, Z up, mm",
        "links": [{"id": f"L{k}"} for k in range(n + 1)],
        "joints": joints,
        "end_effector": {"link": f"L{n}",
                         "point_mm": list(ordered[-1]["point_mm"])},
    }


def _early_verdict(robot_dir: Path, mode: str, count_gate: dict,
                   reason: str) -> dict:
    """FAIL verdict when comparison legs cannot run (still saved)."""
    skipped = {"pass": False, "reason": reason}
    return {
        "robot": robot_dir.name,
        "mode": mode,
        "pipeline": "generic tools + 5D minimal driver",
        "overall_pass": False,
        "gates": {"joint_count": count_gate, "axes": dict(skipped),
                  "links": dict(skipped), "ratios": dict(skipped),
                  "ik": {**skipped, "success_rate": 0.0,
                         "median_pos_mm": None, "n": 0}},
        "notes": reason,
    }


def run_roundtrip(robot_dir: str | Path, infer_fn,
                  n_ik: int = IK_N, seed: int = 0,
                  mode: str = "live", trains_fn=None) -> dict:
    """Full roundtrip; returns the report (raises on pipeline failure)."""
    robot_dir = Path(robot_dir)
    with open(robot_dir / "expected_kinematics.json",
              encoding="utf-8") as fh:
        exp_doc = json.load(fh)
    parts = parse_file(str(robot_dir / "robot.ldr"))
    catalog = load_catalog_merged()
    links_doc = infer_fn(parts, catalog)
    schema_errors = validate_links_joints(links_doc)
    if schema_errors:
        raise ValueError(f"inference output invalid: {schema_errors[:3]}")
    _ = to_public_projection(links_doc)  # contract check, abstract only
    exp_joints = chain_order(exp_doc)
    pred_joints = links_doc["joints"]
    pred_ids = [j["id"] for j in pred_joints]
    count_gate = {"pass": len(pred_joints) == len(exp_joints),
                  "pred": len(pred_joints),
                  "expected": len(exp_joints)}
    if not count_gate["pass"]:
        return _early_verdict(robot_dir, mode, count_gate,
                              "joint count differs; legs skipped")
    try:
        pairs = match_joints(pred_joints, exp_joints)
    except ValueError as exc:
        return _early_verdict(robot_dir, mode, count_gate, str(exc))

    joints = compare_joints(pred_joints, exp_joints, pairs)
    links = compare_links(links_doc["joints"], exp_joints, pairs)
    trains = (trains_fn or infer_transmission)(parts, catalog, links_doc)
    ratios = compare_ratios(trains["trains"], exp_doc, pairs, pred_ids)

    pred_doc = _wrist_doc(build_pred_doc(links_doc["joints"], pairs))
    exp_wrist = _wrist_doc(exp_doc)
    position_only = len(exp_joints) < 6
    ik = ik_eval.evaluate(pred_doc, exp_wrist, n=n_ik, seed=seed,
                          position_only=position_only)
    ik_gate = {"pass": bool(ik["gate_pass"]),
               "success_rate": round(ik["success_rate"], 4),
               "median_pos_mm": round(ik["median_pos_mm"], 3),
               "p95_pos_mm": round(ik["p95_pos_mm"], 3),
               "n": ik["n"], "position_only": position_only}

    tip = [float(v) for v in
           exp_doc["end_effector"].get("point_mm",
                                       exp_joints[-1]["point_mm"])]
    wrist = [float(v) for v in exp_joints[-1]["point_mm"]]
    tip_beyond = sum((a - b) ** 2 for a, b in zip(tip, wrist)) ** 0.5
    overall = all(g["pass"] for g in
                  (count_gate, joints, links, ratios, ik_gate))
    return {
        "robot": robot_dir.name,
        "mode": mode,
        "pipeline": "generic tools + 5D minimal driver",
        "overall_pass": bool(overall),
        "gates": {"joint_count": count_gate, "axes": joints,
                  "links": links, "ratios": ratios, "ik": ik_gate},
        "notes": ("IK evaluated at wrist centre on both sides; "
                  f"tool tip {tip_beyond:.1f} mm beyond wrist "
                  "(reported, not gated). C compared on magnitudes and "
                  "signs; unknown-sign paths fail."
                  + (" Position-only IK: under-6-joint arms cannot "
                     "control full orientation." if position_only else "")),
    }


def write_roundtrip(report: dict, robot_dir: str | Path) -> Path:
    path = Path(robot_dir) / "roundtrip.json"
    path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return path


def oracle_doc(robot_dir: str | Path) -> dict:
    """Deterministic links_doc from assembly truth (members + joints).

    For plumbing verification (driver ratios, IK, link recompute), not a
    judgment verdict: count/axes compare truth to truth. The live-agent
    verdict stays separate (scripts/live_roundtrip_arm_4dof.py).
    """
    from src.render.modal_render import split_links

    robot_dir = Path(robot_dir)
    parts = parse_file(str(robot_dir / "robot.ldr"))
    with open(robot_dir / "report.json", encoding="utf-8") as fh:
        report = json.load(fh)
    with open(robot_dir / "expected_kinematics.json",
              encoding="utf-8") as fh:
        exp = json.load(fh)
    catalog = load_catalog_merged()
    # The assembler tags the end beam one segment past the last joint
    # (it articulates about every joint); link grouping folds that
    # beam-only trailing segment back into the last module segment so
    # the flood always leaves a free joint. Graph nodes are unaffected
    # (rigid groups already span segments; link_of only splits them).
    # Folding applies only when the top segment holds nothing but gap
    # beams; module-bearing top segments keep their articulation split.
    from src.render.modal_render import _fold_trailing_beam_segment
    remapped, beams = _fold_trailing_beam_segment(
        report["segments"], report["beams"])
    split = split_links(parts, catalog, remapped, beams,
                        joints=exp["joints"])
    n = len(exp["joints"])
    return {
        "links": [{"id": lid, "members": uids}
                  for lid, uids in split["links"].items()],
        "joints": [{**j, "confidence": 1.0,
                    "reason": "oracle assembly truth"}
                   for j in exp["joints"]],
        "base": {"link": "L0"}, "end_effector": {"link": f"L{n}"},
        "sensors": [], "actuators": [],
        "transmission": {"coupling": "q = C m", "C": []},
    }


def append_log(report: dict, log_path: str | Path) -> None:
    gates = report["gates"]
    verdict = "PASS" if report["overall_pass"] else "FAIL"
    med = gates["ik"]["median_pos_mm"]
    med_s = "n/a" if med is None else f"{med:.2f}mm"
    detail = (f"{verdict}: joints {gates['joint_count']['pred']}/"
              f"{gates['joint_count']['expected']}, axes "
              f"{'pass' if gates['axes']['pass'] else 'fail'}, links "
              f"{'pass' if gates['links']['pass'] else 'fail'}, ratios "
              f"{'pass' if gates['ratios']['pass'] else 'fail'}, IK "
              f"{gates['ik']['success_rate'] * 100:.1f}%/{med_s}")
    today = datetime.date.today().isoformat()
    line = (f"| {today} | 5D | Roundtrip {report['robot']} "
            f"({report['mode']}) | gates pending | {detail} | yes |\n")
    with open(log_path, "a", encoding="utf-8") as fh:
        fh.write(line)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="5D roundtrip check.")
    parser.add_argument("--robot-dir", required=False, default=None)
    parser.add_argument("--offline", action="store_true",
                        help="heuristic inference instead of Spark")
    parser.add_argument("--live", action="store_true",
                        help="force Spark inference (needs API key)")
    parser.add_argument("--n", type=int, default=IK_N,
                        help="IK samples")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-calls", type=int, default=60)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--log", default=None,
                        help="append one line to this LOG.md")
    parser.add_argument("--module", default=None,
                        help="module dir (module.json + subfile): "
                             "module-level roundtrip instead of robot")
    args = parser.parse_args(argv)

    if args.module:
        try:
            result = run_module_roundtrip(args.module)
        except (ValueError, RuntimeError) as exc:
            print(f"module roundtrip failed: {exc}")
            return 1
        print(json.dumps(result, indent=2))
        return 0 if result["overall_pass"] else 2

    if not args.robot_dir:
        print("need --robot-dir (or --module for module roundtrip)")
        return 2

    has_key = bool(os.environ.get("META_API_KEY")
                   or os.environ.get("MODEL_API_KEY"))
    use_live = args.live or (has_key and not args.offline)
    if args.live and not has_key:
        print("live inference needs META_API_KEY or MODEL_API_KEY")
        return 2
    if use_live:
        from src.agent.run import run_live

        run_id = args.run_id or \
            f"5d-rt-{Path(args.robot_dir).name}-{int(time.time())}"
        infer_fn = lambda parts, catalog: run_live(  # noqa: E731
            parts, catalog, run_id, args.max_calls)
        mode = "live"
    else:
        from src.agent.heuristic import infer

        if not args.offline:
            print("note: no API key, using heuristic baseline")
        infer_fn = infer
        mode = "heuristic"
    try:
        report = run_roundtrip(args.robot_dir, infer_fn, n_ik=args.n,
                               seed=args.seed, mode=mode)
    except (ValueError, RuntimeError) as exc:
        print(f"roundtrip failed: {exc}")
        return 1
    path = write_roundtrip(report, args.robot_dir)
    gates = report["gates"]
    med = gates["ik"]["median_pos_mm"]
    med_s = "n/a" if med is None else f"{med:.2f}mm"
    print(f"wrote {path}: overall="
          f"{'PASS' if report['overall_pass'] else 'FAIL'} "
          f"(joints {gates['joint_count']['pred']}/"
          f"{gates['joint_count']['expected']}, IK "
          f"{gates['ik']['success_rate'] * 100:.1f}%/{med_s})")
    if args.log:
        append_log(report, args.log)
        print(f"appended {args.log}")
    return 0 if report["overall_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
