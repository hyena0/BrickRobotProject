"""Spark-based joint selection for Stage 2 re-identification.

Each candidate carries an evidence row (support count, axis fit error,
sweep range, connected motor path, both group sizes). Spark returns the
joint selection with reasons as JSON. On consistency failure the reason
sentences go back to Spark, up to 3 attempts total.

Never reads `eval/ground_truth/`.
"""

from __future__ import annotations

import json

from src.agent import heuristic as baseline
from src.tools import joints as jtools

SYSTEM = ("You pick robot revolute joints from geometric evidence. "
          "Reply with a JSON object only.")

PROMPT = """\
Pick the robot joints from these candidates. Evidence per candidate:
supports = fitted axis support-point count; fit_err_mm = axis fit error;
sweep = estimated motion range before first new collision (+/- deg);
motor = motor uid whose drive path ends nearest this candidate, with
distance in mm (or none); groups = parent/child rigid-group ids with
member counts.

Base link: {base}. End-effector link: {ee}.

Candidates:
{table}

{failure}Reply with JSON only:
{{"selected": [{{"candidate": <idx>, "id": "J1"..., "reason": "..."}}],
  "dropped": [{{"candidate": <idx>, "reason": "..."}}]}}
Rules: ids J1.. in base-to-tip order; every candidate is either selected
or dropped; a dropped candidate keeps its fitted axis/point for display.
Every motor in the table MUST land on a selected joint; leaving a
motor's candidate unselected fails validation. Connectivity gaps are
recorded, never punished: selecting only motor-less chassis pins to
fake a connected chain is wrong. Prefer candidates with a motor path;
clusters of identical small pins far from any motor path are chassis
detail, not robot joints. Reject fits with fit_err_mm above ~5 unless
the sweep range and motor evidence both support them.
Select the full arm (typically 6 joints for a 6-axis arm); fewer than
4 selected is under-selection. Do NOT drop an arm joint merely for
lacking a motor path: keep it and say what drive evidence is missing.
Known drive gaps (not your fault):
J2 is user-confirmed 149-driven with no CAD path; the J1 slew drive has
no modeled path; the wrist is differentially coupled (one motor, several
joints). The validator records these as sentences; dropping joints to
silence it is wrong.
"""


def _evidence_table(cands: list[dict], sweeps: dict,
                    motor_of: dict[int, tuple | None],
                    sizes: dict[str, int]) -> str:
    lines = []
    for i, cand in enumerate(cands):
        fit = cand["fit"]
        sw = sweeps.get(i, {})
        ga, gb = cand["groups"]
        mot = motor_of.get(i)
        mot_txt = f"{mot[0]}@{mot[1]:.1f}mm" if mot else "none"
        lines.append(
            f"[{i}] groups {ga}(n={sizes.get(ga, '?')})-{gb}"
            f"(n={sizes.get(gb, '?')}); supports={fit['n_supports']}; "
            f"fit_err_mm={fit['fit_error_mm']}; "
            f"axis={fit['axis']}; point={fit['point_mm']}; "
            f"sweep={sw.get('range_note', '?')}; "
            f"motor={mot_txt}; "
            f"turntable_hits={cand.get('turntable_hits', 0)}")
    return "\n".join(lines)


def _parse_json(text: str) -> dict:
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end <= start:
        raise ValueError("no JSON object in Spark reply")
    return json.loads(text[start:end + 1])


def select_joints(client, table: str, base: str, ee: str,
                  failure: str = "") -> dict:
    """One Spark selection turn. `client` exposes `responses.create`."""
    prompt = PROMPT.format(
        table=table, base=base, ee=ee,
        failure=(f"Previous selection failed consistency:\n{failure}\n"
                 if failure else ""))
    response = client.responses.create(
        model="muse-spark-1.3-contributor",
        instructions=SYSTEM,
        input=[{"role": "user", "content": prompt}],
        temperature=0.0)  # deterministic: reruns must not clobber docs
    text = getattr(response, "output_text", "") or ""
    return _parse_json(text)


def _build_doc(parts: list[dict], catalog: dict, cands: list[dict],
               selection: dict) -> tuple[dict, dict[int, str]]:
    doc = baseline.infer(parts, catalog)
    index_to_jid: dict[int, str] = {}
    joints = []
    for sel in selection.get("selected", []):
        i = int(sel["candidate"])
        fit = cands[i]["fit"]
        ga, gb = cands[i]["groups"]
        jid = str(sel.get("id", f"J{i + 1}"))
        index_to_jid[i] = jid
        joints.append({
            "id": jid, "type": "revolute", "parent": ga, "child": gb,
            "axis": fit["axis"], "point_mm": fit["point_mm"],
            "limits_deg": [-180, 180], "confidence": 0.6,
            "status": "selected",
            "reason": str(sel.get("reason", "")),
            "fit_error_mm": fit["fit_error_mm"],
            "n_supports": fit["n_supports"],
        })
    # Only the top dropped candidates render as thin gray arrows; the
    # rest are rejected outright and not drawn.
    for rank, drop in enumerate(selection.get("dropped", [])):
        i = int(drop["candidate"])
        fit = cands[i]["fit"]
        ga, gb = cands[i]["groups"]
        joints.append({
            "id": f"X{i}", "type": "revolute", "parent": ga, "child": gb,
            "axis": fit["axis"], "point_mm": fit["point_mm"],
            "limits_deg": [-180, 180], "confidence": 0.2,
            "status": "dropped" if rank < 6 else "rejected",
            "reason": str(drop.get("reason", "")),
        })
    doc["joints"] = joints
    doc["method"] = "joints-spark-selection"
    return doc, index_to_jid


def assign_motors(cands: list[dict], terminal_mm: dict[str, list]) -> dict:
    """Candidate index -> (motor uid, distance) of nearest path terminal."""
    import numpy as np

    motor_of: dict[int, tuple | None] = {}
    for i in range(len(cands)):
        motor_of[i] = None
    for motor, pos in terminal_mm.items():
        p = np.asarray(pos, dtype=float)
        best, best_d = None, jtools.MOTOR_ASSIGN_MM
        for i, cand in enumerate(cands):
            q = np.asarray(cand["fit"]["point_mm"], dtype=float)
            d = float(np.linalg.norm(p - q))
            if d < best_d:
                best, best_d = i, d
        if best is not None and motor_of[best] is None:
            motor_of[best] = (motor, round(best_d, 1))
    return motor_of


def run_selection(client, parts: list[dict], catalog: dict,
                  terminal_mm: dict[str, list],
                  max_attempts: int = 3) -> dict:
    """Full loop: candidates -> sweep -> Spark -> consistency (x3)."""
    found = jtools.joint_candidates(parts, catalog)
    cands = found["candidates"]
    groups = found["groups"]
    sizes = {g["id"]: g["part_count"] for g in groups}
    sweeps = {i: jtools.sweep_test(c, parts, catalog, groups)
              for i, c in enumerate(cands)}
    motor_of = assign_motors(cands, terminal_mm)
    table = _evidence_table(cands, sweeps, motor_of, sizes)
    doc0 = baseline.infer(parts, catalog)
    base = (doc0.get("base") or {}).get("link", "?")
    ee = (doc0.get("end_effector") or {}).get("link", "?")
    failure = ""
    attempts = 0
    attempt_docs: list[dict] = []
    while True:
        attempts += 1
        selection = select_joints(client, table, base, ee, failure)
        doc, index_to_jid = _build_doc(parts, catalog, cands, selection)
        attempt_docs.append({"attempt": attempts, "doc": doc,
                             "selection": selection})
        motor_claims = {}
        for i, mot in motor_of.items():
            if mot is not None and i in index_to_jid:
                motor_claims[mot[0]] = index_to_jid[i]
        checked = jtools.check_kinematic_consistency(doc, motor_claims)
        errors = checked["errors"]
        if not errors or attempts >= max_attempts:
            doc["drive_gaps"] = checked["gaps"]
            return {"doc": doc, "errors": errors,
                    "gaps": checked["gaps"], "attempts": attempts,
                    "attempt_docs": attempt_docs,
                    "counts": found["counts"], "sweeps": sweeps,
                    "motor_of": motor_of}
        failure = "\n".join(errors)
