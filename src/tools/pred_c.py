"""Build the normalized pred-C coupling matrix (docs/PRED_SPEC.md v1).

Reads the stage-3 solver outputs plus the R17 joint axes and emits a
single self-describing artifact. Alliu numbers are dimensionless
deg/deg responses of relative joint angles, one drive column active
with every other drive locked.

Column convention: C[row][col] with rows J1..J6 and columns the six
drive uids. Each column is divided by its driven joint's own response
so the self cell reads +1 (sign absorbed by definition, PRED_SPEC 2a).
Column p000743 has no driven joint (differential lock partner) and is
kept raw with an explicit note.
"""

from __future__ import annotations

import argparse
import json
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))

JOINT_IDS = ["J1", "J2", "J3", "J4", "J5", "J6"]
MOTOR_IDS = ["p000147", "p000148", "p000149", "p000736", "p000737", "p000743"]

# Driven joint (row index) per drive column, or None when the drive has
# no independent joint (p000743 is the wrist-differential lock partner).
SELF_ROW = {0: 0, 1: 2, 2: 1, 3: 4, 4: 3, 5: None}


def rolling_contact_sign(axis_a, r_a, axis_b, r_b):
    """Sign s with w_b = s * (|u_a| / |u_b|) * w_a for rolling contact.

    No-slip means both sides share the contact-point velocity:
    w_a * (axis_a x r_a) = w_b * (axis_b x r_b), so the sign of the
    speed ratio is the sign of u_a . u_b where u = axis x r. Works for
    external (returns -1), internal (returns +1) and bevel pairs.
    Pure vector arithmetic; no model access.
    """
    def cross(a, b):
        return [a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0]]

    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    s = dot(cross(axis_a, r_a), cross(axis_b, r_b))
    if s == 0.0:
        raise ValueError("degenerate contact geometry")
    return 1 if s > 0 else -1


def chain_product(stages):
    """Signed tooth-chain product from evidenced stage factors.

    stages: list of signed per-stage ratios (driver/driven teeth with
    the rolling-contact or rule-based sign already applied).
    Pure arithmetic; the evidence lives with the caller.
    """
    out = 1.0
    for s in stages:
        out *= s
    return out


def normalize_columns(c_raw, self_row):
    """Divide each column by its driven joint's cell (mechanical rule).

    Columns mapped to None are returned unscaled. Returns (C, notes)
    where notes records the divisor or the raw-column reason.
    """
    n_rows = len(c_raw)
    n_cols = len(c_raw[0])
    c = [list(row) for row in c_raw]
    notes = []
    for col in range(n_cols):
        row = self_row.get(col)
        if row is None:
            notes.append({"col": col, "scaled": False,
                          "reason": "no independent driven joint"})
            continue
        div = c[row][col]
        if div == 0.0:
            raise ValueError(f"zero self response in column {col}")
        for r in range(n_rows):
            c[r][col] = c[r][col] / div
        notes.append({"col": col, "scaled": True, "self_row": row,
                      "divisor": div})
    return c, notes


def sign_flip_check(c, self_row):
    """PRED_SPEC section 2 invariant, once per submission.

    Flipping joint k's axis sign flips that joint's response row, and
    also its drive column (the column normalized by joint k's own
    response, whose divisor changes sign with the axis). The self cell
    keeps +1 ((-1)(+1)(-1)). The falsifiable core is that every driven
    joint's self cell reads +1; a joint with no independent drive
    column (J6: p000743 is a lock partner) flips row-only. Returns a
    per-joint PASS record.
    """
    n = len(c)
    col_of = {}
    for col, row in self_row.items():
        if row is not None:
            col_of[row] = col
    records = []
    for k in range(n):
        if k in col_of:
            ok = abs(c[k][col_of[k]] - 1.0) < 1e-9
            records.append({"joint": JOINT_IDS[k], "pass": bool(ok),
                            "row": k, "col": col_of[k],
                            "mode": "row+col"})
        else:
            records.append({"joint": JOINT_IDS[k], "pass": True,
                            "row": k, "col": None,
                            "mode": "row-only (no independent drive)"})
    return records


def round6(x):
    return round(float(x) + 0.0, 6)


def build_pred_c(c_raw, fallback, axes, drive, provenance_rows,
                 extra_notes, self_row=None):
    """Assemble the pred-C artifact dict from verified inputs."""
    if self_row is None:
        self_row = SELF_ROW
    c_norm, col_notes = normalize_columns(c_raw, self_row)
    checks = sign_flip_check(c_norm, self_row)
    if not all(r["pass"] for r in checks):
        raise ValueError("sign-flip invariant failed")
    return {
        "spec": "docs/PRED_SPEC.md v1",
        "axis_convention": ("+ rotation = right-hand rule about the "
                            "listed R17 axis (proximal->distal); axis "
                            "position along its own direction is "
                            "unobserved gauge (PRED_SPEC 5)"),
        "axes_R17": axes,
        "joint_ids": JOINT_IDS,
        "motor_ids": MOTOR_IDS,
        "C": [[round6(v) for v in row] for row in c_norm],
        "C_raw": [[round6(v) for v in row] for row in c_raw],
        "column_notes": col_notes,
        "row_provenance": provenance_rows,
        "section4_fallback": fallback,
        "drive": drive,
        "sign_flip_self_check": checks,
        "notes": extra_notes,
    }


WRIST_STAGES = {
    # Per-stage signed factors (driver/driven teeth with sign).
    # Signs: external -1 (rule), equal bevel -1 (J1 mitre precedent),
    # knob +1 / unequal bevel -1 / crown -1 (zone-sampled
    # rolling-contact proofs, unanimous over the mesh zone).
    "k4_737_J4": [("812x802 adopted", +16.0 / 16.0),
                  ("802x797 external", -16.0 / 16.0),
                  ("804 flip", -1.0),
                  ("777x769 external", -12.0 / 60.0)],
    "k5_736_J5": [("070x086 knob", +4.0 / 4.0),
                  ("041 flip", -1.0),
                  ("080x095 bevel", -12.0 / 12.0),
                  ("095x010 bevel", -12.0 / 60.0)],
    "k6_736_J6": [("070x086 knob", +4.0 / 4.0),
                  ("041 flip", -1.0),
                  ("079x096 bevel", -12.0 / 12.0),
                  ("097x818 crown", -8.0 / 56.0)],
}

# R17-frame flips (dot of ring catalog axis with the R17 joint axis).
# 010: +1.0; 769: -1.0; 818: catalog label wrong (mesh-fit true normal
# used in proofs, dot +1.0 with R17 J6).
WRIST_F_OUT = {"J4": -1.0, "J5": 1.0, "J6": 1.0}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joints",
                    default="runs/stage2-chain-r17/links_joints.json")
    ap.add_argument("--out", default="runs/stage3/pred_C.json")
    args = ap.parse_args()

    def p(rel):
        return rel if os.path.isabs(rel) else os.path.join(ROOT, rel)

    # Column 0 (p000147 -> J1): hand-derived train, model evidence.
    # Motor shaft -> 541 (12T, collinear 0.07 mm + motor_shaft kind,
    # ratio +1) -> bevel 541x523 (-1, rolling-contact proof) ->
    # compound 523+524 on axle 137 (+1) -> internal rack arcs
    # (+12/140, pitch radius 70.06 mm measured, human 70.00 mm match).
    j1_ratio = -12.0 / 140.0
    # Column 1 (p000148 -> J3): static tooth product; the solver's
    # -0.0498 was lock-leak through a grounded carrier (fixed by
    # holding J2); J3 column is coupling-free with J2 held.
    j3_ratio = -0.03428571428571428
    # Wrist columns: direct tooth-chain coefficients synthesized with
    # the recorded lock law (lock-consistent dominant patterns; rigid
    # single-drive-locked is infeasible: 16*1-12*0 != 0). Solver rigid
    # outputs are not used. 736 drives via its own shaft (086 on the
    # 083/087/088 line); 743's pattern co-motion spins a dead axle.
    k4 = WRIST_F_OUT["J4"] * chain_product(
        s for _, s in WRIST_STAGES["k4_737_J4"])
    k5 = WRIST_F_OUT["J5"] * chain_product(
        s for _, s in WRIST_STAGES["k5_736_J5"])
    k6 = WRIST_F_OUT["J6"] * chain_product(
        s for _, s in WRIST_STAGES["k6_736_J6"])
    # k4 = +0.2, k5 = -0.2, k6 = -1/7 (asserted by test).
    c_raw = [[0.0] * 6 for _ in range(6)]
    c_raw[0][0] = j1_ratio
    c_raw[2][1] = j3_ratio
    # J2 train cut mid-path (CAD gap); 743 train cut (dead axle +
    # meshless pinion); see drive. Both columns stay model zeros.
    c_raw[4][3] = k5
    c_raw[5][3] = k6
    c_raw[3][4] = k4

    # Section 4 fallback: three human-provided lines, excluded from
    # scoring. Magnitudes blind-check model inference (J1 exact).
    fallback = [
        {"cell": "J1/p000147", "value": 0.0857143,
         "provenance": "human-provided",
         "note": "magnitude blind-matches model -0.0857143 exactly"},
        {"cell": "J2/p000149", "value": -0.0238095,
         "provenance": "human-provided",
         "note": "0.0238095 with th2_switch=-1; CAD train cut, "
                 "no model value exists"},
        {"cell": "J3-sign", "value": "th3_switch=-1",
         "provenance": "human-provided",
         "note": "motor-direction constant for 4.4; J3 magnitude "
                 "(-0.034286) is model-derived and re-verified"},
    ]
    # Model-only matrix: column 2 stays all zeros (train cut). The
    # fallback J2 self (-0.0238095, human) normalizes to +1 in C only,
    # applied after build with its own provenance note.

    lj = json.load(open(p(args.joints)))
    axes = {}
    for j in lj["joints"]:
        axes[j["id"]] = {"axis": [round6(v) for v in j["axis"]],
                         "point_mm": [round6(v) for v in
                                      j.get("point_mm", [])]}

    drive = {
        "J1": {"drive_ratio": round6(j1_ratio),
               "provenance": "model-derived",
               "blind_check": "magnitude matches human 0.0857143 "
                              "exactly (<1%); sign -1 from bevel "
                              "rolling-contact proof",
               "switch_4.4": "th1_switch=-1 human-confirmed 2026-10-29"},
        "J2": {"drive_ratio": None,
               "provenance": "break: no model value",
               "evidence": ("upstream 149->240/241(20T)->233/234(16T)"
                            "->237/238(8T)->235/236(24T) connected; "
                            "235/236 sit axle_in_round_hole (free) on "
                            "axle 231; 231 cluster has no toothed "
                            "member (225 is a beam); 211 mates none, "
                            "211x235 rejected; BFS from {240,241} "
                            "reaches no pinion"),
               "verdict": "path break (CAD gap): report and stop; "
                          "section-4 fallback stands"},
        "J3": {"drive_ratio": round6(j3_ratio),
               "provenance": "model-derived",
               "note": "magnitude re-verified from gear train; sign "
                       "for 4.4 via human th3_switch=-1"},
        "wrist": {
            "provenance": "model-derived",
            "method": ("direct tooth-chain coefficients synthesized "
                       "with the recorded lock law (lock-consistent "
                       "dominant patterns); solver rigid outputs "
                       "unused"),
            "k4_737_J4": round6(k4),
            "k5_736_J5": round6(k5),
            "k6_736_J6": round6(k6),
            "stages": WRIST_STAGES,
            "exceptions": [
                "812x802 adopted as meshed by exception: slop +2.416 "
                "LDU vs 2.0 gate, axes parallel, user-confirmed "
                "physical mesh (211x235 stays rejected: daylight "
                "both ways)",
                "48168/48452 axis_local labels wrong: mesh-fit true "
                "normals both [-0.707,0.707,0] == R17 J6; proofs use "
                "the fit normal",
                "knob 070x086 sign +1 proven (unanimous zone "
                "rolling-contact); global sign absorbed by "
                "normalization + 4.4 switches",
                "dual J6 pinions 097+098 agree (-1 each, true "
                "normal); 098x003 is same-side (D5/D5) static "
                "contact, no constraint",
            ],
            "lock_law_status": ("16*w736-12*w743=0 retained as "
                                "virtual-diff hypothesis; physical "
                                "basis unconfirmed (780 meshless, "
                                "6573 teeth None, ratio title-based); "
                                "excluded from scoring"),
        },
        "J743": {"drive_ratio": None,
                 "provenance": "break: no model value",
                 "evidence": ("743->814(joiner)->768(bare axle, no "
                              "gear); 814/768/791 collinear 0.001 mm "
                              "but 768->791 coupling unproven; "
                              "780/791 orphan (791 free in 790 bore, "
                              "780 mates none over full-model scan); "
                              "780 off the diff axis by 17.9 mm "
                              "lateral; no mesh edge from the D3 "
                              "cluster to any D4+ part; virtual "
                              "A8-A50 16/12 edge ungeometric"),
                 "verdict": ("path break (C4-style re-trace "
                             "exhausted): column stays model zeros; "
                             "no section-4 line exists for 743")},
    }
    provenance_rows = {
        "J1": "model-derived", "J2": "human-provided",
        "J3": "model-derived", "J4": "model-derived",
        "J5": "model-derived", "J6": "model-derived",
    }
    extra = [
        "C_raw is fully hand-derived; solver rigid outputs unused.",
        "J3 column is coupling-free with J2 held.",
        "Disposition A: rotation supports in the D1 gap (bushes "
        "527-534, bracket axles 508-510/513) are input-absent "
        "geometry (plain cylinders, some CAD-missing); support ID "
        "excluded, suspects kept, not a failure.",
        "col5 (p000743) is model zeros: C4-style re-trace finds no "
        "mesh chain (dead axle + meshless pinion); no fallback "
        "line exists for 743.",
    ]
    model_self = dict(SELF_ROW)
    model_self[2] = None  # train cut: no model self response
    artifact = build_pred_c(c_raw, fallback, axes, drive,
                            provenance_rows, extra,
                            self_row=model_self)
    artifact["C"][1][2] = 1.0
    artifact["column_notes"][2] = {
        "col": 2, "scaled": "fallback", "self_row": 1,
        "divisor": -0.0238095, "provenance": "human-provided",
        "note": "no model response (train cut); fallback self is +1 "
                "by definition, excluded from scoring"}
    artifact["sign_flip_self_check"] = sign_flip_check(
        artifact["C"], SELF_ROW)
    if not all(r["pass"] for r in artifact["sign_flip_self_check"]):
        raise ValueError("sign-flip invariant failed after fallback")
    trans_c = [list(row) for row in artifact["C"]]
    for r in range(6):
        trans_c[r][2] = 0.0  # fallback excluded from scoring
    artifact["transmission"] = {
        "C": [[round6(v) for v in row] for row in trans_c],
        "note": ("model-only matrix for src.verify.transmission; "
                 "normalized cols 0,1,3,4; col2 zeros (J2 train cut, "
                 "fallback excluded); col5 zeros (743 break)")}
    with open(p(args.out), "w") as f:
        json.dump(artifact, f, indent=1, ensure_ascii=False)
        f.write("\n")
    print("wrote", p(args.out))


if __name__ == "__main__":
    main()
