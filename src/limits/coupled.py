"""Coupled joint constraints from fold collisions (Stage 5A.3).

Grid-search joint pairs over the 5A.1/5A.2 box, fit conservative
linear inequalities, and validate on random full poses. The fit is
tightened until zero accepted validation poses collide; when the
accepted free space shrinks too far the pair falls back to a grid
lookup table used by is_path_safe. Validation folds grow
pair tables with the observed cells instead of pushing linear
half-planes, which would collapse the box.
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import random

import numpy as np

from .common import (MeshContext, ensure_dir, load_inputs, load_library,
                     pose_collisions)
from .enforce import (LOOKUP_RADIUS_DEG, build_limits, check_lookup)

GRID_DEG = 5.0
N_VALIDATE = 2000
FREE_SPACE_LOSS_FALLBACK = 0.40
# Stage 5A.3 conservative finish (user-ordered, time-boxed): the
# lookup fallback stays implemented but is switched OFF — no tables
# are recorded or consulted, no cells grown. Only per-joint ranges
# plus linear inequalities gate (at most two forms on J2-J3).
USE_LOOKUP_TABLES = False
# Auto-refit during validation (new pairs, bound tightening) is also
# off: fitted bounds stay frozen, validation only measures. Any
# residual fold is handled by narrowing the joint ranges by hand.
ALLOW_TIGHTEN = False
WRIST_POSES = ((0.0, 0.0, 0.0), (20.0, -15.0, 10.0),
               (-20.0, 15.0, -10.0), (30.0, 30.0, -30.0))


def pair_contact_counts(report_path: str = "runs/stage5/"
                         "pose_check_report.json") -> dict:
    """Link-pair mesh contact totals from the Stage 5 pose report."""
    try:
        report = json.load(open(report_path, encoding="utf-8"))
    except OSError:
        return {}
    counts: dict[tuple[str, str], int] = {}
    for pose in report.get("poses", []):
        for c in pose.get("mesh_contacts", []) or []:
            key = tuple(sorted([c.get("link_a", "?"),
                                c.get("link_b", "?")]))
            counts[key] = counts.get(key, 0) + 1
    return counts


def candidate_pairs(ctx: dict, counts: dict,
                    min_contacts: int = 10) -> list[tuple[str, str]]:
    """Joint pairs spanning frequent fold contacts, always with J2-J3."""
    ids = [j["id"] for j in ctx["ordered"]]
    link_ids = ctx["link_ids"]
    pos_of = {lid: n for n, lid in enumerate(link_ids)}
    out: list[tuple[str, str]] = [("J2", "J3")]
    for (la, lb), n in sorted(counts.items(), key=lambda kv: -kv[1]):
        if n < min_contacts or la == lb:
            continue
        ia, ib = pos_of.get(la), pos_of.get(lb)
        if ia is None or ib is None:
            continue
        lo, hi = sorted([ia, ib])
        # Joints strictly between the two links fold them together.
        span = [ids[k] for k in range(lo, hi)
                if 0 <= k < len(ids)]
        for a, b in itertools.combinations(span, 2):
            if (a, b) not in out and (b, a) not in out:
                out.append((a, b))
    return out


def span_of_pair(ctx: dict, ja: str, jb: str) -> set[str]:
    """Links moved by a joint pair: everything from the earlier
    parent to the tip."""
    names = [j["id"] for j in ctx["ordered"]]
    link_ids = ctx["link_ids"]
    ia, ib = names.index(ja), names.index(jb)
    lo = min(ia, ib)
    return set(link_ids[lo:])


def march_pair(farm, mesh: MeshContext, ja: str, jb: str,
                 ranges: dict[str, list[float]],
                 n_rays: int = 16) -> dict:
    """Ray-marched fold boundary for one joint pair (exact verdicts).

    A full 5° grid over wide wrist ranges costs tens of thousands of
    exact poses (multi-hour); the linear fit only reads the boundary,
    so sixteen rays march out from the clean home pose to the first
    collision each (15° coarse, 5° refine, 1° bisect) plus the same
    rays over three wrist variants. Clean samples are free points,
    first hits are colliding points. Validation plus tightening back
    the exit criterion, so proposal resolution never weakens the
    guarantee.
    """
    from .common import span_pairs
    ctx = mesh.ctx
    ordered = ctx["ordered"]
    names = [j["id"] for j in ordered]
    ia, ib = names.index(ja), names.index(jb)
    wrist_ids = names[-3:]
    loa, hia = ranges[ja]
    lob, hib = ranges[jb]
    pairs = span_pairs(mesh, span_of_pair(ctx, ja, jb))
    exclude = mesh.excluded()

    def pose_at(qa, qb, wrist):
        q = [0.0] * len(ordered)
        q[ia], q[ib] = round(qa, 2), round(qb, 2)
        for n, wid in enumerate(wrist_ids):
            q[names.index(wid)] = wrist[n]
        return q

    def check(qs, verdict="mesh"):
        return farm.map(qs, pairs=pairs, exclude=exclude,
                        first_only=True, budget_s=60.0, verdict=verdict)

    # Marching starts from a clean pose. The home wrist must be
    # clean (it is verified globally); wrist variants whose start
    # pose already collides are skipped for that wrist.
    homes = [pose_at(0.0, 0.0, w) for w in
             [(0.0, 0.0, 0.0)] + list(WRIST_POSES)]
    import time as _time
    _t0 = _time.time()
    home_res = check(homes)
    print(f"  [{ja}-{jb}] homes: {_time.time() - _t0:.1f}s, "
          f"skipped={sum(1 for h in home_res[1:] if h)}", flush=True)
    if home_res[0]:
        return {"coll": {(0.0, 0.0)}, "free": set(), "rays": n_rays,
                "mode": "home-colliding",
                "note": "home pose collides; pair locked at home"}
    wrists = [(0.0, 0.0, 0.0)] + [w for w, h in
                                  zip(list(WRIST_POSES), home_res[1:])
                                  if not h]
    skipped = len(WRIST_POSES) + 1 - len(wrists)
    import math as _math
    # Coarse march cells for every (wrist, ray): one batched call.
    jobs: list[tuple] = []
    for wrist in wrists:
        for r in range(n_rays):
            phi = 2.0 * _math.pi * r / n_rays
            dx, dy = _math.cos(phi), _math.sin(phi)
            tmax = min(hia / dx if dx > 1e-9 else float("inf"),
                       -loa / -dx if dx < -1e-9 else float("inf"),
                       hib / dy if dy > 1e-9 else float("inf"),
                       -lob / -dy if dy < -1e-9 else float("inf"))
            if not (tmax > 0) or tmax == float("inf"):
                continue
            steps = max(int(tmax / 15.0), 1)
            seq = [(round(t * dx, 2), round(t * dy, 2))
                   for t in
                   [s * tmax / steps for s in range(1, steps + 1)]]
            jobs.append((wrist, r, seq))
    flat = [c for (_, _, seq) in jobs for c in seq]
    # Coarse march on the coarse mesh (fast proposal); refinement
    # and bisection below use the exact mesh.
    _t0 = _time.time()
    res = check([pose_at(*c, w) for (w, _, seq) in jobs
                 for c in seq for w in [w]], verdict="coarse")
    n_eval = len(flat)
    print(f"  [{ja}-{jb}] coarse: {len(flat)} poses, "
          f"{_time.time() - _t0:.1f}s", flush=True)
    # First hit per (wrist, ray); everything before is free.
    coll: set = set()
    free: set = set()
    segs: list[tuple] = []
    off = 0
    for (wrist, _, seq) in jobs:
        sub = res[off:off + len(seq)]
        off += len(seq)
        prev = (0.0, 0.0)
        hit = next((c for c, h in zip(seq, sub) if h), None)
        for c, h in zip(seq, sub):
            if h:
                break
            prev = c
            free.add(c)
        if hit is not None:
            segs.append((wrist, prev, hit))
    # Refine every first-hit segment (5° samples, then bisect).
    rqs: list[tuple] = []
    rmeta: list[tuple] = []
    for (wrist, prev, hit) in segs:
        dist = max(abs(hit[0] - prev[0]), abs(hit[1] - prev[1]))
        n_mid = max(int(dist / 5.0), 1)
        seq = [(round(prev[0] + (hit[0] - prev[0]) * k / n_mid, 2),
                round(prev[1] + (hit[1] - prev[1]) * k / n_mid, 2))
               for k in range(1, n_mid + 1)]
        for c in seq:
            rqs.append(pose_at(*c, wrist))
            rmeta.append((wrist, prev, hit, c))
    # Exact-mesh verdicts for one 5° refinement batch.
    _t0 = _time.time()
    rres = check(rqs)
    n_eval += len(rqs)
    print(f"  [{ja}-{jb}] refine: {len(rqs)} poses, "
          f"{_time.time() - _t0:.1f}s", flush=True)
    # Re-group refinement hits per segment, preserving order.
    seen: dict = {}
    for (wrist, prev, hit, c), h in zip(rmeta, rres):
        key = (wrist, prev, hit)
        seen.setdefault(key, []).append((c, bool(h)))
    # Batched bisection rounds on the exact mesh: all live midpoints
    # per round in one farm call.
    live: dict = {}
    for (wrist, prev, hit), ch in seen.items():
        first = next((c for c, h in ch if h), hit)
        idx = next((n for n, (c, h) in enumerate(ch) if h),
                   len(ch))
        for c, h in ch[:idx]:
            free.add(c)
        lo, hi = (ch[idx - 1][0] if idx else prev), first
        live[(wrist, prev, hit)] = [lo, hi]
    for _ in range(4):
        mids = []
        keys = []
        for key, (lo, hi) in live.items():
            mid = (round((lo[0] + hi[0]) / 2.0, 2),
                   round((lo[1] + hi[1]) / 2.0, 2))
            if mid == lo or mid == hi:
                continue
            mids.append(pose_at(*mid, key[0]))
            keys.append((key, mid))
        if not mids:
            break
        n_eval += len(mids)
        for (key, mid), h in zip(keys, check(mids)):
            lo, hi = live[key]
            if h:
                live[key] = [lo, mid]
            else:
                live[key] = [mid, hi]
                free.add(mid)
    for (wrist, prev, hit), (lo, hi) in live.items():
        coll.add(hi)
        free.add(lo)
    return {"coll": coll, "free": free - coll, "rays": n_rays,
            "evals": n_eval, "mode": "ray16",
            "skipped_wrists": skipped}


def fit_inequalities(coll: set, free: set = None) -> list[dict]:
    """Conservative linear bounds excluding every colliding cell.

    Fold geometry is sum-like, so candidate forms are qa+qb >= c,
    qa+qb <= c, qa-qb >= c, qa-qb <= c. Every form excludes the whole
    colliding set by construction; prune_forms keeps the cheapest
    cover. Returns inequalities as {form, op, c}.
    """
    if not coll:
        return []
    cand = {"sum": [p[0] + p[1] for p in coll],
            "diff": [p[0] - p[1] for p in coll]}
    out = []
    # >= excludes the half-plane below c: c must pass the colliding
    # maximum (plus a 2.5° half-cell margin) to exclude them all.
    # <= mirrors it past the minimum.
    margin = 2.5
    for name, vals in cand.items():
        out.append({"form": name, "op": ">=",
                    "c": round(max(vals) + margin, 2)})
        out.append({"form": name, "op": "<=",
                    "c": round(min(vals) - margin, 2)})
    return out


def _eval(forms: list[dict], ja: str, jb: str, qa: float,
          qb: float) -> bool:
    for f in forms:
        v = qa + qb if f["form"] == "sum" else qa - qb
        if f["op"] == ">=" and not v >= f["c"] - 1e-9:
            return False
        if f["op"] == "<=" and not v <= f["c"] + 1e-9:
            return False
    return True


def prune_forms(forms: list[dict], ja: str, jb: str, coll: set,
                free: set) -> list[dict]:
    """Drop redundant bounds; keep the set that excludes all colliding
    cells while rejecting the least free space."""
    best = [f for f in forms
            if all(not _eval([f], ja, jb, a, b) for (a, b) in coll)]
    # Greedy cover: repeatedly keep the bound that removes the most
    # remaining colliding cells per free cell lost.
    remaining = set(coll)
    kept: list[dict] = []
    cands = [f for f in best]
    while remaining and cands:
        def score(f):
            kills = sum(1 for cell in remaining
                        if not _eval([f], ja, jb, *cell))
            lost = sum(1 for cell in free
                       if not _eval([f], ja, jb, *cell))
            return (kills / max(lost, 1), kills, -lost)
        cands.sort(key=score, reverse=True)
        top = cands.pop(0)
        if score(top)[1] == 0:
            break
        kept.append(top)
        remaining = {cell for cell in remaining
                     if _eval([top], ja, jb, *cell)}
    return kept


def free_space_loss(kept: list[dict], ja: str, jb: str, free: set) -> float:
    if not free:
        return 0.0
    lost = sum(1 for cell in free if not _eval(kept, ja, jb, *cell))
    return lost / len(free)


def check_inequalities(q: list[float], names: list[str],
                       inequalities: list[dict]) -> bool:
    """True when every linear coupled constraint holds."""
    pos = {jid: v for jid, v in zip(names, q)}
    for d in inequalities:
        ja, jb = d["joints"]
        v = (pos[ja] + pos[jb] if d["form"] == "sum"
             else pos[ja] - pos[jb])
        if d["op"] == ">=" and v < d["c"] - 1e-9:
            return False
        if d["op"] == "<=" and v > d["c"] + 1e-9:
            return False
    return True


def validate(farm, mesh: MeshContext, limits: dict,
             inequalities: list[dict],
             n: int = N_VALIDATE, seed: int = 5,
             tables: dict | None = None) -> dict:
    """Random full poses inside the box: accepted ones must be clean.

    Inequality- or lookup-rejected poses skip the oracle; accepted
    poses run batched on the farm.
    """
    rng = random.Random(seed)
    names = [j["id"] for j in mesh.ctx["ordered"]]
    boxes = {j: limits[j] for j in names}
    qs = [[rng.uniform(*boxes[j]) for j in names] for _ in range(n)]
    accepted_qs = [q for q in qs
                   if check_inequalities(q, names, inequalities)
                   and check_lookup(q, names, tables or {})]
    rejected = n - len(accepted_qs)
    # Bearing pairs are excluded from the collision rule (doc 5A.2),
    # like in the single-joint sweeps: union over all joints.
    exclude = set(mesh.excluded())
    for jid in names:
        exclude |= set(mesh.excluded(jid))
    results = farm.map(accepted_qs, pairs=None, exclude=exclude,
                       first_only=True, budget_s=60.0)
    bad = []
    timeouts = 0
    for q, hits in zip(accepted_qs, results):
        if hits:
            if all(h.get("a") == "TIMEOUT" for h in hits):
                timeouts += 1
            bad.append({"q_deg": [round(v, 2) for v in q],
                        "hits": hits[:5]})
    return {"n": n, "accepted": len(accepted_qs), "rejected": rejected,
            "accepted_colliding": len(bad), "timeouts": timeouts,
            "examples": bad}


def tighten(inequalities: list[dict], names: list[str],
              bad: list[dict]) -> int:
    """Push the nearest satisfied bound past each bad pose (+margin).

    Returns the number of bounds changed. Every change excludes at
    least the bad pose that caused it, so the loop terminates.
    TIMEOUT-only examples (narrowphase budget expiry, not a mesh
    verdict) never tighten bounds: they are inconclusive, and
    tightening on them would destroy free space spuriously.
    """
    changed = 0
    for ex in bad:
        if ex.get("hits") and all(h.get("a") == "TIMEOUT"
                                  for h in ex["hits"]):
            continue
        pos = {jid: v for jid, v in zip(names, ex["q_deg"])}
        best = None
        for d in inequalities:
            ja, jb = d["joints"]
            v = (pos[ja] + pos[jb] if d["form"] == "sum"
                 else pos[ja] - pos[jb])
            dist = (v - d["c"]) if d["op"] == ">=" else (d["c"] - v)
            if dist >= 0 and (best is None or dist < best[0]):
                best = (dist, d, v)
        if best is None:
            continue
        _dist, d, v = best
        step = 2.5
        d["c"] = round(v + step if d["op"] == ">=" else v - step, 2)
        changed += 1
    return changed


def select_conservative(forms: list[dict], ja: str, jb: str,
                        coll: set, free: set,
                        max_forms: int = 2) -> tuple[list[dict], bool,
                                                    float]:
    """Cover colliding cells with at most `max_forms` linear bounds.

    Tries single forms first, then pairs: the first subset excluding
    every colliding cell wins, cheapest free-space loss first.
    Returns (selected, covers_all, loss). When nothing covers all
    cells, returns the best-effort subset and covers_all False so
    the caller reports the gap instead of silently accepting it.
    """
    import itertools as _it
    cands = list(forms)
    for k in range(1, max_forms + 1):
        scored = []
        for combo in _it.combinations(cands, k):
            uncovered = [c for c in coll
                         if _eval(list(combo), ja, jb, *c)]
            lost = sum(1 for c in free
                       if not _eval(list(combo), ja, jb, *c))
            loss = lost / max(len(free), 1)
            scored.append((len(uncovered), loss, list(combo)))
        scored.sort(key=lambda t: (t[0], t[1]))
        if scored and scored[0][0] == 0:
            return scored[0][2], True, round(scored[0][1], 3)
        if scored and k == max_forms:
            return scored[0][2], False, round(scored[0][1], 3)
    return [], False, 1.0


def fit_pair(farm, mesh: MeshContext, ja: str, jb: str,
             ranges: dict) -> tuple[list[dict], dict]:
    """Ray-marched boundary + conservative linear fit, one pair."""
    grid = march_pair(farm, mesh, ja, jb, ranges)
    coll, free = grid["coll"], grid["free"]
    if not coll:
        return [], {"joints": [ja, jb], "n_coll_cells": 0,
                    "n_free_cells": len(free),
                    "free_space_loss": 0.0, "mode": "clean15",
                    "note": "no collisions on the 15° base grid; "
                            "no constraint fitted"}
    forms = fit_inequalities(coll, free)
    kept = prune_forms(forms, ja, jb, coll, free)
    loss = free_space_loss(kept, ja, jb, free)
    rec: dict = {"joints": [ja, jb], "n_coll_cells": len(coll),
                 "n_free_cells": len(free),
                 "free_space_loss": round(loss, 3),
                 "evals": grid.get("evals", 0)}
    if loss > FREE_SPACE_LOSS_FALLBACK and USE_LOOKUP_TABLES:
        rec["mode"] = "lookup"
        rec["note"] = ("free-space loss above fallback; grid "
                       "lookup table used")
        rec["cells"] = sorted([list(c) for c in coll])
        return [], rec
    if loss > FREE_SPACE_LOSS_FALLBACK:
        # Conservative finish: no lookup — cover the cells with at
        # most two linear bounds, whatever the free-space cost.
        sel, covers, sel_loss = select_conservative(
            forms, ja, jb, coll, free)
        rec["mode"] = "linear-conservative"
        rec["note"] = ("lookup off; conservative cover of all "
                       "colliding cells" if covers else
                       "lookup off; best-effort partial cover — "
                       "gap reported, narrow ranges by hand")
        rec["covers_all"] = covers
        rec["cells"] = sorted([list(c) for c in coll])
        rec["free_space_loss"] = sel_loss
        out = [{"joints": [ja, jb], "form": f["form"], "op": f["op"],
                "c": f["c"]} for f in sel]
        return out, rec
    rec["mode"] = "linear"
    rec["inequalities"] = kept
    out = [{"joints": [ja, jb], "form": f["form"], "op": f["op"],
            "c": f["c"]} for f in kept]
    return out, rec


def record_table(tables: dict, rec: dict) -> None:
    """Store a lookup-mode pair grid for runtime enforcement.

    Merges with validation-grown cells when the pair already has a
    table: refits must not discard observed fold tiles.
    """
    if rec.get("mode") != "lookup":
        return
    key = "%s_%s" % tuple(sorted(rec["joints"]))
    old = tables.get(key, {})
    seen = {tuple(c) for c in old.get("cells", [])}
    cells = list(old.get("cells", []))
    for c in rec.get("cells", []):
        if tuple(c) not in seen:
            seen.add(tuple(c))
            cells.append(c)
    tables[key] = {
        "cells": cells,
        "radius_deg": old.get("radius_deg", LOOKUP_RADIUS_DEG),
        "n_free_cells": rec.get("n_free_cells", 0)}


def grow_lookup_cells(ctx: dict, tables: dict, names: list[str],
                      bad: list[dict]) -> tuple[int, set]:
    """Tile observed fold poses into lookup tables.

    Each bad validation pose adds its (qa, qb) projection as a
    colliding cell on every attributed pair (creating the table when
    the pair has none), so the exact pose is rejected next round. A
    2.5 deg square costs negligible free space, unlike pushing a
    linear half-plane past a scattered fold (which collapses the
    whole box: tightening alone drove acceptance to 0/2000). Fitted
    linear bounds stay frozen; tightening handles only examples with
    no pair attribution. Returns (cells added, example indices
    handled).
    """
    from .enforce import lookup_rejects
    link_ids = ctx["link_ids"]
    pos_of = {lid: n for n, lid in enumerate(link_ids)}
    grown = 0
    handled: set[int] = set()
    for i, ex in enumerate(bad):
        pq = {jid: v for jid, v in zip(names, ex["q_deg"])}
        for h in ex.get("hits", []) or []:
            la, lb = h.get("link_a"), h.get("link_b")
            if la is None or lb is None or la == lb:
                continue
            ia, ib = pos_of.get(la), pos_of.get(lb)
            if ia is None or ib is None:
                continue
            lo, hi = sorted([ia, ib])
            span = [names[k] for k in range(lo, hi)
                    if 0 <= k < len(names)]
            for a, b in zip(span, span[1:]):
                key = "%s_%s" % tuple(sorted([a, b]))
                tab = tables.get(key)
                if tab is None:
                    tab = {"cells": [],
                           "radius_deg": LOOKUP_RADIUS_DEG,
                           "n_free_cells": 0}
                    tables[key] = tab
                cell = [round(pq[a], 2), round(pq[b], 2)]
                radius = float(tab.get("radius_deg",
                                       LOOKUP_RADIUS_DEG))
                if lookup_rejects(cell[0], cell[1],
                                  tab.get("cells", []), radius):
                    handled.add(i)
                    continue
                tab["cells"].append(cell)
                grown += 1
                handled.add(i)
    return grown, handled


def attribute_spans(ctx: dict, examples: list[dict]) -> list[tuple]:
    """Joint spans of colliding link pairs in bad validation poses."""
    link_ids = ctx["link_ids"]
    names = [j["id"] for j in ctx["ordered"]]
    pos_of = {lid: n for n, lid in enumerate(link_ids)}
    spans: dict[tuple, int] = {}
    for ex in examples:
        for h in ex.get("hits", []) or []:
            if h.get("a") == "TIMEOUT":
                continue
            la, lb = h.get("link_a"), h.get("link_b")
            if la is None or lb is None or la == lb:
                continue
            ia, ib = pos_of.get(la), pos_of.get(lb)
            if ia is None or ib is None:
                continue
            lo, hi = sorted([ia, ib])
            js = tuple(names[k] for k in range(lo, hi)
                       if 0 <= k < len(names))
            if len(js) >= 2:
                for a, b in zip(js, js[1:]):
                    spans[(a, b)] = spans.get((a, b), 0) + 1
    return sorted(spans, key=lambda k: -spans[k])


def main() -> None:
    from .common import PoseFarm
    ap = argparse.ArgumentParser(description="Coupled constraint fit")
    ap.add_argument("--limits", default="runs/stage5a/limits.json")
    ap.add_argument("--out", default="runs/stage5a/coupled.json")
    ap.add_argument("--validate-n", type=int, default=N_VALIDATE)
    ap.add_argument("--fast-n", type=int, default=500)
    ap.add_argument("--jobs", type=int, default=0)
    ap.add_argument("--pairs", default=None,
                    help="only fit these pairs, e.g. 'J2-J3,J3-J5'")
    args = ap.parse_args()
    ctx = load_inputs()
    lib = load_library()
    mesh = MeshContext(ctx, lib)
    farm = PoseFarm(jobs=args.jobs)
    farm.bind(mesh)
    names = [x["id"] for x in ctx["ordered"]]
    limits_doc = json.load(open(args.limits, encoding="utf-8"))
    ranges = {j: limits_doc["limits_deg"][j] for j in names}
    counts = pair_contact_counts()
    pairs = candidate_pairs(ctx, counts)
    if args.pairs:
        want = {tuple(p.split("-")) for p in args.pairs.split(",")}
        want |= {(b, a) for a, b in want}
        pairs = [p for p in pairs if p in want]
    print(f"candidate pairs: {pairs}", flush=True)
    inequalities: list[dict] = []
    tables: dict[str, list] = {}
    fitted: set[tuple] = set()
    for (ja, jb) in pairs:
        new, rec = fit_pair(farm, mesh, ja, jb, ranges)
        inequalities.extend(new)
        fitted.add((ja, jb))
        if USE_LOOKUP_TABLES:
            record_table(tables, rec)
        print(f"{ja}-{jb}: {rec}", flush=True)
    # Validate on fast samples; the reported round always uses the
    # full requested sample count. Conservative finish: no tables,
    # no auto-refit — validation only measures.
    report: dict = {"accepted_colliding": 1, "accepted": 0,
                    "rejected": 0, "n": 0, "timeouts": 0}
    rounds = 0
    finals = 0
    while rounds < 8 and (inequalities or tables):
        n = args.validate_n if finals else args.fast_n
        report = validate(farm, mesh, ranges, inequalities, n=n,
                          seed=5 + rounds,
                          tables=tables if USE_LOOKUP_TABLES else None)
        print(f"validate round {rounds} (n={n}): "
              f"{report['accepted']} accepted, "
              f"{report['accepted_colliding']} colliding, "
              f"timeouts={report.get('timeouts', 0)}", flush=True)
        real = [e for e in report.get("examples", [])
                if not all(h.get("a") == "TIMEOUT" for h in e["hits"])]
        if not report["accepted_colliding"]:
            if n >= args.validate_n:
                break
            finals += 1
            report = validate(farm, mesh, ranges, inequalities,
                              n=args.validate_n, seed=500 + finals,
                              tables=(tables if USE_LOOKUP_TABLES
                                      else None))
            print(f"validate FINAL (n={args.validate_n}): "
                  f"{report['accepted']} accepted, "
                  f"{report['accepted_colliding']} colliding, "
                  f"timeouts={report.get('timeouts', 0)}", flush=True)
            if not report["accepted_colliding"]:
                break
            real = [e for e in report.get("examples", [])
                    if not all(h.get("a") == "TIMEOUT"
                               for h in e["hits"])]
        added = 0
        n_changed = 0
        grown = 0
        if ALLOW_TIGHTEN:
            for span in attribute_spans(ctx, real):
                if span not in fitted and span not in pairs:
                    new, rec = fit_pair(farm, mesh, span[0], span[1],
                                        ranges)
                    inequalities.extend(new)
                    if USE_LOOKUP_TABLES:
                        record_table(tables, rec)
                    fitted.add(span)
                    pairs.append(span)
                    added += 1
                    print(f"  added pair {span}: {rec}", flush=True)
                    if added >= 2:
                        break
        if USE_LOOKUP_TABLES:
            # Validation folds grow pair tables with the observed
            # cells (tiny squares); linear bounds stay frozen and
            # tighten only for examples with no pair attribution.
            grown, handled = grow_lookup_cells(ctx, tables, names,
                                               real)
            rest = [ex for i, ex in enumerate(real)
                    if i not in handled]
        else:
            rest = list(real)
        if ALLOW_TIGHTEN:
            n_changed = tighten(inequalities, names, rest)
        print(f"  tightened {n_changed} bounds, added {added} pairs, "
              f"grew {grown} cells", flush=True)
        if not n_changed and not added and not grown:
            break
        rounds += 1
    if report["accepted_colliding"] and (inequalities or tables):
        if USE_LOOKUP_TABLES:
            tables["_residual_lookup"] = [
                e["q_deg"] for e in report.get("examples", [])]
        print(f"WARNING: residual colliding poses after validation "
              f"(timeouts={report.get('timeouts', 0)}); flagged for "
              f"Stage 5C physical review", flush=True)
    ensure_dir(args.out)
    with open(args.out, "w", encoding="utf-8") as fh:
        json.dump({"pairs": pairs,
                   "contact_counts": {f"{a}-{b}": n for (a, b), n in
                                      counts.items()},
                   "inequalities": inequalities,
                   "lookup_tables": tables,
                   "validation": dict(report),
                   "tighten_rounds": rounds,
                   "grid_deg": GRID_DEG,
                   "grid_mode": ("ray-march proposal (16 rays x 4 "
                                 "wrist variants, coarse mesh) + exact "
                                 "mesh refinement/bisection; exact mesh "
                                 "validation"),
                   "note": ("linear forms use qa+qb (sum) or qa-qb "
                            "(diff) with a/b stored explicitly in "
                            "limits.json")}, fh, indent=2)
    print(f"-> {args.out}")


if __name__ == "__main__":
    main()
