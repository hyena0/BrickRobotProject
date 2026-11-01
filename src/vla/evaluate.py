"""6.4 eval: closed-loop vs first-frame-only vs expert vs random.

Policy-agnostic core: rollouts take an ``act_fn(step, q_now, img)``
callback, so the trajectory logic is testable without torch.
Modal wires real SmolVLA predictors (with queue reset for replanning).
"""

from __future__ import annotations

import math


def score_episode(tip_xyz, blocks, target_colour):
    """Final-tip metrics: success / wrong-block / horizontal error."""
    tb = next(b for b in blocks if b["colour"] == target_colour)
    herr = math.hypot(tip_xyz[0] - tb["x"], tip_xyz[1] - tb["y"])
    z = float(tip_xyz[2])
    success = herr <= 30.0 and 40.0 <= z <= 90.0
    wrong = False
    if not success:
        for b in blocks:
            if b["colour"] == target_colour:
                continue
            if math.hypot(tip_xyz[0] - b["x"],
                          tip_xyz[1] - b["y"]) <= 30.0:
                wrong = True
                break
    return {"success": success, "wrong_block": wrong,
            "hold_herr_mm": round(herr, 2),
            "hold_z_mm": round(z, 2)}


def summarize_eval(rows):
    """Rates + mean error over scored episodes (pure)."""
    n = len(rows)
    ok = sum(1 for r in rows if r["success"])
    wb = sum(1 for r in rows if r["wrong_block"])
    herr = [r["hold_herr_mm"] for r in rows]
    import statistics
    return {"n": n, "success_rate": round(100 * ok / n, 1) if n else 0.0,
            "wrong_block_rate": round(100 * wb / n, 1) if n else 0.0,
            "mean_herr_mm": round(statistics.mean(herr), 1) if herr else
            None}


def rollout(scene, rest_q, doc, cam, act_fn, render_each=True):
    """50-step rollout. act_fn(step, q_now, img) -> q_next."""
    import src.vla.scene as S
    qs = [list(rest_q)]
    tips = []
    frames = []
    tdoc = S.tip_doc(doc)
    img = None
    for t in range(50):
        if render_each or img is None:
            img = S.render_frame(scene, qs[-1], doc, cam, None)
            if render_each:
                frames.append(img)
        qs.append([float(v) for v in act_fn(t, qs[-1], img)])
        tip, _ = S.tip_world(qs[-1], tdoc)
        tips.append([float(v) for v in tip])
    if not render_each:
        frames.append(img)
    return {"traj": qs[1:], "tips": tips, "frames": frames}


def sample_scene_for_eval(rng):
    """Same generator as training scenes (unseen via fresh seed)."""
    import src.vla.scene as S
    scene = S.sample_scene(rng)
    order = ["red", "blue", "green", "yellow"]
    scene["target"] = rng.choice(order)
    if scene["target"] not in [b["colour"] for b in scene["blocks"]]:
        scene["blocks"][rng.randrange(len(scene["blocks"]))][
            "colour"] = scene["target"]
    return scene


def expert_traj(scene, rest_q, doc, tdoc, seed_q):
    """Reference trajectory via the 6.1 demo pipeline (no rendering)."""
    import src.vla.demo as D
    rec, traj = D.run_demo(0, scene, rest_q, tdoc, seed_q)
    return rec, traj


def summarize_sens(obj: dict) -> dict:
    """Instruction-sensitivity verdict from a sens run (pure).

    Same image x 4 colour instructions: per-scene tip spread (max
    pairwise hold-tip distance) plus named-colour hit rate. Returns
    ``{"n", "median_spread_mm", "identical_rate", "named_hit_rate",
    "verdict"}`` where verdict is one of ``ignored`` (spread ~ 0:
    language causally inert), ``weak`` (moves but not to the named
    colour), ``follows`` (hits the named colour).
    """
    scenes = obj.get("scenes", [])
    holds = obj.get("holds", {})
    colours = obj.get("colours",
                      ["red", "blue", "green", "yellow"])
    spreads, hits = [], 0
    for i, sc in enumerate(scenes):
        tips = {}
        for c in colours:
            tip = (holds.get(str(i)) or holds.get(i) or {}).get(c, {})
            tip = tip.get("hold_tip") if isinstance(tip, dict) else tip
            if tip is not None:
                tips[c] = [float(v) for v in tip]
        if len(tips) < 2:
            continue
        pts = list(tips.values())
        spread = max(math.dist(a, b)
                     for a in pts for b in pts)
        spreads.append(spread)
        by_colour = {b["colour"]: b for b in sc["blocks"]}
        # Hit fraction: colours whose own tip lands on their block.
        per = 0
        for c in colours:
            b = by_colour.get(c)
            if b is None or c not in tips:
                continue
            tip = tips[c]
            if math.hypot(tip[0] - b["x"], tip[1] - b["y"]) <= 30.0:
                per += 1
        hits += per / max(1, len([c for c in colours
                                  if c in by_colour and c in tips]))
    import statistics
    n = len(spreads)
    med = round(statistics.median(spreads), 1) if spreads else None
    ident = (round(sum(1 for s in spreads if s < 1.0) / n, 3)
             if n else None)
    hit_rate = round(hits / n, 3) if n else None
    if med is None:
        verdict = "no-data"
    elif med < 1.0:
        verdict = "ignored"
    elif (hit_rate or 0) >= 0.5:
        verdict = "follows"
    else:
        verdict = "weak"
    return {"n": n, "median_spread_mm": med,
            "identical_rate": ident, "named_hit_rate": hit_rate,
            "verdict": verdict}


def random_step(rng, lims, coupled):
    """Uniform joint sample, coupled-ok when possible."""
    import src.vla.scene as S
    from src.kin.ik import _coupled_ok
    for _ in range(50):
        q = [rng.uniform(*lims[j]) for j in S.JOINTS]
        if _coupled_ok(q, S.JOINTS, coupled):
            return q
    return q
