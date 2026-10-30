"""Stage 5D.3: deterministic design enumeration for generated robots.

Default path is exhaustive enumeration: per joint, kind/range
compatible modules x link-driven spans x run-covering beams, ranked
by link-error sum (1st) and part count (2nd), stock violators
eliminated. Fully deterministic; the assembler (5D.4) tries the
ranking in order until its pre-checks pass.

The Spark loop (``run_live``) survives behind ``--explain`` as a
narrator only: it never decides the design. Conventions mirror
``src.agent.run``: Responses API, usage to ``runs/usage.jsonl``,
trace to ``runs/<id>/trace.jsonl``, fixed prompt prefix for caching,
exponential backoff on 429.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import yaml

from src.agent import MODEL_ID
from src.gen import assemble as asm
from src.gen.inventory import load_catalog_merged
from src.gen.modules import load_module, validate_module
from src.gen.spec import load_spec, validate_spec

MAX_ATTEMPTS = 4  # initial proposal + up to 3 revisions
BACKOFF_TRIES = 5

SYSTEM_PROMPT = """You design LEGO robot arms from a library of joint modules.
You pick one module and one beam per joint. A deterministic assembler
places your picks, then reports exact link errors, collisions, stock
shortfalls and an optimization score. You revise from that feedback.

Rules:
1. Reply with a single JSON object only: {"joints": [{"module": name,
   "beam": part_id, "reason": short text}], "notes": short text}.
2. Every joint needs a module whose design range overlaps the spec range.
3. Beams come in 8 mm hole steps; adjust spans from the reported errors.
4. Stay within owned stock. Prefer fewer part kinds, then fewer parts,
   then fewer precious parts (motors, sensors, turntables, large gears).
5. Loaded joints (shoulder, elbow) want large motors; base yaw and wrist
   want medium motors.
"""

TASK_TEMPLATE = """Design this arm.

Spec joints (variable data last):
{spec}

Module library:
{modules}

Available beams (part_id, holes, span_mm):
{beams}

Owned stock (part_id: total [!precious]):
{stock}

Reply with the design JSON object only.
"""

FEEDBACK_TEMPLATE = """Attempt {n} result: {verdict}

{detail}

Best feasible score so far (kinds, parts, precious): {best}.
Reply with the revised design JSON object only.
"""


def _append_usage(record: dict,
                  usage_path: str = "runs/usage.jsonl") -> None:
    os.makedirs(os.path.dirname(usage_path) or ".", exist_ok=True)
    with open(usage_path, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(record) + "\n")


def _save_trace(run_id: str, events: list[dict]) -> str:
    path = os.path.join("runs", run_id, "trace.jsonl")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        for event in events:
            fh.write(json.dumps(event) + "\n")
    return path


def _usage_dict(response) -> dict | None:
    usage = getattr(response, "usage", None)
    if usage is None:
        return None
    if isinstance(usage, dict):
        return usage
    for method in ("model_dump", "dict"):
        if hasattr(usage, method):
            try:
                return getattr(usage, method)()
            except TypeError:
                continue
    return {"repr": str(usage)[:500]}


def _response_text(response) -> str:
    text = getattr(response, "output_text", "") or ""
    if text.strip():
        return text
    chunks = []
    for item in getattr(response, "output", None) or []:
        blocks = item.get("content", []) if isinstance(item, dict) \
            else getattr(item, "content", []) or []
        for block in blocks:
            if isinstance(block, dict) and block.get("text"):
                value = block["text"]
                chunks.append(value.get("value", "") if isinstance(
                    value, dict) else str(value))
            elif getattr(block, "text", None):
                chunks.append(str(block.text))
    return "\n".join(chunks)


def extract_json(text: str) -> dict:
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end <= start:
        raise ValueError("no JSON object in model output")
    return json.loads(text[start:end + 1])


def validate_design(design: dict, spec: dict, library: dict,
                    beam_ids: set[str]) -> list[str]:
    """Check design shape against the spec and library (no assembly)."""
    errors: list[str] = []
    picks = design.get("joints")
    if not isinstance(picks, list) or len(picks) != len(spec["joints"]):
        return [f"design needs {len(spec['joints'])} joint picks"]
    for i, pick in enumerate(picks, 1):
        if not isinstance(pick, dict):
            errors.append(f"J{i}: pick must be an object")
            continue
        if pick.get("module") not in library:
            errors.append(f"J{i}: unknown module {pick.get('module')!r} "
                          f"(library: {sorted(library)})")
        if pick.get("beam") not in beam_ids:
            errors.append(f"J{i}: beam {pick.get('beam')!r} not available")
        if not pick.get("reason"):
            errors.append(f"J{i}: missing reason")
    return errors


def describe_module(name: str, module: dict) -> str:
    joint = module["joint"]
    mounts = module["input_mount"], module["output_mount"]
    stack = sum((a - b) ** 2 for a, b in
                zip(mounts[1]["position_mm"], mounts[0]["position_mm"])) ** 0.5
    bom = module["bom"]
    sensor = module.get("sensor_mount") or {}
    return (f"- {name}: axis {joint['axis']} at {joint['point_mm']}, "
            f"range {joint['range_deg']}, ratio "
            f"{joint['joint_deg_per_motor_deg']}, "
            f"{module['motor']['kind']} motor, sensor "
            f"{sensor.get('kind', 'none')}, stack {stack:.0f} mm, "
            f"{len(bom)} kinds / {sum(bom.values())} parts")


def build_prompt(spec: dict, library: dict, beams: list[dict],
                 inventory: dict | None) -> str:
    spec_text = (f"joints {spec['joints']}, links "
                 f"{spec['link_lengths_mm']} mm, ranges "
                 f"{spec['ranges_deg']}")
    modules_text = "\n".join(describe_module(name, module)
                             for name, module in sorted(library.items()))
    beams_text = "\n".join(f"- {b['part_id']}: {b['holes']} holes, "
                           f"{b['span_mm']:.0f} mm" for b in beams)
    if inventory is not None:
        rows = []
        precious = set(inventory.get("precious", []))
        for pid, row in sorted((inventory.get("parts") or {}).items()):
            if row.get("total", 0) > 0:
                flag = " !precious" if pid in precious else ""
                rows.append(f"{pid}: {row['total']}{flag}")
        stock_text = "\n".join(rows)
    else:
        stock_text = "(no inventory given)"
    return TASK_TEMPLATE.format(spec=spec_text, modules=modules_text,
                                beams=beams_text, stock=stock_text)


def feedback_text(verdict: str, detail: str, best: list | None,
                  attempt: int) -> str:
    return FEEDBACK_TEMPLATE.format(n=attempt, verdict=verdict,
                                    detail=detail, best=best)


def _report_detail(report: dict) -> str:
    lines = [f"feasible={report['ok']}, score={report['score']} "
             f"(kinds, parts, precious)",
             f"link errors mm: {[round(e, 2) for e in report['link_errors_mm']]} "
             f"(tol {asm.LINK_TOL_MM})",
             f"reach spec {report['reach_spec_mm']:.0f} vs actual "
             f"{report['reach_actual_mm']:.0f}"]
    if not report["overlap"]["ok"]:
        lines.append(f"zero-pose overlaps: {report['overlap']['pairs'][:3]}")
    if not report["bom_check"]["ok"]:
        lines.append(f"stock shortfalls: {report['bom_check']['shortfalls']}")
    return "\n".join(lines)


def run_design_loop(call_model, evaluate, spec: dict, library: dict,
                    beams: list[dict], inventory: dict | None,
                    max_attempts: int = MAX_ATTEMPTS) -> dict:
    """Propose/revise loop. Returns {report, design, rounds, history}."""
    beam_ids = {b["part_id"] for b in beams}
    history = [{"role": "system", "content": SYSTEM_PROMPT},
               {"role": "user",
                "content": build_prompt(spec, library, beams, inventory)}]
    trace = [{"role": "system", "content": SYSTEM_PROMPT[:200] + "..."},
             {"role": "user", "content": history[1]["content"][:4000]}]
    best: dict | None = None
    log: list[dict] = []
    for attempt in range(1, max_attempts + 1):
        text = call_model(history)
        trace.append({"role": "assistant", "attempt": attempt,
                      "text": text[:4000]})
        try:
            design = extract_json(text)
        except ValueError as exc:
            detail = f"unparsable reply: {exc}"
            verdict, report = "INVALID", None
        else:
            shape_errors = validate_design(design, spec, library, beam_ids)
            if shape_errors:
                detail = "design rejected:\n" + "\n".join(shape_errors[:8])
                verdict, report = "INVALID", None
            else:
                try:
                    report = evaluate(design)
                except ValueError as exc:
                    detail = f"assembly failed: {exc}"
                    verdict, report = "INFEASIBLE", None
                else:
                    detail = _report_detail(report)
                    verdict = "FEASIBLE" if report["ok"] else "INFEASIBLE"
        entry: dict = {"attempt": attempt, "verdict": verdict,
                       "detail": detail}
        if report is not None:
            entry["score"] = report.get("score")
            if report["ok"] and (best is None or report["score"]
                                 < best["report"]["score"]):
                best = {"report": report, "design": design,
                        "attempt": attempt}
        log.append(entry)
        if report is not None and report["ok"]:
            return {"report": report, "design": design,
                    "rounds": attempt, "history": log, "trace": trace}
        history.append({"role": "assistant", "content": text})
        feedback = feedback_text(
            verdict, detail,
            best["report"]["score"] if best else None, attempt)
        history.append({"role": "user", "content": feedback})
        trace.append({"role": "feedback", "content": feedback[:2000]})
    if best is not None:  # pragma: no cover - early return hits first
        return {"report": best["report"], "design": best["design"],
                "rounds": best["attempt"], "history": log, "trace": trace}
    raise RuntimeError("no feasible design in "
                       f"{max_attempts} attempts:\n" +
                       "\n".join(e["detail"] for e in log))


def _is_rate_limit(exc: Exception) -> bool:
    return getattr(exc, "status_code", None) == 429 \
        or "429" in str(exc) or "rate limit" in str(exc).lower()


def run_live(spec: dict, library: dict, beams: list[dict],
             inventory: dict | None, evaluate, run_id: str,
             max_attempts: int = MAX_ATTEMPTS) -> dict:
    """Call the Responses API for designs. Raises on failure."""
    from openai import OpenAI

    client = OpenAI(base_url="https://api.meta.ai/v1",
                    api_key=os.environ.get("META_API_KEY")
                    or os.environ.get("MODEL_API_KEY"))
    started = time.time()

    def call_model(history: list[dict]) -> str:
        for attempt in range(BACKOFF_TRIES):
            try:
                response = client.responses.create(
                    model=MODEL_ID, instructions=SYSTEM_PROMPT,
                    input=history)
            except Exception as exc:  # noqa: BLE001 - 429 backoff
                if _is_rate_limit(exc) and attempt + 1 < BACKOFF_TRIES:
                    time.sleep(2.0 ** attempt)
                    continue
                raise
            _append_usage({"run": run_id, "model": MODEL_ID,
                           "stage": "5d-design",
                           "usage": _usage_dict(response),
                           "elapsed_s": round(time.time() - started, 1)},
                          os.path.join("runs", "usage.jsonl"))
            return _response_text(response)
        raise RuntimeError("unreachable")  # pragma: no cover

    try:
        outcome = run_design_loop(call_model, evaluate, spec, library,
                                  beams, inventory, max_attempts)
    except (RuntimeError, ValueError):
        raise
    _save_trace(run_id, outcome.pop("trace"))
    return outcome


# --------------------------------------------------- enumeration


def kind_compatible(module: dict, kind: str) -> bool:
    """Whether a module suits a spec joint kind (strict gates).

    yaw wants a Z axis, pitch wants an axis perpendicular to Z and
    to the mount stack, roll wants a stack-parallel axis. Coaxial
    yaw/roll modules match both (documented ambiguity: pin the
    design explicitly when it matters).
    """
    import math

    axis = module["joint"]["axis"]
    norm = math.sqrt(sum(v * v for v in axis)) or 1.0
    unit = [v / norm for v in axis]
    mounts = module["input_mount"], module["output_mount"]
    chain = [b - a for a, b in zip(mounts[0]["position_mm"],
                                   mounts[1]["position_mm"])]
    cnorm = math.sqrt(sum(v * v for v in chain)) or 1.0
    unit_c = [v / cnorm for v in chain]
    dot_z = abs(unit[2])
    dot_c = abs(sum(a * b for a, b in zip(unit, unit_c)))
    if kind == "yaw":
        return dot_z > 0.99
    if kind == "roll":
        return dot_c > 0.99
    return dot_z < 0.01 and dot_c < 0.01  # pitch


def _range_compatible(module: dict, span) -> bool:
    mspan = module["joint"]["range_deg"]
    return not (mspan[1] < span[0] or mspan[0] > span[1])


def _best_span(modules: list[dict], decided: list[float],
               link_idx: int, want_mm: float) -> float:
    """8 mm span minimizing the link error (scan, exact, tiny first)."""
    best, best_err = None, None
    for span in [8.0 * k for k in range(1, 21)]:
        trial = decided + [span, 0.0]
        got = asm.compose_frames(modules, trial)["link_lengths"][link_idx]
        err = abs(got - want_mm)
        if best_err is None or err < best_err - 1e-9:
            best, best_err = span, err
    assert best is not None
    return best


def enumerate_designs(spec: dict, library: dict, catalog: dict,
                      inventory: dict | None = None,
                      min_gap_mm: float = asm.MIN_GAP_MM,
                      last_span_mm: float = 24.0,
                      max_candidates: int | None = None) -> list[dict]:
    """Exhaustive (module x span x beam) ranking (deterministic).

    Per joint: kind/range-compatible modules, link-driven spans and
    run-covering beams (every beam is run-solved against the gap
    holes; misfits are skipped, never guessed). Ranked by link-error
    sum (1st) and part count (2nd); stock violators are eliminated.
    Full tiebreaks keep repeated runs identical. Returns ranked
    ``[{design, score, link_errors, spans, bom}]``; raises when no
    candidate survives (loud, with elimination stats).
    """
    from collections import Counter

    from src.gen import mounts as _mounts

    if spec.get("gripper"):
        raise ValueError("gripper arms are not supported in Part 1")
    beams = _mounts.list_axial_beams(catalog, inventory)
    if not beams:
        raise ValueError("no beams available")
    names = sorted(library)
    compat: list[list[str]] = []
    for i, kind in enumerate(spec["joints"]):
        cands = [name for name in names
                 if kind_compatible(library[name], kind)
                 and _range_compatible(library[name],
                                       spec["ranges_deg"][i])]
        if not cands:
            raise ValueError(f"J{i + 1} ({kind}): no compatible module "
                             f"(library: {names})")
        compat.append(cands)
    stock: dict[str, int] = {}
    if inventory is not None:
        stock = {pid: row.get("total", 0)
                 for pid, row in (inventory.get("parts") or {}).items()}
    # Prefixes: (modules, spans, beams, elements). Step 0 plants the
    # first module; step i >= 1 appends module i with the gap behind
    # (span + run-covering beam); the last gap closes at the end.
    prefixes = [([], [], [], Counter())]
    for i in range(len(spec["joints"])):
        nxt = []
        for mods, spans, picked, elems in prefixes:
            for name in compat[i]:
                modules = mods + [name]
                lib = [library[m] for m in modules]
                if i == 0:
                    nxt.append((modules, [0.0], [], Counter()))
                    continue
                want = spec["link_lengths_mm"][i]
                star = _best_span(lib, spans[:-1], i, want)
                if star >= min_gap_mm:
                    options = [star, star + 8.0]
                else:
                    options = [min_gap_mm, min_gap_mm + 8.0]
                for span in options:
                    trial = spans[:-1] + [span, 0.0]
                    comp = asm.compose_frames(lib, trial)
                    for beam in beams:
                        try:
                            solved = asm.solve_gap(
                                lib[-2], comp["frames"][-2], lib[-1],
                                comp["frames"][-1], beam["part_id"],
                                catalog)
                        except ValueError:
                            continue
                        more = Counter(solved["elements"])
                        nxt.append((modules, trial,
                                    picked + [beam["part_id"]],
                                    elems + more))
        prefixes = nxt
        if not prefixes:
            raise ValueError(f"J{i + 1}: no module/span/beam covers "
                             "the gap (library/beams exhausted)")
    ranked = []
    for mods, spans, picked, elems in prefixes:
        lib = [library[m] for m in mods]
        final = spans[:-1] + [last_span_mm]
        comp = asm.compose_frames(lib, final)
        for beam in beams:
            try:
                solved = asm.solve_gap(
                    lib[-1], comp["frames"][-1], None, None,
                    beam["part_id"], catalog)
            except ValueError:
                continue
            more = Counter(solved["elements"])
            total_elems = elems + more
            bom: Counter = Counter()
            for module in lib:
                bom.update(module["bom"])
            for part_id in picked + [beam["part_id"]]:
                bom[part_id] += 1
            bom.update(total_elems)
            if inventory is not None and any(
                    bom[pid] > stock.get(pid, 0) for pid in bom):
                continue
            link_errors = [abs(got - want) for got, want in
                           zip(comp["link_lengths"],
                               spec["link_lengths_mm"])]
            score = (round(sum(link_errors), 6), sum(bom.values()))
            joints = []
            for k, (m, b, s) in enumerate(zip(
                    mods, picked + [beam["part_id"]], final)):
                joints.append({"module": m, "beam": b, "span_mm": s,
                               "reason": "enumerate"})
            all_beams = picked + [beam["part_id"]]
            usable = sum(next(b["max_span_mm"] for b in beams
                              if b["part_id"] == pid)
                         for pid in all_beams)
            ranked.append({
                "design": {"joints": joints,
                           "notes": "deterministic enumeration"},
                "score": list(score), "link_errors_mm": link_errors,
                "spans_mm": list(final), "bom": dict(sorted(bom.items())),
                "beams": all_beams, "usable_mm": usable})
    if not ranked:
        raise ValueError("no candidate survives stock "
                         f"({len(prefixes)} prefixes assembled)")
    ranked.sort(key=lambda c: (c["score"], c["usable_mm"],
                               [j["module"] for j in
                                c["design"]["joints"]],
                               c["beams"], c["spans_mm"]))
    if max_candidates is not None:
        ranked = ranked[:max_candidates]
    return ranked


def load_library(modules_dir: str | Path, catalog: dict) -> dict:
    library = {}
    for path in sorted(Path(modules_dir).iterdir()):
        if not path.is_dir() or not (path / "module.json").is_file():
            continue
        module = load_module(path / "module.json")
        errors = validate_module(module, path, catalog)
        if errors:
            raise ValueError(f"{path.name}: {errors[0]}")
        library[path.name] = module
    if not library:
        raise ValueError(f"no modules in {modules_dir}")
    return library


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Design a 5D robot.")
    parser.add_argument("--spec", required=True)
    parser.add_argument("--modules", default="assets/modules")
    parser.add_argument("--out", required=True)
    parser.add_argument("--name", default="robot")
    parser.add_argument("--inventory", default="config/inventory.local.yaml")
    parser.add_argument("--explain", action="store_true",
                        help="also ask Spark to narrate the top design "
                        "(needs API key; never decides)")
    parser.add_argument("--max-attempts", type=int, default=MAX_ATTEMPTS)
    parser.add_argument("--run-id", default=None)
    parser.add_argument("--max-candidates", type=int, default=200)
    args = parser.parse_args(argv)

    spec = load_spec(args.spec)
    spec_errors = validate_spec(spec)
    if spec_errors:
        print(f"{args.spec}: INVALID: {spec_errors[0]}")
        return 1
    catalog = load_catalog_merged()
    try:
        library = load_library(args.modules, catalog)
    except ValueError as exc:
        print(f"library error: {exc}")
        return 1
    inventory = None
    inventory_path = Path(args.inventory)
    if inventory_path.exists():
        with open(args.inventory, encoding="utf-8") as fh:
            inventory = yaml.safe_load(fh) or {}
    else:
        print(f"note: {args.inventory} missing, skipping stock check")
    started = time.perf_counter()
    try:
        ranked = enumerate_designs(spec, library, catalog, inventory,
                                   max_candidates=args.max_candidates)
    except (ValueError, RuntimeError) as exc:
        print(f"enumeration failed: {exc}")
        return 1
    elapsed = time.perf_counter() - started
    top = ranked[0]
    (Path(args.out)).mkdir(parents=True, exist_ok=True)
    (Path(args.out) / "design.json").write_text(
        json.dumps({"design": top["design"], "score": top["score"],
                    "link_errors_mm": top["link_errors_mm"],
                    "candidates": len(ranked),
                    "elapsed_s": round(elapsed, 3)}, indent=2),
        encoding="utf-8")
    (Path(args.out) / "candidates.json").write_text(
        json.dumps([{"design": c["design"], "score": c["score"]}
                    for c in ranked], indent=2),
        encoding="utf-8")
    print(f"wrote {args.out}/design.json: score={top['score']} "
          f"candidates={len(ranked)} elapsed_s={elapsed:.3f}")
    if args.explain:
        from src.gen import mounts as _mounts

        beams = _mounts.list_axial_beams(catalog, inventory)

        def evaluate(design: dict) -> dict:
            return asm.assemble_design(design, spec, args.modules,
                                       catalog, inventory)

        run_id = args.run_id or f"5d-design-{int(time.time())}"
        try:
            outcome = run_live(spec, library, beams, inventory,
                               evaluate, run_id, args.max_attempts)
        except (RuntimeError, ValueError) as exc:
            print(f"explain failed: {exc}")
            return 1
        (Path(args.out) / "explain.json").write_text(
            json.dumps({"design": outcome["design"],
                        "rounds": outcome["rounds"]}, indent=2),
            encoding="utf-8")
        print(f"wrote {args.out}/explain.json "
              f"(narrative only, rounds={outcome['rounds']})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
