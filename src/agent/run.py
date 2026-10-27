"""Stage 2 agent runner.

- Live path: Responses API with `muse-spark-1.3-contributor`, tool
  calling with the Stage 1 tools, step limit, forced final answer,
  JSON validation with one repair turn. Usage goes to
  `runs/usage.jsonl`; the full trace goes to `runs/<id>/trace.jsonl`.
- Offline path (`--heuristic` or no API key): deterministic baseline
  from `src.agent.heuristic` using the same geometry tools.

Agent code never reads `eval/ground_truth/`.
"""

from __future__ import annotations

import argparse
import json
import os
import time

from src.agent import (MAX_TOOL_CALLS, MODEL_ID, SYSTEM_PROMPT,
                       TASK_TEMPLATE, TOOL_DEFINITIONS)
from src.agent import heuristic as baseline
from src.agent import input as agent_input
from src.agent.schema import to_public_projection, validate_links_joints
from src.tools import geometry as geom


def _load_json(path: str):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _load_yaml(path: str) -> dict:
    import yaml

    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def _append_usage(record: dict, usage_path: str = "runs/usage.jsonl") -> None:
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


def dispatch_tool(name: str, args: dict, parts: list[dict],
                    catalog: dict):
    """Run one Stage 1 geometry tool; facts only, never conclusions."""
    if name == "get_parts":
        return geom.get_parts(parts, catalog,
                              category=args.get("category"),
                              near=args.get("near"),
                              radius_mm=args.get("radius_mm"))
    if name == "find_connections":
        return geom.find_connections(args["uid"], parts, catalog)
    if name == "check_gear_mesh":
        return geom.check_gear_mesh(args["uid_a"], args["uid_b"],
                                    parts, catalog)
    if name == "trace_axle":
        return geom.trace_axle(args["uid"], parts, catalog)
    if name == "rigid_groups":
        return geom.rigid_groups(parts, catalog,
                                 set(args["rigid_types"])
                                 if args.get("rigid_types") else None)
    raise ValueError(f"unknown tool: {name}")


def _iter_output_items(response) -> list:
    output = getattr(response, "output", None) or []
    if isinstance(output, str):
        return []
    return list(output)


def _item_kind(item) -> str:
    if isinstance(item, dict):
        return item.get("type", "")
    return getattr(item, "type", "") or ""


def _item_text(item) -> str:
    if isinstance(item, dict):
        chunks = []
        for block in item.get("content", []) or []:
            if isinstance(block, dict) and block.get("text"):
                text = block["text"]
                chunks.append(text.get("value", "") if isinstance(text,
                                                                  dict)
                              else str(text))
            elif getattr(block, "text", None):
                chunks.append(str(block.text))
        return "\n".join(chunks)
    chunks = []
    for block in getattr(item, "content", []) or []:
        text = block.text if hasattr(block, "text") else None
        if text is not None:
            chunks.append(str(text))
    return "\n".join(chunks)


def _item_call(item) -> tuple[str, str, dict] | None:
    """(call_id, name, args) for a function_call item, else None."""
    if _item_kind(item) != "function_call":
        return None
    if isinstance(item, dict):
        call_id = item.get("call_id", "")
        name = item.get("name", "")
        raw = item.get("arguments", "{}")
    else:
        call_id = getattr(item, "call_id", "")
        name = getattr(item, "name", "")
        raw = getattr(item, "arguments", "{}")
    try:
        args = json.loads(raw) if isinstance(raw, str) else dict(raw)
    except (json.JSONDecodeError, TypeError, ValueError):
        args = {}
    return call_id, name, args


def _extract_json(text: str) -> dict:
    start = text.find("{")
    end = text.rfind("}")
    if start < 0 or end <= start:
        raise ValueError("no JSON object in model output")
    return json.loads(text[start:end + 1])


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


def run_loop(client, parts: list[dict], catalog: dict, run_id: str,
             max_calls: int = MAX_TOOL_CALLS) -> dict:
    """Responses-API tool loop with validation and one repair turn.

    `client` is an OpenAI-like object exposing `responses.create`. Pure
    loop logic; testable with a fake client (no network needed).
    """
    summary = agent_input.build_summary(parts, catalog)
    prompt = TASK_TEMPLATE.format(
        summary=agent_input.summary_text(summary), max_calls=max_calls)
    history: list = [{"role": "system", "content": SYSTEM_PROMPT},
                     {"role": "user", "content": prompt}]
    trace: list[dict] = [{"role": "system",
                          "content": SYSTEM_PROMPT[:200] + "..."},
                         {"role": "user", "content": prompt[:2000]}]
    calls = 0
    started = time.time()
    pending_text = ""
    while True:
        if calls >= max_calls:
            raise RuntimeError(
                f"live run stopped after {calls} calls without a valid "
                f"structured answer; trace kept for repair")
        response = client.responses.create(
            model=MODEL_ID, instructions=SYSTEM_PROMPT,
            input=history, tools=TOOL_DEFINITIONS)
        _append_usage({"run": run_id, "model": MODEL_ID,
                       "usage": _usage_dict(response),
                       "elapsed_s": round(time.time() - started, 1)},
                      os.path.join("runs", "usage.jsonl"))
        calls += 1
        tool_calls = []
        for item in _iter_output_items(response):
            call = _item_call(item)
            if call is not None:
                tool_calls.append(call)
            else:
                text = _item_text(item)
                if text.strip():
                    pending_text = text
                    trace.append({"role": "assistant", "text": text[:4000]})
        if not tool_calls:
            text = pending_text or getattr(response, "output_text", "") or ""
            trace.append({"role": "assistant-final",
                          "text": text[:8000]})
            _save_trace(run_id, trace)
            try:
                doc = _extract_json(text)
            except ValueError:
                raise RuntimeError(
                    "live run ended without tool calls or JSON; "
                    "trace kept for repair")
            errors = validate_links_joints(doc)
            if not errors:
                return doc
            if calls >= max_calls:
                raise RuntimeError(
                    f"invalid structured answer: {errors[:5]}")
            repair = ("Your JSON failed validation:\n"
                      + "\n".join(errors[:10])
                      + "\nReply with the corrected JSON object only.")
            history.append({"role": "user", "content": repair})
            trace.append({"role": "repair", "content": repair[:2000]})
            max_calls = calls + 1  # one repair turn only
            pending_text = ""
            continue
        history.append({"role": "assistant",
                        "content": f"{len(tool_calls)} tool call(s)"})
        for call_id, name, args in tool_calls:
            trace.append({"role": "tool_call",
                          "name": name, "args": args})
            try:
                result = dispatch_tool(name, args, parts, catalog)
                payload = json.dumps(result)[:12000]
            except Exception as exc:  # noqa: BLE001 - reported to model
                payload = json.dumps({"error": str(exc)[:500]})
            trace.append({"role": "tool_result", "name": name,
                          "result": payload[:4000]})
            history.append({"role": "user", "content":
                            f"Tool result for {name}:\n{payload}"})


def run_live(parts: list[dict], catalog: dict, run_id: str,
             max_calls: int = MAX_TOOL_CALLS) -> dict:
    """Call the Responses API with tool calling. Raises on failure."""
    from openai import OpenAI

    client = OpenAI(base_url="https://api.meta.ai/v1",
                    api_key=os.environ.get("META_API_KEY")
                    or os.environ.get("MODEL_API_KEY"))
    return run_loop(client, parts, catalog, run_id, max_calls)


def run_offline(parts: list[dict], catalog: dict, run_id: str) -> dict:
    events = [{"role": "note",
               "content": "offline heuristic baseline; no model calls"}]
    doc = baseline.infer(parts, catalog)
    _save_trace(run_id, events)
    return doc


def write_outputs(doc: dict, run_id: str) -> tuple[str, str]:
    errors = validate_links_joints(doc)
    if errors:
        raise ValueError(f"invalid links/joints output: {errors[:5]}")
    private_path = os.path.join("runs", run_id, "links_joints.json")
    public_path = os.path.join("runs", run_id, "links_joints_public.json")
    os.makedirs(os.path.dirname(private_path), exist_ok=True)
    with open(private_path, "w", encoding="utf-8") as fh:
        json.dump(doc, fh, indent=2)
    with open(public_path, "w", encoding="utf-8") as fh:
        json.dump(to_public_projection(doc), fh, indent=2)
    return private_path, public_path


def main() -> None:
    ap = argparse.ArgumentParser(description="Stage 2 agent run")
    ap.add_argument("--parts", required=True)
    ap.add_argument("--catalog", required=True)
    ap.add_argument("--run-id", required=True)
    ap.add_argument("--heuristic", action="store_true",
                    help="offline deterministic baseline")
    ap.add_argument("--max-calls", type=int, default=MAX_TOOL_CALLS)
    args = ap.parse_args()

    parts = _load_json(args.parts)
    catalog = _load_yaml(args.catalog)
    has_key = bool(os.environ.get("META_API_KEY")
                   or os.environ.get("MODEL_API_KEY"))
    if args.heuristic or not has_key:
        doc = run_offline(parts, catalog, args.run_id)
    else:
        doc = run_live(parts, catalog, args.run_id, args.max_calls)
    private_path, public_path = write_outputs(doc, args.run_id)
    print(f"joints={len(doc['joints'])} links={len(doc['links'])}")
    print(f"private: {private_path}")
    print(f"public: {public_path}")
    _ = geom  # tools are exercised inside heuristic/live paths


if __name__ == "__main__":
    main()
