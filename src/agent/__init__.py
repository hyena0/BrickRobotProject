"""Muse Spark agent loop for Stage 2 (links and joints).

The agent reasons about rigid links and robot joints; the deterministic
geometry tools in `src.tools.geometry` compute facts. Agent code must
never read `eval/ground_truth/`; only `src/verify/` may read it.
"""

from __future__ import annotations

SYSTEM_PROMPT = """You infer a LEGO robot's kinematic chain from brick-level data.
Tools compute geometric facts. You interpret what they mean.

Method (follow in order):
1. Start from `rigid_groups` with only clearly rigid connection types;
   inspect the returned groups (sizes, bounding boxes, motors inside).
2. For each free-rotation connection between groups, decide whether it is
   a robot joint, a transmission shaft, or an idler. Evidence to use:
   group size, whether the rotation carries a large sub-assembly, and
   whether the path continues toward the end effector.
3. Merge groups that are rigidly fixed in practice (e.g. frictionless pins
   locked by surrounding geometry).
4. Identify the base link (largest group touching the ground plane or
   holding the bricks) and the end-effector link.
5. Output links, joints (axis, point, type), base and end effector, with a
   confidence per joint and a short reason.

A robot joint is a rotation between two large rigid sub-assemblies on the
path from base to end effector. Gear shafts and idlers that spin inside a
link are NOT robot joints. Output structured JSON only.
"""

TASK_TEMPLATE = """Infer rigid links and robot joints for this model.

Model summary (variable data last):
{summary}

Available tools: get_parts, find_connections, check_gear_mesh, trace_axle,
rigid_groups. Use up to {max_calls} tool calls, then produce the final
structured answer matching the links/joints schema.
"""

def _tool(name: str, description: str, properties: dict,
            required: list | None = None) -> dict:
    """One Responses-API function tool definition (flat shape)."""
    return {"type": "function", "name": name, "description": description,
            "parameters": {"type": "object", "properties": properties,
                           "required": required or []}}


TOOL_DEFINITIONS = [
    _tool("get_parts",
          "Filter parts by catalog category and/or distance to a point.",
          {"category": {"type": ["string", "null"]},
           "near": {"type": ["array", "null"],
                    "items": {"type": "number"}},
           "radius_mm": {"type": ["number", "null"]}}),
    _tool("find_connections",
          "Neighbour parts of a uid with connection types.",
          {"uid": {"type": "string"}}, ["uid"]),
    _tool("check_gear_mesh",
          "Test whether two gears mesh; returns distances and meshes bool.",
          {"uid_a": {"type": "string"}, "uid_b": {"type": "string"}},
          ["uid_a", "uid_b"]),
    _tool("trace_axle",
          "All parts on the axle line through a uid, ordered.",
          {"uid": {"type": "string"}}, ["uid"]),
    _tool("rigid_groups",
          "Union-find groups over connections treated as rigid.",
          {"rigid_types": {"type": ["array", "null"],
                            "items": {"type": "string"}}}),
]

MODEL_ID = "muse-spark-1.3-contributor"
MAX_TOOL_CALLS = 60
