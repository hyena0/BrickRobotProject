# Stage 2: Agent v1, Links and Joints

**Latest:** Day 3 (Gate 1) · **Goal:** the Spark agent splits the robot into rigid links and identifies the joints between them, using only the parsed parts and the geometry tools.

## Key idea

Many parts rotate inside a link without being robot joints (gear shafts, idlers). A **robot joint** is a rotation between two large rigid sub-assemblies on the path from base to end effector. Telling these apart is the agent's main job in this stage.

## Tasks

### 2.1 Agent loop (≈2 h) · `src/agent/`

- Responses API, `muse-spark-1.3-contributor`, tool calling with the Stage 1 tools.
- Stable prompt prefix: system prompt → tool definitions → task. Variable data last.
- Step limit (e.g., 60 tool calls), then force a final answer.
- Final answer validated against a JSON schema (structured output). Invalid output → one repair turn.
- Every call appends `usage` to `runs/usage.jsonl`. Full trace saved to `runs/<id>/trace.jsonl` (private).

### 2.2 Agent input

- A summary of the model: part counts by category, list of motors and sensors with positions, overall bounding box.
- **Not** included: sub-model names, colours as hints, anything from `eval/ground_truth/`.

### 2.3 Prompt strategy

Give the agent a method, not answers:

1. Start from `rigid_groups` with only clearly rigid connection types; inspect the groups.
2. For each free-rotation connection between groups, decide: robot joint, transmission shaft, or idler. Use group size, whether the rotation carries a large sub-assembly, and whether the path continues toward the end effector.
3. Merge groups that are rigidly fixed in practice (e.g., frictionless pins locked by geometry).
4. Identify the base link (largest group touching the ground plane or holding the bricks) and the end-effector link.
5. Output links, joints (axis, point, type), base and end effector, with a confidence per joint and a short reason.

### 2.4 Output

`runs/<id>/links_joints.json` (private: contains part membership) and a public projection in schema format (joint axes and points only).

### 2.5 Evaluation · `src/verify/segmentation.py`, `src/verify/joints.py`

- Segmentation: fraction of parts assigned to the correct link (after matching predicted links to ground-truth links), weighted by part count.
- Joints: count match; axis angle error (°); axis line distance (mm) between predicted and true axes.
- Report as a short table in `LOG.md`.

### 2.6 Iterate

- Up to 3 prompt/tool revisions. Change one thing at a time and log the result.
- If a failure comes from a wrong tool fact, fix the tool, not the prompt.

### 2.7 Sensor association

- For each sensor in `config/ports.local.yaml`, the agent infers which joint it homes: which link it is mounted on, and which joint's motion brings a moving part into contact (touch) or into view (colour).
- Output per sensor: joint id, homing direction, confidence.
- Evaluated against `wiring_semantic.yaml`. If wrong at the gate, the sensor-joint pairs become human-provided.

### 2.8 Visual check with Blender MCP (development aid)

- `src/viz/overlay.py`: colours parts by predicted link, draws each joint axis as an arrow, labels joints. Run through Blender MCP from Muse Code, or headless with `blender -b`.
- Render front/side/top views to `assets/private/renders/<run>/` after every agent run. A human looks at them before reading the metrics; wrong groupings are obvious in seconds.
- These renders are also video material (Stage 8).

### 2.9 Optional agent tool: `render_view`

- If tool facts alone leave a joint ambiguous, give the agent `render_view(azimuth, elevation, zoom, highlight_uids, xray)` backed by Blender, returning an image.
- Use only after 2.1–2.6 work; it adds tokens (images) and latency.

## Gate 1 thresholds (set before the first run)

- Segmentation accuracy ≥ 90 % of parts.
- Joint count exact.
- Axis angle error ≤ 5° and axis distance ≤ 5 mm for every joint.

## Exit criteria

- [ ] Gate 1 thresholds met on the target robot, or fallback applied.
- [ ] Results and token cost per run recorded in `LOG.md`.

## Fallback (Gate 1 fails at Day 3)

Feed link membership from `segmentation.json` as **human-provided input**, document this in README, and keep the agent's job to joint axes, gear trains and kinematics.

## Token estimate

Per agent run: 20–60 tool calls, ≈0.3–1.5 M input tokens mostly cached → $0.03–0.15. Development total $3–8.

## Muse Code prompt (example)

> Stage 2 tasks 2.1–2.5 in docs/stages/stage-2-links-joints.md. Build the agent loop with the existing tools, the JSON schema validation and the evaluation scripts. Do not read anything in eval/ground_truth/ from agent code; only src/verify/ may read it.
