# AGENTS.md

Hackathon project: a Muse Spark agent infers a LEGO robot's kinematic chain from brick-level Studio data and generates FK/IK and EV3 control code. Full plan: `PLAN.md`. Detailed stage plans: `docs/stages/` (work only on the stage and task you are given).

## Language

- The user may write in Korean. Reply in Korean.
- Write all code, comments, commit messages, docs and README in **English**.

## Hard rules

1. **New work only.** Do not copy code from any earlier project. Write everything fresh in this repo.
2. **Never commit appearance or assembly data.** `.io`, `.ldr`, `.mpd`, renders and mesh-based MJCF live in `assets/private/` or `eval/ground_truth/` (git-ignored). Agent logs and tool outputs with part IDs/poses go to `runs/` (git-ignored). Only abstract kinematics (axes, link dimensions, gear ratios, coupling, limits), IK code/weights, primitive-geometry MJCF and generated EV3 code may be committed. See `PLAN.md` section 4a.
3. **Never commit secrets.** API keys come from environment variables (`META_API_KEY`, `MODEL_API_KEY`). `.env` is git-ignored.
4. **Ground truth stays out of the agent.** Files in `eval/ground_truth/` and `gt/` (including `gt/gt_joints.json`) are for the verifier only. Never pass their contents (joint lists, gear ratios, link segmentation, link lengths) into agent prompts or tool outputs. A `gt/` path reaches code only as a scorer CLI argument (`--gt`); extraction/inference code under `src/` (everything except `src/verify/`) must not import or read it — pinned by `tests/test_gt_isolation.py`.
5. **No Streamlit.** No dashboard as a main feature.
6. Do not commit, push or amend unless asked.
7. **BrickRobotProject only.** An extraction session works only inside this repository. Never read, list, or depend on files outside it.

## Architecture

```
src/
  parse/     .io / LDraw parser → parts with id, type, pose
  catalog/   part catalog: part id → category, teeth, axle axis, motor type
  tools/     deterministic tools exposed to the agent
             get_parts, find_connections, check_gear_mesh, trace_axle,
             rigid_groups, gear_graph, axle_carrier, solve_transmission,
             axes_intersect
  agent/     Muse Spark agent loop (tool calling, structured JSON output)
  kin/       kinematic model → MJCF, FK, IK, motor-space conversion
  verify/    compare with ground truth; MuJoCo end-effector error
  codegen/   EV3 program generator → generated/<robot>/
  ev3/       brick programs (master: Wi-Fi MQTT, slave: Bluetooth via master),
             on-brick safety guards, host driver (ports only, protocol in docs/protocol.md)
  viz/       Blender scene import and link/axis overlays (renders stay private)
  demo/      O/X demos (physical + simulation)
  vla/       demo generation + LeRobot conversion + Modal training (stretch)
  gen/       procedural LDraw arm generator (generalization tests)
models/              public primitive-geometry MJCF
config/*.local.yaml  hardware, port wiring (motor/sensor uid → port, no joint names)
                     and zero offsets (git-ignored, human-provided)
eval/ground_truth/   ground-truth kinematics, segmentation, MJCF (verifier only)
docs/schema.md       kinematic model JSON schema
docs/stages/         stage plans and LOG.md
```

- Tools compute facts. The agent interprets them. Do not hard-code robot-specific answers in tools.
- Kinematic model is the contract between stages. Keep `docs/schema.md` in sync. The schema holds abstract kinematics only, never part IDs or part poses.
- VLA training scenes use the primitive-geometry MJCF only, never meshes.
- Safety: on-brick guards (sensor, stall, heartbeat, Bluetooth link) must stay active for every motion. Never bypass or weaken them to make a test pass.
- The agent may see `config/ports.local.yaml` (hardware layer). It must never see `eval/ground_truth/wiring_semantic.yaml` (which port drives which joint, sensor meanings).
- IK takes an end-effector target pose and returns joint angles. Keep this interface stable.

## Model API

- Base URL `https://api.meta.ai/v1`, OpenAI SDK, Responses API.
- Model: `muse-spark-1.3-contributor`. Do not switch to Standard or Muse Wonder without asking.
- Contributor limit: 150 RPM. Use exponential backoff on 429.
- Log `usage` (input, cached, output, reasoning tokens) for every agent call to `runs/usage.jsonl`.
- Keep prompt prefixes stable (system prompt + tool definitions first, variable data last) for caching.

## Cost and workflow discipline

- Long jobs (data generation, training, batch evaluation) run as scripts, unattended. Do not poll or tail logs in the session; read a short summary when they finish.
- Read files partially (`head`, `grep`). Do not dump large files, scenes or datasets into context.
- Every generation or training run starts with a preflight check (joint limits, IK sanity, scene validity) and aborts on failure.
- Run independent experiments in parallel.
- Modal jobs must set a timeout.

## Commands

```bash
python -m src.parse.ldr assets/private/model.ldr -o runs/<id>/parts.json  # parse
python -m src.catalog.draft runs/<id>/parts.json -o src/catalog/parts.yaml  # catalog draft
python -m src.tools.wiring --parts runs/<id>/parts.json --catalog src/catalog/parts.yaml --ports config/ports.local.yaml  # wiring check
python -m src.agent.run --parts runs/<id>/parts.json --catalog src/catalog/parts.yaml --run-id <id> --heuristic  # stage-2 baseline
python -m src.verify.segmentation --pred runs/<id>/links_joints.json --gt eval/ground_truth/segmentation.json  # link score
python -m src.verify.joints --pred runs/<id>/links_joints_public.json --gt gt/gt_joints.json  # joint score
python -m src.viz.overlay --pred runs/<id>/links_joints.json -o runs/<id>/overlay.json  # Blender overlay data
python -m src.agent.sensors --parts runs/<id>/parts.json --links runs/<id>/links_joints.json --ports config/ports.local.yaml -o runs/<id>/sensors.json  # sensor-joint association
python -m src.ev3.check                                           # EV3 connectivity
python -m src.kin.assemble --joints runs/<id>/links_joints_public.json --coupling runs/stage3/pred_C.json --ports config/ports.local.yaml --parts runs/stage1/parts.json --catalog src/catalog/parts.yaml --out models/robot_inferred.json  # assemble model
python -m src.kin.mjcf --model models/robot_inferred.json --out models/robot_inferred.xml  # primitive MJCF
python -m src.kin.fk --model models/robot_inferred.json --q 0,0,0,0,0,0  # FK pose
python -m src.verify.ik_eval --pred models/robot_inferred.json --gt eval/ground_truth/kinematics_gt.json --n 500  # Gate 2 (GT-blind)
```

Update this section when commands change.
