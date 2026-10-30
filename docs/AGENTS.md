# AGENTS.md

Hackathon project: a Muse Spark agent infers a LEGO robot's kinematic chain from brick-level Studio data and generates FK/IK and EV3 control code. Full plan: `PLAN.md` (v1) with updates in `PLAN.v2.md` (v2 wins where they differ). Detailed stage plans: `docs/stages/` (work only on the stage and task you are given).

## Language

- The user may write in Korean. Reply in Korean.
- Write all code, comments, commit messages, docs and README in **English**.

## Hard rules

1. **New work only.** Do not copy code from any earlier project. Write everything fresh in this repo.
2. **Never commit appearance or assembly data.** `.io`, `.ldr`, `.mpd`, renders and mesh-based MJCF live in `assets/private/` or `eval/ground_truth/` (git-ignored). Agent logs and tool outputs with part IDs/poses go to `runs/` (git-ignored). Only abstract kinematics (axes, link dimensions, gear ratios, coupling, limits), IK code/weights, primitive-geometry MJCF and generated EV3 code may be committed. See `PLAN.md` section 4a. Real LEGO meshes for development, testing and video live only in the private visual pack `assets/private/visual_pack/` and are served only behind `--private-visuals`; `python -m src.release.check` must pass before any release.
3. **Never commit secrets.** API keys come from environment variables (`META_API_KEY`, `MODEL_API_KEY`). `.env` is git-ignored.
4. **Ground truth stays out of the agent.** Files in `eval/ground_truth/` are for the verifier only. Never pass their contents (joint lists, gear ratios, link segmentation, link lengths) into agent prompts or tool outputs.
5. **No Streamlit.** No dashboard as a main feature.
6. Do not commit, push or amend unless asked.
7. **Local-only hardware and console.** The MQTT broker runs on the hub device (the PC for the hackathon; no cloud or public brokers), with authentication (`hub`, `brick_left` accounts) and topic ACLs. Bricks read the hub address from config. Console and brick code make no internet requests; all web assets are vendored locally (no CDNs, no web fonts). Meta API calls are allowed only in Stage 6 modules.
8. **Controller package layering.** Code in `src/core/` must not call files, network, OS, MuJoCo or web frameworks directly; all I/O goes through the adapter interfaces in `src/core/ports.py`. The robot package (`packages/<robot>/robot_package.json`) is the only runtime input from the authoring tools. The UI contains no kinematics or safety logic. Any change to core behaviour must keep `conformance/` passing (regenerate golden cases only on purpose and log why).

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
  limits/    joint limits from sensor geometry, collisions and coupled constraints
  package/   robot package builder and loader (authoring → runtime contract)
  core/      platform-neutral controller core: FK/IK, limits, path check, motion,
             homing, wizard state machines, protocol; adapter interfaces in ports.py
  adapters/  platform adapters; pc/: MQTT transport, MuJoCo sim transport,
             file storage, process lifecycle
  console/   PC UI bridge (FastAPI + WebSocket shell, no robot logic), CLI wizards
  ui/        shared web UI (plain HTML/JS + three.js vendored in ui/vendor/, no Streamlit)
  demo/      pointing demos (physical + simulation)
  vla/       demo generation + LeRobot conversion + Modal training (stretch)
  gen/       procedural LDraw arm generator (generalization tests)
models/              public primitive-geometry MJCF
packages/<robot>/    robot packages (abstract data only, versioned)
conformance/         golden cases exported from the Python core + runner (porting contract)
config/*.local.yaml  hardware, port wiring (motor/sensor uid → port, no joint names)
                     and zero offsets (git-ignored, human-provided)
eval/ground_truth/   ground-truth kinematics, segmentation, MJCF (verifier only)
docs/schema.md       kinematic model JSON schema
docs/robot_package.md  robot package schema
docs/protocol.md     MQTT protocol (platform-neutral contract)
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
python -m src.agent.run --model-file assets/private/<robot>.io   # run inference
python -m src.verify.run --pred runs/<id>/kinematics.json         # verify
python -m src.ev3.check                                           # EV3 connectivity
```

Update this section when commands change.
