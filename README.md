# Brick Robot Project

Muse Spark agent infers a brick robot's kinematic chain from brick-level Studio data and generates FK/IK and EV3 control code.

Full plan: `docs/PLAN.md`. Agent rules: `docs/AGENTS.md`. Stage plans: `docs/stage/English/`.

## Status

Stage 1 (data and tools) in progress. Stage 0 PC-side done (9/9 tests),
hardware jog/sensor checks pending.

## Built / External / Pre-existing / Evaluation only / Human-provided

| Category | Items |
|---|---|
| Built during the hackathon | MQTT/Bluetooth brick programs and safety guards, host driver, connectivity check (Stage 0); LDraw parser, part catalog draft, geometry tools, wiring checker (Stage 1, in progress) |
| External libraries | MuJoCo, ev3dev/Pybricks runtime, OpenAI SDK, LeRobot/SmolVLA, LDraw part data (CC BY) |
| Pre-existing asset (private) | Studio model rebuilt from purchased instructions (credited, not published; appears only in the demo video) |
| Published outputs | Abstract kinematic model, closed-form and learned IK, primitive-geometry MJCF, generated EV3 code (see `docs/PLAN.md` section 4a) |
| Evaluation only | Prior analysis results and existing EV3 code constants (never given to the agent; `eval/ground_truth/`) |
| Human-provided | Port wiring (motor/sensor to port), zero offsets, camera calibration (`config/*.local.yaml`) |
