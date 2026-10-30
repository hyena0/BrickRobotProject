# Brick Robot Project

Muse Spark agent infers a brick robot's kinematic chain from brick-level Studio data and generates FK/IK and EV3 control code.

Full plan: `docs/PLAN.md`. Agent rules: `docs/AGENTS.md`. Stage plans: `docs/stage/English/`.

## Status

Stage 1 (data and tools) done PC-side. Stage 2 Gate 1 fallback applied:
link membership is human-provided from the separated Studio file
(div-vs-Comb displacement, 7 links); joint axes inferred from tool
facts (6 joints). Ground-truth scoring pending human measurement.
Stage 3 done via Day-4 fallback (direct ratios model-derived, wrist
synthesized, J2/743 breaks reported): `runs/stage3/pred_C.json`.
Stage 4 handoff: `docs/stage/English/stage3_dev.md` (read first).
Stage 4 done (Gate 2 PASS): `src/kin/`
(MJCF/FK/numeric IK/motor conversion) + `models/robot_inferred.json`
/`.xml` (public, primitive geometry) + `src/verify/conventions.py`
(sign guard, EE roll gauge); Gate 2 vs user GT v2 100% (median
0.45mm, p95 0.97mm, n=500).
Stage 5 sim-closed (user chose sim-only; hardware deferred):
`src/codegen/` (generator + bring-up runner) +
`config/homing.local.yaml.example` (human calibration) +
`generated/robot_inferred/` (public package) + `runs/stage5/` run kit
(5 coupling-consistent poses, 6 screened Cartesian targets); fake-brick
rehearsal PASS (poses 10 moves, cartesian 6x3) + sim tip errors
(poses 0.000mm, cartesian median 1.92mm / max 4.25mm); 148/148 tests.
Homing directions UNCONFIRMED and zero offsets 0 until a physical run;
blockers (no hardware.local.yaml, no broker, no brick access) in LOG.md.
Blender pose validation (`src/viz/pose_check.py`): 10 sampled poses
articulated from Stage 2 segmentation; support/gear/EE checks pass
(EE <= 0.001mm vs FK), but all 10 self-collide (BVH mesh-confirmed;
hinge neighbours + genuine folds). `poses10.json` carries per-pose
verdicts — none qualify for physical replay; re-screen before hardware.

## Built / External / Pre-existing / Evaluation only / Human-provided

| Category | Items |
|---|---|
| Built during the hackathon | MQTT/Bluetooth brick programs and safety guards, host driver, connectivity check (Stage 0); LDraw parser, part catalog draft, geometry tools, wiring checker (Stage 1, in progress) |
| External libraries | MuJoCo, ev3dev/Pybricks runtime, OpenAI SDK, LeRobot/SmolVLA, LDraw part data (CC BY) |
| Pre-existing asset (private) | Studio model rebuilt from purchased instructions (credited, not published; appears only in the demo video) |
| Published outputs | Abstract kinematic model, closed-form and learned IK, primitive-geometry MJCF, generated EV3 code (see `docs/PLAN.md` section 4a) |
| Evaluation only | Prior analysis results and existing EV3 code constants (never given to the agent; `eval/ground_truth/`) |
| Human-provided | Port wiring (motor/sensor to port), zero offsets, camera calibration (`config/*.local.yaml`); link segmentation from the separated Studio file (div-vs-Comb displacement, Gate 1 fallback, `runs/stage2-div/`); J2 self ratio + motor direction switches th1/th2/th3 (th1 approved 2026-10-29; J2 CAD train cut, `runs/stage3/pred_C.json` section-4, excluded from scoring) |
| Model-derived (Stage 3 wrist) | Wrist coupling columns (736/737) as hand-derived tooth-chain synthesis with the recorded lock law (`runs/stage3/pred_C.json`, solver outputs unused): k4 +0.2, k5 -0.2, k6 -1/7 (R17 frame). Adopted by exception: 812x802 meshed (user-confirmed, +2.4 vs 2.0 LDU gate); 48168/48452 axis labels corrected by mesh fit; knob/bevel/crown signs by unanimous zone rolling-contact proofs; dual J6 pinions consistent. 743 column is model zeros (C4-style re-trace: dead axle + meshless pinion, no fallback line); lock law kept as virtual-diff hypothesis |
