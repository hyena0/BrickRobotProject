# Development Plan: Update v2

**Date:** Oct 29, 2026 (Day 3) · **Base:** `PLAN.md` (kept unchanged as v1)

This update (1) inserts three stages between Stage 5 and Stage 6 and (2) defines the **controller package** so control can run standalone on a PC or a mobile device. Everything in `PLAN.md` still applies unless this file says otherwise. If the two disagree, **v2 wins**.

## 1. Why the update

- Stage 5 simulation work succeeded (generated code, rehearsal, Blender pose check), but **joint limits were never inferred**: 10/10 random poses self-collide.
- The physical side was never run: no broker, no brick connection, homing directions unconfirmed.
- Stage 6 A and B2 assume a working, safe physical robot. A dedicated step was missing between 5 and 6.
- The broker and all control logic were tied to the PC, so a Pad–robot setup would need a new server. PC and mobile runtimes differ (mobile cannot run Python services, external broker programs or background servers the same way), so the control side must be split into platform-neutral and platform-specific parts from the start.

## 2. Inserted stages

| Stage | Name | Doc | Requires | Latest |
|---|---|---|---|---|
| **5A** | Joint Limits and Safe Poses in Simulation | [stage-5a-sim-limits.md](docs/stages/stage-5a-sim-limits.md) | Stage 5 sim work | Day 4 |
| **5B** | Hardware Connectivity (parallel track) | [stage-5b-hw-connectivity.md](docs/stages/stage-5b-hw-connectivity.md) | none (can start any time) | Day 4 |
| **5C** | Robot Control Program and Web Console | [stage-5c-control-console.md](docs/stages/stage-5c-control-console.md) | 5A; 5B for physical steps | Day 6 |

Korean versions: same names with `.ko.md`. Muse Code prompts for these stages: [prompts-5abc.md](docs/stages/prompts-5abc.md).

### What each stage owns

- **5A (sim only):** sensor boundaries from geometry, collision boundaries, coupled constraints (e.g., J2–J3), enforcement in IK and generated code, `poses10_safe.json`. Limits and coupled constraints are written as **robot package** fields (section 5).
- **5B (hardware only):** MQTT broker on the **hub device**, master Wi-Fi + slave Bluetooth, robot-agnostic brick programs, on-brick guards (sensor, homing, stall, motor-angle limits, heartbeat, Bluetooth), host driver, connectivity and timing checks. Bricks connect to a configurable hub address; MQTT account `hub`.
- **5C (controller package + physical validation):** controller package built in four layers (section 5), local web UI (no Streamlit), wizards for homing, direction check, physical limit capture from real sensors, safe pose replay, Cartesian test; reports and side-by-side images; conformance suite export.

## 3. How existing stages are reinterpreted (docs unchanged)

| Existing | v2 interpretation |
|---|---|
| Stage 0 tasks 0.2–0.6 (brick programs, guards, driver, check) | Code may already exist; **physical verification moves to 5B** |
| Stage 5 tasks 5.1–5.2 (codegen, offsets) | Done at simulation level; offsets are measured in 5C |
| Stage 5 tasks 5.3–5.4 (bring-up, measurement) | **Carried out in 5C** through the console wizards |
| Stage 5 "Stretch A check" | Replaced: Stretch A starts after **5C** |
| "PC broker" / account `pc` (Stage 0, 5B) | **Hub broker** / account `hub` |

## 4. Updated stage sequence

```
0 → 1 → 2 → 3 → 4 → 5 (sim) → 5A ─┐
                                   ├→ 5C → 6 → 7 → 8
5B (parallel, from any day) ───────┘
```

| Stage | Latest (v2) |
|---|---|
| 5A | Day 4 |
| 5B | Day 4 |
| 5C | Day 6 |
| 6 (Stretch) | Day 8 |
| 7 (Generalization) | Day 8 |
| 8 (Ship) | Day 9, noon ET |

## 5. Controller package (PC or mobile, standalone)

All control (broker, control logic, UI) is packaged to run on **one device**, either a PC or a mobile device (iPad, phone), giving standalone **PC–robot** or **Pad–robot** setups.

```
┌──────────────────────────────────────────────┐
│ ④ UI (HTML/JS)                                │ PC: browser · Mobile: WebView in an app shell
├──────────────────────────────────────────────┤
│ ② Core logic (platform-neutral)               │ FK/IK, limits, coupled constraints, path check,
│                                               │ synchronized moves, homing sequence,
│                                               │ wizard state machines, protocol encode/decode
├──────────────────────────────────────────────┤
│ ③ Adapters (per platform)                     │ Transport · Storage · Lifecycle
├──────────────────────────────────────────────┤
│ ① Robot package (data, JSON)                  │ kinematic model, limits, coupled constraints,
│                                               │ port wiring, offsets, safe poses
└──────────────────────────────────────────────┘
            ↕ MQTT (same topics and payloads on every platform)
   EV3 bricks: device-agnostic; final safety decisions stay on the bricks
```

- The **hub** is the single device running the controller package: it hosts the broker and holds control. Exactly one hub per session.
- PC-only **authoring tools** (inference agent, Blender, MuJoCo, code generation, BVH collision analysis) stay separate; their output is the robot package.

### ① Robot package

- One versioned JSON bundle (`robot_package.json` + schema version) produced by the authoring tools.
- Contains only runtime needs: abstract kinematics, `limits_deg`, `coupled_limits`, actuator and sensor port map, homing config, zero offsets, safe poses, speed caps.
- No meshes or part data: light for mobile and consistent with the publication policy (PLAN v1 section 4a). Package content constraints and a validator are defined in Stage 5A.
- **Private visual pack:** real LEGO meshes per link (`assets/private/visual_pack/`, git-ignored) for development, testing and the demo video. Shown in the console only with `--private-visuals`; excluded from every distribution by `src.release.check`.
- Runtime collision safety uses the **precomputed limits and inequalities from 5A**, not mesh checks.

### ② Core logic

- Pure computation and state machines. **No direct file, network or OS calls**; all I/O goes through adapters.
- Hackathon implementation: Python (PC).
- **Conformance suite:** the Python core exports golden cases as JSON (FK/IK inputs and outputs, limit and path verdicts, protocol messages, wizard state transitions, safety rejections). Any future implementation (e.g., mobile) must pass all of them. This is the porting contract.

### ③ Adapters

| Adapter | PC (hackathon) | Mobile (roadmap) |
|---|---|---|
| Transport | External Mosquitto on the PC | Broker embedded in the app |
| Storage | File system (`config/`, `runs/`) | App sandbox storage |
| Lifecycle | Normal process | Backgrounding or screen lock stops heartbeats → bricks stop the robot |

### ④ UI

- Plain HTML/JS, all assets local. Same UI code in a PC browser and a mobile WebView.
- E-STOP always visible; press-and-hold jog; touch-friendly layout.

### Brick side (protocol unchanged)

- Bricks connect to a configurable **hub address** (fixed IP or local `.local` name).
- MQTT account `hub`; ACL rules otherwise as in 5B.
- Safety stays on the bricks for any hub: sensor guards, motor-angle limits, heartbeat watchdog (covers mobile backgrounding), Bluetooth watchdog, latch.

### Scope

| Item | Hackathon | Roadmap |
|---|---|---|
| PC–robot controller package (Python core + Mosquitto + web UI) | ✅ | |
| Four-layer separation and robot package format | ✅ from the start | |
| Conformance suite exported from the Python core | ✅ | used to verify the mobile core |
| Brick hub addressing and `hub` account | ✅ | |
| Pad–robot controller package (app shell, embedded broker, core re-implementation) | design notes only | ✅ |

## 6. Updated rules

- Stretch A (physical pointing) starts after **5C**, with ≥ 2 days left before Day 9.
- Stretch B (VLA in sim) may start after **5A**, with ≥ 3 days left; it does not need hardware.
- Stretch B2 (sim → real) needs A, B and 5C done, with ≥ 1 day left.
- **5C physical steps cannot start before 5B passes.** If 5B is not done by end of Day 4, stop other work and fix connectivity first.
- Stage 7 may reuse the 5C console in `sim` mode to load procedural robots.
- **`AGENTS.md` rule to add:** core logic must not call files, network or OS directly; use adapters. The robot package is the only runtime input from authoring tools.

## 7. Updated fallbacks

- 5A coupled fit too conservative → grid lookup table.
- 5B MQTT unstable → same messages over TCP; Bluetooth unstable → slave on Wi-Fi.
- 5C UI late → same wizards from CLI; finish UI later.
- 5C measured vs. simulated sensor boundary differ by > 10° → operate on measured value, flag the joint, re-check Stage 2.7.
- Layer separation slows 5C → keep the robot package format and conformance export; simplify adapters to the PC implementations only.

## 8. Budget delta

| Item | Added estimate |
|---|---|
| 5A development | $3–6 |
| 5B development | $1–3 (mostly done in Stage 0) |
| 5C development | $5–10 |
| Layer separation and robot package export | $1–2 |
| Conformance suite export | $1–2 |
| **New Meta credit total (v1 + v2)** | **≈ $27–70 of $150** |

## 9. README / video additions

- README "Human-provided" now also lists: measured joint limits (5C), zero offsets, camera calibration.
- README: controller package diagram; "PC–robot today, Pad–robot next" with the conformance suite as the porting contract.
- Video: the console's limit-capture moment and the render | photo side-by-side from 5C; roadmap shot of the same robot package and UI on a tablet.
- The web console is a validation tool, not the product's main feature.
