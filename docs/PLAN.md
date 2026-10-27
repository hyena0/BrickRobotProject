# Development Plan

Meta Global AI Developer Hackathon (Indie track, solo)
Build window: Oct 26 – Nov 4, 2026. Submission due **Nov 4, 9:00 PM ET** (Nov 5, 11:00 KST). Target: submit by **Nov 4, 12:00 PM ET**.

## 1. Goal

Given only a **brick-level LEGO assembly** (part IDs and poses, with no joints, rig or kinematics defined), a **Muse Spark agent** geometrically infers the robot's kinematic chain: rigid links, joint axes, gear trains, ratios and joint coupling. It then generates forward/inverse kinematics and an executable EV3 control program. The generated IK drives the physical robot and automatically produces demonstrations for training a VLA policy.

Theme: **Push the Frontier**. Meta models used: Muse Spark 1.3 (Model API), SAM 3.1 (Model API, Stretch A), Muse Code (development), Muse Spark Open (optional, Baseten).

## 2. Scope

| Tier | Content | Condition |
|---|---|---|
| **Core** | Studio file → kinematic inference → FK/IK → verification → EV3 runs on the physical robot | Required |
| **Stretch A** | Physical pointing: "Where is the blue one?" → Spark parses the colour → SAM 3.1 finds the brick → generated IK points at it | Stage 5 done, ≥ 2 days left before Day 9 |
| **Stretch B** | VLA pointing: generated IK auto-generates demos in MuJoCo → SmolVLA fine-tune → evaluate in simulation | Stage 4 done, ≥ 3 days left before Day 9 |
| **Stretch B2** | Sim → real transfer of the B policy (first-frame action chunk, safety-checked) | A and B done, ≥ 1 day left before Day 9 |
| Roadmap | Vision-only inference, webcam error compensation, on-device mobile VLA, educational kit | Mention in video only |

## 3. Pipeline

```
[Input]  Studio .io file (part IDs + positions/orientations only)
   ↓ parser + part catalog
[Core]   Muse Spark agent + geometry tools
         get_parts / find_connections / check_gear_mesh / trace_axle
         → link segmentation, joint position/axis/type, gear trains, ratios,
           joint coupling, link lengths
   ↓
[Verify] compare with ground truth + end-effector error in MuJoCo ground-truth model
         → on failure, feed errors back to the agent (optional critic: Muse Spark Open)
   ↓
[Generate] FK/IK with coupling compensation → EV3 control program
   ↓
[Physical] MQTT over Wi-Fi → Left brick (master) ⇄ Bluetooth ⇄ Right brick (slave)
           on-brick safety guards (sensors, stall, heartbeat) → robot motion
[Visual]   Blender MCP overlays of inferred links and axes (development check, video)
   ↓
[Stretch A]  command → Spark (colour) + camera → SAM 3.1 (brick) → homography → IK → point
[Stretch B]  generated IK → MuJoCo pointing demos → SmolVLA training → sim evaluation
[Stretch B2] B policy on the real robot (first frame → one action chunk)
```

### Division of labour inside the agent

- **Tools compute facts** deterministically: geometry, distances, axis parallelism, which parts share an axle.
- **Spark reasons** about what those facts mean: rigid vs. rotational connections, which gear drives which joint, coupling through hollow turntables and differentials, and how to assemble the kinematic chain.
- The agent outputs a structured kinematic model (JSON, schema in `docs/schema.md`).

## 4. Target robot

- 6-DOF arm based on the **6DOF Robotic Arm by MR JOS (Rebrickable MOC-76666)**, purchased, rebuilt brick by brick in Studio, and modified (added gripper; replaced the PS2 controller link with a Wi-Fi setup).
- Joints θ1–θ6, 2 large + 4 medium motors, two EV3 bricks (master/slave).
- Key difficulty: multi-stage gear trains and **θ4 → θ5 → θ6 coupling** through hollow turntables.
- Permission requested from the designer. **Fallback:** public model (e.g., EV3 Education Robot Arm H25); the pipeline stays the same.

## 4a. Publication policy

The submission's deliverable is the **inferred kinematics and the generic inference logic**, not the robot design. The MR JOS-based robot's appearance appears **only in the demo video**, with credit.

| Artifact | Contains | Status |
|---|---|---|
| Studio `.io` / LDraw files, renders | Appearance + assembly | 🔴 Private (`assets/private/`) |
| Agent logs, tool outputs, link segmentation with part IDs/poses | Assembly (reconstructable) | 🔴 Private (`runs/`) |
| Mesh-based ground-truth MJCF | Appearance | 🔴 Private (`eval/ground_truth/`) |
| **Abstract kinematic model** (joint axes, DH/link dimensions, gear ratios, coupling matrix, limits) | Functional facts only | 🟢 Public |
| **Closed-form IK** re-derived from the kinematic model (e.g., spherical-wrist decoupling) | Standard math | 🟢 Public |
| **Learned IK** (pose → joint-angle network from FK samples, no vision) | Kinematic function only | 🟢 Public |
| **Primitive-geometry MJCF** (capsules/cylinders from the kinematic model) | Kinematics only | 🟢 Public |
| Generated EV3 code, all pipeline code | Own work | 🟢 Public |
| VLA dataset and weights (Stretch B) | Must be trained on the **primitive-geometry robot only** | 🟢 Public |
| Results on public models (e.g., H25) | Per that model's license | 🟢 Public if licensed |

Rule of thumb: anything that would let someone rebuild or render the robot stays private; anything that only describes how it moves can be public.

## 5. Stages

Detailed task lists, exit checks and Muse Code prompts for each stage: `docs/stages/`.

Work is **stage-based, not date-based**. Move to the next stage as soon as the exit criteria are met. The "Latest" column is the last day a stage may finish before its fallback kicks in; finishing earlier is the goal.

Day 1 = Oct 27. Day 9 = Nov 4 (submission day, reserved for video and submission).

| Stage | Work | Exit criteria | Latest |
|---|---|---|---|
| **0. Setup** | Repo, `AGENTS.md`, `.gitignore`, **MQTT + Bluetooth brick programs with on-brick safety guards**, Muse Code token check, email designer, apply for Modal/Baseten | Every motor moves and reads back on both bricks; sensor guards stop motion; token cost of one session measured | Day 1 |
| **1. Data and tools** | `.io` parser, part catalog (84 part types), geometry tools, port wiring config, ground truth, Blender + MCP setup | Parser loads all parts; tools return correct results on hand-checked cases; wiring complete | Day 2 |
| **2. Agent v1: links and joints** | Agent does link segmentation + joint identification | 🔶 **Gate 1:** segmentation and joint count/axes match ground truth within tolerance | Day 3 |
| **3. Gears and coupling** | Gear-train tracing, ratios, joint coupling (θ4→θ5→θ6) | Per-joint ratios and coupling match ground truth | Day 4 |
| **4. Kinematics** | Kinematic chain → FK/IK, end-effector error in MuJoCo | 🔶 **Gate 2:** IK success rate and end-effector error within tolerance over random targets | Day 5 |
| **5. Physical** | Generate and deploy EV3 program; physical motion | Robot reaches a set of commanded targets on camera | Day 6 |
| **6. Stretch** | Stretch A, B, B2 (see section 6), run in parallel where possible | Per stretch item | Day 8 |
| **7. Generalization** | Run the pipeline on a public model | Pipeline runs end to end; accuracy reported | Day 8 |
| **8. Ship** | Filming, 3-minute video, README, submission | Submitted | **Day 9, noon ET** |

### Rules for moving between stages

- **Done early → start the next stage immediately.** Unused days roll forward to stretch goals.
- **Late → apply the fallback at the "Latest" day**; do not keep pushing.
- **Stretch go/no-go** is decided when Stage 4/5 finishes, based on days left:
  - Stretch B needs **≥ 3 days** left before Day 9 (start right after Stage 4; it can run in parallel with Stage 5).
  - Stretch A needs **≥ 2 days** left before Day 9 (start after Stage 5).
  - Stretch B2 needs A and B done and **≥ 1 day** left before Day 9.
  - Not enough days → skip, go to Stage 7.
- Long-running jobs (training, batch evaluation) run unattended in the background while the next stage proceeds.
- Stage 8 always keeps one full day, regardless of progress.

### Fallbacks

- **Gate 1 fails** (links/joints not identified): give link segmentation from the separated file as input; focus inference on joints, gears and IK.
- **Gate 2 fails** (end-effector error too large): show only simple physical motion; build the demo around gear-ratio/coupling accuracy. Cancel Stretch A/B.
- **Designer declines** (or no reply by Day 5): switch to a public model; pipeline unchanged.

### Fixed external events

- "Agents 102: Best Practices for Verifiable Agents" (Day 2, Oct 28, 12:00 PM ET); recording if missed.
- Discord office hours on weekdays, 10–11 AM ET.
- Last day for organizer questions: Day 8 (Nov 3).

## 6. Stretch details

Task: 3–4 differently coloured LEGO bricks on the table; "Where is the blue one?" → the arm points at that brick (hovers 60 mm above it, never touches). Full settings in `docs/stages/stage-6-stretch.md`.

- **Shared setup:** fixed top-down camera, 4 ArUco markers in the robot base frame → homography; camera pose measured once and reused by the MuJoCo camera; pointing area found from the inferred workspace.
- **A (≈1 day):** Spark extracts the target colour from any-language command → SAM 3.1 text-prompted segmentation finds the brick → homography → IK → point. Human confirms with Enter. Target ≥ 18/20 successes.
- **B (≈1.5 days):** MuJoCo scene with the **primitive-geometry robot**, camera matching the real one, domain randomisation; 400 IK-generated episodes; SmolVLA on Modal A100 (timeout 4 h); evaluated closed-loop and first-frame-only.
- **B2 (≈0.5–1 day):** run B on the real robot from the first frame (arm out of view) as one safety-checked action chunk; 20 real trials; optional fine-tune with 30 real episodes collected automatically using pipeline A as the expert.

## 7. Verification metrics

| Stage | Metric |
|---|---|
| Link segmentation | Agreement with the separated file |
| Joints | Joint count, position error (mm), axis angle error (°) |
| Gears | Per-joint ratio match (ground truth: prior analysis + existing code constants) |
| IK | End-effector error and success rate over hundreds of random targets (MuJoCo ground-truth model) |
| Physical | Reaches commanded targets (video) |
| Pointing (A) | Success within 30 mm horizontal of the target brick, 20 trials |
| VLA (B, B2) | Success rate, wrong-brick rate, mean error; sim vs. real side by side |

## 8. Budget (Contributor mode, reasoning effort medium)

| Item | Estimate | Source |
|---|---|---|
| Muse Code development | $10–30 | Meta $150 |
| Spark agent + verification loop | $5–15 | Meta $150 |
| Stretch A (Spark + SAM 3.1) | < $1 | Meta $150 |
| Stretch B / B2 training | $10–30 | Modal $100 |
| Critic / model comparison | small | Baseten $100 (optional) |
| **Meta credit total** | **≈ $15–45 of $150** | |

Pricing (per 1M tokens): `muse-spark-1.3-contributor` input $0.10, cached $0.002, output $0.20 (reasoning tokens count as output). Rate limit: 150 RPM, 3M TPM.

## 9. What is new vs. external (for README)

| Category | Items |
|---|---|
| **Built during the hackathon** | `.io` parser, part catalog, geometry tools, Spark agent, verifier, ground-truth MJCF, FK/IK + EV3 code generator, MQTT/Bluetooth brick programs and safety guards, Blender overlays, pointing demo, VLA data generator |
| **External libraries** | MuJoCo, ev3dev/Pybricks, OpenAI SDK, LeRobot/SmolVLA, LDraw part data (CC BY) |
| **Pre-existing asset (private)** | Studio model rebuilt from the purchased MR JOS 6DOF instructions (credited, not published; appears only in the demo video) |
| **Published outputs** | Abstract kinematic model, closed-form and learned IK, primitive-geometry MJCF, generated EV3 code (see section 4a) |
| **Evaluation only** | Prior analysis results and existing EV3 code constants (never given to the agent) |
| **Human-provided** | Port wiring (motor/sensor → port), zero offsets, camera calibration; link segmentation only if Gate 1 fallback |

## 10. Risks

| Risk | Mitigation |
|---|---|
| EV3 connectivity | Driver first, on day 1 |
| Inference accuracy | Gate 1 fallback; strengthen tools so exact computation lives in tools |
| Token cost | Short per-task sessions, unattended scripts for training/generation, check usage daily |
| Hidden premise errors | Preflight checks before every generation/training run; pointing instead of grasp physics |
| Model rights | Model files private, credit designer, permission requested |
| Deadline | Submit by noon ET Nov 4; keep a full day for the video |

## 11. Video (3 min)

1. **Problem** (20 s): LEGO arms are easy to build, hard to control when gears are coupled.
2. **Inference** (60 s): Spark finds links, joints, gear trains and coupling from brick data alone.
3. **Verification** (25 s): accuracy vs. ground truth + simulated end-effector error.
4. **Physical** (35 s): generated program moves the robot; "Where is the blue one?" → the arm points at it.
5. **VLA** (20 s): VLA trained on IK-generated data points in simulation, then on the real robot if B2 is done.
6. **Roadmap** (20 s): vision-only inference, error compensation, mobile VLA, educational kit.

## 12. Rules checklist

- [ ] Public repo; all implementation commits after Oct 26
- [ ] Model files private; designer credited
- [ ] README separates built / external / pre-existing / evaluation-only / human-provided
- [ ] No Streamlit; no dashboard as the main feature
- [ ] No "LEGO" trademark in the project name or logo
- [ ] Video shows only what was built during the hackathon
