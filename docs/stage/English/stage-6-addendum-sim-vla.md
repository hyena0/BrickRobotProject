# Stage 6 Addendum: Simulation VLA First (current situation)

**Status: add-on document.** `stage-6-stretch.md` (original Stage 6) and `stage-6b-vla-generated.md` stay unchanged; this document takes precedence. The Stage 6 session reads this first and otherwise follows **section B (VLA in simulation)** of the original Stage 6 doc.

## 1. Scope decision

| Item | Original | Now |
|---|---|---|
| A. Physical pointing (Spark + SAM + camera) | stretch | **on hold** (no time for camera setup and calibration) |
| **B. VLA in simulation** | stretch | **in progress** |
| B2. Sim → real | stretch | **on hold** (needs real camera calibration S.3) |
| 6B v2 (generated robot) | draft | **not adopted** (5D on hold) |

## 2. What matches original Stage 6 B (follow as is)

- Robot: `robot_inferred` primitive MJCF (no meshes)
- 3–4 coloured bricks, ≥ 80 mm apart, random yaw
- English command templates (B.2), episode layout 5 s / 50 steps (B.3)
- Expert = IK-based `point_at`, 400 episodes in LeRobot format (B.4)
- Preflight (B.5), Modal training settings (B.6), two evaluation modes and metrics (B.7)
- Dataset and weights contain primitives only

## 3. Changes from the original

| Item | Original Stage 6 | Change | Reason |
|---|---|---|---|
| **Pointing plane** | table surface, 60 mm above the brick | **top of a stand** (height chosen in 5C), 60 mm above | the 5A coupled constraint keeps the TCP above ≈212 mm over the table |
| **TCP** | position-only IK (wrist centre) | **gripper tip** (fixed offset in the J6 frame, ≈56 mm), optional "gripper pointing down" constraint | with the wrist centre the gripper misses by ≈56 mm |
| **Pointing area (S.2)** | computed on the table plane | computed on the stand plane; reuse the 5C mat points and reach computation | same reason |
| **Camera** | copies the real camera pose (S.3) | **fixed virtual camera pose** (50–60 cm above the stand, looking down), `config/sim_camera.yaml` | real calibration skipped; replace with S.3 if B2 is done |
| Domain randomisation | for sim → real | keep light (brightness, colour, camera ±1 cm) | in case B2 is done later |
| **Limits and coupling** | not mentioned | console, IK and data generation use **the same limits function** | avoid the 5C bug from two limit paths |
| Action | 6 joint targets | keep 6; J6 constant | J6 fixed in position IK |
| Success | ≤ 30 mm horizontal from the brick centre, 40–90 mm above the table | ≤ 30 mm horizontal, 40–90 mm **above the stand** | plane change |
| **Smoke test** | none | **small smoke test** before full training (6.2) | cost protection |
| Upload guard | primitives only | automatic upload-list check: abort if `assets/private/` or MR JOS meshes/renders are present | privacy rule |
| Done | report metrics | done only with **a table and image evidence** | 5C lesson |

## 4. Steps

| Step | Content | Where | Pass | Time box |
|---|---|---|---|---|
| **6.0 Scene** | stand plane, bricks, virtual camera, shared limits function | local | 9 sample renders (3×3), all bricks visible, arm out of view | 1.5 h |
| **6.1 Expert** | `point_at` (gripper tip, waypoint 120 mm → 60 mm → hold 1 s → rest) | local | success ≥ 95 %, colour balance 23–27 %, 0 limit violations | 1.5 h |
| **6.2 Smoke test** | 20 episodes, 200 steps, cheap GPU, 30 min timeout | Modal | train → save → load → one inference complete; cost and time recorded | 1 h |
| 6.3 Full training | 400 episodes (360/40), original B.6 settings | Modal | validation loss falls, checkpoints saved | 4 h (unattended) |
| 6.4 Evaluation | 100 new scenes, closed loop + first-frame only | local/Modal | success, wrong-brick rate, mean error, video grid | 2 h |

**Gates**

- 6.1 fails: stop the VLA; show the IK expert videos only.
- 6.2 fails, or cost is ≥ 2× the estimate: report the cause; no full training.
- 6.3 not started by Day 7 morning: move the VLA to the roadmap.

## 5. Budget

| Item | Estimate | Note |
|---|---|---|
| Muse development (Meta credit, ≈$50 left) | $2–5 | **contributor + medium required.** On Standard pricing one session can exceed $20 |
| 6.2 smoke test (Modal) | < $1 | |
| 6.3 full training (Modal, A100 4 h) | $8–15 | set a spending limit |
| 6.4 evaluation | $0–3 | can run locally |

## 6. Muse prompts

The step-by-step prompts are in a separate document, `prompts-6-sim-vla.md`.

## 7. Physical test approach

Training uses virtual camera images, so the physical test uses a **digital twin**. Real bricks are placed on mat points and the same layout is entered into the simulation; the VLA produces joint commands from the virtual image, and those commands are sent to the real robot. The video and README state that no real camera images were used. B2 (with a real camera) stays on hold.
