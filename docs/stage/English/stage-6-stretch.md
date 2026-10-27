# Stage 6: Stretch Goals, "Point to the Coloured Block"

**Latest:** Day 8 · Start an item only when its condition holds.

Task: 3–4 LEGO bricks of different colours lie on the table. A command such as "Where is the blue one?" makes the arm **point at that brick**, hovering above it without touching.

| Item | What decides | Where | Start condition | Effort |
|---|---|---|---|---|
| **A. Physical pointing** | Muse Spark (command) + SAM 3.1 (find brick) + generated IK | Real robot, real camera | Stage 5 done, ≥ 2 days before Day 9 | ≈1 day |
| **B. VLA pointing** | SmolVLA trained by us | MuJoCo | Stage 4 done, ≥ 3 days before Day 9 | ≈1.5 days |
| **B2. Sim → real transfer** | The B policy on the real robot | Real robot | A and B done, ≥ 1 day before Day 9 | ≈0.5–1 day |

## Decisions already made (change only with a reason)

| Topic | Decision |
|---|---|
| Bricks | Four 2×4 bricks: red, blue, green, yellow. 3 or 4 on the table per trial, one per colour |
| Layout | Random positions inside the **pointing area** (found in A.2), ≥ 80 mm apart, random rotation |
| Pointing pose | End effector 60 mm above the brick centre, position-only IK. Approach via a waypoint 120 mm above |
| Hold | 1 s at the pointing pose, then return to rest |
| Rest pose | Outside the camera's field of view (needed for B2) |
| Speed | 30 % of the Stage 5 cap |
| Camera | One fixed camera looking down at the table, 1280 × 720. USB webcam, or iPhone via Continuity Camera (Center Stage off, auto-lock off, on power) |
| Calibration | 4 printed ArUco markers at known positions in the robot base frame → homography (image → table) |
| Success | End effector within **30 mm (horizontal)** of the target brick centre and 40–90 mm above the table at hold time |
| Language | A accepts any language (Spark parses it). B uses English templates only |

---

## Shared setup (do once, ≈2–3 h)

### S.1 Camera mount

```
              [camera] 50–60 cm above the table, looking straight down
                  │
   ┌──────────────┴──────────────┐
   │ M1                       M2 │   M1–M4: ArUco markers at the corners
   │        pointing area        │          of the pointing area
   │   (bricks go here)          │
   │ M3                       M4 │
   └─────────────────────────────┘
              ROBOT BASE
```

- Clamp or tripod; the camera must not move after calibration. Plain light-coloured table surface, even lighting.
- A second phone on a tripod films the robot from the front for the video (not connected to the software).

### S.2 Pointing area · `src/demo/workspace.py`

- Sample joint configurations → FK; keep table points where position-only IK succeeds at both 120 mm and 60 mm above the table.
- Choose the largest rectangle of such points in front of the robot; save `runs/point/area.json` (corners in base frame).
- Print and place the 4 markers at its corners; measure their positions relative to the base (ruler, ±2 mm). Save to `config/camera.local.yaml`.

### S.3 Calibration · `src/demo/calib.py`

- Detect markers (OpenCV ArUco) → homography image → table plane (base frame, mm).
- Camera intrinsics: OpenCV checkerboard calibration (≈30 min, 15–20 shots); then `solvePnP` with the markers → **camera pose in the base frame**. Saved to `config/camera.local.yaml` and reused by the MuJoCo camera in B.
- Check: place a brick at 5 measured spots; homography error ≤ 10 mm.

### S.4 Pointing motion · `src/demo/point.py`

`point_at(xy_mm)`: rest → waypoint (xy, 120 mm) → pointing pose (xy, 60 mm) → hold 1 s → rest. Preflight: IK success for all waypoints, joint steps < 5°, inside the pointing area.

---

## A. Physical pointing (Spark + SAM 3.1)

### A.1 Pipeline · `src/demo/point_physical.py`

```
command text ("파란색 어딨어?" / "Where is the blue one?")
  → Spark: {"colour": "blue"} (structured output)
camera frame
  → SAM 3.1 text-prompted segmentation: "blue LEGO brick" → mask → centroid (px)
  → homography → table xy (mm)
  → show the frame with the mask and target cross; human presses Enter
  → point_at(xy)
```

- Spark: `muse-spark-1.3-contributor`, schema `{"colour": "red"|"blue"|"green"|"yellow"|"unknown"}`. `unknown` → ask again.
- SAM 3.1: via Model API (check `docs/sam/segmenting` for the request format). If several masks return, take the highest score; if none, report "not found".
- Parallax: with a top-down camera, using the mask centroid on the brick top is accurate to a few mm; ignore unless the check in S.3 says otherwise.

### A.2 Commands

```bash
python -m src.demo.workspace --plan
python -m src.demo.calib
python -m src.demo.point_physical      # type a command, confirm, point
```

### A exit criteria

- [ ] 20 trials (5 per colour, random layouts): ≥ 18 successes.
- [ ] Footage of at least 3 trials with the command on screen.

---

## B. VLA pointing in simulation

### B.1 Scene · `src/vla/scene.py`

| Element | Setting |
|---|---|
| Robot | `models/robot_inferred.xml` (primitive geometry only) |
| Table | Plane; pointing area from S.2 |
| Bricks | Boxes 32 × 16 × 11 mm, colours red/blue/green/yellow, 3–4 per scene, ≥ 80 mm apart, random yaw |
| Camera | **Same pose and field of view as the real camera** (from `config/camera.local.yaml`); render 256 × 256 RGB |
| State | 6 joint positions |
| Action | 6 absolute joint targets, 10 Hz |
| Domain randomisation | Brick hue ±8 %, brightness ±20 %, table colour (white/light grey/beige), light direction, camera pose ±2 cm / ±3°, image noise and slight blur |

### B.2 Commands (templates, English)

"Point to the {colour} block." · "Where is the {colour} one?" · "Show me the {colour} brick." Target colour balanced across episodes.

### B.3 Episode (5 s = 50 steps)

| Phase | Time |
|---|---|
| Rest (arm out of view) | 0.5 s |
| Move to waypoint, then pointing pose | 3 s |
| Hold | 1 s |
| Return toward rest | 0.5 s |

Expert = `point_at` from S.4 using the true brick position.

### B.4 Data · `src/vla/generate.py`

- 400 episodes (100 per target colour); 360 train / 40 validation.
- LeRobot dataset format (check the current docs for the writer API). Local `runs/vla/dataset/` → Modal Volume.

### B.5 Preflight (abort on failure)

- Frame 0: all bricks visible, arm not visible.
- Target colour balance within 23–27 % each.
- IK success for every episode; no joint-limit violations.
- Save a 3 × 3 grid of frame-0 images for a human glance.

### B.6 Training on Modal · `src/vla/train_modal.py`

| Setting | Value |
|---|---|
| Base | public SmolVLA base checkpoint |
| GPU | A100 40 GB |
| Steps | 10,000 (stop early if validation loss flattens) |
| Batch | 32 |
| Checkpoints | every 2,000 steps to the Modal Volume |
| **Timeout** | **4 h** |
| Run | detached, unattended |

Check the current LeRobot training CLI and SmolVLA config names first.

### B.7 Evaluation · `src/vla/eval.py`

- 100 new scenes. Two modes:
  - **Closed loop:** policy queried every chunk.
  - **First-frame only:** one query on frame 0, execute the whole chunk (this is the B2 mode).
- Metrics: success rate, **wrong-brick rate** (pointed at another colour), mean horizontal error. Chance ≈ 25–33 %.
- Save a 2 × 3 video grid for the final video.

### B exit criteria

- [ ] Both modes evaluated and reported honestly.
- [ ] Dataset and weights contain only the primitive-geometry robot.

---

## B2. Sim → real transfer

### B2.1 Inference on the Mac · `src/vla/run_real.py`

1. Capture frame 0 (arm at rest, out of view) → resize to 256 × 256.
2. One policy forward pass (MPS or CPU) with the command → action chunk.
3. **Safety check on the chunk:** joint limits, max step < 5°, final end-effector point inside the pointing area and ≥ 40 mm above the table. Reject otherwise.
4. Show the predicted target on the frame; human presses Enter.
5. Execute the chunk through `move_joints` at 10 Hz with guards active.

### B2.2 Real evaluation

- 20 trials (5 per colour), random layouts.
- Measure success automatically: end-effector xy from encoders (FK) vs. brick centre from SAM 3.1 + homography.
- Report sim vs. real success side by side.

### B2.3 Optional: small real fine-tune

- Collect 30 real episodes **automatically**: a human places the bricks; pipeline A (Spark + SAM + IK) acts as the expert while the camera and encoders are recorded.
- Fine-tune on sim 400 + real 30 (Modal, ≈1 h). Re-run B2.2.

### B2 exit criteria

- [ ] 20 real trials done; success rate reported (any value).
- [ ] Footage of at least 2 trials, including a failure if any (honest result).

---

## Token and credit estimate

- A: Spark + SAM 3.1 for ≈40 trials, < $0.50; development $1–3.
- B: Modal $10–25; development $2–5.
- B2: Modal ≈$5 (if fine-tuned); development $1–2.
