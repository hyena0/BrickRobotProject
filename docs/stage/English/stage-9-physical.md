# Stage 9: Physical Session (IK control + VLA webcam test)

**Status: added stage.** Gathers all hardware work into one session. Replaces the physical items left in the 5C and Stage 6 docs, and the "digital twin" VLA test from the earlier Stage 9 draft.

**Principle:** focus on **what works well**, not complete verification; secure success shots and numbers. Record failures briefly and move on.

**Filming:** per the MR JOS designer's permission, film the physical robot clearly in the video. Do not put photos or video files in the repo (store under `runs/` only).

| Part | Content | Real bricks | Priority |
|---|---|---|---|
| A | IK control (console → robot) | needed | 1 (once the port issue is solved) |
| **B** | **VLA webcam test (decision only)** | **not needed** (fake brick) | **1 (do today for sure)** |
| C | (optional) fine-tune with real photos | not needed | 3 |
| D | (optional) move the real robot from B's result | needed | 4 |

---

## A. IK control (~60 min)

### A.0 Port-drop precautions (before starting)

- **Adapters on both bricks** (avoid voltage sag)
- Reseat every cable fully
- **Remove the color sensor** from console safety rules
- If a problem occurs, save `ssh robot@<brick IP> 'dmesg | tail -40'` and Read state (`latch_reason`, `missing`) right away

### A.1 Connect and calibrate

1. Broker (`scripts/start_broker.sh`) → left → right brick → console in real mode
2. **Start from the post-calibration pose:** load saved calibration or restore from park → compare screen and robot at the check pose; redo only joints that are off

### A.2 IK motion

- Start with mat points that had small error in simulation: click → preview → send to robot → rest pose
- Goal: 3–5 successes. Record and skip points that are off
- Film one e-stop

**Pass:** robot reaches 3+ mat points + footage. **If the port issue blocks progress, use the fake-brick footage for A and move to B.**

---

## B. VLA webcam test (decision only, ~2 h)

**Goal:** measure how well the VLA, trained only in simulation, decides on **real camera images**. The robot moves only as the fake brick (console 3D view).

**Expectation:** success on virtual images was 41 %. On real images it is likely lower, near chance (25–33 %). Even if low, record it honestly as **a measurement of the sim-to-real gap**.

### B.1 What you need

| Item | Detail |
|---|---|
| Camera | USB webcam or iPhone (Continuity Camera) |
| Mount | tripod or stand, 50–60 cm above the stand, looking down |
| Stand surface | plain light paper (keep the mat's text and lines out of view) |
| Blocks | 2×4 bricks: red, blue, green, yellow (3 colours if one is missing) |
| Checkpoint | step 6,000, downloaded locally, **first-frame mode** |

### B.2 Camera check (10 min)

1. List connected cameras (command from Muse)
2. Capture one frame → confirm it is saved in `runs/stage9/webcam/`
3. On macOS, allow **camera access** for Terminal/VS Code (System Settings → Privacy & Security → Camera)

### B.3 Camera alignment (30 min)

1. Mount the camera at the pose in `config/sim_camera.yaml`
2. Use the **overlay tool** to blend the real photo with the virtual image of the same layout
3. Adjust until slot S1–S6 centres differ by **about 1 cm or less**
4. Do not touch the camera afterwards

### B.4 Trials (40 min)

Test several instructed colours per layout to save time.

1. Place 3–4 blocks on slots (e.g. `red=S2, blue=S5, green=S1`)
2. Enter the layout in the test tool → choose the colour → capture → record the slot the VLA chose
3. The same layout also runs on the virtual image (automatic)
4. Test 3–4 colours per layout, then change the layout
5. **5–10 layouts, 20–30 trials in total**

> **Correction 2026-11-02 (single-call bug, retraction).** The earlier
> session note "the policy collapses to one pose regardless of target"
> (1/3 on layout 1) is withdrawn as a policy verdict. The trials called
> `select_action` once and read the single returned action, which is
> always the rest-side first action of the queued chunk; the real
> first-frame protocol drains all 50 queued actions and reads the last
> one. Cause, not the checkpoint: fixed by the shared
> `src/vla/drain.py` drain used by every consumer. The 6.x first-frame
> figure (ckpt 6000: 41%) stands. Re-score B.4 after the fix before
> judging the policy.

> **Result 2026-11-02 (eval-ft, fixed drain).** Real held-out test
> (5 batches x 3 colours = 15 samples): before 0/15, after **15/15**
> at 4.1 mm mean error. Virtual 100 fresh scenes: 43% -> **46%**.
> Conditions: camera fixed at the overlay-aligned pose
> (`config/sim_camera.yaml` unchanged), 6 slots S1-S6, 3 physical
> colours (red/yellow/green; no blue blocks on hand), same paper-tray
> StreamCam setup with the center-square-crop pipeline.
> Non-leakage: sha256 of the 5 test photos against all 65,000
> training frames = 0 exact matches (nearest perceptual hash 2-5/64
> bits, composition similarity only). Follow-up: 5 extra batches in
> slightly different lighting as `holdout` split (never training),
> scored with `stage9_eval_real.py --splits holdout`.

> **Final 2026-11-03 (wrap-up, no further training).**
> | Condition | Before | After |
> |---|---|---|
> | Virtual 100 scenes | 43% | 46% |
> | Real same setup 15 | 0/15 Modal, 1/15 local | 15/15 (4.1 mm) |
> | Real new lighting 14 | 3/14 | 3/14 |
> | Random baseline | 0% virt, 33% real | same |
>
> Local and Modal evals agree on the same setup (the one-sample gap
> is flow-matching sampling noise: batch 12 green passes locally,
> misses on Modal). Checkpoints: before
> `/data/train-20261031-203906/checkpoints/006000/pretrained_model`
> (= local `runs/vla/ckpt`), after
> `/data/ft9b-20261102-132927/checkpoints/003000/pretrained_model`
> (= local `runs/vla/ckpt-ft9b`, dir verified; exact local --b string
> truncated in the pasted command). Code at commit `ca87c7d`
> (dirty tree), eval-ft 2026-11-02, local evals 2026-11-03 session.
> Camera fixed (pos [-133,-60,550], look [-133,235,0], fov 22, 256px,
> center-square crop, yaml unchanged). Photos: train 15 batches
> (45 samples), test 5 batches (15), holdout batches 21-25 (14;
> lighting slightly changed, details unrecorded).
> Full rows: `runs/stage9/eval_real_local_test.json`
> (holdout file pending the pasted output).
>
> Conclusion: local and Modal evals agree. Same setup improved
> 0~1/15 -> 15/15, but new lighting gives 3/14, same as before
> fine-tuning. A small real dataset only fits its shooting setup;
> varied-setup data is the next task.
>
> Preservation: do NOT delete `runs/vla/ckpt`, `runs/vla/ckpt-ft9b`,
> or the Volume runs `train-20261031-203906`, `ft9b-20261102-132927`.

| Record (per trial) | |
|---|---|
| Layout, instructed colour | |
| Slot chosen on the real photo, correct? | |
| Slot chosen on the virtual image, correct? | |
| Notes (lighting, shadows, etc.) | |

### B.5 Filming

Screen-record one scene showing **real webcam image → slot chosen by the VLA → pointing pose in the console 3D view**. 1–2 successes, 1 failure.

**Pass:** 20+ trials recorded + a real vs. virtual success comparison table + at least one clip.

---

## C. (optional) Fine-tune with real photos (~2 h, Modal ~$1–2)

Only if B is low and time remains.

1. Shoot 10–15 layouts; attach 3–4 instructed colours each for **30–50 photo samples**. Expert actions come automatically from IK using the known slot positions (no human labelling)
2. Mix with the existing 400 virtual demos and fine-tune briefly on Modal (after passing the Modal check gate)
3. Re-run B.4 and compare before and after

## D. (optional) Move the real robot

Only if A's port issue is solved: send the hold pose chosen in B to the real robot slowly, after safety checks. Aim for one success shot.

---

## E. Wrap-up (10 min)

1. Save the park pose and shut down the bricks (if A was done)
2. Records: `runs/stage9/<date>/` (IK point table, VLA webcam table, notes)
3. Name footage files by scene

## Stop rules

| Situation | Response |
|---|---|
| Port drops keep happening | stop A → save `dmesg` and Read state → move to B |
| Camera alignment won't get within 1 cm | accept up to 2 cm and continue; record the offset in the results |
| B is near 0 % | first suspect preprocessing (crop, size, RGB/BGR order) and ask Muse to check. If unchanged, record it and finish |
