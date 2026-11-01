# Stage 6 dev memo (sim-VLA first, addendum priority)

## 6.0 scene — PASS (2026-11-01)
- 9/9 frames: all blocks visible, arm out of frame.
- Virtual camera: pedestal +550 mm, tilted 28° from vertical toward
  the mat (`config/sim_camera.yaml`). Straight-down framing cannot
  exclude the robot base; the tilt keeps the base and the rest pose
  out while all block corners stay in.
- Pointing area = left half of the mat: 2x3 lattice slots S1..S6
  (`src/vla/scene.py:SLOT_IDS`, front row near the robot first).
- Shared range function: `src.console.control` (stage table, tip
  offset, adopted ranges) — no second definition.

## 6.1 expert demos — PASS, 100/100 (2026-11-01)
- Episode 50 steps: rest 5 (0.5 s), approach 30 (3 s), hold 15 (1.5 s).
  No fast return leg; the episode ends holding the point.
- Success 100/100, colour 25/25/25/25, range violations 0,
  hold error max 0.0 mm, max step med/max 3.49/4.12 deg (all ≤5).
- TCP = gripper tip, position IK, J6 pinned per episode (REST_J6 60
  when valid, else nearest mat-seed J6 — one global pin admits no
  coupled-valid solution on some slots, verified).
- Reports: `runs/vla/6.0/report.json`, `runs/vla/6.1/report.json`;
  videos `runs/vla/6.1/demo_00..02.mp4`.

## Slots (physical)
- Overlay sheet: `runs/vla/slots/slots_mat_a3.pdf` (A3, 1:1, with a
  100 mm scale bar). Print at 100%, align the numbered 1..9 crosses
  with the existing mat print, place real blocks on S1..S6 only.
- Console 3D view shows S1..S6 labels (`/api/mat_points` → `slots`).

## Digital-twin execution (decision)
- The VLA trains on virtual-camera images, so the real test does not
  use the real camera image at all.
- Real test: put real blocks on the mat slots, enter the same layout
  in sim, run the VLA on the virtual image, take ONLY the hold pose
  (the final pointing joints) from the VLA output, and send it via
  the console at low speed (`/api/goto`, guards on).
- The full 50-step VLA action chunk is never executed on the real
  robot. README and videos state: "no real camera image used".

## 6.2 trial — PASS (2026-11-01, T4)
- 200 steps, loss 0.104, 3.55 GB, 3.0 s/step. Save -> load ->
  single inference OK (pre/post processors from the ckpt).
- Single-camera fix (verified in lerobot 0.6.1 code): rename_map
  top -> camera1 + empty_cameras=2.

## 6.3 full training — done (2026-11-01, A100 40GB)
- 400 demos (360 train / 40 val), batch 32, 6000 steps, ckpt every
  1000, 37 min. Final loss 0.010-0.013.
- Volume ckpt:
  `/train-20261031-203906/checkpoints/last/pretrained_model`.

## 6.4 eval — 100 unseen scenes (2026-11-01, T4)

| Policy | Mode | Success % | Wrong-block % | Mean err mm |
|---|---|---|---|---|
| expert (6.1 pipe) | — | 100.0 | 0.0 | 0.0 |
| random | — | 0.0 | 5.0 | 139.6 |
| ckpt 3000 | closed | 21.0 | 40.0 | 84.2 |
| ckpt 3000 | first-frame | 38.0 | 45.0 | 57.1 |
| ckpt 6000 | closed | 22.0 | 43.0 | 81.0 |
| ckpt 6000 | first-frame | 41.0 | 45.0 | 55.8 |

- Success = tip within 30 mm of the target, z 40-90 mm above the
  pedestal. Scenes have unique colours per block, target always
  present — no scoring ambiguity.
- Report: `runs/vla/eval/eval-20261101-023150/report.json`
  (Volume: `/data/eval-20261101-023150/`).
- Videos: `grid_3000.mp4`, `grid_6000.mp4` (2x3 closed-loop) plus
  `ckpt_3000_frames/ep*.mp4`, `ckpt_6000_frames/ep*.mp4` in the
  same dirs.

## Instruction strings (train vs eval)
- Train (`src/vla/to_lerobot.py`, i % 3): "Point to the red block."
  / "Where is the red one?" / "Show me the red brick."
- Eval (`src/vla/trial_modal.py`): always "Point to the {target}
  block." — template #1, in-distribution. Colour words identical.
- Obs plumbing matches the proven infer path (raw "task" string
  through the saved pre/post processors). No string/key bug found.

## Failure analysis
- The policy reaches A block in ~65% of episodes (22% right +
  43% wrong) but picks the wrong colour in 43-45%. Visuomotor
  works; colour grounding does not.
- First-frame beats closed-loop (41% vs 22% @6000). Replanning
  every step from a slightly-off state diverges; the single
  open-loop drain from rest follows the demo shape better.
- Decider done: `--which sens` (2026-11-01, T4, 20 scenes x 4
  instructions, 6000 ckpt). Median spread 9.8 mm, identical 0.0,
  named-hit 0.0 -> verdict **weak**. Full data:
  `runs/vla/sens-20261101-145902.json`.
- Verdict: **learning limit, not a plumbing bug**. The instruction
  moves the output (~10 mm spread, so the language path functions),
  but the named colour never lands on its block. Suspects: 360
  demos, VLM cut to 16 layers, minimal domain randomization.
  Retraining is the user's call.

## Limits
- 360 training demos, 4 colours, fixed virtual camera, blocks only
  on S1..S6. No domain randomization beyond +-10 mm camera jitter.
- Real-robot execution deliberately excluded from this stage
  (spare-day work, see `stage6_physical_plan.ko.md`).
