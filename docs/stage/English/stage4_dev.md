# Stage 4 Development Record — Handoff to Stage 5

Read this before starting Stage 5. It carries the entry state, what
Stage 4 built, the conventions that hold, and the traps that must not
be repeated. Companion: `models/robot_inferred.json` + `.xml` (the
public artifact), `runs/stage4_gate2_v2.json` (Gate 2 summary),
`docs/stage/English/LOG.md` (the history). Ground-truth values live
only behind the scorers; this file contains none.

## 1. Stage 5 entry state

- Model: `models/robot_inferred.json` — R17 joints
  (`runs/stage2-chain-r17/links_joints_public.json`), normalized
  coupling from `runs/stage3/pred_C.json`, hardware-layer actuators
  from `config/ports.local.yaml` + motor types via catalog.
  Assembled by `src/kin/assemble.py` (CLI args only, no baked paths).
- MJCF: `models/robot_inferred.xml` — primitive capsules between
  consecutive joint points, base stub, tool stub, `ee` site at the
  J6 point. Public, primitive geometry only.
- FK: `src/kin/fk.py` — product of exponentials, closed-form
  Rodrigues (no scipy in the loop). Matches MuJoCo site to 0.0 mm
  over 20 random poses (gate: < 0.01 mm).
- IK: `src/kin/ik.py` — `ik(target_pos, target_mat, doc, ...)` →
  `(q_deg, success, pos_err_mm, rot_err_deg)`. Numeric DLS only,
  joint-limit clamp, adaptive restarts (8/batch, max 32), low-damping
  refinement. Success = Gate 2 thresholds (5 mm / 3 deg).
- Motors: `src/kin/motors.py` — `joints_to_motors` /
  `motors_to_joints`, limits checked in joint space first,
  zero offsets from `config/zero.local.yaml` (human-provided,
  absent = all zeros). The 743 column of C is zeros (Stage 3 break),
  so conversion is least-squares with a reported residual; 743
  solves to 0.0 (unused). Verified roundtrip 7e-6 deg on a feasible
  pose.
- Eval: `src/verify/ik_eval.py` — 500-sample MuJoCo protocol,
  aggregates only, `--gt` CLI only, with `--no-sign-align` /
  `--no-ee-align` ablation flags.
- Gate 2: PASS on `gt/kinematics_gt_v2.json` — 100.0% success,
  median 0.45 mm, p95 0.97 mm, rot 0.12/0.22 deg. Residual 0.45 mm
  is genuine inference accuracy.

## 2. Conventions that hold for all future work

- GT-blind: `gt/` paths enter code only as scorer CLI arguments;
  probes print error norms / booleans / ID sequences only, never
  coordinates or values — pinned by `tests/test_gt_isolation.py`.
- Tests are synthetic-only: never read `runs/`, `assets/private/`,
  `gt/`, or `eval/ground_truth/` from tests; use `tmp_path`
  fixtures (see `tests/test_stage4.py`, 13 tests,
  `tests/test_stage4_conventions.py`, 8 tests).
- Convention alignment lives in the evaluator
  (`src/verify/conventions.py`), never in the model:
  per-joint sign map `s_i = sign(dot)` with a `|dot| >= 0.5` guard,
  sampled angles mapped by the same signs (identical physics), and
  EE roll-gauge alignment (pred EE shifted along its own roll axis,
  lateral preserved). Only signs and the removed offset scalar are
  reported. Our model files are never tuned to GT (no back-solving).
- PRED_SPEC still binds: self = +1 by column normalization; `+`
  rotation = right-hand about R17 axes; EE default = J6 point unless
  `end_effector.point_mm` says otherwise (the choice is recorded in
  the evaluated model file).

## 3. Findings that constrain Stage 5

- Wrist J4/J5/J6 axes concur within 0.07 mm (`axes_intersect`):
  the wrist IS spherical (J4/J6 coaxial diagonal, J5 vertical
  through the same line at the J5 point). Closed-form decoupling
  was deferred; numeric IK carries Stage 5.
- GT v1 vs R17 differed by wrist sign flips (J5 proven, J4/J6 at
  least one, plausibly all three — same traversal convention) plus
  a 6.14 mm EE roll-gauge offset. GT v2 adopted R17 conventions;
  the gauge is handled by alignment, not by model edits.
- J4 and J6 senses are unobservable through EE motion (EE sits on
  both axis lines; J4 +10 deg moves the EE 0.02 mm). Do not probe
  wrist senses via EE displacement — compare signed axis vectors.
- The 6 mm baseline EE gap alone exceeds the 5 mm bar, so the gauge
  alignment is load-bearing for Gate-style scoring, not cosmetic.
- DLS branch behavior: with a mirrored wrist the solver finds
  exact pred-space solutions on the wrong branch (100% pred-ok vs
  0% GT-ok observed). Refinement selects by pred residual, which
  actively prefers the wrong branch. Any future cross-convention
  eval must align FIRST (it now does).

## 4. Traps found in Stage 4 (do not repeat)

- Orientation error must be space-frame (`R_tgt * R_cur^T`) to match
  the space Jacobian; body-frame (`R_cur^T * R_tgt`) anti-converges
  orientation to ~180 deg while position looks perfect.
- Local minima need seed extension, not just more iterations: the
  stuck cases solved with more restarts, so restarts are adaptive
  (batches to max 32, early exit on success).
- Test fixtures must respect the fixture's own axes (J6 axis is X
  in `make_doc`, not Z); wrong-axis fixtures fail for the wrong
  reason and look like product bugs.
- Running a probe from `/tmp` shadows stdlib (`bisect` vs a stray
  `/private/tmp/bisect.py`): use `PYTHONSAFEPATH=1` with
  `sys.path.insert(0, '.')`.
- The `src.kin` package `__init__` import + `python -m` triggers a
  harmless `runpy` RuntimeWarning; leave it.
- `mujoco` + `scipy` were installed into `.venv` mid-stage (they
  were declared in `pyproject.toml` but missing); keep them pinned
  there for Stage 5+.

## 5. Carried forward (non-blocking)

- Closed-form spherical-wrist IK (position + Euler decomposition)
  is available in principle; implement only if numeric IK limits
  Stage 5 demos.
- Learned IK (MLP on FK samples) was not started (optional per
  stage plan).
- Zero offsets are all default zero (human-provided file absent);
  physical zeroing may shift motor commands in Stage 5.
- Stretch B check: Stage 4 passed — start Stretch B now iff ≥ 3
  days remain before Day 9 (PLAN section 5).
