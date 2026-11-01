# Stage 7 dev log (T-Bot generalization)

Target: Rebrickable MOC-85202 "Gantry Robot T-Bot", designer glaysche
(Technic 2021; page notes it was inspired by MOC-80519/Mr_Jos t-bot gantry).
Licence: no open licence stated (page fetch 403; search snippet only).
Until terms are confirmed, MR JOS private rules apply: `.io`/LDraw/meshes/renders
stay under `assets/private/` (git-ignored), no Modal uploads. Nothing uploaded.

## Regression baseline (T.0) — done, passes

- Baseline dir: `runs/baseline/mrjos_before_stage7/` (12 files + MANIFEST.sha256).
  Copies of frozen outputs: r17 `links_joints(.public).json`, `pred_C/pred_raw/transmission.json`,
  `robot_inferred.json`, Gate 2 v2 summary, ik_check_multi report+table, sim gt_eval.
- Git tag: `baseline-before-stage7` @ a119721 (before any Stage 7 change).
- Script: `scripts/regress_mrjos.sh` (commit 17643f5). Checks: frozen hashes,
  joint count/axes (0.5°)/points (1 mm, axis-normalized), C/C_raw exact + coupling
  pattern, fresh Gate 1 rescore (must stay 6/6), pytest default suite.
  Gate 2 re-run only with `--full`; live-console IK 59/60 is hash-pinned, manual at milestones.
- Baseline values: Gate 1 6/6 (worst 0.21 mm J2 / 0.069° J1); Gate 2 v2 100.0% n=500
  (median 0.45 mm, p95 0.97 mm); IK gizmo reachable 59/60 + unreachable 5/5;
  pred_C sign self-check 6/6; blind max-abs 1.2 FAIL + sim gt_eval 9/20 FAIL recorded as-is.
- Result now: `RESULT: NO DIFFERENCE`, exit 0
  (joints 6x 0.0000°/0.0000 mm; ratios identical; Gate 1 6/6; pytest 395 passed, 1 skipped).
  Note: first run showed 0.35° self-diff — comparator artifact (unnormalized axes);
  fixed by normalizing in the script (new file, no existing code touched).

## T.0 file handling — blocked

- `.io` was not at `assets/private/models_ext/tbot/`; found the matching file in
  `~/Downloads/MOC-85202_glaysche_Gantry_v4.io` (422,414 bytes, 2021-12-06) and copied it
  to `assets/private/models_ext/tbot/`. `git check-ignore` confirms ignored; `git status`
  shows no trace. No Modal use anywhere in this session.
- Blocker: all 81 zip entries are ZipCrypto-flagged (MR JOS `.io` files are flag 0/plain).
  Python zipfile + `unzip` both refuse without a password; not even `.info`/thumbnail readable.
  No password guessing attempted. Studio 2.0 is installed at `/Applications/Studio 2.0`.
- Need from human: either the archive password, or open the `.io` in Studio and
  File > Export as LDraw into `assets/private/models_ext/tbot/tbot.ldr` (then T.1 resumes:
  `src/parse/ldr.py` → parts.json → part counts, motor count, missing-catalog list,
  data-only catalog additions + regress re-run, linear-joint candidate list).

## T.1 — waiting on the LDraw export (nothing parsed yet)
