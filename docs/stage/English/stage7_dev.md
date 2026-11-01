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

## T.1 parse — done (human exported LDraw; parser fix + catalog data)

- Export: `assets/private/models_ext/tbot/MOC-85202_glaysche_Gantry_v4.ldr`
  (MPD, 17 FILE blocks, Studio NumOfBricks 966). Parsed with the Stage 1 parser to
  private `parts.json`: **966 parts, 64 types — exact match with Studio's 966.**
  det<0 poses: 0. git-ignored; no Modal.
- Parser fix (general LEGO-MPD rule, not T-Bot-specific): type-1 refs with spaces
  were dropped (`len(tokens) != 15`), and the root fallback (max type-1 block) picked
  the `Chain` leaf (201 parts) instead of the MPD root. Now the ref is the rest of the
  line and the root is the first FILE block (max heuristic kept as last resort).
  MR JOS re-parse is byte-identical to `runs/stage1/parts.json` (836 parts).
  Test: `test_parser_spaced_submodel_names_and_root_first` (failed before, passes now).
  Regression: NO DIFFERENCE (pytest 396 passed incl. the new test).
- Counts: parts 966, distinct 64, **motors 3** (`54696` SPIKE Prime Small Angular,
  `motor: small` is a new size class — EV3 code paths expect large/medium, see T.2),
  plus 1 Control+ Hub (`22127`, brick per EV3 95646 precedent).
- Catalog: 17 missing types added, data only, all `review: true`, axes/holes unmeasured:
  11955 gear 8T; 18940/18942 racks; 22127 brick; 24375 pin (friction, rubber);
  32002 pin; 3711/3873 chain (neutral `other`); 3749 axle; 39794 frame; 43857 beam;
  48989 connector; 50450 axle 32L; 54696 motor; 65249 axle 2L; 71710/73507 beams.
  Coverage now 64/64. Regression after the addition: NO DIFFERENCE.
- Linear-joint candidates (title-based, pipeline recognition is T.2's job):
  direct — 18940/18942 racks (one with beam housing = slide guide), 3711x199 +
  3873x2 + 24375x24 chain-drive loop (`Chain` submodel), 11955 sliding gear x3;
  rail-like structure — 50450 axle-32 x6, beams 15/13/11 (32278/41239/32525),
  71710/73507 long liftarms, 64782 panel 5x11.

## T.1b catalog geometry fill — done (data only)

- Same method as MR JOS (`review_part`: title + Studio-tree geometry, /tmp probes).
  Titles re-checked from `.dat` headers. **11955 teeth: 8 tooth placements counted
  in geometry — title confirmed.** Racks: 18942 32 teeth at 8.0 LDU counted;
  18940 same 1x14 profile/periodicity (teeth built from boxes, 32 by family).
  Pitch 8 LDU = 3.2 mm (both). 50450: single axle.dat scaled 640 LDU = 32L along X.
  54696 shaft line Y from 2 collinear axle bores (output end unconfirmed).
- Filled: axes for 11955/24375/3749/39794/43857/48989/50450/54696/65249/71710/73507;
  holes for 11955(2) / 43857(6) / 48989(6) / 39794(72) / 71710(43) / 73507(31);
  teeth 11955=8, 18940/18942=32. review:false where title+geometry agree (9 of 17
  stay true: racks-18940, pins, chain, 3749, 39794-frame, 48989, motor).
- Judgment calls (documented in evidence): 24375 rubber→friction true (textual rule
  says frictionless); 3749→axle (mirror 43093; textual rule says pin);
  3711/3873→neutral `other`; motor size `small` (new class).
- Regression after fill: NO DIFFERENCE (pytest 396).

## T.2 as-is run — done (no hints, no code changes; script /tmp/tbot_t2.py, private runs/stage7_tbot_t2/)

- S1 tools ran fully: 966 parts; rigid groups 278 (largest 534 frame lattice,
  255 singletons incl. chain links); 3 motors each with 2-part motor_shaft traces.
  Gear scan (21 gear/rack parts, all pairs): 5 true meshes (8T/24T ext, 8T/8T ext,
  24T/28T compound-shaft, 28T/8T ext, 12T/12T bevel) + 2 same-shaft compounds.
  **Racks mesh with nothing** (no rack rule, gear_kind null). **Chain invisible**
  (category other, outside powertrain step-a).
- S2: `joint_candidates` ran: 278 groups → 37 powertrain → 17 structural candidates
  (fitted axes, err 0–31 mm, n=5–34; table in t2_report.json). chain_run BLOCKED
  (needs --div; MPD blocks are designer grouping = hint, not used). joints_run
  BLOCKED (needs Stage-3 motor_paths + Spark selection).
- S3: `gear_graph` ran: 19 nodes, 4 edges, motor_drive 2→A18 + 1→A3, **intra_meshes
  1 (A17 bevel pair p000132×p000133 rigidly connected yet meshing)**.
  build_topology BLOCKED (needs div+joints). No automated ratios exist.
- S4 assemble BLOCKED (no joints doc / coupling / ports file for T-Bot).
- Result: links = 278 rigid groups (no segmentation); joints = 17 unselected
  candidates, all revolute-hypothesis; ratios = none; coupling = none.
  Rack-pinion and chain drive unrecognized — expected generalisation failure,
  not fixed.

## Fix-approval list (no fixes without human approval)

1. `classify()` wrong-category defaults (50450→beam, 48989→pin, 3749→pin,
   motors/hub→beam) + "curved rack" evidence text for straight racks. Propose:
   general title rules (axle-N without L, SPIKE/Control+ electric, straight-rack
   wording). Data already overrides; this is future-model hygiene.
2. intra_meshes non-empty (A17 double-bevel pair). Propose: investigate whether
   rigid-grouping or the bevel rule misfires; no fix sketched yet.
3. Motor size `small` unhandled downstream (codegen default medium, ev3 expects
   large/medium). Propose: carry the string through (display) until T.3 drive work.
4. Data watch (no code): 43857 'Beam 2' with 6 holes; 71710/73507 asymmetric
   side-hole rows — measured as-is, human eyeball welcome.

## T.3 proposal (scope only, not started)

- Rack+pinion → prismatic joint: rack tooth line (placements on file) + mating
  pinion from proximity/phase (T.2 found none — identification is the work);
  travel = pinion_teeth × 3.2 mm/rev.
- Chain+sprocket → prismatic (virtual): loop path from 199 links + engaging
  sprockets; travel = links/rev × link pitch (link pitch still to measure).
- Guardrails: new functions only, no threshold edits, no existing-function
  rewrites; MR JOS regress must stay green (24121 slew ring must not reclassify).
  2 h box per addendum.
