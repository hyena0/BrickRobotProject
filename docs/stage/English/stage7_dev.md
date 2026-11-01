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

## MR JOS path inventory (auto vs human; LOG/stage-docs/runs evidence)

- Parsed model.ldr is FLAT (1 FILE block, 836 parts): MPD structure NOT used.
  model2.ldr's ~395 FILE blocks are part definitions, not robot submodels.
  Segmentation = HUMAN: div.io separate file, 7 displacement clusters
  (Gate-1 fallback, LOG rows 40-41).
- Ports = HUMAN: wiring photo → config/ports.local.yaml 11/11 (rows 42-43).
- GT = HUMAN: gt/gt_joints.json, kinematics_gt_v2.json, eval/ground_truth
  (rows 56, 92, 96). Catalog = tool draft + human review.
- Joints = tools (candidates/fits/sweeps) + Spark SELECTION 3 runs temp 0
  (row 78; clobber guard row 80); final r17 chain path = deterministic tools,
  no Spark (row 83). C0-C6 spec + void/bracket verdicts = human (rows 81-84).
- Ratios/coupling = mixed: J1 geometric + human triple-lock + th1 approval;
  J2 human fallback; wrist hand re-trace; 743 break (rows 85, 87, 88).
  Solver outputs unused. Switches th1/th2/th3 = human.
- S4 FK/IK/eval/conventions = auto code; EE gauge + signs auto vs human GT.

## Agent execution attempts (as-is, $0 spent)

- joints_run: BLOCKED — missing required --stage3 (motor_paths is a hand-built
  Stage-3 artifact; gear_graph motor_drive is a different structure, adapting
  it would be a new hint). Fails before any API call.
- chain_run: BLOCKED — missing required --div (MPD blocks = designer grouping,
  excluded as hint). Fails before any API call.
- Spark cost estimate (not executed): 1–3 select_joints calls on
  muse-spark-1.3-contributor, temp 0 (run_selection loop); evidence table is
  17 T-Bot candidates vs 38 for MR JOS, so smaller than the historical runs.
  Stage-7 budget $1–3 total ($1–2 with extension); a selection loop of this
  size is a fraction of that (order $0.1). Credentials present (2 keys).
  Sweep tests (17, local compute) run before any API call once inputs exist.

## Approved fixes

1. classify() general rules — DONE (commit above): L-less axle lengths,
   axle-pin family, cross-block connectors, non-EV3 motor (size small/medium/
   large/None) + hub-as-brick, neutral rack wording. New-rule categories match
   committed data 98/101 (rest: 3 deliberate human overrides). Regression:
   NO DIFFERENCE (pytest 397). During the work an ordering bug (hub before
   connector) was caught by the alignment check and fixed.
2. A17 cause — INVESTIGATED ONLY: both 32270 lock to frame-fixed axles via
   axle bores (catalog-correct: axle hole = locked), bevel mesh numerically
   valid (27.48 vs 28.0 LDU). Locked mesh = static analysis cannot tell a
   hinge pivot from a lock (documented gear_graph limitation). No code change.
3-4. Left as-is per approval.

## Div file validation — PASS (human input at MR JOS level)

- Files: `MOC-85202_glaysche_Gantry_v4_div.io` (sha256 0bdb6ea2…, plain zip —
  unlike the encrypted original) + exported `div.ldr` (sha256 ed33f41e…).
  Both private, git-ignored. Provenance: human division, agent verification.
- Structure: same 17 MPD blocks as original. Whole-file edge diff: exactly
  TWO instance moves, both pure translations (R identical), root identical.
  `top shuttle`: +932.13/+764.343/−660 LDU. `vertical axis`: +820/+438.482/+20.
- Links (structural, disjoint, exact): base 845 + top_shuttle 53 + vertical_axis
  68 = 966. Every part in exactly one link. Saved (private):
  runs/stage7_tbot_t2/tbot_div_links.json.
- Pose check: static 845 exact incl. rotations (0.0001 mm); moved 121 =
  53+68 exactly, part-id multisets equal (no add/drop), rotations preserved.
  No origin-shift noise. No stop needed.
- Honest flag: static 845 is one RIGID pose, not one physical link (chain
  loop + undisplaced carriages circulate/move within it — rigid displacement
  cannot segment circulation). 3 rigid groups vs 3 DOF is the human's call.
- Motors/hub: all 3×54696 + 22127 in base, one link each — matches MR JOS rule
  (147/149→D1, 148→D2, 736/737/743→D3; ports separate, GT-blind).

## Motor path proposals (agent proposal + human confirmation; user saves motor_paths)

- M1 p000001 → shaft {32062, 3648b p9} → TOP cluster {10928 p31 × 3648b p39
  × 46372 p40/p49} + BASE {11955 p63/p65} → same-shaft p65→32270 p133 VERT →
  VERT bevel pair → candidates L_TOP / L_VERT (exact p9 shaft link unproven).
- M2 p000002 → shaft {32062, 3648b p8} → same cluster (M1/M2 indistinguishable
  in-graph — human must split X vs Y).
- M3 p000003 → shaft {32062, 32269 p5 A3} → graph dead end → UNKNOWN, needs
  human/Studio inspection.
- Chain loop: no rigid path (circulation) → unassigned, T.3.
- Racks (VERT p126/p127): mating pinions unidentified → T.3.
- Regression still green (397 passed). Awaiting user motor_paths.

## Division confirmed + user movements/motor_paths (human input, MR JOS level)

- Div file confirmed as-is. Base intentionally holds the chain (drive part) and
  undisplaced carriages — 845 is one rigid pose, not one physical link.
- User table: 1/top_shuttle/A/chain+gear/좌우; 2/vertical_axis/B/chain+gear/상하;
  3/plunger/C/gear/전후. Structure: 1:1 drives, NO co-drive; movement 3 is a
  gear-driven plunger nested inside the vertical rigid group (independent motion
  human-confirmed, not rigidly segmented).
- Saved (private): runs/stage7_tbot_t2/motor_paths.json, provenance "agent
  proposal + human confirmation". Chains end at verified terminals (p31 TOP,
  p133 VERT, p5 BASE); chain hops human-confirmed (640/453 mm, no mesh edge).
  UNCONFIRMED: port A/B/C ↔ uid p1/p2/p3; M1/M2 ↔ movements 1/2 (same cluster);
  M3→3 by drive-type elimination only.

## T.2b pipeline (MR JOS order; Spark cost restated: 1–3 calls, order $0.1)

- gear_graph with div link_of: 20 nodes, 4 edges, M1+M2→A19, M3→A3 (was A18
  without links). intra still 1 (bevel pair).
- joints_run DONE (run-id stage7-tbot-joints): 1 attempt, 1 Spark call,
  6 selected + 6 dropped + 5 rejected, consistency OK. Drive gaps recorded
  (J1/J2/J5/J6 motor-less; G73 chain unreached; EE G177 unconnected).
  All 6 selected are REVOLUTE (schema has no prismatic); positions z=290–500.
  Cost: 1 call, 17-row table (within the order-$0.1 estimate).
- PROMPT BIAS FOUND (not fixed): joints_spark PROMPT hardcodes MR JOS
  knowledge ("J2 user-confirmed 149-driven", "J1 slew", "differentially
  coupled wrist", "typically 6 joints", "fewer than 4 is under-selection").
  T-Bot reasons parrot it (149, slew, wrist). Selection pushed toward 6
  joints for a 3-movement robot. Approval item: generalize the prompt
  (per-model gap input or drop the paragraph).
- build_topology: NOT run by design (needs div joints + injects MR JOS RING140
  virtual gear — running it on T-Bot would be model contamination).
- assemble: attempted, BLOCKED — FileNotFoundError pred_C_tbot.json (no
  coupling inference path as-is) and no T-Bot ports file. Exact errors kept.

## T.3A MR JOS-only assumptions (audit, no fixes)

- joints_spark PROMPT: "known drive gaps" (J2/149, J1 slew, differential
  wrist), "typically 6 joints", "fewer than 4 is under-selection".
- build_topology: RING140 virtual gear + p000522 copy + z=-143.0 + "D1";
  manual edge p000812×p000802 @16.966; J5 096/095; confirmed_mapping
  p000148→J3, p000149→J2; serial D0→ order; turntable-half logic.
- assemble: type hardcoded revolute; EE default link "D6"; coupling text in
  joint degrees; motor default medium. Neighbours: FK/IK/MJCF revolute-only.
- Fixed in T.3B/C instead: generic prompt profile; prismatic schema +
  assemble type carry-through (default revolute).

## T.3B generic prompt + MR JOS comparison

- Added `--prompt-profile generic` (legacy prompt byte-identical; test pins
  it). Generic takes movement count only from --stage3 user_movements.
- MR JOS generic rerun (run-id stage7-mrjos-generic, new dir, baseline
  untouched): 4 selected + 6 dropped + 22 rejected, 1 attempt, consistency
  OK, lore-free reasons. Cost: 1 call (within order-$0.1 estimate).
- Comparison vs legacy joints path (r3, 6 selected): 3 exact (J1, J5-, J6-
  interfaces, 0.00 mm/deg) + 1 near (J3-interface, 64 mm off, axis
  antiparallel = same line). Legacy J2-void + J4 not selected (no count
  push). Differences are selection judgment, not regressions: legacy code
  path untouched (test-pinned), baseline files untouched.

## T.3C linear joints (additive: src/tools/linear.py, schema, 1-line carry)

- check_rack_mesh (spur + travel axis + tooth line + pitch radius rule):
  T-Bot 0 meshes (racks disengaged in CAD — recorded, not fixed).
- trace_chain: 200-link strand, pitch 6.4 mm, loop 1274.8 mm; closure gap
  (dead end p000767 + spur p000766; 25 parts off-path — turnaround at
  sprockets engages teeth, not links).
- Sprockets: 2×94925 16T (102.4 mm/rev) + 3648b p000009 24T (153.6 mm/rev,
  M1 shaft-direct 1:1 pending human port confirm).
- motor→sprocket graph paths: none (chain hops aren't graph edges) —
  mm/motor-rev uncomputable as-is (recorded gap).
- Catalog data: linear_axis on both racks; tooth_line_offset on 18942.
  Schema accepts prismatic; assemble carries type (default revolute).
- T-Bot generic rerun (stage7-tbot-generic): 3 selected (J1/J2 X-axes, J3
  Z-axis) + 6 dropped, 1 attempt, consistency OK, lore-free reasons. Cost:
  1 call. Mapping: M1(좌우/X)↔J1/J2 ambiguous (2 X-joints, 1 movement);
  M2(상하/Z)↔J3; M3(전후/Y)↔none selected. Motor proximity vs drive-path
  chains disagree on M3 (p5 terminal vs p000003-near-J1) — recorded, human call.
- Prismatic joints P1/P2/P3 emitted to linear.json (links
  base/top/vertical/plunger), schema-clean. Travel: P1 153.6 mm/rev
  (M1 shaft-direct, port unconfirmed); P2/P3 null with gaps (no
  motor-sprocket train; rack mates unidentified). FK/IK for prismatic:
  kin still revolute-only — remaining block.
- Remaining blocks: pred_C/coupling inference (no automated path); T-Bot
  ports file; port A/B/C↔uid mapping; M1/M2↔movements split; chain loop
  closure (25 parts off-path); plunger not rigidly segmented (by design).

## T.3 re-proposal (after T.2 + approvals)

- Same two extensions, tightened inputs: rack tooth lines + 3.2 mm pitch are
  now measured (18942: 32 teeth counted); motor shafts traced (3 × 2-part);
  mating pinions/sprockets still unidentified — that IS the T.3 work.
- Pre-blockers for any agent re-run: div segmentation + motor_paths report
  (both need human/div input — same human-input points as MR JOS, not
  automatable as-is). Spark selection loop itself is cheap (order $0.1,
  1–3 calls) once those inputs exist.
- Guardrails unchanged (additive only; 24121 must not reclassify; regress green).

## Finishing-4 console T-Bot (additive, MR JOS paths unchanged)

- Package: runs/stage7_tbot_t2/tbot_package.json (private, git-ignored):
  D0..D3 coarse envelopes (bbox+15mm pad; D3 temp box at rack origin),
  limits_mm (P1 ±120, P2 ±80, P3 ±47.2), safe home q, drives (P1 153.6
  mm/rev provisional; P2/P3 null), stage (no table: floor below lowest
  part; reach 200 covers limits box).
- Server (src/console/control.py, additive): --robot mrjos|tbot (default
  mrjos); /api/robot + /api/robot/select (in-place model+jogmap swap);
  package_doc(handler=None) — all prior callers unchanged; tip_doc
  all-prismatic bypass (kind-based, no model names); model_payload
  kind/unit per joint + last-point gripper ref; solve_ik passes limits_mm
  alongside limits_deg + safe-pose "q" fallback; ranges accepts limits_mm/
  operating_margin_mm; overview coupled-readback KeyError fallback to
  independent readback; /api/mode forces virtual+disconnected for tbot;
  /api/table serves package stage; /api/visual link count from chain.
  TBOT_BLOCKED refuses 19 motion/sensor APIs ("tbot is virtual-only").
  T-Bot jog map is display-only (brick/port "-", P1 ratio 0.4267 mm/deg,
  P2/P3 None = unknown drive, honest).
- Client (same file, kind-branched): robot select in bar (reload on
  switch); fkine prismatic translation branch; qNative/qvec replace all
  deg->rad maps; joint-count loops generalized (mrjos values identical);
  joint panel + ranges show mm for prismatic; isConnected false for tbot
  (send row hidden, body follows IK); mode banner "+ T-Bot".
- ui_smoke.py: --phase tbot (select -> banner/panel/send-hidden checks ->
  ik-reach -> live<=5mm). Not run here; user runs with browser.
- Verify: socket-free API check 14/14 PASS (model/kinds/ee/mode/table/
  ranges/overview/reachable/ik pos_err 6.8e-9/goto+ jog blocked/visual/
  mrjos reselect 6R); tests/test_control.py + test_mini_console.py 61
  passed; regress_mrjos.sh RESULT: NO DIFFERENCE (410 passed).
- Private meshes: none built; T-Bot uses primitives by default;
  --private-visuals tbot would serve visual_pack/tbot/*.glb (absent).
