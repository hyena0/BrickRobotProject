# Stage 3 Development Record — Handoff to Stage 4

Read this before starting Stage 4. It carries the entry state, the
conventions Stage 3 settled, and the traps that must not be repeated.
Companion: `runs/stage3/pred_C.json` (the numbers),
`docs/stage/English/LOG.md` (the history). Ground-truth values live
only behind the scorers; this file contains none.

## 1. Stage 4 entry state

- Joints (R17): `runs/stage2-chain-r17/links_joints_public.json` —
  6/6 PASS against `gt/gt_joints.json` (worst lateral 0.21 mm,
  worst angle 0.069 deg). Axes + points + limits feed 4.1/4.2 as-is.
- Coupling: `runs/stage3/pred_C.json`, built by
  `src/tools/pred_c.py --out runs/stage3/pred_C.json` (no solver
  inputs; fully hand-derived, see `drive` block for per-cell evidence).
  - Columns = drives [147, 148, 149, 736, 737, 743], column-normalized
    (self = +1). J3 column coupling-free. Col 743 and col 149(J2) are
    model zeros (path breaks, evidence in `drive.J743` / `drive.J2`).
    J2 self = 1.0 is human fallback (section-4, excluded from scoring).
  - Transmitter for 4.4: J1 −0.0857143, J3 −0.034286, wrist k4 +0.2 /
    k5 −0.2 / k6 −1/7 (R17 frame). Switches th1/th2/th3 = −1
    (human-confirmed). J1 magnitude blind-matched the human constant
    to 6 decimals; J3 magnitude re-verified from its gear train.
  - Whole-matrix blind score: C max-abs 1.2 FAIL (threshold 0.02).
    The gap is wrist joint-space semantics + the two zero columns —
    a logged model limit, not a blocker (Day-4 fallback disposition).
- Stage 4 first builds: `axes_intersect` (diagonal J6 re-evaluation for
  4.3 closed-form), zero offsets (values absent from `config/`;
  human-provided category), then MJCF/FK/IK per
  `docs/stage/English/stage-4-kinematics.md`.

## 2. Conventions that hold for all future work

- PRED_SPEC (`docs/PRED_SPEC.md`): self = +1 by column normalization;
  + rotation = right-hand about R17 axes; every pred runs the
  row+col sign-flip self-check (`sign_flip_check`, J6 row-only).
- A drive column with no traced driven joint stays raw/zero — never
  invent the joint (743 precedent).
- GT-blind: scorers print errors only. Never back-solve GT values
  from error percentages into code, docs, or tuning. Labels and
  positions only. `gt/` paths enter code solely as scorer `--gt`
  arguments — pinned by `tests/test_gt_isolation.py` (AST scan of
  `src/` minus `src/verify/`).
- Tests are synthetic-only: never read `runs/`, `assets/private/`,
  `gt/`, or `eval/ground_truth/` from tests; use `tmp_path`
  fixtures. Session scope is this repository only (AGENTS.md rule 7).

## 3. Traps found in Stage 3 (do not repeat)

- Catalog `axis_local` lies for turntables: 48168/48452 labels are
  perpendicular to the mesh-fit true normals (both fit
  [-0.707, 0.707, 0] == R17 J6). Rule: trust a label only when
  |dot(label, fitted axis)| ≈ 1, else fit the mesh.
- `check_gear_mesh` gate is 2.0 LDU and misses real meshes two ways:
  adopted-by-exception needs physical evidence (812x802: user
  confirmed + parallel + 0.4 LDU over → adopted; 211x235: daylight
  both ways → rejected). No silent tolerance widening.
- The tool has no internal-arc kind (524xrack-arc returns False):
  prove geometrically (pinion lateral + pitch radius + tooth tips).
- Knob sign is +1, not external-analogy −1 (zone rolling-contact
  proof). Global signs are absorbed by normalization + 4.4 switches;
  relative in-column signs are not — prove the stages that carry
  them (unequal bevel 095x010, crowns 097/098x818 were load-bearing).
- Dual pinions must agree (097+098 both −1 with the true normal);
  same-link static contacts (098x003, D5/D5) constrain nothing.
- Round-hole seat = free, axle-hole = driven (break-evidence
  standard). Collinearity ≠ coupling: 814/768/791 align to 0.001 mm
  yet uncoupled — a joiner or bore is required to claim a shaft.
- Absence needs a full-model scan (780 mates: none over ALL toothed
  parts), not a local look. One systematic C4-style re-trace precedes
  any break declaration (report the break point + evidence, then stop).
- The recorded lock law (16·w736−12·w743=0) is a virtual-diff
  hypothesis (780 meshless, 6573 teeth None, ratio title-based):
  synthesize lock-consistent dominant patterns from it, never feed
  rigid single-drive-locked (infeasible: 16·1−12·0 ≠ 0), never score it.
