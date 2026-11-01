# Stage 7 Addendum: Public Model Analysis (Gantry Robot T-Bot)

**Status: add-on document.** The existing `stage-7-generalization.md` stays unchanged.

## Relation to the existing plan

| Original Stage 7 | Now |
|---|---|
| Main test set: procedural arms | **on hold** (same problem as the 5D generator) |
| Task 3: run and report one public model | **expanded into the main task of this document** |

**Goal:** check whether the same pipeline infers joints, gears and coupling on a LEGO robot designed by someone else, with a structure very different from MR JOS.

## Target model

| Item | Detail |
|---|---|
| Model | **Gantry Robot "T-Bot" – 3 Degrees of Freedom** (Rebrickable MOC-85202, designer glaysche) |
| Source | https://rebrickable.com/mocs/MOC-85202/glaysche/gantry-robot-t-bot-3-degrees-of-freedom/ |
| File | the `.io` file already obtained → store in `assets/private/models_ext/tbot/` (git-ignored) |
| Structure | gantry (Cartesian) robot, 3 DOF. **Likely driven mostly by linear (prismatic) joints** |

## Two things to check

**1. Licence and publication**

Rebrickable MOCs are copyrighted by their designers; "free download" is not an open licence. The designer (glaysche) granted permission on Nov. 1, 2026 — recorded in `assets/public_models/tbot/PERMISSION.md` with the original text. MR JOS rules still apply to everything else.

- Permitted by the designer: renders of the rebuilt model (as AI input and in the 3-minute demo video) and showing the physical robot in the video.
- NOT permitted (not in the original): README images, console screenshots containing meshes, and redistribution of original files (`.io`, exported `.ldr`/`.mpd`, per-link mesh files) — these stay under `assets/private/` (git-ignored). No Modal uploads.
- Published renders live under `assets/public_models/tbot/renders/`. Motion-figure plots, console captures, originals and meshes stay private.
- Analysis tables and joint/gear numerics are our own measurements and stay public (owner decision, not designer permission).

Attribution (no trademark notice in it): T-Bot by glaysche (MOC-85202), used with permission — https://rebrickable.com/mocs/MOC-85202/glaysche/gantry-robot-t-bot-3-degrees-of-freedom/

**2. Linear joint support**

The current pipeline (Stages 2–4) was built around **revolute joints**. A gantry robot probably moves with **linear joints** (racks and pinions, rails).

- First run it **without code changes** and record the result. If linear joints are not recognised, that is a "generalisation failure case".
- Then, **optionally**, add linear joint support as an **extension** within a 2 h time box. Report it honestly as a "pipeline extension", not as "generalisation without code changes".

## Regression rules (most important)

The point of Stage 7 is **analysing a new model with the pipeline already built**. If fixes for the new model break the ability to analyse MR JOS, Stage 7 has failed.

**1. Freeze a baseline (once, before any work)**

- Before touching code, save the full MR JOS results as the **baseline**: link segmentation, joint list/axes/positions, gear ratios, coupling, Gate 1/2 metrics, IK test result (59/60), GT comparison.
- Location: `runs/baseline/mrjos_before_stage7/` (private, git-ignored). Tag the repo `baseline-before-stage7`.

**2. Allowed changes**

| Kind | Example | Condition |
|---|---|---|
| Data additions | new parts in the catalog | always allowed |
| General bug fixes | edge cases, parsing errors | must be justified as a general LEGO-mechanics rule |
| New capabilities | linear joint recognition | **only as additions that do not change existing revolute logic** |

**3. Forbidden changes**

- Model-specific handling: `if model == "tbot"`, hard-coded T-Bot part numbers, etc.
- Threshold changes that alter MR JOS results
- Rewriting or deleting existing functions wholesale
- Feeding ground truth into the inference path

**4. Regression check after every change**

| Item | Pass |
|---|---|
| MR JOS joint count, coupling structure | **identical** to baseline |
| Joint axis direction / position | ≤ 0.5° / ≤ 1 mm difference |
| Gear ratios | identical to baseline |
| Gate 1/2 metrics, IK test (59/60), GT comparison | no worse than baseline |
| All existing tests | pass |

One script, `scripts/regress_mrjos.sh`, runs these checks and prints a diff table against the baseline.

**5. If results differ from the baseline**

- Stop immediately and report the difference. A human decides.
- If the difference is an improvement (closer to GT), record why and save a new baseline. Otherwise revert the change.

**6. Change log**

For each change, record in `stage7_dev.md`: what, why (which model exposed it), the general-rule justification, and the regression result. Commit each change separately so it is easy to revert.

## Tasks (time box 3 h, +2 h with the extension)

| Step | Content | Pass |
|---|---|---|
| T.0 Prep | **save the MR JOS baseline and write `scripts/regress_mrjos.sh`.** Convert `.io` → LDraw, store privately, confirm with `git check-ignore`. Note the terms of use | baseline saved, regression passes, private confirmed |
| T.1 Parse | read with the Stage 1 parser; add missing parts to the catalog (data only) | missing part list and additions |
| T.2 Run as is | run Stages 1–4 **with no hints and no code changes** | result table (links, joint types and axes, ratios, coupling); on failure, record where |
| T.3 (optional) Linear joint extension | recognise racks/pinions and rails; compute linear joints and "travel per motor revolution (mm/rev)", added on top of the existing revolute logic | side-by-side with T.2; **MR JOS regression passes** |
| T.4 Visual check | link-coloured render (private) and joint motion in the primitive 3D view | human judgement |
| T.5 Ground truth | type and ratio (or mm/rev) of 3 joints checked by a human in Studio or the instructions. **Never given to the agent** | comparison table |
| T.6 Report | `stage7_dev.md`, abstract result table for README and video, including the final regression result | table and limitations; MR JOS baseline preserved |

## Budget

Spark (contributor) about $1–3, +$1–2 with the extension. No Modal.

## Today's schedule

| Time | Work |
|---|---|
| Afternoon ① | finish Stage 6 simulation |
| Afternoon ② | **Stage 7 T-Bot analysis** (this doc) |
| Late night | **Stage 6 IK physical test** (`physical_session` doc) |
