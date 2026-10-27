# Stage 4: Kinematics

**Latest:** Day 5 (Gate 2) · **Goal:** from the inferred kinematic model, build FK and IK, generate a primitive-geometry MJCF, and measure end-effector error against ground truth.

## Tasks

### 4.1 MJCF generator (≈1–2 h) · `src/kin/mjcf.py`

- Input: kinematic model JSON (schema). Output: MJCF with capsules/cylinders between joint points, joint axes and limits as given.
- Same generator builds `eval/ground_truth/robot_gt.xml` (private) and `models/robot_inferred.xml` (public).

### 4.2 Forward kinematics (≈1 h) · `src/kin/fk.py`

- Product of exponentials: each revolute joint is a screw with axis `w` and point `p` at the zero pose; `T(q) = exp([S₁]q₁) ··· exp([Sₙ]qₙ) · M`.
- Optional: derive DH parameters for display only.
- Test: FK matches MuJoCo site position for random `q` on the inferred MJCF (< 0.01 mm).

### 4.3 Inverse kinematics (≈2–3 h) · `src/kin/ik.py`

Interface: `ik(target_pose, q_init=None, position_only=False) → (q, success, pos_err_mm, rot_err_deg)`.

1. **Numeric (always):** damped least squares with joint-limit clamping, up to 8 random restarts.
2. **Closed-form (when available):** if the last three joint axes meet at one point (spherical wrist), split into position IK for the first three joints and orientation IK for the wrist. The agent checks the condition with a tool `axes_intersect(joint_ids)`.
3. **Learned IK (optional):** small MLP trained on FK samples (no vision). Only if time remains.

### 4.4 Motor-space conversion · `src/kin/motors.py`

- `m = C⁻¹ · q` plus zero offsets → motor targets in degrees.
- Joint limits checked in joint space before conversion.

### 4.5 Evaluation · `src/verify/ik_eval.py`

1. Sample 500 random `q_true` within ground-truth limits.
2. `pose = FK_gt(q_true)`.
3. `q_pred = ik_inferred(pose)`.
4. Error = distance between `FK_gt(q_pred)` and `pose` (position mm, orientation °).
5. Report success rate and median/95th-percentile errors.

This measures how well the **inferred** model controls the **true** robot.

## Gate 2 thresholds (set before the first run)

- Success (pos ≤ 5 mm and rot ≤ 3°) on ≥ 95 % of samples.
- Median position error ≤ 2 mm.

## Exit criteria

- [ ] Gate 2 met, or fallback applied.
- [ ] `models/robot_inferred.xml` committed (public, primitive geometry only).
- [ ] Results in `LOG.md`.

## Fallback (Gate 2 fails at Day 5)

Use position-only IK for demos (enough for pointing and air drawing). If still failing, demo motion in joint space only and centre the video on ratio/coupling accuracy. Cancel Stretch A and B.

## Stretch B check

When this stage passes, count days left before Day 9. If ≥ 3, start Stretch B now in parallel with Stage 5.

## Token estimate

Development $2–5. No agent calls needed except `axes_intersect` checks.
