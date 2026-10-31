# Stage 5 Development Record — Handoff to Stage 6

Read this before starting Stage 6. It carries the entry state, the
conventions Stage 5 settled, and the traps that must not be repeated.
Companions: `generated/robot_inferred/` (the program),
`runs/stage5/poses10.json` (replay angles + verdicts),
`runs/stage5/pose_check_report.json` (the numbers),
`docs/stage/English/LOG.md` (the history). Ground-truth values live
only behind the scorers; this file contains none.

## 1. Stage 6 entry state

- Control package (public, abstract kinematics only):
  `generated/robot_inferred/{robot_config,motion,home,guards}.py`,
  built by `python -m src.codegen.generate` from
  `models/robot_inferred.json` + `config/ports.local.yaml` +
  `runs/stage5/sensors.json`. Drive motor per joint = argmax |C| row
  entry (matches the confirmed mapping). Soft limits narrow the model
  by 5 deg each side; first-run speed cap 30 deg/s; synchronized
  `run_targets` per brick, speeds scaled by travel.
- Homing plan: local x3 (motor and sensor share a brick),
  remote x1 (J4: motor and sensor on different bricks, PC polls),
  stall x2 with one shared motor homed once. All six homing
  directions are UNCONFIRMED and all zero offsets are 0.0 until a
  physical run confirms them (`config/homing.local.yaml.example` is
  the calibration form). Bring-up runner:
  `python -m src.codegen.bringup --generated generated/robot_inferred
  --model models/robot_inferred.json ... home|jog|poses|cartesian`
  (`--fake` rehearses against full-port fake bricks).
- Pose validation (`python -m src.viz.pose_check`, 10 poses =
  7 random motor-space + 3 near-end, q = C m, soft-limit resample):
  support holes stay on axes, all 16 cross-link gear meshes hold,
  Blender EE matches FK within 0.001 mm on every pose — but all 10
  poses self-collide (BVH mesh-confirmed; hinge neighbours plus
  genuine folds, none of them known gear meshes). `poses10.json`
  carries a per-pose `mesh_contacts` verdict: none of the 10 qualify
  for physical replay. Re-screen before commanding hardware.
- Sim measurement (perfect ruler): joint poses 0.000 mm tip error;
  Cartesian 6/6, median 1.92 mm, max 4.25 mm (bringup-identical IK
  seed; the first kit failed screening on seed mismatch, see traps).
- Hardware status: BLOCKED. No `config/hardware.local.yaml` (broker
  address and brick identities unknown), no broker on localhost or
  the LAN, no brick access from this environment. Nothing in Stage 6
  item A (physical pointing) can start until a human provides the
  hardware path. Item B (VLA in sim) needs only Stage 4 + this
  record and is startable.
- Suite: 166/166 tests PASS (synthetic `tmp_path` fixtures only).

## 2. Conventions that hold for all future work

- Bricks see ports only; joint names live on the PC side. Generated
  files carry no part IDs, poses, or appearance data (the generator
  asserts this on every run).
- Regenerate, never hand-edit, `generated/robot_inferred/` after any
  model or calibration change, and re-run the bring-up checks.
- AABB overlap is a candidate generator only (the pack is dense:
  hundreds of baseline pairs). Failures require BVH mesh truth;
  known gear-mesh pairs are excluded up front (designed to
  interleave). Zero-pose BVH spot checks separate CAD interference
  from motion-induced contact.
- IK seed is part of the behavior contract: screen targets with the
  exact seed the runner uses (bringup default), or the verified
  numbers will not reproduce.
- Renders and run JSON stay git-ignored (`assets/private/`,
  `runs/`). Stage 6 items A/B2 need real footage; plan the camera
  before the motion.

## 3. Traps found in Stage 5 (do not repeat)

- Scripts under `/tmp` shadow stdlib (`bisect` vs a stray
  `/private/tmp/bisect.py`): run probes with `PYTHONSAFEPATH=1`.
- Rodrigues rotation: R[1][0] used x*z instead of x*y — invisible
  for principal axes. Every rotation helper gets a diagonal-axis
  test against PoE FK.
- Blender joint hierarchy: child empties need locations *relative*
  to the parent frame with the identity parent-inverse; absolute
  locations plus the build-time inverse scatter parts across metres
  (caught by the EE-vs-FK check, then by a tiny render).
- Read `matrix_world` only after `view_layer.update()`; stale
  matrices silently poison parenting inverses and bounding boxes.
- Exclude helpers from measurement: the shadow-catcher Floor and
  the joint arrows do not belong in AABB sets or renders.
- `min()`/`max()` over pose subsets must tolerate the empty set
  (all-colliding joints); report `null`, not a crash.
- `FakeMotor.wait_done` on a never-started motor raised
  AttributeError; idle waits return True (real-motor path
  untouched).
- The 743 column is all zeros, so least-squares assigns that motor
  an arbitrary angle: harmless (unconnected shaft) but visible in
  `poses10.json` — do not mistake it for a drive command.
- Soft limits of +/-175 with +/-180 model limits: IK solutions
  near the ends fail planning after succeeding at IK. Screen the
  planned move, not just IK success.
- The +/-180 deg soft range is aspirational, not collision-free:
  even +-35 deg poses collide (wrist cluster, leverage). Stage 6
  `workspace.py` must screen waypoints for mesh contact and bias to
  small angles; treat the pointing-area rectangle as guilty until
  a screened sweep proves otherwise.

## Stage 5A addendum — conservative finish (2026-10-30)

Final approach (user-ordered, time-boxed 2 h): per-joint ranges plus
at most two linear inequalities on J2–J3. No cell growth, no split
adjustments. Lookup code stays in the tree but is switched OFF by
flag (`USE_COUPLED_LOOKUP = False` in `src/limits/enforce.py`,
`EMIT_COUPLED_LOOKUP = False` in `src/codegen/generate.py`):
IK, generated code and the package use only `limits_deg` and
`coupled_limits`.

Why lookup was abandoned: the grid fallback had no runtime
enforcement (tables stored empty, never read), and wiring it in
showed the validate-tighten loop collapses the box — tightening
linear half-planes past scattered folds drove acceptance to 1/2000
(then 0/2000) while the exact fold poses were tileable. The
conservative finish drops the fallback instead of rescuing it;
the observed fold cells stay in the authoring record for Stage 5C.

Final constraints (`runs/stage5a/limits.json`): box J1 [-180, 39],
J2 [-116.25, 40], J3 [-74.38, 21], J4 [-180, 180],
J5 [-110, 124.38], J6 [-60, 180] (J2 hi 46 -> 40, J5 lo -130.62
-> -110 from validation folds, cuts at sweep-verified clean grid
points); inequalities J2+J3 >= -27.2 (covers all 10 J2-J3 march
cells, home kept), J3-J5 >= -34.69, J2+J4 >= -206.74.

Validation (2000 random full poses, exact mesh, bearing-union
exclusions): run5 wide box 336 accepted / 10 flagged, of which 2
shown examples are bearing-rule exclusions; narrowed box 303
accepted (15.2%) / 5 colliding (depths 2.1-2.7 mm, D5-D4 x2,
D4-D3 x2, D2-D3 x1 — all real, none bearing). Zero NOT achieved;
the 5 residual poses are listed in /tmp/revalidate.json for Stage 5C
physical review. Acceptance is reported, not improved, per order.

Changed files: `src/limits/enforce.py` (flag, narrowing,
bearing-union in `validate` via `coupled.py`), `src/limits/
coupled.py` (`--pairs`, conservative 1-2 bound selection,
flag-gated tables/tighten/growth, full examples kept),
`src/limits/safe_poses.py` (bearing-union exclusions),
`src/kin/ik.py` (`_coupled_ok` lookup gate),
`src/codegen/generate.py` (`EMIT_COUPLED_LOOKUP`, `COUPLED_LOOKUP`
in template + config), `src/package/build.py` + `validate.py`
(`coupled_lookup` field, `{}` in v0), `docs/robot_package.md`
(field row), `tests/test_stage5a.py` (31 tests: bisect-mirror
regression, select-conservative cover, flag-off contracts).
Regenerated `generated/robot_inferred/` carries the 3 inequalities
and an empty lookup. `poses10_safe.json` (5 small + 5 large,
oracle-verified endpoints, EE >= table+40 mm) and
`packages/robot_inferred/robot_package.json` (9.5 KB, ACCEPTED)
built on the final limits. Gate: 57/57 pass
(test_stage5a + test_stage5 + test_stage4).

Remaining: (1) zero-proof cycle — one more narrow+revalidate round
(J1-cut candidate, all 5 residuals have J1 <= -58) with a fresh
2000-pose run (~30 min, did not fit the 2 h box); (2) Blender
renders of `pose_batch_safe.json` to
`assets/private/renders/poses_safe/` (poses verified, renders not
run — `pose_check.run_blender(batch, results, renders, quick)`);
(3) 5C physical review of the residual + unverified items
(J1/p000489 colour reference, J4/p000796).

### 5A.3 zero-proof campaign (2026-10-30, second half)

Six 2000-pose exact-mesh validations, same seed, bearing-union
exclusions, tables/tighten/growth all OFF: 336/10 -> 303/5 ->
~270s/2 -> 274/2 -> 274/2 -> 274/1 accepted/colliding. Cuts applied
(J1 lo -50, J2 hi 40, J4 [-115, 125], J5 lo -90, J6 hi 135; three
inequalities J2+J3 >= -27.2, J3-J5 >= -34.69, J2+J4 >= -206.74 kept
frozen). Last residual: D5-D4 p000008-p000063 at (J1 3.8, J4 40,
J5 -61.1, J6 121.9), PC depth 2.4 mm. Zero NOT achieved; narrowing
stopped here because the next cut would destroy J5 for
threshold-edge noise (see below).

Blender BVH truth (triangle overlap, original meshes) on the
residuals: 4/4 mesh-true, INCLUDING home-context controls
(home+J5 = -55/-61/-65/-70/-50/-45/-40/-30/-20/-10/0 ALL hit for
p000008-p000063). So the contact is a broad graze band (tight LEGO
sliding), while PC depths straddle the 2.0 mm line: clean at home,
2.1-2.8 mm off-home at the same J5. Same-relative-geometry
different-verdict across world orientations = the PC exact verdict
is not rotation-invariant (world-axis rays on decimated open
meshes): ~0.5 mm+ pose chaos at the threshold. Consequences: (1)
single-joint sweeps have systematic holes for this pair family
(grid + tier + decimation blindness); (2) PC-validation zero is not
provable by narrowing once the meter itself flips at the line;
(3) further cuts (J5 lo above 0) would gut the joint.
Frozen box stands: every cut tied to an observed fold; acceptance
13.7% (274/2000), reported not improved.

5C follow-ups: rotation-invariant verdict (link-local rays or
Blender-depth-gated confirmation); dense Blender-gated re-sweep of
J5/J6 with exact meshes; physical check whether the D5-D4 band
binds motion or is a tight sliding fit; the 2.4 mm residual pose
above plus /tmp/revalidate*.json + /tmp/win*_confirm.json evidence.

## 5C stall-fix round (2026-10-30, brick sw 5B.12)

Hardware jogs kept tripping the stall guard. Root causes: verdicts ran
from move start (no grace), a single firmware-stalled poll latched at
once, and the jog map ratios are normalized placeholders (1.0 for
J1-J5, 0.714286 for J6) while the true Stage-3 drive ratios are
J1 -0.0857, J3 -0.0343, J5 -0.2, J6 -0.1429, J2 fallback -0.0238,
J4 +0.2 (runs/stage3/pred_C.json C_raw/drive). A "3 deg joint" jog
therefore sends ~3 motor deg where J1 needs ~35.

Fix (src/ev3/brick_mp/{guards,core}.py, mirrored to brick/):
judge stall only inside the supervised move wait (idle HOLD never
reaches it); skip all stall verdicts for STALL_GRACE_MS=300 after
move start; firmware stalled() must read true continuously for
FW_STALL_MS=200 (accumulator, resets on false); latch_reason now
"stall:<port> target=.. position=.. speed=.. elapsed_ms=..".
Jog speed stays 250 motor deg/s (>= 200 ordered).
New tests in tests/test_brick_mp.py (hold-no-stall, start-grace,
firmware timing, reason format); tests/test_brick_hw.py two
expectations updated to the ordered contract (long move for the
firmware case). Full suite: 420 passed, 9 pre-existing tests/gen
failures unchanged. Brick re-copy needed: 12 .py files, 5B.12.

## 5C jog-ratio + arrival-band round (2026-10-30, brick sw 5B.13)

J6 hardware report `stall:outA target=11.0 position=6.0 ... elapsed_ms=632`
was a 5.0-motor-deg shortfall judged as stall. Fixes:
(1) mini.py DRIVE_RATIOS = real pred_C C_raw values (J1 -0.0857,
J2 -0.0238 temp, J3 -0.0343, J4 +0.2, J5 -0.2, J6 -0.1429); wiring
still from the model, ratios flippable per joint in
config/jogmap.local.json (negative allowed, no code change for M4).
Jog shows joint_delta + motor_delta and refuses < 10 motor deg with
a warning (executed: false).
(2) Brick: stall judged only for ports > 5 motor deg from target
(ARRIVE_BAND_MOTOR_DEG); stopping inside the band is arrival.
run_target/run_targets acks now carry residual(s).
(3) check.py default speed 30 -> 200 motor deg/s.
Tests: mini ratio asserts updated, arrival-band + min-move-warning
tests added; full suite 422 passed, 9 pre-existing tests/gen
failures unchanged. Brick re-copy needed: 12 .py files, 5B.13.

## 5C side json files (2026-10-30, brick sw 5B.14)

Manual per-brick editing of one brick.local.json was the error
source. Replaced with brick_left.json + brick_right.json shipped in
src/ev3/brick_mp/ (+ brick/): each main loads its own side file
first (legacy paths still fallback) and refuses a file naming the
other side. Old brick.local.json removed from both trees. Copy the
whole folder to both EV3s, run left_main on the left and right_main
on the right, no editing.

## 5C jog step select (2026-10-30)

Jog step now selectable (1/5/10 joint deg, default 5, clamp 10) in
mini console; server clamp raised 5 -> 10. Reported symptom
(move-stop-repeat even at ratio 0.1) is consistent with genuine
stall under load: thresholds are already lenient (300 ms grace + 5
deg arrival band), so a trip means the motor truly is not turning.
Asked for latch_reason + moved vs motor_delta to confirm.

## 5C homing generator (2026-10-30)

New src/homing/plan.py: merges user config/homing.local.yaml (per
joint: method/sensor/direction/speed) with inference (DRIVE_RATIOS
wiring, 5A.1 trip angles J1 44 / J2 51 / J3 26 deg, J4 unverified)
and wiring (ports.local.yaml + sensors.json). Validates before any
motion (empty values, unknown joints/ports, bad direction/speed);
cross-brick (J4: motor right, sensor left) warns and skips. Zero
offsets q = ratio*m + offset recorded per joint (stall/manual give
null/0-based); M3 writes them to the package later. Runner executes
on fake or real transport, clears rules+latches per joint (a resting
press would otherwise cross-latch the other brick), saves
runs/stage5c/<date>/homing.json+md. mini gained Mark zero button +
/api/markzero for manual joints. Fake-verified: 5 homing tests;
suite 428 pass / 9 pre-existing gen fails.

## 5C homing fake auto-trip + --only (2026-10-30)

Fake runs no longer report failed for sensor joints:
arm_fake_sensors trips each homing sensor at run start (stands in
for a press after travel; fake motors finish in 1-2 polls) and the
run is labeled simulated/fake in console output and homing.json/md.
Added --only J1[,J3...] (unknown names exit 1 before motion) for
one-joint-at-a-time physical checks. Suite green.

## 5C manual inline jog (2026-10-31)

J5/J6 manual prompt stalled: motors HOLD, so positioning by hand
was impossible and a second console was needed. The prompt now
accepts joint degrees inline (+/-N jogs at 60 motor deg/s, Enter
marks, q skips); --auto keeps the old accept-at-once path.

## 5C homing backoff + merge (2026-10-31)

Real J1 run failed refused (sensor already pressed at rule time).
Runner now backs off (opposite direction, 10 motor deg x5 max at
30 deg/s) before arming the rule; stuck sensor fails with a clear
message. Same-day homing.json merges by joint so --only runs
accumulate instead of overwriting. Suite green.

## 5C manual stale-latch clear (2026-10-31)

Real J5/J6: a stall latch refused later jogs with its old reason
(12.5 s elapsed), then J6 stalled genuinely 6 deg short of target
(speed 0, motor truly blocked). Manual jog now clears the latch
first and prints a back-off hint on guard stop. Genuine stuck
needs backing off, not looser thresholds.

## 5C backoff in joint degrees (2026-10-31)

J2/J3 backoff never cleared: 5x10 motor deg is ~1-2 joint deg on
high-ratio joints, inside the press region. Backoff now moves in
joint-degree steps (1 deg x5, motor delta = step/|ratio|) and the
stuck message tells the operator to check the search direction
sign or wiring.

## 5C assisted homing (2026-10-31)

J2/J3 would not home by flipping the search direction. New method
assisted: the operator jogs inline (+/-/Enter/q, like manual) until
the sensor trips; the trip is auto-detected by polling, Enter
requires the sensor pressed. Works cross-brick (polls, never arms
a guard) so J4 is reachable this way too. --auto steps the search
direction in 2 joint-deg steps up to 30 joint deg.

## 5C bounded sensor stepping (2026-11-01)

Blind 360-deg homing runs risked breaking the arm on a wrong
direction. Sensor homing now steps 2 joint deg at a time (max 30
joint deg) with the trip guard armed per step; a wrong direction
or obstruction ends the run instead of driving through it.

## 5C M3 homing complete (2026-10-31, real bricks)

All 5 sensor/manual joints homed; offsets in
runs/stage5c/2026-10-31/homing.json: J1 38.34 (sensor, dir -1,
3 steps), J2 26.87 (sensor, dir -1, 12 steps), J3 37.28 (sensor,
dir +1), J5 -20.0 / J6 -4.00 (manual mark). J4 skipped by design
(cross-brick). q = ratio*m + offset verified against 5A.1 trip
angles for J1/J2/J3. M4 (direction signs) still open.

## 5C webpage exonerated for J1/J2 (2026-11-01)

Sweep showed J1+J2 moving together; re-verified the web path:
mini /api/jog sends exactly one run_target to exactly one
brick/port (J1 -> right/outB, J2 -> left/outA; new committed
test_jog_sends_to_exactly_one_motor), and the transport publishes
only to robot/cmd/{brick}. The page cannot drive two joints at
once. Cause lies downstream: mechanics, wiring, or observation.

## 5C map poisoning found (2026-11-01)

User screenshot proved J1 jog -> left/outA ratio 1: a stale
Apply-map override (from combo experiments) lived in the running
server while the repo file was clean. Also, my earlier blind
write_file of jogmap.local.json destroyed their file without
reading it first -- do not repeat; read-merge instead. Fixes:
Brick/Port selects labeled Diag-only (jog always follows the map),
new /api/jogmap/delete + Delete entry button for single-joint
recovery, committed isolation test. Recovery: restart the server.

## M4 observed overrides (2026-10-31): hw_observed.local.yaml rules

- `config/hw_observed.local.yaml` (git-ignored) is the top authority.
  Any item with a value there is never re-measured or re-asked.
- Changed: `config/homing.local.yaml` only -- added `source:`
  markers (measured / observed-skipped). Values were already
  identical to the observed file; fake-brick `--auto` plan still
  validates (J1-J3/J5-J6 homed, J4 skipped).
- NOT changed, with reason:
  - `config/ports.local.yaml`: J1-J3 confirmed equal to current
    values, but the file header forbids joint keys, so measured
    status lives here, not there.
  - `config/jogmap.local.json`: all six `joint_sign` are null, so
    no ratio sign flips. Read-merge only, never blind rewrite.
  - `packages/robot_inferred/robot_package.json`: generated from
    the model (inference output) with no calibration block --
    measured zero offsets stay in the run record
    (`runs/stage5c/2026-10-31/homing.json`) and the observed file.
  - Inference files (`runs/stage3/pred_C.json`, model, bounds):
    untouched.
- GT separation: this file is never fed into stage 1-4 code or
  agent inputs.

Inference vs measured (sign = joint_deg per motor_deg):

| Joint | Motor port: inference -> observed | Ratio sign: inference -> observed | Sensor angle (5A.1) -> measured zero |
|---|---|---|---|
| J1 | right/B (p000147) -> same, measured (M3 homing ok) | -0.0857 -> null (M4 open) | trip 44.0 deg (right/in1) -> offset 38.34, dir -1 |
| J2 | left/A (p000149) -> same, measured (M3 homing ok) | -0.0238 (temp fallback) -> null (M4 open) | trip 51.0 deg (left/in1) -> offset 26.87, dir -1; prior run 45.72, repeatability open (2 deg goal) |
| J3 | left/B (p000148) -> same, measured (M3 homing ok) | -0.0343 -> null (M4 open) | trip 26.0 deg (left/in2) -> offset 37.28, dir +1 |
| J4 | right/A (p000737); jogmap override left/outC unverified -> null | +0.2 -> null (M4 open) | left/in3 trip unverified -> homing skipped |
| J5 | left/C (p000736); jogmap override left/outD unverified -> null, coupled with J6 | -0.2 -> null (M4 open) | no sensor -> manual offset -20.0 |
| J6 | left/C shared (p000736, C=0.7143); jogmap override right/outA unverified -> null, coupled with J5 | -0.1429 -> null (M4 open) | color-sensor assignment unknown -> manual offset -4.00 |

Motion rules now authoritative from the observed file: arrival
band 5 deg, stall only above 5 deg + 300 ms no-progress,
min motor move 10 deg, jog 250 dps, homing 150 dps,
first-jog limit 5 deg on unconfirmed joints.

## M4 signs from GT, ranges from 5A (2026-10-31, no hardware)

Physical measurement halted; nulls filled from existing records
only. GT (`gt/kinematics_gt_v2.json`, status measured-human,
R17 convention, user-verified wiring) went to control settings
only -- never into stage 1-4 inputs.

Signs (control ratio = joint_deg/motor_deg; magnitude from
inference, sign from GT drive diagonal, all +1.0 in R17):

| Joint | Inference sign (DRIVE_RATIOS, stage 3) | GT sign (C drive, R17) | Adopted jogmap ratio |
|---|---|---|---|
| J1 | -0.0857 | +1 | +0.0857 (from_gt) |
| J2 | -0.0238 (temp fallback, no model ratio) | +1 | +0.0238 (from_gt) |
| J3 | -0.0343 | +1 | +0.0343 (from_gt) |
| J4 | +0.2 | +1 | +0.2 (from_gt) |
| J5 | -0.2 | +1 | +0.2 (from_gt) |
| J6 | -0.1429 | +1 | +0.1429 (from_gt) |

5 of 6 signs flip vs inference; J4 agrees. Read-merged into
`config/jogmap.local.json` (existing brick/port entries
preserved). GT coupling magnitudes (gt_measured.json: J5<-736
-0.2, J6<-743 +0.159/-0.082) are joint-joint shape, not control
ratios -- signs only were taken.

Ranges: 5A package limits adopted as-is (from_docs); the wider GT
original-controller envelope was not adopted:

| Joint | 5A adopted (pkg limits_deg) | GT limits (not adopted) |
|---|---|---|
| J1 | [-50.0, 39.0] | [-180, 180] |
| J2 | [-116.25, 40.0] | [-95, 44] |
| J3 | [-74.38, 21.0] | [-45, 75] |
| J4 | [-115.0, 125.0] | [-180, 180] |
| J5 | [-90.0, 124.38] | [-5, 120] |
| J6 | [-60.0, 135.0] | [-180, 180] |

Ports J4/J5/J6 + color sensor: records disagree (see open
items) -- user decision pending, nothing filled.

## Ports decided, signs classified, coupling wired (2026-10-31, no hardware)

Ports adopted from diagram + design + GT (all three agree):
J4 left/C, J5 left/D, J6 right/A, color sensor right/in2 (J1
reference). The outlier was `ports.local.yaml` (p000737/p000743
swapped); fixed to GT (p000736 left/C, p000737 left/D, p000743
right/A). J1-J3 were identical in all records. `hw_observed`
ports/sensors/color entries now from_docs; J5/J6 manual offsets
(-20.0/-4.00) marked invalid (recorded under wrong ports);
`config/homing.local.yaml` J4 marked from_docs (same-brick target,
direction still unconfirmed).

Sign check per joint (GT C vs model C_raw, same R17 frame):
GT and model joint axes agree in direction on all six joints
(J1 [0,0,1], J2/J3 [0.707,0.707,0], J4/J6 [-0.707,0.707,0],
J5 [0,0,-1] both sides), so no convention flip applies --
control sign = GT sign directly. Result: 5 inference sign errors
(J1, J2, J3, J5, J6), J4 agreed:

| Joint | Inference sign | GT sign | Verdict |
|---|---|---|---|
| J1 | -0.0857 | +1 | 추론 오류 |
| J2 | -0.0238 (fallback) | +1 | 추론 오류 |
| J3 | -0.0343 | +1 | 추론 오류 |
| J4 | +0.2 | +1 | 일치 |
| J5 | -0.2 | +1 | 추론 오류 |
| J6 | -0.1429 | +1 | 추론 오류 |

`config/jogmap.local.json` rewritten with model-frame signs
(all +magnitudes, GT signs; existing brick/port entries kept).
GT went to control settings only, never stage 1-4 inputs.

Coupling: the converters used independent ratios only. Now
`coupled_motor_targets` / `coupled_joint_readback` in
`src/console/mini.py` implement the GT wrist coupling
(q5 = r5*(-0.2*m_a + m_b),
q6 = r6*(0.159*m_a - 0.0816*m_b + m_c), J1-J3 identity),
wired into mini jog, control pose apply, overview and pose error
readback. pred_C's shared-736 J5/J6 relation is superseded by the
corrected wiring. Fake-brick suites green (homing + mini +
control + stage4: 77 passed).

`src/homing/plan.py` now loads the local jog overrides like the
consoles do (it previously used model-only wiring, so J4 stayed
cross-brick/skipped after the port fix). Fake `--auto` plan:
all six homed.

## Gizmo-path IK check (2026-10-31, no hardware)

Cause of "only J6 moves": two stacked faults. (1) /api/ik is
position-only, but the live adopted ranges still held inverted
captures (J1 [23.34,13.34], J3 [42.28,16.0]) until the degenerate
lo>=hi guard; pinned joints forced the solver onto the only free
variable. (2) The TCP sits on the J6 axis (position-Jacobian
column < 1.0 mm lever), so J6 is a free spin variable from random
restart seeds. Fix: freeze J6 (position-only) at the incoming pose
+ ignore degenerate point captures + neighborhood seeds around
the drag start. Solved-branch check (start from true q): spread 50/50
converge -- the model is fine, DLS-from-Q0 coverage is
the gap (13/60 stick at J2/J3 adopted limits).
`src/verify/ik_check.py` calls the gizmo endpoint
(dispatch /api/ik, Q0 start), 50 spread + 10 near + 5 far.
Multi-start (current + 10 safe poses + previous solution, min
TCP error; locked wrist joints keep their own seed value, not
the Q0 value -- the global-freeze bug hid the wrist basins):
reachable 59/60 (median err 0.0 mm, max 0.06), unreachable
5/5. The single miss r33 reaches err 0.0 with one joint, so it
is a criterion artifact, not a solver miss. J6 delta = 0.0 on
all 65. Evidence: `runs/stage5c/ik_check_multi/` (report.json,
table.md, per-target front/side PNG with start/result/GT/TCP).
Console: TCP dot + TCP-target line + live distance, pre-drag
pose, GT overlay, ik-stat line, "닿는 목표" button
(/api/reachable_target), mat points (/api/mat_points), demo
wizard, /api/goto (15 dps default).

## Inference vs GT accuracy (eval only, 2026-10-31, no hardware)

GT models (`gt/kinematics_gt_v2.json`, `gt_measured.json`) went to
control/eval display only, never into stage 1-4 inputs.

Recorded result, no cause analysis: 추론 모델 vs GT: 위치 오차
중앙값 5.3mm, J1·J6 형상 오차, 방향은 일치, 제어 영향 없음.
Evidence: `runs/stage5c/sim/report.json` (`gt_eval.json` FAIL 9/20,
median 5.256mm gauge-corrected, rot max 0.15deg).

## Roadmap (deferred, needs no hardware except where noted)

- `conformance/` suite (spec 5C.7): not started.
- `docs/mobile_port.md` (spec 5C.8, design only): not started.
- Release guard `src/release/check.py` is a rough cut (a few
  tests); currently FAILs on 21 pre-existing tracked files
  (`generated_robots/*`, `assets/modules/*.ldr`, `gt/*`,
  `tests/fixtures/*.ldr`, `tests/*.blend`). Passing needs an
  explicit untrack/allowlist decision -- history untouched.
- Hardware-gated (6단계 실기): J4 homing direction, J5/J6 manual
  re-mark, J2 repeatability (2°), M4 direction verdicts, 3x homing,
  measured ranges into package, 10 safe poses on hardware.

## Console grounding + shared table (2026-10-31, no hardware)

One table definition in `config/calibration.local.yaml` stage
(table_z -220.8 from D0 mesh bbox stride-1 = lowest part vertex
32278, center 34.0/79.5, reach R 335.3); screen, IK targets, mat
points, reports all read `/api/table`. Stage = model - C, table =
z 0, base bottom center = origin; arm.position carries the single
model->screen transform (primitives and LDraw share it).
Gizmo starts/snaps at TCP, clamped at table (server clamps again),
out-of-range turns marker red and follows nearest reachable only.
Reach envelope shown as translucent shoulder sphere; mat dots live
on the same frame. Desk+40 proved unreachable (min TCP height
table+212 under 5A+coupled, binding constraint J2+J3>=0 margin).

## Tip TCP + gripper-down + mat riser (2026-10-31, no hardware)

TCP redefined to the gripper tip (D6-frame offset
[-160.0, 236.8, 134.2], 56.4mm from J6; wrist kept for GT
overlay). "Gripper down" checkbox solves position+orientation with
minimal-rotation mat. Orientation DLS sticks at 0.05 damping near
seeds but converges at 0.1; damping change reverted (the failing
point needed J4 -120 < -115 limit, genuinely out of range).
(A) lowest riser with 9/9 tip-down under 5A+coupled: table+220.8
(model z=0), all err <=1.9/1.7, seeds stored in mat JSON, console
sends them as q_init. `assets/private/mat_a3.pdf` + riser box
regenerated. (C) desk+40 without coupled: positions exist but
down solves 3/9 (5A per-joint ranges block, not coupling); leg
gaps fine on solved legs. Not adopted.

## UI smoke harness (2026-10-31, local only)

`scripts/ui_smoke.sh`: fake-brick server + headless Chromium,
fails on any browser console error, checks scene via
`window._scene`, drives mat 1 + grip-down mat 2 (TCP-target
<=5mm), saves 3 shots to `runs/ui_smoke/`. Trap fixed: #ik-live
refreshes every frame, so waits key off #ik-stat rewrites.
ALL PASS 2026-10-31. `dirLine` scope bug (init-local used in
tick) fixed in the same pass.
