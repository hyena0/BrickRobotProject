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
