# Stage 5A: Joint Limits and Safe Poses in Simulation

**Latest:** Day 4 · **Requires:** Stage 5 code generation and simulation rehearsal (done). **No hardware.**

**Goal:** prove in simulation that the inferred model yields a usable, safe IK: joint limits inferred from the model itself (sensor geometry, collisions, multi-joint folding), enforced in IK and generated code, plus a set of safe poses for physical replay in Stage 5C.

**Relation to existing docs:** Stage 5 (kept unchanged) covers code generation and the physical bring-up. In PLAN v2, Stage 5 is closed at the simulation level; its physical steps (5.3–5.4) are carried out in Stage 5C after Stage 5B hardware connectivity.

## Starting point

- Generated code and simulation rehearsal: 5 joint poses exact; 6 Cartesian targets, median 1.92 mm.
- Blender pose check (10 poses): support holes, cross-link gear spacing and FK all pass. **10/10 poses self-collide** because limits are still ±180°.

## Tasks

### 5A.1 Sensor boundaries from geometry · `src/limits/sensor_bounds.py`

For each sensor in `config/ports.local.yaml`, using the sensor ↔ joint association from Stage 2.7:

1. **Presser candidates:** parts on the moving side of the joint whose swept path passes within 10 mm of the sensor's active region (touch: button tip along `axis_local`; colour: view cone along `axis_local`, ≤ 20 mm).
2. **Sweep** the joint from the home pose in 0.5° steps both ways (others at home). Boundary = first angle where a presser reaches the active region.
3. Record: joint, boundary angle, homing direction, presser uids → `runs/stage5a/sensor_bounds.json` (private).
4. **Colour sensor:** `limit` only if it blocks motion; otherwise `reference` (sets the zero only).

### 5A.2 Collision boundaries · `src/limits/collision_bounds.py`

- From home, sweep each joint away from its sensor (J5: both ways) to the first collision.
- **Collision rule:** BVH penetration > 1 mm between parts of different links; exclude contacts already present at home (tight LEGO fits), designed gear meshes and the joint's bearing parts.
- Boundary = first collision − 5°.

### 5A.3 Coupled constraints · `src/limits/coupled.py`

- Pairs: those with fold collisions in the Stage 5 pose report (at least J2–J3, plus any pair with ≥ 10 recorded contacts).
- 5° grid per pair inside 5A.1/5A.2 limits, other joints at home and at 3 representative wrist poses → conservative linear inequalities (e.g., `a·q2 + b·q3 ≥ c`).
- Validate with 2,000 random full poses: **0 accepted poses may collide**; report the free space given up.

### 5A.4 Model and enforcement

- Schema and model: `limits_deg`, `limit_source` (`sensor` / `collision`; `measured` is added in Stage 5C), `home`, `coupled_limits`:
  ```json
  "joints": [{"id": "J2", "limits_deg": [lo, hi],
              "limit_source": {"lo": "sensor", "hi": "collision"},
              "home": {"sensor": true, "direction": "+"}}],
  "coupled_limits": [{"joints": ["J2", "J3"], "a": [1.0, 1.0], "b": -120.0, "op": ">="}]
  ```
- Operating range = sensor boundary − 5° (normal motion never presses a sensor).
- IK solves within limits and coupled constraints; `is_path_safe(q_from, q_to)` checks interpolated poses every ≤ 5° (limits, inequalities, BVH).
- Regenerate `generated/robot_inferred/` so `robot_config.py` and `motion.py` carry and enforce the limits.

### 5A.5 Safe pose set · `runs/stage5a/poses10_safe.json`

- 5 small poses (≤ 30 % of each range) + 5 large; every pose and its path from home collision-free, inside limits and inequalities, end effector ≥ 40 mm above the table.
- Renders to `assets/private/renders/poses_safe/`.

### 5A.6 Robot package v0 · `src/package/build.py` → `packages/<robot>/robot_package.json`

The robot package is the only runtime input of the controller package (PLAN v2 section 5).

- Schema documented in `docs/robot_package.md`, with a `schema_version` field.
- v0 contents from this stage: abstract kinematics (axes, link dimensions, `C`), `limits_deg` with sources, `coupled_limits`, operating margins, safe poses, speed caps.
- Added later in Stage 5C: port wiring, homing config, zero offsets, measured limits (each change bumps `package_version`).
- No meshes, no part uids or poses: light enough for mobile and publishable under PLAN v1 section 4a.
- Runtime collision safety relies only on `limits_deg` and `coupled_limits`; mesh/BVH checks stay in the PC authoring tools.

#### Package content constraints (enforced by the validator)

**Allowed**

| Field | Content |
|---|---|
| Kinematics | Joint axes, joint points, link dimensions, `C`, end-effector point |
| Limits | `limits_deg` with sources, `coupled_limits`, margins, speed caps |
| Safe poses | Joint-angle lists only |
| Hardware | Port wiring (motor/sensor id → brick, port), homing config, zero offsets (added in 5C) |
| `link_envelopes` (optional) | **At most one oriented box per link**: `{link, center_mm, size_mm, orientation}`. Used only for display and clearance hints in the UI |

**Forbidden** (validator rejects the package)

- Meshes, vertex or face lists, point clouds, convex hulls, or any geometry other than one box per link.
- Part ids, part names, part poses, part counts, colours, sub-model or sub-assembly names.
- Images, renders, textures or file references to `assets/private/`.
- Free-text fields describing the original design (design name and credit live in README only).

**Limits**

- Box size precision rounded to 1 mm; orientation to 1°.
- Package file ≤ 100 KB.
- `link_envelopes` is computed by the PC authoring tool from the private model and written as numbers only; the model itself never leaves `assets/private/`.

Validator: `python -m src.package.validate packages/<robot>/robot_package.json` (schema check + forbidden-field scan + size check). Runs in tests and before every package write.

#### Private visual pack (development, testing and video only)

Real LEGO shapes are needed to check motion visually and for the demo video, but they must never be distributed. They therefore live in a **separate, private visual pack**, not in the robot package.

- Location: `assets/private/visual_pack/<robot>/` (git-ignored): one decimated mesh per link (`.glb`) exported from Blender in the link frame, plus `manifest.json` (link id → file, mesh-to-link transform, package version it matches).
- Built by `src/viz/export_visual_pack.py` from the Stage 2 link segmentation; total ≤ 200k triangles for smooth browser display.
- The robot package never references the visual pack; the validator rules above stay unchanged.
- Used by the console's private display mode (Stage 5C) and by Blender renders. Never committed, never bundled into `src/ui/`, never copied into `packages/`.

## Exit criteria

- [ ] Every joint has `limits_deg` with a recorded source.
- [ ] Coupled constraints: 0 collisions in 2,000 accepted samples.
- [ ] IK, path check and generated code enforce limits; tests reject an out-of-range target and a folding target.
- [ ] `poses10_safe.json` passes all checks; renders saved.
- [ ] `robot_package.json` v0 written and validated against `docs/robot_package.md`; contains no part uids, poses or meshes.
- [ ] Validator rejects test packages containing a mesh, a part id, a second box for one link and a file > 100 KB.
- [ ] Evaluation only: inferred ranges vs. ground-truth ranges in `LOG.md`.

## Fallbacks

- Coupled fit loses > 40 % of free space → use the grid as a lookup table.
- A sensor association looks wrong → mark that joint's sensor boundary `unverified`; Stage 5C measures it physically.

## Token estimate

Development $3–6.
