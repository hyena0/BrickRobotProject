# Robot Package Schema (v0: Stage 5A)

The robot package (`packages/<robot>/robot_package.json`) is the only
runtime input of the controller package (PLAN v2 section 5). It is
versioned, lightweight (<= 100 KB), and publishable under PLAN v1
section 4a: abstract motion facts only, nothing that rebuilds or
renders the robot.

## Top-level fields

| Field | Content |
|---|---|
| `schema_version` | Package schema version (`"0.1.0"` for v0). |
| `package_version` | Integer, bumped on every content change (5C adds wiring, offsets, measured limits). |
| `robot` | Generic robot key (never a designer model name). |
| `frame` | Base link frame, Z up, mm (same convention as `docs/schema.md`). |
| `joints` | Array of `{id, type, parent, child, axis, point_mm, limits_deg, limit_source}`. `limit_source.lo/hi` is `sensor`, `collision`, or (from 5C) `measured`. |
| `coupling` | `{coupling: "q = C m", C, motor_ids}`. v0 columns are generic motor slots (`M1..Mn` in actuator order); 5C binds them to brick ports. |
| `coupled_limits` | Array of `{joints: [Ja, Jb], a: [a1, a2], b, op}` meaning `a1*qJa + a2*qJb + b >= 0` (or `<= 0`). |
| `coupled_lookup` | Map `'JA_JB' -> {cells: [[qa, qb], ...], radius_deg}` (v0: `{}`, enforcement off — code retained behind `USE_COUPLED_LOOKUP`; ranges + inequalities gate). |
| `operating_margin_deg` | Normal motion stays this far inside every sensor boundary (5.0). |
| `end_effector` | `{link, point_mm}`. |
| `safe_poses` | Array of `{id, q_deg}` joint-angle lists only, each path-safe from home. |
| `speed_caps` | `{first_run_deg_s, max_deg_s, homing_deg_s}`. |
| `link_envelopes` | Optional display/clearance hint: at most one oriented box per link (`{link, center_mm, size_mm, orientation}`), 1 mm / 1 deg rounding. |

## Content constraints (enforced by `src/package/validate.py`)

Allowed: kinematics, limits, coupled constraints, margins, caps, safe
joint-angle lists, generic motor slots, one envelope box per link,
and (from 5C) port wiring, homing config, zero offsets, measured
limits.

Forbidden: meshes or any geometry beyond one box per link; part ids,
names, poses, counts, colours; sub-model names; images, renders,
textures, or references into `assets/private/`; free-text design
descriptions. The box count, precision, and file size rules above
are hard errors, not warnings.

## Private visual pack (not part of the package)

Real LEGO meshes per link live separately in
`assets/private/visual_pack/<robot>/` (git-ignored, built by
`src/viz/export_visual_pack.py`): one decimated `.glb` per link plus
`manifest.json`. The package never references it; the validator
rejects any such reference. The console shows it only behind
`--private-visuals`, and `src/release/check` must pass before any
release.
