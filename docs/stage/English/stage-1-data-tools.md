# Stage 1: Data and Tools

**Latest:** Day 2 · **Goal:** turn the Studio file into structured parts, give the agent accurate geometry tools, and prepare ground truth for evaluation.

## Tasks

### 1.1 Export and parse (≈2 h) · `src/parse/`

- Export the model from Studio as LDraw (`.mpd`/`.ldr`) into `assets/private/`.
- Parse LDraw:
  - Type-1 lines: `1 <colour> x y z a b c d e f g h i <part>`. Position in LDU (1 LDU = 0.4 mm), 3×3 rotation, part file name.
  - Flatten sub-models (`0 FILE` blocks) by composing transforms.
  - Convert to millimetres and a right-handed, **Z-up** frame (LDraw uses −Y up). Document the conversion in `docs/schema.md`.
- Output `runs/<id>/parts.json` (private): `uid`, `part_id`, `colour`, `position_mm`, `rotation`.
- Keep sub-model names out of agent input; they may reveal the designer's grouping.

### 1.2 Part catalog (≈2–3 h) · `src/catalog/parts.yaml`

Only the part types used in this model (≈84) plus common Technic parts for generalization.

| Field | Meaning |
|---|---|
| `category` | gear, turntable, worm, rack, axle, pin, bush, beam, frame, connector, motor, sensor, brick, other |
| `teeth` | for gears/turntables/racks |
| `gear_kind` | spur, double-bevel, bevel, crown, worm, internal |
| `axis_local` | rotation/insertion axis in the part's local frame |
| `holes` | list of (local position, axis, kind: `axle` / `round`) |
| `axle_length` | for axles, in studs |
| `motor` | `large` / `medium`, output-shaft position and axis |

- Start from LDraw part titles (e.g., "Technic Gear 24 Tooth") to fill `category` and `teeth` automatically, then check axes and holes by hand.
- For beams, generate holes from the beam length (20 LDU spacing).
- Spark may help draft entries from part titles. Every entry is reviewed by a human.

### 1.3 Geometry tools (≈3 h) · `src/tools/`

All deterministic, all tested. Tools return **facts**, never conclusions such as "this is a joint".

| Tool | Returns |
|---|---|
| `get_parts(category=None, near=None, radius_mm=None)` | uid, part_id, category, position, axis |
| `find_connections(uid)` | neighbours and connection type: `axle_in_axle_hole` (locks rotation), `axle_in_round_hole` (free rotation), `pin_friction`, `pin_frictionless`, `stud`, `contact` |
| `check_gear_mesh(uid_a, uid_b)` | axis relation (parallel / perpendicular / skew), centre distance, expected distance, `meshes: bool`, mesh kind (external / internal / bevel / worm) |
| `trace_axle(uid)` | all parts on the same axle line, ordered, with connection type |
| `rigid_groups(rigid_types)` | union-find groups when the given connection types are treated as rigid: part count, bounding box, motors and sensors inside |

Mesh rules (tolerance ±2 LDU):

- Parallel spur and double-bevel gears: centre distance = 1.25 × (T₁ + T₂) LDU (e.g., 8T + 24T → 40 LDU).
- Perpendicular bevel, worm, rack, turntable and internal meshes: rules per `gear_kind` in the catalog. Verify each rule with a small test model built in Studio.

### 1.4 Kinematic schema · `docs/schema.md`

Public. Abstract kinematics only: no part IDs, no part poses.

```json
{
  "frame": "base link, Z up, mm",
  "links": [{"id": "L0"}, {"id": "L1"}],
  "joints": [{
    "id": "J1", "type": "revolute", "parent": "L0", "child": "L1",
    "axis": [0, 0, 1], "point_mm": [0, 0, 0],
    "limits_deg": [-180, 180], "zero": "pose in the source file"
  }],
  "actuators": [{"id": "M1", "brick": "slave", "port": "B", "motor": "medium"}],
  "transmission": {
    "coupling": "q = C · m, q in joint degrees, m in motor degrees",
    "C": [[0.0]]
  },
  "end_effector": {"link": "L6", "point_mm": [0, 0, 0]}
}
```

### 1.5 Ground truth (private) · `eval/ground_truth/`

- `segmentation.json`: link membership per part, derived from the separated Studio file.
- `kinematics_gt.json`: same schema as 1.4, filled from the earlier analysis, the existing EV3 code constants and measurement.
- `robot_gt.xml`: MJCF generated from `kinematics_gt.json` with primitive geometry (use the generator from Stage 4 task 4.1 if ready; otherwise write by hand).
- `wiring_semantic.yaml`: the **semantic layer** of the wiring diagram: which joint each port drives, and what each sensor means (e.g., base yaw home). Used to evaluate Stage 3 actuator mapping and Stage 2 sensor-joint association.
- Nothing in this folder is ever passed to the agent.

### 1.6 Synthetic test models · `tests/fixtures/`

Small LDraw snippets made in Studio or by script: two meshing gears, a gear train with an idler, an axle through a round hole, a turntable joint. Expected tool outputs are written next to each fixture. These are public (own work).

### 1.7 Hardware wiring layer (human-provided) · `config/ports.local.yaml`

The wiring diagram has two layers. Only the **hardware layer** goes here; the semantic layer goes to `eval/ground_truth/wiring_semantic.yaml`.

```yaml
bricks:
  left:  {role: master, link: wifi_mqtt}
  right: {role: slave,  link: bluetooth_to_left}
motors:            # model motor uid → physical port (no joint names)
  - {uid: <motor uid from parts.json>, brick: left,  port: A}
sensors:           # model sensor uid → physical port and trigger (no meanings)
  - {uid: <sensor uid>, brick: left,  port: 1, type: touch, trigger: pressed}
  - {uid: <sensor uid>, brick: right, port: 2, type: color, trigger: "reflect > <threshold>"}
```

- Fill it once by matching each motor and sensor in `parts.json` to the diagram (6 motors, 5 sensors).
- **Do not** write joint numbers or sensor names such as "yaw base" here. The agent must infer which joint each motor drives and which joint each sensor homes.
- Check: every motor/sensor in the model has exactly one port; every port in `info` (Stage 0) has exactly one model part.
- README lists this file's content type as human-provided.

### 1.8 Blender setup for visual checks

- Install an LDraw importer add-on in Blender; confirm the exported model imports with correct scale and orientation.
- Connect Blender MCP to Muse Code (`~/.config/muse/settings.json`, `mcp_servers.blender`, `mode: optional`). Check with `/mcp`.
- Write `src/viz/blender_scene.py`: import the model, apply the same Z-up conversion as the parser, and tag each object with its part `uid` so later overlays can colour parts by link.
- All renders stay in `assets/private/` (appearance).

## Exit criteria

- [ ] Parser loads every part; part count matches Studio.
- [ ] Catalog covers 100% of part types in the model.
- [ ] All tools pass unit tests on the fixtures.
- [ ] Spot check on the real model: 5 known gear meshes and 3 known free-rotating connections are reported correctly.
- [ ] `docs/schema.md` written; ground-truth files exist and are git-ignored.
- [ ] `config/ports.local.yaml` complete and consistent with `info`.
- [ ] Model imports in Blender with part uids; Blender MCP visible in `/mcp`.

## Fallback

If connection detection is unreliable by Day 2, restrict `find_connections` to gears, axles, turntables and motors (enough for gear trains), and use `rigid_groups` with conservative defaults.

## Token estimate

Muse Code: $1–4 (catalog and tools are the largest coding effort).
