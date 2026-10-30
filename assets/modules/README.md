# 5D module library (human-built, public)

Each joint module is an original single-joint unit designed in Studio from
owned parts only. Modules are our own work and ship under the repo license.

## Layout

```text
assets/modules/<name>/
    module.json   # frames, joint, motor, BOM (validated)
    module.ldr    # LDraw geometry in the module frame (from Studio)
```

Planned modules: `M_base_yaw`, `M_pitch_worm`, `M_pitch_direct`,
`M_wrist_roll` (optional).

## Module frame

Both files share the module frame. The `.ldr` uses standard LDraw
convention (LDU, -Y up); `module.json` uses the same frame in millimetres
with Z up (`docs/schema.md` maps one to the other). Build the Studio model
around the origin with the joint at its zero pose.

## module.json format

```json
{
  "name": "M_base_yaw",
  "frame": "module local, Z up, mm",
  "geometry": "module.ldr",
  "joint": {
    "axis": [0, 0, 1],
    "point_mm": [0, 0, 0],
    "range_deg": [-150, 150],
    "joint_deg_per_motor_deg": 0.05
  },
  "motor": {"kind": "large"},
  "sensor_mount": {"kind": "touch", "position_mm": [10, 0, 5]},
  "input_mount": {"position_mm": [0, 0, 0], "rotation": [[1,0,0],[0,1,0],[0,0,1]]},
  "output_mount": {"position_mm": [0, 0, 40], "rotation": [[1,0,0],[0,1,0],[0,0,1]]},
  "size_mm": [60, 60, 50],
  "bom": {"95658.dat": 1, "11478.dat": 4}
}
```

- `joint.axis` is a unit vector in the module frame; `point_mm` is a point
  on the axis. `joint_deg_per_motor_deg` is joint degrees per motor degree
  (the `q = C m` entry of `docs/schema.md`), always positive here; the
  sign is resolved at assembly time.
- `motor.kind` is `large` (loaded joints: shoulder, elbow) or `medium`
  (base yaw, wrist).
- `sensor_mount` is `null` when the module has no touch-sensor dock; such
  joints home by stall detection.
- `input_mount` is where the parent chain (or ground) attaches;
  `output_mount` is where the child link attaches at the zero pose.
  Model both mounts with +Z pointing base-to-tip along the chain: the
  assembler extends the beam from the output mount along its +Z and
  places the next input mount there with the same orientation.
- `mount_in` / `mount_out` repeat the legacy mount as a 3x4 `frame`
  (mm) and add `holes`: 2+ entries of `{pos_ldu, type}` in module
  axes (LDU). Every hole must sit on the 10-LDU grid, span at least
  two pin lines (pins run along the mount frame +X), and coincide
  within 0.1 LDU with a beam hole centre in `module.ldr`. `type` is
  `pin` (fits any bore) or `axle` (needs a cross bore: an axle shaft
  must fit it). The assembler mates connector beams to these holes
  with one pin/axle element each.
- `bom` maps LDraw part ids to counts. Every id must be in the catalog
  (`src/catalog/parts.yaml`, plus new parts in `src/catalog/parts_5d.yaml`)
  and within owned stock (`config/inventory.local.yaml`).

## Adding a module (human steps)

1. Design the module in Studio from owned parts only.
2. Export the module `.ldr` (zero pose, module frame) into a new folder.
3. Measure input/output mounts, joint axis/point, range, size; write
   `module.json` (copy the skeleton above).
4. List the module BOM in `module.json` (Studio part list).
5. Validate: `PYTHONPATH=. .venv/bin/python -m src.gen.modules
   assets/modules/<name>/module.json`
