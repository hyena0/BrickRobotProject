# Kinematic Model Schema

Public. Abstract kinematics only: no part IDs, no part poses.
The contract between stages; kept in sync by every stage.

## Frame

Base link frame, Z up, millimetres, right-handed.

## LDraw conversion

Parser input is LDraw in LDU (1 LDU = 0.4 mm) with `-Y` up.
Point `(x, y, z)` maps to `(0.4x, 0.4z, -0.4y)`.
Stored rotations are true world rotations `R = C @ L @ C^-1`, where
`C` is the point remap and `L` the LDraw matrix. With
`remap(r) = (r0, r2, -r1)`, the stored rows are `remap(L row 0)`,
`remap(L row 2)`, `-remap(L row 1)`. (A row-only remap would map
converted-local vectors back into the LDraw frame for rotated
parts.) Local direction `v` converts as `(vx, vz, -vy)` before the
stored rotation is applied. The map is a proper rotation, so
handedness is preserved.

## Model JSON

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

Field notes:

- `joints[].type` is `revolute`. Prismatic joints do not occur in this
  build; if one is ever inferred, the schema is extended first.
- `axis` is a unit vector in the parent link frame.
- `point_mm` is a point on the joint axis, in the parent link frame.
- `zero` is always `"pose in the source file"`: joint angles are zero
  at the Studio pose.
- `transmission.C` rows are joints, columns are motors.
- `actuators[].port` uses the hardware-layer names from
  `config/ports.local.yaml`. The semantic mapping (which port drives
  which joint) lives only in `eval/ground_truth/wiring_semantic.yaml`
  and is never shown to the agent.
