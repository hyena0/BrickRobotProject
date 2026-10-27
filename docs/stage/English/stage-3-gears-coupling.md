# Stage 3: Gears and Coupling

**Latest:** Day 4 · **Goal:** for every motor, find the gear path to the joint it drives, compute the ratio and direction, and capture coupling between joints in a single matrix `C` where `q = C · m`.

## Key idea

When a drive shaft passes through a hollow joint (e.g., through a turntable), rotating that joint also turns the shaft relative to the next link, even if the motor is still. That is **coupling**. Each gear mesh obeys a carrier-relative constraint (Willis equation): for gears a and b mounted on carrier link L,

`T_a · (ω_a − ω_L) = s · T_b · (ω_b − ω_L)`

where `s` is −1 for external meshes, +1 for internal meshes, and set by geometry for bevel meshes. Writing one constraint per mesh and solving the linear system gives `C` exactly.

## Tasks

### 3.1 Transmission tools (≈2–3 h) · `src/tools/`

| Tool | Returns |
|---|---|
| `gear_graph()` | nodes = axle groups (from `trace_axle`), edges = meshes (from `check_gear_mesh`) with teeth and mesh kind |
| `axle_carrier(axle_group)` | the link each axle group is mounted in (from the Stage 2 links) |
| `solve_transmission(topology)` | given the agent's topology (which axle group is each motor output, which is fixed to each joint's child link, which meshes are active), builds the Willis constraints and returns `C`, per-joint ratios and signs, and any rank problems |

The solver is deterministic linear algebra. The agent supplies the topology; the solver supplies the numbers.

### 3.2 Agent extension

1. Call `gear_graph`; for each motor, follow meshes from its output shaft.
2. Decide which axle group ends each path (fixed to a child link of a joint, or drives a turntable that is the joint).
3. Mark inactive or decorative gears (no mesh partner, or meshing only with racks used as structure).
4. Recognise special mechanisms: differentials, shafts through hollow turntables, one pinion driving two gears.
5. Call `solve_transmission`; if rank is deficient or a ratio looks implausible, revisit the topology.
6. Output actuators, the matrix `C`, and a short explanation per joint. Brick and port for each motor come from `config/ports.local.yaml` (motor uid → port); the agent infers **which joint each motor drives**.

Note: motor uid → port is human-provided hardware wiring (README). Motor → joint is inferred and evaluated against `wiring_semantic.yaml`.

### 3.3 Evaluation · `src/verify/transmission.py`

- Per-joint ratio error (%) and direction sign versus ground truth.
- Full `C` comparison: max absolute element difference.
- Report in `LOG.md`.

## Thresholds

- Every ratio within 1 % and every sign correct.
- `C` off-diagonal (coupling) terms within 0.02 of ground truth.

## Exit criteria

- [ ] Thresholds met.
- [ ] The coupling explanation for the wrist joints is correct and readable (used in the video).

## Fallback (Day 4)

If coupling is wrong but direct ratios are right, keep direct ratios and add coupling terms as **human-provided** in README. Continue to Stage 4.

## Token estimate

Development $2–5. Per agent run up to $0.2 (longer traces than Stage 2).

## Muse Code prompt (example)

> Stage 3 task 3.1 in docs/stages/stage-3-gears-coupling.md. Implement gear_graph, axle_carrier and solve_transmission using carrier-relative (Willis) constraints. Add unit tests: simple 2-gear train, idler (sign check), internal mesh, and a shaft through a hollow rotating carrier (coupling term).
