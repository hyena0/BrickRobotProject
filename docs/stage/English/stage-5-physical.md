# Stage 5: Physical Robot

**Latest:** Day 6 · **Goal:** generate the EV3 control program from the inferred model and drive the physical arm to commanded targets.

## Design

- **Host (PC):** IK and motor-space conversion (Stage 4).
- **Bricks:** the generated program adds motion primitives on top of the Stage 0 brick server: synchronized multi-motor moves and soft limits.
- The generated part is a file of constants and routines produced from the kinematic model, so a different robot gets a different program without hand edits.

## Tasks

### 5.1 Code generator (≈2 h) · `src/codegen/`

Produces `generated/<robot>/`:

- `robot_config.py`: joint list, limits, `C`, actuator map (brick, port, motor type), speed caps.
- `motion.py`: `move_joints(q_target, duration)` that converts to motor targets and sends `run_targets` so every motor starts and finishes together (speeds scaled by each motor's travel).
- `home.py`: homing routine. Sensor ↔ joint pairs and homing direction come from the agent (Stage 2.7); each homing move uses the brick's homing guard (`home_motor`), so the sensor stops the motor locally. Joints without a sensor use the stall guard.
- `guards.py`: the guard set sent to each brick with `set_guards` before any motion: every sensor in `ports.local.yaml` → stop all on trigger outside homing.

Generated files are public (abstract kinematics + code; no part uids).

### 5.2 Zero offsets · `config/homing.local.yaml`

- Offset from each home position to the model's zero pose, and colour thresholds if needed.
- This is **human-provided** calibration. Document it in README.

### 5.3 Bring-up sequence

1. Home all joints.
2. Move one joint at a time by +15°, check direction against the model. Fix sign errors in the **model** (and log them), never by patching generated code.
3. Move to 5 joint-space poses and back.
4. Move to 5–10 Cartesian targets marked on the table (position-only IK).
5. Repeat each target 3 times to measure repeatability.

### 5.4 Measurement

- Mark targets on paper with known coordinates relative to the base.
- Measure the end-effector tip error with a ruler (mm) for each target.
- Film every run from a fixed tripod angle (needed for the video).

## Safety

- Speed cap 30 % for the first full run.
- Soft limits from the model, narrowed by 5° on each side.
- Emergency stop key active in every script.
- Watch for gear slipping under load; reduce speed if it skips.

## Exit criteria

- [ ] Guards are active before every motion (check `set_guards` ack in the log).
- [ ] Pressing any sensor by hand during a normal move stops the arm and requires `clear_latch` + re-homing.
- [ ] Homing works three times in a row.
- [ ] All joint directions match the model.
- [ ] The arm reaches all Cartesian targets; errors recorded (no threshold; LEGO backlash is expected).
- [ ] Usable footage recorded.

## Fallback (Day 6)

If Cartesian moves are unreliable, show joint-space moves plus the simulation comparison. Skip Stretch A.

## Stretch A check

When this stage passes, count days left before Day 9. If ≥ 2, start Stretch A.

## Token estimate

Development $1–3.
