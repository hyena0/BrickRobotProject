# Stage 7: Generalization

**Latest:** Day 8 · **Goal:** show the same pipeline works on robots it has never seen, without code changes.

## Test set

| Source | Why | Ground truth |
|---|---|---|
| **Procedural arms** (`src/gen/procedural.py`) | Our own LDraw generator: 2–4 revolute joints, random link lengths, random gear trains (1–3 stages, idlers), optional shaft through a hollow joint | Known by construction; fully public |
| Public model with a clear licence (e.g., an official set model or an education model) | A real design made by someone else | From its instructions or official program |

Procedural arms are the main set: they are license-free, unlimited, and come with exact answers.

## Tasks

1. **Procedural generator** (≈3 h): writes LDraw files plus their ground-truth kinematics. Start with planar 2–3 joint arms, then add one coupled joint.
2. **Batch run** (`src/eval/batch.py`): run the agent on 10–20 generated robots. Respect the 150 RPM limit, or run part of the batch on Muse Spark Open via Baseten if credits were granted.
3. **One public model:** run once, report results.
4. **Report:** one table (robot, DOF, segmentation accuracy, ratio error, IK success) for README and the video.

## Optional: model comparison

Run the same batch with `muse-spark-1.3-contributor` and Muse Spark Open. Report accuracy and cost side by side.

## Exit criteria

- [ ] Batch completes without code changes between robots.
- [ ] Results table in `LOG.md` and README.

## Token estimate

Batch of 20 runs: $1–4. Development $2–4.
