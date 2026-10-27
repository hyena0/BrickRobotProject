# Stage Plans

Detailed plans for each stage in `PLAN.md` section 5. Work one stage at a time and move on as soon as the exit criteria pass.

| Stage | File | Latest |
|---|---|---|
| 0. Setup | [stage-0-setup.md](stage-0-setup.md) | Day 1 |
| 1. Data and tools | [stage-1-data-tools.md](stage-1-data-tools.md) | Day 2 |
| 2. Agent v1: links and joints | [stage-2-links-joints.md](stage-2-links-joints.md) | Day 3 (Gate 1) |
| 3. Gears and coupling | [stage-3-gears-coupling.md](stage-3-gears-coupling.md) | Day 4 |
| 4. Kinematics | [stage-4-kinematics.md](stage-4-kinematics.md) | Day 5 (Gate 2) |
| 5. Physical | [stage-5-physical.md](stage-5-physical.md) | Day 6 |
| 6. Stretch A / B | [stage-6-stretch.md](stage-6-stretch.md) | Day 8 |
| 7. Generalization | [stage-7-generalization.md](stage-7-generalization.md) | Day 8 |
| 8. Ship | [stage-8-ship.md](stage-8-ship.md) | Day 9, noon ET |

Progress log: [LOG.md](LOG.md)

## How to work a stage

1. Start a **new Muse Code session** for each stage (or each task inside a large stage). `AGENTS.md` loads automatically.
2. Tell Muse Code which stage file and task to work on. Do not paste long chat history.
3. Run the exit-criteria checks yourself and record the result in `LOG.md`.
4. If the stage passes, commit and move on. If the "Latest" day arrives first, apply the fallback written in the stage file.

## Rules that apply to every stage

- These docs are public. **Never write ground-truth values** (gear ratios, link lengths, joint lists, segmentation) into any file outside `eval/ground_truth/`.
- Model files, renders, agent logs with part IDs/poses stay in git-ignored folders (`assets/private/`, `eval/ground_truth/`, `runs/`).
- Long jobs run as unattended scripts. Read summaries, not logs.
- Default model: `muse-spark-1.3-contributor`, reasoning effort `medium`. Use `high` only for hard design or debugging steps.
