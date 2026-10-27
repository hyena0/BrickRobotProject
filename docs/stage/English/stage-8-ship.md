# Stage 8: Ship

**Latest:** Day 9, 12:00 PM ET (hard deadline 9:00 PM ET) · **Goal:** a 3-minute video, a clear README and a clean public repo.

Keep one full day for this stage regardless of progress.

## 8.1 Repo cleanup (≈1 h)

- [ ] `git ls-files` contains nothing from `assets/private/`, `eval/ground_truth/`, `runs/`.
- [ ] Secret scan: search the history for `META_API_KEY`, `sk-`, IP addresses in configs.
- [ ] No `.io`, `.ldr`, `.mpd`, mesh files or renders of the robot anywhere in history.
- [ ] Repo is public; open it in a private browser window to check.

## 8.2 README (≈2 h)

1. One-paragraph summary (from `PLAN.md` section 1).
2. Demo video link.
3. How it works: pipeline diagram, agent/tool split.
4. Results: Gate 1/2 metrics, physical accuracy, generalization table, Stretch results if any.
5. **Built during the hackathon / External / Pre-existing / Evaluation only / Human-provided** table.
6. Credits: "Robot design: 6DOF Robotic Arm by MR JOS (Rebrickable MOC-76666), rebuilt and modified by the author. Model files are not included."
7. How to run with your own model.
8. Limitations and roadmap.

## 8.3 Video (≈4–5 h)

| Time | Content | Footage |
|---|---|---|
| 0:00–0:20 | Problem: easy to build, hard to control coupled gears | Physical arm, gear close-ups |
| 0:20–1:20 | Inference: agent trace (tool calls, reasoning summary), links coloured, gear paths highlighted | Screen capture + renders |
| 1:20–1:45 | Verification: metrics vs. ground truth, sim overlay | Plots, MuJoCo |
| 1:45–2:20 | Physical: robot reaches targets; "Where is the blue one?" → points at it | Tripod footage + camera view with SAM mask |
| 2:20–2:40 | VLA pointing in simulation, on the real robot if B2 is done; generalization table | MuJoCo grid, real footage, table |
| 2:40–3:00 | Roadmap: vision-only, error compensation, mobile VLA, education kit | Slides |

- Show only what was built during the hackathon.
- Subtitles in English.
- Upload unlisted, check the link in a private window.

## 8.4 Submission

- [ ] Submit at the Cerebral Valley portal by noon ET.
- [ ] Track: Indie. Theme: Push the Frontier.
- [ ] Repo link, video link, team member list (solo).
- [ ] Recheck that both links open without login.
