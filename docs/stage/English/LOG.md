# Progress Log

Public file. Record metrics as pass/fail and errors, **never ground-truth values**.

## Stage status

| Stage | Started | Finished | Result | Notes |
|---|---|---|---|---|
| 0. Setup | 2026-10-27 | | partial (PC-side PASS, hardware pending) | scaffold+protocol+brick/driver/check done; 9/9 PC tests PASS; fake check PASS; real-brick jog/sensor/estop/watchdog checks need hardware |
| 1. Data and tools | 2026-10-28 | | partial (parser+tools+schema done; catalog review, ground-truth values, wiring map, Blender import pending human) | |
| 2. Links and joints | | | | |
| 3. Gears and coupling | | | | |
| 4. Kinematics | | | | |
| 5. Physical | | | | |
| 6. Stretch A / B | | | | |
| 7. Generalization | | | | |
| 8. Ship | | | | |

## Token and credit usage

| Date | Session / run | Turns | Input | Cached | Output | Cost | Notes |
|---|---|---|---|---|---|---|---|
| 2026-10-28 | stage-1 session | 48+ | ~1.5M | n/a | ~37K | dev-session (see PLAN 8) | user-reported baseline at stage-1 start; parser+tools built in same session |
| 2026-10-28 | stage-1 run | - | - | - | - | - | parser 836/836 parts, 84/84 types; 19/19 tests pass (9 stage-0 + 10 stage-1) |

## Experiment log

| Date | Stage | Change | Metric before | Metric after | Keep? |
|---|---|---|---|---|---|
| 2026-10-28 | 1 | skip empty lines in MPD block split; fall back to largest type-1 block | parser 0 parts | parser 836 parts, count matches Studio | yes |
