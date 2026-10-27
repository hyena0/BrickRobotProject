# Progress Log

Public file. Record metrics as pass/fail and errors, **never ground-truth values**.

## Stage status

| Stage | Started | Finished | Result | Notes |
|---|---|---|---|---|
| 0. Setup | 2026-10-27 | | partial (PC-side PASS, hardware pending) | scaffold+protocol+brick/driver/check done; 9/9 PC tests PASS; fake check PASS; real-brick jog/sensor/estop/watchdog checks need hardware |
| 1. Data and tools | 2026-10-28 | | partial (parser+tools+schema done; catalog review, ground-truth values, wiring map, Blender import pending human) | |
| 2. Links and joints | 2026-10-28 | | partial (machinery done; Gate 1 UNSCORED, see blockers) | v4 heuristic: 284 links, 118 joint candidates; verifiers fail loudly on pending GT; live loop code-complete, execution blocked (no network) |
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
| 2026-10-28 | stage-2 v1 (offline heuristic) | 0 | 0 | 0 | 0 | $0, no model calls (no API key); 836 singleton links, 0 candidate joints; overlay written; both verifiers exit pending-GT |
| 2026-10-28 | stage-2 v2-v4 (reviewed catalog + fixed frames) | 0 | 0 | 0 | 0 | $0, offline only (no network: DNS fails, no openai SDK); catalog 76/84 reviewed, 8 flagged; v4: 284 links, 118 candidates; loop logic verified by 2 fake-client tests; live execution needs one networked run |

## Experiment log

| Date | Stage | Change | Metric before | Metric after | Keep? |
|---|---|---|---|---|---|
| 2026-10-28 | 1 | skip empty lines in MPD block split; fall back to largest type-1 block | parser 0 parts | parser 836 parts, count matches Studio | yes |
| 2026-10-28 | 2 | offline heuristic baseline on real parts.json (unreviewed catalog) | no output | 836 singleton links, 0 joints; verifiers fail loudly on pending GT | yes (v1 baseline; blockers: catalog review, human segmentation/measurement, wiring map) |
| 2026-10-28 | 2 | catalog review from embedded LDraw titles+geometry (84 types) | 84 x `other`, tools blind | 76 reviewed, 8 flagged with reasons; pin/axle/turntable/motor axes evidenced | yes (review.py + parts.yaml; audit in /tmp, regenerate anytime) |
| 2026-10-28 | 2 | frame-convention fix (stored R must be C@L@C^-1) | rotated parts mislocated axes/holes | 2 new tests failed before, pass after; 1 old trace order updated with reason | yes (parser+geometry+schema; parts.json regenerated, poses unchanged) |
| 2026-10-28 | 2 | hole-aware pin/axle/motor lines + v3/v4 baselines | 643 links, contacts only | 284 links (top groups 147/114/53/44/35), 118 candidates, motor shafts excluded | yes (v4 in runs/stage2-v4; agent prunes to 6 next) |
| 2026-10-28 | 2 | live loop dispatch + repair + fake-client tests | stub loop, never executes tools | 2 loop tests pass; live call blocked (no network/DNS) | yes (run.py; needs one networked smoke run) |
| 2026-10-28 | 2.8 | Blender visual check via local headless Blender (no MCP tool in this env) | no visual check | 80/84 part meshes imported, 8 final renders + .blend; true-color matches io silhouette; link colours show coherent groups, arrows at base/elbow/wrist | yes (src/viz/build_blend.py; 4 placeholder types noted; renders git-ignored) |
