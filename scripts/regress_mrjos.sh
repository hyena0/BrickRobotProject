#!/bin/bash
# MR JOS regression check — Stage 7 rule.
# After EVERY code change: run this. Any difference vs the baseline
# (runs/baseline/mrjos_before_stage7/) -> STOP and report to a human.
#
# Usage:
#   scripts/regress_mrjos.sh            # fast checks (hashes, joints, ratios, Gate 1, tests)
#   scripts/regress_mrjos.sh --full     # + Gate 2 re-run (n=500, slow, offline)
#   JOINTS=<new links_joints.json> COUPLING=<new pred_C.json> scripts/regress_mrjos.sh
#       # compare a fresh inference run against the baseline (0.5 deg / 1 mm)
#
# The live-console IK check (reachable 59/60) cannot run headless, so its
# baseline report is hash-pinned; re-running it is a manual milestone step.
set -u
BASE="runs/baseline/mrjos_before_stage7"
JOINTS="${JOINTS:-runs/stage2-chain-r17/links_joints.json}"
COUPLING="${COUPLING:-runs/stage3/pred_C.json}"
PY=".venv/bin/python"
FULL=0
[ "${1:-}" = "--full" ] && FULL=1
FAIL=0

say() { printf '%-28s %s\n' "$1" "$2"; }

echo "== MR JOS regression vs $BASE =="

# 1. Frozen canonical artifacts untouched
if (cd "$BASE" && sha256sum -c MANIFEST.sha256 > /tmp/regress_hash.log 2>&1); then
  say "frozen-hashes" "IDENTICAL ($(grep -c ': OK' /tmp/regress_hash.log) files)"
else
  say "frozen-hashes" "DIFFERS -- see /tmp/regress_hash.log"; FAIL=1
  grep -v ': OK' /tmp/regress_hash.log | head -5
fi

# 2. Joints: count identical, axes <= 0.5 deg, points <= 1 mm
JOINTS_OUT=$("$PY" - "$BASE/links_joints.json" "$JOINTS" <<'EOF'
import json, math, sys
base = json.load(open(sys.argv[1]))["joints"]
cur = {j["id"]: j for j in json.load(open(sys.argv[2]))["joints"]}
ok = True
if sorted(j["id"] for j in base) != sorted(cur):
    print(f"JOINT-COUNT-OR-ID DIFF: base={[j['id'] for j in base]} cur={sorted(cur)}")
    ok = False
for j in base:
    c = cur.get(j["id"])
    if c is None:
        continue
    a = j["axis"]; b = c["axis"]
    na = math.dist(a, (0, 0, 0)); nb = math.dist(b, (0, 0, 0))
    cosv = max(-1.0, min(1.0, sum(x*y for x, y in zip(a, b)) / (na * nb)))
    ang = math.degrees(math.acos(cosv))
    d = math.dist(j["point_mm"], c["point_mm"])
    stat = "ok" if (ang <= 0.5 and d <= 1.0) else "DIFF"
    if stat == "DIFF":
        ok = False
    print(f"  {j['id']}: axis {ang:.4f}deg (<=0.5) point {d:.4f}mm (<=1.0) {stat}")
print("JOINTS-IDENTICAL" if ok else "JOINTS-DIFFER")
EOF
)
echo "$JOINTS_OUT"
echo "$JOINTS_OUT" | grep -q "JOINTS-IDENTICAL" && say "joints" "IDENTICAL (count, axes, points)" || { say "joints" "DIFFERS"; FAIL=1; }

# 3. Gear ratios identical + coupling nonzero-pattern identical
RATIO_OUT=$("$PY" - "$BASE/pred_C.json" "$COUPLING" <<'EOF'
import json, sys
b = json.load(open(sys.argv[1])); c = json.load(open(sys.argv[2]))
for key in ("C", "C_raw"):
    if key in b or key in c:
        bb, cc = b.get(key), c.get(key)
        same = (bb == cc)
        print(f"  {key}: {'identical' if same else 'DIFFERS'}")
        if not same:
            print("RATIOS-DIFFER"); break
else:
    print("RATIOS-IDENTICAL")
    def nz(m):
        return {(i, j) for i, row in enumerate(m) for j, v in enumerate(row) if v}
    for key in ("C", "C_raw"):
        if key in b and nz(b[key]) != nz(c.get(key, [])):
            print(f"  {key} coupling pattern DIFFERS"); print("RATIOS-DIFFER"); break
    else:
        print("  coupling nonzero-pattern identical")
EOF
)
echo "$RATIO_OUT"
echo "$RATIO_OUT" | grep -q "RATIOS-IDENTICAL" && say "ratios+coupling" "IDENTICAL" || { say "ratios+coupling" "DIFFERS"; FAIL=1; }

# 4. Gate 1 rescore (fresh, current code): must stay 6/6 PASS
GATE1=$("$PY" -m src.verify.joints --pred runs/stage2-chain-r17/links_joints_public.json --gt gt/gt_joints.json 2>&1)
echo "$GATE1" | sed 's/^/  /'
PASSES=$(echo "$GATE1" | grep -c "PASS")
if [ "$PASSES" -ge 7 ]; then say "gate1" "6/6 PASS (no worse)"; else say "gate1" "WORSE ($PASSES PASS lines)"; FAIL=1; fi

# 5. Gate 2 / IK / GT-sim: hash-pinned; --full re-runs Gate 2 offline
if [ "$FULL" = "1" ]; then
  G2_OUT=$("$PY" -m src.verify.ik_eval --pred models/robot_inferred.json --gt gt/kinematics_gt_v2.json --n 500 --seed 0 2>&1 | tail -3)
  echo "$G2_OUT" | sed 's/^/  /'
  say "gate2-full" "see runs/stage4_gate2_v2.json comparison (manual verdict)"
else
  say "gate2+ik60+gtsim" "hash-pinned (re-run with --full / live console at milestones)"
fi

# 6. Existing test suite (default markers, same as CI)
if "$PY" -m pytest -q > /tmp/regress_pytest.log 2>&1; then
  say "pytest" "PASS ($(tail -1 /tmp/regress_pytest.log))"
else
  say "pytest" "FAIL -- see /tmp/regress_pytest.log"; FAIL=1
  tail -5 /tmp/regress_pytest.log
fi

echo "----------------------------------------"
if [ "$FAIL" = "0" ]; then echo "RESULT: NO DIFFERENCE (regression passes)"; else echo "RESULT: DIFFERS -- stop and report"; fi
exit "$FAIL"
