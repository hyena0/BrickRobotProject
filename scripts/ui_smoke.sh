#!/bin/bash
# UI smoke test: console + headless Chromium, one command.
# Phase 1 (connected): fake-brick server, ghost preview, homing, send.
# Phase 2 (virtual): --offline server, body follows IK, send hidden.
# Usage: scripts/ui_smoke.sh
# Only touches the local .venv and runs/ui_smoke/. Kills servers after.
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT" || exit 1
PY="$ROOT/.venv/bin/python"
OUT="$ROOT/runs/ui_smoke"
PORT=8766

if [ ! -x "$PY" ]; then
  echo "FAIL: .venv python not found at $PY"
  exit 1
fi

# Headless Chromium (user-level cache, no sudo, no system install).
if [ -z "$(ls ~/.cache/ms-playwright/ 2>/dev/null | grep -i chromium || true)" ]; then
  echo "Installing headless Chromium into ~/.cache/ms-playwright ..."
  "$PY" -m playwright install chromium || {
    echo "FAIL: playwright chromium install failed"
    exit 1
  }
fi

mkdir -p "$OUT"
# Deterministic first-load state: today's homing record (if any) is
# backed up and hidden so the run starts unhomed; the backup is
# restored on exit. Without this, a previous smoke run's fake-brick
# homing would enable send from the first screen.
DAY=$(date +%F)
HOMING="$ROOT/runs/stage5c/$DAY/homing.json"
BAK="$OUT/homing.json.bak"
HAD_HOMING=""
if [ -f "$HOMING" ]; then
  cp "$HOMING" "$BAK"
  HAD_HOMING="yes"
fi
rm -f "$HOMING"

SRV=""
cleanup() {
  if [ -n "$SRV" ]; then
    kill "$SRV" 2>/dev/null; wait "$SRV" 2>/dev/null
    SRV=""
  fi
}

wait_for_server() {
  for _ in $(seq 1 30); do
    if curl -sf -o /dev/null "http://127.0.0.1:$PORT/"; then
      return 0
    fi
    sleep 1
  done
  echo "FAIL: console server did not come up (see $OUT/server.log)"
  return 1
}

start_server() {
  # $1: extra args for src.console.control
  cleanup
  # shellcheck disable=SC2086
  "$PY" -m src.console.control --private-visuals robot_inferred \
    --port "$PORT" $1 >"$OUT/server.log" 2>&1 &
  SRV=$!
  wait_for_server
}

restore_homing() {
  cleanup
  if [ -n "$HAD_HOMING" ]; then
    cp "$BAK" "$HOMING"
  else
    rm -f "$HOMING"
  fi
}
trap restore_homing EXIT

C1=1
C2=1
if start_server ""; then
  "$PY" scripts/ui_smoke.py --phase connected \
    --port "$PORT" --out "$OUT"
  C1=$?
fi
if start_server "--offline"; then
  "$PY" scripts/ui_smoke.py --phase virtual \
    --port "$PORT" --out "$OUT"
  C2=$?
fi

if [ "$C1" -eq 0 ] && [ "$C2" -eq 0 ]; then
  echo "OVERALL (both phases): PASS"
  exit 0
else
  echo "OVERALL (both phases): FAIL (connected=$C1 virtual=$C2)"
  exit 1
fi
