#!/bin/bash
# UI smoke test: fake-brick console + headless Chromium, one command.
# Usage: scripts/ui_smoke.sh
# Only touches the local .venv and runs/ui_smoke/. Kills its server after.
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
"$PY" -m src.console.control --private-visuals robot_inferred \
  --port "$PORT" >"$OUT/server.log" 2>&1 &
SRV=$!
cleanup() { kill "$SRV" 2>/dev/null; wait "$SRV" 2>/dev/null; }
trap cleanup EXIT

for _ in $(seq 1 30); do
  if curl -sf -o /dev/null "http://127.0.0.1:$PORT/"; then
    break
  fi
  sleep 1
done
if ! curl -sf -o /dev/null "http://127.0.0.1:$PORT/"; then
  echo "FAIL: console server did not come up (see $OUT/server.log)"
  exit 1
fi

"$PY" scripts/ui_smoke.py --port "$PORT" --out "$OUT"
