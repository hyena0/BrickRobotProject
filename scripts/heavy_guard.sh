#!/bin/bash
# Stage 5D local-resource guard for heavy work (Blender, BVH sweeps).
#
# Exit codes: 0 resources available, 10 busy (retry later), 20 timed
# out waiting (skip the work). A check fails (busy) when any holds:
#   - a headless Blender batch job runs (Blender with -b: someone's
#     render or convert; the interactive/MCP Blender is expected and
#     does not block),
#   - the Blender MCP port is held by a NON-Blender process (a real
#     port conflict; the MCP Blender itself is the expected holder),
#   - 1-minute load average exceeds 70% of CPU count.
# Where a probe is unavailable (e.g. pgrep inside a sandbox), the
# guard warns and continues with the remaining checks.
# Usage: heavy_guard.sh [--port PORT] [--once] [--wait-secs S]
#                      [--interval-secs I] [--log PATH]
#                      [--skip-proc] [--skip-port] [--skip-load]
set -u

PORT="${BLENDER_MCP_PORT:-9876}"
ONCE=0
WAIT_SECS=1800
INTERVAL_SECS=300
LOG=""
SKIP_PROC=0
SKIP_PORT=0
SKIP_LOAD=0

while [ $# -gt 0 ]; do
    case "$1" in
        --port) PORT="$2"; shift 2 ;;
        --once) ONCE=1; shift ;;
        --wait-secs) WAIT_SECS="$2"; shift 2 ;;
        --interval-secs) INTERVAL_SECS="$2"; shift 2 ;;
        --log) LOG="$2"; shift 2 ;;
        --skip-proc) SKIP_PROC=1; shift ;;
        --skip-port) SKIP_PORT=1; shift ;;
        --skip-load) SKIP_LOAD=1; shift ;;
        -h|--help)
            sed -n '2,14p' "$0"; exit 0 ;;
        *) echo "unknown flag: $1" >&2; exit 2 ;;
    esac
done

check_once() {
    # Prints the blocking reason, or nothing when available.
    if [ "$SKIP_PROC" -eq 0 ]; then
        procs=$(pgrep -fa -i blender 2>&1)
        if echo "$procs" | grep -qi "cannot get process list\|not permitted"; then
            echo "warning: pgrep unavailable, skipping proc check" >&2
        elif echo "$procs" | grep -w -- "-b" | grep -qi blender; then
            echo "headless blender batch running:"
            echo "$procs" | grep -w -- "-b" | head -3
            return 1
        fi
    fi
    if [ "$SKIP_PORT" -eq 0 ]; then
        holders=$(lsof -i :"${PORT}" -sTCP:LISTEN 2>/dev/null | tail -n +2)
        if [ -n "$holders" ] \
                && ! echo "$holders" | awk '{print $1}' | grep -qi blender; then
            echo "port ${PORT} held by non-blender process:"
            echo "$holders" | head -3
            return 1
        fi
    fi
    if [ "$SKIP_LOAD" -eq 0 ]; then
        load=$(uptime | awk -F'load averages?:' '{print $2}' \
            | awk '{print $1}' | tr -d ',')
        ncpu=$(sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)
        if awk -v l="$load" -v n="$ncpu" \
                'BEGIN { exit !(l + 0 > 0.7 * n) }'; then
            echo "load ${load} above 70% of ${ncpu} CPUs"
            return 1
        fi
    fi
    return 0
}

if [ "$ONCE" -eq 1 ]; then
    if reason=$(check_once); [ -z "$reason" ]; then
        echo "available"
        exit 0
    fi
    echo "busy: $reason"
    exit 10
fi

deadline=$((SECONDS + WAIT_SECS))
while true; do
    if reason=$(check_once); [ -z "$reason" ]; then
        echo "available"
        exit 0
    fi
    if [ "$SECONDS" -ge "$deadline" ]; then
        msg="heavy_guard: skipping after ${WAIT_SECS}s, still busy: $reason"
        echo "$msg"
        if [ -n "$LOG" ]; then
            today=$(date +%F)
            printf '| %s | 5D | heavy local work skipped | - | %s | yes |\n' \
                "$today" "$msg" >>"$LOG"
        fi
        exit 20
    fi
    echo "busy ($reason); retrying in ${INTERVAL_SECS}s"
    sleep "$INTERVAL_SECS"
done
