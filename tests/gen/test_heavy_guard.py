"""Tests for scripts/heavy_guard.sh (subprocess, no mocks)."""

import re
import shutil
import subprocess

import pytest

GUARD = "scripts/heavy_guard.sh"


def _listeners():
    """{port: command} for local TCP LISTEN sockets (best effort)."""
    if shutil.which("lsof") is None:
        return {}
    proc = subprocess.run(["lsof", "-P", "-iTCP", "-sTCP:LISTEN"],
                          capture_output=True, text=True)
    found = {}
    for line in proc.stdout.splitlines()[1:]:
        fields = line.split()
        if len(fields) < 9:
            continue
        match = re.search(r":(\d+)\s*\(LISTEN\)", line)
        if match:
            found[int(match.group(1))] = fields[0]
    return found


def test_available_with_skips():
    proc = subprocess.run([GUARD, "--once", "--skip-proc", "--skip-port",
                           "--skip-load"], capture_output=True, text=True)
    assert proc.returncode == 0
    assert proc.stdout.strip() == "available"


def test_usage_error_and_help():
    proc = subprocess.run([GUARD, "--bogus"], capture_output=True,
                          text=True)
    assert proc.returncode == 2
    proc = subprocess.run([GUARD, "--help"], capture_output=True,
                          text=True)
    assert proc.returncode == 0
    assert "Exit codes" in proc.stdout


def test_busy_port_then_timeout(tmp_path):
    busy = {p: c for p, c in _listeners().items()
            if "blender" not in c.lower()}
    if not busy:
        pytest.skip("no non-blender TCP listener to borrow")
    port = sorted(busy)[0]
    proc = subprocess.run(
        [GUARD, "--once", "--port", str(port), "--skip-proc",
         "--skip-load"], capture_output=True, text=True)
    assert proc.returncode == 10
    assert "busy" in proc.stdout

    log = tmp_path / "LOG.md"
    proc = subprocess.run(
        [GUARD, "--port", str(port), "--skip-proc", "--skip-load",
         "--wait-secs", "1", "--interval-secs", "1", "--log", str(log)],
        capture_output=True, text=True)
    assert proc.returncode == 20
    assert "heavy local work skipped" in log.read_text()


def test_blender_held_port_is_expected():
    mcp = {p: c for p, c in _listeners().items()
           if "blender" in c.lower()}
    if not mcp:
        pytest.skip("no blender-held TCP port present")
    port = sorted(mcp)[0]
    proc = subprocess.run(
        [GUARD, "--once", "--port", str(port), "--skip-proc",
         "--skip-load"], capture_output=True, text=True)
    assert proc.returncode == 0
    assert proc.stdout.strip() == "available"
