"""Modal preflight: CPU hello, cheapest-GPU nvidia-smi, Volume write/read.

Use ONLY via `modal run` (ephemeral). Never `modal deploy` with this file,
so no app is left running.

Local entrypoint flags (passed after `modal run <file>`):
  (no flag)  -> CPU check only
  --gpu       -> GPU check only (cheapest GPU, 3-min timeout)
  --volume    -> Volume check only (CPU, write small file + read back)
  --gpu --volume -> both checks
"""

from __future__ import annotations

import modal

app = modal.App("brickrobot-preflight")

VOLUME_NAME = "brickrobot-preflight-vol"
VOLUME_MOUNT = "/data"
# Cheapest Modal GPU at time of writing.
CHEAP_GPU = "T4"


@app.function(cpu=1.0, timeout=120)
def hello_remote() -> dict:
    import platform
    import sys

    return {
        "message": "hello",
        "python": platform.python_version(),
        "executable": sys.executable,
    }


@app.function(gpu=CHEAP_GPU, timeout=180)
def gpu_remote() -> dict:
    import shutil
    import subprocess

    if shutil.which("nvidia-smi") is None:
        return {"ok": False, "error": "nvidia-smi not found"}
    proc = subprocess.run(
        ["nvidia-smi", "--query-gpu=name,driver_version,memory.total",
         "--format=csv,noheader"],
        capture_output=True,
        text=True,
        timeout=60,
    )
    return {
        "ok": proc.returncode == 0,
        "returncode": proc.returncode,
        "stdout": proc.stdout.strip(),
        "stderr": proc.stderr.strip()[-500:],
    }


vol = modal.Volume.from_name(VOLUME_NAME, create_if_missing=True)


@app.function(volumes={VOLUME_MOUNT: vol}, timeout=120)
def volume_remote() -> dict:
    from pathlib import Path

    probe = Path(VOLUME_MOUNT) / "preflight_probe.txt"
    payload = "hello-volume"
    probe.write_text(payload, encoding="utf-8")
    read_back = probe.read_text(encoding="utf-8")
    return {
        "ok": read_back == payload,
        "path": str(probe),
        "wrote": payload,
        "read": read_back,
    }


@app.local_entrypoint()
def main(gpu: bool = False, volume: bool = False) -> None:
    ran = False
    if gpu:
        print("gpu:", gpu_remote.remote())
        ran = True
    if volume:
        print("volume:", volume_remote.remote())
        ran = True
    if not ran:
        print("cpu:", hello_remote.remote())
