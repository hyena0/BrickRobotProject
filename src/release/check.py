"""Minimal release guard (Stage 5C.4a, rough cut).

Fails when the git-tracked tree contains things that must never
ship, or when code pulls external URLs / embeds API keys:

1. tracked private paths: assets/private/, runs/, gt/,
   eval/ground_truth/, *.local.*, MR JOS originals
   (.io/.ldr/.mpd/.glb/.gltf/.stl/.obj/.blend*)
2. external URL loads in code (CDN hosts, http src/href)
3. API-key-like patterns in tracked text files

Usage: python -m src.release.check [--root PATH]
Exit 0 = pass, 1 = violations found.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys

def _private_dirs() -> tuple:
    try:
        text = (pathlib.Path(__file__).with_name(
            "private_paths.txt")).read_text(encoding="utf-8")
    except OSError:
        text = ""
    return tuple(line.strip() for line in text.splitlines()
                 if line.strip() and not line.startswith("#"))


PRIVATE_SUFFIXES = (".local.",)
ORIGINAL_SUFFIXES = (".io", ".ldr", ".mpd", ".glb", ".gltf",
                     ".stl", ".obj", ".blend", ".blend1")

# Only actual loads fail the guard (comments, license headers and
# API base URLs do not load anything into a served page).
LOAD_RE = re.compile(
    r"((src|href)\s*=\s*[\"']https?://"
    r"|@import\s+[\"']https?://"
    r"|import\s*\(\s*[\"']https?://"
    r"|fetch\s*\(\s*[\"']https?://"
    r"|\.open\s*\(\s*[\"'](?:GET|POST)[\"']\s*,\s*[\"']https?://"
    r"|cdn\.|unpkg\.|cdnjs|jsdelivr|gstatic)",
    re.IGNORECASE)

KEY_RES = [
    re.compile(r"api[_-]?key\s*=\s*[\"'][^\"']{4,}"),
    re.compile(r"sk-[A-Za-z0-9]{10,}"),
    re.compile(r"AKIA[0-9A-Z]{16}"),
    re.compile(r"xox[bap]-[A-Za-z0-9-]+"),
]

TEXT_SUFFIXES = (".py", ".js", ".html", ".md", ".yaml", ".yml",
                 ".json", ".toml", ".cfg", ".ini", ".sh", ".css")
MAX_SCAN_BYTES = 1 << 20


def tracked_files(root: pathlib.Path) -> list[str]:
    out = subprocess.run(["git", "ls-files"], cwd=root,
                         capture_output=True, text=True, check=False)
    if out.returncode != 0:
        raise RuntimeError("git ls-files failed (not a repo?)")
    return out.stdout.splitlines()


def private_hits(paths: list[str],
                 dirs: tuple | None = None) -> list[str]:
    if dirs is None:
        dirs = _private_dirs()
    hits = []
    name_suffix = (".local.yaml", ".local.json")
    for p in paths:
        low = p.lower()
        if (p.startswith(dirs)
                or low.endswith(name_suffix)
                or low.endswith(ORIGINAL_SUFFIXES)):
            hits.append(p)
    return hits


def url_hits(root: pathlib.Path,
             paths: list[str]) -> list[str]:
    hits = []
    for p in paths:
        if not p.startswith(("src/", "scripts/")):
            continue
        if not p.lower().endswith((".py", ".js", ".html")):
            continue
        try:
            text = (root / p).read_text(
                encoding="utf-8", errors="replace")
        except OSError:
            continue
        for i, line in enumerate(text.splitlines(), 1):
            if LOAD_RE.search(line):
                hits.append(f"{p}:{i}")
                break
    return hits


def key_hits(root: pathlib.Path,
             paths: list[str]) -> list[str]:
    hits = []
    for p in paths:
        if not p.lower().endswith(TEXT_SUFFIXES):
            continue
        try:
            if (root / p).stat().st_size > MAX_SCAN_BYTES:
                continue
            text = (root / p).read_text(
                encoding="utf-8", errors="replace")
        except OSError:
            continue
        for i, line in enumerate(text.splitlines(), 1):
            if any(r.search(line) for r in KEY_RES):
                hits.append(f"{p}:{i}")
                break
    return hits


def check(root: pathlib.Path) -> list[str]:
    paths = tracked_files(root)
    problems = []
    problems += [f"private: {p}" for p in private_hits(paths)]
    problems += [f"external-url: {p}" for p in url_hits(root, paths)]
    problems += [f"api-key?: {p}" for p in key_hits(root, paths)]
    return problems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="minimal release guard")
    ap.add_argument("--root", default=".",
                    help="repository root (default: cwd)")
    args = ap.parse_args(argv)
    try:
        problems = check(pathlib.Path(args.root))
    except RuntimeError as e:
        print(f"release check ERROR: {e}")
        return 2
    for p in problems:
        print("VIOLATION:", p)
    print("release check:", "FAIL" if problems else "PASS",
          f"({len(problems)} violations)")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
