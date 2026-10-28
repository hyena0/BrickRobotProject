"""GT isolation regression (AGENTS.md rule 4).

Extraction/inference code must never import or read ground truth:
`gt/` paths reach code only as scorer CLI arguments. This test pins
that boundary by scanning every module under `src/` except
`src/verify/`: no non-docstring string literal may name a ground-truth
location. Docstring mentions that document the ban are allowed.
Real GT files (`gt/`, `eval/ground_truth/`) are never read here.
"""

from __future__ import annotations

import ast
import os

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, "src")

BANNED_FRAGMENTS = ("gt_joints.json", "ground_truth", "gt_measured",
                    "gt/gt_joints")


def _docstring_lines(tree):
    """Line ranges of module/class/function docstrings to excuse."""
    ranges = []

    def add(node):
        body = getattr(node, "body", [])
        if (body and isinstance(body[0], ast.Expr)
                and isinstance(body[0].value, ast.Constant)
                and isinstance(body[0].value.value, str)):
            seg = body[0]
            ranges.append((seg.lineno, seg.end_lineno or seg.lineno))

    add(tree)
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef,
                             ast.ClassDef)):
            add(node)
    return ranges


def _violations(path):
    with open(path, encoding="utf-8") as f:
        source = f.read()
    tree = ast.parse(source, filename=path)
    excused = _docstring_lines(tree)
    hits = []
    for node in ast.walk(tree):
        if not (isinstance(node, ast.Constant)
                and isinstance(node.value, str)):
            continue
        if not any(frag in node.value for frag in BANNED_FRAGMENTS):
            continue
        if any(start <= node.lineno <= end for start, end in excused):
            continue
        hits.append(f"{path}:{node.lineno}: {node.value!r}")
    return hits


def test_no_gt_access_outside_verify():
    bad = []
    for root, dirs, files in os.walk(SRC):
        if os.path.basename(root) == "verify":
            dirs[:] = []
            continue
        for name in sorted(files):
            if name.endswith(".py"):
                bad.extend(_violations(os.path.join(root, name)))
    assert bad == [], "ground-truth references outside src/verify/:\n" \
        + "\n".join(bad)
