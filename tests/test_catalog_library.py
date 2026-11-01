"""Library tests: Studio tree loading (Stage 1 catalog).

UnOfficial part files begin with a self ``0 FILE <name>`` header line.
``Library.from_dir`` prepends its own synthetic header when registering,
so the self-header must be stripped: otherwise every consumer that stops
at the first ``0 FILE`` line (``title``, ``children``, viz face
collection) sees an empty part.
"""

from __future__ import annotations

import os

from src.catalog.review import Library, classify


def test_classify_general_rules_beyond_ev3():
    # Length without L, axle-pin family, cross blocks, non-EV3
    # electrics, neutral rack wording. EV3 precedents must not move.
    assert classify("Technic, Axle 32")["category"] == "axle"
    assert classify("Technic Axle Pin")["category"] == "axle"
    assert classify(
        "Technic Cross Block 1 x 3 (Pin/Pin/Pin)")["category"] == "connector"
    spike = classify("Electric, Motor SPIKE Prime, Small Angular")
    assert (spike["category"], spike["motor"]) == ("motor", "small")
    assert classify("Electric Control+ Hub")["category"] == "brick"
    assert "curved" not in classify(
        "Technic Gear Rack 1 x 14")["evidence"]
    assert classify("Electric, Motor EV3, Large")["motor"] == "large"
    assert classify("Electric Mindstorms EV3")["category"] == "brick"


def _write(root, rel, text):
    path = os.path.join(root, rel)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text)
    return path


def test_self_file_header_stripped_on_register(tmp_path):
    root = str(tmp_path)
    _write(root, os.path.join("UnOfficial", "parts", "x1234.dat"),
           "0 FILE x1234.dat\n"
           "0 Test Part With Self Header\n"
           "3 16 0 0 0 1 0 0 0 1 0\n")
    lib = Library.from_dir(root, ["x1234.dat"])
    rng = lib.blocks.get("x1234.dat")
    assert rng is not None
    first = lib.lines[rng[0] + 1].strip()
    assert not first.startswith("0 FILE")
    assert lib.title("x1234.dat") == "Test Part With Self Header"


def test_transitive_children_visible_past_self_header(tmp_path):
    root = str(tmp_path)
    _write(root, os.path.join("UnOfficial", "parts", "parent.dat"),
           "0 FILE parent.dat\n"
           "1 16 0 0 0 1 0 0 0 1 0 0 0 1 child.dat\n")
    _write(root, os.path.join("UnOfficial", "parts", "child.dat"),
           "0 Child Part\n"
           "3 16 0 0 0 1 0 0 0 1 0\n")
    lib = Library.from_dir(root, ["parent.dat"])
    assert "child.dat" in lib.blocks
    kids = lib.children("parent.dat")
    assert [c[2] for c in kids] == ["child.dat"]
