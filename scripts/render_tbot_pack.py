"""Render T-Bot visual-pack verification stills (headless, no Blender).

Parses D0..D3.glb POSITION accessors (numpy only), samples vertices,
and writes a home assembly plus one posed frame (rigid link offsets,
same as the console FK frames). Needs matplotlib only for drawing;
install it to render ( rendering without it fails with a hint, video
scenes are not covered by this script).

Run: .venv/bin/python scripts/render_tbot_pack.py [--no-caption]
Out (defaults): assets/public_models/tbot/renders/pack_home.png and
pack_posed.png when --out points at the public dir; default out is
assets/private/renders/tbot/ (motion figures stay private).
"""

from __future__ import annotations

import argparse
import json
import struct
import sys

import numpy as np

CAPTION = "T-Bot by glaysche (MOC-85202), used with permission"

LINKS = ["D0", "D1", "D2", "D3"]
COLORS = {"D0": (0.55, 0.55, 0.58), "D1": (0.95, 0.45, 0.15),
          "D2": (0.15, 0.55, 0.85), "D3": (0.9, 0.2, 0.35)}
POSE = {"D1": (120.0, 0.0, 0.0), "D2": (120.0, 0.0, -80.0),
        "D3": (120.0, 47.0, -80.0)}
MAX_POINTS = 15000


def glb_positions(path):
    """All POSITION verts of a .glb (mm, model frame)."""
    with open(path, "rb") as fh:
        data = fh.read()
    if data[:4] != b"glTF":
        raise ValueError(f"not a GLB: {path}")
    off = 12
    js, blob = None, None
    while off < len(data):
        n, typ = struct.unpack("<II", data[off:off + 8])
        chunk = data[off + 8:off + 8 + n]
        if typ == 0x4E4F534A:
            js = json.loads(chunk)
        elif typ == 0x004E4942:
            blob = chunk
        off += 8 + n
    pts = []
    for mesh in js["meshes"]:
        for prim in mesh["primitives"]:
            acc = js["accessors"][prim["attributes"]["POSITION"]]
            bv = js["bufferViews"][acc["bufferView"]]
            start = bv.get("byteOffset", 0) + acc.get("byteOffset", 0)
            count = acc["count"]
            arr = np.frombuffer(blob, dtype=np.float32,
                                count=count * 3,
                                offset=start).reshape(-1, 3)
            pts.append(arr.copy())
    return np.vstack(pts)


def draw(path, verts, pose, title, caption):
    try:
        import matplotlib
    except ImportError:
        raise SystemExit("rendering needs matplotlib: "
                         ".venv/bin/pip install matplotlib")
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rng = np.random.RandomState(3)
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")
    for lid in LINKS:
        v = verts[lid] + np.array(pose.get(lid, (0, 0, 0)))
        sel = rng.choice(len(v), size=min(len(v), MAX_POINTS),
                         replace=False)
        ax.scatter(v[sel, 0], v[sel, 1], v[sel, 2], c=[COLORS[lid]],
                   s=4, depthshade=False)
    ax.view_init(elev=18, azim=-55)
    ax.set_box_aspect((420, 200, 620))
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    fig.suptitle(title, fontsize=12)
    if caption:
        fig.text(0.5, 0.02, caption, ha="center", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print("saved", path)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pack",
                    default="assets/private/visual_pack/tbot")
    ap.add_argument("--out", default="assets/private/renders/tbot")
    ap.add_argument("--caption", default=CAPTION,
                    help="attribution line on each render "
                         "(no trademark notice in it)")
    ap.add_argument("--no-caption", action="store_true")
    args = ap.parse_args(argv)
    import os

    verts = {}
    for lid in LINKS:
        verts[lid] = glb_positions(os.path.join(args.pack, lid + ".glb"))
        print(lid, "verts", len(verts[lid]))
    cap = None if args.no_caption else args.caption
    os.makedirs(args.out, exist_ok=True)
    draw(os.path.join(args.out, "pack_home.png"), verts, {},
         "T-Bot LDraw pack assembly (home)", cap)
    draw(os.path.join(args.out, "pack_posed.png"), verts, POSE,
         "T-Bot LDraw pack posed (P1+120, P2+80, P3+47mm)", cap)
    return 0


if __name__ == "__main__":
    sys.exit(main())
