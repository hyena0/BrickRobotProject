"""Printable 1:1 slot overlay for the physical mat (A3 landscape).

The 2x3 pointing slots (S1..S6, shared with src.vla.scene.SLOT_IDS) plus
the 9 mat-point registration crosses, in the mat frame (base mm).
Print at 100% on A3, overlay on the existing mat print by matching the
numbered crosses, then place real blocks on the S1..S6 marks only.
A 100 mm scale bar verifies the 1:1 print.

Output: runs/vla/slots/slots_mat_a3.pdf (+ preview PNG).
Stdlib only.
"""

from __future__ import annotations

import json
import math
import os
import struct
import sys
import zlib

sys.path.insert(0, ".")

PT_PER_MM = 72.0 / 25.4
SHEET_W_MM, SHEET_H_MM = 420.0, 297.0
MARGIN_MM = 30.0


def load():
    mat = json.load(open("assets/private/mat_points.json", encoding="utf-8"))
    from src.vla.scene import SLOT_IDS
    return mat["points"], SLOT_IDS


def build_pdf(path, preview_path):
    points, slot_ids = load()
    xs = [p["x"] for p in points] + [x for x, y in slot_ids]
    ys = [p["y"] for p in points] + [y for x, y in slot_ids]
    x0, y0 = min(xs) - 10.0, min(ys) - 10.0

    def SX(x):
        return (MARGIN_MM + (x - x0)) * PT_PER_MM

    def SY(y):
        return (SHEET_H_MM - MARGIN_MM - (y - y0)) * PT_PER_MM

    ops = []
    ops.append("2 w")
    # registration crosses: 8 mm arms + number
    for p in points:
        x, y = SX(p["x"]), SY(p["y"])
        a = 4.0 * PT_PER_MM
        ops.append(f"{x - a} {y} m {x + a} {y} l S")
        ops.append(f"{x} {y - a} m {x} {y + a} l S")
        ops.append(f"BT /F1 10 Tf {x + a + 2} {y - 4} Td "
                   f"({p['n']}) Tj ET")
    # slots: 44 mm square + cross + S-label
    for (x, y), sid in sorted(slot_ids.items(), key=lambda kv: kv[1]):
        cx, cy = SX(x), SY(y)
        h = 22.0 * PT_PER_MM
        ops.append(f"{cx - h} {cy - h} m {cx + h} {cy - h} l "
                   f"{cx + h} {cy + h} l {cx - h} {cy + h} l h S")
        a = 6.0 * PT_PER_MM
        ops.append(f"{cx - a} {cy} m {cx + a} {cy} l S")
        ops.append(f"{cx} {cy - a} m {cx} {cy + a} l S")
        ops.append(f"BT /F1 14 Tf {cx + h + 4} {cy - 5} Td "
                   f"({sid}) Tj ET")
    # 100 mm scale bar
    bx, by = MARGIN_MM * PT_PER_MM, (MARGIN_MM - 12) * PT_PER_MM
    ops.append(f"{bx} {by} m {bx + 100 * PT_PER_MM} {by} l S")
    ops.append(f"{bx} {by - 4} m {bx} {by + 4} l S")
    ops.append(f"{bx + 100 * PT_PER_MM} {by - 4} m "
               f"{bx + 100 * PT_PER_MM} {by + 4} l S")
    ops.append(f"BT /F1 10 Tf {bx} {by - 14} Td (100 mm - measure me) Tj ET")
    ops.append("BT /F1 12 Tf "
               f"{MARGIN_MM * PT_PER_MM} "
               f"{(SHEET_H_MM - MARGIN_MM + 8) * PT_PER_MM} Td "
               "(VLA slots overlay 1:1 - match crosses 1..9) Tj ET")
    stream = "\n".join(ops).encode("latin-1")

    objs = []
    objs.append("<< /Type /Catalog /Pages 2 0 R >>")
    objs.append("<< /Type /Pages /Kids [3 0 R] /Count 1 >>")
    objs.append(f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 "
                f"{SHEET_W_MM * PT_PER_MM:.2f} {SHEET_H_MM * PT_PER_MM:.2f}] "
                f"/Resources << /Font << /F1 5 0 R >> >> "
                f"/Contents 4 0 R >>")
    objs.append(f"<< /Length {len(stream)} >>\nstream\n".encode("latin-1")
                + stream + b"\nendstream")
    objs.append("<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>")

    pdf = bytearray(b"%PDF-1.4\n")
    offsets = []
    for i, body in enumerate(objs, start=1):
        if isinstance(body, str):
            body = body.encode("latin-1")
        offsets.append(len(pdf))
        pdf += f"{i} 0 obj\n".encode("latin-1") + body + b"\nendobj\n"
    xref = len(pdf)
    pdf += f"xref\n0 {len(objs) + 1}\n".encode("latin-1")
    pdf += b"0000000000 65535 f \n"
    for off in offsets:
        pdf += f"{off:010d} 00000 n \n".encode("latin-1")
    pdf += (f"trailer\n<< /Size {len(objs) + 1} /Root 1 0 R >>\n"
            f"startxref\n{xref}\n%%EOF\n").encode("latin-1")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as fh:
        fh.write(bytes(pdf))

    # preview PNG (geometry only): 2 px/mm top-down
    W = int(SHEET_W_MM * 2)
    H = int(SHEET_H_MM * 2)
    import numpy as np
    img = np.full((H, W, 3), 1.0)
    allx = [MARGIN_MM + (x - x0) for x in xs]
    ally = [MARGIN_MM + (y - y0) for y in ys]

    def px(x):
        return int((MARGIN_MM + (x - x0)) * 2)

    def py(y):
        return int((MARGIN_MM + (y - y0)) * 2)
    for p in points:
        cx, cy = px(p["x"]), py(p["y"])
        img[cy - 8:cy + 8, cx - 1:cx + 1] = (0, 0, 0)
        img[cy - 1:cy + 1, cx - 8:cx + 8] = (0, 0, 0)
    for (x, y) in slot_ids:
        cx, cy = px(x), py(y)
        img[cy - 44:cy + 44, cx - 44:cx - 43] = (0.8, 0.1, 0.1)
        img[cy - 44:cy + 44, cx + 43:cx + 44] = (0.8, 0.1, 0.1)
        img[cy - 44:cy - 43, cx - 44:cx + 44] = (0.8, 0.1, 0.1)
        img[cy + 43:cy + 44, cx - 44:cx + 44] = (0.8, 0.1, 0.1)
        img[cy - 12:cy + 12, cx - 1:cx + 1] = (0.8, 0.1, 0.1)
        img[cy - 1:cy + 1, cx - 12:cx + 12] = (0.8, 0.1, 0.1)
    raw = bytearray()
    px8 = (np.clip(img, 0, 1) * 255 + 0.5).astype(np.uint8)
    for yy in range(H):
        raw.append(0)
        raw.extend(px8[yy].tobytes())

    def chunk(t, d):
        return (struct.pack(">I", len(d)) + t + d
                + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff))
    ihdr = struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
           + chunk(b"IDAT", zlib.compress(bytes(raw), 6))
           + chunk(b"IEND", b""))
    with open(preview_path, "wb") as fh:
        fh.write(png)
    print(f"wrote {path} + preview {preview_path}")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="runs/vla/slots/slots_mat_a3.pdf")
    ap.add_argument("--preview",
                    default="runs/vla/slots/slots_preview.png")
    args = ap.parse_args()
    build_pdf(args.out, args.preview)


if __name__ == "__main__":
    main()
