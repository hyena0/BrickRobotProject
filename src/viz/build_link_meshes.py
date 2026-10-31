"""Build per-link triangle meshes from LDraw (pure Python, no Blender).

Reads the assembled model (.ldr/.mpd), resolves each part instance's
triangles from the LDraw library, groups part instances by link using
the Stage-2 links file, and writes one .glb per link plus manifest.json
into assets/private/visual_pack/<robot>/ (git-ignored).

Link-frame convention follows src/viz/export_visual_pack.py:
``mesh_to_link`` is ``identity(link frame)``, i.e. mesh vertices are in
the model frame and the viewer poses link k with the FK frame F_k.
A zero-pose AABB check against the package link_envelopes runs before
writing and fails loudly on mismatch.

Run: python -m src.viz.build_link_meshes [--ldr ...] [--links ...]
     [--lib \"/Applications/Studio 2.0/ldraw\"] [--robot NAME]
"""

from __future__ import annotations

import argparse
import json
import os
import struct
import sys

LDU_TO_MM = 0.4


# -- LDraw library parsing (LDraw frame, LDU units) -------------------------

def _find_dat(lib, part_id, ref_dir=None):
    """Resolve an LDraw subfile reference to a library path.

    References use backslash separators and may live in subdirs
    (parts/s/, p/48/). Resolution order: the referencing file's own
    directory, the library roots, then the well-known subdirs.
    """
    rel = part_id.replace("\\", os.sep).lower()
    cands = []
    if ref_dir:
        cands.append(os.path.join(ref_dir, rel))
    cands.extend(os.path.join(lib, sub, rel)
                 for sub in ("parts", "p"))
    base = os.path.basename(rel)
    cands.extend(
        os.path.join(lib, sub, base)
        for sub in (os.path.join("parts", "s"),
                    os.path.join("p", "48"),
                    os.path.join("UnOfficial", "parts"),
                    os.path.join("UnOfficial", "p"),
                    os.path.join("UnOfficial", "parts", "s"),
                    os.path.join("UnOfficial", "p", "48")))
    for path in cands:
        if os.path.isfile(path):
            return path
    return None


def _parse_dat(path, lib, cache, stack):
    """Triangles + edge segments in local LDraw frame.

    Returns (tris, edges): tris [(colour, [v,v,v])], edges [(x,y)...] in
    LDraw colour 24 (edge colour) resolved at use time; stored raw here
    as point pairs, colour applied by the caller.
    """
    key = os.path.normpath(path).lower()
    if key in cache:
        return cache[key]
    if key in stack:
        return ([], [])
    stack = stack + [key]
    tris = []
    edges = []
    with open(path, encoding="utf-8-sig") as fh:
        text = fh.read()
    for line in text.splitlines():
        s = line.strip()
        if not s or s[0] not in "1234":
            continue
        t = s.split()
        try:
            kind = int(t[0])
        except ValueError:
            continue
        if kind == 1 and len(t) == 15:
            try:
                nums = [float(x) for x in t[1:14]]
            except ValueError:
                continue
            colour = t[1]
            sub = _find_dat(lib, t[14],
                            ref_dir=os.path.dirname(path))
            if sub is None:
                continue
            # type-1: colour, tx,ty,tz, then row-major 3x3.
            tx, ty, tz = nums[1], nums[2], nums[3]
            rows = [(nums[4], nums[5], nums[6], tx),
                    (nums[7], nums[8], nums[9], ty),
                    (nums[10], nums[11], nums[12], tz)]
            sub_tris, sub_edges = _parse_dat(sub, lib, cache, stack)
            cur = None if colour == "16" else colour
            for c, tri in sub_tris:
                out = []
                for v in tri:
                    out.append([
                        rows[0][0] * v[0] + rows[0][1] * v[1]
                        + rows[0][2] * v[2] + rows[0][3],
                        rows[1][0] * v[0] + rows[1][1] * v[1]
                        + rows[1][2] * v[2] + rows[1][3],
                        rows[2][0] * v[0] + rows[2][1] * v[1]
                        + rows[2][2] * v[2] + rows[2][3]])
                tris.append((c if cur is None else cur, out))
            for c, seg in sub_edges:
                out = []
                for v in seg:
                    out.append([
                        rows[0][0] * v[0] + rows[0][1] * v[1]
                        + rows[0][2] * v[2] + rows[0][3],
                        rows[1][0] * v[0] + rows[1][1] * v[1]
                        + rows[1][2] * v[2] + rows[1][3],
                        rows[2][0] * v[0] + rows[2][1] * v[1]
                        + rows[2][2] * v[2] + rows[2][3]])
                edges.append((c if cur is None else cur, out))
        elif kind in (3, 4) and len(t) in (11, 14):
            try:
                nums = [float(x) for x in t[2:]]
            except ValueError:
                continue
            colour = t[1]
            vs = [nums[i:i + 3] for i in range(0, len(nums), 3)]
            if len(vs) < 3:
                continue
            tris.append((colour, vs[:3]))
            if len(vs) == 4:
                tris.append((colour, [vs[0], vs[2], vs[3]]))
        elif kind == 2 and len(t) == 8:
            try:
                nums = [float(x) for x in t[2:]]
            except ValueError:
                continue
            colour = t[1]
            vs = [nums[i:i + 3] for i in range(0, len(nums), 3)]
            if len(vs) == 2:
                edges.append((colour, vs))
    cache[key] = (tris, edges)
    return tris, edges


def _to_model_frame(tri_ldu, rotation, position):
    """LDraw-frame LDU tri -> model-frame mm tri (matches ldr.py map)."""
    out = []
    for v in tri_ldu:
        q = [v[0] * LDU_TO_MM, v[2] * LDU_TO_MM, -v[1] * LDU_TO_MM]
        out.append([
            rotation[0][0] * q[0] + rotation[0][1] * q[1]
            + rotation[0][2] * q[2] + position[0],
            rotation[1][0] * q[0] + rotation[1][1] * q[1]
            + rotation[1][2] * q[2] + position[1],
            rotation[2][0] * q[0] + rotation[2][1] * q[1]
            + rotation[2][2] * q[2] + position[2]])
    return out


# -- colours ------------------------------------------------------------------

LDRAW_RGB = {
    "0": [0x1B, 0x1B, 0x1B], "1": [0x00, 0x50, 0x9C],
    "2": [0x00, 0x7A, 0x36], "4": [0xC9, 0x00, 0x00],
    "7": [0x96, 0x96, 0x96], "14": [0xF5, 0xCD, 0x30],
    "15": [0xFF, 0xFF, 0xFF], "23": [0x00, 0x56, 0xD6],
    "70": [0x5A, 0x2E, 0x1B], "72": [0x5C, 0x67, 0x70],
    "191": [0xFE, 0x8A, 0x18], "216": [0xA5, 0xCA, 0xE4],
}


def _rgb(code):
    if code in LDRAW_RGB:
        return [c / 255.0 for c in LDRAW_RGB[code]]
    try:
        return [c / 255.0 for c in LDRAW_RGB[str(int(code))]]
    except (ValueError, KeyError):
        return [0.6, 0.6, 0.6]


# -- minimal GLB writer (stdlib only) ------------------------------------------

def _face_normal(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx, ny, nz = (uy * vz - uz * vy, uz * vx - ux * vz,
                  ux * vy - uy * vx)
    n = (nx * nx + ny * ny + nz * nz) ** 0.5
    if n < 1e-12:
        return (0.0, 0.0, 1.0)
    return (nx / n, ny / n, nz / n)


def _write_glb(path, positions, normals, indices, runs, materials):
    """Indexed soup: smooth-shaded triangles (mode 4) + edge lines (mode 1).

    positions/normals: flat float lists (3 per vertex).
    indices: flat int list. runs: [(material_index, start_index,
    index_count, gl_mode)].
    """
    assert len(positions) == len(normals)
    assert max(indices) * 3 + 2 < len(positions)
    pos_bytes = struct.pack("<%df" % len(positions), *positions)
    nrm_bytes = struct.pack("<%df" % len(normals), *normals)
    idx_bytes = struct.pack("<%dI" % len(indices), *indices)
    n_vert = len(positions) // 3
    views = [{"buffer": 0, "byteOffset": 0,
              "byteLength": len(pos_bytes)},
             {"buffer": 0, "byteOffset": len(pos_bytes),
              "byteLength": len(nrm_bytes)},
             {"buffer": 0,
              "byteOffset": len(pos_bytes) + len(nrm_bytes),
              "byteLength": len(idx_bytes)}]
    accessors = [{"bufferView": 0, "componentType": 5126,
                  "count": n_vert, "type": "VEC3",
                  "max": [0, 0, 0], "min": [0, 0, 0]},
                 {"bufferView": 1, "componentType": 5126,
                  "count": n_vert, "type": "VEC3"}]
    for i in range(n_vert):
        for k in range(3):
            v = positions[3 * i + k]
            accessors[0]["max"][k] = max(accessors[0]["max"][k], v)
            accessors[0]["min"][k] = min(accessors[0]["min"][k], v)
    primitives = []
    for mi, start, count, mode in runs:
        accessors.append({"bufferView": 2, "componentType": 5125,
                          "count": count, "type": "SCALAR",
                          "byteOffset": start * 4})
        prim = {"attributes": {"POSITION": 0},
                "indices": len(accessors) - 1,
                "material": mi, "mode": mode}
        if mode == 4:
            prim["attributes"]["NORMAL"] = 1
        primitives.append(prim)
    gltf = {"asset": {"version": "2.0",
                      "generator": "build_link_meshes"},
            "scene": 0, "scenes": [{"nodes": [0]}],
            "nodes": [{"mesh": 0}],
            "meshes": [{"primitives": primitives}],
            "materials": [{"name": "c" + str(i), "doubleSided": True,
                           "pbrMetallicRoughness": {
                               "baseColorFactor": _rgb(m) + [1.0],
                               "metallicFactor": 0.0,
                               "roughnessFactor": 0.8}}
                          for i, m in enumerate(materials)],
            "buffers": [{"byteLength": len(pos_bytes + nrm_bytes
                                               + idx_bytes)}],
            "bufferViews": views,
            "accessors": accessors}
    js = json.dumps(gltf, separators=(",", ":")).encode("utf-8")
    while len(js) % 4:
        js += b" "
    bin_blob = pos_bytes + nrm_bytes + idx_bytes
    with open(path, "wb") as fh:
        fh.write(struct.pack("<III", 0x46546C67, 2,
                             12 + 8 + len(js) + 8 + len(bin_blob)))
        fh.write(struct.pack("<II", len(js), 0x4E4F534A))
        fh.write(js)
        fh.write(struct.pack("<II", len(bin_blob), 0x004E4942))
        fh.write(bin_blob)


# -- assembly ------------------------------------------------------------------

def _weld_indexed(tris_by_colour, quantum=0.02):
    """Dedupe verts per colour; smooth normals averaged at shared verts.

    Returns (positions, normals, indices, runs, colours) for _write_glb.
    Indices are absolute vertex ids; runs carry index-buffer offsets.
    """
    positions, normals, indices, runs, colours = [], [], [], [], []
    for colour in sorted(tris_by_colour):
        idx_of = {}
        acc = {}
        for tri in tris_by_colour[colour]:
            n = _face_normal(*tri)
            for v in tri:
                q = (round(v[0] / quantum), round(v[1] / quantum),
                     round(v[2] / quantum))
                if q not in idx_of:
                    idx_of[q] = len(positions) // 3
                    positions.extend([v[0], v[1], v[2]])
                    normals.extend([0.0, 0.0, 0.0])
                    acc[q] = [0.0, 0.0, 0.0]
                acc[q][0] += n[0]
                acc[q][1] += n[1]
                acc[q][2] += n[2]
        start = len(indices)
        for tri in tris_by_colour[colour]:
            for v in tri:
                q = (round(v[0] / quantum), round(v[1] / quantum),
                     round(v[2] / quantum))
                indices.append(idx_of[q])
        for q, i in idx_of.items():
            a = acc[q]
            n = (a[0] ** 2 + a[1] ** 2 + a[2] ** 2) ** 0.5 or 1.0
            normals[3 * i] = a[0] / n
            normals[3 * i + 1] = a[1] / n
            normals[3 * i + 2] = a[2] / n
        colours.append(colour)
        runs.append((len(colours) - 1, start, len(indices) - start, 4))
    return positions, normals, indices, runs, colours


def _aabb(tris):
    lo = [1e18, 1e18, 1e18]
    hi = [-1e18, -1e18, -1e18]
    for tri in tris:
        for v in tri:
            for k in range(3):
                lo[k] = min(lo[k], v[k])
                hi[k] = max(hi[k], v[k])
    return lo, hi


def main(argv=None):
    ap = argparse.ArgumentParser(description="per-link glb from LDraw")
    ap.add_argument("--ldr", default="assets/private/model.ldr")
    ap.add_argument("--links",
                    default="runs/stage2-chain-r17/links_joints.json")
    ap.add_argument("--package",
                    default="packages/robot_inferred/robot_package.json")
    ap.add_argument("--lib", default="/Applications/Studio 2.0/ldraw")
    ap.add_argument("--robot", default="robot_inferred")
    ap.add_argument("--out", default=None)
    ap.add_argument("--budget", type=int, default=0,
                    help="per-link tri cap (0 = keep all)")
    args = ap.parse_args(argv)
    sys.path.insert(0, ".")
    from src.parse.ldr import parse_file

    parts = parse_file(args.ldr)
    links_doc = json.load(open(args.links, encoding="utf-8"))
    uid_to_link = {}
    for link in links_doc.get("links", []):
        for uid in link.get("members", []):
            uid_to_link[uid] = link["id"]
    link_ids = [link["id"] for link in links_doc.get("links", [])]
    per_link = {lid: {} for lid in link_ids}
    missing_parts = set()
    cache = {}
    for part in parts:
        lid = uid_to_link.get(part["uid"])
        if lid is None:
            continue
        dat = _find_dat(args.lib, part["part_id"])
        if dat is None:
            missing_parts.add(part["part_id"])
            continue
        bucket = per_link[lid].setdefault(part["uid"], [])
        edge_bucket = per_link[lid].setdefault(part["uid"] + "#edges", [])
        p_tris, p_edges = _parse_dat(dat, args.lib, cache, [])
        for colour, tri in p_tris:
            cur = part["colour"] if colour == "16" else colour
            bucket.append(
                (cur, _to_model_frame(tri, part["rotation"],
                                       part["position_mm"])))
        for colour, seg in p_edges:
            edge_bucket.append(
                ("edge", _to_model_frame(seg, part["rotation"],
                                          part["position_mm"])))
    if missing_parts:
        print("missing library parts:",
              sorted(missing_parts)[:10])
    out = args.out or os.path.join("assets", "private", "visual_pack",
                                   args.robot)
    os.makedirs(out, exist_ok=True)
    pkg = json.load(open(args.package, encoding="utf-8"))
    envelopes = {e["link"]: e for e in pkg.get("link_envelopes", [])}
    manifest = {"robot": args.robot, "links": {}, "total_tris": 0}
    worst = 1e18
    for lid in link_ids:
        buckets = per_link[lid]
        by_colour = {}
        edge_segs = []
        for uid, bucket in buckets.items():
            if uid.endswith("#edges"):
                # Edge (type-2) segments: emitted as-is (lines are cheap).
                edge_segs.extend(seg for _, seg in bucket)
                continue
            items = bucket
            if args.budget and len(bucket) > 8:
                total = sum(len(b) for u, b in buckets.items()
                            if not u.endswith("#edges"))
                if total > args.budget:
                    keep = max(8, int(len(bucket) * args.budget / total))
                    stride = len(bucket) / keep
                    items = [bucket[int(i * stride)]
                             for i in range(keep)]
            for colour, tri in items:
                by_colour.setdefault(colour, []).append(tri)
        positions, normals, indices, runs, colours = \
            _weld_indexed(by_colour)
        # Append edge verts (zero normals; line primitives ignore them).
        edge_start = len(indices)
        edge_base = len(positions) // 3
        for seg in edge_segs:
            for v in seg:
                positions.extend([v[0], v[1], v[2]])
                normals.extend([0.0, 0.0, 0.0])
                indices.append(edge_base)
                edge_base += 1
        if edge_segs:
            # LDraw edge colour: near-black lines (colour 0).
            colours.append("0")
            runs.append((len(colours) - 1, edge_start,
                         2 * len(edge_segs), 1))
        path = os.path.join(out, lid + ".glb")
        _write_glb(path, positions, normals, indices, runs, colours)
        tris = [tri for tris in by_colour.values() for tri in tris]
        lo, hi = _aabb(tris)
        env = envelopes.get(lid, {})
        c = env.get("center_mm", [0, 0, 0])
        s = env.get("size_mm", [0, 0, 0])
        # Envelopes are AABBs of member part *origins* (build.py), so the
        # full-detail mesh must contain them; anything else is a frame bug.
        gap = 1e18
        for k in range(3):
            gap = min(gap, (c[k] - s[k] / 2) - lo[k],
                      hi[k] - (c[k] + s[k] / 2))
        worst = min(worst, gap)
        mc = [(lo[k] + hi[k]) / 2 for k in range(3)]
        dev = max(abs(mc[k] - c[k]) for k in range(3))
        manifest["links"][lid] = {
            "file": lid + ".glb", "mesh_to_link": "identity(link frame)",
            "tris": len(tris), "aabb_mm": [lo, hi],
            "center_dev_mm": round(dev, 2)}
        manifest["total_tris"] += len(tris)
        print(f"{lid}: tris={len(tris)} env_gap={gap:.1f}mm "
              f"center_dev={dev:.1f}mm")
    with open(os.path.join(out, "manifest.json"), "w",
              encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=1)
    print(f"total_tris={manifest['total_tris']} min_gap={worst:.1f}mm"
          f" -> {out}")
    if worst < -2.0:
        raise SystemExit(
            f"ZERO-POSE MISMATCH: envelope sticks out of mesh by "
            f"{-worst:.1f}mm; check link frames before wiring "
            "the viewer.")


if __name__ == "__main__":
    main()
