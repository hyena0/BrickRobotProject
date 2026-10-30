"""Confirm AABB contact candidates with exact mesh tests (headless).

Loads the base blend, rebuilds the joint-empty hierarchy from the
pose batch, and for every candidate pair runs a BVH face-intersection
test plus a vertex-containment ray cast (catches full enclosure,
which overlap() alone misses). Writes mesh-true contacts per pose.

Run via src.viz.pose_check (which launches Blender)::

    Blender -b --factory-startup --python-exit-code 1 \\
        -P src/viz/pose_confirm_inner.py -- --batch ... --candidates ... \\
        --out ...
"""

import json
import os
import sys

REPO = None
MM = 0.001


def parse_argv():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = {}
    for i, a in enumerate(argv):
        if a in ("--batch", "--candidates", "--out") and i + 1 < len(argv):
            out[a[2:]] = argv[i + 1]
    return out


def build_hierarchy(col, joints, members, link_ids):
    import bpy
    from mathutils import Vector
    empties = {}
    prev_pt = [0.0, 0.0, 0.0]
    parent = None
    for j in joints:
        e = bpy.data.objects.new("EJ_" + j["id"], None)
        col.objects.link(e)
        if parent is not None:
            e.parent = parent
        e.location = Vector([(a - b) * MM for a, b in
                             zip(j["point_mm"], prev_pt)])
        e.rotation_mode = "AXIS_ANGLE"
        empties[j["id"]] = e
        parent = e
        prev_pt = j["point_mm"]
    bpy.context.view_layer.update()
    for n, j in enumerate(joints):
        inv = empties[j["id"]].matrix_world.inverted()
        child = [bpy.data.objects[u]
                 for u in members.get(link_ids[n + 1], [])
                 if u in bpy.data.objects]
        for o in child:
            o.parent = empties[j["id"]]
            o.matrix_parent_inverse = inv
    return empties


def main():
    args = parse_argv()
    global REPO
    import bpy
    import bmesh
    from mathutils import Vector
    from mathutils.bvhtree import BVHTree

    batch = json.load(open(args["batch"], encoding="utf-8"))
    REPO = batch["repo"]
    os.chdir(REPO)
    bpy.ops.wm.open_mainfile(
        filepath=os.path.join(REPO, batch["base_blend"]))
    joints = batch["joints"]
    link_ids = batch["link_ids"]
    members = batch["members"]
    link_of = {}
    for lid, uids in members.items():
        for u in uids:
            link_of[u] = lid
    for o in bpy.data.objects:
        if o.name.startswith(("ax_", "ah_")):
            o.hide_render = True
            o.hide_viewport = True
    col = bpy.data.collections.get("robot")
    empties = build_hierarchy(col, joints, members, link_ids)

    candidates = json.load(open(args["candidates"], encoding="utf-8"))
    depsgraph = bpy.context.evaluated_depsgraph_get()
    out = {}
    for pose in batch["poses"]:
        for j, qd in zip(joints, pose["q_deg"]):
            ax = Vector(j["axis"]).normalized()
            import math
            empties[j["id"]].rotation_axis_angle = [
                math.radians(qd), ax[0], ax[1], ax[2]]
        bpy.context.view_layer.update()
        depsgraph.update()
        pairs = candidates.get(pose["id"], [])
        objs = set()
        for a, b in pairs:
            objs.add(a)
            objs.add(b)
        trees = {}
        meshes = {}
        for name in objs:
            o = bpy.data.objects.get(name)
            if o is None or o.type != "MESH":
                continue
            ev = o.evaluated_get(depsgraph)
            bm = bmesh.new()
            bm.from_object(ev, depsgraph)
            bm.transform(o.matrix_world)
            trees[name] = BVHTree.FromBMesh(bm)
            meshes[name] = bm
        true_hits = []
        for a, b in pairs:
            if a not in trees or b not in trees:
                continue
            hit = trees[a].overlap(trees[b])
            if not hit:
                # containment either way: sample vertices of one mesh
                # against the other via ray cast parity
                for src, dst in ((a, b), (b, a)):
                    inside = False
                    for k, v in enumerate(meshes[src].verts):
                        if k >= 12:
                            break
                        origin = v.co
                        _loc, _n, _idx, dist = trees[dst].ray_cast(
                            origin, Vector((1.0, 0.0, 0.0)))
                        if dist is not None and dist > 1e-6:
                            inside = True
                            break
                    if inside:
                        hit = [True]
                        break
            if hit:
                true_hits.append([a, b])
        for bm in meshes.values():
            bm.free()
        out[pose["id"]] = true_hits
        print(f"{pose['id']}: {len(true_hits)}/{len(pairs)} mesh-true")
    json.dump(out, open(args["out"], "w"), indent=1)
    print(f"wrote {args['out']}")


if __name__ == "__main__":
    main()
