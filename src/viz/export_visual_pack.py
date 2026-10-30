"""Export the private visual pack (Stage 5A.6, PC authoring side).

One decimated .glb per link (Blender, link frame) plus manifest.json
into assets/private/visual_pack/<robot>/. Private and git-ignored:
development, testing and demo video only, shown in the console only
behind --private-visuals. Never referenced by the robot package.

Run: Blender -b --factory-startup --python-exit-code 1 \\
         -P src/viz/export_visual_pack.py -- --robot robot_inferred
"""

import json
import os
import sys

REPO = None
MAX_TRIS = 200000


def parse_argv():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = {"robot": "robot_inferred",
           "blend": "runs/stage2-chain-r17/model_chain_r17.blend",
           "links": "runs/stage2-chain-r17/links_joints.json",
           "out": None}
    for i, a in enumerate(argv):
        if a in ("--robot", "--blend", "--links", "--out") \
                and i + 1 < len(argv):
            out[a[2:]] = argv[i + 1]
    if out["out"] is None:
        out["out"] = os.path.join("assets", "private", "visual_pack",
                                  out["robot"])
    return out


def main():
    global REPO
    args = parse_argv()
    import bpy  # noqa: E402
    from mathutils import Matrix  # noqa: E402

    links_doc = json.load(open(args["links"], encoding="utf-8"))
    REPO = os.getcwd()
    bpy.ops.wm.open_mainfile(
        filepath=os.path.join(REPO, args["blend"]))
    members = {link["id"]: link.get("members", [])
               for link in links_doc.get("links", [])}
    os.makedirs(args["out"], exist_ok=True)
    manifest = {"robot": args["robot"], "links": {}, "total_tris": 0}
    for lid, uids in members.items():
        objs = [bpy.data.objects[u] for u in uids
                if u in bpy.data.objects]
        if not objs:
            continue
        bpy.ops.object.select_all(action="DESELECT")
        for o in objs:
            o.select_set(True)
        bpy.context.view_layer.objects.active = objs[0]
        bpy.ops.object.duplicate()
        dupes = list(bpy.context.selected_objects)
        bpy.ops.object.join()
        joined = bpy.context.view_layer.objects.active
        # Decimate to a small triangle budget per link.
        tris = len(joined.data.polygons)
        if tris > 20000:
            mod = joined.modifiers.new("DEC_5A", "DECIMATE")
            mod.decimate_type = "COLLAPSE"
            mod.ratio = min(1.0, 20000.0 / max(tris, 1))
            bpy.ops.object.modifier_apply(modifier=mod.name)
        bpy.ops.object.transform_apply(location=False, rotation=False,
                                       scale=True)
        path = os.path.join(args["out"], lid + ".glb")
        bpy.ops.export_scene.gltf(
            filepath=path, export_format="GLB",
            use_selection=True, export_materials="NONE")
        n_tris = len(joined.data.polygons)
        manifest["links"][lid] = {"file": lid + ".glb",
                                  "mesh_to_link": "identity(link frame)",
                                  "tris": n_tris}
        manifest["total_tris"] += n_tris
        bpy.data.objects.remove(joined, do_unlink=True)
    with open(os.path.join(args["out"], "manifest.json"), "w",
              encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2)
    print(f"links={len(manifest['links'])} "
          f"tris={manifest['total_tris']} -> {args['out']}")
    if manifest["total_tris"] > MAX_TRIS:
        raise SystemExit(f"visual pack {manifest['total_tris']} tris "
                         f"over the {MAX_TRIS} budget")


if __name__ == "__main__":
    main()
