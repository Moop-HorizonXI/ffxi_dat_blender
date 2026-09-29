"""
FFXI DAT <-> Blender mesh import/export (skinned equipment / face / body meshes).

Import: reads a model DAT (e.g. 100.dat) plus the race skeleton DAT (e.g. 93.DAT for
Tarutaru), builds an armature, one Blender mesh per mesh chunk (0x2A), bone weights,
UVs, normals, materials, and the DXT textures.

Export: rebuilds each mesh chunk from the Blender mesh -- vertex and triangle counts
are free to change -- and writes a new DAT where every other chunk is copied byte for
byte from the source DAT.

Format facts this relies on (verified against real DATs):
  * FFXI space is -Y up, Z is the left/right axis. Blender = (z, -x, -y).
  * Vertices are stored in bone-local space. 2-bone vertices store
    x1 x2 y1 y2 z1 z2 w1 w2 nx1 nx2 ny1 ny2 nz1 nz2, with each bone's position
    and normal pre-multiplied by that bone's weight.
  * Bone reference u16: bits 0-6 bone-table index, bits 7-13 bone-table index used
    for the mirrored copy, bits 14-15 local axis flipped for the mirror (1=X 2=Y 3=Z).
  * Mesh flags u16 @0x04 == 1 -> the game also draws a copy mirrored across FFXI Z=0.
  * Triangles are clockwise; triangle lists hold at most 128 triangles.
"""

bl_info = {
    "name": "FFXI DAT Mesh Import/Export",
    "author": "XI Modding Tools for Preservation",
    "version": (1, 0, 0),
    "blender": (4, 1, 0),
    "location": "File > Import/Export > FFXI DAT",
    "description": "Edit FFXI model DATs in Blender, including adding new geometry",
    "category": "Import-Export",
}

import math
import os
import struct

try:
    import bpy
    import bmesh
    from bpy.props import StringProperty, BoolProperty
    from bpy_extras.io_utils import ImportHelper, ExportHelper
    from mathutils import Matrix, Vector, Quaternion
    from mathutils.kdtree import KDTree
except ImportError:  # allows the parsing half to be used from plain Python
    bpy = None

MAX_TRIS_PER_LIST = 128

# Every section count in the mesh header -- including the total-size field -- is a 16-bit
# number of 2-byte words, so a whole mesh chunk tops out at 65,535 words (~128 KB).
MAX_MESH_WORDS = 0xFFFF
MAX_BONES = 128                     # bone references use 7-bit bone-table indices
VERTEX_WORDS_1 = 12 + 2             # 1-bone vertex: 6 floats + 2 bone refs
VERTEX_WORDS_2 = 28 + 2             # 2-bone vertex: 14 floats + 2 bone refs
TRIANGLE_WORDS = 15                 # 3 indices + 6 UV floats


def mesh_words(n1, n2, n_bones, runs):
    """Size of a mesh chunk body in 16-bit words, exactly as build_mesh_chunk writes it.
    runs: [(has_material, has_texture, triangle_count)] in draw order."""
    poly = 1  # 0xFFFF end marker
    for has_mat, has_tex, n in runs:
        poly += 23 * has_mat + 9 * has_tex + 2 * math.ceil(n / MAX_TRIS_PER_LIST) + TRIANGLE_WORDS * n
    return 0x20 + poly + n_bones + 2 + VERTEX_WORDS_1 * n1 + VERTEX_WORDS_2 * n2
CHUNK_MESH, CHUNK_SKELETON, CHUNK_IMAGE = 0x2A, 0x29, 0x20


# --------------------------------------------------------------------------- chunks

def read_chunks(data):
    out, off = [], 0
    while off + 16 <= len(data):
        v = struct.unpack_from("<I", data, off + 4)[0]
        size = (v >> 7) * 16
        out.append(dict(name=data[off:off + 4], type=v & 0x7F, off=off, size=size,
                        raw=data[off:off + size]))
        if size <= 0:
            break
        off += size
    return out


def chunk_header(name, ctype, total_size):
    return name + struct.pack("<I", ctype | ((total_size // 16) << 7)) + bytes(8)


# --------------------------------------------------------------------------- skeleton

def parse_skeleton(data):
    for c in read_chunks(data):
        if c["type"] == CHUNK_SKELETON:
            b = c["off"] + 16
            count = struct.unpack_from("<H", data, b + 2)[0]
            bones = []
            for i in range(count):
                o = b + 4 + 30 * i
                parent, flag = struct.unpack_from("<BB", data, o)
                qx, qy, qz, qw = struct.unpack_from("<4f", data, o + 2)
                t = struct.unpack_from("<3f", data, o + 18)
                bones.append(dict(parent=parent, flag=flag, q=(qw, qx, qy, qz), t=t))
            return bones
    raise ValueError("No skeleton (0x29) chunk found - pick the race skeleton DAT")


def skeleton_bone_count(path):
    """Bone count if the file is a DAT containing a skeleton, else 0. Reads chunk headers only."""
    try:
        size = os.path.getsize(path)
        with open(path, "rb") as f:
            off = 0
            while off + 16 <= size:
                f.seek(off)
                head = f.read(20)
                v = struct.unpack_from("<I", head, 4)[0]
                csize = (v >> 7) * 16
                if csize < 16 or off + csize > size:
                    return 0  # not a well-formed DAT
                if v & 0x7F == CHUNK_SKELETON:
                    return struct.unpack_from("<H", head, 18)[0]
                off += csize
    except (OSError, struct.error):
        pass
    return 0


def find_skeletons(folder, exclude=None, max_files=400):
    """Skeleton DATs sitting in a folder (e.g. 93.DAT next to the model being edited)."""
    try:
        names = sorted(n for n in os.listdir(folder) if n.lower().endswith(".dat"))[:max_files]
    except OSError:
        return []
    out = []
    for n in names:
        p = os.path.join(folder, n)
        if exclude and os.path.normcase(os.path.abspath(p)) == os.path.normcase(os.path.abspath(exclude)):
            continue
        if skeleton_bone_count(p):
            out.append(p)
    return out


def highest_bone_used(model_data):
    """Largest skeleton bone index any mesh in the DAT refers to (-1 if no meshes)."""
    top = -1
    for c in read_chunks(model_data):
        if c["type"] == CHUNK_MESH:
            top = max([top] + parse_mesh(c["raw"])["bone_table"])
    return top


# --------------------------------------------------------------------------- mesh parse

def parse_mesh(raw):
    """raw = full chunk bytes including the 16-byte chunk header."""
    d = memoryview(raw)[16:]
    head = bytes(d[:0x40])
    flags = struct.unpack_from("<H", d, 4)[0]
    secs = [struct.unpack_from("<IH", d, 6 + i * 6) for i in range(7)]
    (po, pc), (bto, btc), (vco, _), (bro, brc), (vo, _) = secs[:5]
    bone_table = list(struct.unpack_from(f"<{btc}H", d, bto * 2))
    n1, n2 = struct.unpack_from("<HH", d, vco * 2)
    refs = list(struct.unpack_from(f"<{brc}H", d, bro * 2))
    verts, p = [], vo * 2
    for _ in range(n1):
        verts.append(struct.unpack_from("<6f", d, p)); p += 24
    for _ in range(n2):
        verts.append(struct.unpack_from("<14f", d, p)); p += 56

    groups = []  # [{mat, tex, tris:[((i0,i1,i2),(u0,v0,u1,v1,u2,v2))]}]
    cur_mat, cur_tex, state_changed = None, None, True
    p, end = po * 2, (po + pc) * 2

    def group():
        nonlocal state_changed
        if state_changed or not groups:
            groups.append(dict(mat=cur_mat, tex=cur_tex, tris=[]))
            state_changed = False
        return groups[-1]["tris"]

    while p < end:
        op = struct.unpack_from("<H", d, p)[0]
        if op == 0x8010:
            cur_mat = bytes(d[p + 2:p + 46]); p += 46; state_changed = True
        elif op == 0x8000:
            cur_tex = bytes(d[p + 2:p + 18]); p += 18; state_changed = True
        elif op == 0x0054:
            n = struct.unpack_from("<H", d, p + 2)[0]; p += 4
            tris = group()
            for _ in range(n):
                tris.append((struct.unpack_from("<3H", d, p), struct.unpack_from("<6f", d, p + 6)))
                p += 30
        elif op == 0x5453:  # triangle strip
            n = struct.unpack_from("<H", d, p + 2)[0]; p += 4
            i0, i1, i2 = struct.unpack_from("<3H", d, p)
            uv = list(struct.unpack_from("<6f", d, p + 6)); p += 30
            tris = group()
            tris.append(((i0, i1, i2), tuple(uv)))
            a, b = (i1, uv[2:4]), (i2, uv[4:6])
            for k in range(n - 1):
                i = struct.unpack_from("<H", d, p)[0]
                u, v = struct.unpack_from("<2f", d, p + 2); p += 10
                c = (i, [u, v])
                # strips alternate winding: (v2,v1,v3), (v2,v3,v4), (v4,v3,v5), ...
                t = (b, a, c) if k % 2 == 0 else (a, b, c)
                tris.append(((t[0][0], t[1][0], t[2][0]), tuple(t[0][1]) + tuple(t[1][1]) + tuple(t[2][1])))
                a, b = b, c
        elif op == 0xFFFF:
            break
        else:
            raise ValueError(f"Unknown mesh opcode 0x{op:04X} at +0x{p:X}")
    return dict(head=head, flags=flags, bone_table=bone_table, n1=n1, n2=n2,
                refs=refs, verts=verts, groups=groups)


# --------------------------------------------------------------------------- mesh build

def build_mesh_chunk(name, head, bone_table, refs, verts1, verts2, groups):
    """verts1: list of 6-float tuples, verts2: list of 14-float tuples, refs: 2 per vertex
    (1-bone vertices first). groups: [{mat, tex, tris}] with tri indices into v1+v2."""
    label = name.decode("latin1") if isinstance(name, bytes) else name
    if len(bone_table) > MAX_BONES:
        raise ValueError(f"{label}: {len(bone_table)} bones used; one mesh can reference at most {MAX_BONES}")
    runs = [(g["mat"] is not None, g["tex"] is not None, len(g["tris"])) for g in groups]
    words = mesh_words(len(verts1), len(verts2), len(bone_table), runs)
    if words > MAX_MESH_WORDS:
        raise ValueError(f"{label}: mesh is {words:,} words, over the DAT limit of {MAX_MESH_WORDS:,} "
                         f"({words / MAX_MESH_WORDS:.0%}); remove geometry, or use fewer hard edges / flat-shaded faces (each one splits vertices)")
    poly = bytearray()
    for g in groups:
        if g["mat"] is not None:
            poly += struct.pack("<H", 0x8010) + g["mat"]
        if g["tex"] is not None:
            poly += struct.pack("<H", 0x8000) + g["tex"]
        tris = g["tris"]
        for s in range(0, len(tris), MAX_TRIS_PER_LIST):
            part = tris[s:s + MAX_TRIS_PER_LIST]
            poly += struct.pack("<HH", 0x0054, len(part))
            for idx, uv in part:
                poly += struct.pack("<3H6f", *idx, *uv)
    poly += struct.pack("<H", 0xFFFF)

    body = bytearray(0x40)
    def put(blob):
        off = len(body) // 2
        body.extend(blob)
        return off, len(blob) // 2

    secs = [put(poly)]
    secs.append(put(struct.pack(f"<{len(bone_table)}H", *bone_table)))
    secs.append(put(struct.pack("<HH", len(verts1), len(verts2))))
    secs.append(put(struct.pack(f"<{len(refs)}H", *refs)))
    vblob = b"".join(struct.pack("<6f", *v) for v in verts1) + b"".join(struct.pack("<14f", *v) for v in verts2)
    secs.append(put(vblob))
    end_words = len(body) // 2
    secs += [(end_words, 0), (0, end_words)]

    body[0:6] = head[0:6]
    for i, (o, c) in enumerate(secs):
        struct.pack_into("<IH", body, 6 + i * 6, o, c)
    body[0x30:0x40] = head[0x30:0x40]

    total = 16 + len(body)
    total += (-total) % 16
    body.extend(bytes(total - 16 - len(body)))
    assert end_words == words, (end_words, words)
    return chunk_header(name, CHUNK_MESH, total) + bytes(body)


# --------------------------------------------------------------------------- textures

def image_chunk_to_dds(raw):
    """Returns (texture_name, dds_bytes) for DXT images, else None."""
    d = raw[16:]
    if d[0] != 0xA1:
        return None
    tex_name = d[1:17]
    width, height = struct.unpack_from("<ii", d, 17 + 4)
    fourcc = d[57:61][::-1]
    if fourcc not in (b"DXT1", b"DXT3", b"DXT5"):
        return None
    size = struct.unpack_from("<I", d, 61)[0]
    pixels = d[69:69 + size]
    hdr = struct.pack("<4sIIIIIII44x", b"DDS ", 124, 0x81007, height, width, size, 0, 1)
    hdr += struct.pack("<II4s20x", 32, 0x4, fourcc)
    hdr += struct.pack("<I16x", 0x1000)
    return tex_name, hdr + pixels


# --------------------------------------------------------------------------- blender side

if bpy is not None:
    # FFXI (x, y, z) -> Blender (z, -x, -y). A pure rotation, so no handedness flip.
    TO_BL = Matrix(((0, 0, 1, 0), (-1, 0, 0, 0), (0, -1, 0, 0), (0, 0, 0, 1)))
    TO_XI = TO_BL.inverted()

    def bone_name(i):
        return f"B{i:03d}"

    def bone_index(name):
        return int(name[1:]) if len(name) == 4 and name[0] == "B" and name[1:].isdigit() else None

    def skeleton_matrices(bones):
        world = []
        for i, b in enumerate(bones):
            local = Matrix.Translation(b["t"]) @ Quaternion(b["q"]).to_matrix().to_4x4()
            world.append(local if i == 0 else world[b["parent"]] @ local)
        return world

    def mirror_table(world):
        """For each bone b: (mirror bone, axis code) so that
        W[mb] @ flip_axis(p) == reflectZ(W[b] @ p) for any bone-local p."""
        S = Matrix.Diagonal((1, 1, -1, 1))
        flips = {1: Matrix.Diagonal((-1, 1, 1, 1)), 2: Matrix.Diagonal((1, -1, 1, 1)),
                 3: Matrix.Diagonal((1, 1, -1, 1))}
        table = []
        for b, Wb in enumerate(world):
            target = S @ Wb
            best = None
            for mb, Wm in enumerate(world):
                for ax, F in flips.items():
                    M = Wm @ F
                    err = sum(abs(M[r][c] - target[r][c]) for r in range(3) for c in range(4))
                    if err < 1e-3 and (best is None or err < best[2] - 1e-6 or (abs(err - best[2]) < 1e-6 and mb == b)):
                        best = (mb, ax, err)
            table.append((best[0], best[1]) if best else (b, 3))
        return table

    def build_armature(bones, world, name):
        arm = bpy.data.armatures.new(name)
        obj = bpy.data.objects.new(name, arm)
        bpy.context.collection.objects.link(obj)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.mode_set(mode="EDIT")
        ebs = []
        for i, W in enumerate(world):
            eb = arm.edit_bones.new(bone_name(i))
            m = TO_BL @ W
            eb.head = m.to_translation()
            eb.tail = eb.head + m.to_3x3() @ Vector((0, 0.02, 0))
            eb.roll = 0
            ebs.append(eb)
        for i, b in enumerate(bones):
            if i:
                ebs[i].parent = ebs[b["parent"]]
        bpy.ops.object.mode_set(mode="OBJECT")
        obj["ffxi_skeleton"] = True
        obj.show_in_front = True
        return obj

    def write_textures(chunks, folder):
        os.makedirs(folder, exist_ok=True)
        out = {}
        for c in chunks:
            if c["type"] == CHUNK_IMAGE:
                res = image_chunk_to_dds(c["raw"])
                if res:
                    tex_name, dds = res
                    fname = tex_name.decode("latin1").split()[-1] + ".dds"
                    path = os.path.join(folder, fname)
                    with open(path, "wb") as f:
                        f.write(dds)
                    out[tex_name] = path
        return out

    def get_material(mat_bytes, tex_bytes, tex_paths):
        key = (mat_bytes or b"").hex() + "|" + (tex_bytes or b"").hex()
        for m in bpy.data.materials:
            if m.get("ffxi_key") == key:
                return m
        label = (tex_bytes or b"notex").decode("latin1").split()[-1]
        mat = bpy.data.materials.new(f"ffxi_{label}")
        mat["ffxi_key"] = key
        mat["ffxi_mat"] = (mat_bytes or b"").hex()
        mat["ffxi_tex"] = (tex_bytes or b"").hex()
        mat.use_nodes = True
        path = tex_paths.get(tex_bytes)
        if path:
            nt = mat.node_tree
            bsdf = next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED")
            tn = nt.nodes.new("ShaderNodeTexImage")
            tn.image = bpy.data.images.load(path, check_existing=True)
            nt.links.new(tn.outputs["Color"], bsdf.inputs["Base Color"])
            nt.links.new(tn.outputs["Alpha"], bsdf.inputs["Alpha"])
        return mat

    def import_dat(context, model_path, skel_path):
        model = open(model_path, "rb").read()
        bones = parse_skeleton(open(skel_path, "rb").read())
        world = skeleton_matrices(bones)
        chunks = read_chunks(model)
        base = os.path.splitext(os.path.basename(model_path))[0]
        tex_paths = write_textures(chunks, os.path.join(os.path.dirname(model_path), base + "_textures"))
        arm = build_armature(bones, world, base + "_skeleton")
        arm["ffxi_source"] = model_path
        arm["ffxi_skeleton_path"] = skel_path

        for chunk_index, c in enumerate(chunks):
            if c["type"] != CHUNK_MESH:
                continue
            m = parse_mesh(c["raw"])
            bt, refs = m["bone_table"], m["refs"]
            positions, normals, weights, nlens = [], [], [], []
            for i, v in enumerate(m["verts"]):
                r1, r2 = refs[2 * i], refs[2 * i + 1]
                if len(v) == 6:
                    W = world[bt[r1 & 0x7F]]
                    positions.append(TO_BL @ (W @ Vector(v[:3])))
                    normals.append(TO_BL.to_3x3() @ (W.to_3x3() @ Vector(v[3:6])))
                    weights.append([(bt[r1 & 0x7F], 1.0)])
                else:
                    W1, W2 = world[bt[r1 & 0x7F]], world[bt[r2 & 0x7F]]
                    w1, w2 = v[6], v[7]
                    p = (W1.to_3x3() @ Vector((v[0], v[2], v[4])) + W1.to_translation() * w1 +
                         W2.to_3x3() @ Vector((v[1], v[3], v[5])) + W2.to_translation() * w2)
                    n = W1.to_3x3() @ Vector((v[8], v[10], v[12])) + W2.to_3x3() @ Vector((v[9], v[11], v[13]))
                    positions.append(TO_BL @ p)
                    normals.append(TO_BL.to_3x3() @ n)
                    weights.append([(bt[r1 & 0x7F], w1), (bt[r2 & 0x7F], w2)])
                # some original normals are deliberately shorter than 1 (seam vertices);
                # remember the length so untouched vertices export unchanged
                nlens.append(normals[-1].length)
                normals[-1] = normals[-1].normalized()

            faces, face_uvs, face_mat, mats = [], [], [], []
            for g in m["groups"]:
                mat = get_material(g["mat"], g["tex"], tex_paths)
                if mat.name not in [x.name for x in mats]:
                    mats.append(mat)
                mi = [x.name for x in mats].index(mat.name)
                for (i0, i1, i2), uv in g["tris"]:
                    # DAT is clockwise; Blender wants counter-clockwise
                    faces.append((i0, i2, i1))
                    face_uvs.append(((uv[0], 1 - uv[1]), (uv[4], 1 - uv[5]), (uv[2], 1 - uv[3])))
                    face_mat.append(mi)

            name = c["name"].decode("latin1")
            me = bpy.data.meshes.new(name)
            me.from_pydata([tuple(p) for p in positions], [], faces)
            for mat in mats:
                me.materials.append(mat)
            uvl = me.uv_layers.new(name="UVMap")
            for poly, uvs, mi in zip(me.polygons, face_uvs, face_mat):
                poly.material_index = mi
                for li, uv in zip(poly.loop_indices, uvs):
                    uvl.data[li].uv = uv
            # keep the original DAT vertex order so unchanged vertices stay put on export
            attr = me.attributes.new("ffxi_index", "INT", "POINT")
            attr.data.foreach_set("value", list(range(len(positions))))
            attr = me.attributes.new("ffxi_normal_length", "FLOAT", "POINT")
            attr.data.foreach_set("value", nlens)
            me.shade_smooth()
            me.normals_split_custom_set_from_vertices([tuple(n) for n in normals])
            me.update()

            obj = bpy.data.objects.new(name, me)
            context.collection.objects.link(obj)
            obj.parent = arm
            groups = {}
            for vi, ws in enumerate(weights):
                for b, w in ws:
                    g = groups.get(b) or obj.vertex_groups.new(name=bone_name(b))
                    groups[b] = g
                    g.add([vi], w, "REPLACE")
            obj["ffxi_chunk"] = name
            obj["ffxi_chunk_index"] = chunk_index
            obj["ffxi_head"] = m["head"].hex()
            obj["ffxi_bone_table"] = list(bt)
            obj["ffxi_mirrored"] = bool(m["flags"] & 1)
            mod = obj.modifiers.new("Armature", "ARMATURE")
            mod.object = arm
            if m["flags"] & 1:
                # preview of the copy the game draws; not exported
                mir = obj.modifiers.new("FFXI mirror (preview only)", "MIRROR")
                mir.use_axis[0] = True
                mir.use_clip = False
                mir.use_mirror_merge = False
        return arm

    # ----------------------------------------------------------------------- export plan
    # plan_mesh() decides exactly what the exporter will write (vertex splits, order, bone
    # table, triangle groups). The viewport budget display uses the same plan, so the
    # numbers it shows are the numbers the DAT will get.

    _skel_cache, _src_cache = {}, {}

    def skeleton_data(path):
        key = (path, os.path.getmtime(path))
        if key not in _skel_cache:
            world = skeleton_matrices(parse_skeleton(open(path, "rb").read()))
            _skel_cache.clear()
            _skel_cache[key] = (world, [W.inverted() for W in world], mirror_table(world))
        return _skel_cache[key]

    def source_mesh(path, chunk_index):
        key = (path, os.path.getmtime(path))
        if key not in _src_cache:
            _src_cache.clear()
            _src_cache[key] = dict(chunks=read_chunks(open(path, "rb").read()), parsed={})
        entry = _src_cache[key]
        if chunk_index not in entry["parsed"]:
            entry["parsed"][chunk_index] = parse_mesh(entry["chunks"][chunk_index]["raw"])
        return entry["chunks"][chunk_index], entry["parsed"][chunk_index]

    def working_mesh(obj):
        """Triangulated temporary copy of the object's current geometry (Edit Mode aware)."""
        if obj.data.is_editmode:
            bm = bmesh.from_edit_mesh(obj.data).copy()
        else:
            bm = bmesh.new()
            bm.from_mesh(obj.data)
        bmesh.ops.triangulate(bm, faces=bm.faces[:])
        me = bpy.data.meshes.new("~ffxi_tmp")
        bm.to_mesh(me)
        bm.free()
        return me

    def plan_mesh(obj, arm):
        world, inv, mirror = skeleton_data(arm["ffxi_skeleton_path"])
        chunk, orig = source_mesh(arm["ffxi_source"], obj["ffxi_chunk_index"])
        name = obj["ffxi_chunk"]
        me = working_mesh(obj)
        try:
            # ---- per-vertex weights (top two bone groups), auto-weights for new geometry
            gid_to_bone = {g.index: bone_index(g.name) for g in obj.vertex_groups}
            raw_w = []
            for v in me.vertices:
                ws = sorted(((g.weight, gid_to_bone.get(g.group)) for g in v.groups
                             if gid_to_bone.get(g.group) is not None and g.weight > 1e-4), reverse=True)[:2]
                raw_w.append(ws)
            kd = KDTree(len(me.vertices))
            weighted = 0
            for i, v in enumerate(me.vertices):
                if raw_w[i]:
                    kd.insert(v.co, i); weighted += 1
            kd.balance()
            auto = 0
            for i, v in enumerate(me.vertices):
                if not raw_w[i]:
                    if not weighted:
                        raise ValueError(f"{name}: no vertex has bone weights")
                    _, j, _ = kd.find(v.co)
                    raw_w[i] = raw_w[j]; auto += 1
            across = 0
            if obj.get("ffxi_mirrored"):
                across = sum(1 for v in me.vertices if (obj.matrix_world @ v.co).x < -1e-4)

            ffxi_index = me.attributes.get("ffxi_index")
            orig_idx = [0] * len(me.vertices)
            if ffxi_index:
                ffxi_index.data.foreach_get("value", orig_idx)
            n_orig = len(orig["verts"])
            nlen_attr = me.attributes.get("ffxi_normal_length")
            nlens = [1.0] * len(me.vertices)
            if nlen_attr:
                nlen_attr.data.foreach_get("value", nlens)
            # new geometry has 0 here (or a copied value); only trust plausible lengths
            nlens = [x if 0.5 < x <= 1.0 else 1.0 for x in nlens]

            # ---- one DAT vertex per (Blender vertex, distinct corner normal); hard edges split.
            # Faces stay in Blender order (= original DAT draw order, new faces at the end),
            # which matters for alpha-blended parts such as hair.
            me_normals = [cn.vector.copy() for cn in me.corner_normals]
            uvl = me.uv_layers.active
            per_vertex = {}  # blender vertex -> [(normal, new index)]
            new_verts = []  # (sort_key, pos, nrm, weights, normal length)
            tri_runs = []  # [(material_index, [corners...])]
            for poly in me.polygons:
                corners = []
                for li in poly.loop_indices:
                    vi = me.loops[li].vertex_index
                    n = me_normals[li]
                    slots = per_vertex.setdefault(vi, [])
                    hit = next((ni for sn, ni in slots if sn.dot(n) > 0.9995), None)
                    if hit is None:
                        hit = len(new_verts)
                        slots.append((n, hit))
                        oi = orig_idx[vi] if ffxi_index and 0 <= orig_idx[vi] < n_orig else n_orig + vi
                        new_verts.append((oi, me.vertices[vi].co.copy(), n, raw_w[vi], nlens[vi]))
                    uv = uvl.data[li].uv if uvl else (0.0, 0.0)
                    corners.append((hit, (uv[0], 1 - uv[1])))
                if not tri_runs or tri_runs[-1][0] != poly.material_index:
                    tri_runs.append((poly.material_index, []))
                tri_runs[-1][1].append(corners)
            blender_verts = len(me.vertices)
        finally:
            bpy.data.meshes.remove(me)

        # ---- bone table: keep the original order, append new bones and their mirrors
        bone_table = list(obj.get("ffxi_bone_table", []))
        def slot(b):
            if b not in bone_table:
                bone_table.append(b)
            return bone_table.index(b)
        def ref(b):
            mb, ax = mirror[b]
            s, ms = slot(b), slot(mb)
            return s | (ms << 7) | (ax << 14)

        # ---- order: 1-bone first, then 2-bone; each keeps original DAT order where known
        one = sorted([i for i, v in enumerate(new_verts) if len(v[3]) == 1], key=lambda i: new_verts[i][0])
        two = sorted([i for i, v in enumerate(new_verts) if len(v[3]) == 2], key=lambda i: new_verts[i][0])
        remap = {old: new for new, old in enumerate(one + two)}
        refs = []
        for i in one:
            refs += [ref(new_verts[i][3][0][1]), 0]
        for i in two:
            (_, b1), (_, b2) = new_verts[i][3]
            refs += [ref(b1), ref(b2)]

        groups = []
        for mi, run in tri_runs:
            mat = obj.material_slots[mi].material if mi < len(obj.material_slots) else None
            mat_b = bytes.fromhex(mat["ffxi_mat"]) if mat and mat.get("ffxi_mat") else (orig["groups"][0]["mat"] if orig["groups"] else None)
            tex_b = bytes.fromhex(mat["ffxi_tex"]) if mat and mat.get("ffxi_tex") else (orig["groups"][0]["tex"] if orig["groups"] else None)
            tris = []
            for (a, ua), (b, ub), (c, uc) in run:
                # back to clockwise: (a, c, b)
                tris.append(((remap[a], remap[c], remap[b]), (*ua, *uc, *ub)))
            groups.append(dict(mat=mat_b or None, tex=tex_b or None, tris=tris))

        return dict(name=name, chunk=chunk, orig=orig, new_verts=new_verts, one=one, two=two,
                    refs=refs, bone_table=bone_table, groups=groups, auto=auto, across=across,
                    blender_verts=blender_verts, mirrored=bool(obj.get("ffxi_mirrored")))

    def plan_stats(plan):
        n1, n2 = len(plan["one"]), len(plan["two"])
        runs = [(g["mat"] is not None, g["tex"] is not None, len(g["tris"])) for g in plan["groups"]]
        tris = sum(r[2] for r in runs)
        words = mesh_words(n1, n2, len(plan["bone_table"]), runs)
        orig = plan["orig"]
        orig_tris = sum(len(g["tris"]) for g in orig["groups"])
        # what one more vertex costs, including its share of triangles at the current density
        verts = n1 + n2
        tri_ratio = tris / verts if verts else 2.0
        per_vert = ((VERTEX_WORDS_1 * n1 + VERTEX_WORDS_2 * n2) / verts if verts else VERTEX_WORDS_1) \
            + tri_ratio * TRIANGLE_WORDS
        room = max(0, MAX_MESH_WORDS - words)
        return dict(name=plan["name"], verts=verts, n1=n1, n2=n2, tris=tris, words=words,
                    frac=words / MAX_MESH_WORDS, bones=len(plan["bone_table"]),
                    bone_frac=len(plan["bone_table"]) / MAX_BONES,
                    orig_verts=len(orig["verts"]), orig_tris=orig_tris,
                    room_verts=int(room // per_vert), room_tris=int(room // per_vert * tri_ratio),
                    blender_verts=plan["blender_verts"], auto=plan["auto"], across=plan["across"],
                    mirrored=plan["mirrored"])

    # ----------------------------------------------------------------------- export

    def export_dat(context, out_path, arm=None, report=print):
        arm = arm or next((o for o in context.scene.objects if o.get("ffxi_source")), None)
        if arm is None:
            raise ValueError("No imported FFXI skeleton found in the scene")
        src = open(arm["ffxi_source"], "rb").read()
        # matched by position in the file, since names can repeat (e.g. LOD copies)
        objs = {o["ffxi_chunk_index"]: o for o in context.scene.objects
                if o.type == "MESH" and "ffxi_chunk_index" in o and o.parent == arm}

        out = bytearray()
        for chunk_index, c in enumerate(read_chunks(src)):
            if c["type"] == CHUNK_MESH and chunk_index in objs:
                out += mesh_chunk_from_object(objs[chunk_index], arm, report)
            else:
                out += c["raw"]
        with open(out_path, "wb") as f:
            f.write(out)
        return out_path

    def mesh_chunk_from_object(obj, arm, report):
        world, inv, mirror = skeleton_data(arm["ffxi_skeleton_path"])
        plan = plan_mesh(obj, arm)
        name = plan["name"]
        if plan["auto"]:
            report(f"{name}: {plan['auto']} unweighted vertices copied weights from the nearest weighted vertex")
        if plan["across"]:
            report(f"{name}: WARNING {plan['across']} vertices are on the -X side; this mesh is mirrored "
                   f"by the game across X=0, so they will be doubled up on the other side")

        to_world = TO_XI @ obj.matrix_world  # object space -> FFXI space
        nrm_mat = to_world.to_3x3().inverted().transposed()
        new_verts = plan["new_verts"]
        verts1, verts2 = [], []
        for i in plan["one"]:
            _, co, n, ws, nl = new_verts[i]
            b = ws[0][1]
            p = inv[b] @ (to_world @ co)
            nn = inv[b].to_3x3() @ (nrm_mat @ n).normalized() * nl
            verts1.append((p.x, p.y, p.z, nn.x, nn.y, nn.z))
        for i in plan["two"]:
            _, co, n, ws, nl = new_verts[i]
            (w1, b1), (w2, b2) = ws
            s = w1 + w2; w1, w2 = w1 / s, w2 / s
            pw = to_world @ co
            nw = (nrm_mat @ n).normalized() * nl
            p1 = (inv[b1] @ pw) * w1; p2 = (inv[b2] @ pw) * w2
            n1 = (inv[b1].to_3x3() @ nw) * w1; n2 = (inv[b2].to_3x3() @ nw) * w2
            verts2.append((p1.x, p2.x, p1.y, p2.y, p1.z, p2.z, w1, w2,
                           n1.x, n2.x, n1.y, n2.y, n1.z, n2.z))

        st = plan_stats(plan)
        report(f"{name}: {st['verts']} vertices (was {st['orig_verts']}), {st['tris']} triangles, "
               f"{st['bones']} bones, {st['frac']:.1%} of the mesh size limit")
        head = bytes.fromhex(obj["ffxi_head"])
        return build_mesh_chunk(plan["chunk"]["name"], head, plan["bone_table"], plan["refs"],
                                verts1, verts2, plan["groups"])

    # ----------------------------------------------------------------------- budget display

    import time
    import blf
    import gpu
    from gpu_extras.batch import batch_for_shader
    from bpy.app.handlers import persistent

    _stats = {}      # object name -> plan_stats() result, or {"error": text}
    _dirty = set()   # object names whose stats need recomputing
    _state = {"cost": 0.0, "draw": None}

    COL_OK, COL_WARN, COL_BAD = (0.45, 0.85, 0.45, 1), (0.95, 0.78, 0.30, 1), (0.98, 0.38, 0.32, 1)
    COL_TEXT, COL_DIM = (0.95, 0.95, 0.95, 1), (0.72, 0.72, 0.72, 1)

    def level_color(frac):
        return COL_OK if frac < 0.6 else COL_WARN if frac < 0.85 else COL_BAD

    def ffxi_objects(scene):
        return [o for o in scene.objects if o.type == "MESH" and "ffxi_chunk_index" in o
                and o.parent is not None and o.parent.get("ffxi_source")]

    def _schedule(delay=None):
        # Ask Blender whether the timer is pending rather than keeping our own flag: loading
        # a file (File > New, Open, ...) silently drops non-persistent timers, and a stale
        # flag left the budget stuck on "calculating..." for good.
        if not bpy.app.timers.is_registered(_recompute):
            # back off on heavy meshes so dragging vertices stays smooth
            bpy.app.timers.register(_recompute, first_interval=delay or max(0.2, _state["cost"] * 3),
                                    persistent=True)

    def _recompute():
        start = time.perf_counter()
        for name in list(_dirty):
            _dirty.discard(name)
            obj = bpy.data.objects.get(name)
            if obj is None:
                _stats.pop(name, None)
                continue
            try:
                _stats[name] = plan_stats(plan_mesh(obj, obj.parent))
            except Exception as e:  # shown in the UI instead of failing silently
                _stats[name] = {"name": obj.get("ffxi_chunk", name), "error": str(e)}
        _state["cost"] = time.perf_counter() - start
        for win in bpy.context.window_manager.windows:
            for area in win.screen.areas:
                if area.type in {"VIEW_3D", "PROPERTIES"}:
                    area.tag_redraw()
        return None

    def ensure_stats(objs):
        missing = [o.name for o in objs if o.name not in _stats]
        if missing:
            _dirty.update(missing)
            _schedule(0.05)

    @persistent
    def _on_depsgraph(scene, depsgraph):
        objs = {o.name: o for o in ffxi_objects(scene)}
        if not objs:
            return
        mesh_owner = {o.data.name: n for n, o in objs.items()}
        for u in depsgraph.updates:
            idd = getattr(u.id, "original", u.id)
            if isinstance(idd, bpy.types.Object) and idd.name in objs:
                _dirty.add(idd.name)
            elif isinstance(idd, bpy.types.Mesh) and idd.name in mesh_owner:
                _dirty.add(mesh_owner[idd.name])
        if _dirty:
            _schedule()

    @persistent
    def _on_reload(*_args):
        _stats.clear()
        _dirty.clear()

    def _rect(x, y, w, h, color):
        shader = gpu.shader.from_builtin("UNIFORM_COLOR")
        batch = batch_for_shader(shader, "TRIS", {"pos": [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]},
                                 indices=[(0, 1, 2), (0, 2, 3)])
        shader.bind()
        shader.uniform_float("color", color)
        batch.draw(shader)

    def overlay_rows(st, detailed):
        """Rows for one mesh: ("text", [(text, color), ...]) or ("bar", fraction, [(text, color), ...])."""
        if "error" in st:
            return [("text", [(f"FFXI {st['name']}: ", COL_TEXT), (st["error"], COL_BAD)])]
        pct = f"{st['frac']:.1%}"
        if not detailed:
            return [("bar", st["frac"], [(f"{st['name']}  ", COL_TEXT), (pct, level_color(st["frac"])),
                                        (f"  {st['verts']:,} verts · {st['tris']:,} tris", COL_DIM)])]
        dv, dt = st["verts"] - st["orig_verts"], st["tris"] - st["orig_tris"]
        title = [(f"FFXI  {st['name']}", COL_TEXT)]
        if st["mirrored"]:
            title.append(("   mirrored: counts the stored half only; the game draws it twice", COL_DIM))
        rows = [("text", title),
                ("bar", st["frac"], [("Mesh size  ", COL_TEXT), (pct, level_color(st["frac"])),
                                     (f" of the DAT limit  ({st['words']:,} / {MAX_MESH_WORDS:,} words)", COL_DIM)]),
                ("text", [(f"Vertices {st['verts']:,} ({dv:+,})    Triangles {st['tris']:,} ({dt:+,})    ", COL_TEXT),
                          (f"Bones {st['bones']} / {MAX_BONES}", level_color(st["bone_frac"]))]),
                ("text", [("Room left  ", COL_TEXT),
                          (f"≈ {st['room_verts']:,} more vertices (≈ {st['room_tris']:,} triangles) at this mesh's density",
                           level_color(st["frac"]))])
                if st["words"] <= MAX_MESH_WORDS else
                ("text", [(f"Over the limit by {st['words'] - MAX_MESH_WORDS:,} words; export will refuse this mesh", COL_BAD)])]
        extra = st["verts"] - st["blender_verts"]
        if extra > 0:
            rows.append(("text", [(f"{st['blender_verts']:,} Blender vertices become {st['verts']:,} in the DAT: "
                                   f"{extra:,} extra from hard edges / flat shading", COL_DIM)]))
        if st["across"]:
            rows.append(("text", [(f"{st['across']:,} vertices are past the mirror line (X < 0) and will be doubled", COL_BAD)]))
        if st["auto"]:
            rows.append(("text", [(f"{st['auto']:,} unweighted vertices will copy the nearest vertex's bone weights", COL_WARN)]))
        return rows

    def _draw_overlay():
        context = bpy.context
        scene = context.scene
        if not getattr(scene, "ffxi_show_budget", False):
            return
        objs = ffxi_objects(scene)
        if not objs:
            return
        ensure_stats(objs)
        active = context.active_object
        if active is not None and active.type == "ARMATURE":
            active = None
        objs.sort(key=lambda o: (o != active, o.name))
        rows = []
        for o in objs:
            st = _stats.get(o.name)
            if st is None:
                rows.append(("text", [(f"FFXI {o.get('ffxi_chunk', o.name)}: calculating…", COL_DIM)]))
            else:
                rows += overlay_rows(st, detailed=(o == active or len(objs) == 1))

        scale = context.preferences.system.ui_scale
        size, line, pad, bar_w = 11 * scale, 17 * scale, 8 * scale, 120 * scale
        x0 = 12 * scale
        for r in context.area.regions:
            if r.type == "TOOLS" and r.width > 1 and context.preferences.system.use_region_overlap:
                x0 += r.width
        font = 0
        blf.size(font, size)
        width = 0
        for row in rows:
            w = sum(blf.dimensions(font, t)[0] for t, _ in row[-1]) + (bar_w + 8 * scale if row[0] == "bar" else 0)
            width = max(width, w)
        height = line * len(rows)
        y0 = 12 * scale
        gpu.state.blend_set("ALPHA")
        _rect(x0 - pad, y0 - pad, width + 2 * pad, height + 2 * pad - (line - size), (0, 0, 0, 0.55))
        y = y0 + height - line
        for row in rows:
            x = x0
            if row[0] == "bar":
                frac = min(row[1], 1.0)
                _rect(x, y, bar_w, size * 0.8, (0.28, 0.28, 0.28, 1))
                _rect(x, y, max(bar_w * frac, 2 * scale), size * 0.8, level_color(row[1]))
                x += bar_w + 8 * scale
            for text, col in row[-1]:
                blf.color(font, *col)
                blf.position(font, x, y, 0)
                blf.draw(font, text)
                x += blf.dimensions(font, text)[0]
            y -= line
        gpu.state.blend_set("NONE")

    class VIEW3D_PT_ffxi_budget(bpy.types.Panel):
        """How much of the FFXI DAT's mesh size limit each mesh is using"""
        bl_space_type = "VIEW_3D"
        bl_region_type = "UI"
        bl_category = "FFXI"
        bl_label = "FFXI Mesh Budget"

        def draw(self, context):
            layout = self.layout
            layout.prop(context.scene, "ffxi_show_budget", text="Show in viewport")
            objs = ffxi_objects(context.scene)
            if not objs:
                layout.label(text="Import an FFXI DAT first (File > Import)")
                return
            ensure_stats(objs)
            for o in sorted(objs, key=lambda o: o.name):
                st = _stats.get(o.name)
                box = layout.box()
                if st is None:
                    box.label(text=f"{o.name}: calculating…")
                    continue
                if "error" in st:
                    box.label(text=f"{st['name']}: {st['error']}", icon="ERROR")
                    continue
                box.label(text=st["name"] + ("  (mirrored)" if st["mirrored"] else ""), icon="MESH_DATA")
                box.progress(factor=min(st["frac"], 1.0), type="BAR", text=f"{st['frac']:.1%} of DAT limit")
                col = box.column(align=True)
                col.label(text=f"Verts {st['verts']:,}  Tris {st['tris']:,}")
                col.label(text=f"Bones {st['bones']} / {MAX_BONES}")
                if st["words"] <= MAX_MESH_WORDS:
                    col.label(text=f"Room ≈ {st['room_verts']:,} verts")
                else:
                    col.label(text="Over limit: won't export", icon="ERROR")
                extra = st["verts"] - st["blender_verts"]
                if extra > 0:
                    col.label(text=f"+{extra:,} verts from hard edges")
                if st["mirrored"]:
                    col.label(text="Stored half only counts", icon="MOD_MIRROR")
                if st["across"]:
                    col.label(text=f"{st['across']} verts past X=0", icon="ERROR")
            col = layout.column(align=True)
            col.label(text="Limit is the DAT format's;", icon="INFO")
            col.label(text="the game's is untested.")

    # ----------------------------------------------------------------------- operators

    # Blender can't open a file browser from inside another one, so the skeleton is never
    # picked with a file-path button inside the import dialog. Instead it is remembered,
    # auto-detected next to the model, or asked for in a second dialog afterwards.

    class FFXIDatPreferences(bpy.types.AddonPreferences):
        bl_idname = __name__
        skeleton_path: StringProperty(
            name="Skeleton DAT", subtype="FILE_PATH",
            description="Race skeleton used for the last import; offered again next time")

        def draw(self, context):
            self.layout.prop(self, "skeleton_path")

    def _prefs():
        addon = bpy.context.preferences.addons.get(__name__)
        return addon.preferences if addon else None

    def remembered_skeleton():
        p = _prefs()
        path = (p.skeleton_path if p else "") or _state.get("skeleton", "")
        return path if path and os.path.isfile(bpy.path.abspath(path)) else ""

    def remember_skeleton(path):
        _state["skeleton"] = path
        p = _prefs()
        if p:
            p.skeleton_path = path

    def run_import(op, context, model_path, skel_path):
        bones = skeleton_bone_count(skel_path)
        if not bones:
            op.report({"ERROR"}, f"{os.path.basename(skel_path)} has no skeleton in it. "
                                 f"Pick the race skeleton DAT (93.DAT for Tarutaru).")
            return {"CANCELLED"}
        model = open(model_path, "rb").read()
        needed = highest_bone_used(model)
        if needed < 0:
            op.report({"WARNING"}, f"{os.path.basename(model_path)} has no meshes to edit "
                                   f"(is it a skeleton/animation DAT?)")
        elif needed >= bones:
            op.report({"ERROR"}, f"{os.path.basename(model_path)} uses bone {needed}, but "
                                 f"{os.path.basename(skel_path)} only has {bones} bones: wrong race skeleton?")
            return {"CANCELLED"}
        import_dat(context, model_path, skel_path)
        remember_skeleton(skel_path)
        op.report({"INFO"}, f"Imported {os.path.basename(model_path)} with skeleton {os.path.basename(skel_path)}")
        return {"FINISHED"}

    def ask_for_skeleton(model_path):
        """Open the skeleton file browser once the model file browser has closed."""
        def _open():
            wm = bpy.context.window_manager
            win = bpy.context.window or (wm.windows[0] if wm.windows else None)
            if win is None:
                return None
            with bpy.context.temp_override(window=win):
                bpy.ops.import_scene.ffxi_dat_skeleton(
                    "INVOKE_DEFAULT", model_path=model_path,
                    filepath=os.path.join(os.path.dirname(model_path), ""))
            return None
        bpy.app.timers.register(_open, first_interval=0.05)

    class IMPORT_OT_ffxi_dat(bpy.types.Operator, ImportHelper):
        """Import an FFXI model DAT together with its race skeleton DAT"""
        bl_idname = "import_scene.ffxi_dat"
        bl_label = "Import FFXI DAT"
        filter_glob: StringProperty(default="*.dat;*.DAT", options={"HIDDEN"})
        # plain text on purpose: a file-path button can't open a browser inside this one
        skeleton_path: StringProperty(
            name="Skeleton DAT",
            description="Race skeleton DAT (93.DAT for Tarutaru). Leave empty to auto-detect it "
                        "in the model's folder, or to be asked for it after this dialog")

        def invoke(self, context, event):
            if not self.skeleton_path:
                self.skeleton_path = remembered_skeleton()
            return ImportHelper.invoke(self, context, event)

        def draw(self, context):
            col = self.layout.column()
            col.label(text="Skeleton DAT (race skeleton):")
            row = col.row(align=True)
            row.prop(self, "skeleton_path", text="")
            if self.skeleton_path:
                row.operator(IMPORT_OT_ffxi_dat_clear_skeleton.bl_idname, text="", icon="X")
            box = col.box().column(align=True)
            if self.skeleton_path:
                box.label(text=os.path.basename(self.skeleton_path), icon="ARMATURE_DATA")
                box.label(text="Clear it to use a different race.")
            else:
                box.label(text="Empty: it's found automatically", icon="INFO")
                box.label(text="next to the model, or you'll")
                box.label(text="be asked for it after this.")

        def execute(self, context):
            model = self.filepath
            skel = bpy.path.abspath(self.skeleton_path.strip().strip('"'))
            if not skel:
                found = find_skeletons(os.path.dirname(model), exclude=model)
                if len(found) == 1:
                    skel = found[0]
                else:
                    ask_for_skeleton(model)
                    self.report({"INFO"}, "Now pick the race skeleton DAT for this model")
                    return {"FINISHED"}
            elif not os.path.isfile(skel):
                self.report({"ERROR"}, f"Skeleton DAT not found: {skel}")
                return {"CANCELLED"}
            return run_import(self, context, model, skel)

    class IMPORT_OT_ffxi_dat_clear_skeleton(bpy.types.Operator):
        """Forget the remembered skeleton so it's auto-detected or asked for"""
        bl_idname = "import_scene.ffxi_dat_clear_skeleton"
        bl_label = "Clear Skeleton"
        bl_options = {"INTERNAL"}

        def execute(self, context):
            op = getattr(context.space_data, "active_operator", None)
            if op is not None and hasattr(op, "skeleton_path"):
                op.skeleton_path = ""
            remember_skeleton("")
            return {"FINISHED"}

    class IMPORT_OT_ffxi_dat_skeleton(bpy.types.Operator, ImportHelper):
        """Pick the race skeleton DAT for the FFXI model being imported"""
        bl_idname = "import_scene.ffxi_dat_skeleton"
        bl_label = "Use as Skeleton"
        bl_options = {"INTERNAL"}
        filter_glob: StringProperty(default="*.dat;*.DAT", options={"HIDDEN"})
        model_path: StringProperty(options={"HIDDEN"})

        def draw(self, context):
            col = self.layout.column(align=True)
            col.label(text="Pick the race skeleton for", icon="ARMATURE_DATA")
            col.label(text=os.path.basename(self.model_path))
            col.separator()
            col.label(text="Tarutaru: 93.DAT")

        def execute(self, context):
            return run_import(self, context, self.model_path, self.filepath)

    class EXPORT_OT_ffxi_dat(bpy.types.Operator, ExportHelper):
        """Write a new FFXI DAT from the imported model (source DAT is left untouched)"""
        bl_idname = "export_scene.ffxi_dat"
        bl_label = "Export FFXI DAT"
        filename_ext = ".dat"
        filter_glob: StringProperty(default="*.dat;*.DAT", options={"HIDDEN"})

        def execute(self, context):
            arm = context.active_object
            if arm and arm.type == "MESH":
                arm = arm.parent
            if not (arm and arm.get("ffxi_source")):
                arm = None
            try:
                export_dat(context, self.filepath, arm, report=lambda m: self.report({"INFO"}, m))
            except ValueError as e:
                self.report({"ERROR"}, str(e))
                return {"CANCELLED"}
            return {"FINISHED"}

    def menu_import(self, context):
        self.layout.operator(IMPORT_OT_ffxi_dat.bl_idname, text="FFXI DAT (.dat)")

    def menu_export(self, context):
        self.layout.operator(EXPORT_OT_ffxi_dat.bl_idname, text="FFXI DAT (.dat)")

    classes = (FFXIDatPreferences, IMPORT_OT_ffxi_dat, IMPORT_OT_ffxi_dat_clear_skeleton,
               IMPORT_OT_ffxi_dat_skeleton, EXPORT_OT_ffxi_dat, VIEW3D_PT_ffxi_budget)

    def register():
        for c in classes:
            bpy.utils.register_class(c)
        bpy.types.TOPBAR_MT_file_import.append(menu_import)
        bpy.types.TOPBAR_MT_file_export.append(menu_export)
        bpy.types.Scene.ffxi_show_budget = BoolProperty(
            name="Show FFXI budget", default=True,
            description="Show how close each FFXI mesh is to the DAT size limit in the 3D viewport")
        bpy.app.handlers.depsgraph_update_post.append(_on_depsgraph)
        for h in ("load_post", "undo_post", "redo_post"):
            getattr(bpy.app.handlers, h).append(_on_reload)
        _state["draw"] = bpy.types.SpaceView3D.draw_handler_add(_draw_overlay, (), "WINDOW", "POST_PIXEL")

    def unregister():
        if bpy.app.timers.is_registered(_recompute):
            bpy.app.timers.unregister(_recompute)
        if _state["draw"] is not None:
            bpy.types.SpaceView3D.draw_handler_remove(_state["draw"], "WINDOW")
            _state["draw"] = None
        if _on_depsgraph in bpy.app.handlers.depsgraph_update_post:
            bpy.app.handlers.depsgraph_update_post.remove(_on_depsgraph)
        for h in ("load_post", "undo_post", "redo_post"):
            lst = getattr(bpy.app.handlers, h)
            if _on_reload in lst:
                lst.remove(_on_reload)
        del bpy.types.Scene.ffxi_show_budget
        bpy.types.TOPBAR_MT_file_import.remove(menu_import)
        bpy.types.TOPBAR_MT_file_export.remove(menu_export)
        for c in reversed(classes):
            bpy.utils.unregister_class(c)
