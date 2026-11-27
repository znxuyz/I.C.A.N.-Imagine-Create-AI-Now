"""
I.C.A.N. 桌上 AI 機器人 — Blender 精修版建模腳本（v0.1 轉頭版外觀重製）
適用：Blender 4.2 LTS 以上（已在 Blender 5.3 實測）
用法：Blender → Scripting 工作區 → 開啟本檔 → Run Script
      或命令列：blender --background --python ican_robot_blender.py -- --render out.png --stl out_dir

單位：場景設為公釐（1 Blender 單位 = 1 mm），以下所有數字都是 mm。

設計重點（對應需求）
  1. 所有外殼都用 Bevel 修改器（segments ≥ 3）＋ Shade Smooth by Angle，沒有銳利邊。
  2. 底座：上窄下寬的流線圓角台，頂部兩側自然收弧；正面喇叭孔＝Array 修改器排出的
     「沉頭小圓柱」做 Boolean 差集，外圍再挖一圈 0.6 mm 的淺凹槽當喇叭網區。
  3. 頭部：一體成型的圓角膠囊殼；正面用 Inset 做出邊框，再往內推 1.5 mm 做螢幕凹槽，
     玻璃面板是獨立零件，四周留 0.5 mm 結構縫。
  4. 關節：底座頂部是兩層交錯的圓柱轉台（左右轉），頸部上方是三節交錯的鉸鏈轉軸（上下點頭）。
  5. 分模線：底座與頭部中段各有一圈 1 mm 寬、1 mm 深的環狀內凹細線。
"""
import bpy, bmesh, math, sys, os
from mathutils import Vector, Matrix

# ───────────────────────── 可調參數 ─────────────────────────
BASE_W, BASE_D, BASE_H = 96.0, 74.0, 30.0     # 底座 寬、深、高
BASE_TOP_SCALE = 0.86                         # 頂面相對底面的縮小比例（流線收腰）
BASE_BEVEL = 9.0                              # 底座圓角
HEAD_W, HEAD_D, HEAD_H = 66.0, 44.0, 54.0     # 頭部 寬、深、高
HEAD_BEVEL = 11.0                             # 頭部圓角（越大越像膠囊）
SCREEN_FRAME = 3.0                            # 螢幕邊框寬
SCREEN_DEPTH = 1.5                            # 螢幕凹槽深度（需求 1–2 mm）
GLASS_GAP = 0.5                               # 玻璃與邊框的結構縫
PANEL_W, PANEL_DEPTH = 1.0, 1.0               # 分模線 寬、深
NECK_Z = BASE_H                               # 轉台起點
HINGE_Z = BASE_H + 24.0                       # 點頭轉軸高度
HEAD_Z = HINGE_Z + 6.0 + HEAD_H / 2           # 頭部中心高度
SEG = 64                                      # 圓柱圓周段數

C_SHELL = (0.851, 0.443, 0.310, 1)            # 陶土橘外殼（sRGB，與網站同色）
C_BASE = (0.93, 0.92, 0.89, 1)                # 暖白底座
C_METAL = (0.48, 0.49, 0.52, 1)               # 拉絲鈦
C_DARK = (0.03, 0.035, 0.045, 1)              # 深色邊框／縫


# ───────────────────────── 共用工具 ─────────────────────────
def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    s = bpy.context.scene
    s.unit_settings.system = 'METRIC'
    s.unit_settings.scale_length = 0.001
    s.unit_settings.length_unit = 'MILLIMETERS'


def activate(obj):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def add_bevel(obj, width, segments=4, angle=30, harden=False):
    """需求 1：用 bpy.ops 加 Bevel 修改器，segments 至少 3。"""
    activate(obj)
    bpy.ops.object.modifier_add(type='BEVEL')
    m = obj.modifiers[-1]
    m.width = width
    m.segments = max(3, segments)
    m.limit_method = 'ANGLE'
    m.angle_limit = math.radians(angle)
    m.profile = 0.5
    m.use_clamp_overlap = True
    m.harden_normals = harden
    return m


def smooth(obj, angle=35):
    """需求 1：Shade Smooth + 依角度自動平滑（4.1+ 用 shade_smooth_by_angle，舊版用 Auto Smooth）。"""
    activate(obj)
    bpy.ops.object.shade_smooth()
    if hasattr(bpy.ops.object, "shade_smooth_by_angle"):
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(angle), keep_sharp_edges=True)
    elif hasattr(obj.data, "use_auto_smooth"):
        obj.data.use_auto_smooth = True
        obj.data.auto_smooth_angle = math.radians(angle)


def apply_all(obj):
    activate(obj)
    for m in list(obj.modifiers):
        bpy.ops.object.modifier_apply(modifier=m.name)


def boolean(target, cutter, op='DIFFERENCE', apply=True, hide=True):
    activate(target)
    bpy.ops.object.modifier_add(type='BOOLEAN')
    m = target.modifiers[-1]
    m.operation = op
    m.object = cutter
    m.solver = 'EXACT'
    if apply:
        bpy.ops.object.modifier_apply(modifier=m.name)
    if hide:
        bpy.data.objects.remove(cutter, do_unlink=True)


def rounded_box(name, w, d, h, r, seg=6, loc=(0, 0, 0)):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = bpy.context.object
    o.name = name
    o.scale = (w, d, h)
    bpy.ops.object.transform_apply(scale=True)
    add_bevel(o, r, seg, angle=60)
    return o


def cylinder(name, r, h, loc=(0, 0, 0), rot=(0, 0, 0), verts=SEG):
    bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=r, depth=h, location=loc, rotation=rot)
    o = bpy.context.object
    o.name = name
    return o


def srgb(c):
    """把網頁色（sRGB）轉成 Blender 節點用的線性色，顏色才不會發白。"""
    return tuple((x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4) for x in c[:3]) + (1,)


def material(name, color, rough=0.5, metal=0.0, coat=0.0, emit=None, emit_strength=0.0, alpha=1.0, trans=0.0):
    m = bpy.data.materials.new(name)
    if not getattr(m, "node_tree", None): m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = srgb(color)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    for key in ("Coat Weight", "Clearcoat"):
        if key in b.inputs:
            b.inputs[key].default_value = coat
            break
    if emit is not None:
        for key in ("Emission Color", "Emission"):
            if key in b.inputs:
                b.inputs[key].default_value = srgb(emit)
                break
        b.inputs["Emission Strength"].default_value = emit_strength
    if trans:
        key = "Transmission Weight" if "Transmission Weight" in b.inputs else "Transmission"
        b.inputs[key].default_value = trans
    b.inputs["Alpha"].default_value = alpha
    return m


def assign(obj, mat):
    obj.data.materials.clear()
    obj.data.materials.append(mat)


def panel_line(shell, axis, at, size):
    """需求 5：1 mm 寬、1 mm 深的環狀分模線。
    做法：外殼複製一份往內縮 1 mm 當「核心」，再用一片 1 mm 厚的大板子減掉核心，
    剩下的薄環就是刀具，拿去對外殼做差集，形狀會完全貼合任何曲面。"""
    core = shell.copy(); core.data = shell.data.copy(); bpy.context.collection.objects.link(core)
    core.modifiers.clear()
    activate(core)
    bpy.ops.object.modifier_add(type='DISPLACE')
    d = core.modifiers[-1]; d.strength = -PANEL_DEPTH; d.mid_level = 0; d.direction = 'NORMAL'
    apply_all(core)
    dims = [size * 2] * 3
    dims["xyz".index(axis)] = PANEL_W
    loc = [0.0, 0.0, 0.0]
    loc["xyz".index(axis)] = at          # 世界座標（mm）
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    slab = bpy.context.object
    slab.scale = dims
    bpy.ops.object.transform_apply(scale=True)
    boolean(slab, core)                 # 板子 − 核心 = 1 mm 薄環
    boolean(shell, slab)                # 外殼 − 薄環 = 分模線


# ───────────────────────── 零件 ─────────────────────────
def build_base(mats):
    # 需求 2：流線圓角台。先做方塊，把頂面縮小（上窄下寬），再上大圓角。
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, BASE_H / 2))
    base = bpy.context.object; base.name = "Base"
    base.scale = (BASE_W, BASE_D, BASE_H)
    bpy.ops.object.transform_apply(scale=True)
    bm = bmesh.new(); bm.from_mesh(base.data)
    for v in bm.verts:
        if v.co.z > BASE_H / 2:                       # 頂面四點往內收
            v.co.x *= BASE_TOP_SCALE
            v.co.y *= BASE_TOP_SCALE
    bm.to_mesh(base.data); bm.free()
    # 再細分一次，讓頂部兩側的圓角變成連續漸變的弧度
    add_bevel(base, BASE_BEVEL, segments=8, angle=60)
    apply_all(base)
    activate(base)
    bpy.ops.object.modifier_add(type='SUBSURF')
    base.modifiers[-1].levels = 1; base.modifiers[-1].render_levels = 1
    apply_all(base)

    # 正面是斜的：算出斜角與網區中心所在的表面位置
    slope = math.atan((BASE_D * (1 - BASE_TOP_SCALE) / 2) / BASE_H)
    gz = BASE_H * 0.36
    front_y = -BASE_D / 2 + (BASE_D * (1 - BASE_TOP_SCALE) / 2) * (gz / BASE_H)
    pivot = (0, front_y, gz)

    # 喇叭網區：先挖一塊 0.6 mm 淺凹的圓角矩形（內凹層次 1）
    recess = rounded_box("GrilleRecess", 38, 4, 10, 3.0, seg=5)
    apply_all(recess)
    recess.data.transform(Matrix.Translation((0, -2 + 0.6, 0)))     # 背面落在表面內 0.6 mm
    recess.location = pivot; recess.rotation_euler.x = -slope
    boolean(base, recess)

    # 喇叭孔：小圓柱＋沉頭錐（內凹層次 2），用 Array 修改器排成 8 × 2 陣列，再做 Boolean 差集
    cols, rows, pitch = 8, 2, 4.0
    hole = cylinder("SpeakerHole", 0.9, 12, rot=(math.radians(90), 0, 0), verts=24)
    bpy.ops.mesh.primitive_cone_add(vertices=24, radius1=1.7, radius2=0.9, depth=1.2,
                                    location=(0, 0.6 + 0.4, 0), rotation=(math.radians(-90), 0, 0))
    sink = bpy.context.object
    activate(sink); hole.select_set(True); bpy.context.view_layer.objects.active = hole
    bpy.ops.object.join()
    hole = bpy.context.object
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    hole.data.transform(Matrix.Translation((-pitch * (cols - 1) / 2, 0, -pitch * (rows - 1) / 2)))
    for axis, count in ((0, cols), (2, rows)):
        activate(hole)
        bpy.ops.object.modifier_add(type='ARRAY')
        a = hole.modifiers[-1]
        a.count = count
        a.use_relative_offset = False
        a.use_constant_offset = True
        off = [0, 0, 0]; off[axis] = pitch
        a.constant_offset_displace = off
    apply_all(hole)
    hole.location = pivot; hole.rotation_euler.x = -slope
    boolean(base, hole)

    # 底座分模線（需求 5）：在高度 40% 處一圈
    panel_line(base, "z", BASE_H * 0.62, 200)                    # 離地約 19 mm，上蓋／下蓋分界

    # 細部小圓角：把布林切出的孔緣、凹槽邊也修圓（角度限制避免重複倒已經圓的面）
    add_bevel(base, 0.35, segments=3, angle=40)
    smooth(base)
    assign(base, mats["base"])

    # 底部一圈止滑腳墊（深色）
    foot = rounded_box("BaseFootRing", BASE_W - 6, BASE_D - 6, 2.0, 0.8, seg=4, loc=(0, 0, 0.6))
    smooth(foot); assign(foot, mats["dark"])
    return base


def build_joints(mats):
    """需求 4：轉台（兩層交錯圓柱）＋ 三節交錯鉸鏈（點頭軸）。"""
    parts = []
    z = NECK_Z
    # 左右轉台：大圓盤 + 縮進一圈深色縫 + 小圓盤，像相機雲台
    t1 = cylinder("YawRing_Lower", 17, 5, loc=(0, 0, z + 2.5)); add_bevel(t1, 1.2, 4, 40); parts.append((t1, "metal"))
    gap = cylinder("YawGap", 15.5, 1.0, loc=(0, 0, z + 5.5)); parts.append((gap, "dark"))
    t2 = cylinder("YawRing_Upper", 14, 4, loc=(0, 0, z + 8)); add_bevel(t2, 1.2, 4, 40); parts.append((t2, "metal"))
    # 頸柱：上細下粗的圓錐台
    bpy.ops.mesh.primitive_cone_add(vertices=SEG, radius1=8.5, radius2=6.5, depth=HINGE_Z - (z + 10), location=(0, 0, (z + 10 + HINGE_Z) / 2))
    neck = bpy.context.object; neck.name = "NeckPost"; add_bevel(neck, 1.0, 4, 40); parts.append((neck, "metal"))
    # 點頭鉸鏈：中間一節屬於頸部，兩側兩節屬於頭部，彼此留 0.6 mm 縫 → 交錯的機械轉軸
    rx = (0, math.radians(90), 0)
    k_mid = cylinder("Hinge_Center", 7.0, 14, loc=(0, 0, HINGE_Z), rot=rx); add_bevel(k_mid, 1.0, 4, 40); parts.append((k_mid, "metal"))
    for s in (-1, 1):
        k = cylinder(f"Hinge_Side_{'L' if s < 0 else 'R'}", 7.0, 7, loc=(s * (7 + 0.6 + 3.5), 0, HINGE_Z), rot=rx)
        add_bevel(k, 1.0, 4, 40); parts.append((k, "shell"))
        cap = cylinder(f"Hinge_Cap_{'L' if s < 0 else 'R'}", 3.2, 1.2, loc=(s * (7 + 0.6 + 7 + 0.4), 0, HINGE_Z), rot=rx)
        add_bevel(cap, 0.4, 3, 40); parts.append((cap, "metal"))
        # 側節往上接到頭部底面的支臂（同為外殼色，看起來和頭一體成型）
        arm = rounded_box(f"Hinge_Arm_{'L' if s < 0 else 'R'}", 7, 12, HEAD_Z - HEAD_H / 2 - HINGE_Z + 4,
                          2.5, seg=4, loc=(s * (7 + 0.6 + 3.5), 0, (HINGE_Z + HEAD_Z - HEAD_H / 2 + 4) / 2))
        parts.append((arm, "shell"))
    for o, key in parts:
        smooth(o)
        assign(o, mats[key])


def build_head(mats):
    # 需求 3：一體成型的圓角膠囊殼
    bpy.ops.mesh.primitive_cube_add(size=1, location=(0, 0, HEAD_Z))
    head = bpy.context.object; head.name = "HeadShell"
    head.scale = (HEAD_W, HEAD_D, HEAD_H)
    bpy.ops.object.transform_apply(scale=True)
    add_bevel(head, HEAD_BEVEL, segments=10, angle=60)
    apply_all(head)

    # 螢幕：找出正面（-Y）平面，Inset 出邊框，再往內推 SCREEN_DEPTH 做凹槽
    activate(head)
    bpy.ops.object.mode_set(mode='EDIT')
    bm = bmesh.from_edit_mesh(head.data)
    bm.faces.ensure_lookup_table()
    front = max((f for f in bm.faces if f.normal.y < -0.99), key=lambda f: f.calc_area())
    for f in bm.faces:
        f.select = False
    front.select = True
    bmesh.update_edit_mesh(head.data)
    bpy.ops.mesh.inset(thickness=SCREEN_FRAME, depth=0)            # 邊框
    bpy.ops.mesh.extrude_region_shrink_fatten(TRANSFORM_OT_shrink_fatten={"value": -SCREEN_DEPTH})   # 往內縮 1.5 mm
    bm = bmesh.from_edit_mesh(head.data)
    sel = [f for f in bm.faces if f.select][0]
    screen_w = max(v.co.x for v in sel.verts) - min(v.co.x for v in sel.verts)
    screen_h = max(v.co.z for v in sel.verts) - min(v.co.z for v in sel.verts)
    screen_y = sel.verts[0].co.y                  # 凹槽底面
    surface_y = screen_y - SCREEN_DEPTH            # 邊框表面（-Y 朝外）
    bpy.ops.object.mode_set(mode='OBJECT')

    # 分模線：頭殼前後蓋在螢幕後方 8 mm 處分開
    panel_line(head, "y", 8.0, 400)

    # 細部圓角：邊框內緣、凹槽底角也帶 0.6 mm 小圓角，邊緣不會刺眼
    add_bevel(head, 0.6, segments=3, angle=40)
    smooth(head)
    assign(head, mats["shell"])

    # 玻璃面板：獨立零件，四周留 GLASS_GAP 結構縫，表面比邊框低 0.3 mm
    gw, gh = screen_w - 2 * GLASS_GAP, screen_h - 2 * GLASS_GAP
    # 螢幕凹槽：表面在 screen_y，底在 screen_y + SCREEN_DEPTH（+Y 是往頭內）
    glass = rounded_box("ScreenGlass", gw, 1.0, gh, 0.45, seg=4, loc=(0, surface_y + 0.35 + 0.5, HEAD_Z))
    smooth(glass); assign(glass, mats["glass"])
    # 玻璃後方的深色底板：從四周 0.5 mm 縫看進去是黑的
    bed = rounded_box("ScreenBed", screen_w, 0.2, screen_h, 0.3, seg=3, loc=(0, screen_y - 0.1, HEAD_Z))
    assign(bed, mats["dark"])
    # LED 直條眼睛＋嘴巴（發光），印在玻璃表面
    ly = surface_y + 0.3
    for s in (-1, 1):
        eh = gh * 0.42
        eye = rounded_box(f"Eye_{'L' if s < 0 else 'R'}", eh * 0.36, 0.08, eh, eh * 0.17, seg=6, loc=(s * gw * 0.2, ly, HEAD_Z + gh * 0.08))
        smooth(eye); assign(eye, mats["led"])
    mouth = rounded_box("Mouth", gw * 0.1, 0.08, 1.3, 0.6, seg=3, loc=(0, ly, HEAD_Z - gh * 0.27))
    assign(mouth, mats["led"])

    # 頭頂星芒天線底座（呼應網站吉祥物）
    top = HEAD_Z + HEAD_H / 2
    ab = cylinder("AntennaBase", 4.5, 2.4, loc=(0, 0, top + 0.6)); add_bevel(ab, 0.6, 3, 40); smooth(ab); assign(ab, mats["metal"])
    stem = cylinder("AntennaStem", 0.9, 10, loc=(0, 0, top + 6.5), verts=24); smooth(stem); assign(stem, mats["metal"])
    star = sparkle("AntennaSparkle", 5.5, loc=(0, 0, top + 13.5)); assign(star, mats["led_warm"])
    return head


def sparkle(name, r, loc):
    """通用四角星芒（非任何品牌標誌），擠出並倒角。"""
    bm = bmesh.new()
    pts = []
    for i in range(8):
        a = i / 8 * math.tau + math.pi / 2
        rr = r if i % 2 == 0 else r * 0.24
        pts.append(bm.verts.new((math.cos(a) * rr, 0, math.sin(a) * rr)))
    face = bm.faces.new(pts)
    me = bpy.data.meshes.new(name); bm.to_mesh(me); bm.free()
    o = bpy.data.objects.new(name, me); bpy.context.collection.objects.link(o)
    o.location = loc
    activate(o)
    bpy.ops.object.modifier_add(type='SOLIDIFY'); o.modifiers[-1].thickness = 1.6; o.modifiers[-1].offset = 0
    add_bevel(o, 0.5, 3, 40)
    smooth(o)
    return o


# ───────────────────────── 場景、燈光、輸出 ─────────────────────────
def stage():
    s = bpy.context.scene
    w = bpy.data.worlds.new("World"); s.world = w
    if not getattr(w, "node_tree", None): w.use_nodes = True
    w.node_tree.nodes["Background"].inputs[0].default_value = srgb((0.42, 0.47, 0.46))
    w.node_tree.nodes["Background"].inputs[1].default_value = 0.9
    bpy.ops.mesh.primitive_plane_add(size=2000, location=(0, 0, 0))
    floor = bpy.context.object; floor.name = "Floor"
    assign(floor, material("Floor", (0.36, 0.41, 0.40, 1), rough=0.55))
    def light(name, kind, loc, energy, color, size):
        bpy.ops.object.light_add(type=kind, location=loc)
        l = bpy.context.object; l.name = name; l.data.energy = energy; l.data.color = color
        if kind == 'AREA': l.data.size = size
        l.rotation_euler = (Vector((0, 0, HEAD_Z * 0.6)) - Vector(loc)).to_track_quat('-Z', 'Y').to_euler()
    light("Key", 'AREA', (-180, -220, 260), 6.0e5, (1.0, 0.93, 0.85), 160)
    light("Fill", 'AREA', (220, -160, 120), 1.5e5, (0.9, 0.95, 1.0), 200)
    light("Rim", 'AREA', (60, 260, 220), 8.0e5, (0.6, 0.85, 1.0), 120)
    bpy.ops.object.camera_add(location=(150, -330, 150))
    cam = bpy.context.object; s.camera = cam
    cam.data.lens = 85; cam.data.clip_start = 1; cam.data.clip_end = 10000
    cam.rotation_euler = (Vector((0, 0, HEAD_Z * 0.62)) - cam.location).to_track_quat('-Z', 'Y').to_euler()
    s.render.engine = 'CYCLES'; s.cycles.samples = 64; s.cycles.device = 'CPU'
    s.render.resolution_x, s.render.resolution_y = 1200, 1200
    s.view_settings.view_transform = "AgX" if "AgX" in [i.identifier for i in s.view_settings.bl_rna.properties["view_transform"].enum_items] else "Filmic"
    s.view_settings.look = "None"


def main():
    reset_scene()
    mats = {
        "shell": material("Shell_MatteCeramic", C_SHELL, rough=0.62, coat=0.1),
        "base": material("Base_MatteWhite", C_BASE, rough=0.58, coat=0.1),
        "metal": material("Titanium_Brushed", C_METAL, rough=0.36, metal=1.0),
        "dark": material("Seam_Dark", C_DARK, rough=0.8),
        "glass": material("Screen_Obsidian", (0.01, 0.012, 0.016, 1), rough=0.05, metal=0.3, coat=1.0),
        "led": material("LED_Gold", (1, 0.8, 0.5, 1), emit=(1.0, 0.78, 0.45, 1), emit_strength=6),
        "led_warm": material("Sparkle_Glow", (1, 0.9, 0.75, 1), emit=(1.0, 0.85, 0.6, 1), emit_strength=8),
    }
    build_base(mats)
    build_joints(mats)
    build_head(mats)
    stage()

    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if "--stl" in argv:                                    # 每個零件各存一個 STL（mm）
        out = argv[argv.index("--stl") + 1]; os.makedirs(out, exist_ok=True)
        for o in bpy.data.objects:
            if o.type == 'MESH' and o.name != "Floor":
                activate(o)
                if hasattr(bpy.ops.wm, "stl_export"):
                    bpy.ops.wm.stl_export(filepath=os.path.join(out, o.name + ".stl"), export_selected_objects=True, global_scale=1.0, apply_modifiers=True)
    if "--render" in argv:
        bpy.context.scene.render.filepath = argv[argv.index("--render") + 1]
        bpy.ops.render.render(write_still=True)
    if "--blend" in argv:
        bpy.ops.wm.save_as_mainfile(filepath=argv[argv.index("--blend") + 1])


if __name__ == "__main__":
    main()
