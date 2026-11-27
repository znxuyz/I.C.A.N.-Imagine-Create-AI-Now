"""
I.C.A.N. 桌上 AI 機器人 — 圓球頭＋圓盤底座版（可 3D 列印、內部可裝硬體）
適用：Blender 4.2 LTS 以上（已在 Blender 5.3 實測）
用法：Blender → Scripting 工作區 → 開啟本檔 → Run Script
      命令列：blender --background --python ican_robot_sphere.py -- --render out.png --stl 資料夾 --blend 檔名.blend

單位：公釐（1 Blender 單位 = 1 mm）。

結構
  頭部  Ø70 圓球，殼厚 2.5 mm，拆成「前蓋（含螢幕窗）」與「後殼（含轉軸橋板）」兩件，方便裝螢幕。
        正面削出平台、內凹 1.5 mm 放玻璃片，中間開 Ø33 窗口給 1.28 吋圓形螢幕。
  底座  Ø130 × 30 扁圓盤，殼厚 2.6 mm，頂部中央用布林差集挖出半球形凹槽，頭部下陷 12.5 mm。
        底蓋可拆（放電路板），底蓋有散熱槽與喇叭孔；背面有電源線孔，頂部前方有麥克風孔。
  轉軸  凹槽中央立一根固定的圓柱軸塔，SG90 伺服馬達藏在塔裡，轉軸朝上插進頭部底部的橋板，
        從外面看不到任何關節，頭像是直接在凹槽裡轉。
  「硬體位置參考」集合：螢幕、主板、馬達、喇叭等零件的實際尺寸方塊，執行時會自動檢查有沒有撞到外殼。
"""
import bpy, bmesh, math, sys, os
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree

# ───────────────────────── 可調參數（mm）─────────────────────────
WALL = 2.6              # 底座殼厚
HEAD_WALL = 2.5         # 頭部殼厚
CLEAR = 1.5             # 頭部與凹槽的間隙
BASE_R, BASE_H = 65.0, 30.0       # 底座半徑、高度
BASE_BEVEL = 5.0                  # 底座外緣倒角
HEAD_R = 35.0                     # 頭部半徑（Ø70）
SINK = 12.5                       # 頭部陷入底座的深度
SOCKET_R = HEAD_R + CLEAR         # 凹槽球半徑
HEAD_C = Vector((0, 0, BASE_H - SINK - CLEAR + HEAD_R + 0.0))   # 頭部球心（與凹槽同心）
# 螢幕（1.28 吋 GC9A01 圓形模組：可視區 Ø32.4，模組 Ø37.5）
FACET_R = 22.0          # 正面平台半徑
SCREEN_RECESS = 1.5     # 玻璃凹槽深度
GLASS_R = 19.0          # 玻璃片半徑（凹槽 19.5，四周留 0.5 縫）
WINDOW_R = 16.6         # 螢幕窗口半徑
SPLIT_Y = -12.0         # 頭部前蓋／後殼分割面
SPLIT_GAP = 0.4
# 轉軸塔（SG90：本體 22.8 × 12.2 × 22.7）
TOWER_R = 15.0
TOWER_TOP = 30.0        # 塔頂＝馬達本體頂面高度
SERVO = (23.4, 12.8)    # 馬達槽（含 0.3 間隙）
NECK_OPEN_R = 17.5      # 頭部底部開口（讓軸塔伸進去）
BRIDGE_Z = 36.5         # 頭內橋板高度（舵盤鎖在這）
SEG = 96
DEBUG = '--debug' in sys.argv

C_SHELL = (0.851, 0.443, 0.310)     # 陶土橘
C_BASE = (0.93, 0.92, 0.89)         # 暖白
C_METAL = (0.48, 0.49, 0.52)
C_DARK = (0.03, 0.035, 0.045)


# ───────────────────────── 工具 ─────────────────────────
def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    u = bpy.context.scene.unit_settings
    u.system = 'METRIC'; u.scale_length = 0.001; u.length_unit = 'MILLIMETERS'


def activate(o):
    bpy.ops.object.select_all(action='DESELECT')
    o.select_set(True); bpy.context.view_layer.objects.active = o


def apply_all(o):
    activate(o)
    for m in list(o.modifiers):
        bpy.ops.object.modifier_apply(modifier=m.name)


def add_bevel(o, width, segments=4, angle=30):
    activate(o)
    bpy.ops.object.modifier_add(type='BEVEL')
    m = o.modifiers[-1]
    m.width = width; m.segments = max(3, segments)
    m.limit_method = 'ANGLE'; m.angle_limit = math.radians(angle)
    m.use_clamp_overlap = True
    return m


def check(o, label):
    """每一步檢查網格：必須是封閉、沒有破面的實體（3D 列印才不會出錯）。"""
    bm = bmesh.new(); bm.from_mesh(o.data)
    bad = sum(1 for e in bm.edges if not e.is_manifold)
    vol = bm.calc_volume(); bm.free()
    print(f"  [{label}] 非流形邊 {bad}，體積 {abs(vol) / 1000:.1f} cm³" + ("" if bad == 0 else "  ⚠"))


def clean(o):
    """清掉布林留下的零體積碎三角形與重複點。"""
    activate(o); bpy.ops.object.mode_set(mode='EDIT'); bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.remove_doubles(threshold=0.0005)
    bpy.ops.mesh.dissolve_degenerate(threshold=0.0005)
    bpy.ops.mesh.select_all(action='DESELECT'); bpy.ops.mesh.select_loose(); bpy.ops.mesh.delete(type='VERT')
    bpy.ops.object.mode_set(mode='OBJECT')


def smooth(o, angle=35):
    activate(o)
    bpy.ops.object.shade_smooth()
    if hasattr(bpy.ops.object, "shade_smooth_by_angle"):
        bpy.ops.object.shade_smooth_by_angle(angle=math.radians(angle), keep_sharp_edges=True)
    elif hasattr(o.data, "use_auto_smooth"):
        o.data.use_auto_smooth = True; o.data.auto_smooth_angle = math.radians(angle)


def boolean(target, cutter, op='DIFFERENCE', keep=False):
    activate(target)
    bpy.ops.object.modifier_add(type='BOOLEAN')
    m = target.modifiers[-1]; m.operation = op; m.object = cutter
    # Blender 4.5+ 有 Manifold 求解器：輸出保證封閉，最適合 3D 列印；舊版用 Exact
    m.solver = 'MANIFOLD' if 'MANIFOLD' in [i.identifier for i in m.bl_rna.properties['solver'].enum_items] else 'EXACT'
    bpy.ops.object.modifier_apply(modifier=m.name)
    if DEBUG: check(target, f"{target.name} {op} {cutter.name}")
    if not keep:
        bpy.data.objects.remove(cutter, do_unlink=True)


def cyl(name, r, h, loc=(0, 0, 0), rot=(0, 0, 0), verts=SEG):
    bpy.ops.mesh.primitive_cylinder_add(vertices=verts, radius=r, depth=h, location=loc, rotation=rot)
    o = bpy.context.object; o.name = name; return o


def box(name, x, y, z, loc=(0, 0, 0)):
    bpy.ops.mesh.primitive_cube_add(size=1, location=loc)
    o = bpy.context.object; o.name = name; o.scale = (x, y, z)
    bpy.ops.object.transform_apply(scale=True); return o


def sphere(name, r, loc=(0, 0, 0), seg=SEG, rings=48):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=seg, ring_count=rings, radius=r, location=loc)
    o = bpy.context.object; o.name = name; return o


def array(o, count, offset):
    activate(o)
    bpy.ops.object.modifier_add(type='ARRAY')
    a = o.modifiers[-1]
    a.count = count; a.use_relative_offset = False; a.use_constant_offset = True
    a.constant_offset_displace = offset


def srgb(c):
    return tuple((x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4) for x in c[:3]) + (1,)


def material(name, color, rough=0.5, metal=0.0, coat=0.0, emit=None, strength=0.0):
    m = bpy.data.materials.new(name)
    if not getattr(m, "node_tree", None): m.use_nodes = True
    b = m.node_tree.nodes.get("Principled BSDF")
    b.inputs["Base Color"].default_value = srgb(color)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    for k in ("Coat Weight", "Clearcoat"):
        if k in b.inputs: b.inputs[k].default_value = coat; break
    if emit:
        for k in ("Emission Color", "Emission"):
            if k in b.inputs: b.inputs[k].default_value = srgb(emit); break
        b.inputs["Emission Strength"].default_value = strength
    return m


def assign(o, mat):
    o.data.materials.clear(); o.data.materials.append(mat)


# ───────────────────────── 底座 ─────────────────────────
def build_base(M):
    # 外形：倒角圓盤 − 凹槽球
    # 只倒上緣：先做高一點的圓柱倒角，再切掉下半，底面保持平整（列印與裝底蓋都需要）
    base = cyl("Base_Shell", BASE_R, BASE_H + BASE_BEVEL, loc=(0, 0, (BASE_H - BASE_BEVEL) / 2))
    add_bevel(base, BASE_BEVEL, 6, 60); apply_all(base)                     # 圓盤外緣大倒角
    boolean(base, box("BottomTrim", 300, 300, 50, loc=(0, 0, -25)))
    add_bevel(base, 1.0, 3, 60); apply_all(base)                            # 底緣小倒角
    boolean(base, sphere("Socket_Cutter", SOCKET_R, loc=HEAD_C))            # 半球形凹槽
    check(base, "底座外形")
    # 空心化：再減掉一個「內縮 WALL 的內部實體」＝ 內圓柱 − 加厚的凹槽球
    #（直接對布林後的網格用 Solidify 容易破面，用這個方法殼厚一致又乾淨）
    inner = cyl("Inner", BASE_R - WALL, BASE_H - WALL + 5, loc=(0, 0, (BASE_H - WALL - 5) / 2))
    add_bevel(inner, max(0.6, BASE_BEVEL - WALL), 6, 60); apply_all(inner)
    boolean(inner, sphere("SocketWall", SOCKET_R + WALL, loc=HEAD_C))
    boolean(base, inner)                                                    # 底部同時開口，裝可拆底蓋
    check(base, "底座空心")

    # 隱藏式轉軸塔：從凹槽底部往下長到底蓋上方，往上伸進頭部
    tower = cyl("Tower", TOWER_R, TOWER_TOP - (WALL + 1.0), loc=(0, 0, (TOWER_TOP + WALL + 1.0) / 2))
    add_bevel(tower, 1.0, 4, 40); apply_all(tower)
    boolean(base, tower, op='UNION')
    pocket = box("ServoPocket", SERVO[0], SERVO[1], TOWER_TOP + 6, loc=(0, 0, TOWER_TOP / 2 + 1))
    boolean(base, pocket)                                                   # 馬達從下方塞進塔
    tab_top = TOWER_TOP - 6.8 + 2.5                                         # SG90 耳朵上緣（本體頂往下約 4.3 mm）
    tabs = box("ServoTabSlot", 33.0, SERVO[1], tab_top + 2, loc=(0, 0, tab_top / 2 - 1))
    boolean(base, tabs)                                                     # 耳朵寬的通道：馬達從下方推入，耳朵頂住上方擋肩，用螺絲往上鎖
    wire = box("WireSlot", 8, 6, 14, loc=(0, -(SERVO[1] / 2 + 2.5), TOWER_TOP - 5))
    boolean(base, wire)                                                     # 螢幕排線從塔內穿到頭部

    # 背面電源線孔（USB-C 線頭 12 × 6.5）
    usb = box("USB_Hole", 12.5, 20, 7.0, loc=(0, BASE_R - 3, 9.0))
    add_bevel(usb, 2.4, 4, 60); apply_all(usb)
    boolean(base, usb)
    # 背面散熱槽（5 條）
    vent = box("BackVent", 1.8, 20, 9, loc=(-16, BASE_R - 3, 18))
    add_bevel(vent, 0.8, 3, 60); apply_all(vent)
    array(vent, 5, (8, 0, 0)); apply_all(vent)
    boolean(base, vent)
    # 頂部前方麥克風孔
    mic = cyl("MicHole", 0.9, 10, loc=(0, -50, BASE_H), verts=24)
    boolean(base, mic)

    # 底蓋固定柱 ×3（M2 自攻螺絲，從底部鎖上）
    for i in range(3):
        a = math.radians(60 + 120 * i)
        p = (math.cos(a) * (BASE_R - WALL - 2.9), math.sin(a) * (BASE_R - WALL - 2.9))
        post = cyl(f"LidPost_{i+1}", 3.5, 10, loc=(p[0], p[1], WALL + 5), verts=32)
        boolean(base, post, op='UNION')
        boolean(base, cyl(f"LidPostHole_{i+1}", 0.8, 9, loc=(p[0], p[1], WALL + 4.4), verts=16))
    clean(base)
    check(base, "底座開孔")
    smooth(base); assign(base, M["base"])

    # 可拆底蓋：散熱槽＋喇叭孔，四個止滑腳
    lid = cyl("Base_Lid", BASE_R - WALL - 0.3, WALL, loc=(0, 0, WALL / 2))
    add_bevel(lid, 0.8, 3, 60); apply_all(lid)
    slots = box("LidVents", 26, 2.0, 10, loc=(0, 18, WALL / 2))
    add_bevel(slots, 0.9, 3, 60); apply_all(slots)
    array(slots, 6, (0, 4.5, 0)); apply_all(slots)
    boolean(lid, slots)
    grill = cyl("SpeakerHoles", 1.0, 10, loc=(-6, -52, WALL / 2), verts=20)
    array(grill, 5, (3, 0, 0)); array(grill, 5, (0, 3, 0)); apply_all(grill)
    boolean(lid, grill)
    for i in range(3):                                                      # 對應固定柱的沉頭螺絲孔
        a = math.radians(60 + 120 * i)
        p = (math.cos(a) * (BASE_R - WALL - 2.9), math.sin(a) * (BASE_R - WALL - 2.9))
        boolean(lid, cyl(f"LidScrew_{i+1}", 1.1, 10, loc=(p[0], p[1], 0), verts=16))
        bpy.ops.mesh.primitive_cone_add(vertices=24, radius1=2.2, radius2=1.1, depth=1.2, location=(p[0], p[1], 0.6))
        boolean(lid, bpy.context.object)
    clean(lid); check(lid, "底蓋")
    smooth(lid); assign(lid, M["base"])
    for a in range(4):
        ang = math.radians(45 + 90 * a)
        f = cyl(f"Foot_{a+1}", 5.0, 2.0, loc=(math.cos(ang) * 48, math.sin(ang) * 48, -1.0), verts=32)
        add_bevel(f, 0.6, 3, 60); smooth(f); assign(f, M["dark"])
    return base


# ───────────────────────── 頭部 ─────────────────────────
def build_head(M):
    head = sphere("Head", HEAD_R, loc=HEAD_C, seg=SEG, rings=64)
    fy = -math.sqrt(HEAD_R ** 2 - FACET_R ** 2)            # 正面平台的位置
    rx = (math.radians(90), 0, 0)
    facet = cyl("FacetCutter", FACET_R + 6, 20, loc=(0, HEAD_C.y + fy - 10, HEAD_C.z), rot=rx)
    boolean(head, facet)                                   # 削出圓形平台
    recess = cyl("RecessCutter", GLASS_R + 0.5, 20, loc=(0, HEAD_C.y + fy - 10 + SCREEN_RECESS, HEAD_C.z), rot=rx)
    boolean(head, recess)                                  # 內凹 1.5 mm 放玻璃
    add_bevel(head, 1.2, 4, 25); apply_all(head)           # 平台邊、凹槽邊倒圓，與球面流暢銜接
    check(head, "頭部外形")
    # 空心化：減掉內球（正面再切平，讓螢幕後方也保留 2.5 mm 壁厚）
    inner = sphere("HeadInner", HEAD_R - HEAD_WALL, loc=HEAD_C, seg=SEG, rings=64)
    front = box("InnerFront", 200, 200, 200, loc=(0, HEAD_C.y + fy + SCREEN_RECESS + HEAD_WALL - 100, HEAD_C.z))
    boolean(inner, front)
    boolean(head, inner)
    win = cyl("WindowCutter", WINDOW_R, 30, loc=(0, HEAD_C.y + fy, HEAD_C.z), rot=rx)
    boolean(head, win)                                     # 螢幕窗口
    neck = cyl("NeckOpening", NECK_OPEN_R, 30, loc=(0, 0, HEAD_C.z - HEAD_R))
    boolean(head, neck)                                    # 底部開口給轉軸塔
    check(head, "頭部空心")

    # 頭內橋板：舵盤鎖在中央凸台，橋板貼齊內壁
    bridge = cyl("Bridge", HEAD_R, 3.0, loc=(0, 0, BRIDGE_Z + 1.5))
    inner = sphere("InnerClip", HEAD_R - 0.5, loc=HEAD_C)
    boolean(bridge, inner, op='INTERSECT')
    front = box("BridgeFrontClip", 200, 200, 200, loc=(0, SPLIT_Y + 1.3 - 100, HEAD_C.z))
    boolean(bridge, front)                                 # 橋板只留在後殼，前方讓給螢幕與排線
    boss = cyl("HornBoss", 8.0, 4.0, loc=(0, 0, BRIDGE_Z - 1.0)); boolean(bridge, boss, op='UNION')
    horn = box("HornSlot", 20.0, 6.2, 2.2, loc=(0, 0, BRIDGE_Z - 2.0)); boolean(bridge, horn)   # 單臂舵盤嵌槽
    shaft = cyl("ShaftHole", 3.6, 10, loc=(0, 0, BRIDGE_Z)); boolean(bridge, shaft)
    boolean(head, bridge, op='UNION')

    # 拆成前蓋＋後殼（分割縫 0.4 mm，就是外觀上的分模線）
    back = head.copy(); back.data = head.data.copy(); bpy.context.collection.objects.link(back)
    head.name, back.name = "Head_Front", "Head_Back"
    cf = box("CutFront", 200, 200, 200, loc=(0, SPLIT_Y - 100 - SPLIT_GAP / 2, HEAD_C.z))
    boolean(back, cf)
    cb = box("CutBack", 200, 200, 200, loc=(0, SPLIT_Y + 100 + SPLIT_GAP / 2, HEAD_C.z))
    boolean(head, cb)
    # 前後蓋卡合唇邊（後殼內側長出 3 mm 唇，插進前蓋）
    lip = sphere("Lip", HEAD_R - HEAD_WALL - 0.15, loc=HEAD_C)
    lip_in = sphere("LipIn", HEAD_R - HEAD_WALL - 1.6, loc=HEAD_C)
    boolean(lip, lip_in)
    band = box("LipBand", 200, 3.6, 200, loc=(0, SPLIT_Y - 1.0, HEAD_C.z))      # 伸進前蓋 3 mm 的卡合唇
    boolean(lip, band, op='INTERSECT')
    # 唇邊根部：往外長進後殼內壁 0.6 mm，讓唇邊和後殼確實長在一起
    root = sphere("LipRoot", HEAD_R - HEAD_WALL + 0.6, loc=HEAD_C)
    boolean(root, sphere("LipRootIn", HEAD_R - HEAD_WALL - 1.6, loc=HEAD_C))
    boolean(root, box("LipRootBand", 200, 1.4, 200, loc=(0, SPLIT_Y + SPLIT_GAP / 2 + 0.5, HEAD_C.z)), op='INTERSECT')
    boolean(lip, root, op='UNION')
    nk = cyl("LipNeck", NECK_OPEN_R + 1.5, 30, loc=(0, 0, HEAD_C.z - HEAD_R)); boolean(lip, nk)
    boolean(back, lip, op='UNION')
    for o in (head, back):
        clean(o); check(o, o.name); smooth(o); assign(o, M["shell"])

    # 玻璃片＋表情（展示用，實際由螢幕顯示）
    gy = HEAD_C.y + fy + SCREEN_RECESS - 0.5
    glass = cyl("Screen_Glass", GLASS_R, 1.0, loc=(0, gy, HEAD_C.z), rot=rx)
    add_bevel(glass, 0.35, 3, 60); smooth(glass); assign(glass, M["glass"])
    for s in (-1, 1):
        e = box(f"Eye_{'L' if s < 0 else 'R'}", 3.6, 0.1, 9.0, loc=(s * 6.5, gy - 0.56, HEAD_C.z + 1.5))
        add_bevel(e, 1.7, 6, 60); smooth(e); assign(e, M["led"])
    mo = box("Mouth", 3.6, 0.1, 1.2, loc=(0, gy - 0.56, HEAD_C.z - 7.5)); add_bevel(mo, 0.5, 3, 60); assign(mo, M["led"])
    return head, back


# ───────────────────────── 硬體位置參考（顯示用，不輸出、不渲染）─────────────────────────
def hardware_proxies(M):
    col = bpy.data.collections.new("硬體位置參考"); bpy.context.scene.collection.children.link(col)
    items = []
    def put(o):
        for c in o.users_collection: c.objects.unlink(o)
        col.objects.link(o); assign(o, M["proxy"]); items.append(o)
    rx = (math.radians(90), 0, 0)
    fy = HEAD_C.y - math.sqrt(HEAD_R ** 2 - FACET_R ** 2) + SCREEN_RECESS + HEAD_WALL
    put(cyl("HW_圓形螢幕_GC9A01_1.28", 18.75, 4.0, loc=(0, fy + 2.3, HEAD_C.z), rot=rx))
    put(box("HW_SG90", 22.8, 12.2, 22.6, loc=(0, 0, TOWER_TOP - 0.05 - 22.6 / 2)))
    put(cyl("HW_SG90_轉軸", 2.4, 5.8, loc=(0, 0, TOWER_TOP + 3.05)))
    put(box("HW_ESP32-S3_DevKit", 63.0, 25.4, 10.0, loc=(0, 30, WALL + 6.0)))
    put(cyl("HW_喇叭_28mm", 14.0, 5.0, loc=(0, -46, WALL + 3.0)))
    put(box("HW_MAX98357A", 18.0, 18.0, 4.0, loc=(-34, -22, WALL + 3.0)))
    put(box("HW_INMP441", 14.0, 14.0, 3.5, loc=(0, -50, BASE_H - WALL - 2.2)))
    put(box("HW_USB-C母座", 9.0, 12.0, 4.0, loc=(0, BASE_R - WALL - 7.2, 9.0)))
    for o in items:
        o.display_type = 'WIRE'; o.hide_render = True
    return items


def fit_check(shells, proxies):
    """檢查每個硬體方塊有沒有跟外殼互相穿透。"""
    dg = bpy.context.evaluated_depsgraph_get()
    def bvh(o):
        bm = bmesh.new(); bm.from_object(o, dg); bm.transform(o.matrix_world)
        t = BVHTree.FromBMesh(bm); bm.free(); return t
    trees = {s.name: bvh(s) for s in shells}
    bad = []
    for p in proxies:
        tp = bvh(p)
        for n, t in trees.items():
            if tp.overlap(t): bad.append((p.name, n))
    print("── 硬體容納檢查 ──")
    print("全部放得下，沒有撞到外殼" if not bad else "\n".join(f"⚠ {a} 碰到 {b}" for a, b in bad))
    return bad


# ───────────────────────── 場景 ─────────────────────────
def stage():
    s = bpy.context.scene
    w = bpy.data.worlds.new("World"); s.world = w
    if not getattr(w, "node_tree", None): w.use_nodes = True
    w.node_tree.nodes["Background"].inputs[0].default_value = srgb((0.42, 0.47, 0.46))
    w.node_tree.nodes["Background"].inputs[1].default_value = 0.9
    bpy.ops.mesh.primitive_plane_add(size=3000, location=(0, 0, -2.0))
    fl = bpy.context.object; fl.name = "Floor"; assign(fl, material("Floor", (0.36, 0.41, 0.40), rough=0.55))
    tgt = Vector((0, 0, 45))
    def light(n, loc, e, c, size):
        bpy.ops.object.light_add(type='AREA', location=loc)
        l = bpy.context.object; l.name = n; l.data.energy = e; l.data.color = c; l.data.size = size
        l.rotation_euler = (tgt - Vector(loc)).to_track_quat('-Z', 'Y').to_euler()
    light("Key", (-200, -240, 280), 7e5, (1.0, 0.93, 0.85), 180)
    light("Fill", (240, -180, 140), 1.8e5, (0.9, 0.95, 1.0), 220)
    light("Rim", (70, 280, 240), 9e5, (0.6, 0.85, 1.0), 140)
    bpy.ops.object.camera_add(location=(190, -380, 190))
    cam = bpy.context.object; s.camera = cam; cam.data.lens = 85; cam.data.clip_start = 1; cam.data.clip_end = 10000
    cam.rotation_euler = (Vector((0, 0, 42)) - cam.location).to_track_quat('-Z', 'Y').to_euler()
    s.render.engine = 'CYCLES'; s.cycles.samples = 64; s.cycles.device = 'CPU'
    s.render.resolution_x = s.render.resolution_y = 1200
    vt = [i.identifier for i in s.view_settings.bl_rna.properties["view_transform"].enum_items]
    s.view_settings.view_transform = "AgX" if "AgX" in vt else "Filmic"


def main():
    reset_scene()
    M = {
        "shell": material("Shell_MatteCeramic", C_SHELL, rough=0.62, coat=0.1),
        "base": material("Base_MatteWhite", C_BASE, rough=0.58, coat=0.1),
        "dark": material("Rubber_Dark", C_DARK, rough=0.85),
        "glass": material("Screen_Obsidian", (0.01, 0.012, 0.016), rough=0.05, metal=0.3, coat=1.0),
        "led": material("LED_Gold", (1, 0.8, 0.5), emit=(1.0, 0.78, 0.45), strength=6),
        "proxy": material("Hardware_Proxy", (0.2, 0.7, 1.0), rough=0.5),
    }
    base = build_base(M)
    hf, hb = build_head(M)
    proxies = hardware_proxies(M)
    fit_check([base, hf, hb, bpy.data.objects["Base_Lid"]], proxies)
    stage()

    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if "--stl" in argv:              # 只輸出要列印的 5 件
        out = argv[argv.index("--stl") + 1]; os.makedirs(out, exist_ok=True)
        for n in ("Base_Shell", "Base_Lid", "Head_Front", "Head_Back", "Screen_Glass"):
            activate(bpy.data.objects[n])
            bpy.ops.wm.stl_export(filepath=os.path.join(out, n + ".stl"), export_selected_objects=True, apply_modifiers=True)
    if "--render" in argv:
        bpy.context.scene.render.filepath = argv[argv.index("--render") + 1]
        bpy.ops.render.render(write_still=True)
    if "--blend" in argv:
        bpy.ops.wm.save_as_mainfile(filepath=argv[argv.index("--blend") + 1])


if __name__ == "__main__":
    main()
