"""
桌上型 AI 機器人 — 參數化模型
兩種方案共用「底座 + U 型轉頭支架 + 頭部」結構：
  pan  伺服（底座內，垂直）→ 左右轉頭
  tilt 伺服（支架右臂，水平）→ 上下點頭
單位 mm。改上方參數後重跑：python3 build.py
"""
import math, sys, os
import numpy as np
import manifold3d as mf
import trimesh

SEG = 64
mf.set_circular_segments(SEG) if hasattr(mf, "set_circular_segments") else None
M = mf.Manifold

# ---------- 共用參數 ----------
WALL = 2.4          # 一般壁厚
CLR = 0.3           # 配合間隙
# SG90 伺服馬達（實際尺寸各廠略有差異，買到後量一下）
SG_L, SG_W = 22.8, 12.2      # 本體長、寬
SG_TAB_HOLE = 27.8           # 兩個耳朵螺絲孔距
SG_SHAFT_OFF = 5.5           # 轉軸距本體中心的偏移
SG_TAB_TO_HORN = 11.1        # 耳朵底面 → 舵盤底面的距離
HORN_HUB_D = 7.8             # 舵盤中心圓直徑
HORN_ARM_W, HORN_ARM_L = 6.0, 17.0   # 單臂舵盤寬、長（由中心算）
HORN_DEPTH = 2.0             # 舵盤嵌入深度


def box(x, y, z, cx=0, cy=0, cz=0, center=True):
    b = M.cube([x, y, z], center=True)
    return b.translate([cx, cy, cz])


def cyl(h, d, cx=0, cy=0, cz=0, axis="z"):
    c = M.cylinder(h, d / 2, d / 2, SEG, True)
    if axis == "x":
        c = c.rotate([0, 90, 0])
    elif axis == "y":
        c = c.rotate([90, 0, 0])
    return c.translate([cx, cy, cz])


def rbox(x, y, z, r, cx=0, cy=0, cz=0):
    """xy 平面圓角方塊"""
    r = min(r, x / 2 - 0.01, y / 2 - 0.01)
    core = box(x - 2 * r, y, z) + box(x, y - 2 * r, z)
    for sx in (-1, 1):
        for sy in (-1, 1):
            core = core + cyl(z, 2 * r, sx * (x / 2 - r), sy * (y / 2 - r))
    return core.translate([cx, cy, cz])


def horn_recess(depth=HORN_DEPTH):
    """單臂舵盤嵌槽（沿 +x 方向的臂），z 方向向下挖；回傳要被減掉的實體，頂面在 z=0"""
    hub = cyl(depth + 2.2, HORN_HUB_D + 0.4, cz=-(depth + 2.2) / 2)   # 中心較深
    arm = box(HORN_ARM_L, HORN_ARM_W + 0.4, depth, cx=HORN_ARM_L / 2, cz=-depth / 2)
    screw_access = cyl(12, 5.5, cz=-6)                                  # 螺絲起子孔
    pilots = cyl(8, 1.6, cx=9, cz=-4) + cyl(8, 1.6, cx=13.5, cz=-4)     # 固定舵盤的小螺絲
    return hub + arm + screw_access + pilots


def servo_cut_vertical(cx, cy, z_top, plate_t):
    """在水平板上挖垂直安裝 SG90 的孔（轉軸在 cx+SG_SHAFT_OFF 處朝上）"""
    body = box(SG_L + 0.4, SG_W + 0.4, plate_t + 2, cx=cx, cy=cy, cz=z_top - plate_t / 2)
    holes = cyl(plate_t + 2, 1.8, cx - SG_TAB_HOLE / 2, cy, z_top - plate_t / 2) + \
        cyl(plate_t + 2, 1.8, cx + SG_TAB_HOLE / 2, cy, z_top - plate_t / 2)
    return body + holes


# ---------- 方案設定 ----------
PLANS = {
    # 高 CP：M5Stack CoreS3 SE（54×54×15.5，自帶螢幕、雙麥克風、喇叭）
    "hicp": dict(
        board_w=54.0, board_h=54.0, board_t=15.5,   # 板子寬、高、厚
        window_w=50.0, window_h=50.0, window_dz=0.0,
        base=dict(w=70, d=60, h=34),
        side_ports=True,          # 頭部兩側下方開口（USB-C / Grove）
        mic_hole=False,
    ),
    # 低價：ESP32-S3-DevKitC-1 + 1.54" ST7789 螢幕 + INMP441 麥克風 + MAX98357A 功放 + 喇叭
    "budget": dict(
        board_w=32.0, board_h=40.0, board_t=11.0,   # 1.54 吋螢幕模組 PCB（含排針厚度）
        window_w=28.5, window_h=28.5, window_dz=4.0,
        base=dict(w=92, d=70, h=52),
        side_ports=False,
        mic_hole=True,
        devkit=(70.0, 28.6),      # 開發板放置槽（相容 63~69mm 長的版本）
        speaker_d=40.0,           # 喇叭直徑
    ),
}


def make_parts(p):
    parts = {}
    # ===== 頭部（以點頭軸為原點；-y 是臉的方向；頂部開口、板子從上方插入）=====
    iw, ih, it = p["board_w"] + 2 * CLR, p["board_h"] + 2 * CLR, p["board_t"] + 2 * CLR
    ow, od = iw + 2 * WALL, it + 2 * WALL
    z_bot = -ih / 2 - WALL
    z_top = ih / 2
    oh = z_top - z_bot
    head = rbox(ow, od, oh, 3, cz=(z_top + z_bot) / 2)
    head = head - box(iw, it, ih + 10, cz=5)                         # 內腔（往上貫穿）
    head = head - box(p["window_w"], WALL * 3, p["window_h"], cy=-od / 2, cz=p["window_dz"])   # 螢幕窗
    head = head - box(iw - 8, WALL * 3, 12, cy=od / 2, cz=z_bot + WALL + 6)                     # 背面走線槽
    if p["side_ports"]:
        for sx in (-1, 1):
            head = head - box(WALL * 3, it, 16, cx=sx * ow / 2, cz=z_bot + WALL + 10)
    # 右側（+x）舵盤嵌槽，臂朝下
    hr = horn_recess().rotate([0, 90, 0]).translate([ow / 2, 0, 0])
    head = head - hr
    # 左側（-x）樞軸凸台 + M3 螺絲孔
    gap = SG_TAB_TO_HORN - 3.0          # 支架臂內側 → 頭側壁 的間隙
    boss_len = gap - 0.6
    head = head + cyl(boss_len, 9, cx=-ow / 2 - boss_len / 2, axis="x")
    head = head - cyl(ow + 2 * boss_len + 4, 2.7, axis="x", cx=-ow / 2)
    parts["head"] = head
    # 頂蓋（卡入內腔）
    cap = rbox(ow, od, WALL, 3, cz=WALL / 2) + box(iw - 0.4, it - 0.4, 3, cz=-1.5)
    cap = cap - box(iw - 4, it - 4, 3.2, cz=-1.5)
    parts["head_cap"] = cap

    # ===== U 型支架（原點：pan 舵盤中心，底面 z=0）=====
    xo = ow / 2 + SG_TAB_TO_HORN          # 臂外側面 x
    rc = math.hypot(od / 2, max(-z_bot, z_top))
    za = max(rc + 8.0, 11.6 + 5 + SG_SHAFT_OFF + 2)   # 點頭軸高度
    bar_t, arm_t, yw = 6.0, 3.0, 24.0
    arm_top = za + SG_L / 2 - SG_SHAFT_OFF + 5
    yoke = box(2 * xo, yw, bar_t, cz=bar_t / 2)
    for sx in (-1, 1):
        yoke = yoke + box(arm_t, yw, arm_top, cx=sx * (xo - arm_t / 2), cz=arm_top / 2)
    # 臂的圓頂
    for sx in (-1, 1):
        yoke = yoke + cyl(arm_t, yw, cx=sx * (xo - arm_t / 2), cz=arm_top, axis="x")
    # 右臂：tilt 伺服安裝孔（本體長邊沿 z，轉軸在上方對準 za）
    bc = za - SG_SHAFT_OFF
    yoke = yoke - box(arm_t + 2, SG_W + 0.4, SG_L + 0.4, cx=xo - arm_t / 2, cz=bc)
    for dz in (-SG_TAB_HOLE / 2, SG_TAB_HOLE / 2):
        yoke = yoke - cyl(arm_t + 2, 1.8, cx=xo - arm_t / 2, cz=bc + dz, axis="x")
    # 左臂：M3 樞軸孔
    yoke = yoke - cyl(arm_t + 2, 3.4, cx=-(xo - arm_t / 2), cz=za, axis="x")
    # 底面 pan 舵盤嵌槽
    yoke = yoke - horn_recess().rotate([180, 0, 0])
    # 走線孔
    yoke = yoke - box(10, 8, bar_t + 2, cx=-xo / 2, cz=bar_t / 2)
    parts["yoke"] = yoke

    # ===== 底座（原點：中心，底面 z=0；倒過來印）=====
    b = p["base"]
    bw, bd, bh = b["w"], b["d"], b["h"]
    top_t = 3.0
    base = rbox(bw, bd, bh, 6, cz=bh / 2)
    base = base - rbox(bw - 2 * WALL, bd - 2 * WALL, bh, 4, cz=bh / 2 - top_t)
    # pan 伺服：轉軸在 (0,0)
    base = base - servo_cut_vertical(-SG_SHAFT_OFF, 0, bh, top_t)
    # 走線孔（頂板後方）
    base = base - cyl(top_t + 2, 10, 0, bd / 2 - WALL - 9, bh - top_t / 2)
    # 背面 USB 線孔
    base = base - box(14, WALL * 3, 9, cy=bd / 2, cz=8)
    if "devkit" in p:
        # 正面喇叭網孔
        sd = p["speaker_d"]
        sz = 26.0
        for r in (0, 6, 12, 16):
            n = 1 if r == 0 else int(2 * math.pi * r / 5.5)
            for k in range(n):
                a = 2 * math.pi * k / n
                base = base - cyl(WALL * 3, 2.6, bw / 4 + r * math.cos(a), -bd / 2, sz + r * math.sin(a), axis="y")
        # 麥克風孔（INMP441 貼在內側）
        base = base - cyl(WALL * 3, 3.0, -bw / 4, -bd / 2, sz, axis="y")
        # 喇叭固定環
        ring = cyl(4, sd + 4.4, bw / 4, -bd / 2 + WALL + 2, sz, axis="y") - \
            cyl(6, sd + 0.6, bw / 4, -bd / 2 + WALL + 2, sz, axis="y")
        base = base + ring
        # 麥克風定位框（14mm 圓板）
        mring = cyl(3, 19, -bw / 4, -bd / 2 + WALL + 1.5, sz, axis="y") - cyl(5, 15, -bw / 4, -bd / 2 + WALL + 1.5, sz, axis="y")
        base = base + mring
        # 側面 USB-C 開口（開發板燒錄/供電）
        base = base - box(WALL * 3, 13, 8, cx=-bw / 2, cy=bd / 2 - WALL - p["devkit"][1] / 2 - 2, cz=7.5)
    parts["base"] = base

    # ===== 底板（塞入底座）=====
    lip = 3.0
    plate = rbox(bw, bd, 2.4, 6, cz=1.2)
    plug = rbox(bw - 2 * WALL - 2 * 0.25, bd - 2 * WALL - 2 * 0.25, lip, 4, cz=2.4 + lip / 2)
    plug = plug - rbox(bw - 2 * WALL - 4.5, bd - 2 * WALL - 4.5, lip + 1, 3, cz=2.4 + lip / 2)
    plate = plate + plug
    if "devkit" in p:
        kl, kw = p["devkit"]
        cy_k = bd / 2 - WALL - kw / 2 - 2
        rail = box(kl + 4, kw + 4, 4, cy=cy_k, cz=2.4 + 2) - box(kl, kw, 6, cy=cy_k, cz=2.4 + 2)
        rail = rail - box(14, kw + 6, 6, cx=-kl / 2, cy=cy_k, cz=2.4 + 2)   # USB 端開口
        plate = plate + rail
    else:
        # 配重槽：可放 10 元硬幣增加穩定度
        for sx in (-1, 1):
            plate = plate + (cyl(4, 31, sx * 17, -8, 2.4 + 2) - cyl(5, 27.4, sx * 17, -8, 2.4 + 2))
    parts["bottom"] = plate

    geo = dict(xo=xo, za=za, bh=bh, ow=ow, od=od, z_bot=z_bot, z_top=z_top)
    return parts, geo


def to_trimesh(m):
    mesh = m.to_mesh()
    v = np.array(mesh.vert_properties)[:, :3]
    f = np.array(mesh.tri_verts)
    return trimesh.Trimesh(v, f, process=False)


def assemble(parts, geo, pan=0, tilt=0):
    """擺成組裝姿勢（只用於預覽）"""
    zy = geo["bh"] + SG_TAB_TO_HORN
    out = {}
    out["base"] = parts["base"]
    out["bottom"] = parts["bottom"].rotate([180, 0, 0]).translate([0, 0, 2.4 + 3.0 - 3.0 - 0.0]).translate([0, 0, 0])
    out["bottom"] = parts["bottom"].translate([0, 0, -2.4])
    out["yoke"] = parts["yoke"].rotate([0, 0, pan]).translate([0, 0, zy])
    head = parts["head"] + parts["head_cap"].translate([0, 0, geo["z_top"]])
    out["head"] = head.rotate([tilt, 0, 0]).translate([0, 0, geo["za"]]).rotate([0, 0, pan]).translate([0, 0, zy])
    return out


if __name__ == "__main__":
    outdir = sys.argv[1] if len(sys.argv) > 1 else "out"
    os.makedirs(outdir, exist_ok=True)
    report = {}
    for name, p in PLANS.items():
        parts, geo = make_parts(p)
        # 列印方向：底座倒放、頭部正放、頂蓋翻面
        printable = {
            "base": parts["base"].rotate([180, 0, 0]),
            "bottom": parts["bottom"],
            "yoke": parts["yoke"],
            "head": parts["head"],
            "head_cap": parts["head_cap"].rotate([180, 0, 0]),
        }
        for k, m in printable.items():
            tm = to_trimesh(m)
            tm.apply_translation([0, 0, -tm.bounds[0][2]])
            tm.export(f"{outdir}/{name}_{k}.stl")
            report[f"{name}_{k}"] = dict(size=np.round(tm.extents, 1).tolist(), watertight=tm.is_watertight,
                                         vol_cm3=round(tm.volume / 1000, 1))
        asm = assemble(parts, geo)
        np.save(f"{outdir}/{name}_geo.npy", geo)
        import pickle
        pickle.dump({k: to_trimesh(v) for k, v in asm.items()}, open(f"{outdir}/{name}_asm.pkl", "wb"))
        allm = trimesh.util.concatenate(list(pickle.load(open(f"{outdir}/{name}_asm.pkl", "rb")).values()))
        report[f"{name}_ASSEMBLY"] = dict(size=np.round(allm.extents, 1).tolist())
    for k, v in report.items():
        print(k, v)
