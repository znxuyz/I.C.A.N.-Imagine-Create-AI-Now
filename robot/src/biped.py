"""
桌上型 AI 機器人 — 走路版（雙足，Otto 式 4 馬達）
  2 顆髖關節馬達（身體底部，轉軸朝下）→ 腿左右轉
  2 顆腳踝馬達（腿裡，轉軸朝前）      → 腳板左右翻
  頭／身體：M5Stack CoreS3 SE 從上方插入，螢幕當臉
單位 mm。python3 biped.py out_biped
"""
import os, sys, math, pickle
import numpy as np
from build import (M, box, cyl, rbox, horn_recess, to_trimesh, WALL, CLR,
                   SG_L, SG_W, SG_TAB_HOLE, SG_SHAFT_OFF)

TAB_T = 2.5
HORN_FROM_TABFACE = 8.6      # 耳朵（轉軸側那面）→ 舵盤靠馬達那面
TAB_OVER = (32.2 - SG_L) / 2  # 耳朵超出本體的長度

# ---- 身體 ----
BW, BH, BT = 54.0 + 2 * CLR, 54.0 + 2 * CLR, 15.5 + 2 * CLR   # CoreS3 SE
SHELF = 3.5                   # 板子墊高（讓馬達前耳朵塞在板子下面）
PLATE = 3.0
SERVO_ZONE = SG_L + 0.4 + TAB_OVER     # 板子後方給髖馬達的深度
IN_W = BW
IN_D = BT + 1.0 + SERVO_ZONE
OW, OD = IN_W + 2 * WALL, IN_D + 2 * WALL
HIP_X = 17.0
Z_BOARD = PLATE + SHELF
Z_TOP = Z_BOARD + BH

y_in_front, y_in_back = -IN_D / 2, IN_D / 2
SERVO_CY = y_in_back - TAB_OVER - (SG_L + 0.4) / 2
HIP_Y = SERVO_CY - SG_SHAFT_OFF          # 轉軸朝前偏


def torso():
    t = rbox(OW, OD, Z_TOP, 5, cz=Z_TOP / 2)
    t = t - rbox(IN_W, IN_D, Z_TOP, 3, cz=PLATE + Z_TOP / 2)            # 內腔，頂部開口
    # 板子墊高條（只放兩側，避開馬達耳朵）
    for sx in (-1, 1):
        t = t + box(3, BT, SHELF, cx=sx * (IN_W / 2 - 1.5), cy=y_in_front + BT / 2, cz=PLATE + SHELF / 2)
    # 螢幕窗
    t = t - box(50, WALL * 3, 50, cy=-OD / 2, cz=Z_BOARD + BH / 2)
    # 髖馬達孔（倒裝，轉軸朝下，長邊沿 y）
    for sx in (-1, 1):
        t = t - box(SG_W + 0.4, SG_L + 0.4, PLATE + 2, cx=sx * HIP_X, cy=SERVO_CY, cz=PLATE / 2)
        for dy in (-SG_TAB_HOLE / 2, SG_TAB_HOLE / 2):
            t = t - cyl(PLATE + 2, 1.8, sx * HIP_X, SERVO_CY + dy, PLATE / 2)
    # 背面走線／USB 開口
    t = t - box(34, WALL * 3, 12, cy=OD / 2, cz=Z_TOP - 14)
    t = t - box(16, WALL * 3, 9, cy=OD / 2, cz=PLATE + 10)
    return t


def torso_cap():
    cap = rbox(OW, OD, WALL, 5, cz=WALL / 2) + box(IN_W - 0.4, IN_D - 0.4, 3, cz=-1.5)
    cap = cap - box(IN_W - 4, IN_D - 4, 3.2, cz=-1.5)
    return cap


# ---- 腿（原點：髖轉軸，z=0 是腿頂面）----
LEG_TOP_T = 4.0
LW_IN = SG_W + 0.4
LW = LW_IN + 2 * WALL
LEG_FRONT_T = 3.0
LEG_IN_D = TAB_T + 15.9 + 1.8
LD = LEG_FRONT_T + LEG_IN_D + WALL
LEG_YF = -LD / 2                                   # 腿前面
Z_AX = -(LEG_TOP_T + SG_L / 2 + SG_SHAFT_OFF + TAB_OVER + 0.5)   # 腳踝軸高度
LEG_BOT = Z_AX - 6
FRONT_BOT = Z_AX - SG_TAB_HOLE / 2 + SG_SHAFT_OFF - 3.5
Y_HORN = LEG_YF + LEG_FRONT_T - HORN_FROM_TABFACE   # 腳踝舵盤靠馬達那面


def leg():
    h = -LEG_BOT
    lg = box(LW, LD, h, cz=-h / 2)
    lg = lg - box(LW_IN, LD - LEG_FRONT_T - WALL, h, cy=(LEG_FRONT_T - WALL) / 2, cz=-h / 2 - LEG_TOP_T)
    # 前牆往下延伸（固定下方耳朵）
    fh = LEG_BOT - FRONT_BOT
    lg = lg + box(LW, LEG_FRONT_T, fh, cy=LEG_YF + LEG_FRONT_T / 2, cz=LEG_BOT - fh / 2)
    # 前牆：腳踝馬達孔
    bc = Z_AX + SG_SHAFT_OFF
    lg = lg - box(SG_W + 0.4, LEG_FRONT_T + 2, SG_L + 0.4, cy=LEG_YF + LEG_FRONT_T / 2, cz=bc)
    for dz in (-SG_TAB_HOLE / 2, SG_TAB_HOLE / 2):
        lg = lg - cyl(LEG_FRONT_T + 2, 1.8, 0, LEG_YF + LEG_FRONT_T / 2, bc + dz, axis="y")
    # 頂面：髖舵盤嵌槽（臂朝後）
    lg = lg - horn_recess().rotate([0, 0, 90])
    # 走線孔
    lg = lg - box(8, 6, LEG_TOP_T + 2, cy=LD / 2 - WALL - 4, cz=-LEG_TOP_T / 2)
    return lg


# ---- 腳板（原點同腿座標）----
SOLE_W, SOLE_L, SOLE_T = 30.0, 46.0, 3.0
SOLE_TOP = Z_AX - 15.0
POST_T = 4.5


def foot(side):
    """side=+1 右腳，-1 左腳；腳板往外側偏一點增加穩定"""
    y0 = Y_HORN - POST_T
    sole = rbox(SOLE_W, SOLE_L, SOLE_T, 6, cx=side * 3, cy=y0 + SOLE_L / 2, cz=SOLE_TOP - SOLE_T / 2)
    post_h = (Z_AX + 7) - (SOLE_TOP - SOLE_T)
    post = box(14, POST_T, post_h, cy=Y_HORN - POST_T / 2, cz=(SOLE_TOP - SOLE_T) + post_h / 2)
    post = post + cyl(POST_T, 14, 0, Y_HORN - POST_T / 2, Z_AX + 7, axis="y")
    f = sole + post
    hr = horn_recess().rotate([-90, 0, 0]).rotate([0, 90, 0]).translate([0, Y_HORN, Z_AX])
    return f - hr


def parts():
    return {"torso": torso(), "torso_cap": torso_cap(), "leg_L": leg(), "leg_R": leg(),
            "foot_L": foot(-1), "foot_R": foot(1)}


Z_LEG_TOP = -(HORN_FROM_TABFACE - PLATE)   # 腿頂面相對身體底面


def assemble(p, hipL=0, hipR=0, ankL=0, ankR=0):
    out = {}
    out["torso"] = p["torso"] + p["torso_cap"].translate([0, 0, Z_TOP])
    for side, name, hip, ank in ((-1, "L", hipL, ankL), (1, "R", hipR, ankR)):
        lg = p["leg_" + name]
        ft = p["foot_" + name].translate([0, 0, -Z_AX]).rotate([0, ank, 0]).translate([0, 0, Z_AX])
        grp = (lg + ft).rotate([0, 0, hip]).translate([side * HIP_X, HIP_Y, Z_LEG_TOP])
        out["leg_" + name] = grp
    return out


if __name__ == "__main__":
    outdir = sys.argv[1] if len(sys.argv) > 1 else "out_biped"
    os.makedirs(outdir, exist_ok=True)
    p = parts()
    printable = {
        "torso": p["torso"], "torso_cap": p["torso_cap"].rotate([180, 0, 0]),
        "leg": p["leg_L"].rotate([180, 0, 0]),          # 倒過來印（頂面朝下）
        "foot_L": p["foot_L"], "foot_R": p["foot_R"],
    }
    for k, m in printable.items():
        assert str(m.status()) == "Error.NoError", k
        tm = to_trimesh(m)
        tm.apply_translation([0, 0, -tm.bounds[0][2]])
        tm.export(f"{outdir}/{k}.stl")
        print(k, np.round(tm.extents, 1).tolist(), tm.is_watertight, round(tm.volume / 1000, 1), "cm3")
    asm = assemble(p)
    allm = trimesh_all = [to_trimesh(v) for v in asm.values()]
    import trimesh
    a = trimesh.util.concatenate(allm)
    print("ASSEMBLY", np.round(a.extents, 1).tolist(), "bottom z", round(a.bounds[0][2], 1))
    print(dict(HIP_Y=HIP_Y, Z_AX=Z_AX, LD=LD, LW=LW, OD=OD, OW=OW, Z_TOP=Z_TOP))
