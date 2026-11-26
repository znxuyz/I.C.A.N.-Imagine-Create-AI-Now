"""
桌上型 AI 機器人 — 兩輪自平衡版 v0.3
  N20 減速馬達（含霍爾編碼器）×2，背對背裝在身體底部
  平衡控制：ESP32-S3 開發板 + MPU6050 + TB6612FNG（裝在背蓋內側）
  臉：M5Stack CoreS3 SE（從上方插入），透過 UART 跟平衡板溝通
  電源：18650 ×1 + 5V 升壓板
python3 balance.py out_balance
"""
import os, sys
import numpy as np
from build import M, box, cyl, rbox, to_trimesh, WALL, CLR

# N20 + 編碼器（買到後量一下）
N20_L, N20_W, N20_H = 34.0, 12.0, 10.0     # 含編碼器總長、寬、高
N20_HOLE = 9.0                              # 齒輪箱面兩個 M1.6 孔距
BAT_LEN = 78.0
SHAFT_D, SHAFT_FLAT = 3.0, 2.5             # D 型軸
WHEEL_D, WHEEL_W = 66.0, 10.0

IN_W = max(2 * N20_L + 2.0, BAT_LEN + 6.0)
IN_D = 30.0
OW, OD = IN_W + 2 * WALL, IN_D + 2 * WALL
FLOOR = 3.0
AX_Z = FLOOR + N20_H / 2                   # 馬達軸高度（身體座標）
BAT_Z0 = FLOOR + N20_H + 2.0               # 電池座底
BAT = (78.0, 21.0, 20.0)                   # 18650 單槽電池座
BOARD = (54.0 + 2 * CLR, 54.0 + 2 * CLR, 15.5 + 2 * CLR)   # CoreS3 SE
Z_BOARD = BAT_Z0 + BAT[2] + 2.0
Z_TOP = Z_BOARD + BOARD[1]


def chassis():
    c = rbox(OW, OD, Z_TOP, 5, cz=Z_TOP / 2)
    c = c - rbox(IN_W, IN_D, Z_TOP, 3, cz=FLOOR + Z_TOP / 2)
    yf = -IN_D / 2
    # 馬達槽：兩個擋塊夾住，束帶固定
    for sx in (-1, 1):
        x0 = sx * (IN_W / 2 - N20_L / 2)
        rails = box(N20_L, 2.0, N20_H * 0.8, cx=x0, cy=-(N20_W / 2 + 1.2), cz=FLOOR + N20_H * 0.4) + \
            box(N20_L, 2.0, N20_H * 0.8, cx=x0, cy=(N20_W / 2 + 1.2), cz=FLOOR + N20_H * 0.4)
        c = c + rails
        for dx in (-9, 9):
            c = c - box(3.5, IN_D + 6, 1.6, cx=x0 + dx, cz=FLOOR / 2 + 0.2)          # 束帶槽（底板）
        # 側牆：軸孔 + 兩個 M1.6 固定孔
        c = c - cyl(WALL * 3, 4.8, sx * OW / 2, 0, AX_Z, axis="x")
        for dy in (-N20_HOLE / 2, N20_HOLE / 2):
            c = c - cyl(WALL * 3, 1.8, sx * OW / 2, dy, AX_Z, axis="x")
    # 電池座兩側擋塊
    for sx in (-1, 1):
        c = c + box(2.0, BAT[1] + 4, 6, cx=sx * (BAT[0] / 2 + 1.2), cz=BAT_Z0 + 3)
    # 板子擱架
    for sx in (-1, 1):
        c = c + box(3, BOARD[2], 3, cx=sx * (IN_W / 2 - 1.5), cy=yf + BOARD[2] / 2, cz=Z_BOARD - 1.5)
    c = c - box(50, WALL * 3, 50, cy=-OD / 2, cz=Z_BOARD + BOARD[1] / 2)      # 螢幕窗
    # 背面大開口（背蓋塞入）
    c = c - box(IN_W - 8, WALL * 3, Z_TOP - 16, cy=OD / 2, cz=8 + (Z_TOP - 16) / 2)
    return c


def back_cover():
    """背蓋：內側立柱放 ESP32-S3 開發板、TB6612、MPU6050、升壓板"""
    w, h = IN_W - 8, Z_TOP - 16
    p = box(w + 6, WALL, h + 6, cy=WALL / 2)
    lip = box(w - 0.5, 2.5, h - 0.5, cy=-1.25) - box(w - 4.5, 3, h - 4.5, cy=-1.25)
    p = p + lip
    def posts(cx, cz, dx, dz):
        s = None
        for sx in (-1, 1):
            for sz in (-1, 1):
                q = cyl(5, 5, cx + sx * dx / 2, -2.5 - 2.5, cz + sz * dz / 2, axis="y") - \
                    cyl(6, 1.8, cx + sx * dx / 2, -2.5 - 2.5, cz + sz * dz / 2, axis="y")
                s = q if s is None else s + q
        return s
    # 位置（背蓋中心為原點，z 向上）
    p = p + posts(0, h / 2 - 18, 60, 20)          # ESP32-S3 開發板（約 63×26 的孔位，請依實物調整）
    p = p + posts(-18, -h / 2 + 18, 15, 15)       # TB6612 模組（約 20×20）
    p = p + posts(16, -h / 2 + 18, 15, 11)        # MPU6050 GY-521（約 21×16）
    p = p + posts(0, -2, 30, 12)                  # 升壓板
    # 走線孔
    p = p - box(10, 10, 6, cz=0)
    return p


def wheel():
    r = WHEEL_D / 2
    w = cyl(WHEEL_W, WHEEL_D, axis="z")
    w = w - (cyl(2.4, WHEEL_D + 1) - cyl(2.4, WHEEL_D - 4.0))           # 輪胎溝（套 O 型環或橡皮筋）
    w = w - cyl(WHEEL_W - 4, WHEEL_D - 12, cz=0)                         # 減重
    w = w + cyl(WHEEL_W, 14)                                             # 輪轂
    for k in range(5):                                                   # 輪輻
        w = w + box(r - 6, 4, WHEEL_W - 4).translate([(r - 6) / 2, 0, 0]).rotate([0, 0, k * 72])
    w = w + cyl(8, 10, cz=WHEEL_W / 2 + 4 - 0.01)                       # 軸套
    dshaft = cyl(40, SHAFT_D + 0.2) - box(5, 5, 40, cx=SHAFT_FLAT + 0.1 - (SHAFT_D + 0.2) / 2 + 2.5)
    w = w - dshaft
    w = w - cyl(6, 1.8, 0, 0, WHEEL_W / 2 + 4, axis="x").translate([3, 0, 0])   # M2 止付螺絲孔
    return w


def head_cap():
    cap = rbox(OW, OD, WALL, 5, cz=WALL / 2) + box(IN_W - 0.4, IN_D - 0.4, 3, cz=-1.5)
    return cap - box(IN_W - 4, IN_D - 4, 3.2, cz=-1.5)


def parts():
    return dict(chassis=chassis(), back_cover=back_cover(), wheel=wheel(), head_cap=head_cap())


GROUND = WHEEL_D / 2 - AX_Z     # 身體底面離地高度


def assemble(p, lean=0):
    out = {}
    body = p["chassis"] + p["head_cap"].translate([0, 0, Z_TOP]) + \
        p["back_cover"].translate([0, OD / 2, 8 + (Z_TOP - 16) / 2])
    out["torso"] = body
    wl = None
    for sx in (-1, 1):
        w = p["wheel"].rotate([0, 90 * sx, 0]).translate([sx * (OW / 2 + 1 + WHEEL_W / 2), 0, AX_Z])
        wl = w if wl is None else wl + w
    out["leg_L"] = wl
    tot = {k: v.translate([0, 0, -AX_Z]).rotate([lean, 0, 0]).translate([0, 0, WHEEL_D / 2]) for k, v in out.items()}
    return tot


if __name__ == "__main__":
    outdir = sys.argv[1] if len(sys.argv) > 1 else "out_balance"
    os.makedirs(outdir, exist_ok=True)
    import trimesh
    p = parts()
    pr = {"chassis": p["chassis"], "back_cover": p["back_cover"].rotate([-90, 0, 0]),
          "wheel": p["wheel"], "head_cap": p["head_cap"].rotate([180, 0, 0])}
    for k, m in pr.items():
        assert str(m.status()) == "Error.NoError", k
        tm = to_trimesh(m); tm.apply_translation([0, 0, -tm.bounds[0][2]])
        tm.export(f"{outdir}/{k}.stl")
        print(k, np.round(tm.extents, 1).tolist(), tm.is_watertight, round(tm.volume / 1000, 1), "cm3")
    a = trimesh.util.concatenate([to_trimesh(v) for v in assemble(p).values()])
    print("ASSEMBLY", np.round(a.extents, 1).tolist(), "ground clearance", round(GROUND, 1), "Z_TOP", Z_TOP)
