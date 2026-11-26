import matplotlib, build, balance as Bl
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from render import draw, COL
COL.update({"torso":"#f2b880","leg_L":"#3a3f4a","screen":"#1d2330","eye":"#ffffff"})
P=Bl.parts()
def scene(lean):
    a=Bl.assemble(P,lean)
    zc=Bl.Z_BOARD+Bl.BOARD[1]/2; yf=-Bl.OD/2
    sc=build.box(51,1,51,cy=yf+1.6,cz=zc); ey=build.box(7,1,15,cx=-10,cy=yf+0.9,cz=zc+3)+build.box(7,1,15,cx=10,cy=yf+0.9,cz=zc+3)
    f=lambda m:m.translate([0,0,-Bl.AX_Z]).rotate([lean,0,0]).translate([0,0,Bl.WHEEL_D/2])
    a["screen"]=f(sc); a["eye"]=f(ey)
    return {k:build.to_trimesh(v) for k,v in a.items()}
fig=plt.figure(figsize=(13,5.4),dpi=110)
for i,(lean,e,az) in enumerate([(0,10,-65),(-10,8,-10),(8,25,-140)]):
    ax=fig.add_subplot(1,3,i+1,projection="3d"); draw(ax,scene(lean),e,az)
plt.tight_layout(); plt.savefig("out_balance/balance_preview.png",facecolor="white")
