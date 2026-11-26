import numpy as np, matplotlib, biped as B, build
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from render import draw, COL
COL.update({"torso":"#f2b880","leg_L":"#7fa7c9","leg_R":"#7fa7c9","servo":"#3a3f4a","screen":"#1d2330","eye":"#ffffff"})
T=build.to_trimesh
P=B.parts()
def scene(hl,hr,al,ar):
    a=B.assemble(P,hl,hr,al,ar)
    zc=B.Z_BOARD+B.BH/2; yf=-B.OD/2
    a["screen"]=build.box(51,1,51,cy=yf+1.6,cz=zc)
    a["eye"]=build.box(7,1,15,cx=-10,cy=yf+0.9,cz=zc+3)+build.box(7,1,15,cx=10,cy=yf+0.9,cz=zc+3)
    sv=None
    for side,h in ((-1,hl),(1,hr)):
        s=build.box(B.SG_W,B.SG_L,B.HORN_FROM_TABFACE-B.PLATE,cy=B.SG_SHAFT_OFF,cz=-(B.HORN_FROM_TABFACE-B.PLATE)/2).translate([side*B.HIP_X,B.HIP_Y,0])
        sv=s if sv is None else sv+s
    a["servo"]=sv
    return {k:T(v) for k,v in a.items()}
fig=plt.figure(figsize=(13,5.4),dpi=110)
poses=[((0,0,0,0),(10,-65),"站立"),((20,20,-18,-18),(10,-120),"抬腳／扭身"),((-25,25,12,-12),(18,-35),"內八轉身")]
for i,(pz,(e,az),t) in enumerate(poses):
    ax=fig.add_subplot(1,3,i+1,projection="3d"); draw(ax,scene(*pz),e,az)
plt.tight_layout(); plt.savefig("out_biped/biped_preview.png",facecolor="white")
