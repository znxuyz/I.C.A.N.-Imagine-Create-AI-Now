import sys, numpy as np, trimesh, matplotlib, build as B
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from render import draw, COL
COL.update({"servo":"#3a3f4a","screen":"#1d2330","eye":"#ffffff"})
def T(m): return B.to_trimesh(m)
def scene(name, pan=0, tilt=0):
    p=B.PLANS[name]; parts,g=B.make_parts(p)
    zy=g["bh"]+B.SG_TAB_TO_HORN
    out={}
    out["base"]=parts["base"]; out["bottom"]=parts["bottom"].translate([0,0,-2.4])
    # pan servo dummy (gear part above base)
    sv=B.box(B.SG_L,B.SG_W,B.SG_TAB_TO_HORN,cx=-B.SG_SHAFT_OFF,cz=g["bh"]+B.SG_TAB_TO_HORN/2)
    out["servo"]=sv
    yoke=parts["yoke"]
    bc=g["za"]-B.SG_SHAFT_OFF
    ts=B.box(15.9,B.SG_W,B.SG_L,cx=g["xo"]+15.9/2,cz=bc)+B.box(32.2,B.SG_W,2.5,cx=0,cz=0).rotate([0,90,0]).translate([g["xo"]+1.25,0,bc])
    head=parts["head"]+parts["head_cap"].translate([0,0,g["z_top"]])
    ww,wh,dz=p["window_w"],p["window_h"],p["window_dz"]
    scr=B.box(ww+1,1,wh+1,cy=-g["od"]/2+1.6,cz=dz)
    eyes=B.box(ww*0.14,1,wh*0.3,cx=-ww*0.2,cy=-g["od"]/2+0.9,cz=dz+wh*0.06)+B.box(ww*0.14,1,wh*0.3,cx=ww*0.2,cy=-g["od"]/2+0.9,cz=dz+wh*0.06)
    def place_head(m): return m.rotate([tilt,0,0]).translate([0,0,g["za"]]).rotate([0,0,pan]).translate([0,0,zy])
    def place_yoke(m): return m.rotate([0,0,pan]).translate([0,0,zy])
    out["yoke"]=place_yoke(yoke); out["servo"]=out["servo"]+place_yoke(ts)
    out["head"]=place_head(head); out["screen"]=place_head(scr); out["eye"]=place_head(eyes)
    return {k:T(v) for k,v in out.items()}
name=sys.argv[1]
fig=plt.figure(figsize=(13,5.2),dpi=110)
views=[(12,-70,0,0,"正面"),(12,-125,25,-15,"左轉＋抬頭"),(18,-20,-20,12,"右轉＋點頭")]
for i,(e,a,pan,tilt,t) in enumerate(views):
    ax=fig.add_subplot(1,3,i+1,projection="3d"); draw(ax,scene(name,pan,tilt),e,a)
plt.tight_layout(); plt.savefig(f"out/{name}_preview.png",facecolor="white")
