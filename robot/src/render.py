import pickle, sys, numpy as np, trimesh, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
COL={"base":"#e9e4da","bottom":"#cfc8bb","yoke":"#7fa7c9","head":"#f2b880"}
def shade(tm, color, light=np.array([0.4,-0.7,0.6])):
    n=tm.face_normals; l=light/np.linalg.norm(light)
    k=0.45+0.55*np.clip(n@l,0,1)
    c=np.array(matplotlib.colors.to_rgb(color))
    return np.clip(k[:,None]*c,0,1)
def draw(ax, meshes, elev, azim):
    tris=np.vstack([tm.triangles for tm in meshes.values()])
    cols=np.vstack([shade(tm,COL.get(name,"#ccc")) for name,tm in meshes.items()])
    ax.add_collection3d(Poly3DCollection(tris, facecolors=cols, edgecolor=cols, linewidth=0.1))
    allv=np.vstack([m.vertices for m in meshes.values()])
    mn,mx=allv.min(0),allv.max(0); c=(mn+mx)/2; r=(mx-mn).max()/2
    ax.set_xlim(c[0]-r,c[0]+r); ax.set_ylim(c[1]-r,c[1]+r); ax.set_zlim(c[2]-r,c[2]+r)
    ax.view_init(elev,azim); ax.set_box_aspect([1,1,1]); ax.axis("off")
if __name__=="__main__":
    name=sys.argv[1]
    asm=pickle.load(open(f"out/{name}_asm.pkl","rb"))
    fig=plt.figure(figsize=(12,6),dpi=110)
    for i,(e,a) in enumerate([(15,-60),(15,-120),(10,-90+180)]):
        ax=fig.add_subplot(1,3,i+1,projection="3d"); draw(ax,asm,e,a)
    plt.tight_layout(); plt.savefig(f"out/{name}_preview.png",facecolor="white")
    # parts layout
    fig=plt.figure(figsize=(15,4),dpi=100)
    for i,p in enumerate(["base","bottom","yoke","head","head_cap"]):
        tm=trimesh.load(f"out/{name}_{p}.stl")
        ax=fig.add_subplot(1,5,i+1,projection="3d"); draw(ax,{p.split('_')[0]:tm},25,-55); ax.set_title(p)
    plt.tight_layout(); plt.savefig(f"out/{name}_parts.png",facecolor="white")
