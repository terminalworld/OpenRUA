"""Measure lying mug pose from a fresh birdview cloud (arm must be parked)."""
import numpy as np, subprocess, sys
if '--nosnap' not in sys.argv: subprocess.run("timeout 300 python3 cloud_any.py birdview >/dev/null 2>&1", shell=True)
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
box=(P[:,0]>-0.34)&(P[:,0]<-0.04)&(P[:,1]<-0.35)&(P[:,1]>-0.62)
Q=P[box&(P[:,2]>0.93)&(P[:,2]<1.05)]
R=P[box&(P[:,2]>0.98)&(P[:,2]<1.05)]              # top ridge of the lying cone
cr=R.mean(0); u,s,vt=np.linalg.svd(R-cr); ax=vt[0].copy(); ax[2]=0; ax/=np.linalg.norm(ax)
tr=(R-cr)@ax
# rim end = end where ridge is higher
zlo=R[tr<np.percentile(tr,25),2].mean(); zhi=R[tr>np.percentile(tr,75),2].mean()
if zlo>zhi: ax=-ax
perp=np.array([-ax[1],ax[0],0])
c=Q.mean(0); c[2]=0.95
t=(Q-c)@ax; w=(Q-c)@perp
rim=c+ax*t.max(); base=c+ax*t.min()
L=P[box&(P[:,2]>0.906)&(P[:,2]<0.93)]; wl=(L-c)@perp; tl=(L-c)@ax; sel=(np.abs(tl)<0.06)&(np.abs(wl)<0.10)&(np.abs(wl)>0.03)
hs=float(np.sign(np.median(wl[sel]))) if sel.sum()>10 else 0.0
print(f"centroid {c.round(3)} axis(base->rim) {ax.round(3)} yaw_deg {np.degrees(np.arctan2(ax[1],ax[0])).round(1)}  ridge n={len(R)} zlo/zhi {zlo:.3f}/{zhi:.3f}")
print(f"rim centre {rim.round(3)} base centre {base.round(3)} len {t.max()-t.min():.3f} width {w.min():+.3f}..{w.max():+.3f} handle_side {hs:+.0f} (perp={perp.round(3)}) zmax {Q[:,2].max():.3f}")
np.save('mugpose.npy',np.array([*c,*ax,*rim,*base,hs]))
