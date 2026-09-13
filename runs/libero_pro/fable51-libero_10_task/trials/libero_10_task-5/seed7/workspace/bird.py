import numpy as np, subprocess, sys
from scipy.optimize import least_squares
tcp=np.array(list(map(float,sys.argv[1:4])))
for _ in range(3):
    if subprocess.run(["timeout","120","python3","depth_world.py","birdview"],stdout=subprocess.DEVNULL).returncode==0: break
xyz=np.load("birdview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,0]>tcp[0]-0.09)&(xyz[:,0]<tcp[0]+0.09)&(xyz[:,1]>tcp[1]-0.03)&(xyz[:,1]<tcp[1]+0.14)
p=xyz[m]
def fit(q,r0,label):
    if len(q)<8: print(label,"too few",len(q)); return None
    f=lambda c: np.hypot(q[:,0]-c[0],q[:,1]-c[1])-r0
    c=least_squares(f,[q[:,0].mean(),q[:,1].mean()]).x
    print(f"{label}: n={len(q)} center {c[0]:.4f},{c[1]:.4f} resid {np.abs(f(c)).mean():.4f}  xrange {q[:,0].min():.3f}..{q[:,0].max():.3f} yrange {q[:,1].min():.3f}..{q[:,1].max():.3f}")
    return c
for zlo in np.arange(tcp[2]-0.12,tcp[2]+0.03,0.01):
    q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.01)]
    if len(q): print(f"  z{zlo:.3f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
rim=p[(p[:,2]>tcp[2]-0.005)&(p[:,2]<tcp[2]+0.02)]
base=p[(p[:,2]>tcp[2]-0.105)&(p[:,2]<tcp[2]-0.08)]
cr=fit(rim,0.052,"rim(r=.052)"); cb=fit(base,0.037,"innerbase(r=.037)")
if cr is not None and cb is not None: print("base - rim offset:", np.round(cb-cr,4))
