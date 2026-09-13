import numpy as np, sys
sys.argv=["scene.py","robot0_eye_in_hand"]
import scene
D,C,K,R,tt=scene.cam_model("robot0_eye_in_hand")
P=scene.to_world(D,K,R,tt)
np.save("snaps/eih_P.npy",P)
x,y,z=P[...,0],P[...,1],P[...,2]
ok=np.isfinite(z)&(D>0.05)
print("depth range", D[ok].min(), D[ok].max())
# points inside cavity footprint
m=ok&(x>-0.27)&(x<-0.07)&(y>-0.35)&(y<-0.16)
print("cavity pts", m.sum())
if m.sum():
    zs=z[m]
    print("z percentiles", np.percentile(zs,[0,5,25,50,75,95,100]).round(3))
    for lo in np.arange(0.88,1.12,0.02):
        mm=m&(z>=lo)&(z<lo+0.02)
        if mm.sum(): print(f"z[{lo:.2f},{lo+0.02:.2f}) n={mm.sum()} y[{y[mm].min():.3f},{y[mm].max():.3f}] x[{x[mm].min():.3f},{x[mm].max():.3f}]")
# opening frame: points near the face plane y in [-0.37,-0.35]
f=ok&(y>-0.38)&(y<-0.35)&(x>-0.32)&(x<0.1)
print("face pts", f.sum(), "z range", z[f].min().round(3), z[f].max().round(3))
for xx in np.arange(-0.30,0.10,0.02):
    mm=f&(abs(x-xx)<0.01)
    if mm.sum(): print(f"face x={xx:.2f} z[{z[mm].min():.3f},{z[mm].max():.3f}] n={mm.sum()}")
