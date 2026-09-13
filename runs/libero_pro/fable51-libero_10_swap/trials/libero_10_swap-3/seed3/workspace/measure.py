import numpy as np, sys
cam = sys.argv[1] if len(sys.argv)>1 else "robot0_robotview"
d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
# bowl: above table in central area, excluding drawer (y<-0.02) and shelf(y>0.15)
m = ok&(z>0.905)&(z<1.0)&(x>-0.3)&(x<0.1)&(y>-0.01)&(y<0.15)
print("bowl pts", m.sum(), "x", x[m].min(), x[m].max(), "y", y[m].min(), y[m].max(), "z", z[m].min(), z[m].max())
rim = m&(z>z[m].max()-0.008)
print(" rim center", x[rim].mean(), y[rim].mean(), "rim x", x[rim].min(), x[rim].max(), "rim y", y[rim].min(), y[rim].max())
# bottle
mb = ok&(z>0.905)&(x>-0.05)&(x<0.08)&(y>-0.12)&(y<0.0)
print("bottle pts", mb.sum(), "x", x[mb].min(), x[mb].max(), "y", y[mb].min(), y[mb].max(), "ztop", z[mb].max())
for zl in [0.92, 1.0, 1.1, 1.2]:
    s = mb&(z>zl)&(z<zl+0.05)
    if s.sum(): print(f"  z {zl:.2f}: x {x[s].mean():.3f} y {y[s].mean():.3f} n={s.sum()}")
# drawer front face & floor
mf = ok&(x>-0.19)&(x<-0.07)&(y>-0.10)&(y<0.0)&(z>0.93)&(z<1.0)
h,e = np.histogram(y[mf], bins=20)
print("drawer front y-hist:", [(round(ee,3),hh) for hh,ee in zip(h,e) if hh>0])
fl = ok&(x>-0.18)&(x<-0.04)&(y>-0.19)&(y<-0.09)&(z>0.905)&(z<0.96)
print("floor z", np.median(z[fl]), "n", fl.sum())
