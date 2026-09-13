import numpy as np
d = np.load("sideview_cloud.npz"); pw = d["pw"]; col = d["color"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
# drawer region: x -0.25..0.05, y -0.25..0.0, z 0.905..1.15
m = ok & (x>-0.26)&(x<0.06)&(y>-0.26)&(y<0.0)&(z>0.905)&(z<1.15)
print("drawer+bottle pts", m.sum())
# slice by y to find the front face: histogram of y for points with z in 0.93..1.0 and x in -0.2..-0.05 (away from bottle)
mm = m & (x>-0.2)&(x<-0.06)&(z>0.93)&(z<1.0)
h,e = np.histogram(y[mm], bins=25)
for hh,ee in zip(h,e): print(f"  y>{ee:.3f}: {hh}")
# top of drawer front
mf = m & (x>-0.2)&(x<-0.06)&(y>-0.07)&(y<-0.02)
print("front face x range", x[mf].min(), x[mf].max(), "z range", z[mf].min(), z[mf].max(), "y range", y[mf].min(), y[mf].max())
# bottle: cylinder near (0.01,-0.044)
mb = ok & (x>-0.05)&(x<0.08)&(y>-0.12)&(y<0.02)&(z>0.905)&(z<1.4)
print("bottle pts", mb.sum(), "x", x[mb].min(), x[mb].max(), "y", y[mb].min(), y[mb].max(), "z", z[mb].min(), z[mb].max())
for zl in np.arange(0.9,1.35,0.05):
    s = mb&(z>zl)&(z<zl+0.05)
    if s.sum(): print(f"  z {zl:.2f}: x {x[s].min():.3f}..{x[s].max():.3f} y {y[s].min():.3f}..{y[s].max():.3f} n={s.sum()}")
