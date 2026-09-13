import numpy as np, sys
cam = sys.argv[1]
d = np.load(f"{cam}_cloud.npz"); pw, color = d["pw"], d["color"]
z = pw[...,2]; x = pw[...,0]; y = pw[...,1]
ok = np.isfinite(z)
# Bowl region candidates: x in [-0.22,-0.04], y in [-0.05, 0.14], z in 0.905..1.05
m = ok & (x>-0.22)&(x<-0.04)&(y>-0.05)&(y<0.14)&(z>0.905)&(z<1.10)
print("bowl pts", m.sum())
if m.sum():
    print(" x range", x[m].min(), x[m].max(), " y range", y[m].min(), y[m].max(), " z range", z[m].min(), z[m].max())
    # rim: top 10% of z
    zt = np.percentile(z[m], 95); mt = m & (z>zt-0.005)
    print(" rim z~", zt, " rim center x,y", x[mt].mean(), y[mt].mean(), " rim x range", x[mt].min(), x[mt].max(), " y range", y[mt].min(), y[mt].max())
    for lo,hi in [(0.905,0.93),(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.01),(1.01,1.03),(1.03,1.06)]:
        mm = m&(z>=lo)&(z<hi)
        if mm.sum(): print(f"  z {lo:.3f}-{hi:.3f}: n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
# Drawer: region x in [-0.26,0.03], y in [-0.24,-0.02], z>0.905
m = ok & (x>-0.26)&(x<0.03)&(y>-0.24)&(y<-0.02)&(z>0.905)&(z<1.12)
print("drawer pts", m.sum())
for lo,hi in [(0.905,0.93),(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.01),(1.01,1.03),(1.03,1.06),(1.06,1.12)]:
    mm = m&(z>=lo)&(z<hi)
    if mm.sum(): print(f"  z {lo:.3f}-{hi:.3f}: n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
# Bottle
m = ok & (x>0.0)&(x<0.12)&(y>-0.12)&(y<0.0)&(z>0.905)&(z<1.2)
print("bottle pts", m.sum())
if m.sum(): print(" x", x[m].min(), x[m].max(), "y", y[m].min(), y[m].max(), "z", z[m].min(), z[m].max())
