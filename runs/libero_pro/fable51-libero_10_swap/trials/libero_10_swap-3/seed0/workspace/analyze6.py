import numpy as np
d = np.load("robot0_eye_in_hand_cloud.npz"); pw, color = d["pw"], d["color"]
z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
r = np.hypot(x+0.14, y-0.03)
m = ok&(r<0.09)&(z>0.945)&(z<0.975)
print("rim n", m.sum(), "center", x[m].mean(), y[m].mean(), "x", x[m].min(), x[m].max(), "y", y[m].min(), y[m].max(), "z", z[m].min(), z[m].max())
# fit circle to rim points
A = np.c_[2*x[m], 2*y[m], np.ones(m.sum())]; b = x[m]**2+y[m]**2
cx, cy, c = np.linalg.lstsq(A,b,rcond=None)[0]; R = np.sqrt(c+cx**2+cy**2)
print("circle fit center", cx, cy, "R", R)
# drawer: x in [-0.26,0.06], y in [-0.26,-0.03]
m = ok&(x>-0.26)&(x<0.06)&(y>-0.26)&(y<-0.03)&(z>0.905)&(z<1.0)
for lo,hi in [(0.905,0.93),(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.0)]:
    mm = m&(z>=lo)&(z<hi)
    if mm.sum()>10: print(f"drawer z {lo:.3f}-{hi:.3f}: n={mm.sum()} x[{x[mm].min():.3f},{x[mm].max():.3f}] y[{y[mm].min():.3f},{y[mm].max():.3f}]")
# height map of drawer region at 1cm
print("drawer heightmap (cm above 0.90), rows x, cols y from -0.26 to -0.02")
ys = np.arange(-0.26,-0.02,0.01); xs=np.arange(-0.26,0.08,0.01)
print("     "+"".join(f"{int(round(yy*100)):4d}" for yy in ys))
for xx in xs:
    row=""
    for yy in ys:
        mm = ok&(np.abs(x-xx)<0.005)&(np.abs(y-yy)<0.005)
        row += f"{int(round((np.median(z[mm])-0.90)*100)):4d}" if mm.sum() else "   ."
    print(f"{int(round(xx*100)):4d} {row}")
