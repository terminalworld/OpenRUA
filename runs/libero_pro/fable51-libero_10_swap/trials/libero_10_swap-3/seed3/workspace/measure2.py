import numpy as np
d = np.load("robot0_robotview_cloud.npz"); pw = d["pw"]; col=d["color"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
sub = np.zeros_like(ok); sub[115:210, 310:435] = True
m = ok&sub&(z>0.905)
# exclude the bottle cap: dark/brown; bowl is grey-ish. Use x<0.0 or y>0.03
m2 = m & ((x < -0.0) | (y > 0.03))
print("bowl pts", m2.sum(), "x", x[m2].min(), x[m2].max(), "y", y[m2].min(), y[m2].max(), "z", z[m2].min(), z[m2].max())
rim = m2&(z>z[m2].max()-0.006)
print(" rim center", x[rim].mean(), y[rim].mean(), "rim x", x[rim].min(), x[rim].max(), "rim y", y[rim].min(), y[rim].max(), "n", rim.sum())
# bottle: region below the bowl in image
sub = np.zeros_like(ok); sub[160:330, 295:370] = True
mb = ok&sub&(z>0.905)
print("bottle pts", mb.sum(), "x", x[mb].min(), x[mb].max(), "y", y[mb].min(), y[mb].max(), "ztop", z[mb].max())
for xl in np.arange(-0.06, 0.26, 0.04):
    s = mb&(x>xl)&(x<xl+0.04)
    if s.sum(): print(f"  x {xl:.2f}: y {y[s].min():.3f}..{y[s].max():.3f} ztop {z[s].max():.3f} n={s.sum()}")
