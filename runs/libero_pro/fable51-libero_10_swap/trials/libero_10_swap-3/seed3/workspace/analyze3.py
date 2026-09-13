import numpy as np
d = np.load("robot0_robotview_cloud.npz"); pw = d["pw"]; col = d["color"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
# bowl region in robot0_robotview pixels
sub = np.zeros_like(z, bool); sub[60:160, 305:445] = True
m = sub & (z > 0.905)
print("bowl pts", m.sum())
print(" x:", x[m].min(), x[m].max(), " y:", y[m].min(), y[m].max(), " z:", z[m].min(), z[m].max())
# rim = highest points
rim = m & (z > z[m].max() - 0.01)
print(" rim x:", x[rim].min(), x[rim].max(), " y:", y[rim].min(), y[rim].max(), "n", rim.sum())
print(" rim center:", x[rim].mean(), y[rim].mean(), " top z:", z[m].max())
# bottle region: pixels ~ u 235-295, v 70-265
sub = np.zeros_like(z, bool); sub[65:270, 230:300] = True
mb = sub & (z > 0.92)
print("bottle pts", mb.sum(), " x:", x[mb].min(), x[mb].max(), " y:", y[mb].min(), y[mb].max(), " ztop", z[mb].max())
top = mb & (z > z[mb].max()-0.03)
print(" bottle cap center:", x[top].mean(), y[top].mean())
# drawer: the open bottom drawer, pixels u 60-250, v 90-300
sub = np.zeros_like(z, bool); sub[85:300, 55:250] = True
md = sub & (z > 0.905)
print("drawer pts", md.sum(), " x:", x[md].min(), x[md].max(), " y:", y[md].min(), y[md].max(), " z:", z[md].min(), z[md].max())
# histogram of drawer z
h, e = np.histogram(z[md], bins=20)
for hh, ee in zip(h, e): print(f"  z>{ee:.3f}: {hh}")
