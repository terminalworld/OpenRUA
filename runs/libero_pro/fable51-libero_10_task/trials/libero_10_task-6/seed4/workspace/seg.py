import numpy as np
d = np.load("birdview_cloud.npz"); Pw, color = d["pw"], d["color"]
z = Pw[...,2]
valid = np.isfinite(z)
# table height: mode of z in the table region
hist, edges = np.histogram(z[valid], bins=200, range=(0,1.2))
top = np.argsort(hist)[-6:]
for i in sorted(top): print(f"z~{edges[i]:.3f} count={hist[i]}")
