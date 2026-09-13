import numpy as np
P = np.load("birdview_xyz.npy"); z = P[...,2]
print("u-profile (y) at v=172..178 avg:")
for u in range(250,310):
    zz = z[172:179,u].mean(); print(u, f"y={P[175,u,1]:.3f} z={zz:.3f}", "#" if zz>1.0 else ("." if zz<0.93 else "o"))
