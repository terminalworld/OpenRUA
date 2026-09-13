import numpy as np
P = np.load("birdview_xyz.npy"); z = P[...,2]
print("v-profile (x) at u=275..285 avg:")
for v in range(150,200):
    zz = z[v,275:286].mean(); print(v, f"x={P[v,280,0]:.3f} z={zz:.3f}", "#" if zz>1.0 else ("." if zz<0.93 else "o"))
print("u-profile (y) at v=162..168 avg:")
for u in range(240,320):
    zz = z[162:169,u].mean(); print(u, f"y={P[165,u,1]:.3f} z={zz:.3f}", "#" if zz>1.0 else ("." if zz<0.93 else "o"))
