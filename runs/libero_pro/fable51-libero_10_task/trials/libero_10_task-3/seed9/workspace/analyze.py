import numpy as np
P = np.load("snaps/birdview_xyz.npy")
Z = P[...,2]
print("z histogram (birdview):")
h, e = np.histogram(Z[np.isfinite(Z)], bins=60, range=(0.0, 1.6))
for c, lo in zip(h, e[:-1]):
    if c > 50: print(f"  {lo:.3f}-{lo+e[1]-e[0]:.3f}: {c}")
# table height: mode
# Bottle: birdview pixel ~ (345, 240)? check world coords around candidate pixels
for (u,v) in [(345,240),(348,238),(300,300),(400,300),(320,240),(420,300),(360,300)]:
    print((u,v), P[v,u])
