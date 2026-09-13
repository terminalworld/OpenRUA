import numpy as np
for cam in ["sideview", "frontview"]:
    P = np.load(f"/workspace/P_{cam}.npy"); Z = P[...,2]
    reg = (P[...,1] < -0.15) & (P[...,1] > -0.35) & (P[...,0] > -0.15) & (P[...,0] < 0.1) & (Z > 0.905)
    print(cam)
    for zb in np.arange(0.90, 1.07, 0.01):
        s = reg & (Z >= zb) & (Z < zb+0.01)
        if s.sum(): print(f" z={zb:.2f}: n={s.sum()} x[{P[s][:,0].min():.3f},{P[s][:,0].max():.3f}] y[{P[s][:,1].min():.3f},{P[s][:,1].max():.3f}]")
