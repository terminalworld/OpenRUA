import numpy as np, sys
cam = sys.argv[1]
P = np.load(f"/workspace/P_{cam}.npy"); Z = P[...,2]
reg = (P[...,0] > -0.3) & (P[...,0] < -0.12) & (P[...,1] > 0.1) & (P[...,1] < 0.3)
for lo, hi in [(0.905, 0.935), (0.935, 0.948), (0.948, 0.97)]:
    m = reg & (Z > lo) & (Z <= hi)
    if m.sum(): print(f"z({lo},{hi}] n={m.sum()} x[{P[m][:,0].min():.3f},{P[m][:,0].max():.3f}] y[{P[m][:,1].min():.3f},{P[m][:,1].max():.3f}] c=({P[m][:,0].mean():.4f},{P[m][:,1].mean():.4f}) mid=({(P[m][:,0].min()+P[m][:,0].max())/2:.4f},{(P[m][:,1].min()+P[m][:,1].max())/2:.4f})")
