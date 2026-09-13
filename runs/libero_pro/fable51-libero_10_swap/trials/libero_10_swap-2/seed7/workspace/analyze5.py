import numpy as np
P = np.load("/workspace/P_robot0_eye_in_hand.npy"); Z = P[...,2]
reg = (P[...,0] > -0.15) & (P[...,0] < 0.1) & (P[...,1] > -0.35) & (P[...,1] < -0.15) & (Z > 0.905) & (Z < 1.1)
for lo in np.arange(0.95, 1.06, 0.01):
    m = reg & (Z > lo) & (Z <= lo+0.01)
    if m.sum(): print(f"z({lo:.2f},{lo+0.01:.2f}] n={m.sum()} x[{P[m][:,0].min():.4f},{P[m][:,0].max():.4f}] w={P[m][:,0].max()-P[m][:,0].min():.4f} y[{P[m][:,1].min():.4f},{P[m][:,1].max():.4f}] cx={(P[m][:,0].min()+P[m][:,0].max())/2:.4f}")
m = reg & (Z > 1.02)
print("top region x-extent per y-bin:")
for yb in np.arange(-0.30, -0.18, 0.005):
    s = m & (P[...,1] >= yb) & (P[...,1] < yb+0.005)
    if s.sum(): print(f"  y={yb:.3f}: x[{P[s][:,0].min():.4f},{P[s][:,0].max():.4f}] w={P[s][:,0].max()-P[s][:,0].min():.4f} zmax={Z[s].max():.3f}")
