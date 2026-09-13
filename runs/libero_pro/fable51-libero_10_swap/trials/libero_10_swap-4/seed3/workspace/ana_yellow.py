import numpy as np
P = np.load("robot0_eye_in_hand_P.npy")
Z = P[..., 2]
sel = (P[...,0] > -0.08) & (P[...,0] < 0.04) & (P[...,1] > 0.13) & (P[...,1] < 0.27) & (Z > 0.44) & (Z < 0.65) & np.isfinite(Z)
pts = P[sel]
print("n", len(pts), "zmax", pts[:,2].max())
for y0 in np.arange(0.13, 0.27, 0.01):
    s = (pts[:,1] >= y0) & (pts[:,1] < y0 + 0.01)
    if s.sum() < 5: continue
    q = pts[s]
    i = q[:,2].argmax()
    print(f"y[{y0:.2f},{y0+0.01:.2f}] n={s.sum():4d} zmax={q[i,2]:.4f} at x={q[i,0]:+.4f}  x-range[{q[:,0].min():+.3f},{q[:,0].max():+.3f}]  z>0.46 x-range: "
          + (f"[{q[q[:,2]>0.46][:,0].min():+.3f},{q[q[:,2]>0.46][:,0].max():+.3f}]" if (q[:,2]>0.46).any() else "-"))
