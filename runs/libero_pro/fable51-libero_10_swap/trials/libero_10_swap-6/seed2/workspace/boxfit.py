import numpy as np, sys, cv2
sys.path.insert(0, "/workspace")
from scene import grab, cloud
cam = sys.argv[1]
color, depth, K, T = grab(cam)
P = cloud(depth, K, T); Z = P[...,2]
cv2.imwrite(f"snaps/{cam}_box.png", color)
box = [float(v) for v in sys.argv[2:6]]; zlo, zhi = float(sys.argv[6]), float(sys.argv[7])
m = (P[...,0]>box[0])&(P[...,0]<box[1])&(P[...,1]>box[2])&(P[...,1]<box[3])&(Z>zlo)&(Z<zhi)&np.isfinite(Z)
pts = P[m][:, :2]
print("n", len(pts))
c = pts.mean(0)
u, s, vt = np.linalg.svd(pts - c, full_matrices=False)
ax = vt[0]; yaw = np.degrees(np.arctan2(ax[1], ax[0]))
proj = (pts - c) @ vt.T
print(f"center=({c[0]:.4f},{c[1]:.4f}) long-axis yaw={yaw:.1f}deg extents long={proj[:,0].min():.3f}..{proj[:,0].max():.3f} short={proj[:,1].min():.3f}..{proj[:,1].max():.3f} z=({P[m][:,2].min():.3f},{P[m][:,2].max():.3f})")
