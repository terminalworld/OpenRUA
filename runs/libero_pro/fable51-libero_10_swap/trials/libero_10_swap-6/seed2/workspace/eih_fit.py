import numpy as np, sys
sys.path.insert(0, "/workspace")
from scene import grab, cloud
color, depth, K, T = grab("robot0_eye_in_hand")
P = cloud(depth, K, T); Z = P[...,2]
import cv2; cv2.imwrite("snaps/eih.png", color); np.save("snaps/eih_P.npy", P)
def fit_circle(xy):
    x,y = xy[:,0], xy[:,1]
    A = np.c_[2*x, 2*y, np.ones_like(x)]
    c, *_ = np.linalg.lstsq(A, x*x+y*y, rcond=None)
    return c[0], c[1], np.sqrt(c[2] + c[0]**2 + c[1]**2)
box = [float(v) for v in sys.argv[1:5]]; zlo, zhi = float(sys.argv[5]), float(sys.argv[6])
m = (P[...,0]>box[0])&(P[...,0]<box[1])&(P[...,1]>box[2])&(P[...,1]<box[3])&(Z>zlo)&(Z<zhi)&np.isfinite(Z)
pts = P[m]
print("n", len(pts))
if len(pts):
    cx,cy,r = fit_circle(pts[:,:2])
    print(f"circle c=({cx:.4f},{cy:.4f}) r={r:.4f} | xr=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yr=({pts[:,1].min():.3f},{pts[:,1].max():.3f}) z=({pts[:,2].min():.3f},{pts[:,2].max():.3f}) mean=({pts[:,0].mean():.4f},{pts[:,1].mean():.4f})")
