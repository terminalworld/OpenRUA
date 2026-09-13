import numpy as np, sys, cv2
from scipy.spatial.transform import Rotation as Rot
npy = sys.argv[1]; t = np.array([float(v) for v in sys.argv[2:5]]); q = [float(v) for v in sys.argv[5:9]]
fx = 312.77408948188935; cx=320; cy=240
d = np.load(npy); H,W = d.shape
vv,uu = np.mgrid[0:H,0:W]
P = np.stack([(uu-cx)*d/fx,(vv-cy)*d/fx,d],-1) @ Rot.from_quat(q).as_matrix().T + t
np.save("eih_cloud.npy", P)
z = P[...,2]
print("table z median (central band):", np.median(z[(np.abs(z-0.9)<0.03)]))
for lo in np.arange(0.90, 1.08, 0.01):
    m = (z>=lo)&(z<lo+0.01)&(vv<360)
    if m.sum()>5:
        s = P[m]; print(f"z[{lo:.2f},{lo+0.01:.2f}) n={m.sum():5d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
# object above table near center
m = (z>0.95)&(vv<360)&(np.abs(P[...,0]-P[...,0][240,320])<0.12)&(np.abs(P[...,1]-P[...,1][240,320])<0.12)
s = P[m]
print("object pts", len(s), "z range", s[:,2].min(), s[:,2].max())
top = s[s[:,2]>1.025]
print("top plateau: x[%.4f,%.4f] y[%.4f,%.4f] center (%.4f,%.4f)"%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max(), (top[:,0].min()+top[:,0].max())/2,(top[:,1].min()+top[:,1].max())/2))
# per-slab in y (exclude handle/spout) -> body x extent
body = top[(np.abs(top[:,1]-np.median(top[:,1]))<0.02)]
print("body(|dy|<2cm) x extent", body[:,0].min(), body[:,0].max(), "width", body[:,0].max()-body[:,0].min())
knob = s[s[:,2]>1.045]
if len(knob): print("knob center", knob[:,0].mean(), knob[:,1].mean(), "zmax", knob[:,2].max(), "n", len(knob))
# pixel of the top plateau centroid
ys, xs = np.where(m & (z>1.025))
print("top centroid px", xs.mean(), ys.mean())
