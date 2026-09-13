import numpy as np
from scipy.optimize import least_squares
def fit(pts, r=None):
    x,y = pts[:,0], pts[:,1]
    if r is None:
        f = lambda p: np.hypot(x-p[0], y-p[1]) - p[2]; p0=[x.mean(), y.mean(), 0.04]
    else:
        f = lambda p: np.hypot(x-p[0], y-p[1]) - r; p0=[x.mean(), y.mean()]
    return least_squares(f, p0, loss="soft_l1", f_scale=0.005).x
for cam, zlo in [("birdview",0.54),("agentview",0.555)]:
    d = np.load(f"{cam}_cloud.npz"); Pw=d["pw"]; z=Pw[...,2]
    m = np.isfinite(z)&(Pw[...,0]>-0.30)&(Pw[...,0]<-0.12)&(Pw[...,1]>-0.03)&(Pw[...,1]<0.11)&(z>zlo)&(z<0.60)
    pts = Pw[m][:,:2]
    print(cam, "red mug rim free fit", fit(pts).round(4), "fixed r=0.0435:", fit(pts,0.0435).round(4))
    m = np.isfinite(z)&(Pw[...,0]>-0.16)&(Pw[...,0]<-0.04)&(Pw[...,1]>-0.25)&(Pw[...,1]<-0.10)&(z>zlo-0.02)&(z<0.60)
    pts = Pw[m][:,:2]
    print(cam, "white mug rim free fit", fit(pts).round(4))
