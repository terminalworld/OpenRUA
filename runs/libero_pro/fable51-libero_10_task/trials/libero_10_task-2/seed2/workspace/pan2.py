import numpy as np
from rob import quat_R
np.set_printoptions(linewidth=250, precision=3, suppress=True)
fx=579.4112549695428; cx=320; cy=240
cams = {"front": ("snaps/frontview_depth2.npy", np.array([1.0,0.0,1.48]), quat_R(0.5608,0.5608,-0.4306,-0.4306)),
        "side": ("snaps/sideview_depth2.npy", np.array([-0.0565,1.2761,1.488]), quat_R(0.0099,0.8064,-0.5912,-0.0069))}
allpts = []
for name,(f,t,R) in cams.items():
    d = np.load(f)
    P = np.zeros((480,640,3))
    for v in range(480):
        Z = d[v,:]; u = np.arange(640)
        P[v] = np.stack([(u-cx)*Z/fx, (v-cy)*Z/fx, Z], 1)@R.T + t
    np.save(f"snaps/P2_{name}.npy", P)
    pts = P.reshape(-1,3)
    # pan region: above table, within a box
    m = (pts[:,2]>0.905)&(pts[:,2]<1.0)&(pts[:,0]>-0.25)&(pts[:,0]<0.1)&(pts[:,1]>-0.45)&(pts[:,1]<0.0)
    p = pts[m]; allpts.append(p)
    print(name, "pan pts", p.shape, "z max", p[:,2].max().round(3))
    # rim: points with z>0.935 and y<-0.14
    rim = p[(p[:,2]>0.935)&(p[:,1]<-0.14)]
    print("  rim x range", rim[:,0].min().round(3), rim[:,0].max().round(3), "y range", rim[:,1].min().round(3), rim[:,1].max().round(3), "center", ((rim[:,0].min()+rim[:,0].max())/2).round(3), ((rim[:,1].min()+rim[:,1].max())/2).round(3))
    # handle: y > -0.135
    h = p[(p[:,1]>-0.13)&(p[:,2]>0.92)]
    if h.size:
        print("  handle x range", h[:,0].min().round(3), h[:,0].max().round(3), "y range", h[:,1].min().round(3), h[:,1].max().round(3), "z range", h[:,2].min().round(3), h[:,2].max().round(3))
        for y0 in np.arange(-0.12,-0.02,0.02):
            s = h[(h[:,1]>=y0)&(h[:,1]<y0+0.02)]
            if s.size: print(f"   y[{y0:.2f},{y0+0.02:.2f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}] n={len(s)}")
