import numpy as np
from rob import quat_R
fx=579.4112549695428; cx=320; cy=240
cams = {"front": ("snaps/frontview_depth3.npy", np.array([1.0,0.0,1.48]), quat_R(0.5608,0.5608,-0.4306,-0.4306)),
        "side": ("snaps/sideview_depth3.npy", np.array([-0.0565,1.2761,1.488]), quat_R(0.0099,0.8064,-0.5912,-0.0069)),
        "bird": ("snaps/birdview_depth3.npy", np.array([-0.2,0,3.0]), quat_R(0.7071,0.7071,0,0))}
for name,(f,t,R) in cams.items():
    d = np.load(f); P = np.zeros((480,640,3))
    for v in range(480):
        Z = d[v,:]; u = np.arange(640)
        P[v] = np.stack([(u-cx)*Z/fx, (v-cy)*Z/fx, Z], 1)@R.T + t
    pts = P.reshape(-1,3)
    box = (pts[:,0]>-0.18)&(pts[:,0]<0.10)&(pts[:,1]>0.09)&(pts[:,1]<0.45)&(pts[:,2]>0.905)&(pts[:,2]<1.02)
    p = pts[box]
    hist = np.histogram(p[:,2], bins=np.arange(0.90,1.02,0.01))
    print(name, "z hist", list(zip(hist[1][:-1].round(2), hist[0])))
    body = p[(p[:,2]>0.95)&(p[:,1]<0.31)]  # pan body above the stove plate level
    print(f"   pan body(z>0.95,y<0.31): x[{body[:,0].min():.3f},{body[:,0].max():.3f}] y[{body[:,1].min():.3f},{body[:,1].max():.3f}] zmax {body[:,2].max():.3f} center ({(body[:,0].min()+body[:,0].max())/2:.3f},{(body[:,1].min()+body[:,1].max())/2:.3f})")
    handle = p[(p[:,1]>0.31)&(p[:,2]>0.94)]
    if handle.size: print(f"   handle: y[{handle[:,1].min():.3f},{handle[:,1].max():.3f}] z[{handle[:,2].min():.3f},{handle[:,2].max():.3f}]")
