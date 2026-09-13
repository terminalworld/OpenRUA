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
    # stove plate region
    m = (pts[:,0]>-0.20)&(pts[:,0]<0.12)&(pts[:,1]>0.05)&(pts[:,1]<0.36)&(pts[:,2]>0.905)&(pts[:,2]<1.1)
    p = pts[m]
    rim = p[p[:,2]>0.96]
    rim = rim[rim[:,1]<0.30]
    if rim.size:
        print(f"{name}: pan rim z max {rim[:,2].max():.3f}; rim x[{rim[:,0].min():.3f},{rim[:,0].max():.3f}] y[{rim[:,1].min():.3f},{rim[:,1].max():.3f}] center ({(rim[:,0].min()+rim[:,0].max())/2:.3f},{(rim[:,1].min()+rim[:,1].max())/2:.3f})")
    # anything on table between pan and knob? knob top
    k = pts[(np.hypot(pts[:,0]+0.197, pts[:,1]-0.201)<0.045)&(pts[:,2]>0.905)&(pts[:,2]<1.0)]
    print(f"   knob z max {k[:,2].max():.3f}")
