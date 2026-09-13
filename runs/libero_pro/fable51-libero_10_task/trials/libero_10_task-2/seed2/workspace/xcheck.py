import numpy as np
from rob import quat_R
fx=579.4112549695428; cx=320; cy=240
cams = {"front": ("snaps/front_depth.npy", np.array([1.0,0.0,1.48]), quat_R(0.5608,0.5608,-0.4306,-0.4306)),
        "side": ("snaps/side_depth.npy", np.array([-0.0565,1.2761,1.488]), quat_R(0.0099,0.8064,-0.5912,-0.0069)),
        "bird": ("snaps/birdview_depth.npy", np.array([-0.2,0,3.0]), quat_R(0.7071,0.7071,0,0))}
for name,(f,t,R) in cams.items():
    d = np.load(f)
    P = np.zeros((480,640,3))
    for v in range(480):
        Z = d[v,:]
        u = np.arange(640)
        pc = np.stack([(u-cx)*Z/fx, (v-cy)*Z/fx, Z], 1)
        P[v] = pc@R.T + t
    np.save(f"snaps/P_{name}.npy", P)
    # table height
    tab = np.median(P[...,2][(np.abs(P[...,0]+0.3)<0.1)&(np.abs(P[...,1]-0.4)<0.1)])
    # pan region: within 0.14 of (-0.064,-0.266) in xy
    dpan = np.hypot(P[...,0]+0.064, P[...,1]+0.266)
    pan = P[...,2][(dpan<0.14)]
    # knob region within 0.045 of (-0.197,0.201)
    dk = np.hypot(P[...,0]+0.197, P[...,1]-0.201); knob = P[...,2][dk<0.045]
    # stove region within 0.08 of (-0.044,0.203) 
    ds = np.hypot(P[...,0]+0.044, P[...,1]-0.203); stove = P[...,2][ds<0.08]
    # moka
    dm = np.hypot(P[...,0]-0.03, P[...,1]-0.01); moka = P[...,2][dm<0.05]
    # handle: x in [-0.09,-0.04], y in [-0.12,-0.03]
    hm = (P[...,0]>-0.09)&(P[...,0]<-0.04)&(P[...,1]>-0.12)&(P[...,1]<-0.03)
    handle = P[...,2][hm]
    f2 = lambda a: f"n={a.size} max={a.max():.3f} p95={np.percentile(a,95):.3f}" if a.size else "none"
    print(f"{name}: table={tab:.3f} pan[{f2(pan)}] knob[{f2(knob)}] stove[{f2(stove)}] moka[{f2(moka)}] handle[{f2(handle)}]")
