import numpy as np
fx=579.4112549695428; cx=320; cy=240
def R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
cams = {"frontview":((1.0,0,1.48),(0.5608,0.5608,-0.4306,-0.4306)),
        "sideview":((-0.0565,1.2761,1.4880),(0.0099,0.8064,-0.5912,-0.0069))}
def cloud(name):
    d=np.load(f"{name}_depth.npy"); t,q=cams[name]; Rm=R(*q)
    v,u=np.mgrid[0:480,0:640]
    pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fx,d],-1).reshape(-1,3)
    return pc@Rm.T+np.array(t)
for name in cams:
    P=cloud(name)
    for pot,(px,py) in {"A":(-0.2,-0.2),"B":(-0.037,0.24)}.items():
        m=(np.abs(P[:,0]-px)<0.08)&(np.abs(P[:,1]-py)<0.08)&(P[:,2]>0.905)
        Q=P[m]
        print(name,pot,"n",len(Q),"zmax",Q[:,2].max() if len(Q) else None)
        for z0 in np.arange(0.91,1.06,0.01):
            s=Q[np.abs(Q[:,2]-z0)<0.005]
            if len(s)<3: continue
            print(f"  z={z0:.2f} n={len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
