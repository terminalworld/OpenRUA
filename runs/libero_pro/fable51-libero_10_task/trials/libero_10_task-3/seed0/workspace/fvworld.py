import numpy as np
d=np.load("/workspace/fv_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
qx,qy,qz,qw=0.5608,0.5608,-0.4306,-0.4306
t=np.array([1.0,0.0,1.48])
x,y,z,w=qx,qy,qz,qw
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
H,W=d.shape; uu,vv=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1)
Wd=P@R.T+t; np.save("/workspace/fv_world.npy",Wd)
wx,wy,wz=Wd[...,0],Wd[...,1],Wd[...,2]
# cabinet front face (facing -y) pixels: y between 0.20 and 0.26, x in [-0.2,0.12], z>0.9
m=(wy>0.195)&(wy<0.26)&(wx>-0.2)&(wx<0.12)&(wz>0.905)
print("front-face-ish pixels:",m.sum())
for lo in np.arange(0.90,1.14,0.01):
    mm=m&(wz>=lo)&(wz<lo+0.01)
    if mm.sum(): print(f"z[{lo:.3f}] n={mm.sum():4d} y[{wy[mm].min():.3f},{wy[mm].max():.3f}] x[{wx[mm].min():.3f},{wx[mm].max():.3f}]")
print("---- front face only: y in [0.225,0.245], x in [-0.10,0.09]")
m=(wy>0.225)&(wy<0.245)&(wx>-0.10)&(wx<0.09)&(wz>0.905)
for lo in np.arange(0.90,1.14,0.005):
    mm=m&(wz>=lo)&(wz<lo+0.005)
    print(f"z[{lo:.3f}] n={mm.sum():4d}" + (f" y[{wy[mm].min():.3f},{wy[mm].max():.3f}] x[{wx[mm].min():.3f},{wx[mm].max():.3f}]" if mm.sum() else ""))
print("---- handles: y in [0.195,0.225]")
m=(wy>0.195)&(wy<0.225)&(wx>-0.10)&(wx<0.09)&(wz>0.905)
for lo in np.arange(0.90,1.14,0.01):
    mm=m&(wz>=lo)&(wz<lo+0.01)
    if mm.sum(): print(f"z[{lo:.3f}] n={mm.sum():4d} y[{wy[mm].min():.3f},{wy[mm].max():.3f}] x[{wx[mm].min():.3f},{wx[mm].max():.3f}]")
