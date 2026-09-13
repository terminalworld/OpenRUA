import numpy as np, subprocess
d=np.load("/workspace/rv_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
qx,qy,qz,qw=0.6531,0.6531,-0.2710,-0.2710
t=np.array([0.3400,0.0,1.3120])
x,y,z,w=qx,qy,qz,qw
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
H,W=d.shape; uu,vv=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1)
Wd=P@R.T+t; np.save("/workspace/rv_world.npy",Wd)
wx,wy,wz=Wd[...,0],Wd[...,1],Wd[...,2]
# the drawer's center column in image: u ~ 500 ; scan v from 140 to 400
for u in (470,500,530):
  print("column u=",u)
  for v in range(140,400,6):
    print(f"  v={v} x={wx[v,u]:.3f} y={wy[v,u]:.3f} z={wz[v,u]:.3f}")
