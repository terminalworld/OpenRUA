import numpy as np
d=np.load("/workspace/agentview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
qx,qy,qz,qw=0.6380,0.6380,-0.3048,-0.3048
t=np.array([0.6586,0.0,1.6104])
x,y,z,w=qx,qy,qz,qw
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
H,W=d.shape; uu,vv=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1)
Wd=P@R.T+t
np.save("/workspace/agent_world.npy",Wd)
wx,wy,wz=Wd[...,0],Wd[...,1],Wd[...,2]
# column through the open drawer's +x side wall, u=420 (drawer box), v from 230 to 360
for u in (400,430):
  print("column u=",u)
  for v in range(230,370,4):
    print(f"  v={v} x={wx[v,u]:.3f} y={wy[v,u]:.3f} z={wz[v,u]:.3f}")
