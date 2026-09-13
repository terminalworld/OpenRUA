import numpy as np, sys, json
from ctl import *; from mp import *
r=Robot("s15")
def pose(tip,deg,yaw=0):
    th=np.radians(deg); zh=np.array([-np.sin(th),0,-np.cos(th)]); yh=np.array([0,-1,0.]); xh=np.cross(yh,zh)
    R=np.stack([xh,yh,zh],1); c,s=np.cos(yaw),np.sin(yaw); Rz=np.array([[c,-s,0],[s,c,0],[0,0,1]])
    R=Rz@R; return np.array(tip)-0.1034*R[:,2], R_quat(R)
seed=r.arm_q(); print(np.round(seed,2))
tip=json.loads(sys.argv[1])
for deg in [0,5,10,15,-5,-10,-15,-20]:
  for yaw in [0,0.2,-0.2,0.4,-0.4]:
    pos,q=pose(tip,deg,yaw); s=r.ik_solve(pos,q,seed,collide=True)
    if s is None: print(deg,yaw,"none"); continue
    print(deg,yaw,"delta %.2f"%np.abs(np.array(s)-np.array(seed)).max(), np.round(s,2))
