import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('look2')
prev=np.array(r.arm_q())
x,y,z=map(float,sys.argv[1:4]); d=np.array(list(map(float,sys.argv[4:7])))
xd=np.array(list(map(float,sys.argv[7:10]))) if len(sys.argv)>9 else np.array([1,0,0])
qh=hand_quat(d,xd)
s=best_ik(r,np.array([x,y,z]),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
if s is None: print('no ik'); sys.exit(1)
r.move_joints(s,3.0); print('tcp',np.round(r.tcp()[0],4),'q',np.round(r.arm_q(),2))
