import numpy as np, sys, json
sys.path.insert(0,'.')
from rob import *
r=Robot('pinch')
prev=np.array(r.arm_q())
p=np.array([-0.149,-0.314,0.947])          # ring point, -x side
n_in=np.array([-0.31,0.95,0.0])            # into the mouth
yhat=np.array([0.95,0.31,0.0])             # closing axis (radial)
b=np.deg2rad(45)
d=np.cos(b)*n_in+np.sin(b)*np.array([0,0,-1.0])
qh=hand_quat(d, -np.cross(yhat,d))
tcp=p+0.018*n_in
def go(pt,secs=2.5):
    global prev
    s=best_ik(r,np.array(pt),qh,seed=prev,prefer=prev,avoid=lambda L: not clear_of_door(L))
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(r.wrench()[:3],2))
r.gripper(0.08)
go(tcp-0.08*d,4.0); go(tcp-0.05*d); go(tcp-0.03*d,2.0); go(tcp,2.0)
r.gripper(0.0)
print('fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
go(tcp+np.array([0,0,0.05]),2.0)
go(tcp+np.array([0,0,0.12]),2.5)
print('fingers',np.round(r.fingers(),4),'wrench',np.round(r.wrench()[:3],2))
json.dump(dict(q=list(map(float,r.arm_q())),d=d.tolist(),yhat=yhat.tolist()),open('pinch_state.json','w'))
