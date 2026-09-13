import numpy as np, rclpy, time, subprocess
from rob import Robot, quat_from_axes
r=Robot(); p,qcur=r.fk(); print("hand",np.round(p,3))
st=np.load('pinch_state.npy'); q0=st[3:7]
# lift straight up 0.2 keeping orientation
rc=r.cart_path([(p+[0,0,0.1],q0),(p+[0,0,0.2],q0)],seconds_per_m=6.0,min_step_t=1.0,max_jump=0.8); print("lift",rc is not None)
r.gripper(0.04); time.sleep(0.5); print("fingers",r.fingers())
# pose: hand pointing +x, fingers closing along y, at (-0.15,0.2,1.15)
q=quat_from_axes(np.array([1.0,0,0]),np.array([0,1.0,0]))
H=np.array([-0.15,0.20,1.15])
rc=r.plan_go(H,q,seed=r.arm_q()); print("plan_go",rc is not None)
p,_=r.fk(); print("hand now",np.round(p,3),"q",np.round(r.arm_q(),2))
