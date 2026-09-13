import math, numpy as np
from robot import Robot, yaw_down_quat, quat_R
r = Robot("step1")
print("joints", {k: round(v,3) for k,v in r.joints().items()})
r.gripper(0.04)   # open
theta = math.radians(-40.5)
q = yaw_down_quat(theta)
R = quat_R(*q); print("hand X", R[:,0].round(3), "hand Y", R[:,1].round(3), "hand Z", R[:,2].round(3))
target = np.array([-0.092, 0.023, 1.20])
sol = r.ik_solve(target, q)
print("sol", None if sol is None else np.round(sol,3))
if sol:
    hp,hq = r.fk_pose(sol); print("FK hand", hp.round(4), "tcp", r.tcp_from_hand(hp,hq).round(4), np.round(hq,4))
    r.move_joints(sol, 4.0)
    hp,hq = r.fk_pose(); print("now hand", hp.round(4), "tcp", r.tcp_from_hand(hp,hq).round(4), np.round(hq,4))
