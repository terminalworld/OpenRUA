import math, numpy as np
from robot import Robot, yaw_down_quat, quat_R
r = Robot("iktest")
target = np.array([-0.092, 0.023, 1.20])
for deg in [0, -20, -40.5, -60, -90, 45, 90]:
    q = yaw_down_quat(math.radians(deg))
    sol = r.ik_solve(target, q)
    if sol:
        hp,hq = r.fk_pose(sol)
        R=quat_R(*hq)
        print(deg, "req q", np.round(q,3), "-> FK q", np.round(hq,3), "handX", R[:,0].round(3), "tcp", r.tcp_from_hand(hp,hq).round(3), "j7", round(sol[6],3))
