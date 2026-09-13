import math, numpy as np
from robot import Robot, yaw_down_quat, quat_R
r = Robot("step2")
def hand_yaw_quat(deg): return yaw_down_quat(math.radians(deg - 45.0))
q = hand_yaw_quat(-40.5)
tcp = r.move_tcp([-0.092, 0.023, 1.20], q, 3.0)
hp,hq = r.fk_pose(); R=quat_R(*hq); print("handX", R[:,0].round(3), "handY(close dir)", R[:,1].round(3))
