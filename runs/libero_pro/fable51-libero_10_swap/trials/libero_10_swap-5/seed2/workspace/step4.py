import math, numpy as np
from robot import Robot, yaw_down_quat, quat_R
r = Robot("step4")
def hand_yaw_quat(deg): return yaw_down_quat(math.radians(deg - 45.0))
q = hand_yaw_quat(-40.5)
for attempt in range(3):
    r.move_tcp([-0.092, 0.023, 1.30], q, 3.0)
    hp,hq = r.fk_pose()
    if abs(r.tcp_from_hand(hp,hq)[2]-1.30) < 0.005: break
js = r.joints(); print("finger gap", js['panda_finger_joint1'] - js['panda_finger_joint2'])
