import math, numpy as np
from robot import Robot, yaw_down_quat, quat_R
r = Robot("step3")
def hand_yaw_quat(deg): return yaw_down_quat(math.radians(deg - 45.0))
q = hand_yaw_quat(-40.5)
for z in [1.10, 1.045]:
    for attempt in range(3):
        tcp = r.move_tcp([-0.092, 0.023, z], q, 2.5)
        hp,hq = r.fk_pose()
        if abs(r.tcp_from_hand(hp,hq)[2]-z) < 0.005: break
js = r.gripper(0.0)
print("finger gap", js['panda_finger_joint1'] - js['panda_finger_joint2'])
