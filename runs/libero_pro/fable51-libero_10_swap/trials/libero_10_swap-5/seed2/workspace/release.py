import math, numpy as np, sys
from robot import Robot, yaw_down_quat, quat_R
px, py, yaw, zrel = map(float, sys.argv[1:5])
r = Robot("release")
q = yaw_down_quat(math.radians(yaw - 45.0))
def goto(x,y,z, secs=3.0):
    for attempt in range(3):
        tcp = r.move_tcp([x, y, z], q, secs)
        if tcp is not None and np.linalg.norm(tcp-[x,y,z]) < 0.005: return True
    return False
def gap():
    js = r.joints(); return js['panda_finger_joint1'] - js['panda_finger_joint2']
goto(px, py, 1.15, 2.5); print("gap", gap())
goto(px, py, zrel, 2.5); print("gap", gap())
r.gripper(0.04)
goto(px, py, 1.30, 3.0)
