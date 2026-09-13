import math, numpy as np, sys
from robot import Robot, yaw_down_quat, quat_R
bx, by, ztop, yaw = map(float, sys.argv[1:5])
r = Robot("grasp")
q = yaw_down_quat(math.radians(yaw - 45.0))
r.gripper(0.04)
def goto(z, secs=3.0):
    for attempt in range(3):
        tcp = r.move_tcp([bx, by, z], q, secs)
        if tcp is not None and np.linalg.norm(tcp-[bx,by,z]) < 0.005: return True
    return False
goto(ztop + 0.12)
goto(ztop + 0.05, 2.0)
goto(ztop - 0.03, 2.0)
js = r.gripper(0.0)
print("gap after close", js['panda_finger_joint1'] - js['panda_finger_joint2'])
goto(ztop + 0.03, 2.0)
js = r.joints(); print("gap after small lift", js['panda_finger_joint1'] - js['panda_finger_joint2'])
goto(ztop + 0.25, 3.0)
js = r.joints(); print("gap after lift", js['panda_finger_joint1'] - js['panda_finger_joint2'])
