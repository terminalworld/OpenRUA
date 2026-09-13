import numpy as np, rclpy
from arm import Arm
from geometry_msgs.msg import WrenchStamped
a = Arm()
w = {}
a.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: w.__setitem__("m", m), 1)
def wrench():
    w.pop("m", None)
    while "m" not in w: a.spin(0.2)
    f = w["m"].wrench.force; return np.round([f.x, f.y, f.z], 2)
Q = (0.7071068, 0.7071068, 0.0, 0.0)
TX, TY = -0.055, -0.254
print("wrench before", wrench())
q1 = a.ik([TX, TY, 1.065], Q)
q2 = a.ik([TX, TY, 1.050], Q, seed=q1)
for i in range(3):
    code, err = a.move([q1, q2], 3.0)
    if err < 0.005: break
print("tcp", np.round(a.tcp()[0],4), "fingers", a.fingers(), "wrench", wrench())
a.gripper(0.04)
q3 = a.ik([TX, TY, 1.15], Q, seed=q2)
for i in range(3):
    code, err = a.move(q3, 3.0)
    if err < 0.005: break
print("tcp", np.round(a.tcp()[0],4), "fingers", a.fingers())
