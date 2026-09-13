"""Descend to TCP (x,y,z) in two steps, close the gripper, report the gap, lift to zlift."""
import sys, numpy as np
from arm import Arm, down_quat
x, y, z, zlift = map(float, sys.argv[1:5])
yaw = np.deg2rad(float(sys.argv[5])) if len(sys.argv) > 5 else 0.0
a = Arm("grasp")
qd = down_quat(yaw)
print("start TCP", a.tcp_world()[0].round(4), "gap", round(a.finger_gap(), 4), flush=True)
a.move_tcp_world([x, y, z + 0.05], qd, seconds=2.5)
a.move_tcp_world([x, y, z], qd, seconds=2.0)
gap = a.gripper(0.0)
print("GAP after close:", round(gap, 4), flush=True)
a.move_tcp_world([x, y, zlift], qd, seconds=2.5)
print("gap after lift:", round(a.finger_gap(), 4), flush=True)
