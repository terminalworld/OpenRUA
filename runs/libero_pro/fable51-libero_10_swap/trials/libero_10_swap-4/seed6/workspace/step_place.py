"""Carry to (x,y) at zcarry, descend to zplace, open, lift back."""
import sys, numpy as np
from arm import Arm, down_quat
x, y, zcarry, zplace = map(float, sys.argv[1:5])
yaw = np.deg2rad(float(sys.argv[5])) if len(sys.argv) > 5 else 0.0
a = Arm("place")
qd = down_quat(yaw)
print("start TCP", a.tcp_world()[0].round(4), "gap", round(a.finger_gap(), 4), flush=True)
a.move_tcp_world([x, y, zcarry], qd, seconds=4.0)
print("gap after carry:", round(a.finger_gap(), 4), flush=True)
a.move_tcp_world([x, y, zplace + 0.04], qd, seconds=2.5)
a.move_tcp_world([x, y, zplace], qd, seconds=2.0)
a.gripper(0.04)
a.move_tcp_world([x, y, zcarry], qd, seconds=2.5)
print("done; TCP", a.tcp_world()[0].round(4), flush=True)
