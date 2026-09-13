# Pick the alphabet soup: open, pre-grasp above, straight descent, close, lift.
from rob import *
r = Robot()
SX, SY = -0.244, -0.173
r.gripper(0.04)
assert r.move_tcp(SX, SY, 0.62, yaw=0.0, secs=4), "pre-grasp failed"
assert r.move_tcp_line(SX, SY, 0.455, yaw=0.0, secs=3, n=4), "descent failed"
f = r.gripper(0.0)
gap = f[0] - f[1]
log(f"finger gap after close = {gap:.4f} (closed_m=0 -> nothing held)")
assert r.move_tcp_line(SX, SY, 0.72, yaw=0.0, secs=3, n=3), "lift failed"
log("fingers after lift", r.fingers())
log("PHASE1 DONE")
