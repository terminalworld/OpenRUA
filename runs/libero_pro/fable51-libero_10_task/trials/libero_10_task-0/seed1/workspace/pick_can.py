from arm import *
r = Robot()
Q = topdown_quat(0.0)
cx, cy = -0.079, 0.042
print("fingers", r.fingers())
print("pregrasp"); assert r.move_world([cx, cy, 0.63], Q, 4.0)
print("descend"); assert r.move_world([cx, cy, 0.475], Q, 2.5)
print("close"); r.gripper(0.0)
print("fingers after close", r.fingers())
print("lift"); assert r.move_world([cx, cy, 0.70], Q, 2.5)
print("fingers after lift", r.fingers())
