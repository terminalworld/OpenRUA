from arm import *
r = Robot()
Q = topdown_quat(0.0)
cx, cy = 0.09, -0.188
print("open gripper"); r.gripper(0.04)
print("pregrasp"); assert r.move_world([cx, cy, 0.56], Q, 4.0)
print("descend"); assert r.move_world([cx, cy, 0.435], Q, 2.5)
print("close"); r.gripper(0.0)
f = r.fingers(); print("fingers after close", f)
print("lift"); assert r.move_world([cx, cy, 0.65], Q, 2.5)
print("fingers after lift", r.fingers())
