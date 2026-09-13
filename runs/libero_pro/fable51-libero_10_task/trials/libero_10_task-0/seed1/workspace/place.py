from arm import *
r = Robot()
Q = topdown_quat(0.0)
bx, by = 0.008, 0.253
print("fingers before", r.fingers())
print("to basket"); assert r.move_world([bx, by, 0.72], Q, 4.0)
print("fingers at basket", r.fingers())
print("release"); r.gripper(0.04)
print("retreat up"); assert r.move_world([bx, by, 0.80], Q, 2.0)
