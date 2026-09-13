from arm import *
r = Robot()
Q = topdown_quat(0.0)
print("clear"); assert r.move_world([-0.15, -0.05, 0.75], Q, 4.0)
