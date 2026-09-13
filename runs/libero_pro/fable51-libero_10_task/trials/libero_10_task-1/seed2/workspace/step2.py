from rob import *
r = Robot()
q = down_quat(0)
print("descend to grasp")
r.move_tcp([-0.111, -0.157, 0.465], q, 2.5)
print("close")
f = r.gripper(0.0)
print("fingers after close", f)
