from rob import *
r = Robot()
q = down_quat(0)
print("pre-grasp above alphabet soup")
r.move_tcp([-0.115, -0.153, 0.60], q, 3.0)
print("fingers", r.fingers())
