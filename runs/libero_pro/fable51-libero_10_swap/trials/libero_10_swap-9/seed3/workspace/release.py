import numpy as np, rclpy
from rob import Robot
from ins_geom import pose
r=Robot()
q33,info=pose(33,-0.17)
r.gripper(0.04)
p=r.fk()[0]
r.cart_path([(p+np.array([0,-0.06,0.0]),q33),(p+np.array([0,-0.12,0.02]),q33)], min_step_t=1.5)
print("hand",np.round(r.fk()[0],3),"fingers",r.fingers())
rclpy.shutdown()
