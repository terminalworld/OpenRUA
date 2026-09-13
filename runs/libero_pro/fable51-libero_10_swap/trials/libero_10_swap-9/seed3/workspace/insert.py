import numpy as np, rclpy
from rob import Robot
from ins_geom import pose
r=Robot()
q33,info=pose(33,-0.17); H=info['H']
r.cart_path([(H+np.array([0,-0.05,0]),q33),(H,q33)], min_step_t=1.5)
print("hand",np.round(r.fk()[0],3),"fingers",r.fingers())
rclpy.shutdown()
