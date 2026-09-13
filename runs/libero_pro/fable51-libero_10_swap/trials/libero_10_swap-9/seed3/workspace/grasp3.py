import rclpy
from rob import Robot
r=Robot()
r.gripper(0.0)
print("fingers", r.fingers())
rclpy.shutdown()
