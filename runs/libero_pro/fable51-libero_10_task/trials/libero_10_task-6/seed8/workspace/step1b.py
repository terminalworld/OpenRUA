from robot import *
import robot
r = Robot()
r.spin(1.0)
t = r.tfbuf.lookup_transform("world", "panda_hand", rclpy.time.Time())
print("TF world->hand", t.transform.translation)
robot.BASE_IN_WORLD = np.zeros(3)
print("FK (no offset)", r.hand_pose())
print("IK world-coords:", r.ik([-0.185, 0.0, 0.90], down_quat(90)))
print("IK base-coords :", r.ik([-0.185+0.51, 0.0, 0.90-0.42], down_quat(90)))
