from rob import *
import rclpy
from geometry_msgs.msg import WrenchStamped
r = Robot()
q = r.arm_q(); print("q now", np.round(q,3))
tgt = np.array([0.21, 0.93, -0.11, -1.54, 1.74, 1.6, -0.11])
print("target", tgt); print("diff", np.round(tgt - np.array(q),3))
print("fk", np.round(r.fk_hand()[2],4), "fingers", r.fingers())
got = []
sub = r.node.create_subscription(WrenchStamped, '/franka_robot_state_broadcaster/external_wrench', lambda m: got.append(m), 1)
import time; t=time.time()
while not got and time.time()-t < 5: rclpy.spin_once(r.node, timeout_sec=0.2)
if got: w = got[0].wrench; print("wrench F", round(w.force.x,2), round(w.force.y,2), round(w.force.z,2), "T", round(w.torque.x,2), round(w.torque.y,2), round(w.torque.z,2))
