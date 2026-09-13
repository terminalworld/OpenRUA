import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node("urdf_get")
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got = []
n.create_subscription(String, "/robot_description", lambda m: got.append(m.data), qos)
import time
for _ in range(50):
    rclpy.spin_once(n, timeout_sec=0.2)
    if got: break
open("robot.urdf","w").write(got[0]); print(len(got[0]), "bytes")
rclpy.shutdown()
