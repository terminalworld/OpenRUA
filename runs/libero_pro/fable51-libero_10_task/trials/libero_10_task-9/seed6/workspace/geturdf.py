import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); node = rclpy.create_node("urdf")
got = []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String, "/robot_description", got.append, qos)
import time; end=time.time()+10
while not got and time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
open("robot.urdf","w").write(got[0].data if got else "")
print(len(got[0].data) if got else "none")
