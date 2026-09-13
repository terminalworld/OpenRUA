import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node("gd")
qos = QoSProfile(depth=1); qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
got = []
n.create_subscription(String, "/robot_description", got.append, qos)
while not got: rclpy.spin_once(n, timeout_sec=0.5)
open("/workspace/robot.urdf","w").write(got[0].data); print(len(got[0].data))
