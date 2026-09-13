import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); node = rclpy.create_node("urdf")
got = {}
node.create_subscription(String, "/robot_description", lambda m: got.setdefault("m", m), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
open("/workspace/robot.urdf","w").write(got["m"].data); print(len(got["m"].data))
