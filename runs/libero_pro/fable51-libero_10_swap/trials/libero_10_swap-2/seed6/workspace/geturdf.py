import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n = rclpy.create_node("geturdf")
got = {}
n.create_subscription(String, "/robot_description", lambda m: got.setdefault("m", m), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
while "m" not in got: rclpy.spin_once(n, timeout_sec=0.5)
open("/workspace/robot.urdf","w").write(got["m"].data)
print(len(got["m"].data))
