import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n = rclpy.create_node("u")
got = {}
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String, "/robot_description", lambda m: got.setdefault("m", m), qos)
while "m" not in got: rclpy.spin_once(n, timeout_sec=0.5)
open("robot.urdf","w").write(got["m"].data); print(len(got["m"].data))
rclpy.shutdown()
