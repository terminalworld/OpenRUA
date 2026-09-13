import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n = rclpy.create_node("u"); got=[]
n.create_subscription(String, "/robot_description", got.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
while not got: rclpy.spin_once(n, timeout_sec=0.5)
open("robot.urdf","w").write(got[0].data); print(len(got[0].data))
