import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node("u")
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,"/robot_description",lambda m: got.append(m.data),qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open("snaps/robot.urdf","w").write(got[0]); print(len(got[0]))
