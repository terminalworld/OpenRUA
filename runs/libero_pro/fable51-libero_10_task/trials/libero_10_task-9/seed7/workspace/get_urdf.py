import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node("u")
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]
n.create_subscription(String,"/robot_description",lambda m:got.append(m.data),qos)
import time; end=time.time()+10
while not got and time.time()<end: rclpy.spin_once(n,timeout_sec=0.2)
open("robot.urdf","w").write(got[0] if got else "")
print(len(got[0]) if got else "none")
