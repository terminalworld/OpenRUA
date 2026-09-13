import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("ud"); got=[]
n.create_subscription(String,"/robot_description",lambda m:got.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open("robot.urdf","w").write(got[0]); print(len(got[0]))
