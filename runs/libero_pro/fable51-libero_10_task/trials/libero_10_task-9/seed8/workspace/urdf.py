import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); node=rclpy.create_node("u")
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String,"/robot_description",got.append,qos)
while not got: rclpy.spin_once(node,timeout_sec=0.5)
open("robot.urdf","w").write(got[0].data); print(len(got[0].data))
rclpy.shutdown()
