import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("u")
got=[]
n.create_subscription(String,"/robot_description",got.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
import time; t=time.time()
while not got and time.time()-t<10: rclpy.spin_once(n,timeout_sec=0.2)
s=got[0].data; open("/workspace/robot.urdf","w").write(s); print(len(s))
