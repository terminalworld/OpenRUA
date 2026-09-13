import rclpy, re
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("u"); got=[]
n.create_subscription(String,"/robot_description",got.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
while not got: rclpy.spin_once(n,timeout_sec=0.2)
s=got[0].data; open("snaps/robot.urdf","w").write(s)
i=s.find('name="panda_hand"'); print(s[i-20:i+2500])
