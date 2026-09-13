import rclpy
from geometry_msgs.msg import WrenchStamped
rclpy.init(); n=rclpy.create_node("w"); got=[]
n.create_subscription(WrenchStamped,"/franka_robot_state_broadcaster/external_wrench",got.append,1)
while not got: rclpy.spin_once(n,timeout_sec=0.2)
f=got[0].wrench.force; t=got[0].wrench.torque
print(f"force ({f.x:.2f},{f.y:.2f},{f.z:.2f}) torque ({t.x:.2f},{t.y:.2f},{t.z:.2f})")
rclpy.shutdown()
