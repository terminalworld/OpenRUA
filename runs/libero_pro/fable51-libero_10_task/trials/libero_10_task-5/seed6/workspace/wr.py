import rclpy
from geometry_msgs.msg import WrenchStamped
class Wrench:
    def __init__(self,r):
        self.r=r; self.m=None; r.node.create_subscription(WrenchStamped,"/franka_robot_state_broadcaster/external_wrench",self._cb,1)
    def _cb(self,m): self.m=m
    def get(self):
        self.m=None
        while self.m is None: rclpy.spin_once(self.r.node,timeout_sec=0.2)
        f=self.m.wrench.force; return (round(f.x,2),round(f.y,2),round(f.z,2))
