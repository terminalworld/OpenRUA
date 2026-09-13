from ctl import *
from geometry_msgs.msg import WrenchStamped
c = Ctl()
w={}
c.n.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", lambda m: w.update(f=(m.wrench.force.x,m.wrench.force.y,m.wrench.force.z)), 10)
def wr(): c.spin(0.4); return np.round(w.get("f",(0,0,0)),2)
print("wrench before", wr())
c.move([-0.2255,0.035,0.60], DOWN, secs=2.5); print("wrench", wr())
c.move([-0.2255,0.035,0.545], DOWN, secs=2.5); print("wrench", wr())
c.gripper(0.0)
print("wrench after close", wr())
