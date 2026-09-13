import sys; sys.argv=["x","state"]
from arm import *
a = Arm()
p, q, tcp, j = a.state()
# lift straight up 12 cm keeping current orientation
sol = plan_only(a, tcp + [0, 0, 0.12], q)
if sol: a.traj(sol, 2.5)
a.state()
print("--- IK for hover/descend from here:")
plan_only(a, (-0.200, -0.126, 0.66))
plan_only(a, (-0.200, -0.126, 0.49))
rclpy.shutdown()
