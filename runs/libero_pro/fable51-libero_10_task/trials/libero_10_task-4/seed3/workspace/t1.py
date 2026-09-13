import numpy as np
from rob import *
r = Robot("t1")
print("joints", np.round(r.joints(), 4))
print("fingers", r.fingers())
pos, q = r.hand_pose_world()
print("FK hand world", np.round(pos, 4), np.round(q, 4))
print("TCP world", np.round(r.tcp_world()[0], 4))
print("force", r.force())
# IK for the current hand pose: does it return ~current joints?
sol = r.ik_solve(pos, q)
print("IK(current hand pose) ->", None if sol is None else np.round(sol, 4))
# IK for a pure down orientation at the same spot
sol2 = r.ik_solve(pos, (1, 0, 0, 0))
print("IK(q=1000) ->", None if sol2 is None else np.round(sol2, 4))
sol3 = r.ik_solve(pos, yaw_down_quat(math.pi / 2))
print("IK(yaw90) ->", None if sol3 is None else np.round(sol3, 4))
