from robot import *
r = Robot()
q = r.arm_q(); log("q", q.round(3))
log("fingers", r.fingers())
fk = r.fk_world(q); log("tcp world", fk)
# check IK round trip for a pre-grasp pose above ketchup
quat = Robot.quat_topdown(90)
log("quat", quat)
sol = r.ik_world([-0.22, -0.125, 0.62], quat)
log("ik sol", None if sol is None else sol.round(3))
if sol is not None: log("fk of sol", r.fk_world(sol))
