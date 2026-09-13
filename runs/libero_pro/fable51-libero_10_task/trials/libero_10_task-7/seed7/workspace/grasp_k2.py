from robot import *
r = Robot()
KX, KY = -0.150, -0.133
Q = Robot.quat_topdown(0)          # fingers open along world y (across the lying bottle)
if r.fingers()[0] < 0.035: r.gripper(GRIP["open_m"])
chain = r.plan_descent([KX, KY, 0.44], Q, 0.60, n_seeds=40, margin=0.10)
assert chain is not None
log("top", chain[0].round(3), "grasp", chain[-1].round(3))
assert r.move_q(chain[0], 4.0), "move to chain top failed"
log("tcp top", r.fk_world()[0].round(4))
ok = r.run_chain(chain, speed=0.03)
log("descent ok", ok, "tcp", r.fk_world()[0].round(4))
f = r.gripper(GRIP["closed_m"])
log("fingers after close", f)
