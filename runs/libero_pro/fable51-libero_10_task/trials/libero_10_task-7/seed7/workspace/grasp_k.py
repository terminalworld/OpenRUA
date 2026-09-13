from robot import *
r = Robot()
KX, KY = -0.215, -0.125
Q = Robot.quat_topdown(90)
chain = r.plan_descent([KX, KY, 0.475], Q, 0.64, n_seeds=40, margin=0.10)
assert chain is not None
log("top", chain[0].round(3), "grasp", chain[-1].round(3))
assert r.move_q(chain[0], 3.0), "move to chain top failed"
log("tcp top", r.fk_world()[0].round(4))
ok = r.run_chain(chain, speed=0.04)
log("descent ok", ok, "tcp", r.fk_world()[0].round(4))
f = r.gripper(GRIP["closed_m"])
log("fingers after close", f)
