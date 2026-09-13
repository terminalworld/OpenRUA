from arm import *
r = Robot()
print("joints", r.arm_q()); print("fingers", r.fingers())
p, q, c = r.fk_world(); print("hand world", p, "quat", q, "code", c)
print("hand R:\n", np.round(quat_to_R(*q), 3))
print("topdown quat", topdown_quat(0.0), "R:\n", np.round(quat_to_R(*topdown_quat(0.0)),3))
