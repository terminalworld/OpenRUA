from robot import *
import robot
r = Robot()
q,_ = r.joints()
robot.BASE_T = np.zeros(3); r_pos, r_quat = r.fk_hand(q)   # true world pose (no offset)
print("hand world", np.round(r_pos,4))
for label, p in [("world coords", r_pos), ("base-relative coords", r_pos - np.array([-0.51,0,0.42]))]:
    robot.BASE_T = np.zeros(3)
    sol = r.ik_hand(p, r_quat, seed=q)
    print(label, "->", None if sol is None else np.round(sol,3), "| current", np.round(q,3))
