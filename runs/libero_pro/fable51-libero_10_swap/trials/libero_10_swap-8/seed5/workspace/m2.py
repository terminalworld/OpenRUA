from lib import *
r = Robot("m2")
q = r.ik_world((-0.206,-0.195,1.25), down_quat(90))
print("ik", np.round(q,3))
r.move_q(q, 4)
print("q now", np.round(r.arm_q(),3))
pos, quat = r.fk_world(); print("hand", np.round(pos,4), "euler", np.round(Rot.from_quat(quat).as_euler("xyz", degrees=True),1))
