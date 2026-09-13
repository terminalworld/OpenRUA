from lib import *
r = Robot("t4")
q = r.arm_q(); print("q", np.round(q,3))
pos, quat = r.fk_world(); print("hand", np.round(pos,4), "quat", np.round(quat,4))
print("euler xyz deg", np.round(Rot.from_quat(quat).as_euler("xyz", degrees=True),1))
print("target quat", np.round(down_quat(90),4), np.round(Rot.from_quat(down_quat(90)).as_euler("xyz",degrees=True),1))
print("gap", r.finger_gap())
