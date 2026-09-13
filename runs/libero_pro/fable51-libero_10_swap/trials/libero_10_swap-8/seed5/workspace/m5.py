from lib import *
r = Robot("m5")
r.move_tcp((-0.068, 0.233, 1.25), down_quat(90), 5)
pos, quat = r.fk_world(); R = Rot.from_quat(quat).as_matrix()
print("hand y axis", np.round(R[:,1],3), "z axis", np.round(R[:,2],3), "gap", r.finger_gap())
