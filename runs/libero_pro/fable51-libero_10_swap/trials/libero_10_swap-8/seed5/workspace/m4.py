from lib import *
r = Robot("m4")
q = r.move_tcp((-0.206,-0.195,1.25), down_quat(90), seconds=4)
pos, quat = r.fk_world(); R = Rot.from_quat(quat).as_matrix()
print("hand y axis", np.round(R[:,1],3), "z axis", np.round(R[:,2],3))
print("q", np.round(r.arm_q(),3), "gap", r.finger_gap())
