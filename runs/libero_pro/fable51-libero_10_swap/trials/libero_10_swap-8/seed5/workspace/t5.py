from lib import *
r = Robot("t5")
q = r.arm_q(); print("q", np.round(q,3))
pos, quat = r.fk_world()
R = Rot.from_quat(quat).as_matrix()
print("hand", np.round(pos,4)); print("hand x axis", np.round(R[:,0],3), "y axis", np.round(R[:,1],3), "z axis", np.round(R[:,2],3))
Rt = Rot.from_quat(down_quat(90)).as_matrix()
print("target x", np.round(Rt[:,0],3), "y", np.round(Rt[:,1],3), "z", np.round(Rt[:,2],3))
print("angle diff deg", np.degrees((Rot.from_quat(quat).inv()*Rot.from_quat(down_quat(90))).magnitude()))
