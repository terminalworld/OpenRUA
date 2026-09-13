from robot import *
r = Robot()
q, d = r.joints(); print("joints", np.round(q,4)); print("fingers", r.fingers())
pos, quat = r.fk_hand(); print("hand world pos", np.round(pos,4), "quat", np.round(quat,4))
R = quat_R(*quat); print("hand x axis", np.round(R[:,0],3), "y", np.round(R[:,1],3), "z", np.round(R[:,2],3))
print("tcp", np.round(pos + TCP*R[:,2],4))
