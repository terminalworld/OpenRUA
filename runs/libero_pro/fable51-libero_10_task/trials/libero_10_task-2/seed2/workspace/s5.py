from rob import *
r = R()
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)
print("goto knob_pre", r.goto(-0.197, 0.201, 1.05, Q_DOWN_Y, 3.0))
pos,q = r.fk(); Rm = quat_R(*q)
print("hand q", np.round(q,4), "finger axis", Rm[:,1].round(3))
print("joints", np.round(r.joints(),3))
