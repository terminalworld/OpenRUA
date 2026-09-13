from rob import *
r = R()
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)
j = r.ik(-0.197, 0.201, 1.05, *Q_DOWN_Y)
print("target", np.round(j,3))
for i in range(3):
    code = r.move(j, 3.0)
    if code == 0: break
pos,q = r.fk(); Rm = quat_R(*q)
print("hand q", np.round(q,4), "finger axis", Rm[:,1].round(3), "tcp", r.tcp()[0].round(4))
print("joints", np.round(r.joints(),3))
