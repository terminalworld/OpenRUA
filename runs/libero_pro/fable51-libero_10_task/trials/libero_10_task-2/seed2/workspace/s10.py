from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
j = r.ik(-0.063, -0.075, 1.15, *QX)
print("target", np.round(j,3))
for i in range(4):
    if r.move(j, 4.0) == 0: break
pos,q = r.fk(); Rm = quat_R(*q)
print("tcp", r.tcp()[0].round(4), "finger axis", Rm[:,1].round(3), "joints", np.round(r.joints(),3))
