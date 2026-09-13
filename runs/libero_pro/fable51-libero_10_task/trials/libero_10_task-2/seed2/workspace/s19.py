from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
X, Y = -0.0585, -0.108
r.grip(0.04)
qs=[]; prev=r.joints()
for z in (1.10, 1.05, 1.0, 0.97, 0.95, 0.93):
    jj = r.ik(X, Y, z, *QX, seed=prev); qs.append(jj); prev=jj
r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
r.grip(0.0)
j = r.joints(); r.move(j, 1.0)
print("fingers after settle", r.fingers())
qs=[]; prev=r.joints()
for z in (0.95, 0.98, 1.02, 1.07, 1.12):
    jj = r.ik(X, Y, z, *QX, seed=prev); qs.append(jj); prev=jj
r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
