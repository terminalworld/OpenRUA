from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
XT, YT = -0.0425, 0.348
qs=[]; prev=r.joints()
for z in (0.99, 0.975, 0.965):
    jj = r.ik(XT, YT, z, *QX, seed=prev); qs.append(jj); prev=jj
r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
r.grip(0.04)
j = r.joints(); r.move(j, 1.0)
print("fingers", r.fingers())
qs=[]; prev=r.joints()
for z in (1.0, 1.05, 1.12):
    jj = r.ik(XT, YT, z, *QX, seed=prev); qs.append(jj); prev=jj
r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4))
