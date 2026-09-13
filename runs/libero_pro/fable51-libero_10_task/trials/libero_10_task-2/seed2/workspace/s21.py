from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
XT, YT = -0.0425, 0.348
qs=[]; prev=r.joints()
for z in (1.15, 1.10, 1.05, 1.01):
    jj = r.ik(XT, YT, z, *QX, seed=prev); qs.append(jj); prev=jj
r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
