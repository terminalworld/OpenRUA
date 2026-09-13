from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
r.grip(0.0)
j = r.joints(); r.move(j, 1.0)
print("fingers after settle", r.fingers())
# lift slowly straight up via waypoints
X, Y = -0.0585, -0.108
qs=[]; prev=r.joints()
for z in (0.97, 1.0, 1.05, 1.10, 1.15):
    jj = r.ik(X, Y, z, *QX, seed=prev); qs.append(jj); prev=jj
r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
