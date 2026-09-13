from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
X, Y = -0.0585, -0.108
j0 = r.ik(X, Y, 1.05, *QX)
for i in range(4):
    if r.move(j0, 3.0) == 0: break
print("pre tcp", r.tcp()[0].round(4))
zs = np.arange(1.03, 0.947, -0.02).tolist() + [0.948]
qs = []; prev = r.joints()
for z in zs:
    j = r.ik(X, Y, z, *QX, seed=prev)
    if j is None: raise SystemExit("ik fail at z=%s" % z)
    qs.append(j); prev = j
code = r.move_path(qs, 0.8)
print("tcp", r.tcp()[0].round(4), "joints", np.round(r.joints(),3))
