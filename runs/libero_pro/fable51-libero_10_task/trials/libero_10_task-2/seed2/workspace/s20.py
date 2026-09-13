from rob import *
r = R()
QX = (0.7071068, 0.7071068, 0.0, 0.0)
X0, Y0 = -0.0585, -0.108
XT, YT = -0.0425, 0.348
# check target IK first
jt = r.ik(XT, YT, 0.965, *QX)
print("target ik", None if jt is None else np.round(jt,3))
if jt is None: raise SystemExit("unreachable")
qs=[]; prev=r.joints()
# rise
for z in (1.16, 1.20):
    jj = r.ik(X0, Y0, z, *QX, seed=prev); qs.append(jj); prev=jj
# translate in y (and x) at z=1.20
n = 10
for k in range(1, n+1):
    x = X0 + (XT-X0)*k/n; y = Y0 + (YT-Y0)*k/n
    jj = r.ik(x, y, 1.20, *QX, seed=prev)
    if jj is None: raise SystemExit(f"ik fail at {x},{y}")
    print(f"wp {k} jump={np.abs(np.array(jj)-np.array(prev)).max():.3f}")
    qs.append(jj); prev=jj
r.move_path(qs, 0.7)
print("tcp", r.tcp()[0].round(4), "fingers", r.fingers())
