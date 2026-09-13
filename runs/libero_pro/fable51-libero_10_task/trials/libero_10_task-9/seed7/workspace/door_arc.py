import sys
from door2 import *
rho, press, zt = 0.19, float(sys.argv[3]), 1.085
th0, th1 = float(sys.argv[1]), float(sys.argv[2])
degs = list(np.arange(th0, th1, -6.0)) + [th1]
seed = r.joints(); qs = []
for dg in degs:
    tcp, R = pose(dg, rho, press, zt)
    q = r.ik_world(hand_pose_from_tcp(tcp, R), quat_from_R(R), seed=seed, attempts=3)
    if q is None: sys.exit(f"IK fail {dg}")
    j = max(abs(a-b) for a,b in zip(q, seed)); print(f"th={dg:5.1f} tcp={np.round(tcp,3)} jump={j:.3f} q={np.round(q,2)}")
    if j > 0.5: sys.exit("jump too large")
    seed = q; qs.append(q)
if "--go" in sys.argv:
    q0 = np.array(r.joints()); tot = np.abs(np.diff(np.array([q0]+qs), axis=0)).sum(0)
    secs = max(8.0, 6.0*tot[6] + 2.0, tot.max()*3); print("secs", secs)
    ok = run(qs, r, secs); pos, quat = r.fk_world(); Rn = Rotation.from_quat(quat).as_matrix()
    print("ok", ok, "tcp", np.round(pos + 0.1034*Rn[:,2],3), "q", np.round(r.joints(),2))
