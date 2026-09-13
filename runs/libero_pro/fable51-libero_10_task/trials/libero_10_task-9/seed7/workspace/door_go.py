import sys
from door2 import *
rho, press, zt = 0.19, 0.0, 1.085
def seg(tcp1, R1, n=6):
    global tcp0, R0
    qs = cart_path(r, tcp0, R0, tcp1, R1, n)
    q0 = np.array(r.joints()); tot = np.abs(np.diff(np.array([q0]+qs), axis=0)).sum(0)
    secs = max(5.0, 6.0*tot[6] + 2.0, tot.max()*3)
    ok = run(qs, r, secs); pos, quat = r.fk_world(); Rn = Rotation.from_quat(quat).as_matrix()
    print("  ok", ok, "tcp", np.round(pos + 0.1034*Rn[:,2],3), "q", np.round(r.joints(),2))
    if not ok: sys.exit("fail")
    tcp0, R0 = np.asarray(tcp1, float), R1
print("fingers", r.gripper(0.0))
pos, quat = r.fk_world(); R0 = Rotation.from_quat(quat).as_matrix(); tcp0 = pos + 0.1034*R0[:,2]
tcpd, Rd = pose(97, rho, press, zt)
stage = sys.argv[1]
if stage == "approach":
    seg(tcp0 + [0,0,0.30], R0, 4)
    seg([tcpd[0], tcpd[1], 1.30], Rd, 8)
    seg(tcpd, Rd, 4)
