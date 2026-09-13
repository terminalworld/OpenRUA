"""Close the microwave door: hand vertical (fingers down, closed), fingertips push the door's outer face along the hinge arc."""
import sys
from cart import *
r = Robot()
H = np.array([-0.19, 0.25]); rho = float(sys.argv[1]); press = float(sys.argv[2]); zt = float(sys.argv[3])
th0, th1 = float(sys.argv[4]), float(sys.argv[5])
def pose(deg):
    t = np.radians(deg); d = np.array([np.cos(t), -np.sin(t)]); n = np.array([-np.sin(t), -np.cos(t)])
    p = H + rho*d + (0.01 - press)*n
    tcp = np.array([p[0], p[1], zt])
    yh = np.array([d[0], d[1], 0]); zh = np.array([0,0,-1.0]); xh = np.cross(yh, zh)
    return tcp, R_from_axes(xh, yh, zh)
degs = np.arange(th0, th1 - 1e-6, -8.0 if th1 < th0 else 8.0); degs = np.append(degs, th1)
seed = r.joints(); qs = []
for dg in degs:
    tcp, R = pose(dg); hand = hand_pose_from_tcp(tcp, R)
    q = r.ik_world(hand, quat_from_R(R), seed=seed, attempts=3)
    if q is None: sys.exit(f"IK fail at {dg}")
    print(f"th={dg:5.1f} tcp={np.round(tcp,3)} jump={max(abs(a-b) for a,b in zip(q,seed)):.3f} q={np.round(q,2)}")
    seed = q; qs.append(q)
if "--go" in sys.argv:
    q0 = np.array(r.joints()); tot = sum(np.abs(np.diff(np.array([q0]+qs), axis=0)), 0)
    secs = max(6.0, 6.0*tot[6] + 2.0, tot.max()*3)
    print("secs", secs); ok = run(qs, r, secs); print("ok", ok, "q", np.round(r.joints(),2))
    pos, quat = r.fk_world(); Rn = Rotation.from_quat(quat).as_matrix(); print("tcp", np.round(pos + 0.1034*Rn[:,2],3))
