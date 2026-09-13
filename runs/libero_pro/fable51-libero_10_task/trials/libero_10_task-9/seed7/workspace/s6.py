"""Execute a sequence of TCP waypoints (x,y,z,roll,pitch) via Cartesian interpolation."""
import sys
from geo import *
r = Robot()
go = "--go" in sys.argv
specs = [a for a in sys.argv[1:] if not a.startswith("--")]
n = 6
pos, quat = r.fk_world(); R0 = Rotation.from_quat(quat).as_matrix(); tcp0 = pos + 0.1034*R0[:,2]
print("start tcp", np.round(tcp0,3), "q", np.round(r.joints(),2))
for a in specs:
    x,y,z,roll,pitch = [float(v) for v in a.split(",")]
    R1 = Rrp(roll,pitch); tcp1 = np.array([x,y,z])
    report(tcp1, R1, f"--> roll{roll:.0f} pitch{pitch:.0f}")
    qs = cart_path(r, tcp0, R0, tcp1, R1, n)
    if go:
        q0 = np.array(r.joints()); dq = np.abs(np.array(qs[-1]) - q0)
        secs = max(5.0, 6.0*dq[6] + 2.0, dq.max()*4)
        ok = run(qs, r, secs)
        pos, quat = r.fk_world(); Rn = Rotation.from_quat(quat).as_matrix()
        print("  ok", ok, "tcp", np.round(pos + 0.1034*Rn[:,2],3), "fingers", np.round(r.fingers(),4), "q", np.round(r.joints(),2))
        if not ok: sys.exit("move failed")
    tcp0, R0 = tcp1, R1
