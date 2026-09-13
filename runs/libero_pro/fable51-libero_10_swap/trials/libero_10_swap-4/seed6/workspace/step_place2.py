"""Carry to (x,y,zcarry) (2-3 waypoints), chained-IK descent to zplace, open, lift."""
import sys, numpy as np
from arm import Arm, down_quat
x, y, zcarry, zplace = map(float, sys.argv[1:5])
a = Arm("place2")
qd = down_quat(0)
tcp0, _ = a.tcp_world(); q0 = a.arm_q()
print("start TCP", tcp0.round(4), "gap", round(a.finger_gap(), 4), flush=True)
# carry: interpolate in cartesian in 3 segments, chained IK
pts = [tcp0 + (np.array([x, y, zcarry]) - tcp0) * s for s in (0.34, 0.67, 1.0)]
pts[0][2] = pts[1][2] = zcarry
qs, q = [], q0
for p in pts:
    q = a.ik(p, qd, seed=q)
    if q is None: raise SystemExit(f"IK failed at {p}")
    qs.append(q)
prev = q0
for i, qq in enumerate(qs):
    mid = (prev + qq) / 2; tm, _ = a.tcp_world(mid)
    print(f"  carry wp {pts[i].round(3)} dq_max={np.abs(qq-prev).max():.3f} mid TCP {tm.round(3)}", flush=True)
    prev = qq
via = [(qq, 2.0 * (i + 1)) for i, qq in enumerate(qs[:-1])]
a.move_q(qs[-1], 2.0 * len(qs), via=via)
print("TCP after carry", a.tcp_world()[0].round(4), "gap", round(a.finger_gap(), 4), flush=True)
# descend
q = a.arm_q()
zs = list(np.arange(zcarry - 0.02, zplace, -0.02)) + [zplace]
qs = []
for zz in zs:
    q = a.ik([x, y, zz], qd, seed=q)
    if q is None: raise SystemExit(f"IK failed at z={zz}")
    qs.append(q)
if np.abs(np.diff(qs, axis=0)).max() > 0.5: raise SystemExit("descent chain not smooth")
via = [(qq, 1.0 * (i + 1)) for i, qq in enumerate(qs[:-1])]
a.move_q(qs[-1], 1.0 * len(qs), via=via)
print("TCP at place", a.tcp_world()[0].round(4), flush=True)
a.gripper(0.04)
rev = list(reversed(qs[:-1]))
via = [(qq, 1.0 * (i + 1)) for i, qq in enumerate(rev)]
a.move_q(rev[-1] if rev else qs[0], 1.0 * len(rev), via=via[:-1])
print("done; TCP", a.tcp_world()[0].round(4), flush=True)
