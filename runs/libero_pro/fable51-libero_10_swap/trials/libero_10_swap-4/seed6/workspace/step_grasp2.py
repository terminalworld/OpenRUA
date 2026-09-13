"""Straight vertical descent via chained IK waypoints, close, lift."""
import sys, numpy as np
from arm import Arm, down_quat
x, y, z, zlift = map(float, sys.argv[1:5])
a = Arm("grasp2")
qd = down_quat(0)
tcp0, _ = a.tcp_world(); q = a.arm_q()
print("start TCP", tcp0.round(4), flush=True)
zs = list(np.arange(tcp0[2] - 0.02, z, -0.02)) + [z]
via, qs = [], []
t = 0.0
for zz in zs:
    q = a.ik([x, y, zz], qd, seed=q)
    if q is None: raise SystemExit(f"IK failed at z={zz}")
    qs.append(q)
# check smoothness of the chain: joint deltas and FK of midpoints
for i in range(1, len(qs)):
    mid = (qs[i-1] + qs[i]) / 2
    tm, _ = a.tcp_world(mid)
    print(f"  wp z={zs[i]:.3f} dq_max={np.abs(qs[i]-qs[i-1]).max():.3f} mid TCP {tm.round(3)}", flush=True)
if np.abs(np.diff(qs, axis=0)).max() > 0.5:
    raise SystemExit("joint chain not smooth; aborting")
dt = 1.0
via = [(qq, dt * (i + 1)) for i, qq in enumerate(qs[:-1])]
a.move_q(qs[-1], dt * len(qs), via=via)
print("TCP at grasp", a.tcp_world()[0].round(4), flush=True)
gap = a.gripper(0.0)
print("GAP after close:", round(gap, 4), flush=True)
# lift the same way (reverse the chain) then up to zlift
q = a.arm_q()
qlift = a.ik([x, y, zlift], qd, seed=qs[0])
if qlift is None: raise SystemExit("IK failed for lift")
rev = list(reversed(qs[:-1]))
via = [(qq, dt * (i + 1)) for i, qq in enumerate(rev)]
a.move_q(qlift, dt * (len(rev) + 2), via=via)
print("TCP after lift", a.tcp_world()[0].round(4), "gap", round(a.finger_gap(), 4), flush=True)
