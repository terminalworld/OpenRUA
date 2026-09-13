import numpy as np, sys
from rob import *
from goto import margin
from close_plan2 import check2, R_tilt, WAY

r = Robot("ce")
if r.finger_gap() > 0.01:
    r.gripper(0.0)
print("gap", round(r.finger_gap(), 4))
t0, R0 = r.tcp(); q = r.arm_q()
print("start tcp", np.round(t0, 3))


def seg(tcp1, R1, n, seconds, shift1, shift0):
    global t0, R0, q
    path = cart_path(r, t0, R0, tcp1, R1, n, q, max_step=0.6)
    if path is None:
        print("   cart_path failed"); return False
    for i, qq in enumerate(path):
        sh = shift0 + (shift1 - shift0) * (i + 1) / len(path)
        c = check2(r, qq, sh)
        print(f"   via {i}: margin={margin(qq):.2f} clear={c[0]:.3f} {c[1]}")
        if c[0] < -0.01:
            print("   !! unsafe"); return False
    r.move_q(path[-1], seconds, via=path[:-1])
    t0, R0 = r.tcp(); q = r.arm_q()
    print("   tcp", np.round(t0, 4), "force", np.round(r.force(), 1), flush=True)
    return True


shift0 = 0.0
for name, tcp, tilt, shift in WAY:
    print(name)
    n = {"high": 3, "pre": 4}.get(name, 2)
    secs = {"high": 5, "pre": 4}.get(name, 3)
    if not seg(tcp, R_tilt(tilt), n, secs, shift, shift0):
        sys.exit(name + " failed")
    shift0 = shift
    if name in ("high", "pre"):
        r.snap("agentview", f"/workspace/snaps/av_{name}.png")
r.snap("agentview", "/workspace/snaps/av_pushed.png"); r.snap("frontview", "/workspace/snaps/fv_pushed.png")
