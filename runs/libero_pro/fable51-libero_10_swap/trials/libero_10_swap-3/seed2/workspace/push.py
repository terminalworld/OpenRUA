#!/usr/bin/env python3
"""Close the bottom drawer: push its front panel toward -y with the closed
fingertips, hand pitched 45deg so the fingertips lead the palm (keeps the
palm clear of the middle-drawer handle at y=-0.189, z 1.005-1.035)."""
import subprocess
import sys

import numpy as np

import ctl

r = ctl.Robot("push")
th = -np.pi / 4
Q = ctl.quat_mul((np.sin(th / 2), 0, 0, np.cos(th / 2)), ctl.Q_DOWN_FINGERS_X)
X, Z = -0.19, 0.955


def go(pos, secs=3.0):
    q = r.ik_tcp(pos, Q)
    assert q is not None, "IK failed"
    code, err = r.move_q(q, secs)
    return code == 0 and err < 0.02


def panel_y():
    subprocess.run([sys.executable, "scene.py", "agentview"], check=True, capture_output=True)
    xyz = np.load("snaps/agentview_xyz.npy")
    z = xyz[..., 2]
    # panel top strip (z ~0.983) away from handle/fingers: x in [-0.22,-0.16]
    k = (z > 0.975) & (z < 0.992) & (xyz[..., 0] > -0.24) & (xyz[..., 0] < -0.16) & \
        (xyz[..., 1] > -0.24) & (xyz[..., 1] < 0.0)
    p = xyz[k]
    return (float(np.percentile(p[:, 1], 2)), float(np.percentile(p[:, 1], 98)), int(k.sum())) if k.sum() > 5 else None


print("gap", r.finger_gap())
print("panel top y before:", panel_y(), flush=True)
assert go([X, -0.03, 1.03], 4.0)
assert go([X, -0.03, Z], 3.0)
for y in (-0.05, -0.08, -0.11, -0.14, -0.17, -0.19, -0.205, -0.212):
    ok = go([X, y, Z], 3.0)
    tcp = r.fk_tcp()[0]
    print(f"push to y={y}: ok={ok} tcp={tcp.round(4)}", flush=True)
    if not ok:
        print("stall -> stop pushing", file=sys.stderr)
        break
print("panel top y after:", panel_y(), flush=True)
# retreat: back off +y then up
tcp = r.fk_tcp()[0]
go([X, tcp[1] + 0.03, Z], 2.0)
go([X, tcp[1] + 0.03, 1.08], 2.0)
print("retreated; tcp", r.fk_tcp()[0].round(4), flush=True)
