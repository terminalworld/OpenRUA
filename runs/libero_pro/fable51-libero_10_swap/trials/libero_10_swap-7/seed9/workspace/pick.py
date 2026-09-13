#!/usr/bin/env python3
"""Pick an object with a top-down grasp and drop it into the basket.

Usage: python3 -u pick.py <x> <y> <grasp_z> <yaw_rad> <drop_x> <drop_y>
"""
import subprocess
import sys

import numpy as np

import ctl
from ctl import log

x, y, gz, yaw, dx, dy = map(float, sys.argv[1:7])
ABOVE_Z = 0.62
CARRY_Z = 0.78
DROP_Z = 0.78

c = ctl.Ctl("pick")
log("start tcp", np.round(c.tcp_world()[0], 3), "gap", round(c.finger_gap(), 4))

# 1. open
c.gripper(ctl.GRIP["open_m"])

# 2. above the object
q_above = c.move_tcp([x, y, ABOVE_Z], yaw, 4.0)
if q_above is None:
    sys.exit("IK above failed")

# 3. descend in two steps, seeded for branch continuity
q_prev = q_above
for z in (gz + 0.08, gz):
    q = c.ik_tcp_world([x, y, z], yaw, seed=q_prev)
    if q is None:
        sys.exit(f"IK failed at z={z}")
    jump = np.abs(np.array(q) - np.array(q_prev)).max()
    log(f"descend to z={z:.3f} max joint jump {jump:.3f}")
    if jump > 1.0:
        sys.exit("branch flip on descent; aborting for safety")
    c.move_q(q, 2.0)
    q_prev = q
tcp, _ = c.tcp_world()
log("at grasp height tcp", np.round(tcp, 4))
subprocess.run([sys.executable, "tools/perception/cam_snap.py", "robot0_eye_in_hand",
                "eih_pregrasp.png"], timeout=120)

# 4. close and check
gap = c.gripper(ctl.GRIP["closed_m"])
if gap < 0.01:
    log("WARNING: gripper closed on air (gap %.4f)" % gap)

# 5. lift straight up, then carry to the basket
q_lift = c.ik_tcp_world([x, y, ABOVE_Z], yaw, seed=q_prev)
c.move_q(q_lift, 2.5)
log("after lift gap", round(c.finger_gap(), 4))
q_carry = c.ik_tcp_world([x, y, CARRY_Z], yaw, seed=q_lift)
c.move_q(q_carry, 2.0)
q_drop = c.ik_tcp_world([dx, dy, DROP_Z], 0.0, seed=q_carry)
if q_drop is None:
    sys.exit("IK drop failed")
c.move_q(q_drop, 4.0)
tcp, _ = c.tcp_world()
log("over basket tcp", np.round(tcp, 4), "gap", round(c.finger_gap(), 4))

# 6. release
c.gripper(ctl.GRIP["open_m"])
log("released; tcp", np.round(c.tcp_world()[0], 3))
subprocess.run([sys.executable, "tools/perception/cam_snap.py", "agentview",
                "after_drop.png"], timeout=120)
log("DONE")
