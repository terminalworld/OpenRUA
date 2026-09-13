#!/usr/bin/env python3
"""Re-grasp the fin at its current angle and rotate it further by DELTA (one move)."""
import sys, subprocess
from rob import *

KNOB = np.array([-0.205, 0.194])
FIN_DEG = float(sys.argv[1])
DELTA = float(sys.argv[2]) if len(sys.argv) > 2 else -0.4
HOME = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
yaw = math.radians(FIN_DEG)
r = Robot("step4")
r.gripper(GRIP["open_m"])
r.move_tcp((KNOB[0], KNOB[1], 1.02), quat_down(yaw))
subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", "eih_knob4.png"])
r.move_tcp((KNOB[0], KNOB[1], 0.945), quat_down(yaw))
f = r.gripper(GRIP["closed_m"])
if abs(f[0]) < 0.008:
    print("MISSED the fin (fingers closed on air); releasing and lifting", flush=True)
    r.gripper(GRIP["open_m"])
    r.move_tcp((KNOB[0], KNOB[1], 1.05), quat_down(yaw))
    r.move_q(HOME)
    raise SystemExit(1)
q = r.arm_q()
q2 = list(q); q2[6] = q[6] + DELTA
code, err = r.move_q(q2, seconds=2.0)
qn = r.arm_q()
print(f"rotate: q7 {q[6]:.3f} -> {qn[6]:.3f} (moved {qn[6]-q[6]:.3f}) code={code} fingers={r.fingers()}", flush=True)
r.gripper(GRIP["open_m"])
r.move_tcp((KNOB[0], KNOB[1], 1.05), quat_down(yaw))
r.move_q(HOME)
print("tcp", r.tcp()[0].round(3), flush=True)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_knob4.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "birdview", "bird_knob4.png"])
