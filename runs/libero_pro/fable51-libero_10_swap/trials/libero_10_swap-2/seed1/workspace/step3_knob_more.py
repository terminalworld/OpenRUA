#!/usr/bin/env python3
"""Re-grasp the (already partly turned) fin and rotate it further in small steps."""
import sys, subprocess
from rob import *

KNOB = np.array([-0.205, 0.194])
FIN_DEG = float(sys.argv[1]) if len(sys.argv) > 1 else -31.0
STEPS = int(sys.argv[2]) if len(sys.argv) > 2 else 3
STEP = -0.35
yaw = math.radians(FIN_DEG)
r = Robot("step3")
r.gripper(GRIP["open_m"])
r.move_tcp((KNOB[0], KNOB[1], 1.02), quat_down(yaw), seconds=3)
subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", "eih_knob2.png"])
r.move_tcp((KNOB[0], KNOB[1], 0.945), quat_down(yaw), seconds=2)
f = r.gripper(GRIP["closed_m"])
q0 = r.arm_q()
print("q7 start", round(q0[6], 3), flush=True)
for i in range(STEPS):
    q = r.arm_q()
    q2 = list(q); q2[6] = q[6] + STEP
    if not (LIMITS[6][0] < q2[6] < LIMITS[6][1]):
        print("joint7 limit reached"); break
    code, err = r.move_q(q2, seconds=1.5)
    qn = r.arm_q()
    print(f"step {i}: q7 {q[6]:.3f} -> {qn[6]:.3f} (moved {qn[6]-q[6]:.3f}) code={code} fingers={r.fingers()}", flush=True)
    if abs(qn[6] - q[6]) < 0.15:
        print("knob resisting; stop"); break
print("total joint7 rotation this session", round(r.arm_q()[6] - q0[6], 3), flush=True)
r.gripper(GRIP["open_m"])
r.move_tcp((KNOB[0], KNOB[1], 1.08), quat_down(yaw), seconds=2)
# park the arm up and away from the knob so cameras can see it
r.move_tcp((-0.35, 0.0, 1.20), quat_down(0), seconds=3)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_knob3.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "birdview", "bird_knob3.png"])
