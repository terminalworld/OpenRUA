import numpy as np
from rob import Robot, topdown_quat
r = Robot("s6")
Q = topdown_quat(0.0)
C = np.array([-0.226, 0.033]); RIM_Z = 0.5415; R_WALL = 0.039
pinch = [C[0], C[1] + R_WALL]
print("pinch", pinch)
for z, t in [(RIM_Z + 0.035, 3.0), (RIM_Z - 0.025, 2.5)]:
    for attempt in range(3):
        q = r.ik_world([pinch[0], pinch[1], z], Q)
        if q is None: raise SystemExit(f"IK failed at z={z}")
        code, err = r.move_joints(q, t)
        if err < 0.02: break
        print("  retrying (tolerance violation)")
    tcp, _ = r.tcp_pose_world(); print(f"tcp {np.round(tcp,4)} target z {z:.4f}")
    if err >= 0.02: raise SystemExit("could not converge")
print("wrench before close", np.round(r.read_wrench(), 3))
r.gripper(0.0)
print("wrench after close", np.round(r.read_wrench(), 3))
print("lift to 0.80")
for attempt in range(3):
    q = r.ik_world([pinch[0], pinch[1], 0.80], Q)
    code, err = r.move_joints(q, 3.5)
    if err < 0.02: break
tcp, _ = r.tcp_pose_world(); print("tcp", np.round(tcp, 4), "fingers", np.round(r.fingers(), 4), "wrench", np.round(r.read_wrench(), 3))
