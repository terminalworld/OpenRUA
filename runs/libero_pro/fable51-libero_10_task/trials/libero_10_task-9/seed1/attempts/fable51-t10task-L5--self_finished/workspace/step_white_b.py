"""White mug is lying on its side (axis along y, rim facing +y at y=-0.10,
body x in [-0.058,0.032], centre z 0.949). Pinch the rim wall at its +x side
with a near-horizontal hand pointing -y (tilted 20 deg down), lift, stop."""
import numpy as np
from rob import *
from pathcheck import check

B = np.radians(20)
APP = np.array([0, -np.cos(B), -np.sin(B)])      # approach: -y, slightly down
R_S = hand_R(APP, [1, 0, 0])                     # fingers close along x
PX, PY, PZ = 0.030, -0.125, 0.949                # wall centre, 2.5 cm inside rim
grasp = np.array([PX, PY, PZ])
pregrasp = grasp - 0.08 * APP
lift = grasp + np.array([0, 0, 0.25])

r = Rob("white_b")
print("start tcp", np.round(r.fk()[0], 4).tolist(), "fingers", r.finger_gap())
r.gripper(0.04)


def go(p, R, secs, seed=None):
    q0, _ = r.joints()
    q, d = r.ik_near(p, R, seed=seed if seed is not None else q0)
    assert q is not None, f"IK failed {p}"
    assert check(r, q0, q), "hazard on path"
    r.movej([q], [secs])
    tcp, _ = r.fk()
    print(f"[go] tcp={np.round(tcp,4).tolist()} target={np.round(p,4).tolist()} err={np.linalg.norm(tcp-p):.4f}")
    return q


q = go(pregrasp, R_S, 8.0)
q = go(grasp, R_S, 3.0, seed=q)
gap = r.gripper(0.0)
print("GAP after close:", gap)
q = go(lift, R_S, 4.0, seed=q)
print("fingers after lift:", r.finger_gap())
print("DONE_B")
rclpy.shutdown()
