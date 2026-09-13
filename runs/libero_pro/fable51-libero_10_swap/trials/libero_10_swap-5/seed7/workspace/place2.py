"""Stage C: lower the (now y-aligned) book into the back compartment until the
fingertips are just above the wall tops, release, retreat."""
from lib import *

r = Robot()
R90 = down_R(np.pi / 2)
cx, cy = -0.4315, -0.1465
prev = np.array(r.arm_q())
t, Ra = r.tcp(list(prev))
print("start tcp", np.round(t, 4), "yaw", np.degrees(np.arctan2(Ra[1, 0], Ra[0, 0])), "fingers", r.fingers())

for z in (1.18, 1.12, 1.07):
    qn = r.ik([cx, cy, z], R90, seed=list(prev), tries=3)
    assert qn is not None, f"IK failed at z={z}"
    d = np.abs(np.array(qn) - prev).max()
    print(f"z={z}: max joint delta {d:.3f}")
    assert d < 0.6, "joint jump too large; aborting before motion"
    q = r.move_tcp([cx, cy, z], R90, seconds=2.0, seed=list(prev))
    prev = np.array(q)
    t, Ra = r.tcp()
    print("   yaw", np.degrees(np.arctan2(Ra[1, 0], Ra[0, 0])), "fingers", r.fingers())

r.gripper(GRIP["open_m"])
print("fingers after open:", r.fingers())
q = r.move_tcp([cx, cy, 1.30], R90, seconds=3.0, seed=list(prev))
print("retreated; fingers", r.fingers())
