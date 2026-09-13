#!/usr/bin/env python3
"""Pick the cream cheese box and drop it in the basket."""
import sys
from arm import *
import seg

EXPECT = np.array([-0.163, 0.058])
AXIS = 0.0            # long axis roughly along world x
GRASP_Z = 0.440       # box top 0.455, table 0.425
HOVER_Z = 0.58
CARRY_Z = 0.72
BASKET = np.array([0.0, 0.255])
BASKET_YAW = 45.0
RELEASE_Z = 0.66

def yawq(deg):
    return quat_mul((0.0, 0.0, np.sin(np.radians(deg) / 2), np.cos(np.radians(deg) / 2)), DOWN_Q)

def norm_deg(deg):
    while deg > 90: deg -= 180
    while deg <= -90: deg += 180
    return deg

a = Arm()
a.state()
print("== open"); a.gripper(True)
gp, yaw = EXPECT.copy(), AXIS
print("== hover", gp, yaw)
ok = a.move_tcp((gp[0], gp[1], HOVER_Z), 3.5, yawq(yaw))
if not ok:
    ok = a.move_tcp((gp[0], gp[1], HOVER_Z), 3.0, yawq(yaw))
assert ok, "hover failed"

for attempt in range(3):
    cls = seg.clusters(a.node, a.buf, "robot0_eye_in_hand", minh=0.012, maxh=0.06)
    for c in cls: print("   wrist:", seg.describe(c))
    c = seg.nearest(cls, gp, 0.06)
    if c is None or c["n"] < 100:
        print("   box not found in wrist cam; keeping estimate"); break
    gp2, yaw2 = c["centre"], norm_deg(c["axis_deg"])
    print(f"   refined ({gp2[0]:.4f},{gp2[1]:.4f}) yaw {yaw2:.1f} W={c['width']:.3f} L={c['length']:.3f} (delta {np.linalg.norm(gp2-gp)*100:.1f} cm)")
    if np.linalg.norm(gp2 - gp) < 0.003 and abs(yaw2 - yaw) < 3:
        break
    gp, yaw = gp2, yaw2
    assert a.move_tcp_lin((gp[0], gp[1], HOVER_Z), yawq(yaw), step=0.02, speed=0.05), "re-hover failed"

print("== descend")
assert a.move_tcp_lin((gp[0], gp[1], GRASP_Z), yawq(yaw), step=0.02, speed=0.04), "descend failed"
print("== close"); f1, f2 = a.gripper(False)
gap = abs(f1) + abs(f2)
if gap < 0.008:
    print("GRASP FAILED (gap %.4f)" % gap); a.gripper(True)
    a.move_tcp_lin((gp[0], gp[1], HOVER_Z), yawq(yaw)); sys.exit(2)
print("== lift")
assert a.move_tcp_lin((gp[0], gp[1], CARRY_Z), yawq(yaw), step=0.03, speed=0.06), "lift failed"
j = a.joints(); gap2 = abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"])
print(f"   gap after lift {gap2:.4f}")
if gap2 < 0.008:
    print("LOST OBJECT"); sys.exit(3)
print("== to basket")
ok = a.move_tcp((BASKET[0], BASKET[1], CARRY_Z), 4.0, yawq(BASKET_YAW))
if not ok: ok = a.move_tcp((BASKET[0], BASKET[1], CARRY_Z), 3.0, yawq(BASKET_YAW))
assert ok, "basket move failed"
j = a.joints(); print("   gap at basket", round(abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"]), 4))
print("== lower"); a.move_tcp_lin((BASKET[0], BASKET[1], RELEASE_Z), yawq(BASKET_YAW), step=0.03, speed=0.05)
print("== release"); a.gripper(True)
print("== retreat"); a.move_tcp_lin((BASKET[0], BASKET[1], CARRY_Z), yawq(BASKET_YAW), step=0.03, speed=0.06)
a.state()
print("DONE")
rclpy.shutdown()
