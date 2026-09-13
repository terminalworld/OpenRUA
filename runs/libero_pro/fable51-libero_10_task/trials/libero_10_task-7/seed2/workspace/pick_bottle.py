#!/usr/bin/env python3
"""Pick the lying ketchup bottle and drop it in the basket."""
import sys
from arm import *
import seg

EXPECT = np.array([-0.109, -0.164])   # birdview centroid of the bottle
AXIS = -25.0                           # deg, long axis
BODY_SHIFT = -0.03                     # metres along axis from centroid toward the (wider) body end
GRASP_Z = 0.445
HOVER_Z = 0.60
CARRY_Z = 0.72
BASKET = np.array([0.0, 0.255])
BASKET_YAW = 45.0
RELEASE_Z = 0.63

def yawq(deg):
    return quat_mul((0.0, 0.0, np.sin(np.radians(deg) / 2), np.cos(np.radians(deg) / 2)), DOWN_Q)

a = Arm()
a.state()
print("== open"); a.gripper(True)

def body_centre(c):
    """grasp point: shift from centroid along axis toward the wider end."""
    ax = np.array([np.cos(np.radians(c["axis_deg"])), np.sin(np.radians(c["axis_deg"]))])
    xy = c["pts"][:, :2]; s = (xy - c["centre"]) @ ax
    perp = (xy - c["centre"]) @ np.array([-ax[1], ax[0]])
    wneg = perp[s < -0.02].ptp() if (s < -0.02).sum() > 5 else 0
    wpos = perp[s > 0.02].ptp() if (s > 0.02).sum() > 5 else 0
    sign = -1 if wneg > wpos else 1
    print(f"   widths: s<0 {wneg:.3f}  s>0 {wpos:.3f} -> body on {'negative' if sign<0 else 'positive'} side")
    deg = c["axis_deg"]
    while deg > 90: deg -= 180
    while deg <= -90: deg += 180
    return c["centre"] + sign * abs(BODY_SHIFT) * ax, deg

# 1. hover above the birdview estimate
gp = EXPECT + BODY_SHIFT * np.array([np.cos(np.radians(AXIS)), np.sin(np.radians(AXIS))])
yaw = AXIS
print("== hover", gp, yaw)
assert a.move_tcp((gp[0], gp[1], HOVER_Z), 3.0, yawq(yaw)), "hover failed"
p, q, tcp = a.hand_world(); R = quat_R(*q)
print(f"   hand y-axis in world: ({R[0,1]:.3f},{R[1,1]:.3f}); bottle axis dir ({np.cos(np.radians(yaw)):.3f},{np.sin(np.radians(yaw)):.3f}) dot={R[0,1]*np.cos(np.radians(yaw))+R[1,1]*np.sin(np.radians(yaw)):.3f} (want 0)")

# 2. refine from the wrist camera
for attempt in range(2):
    cls = seg.clusters(a.node, a.buf, "robot0_eye_in_hand", minh=0.012, maxh=0.10)
    for c in cls: print("   wrist:", seg.describe(c))
    c = seg.nearest(cls, gp, 0.08)
    if c is None or c["n"] < 200:
        print("   bottle not found in wrist cam; keeping birdview estimate"); break
    gp2, yaw2 = body_centre(c)
    print(f"   refined grasp ({gp2[0]:.4f},{gp2[1]:.4f}) yaw {yaw2:.1f}  (delta {np.linalg.norm(gp2-gp)*100:.1f} cm)")
    if np.linalg.norm(gp2 - gp) < 0.004 and abs(yaw2 - yaw) < 3:
        break
    gp, yaw = gp2, yaw2
    assert a.move_tcp_lin((gp[0], gp[1], HOVER_Z), yawq(yaw), step=0.02, speed=0.05), "re-hover failed"

# 3. descend, grasp
print("== descend")
assert a.move_tcp_lin((gp[0], gp[1], GRASP_Z), yawq(yaw), step=0.02, speed=0.04), "descend failed"
print("== close"); f1, f2 = a.gripper(False)
gap = abs(f1) + abs(f2)
if gap < 0.01:
    print("GRASP FAILED (gap %.4f)" % gap); a.gripper(True)
    a.move_tcp_lin((gp[0], gp[1], HOVER_Z), yawq(yaw)); sys.exit(2)
print("== lift")
assert a.move_tcp_lin((gp[0], gp[1], CARRY_Z), yawq(yaw), step=0.03, speed=0.06), "lift failed"
j = a.joints(); gap2 = abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"])
print(f"   gap after lift {gap2:.4f}")
if gap2 < 0.01:
    print("LOST OBJECT"); sys.exit(3)
# 4. to basket (rotate so the bottle lies along the basket diagonal)
print("== to basket")
assert a.move_tcp((BASKET[0], BASKET[1], CARRY_Z), 4.0, yawq(BASKET_YAW)), "basket move failed"
j = a.joints(); print("   gap at basket", round(abs(j["panda_finger_joint1"]) + abs(j["panda_finger_joint2"]), 4))
print("== lower"); a.move_tcp_lin((BASKET[0], BASKET[1], RELEASE_Z), yawq(BASKET_YAW), step=0.03, speed=0.05)
print("== release"); a.gripper(True)
print("== retreat"); a.move_tcp_lin((BASKET[0], BASKET[1], CARRY_Z), yawq(BASKET_YAW), step=0.03, speed=0.06)
a.state()
print("DONE")
rclpy.shutdown()
