#!/usr/bin/env python3
"""Descend over the can in 1 cm steps; log pose error, wrench, can position."""
import rclpy
from geometry_msgs.msg import WrenchStamped

from arm import *
from seg import cloud

TILT = np.radians(30)
Q_SOUP = q_grasp(np.pi / 2, TILT)
a = Arm()
w = {}
a.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
                           lambda m: w.__setitem__("m", m), 1)


def wrench():
    w.clear()
    while "m" not in w:
        rclpy.spin_once(a.node, timeout_sec=0.2)
    f = w["m"].wrench.force
    return np.round([f.x, f.y, f.z], 1)


def can_from(cam, guess):
    P = cloud(cam, a.node)
    X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
    m = (abs(X - guess[0]) < 0.06) & (abs(Y - guess[1]) < 0.06) & (Z > 0.435) & (Z < 0.512)
    if m.sum() < 50:
        return None
    xe, ye = np.percentile(X[m], [1, 99]), np.percentile(Y[m], [1, 99])
    return np.array([xe.mean(), ye.mean()]), (xe[1] - xe[0], ye[1] - ye[0])


a.gripper(GRIP["open_m"])
guess = np.array([-0.235, -0.142])
c, wd = can_from("agentview", guess)
c2, wd2 = can_from("sideview", guess)
print("can agentview", np.round(c, 4), np.round(wd, 3), " sideview", np.round(c2, 4), np.round(wd2, 3))
SOUP = (c + c2) / 2
assert a.move_to([*SOUP, 0.62], q=Q_SOUP, seconds=5, seed=[0, -0.785, 0, -2.356, 0, 1.571, 0.785])
print("hover wrench", wrench())
for z in np.arange(0.56, 0.449, -0.01):
    ok = a.move_to([*SOUP, z], q=Q_SOUP, seconds=2, steps=1, tol_m=0.005)
    p, _ = a.hand_pose()
    r = can_from("agentview", SOUP)
    print(f"z={z:.3f} ok={ok} tcp={np.round(p,4)} dxyz={np.round(p-[*SOUP,z],4)} wrench={wrench()} "
          f"can={np.round(r[0],4) if r else None} fingers={np.round(a.finger_gap(),3)}")
    if not ok:
        break
