#!/usr/bin/env python3
"""Eye-in-hand camera geometry. Camera sits at (0.05,0,0) in panda_hand,
looking along hand +z; image-up = hand +x, image-right = hand +y.

  python3 tools/eih.py snap            # save eih.png / eih_depth.npy in cwd
  python3 tools/eih.py px u v          # world point for pixel (uses live FK + depth)
"""
import sys
import numpy as np

F = 312.77408948188935
CX, CY = 320.0, 240.0
CAM_IN_HAND = np.array([0.05, 0.0, 0.0])


def px_to_hand(u, v, d):
    X = (u - CX) / F * d   # image right  -> hand +y
    Y = (v - CY) / F * d   # image down   -> hand -x
    return CAM_IN_HAND + np.array([-Y, X, d])


def px_to_world(u, v, d, hand_pos, hand_R):
    return hand_pos + hand_R @ px_to_hand(u, v, d)


def cloud(depth, hand_pos, hand_R, step=1):
    """Full depth image -> (H,W,3) world points."""
    H, W = depth.shape
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    X = (us - CX) / F * depth
    Y = (vs - CY) / F * depth
    P_hand = np.stack([CAM_IN_HAND[0] - Y, CAM_IN_HAND[1] + X, CAM_IN_HAND[2] + depth], -1)
    return hand_pos + P_hand @ hand_R.T


if __name__ == "__main__":
    import rclpy
    sys.path.insert(0, "/workspace/tools")
    from robot import Robot
    import subprocess
    rclpy.init()
    r = Robot()
    pos, quat, R = r.fk()
    if sys.argv[1] == "px":
        u, v = int(sys.argv[2]), int(sys.argv[3])
        d = np.load("robot0_eye_in_hand_depth.npy")[v, u]
        print("depth", d, "world", np.round(px_to_world(u, v, d, pos, R), 4))
    r.destroy_node(); rclpy.shutdown()
