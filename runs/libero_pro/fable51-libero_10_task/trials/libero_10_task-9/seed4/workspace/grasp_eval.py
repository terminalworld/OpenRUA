import numpy as np
from geom import *
from rlib import R_from_axes, TCP

def rot_axis(axis, ang):
    a = np.asarray(axis, float); a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K

def grasp_hand_pose(mug_base, alpha, theta, d, mug_R=np.eye(3)):
    """Hand pose for a rim pinch. mug_base: world pos of mug base centre (mug frame
    origin); mug_R: mug orientation. Pinch at rim angle alpha (mug frame), fingers
    close radially; hand tilted theta about the closing axis (positive = hand leans
    toward -radial*... see code); d = TCP depth below rim."""
    c = np.array([np.cos(alpha), np.sin(alpha), 0.0])          # radial (mug frame)
    P = np.array([0.0435 * c[0], 0.0435 * c[1], MUG_H - d])     # TCP in mug frame
    y_h = c                                                     # closing dir
    t = np.array([-np.sin(alpha), np.cos(alpha), 0.0])          # tangential
    # z_h: start at -z, rotate about y_h by theta (leans along +/- tangential)
    z_h = rot_axis(y_h, theta) @ np.array([0, 0, -1.0])
    Rh_m = R_from_axes(z_h, y_h)                                # in mug frame
    Rh = mug_R @ Rh_m
    tcp_w = mug_R @ P + mug_base
    hand_w = tcp_w - TCP * Rh[:, 2]
    return hand_w, Rh, tcp_w

def scene_points(mug_base, mug_R, hand_w, Rh, finger=0.0025):
    hp = transform(hand_points((finger, finger)), Rh, hand_w)
    mp = transform(mug_points(), mug_R, mug_base)
    return hp, mp

def finger_mug_penetration(hand_w, Rh, mug_base, mug_R, finger=0.0025):
    """Depth (m) of hand/finger points inside the mug wall/handle, excluding the
    pad contact tolerance."""
    hp = transform(hand_points((finger, finger)), Rh, hand_w)
    q = (hp - mug_base) @ mug_R          # into mug frame
    r = np.hypot(q[:, 0], q[:, 1]); z = q[:, 2]
    in_wall = (r > MUG_RI + 0.0005) & (r < MUG_R - 0.0005) & (z > 0) & (z < MUG_H)
    in_handle = np.all([(q[:, i] > HANDLE[2 * i]) & (q[:, i] < HANDLE[2 * i + 1]) for i in range(3)], axis=0)
    in_floor = (r < MUG_R) & (z > 0) & (z < 0.009)
    return int(in_wall.sum()), int(in_handle.sum()), int(in_floor.sum())

if __name__ == "__main__":
    import itertools
    np.set_printoptions(precision=3, suppress=True)
    # grasp on table
    base0 = np.array([-0.084, -0.288, 0.90])
    for alpha_deg, theta_deg, d in itertools.product([180, 90, 270, 135, 225], [0, 20, 30, 40], [0.010, 0.015, 0.020]):
        for sgn in (1, -1):
            hw, Rh, tcp = grasp_hand_pose(base0, np.radians(alpha_deg), sgn * np.radians(theta_deg), d)
            pen = finger_mug_penetration(hw, Rh, base0, np.eye(3))
            hp = transform(hand_points((0.0025, 0.0025)), Rh, hw)
            cl, _ = clearance(hp)
            print(f"alpha {alpha_deg:3d} theta {sgn*theta_deg:+3d} d {d:.3f}  pen wall/handle/floor {pen}  static clr {cl:+.3f}")
