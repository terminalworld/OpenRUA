"""Soup pick, take 4: can is lying on its side; grasp across its diameter."""
import subprocess
from rob import *


def measure(center_guess):
    subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], check=True, capture_output=True)
    pw = np.load("snaps/robot0_eye_in_hand_world.npy")
    zs = pw[..., 2]
    m = (zs > 0.44) & (zs < 0.52) & (np.abs(pw[..., 0] - center_guess[0]) < 0.09) \
        & (np.abs(pw[..., 1] - center_guess[1]) < 0.09)
    m[350:, :] = False  # fingers live in the bottom rows
    p = pw[m]
    c = p[:, :2].mean(0)
    _, s, vt = np.linalg.svd(p[:, :2] - c, full_matrices=False)
    ax = vt[0]
    axis_yaw = np.degrees(np.arctan2(ax[1], ax[0]))
    grasp_yaw = (axis_yaw + 90) % 180 - 90  # fingers perpendicular to the axis
    proj = (p[:, :2] - c) @ ax
    perp = (p[:, :2] - c) @ np.array([-ax[1], ax[0]])
    print(f"  can: n={m.sum()} centre={c.round(4)} axis_yaw={axis_yaw:.1f} grasp_yaw={grasp_yaw:.1f} "
          f"len={proj.max()-proj.min():.3f} wid={perp.max()-perp.min():.3f} ztop={p[:,2].max():.3f}")
    return c, grasp_yaw


r = Robot("pick_soup4")
c, yaw = measure(np.array([-0.146, -0.114]))
print("== above can, aligned")
assert r.move_line([*c, 0.60], yaw, seconds=4.0)
c, yaw = measure(c)
assert r.move_line([*c, 0.60], yaw, seconds=2.0)
print("== slow straight descent")
assert r.move_line([*c, 0.46], yaw, seconds=6.0, step=0.01)
print("  tcp", r.tcp_world()[0].round(4), "wrench", r.wrench())
print("== close")
gap = r.gripper(0.0)
print("== lift")
assert r.move_line([*c, 0.72], yaw, seconds=5.0, step=0.01)
print("  gap after lift", r.finger_gap(), "wrench", r.wrench())
