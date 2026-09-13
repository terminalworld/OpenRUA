"""Pick the cream cheese box (flat, 8x4.3x3 cm) and drop it in the basket."""
import subprocess
from rob import *


def measure(center_guess):
    subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], check=True, capture_output=True)
    pw = np.load("snaps/robot0_eye_in_hand_world.npy")
    zs = pw[..., 2]
    m = (zs > 0.435) & (zs < 0.48) & (np.abs(pw[..., 0] - center_guess[0]) < 0.07) \
        & (np.abs(pw[..., 1] - center_guess[1]) < 0.07)
    m[350:, :] = False
    p = pw[m]
    c = p[:, :2].mean(0)
    _, s, vt = np.linalg.svd(p[:, :2] - c, full_matrices=False)
    ax = vt[0]
    axis_yaw = np.degrees(np.arctan2(ax[1], ax[0]))
    grasp_yaw = (axis_yaw + 90) % 180 - 90  # fingers across the short side
    proj = (p[:, :2] - c) @ ax
    perp = (p[:, :2] - c) @ np.array([-ax[1], ax[0]])
    print(f"  box: n={m.sum()} centre={c.round(4)} axis_yaw={axis_yaw:.1f} grasp_yaw={grasp_yaw:.1f} "
          f"len={proj.max()-proj.min():.3f} wid={perp.max()-perp.min():.3f} ztop={p[:,2].max():.3f}")
    return c, grasp_yaw


r = Robot("pick_cheese")
BOX0 = np.array([0.0945, -0.178])
BASKET = np.array([-0.01, 0.253])
print("== above box")
assert r.move_line([*BOX0, 0.60], 0.0, seconds=6.0, step=0.02)
c, yaw = measure(BOX0)
assert r.move_line([*c, 0.60], yaw, seconds=2.0)
c, yaw = measure(c)
assert r.move_line([*c, 0.60], yaw, seconds=2.0)
print("== slow straight descent")
assert r.move_line([*c, 0.445], yaw, seconds=6.0, step=0.01)
print("  tcp", r.tcp_world()[0].round(4), "wrench", r.wrench())
print("== close")
gap = r.gripper(0.0)
assert gap > 0.03, "closed on air"
print("== lift")
assert r.move_line([*c, 0.72], yaw, seconds=5.0, step=0.01)
print("  gap after lift", r.finger_gap())
assert r.finger_gap() > 0.03, "lost the box"
print("== transit over basket")
assert r.move_line([*BASKET, 0.72], yaw, seconds=6.0, step=0.02)
print("== lower into basket")
assert r.move_line([*BASKET, 0.60], yaw, seconds=4.0, step=0.01)
print("  gap", r.finger_gap())
print("== release")
r.gripper(0.04)
print("== retreat up")
assert r.move_line([*BASKET, 0.80], yaw, seconds=4.0, step=0.01)
print("gap", r.finger_gap())
