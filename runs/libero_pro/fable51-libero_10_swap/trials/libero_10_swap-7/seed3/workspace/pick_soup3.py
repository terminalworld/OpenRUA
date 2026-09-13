"""Soup pick, take 3: re-localize from above, slow straight descent, close, lift."""
import subprocess
from rob import *

r = Robot("pick_soup3")
SOUP0 = np.array([-0.2117, -0.135])
print("== above approximate can position")
assert r.move_line([*SOUP0, 0.60], 0.0, seconds=4.0)

# re-localize with the eye-in-hand cloud: lid pixels are z>0.50 in a window
subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], check=True, capture_output=True)
pw = np.load("snaps/robot0_eye_in_hand_world.npy")
zs = pw[..., 2]
m = (zs > 0.50) & (zs < 0.53) & (np.abs(pw[..., 0] - SOUP0[0]) < 0.06) & (np.abs(pw[..., 1] - SOUP0[1]) < 0.06)
p = pw[m]
xlo, xhi, ylo, yhi = p[:, 0].min(), p[:, 0].max(), p[:, 1].min(), p[:, 1].max()
print(f"  lid: n={m.sum()} x[{xlo:.4f},{xhi:.4f}] y[{ylo:.4f},{yhi:.4f}] ztop={p[:,2].max():.4f}")
dia = xhi - xlo  # x extent is free of the pull tab
soup = np.array([(xlo + xhi) / 2, ylo + dia / 2])  # tab is on the +y side
print(f"  can centre {soup.round(4)} dia {dia:.4f}")
assert abs(dia - 0.066) < 0.01

print("== centre above can")
assert r.move_line([*soup, 0.60], 0.0, seconds=3.0)
print("== slow straight descent")
assert r.move_line([*soup, 0.462], 0.0, seconds=6.0, step=0.01)
print("  wrench", r.wrench())
print("== close")
gap = r.gripper(0.0)
print("== lift")
assert r.move_line([*soup, 0.72], 0.0, seconds=5.0, step=0.01)
print("  gap after lift", r.finger_gap(), "wrench", r.wrench())
