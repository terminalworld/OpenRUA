"""Pick the moka pot from above (fingers closing along world x), lift, and hold."""
import subprocess, numpy as np, pk

YAW = np.pi / 2       # hand y (finger axis) -> world x
r = pk.Robot("grasp")


def move(pos, R, secs, tries=3):
    for _ in range(tries):
        q, _, _ = r.move_pose(pos, R, secs)
        if np.abs(r.joints() - q).max() < 0.01:
            return
        print("  resend (lag)")


def pot_top_center():
    """Eye-in-hand height map -> (cx, cy) of the octagonal lid."""
    subprocess.run(["python3", "/workspace/heightmap.py", "robot0_eye_in_hand"], check=True, capture_output=True)
    pw = np.load("/workspace/robot0_eye_in_hand_world.npy")
    X, Y, Z = pw[..., 0], pw[..., 1], pw[..., 2]
    box = (X > -0.15) & (X < 0.06) & (Y > -0.40) & (Y < -0.15) & (Z > 1.026) & (Z < 1.06)
    ys = np.arange(-0.40, -0.15, 0.004)
    wide = []
    for yc in ys:
        m = box & (np.abs(Y - yc) < 0.002)
        if m.sum() > 5 and X[m].max() - X[m].min() > 0.04:
            wide.append((yc, X[m].min(), X[m].max()))
    wide = np.array(wide)
    cy = (wide[:, 0].min() + wide[:, 0].max()) / 2
    mid = wide[np.abs(wide[:, 0] - cy) < 0.015]
    cx = (mid[:, 1].mean() + mid[:, 2].mean()) / 2
    print(f"  lid: y∈[{wide[:,0].min():.4f},{wide[:,0].max():.4f}] x∈[{mid[:,1].mean():.4f},{mid[:,2].mean():.4f}] "
          f"width_x={mid[:,2].mean()-mid[:,1].mean():.4f} center=({cx:.4f},{cy:.4f}) zmax={Z[box].max():.4f}")
    return cx, cy


print("open"); r.gripper(pk.GRIP["open_m"])
print("go above pot"); move([-0.0446, -0.268, 1.22], pk.topdown_R(YAW), 4.0)
cx, cy = pot_top_center()
print("re-center"); move([cx, cy, 1.22], pk.topdown_R(YAW), 2.0)
cx2, cy2 = pot_top_center()
print(f"  second estimate ({cx2:.4f},{cy2:.4f})")
cx, cy = (cx + cx2) / 2, (cy + cy2) / 2
f0, _ = r.wrench(); print("  wrench baseline", f0.round(2))
for z in (1.08, 1.04, 1.00):
    print(f"descend to {z}"); move([cx, cy, z], pk.topdown_R(YAW), 1.5)
    f, _ = r.wrench(); print("  wrench", f.round(2), "delta", (f - f0).round(2))
    if abs(f[2] - f0[2]) > 6:
        print("CONTACT - abort before close"); raise SystemExit(1)
print("close"); fg = r.gripper(pk.GRIP["closed_m"])
print("lift"); move([cx, cy, 1.22], pk.topdown_R(YAW), 3.0)
f, _ = r.wrench(); print("  fingers", np.round(r.fingers(), 4), "wrench", f.round(2), "delta", (f - f0).round(2))
print("DONE")
