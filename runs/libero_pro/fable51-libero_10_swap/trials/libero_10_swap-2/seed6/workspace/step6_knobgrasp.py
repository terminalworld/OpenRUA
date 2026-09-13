"""Grasp the moka pot by its lid knob from above, lift, and hold."""
import subprocess, numpy as np, pk

YAW = np.pi / 2
r = pk.Robot("kg")


def move(pos, R, secs, tries=3):
    for _ in range(tries):
        q, _, _ = r.move_pose(pos, R, secs)
        if np.abs(r.joints() - q).max() < 0.01:
            return
        print("  resend (lag)")


def lid_knob():
    subprocess.run(["python3", "/workspace/heightmap.py", "robot0_eye_in_hand"], check=True, capture_output=True)
    pw = np.load("/workspace/robot0_eye_in_hand_world.npy")
    X, Y, Z = pw[..., 0], pw[..., 1], pw[..., 2]
    box = (X > -0.15) & (X < 0.06) & (Y > -0.40) & (Y < -0.15)
    lid = box & (Z > 1.026) & (Z < 1.042)
    knob = box & (Z > 1.044) & (Z < 1.07)
    print(f"  lid zmax={Z[lid].max():.4f} n={lid.sum()}  knob: n={knob.sum()} x∈[{X[knob].min():.4f},{X[knob].max():.4f}] "
          f"y∈[{Y[knob].min():.4f},{Y[knob].max():.4f}] z∈[{Z[knob].min():.4f},{Z[knob].max():.4f}] "
          f"center=({X[knob].mean():.4f},{Y[knob].mean():.4f})")
    return X[knob].mean(), Y[knob].mean(), Z[lid].max(), Z[knob].max()


cx, cy, zlid, zknob = lid_knob()
print("go above knob"); move([cx, cy, 1.22], pk.topdown_R(YAW), 3.0)
cx2, cy2, zlid, zknob = lid_knob()
cx, cy = (cx + cx2) / 2, (cy + cy2) / 2
f0, _ = r.wrench(); print("  wrench baseline", f0.round(2))
zg = zlid + 0.004
for z in (1.10, zlid + 0.02, zg):
    print(f"descend to {z:.4f}"); move([cx, cy, z], pk.topdown_R(YAW), 1.5)
    f, _ = r.wrench(); print("  wrench", f.round(2), "delta", (f - f0).round(2))
    if abs(f[2] - f0[2]) > 6:
        print("CONTACT - abort before close"); raise SystemExit(1)
print("close"); fg = r.gripper(pk.GRIP["closed_m"])
if fg[0] > 0.03:
    print("fingers did not close on the knob (gap too wide) - abort"); raise SystemExit(1)
print("lift 2cm"); move([cx, cy, zg + 0.02], pk.topdown_R(YAW), 1.5)
f, _ = r.wrench(); print("  fingers", np.round(r.fingers(), 4), "wrench", f.round(2), "delta", (f - f0).round(2))
print("lift to 1.22"); move([cx, cy, 1.22], pk.topdown_R(YAW), 3.0)
f, _ = r.wrench(); print("  fingers", np.round(r.fingers(), 4), "wrench", f.round(2), "delta", (f - f0).round(2))
print("DONE")
