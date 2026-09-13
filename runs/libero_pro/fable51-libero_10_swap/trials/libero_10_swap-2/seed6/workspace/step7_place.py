"""Carry the moka pot to the burner, set it down, release, retreat."""
import numpy as np, pk

BURNER = np.array([-0.057, 0.197])
POT_DROP = 0.141          # pot bottom sits this far below the TCP while hanging from the lid knob
Z_BURNER = 0.930
r = pk.Robot("place")


def move(pos, R, secs, tries=3):
    for _ in range(tries):
        q, _, _ = r.move_pose(pos, R, secs)
        if np.abs(r.joints() - q).max() < 0.01:
            return
        print("  resend (lag)")


print("fingers", np.round(r.fingers(), 4))
T = pk.fk(r.joints(), True)
print("raise"); move([T[0, 3], T[1, 3], 1.26], pk.topdown_R(np.pi / 2), 2.0)
print("over burner"); move([*BURNER, 1.26], pk.topdown_R(0.0), 5.0)
print("fingers", np.round(r.fingers(), 4), "wrench", r.wrench()[0].round(2))
f0, _ = r.wrench()
for z in (1.15, Z_BURNER + POT_DROP + 0.004):
    print(f"lower to {z:.4f}"); move([*BURNER, z], pk.topdown_R(0.0), 2.0)
    f, _ = r.wrench(); print("  wrench", f.round(2), "delta", (f - f0).round(2))
print("release"); r.gripper(pk.GRIP["open_m"])
print("retreat"); move([*BURNER, 1.26], pk.topdown_R(0.0), 3.0)
print("DONE")
