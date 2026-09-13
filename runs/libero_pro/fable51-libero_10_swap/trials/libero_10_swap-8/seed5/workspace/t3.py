from lib import *
r = Robot("t3")
def tilted(yaw, pitch):
    return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat(down_quat(yaw))).as_quat()
seed = r.arm_q()
for x in (0.10, 0.12, 0.14, 0.16, 0.19, 0.22, 0.24):
    row = []
    for pitch in (0, -15, -30, -45):
        for yaw in (90, 0):
            q = r.ik_world((x, 0.03, 1.00), tilted(yaw, pitch), seed=seed, tries=2)
            row.append(f"p{pitch}y{yaw}:{'ok' if q else '--'}")
    print(x, " ".join(row))
