from lib import *
r = Robot("t8")
def tilted(yaw, pitch):
    return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat(down_quat(yaw))).as_quat()
seed = r.arm_q()
for z in (1.075, 1.10):
  for pitch in (0, -5, -10, -15):
    row=[]
    for x in (0.14, 0.15, 0.16, 0.17, 0.18):
        q = r.ik_world((x, 0.03, z), tilted(180, pitch), seed=seed, tries=2)
        row.append(f"x{x}:{'ok' if q else '--'}")
    print(f"z{z} pitch{pitch}: "+" ".join(row))
