from lib import *
r = Robot("t7")
seed = r.arm_q()
for yaw in (0, 180, 90):
    for x in (0.13, 0.14, 0.15):
        q = r.ik_world((x, 0.08, 1.084), down_quat(yaw), seed=seed, tries=2)
        q2 = r.ik_world((x, -0.02, 1.084), down_quat(yaw), seed=seed, tries=2)
        print(f"yaw{yaw} x{x}: y=0.08 {'ok '+str(np.round(q,2)) if q else '--'} | y=-0.02 {'ok '+str(np.round(q2,2)) if q2 else '--'}")
