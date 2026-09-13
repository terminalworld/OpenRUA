import numpy as np, pk
r = pk.Robot("rel")
print("open"); r.gripper(pk.GRIP["open_m"])
q = r.joints()
T = pk.fk(q, True); print("tcp now", T[:3,3].round(4))
# lift straight up, keep yaw as is
pos = T[:3,3].copy(); pos[2] = 1.10
r.move_pose(pos, T[:3,:3], 2.5)
# move above moka pot for inspection with eye-in-hand, yaw 0
r.move_pose([-0.054, -0.264, 1.25], pk.topdown_R(0.0), 4.0)
print("DONE")
