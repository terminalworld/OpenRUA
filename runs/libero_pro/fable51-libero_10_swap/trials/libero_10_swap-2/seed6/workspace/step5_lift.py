import numpy as np, pk
r = pk.Robot("lift")
q = r.joints(); T = pk.fk(q, True); print("tcp", T[:3,3].round(4), "fingers", np.round(r.fingers(),4))
pos = T[:3,3].copy(); pos[2] = 1.22
for _ in range(3):
    qq,_,_ = r.move_pose(pos, pk.topdown_R(np.pi/2), 2.5)
    if np.abs(r.joints()-qq).max() < 0.01: break
print("wrench", r.wrench()[0].round(2))
print("DONE")
