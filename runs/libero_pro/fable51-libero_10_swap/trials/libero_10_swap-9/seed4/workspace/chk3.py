import numpy as np
from rob import Robot
from step import dir_quat
from pathchk import check_path, in_box, MW
r = Robot("chk3")
q0 = r.q()
quat = dir_quat(30, -10)
tip = np.array([-0.156, -0.494, 0.948])
z = np.array([0, np.sin(np.radians(30)), -np.cos(np.radians(30))])
qpre = r.solve_ik(tip - 0.06*z, quat, seed=q0, tries=8)
qg = r.solve_ik(tip, quat, seed=qpre, tries=8)
print("qpre", np.round(qpre,3).tolist()); print("qg", np.round(qg,3).tolist())
for name,q in [("cur",q0),("pre",qpre),("grasp",qg)]:
    for l in ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]:
        p=r.fk_pose(q,l)[0]; print(name,l,np.round(p,3), "IN MW" if in_box(p,MW) else "")
print("path cur->pre hits:", check_path(r, q0, qpre, n=20, mug=False, verbose=False))
np.save("qpre.npy", qpre); np.save("qg.npy", qg)
