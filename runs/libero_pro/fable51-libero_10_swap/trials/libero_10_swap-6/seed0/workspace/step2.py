import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
q0=topdown_quat(0.0)
grasp=np.array([-0.093,-0.204,0.525])
cur=w.arm_q()
sol=w.ik(tcp_to_hand(grasp,q0), q0, seed=cur, tries=3)
print("descend sol", np.round(sol,3), "diff", np.round(np.array(sol)-np.array(cur),3))
if np.abs(np.array(sol)-np.array(cur)).max() > 1.0: raise SystemExit("IK branch jump; abort")
w.move(sol, 2.5)
T=w.fk_pose(); print("tcp:", np.round(T[:3,3]+OFF*T[:3,2],4))
w.gripper(0.0)
w.destroy_node()
