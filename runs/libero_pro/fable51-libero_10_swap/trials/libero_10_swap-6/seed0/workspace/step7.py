import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
def report():
    T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
q0=topdown_quat(0.0); cur=w.arm_q()
qg=w.ik(tcp_to_hand([-0.212,0.017,0.445],q0),q0,seed=cur,tries=3); print("grasp q",np.round(qg,3),"jump",np.round(np.abs(np.array(qg)-np.array(cur)).max(),3))
for i in range(3):
    code,err=w.move(qg,3.0)
    if err<0.01: break
report()
w.gripper(0.0)
w.destroy_node()
