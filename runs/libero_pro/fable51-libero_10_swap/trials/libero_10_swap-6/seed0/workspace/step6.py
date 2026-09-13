import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
def report():
    T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
q0=topdown_quat(0.0)
cur=w.arm_q()
grasp=[-0.212,0.017,0.445]; hover=[-0.212,0.017,0.62]; way=[-0.03,-0.02,0.78]
qg=w.ik(tcp_to_hand(grasp,q0),q0,seed=cur,tries=3); print("grasp q",np.round(qg,3))
qh=w.ik(tcp_to_hand(hover,q0),q0,seed=qg,tries=3); print("hover q",np.round(qh,3), "diff to grasp", np.round(np.abs(np.array(qh)-np.array(qg)).max(),3))
qw=w.ik(tcp_to_hand(way,q0),q0,seed=cur,tries=3); print("way q",np.round(qw,3))
w.move(qw, 3.0); report()
w.move(qh, 3.5); report()
w.destroy_node()
