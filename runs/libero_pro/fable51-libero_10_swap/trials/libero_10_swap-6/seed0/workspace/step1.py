import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
q0=topdown_quat(0.0)
print("quat", q0)
T=w.fk_pose(); print("current hand (world):", np.round(T[:3,3],4)); print(np.round(T[:3,:3],3))
# 1. hover above white mug grasp point (rim, -y side)
grasp=np.array([-0.093,-0.204,0.525])
hover=grasp+[0,0,0.10]
r=w.move_pose(tcp_to_hand(hover,q0), q0, 3.0)
print("hover result", r)
T=w.fk_pose(); print("hand now:", np.round(T[:3,3],4), "tcp:", np.round(T[:3,3]+OFF*T[:3,2],4))
w.destroy_node()
