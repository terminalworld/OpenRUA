import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
def report():
    T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
    return T
def fingers(): return {k:round(v,4) for k,v in w.joints().items() if "finger" in k}
def go(tcp, secs=3.0, seed=None):
    q0=topdown_quat(0.0); cur=w.arm_q()
    sol=w.ik(tcp_to_hand(tcp,q0),q0,seed=seed or cur,tries=3)
    if sol is None: raise SystemExit(f"IK failed {tcp}")
    print("target",tcp,"jump",np.round(np.abs(np.array(sol)-np.array(cur)).max(),3), flush=True)
    for i in range(3):
        code,err=w.move(sol,secs)
        if err<0.01: break
    report(); print("fingers",fingers(), flush=True)
    return sol
go([-0.212,0.017,0.65], 2.5)          # lift straight up
go([0.0,0.09,0.70], 3.0)              # waypoint, high over the red mug region
go([0.19,0.15,0.62], 3.0)             # above the destination
w.destroy_node()
