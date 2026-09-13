import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
def go(tcp, yaw=0.0, secs=2.5, maxjump=1.2):
    q0=topdown_quat(yaw); cur=w.arm_q()
    sol=w.ik(tcp_to_hand(tcp,q0), q0, seed=cur, tries=3)
    if sol is None: raise SystemExit(f"IK failed {tcp}")
    d=np.abs(np.array(sol)-np.array(cur)).max()
    if d>maxjump: raise SystemExit(f"IK branch jump {d:.2f}; abort")
    w.move(sol, secs)
    T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
    return T
go([0.172,-0.048,0.68], secs=3.5)
print("fingers", {k:round(v,4) for k,v in w.joints().items() if "finger" in k})
go([0.172,-0.048,0.60], secs=2.5)
print("fingers", {k:round(v,4) for k,v in w.joints().items() if "finger" in k})
w.destroy_node()
