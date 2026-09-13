"""Re-send the TCP target from approach_state.json until the arm converges."""
import json, sys
from rob import *
st=json.load(open("approach_state.json"))
r=Robot(); R=side_grasp_R(st["yaw"],st.get("pitch",10.0))
target=np.array(st["P3"])
for i in range(4):
    sol=r.ik_tcp(target,R,seed=r.arm_q())
    code,err=r.move_joints([sol],2.0)
    t,_=r.tcp(); print("TCP",t.round(4),"err",np.round(t-target,4))
    if err<0.003: break
