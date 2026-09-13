"""Open gripper and retreat along -approach by 6 cm, then up 5 cm."""
import json, sys
from rob import *
st=json.load(open("approach_state.json")); yaw=st["yaw"]
r=Robot(); R=side_grasp_R(yaw,st.get("pitch",10.0))
r.gripper(GRIP["open_m"])
ap=np.array([math.cos(math.radians(yaw)),math.sin(math.radians(yaw)),0.0])
t,_=r.tcp()
for tgt in [t-0.06*ap, t-0.06*ap+np.array([0,0,0.05])]:
    sol=r.ik_tcp(tgt,R,seed=r.arm_q()); r.move_converged([sol],2.5)
print("TCP",r.tcp()[0].round(4))
