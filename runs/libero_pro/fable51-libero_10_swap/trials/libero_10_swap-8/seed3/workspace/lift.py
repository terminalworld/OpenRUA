import json, sys
from rob import *
st=json.load(open("approach_state.json")); yaw=st["yaw"]
dz=float(sys.argv[1]) if len(sys.argv)>1 else 0.15
r=Robot(); R=side_grasp_R(yaw,st.get("pitch",10.0))
t,_=r.tcp()
wps=[]; seed=r.arm_q()
n=max(1,int(dz/0.03))
for i in range(1,n+1):
    tgt=t+np.array([0,0,dz*i/n]); sol=r.ik_tcp(tgt,R,seed=seed); wps.append(sol); seed=sol
r.move_converged(wps,4.0)
print("TCP",r.tcp()[0].round(4),"gap",round(r.finger_gap(),4))
