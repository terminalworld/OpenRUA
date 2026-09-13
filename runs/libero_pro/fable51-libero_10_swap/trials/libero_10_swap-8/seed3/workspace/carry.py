"""Move the TCP along straight world segments, keeping orientation from approach_state.
Usage: python3 carry.py dx dy dz [seconds]  (relative TCP move)"""
import json, sys
from rob import *
st=json.load(open("approach_state.json"))
dx,dy,dz=map(float,sys.argv[1:4]); secs=float(sys.argv[4]) if len(sys.argv)>4 else 4.0
r=Robot(); R=side_grasp_R(st["yaw"],st.get("pitch",10.0))
t,_=r.tcp(); g0=r.finger_gap()
tgt=t+np.array([dx,dy,dz])
r.move_converged(cart_line(r,R,t,tgt,0.03,r.arm_q()),secs)
t2,_=r.tcp(); print("TCP",t2.round(4),"target",tgt.round(4),"gap",round(g0,4),"->",round(r.finger_gap(),4))
