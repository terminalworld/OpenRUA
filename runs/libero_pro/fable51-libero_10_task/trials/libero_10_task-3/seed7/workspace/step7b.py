import numpy as np, sys, math
from rob import *
r=Robot()
TCP_OFF=0.1034
a=np.array([1.0,0,0]); qh=quat_from_R(hand_R(a,(0,1,0)))
def hand_for_tcp(tcp): return np.array(tcp)-TCP_OFF*a
def go(p,q,secs,tol=0.02,retries=3):
    sol=r.ik_hand(p,q)
    if sol is None: print("IK FAIL",p); return None
    code,err,n_=r.move_q_conv(sol,secs,tol=tol,retries=retries)
    pos,quat,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} retries={n_} hand={pos.round(4)}")
    return pos
x0=-0.072; z0=0.963; HALF=0.0143
go(hand_for_tcp([x0,0.17,z0]),qh,3.0)
print("wrench0",r.wrench().round(2))
for y in [0.195,0.205,0.21,0.215,0.22]:
    hp=go(hand_for_tcp([x0,y,z0]),qh,2.0)
    if hp is None: break
    tcp=hp+TCP_OFF*a; w=r.wrench()
    print(f"   tcp_y={tcp[1]:.4f} (cmd {y:.3f}) panel~{tcp[1]+HALF:.4f} wrench={w.round(2)}")
    if y-tcp[1] > 0.008 or abs(w[1])>15: print("   stalled"); break
hp=r.fk_hand()[0]
go(hp+[0,-0.04,0],qh,2.0); go(hp+[0,-0.04,0.12],qh,3.0)
print("done")
