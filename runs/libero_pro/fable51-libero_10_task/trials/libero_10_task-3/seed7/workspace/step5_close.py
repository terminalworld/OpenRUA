import numpy as np, sys, math
from rob import *
r=Robot()
print("fingers",r.fingers())
a=np.array([0,math.sin(math.radians(45)),-math.cos(math.radians(45))])
Rp=hand_R(a,(1,0,0)); qp=quat_from_R(Rp)
TCP_OFF=0.1034
def hand_for_tcp(tcp): return np.array(tcp)-TCP_OFF*a
def go(p,q,secs,tol=0.02):
    sol=r.ik_hand(p,q)
    if sol is None: print("IK FAIL",p); return None
    code,err,n_=r.move_q_conv(sol,secs,tol=tol,retries=2)
    pos,quat,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} retries={n_} hand={pos.round(4)}")
    return pos
# 1. lift straight up from current pose
pos,quat,_=r.fk_hand()
go(pos+[0,0,0.10],quat,3.0)
# 2. pre-push above start, pitched
x0=-0.025
go(hand_for_tcp([x0,0.02,1.10]),qp,5.0)
# 3. descend
go(hand_for_tcp([x0,0.02,0.9475]),qp,3.0)
w0=r.wrench(); print("wrench0",w0.round(2))
# 4. push in steps
for y in np.arange(0.05,0.215,0.03):
    hp=go(hand_for_tcp([x0,y,0.9475]),qp,2.5,tol=0.05)
    if hp is None: break
    tcp=hp+TCP_OFF*a
    w=r.wrench(); print(f"   tcp_y={tcp[1]:.4f} (cmd {y:.3f}) wrench={w.round(2)}")
    if y-tcp[1] > 0.02:
        print("   stalled -> drawer likely closed"); break
# 5. retreat: back and up
hp=r.fk_hand()[0]
go(hp+[0,-0.03,0.10],qp,3.0)
