"""Press down on the jammed bottle so the drawer-front corner it rests on is
pushed back (-y); the bottle should then drop onto the drawer floor."""
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
xp=float(sys.argv[1]) if len(sys.argv)>1 else -0.02
yp=float(sys.argv[2]) if len(sys.argv)>2 else 0.20
pos,quat,_=r.fk_hand(); print("start hand",pos.round(3))
# above the bottle
go(hand_for_tcp([xp,yp,1.07]),qp,4.0)
w0=r.wrench(); print("wrench0",w0.round(2))
for z in [1.03,1.01,1.00,0.99,0.98,0.97,0.96]:
    hp=go(hand_for_tcp([xp,yp,z]),qp,2.0,tol=0.05)
    if hp is None: break
    tcp=hp+TCP_OFF*a
    w=r.wrench(); print(f"   tcp={tcp.round(4)} (cmd z {z:.3f}) wrench={w.round(2)}")
    if tcp[2]-z > 0.012:
        print("   blocked"); break
# retreat up
hp=r.fk_hand()[0]
go(hp+[0,0,0.08],qp,3.0)
print("done; fingers",r.fingers())
