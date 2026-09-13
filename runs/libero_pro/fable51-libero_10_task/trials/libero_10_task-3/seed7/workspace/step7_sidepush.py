"""Close the drawer: horizontal hand approaching along +x from the left of
the cabinet, closed fingers push the panel's front face with their +y side.
Hand body stays at x < -0.14 (outside the cabinet), fingers at the panel's
left end (clear of the bowl and handle)."""
import numpy as np, sys, math
from rob import *
r=Robot()
print("fingers",r.fingers())
TCP_OFF=0.1034
a=np.array([1.0,0,0])                      # approach +x (horizontal)
Rh=hand_R(a,(0,1,0)); qh=quat_from_R(Rh)    # closing axis y
def hand_for_tcp(tcp): return np.array(tcp)-TCP_OFF*a
def go(p,q,secs,tol=0.02,retries=3):
    sol=r.ik_hand(p,q)
    if sol is None: print("IK FAIL",p); return None
    code,err,n_=r.move_q_conv(sol,secs,tol=tol,retries=retries)
    pos,quat,_=r.fk_hand()
    print(f"go {np.round(p,3)} -> code={code} jerr={err:.4f} retries={n_} hand={pos.round(4)}")
    return pos
x0=-0.072; z0=0.963; HALF=0.0143           # finger block half-thickness along closing axis
pos,quat,_=r.fk_hand(); print("start hand",pos.round(3))
# 1. up from the current pose
go(pos+[0,0,0.10],quat,3.0)
# 2. above the pre-push point, horizontal orientation
hp=go(hand_for_tcp([x0,0.10,1.12]),qh,5.0)
if hp is None: sys.exit(1)
# 3. descend
hp=go(hand_for_tcp([x0,0.10,z0]),qh,3.0)
if hp is None: sys.exit(1)
w0=r.wrench(); print("wrench0",w0.round(2))
# 4. push +y in steps; panel face should end at ~0.222 -> tcp_y ~0.208
for y in np.arange(0.12,0.2181,0.02):
    hp=go(hand_for_tcp([x0,y,z0]),qh,2.5,tol=0.02,retries=3)
    if hp is None: break
    tcp=hp+TCP_OFF*a
    w=r.wrench(); print(f"   tcp_y={tcp[1]:.4f} (cmd {y:.3f}) panel~{tcp[1]+HALF:.4f} wrench={w.round(2)}")
    if y-tcp[1] > 0.012 or abs(w[1])>15:
        print("   stalled"); break
# 5. retreat: -y then up
hp=r.fk_hand()[0]
go(hp+[0,-0.04,0],qh,2.0)
go(hp+[0,-0.04,0.12],qh,3.0)
print("done; fingers",r.fingers())
