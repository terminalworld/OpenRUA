"""Pot B: lid-rim pinch from the lid end (pot lying on the table), lift, hang base-down.
  python3 potB.py grasp        -> open, pre, grasp, close, small lift; prints readings
  python3 potB.py hang         -> lift high and rotate hz down (pot hangs base-down)
  python3 potB.py to X Y Z [hyaz]  -> move TCP with hz down, hy at azimuth hyaz deg (default 0 = +x)
  python3 potB.py fingers | open | snap TAG
"""
import sys, numpy as np
from rob import *
r=Rob("potB")
# pot B axis (base->lid) from birdview: base end (-0.09,0.257), lid band (-0.2135,0.297), z 0.933
U=np.array([-1.0,0.022,0.0]); U/=np.linalg.norm(U)
LID=np.array([-0.067,0.2375,0.933])+0.125*U
HZ=-U                                   # approach from the lid end, pointing at the base
V1=np.array([-U[1],U[0],0.0])           # horizontal, perpendicular to the axis (toward +y side)
al=np.radians(0); HY=np.cos(al)*V1+np.sin(al)*np.array([0,0,1.0])
R_G=R_from_axes(HZ,HY)
def R_down(hyaz):
    return R_from_axes([0,0,-1],[np.cos(np.radians(hyaz)),np.sin(np.radians(hyaz)),0])
SEEDS=[np.array(s) for s in ([0,-0.8,0,-2.5,0,1.7,0.785],[0.5,0.6,0,-2.0,0,2.6,0.8],[0.8,0.3,-0.3,-2.2,0.3,2.5,1.5],
       [0.6,0.9,-0.4,-1.6,0.5,2.4,0.0],[0.7,0.5,0.2,-2.3,-0.4,2.8,2.0],[0.36,1.48,-0.66,-0.61,0.74,2.43,0.75])]
def solve(p,R):
    for s in [r.arm_q()]+SEEDS:
        q=r.ik(p,R,seed=s,at_tcp=True)
        if q is not None: return q
    raise SystemExit(f"IK FAIL {np.round(p,3)}")
def go(p,R,tag="",tol=0.02,speed=0.15):
    q=solve(p,R); dur=max(2.0,np.abs(q-r.arm_q()).max()/speed)
    code,err=r.move_q(q,dur); pp,RR=r.tcp()
    print(f"== {tag}{np.round(p,3)}: code={code} jerr={err:.4f} tcp={pp.round(3)} hz={RR[:,2].round(2)} fingers={np.round(np.abs(r.fingers()),4)}",flush=True)
    if code!=0 or err>tol: raise SystemExit("MOVE FAILED")
cmd=sys.argv[1]
if cmd=="grasp":
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    print("hz",HZ.round(3),"hy",HY.round(3),"hx",R_G[:,0].round(3))
    go(LID-0.12*HZ+np.array([0,0,0.08]),R_G,"high ")
    go(LID-0.12*HZ,R_G,"pre ")
    go(LID-0.05*HZ,R_G,"mid ")
    go(LID,R_G,"grasp ",tol=0.03)
    f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
    if not (0.02<min(f)<0.0395): raise SystemExit("GRASP MISSED")
    go(LID+np.array([0,0,0.06]),R_G,"lift6 ")
    print("fingers after lift:",np.round(np.abs(r.fingers()),4))
elif cmd=="hang":
    p,R=r.tcp()
    go(np.array([p[0],p[1],1.15]),R,"up ")
    # rotate hz from horizontal to down in two steps about hy, keeping TCP fixed
    hz0=R[:,2]; hy0=R[:,1]
    for ang in (45,90):
        a=np.radians(ang); hz=np.cos(a)*hz0+np.sin(a)*np.array([0,0,-1.0]); hz-=hy0*hz.dot(hy0)
        go(np.array([p[0],p[1],1.15]),R_from_axes(hz,hy0),f"rot{ang} ",speed=0.12)
    print("fingers:",np.round(np.abs(r.fingers()),4))
elif cmd=="to":
    X,Y,Z=map(float,sys.argv[2:5]); hyaz=float(sys.argv[5]) if len(sys.argv)>5 else 0.0
    go(np.array([X,Y,Z]),R_down(hyaz),"to ",speed=0.12)
    f=np.abs(r.fingers())
    if min(f)<0.01: print("!! POT B LOST")
elif cmd=="fingers":
    print(np.round(np.abs(r.fingers()),4))
elif cmd=="open":
    print(r.gripper(0.04))
elif cmd=="snap":
    for cam in ("agentview","sideview","frontview"): r.snap(cam,f"/workspace/{sys.argv[2]}_{cam}.png")
print("OK",flush=True)
if cmd=="grasp2":
    # top-down pinch across the lid rim: hz down, hy horizontal perpendicular to the pot axis
    C=np.array([-0.067,0.2375,0.938])+0.13*U
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    Rt=None
    for hy in (V1,-V1):
        R=R_from_axes([0,0,-1],hy)
        try: solve(C,R); Rt=R; break
        except SystemExit: pass
    if Rt is None: raise SystemExit("no IK for top pinch")
    print("hx",Rt[:,0].round(3),"hy",Rt[:,1].round(3))
    go(C+np.array([0,0,0.15]),Rt,"high ")
    go(C+np.array([0,0,0.05]),Rt,"mid ")
    go(C,Rt,"grasp ",tol=0.03)
    f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
    if not (0.02<min(f)<0.0395): raise SystemExit("GRASP MISSED")
    go(C+np.array([0,0,0.05]),Rt,"lift5 ",speed=0.08)
    go(C+np.array([0,0,0.22]),Rt,"lift22 ",speed=0.08)
    print("fingers after lift:",np.round(np.abs(r.fingers()),4))
    print("OK",flush=True)
if cmd=="grasp3":
    # pinch across the lid rim with the hand tilted 20 deg toward +y so the -y pad passes under the elevated handle
    C=np.array([-0.197,0.24,0.933]); b=np.radians(20)
    Rt=R_from_axes([0,np.sin(b),-np.cos(b)],[0,np.cos(b),np.sin(b)])
    print("hx",Rt[:,0].round(3),"hy",Rt[:,1].round(3),"hz",Rt[:,2].round(3))
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    go(C+np.array([0,0,0.15]),Rt,"high ")
    go(C+np.array([0,0,0.05]),Rt,"mid ")
    go(C+np.array([0,0,0.02]),Rt,"near ",tol=0.03)
    go(C,Rt,"grasp ",tol=0.03)
    f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
    if not (0.02<min(f)<0.0395): raise SystemExit("GRASP MISSED")
    go(C+np.array([0,0,0.05]),Rt,"lift5 ",speed=0.08)
    go(C+np.array([0,0,0.22]),Rt,"lift22 ",speed=0.08)
    print("fingers after lift:",np.round(np.abs(r.fingers()),4))
    print("OK",flush=True)
if cmd=="grasp4":
    b=np.radians(float(sys.argv[2]) if len(sys.argv)>2 else 25); d=float(sys.argv[3]) if len(sys.argv)>3 else 0.012
    hz=np.array([0,np.sin(b),-np.cos(b)]); hy=np.array([0,np.cos(b),np.sin(b)])
    Rt=R_from_axes(hz,hy); C=np.array([-0.197,0.24,0.933])+d*hz
    print("C",C.round(3),"hy",hy.round(3),"hz",hz.round(3))
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    go(C+np.array([0,0,0.15]),Rt,"high ")
    go(C+np.array([0,0,0.05]),Rt,"mid ")
    go(C+np.array([0,0,0.02]),Rt,"near ",tol=0.03)
    go(C,Rt,"grasp ",tol=0.03)
    f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
    if not (0.02<min(f)<0.0395): raise SystemExit("GRASP MISSED")
    go(C+np.array([0,0,0.05]),Rt,"lift5 ",speed=0.08)
    go(C+np.array([0,0,0.22]),Rt,"lift22 ",speed=0.08)
    print("fingers after lift:",np.round(np.abs(r.fingers()),4))
    print("OK",flush=True)
if cmd=="grasp5":
    # approach along the pot axis from the lid end, hand pitched 25 deg down so the wrist clears the table;
    # pads (planes y=const) clamp the lid rim's equator; handle strut (elevated, -y side) stays 2 cm above the pads
    b=np.radians(25); hz=np.array([np.cos(b),0,-np.sin(b)]); C=np.array([-0.195,0.24,0.933])
    Rt=None
    for hy in ([0,1.0,0],[0,-1.0,0]):
        R=R_from_axes(hz,hy)
        try: solve(C,R); Rt=R; break
        except SystemExit: pass
    if Rt is None: raise SystemExit("no IK for lid-end pinch")
    print("hx",Rt[:,0].round(3),"hy",Rt[:,1].round(3),"hz",Rt[:,2].round(3))
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    go(C-0.10*hz+np.array([0,0,0.10]),Rt,"high ")
    go(C-0.10*hz,Rt,"pre ")
    go(C-0.04*hz,Rt,"mid ",tol=0.03)
    go(C,Rt,"grasp ",tol=0.03)
    f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
    if not (0.02<min(f)<0.0395): raise SystemExit("GRASP MISSED")
    go(C+np.array([0,0,0.03]),Rt,"lift3 ",speed=0.08)
    go(C+np.array([0,0,0.10]),Rt,"lift10 ",speed=0.08)
    go(C+np.array([0,0,0.22]),Rt,"lift22 ",speed=0.08)
    print("fingers after lift:",np.round(np.abs(r.fingers()),4))
    print("OK",flush=True)
