"""Pot A recovery: clamp the handle tip (hy=-y, hand tilted TILT toward +x) and drag the pot along the table.
usage: dragA.py pinch X Y Z | to X Y Z | open | grip | snap tag"""
import sys, numpy as np
from rob import *
r=Rob("dragA")
TILT=np.radians(50)
hz_p=np.array([np.sin(TILT),0,-np.cos(TILT)]); R_p=R_from_axes(hz_p,[0,-1,0])
SEEDS=[np.array([-0.08,1.53,0.19,-0.63,-0.49,3.01,1.08]),np.array([-0.08,1.51,0.19,-0.62,-0.47,2.99,1.06]),np.array([0.03,1.5,-0.04,-0.7,0.03,3.16,0.8])]
def solve(p):
    for s in [r.arm_q()]+SEEDS:
        q=r.ik(p,R_p,seed=s,at_tcp=True)
        if q is not None: return q
    raise SystemExit(f"IK FAIL {p}")
def go(p,tol=0.02,tag=""):
    q=solve(p); dur=max(2.0,np.abs(q-r.arm_q()).max()/0.15)
    code,err=r.move_q(q,dur); pp,_=r.tcp()
    print(f"== {tag}{np.round(p,3)}: code={code} jerr={err:.4f} tcp={pp.round(3)} fingers={np.round(np.abs(r.fingers()),4)} F={r.wrench()[:3].round(1)}",flush=True)
    if code!=0 or err>tol: raise SystemExit("MOVE FAILED")
cmd=sys.argv[1]
if cmd=="pinch":
    X,Y,Z=map(float,sys.argv[2:5])
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    go([X,Y,1.06],tag="pre"); go([X,Y,Z+0.04],tag="mid"); go([X,Y,Z],tag="pinch")
    f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
    if not (0.006<min(f)<0.036): raise SystemExit("PINCH MISSED")
elif cmd=="to":
    X,Y,Z=map(float,sys.argv[2:5]); go([X,Y,Z],tag="to")
    f=np.abs(r.fingers())
    if min(f)<0.005: print("!! SLIPPED (fingers closed)")
elif cmd=="open":
    print(r.gripper(0.04))
elif cmd=="grip":
    print(r.gripper(0.0))
elif cmd=="snap":
    for cam in ("agentview","sideview","robot0_eye_in_hand"): r.snap(cam,f"/workspace/{sys.argv[2]}_{cam}.png")
print("OK",flush=True)
