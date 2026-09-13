"""Pot A (fallen beside stove, x~0.27-0.35): pinch the handle tip (x~0.245-0.256, z~0.975-0.99)
lengthwise in y with a 50deg-tilted hand at the reach frontier, then lift so the pot hangs from its handle."""
import sys, numpy as np
from rob import *
r=Rob("stageA1h")
EXEC = len(sys.argv)>1 and sys.argv[1]=="go"
TILT=np.radians(float(sys.argv[2]) if len(sys.argv)>2 else 50)
hz_p=np.array([np.sin(TILT),0,-np.cos(TILT)]); R_p=R_from_axes(hz_p,[0,-1,0])
X,Y=0.246,0.0125
steps=[("pre",  np.array([X,Y,1.08])),("mid",np.array([X,Y,1.02])),("pinch",np.array([X,Y,0.98])),
       ("lift1",np.array([X,Y,1.05])),("lift2",np.array([0.16,Y,1.15]))]
seeds=[np.array([-0.08,1.51,0.17,-0.66,-0.48,3.03,1.09]),np.array([0.03,1.5,-0.04,-0.7,0.03,3.16,0.8]),
       np.array([0.16,1.65,-0.42,-0.43,0.63,2.99,0.61]),np.array([0.36,1.48,-0.66,-0.61,0.74,2.43,0.75])]
sols={}
order=["pinch","mid","pre","lift1","lift2"]; sd=dict(steps)
for name in order:
    q=None
    for s in ([sols["pinch"]] if "pinch" in sols else [])+seeds:
        q=r.ik(sd[name],R_p,seed=s,at_tcp=True)
        if q is not None: break
    if q is None: print(name,"IK FAIL"); sys.exit(1)
    sols[name]=q; print(f"{name}: p={sd[name].round(3)} q={q.round(2)}")
for a,b in zip(order[:-1],order[1:]): print(f"dq {a}->{b}: {np.abs(sols[a]-sols[b]).max():.2f}")
np.save("solsA1h.npy",sols)
if not EXEC: sys.exit(0)
def go(name,tol=0.02):
    q=sols[name]; dur=max(2.0,np.abs(q-r.arm_q()).max()/0.15)
    code,err=r.move_q(q,dur); p,_=r.tcp()
    print(f"== {name}: code={code} jerr={err:.4f} tcp={p.round(3)} fingers={np.round(np.abs(r.fingers()),4)} F={r.wrench()[:3].round(1)}",flush=True)
    if code!=0 or err>tol: raise SystemExit("MOVE FAILED")
f=np.abs(r.gripper(0.04))
if min(f)<0.035: raise SystemExit("gripper did not open")
go("pre"); go("mid"); go("pinch")
r.snap("robot0_eye_in_hand","/workspace/A1h_pinch_eye.png")
f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
if not (0.007<min(f)<0.036): raise SystemExit("PINCH MISSED")
go("lift1")
f=np.abs(r.fingers()); print("fingers after lift1:",f,flush=True)
if min(f)<0.006: raise SystemExit("DROPPED")
go("lift2")
f=np.abs(r.fingers()); print("fingers after lift2:",f,flush=True)
for cam in ("agentview","sideview","frontview"): r.snap(cam,f"/workspace/A1h_lift_{cam}.png")
print("DONE",flush=True)
