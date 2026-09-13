"""Pot A stage 1a: pinch boiler top-down (tilted), lift (pot pivots lid-down)."""
import sys, numpy as np
from rob import *
r=Rob("stageA1")
EXEC = len(sys.argv)>1 and sys.argv[1]=="go"
TILT=np.radians(float(sys.argv[2]) if len(sys.argv)>2 else 25)
hz_p=np.array([np.sin(TILT),0,-np.cos(TILT)]); hy_p=np.array([-np.cos(TILT),0,-np.sin(TILT)]); R_p=R_from_axes(hz_p,hy_p)
# boiler axis measured from wrist depth: x=0.2115, z=0.972 at y=0.108 (axial ~4cm from base, r~3.4)
C=np.array([0.2115,0.108,0.975]); T=C.copy()
steps=[("pre",   (np.r_[T[:2],1.10], R_p)),
       ("pinch", (T,  R_p)),
       ("lift",  (np.r_[T[:2],1.20], R_p))]
FWD=np.array([0.35,1.48,-0.66,-0.59,0.72,2.37,2.33])
sols={}; sd=dict(steps)
for name,seed in (("pinch",FWD),("pre",None),("lift",None)):
    p,R=sd[name]; q=r.ik(p,R,seed=seed if seed is not None else sols["pinch"],at_tcp=True)
    if q is None: print(name,"IK FAIL"); sys.exit(1)
    zs={l:r.fk(q,l)[0][2] for l in ["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7"]}
    sols[name]=q; print(f"{name}: p={p.round(3)} q={q.round(2)} minlinkz={min(zs.values()):.3f}")
print("pre->pinch dq",np.abs(sols["pre"]-sols["pinch"]).max().round(2)," pinch->lift dq",np.abs(sols["lift"]-sols["pinch"]).max().round(2))
np.save("solsA1.npy",sols)
if not EXEC: sys.exit(0)
def go(name):
    q=sols[name]; dur=max(2.0,np.abs(q-r.arm_q()).max()/0.15)
    code,err=r.move_q(q,dur); p,_=r.tcp()
    print(f"== {name}: code={code} jerr={err:.4f} tcp={p.round(3)} fingers={np.round(np.abs(r.fingers()),4)} F={r.wrench()[:3].round(1)}",flush=True)
    if code!=0 or err>0.03: raise SystemExit("MOVE FAILED")
f=np.abs(r.gripper(0.04))
if min(f)<0.035: raise SystemExit("gripper did not open")
go("pre"); go("pinch")
f=np.abs(r.gripper(0.0)); print("fingers after close:",f,flush=True)
if not (0.015<min(f)<0.036): raise SystemExit("PINCH MISSED")
go("lift")
f=np.abs(r.fingers()); print("fingers after lift:",f,flush=True)
for cam in ("agentview","sideview","frontview"): r.snap(cam,f"/workspace/A1_lift_{cam}.png")
print("DONE",flush=True)
