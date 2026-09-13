"""Hand-body sweep of pot A's handle: hz leaning back (toward -x) by t deg, hy=+x
so the hand body's +hy end (0.10 from TCP) reaches over the handle at x~0.30.
  python3 sweep.py test                 -> IK feasibility grid
  python3 sweep.py go X Y Z T [dur]     -> move TCP there (fingers closed first)
  python3 sweep.py pull X Y Z T         -> same as go (used for the -x drag)
  python3 sweep.py snap TAG
"""
import sys, numpy as np
from rob import *
r=Rob("sweep")
def Rlean(t):
    t=np.radians(t); hz=np.array([-np.sin(t),0,-np.cos(t)]); hy=np.array([np.cos(t),0,-np.sin(t)])
    return R_from_axes(hz,hy)
SEEDS=[[0.0,-0.8,0.0,-2.5,0.0,1.7,0.785],[0.36,1.48,-0.66,-0.61,0.74,2.43,0.75],
       [-0.08,1.51,0.17,-0.66,-0.48,3.03,1.09],[0.0,1.2,0.0,-1.2,0.0,2.4,0.785],
       [0.0,1.5,0.0,-0.8,0.0,2.3,0.785],[0.0,1.6,0.0,-0.5,0.0,2.1,-0.785]]
def ik(p,R):
    for s in [r.arm_q()]+[np.array(x) for x in SEEDS]:
        q=r.ik(p,R,seed=s,at_tcp=True)
        if q is not None: return q
    return None
def hand_end(p,R):  # +hy end of hand body, at hz=-0.045 and -0.11 from TCP
    return [(p+0.10*R[:,1]+d*R[:,2]).round(3) for d in (-0.045,-0.11)]
cmd=sys.argv[1] if len(sys.argv)>1 else ""
if cmd=="test":
    for t in (0,10,20,30):
        for x in (0.17,0.19,0.21):
            for z in (0.94,0.96):
                R=Rlean(t); p=np.array([x,-0.05,z]); q=ik(p,R)
                if q is None: print(f"t={t} x={x} z={z}: FAIL"); continue
                zs={l:r.fk(q,l)[0].round(3) for l in ["panda_link6","panda_link7"]}
                print(f"t={t} x={x} z={z}: q={q.round(2)} end={hand_end(p,R)} l7={zs['panda_link7']}")
elif cmd in ("go","pull"):
    x,y,z,t=map(float,sys.argv[2:6]); dur=float(sys.argv[6]) if len(sys.argv)>6 else None
    R=Rlean(t); p=np.array([x,y,z]); q=ik(p,R)
    if q is None: raise SystemExit("IK FAIL")
    d=dur or max(2.5,np.abs(q-r.arm_q()).max()/0.12)
    code,err=r.move_q(q,d); pt,Rn=r.tcp()
    print(f"{cmd}: code={code} err={err:.4f} tcp={pt.round(3)} hz={Rn[:,2].round(2)} hy={Rn[:,1].round(2)} F={r.wrench()[:3].round(1)} fingers={np.round(np.abs(r.fingers()),4)}",flush=True)
elif cmd=="grip":
    print(r.gripper(0.0))
elif cmd=="open":
    print(r.gripper(0.04))
elif cmd=="snap":
    for cam in ("agentview","sideview","frontview"): r.snap(cam,f"/workspace/{sys.argv[2]}_{cam}.png")
    print("snapped")
