import sys, numpy as np
from rob import *
from plan2 import *
pot=sys.argv[1]           # "A" or "B"
r=Rob("exec_"+pot)
sols=np.load("sols.npy",allow_pickle=True).item()
DUR={"pre_high":4.0,"pre":3.0,"grasp":2.5,"lift":3.0,"over":5.0,"place":3.0,"retreat":2.5,"up":2.5}
if pot=="A": DUR["pre_high"]=6.0   # big unloaded reconfiguration from ready pose

def pot_blob(c):
    P=r.depth_world("birdview",*BIRD).reshape(-1,3); P=P[np.isfinite(P).all(1)]
    d=np.linalg.norm(P[:,:2]-c,axis=1); m=(d<0.06)&(P[:,2]>0.95)&(P[:,2]<1.08)
    if m.sum()<5: return None
    return P[m,:2].mean(0).round(3), P[m,2].max().round(3), int(m.sum())

def go(step, seed_name):
    q=sols[f"{pot}_{step}"]
    dur=max(2.0, np.abs(q-r.arm_q()).max()/0.15)   # joint speed cap ~0.19 rad/s
    code,err=r.move_q(q, dur)
    p,R=r.tcp(); tgt=dict((n,(pp,t)) for n,pp,t in SEQ)[f"{pot}_{step}"][0]
    print(f"== {pot}_{step}: code={code} jerr={err:.4f} tcp={p.round(3)} tgt={tgt.round(3)} "
          f"poserr={np.linalg.norm(p-tgt)*1000:.1f}mm fingers={np.round(np.abs(r.fingers()),4)} F={r.wrench()[:3].round(1)}", flush=True)
    if code!=0 or err>0.02: raise SystemExit("MOVE FAILED")

src = A if pot=="A" else B
dst = FAR if pot=="A" else NEAR
resume = len(sys.argv)>2 and sys.argv[2]=="held"   # pot already in gripper: skip the pick
print("pot at start:", pot_blob(src), " dst:", pot_blob(dst), flush=True)
if not resume:
    f=np.abs(r.gripper(0.04))
    if min(f)<0.035: raise SystemExit("gripper did not open")
    go("pre_high", None); go("pre", None); go("grasp", None)
    print("pot before close:", pot_blob(src), flush=True)
    f=np.abs(r.gripper(0.0))
    print("fingers after close:", f, flush=True)
    # holding the pot's waist ring reads ~0.037 per finger; a miss closes to ~0.004
    if not (0.022<min(f)<0.0395): raise SystemExit("GRASP MISSED (fingers %s)"%(f,))
go("lift", None)
f=np.abs(r.fingers()); print("fingers after lift:", f, " src blob now:", pot_blob(src), flush=True)
if min(f)<0.022: raise SystemExit("POT DROPPED")
go("over", None); go("place", None)
f=r.gripper(0.04)
go("retreat", None); go("up", None)
print("RESULT dst blob:", pot_blob(dst), " src blob:", pot_blob(src), flush=True)
for cam in ("birdview","agentview","frontview"):
    r.snap(cam, f"/workspace/after_{pot}_{cam}.png")
print("DONE", flush=True)
