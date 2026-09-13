import numpy as np
from rob import *

def u(az):
    th=np.radians(az); return np.array([np.cos(th),np.sin(th),0.0])
def hand_R(az_deg):
    hz=u(az_deg); hy=np.array([-hz[1],hz[0],0.0])
    return R_from_axes(hz,hy)
P3=lambda xy,z: np.array([xy[0],xy[1],z])

A=np.array([-0.195,-0.200]); B=np.array([-0.064,0.234])
ZG=0.975          # TCP height at grasp: finger band [TCP-0.025,TCP-0.004] sits in waist 0.950-0.971; pot bottom 0.081 below TCP
ZL=1.17           # transport height (pot bottom 1.089 > other pot top 1.057)
ZP=1.026          # release height (pot bottom ~1.2 cm above burner grate 0.933)
FAR=np.array([0.23,0.05]);  TH_FAR=-30
NEAR=np.array([0.125,0.05]); TH_NEAR=-45
PRE=0.12; RET=0.10

def seq_pick(c, th):
    return [("pre_high", P3(c,ZL)-PRE*u(th), th),
            ("pre",      P3(c,ZG)-PRE*u(th), th),
            ("grasp",    P3(c,ZG), th),
            ("lift",     P3(c,ZL), th)]
def seq_place(c, th):
    return [("over",   P3(c,ZL), th),
            ("place",  P3(c,ZP), th),
            ("retreat",P3(c,ZP)-RET*u(th), th),
            ("up",     P3(c,ZL)-RET*u(th), th)]
SEQ = ([("A_"+n,p,t) for n,p,t in seq_pick(A,-45)] +
       [("A_"+n,p,t) for n,p,t in seq_place(FAR,TH_FAR)] +
       [("B_"+n,p,t) for n,p,t in seq_pick(B,-45)] +
       [("B_"+n,p,t) for n,p,t in seq_place(NEAR,TH_NEAR)])

if __name__=="__main__":
    r=Rob("plan2")
    prev=r.arm_q(); print("current", prev.round(2))
    lim=np.array(M["actuators"][0]["limits_rad"])
    sols={}
    for name,p,th in SEQ:
        q=r.ik(p,hand_R(th),seed=prev,at_tcp=True)
        if q is None:
            print(f"{name:10s} FAIL"); continue
        dq=np.abs(q-prev).max(); marg=np.minimum(q-lim[:,0],lim[:,1]-q).min()
        print(f"{name:10s} q={q.round(2)} dq_max={dq:.2f} lim_margin={marg:.2f}")
        prev=q; sols[name]=q
    np.save("sols.npy", sols, allow_pickle=True)
