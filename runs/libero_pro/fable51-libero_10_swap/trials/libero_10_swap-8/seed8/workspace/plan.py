import numpy as np
from rob import *

def u(az):
    th=np.radians(az); return np.array([np.cos(th),np.sin(th),0.0])
def hand_R(az_deg, hx_up=False):
    hz=u(az_deg); hy=np.array([-hz[1],hz[0],0.0])
    if hx_up: hy=-hy
    return R_from_axes(hz,hy)
P3=lambda xy,z: np.array([xy[0],xy[1],z])

A=np.array([-0.195,-0.200]); B=np.array([-0.064,0.234])
ZG=0.955; ZL=1.10; ZP=1.02
A_PLACE=np.array([0.13,-0.018]); B_PLACE=np.array([0.14,0.085])
poses={
 "A_pre":   (P3(A,ZG)-0.12*u(-45), -45),
 "A_grasp": (P3(A,ZG), -45),
 "A_lift":  (P3(A,ZL), -45),
 "A_liftrot":(P3(A,ZL), 45),
 "A_downrot":(P3(A,ZG), 45),
 "A_over":  (P3(A_PLACE,ZL), -75),
 "A_place": (P3(A_PLACE,ZP), -75),
 "B_pre":   (P3(B,ZG)-0.12*u(-45), -45),
 "B_grasp": (P3(B,ZG), -45),
 "B_lift":  (P3(B,ZL), -45),
 "B_liftrot":(P3(B,ZL), 45),
 "B_downrot":(P3(B,ZG), 45),
 "B_over":  (P3(B_PLACE,ZL), -60),
 "B_place": (P3(B_PLACE,ZP), -60),
}
if __name__=="__main__":
    r=Rob("plan")
    q0=r.arm_q()
    for name,(p,az) in poses.items():
        for hx_up in (False,True):
            q=r.ik(p,hand_R(az,hx_up),seed=q0,at_tcp=True)
            if q is None: print(f"{name} hx_up={hx_up}: IK FAIL"); continue
            zs={l: r.fk(q,l)[0][2] for l in ["panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]}
            pt,_=r.tcp(q)
            print(f"{name} hx_up={hx_up}: q={q.round(2)} tcp={pt.round(3)} minlinkz={min(zs.values()):.3f} ({min(zs,key=zs.get)})")
