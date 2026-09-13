import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('door'); prev=np.array(r.arm_q())
H=np.array([-0.18,0.265]); RHO=0.16; OFF=0.022; ALPHA=np.deg2rad(35); ZT=1.045
def pose(th_deg, extra=0.0):
    th=np.deg2rad(th_deg); d=np.array([np.cos(th),np.sin(th),0.0]); n_out=np.array([np.sin(th),-np.cos(th),0.0])
    tcp=np.array([*(H+RHO*d[:2]+(OFF+extra)*n_out[:2]),ZT])
    z=-np.cos(ALPHA)*np.array([0,0,1.0])-np.sin(ALPHA)*n_out; x=np.cross(d,z)
    return tcp,hand_quat(z,x)
def go(tcp,q,secs,n=40):
    global prev
    s=best_ik(r,tcp,q,seed=prev,prefer=prev,n=n,avoid=lambda L: any(L[k][2]<1.0 for k in ('panda_link6','panda_link7','panda_hand')))
    if s is None: print('no ik',np.round(tcp,3)); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'wrench',np.round(w,2)); return w
mode=sys.argv[1]
if mode=='start':
    th0=float(sys.argv[2])
    tcp,q=pose(th0,0.06)
    # via point high above
    go(np.array([-0.15,0.05,1.30]),q,3.0)
    go(tcp,q,3.0)
    tcp,q=pose(th0,0.0); go(tcp,q,2.0)
elif mode=='sweep':
    th0=float(sys.argv[2]); th1=float(sys.argv[3]); step=float(sys.argv[4])
    wprev=r.wrench()[:3]
    for th in np.arange(th0+step,th1+1e-6,step):
        tcp,q=pose(th); w=go(tcp,q,1.2)
        dw=np.linalg.norm(w-wprev); wprev=w
        if dw>float(sys.argv[5]) if len(sys.argv)>5 else dw>8: print('FORCE JUMP at',th,dw); break
elif mode=='retreat':
    th=float(sys.argv[2]); tcp,q=pose(th,0.06); go(tcp,q,2.0); tcp[2]=1.25; go(tcp,q,2.5)
