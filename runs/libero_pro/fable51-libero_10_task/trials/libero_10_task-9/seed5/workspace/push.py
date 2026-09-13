import numpy as np, sys
sys.path.insert(0,'.')
from rob import *
r=Robot('push'); prev=np.array(r.arm_q())
pitch=np.deg2rad(float(sys.argv[2])) if len(sys.argv)>2 else 0.0   # down-tilt of hand z
z3=np.array([0,np.cos(pitch),-np.sin(pitch)]); y3=np.array([1.0,0,0]); x3=np.cross(y3,z3)
qh3=hand_quat(z3,x3)
def door_only(L):
    for k in ('panda_link5','panda_link6','panda_link7','panda_hand'):
        p=L[k]
        if p[2]<0.95: return False
        # open door: strip ~3cm thick from hinge (-0.165,0.285) to free edge (-0.25,0.02)
        yc=min(max(p[1],0.0),0.30); xd=-0.165-0.32*(0.285-yc)+0.015
        dx=max(0.0,p[0]-xd) if p[0]>xd else 0.0
        if p[0]<xd: dx=0.0   # behind the door face: definitely bad
        dy=max(0.0-p[1],0,p[1]-0.30); dz=max(0,p[2]-1.12)
        if p[0]<xd+0.05 and dy<0.05 and dz<0.05: return False
    return True
def go(pt,secs):
    global prev
    s=best_ik(r,np.array(pt),qh3,seed=prev,prefer=prev,avoid=lambda L: not door_only(L))
    if s is None: print('no ik',pt); sys.exit(1)
    r.move_joints(s,secs); prev=np.array(r.arm_q())
    w=r.wrench()[:3]; print('tcp',np.round(r.tcp()[0],4),'fingers',np.round(r.fingers(),4),'wrench',np.round(w,2)); return w
mode=sys.argv[1]
if mode=='look':
    # via point above, then horizontal hand at entrance height
    go((-0.055,0.05,1.10),3.0)
    go((-0.055,0.10,1.00),2.5)
elif mode=='to':
    x,y,z=map(float,sys.argv[3:6]); go((x,y,z),2.5)
elif mode=='steps':
    z=float(sys.argv[3]); x=float(sys.argv[4]); ys=list(map(float,sys.argv[5:]))
    w0=r.wrench()[:3]
    for y in ys:
        w=go((x,y,z),1.5); dw=np.linalg.norm(w-w0); print('dw',np.round(dw,2))
        if dw>float(__import__("os").environ.get("DWMAX","3.0")): print('CONTACT'); break
