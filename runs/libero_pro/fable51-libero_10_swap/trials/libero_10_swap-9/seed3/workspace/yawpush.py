"""Push the lying mug's rim end sideways (toward -perp) to rotate it CW. args: push_dist_m [t_offset] [pitch_deg] [tip_z]"""
import numpy as np, rclpy, sys
from rob import Robot, quat_from_axes
d=float(sys.argv[1]); toff=float(sys.argv[2]) if len(sys.argv)>2 else 0.045
pitch=float(sys.argv[3]) if len(sys.argv)>3 else 20.0; tipz=float(sys.argv[4]) if len(sys.argv)>4 else 0.955
mp=np.load('mugpose.npy'); c=mp[0:3]; ax=mp[3:6]; hs=mp[12]
perp=np.array([-ax[1],ax[0],0]); side=hs if hs!=0 else 1.0
pdir=-side*perp
s,cth=np.sin(np.radians(pitch)),np.cos(np.radians(pitch))
z_h=pdir*s-np.array([0,0,cth])     # fingers point down+push dir, housing trails behind
q=quat_from_axes(z_h,ax); qv=quat_from_axes((0,0,-1),ax)
tip0=c+ax*toff+side*perp*0.062; tip0[2]=tipz
H0=tip0-0.1034*z_h; H1=H0+pdir*d
print("tip0",tip0.round(3),"H0",H0.round(3),"H1",H1.round(3))
r=Robot(); r.gripper(0.0)
above=H0+np.array([0,0,0.14])
ok=False
for t in range(3):
    r.plan_go(tuple(above),q,seed=None); p,_=r.fk()
    if np.linalg.norm(p-above)<0.01: ok=True; break
if not ok:
    # fallback: vertical hand above, then tilt in place
    av=np.array([H0[0],H0[1],1.20])
    for t in range(3):
        r.plan_go(tuple(av),qv,seed=None); p,_=r.fk()
        if np.linalg.norm(p-av)<0.01: break
    r.cart_path([(tuple(above),q)],seconds_per_m=8,min_step_t=2.0)
r.cart_path([(tuple(H0+np.array([0,0,0.06])),q),(tuple(H0),q)],seconds_per_m=10)
n=max(2,int(d/0.015))
wps=[(tuple(H0+pdir*d*(i+1)/n),q) for i in range(n)]
r.cart_path(wps,seconds_per_m=30,min_step_t=1.0)
p,_=r.fk(); print("after push hand",p.round(3),"lag",np.round(H1-p,3))
r.cart_path([(tuple(p+np.array([0,0,0.15])),q)],seconds_per_m=8)
rclpy.shutdown()
