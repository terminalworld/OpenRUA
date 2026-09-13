import numpy as np, rclpy, sys, time
from rob import Robot, quat_from_axes
from handcheck import pose_from
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0]); Z=np.array([0,0,1.0])
e=np.radians(26); rho=np.cos(e)*perp+np.sin(e)*Z; tau=-np.sin(e)*perp+np.cos(e)*Z
ph=np.radians(28); zh=-np.cos(ph)*tau-np.sin(ph)*rho; pad=c+0.027*a+0.058*rho+0.007*tau; H0=pad-0.093*zh
yh=-a; q=quat_from_axes(zh,yh)
r=Robot(); p,_=r.fk(); print("start hand",np.round(p,3),"fingers",r.fingers(),"q",np.round(r.arm_q(),2))
wps=[H0+np.array([0,0,z]) for z in (0.10,0.06,0.03,0.015,0.0)]
rc=r.cart_path([(w,q) for w in wps],seconds_per_m=8.0,min_step_t=0.8,max_jump=0.6,avoid=False); print("descend:",rc is not None)
p,_=r.fk(); print("hand at",np.round(p,3),"err",np.round(p-H0,4))
if rc is None: sys.exit()
r.gripper(0.0); time.sleep(0.5); f=r.fingers(); print("fingers after close",f)
np.save('pinch_state.npy',np.array([*H0,*q]))
