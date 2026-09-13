import numpy as np, rclpy, sys, time
from handcheck import *
from rob import Robot, quat_from_axes
mp=np.load('mugpose.npy'); c=mp[0:3]; a=mp[3:6]; perp=np.array([-a[1],a[0],0]); Z=np.array([0,0,1.0])
e=np.radians(26); rho=np.cos(e)*perp+np.sin(e)*Z; tau=-np.sin(e)*perp+np.cos(e)*Z
ph=np.radians(28); zh=-np.cos(ph)*tau-np.sin(ph)*rho; pad=c+0.027*a+0.058*rho+0.007*tau; H0=pad-0.093*zh
yh=-a; Rm=pose_from(zh,yh); q=quat_from_axes(zh,yh)
print("H0",np.round(H0,3),"zh",np.round(zh,3))
check(H0,Rm,gap_half=0.04,mug=(c,a,perp,rho))
if len(sys.argv)<2 or sys.argv[1]!='go': sys.exit()
r=Robot()
r.gripper(0.04); time.sleep(0.5); print("fingers",r.fingers())
seed=[0.09,1.27,-1.12,-1.48,1.17,1.97,2.29]
above=H0+np.array([0,0,0.15])
rc=r.plan_go(above,q,seed=seed); print("plan_go above:",rc is not None)
if rc is None:
    rc=r.plan_go(above,q,seed=seed); print("retry:",rc is not None)
    if rc is None: sys.exit("plan failed")
print("hand at",np.round(r.fk()[0],3))
# descend in steps
wps=[H0+np.array([0,0,z]) for z in (0.10,0.06,0.03,0.015,0.0)]
rc=r.cart_path([(w,q) for w in wps],seconds_per_m=8.0,min_step_t=0.8,max_jump=0.6,avoid=False); print("descend:",rc is not None)
p,_=r.fk(); print("hand at",np.round(p,3),"err",np.round(p-H0,4))
r.gripper(0.0); time.sleep(0.5); f=r.fingers(); print("fingers after close",f)
np.save('pinch_state.npy',np.array([*H0,*q]))
