import numpy as np, handcheck as h, rob, sys, time
c=np.array([-0.186,-0.443,0.945]); a=np.array([0.423,-0.906,0.0]); a/=np.linalg.norm(a)
perp=np.array([-a[1],a[0],0.0]); Z=np.array([0,0,1.0])
eps=np.radians(27.0); rho=np.cos(eps)*perp+np.sin(eps)*Z; tau=-np.sin(eps)*perp+np.cos(eps)*Z
mug=(c,a,perp,rho); B=c+(-0.0075)*a+0.083*rho
gamma,beta,delta,sgn=-20,-40,0.006,1
g=np.radians(gamma); b=np.radians(beta); yh=np.cos(b)*tau+np.sin(b)*rho
zdir=-np.cos(g)*rho+np.sin(g)*a; zh=zdir-(zdir@yh)*yh; zh/=np.linalg.norm(zh); yh=sgn*yh
tip=B+delta*zh; H=tip-0.1034*zh; Rm=h.pose_from(zh,yh); quat=rob.quat_from_axes(zh,yh)
print("H",np.round(H,3),"zh",np.round(zh,3),"yh",np.round(yh,3))
for gg in (0.040,0.005):
    dep,wh,Pw,lab=h.check(H,Rm,gap_half=gg,mug=mug); print("  gap",gg,{k:round(v,3) for k,v in h.clearance(Pw,lab,c,a,perp).items()})
np.save("barpinch_pose.npy",np.concatenate([H,quat,zh,yh]))
if len(sys.argv)<2: sys.exit()
r=rob.Robot()
q_grasp=np.array([-0.567,1.23,-0.11,-0.742,-0.001,1.389,1.376])
q_grasp=np.array(r.ik(list(H),quat,seed=list(q_grasp),timeout=5,avoid=True)); print("q_grasp",np.round(q_grasp,3))
Hpre=H-0.10*zh
q_pre=np.array(r.ik(list(Hpre),quat,seed=list(q_grasp),timeout=5,avoid=True)); print("q_pre",np.round(q_pre,3))
np.save("barpinch_q.npy",np.stack([q_grasp,q_pre]))
if sys.argv[1]=="go":
    r.gripper(0.04)
    ok=r.plan_exec(list(q_pre),vel=0.3,acc=0.3,planning_time=10.0,time_scale=2.0)
    print("plan_exec to pre:",ok,"q now",np.round(r.arm_q(),3))
    p,_=r.fk(); print("hand now",np.round(p,3),"target",np.round(Hpre,3))
