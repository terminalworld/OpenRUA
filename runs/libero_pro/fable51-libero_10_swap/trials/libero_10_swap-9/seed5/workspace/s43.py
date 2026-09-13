import arm, numpy as np, sys
a=arm.Arm(); T=arm.tilt_y
OB=[("mw",-0.30,0.086,-0.37,-0.14,0,1.117),("table",-0.8,0.56,-0.66,0.66,0,0.905)]
Q=T(90,False)
H=np.array([-0.29,-0.36]); r=0.20; eps=0.025; ZP=1.09
def P(deg):
    th=np.deg2rad(deg); d=np.array([np.cos(th),np.sin(th)]); n=np.array([np.sin(th),-np.cos(th)])
    xy=H+r*d+eps*n; return np.array([xy[0],xy[1],ZP])
def go(name,xyz,quat,secs=3,check=True):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    if check:
        v=arm.check_path(a,a.arm_q(),[q],n=10,obst=OB)
        if v: print("ABORT path",name,v[:4]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print(name,"-> tcp",np.round(xyz_+0.1034*R[:,2],3),"code",code,"fingers",np.round(a.fingers(),4),flush=True)
    if code!=0: sys.exit(1)
    return q
s0=P(-97); print("start",np.round(s0,3))
# pre-plan arc IK
seed=a.arm_q(); qs=[]
q=a.ik(s0,Q,at_tcp=True,seed=seed,timeout=10)
if q is None: print("ABORT ik start"); sys.exit(1)
seed=q
for deg in list(range(-90,1,10))+[0]:
    p=P(deg); q=a.ik(p,Q,at_tcp=True,seed=seed,timeout=10)
    if q is None: print("ABORT ik arc",deg,np.round(p,3)); sys.exit(1)
    jump=np.abs(np.array(q)-np.array(seed)).max()
    print(f"arc {deg:4d} p={np.round(p,3)} jump={jump:.2f}")
    if jump>0.6: print("ABORT big jump"); sys.exit(1)
    qs.append(q); seed=q
go("high start",s0+[0,0,0.11],Q,5)
print("close",a.grip(0.0))
go("start",s0,Q,3)
code=a.move_joints(qs,seconds_per_seg=1.5)
xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
print("arc done code",code,"tcp",np.round(xyz_+0.1034*R[:,2],3),flush=True)
if code!=0:
    code=a.move_joints([qs[-1]],seconds_per_seg=2)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print("retry final code",code,"tcp",np.round(xyz_+0.1034*R[:,2],3),flush=True)
go("lift",np.array([xyz_[0],xyz_[1],0])+0.1034*R[:,2]*np.array([1,1,0])+[0,0,1.22],Q,3,check=False)
