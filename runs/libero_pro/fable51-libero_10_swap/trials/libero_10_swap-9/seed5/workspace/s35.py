import arm, numpy as np, sys
a=arm.Arm(); T=arm.tilt_y
OB=[("mw",-0.30,0.086,-0.37,-0.14,0,1.117),("door",-0.32,-0.28,-0.62,-0.36,0,1.117),("table",-0.8,0.56,-0.66,0.66,0,0.905)]
Q=T(60,False); Zax=np.array([0,0.5,-0.866])
BAR=np.array([-0.1315,-0.4085,1.003])
def go(name,xyz,quat,secs=3,check=True):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    if check:
        v=arm.check_path(a,a.arm_q(),[q],n=10,obst=OB)
        if v: print("ABORT path",name,v[:4]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print(name,"-> tcp",np.round(xyz_+0.1034*R[:,2],3),"Z",np.round(R[:,2],2),"code",code,"fingers",np.round(a.fingers(),4),flush=True)
    if code!=0: sys.exit(1)
go("approach",BAR-0.06*Zax,Q,4)
go("at bar",BAR,Q,3)
print("pinch",a.grip(0.0))
f=a.fingers()
if abs(f[0])>0.012: print("pinch missed? fingers",f); sys.exit(1)
go("push",BAR+np.array([0,0.03,0]),Q,4,check=False)
print("release",a.grip(0.08))
go("retreat",BAR+np.array([0,0.03,0])-0.06*Zax,Q,3,check=False)
