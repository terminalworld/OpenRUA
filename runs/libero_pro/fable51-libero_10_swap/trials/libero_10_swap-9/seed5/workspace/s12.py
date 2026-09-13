import arm, numpy as np, sys
a = arm.Arm()
MUG=(0.009,0.007)
T=arm.tilt_y
def go(name,xyz,quat,secs=4,obst=None,allow=()):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    v=arm.check_path(a,a.arm_q(),[q],n=10,obst=obst,allow=allow)
    if v: print("ABORT path",name,v[:5]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    print(name,"->",np.round(a.fk()[0],3),"code",code,flush=True)
    return code
# leave the viewing pose: go up/back first (hand ~ tilt45 fx=False now)
go("retreat up",(-0.17,-0.45,1.15),T(45,False),3)
go("high mid",(-0.05,-0.15,1.25),T(90,True),4)
go("above mug",(MUG[0],MUG[1],1.12),T(90,True),4)
print("fingers",a.fingers())
