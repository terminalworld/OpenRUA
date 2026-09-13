import arm, numpy as np, sys
a=arm.Arm(); T=arm.tilt_y
OB=[("mw",-0.30,0.086,-0.37,-0.14,0,1.117),("door",-0.32,-0.28,-0.62,-0.36,0,1.117),("table",-0.8,0.56,-0.66,0.66,0,0.905)]
def go(name,xyz,quat,secs=4):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    v=arm.check_path(a,a.arm_q(),[q],n=10,obst=OB)
    if v: print("ABORT path",name,v[:4]); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    print(name,"->",np.round(a.fk()[0],3),"code",code,"fingers",np.round(a.fingers(),4),flush=True)
    if code!=0: sys.exit(1)
go("lift high",(0.016,-0.0735,1.19),T(90,False),4)
go("over mw",(-0.06,-0.30,1.19),T(90,False),4)
go("front high",(-0.131,-0.51,1.19),T(90,False),4)
go("front mid",(-0.131,-0.51,1.10),T(90,False),3)
go("front low",(-0.131,-0.51,1.02),T(90,False),3)
