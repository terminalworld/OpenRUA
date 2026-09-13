import arm, numpy as np, sys
a=arm.Arm()
T=arm.tilt_y
BAR=(0.016,-0.0735)
def go(name,xyz,quat,secs=3):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    print(name,"jump",np.round(np.abs(np.array(q)-a.arm_q()).max(),2))
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    print(name,"->",np.round(a.fk()[0],3),"code",code,flush=True)
    return code
go("above bar",(BAR[0],BAR[1],1.10),T(90,False),3)
go("pinch height",(BAR[0],BAR[1],0.962),T(90,False),3)
print("grip",a.grip(0.0))
go("lift",(BAR[0],BAR[1],1.04),T(90,False),3)
print("fingers",a.fingers())
