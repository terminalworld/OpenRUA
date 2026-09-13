import arm, numpy as np, sys
a = arm.Arm()
MUG=(0.009,0.007)
T=arm.tilt_y
def go(name,xyz,quat,secs=4):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    print(name,"jump",np.round(np.abs(np.array(q)-a.arm_q()).max(),2))
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    print(name,"->",np.round(a.fk()[0],3),"code",code,flush=True)
go("pregrasp",(MUG[0],MUG[1],1.00),T(90,False),3)
go("grasp",(MUG[0],MUG[1],0.948),T(90,False),3)
print("fingers before",a.fingers())
print("grip",a.grip(0.0))
print("fingers after",a.fingers())
