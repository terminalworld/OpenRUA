import arm, numpy as np
a = arm.Arm()
print("fk now", np.round(a.fk()[0],3), np.round(a.fk()[1],3))
poses=[((-0.17,-0.20,1.32), arm.tilt_y(90,False)),
       ((-0.17,-0.45,1.22), arm.tilt_y(60,False)),
       ((-0.17,-0.45,1.02), arm.tilt_y(45,False))]
qs=arm.chain_ik(a,poses,max_jump=2.5)
if qs:
    v=arm.check_path(a,a.arm_q(),qs,obst=arm.OBST+[arm.YELLOW_MUG])
    print("violations:",len(v)); [print(x) for x in v[:20]]
