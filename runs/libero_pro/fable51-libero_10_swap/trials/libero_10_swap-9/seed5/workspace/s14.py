import arm, numpy as np
a=arm.Arm()
print("grip",a.grip(0.08))
q=a.arm_q()
xyz,quat=a.fk()
print("hand",np.round(xyz,3))
s=a.ik((xyz[0],xyz[1],xyz[2]+0.12),quat,at_tcp=False,seed=q,timeout=10)
print("code",a.move_joints([s],seconds_per_seg=3))
print("hand",np.round(a.fk()[0],3),"fingers",a.fingers())
