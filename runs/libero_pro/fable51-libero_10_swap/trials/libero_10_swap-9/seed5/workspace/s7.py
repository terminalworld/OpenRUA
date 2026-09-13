import arm, numpy as np, sys
a = arm.Arm()
target=((-0.17,-0.45,1.02), arm.tilt_y(45,False))
q=a.ik(*target, at_tcp=True)
print("jump", np.abs(np.array(q)-np.array(a.arm_q())).max())
a.move_joints([q], seconds_per_seg=2.0)
xyz,qq=a.fk(); print("hand", np.round(xyz,3), "tcp", np.round(xyz+arm.TCP*arm.quat_to_R(qq)[:,2],3))
