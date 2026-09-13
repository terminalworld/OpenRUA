import arm, numpy as np
a = arm.Arm()
target=((-0.17,-0.45,1.02), arm.tilt_y(45,False))
q=a.ik(*target, at_tcp=True)
for i in range(3):
    print("jump", np.abs(np.array(q)-np.array(a.arm_q())).max())
    code=a.move_joints([q], seconds_per_seg=2.0)
    if code==0: break
xyz,qq=a.fk(); R=arm.quat_to_R(qq); print("hand", np.round(xyz,3), "tcp", np.round(xyz+arm.TCP*R[:,2],3)); print("hand axes\n", np.round(R.T,3))
