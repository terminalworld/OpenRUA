import arm, numpy as np
a = arm.Arm()
poses=[((-0.17,-0.20,1.32), arm.tilt_y(90,False)),
       ((-0.17,-0.45,1.22), arm.tilt_y(60,False)),
       ((-0.17,-0.45,1.02), arm.tilt_y(45,False))]
qs=arm.chain_ik(a,poses,max_jump=2.5)
code=a.move_joints(qs, seconds_per_seg=3.0)
xyz,q=a.fk(); print("hand", np.round(xyz,3), "tcp", np.round(xyz+arm.TCP*arm.quat_to_R(q)[:,2],3))
