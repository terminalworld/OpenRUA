import arm, numpy as np, sys
a=arm.Arm(); T=arm.tilt_y
for xyz in [(-0.17,-0.45,1.02),(-0.14,-0.46,1.02),(-0.14,-0.48,1.04)]:
    q=a.ik(xyz,T(45,False),at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is not None: break
if q is None: sys.exit("no ik")
print("code",a.move_joints([q],seconds_per_seg=3))
xyz,quat=a.fk(); R=arm.quat_to_R(quat); print("tcp",np.round(xyz+0.1034*R[:,2],3),"Z",np.round(R[:,2],2))
