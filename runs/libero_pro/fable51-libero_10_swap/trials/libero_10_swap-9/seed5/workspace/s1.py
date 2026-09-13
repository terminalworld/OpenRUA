import arm, numpy as np
a = arm.Arm()
print("q", np.round(a.arm_q(),3))
print("fk", a.fk())
print("fingers", a.fingers())
a.grip(arm.GRIP["open_m"])
print("fingers after open", a.fingers())
# test IK for a viewing pose in front of the microwave opening
for xyz in [(-0.17,-0.50,1.0),(-0.17,-0.46,1.0),(-0.17,-0.45,1.05)]:
    q=a.ik(xyz, arm.FWD_Y, at_tcp=True)
    print(xyz, None if q is None else np.round(q,3))
    if q is not None: print("  fk check", a.fk(q))
