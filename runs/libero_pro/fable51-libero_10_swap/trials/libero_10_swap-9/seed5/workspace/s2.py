import arm, numpy as np
a = arm.Arm()
xyz,quat=a.fk(); print("fk", np.round(xyz,4), np.round(quat,4))
q=a.ik(xyz,quat,at_tcp=False); print("ik of current pose", None if q is None else np.round(q,3), "current", np.round(a.arm_q(),3))
for xyz in [(-0.17,-0.50,1.0),(-0.17,-0.46,1.0),(-0.17,-0.45,1.05)]:
    q=a.ik(xyz, arm.FWD_Y, at_tcp=True)
    print(xyz, None if q is None else np.round(q,3))
    if q is not None: print("  fk check", np.round(a.fk(q)[0],4), np.round(a.fk(q)[1],3))
