import arm, numpy as np, sys
a = arm.Arm()
T=arm.tilt_y
print("now", np.round(a.fk()[0],3), np.round(a.arm_q(),2))
for name,xyz,q in [("h45f",(-0.17,-0.45,1.28),T(45,False)),("h45t",(-0.17,-0.45,1.28),T(45,True)),
                   ("h60f",(-0.17,-0.45,1.25),T(60,False)),("h90t",(-0.17,-0.40,1.28),T(90,True)),
                   ("h90t2",(-0.17,-0.45,1.3),T(90,True)),("h90f",(-0.17,-0.45,1.3),T(90,False)),
                   ("h70f",(-0.17,-0.45,1.25),T(70,False)),("h90t3",(-0.12,-0.35,1.3),T(90,True))]:
    s=a.ik(xyz,q,at_tcp=True,seed=a.arm_q(),timeout=8)
    print(name, None if s is None else np.round(s,2), flush=True)
