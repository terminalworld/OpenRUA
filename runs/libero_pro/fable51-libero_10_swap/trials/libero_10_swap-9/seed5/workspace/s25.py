import arm, numpy as np
a=arm.Arm(); T=arm.tilt_y
seq=[("lift high",(0.016,-0.0735,1.19),T(90,False)),
     ("over mw",(-0.06,-0.30,1.19),T(90,False)),
     ("front high",(-0.131,-0.51,1.19),T(90,False)),
     ("front mid",(-0.131,-0.51,1.10),T(90,False)),
     ("front low",(-0.131,-0.51,1.02),T(90,False)),
     ("pitch75",(-0.131,-0.48,1.02),T(75,False)),
     ("pitch60",(-0.131,-0.45,1.02),T(60,False)),
     ("place",(-0.131,-0.398,0.98),T(60,False))]
seed=a.arm_q(); print("now",np.round(seed,2))
for name,xyz,q in seq:
    s=a.ik(xyz,q,at_tcp=True,seed=seed,timeout=8)
    print(f"{name:12s} {'OK '+str(np.round(s,2))+' jump=%.2f'%np.abs(np.array(s)-seed).max() if s is not None else '--'}",flush=True)
    if s is not None: seed=s
