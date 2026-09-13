import arm, numpy as np
a=arm.Arm(); T=arm.tilt_y
seq=[("pre top",(-0.131,-0.50,1.06),T(90,False)),
     ("pre pitched",(-0.131,-0.50,1.06),T(60,False)),
     ("mid pitched",(-0.131,-0.45,1.02),T(60,False)),
     ("place pitched",(-0.131,-0.398,0.98),T(60,False)),
     ("place low",(-0.131,-0.398,0.972),T(60,False)),
     ("retreat",(-0.131,-0.46,1.02),T(60,False))]
seed=a.arm_q(); print("now",np.round(seed,2))
for name,xyz,q in seq:
    s=a.ik(xyz,q,at_tcp=True,seed=seed,timeout=8)
    print(f"{name:14s} {'OK '+str(np.round(s,2))+' jump=%.2f'%np.abs(np.array(s)-seed).max() if s is not None else '--'}",flush=True)
    if s is not None: seed=s
