import arm, numpy as np
a = arm.Arm()
tests = [
 ("down above mug", (0.01,0.0,1.10), arm.DOWN),
 ("down above mug fx", (0.01,0.0,1.10), arm.DOWN_FX),
 ("fwd_y easy", (-0.2,-0.2,1.1), arm.FWD_Y),
 ("fwd_y -0.40", (-0.17,-0.40,1.0), arm.FWD_Y),
 ("fwd_y -0.45", (-0.17,-0.45,1.0), arm.FWD_Y),
 ("fwd_y -0.45 z1.1", (-0.17,-0.45,1.1), arm.FWD_Y),
 ("fwd_y -0.45 x-0.25", (-0.25,-0.45,1.0), arm.FWD_Y),
 ("fwd_y at mug", (0.01,-0.10,0.955), arm.FWD_Y),
 ("fwd_y at mug flipped", (0.01,-0.10,0.955), arm.quat_from_axes([0,0,-1],[-1,0,0],[0,1,0])),
 ("fwd_y -0.45 flipped", (-0.17,-0.45,1.0), arm.quat_from_axes([0,0,-1],[-1,0,0],[0,1,0])),
]
for name,xyz,q in tests:
    for trial in range(2):
        sol=a.ik(xyz,q,at_tcp=True)
        if sol is not None: break
    print(f"{name:28s} {xyz} ->", None if sol is None else np.round(sol,3))
