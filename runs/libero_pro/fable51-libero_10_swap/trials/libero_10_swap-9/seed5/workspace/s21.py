import arm, numpy as np
a=arm.Arm()
def ori(sign, alpha_deg, ysign=1):
    al=np.radians(alpha_deg)
    Z=np.array([sign*np.cos(al),0,-np.sin(al)]); Y=np.array([0,ysign,0.0]); X=np.cross(Y,Z)
    return arm.quat_from_axes(X,Y,Z)
BAR=(0.019,-0.0735,0.962)
tests=[]
for sign in (+1,-1):
  for al in (0,20,40):
    for ys in (1,-1):
      tests.append((f"pick sign{sign} a{al} y{ys}",BAR,ori(sign,al,ys)))
      tests.append((f"mw   sign{sign} a{al} y{ys}",(-0.164,-0.385,1.0),ori(sign,al,ys)))
      tests.append((f"mwhi sign{sign} a{al} y{ys}",(-0.164,-0.43,1.05),ori(sign,al,ys)))
seed=a.arm_q()
for name,xyz,q in tests:
    s=a.ik(xyz,q,at_tcp=True,seed=seed,timeout=6)
    if s is None: s=a.ik(xyz,q,at_tcp=True,seed=[0,-0.785,0,-2.356,0,1.571,0.785],timeout=6)
    print(f"{name:26s} {'OK '+str(np.round(s,2)) if s is not None else '--'}",flush=True)
