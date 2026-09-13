import arm, numpy as np
a = arm.Arm()
MUG=(0.009,0.007)
def T(th,fx): return arm.tilt_y(th,fx)
plan=[
 ("above mug topdown fx", (MUG[0],MUG[1],1.12), T(90,True)),
 ("grasp mug topdown fx", (MUG[0],MUG[1],0.948), T(90,True)),
 ("above stage yaw180",  (-0.15,-0.41,1.12), T(90,False)),
 ("place stage yaw180",  (-0.15,-0.41,0.952), T(90,False)),
 ("tilt45 above stage",  (-0.15,-0.41,1.05), T(45,True)),
 ("tilt45 grasp stage",  (-0.15,-0.41,0.956), T(45,True)),
 ("tilt45 lift",         (-0.15,-0.41,1.01), T(45,True)),
 ("tilt45 enter",        (-0.15,-0.36,1.01), T(45,True)),
 ("tilt30 enter",        (-0.15,-0.36,1.01), T(30,True)),
 ("tilt20 enter",        (-0.15,-0.36,1.01), T(20,True)),
 ("tilt15 enter",        (-0.15,-0.36,1.01), T(15,True)),
 ("tilt20 deep",         (-0.15,-0.332,1.0), T(20,True)),
 ("tilt15 deep",         (-0.15,-0.332,1.0), T(15,True)),
 ("tilt15 deep low",     (-0.15,-0.332,0.985), T(15,True)),
]
for fx in (True,False):
  print("=== fingers_x", fx)
  seed=a.arm_q()
  for name,xyz,q in plan:
    q=arm.quat_from_axes(*[arm.quat_to_R(q)[:,i] for i in range(3)]) if fx else q
    if not fx:
        # flip finger direction: rotate 180 about hand Z
        R=arm.quat_to_R(q); R=np.stack([-R[:,0],-R[:,1],R[:,2]],1); q=arm.R_to_quat(R)
    sol=None
    for _ in range(2):
        sol=a.ik(xyz,q,at_tcp=True,seed=seed,timeout=5)
        if sol is not None: break
    print(f"{name:24s} {xyz} -> {'OK '+str(np.round(sol,2)) if sol is not None else '--'}", flush=True)
    if sol is not None: seed=sol
