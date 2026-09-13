import arm, numpy as np, itertools
a = arm.Arm()
seed=a.arm_q()
res=[]
for th in (0,10,20):
  for fx in (True,False):
    for x in (-0.22,-0.18,-0.14):
      for y in (-0.42,-0.46,-0.50):
        for z in (1.0,1.05):
            q=a.ik((x,y,z), arm.tilt_y(th,fx), at_tcp=True, seed=seed, timeout=5)
            ok = q is not None
            print(f"th={th:2d} fx={int(fx)} tcp=({x:.2f},{y:.2f},{z:.2f}) -> {'OK '+str(np.round(q,2)) if ok else '--'}", flush=True)
