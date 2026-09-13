from rob import *
r = Robot()
seed = r.arm_q()
tests = [("base grasp t0", (0.143,0.096,0.968), 0.0), ("base grasp pre t0", (0.143,0.096,1.10), 0.0),
         ("base grasp t.35", (0.143,0.096,0.968), 0.35),
         ("vbar t.52", (0.247,0.096,1.03), 0.52), ("vbar t.785", (0.247,0.096,1.03), 0.785), ("vbar t1.0", (0.247,0.096,1.03), 1.0),
         ("vbar pre t.785", (0.247,0.096,1.13), 0.785),
         ("arc0 x.217 z1.089 t.785", (0.217,0.096,1.089), 0.785), ("arc1 x.17 z1.088 t.5", (0.17,0.096,1.088), 0.5),
         ("arc2 x.13 z1.075 t.25", (0.13,0.096,1.075), 0.25), ("arc3 x.099 z1.056 t0", (0.099,0.096,1.056), 0.0)]
for name, tcp, t in tests:
    q = r.ik_tcp(tcp, xbar_R(t), seed=seed)
    print(f"{name}: {'ok '+str(np.round(q,2)) if q is not None else 'NO'}")
