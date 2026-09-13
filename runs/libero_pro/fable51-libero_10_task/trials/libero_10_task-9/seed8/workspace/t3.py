from lib import *
from scene import *
r = Robot("t3"); sc = Scene(r.node)
print("publish:", sc.publish(world_objects()))
q0 = r.arm_q()
print("current valid:", sc.check(q0))
R_H  = R_from_axes(np.array([0,0,1.]),  np.array([1.,0,0]),  np.array([0,1.,0]))
R_H2 = R_from_axes(np.array([0,0,-1.]), np.array([-1.,0,0]), np.array([0,1.,0]))
for name,R in [("R_H",R_H),("R_H2",R_H2)]:
    for pt in [(-0.05,-0.12,1.04),(-0.05,-0.15,1.04),(-0.07,-0.15,1.0)]:
        s = r.ik(np.array(pt), R, seed=q0)
        if s is None: print(name, pt, "IK none"); continue
        v,_ = sc.check(s); print(name, pt, "q=",np.round(s,3), "valid",v)
