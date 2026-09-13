from lib import *
r = Robot("t1")
q = r.arm_q()
p, R = r.fk(q); print("FK hand world", np.round(p,4))
# IK for a pose 10cm lower, straight down orientation; FK the result
target = p + np.array([0.1, 0, -0.1])
sol = r.ik(target, R_DOWN, seed=q); print("IK:", None if sol is None else np.round(sol,4))
if sol: 
    p2,R2 = r.fk(sol); print("FK(sol)", np.round(p2,4), "target", np.round(target,4)); print(np.round(R2,3))
# horizontal grasp orientation test near white mug pregrasp
R_H = R_from_axes(np.array([0,0,1.]), np.array([1.,0,0]), np.array([0,1.,0]))
for pt in [(-0.10,-0.40,0.96),(-0.10,-0.40,1.0),(0.0,0.10,1.10),(0.0,0.28,1.005),(0.0,0.20,1.005)]:
    s = r.ik(np.array(pt), R_H, seed=q); print("R_H", pt, "->", None if s is None else np.round(s,3))
R_H2 = R_from_axes(np.array([0,0,-1.]), np.array([-1.,0,0]), np.array([0,1.,0]))
for pt in [(-0.10,-0.40,0.96),(0.0,0.28,1.005)]:
    s = r.ik(np.array(pt), R_H2, seed=q); print("R_H2", pt, "->", None if s is None else np.round(s,3))
