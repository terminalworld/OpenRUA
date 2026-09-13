from arm import *
r = Robot()
sol = [0.058, -0.161, -0.058, -2.445, -0.012, 2.227, 0.009]
p, q, _ = r.fk_world(sol); print("FK of IK sol:", np.round(p,4), np.round(q,4)); print(np.round(quat_to_R(*q),3))
p, q, _ = r.fk_world([-0.18, 0.501, -0.13, -1.914, 0.093, 2.409, -0.364]); print("FK cheese pregrasp:", np.round(p,4), np.round(q,4)); print(np.round(quat_to_R(*q),3))
print("expected tcp", np.round(p + TCP*quat_to_R(*q)[:,2],4))
