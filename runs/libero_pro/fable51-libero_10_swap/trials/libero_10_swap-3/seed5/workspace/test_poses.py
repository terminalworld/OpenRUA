from rlib import *
from scene import *
r = Robot("tp")
sc = Scene(r.node)
print("publish", sc.publish(objects()))
q0 = r.arm_q()
print("current valid:", sc.check(ARM, q0, r.finger()))
xb, yb = -0.138, 0.063
Rpick = top_down_R(0.0)
cands = {
  "pick_pre": ([xb - 0.05, yb, 1.05], Rpick),
  "pick_grasp": ([xb - 0.05, yb, 0.925], Rpick),
  "pick_lift": ([xb - 0.05, yb, 1.05], Rpick),
}
th = np.deg2rad(25)
Rplace = hand_R([1, 0, 0], [0, -np.sin(th), -np.cos(th)])
xp, yp = -0.11, -0.155
cands["place_pre"] = ([xp - 0.05, yp, 1.10], Rplace)
cands["place"] = ([xp - 0.05, yp, 0.985], Rplace)
cands["place_flat"] = ([xp - 0.05, yp, 0.985], Rpick)
seed = q0
for name, (p, R) in cands.items():
    try:
        q = r.ik(p, R, seed=seed)
        seed = q
        tp, Rk = r.tcp(q)
        v, c = sc.check(ARM, q, 0.04)
        print(f"{name}: tcp={np.round(tp,4)} q={np.round(q,3)} valid={v} {c}")
    except Exception as e:
        print(f"{name}: IK error {e}")
