from rlib import *
from scene import *
r = Robot("diag")
sc = Scene(r.node)
sc.publish(objects(drawer_front=-0.105, bowl=(-0.075, -0.17), with_bowl=True))
q_stall = np.array([0.351, 0.514, -0.099, -1.958, -1.042, 1.967, 0.257])
q_pre = np.array([0.428, 0.741, -0.166, -1.923, -0.988, 1.983, 0.243])
for name, q in [("stall", q_stall), ("pre", q_pre)]:
    print(name, sc.check(ARM, q, 0.0))
    for link in ["panda_hand", "panda_link7", "panda_link6", "panda_link5", "panda_link4", "panda_link3"]:
        p, R = r.fk(q, link); print("   ", link, np.round(p, 3))
# interpolate between
for a in np.linspace(0, 1, 6):
    q = q_stall + a * (q_pre - q_stall)
    print(f"a={a:.1f}", sc.check(ARM, q, 0.0))
