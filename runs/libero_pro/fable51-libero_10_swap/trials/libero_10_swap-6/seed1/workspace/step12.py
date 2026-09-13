from arm import *
a = Arm()
home = [0.0, -0.161, 0.0, -2.445, 0.0, 2.227, 0.785]
cur = np.array(a.arm_q())
zs = []
for s in np.linspace(0, 1, 15):
    q = cur + s * (np.array(home) - cur)
    p, _ = a.fk(list(q))
    zs.append((round(s,2), np.round(p,3)))
for s, p in zs: print(s, p)
