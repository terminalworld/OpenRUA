from arm import *
a = Arm()
home = [0.0, -0.161, 0.0, -2.445, 0.0, 2.227, 0.785]
cur = np.array(a.arm_q())
ok = True
for s in np.linspace(0, 1, 12):
    q = cur + s * (np.array(home) - cur)
    p, _ = a.fk(list(q)); print(round(s,2), np.round(p,3))
    if p[2] - 0.11 < 0.60: ok = False
print("path clear:", ok)
if ok:
    for i in range(3):
        code, err = a.move_joints(home, 5.0)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4))
