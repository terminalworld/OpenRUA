from rob import *
r = R()
# retreat toward home-ish pose above table, away from objects
home = [0.0, -0.161, 0.0, -2.445, 0.0, 2.227, 0.785]
for i in range(4):
    if r.move(home, 4.0) == 0: break
print("joints", np.round(r.joints(),3), "tcp", r.tcp()[0].round(3))
