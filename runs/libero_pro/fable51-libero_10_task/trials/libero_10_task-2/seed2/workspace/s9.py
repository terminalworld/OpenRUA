from rob import *
r = R()
j = r.ik(-0.197, 0.201, 1.08, 1.0,0,0,0)
for i in range(3):
    if r.move(j, 3.0) == 0: break
print("tcp", r.tcp()[0].round(4), "joints", np.round(r.joints(),3), "fingers", r.fingers())
