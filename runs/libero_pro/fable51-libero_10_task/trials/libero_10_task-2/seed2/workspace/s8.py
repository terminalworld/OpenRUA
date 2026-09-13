from rob import *
import sys
r = R()
d = float(sys.argv[1])
j = r.joints(); j[6] += d
print("target q7", j[6])
for i in range(3):
    if r.move(j, 3.0) == 0: break
print("joints", np.round(r.joints(),3), "fingers", r.fingers())
