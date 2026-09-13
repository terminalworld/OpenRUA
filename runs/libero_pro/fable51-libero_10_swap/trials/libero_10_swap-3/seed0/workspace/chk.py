from robot import *
from scene import validator
r = Robot("chk"); valid = validator(r)
Q = np.load("Qpush.npy")
seed = [0.968,1.12,-0.866,-2.431,-2.116,2.188,-1.843]
for y in [-0.21,-0.22,-0.23,-0.24]:
    q = r.ik_tcp((-0.19,y,0.972), Q, seed, avoid=False)
    print(y, None if q is None else (np.round(q,3), valid(q)))
    if q is not None: seed = q
