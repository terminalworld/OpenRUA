import sys, json, numpy as np
sys.argv=[sys.argv[0],"noop"]
exec(open('grasp12.py').read().split('r.report("start")')[0])
from ikbest import LO, HI_M
qg=json.load(open('grasp12.json'))["qg"]
lift = chain(qg, lift_steps(TCP, QA), "lift"); rot = chain(lift[-1], rot_steps(ROT_P, QA, QB), "rotate")
top = np.array([TARGET[0] + OFF2[0], TARGET[1] + OFF2[1], ROT_P[2]])
pl = chain(rot[-1], [(ROT_P + (top - ROT_P) * k / 3, QB) for k in (1, 2, 3)] + [(top - [0, 0, dz], QB) for dz in (0.04, 0.08, 0.12, 0.15)], "place")
for name,ch in (("rot",rot),("pl",pl)):
    for q in ch:
        q=np.array(q); m=np.minimum(q-LO,HI_M-q); print(name, q.round(2), "min", m.min().round(2), "joint", m.argmin()+1)
