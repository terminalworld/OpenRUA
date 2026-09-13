import sys, json, numpy as np
sys.argv=[sys.argv[0],"noop"]
exec(open('grasp12.py').read().split('r.report("start")')[0])
from ikbest import LO, HI_M
top = np.array([TARGET[0] + OFF2[0], TARGET[1] + OFF2[1], ROT_P[2]])
for tag,pos in (("low",top-[0,0,0.15]),("rotp",ROT_P)):
    b,sols=ik_best(r,pos,QB,n=40)
    print(tag, [(round(m,2), np.round(q,2).tolist()) for m,q in sols[:6]])
