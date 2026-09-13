import sys, json, numpy as np
sys.argv=[sys.argv[0],"noop"]
exec(open('grasp12.py').read().split('r.report("start")')[0])
b,sols=ik_best(r,TCP,QA,n=60)
for m,q in sols[:12]: print(round(m,2), np.round(q,2).tolist())
