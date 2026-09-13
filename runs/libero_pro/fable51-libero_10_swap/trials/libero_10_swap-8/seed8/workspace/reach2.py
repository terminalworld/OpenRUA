import numpy as np
from reach import fk
res={}
for q2 in np.linspace(1.0,1.76,20):
  for q3 in np.linspace(-0.5,0.5,5):
    for q4 in np.linspace(-1.6,-0.07,32):
      for q5 in np.linspace(-0.8,0.8,5):
        for q6 in np.linspace(0.0,3.75,50):
          q=np.array([0,q2,q3,q4,q5,q6,0.785]); p,R=fk(q)
          if 0.975<=p[2]<=1.0 and abs(p[1])<0.05:
            tilt=np.degrees(np.arccos(-R[2,2])); b=int(tilt//10)*10
            if abs(R[1,2])>0.3: continue
            if b not in res or p[0]>res[b][0]: res[b]=(p[0],p[2],tilt,q.round(2))
for b in sorted(res): print(b,res[b])
