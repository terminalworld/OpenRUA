import sys, numpy as np, rclpy
sys.argv=[sys.argv[0]]
from builtin_interfaces.msg import Duration
import arm
from arm import Arm
a=Arm()
ready=[0,-0.785,0,-2.356,0,1.571,0.785]
pos,q=a.fk(ready); print("fk(ready)", np.round(pos,3), np.round(q,3))
# patch longer timeout
orig=a.ik
def ik2(p,q,seed=None,at_tcp=False,t=5):
    return orig(p,q,seed=seed,at_tcp=at_tcp)
for p in [(0.4,0,0.4),(0.5,0,0.3),(0.3,-0.13,0.4),(0.252,-0.134,0.30)]:
    print(p, "seed ready ->", a.ik(p,[1,0,0,0],seed=ready))
    print(p, "seed ready q=fk(ready) ->", a.ik(p,q,seed=ready))
rclpy.shutdown()
