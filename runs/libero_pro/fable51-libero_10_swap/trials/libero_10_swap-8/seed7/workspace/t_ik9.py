from rob import *
from flip import R_of
HOME=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
r = Robot(); H = math.pi/2
print("q now", np.round(r.arm_q(),3))
for yaw in (0.0,):
  for x in (-0.15, -0.05, 0.05, 0.15):
    for z in (0.96, 1.05, 1.20):
        q = r.ik_tcp((x, 0.0, z), R_of(yaw, H), seed=HOME, tries=2)
        print(f"+x horiz tcp ({x},0,{z}) -> {'NO' if q is None else (np.round(q,2), round(margin(q),2))}", flush=True)
# hand pointing +y horizontal (wrist at -y side), closing along x
for (x,y,z) in ((-0.077,0.205,1.016), (-0.077,0.10,1.016), (0.0,0.20,1.0)):
    q = r.ik_tcp((x,y,z), R_of(H, H), seed=HOME, tries=2)
    print(f"+y horiz tcp {(x,y,z)} -> {'NO' if q is None else (np.round(q,2), round(margin(q),2))}", flush=True)
