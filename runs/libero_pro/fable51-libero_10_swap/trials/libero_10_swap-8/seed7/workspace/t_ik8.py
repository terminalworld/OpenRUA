from rob import *
from flip import R_of
from home import HOME
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
r = Robot(); H = math.pi/2
SEEDS = {"home": HOME, "cur": r.arm_q(), "s1": [0.5,0.5,0,-2.0,0,2.5,0.8], "s2": [-0.5,0.8,0.5,-2.0,-1.0,2.0,-1.0], "s3":[0.3,1.0,-0.5,-1.5,0.3,1.6,2.0], "s4":[1.0,1.0,-1.0,-1.5,0,2.0,1.5]}
def Rroll(yaw, tilt, roll):  # roll about hand z
    R = R_of(yaw, tilt); c,s = math.cos(roll), math.sin(roll)
    return R @ np.array([[c,-s,0],[s,c,0],[0,0,1]])
tests = [("hook in", (-0.077, 0.205, 1.016), 0.0, H, 0.0), ("hook in roll", (-0.077, 0.205, 1.016), 0.0, H, H),
         ("hook end", (-0.077, 0.077, 1.048), 0.0, H, 0.0), ("hook end roll", (-0.077, 0.077, 1.048), 0.0, H, H),
         ("waist grasp -y", (-0.077, 0.14, 0.9615), -H, H, 0.0), ("waist grasp +x", (-0.077-0.0, 0.14, 0.9615), 0.0, H, 0.0),
         ("waist pre +x", (-0.16, 0.14, 0.9615), 0.0, H, 0.0),
         ("place +x", (0.19, 0.10, 0.9955), 0.0, H, 0.0), ("carry +x", (0.19, 0.10, 1.20), 0.0, H, 0.0)]
for name, tcp, yaw, tilt, roll in tests:
    best = []
    for sn, sd in SEEDS.items():
        q = r.ik_tcp(tcp, Rroll(yaw, tilt, roll), seed=sd, tries=1)
        if q is not None: best.append((margin(q), sn, np.round(q,2)))
    best.sort(key=lambda b: -b[0])
    print(name, tcp, "->", [(round(m,2), sn, list(q)) for m, sn, q in best[:2]] or "NO", flush=True)
