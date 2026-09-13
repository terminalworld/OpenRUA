from rob import *
LIM = np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def margin(q): q=np.array(q); return float(np.min(np.minimum(q-LIM[:,0], LIM[:,1]-q)))
HOME=[0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]
r = Robot()
Rs = {"+y rollA": np.array([[-1.0,0,0],[0,0,1.0],[0,1.0,0]]),
      "+y rollB": np.array([[1.0,0,0],[0,0,-1.0],[0,1.0,0]]),
      "-y rollA": np.array([[1.0,0,0],[0,0,1.0],[0,-1.0,0]]),
      "-y rollB": np.array([[-1.0,0,0],[0,0,-1.0],[0,-1.0,0]])}
pts = [(-0.003, 1.0505), (-0.10, 1.086), (-0.22, 1.026)]
seeds = [HOME, r.arm_q(), [0.5,0.8,0.3,-1.8,0.2,2.3,1.5], [-0.8,0.8,0.8,-1.8,-0.5,2.3,-0.5], [0.3,1.0,-0.3,-1.5,0.5,2.0,2.5]]
for name, R in Rs.items():
    print(name, "det", round(np.linalg.det(R),2))
    for x, z in pts:
        y = 0.207 if name.startswith("+y") else 0.207
        best = None
        for sd in seeds:
            q = r.ik_tcp((x, y, z), R, seed=sd, tries=1)
            if q is not None and (best is None or margin(q) > best[0]): best = (margin(q), np.round(q,2))
        print(f"   ({x},{z}) -> {best}", flush=True)
