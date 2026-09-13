import math, numpy as np, subprocess
from rb import Robot, R_quat
r = Robot("look")
a = math.radians(40)
R = np.array([[0,1,0],[math.cos(a),0,math.sin(a)],[math.sin(a),0,-math.cos(a)]])
# columns: x_h=(0,cos,sin), y_h=(1,0,0), z_h=(0,sin,-cos)
print(np.linalg.det(R))
q = R_quat(R)
code = r.move_pose([0.0,-0.10,1.22], q, seconds=4)
if code is None: raise SystemExit
r.report()
