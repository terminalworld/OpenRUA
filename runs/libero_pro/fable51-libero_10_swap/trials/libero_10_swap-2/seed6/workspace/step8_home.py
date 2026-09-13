import numpy as np, pk
r = pk.Robot("home")
home = np.array([0.0, -0.161, 0.0, -2.4446, 0.0, 2.2268, 0.7854])
for _ in range(3):
    code, err = r.move_joints([home], 4.0)
    if err < 0.01: break
print("DONE")
