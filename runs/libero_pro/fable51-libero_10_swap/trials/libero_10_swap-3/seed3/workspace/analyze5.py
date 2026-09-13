import numpy as np
d = np.load("robot0_robotview_cloud.npz"); pw = d["pw"]; col = d["color"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
cx, cy = -0.150, 0.051
r = np.hypot(x-cx, y-cy)
# column through the bowl center in the image: find pixel closest to center
for v in range(60, 175, 5):
    row = []
    for u in range(300, 460, 10):
        row.append(f"{z[v,u]:.3f}")
    print(v, " ".join(row))
print("u from 300 to 450 step 10")
