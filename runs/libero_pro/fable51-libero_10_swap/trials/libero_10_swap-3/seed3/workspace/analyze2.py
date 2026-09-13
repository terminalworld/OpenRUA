import numpy as np, cv2
for cam in ["agentview", "robot0_robotview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; col = d["color"]
    x, y, z = pw[...,0], pw[...,1], pw[...,2]
    # objects above table, in the central region (exclude cabinet y<-0.15 and shelf y>0.15)
    m = (z > 0.915) & (z < 1.05) & (x > -0.3) & (x < 0.3) & (y > -0.17) & (y < 0.17)
    print(cam, "candidate pts:", m.sum())
    if m.sum():
        ys, xs = np.where(m)
        print("  pixel bbox u:", xs.min(), xs.max(), "v:", ys.min(), ys.max())
        print("  world x:", x[m].min(), x[m].max(), " y:", y[m].min(), y[m].max(), " z:", z[m].min(), z[m].max())
        print("  centroid:", x[m].mean(), y[m].mean(), z[m].mean())
    # bottle: tall thin
    mb = (z > 1.05) & (z < 1.3) & (x > -0.3) & (x < 0.3) & (y > -0.3) & (y < 0.0)
    if mb.sum():
        print("  bottle pts", mb.sum(), "x:", x[mb].min(), x[mb].max(), "y:", y[mb].min(), y[mb].max(), "ztop", z[mb].max())
