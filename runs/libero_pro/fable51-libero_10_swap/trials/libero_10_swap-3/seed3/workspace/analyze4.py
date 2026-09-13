import numpy as np
for cam, (v0,v1,u0,u1) in {"agentview": (185,245,305,395), "birdview": (240,290,305,370)}.items():
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]; col = d["color"]
    x, y, z = pw[...,0], pw[...,1], pw[...,2]
    sub = np.zeros_like(z, bool); sub[v0:v1, u0:u1] = True
    m = sub & (z > 0.905) & (z < 1.25) & (y > -0.02) & (y < 0.13) & (x > -0.25) & (x < -0.05)
    print(cam, "bowl pts", m.sum())
    if m.sum():
        print(" x:", x[m].min(), x[m].max(), " y:", y[m].min(), y[m].max(), " z:", z[m].min(), z[m].max())
        h, e = np.histogram(z[m], bins=12)
        for hh, ee in zip(h, e): print(f"  z>{ee:.3f}: {hh}")
        rim = m & (z > z[m].max() - 0.01)
        print(" rim center:", x[rim].mean(), y[rim].mean(), "x range", x[rim].min(), x[rim].max(), "y range", y[rim].min(), y[rim].max())
