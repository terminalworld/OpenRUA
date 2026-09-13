import numpy as np
d = np.load("robot0_robotview_cloud.npz"); pw = d["pw"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
xs = np.arange(-0.30, 0.10, 0.02); ys = np.arange(-0.44, 0.02, 0.02)
print("     y:" + " ".join(f"{yy:5.2f}" for yy in ys))
for xg in xs:
    row = []
    for yg in ys:
        m = ok&(np.abs(x-xg)<0.01)&(np.abs(y-yg)<0.01)
        row.append(f"{z[m].max():5.3f}" if m.sum() else "  .  ")
    print(f"x={xg:5.2f} " + " ".join(row))
