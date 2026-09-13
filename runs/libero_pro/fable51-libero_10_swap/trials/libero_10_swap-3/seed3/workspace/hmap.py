import numpy as np, sys
cam, x0, x1, y0, y1 = sys.argv[1], *map(float, sys.argv[2:6])
d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
x, y, z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(z)
xs = np.arange(x0, x1, 0.02); ys = np.arange(y0, y1, 0.02)
print("     y:" + " ".join(f"{yy:5.2f}" for yy in ys))
for xg in xs:
    row = []
    for yg in ys:
        m = ok&(np.abs(x-xg)<0.01)&(np.abs(y-yg)<0.01)
        row.append(f"{z[m].max():5.3f}" if m.sum() else "  .  ")
    print(f"x={xg:5.2f} " + " ".join(row))
