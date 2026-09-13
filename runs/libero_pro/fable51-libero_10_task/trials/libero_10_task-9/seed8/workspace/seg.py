import numpy as np
d = np.load("birdview_cloud.npz"); pw = d["pw"]; col = d["color"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
# microwave region: above table, y>0.1
m = (Z > 0.93) & (Y > 0.05) & (X > -0.5)
print("microwave-ish pts:", m.sum())
for zlo, zhi in [(0.93,1.0),(1.0,1.05),(1.05,1.09),(1.09,1.12)]:
    mm = m & (Z>=zlo) & (Z<zhi)
    if mm.sum(): print(f"z[{zlo},{zhi}) n={mm.sum()} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
# top face
top = m & (Z>1.09)
print("top face x", X[top].min(), X[top].max(), "y", Y[top].min(), Y[top].max())
# Door: thin thing - print a grid of z in region x[-0.3,0.2], y[0.05,0.5]
xs = np.arange(-0.30, 0.20, 0.025); ys = np.arange(0.05, 0.50, 0.025)
print("z grid (rows=x, cols=y)")
print("      " + " ".join(f"{y:5.2f}" for y in ys))
for x in xs:
    row = []
    for y in ys:
        mm = (np.abs(X-x)<0.0125)&(np.abs(Y-y)<0.0125)
        row.append(f"{Z[mm].max():5.2f}" if mm.sum() else "  -  ")
    print(f"{x:5.2f} " + " ".join(row))
