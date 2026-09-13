import numpy as np
hm = np.load("snaps/hm.npy"); res=0.005; x0,y0=-0.45,-0.35
def cell(x,y): return hm[int((x-x0)/res), int((y-y0)/res)]
def row(x, ys): return " ".join(f"{cell(x,y):.3f}" if np.isfinite(cell(x,y)) else "  nan" for y in ys)
print("Bottle region: rows x=-0.22..-0.11, cols y=0.02..0.13")
ys=np.arange(0.02,0.135,0.005)
print("      y:", " ".join(f"{y:5.3f}" for y in ys))
for x in np.arange(-0.22,-0.105,0.005): print(f"x={x:6.3f}:", row(x,ys))
print()
print("Drawer profile along y at x=0.0 (front handle..cabinet):")
ys=np.arange(0.02,0.30,0.005); print("      y:", " ".join(f"{y:5.3f}" for y in ys)); print("        ", row(0.0,ys))
print("Drawer profile along x at y=0.15:")
xs=np.arange(-0.16,0.17,0.005); print("      x:", " ".join(f"{x:6.3f}" for x in xs)); print("        ", " ".join(f"{cell(x,0.15):6.3f}" if np.isfinite(cell(x,0.15)) else "   nan" for x in xs))
