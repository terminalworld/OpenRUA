import numpy as np, cv2
d = np.load("birdview_cloud.npz"); pw, color = d["pw"], d["color"]
z = pw[...,2]; x = pw[...,0]; y = pw[...,1]
# print a coarse height map over the table region (x -0.35..0.45, y -0.55..0.55) in 2cm cells
xs = np.arange(-0.35, 0.30, 0.02); ys = np.arange(-0.55, 0.30, 0.02)
print("      " + "".join(f"{yy:+.2f}"[1:4].rjust(4) if i%5==0 else "    " for i,yy in enumerate(ys)))
for xx in xs:
    row = ""
    for yy in ys:
        m = (np.abs(x-xx)<0.01)&(np.abs(y-yy)<0.01)
        if m.sum()==0: row += "   ."; continue
        h = np.median(z[m]) - 0.90
        row += f"{int(round(h*100)):4d}" if h>0.005 else "   ."
    print(f"{xx:+.2f} {row}")
