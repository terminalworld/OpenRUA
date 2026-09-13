import numpy as np
d = np.load("birdview_cloud.npz"); pw = d["pw"]; col = d["color"]
z = pw[..., 2]
# table height: mode of z in the region
print("table z candidates:", np.percentile(z[np.isfinite(z)], [5, 25, 50, 75, 95]))
# Print a coarse height map (world x along rows, y along cols)
xs, ys = pw[...,0], pw[...,1]
# grid over the table
for xg in np.arange(-0.5, 0.6, 0.05):
    row = ""
    for yg in np.arange(-0.6, 0.65, 0.05):
        m = (np.abs(xs-xg)<0.025)&(np.abs(ys-yg)<0.025)
        if m.sum()==0: row += "   . "; continue
        row += f"{np.nanmax(z[m]):5.2f}"
    print(f"x={xg:5.2f} {row}")
print("cols y from -0.6 to 0.6 step 0.05")
