import numpy as np
d = np.load("birdview_cloud.npz"); pw, color = d["pw"], d["color"]
def at(u,v): 
    p = pw[v,u]; return f"({u},{v}) -> x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}  bgr={color[v,u]}"
# table surface sample
for uv in [(320,400),(330,258),(320,258),(340,258),(330,248),(330,268),(305,312),(275,265),(260,265),(295,265),(220,265),(300,235),(300,300),(190,230),(250,300)]:
    print(at(*uv))
# height map: z above table
z = pw[...,2]
print("table z median (region):", np.median(z[380:450, 200:450]))
