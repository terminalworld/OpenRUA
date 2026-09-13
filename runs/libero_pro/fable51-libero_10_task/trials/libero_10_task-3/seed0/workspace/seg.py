import numpy as np, cv2
W = np.load("/workspace/bird_world.npy"); wx,wy,wz = W[...,0],W[...,1],W[...,2]
img = cv2.imread("/workspace/birdview.png")
# bottle: region around (335,258), height>0.95
def region(name, u0,u1,v0,v1, zmin, zmax=9):
    m = np.zeros(wz.shape,bool); m[v0:v1,u0:u1]=True
    m &= (wz>zmin)&(wz<zmax)
    if m.sum()==0: print(name,"empty"); return
    print(f"{name}: n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] z[{wz[m].min():.3f},{wz[m].max():.3f}] centroid=({wx[m].mean():.3f},{wy[m].mean():.3f})")
    vs,us = np.where(m); print("   px u[%d,%d] v[%d,%d]"%(us.min(),us.max(),vs.min(),vs.max()))
region("bottle", 315,360,235,285, 0.95)
region("bowl", 280,335,275,330, 0.92)
region("shelf", 190,285,220,320, 0.92)
# cabinet body and drawer
region("cabinet_all", 335,470,240,350, 0.92)
region("cabinet_top(>1.1)", 335,470,240,350, 1.1)
region("drawer_region(0.92-1.1)", 335,470,240,350, 0.92, 1.1)
# drawer interior floor: lower z between drawer walls
region("drawer_interior(<0.95)", 345,400,255,340, 0.905, 0.96)
print("----- refined")
region("bottle_body(0.95-1.15)", 315,360,235,285, 0.95, 1.15)
region("hand(>1.2)", 280,380,200,285, 1.2)
# drawer analysis by columns: for each v row through drawer, print z profile along u
row = 300
print("z profile along row v=300 (u 330..460):")
for u in range(330,460,4):
    print(f"  u={u} x={wx[row,u]:.3f} y={wy[row,u]:.3f} z={wz[row,u]:.3f}")
