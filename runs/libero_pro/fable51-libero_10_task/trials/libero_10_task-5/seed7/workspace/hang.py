import numpy as np, subprocess, sys
# measure hanging cup from frontview: base ring (x range at z in [tcp_z-0.09, tcp_z-0.05]) and rim (x-min near tcp_z-0.02..tcp_z)
tcp=np.array(list(map(float,sys.argv[1:4])))
subprocess.run(["timeout","120","python3","depth_world.py","frontview"],stdout=subprocess.DEVNULL,check=False)
for _ in range(3):
    if subprocess.run(["timeout","120","python3","depth_world.py","frontview"],stdout=subprocess.DEVNULL).returncode==0: break
xyz=np.load("frontview_world.npz")["xyz"].reshape(-1,3)
m=(xyz[:,2]>tcp[2]-0.12)&(xyz[:,2]<tcp[2]+0.03)&(xyz[:,0]>tcp[0]-0.13)&(xyz[:,0]<tcp[0]+0.13)&(xyz[:,1]>tcp[1]-0.06)&(xyz[:,1]<tcp[1]+0.15)
p=xyz[m]
for zlo in np.arange(tcp[2]-0.12,tcp[2]+0.03,0.01):
    q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.01)]
    if len(q): print(f"  z{zlo:.3f}: x {q[:,0].min():.3f}..{q[:,0].max():.3f} (w {q[:,0].max()-q[:,0].min():.3f} c {(q[:,0].min()+q[:,0].max())/2:.3f}) y {q[:,1].min():.3f}..{q[:,1].max():.3f} n={len(q)}")
