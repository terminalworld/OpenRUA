import numpy as np, subprocess, sys
# x-tilt of hanging cup from a +x camera: +x face x-position vs z at y = yc + dy
tcp=np.array(list(map(float,sys.argv[1:4]))); cam=sys.argv[4] if len(sys.argv)>4 else "frontview"
yc=tcp[1]+0.054
for _ in range(3):
    if subprocess.run(["timeout","120","python3","depth_world.py",cam],stdout=subprocess.DEVNULL).returncode==0: break
xyz=np.load(f"{cam}_world.npz")["xyz"].reshape(-1,3)
for dy in (-0.03,-0.02,0.0):
    m=(np.abs(xyz[:,1]-(yc+dy))<0.004)&(xyz[:,0]>tcp[0]-0.02)&(xyz[:,0]<tcp[0]+0.12)&(xyz[:,2]>tcp[2]-0.13)&(xyz[:,2]<tcp[2]+0.02)
    p=xyz[m]
    print(f"dy={dy}: expected offset for vertical cup: rim {np.sqrt(0.054**2-dy**2):.3f} base {np.sqrt(0.042**2-dy**2) if abs(dy)<0.042 else 0:.3f}")
    for zlo in np.arange(tcp[2]-0.13,tcp[2]+0.02,0.01):
        q=p[(p[:,2]>zlo)&(p[:,2]<zlo+0.01)]
        if len(q)>=3: print(f"   z{zlo:.3f}: xmax {q[:,0].max():.3f}  (xmax-tcpx {q[:,0].max()-tcp[0]:+.3f}) n={len(q)}")
