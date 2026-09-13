import numpy as np, sys
cam = sys.argv[1] if len(sys.argv)>1 else "robot0_eye_in_hand"
d = np.load(f"{cam}_cloud.npz"); pw, color = d["pw"], d["color"]
z = pw[...,2]; x = pw[...,0]; y = pw[...,1]; ok=np.isfinite(z)
cx0, cy0 = float(sys.argv[2]) if len(sys.argv)>2 else -0.134, float(sys.argv[3]) if len(sys.argv)>3 else 0.022
zlo = float(sys.argv[4]) if len(sys.argv)>4 else 0.945
r = np.hypot(x-cx0, y-cy0)
m = ok&(r<0.09)&(z>zlo)&(z<zlo+0.03)
print("rim n", m.sum(), "x", round(x[m].min(),4), round(x[m].max(),4), "y", round(y[m].min(),4), round(y[m].max(),4), "z", round(z[m].min(),4), round(z[m].max(),4))
# outer boundary: for angle bins take max r
ang = np.arctan2(y[m]-cy0, x[m]-cx0); rr = r[m]
pts=[]
for a in np.arange(-np.pi, np.pi, np.pi/18):
    mm = (ang>=a)&(ang<a+np.pi/18)
    if mm.sum(): 
        i = np.argmax(rr[mm]); pts.append((x[m][mm][i], y[m][mm][i]))
pts=np.array(pts)
A = np.c_[2*pts[:,0], 2*pts[:,1], np.ones(len(pts))]; b = pts[:,0]**2+pts[:,1]**2
cx, cy, c = np.linalg.lstsq(A,b,rcond=None)[0]; R = np.sqrt(c+cx**2+cy**2)
print(f"outer circle: center ({cx:.4f},{cy:.4f}) R {R:.4f}  from {len(pts)} boundary pts")
print("boundary pts x-min", pts[:,0].min(), "x-max", pts[:,0].max(), "y-min", pts[:,1].min(), "y-max", pts[:,1].max())
