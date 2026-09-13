import numpy as np, sys
P=np.load(sys.argv[1]).reshape(-1,3); P=P[np.isfinite(P).all(1)]
cx,cy=float(sys.argv[2]),float(sys.argv[3]); half=float(sys.argv[4]) if len(sys.argv)>4 else 0.13; res=0.0075
zt=float(sys.argv[5]) if len(sys.argv)>5 else 0.905
xs=np.arange(cx-half,cx+half,res); ys=np.arange(cy-half,cy+half,res)
H=np.full((len(xs),len(ys)),np.nan)
ix=((P[:,0]-(cx-half))/res).astype(int); iy=((P[:,1]-(cy-half))/res).astype(int)
ok=(ix>=0)&(ix<len(xs))&(iy>=0)&(iy<len(ys))
for i,j,z in zip(ix[ok],iy[ok],P[ok,2]):
    if np.isnan(H[i,j]) or z>H[i,j]: H[i,j]=z
# rows = x (increasing downward), cols = y (increasing right); digit = cm above table
print("     y:"+"".join(f"{y:+.2f}"[-3:] if k%4==0 else "   " for k,y in enumerate(ys)))
for i,x in enumerate(xs):
    row=""
    for j in range(len(ys)):
        z=H[i,j]
        if np.isnan(z): row+=" "
        elif z<zt: row+="."
        else:
            d=int(round((z-0.894)*100)); row+=str(min(d,9)) if d<10 else chr(ord('A')+min(d-10,25))
    print(f"x={x:+.3f} {row}")
