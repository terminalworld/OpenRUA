import numpy as np, scene
D,C,K,R,t=scene.cam_model('robot0_eye_in_hand')
P=scene.to_world(D,K,R,t); X,Y,Z=P[...,0],P[...,1],P[...,2]; ok=np.isfinite(Z)
print("cam",np.round(t,3))
m=ok&(X>-0.25)&(X<-0.02)&(Y>-0.40)&(Y<-0.15)&(Z>0.93)&(Z<1.10)
# exclude points that are the microwave walls: keep x in opening interior
print("n",m.sum())
for lo,hi in [(0.93,0.95),(0.95,0.97),(0.97,0.99),(0.99,1.01),(1.01,1.03),(1.03,1.05),(1.05,1.07),(1.07,1.10)]:
    b=m&(Z>lo)&(Z<hi)
    if b.sum(): print(f"  z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
# yellow color mask to isolate mug
hsv=__import__('cv2').cvtColor(C,__import__('cv2').COLOR_BGR2HSV)
yel=(hsv[...,0]>15)&(hsv[...,0]<40)&(hsv[...,1]>80)&(hsv[...,2]>60)
ym=yel&ok
print("yellow pts",ym.sum())
if ym.sum():
    print(" x",np.round([X[ym].min(),X[ym].max()],3),"y",np.round([Y[ym].min(),Y[ym].max()],3),"z",np.round([Z[ym].min(),Z[ym].max()],3))
    for lo,hi in [(0.93,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.10)]:
        b=ym&(Z>lo)&(Z<hi)
        if b.sum(): print(f"  yel z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}] cx={X[b].mean():.3f}")
# white parts of mug (the body is white/yellow)
wh=(hsv[...,1]<40)&(hsv[...,2]>150)&ok&(Y>-0.40)&(Y<-0.15)&(Z>0.93)&(Z<1.1)&(X>-0.25)&(X<-0.02)
print("white pts",wh.sum())
if wh.sum():
    for lo,hi in [(0.93,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06),(1.06,1.10)]:
        b=wh&(Z>lo)&(Z<hi)
        if b.sum(): print(f"  wh z{lo}-{hi} n={b.sum()} x[{X[b].min():.3f},{X[b].max():.3f}] y[{Y[b].min():.3f},{Y[b].max():.3f}]")
