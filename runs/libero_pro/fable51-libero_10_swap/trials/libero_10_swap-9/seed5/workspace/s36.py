import arm, scene, numpy as np, sys, cv2
a=arm.Arm(); T=arm.tilt_y
def go(name,xyz,quat,secs=3):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("ABORT ik",name); sys.exit(1)
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print(name,"-> tcp",np.round(xyz_+0.1034*R[:,2],3),"code",code,flush=True)
#go("view",np.array([-0.17,-0.45,1.02]),T(45,False),4)
D,C,K,R,t=scene.cam_model("robot0_eye_in_hand")
cv2.imwrite("snaps/eih_r.png",C)
P=scene.to_world(D,K,R,t)
hsv=cv2.cvtColor(C,cv2.COLOR_BGR2HSV)
yel=(hsv[...,0]>18)&(hsv[...,0]<38)&(hsv[...,1]>90)&(hsv[...,2]>90)
ok=np.isfinite(P[...,2])
m=yel&ok
pts=P[m]
print("yellow n",m.sum(),"x",np.round([pts[:,0].min(),pts[:,0].max()],3),"y",np.round([pts[:,1].min(),pts[:,1].max()],3),"z",np.round([pts[:,2].min(),pts[:,2].max()],3))
for lo,hi in [(0.94,0.97),(0.97,1.0),(1.0,1.03),(1.03,1.06)]:
    s=pts[(pts[:,2]>=lo)&(pts[:,2]<hi)]
    if len(s): print(f" z{lo}-{hi} n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] ymin5%={np.percentile(s[:,1],5):.3f}")
# anything (any colour) inside the opening region protruding past face plane?
allp=P[ok]
sel=allp[(allp[:,0]>-0.23)&(allp[:,0]<-0.04)&(allp[:,2]>0.95)&(allp[:,2]<1.085)&(allp[:,1]<-0.36)&(allp[:,1]>-0.50)]
print("non-face pts in opening x-range, y<-0.36, z 0.95-1.085:",len(sel))
if len(sel): print("  y range",np.round([sel[:,1].min(),sel[:,1].max()],3),"z",np.round([sel[:,2].min(),sel[:,2].max()],3),"x",np.round([sel[:,0].min(),sel[:,0].max()],3))
