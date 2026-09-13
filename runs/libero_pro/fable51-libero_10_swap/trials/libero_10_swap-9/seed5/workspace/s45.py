import arm, scene, numpy as np, sys, cv2
a=arm.Arm(); T=arm.tilt_y
def go(name,xyz,quat,secs=4):
    q=a.ik(xyz,quat,at_tcp=True,seed=a.arm_q(),timeout=10)
    if q is None: print("IK none",name); return False
    code=a.move_joints([q],seconds_per_seg=secs)
    if code!=0: code=a.move_joints([q],seconds_per_seg=secs)
    xyz_,quat_=a.fk(); R=arm.quat_to_R(quat_)
    print(name,"-> tcp",np.round(xyz_+0.1034*R[:,2],3),"Z",np.round(R[:,2],2),"code",code,flush=True); return code==0
for p in [(-0.15,-0.62,1.15),(-0.15,-0.58,1.2),(-0.17,-0.55,1.25)]:
    if go("look",np.array(p),T(30,False)): break
D,C,K,R,t=scene.cam_model("robot0_eye_in_hand"); cv2.imwrite("snaps/eih_s.png",C)
P=scene.to_world(D,K,R,t); pts=P[np.isfinite(P[...,2])&(D>0.12)]
f=pts[(pts[:,0]>-0.30)&(pts[:,0]<0.08)&(pts[:,1]>-0.50)&(pts[:,1]<-0.30)&(pts[:,2]>0.93)&(pts[:,2]<1.10)]
print("front face pts",len(f))
for xlo in np.arange(-0.30,0.08,0.04):
    q=f[(f[:,0]>=xlo)&(f[:,0]<xlo+0.04)]
    if len(q): print(f" x{xlo:.2f}: n={len(q)} y[{np.percentile(q[:,1],1):.3f},{np.percentile(q[:,1],50):.3f},{np.percentile(q[:,1],99):.3f}] z[{q[:,2].min():.3f},{q[:,2].max():.3f}]")
# any cavity visible (points with y > -0.34 inside opening footprint)?
c=pts[(pts[:,0]>-0.23)&(pts[:,0]<-0.04)&(pts[:,1]>-0.34)&(pts[:,1]<-0.15)&(pts[:,2]>0.94)&(pts[:,2]<1.09)]
print("cavity-interior pts visible",len(c))
