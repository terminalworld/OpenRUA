"""Numerical clearance check of the Franka hand against the lying mug + scene.
Hand frame: y = finger axis, z = pointing (tips at +0.1034), x = thickness.
"""
import numpy as np
R0,R1,L=0.035,0.051,0.106
HINGE=np.array([-0.265,-0.325]); DOOR_DEG=90
def hand_points(gap_half=0.005, fine=0.005):
    pts=[]; lab=[]
    # housing box
    for x in np.arange(-0.03,0.0301,fine):
        for y in np.arange(-0.0995,0.09951,fine):
            for z in np.arange(-0.024,0.0721,fine):
                pts.append((x,y,z)); lab.append('housing')
    # fingers (measured from birdview): pad section z 0.084..0.1034, pad face at |y|=gap_half,
    # body outward 0.014; neck z 0.065..0.084 offset outward gap_half+0.012..+0.026; x +-0.01
    for s in (-1,1):
        for x in np.arange(-0.01,0.0101,fine):
            for y in np.arange(gap_half,gap_half+0.0141,fine/2):
                for z in np.arange(0.084,0.10341,fine/2):
                    pts.append((x,s*y,z)); lab.append('finger%+d'%s)
            for y in np.arange(gap_half+0.012,gap_half+0.0261,fine/2):
                for z in np.arange(0.065,0.084,fine/2):
                    pts.append((x,s*y,z)); lab.append('finger%+d'%s)
    return np.array(pts),np.array(lab)
def mug_model(c,a,elev_deg):
    a=np.array([a[0],a[1],0.0]); a/=np.linalg.norm(a); perp=np.array([-a[1],a[0],0]); c=np.array([c[0],c[1],0.95])
    e=np.radians(elev_deg); rho=np.cos(e)*perp+np.sin(e)*np.array([0,0,1.0])
    return c,a,perp,rho
def penetration(P,c,a,perp,rho):
    """returns per-point penetration depth (>0 inside something) and which"""
    d=P-c; t=d@a; w=d@perp; h=d[:,2]; r=np.hypot(w,h); rt=(R0+R1)/2+(R1-R0)*t/L
    dep=np.zeros(len(P)); what=np.array(['']*len(P),dtype=object)
    # body: solid cone (treat as solid incl. cavity for safety) within |t|<L/2
    inb=(np.abs(t)<L/2)&(r<rt); dep[inb]=np.maximum(dep[inb],np.minimum(rt[inb]-r[inb],L/2-np.abs(t[inb]))); what[inb]='body'
    # handle loop: plane through axis along rho; arms at t=-0.033,+0.020 (1cm), radius 0.043..0.085; bar radius 0.075..0.085
    q=d@rho; s=d@np.cross(a,rho)   # s: offset out of handle plane
    inplane=np.abs(s)<0.006
    for tc in (-0.033,0.020):
        arm=inplane&(np.abs(t-tc)<0.006)&(q>0.03)&(q<0.086)
        dep[arm]=np.maximum(dep[arm],0.003); what[arm]='arm'
    bar=inplane&(t>-0.04)&(t<0.027)&(q>0.074)&(q<0.086)
    dep[bar]=np.maximum(dep[bar],0.003); what[bar]='bar'
    # table
    tb=P[:,2]<0.90; dep[tb]=np.maximum(dep[tb],0.90-P[tb,2]); what[tb]='table'
    # microwave body box (incl door frame region) x -0.265..0.075 y -0.325..-0.125 z<1.107 ; panel x -0.005..0.075 y -0.345..-0.325
    mw=(P[:,0]>-0.265)&(P[:,0]<0.075)&(P[:,1]>-0.325)&(P[:,1]<-0.125)&(P[:,2]<1.107)
    dep[mw]=np.maximum(dep[mw],0.005); what[mw]='mw'
    pn=(P[:,0]>-0.005)&(P[:,0]<0.075)&(P[:,1]>-0.345)&(P[:,1]<-0.325)&(P[:,2]<1.107)
    dep[pn]=np.maximum(dep[pn],0.005); what[pn]='panel'
    # door: hinge, angle
    # door measured at 90 deg open: panel x -0.29..-0.245, y -0.59..-0.325 (handle knob to x -0.31 near y -0.58)
    door=(P[:,0]>-0.295)&(P[:,0]<-0.245)&(P[:,1]>-0.595)&(P[:,1]<-0.325)&(P[:,2]<1.107)
    dep[door]=np.maximum(dep[door],0.005); what[door]='door'
    return dep,what
def pose_from(zh,yh):
    zh=np.array(zh,float); zh/=np.linalg.norm(zh); yh=np.array(yh,float); yh-=yh@zh*zh; yh/=np.linalg.norm(yh); xh=np.cross(yh,zh)
    return np.stack([xh,yh,zh],1)  # columns = hand axes in world
def check(H,Rm,gap_half=0.005,mug=None,verbose=True):
    P,lab=hand_points(gap_half); Pw=(Rm@P.T).T+H
    dep,what=penetration(Pw,*mug)
    bad=dep>0
    if verbose:
        if bad.any():
            for w in set(what[bad]):
                for l in set(lab[bad&(what==w)]):
                    m=bad&(what==w)&(lab==l); print(f"   COLLISION {l} vs {w}: n={m.sum()} maxdepth={dep[m].max():.3f}")
        else: print("   clear")
    return dep,what,Pw,lab
def clearance(Pw,lab,c,a,perp):
    """min distance of housing/finger points to the body cone surface and to door/table (approx)"""
    d=Pw-c; t=d@a; w=d@perp; h=d[:,2]; r=np.hypot(w,h); rt=(R0+R1)/2+(R1-R0)*t/L
    inside_t=np.abs(t)<L/2+0.003
    gap=np.where(inside_t,r-rt,np.hypot(np.maximum(0,r-rt),np.abs(t)-L/2))
    out={}
    for l in set(lab): out[l]=gap[lab==l].min()
    out['table']=(Pw[:,2]-0.9).min()
    return out
