"""Brute-force reach frontier of the TCP (and fingertip) via local Panda FK."""
import numpy as np, itertools, sys
BASE=np.array([-0.66,0,0.912])
DH=[(0,0.333,0),( -np.pi/2,0,0),(np.pi/2,0.316,0),(np.pi/2,0,0.0825),(-np.pi/2,0.384,-0.0825),(np.pi/2,0,0),(np.pi/2,0,0.088)]
def T(alpha,d,a,th):
    ca,sa,ct,st=np.cos(alpha),np.sin(alpha),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
def fk(q):
    M=np.eye(4)
    for (al,d,a),th in zip(DH,q): M=M@T(al,d,a,th)
    M=M@T(0,0.107,0,0)          # flange
    M=M@T(0,0,0,-np.pi/4)       # hand
    p=M[:3,3]+BASE; R=M[:3,:3]
    return p+0.1034*R[:,2], R
if __name__=="__main__":
    if sys.argv[1]=="check":
        q=np.array([0,-0.8,0,-2.5,0,1.7,0.785]); p,R=fk(q); print("park tcp",p.round(4),"hz",R[:,2].round(3))
        q=np.array([0.36,1.48,-0.66,-0.61,0.74,2.43,0.75]); p,R=fk(q); print("seed tcp",p.round(4),"hz",R[:,2].round(3))
    else:
        zlo,zhi=float(sys.argv[1]),float(sys.argv[2])
        best=[]
        for q2 in np.linspace(0.8,1.76,25):
            for q3 in np.linspace(-0.6,0.6,5):
                for q4 in np.linspace(-1.6,-0.07,32):
                    for q5 in np.linspace(-0.8,0.8,5):
                        for q6 in np.linspace(0.0,3.75,38):
                            q=np.array([0,q2,q3,q4,q5,q6,0.785]); p,R=fk(q)
                            if zlo<=p[2]<=zhi and abs(p[1])<0.06:
                                tip=p+0.005*R[:,2]
                                best.append((p[0],tip[0],q2,q3,q4,q5,q6,p[1],p[2],*R[:,2].round(2)))
        best.sort(reverse=True)
        for b in best[:15]: print(np.round(b,3))
