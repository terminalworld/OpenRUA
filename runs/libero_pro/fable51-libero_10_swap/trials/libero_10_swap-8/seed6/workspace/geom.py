import numpy as np
def R_to_q(R):
    """rotation matrix -> (x,y,z,w)"""
    m=R; t=np.trace(m)
    if t>0:
        s=np.sqrt(t+1)*2; w=0.25*s; x=(m[2,1]-m[1,2])/s; y=(m[0,2]-m[2,0])/s; z=(m[1,0]-m[0,1])/s
    elif m[0,0]>m[1,1] and m[0,0]>m[2,2]:
        s=np.sqrt(1+m[0,0]-m[1,1]-m[2,2])*2; w=(m[2,1]-m[1,2])/s; x=0.25*s; y=(m[0,1]+m[1,0])/s; z=(m[0,2]+m[2,0])/s
    elif m[1,1]>m[2,2]:
        s=np.sqrt(1+m[1,1]-m[0,0]-m[2,2])*2; w=(m[0,2]-m[2,0])/s; x=(m[0,1]+m[1,0])/s; y=0.25*s; z=(m[1,2]+m[2,1])/s
    else:
        s=np.sqrt(1+m[2,2]-m[0,0]-m[1,1])*2; w=(m[1,0]-m[0,1])/s; x=(m[0,2]+m[2,0])/s; y=(m[1,2]+m[2,1])/s; z=0.25*s
    return np.array([x,y,z,w])
def Ry(t): c,s=np.cos(t),np.sin(t); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rx(t): c,s=np.cos(t),np.sin(t); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def Rz(t): c,s=np.cos(t),np.sin(t); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
R_PICK=np.array([[0,1,0],[1,0,0],[0,0,-1]],float)      # fingers close along world x, z down
R_PLACE=np.array([[1,0,0],[0,-1,0],[0,0,-1]],float)    # fingers close along world y, z down
def lean(R,theta):
    """tilt hand z from -z toward +x by theta (about world y), keeping closing axis y horizontal"""
    return Ry(-theta)@R
def hand_from_tips(tips,R,off=0.1034):
    """hand frame origin given desired fingertip point and orientation"""
    return np.array(tips)-off*R[:,2]
