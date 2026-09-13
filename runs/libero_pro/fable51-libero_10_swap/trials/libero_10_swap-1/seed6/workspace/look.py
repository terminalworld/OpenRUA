import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
def q_from_dir(d, up=(0,0,1)):
    z=np.array(d,float); z/=np.linalg.norm(z)
    y=np.cross(z,np.array(up,float)); y/=np.linalg.norm(y)   # horizontal
    x=np.cross(y,z)
    R=np.stack([x,y,z],1)
    return tuple(Rot.from_matrix(R).as_quat())  # x,y,z,w
if __name__=="__main__":
    from rob import *
    r=Rob()
    px,py,pz=map(float,sys.argv[1:4]); dx,dy,dz=map(float,sys.argv[4:7])
    q=q_from_dir((dx,dy,dz)); print("q",np.round(q,3))
    ok=r.move_tcp(px,py,pz,q=q,sec=4); print("ok",ok)
    pos,qq,tcp=r.fk_pose(); print("hand",pos.round(3),"q",np.round(qq,3))
    r.shutdown()
