import numpy as np, sys
from rlib import *
from scipy.spatial.transform import Rotation as Ro
import rclpy
from sensor_msgs.msg import Image
def cloud(r, cam):
    got=[]
    sub=r.node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.append(m), 1)
    while not got: rclpy.spin_once(r.node, timeout_sec=0.2)
    r.node.destroy_subscription(sub)
    m=got[0]; d=np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
    fx = 312.77408948188935 if cam=="robot0_eye_in_hand" else 579.4112549695428
    pos,quat=r.tf("world", f"{cam}_optical_frame"); R=Ro.from_quat(quat).as_matrix()
    v,u=np.mgrid[0:m.height,0:m.width]
    X=(u-320)*d/fx; Y=(v-240)*d/fx
    P=np.stack([X,Y,d],-1).reshape(-1,3)@R.T+pos
    ok=np.isfinite(d.reshape(-1))&(d.reshape(-1)>0.05)
    return P[ok]
if __name__=="__main__":
    r=Robot(); P=cloud(r, sys.argv[1])
    lo=np.array([float(v) for v in sys.argv[2].split(",")]); hi=np.array([float(v) for v in sys.argv[3].split(",")])
    S=P[np.all((P>=lo)&(P<=hi),axis=1)]
    print("n",len(S))
    if len(S):
        print("min",np.round(S.min(0),3),"max",np.round(S.max(0),3))
        ax=int(sys.argv[4]) if len(sys.argv)>4 else 2
        for b in np.arange(lo[ax],hi[ax],0.01):
            s=S[(S[:,ax]>=b)&(S[:,ax]<b+0.01)]
            if len(s): print(f"{'xyz'[ax]}[{b:.2f}) n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
