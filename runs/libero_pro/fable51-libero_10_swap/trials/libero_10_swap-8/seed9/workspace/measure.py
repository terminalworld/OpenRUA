import numpy as np
from rob import *
import rclpy
rclpy.init(); node = rclpy.create_node("m")
d = np.load("/workspace/robot0_eye_in_hand_depth.npy")
res = tf_lookup(node, "robot0_eye_in_hand_optical_frame")
K = [312.77408948188935,0,320,0,312.77408948188935,240,0,0,1]
P = cloud_from_depth(d, K, *res)
POT=(-0.196,-0.200)
Q = P[(np.abs(P[:,0]-POT[0])<0.09)&(np.abs(P[:,1]-POT[1])<0.09)]
for z0 in np.arange(0.90,1.07,0.005):
    s = Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.005)]
    if len(s)<5: continue
    print(f"z {z0:.3f}-{z0+0.005:.3f} n={len(s):5d} x[{s[:,0].min():.4f},{s[:,0].max():.4f}] w={s[:,0].max()-s[:,0].min():.4f}  y[{s[:,1].min():.4f},{s[:,1].max():.4f}] w={s[:,1].max()-s[:,1].min():.4f}")
# the pot top seen as a mask: pixels with z>1.02 : the x-extent along rows through the center
top = (P[:,2]>1.02).reshape(d.shape)
ys,xs = np.where(top)
print("top mask px bbox", xs.min(), xs.max(), ys.min(), ys.max(), "cam height above table", res[0][2]-0.90)
