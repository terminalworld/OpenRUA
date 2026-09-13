import numpy as np, rclpy
from tf2_msgs.msg import TFMessage
from geo import quatR
rclpy.init(); n=rclpy.create_node('x'); got={}
n.create_subscription(TFMessage,'/tf',lambda m:[got.setdefault(t.child_frame_id,t.transform) for t in m.transforms],10)
import time; t0=time.time()
while 'robot0_eye_in_hand_optical_frame' not in got and time.time()-t0<10: rclpy.spin_once(n,timeout_sec=0.2)
t=got['robot0_eye_in_hand_optical_frame']; print('cam at',t.translation.x,t.translation.y,t.translation.z)
R=quatR(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w); T=np.array([t.translation.x,t.translation.y,t.translation.z])
d=np.load('eih2_depth.npy').astype(float); H,W=d.shape; f=312.77408948188935
u,v=np.meshgrid(np.arange(W),np.arange(H)); P=np.stack([(u-W/2)*d/f,(v-H/2)*d/f,d],-1)@R.T+T
# can region: pixels near (320,305) radius 60
m=((u-320)**2+(v-305)**2<60**2)&np.isfinite(P[...,2])
p=P[m]; print('z range in region',p[:,2].min().round(3),p[:,2].max().round(3))
top=p[p[:,2]>p[:,2].max()-0.012]
print('can top: n',len(top),'centre',top[:,0].mean().round(4),top[:,1].mean().round(4),'z',top[:,2].mean().round(4),'x[%.3f,%.3f] y[%.3f,%.3f]'%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max()))
