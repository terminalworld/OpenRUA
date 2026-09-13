#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ---ACTIONS; ros2 action list; echo ---SERVICES; ros2 service list | head -60; echo ---NODES; ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once 2>&1 | head -40; for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls *.png

# openrua op 5
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ----; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ----; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80

# openrua op 6
mkdir -p "$(dirname /workspace/cam_cache.py)"
cat > /workspace/cam_cache.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Cache one camera's color, depth, intrinsics and world<-optical TF to
<cam>_cache.npz so pixels can be converted offline many times.

Usage: python3 cam_cache.py <cam> [<cam> ...]
"""
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_T(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z]
    return T


def main():
    rclpy.init()
    node = rclpy.create_node("cam_cache")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    import time
    for cam in sys.argv[1:]:
        color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
        depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        frame = f"{cam}_optical_frame"
        end = time.time() + 15
        while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
            rclpy.spin_once(node, timeout_sec=0.2)
        T = quat_T(tfbuf.lookup_transform("world", frame, rclpy.time.Time()))
        np.savez(f"{cam}_cache.npz", color=color, depth=depth, K=np.array(info.k).reshape(3, 3), T=T)
        cv2.imwrite(f"{cam}.png", color)
        print(cam, depth.shape, "K=", np.array(info.k).reshape(3, 3)[0], "T=", T[:3, 3])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
mkdir -p "$(dirname /workspace/px.py)"
cat > /workspace/px.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Offline pixel -> world from a cam_cache.py npz.

Usage: python3 px.py <cam> u v [u v ...]
       python3 px.py <cam> --box u0 v0 u1 v1   (stats of world pts in box)
"""
import sys

import numpy as np


def load(cam):
    d = np.load(f"{cam}_cache.npz")
    return d["depth"], d["K"], d["T"]


def to_world(depth, K, T, u, v):
    z = depth[v, u]
    if not np.isfinite(z) or z <= 0:
        return None
    p = np.array([(u - K[0, 2]) * z / K[0, 0], (v - K[1, 2]) * z / K[1, 1], z, 1.0])
    return (T @ p)[:3]


def main():
    cam = sys.argv[1]
    depth, K, T = load(cam)
    if sys.argv[2] == "--box":
        u0, v0, u1, v1 = map(int, sys.argv[3:7])
        pts = []
        for v in range(v0, v1 + 1):
            for u in range(u0, u1 + 1):
                p = to_world(depth, K, T, u, v)
                if p is not None:
                    pts.append(p)
        pts = np.array(pts)
        print("n=", len(pts))
        print("min ", pts.min(0))
        print("max ", pts.max(0))
        print("mean", pts.mean(0))
        print("median", np.median(pts, 0))
        return
    args = list(map(int, sys.argv[2:]))
    for u, v in zip(args[::2], args[1::2]):
        p = to_world(depth, K, T, u, v)
        print(f"({u},{v}) depth={depth[v,u]:.4f} ->", None if p is None else np.round(p, 4))


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 200 python3 cam_cache.py birdview agentview frontview sideview 2>&1 | grep -v "^\[" 

# openrua op 9
python3 px.py birdview 247 262 240 255 318 302 425 295 370 235 200 380 500 200 320 240 320 100 && echo --- mug box && python3 px.py birdview --box 232 248 262 278

# openrua op 10
python3 -c "
import numpy as np
d=np.load('birdview_cache.npz'); depth=d['depth']; K=d['K']; T=d['T']
H,W=depth.shape
vv,uu=np.mgrid[0:H,0:W]
z=depth
X=(uu-K[0,2])*z/K[0,0]; Y=(vv-K[1,2])*z/K[1,1]
P=np.stack([X,Y,z,np.ones_like(z)],-1).reshape(-1,4)@T.T
P=P[:,:3].reshape(H,W,3)
# everything above table (z>0.905) in the table region
mask=(P[...,2]>0.91)&(P[...,2]<1.3)
# cluster by connected components in image
import cv2
n,lab=cv2.connectedComponents(mask.astype(np.uint8))
for i in range(1,n):
    m=lab==i
    if m.sum()<30: continue
    pts=P[m]
    us,vs=np.where(m)
    print(f'comp {i}: npx={m.sum()} u[{us.min()},{us.max()}] v[{vs.min()},{vs.max()}]')
    print('   x',pts[:,0].min().round(3),pts[:,0].max().round(3),' y',pts[:,1].min().round(3),pts[:,1].max().round(3),' z',pts[:,2].min().round(3),pts[:,2].max().round(3))
"


# openrua op 11
python3 -c "
import numpy as np
def cloud(cam):
    d=np.load(cam+'_cache.npz'); depth=d['depth']; K=d['K']; T=d['T']
    H,W=depth.shape
    vv,uu=np.mgrid[0:H,0:W]
    z=depth
    X=(uu-K[0,2])*z/K[0,0]; Y=(vv-K[1,2])*z/K[1,1]
    P=np.stack([X,Y,z,np.ones_like(z)],-1).reshape(-1,4)@T.T
    return P[:,:3].reshape(H,W,3)
for cam in ['agentview','frontview','sideview']:
    P=cloud(cam)
    # microwave region points z between 0.92 and 1.1
    m=(P[...,2]>0.93)&(P[...,2]<1.09)&(P[...,1]>0.1)&(P[...,1]<0.5)&(P[...,0]>-0.4)&(P[...,0]<0.3)
    pts=P[m]
    print(cam,'n=',len(pts))
    # histogram in x for points with y in body range
    b=pts[(pts[:,1]>0.25)&(pts[:,1]<0.45)]
    hx,ex=np.histogram(b[:,0],bins=np.arange(-0.4,0.31,0.02))
    print(' x-hist (y in body):',list(zip(ex[:-1].round(2),hx)))
    c=pts[(pts[:,0]>-0.15)&(pts[:,0]<0.12)]
    hy,ey=np.histogram(c[:,1],bins=np.arange(0.1,0.51,0.02))
    print(' y-hist (x in body):',list(zip(ey[:-1].round(2),hy)))
"


# openrua op 12
python3 -c "
import numpy as np, cv2
def cloud(cam):
    d=np.load(cam+'_cache.npz'); depth=d['depth']; K=d['K']; T=d['T']
    H,W=depth.shape
    vv,uu=np.mgrid[0:H,0:W]
    z=depth
    X=(uu-K[0,2])*z/K[0,0]; Y=(vv-K[1,2])*z/K[1,1]
    P=np.stack([X,Y,z,np.ones_like(z)],-1).reshape(-1,4)@T.T
    return P[:,:3].reshape(H,W,3)
# top-down map: x in [-0.5,0.4], y in [-0.5,0.6], 2mm/px ; color by z
S=500
img=np.zeros((int(0.9*1000),int(1.1*1000),3),np.uint8)
cols={'agentview':(0,0,255),'frontview':(0,255,0),'sideview':(255,0,0),'birdview':(255,255,255)}
for cam,c in cols.items():
    P=cloud(cam).reshape(-1,3)
    m=(P[:,2]>0.92)&(P[:,2]<1.2)&(P[:,0]>-0.5)&(P[:,0]<0.4)&(P[:,1]>-0.5)&(P[:,1]<0.6)
    p=P[m]
    r=((p[:,0]+0.5)*1000).astype(int); q=((p[:,1]+0.5)*1000).astype(int)
    img[r,q]=c
cv2.imwrite('topmap.png',img)
"


# openrua op 13
python3 -c "
import numpy as np
def cloud(cam):
    d=np.load(cam+'_cache.npz'); depth=d['depth']; K=d['K']; T=d['T']
    H,W=depth.shape
    vv,uu=np.mgrid[0:H,0:W]
    z=depth
    X=(uu-K[0,2])*z/K[0,0]; Y=(vv-K[1,2])*z/K[1,1]
    P=np.stack([X,Y,z,np.ones_like(z)],-1).reshape(-1,4)@T.T
    return P[:,:3].reshape(H,W,3)
A=cloud('agentview').reshape(-1,3); F=cloud('frontview').reshape(-1,3)
P=np.vstack([A,F])
# cavity interior candidates
m=(P[:,0]>-0.15)&(P[:,0]<0.05)&(P[:,1]>0.28)&(P[:,1]<0.46)
c=P[m]
print('cavity pts',len(c))
hz,ez=np.histogram(c[:,2],bins=np.arange(0.85,1.15,0.01)); print('z-hist',[(round(a,2),b) for a,b in zip(ez[:-1],hz) if b])
hy,ey=np.histogram(c[:,1],bins=np.arange(0.28,0.47,0.01)); print('y-hist',[(round(a,2),b) for a,b in zip(ey[:-1],hy) if b])
hx,ex=np.histogram(c[:,0],bins=np.arange(-0.2,0.1,0.01)); print('x-hist',[(round(a,2),b) for a,b in zip(ex[:-1],hx) if b])
# the -y face / opening frame: points with y in [0.25,0.28]
m2=(P[:,1]>0.25)&(P[:,1]<0.285)&(P[:,0]>-0.25)&(P[:,0]<0.2)&(P[:,2]>0.9)
f=P[m2]
print('front face pts',len(f))
hx,ex=np.histogram(f[:,0],bins=np.arange(-0.25,0.2,0.01)); print('front x-hist',[(round(a,2),b) for a,b in zip(ex[:-1],hx) if b])
hz,ez=np.histogram(f[:,2],bins=np.arange(0.9,1.12,0.01)); print('front z-hist',[(round(a,2),b) for a,b in zip(ez[:-1],hz) if b])
"


# openrua op 14
python3 -c "
import numpy as np
def cloud(cam):
    d=np.load(cam+'_cache.npz'); depth=d['depth']; K=d['K']; T=d['T']
    H,W=depth.shape
    vv,uu=np.mgrid[0:H,0:W]
    z=depth
    X=(uu-K[0,2])*z/K[0,0]; Y=(vv-K[1,2])*z/K[1,1]
    P=np.stack([X,Y,z,np.ones_like(z)],-1).reshape(-1,4)@T.T
    return P[:,:3].reshape(H,W,3)
A=cloud('agentview').reshape(-1,3); F=cloud('frontview').reshape(-1,3)
P=np.vstack([A,F])
m=(P[:,0]>-0.25)&(P[:,0]<0.2)&(P[:,1]>0.27)&(P[:,1]<0.47)&(P[:,2]>0.92)&(P[:,2]<1.08)
c=P[m]
print('interior pts (0.92<z<1.08)',len(c))
for name,i,lo,hi in [('x',0,-0.25,0.2),('y',1,0.27,0.47),('z',2,0.92,1.08)]:
    h,e=np.histogram(c[:,i],bins=np.arange(lo,hi+0.005,0.01)); print(name,[(round(a,2),b) for a,b in zip(e[:-1],h) if b])
# floor points
fl=c[c[:,2]<0.96]
print('floor x range',fl[:,0].min().round(3),fl[:,0].max().round(3),'y range',fl[:,1].min().round(3),fl[:,1].max().round(3), 'z med', np.median(fl[:,2]).round(3))
# side walls: points with z>0.97
w=c[c[:,2]>0.97]
h,e=np.histogram(w[:,0],bins=np.arange(-0.25,0.2,0.01)); print('walls x',[(round(a,2),b) for a,b in zip(e[:-1],h) if b])
h,e=np.histogram(w[:,1],bins=np.arange(0.27,0.47,0.01)); print('walls y',[(round(a,2),b) for a,b in zip(e[:-1],h) if b])
"


# openrua op 15
python3 -c "
import numpy as np
def cloud(cam):
    d=np.load(cam+'_cache.npz'); depth=d['depth']; K=d['K']; T=d['T']
    H,W=depth.shape
    vv,uu=np.mgrid[0:H,0:W]
    z=depth
    X=(uu-K[0,2])*z/K[0,0]; Y=(vv-K[1,2])*z/K[1,1]
    P=np.stack([X,Y,z,np.ones_like(z)],-1).reshape(-1,4)@T.T
    return P[:,:3].reshape(H,W,3)
for cam in ['agentview','frontview','sideview','birdview']:
    P=cloud(cam).reshape(-1,3)
    m=(P[:,0]>-0.22)&(P[:,0]<-0.03)&(P[:,1]>-0.33)&(P[:,1]<-0.13)&(P[:,2]>0.905)
    c=P[m]
    if len(c)==0: print(cam,'none'); continue
    print(cam,'n',len(c),'x',c[:,0].min().round(3),c[:,0].max().round(3),'y',c[:,1].min().round(3),c[:,1].max().round(3),'z',c[:,2].min().round(3),c[:,2].max().round(3))
    h,e=np.histogram(c[:,2],bins=np.arange(0.9,1.12,0.01)); print('  z',[(round(a,2),b) for a,b in zip(e[:-1],h) if b])
    # body (exclude handle): for z in 0.95-1.05 slice, x/y extents
    s=c[(c[:,2]>0.95)&(c[:,2]<1.05)]
    if len(s): print('  slice x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
"


# openrua op 16
python3 px.py agentview 175 245 175 240 175 235 175 225 175 200 175 185 175 250 175 255 200 245; echo; python3 px.py frontview 200 335 200 325 200 300 200 285 200 340 200 345

# openrua op 17
python3 -c "
import numpy as np
def cloud(cam):
    d=np.load(cam+'_cache.npz'); depth=d['depth']; K=d['K']; T=d['T']
    H,W=depth.shape
    vv,uu=np.mgrid[0:H,0:W]
    z=depth
    X=(uu-K[0,2])*z/K[0,0]; Y=(vv-K[1,2])*z/K[1,1]
    P=np.stack([X,Y,z,np.ones_like(z)],-1).reshape(-1,4)@T.T
    return P[:,:3].reshape(H,W,3)
for cam in ['agentview','frontview','sideview']:
    P=cloud(cam).reshape(-1,3)
    m=(P[:,0]>-0.06)&(P[:,0]<0.08)&(P[:,1]>-0.1)&(P[:,1]<0.06)&(P[:,2]>0.905)
    c=P[m]
    print(cam,'yellow mug n',len(c),'z',c[:,2].min().round(3),c[:,2].max().round(3))
    h,e=np.histogram(c[:,2],bins=np.arange(0.9,1.12,0.01)); print('  z',[(round(a,2),b) for a,b in zip(e[:-1],h) if b])
    # white mug: per z-slice horizontal radius
    m=(P[:,0]>-0.22)&(P[:,0]<-0.03)&(P[:,1]>-0.33)&(P[:,1]<-0.13)&(P[:,2]>0.905)
    c=P[m]
    for lo in np.arange(0.97,1.09,0.01):
        s=c[(c[:,2]>=lo)&(c[:,2]<lo+0.01)]
        if len(s): print(f'  white z{lo:.2f}: x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] n={len(s)}')
"


# openrua op 18
python3 -c "
import numpy as np
d=np.load('birdview_cache.npz'); depth=d['depth']; K=d['K']; T=d['T']
H,W=depth.shape
vv,uu=np.mgrid[0:H,0:W]
z=depth
X=(uu-K[0,2])*z/K[0,0]; Y=(vv-K[1,2])*z/K[1,1]
P=(np.stack([X,Y,z,np.ones_like(z)],-1).reshape(-1,4)@T.T)[:,:3]
m=(P[:,0]>-0.22)&(P[:,0]<-0.03)&(P[:,1]>-0.33)&(P[:,1]<-0.13)&(P[:,2]>1.06)
r=P[m]
print('rim pts',len(r))
# fit circle to rim xy (exclude handle: y>-0.19 maybe)
xy=r[:,:2]
A=np.c_[2*xy,np.ones(len(xy))]; b=(xy**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; R=np.sqrt(c[2]+cx**2+cy**2)
print('circle fit all: center',round(cx,4),round(cy,4),'R',round(R,4))
res=np.hypot(xy[:,0]-cx,xy[:,1]-cy)-R
inl=np.abs(res)<0.008
xy2=xy[inl]
A=np.c_[2*xy2,np.ones(len(xy2))]; b=(xy2**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; R=np.sqrt(c[2]+cx**2+cy**2)
print('circle fit inliers',inl.sum(),': center',round(cx,4),round(cy,4),'R',round(R,4))
# interior floor of mug seen from above
m=(P[:,0]>-0.22)&(P[:,0]<-0.03)&(P[:,1]>-0.33)&(P[:,1]<-0.13)&(P[:,2]>0.95)&(P[:,2]<1.0)
f=P[m]; print('inner floor n',len(f),'z med',np.median(f[:,2]).round(4),'x',f[:,0].min().round(3),f[:,0].max().round(3),'y',f[:,1].min().round(3),f[:,1].max().round(3))
# yellow mug too
m=(P[:,0]>-0.06)&(P[:,0]<0.08)&(P[:,1]>-0.1)&(P[:,1]<0.06)&(P[:,2]>1.05)
r=P[m]; xy=r[:,:2]
A=np.c_[2*xy,np.ones(len(xy))]; b=(xy**2).sum(1)
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; R=np.sqrt(c[2]+cx**2+cy**2)
print('yellow rim circle: center',round(cx,4),round(cy,4),'R',round(R,4))
"


# openrua op 19
python3 -c "
import numpy as np
def cloud(cam):
    d=np.load(cam+'_cache.npz'); depth=d['depth']; K=d['K']; T=d['T']
    H,W=depth.shape
    vv,uu=np.mgrid[0:H,0:W]
    z=depth
    X=(uu-K[0,2])*z/K[0,0]; Y=(vv-K[1,2])*z/K[1,1]
    P=np.stack([X,Y,z,np.ones_like(z)],-1).reshape(-1,4)@T.T
    return P[:,:3].reshape(H,W,3)
for cam in ['agentview','frontview']:
    P=cloud(cam).reshape(-1,3)
    m=(P[:,0]>-0.17)&(P[:,0]<-0.15)&(P[:,1]>0.29)&(P[:,1]<0.47)&(P[:,2]>0.9)&(P[:,2]<1.2)
    w=P[m]
    h,e=np.histogram(w[:,2],bins=np.arange(0.9,1.2,0.01)); print(cam,'inner -x wall z',[(round(a,2),b) for a,b in zip(e[:-1],h) if b])
    # top lip front: y in [0.26,0.29], z>1.05
    m=(P[:,1]>0.255)&(P[:,1]<0.295)&(P[:,0]>-0.16)&(P[:,0]<0.06)&(P[:,2]>1.04)
    t=P[m]
    h,e=np.histogram(t[:,2],bins=np.arange(1.04,1.13,0.005)); print(cam,'front lip z',[(round(a,3),b) for a,b in zip(e[:-1],h) if b])
    h,e=np.histogram(t[:,1],bins=np.arange(0.255,0.3,0.005)); print(cam,'front lip y',[(round(a,3),b) for a,b in zip(e[:-1],h) if b])
"


# openrua op 20
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 1.0 2>&1 | tail -2 && timeout 120 python3 cam_cache.py birdview agentview 2>&1 | grep -v "^\["

# openrua op 21
cat > /workspace/measure.py <<'EOF'
#!/usr/bin/env python3
"""Measure the white mug (rim circle, handle, height) and the door line from cached clouds."""
import numpy as np, sys

def cloud(cam):
    d=np.load(cam+'_cache.npz'); depth=d['depth']; K=d['K']; T=d['T']
    H,W=depth.shape
    vv,uu=np.mgrid[0:H,0:W]
    z=depth
    X=(uu-K[0,2])*z/K[0,0]; Y=(vv-K[1,2])*z/K[1,1]
    P=np.stack([X,Y,z,np.ones_like(z)],-1).reshape(-1,4)@T.T
    return P[:,:3].reshape(-1,3)

def circle(xy):
    A=np.c_[2*xy,np.ones(len(xy))]; b=(xy**2).sum(1)
    c=np.linalg.lstsq(A,b,rcond=None)[0]; return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)

B=cloud('birdview'); A=cloud('agentview')
# white mug region (generous)
def mug(P, x0,x1,y0,y1, zmin=0.905):
    m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>zmin)
    return P[m]
for name,(x0,x1,y0,y1) in {'white':(-0.25,-0.02,-0.36,-0.12),'yellow':(-0.08,0.1,-0.12,0.08)}.items():
    b=mug(B,x0,x1,y0,y1); a=mug(A,x0,x1,y0,y1)
    if len(b)==0: print(name,'not found'); continue
    ztop=np.percentile(b[:,2],98)
    rim=b[b[:,2]>ztop-0.012]
    cx,cy,R=circle(rim[:,:2])
    res=np.hypot(rim[:,0]-cx,rim[:,1]-cy)-R; inl=np.abs(res)<0.006
    cx,cy,R=circle(rim[inl][:,:2])
    print(f'{name}: rim z={ztop:.3f} center=({cx:.4f},{cy:.4f}) R={R:.4f}  lowest(side cam)={a[:,2].min():.3f} n_bird={len(b)}')
    # handle: points beyond radius R+0.01 from center
    far=b[np.hypot(b[:,0]-cx,b[:,1]-cy)>R+0.008]
    if len(far):
        ang=np.degrees(np.arctan2(far[:,1]-cy,far[:,0]-cx))
        print(f'   handle: n={len(far)} x[{far[:,0].min():.3f},{far[:,0].max():.3f}] y[{far[:,1].min():.3f},{far[:,1].max():.3f}] z[{far[:,2].min():.3f},{far[:,2].max():.3f}] angle_med={np.median(ang):.0f}deg')
    fa=a[np.hypot(a[:,0]-cx,a[:,1]-cy)>R+0.008]
    if len(fa): print(f'   handle(agentview): y[{fa[:,1].min():.3f},{fa[:,1].max():.3f}] z[{fa[:,2].min():.3f},{fa[:,2].max():.3f}]')
# door: points at z in [0.95,1.09], x<-0.17, y in [-0.1,0.3]
d=B[(B[:,2]>0.95)&(B[:,2]<1.1)&(B[:,0]<-0.17)&(B[:,0]>-0.45)&(B[:,1]>-0.15)&(B[:,1]<0.3)]
if len(d):
    # fit line
    xy=d[:,:2]; c=xy.mean(0); u,s,vt=np.linalg.svd(xy-c); dirv=vt[0]
    t=(xy-c)@dirv
    p0=c+t.min()*dirv; p1=c+t.max()*dirv
    print(f'door: n={len(d)} ends ({p0[0]:.3f},{p0[1]:.3f}) -> ({p1[0]:.3f},{p1[1]:.3f}) ztop={d[:,2].max():.3f}')
EOF
python3 /workspace/measure.py

# openrua op 22
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('urdf_get')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0] if got else '')
print(len(got[0]) if got else 'none')
" 2>&1 | grep -v "^\[" ; grep -n -A12 '<link name="panda_hand"' robot.urdf | head -40; grep -n -A12 '<link name="panda_leftfinger"' robot.urdf | head -30

# openrua op 23
python3 -c "
import numpy as np, struct
def stl_bbox(fn):
    data=open(fn,'rb').read()
    if data[:5]==b'solid':
        import re
        v=np.array([list(map(float,m)) for m in re.findall(r'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',data.decode())])
    else:
        n=struct.unpack('<I',data[80:84])[0]
        arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0),v.max(0)
for f in ['hand','finger']:
    lo,hi=stl_bbox(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl')
    print(f,'min',lo.round(4),'max',hi.round(4))
"

# openrua op 24
find / -iname "hand*.stl" -o -iname "finger*.stl" 2>/dev/null | head; find / -path "*panda*" -name "*.stl" 2>/dev/null | head -5; find / -path "*panda*" -name "*.dae" 2>/dev/null | head -5

# openrua op 25
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Robot helper: joints, FK/IK (MoveIt), trajectories, gripper.

Import and use Rob(), or CLI:
  python3 rob.py js                         # arm joints + finger gap
  python3 rob.py fk                         # TCP pose in world
  python3 rob.py grip open|close
  python3 rob.py movej q1,...,q7 secs
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.660, 0.0, 0.912])  # from tf2_echo world->panda_link0


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def hand_R(approach, finger_axis):
    """Rotation matrix for the panda_hand frame: z = approach (palm->tips),
    y = finger opening axis. Both given in world; orthonormalised."""
    z = np.asarray(approach, float); z /= np.linalg.norm(z)
    y = np.asarray(finger_axis, float); y -= z * (y @ z); y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


class Rob:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        end = time.time() + 15
        while "m" not in self._js and time.time() < end:
            self.spin(0.2)
        m = self._js["m"]
        d = dict(zip(m.name, m.position))
        return np.array([d[j] for j in JOINTS]), d

    def finger_gap(self):
        _, d = self.joints()
        return d.get("panda_finger_joint1", float("nan")), d.get("panda_finger_joint2", float("nan"))

    def fk(self, q=None):
        """Return (tcp_world_xyz, R_hand_world)."""
        if q is None:
            q, _ = self.joints()
        if not self.fk_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no FK service")
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        R = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        hand = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        tcp = hand + TCP_OFF * R[:, 2]
        return tcp, R

    # ---------- IK ----------
    def ik(self, tcp_world, R, seed=None, timeout=5.0):
        """IK for the hand so that the TCP lands at tcp_world with rotation R.
        Returns joint array or None."""
        tcp_world = np.asarray(tcp_world, float)
        hand_world = tcp_world - TCP_OFF * R[:, 2]
        hand_base = hand_world - BASE_IN_WORLD
        if seed is None:
            seed, _ = self.joints()
        if not self.ik_cli.wait_for_service(timeout_sec=10):
            raise RuntimeError("no IK service")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_base)
        qx, qy, qz, qw = R_to_quat(R)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    def ik_near(self, tcp_world, R, seed=None, tries=6, max_dist=2.5):
        """IK, retrying with perturbed seeds; prefers the solution closest
        to the seed. Returns (q, dist) or (None, None)."""
        if seed is None:
            seed, _ = self.joints()
        best, bd = None, None
        rng = np.random.default_rng(0)
        for i in range(tries):
            s = seed if i == 0 else seed + rng.normal(0, 0.3, len(seed))
            s = np.clip(s, [l[0] for l in LIMITS], [l[1] for l in LIMITS])
            q = self.ik(tcp_world, R, seed=s)
            if q is None:
                continue
            d = float(np.abs(q - seed).max())
            if bd is None or d < bd:
                best, bd = q, d
            if d < 0.8:
                break
        if best is not None and bd > max_dist:
            print(f"[ik_near] warning: best solution is far from seed (max joint delta {bd:.2f})")
        return best, bd

    # ---------- action ----------
    def movej(self, waypoints, times):
        """waypoints: list of joint arrays; times: cumulative seconds per point."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for q, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        r = rf.result()
        code = r.result.error_code if r else None
        q, _ = self.joints()
        err = float(np.abs(q - np.asarray(waypoints[-1])).max())
        print(f"[movej] error_code={code} final max joint err={err:.4f}")
        return code, err

    def move_tcp(self, tcp_world, R, secs=3.0, seed=None, via=None):
        q, d = self.ik_near(tcp_world, R, seed=seed)
        if q is None:
            print(f"[move_tcp] IK FAILED for {np.round(tcp_world,3)}")
            return None
        print(f"[move_tcp] -> {np.round(tcp_world,3)} joint delta {d:.2f}")
        wps, ts = ([], [])
        if via is not None:
            wps.append(via); ts.append(secs * 0.5)
        wps.append(q); ts.append(secs)
        self.movej(wps, ts)
        tcp, Rn = self.fk()
        print(f"[move_tcp] reached tcp={np.round(tcp,4)} (target {np.round(tcp_world,4)}), pos err={np.linalg.norm(tcp-tcp_world):.4f}")
        return q

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result().result
        gap = self.finger_gap()
        print(f"[gripper] cmd={width} reached={res.reached_goal} stalled={res.stalled} fingers={gap}")
        return gap


def main():
    r = Rob()
    cmd = sys.argv[1]
    if cmd == "js":
        q, d = r.joints()
        print("arm:", np.round(q, 4).tolist())
        print("fingers:", r.finger_gap())
    elif cmd == "fk":
        tcp, R = r.fk()
        print("tcp world:", np.round(tcp, 4).tolist())
        print("R:\n", np.round(R, 3))
    elif cmd == "grip":
        r.gripper(GRIP["open_m"] if sys.argv[2] == "open" else GRIP["closed_m"])
    elif cmd == "movej":
        q = [float(x) for x in sys.argv[2].split(",")]
        r.movej([q], [float(sys.argv[3])])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 26
timeout 120 python3 rob.py js 2>&1 | grep -v "^\[INFO\|^\[WARN" ; timeout 120 python3 rob.py fk 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 27
timeout 120 python3 -c "
from rob import *
r=Rob('dbg')
q,_=r.joints()
req=GetPositionFK.Request(); req.header.frame_id=''; req.fk_link_names=['panda_hand','panda_link0']
req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=[float(v) for v in q]
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
res=fut.result()
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
print(res.fk_link_names, res.error_code)
" 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 28
timeout 200 python3 -c "
from rob import *
r=Rob('dbg')
q,_=r.joints()
for names in (['panda_hand'],['panda_link0'],['panda_link1']):
    req=GetPositionFK.Request(); req.header.frame_id=''; req.fk_link_names=names
    req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=[float(v) for v in q]
    fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=60)
    res=fut.result()
    if res is None: print(names,'no answer'); continue
    for ps in res.pose_stamped: print(names, 'frame=',repr(ps.header.frame_id), [round(ps.pose.position.x,4),round(ps.pose.position.y,4),round(ps.pose.position.z,4)])
    print('  code',res.error_code.val)
" 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 29
timeout 300 python3 -c "
from rob import *
r=Rob('dbg')
q0,_=r.joints()
R=np.array([[0.998,0,-0.057],[0,-1,0],[-0.057,0,-0.998]])
hand_world=np.array([-0.203,0.0,1.270])
for label,off in (('world-coords',np.zeros(3)),('base-coords',BASE_IN_WORLD)):
    # r.ik subtracts BASE_IN_WORLD from hand_world; compensate to test both conventions
    tcp=hand_world+TCP_OFF*R[:,2]+off
    q=r.ik(tcp,R,seed=q0)
    print(label, None if q is None else np.round(q-q0,3).tolist())
" 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 30
python3 - <<'EOF'
import re
s=open('rob.py').read()
s=s.replace('BASE_IN_WORLD = np.array([-0.660, 0.0, 0.912])  # from tf2_echo world->panda_link0',
            'BASE_IN_WORLD = np.zeros(3)  # MoveIt model frame here IS world (FK of panda_link0 -> (-0.66,0,0.912))')
s=s.replace('        req.ik_request.group_name = M["planning"]["group"]\n',
            '        req.ik_request.group_name = M["planning"]["group"]\n        req.ik_request.ik_link_name = "panda_hand"  # group tip is panda_link8 (45 deg off)\n')
open('rob.py','w').write(s)
EOF
timeout 300 python3 -c "
from rob import *
r=Rob('dbg')
q0,_=r.joints()
tcp,R=r.fk(); print('fk tcp',np.round(tcp,4), 'R z',np.round(R[:,2],3))
q=r.ik(tcp,R,seed=q0)
print('ik delta', None if q is None else np.round(q-q0,3).tolist())
" 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 31
cat > /workspace/plan.py <<'EOF'
import numpy as np
from rob import *
A = np.radians(60)
# grasp orientation: fingers along world x, approach down & toward -y (palm leans +y)
R_G = hand_R([0, -np.sin(A), -np.cos(A)], [1, 0, 0])
# place orientation (after 180 deg yaw): approach down & toward +y (palm leans -y)
R_P = hand_R([0, np.sin(A), -np.cos(A)], [-1, 0, 0])
HX, HY, HZ = -0.124, -0.172, 0.935          # handle bar grasp point (TCP)
POSES = {
 'pregrasp': (np.array([HX, HY, HZ]) - 0.09 * R_G[:, 2], R_G),
 'grasp':    (np.array([HX, HY, HZ]), R_G),
 'lift':     (np.array([HX, HY, 1.13]), R_G),
 'rotated':  (np.array([HX, HY, 1.13]), R_P),
 'preinsert_hi': (np.array([-0.045, 0.13, 1.03]), R_P),
 'preinsert': (np.array([-0.045, 0.13, 0.995]), R_P),
 'insert':   (np.array([-0.045, 0.25, 0.995]), R_P),
 'lower':    (np.array([-0.045, 0.25, 0.982]), R_P),
}
EOF
timeout 600 python3 -c "
from plan import *
r=Rob('plantest')
q,_=r.joints()
print('R_G z',np.round(R_G[:,2],3),'y',np.round(R_G[:,1],3),' R_P z',np.round(R_P[:,2],3),'y',np.round(R_P[:,1],3))
seed=q
for k,(p,R) in POSES.items():
    sol,d=r.ik_near(p,R,seed=seed)
    print(k, np.round(p,3), 'IK', None if sol is None else np.round(sol,3).tolist(), 'delta', d)
    if sol is not None: seed=sol
" 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 32
timeout 600 python3 -c "
from plan import *
r=Rob('pathtest')
for zl in (1.13, 1.25):
    qa,_=r.ik_near(np.array([HX,HY,zl]),R_G,seed=np.array([0.677, 0.355, -0.769, -2.19, -1.06, 2.175, -0.077]))
    qb,_=r.ik_near(np.array([HX,HY,zl]),R_P,seed=qa)
    print('z',zl,'qa',np.round(qa,2).tolist(),'qb',np.round(qb,2).tolist(),'maxdelta',np.abs(qb-qa).max().round(2))
    pts=[]
    for s in np.linspace(0,1,11):
        q=qa+(qb-qa)*s
        tcp,R=r.fk(q); pts.append(tcp)
        print(f'   s={s:.1f} tcp={np.round(tcp,3)} zhand={np.round(R[:,2],2)}')
" 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 33
cat > /workspace/step_yellow.py <<'EOF'
import numpy as np
from rob import *
r = Rob('yellow')
R_V = hand_R([0, 0, -1], [1, 0, 0])   # vertical, fingers along x
cx, cy, rimz, R = 0.0115, -0.0041, 1.004, 0.0477
grasp = np.array([cx + R, cy, rimz - 0.024])
print('fingers now', r.finger_gap())
r.gripper(0.04)
r.move_tcp(grasp + [0, 0, 0.10], R_V, 4.0)
r.move_tcp(grasp, R_V, 2.5)
gap = r.gripper(0.0)
r.move_tcp(grasp + [0, 0, 0.15], R_V, 2.5)
print('after lift fingers', r.finger_gap())
dest = np.array([-0.30 + R, -0.35, rimz - 0.024])
r.move_tcp(dest + [0, 0, 0.15], R_V, 4.0)
r.move_tcp(dest + [0, 0, 0.01], R_V, 2.5)
r.gripper(0.04)
r.move_tcp(dest + [0, 0, 0.12], R_V, 2.5)
print('DONE')
EOF
timeout 1500 python3 -u step_yellow.py 2>&1 | grep -v "^\[INFO\|^\[WARN"

# openrua op 34
timeout 200 python3 cam_cache.py birdview agentview 2>&1 | grep -v "^\[" && python3 measure.py

# openrua op 35
mkdir -p "$(dirname /workspace/step_white_a.py)"
cat > /workspace/step_white_a.py <<'OPENRUA_EOF'
"""White mug: grasp handle bar and lift. Stops after lift for visual check."""
import numpy as np
from rob import *
from plan import R_G

HX, HY, HZ = -0.124, -0.172, 0.945
grasp = np.array([HX, HY, HZ])
pregrasp = grasp - 0.09 * R_G[:, 2]
lift = np.array([HX, HY, 1.13])

r = Rob("white_a")
q0, _ = r.joints()
print("start joints", np.round(q0, 3).tolist())
print("start tcp", np.round(r.fk()[0], 4).tolist(), "fingers", r.finger_gap())

r.gripper(0.04)
q = r.move_tcp(pregrasp, R_G, secs=8.0)
assert q is not None
q = r.move_tcp(grasp, R_G, secs=3.0, seed=q)
assert q is not None
gap = r.gripper(0.0)
print("GAP after close:", gap)
q = r.move_tcp(lift, R_G, secs=3.0, seed=q)
print("fingers after lift:", r.finger_gap())
print("DONE_A")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 36
timeout 600 python3 step_white_a.py 2>&1 | grep -v "^\[WARN\]"

# openrua op 37
timeout 120 python3 cam_cache.py agentview birdview 2>&1 | grep -v WARN

# openrua op 38
timeout 300 python3 -c "
from rob import *
r=Rob('park')
R_V = hand_R([0,0,-1],[0,1,0])
r.move_tcp(np.array([-0.35,-0.05,1.25]), R_V, secs=6.0)
rclpy.shutdown()
" 2>&1 | grep -v WARN && timeout 120 python3 cam_cache.py agentview birdview sideview frontview 2>&1 | grep -v WARN

# openrua op 39
python3 -c "
import numpy as np
d=np.load('birdview_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
z=depth
X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
P=P[...,:3]
m=(P[...,2]>0.905)&(P[...,0]>-0.2)&(P[...,0]<0.05)&(P[...,1]>-0.35)&(P[...,1]<-0.12)
pts=P[m]; print('n',len(pts)); print('min',pts.min(0)); print('max',pts.max(0))
# height histogram
for zlo in np.arange(0.90,1.02,0.01):
    s=pts[(pts[:,2]>=zlo)&(pts[:,2]<zlo+0.01)]
    if len(s): print(f'z {zlo:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
# top-down occupancy map in 1cm cells
xs=np.arange(-0.2,0.05,0.01); ys=np.arange(-0.35,-0.12,0.01)
print('     '+''.join(f'{y*100:+.0f}'[-2:] for y in ys))
for x in xs:
    row=''
    for y in ys:
        s=pts[(abs(pts[:,0]-x)<0.005)&(abs(pts[:,1]-y)<0.005)]
        row+= ' .' if len(s)==0 else f'{int((s[:,2].max()-0.9)*100):2d}'
    print(f'{x:+.2f} '+row)
"


# openrua op 40
python3 -c "
import numpy as np
d=np.load('birdview_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
z=depth
X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
P=P[...,:3]
m=(P[...,2]>0.905)&(P[...,2]<1.05)&(P[...,0]>-0.15)&(P[...,0]<0.06)&(P[...,1]>-0.25)&(P[...,1]<0.0)
pts=P[m]; print('n',len(pts)); print('min',pts.min(0)); print('max',pts.max(0))
xs=np.arange(-0.12,0.05,0.01); ys=np.arange(-0.24,0.0,0.01)
print('      '+''.join(f'{abs(y)*100:3.0f}' for y in ys))
for x in xs:
    row=''
    for y in ys:
        s=pts[(abs(pts[:,0]-x)<0.005)&(abs(pts[:,1]-y)<0.005)]
        row+= '  .' if len(s)==0 else f'{int((s[:,2].max()-0.9)*100):3d}'
    print(f'{x:+.2f} '+row)
"


# openrua op 41
python3 -c "
import numpy as np
for cam in ['sideview','agentview','frontview']:
    d=np.load(cam+'_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']
    H,W=depth.shape
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    z=depth
    X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
    P=(np.stack([X,Y,z,np.ones_like(z)],-1)@T.T)[...,:3]
    m=(P[...,2]>0.905)&(P[...,2]<1.0)&(P[...,0]>-0.11)&(P[...,0]<0.04)&(P[...,1]>-0.25)&(P[...,1]<-0.05)
    pts=P[m]; print(cam,'n',len(pts))
    if len(pts)==0: continue
    for ylo in np.arange(-0.25,-0.05,0.01):
        s=pts[(pts[:,1]>=ylo)&(pts[:,1]<ylo+0.01)]
        if len(s): print(f'  y {ylo:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
"


# openrua op 42
mkdir -p "$(dirname /workspace/step_white_b.py)"
cat > /workspace/step_white_b.py <<'OPENRUA_EOF'
"""White mug is lying on its side (axis along y, rim facing +y at y=-0.10,
body x in [-0.058,0.032], centre z 0.949). Pinch the rim wall at its +x side
with a near-horizontal hand pointing -y (tilted 20 deg down), lift, stop."""
import numpy as np
from rob import *

B = np.radians(20)
APP = np.array([0, -np.cos(B), -np.sin(B)])      # approach: -y, slightly down
R_S = hand_R(APP, [1, 0, 0])                     # fingers close along x
PX, PY, PZ = 0.030, -0.125, 0.949                # wall centre, 2.5 cm inside rim
grasp = np.array([PX, PY, PZ])
pregrasp = grasp - 0.08 * APP
lift = grasp + np.array([0, 0, 0.25])

r = Rob("white_b")
print("start tcp", np.round(r.fk()[0], 4).tolist(), "fingers", r.finger_gap())
r.gripper(0.04)

q, d = r.ik_near(pregrasp, R_S)
print("pregrasp IK", None if q is None else np.round(q, 3).tolist(), d)
assert q is not None
# check link positions of the solution for door/table clearance
from moveit_msgs.srv import GetPositionFK
req = GetPositionFK.Request(); req.header.frame_id = ""
req.fk_link_names = [f"panda_link{i}" for i in range(3, 9)] + ["panda_hand"]
req.robot_state.joint_state.name = list(JOINTS)
req.robot_state.joint_state.position = [float(v) for v in q]
fut = r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
for n, ps in zip(req.fk_link_names, fut.result().pose_stamped):
    p = ps.pose.position; print(f"  {n}: ({p.x:.3f},{p.y:.3f},{p.z:.3f})")

q = r.move_tcp(pregrasp, R_S, secs=8.0, seed=q)
assert q is not None
q2 = r.move_tcp(pregrasp, R_S, secs=3.0, seed=q)   # resend to converge
q = r.move_tcp(grasp, R_S, secs=3.0, seed=q)
assert q is not None
gap = r.gripper(0.0)
print("GAP after close:", gap)
q = r.move_tcp(lift, R_S, secs=4.0, seed=q)
print("fingers after lift:", r.finger_gap())
print("DONE_B")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 43
timeout 300 python3 -c "
import numpy as np
from rob import *
from moveit_msgs.srv import GetPositionFK
B=np.radians(20); APP=np.array([0,-np.cos(B),-np.sin(B)]); R_S=hand_R(APP,[1,0,0])
grasp=np.array([0.030,-0.125,0.949]); pre=grasp-0.08*APP; lift=grasp+[0,0,0.25]
r=Rob('dry')
seed,_=r.joints()
for name,p in [('pre',pre),('grasp',grasp),('lift',lift)]:
    q,d=r.ik_near(p,R_S,seed=seed)
    print(name, np.round(p,3).tolist(), None if q is None else np.round(q,3).tolist(), d)
    if q is None: continue
    seed=q
    req=GetPositionFK.Request(); req.header.frame_id=''
    req.fk_link_names=[f'panda_link{i}' for i in range(2,9)]+['panda_hand']
    req.robot_state.joint_state.name=list(JOINTS); req.robot_state.joint_state.position=[float(v) for v in q]
    fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
    print('   '+' '.join(f'{n[-5:]}=({ps.pose.position.x:.2f},{ps.pose.position.y:.2f},{ps.pose.position.z:.2f})' for n,ps in zip(req.fk_link_names,fut.result().pose_stamped)))
rclpy.shutdown()
" 2>&1 | grep -v WARN

# openrua op 44
mkdir -p "$(dirname /workspace/pathcheck.py)"
cat > /workspace/pathcheck.py <<'OPENRUA_EOF'
"""Sample joint-space interpolation between configs and report link poses
that come near the microwave / door / table. Usage from python:
   from pathcheck import check; check(r, q0, q1)"""
import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionFK
from rob import JOINTS

LINKS = [f"panda_link{i}" for i in range(3, 9)] + ["panda_hand"]
RAD = {"panda_link3": 0.07, "panda_link4": 0.07, "panda_link5": 0.07, "panda_link6": 0.07,
       "panda_link7": 0.05, "panda_link8": 0.05, "panda_hand": 0.1}


def fk_links(r, q):
    req = GetPositionFK.Request(); req.header.frame_id = ""
    req.fk_link_names = LINKS
    req.robot_state.joint_state.name = list(JOINTS)
    req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk_cli.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    return {n: np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
            for n, ps in zip(LINKS, fut.result().pose_stamped)}


def hazards(p, rad):
    h = []
    x, y, z = p
    # microwave body x[-0.19,0.17] y[0.265,0.465] z<1.107
    if -0.19 - rad < x < 0.17 + rad and 0.265 - rad < y < 0.465 + rad and z < 1.107 + rad:
        h.append("MICROWAVE")
    # open door: segment (-0.19,0.265)->(-0.30,0.03), z<1.1
    a, b = np.array([-0.19, 0.265]), np.array([-0.30, 0.03])
    t = np.clip(((np.array([x, y]) - a) @ (b - a)) / ((b - a) @ (b - a)), 0, 1)
    if np.linalg.norm(np.array([x, y]) - (a + t * (b - a))) < rad and z < 1.1 + rad:
        h.append("DOOR")
    if z < 0.90 + rad and x > -0.55:
        h.append("TABLE")
    return h


def check(r, q0, q1, n=8, verbose=False):
    bad = False
    for i in range(n + 1):
        q = q0 + (q1 - q0) * i / n
        L = fk_links(r, q)
        zmin = min(p[2] for p in L.values())
        flags = {k: hazards(p, RAD[k]) for k, p in L.items()}
        flags = {k: v for k, v in flags.items() if v}
        if flags or verbose:
            print(f"  step {i}/{n}: zmin={zmin:.3f} " + " ".join(f"{k[-5:]}{v}@{np.round(L[k],2).tolist()}" for k, v in flags.items()))
        bad |= bool(flags)
    print("  path", "HAZARD" if bad else "clear")
    return not bad
OPENRUA_EOF

# openrua op 45
timeout 300 python3 -c "
import numpy as np
from rob import *
from pathcheck import check
B=np.radians(20); APP=np.array([0,-np.cos(B),-np.sin(B)]); R_S=hand_R(APP,[1,0,0])
grasp=np.array([0.030,-0.125,0.949]); pre=grasp-0.08*APP
r=Rob('dry')
q0,_=r.joints(); print('q0',np.round(q0,3).tolist())
q1=np.array([2.703, -1.429, -2.075, -1.326, -2.127, 1.1, -0.012])
check(r,q0,q1,n=10,verbose=True)
rclpy.shutdown()
" 2>&1 | grep -v WARN

# openrua op 46
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pathcheck.py", "old_string": "    fut = r.fk_cli.call_async(req)\\n    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)\\n    return {n: np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])\\n            for n, ps in zip(LINKS, fut.result().pose_stamped)}", "new_string": "    r.fk_cli.wait_for_service(timeout_sec=10)\\n    for _ in range(3):\\n        fut = r.fk_cli.call_async(req)\\n        rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)\\n        if fut.result() is not None:\\n            break\\n    return {n: np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])\\n            for n, ps in zip(LINKS, fut.result().pose_stamped)}", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 47
timeout 300 python3 -c "
import numpy as np
from rob import *
from pathcheck import check
r=Rob('dry')
q0,_=r.joints(); print('q0',np.round(q0,3).tolist())
q1=np.array([2.703, -1.429, -2.075, -1.326, -2.127, 1.1, -0.012])
check(r,q0,q1,n=10,verbose=True)
rclpy.shutdown()
" 2>&1 | grep -v WARN

# openrua op 48
mkdir -p "$(dirname /workspace/step_white_b.py)"
cat > /workspace/step_white_b.py <<'OPENRUA_EOF'
"""White mug is lying on its side (axis along y, rim facing +y at y=-0.10,
body x in [-0.058,0.032], centre z 0.949). Pinch the rim wall at its +x side
with a near-horizontal hand pointing -y (tilted 20 deg down), lift, stop."""
import numpy as np
from rob import *
from pathcheck import check

B = np.radians(20)
APP = np.array([0, -np.cos(B), -np.sin(B)])      # approach: -y, slightly down
R_S = hand_R(APP, [1, 0, 0])                     # fingers close along x
PX, PY, PZ = 0.030, -0.125, 0.949                # wall centre, 2.5 cm inside rim
grasp = np.array([PX, PY, PZ])
pregrasp = grasp - 0.08 * APP
lift = grasp + np.array([0, 0, 0.25])

r = Rob("white_b")
print("start tcp", np.round(r.fk()[0], 4).tolist(), "fingers", r.finger_gap())
r.gripper(0.04)


def go(p, R, secs, seed=None):
    q0, _ = r.joints()
    q, d = r.ik_near(p, R, seed=seed if seed is not None else q0)
    assert q is not None, f"IK failed {p}"
    assert check(r, q0, q), "hazard on path"
    r.movej([q], [secs])
    qn, _ = r.joints()
    if np.abs(qn - q).max() > 0.02:
        print("  resend to converge")
        r.movej([q], [2.0])
    tcp, _ = r.fk()
    print(f"[go] tcp={np.round(tcp,4).tolist()} target={np.round(p,4).tolist()} err={np.linalg.norm(tcp-p):.4f}")
    return q


q = go(pregrasp, R_S, 8.0)
q = go(grasp, R_S, 3.0, seed=q)
gap = r.gripper(0.0)
print("GAP after close:", gap)
q = go(lift, R_S, 4.0, seed=q)
print("fingers after lift:", r.finger_gap())
print("DONE_B")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 49
timeout 600 python3 step_white_b.py 2>&1 | grep -v "^\[WARN\]"

# openrua op 50
timeout 120 python3 -c "
import numpy as np
from rob import *
r=Rob('c')
q,_=r.joints(); print('q',np.round(q,3).tolist())
tcp,R=r.fk(); print('tcp',np.round(tcp,4).tolist()); print(np.round(R,2))
rclpy.shutdown()
" 2>&1 | grep -v WARN

# openrua op 51
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def movej(self, waypoints, times):\\n        \\"\\"\\"waypoints: list of joint arrays; times: cumulative seconds per point.\\"\\"\\"\\n        if not self.fjt.wait_for_server(timeout_sec=10):\\n            raise RuntimeError(\\"no FJT server\\")\\n        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = list(JOINTS)\\n        for q, t in zip(waypoints, times):\\n            pt = JointTrajectoryPoint(positions=[float(v) for v in q])\\n            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))\\n            goal.trajectory.points.append(pt)\\n        send = self.fjt.send_goal_async(goal)\\n        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)\\n        gh = send.result()\\n        if gh is None or not gh.accepted:\\n            raise RuntimeError(\\"trajectory goal rejected\\")\\n        rf = gh.get_result_async()\\n        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)\\n        r = rf.result()\\n        code = r.result.error_code if r else None\\n        q, _ = self.joints()\\n        err = float(np.abs(q - np.asarray(waypoints[-1])).max())\\n        print(f\\"[movej] error_code={code} final max joint err={err:.4f}\\")\\n        return code, err", "new_string": "    def _send_traj(self, waypoints, times):\\n        if not self.fjt.wait_for_server(timeout_sec=10):\\n            raise RuntimeError(\\"no FJT server\\")\\n        goal = FollowJointTrajectory.Goal()\\n        goal.trajectory.joint_names = list(JOINTS)\\n        for q, t in zip(waypoints, times):\\n            pt = JointTrajectoryPoint(positions=[float(v) for v in q])\\n            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))\\n            goal.trajectory.points.append(pt)\\n        send = self.fjt.send_goal_async(goal)\\n        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)\\n        gh = send.result()\\n        if gh is None or not gh.accepted:\\n            raise RuntimeError(\\"trajectory goal rejected\\")\\n        rf = gh.get_result_async()\\n        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)\\n        r = rf.result()\\n        return r.result.error_code if r else None\\n\\n    def movej(self, waypoints, times, settle_tol=0.01, settle_tries=3):\\n        \\"\\"\\"waypoints: list of joint arrays; times: cumulative seconds per point.\\n        The controller lags on big moves (sim clock only runs while a goal is\\n        active), so after the goal we re-send the final point as a hold until\\n        the joints settle within settle_tol.\\"\\"\\"\\n        code = self._send_traj(waypoints, times)\\n        tgt = np.asarray(waypoints[-1])\\n        q, _ = self.joints()\\n        err = float(np.abs(q - tgt).max())\\n        print(f\\"[movej] error_code={code} final max joint err={err:.4f}\\")\\n        for _ in range(settle_tries):\\n            if err <= settle_tol:\\n                break\\n            code = self._send_traj([tgt], [3.0])\\n            q, _ = self.joints()\\n            err = float(np.abs(q - tgt).max())\\n            print(f\\"[movej]   settle: error_code={code} max joint err={err:.4f}\\")\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 52
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/pathcheck.py", "old_string": "       \\"panda_link7\\": 0.05, \\"panda_link8\\": 0.05, \\"panda_hand\\": 0.1}", "new_string": "       \\"panda_link7\\": 0.05, \\"panda_link8\\": 0.05, \\"panda_hand\\": 0.05}", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 53
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/step_white_b.py", "old_string": "    r.movej([q], [secs])\\n    qn, _ = r.joints()\\n    if np.abs(qn - q).max() > 0.02:\\n        print(\\"  resend to converge\\")\\n        r.movej([q], [2.0])\\n    tcp, _ = r.fk()", "new_string": "    r.movej([q], [secs])\\n    tcp, _ = r.fk()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 54
timeout 600 python3 step_white_b.py 2>&1 | grep -v "^\[WARN\]"

# openrua op 55
timeout 120 python3 cam_cache.py agentview birdview robot0_eye_in_hand frontview 2>&1 | grep -v WARN

# openrua op 56
timeout 120 python3 -c "
import numpy as np
from rob import *
r=Rob('h'); tcp,R=r.fk(); hand=tcp-TCP_OFF*R[:,2]; print('hand',np.round(hand,3),'R',np.round(R,2).tolist())
rclpy.shutdown()
allp=[]
for cam in ['agentview','frontview','birdview']:
    d=np.load(cam+'_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']
    H,W=depth.shape; u,v=np.meshgrid(np.arange(W),np.arange(H)); z=depth
    X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
    P=(np.stack([X,Y,z,np.ones_like(z)],-1)@T.T)[...,:3].reshape(-1,3)
    P=P[np.isfinite(P).all(1)]
    allp.append(P)
P=np.vstack(allp)
L=(P-hand)@R   # hand coords
m=(abs(L[:,0])<0.2)&(abs(L[:,1])<0.2)&(L[:,2]>-0.25)&(L[:,2]<0.15)
L=L[m]; print('n',len(L))
for zlo in np.arange(-0.25,0.15,0.01):
    s=L[(L[:,2]>=zlo)&(L[:,2]<zlo+0.01)]
    if len(s): print(f'hz {zlo:+.2f}: n={len(s):4d} hx[{s[:,0].min():+.3f},{s[:,0].max():+.3f}] hy[{s[:,1].min():+.3f},{s[:,1].max():+.3f}]')
" 2>&1 | grep -v WARN

# openrua op 57
timeout 120 python3 -c "
import numpy as np
hand=np.array([0.03,-0.028,1.234]); R=np.array([[0.0,1.0,0.0],[0.34,0.0,-0.94],[-0.94,0.0,-0.34]])
allp=[]
for cam in ['agentview','frontview','birdview']:
    d=np.load(cam+'_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']
    H,W=depth.shape; u,v=np.meshgrid(np.arange(W),np.arange(H)); z=depth
    X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
    P=(np.stack([X,Y,z,np.ones_like(z)],-1)@T.T)[...,:3].reshape(-1,3)
    allp.append(P[np.isfinite(P).all(1)])
P=np.vstack(allp)
L=(P-hand)@R
m=(L[:,2]>-0.05)&(L[:,2]<0.01)&(abs(L[:,1])<0.12)&(L[:,0]>-0.25)&(L[:,0]<0.1)
S=L[m]; W=P[m]
for xlo in np.arange(-0.2,0.05,0.01):
    s=S[(S[:,0]>=xlo)&(S[:,0]<xlo+0.01)]; w=W[(S[:,0]>=xlo)&(S[:,0]<xlo+0.01)]
    if len(s): print(f'hx {xlo:+.2f}: n={len(s):4d} hy[{s[:,1].min():+.3f},{s[:,1].max():+.3f}] hz[{s[:,2].min():+.3f},{s[:,2].max():+.3f}] world mean {np.round(w.mean(0),3)}')
"


# openrua op 58
python3 -c "
import numpy as np
allp=[]
for cam in ['agentview','frontview','birdview']:
    d=np.load(cam+'_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']
    H,W=depth.shape; u,v=np.meshgrid(np.arange(W),np.arange(H)); z=depth
    X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
    P=(np.stack([X,Y,z,np.ones_like(z)],-1)@T.T)[...,:3].reshape(-1,3)
    allp.append(P[np.isfinite(P).all(1)])
P=np.vstack(allp)
m=(P[:,2]>0.905)&(P[:,2]<1.02)&(P[:,0]>-0.15)&(P[:,0]<0.1)&(P[:,1]>-0.30)&(P[:,1]<0.0)
pts=P[m]; print('n',len(pts),'zmax',pts[:,2].max())
xs=np.arange(-0.12,0.08,0.01); ys=np.arange(-0.28,0.0,0.01)
print('      '+''.join(f'{abs(y)*100:3.0f}' for y in ys))
for x in xs:
    row=''
    for y in ys:
        s=pts[(abs(pts[:,0]-x)<0.005)&(abs(pts[:,1]-y)<0.005)]
        row+= '  .' if len(s)==0 else f'{int((s[:,2].max()-0.9)*100):3d}'
    print(f'{x:+.2f} '+row)
"


# openrua op 59
python3 -c "
import numpy as np
allp=[]
for cam in ['agentview','frontview','birdview']:
    d=np.load(cam+'_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']
    H,W=depth.shape; u,v=np.meshgrid(np.arange(W),np.arange(H)); z=depth
    X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
    P=(np.stack([X,Y,z,np.ones_like(z)],-1)@T.T)[...,:3].reshape(-1,3)
    allp.append(P[np.isfinite(P).all(1)])
P=np.vstack(allp)
m=(P[:,2]>0.905)&(P[:,2]<1.03)&(P[:,0]>-0.15)&(P[:,0]<0.12)&(P[:,1]>-0.12)&(P[:,1]<0.2)
pts=P[m]; print('n',len(pts),'zmax',pts[:,2].max())
xs=np.arange(-0.10,0.10,0.01); ys=np.arange(-0.10,0.18,0.01)
print('      '+''.join(f'{y*100:3.0f}' for y in ys))
for x in xs:
    row=''
    for y in ys:
        s=pts[(abs(pts[:,0]-x)<0.005)&(abs(pts[:,1]-y)<0.005)]
        row+= '  .' if len(s)==0 else f'{int((s[:,2].max()-0.9)*100):3d}'
    print(f'{x:+.2f} '+row)
# check whether the mug interior is visible: points inside x[-0.0,0.05], and y-lengthwise
" 

# openrua op 60
timeout 300 python3 -c "
from rob import *
from pathcheck import check
r=Rob('park')
R_V = hand_R([0,0,-1],[0,1,0])
q0,_=r.joints()
q,d=r.ik_near(np.array([-0.35,-0.05,1.30]), R_V, seed=q0)
print(check(r,q0,q))
r.movej([q],[8.0])
print(np.round(r.fk()[0],3))
rclpy.shutdown()
" 2>&1 | grep -v WARN && timeout 120 python3 cam_cache.py agentview birdview frontview sideview 2>&1 | grep -v WARN

# openrua op 61
python3 -c "
import numpy as np
allp=[]
for cam in ['agentview','frontview','birdview','sideview']:
    d=np.load(cam+'_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']
    H,W=depth.shape; u,v=np.meshgrid(np.arange(W),np.arange(H)); z=depth
    X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
    P=(np.stack([X,Y,z,np.ones_like(z)],-1)@T.T)[...,:3].reshape(-1,3)
    P=P[np.isfinite(P).all(1)]
    m=(P[:,2]>0.903)&(P[:,2]<1.05)&(P[:,0]>-0.15)&(P[:,0]<0.15)&(P[:,1]>-0.15)&(P[:,1]<0.15)
    print(cam,m.sum()); allp.append(P[m])
P=np.vstack(allp)
print('bbox',P.min(0),P.max(0))
c=P.mean(0); U,S,Vt=np.linalg.svd(P-c,full_matrices=False)
print('centroid',np.round(c,4)); print('sv',np.round(S/np.sqrt(len(P)),4)); print('axes',np.round(Vt,3))
ax=Vt[0]
t=(P-c)@ax
print('extent along axis',t.min(),t.max())
# radial distance from axis
rad=np.linalg.norm((P-c)-np.outer(t,ax),axis=1)
for tlo in np.arange(-0.08,0.08,0.01):
    s=rad[(t>=tlo)&(t<tlo+0.01)]
    if len(s): print(f't {tlo:+.2f}: n={len(s):4d} rad med={np.median(s):.3f} max={s.max():.3f} min={s.min():.3f}')
np.save('mugpts.npy',P)
"


# openrua op 62
python3 -c "
import numpy as np
from scipy.optimize import least_squares
P=np.load('mugpts.npy')
R0=0.0452; L=0.112
def axis(a,b): return np.array([np.cos(b)*np.cos(a),np.cos(b)*np.sin(a),np.sin(b)])
def resid(p):
    c=p[:3]; ax=axis(p[3],p[4])
    d=P-c; t=d@ax; rad=np.linalg.norm(d-np.outer(t,ax),axis=1)
    r1=rad-R0
    # end caps: points beyond the length are penalised
    r2=np.maximum(np.abs(t)-L/2,0)
    return np.concatenate([r1,r2])
best=None
for a0 in np.radians([60,90,120]):
  for b0 in np.radians([0,20]):
    for c0 in [np.array([0.01,-0.02,0.945]),np.array([0.02,-0.03,0.95])]:
        p0=np.r_[c0,a0,b0]
        s=least_squares(resid,p0,loss='soft_l1',f_scale=0.005)
        if best is None or s.cost<best.cost: best=s
p=best.x; c=p[:3]; ax=axis(p[3],p[4]); print('cost',best.cost)
print('center',np.round(c,4),'axis',np.round(ax,3),'tilt deg',np.degrees(np.arcsin(ax[2])),'yaw deg',np.degrees(np.arctan2(ax[1],ax[0])))
d=P-c; t=d@ax; rad=np.linalg.norm(d-np.outer(t,ax),axis=1)
inl=np.abs(rad-R0)<0.006
print('inliers',inl.sum(),'/',len(P),' t range of inliers',t[inl].min(),t[inl].max())
# interior points (rad<0.035) indicate opening direction
inter=rad<0.035
print('interior pts',inter.sum(),' t of interior: ',np.round(np.percentile(t[inter],[5,50,95]),3) if inter.sum() else None)
ends=[c-ax*L/2,c+ax*L/2]; print('end -:',np.round(ends[0],3),' end +:',np.round(ends[1],3))
# outliers (handle?)
out=(rad-R0>0.01)&(np.abs(t)<L/2+0.01)
print('outer outliers',out.sum(),'mean',np.round(P[out].mean(0),3) if out.sum() else None, 'bbox',np.round(P[out].min(0),3),np.round(P[out].max(0),3))
np.save('mugfit.npy',np.r_[c,ax])
"


# openrua op 63
python3 -c "
import numpy as np
P=np.load('mugpts.npy'); f=np.load('mugfit.npy'); c=f[:3]; ax=f[3:]
d=P-c; t=d@ax; rad=np.linalg.norm(d-np.outer(t,ax),axis=1)
m=(t>0.04)&(rad>0.035)&(rad<0.055)
Q=P[m]; print('rim candidates',len(Q))
cq=Q.mean(0); U,S,Vt=np.linalg.svd(Q-cq,full_matrices=False); n=Vt[2]
if n[1]<0: n=-n
print('rim plane normal',np.round(n,3),'sv',np.round(S/np.sqrt(len(Q)),4))
# fit circle in plane: project
e1=np.cross(n,[0,0,1]); e1/=np.linalg.norm(e1); e2=np.cross(n,e1)
uv=np.c_[(Q-cq)@e1,(Q-cq)@e2]
A=np.c_[2*uv,np.ones(len(uv))]; b=(uv**2).sum(1)
sol=np.linalg.lstsq(A,b,rcond=None)[0]; u0,v0=sol[:2]; r=np.sqrt(sol[2]+u0**2+v0**2)
cen=cq+u0*e1+v0*e2
print('rim circle center',np.round(cen,4),'radius',round(r,4))
print('angular coverage: ',np.round(np.degrees(np.sort(np.arctan2(uv[:,1]-v0,uv[:,0]-u0)))[::max(1,len(uv)//12)],0))
print('e1',np.round(e1,3),'e2',np.round(e2,3))
np.save('rimfit.npy',np.r_[cen,n,r])
"


# openrua op 64
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def gripper(self, width):", "new_string": "    def move_tcp_cl(self, tcp_world, R, secs=3.0, seed=None, iters=3, tol=0.004, check=None):\\n        \\"\\"\\"Closed-loop move: the controller sags under load in some\\n        configurations, so after each move we measure the TCP via FK and\\n        re-command with the residual added. `check(q0, q)` may veto a path.\\"\\"\\"\\n        tcp_world = np.asarray(tcp_world, float)\\n        p_cmd, R_cmd = tcp_world.copy(), R.copy()\\n        q = None\\n        for i in range(iters + 1):\\n            q0, _ = self.joints()\\n            q, d = self.ik_near(p_cmd, R_cmd, seed=seed if (seed is not None and q is None) else (q if q is not None else q0))\\n            if q is None:\\n                print(f\\"[move_cl] IK FAILED for {np.round(p_cmd,3)}\\")\\n                return None\\n            if check is not None and not check(q0, q):\\n                print(\\"[move_cl] path check vetoed move\\")\\n                return None\\n            self.movej([q], [secs if i == 0 else 2.0])\\n            tcp, Rm = self.fk()\\n            dp = tcp_world - tcp\\n            ang = math.degrees(math.acos(np.clip((np.trace(R @ Rm.T) - 1) / 2, -1, 1)))\\n            print(f\\"[move_cl] iter {i}: tcp={np.round(tcp,4).tolist()} pos err={np.linalg.norm(dp):.4f} rot err={ang:.1f}deg\\")\\n            if np.linalg.norm(dp) <= tol and ang < 2.0:\\n                break\\n            p_cmd = p_cmd + dp\\n            R_cmd = (R @ Rm.T) @ R_cmd\\n        return q\\n\\n    def gripper(self, width):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 65
mkdir -p "$(dirname /workspace/step_white_c.py)"
cat > /workspace/step_white_c.py <<'OPENRUA_EOF'
"""White mug lying on the table, rim fit in rimfit.npy (centre, normal, r).
Pinch the rim wall at its +x side point, fingers along the horizontal
tangent, approach along the mug axis into the opening. Lift and stop."""
import sys
import numpy as np
from rob import *
from pathcheck import check, fk_links

f = np.load("rimfit.npy"); cen, n, rr = f[:3], f[3:6], f[6]
n /= np.linalg.norm(n)
e1 = np.cross(n, [0, 0, 1]); e1 /= np.linalg.norm(e1)
if e1[0] < 0: e1 = -e1
APP = -n
R_S = hand_R(APP, e1)
pinch = cen + (rr - 0.002) * e1
grasp = pinch + 0.02 * APP
pregrasp = grasp - 0.05 * APP
lift = grasp + np.array([0, 0, 0.20])
print("rim centre", np.round(cen, 4), "normal", np.round(n, 3), "e1", np.round(e1, 3))
print("pregrasp", np.round(pregrasp, 4), "grasp", np.round(grasp, 4))
hand = grasp - TCP_OFF * APP
print("hand origin at grasp", np.round(hand, 4), " palm low z ~", round(hand[2] - 0.058 * abs(APP[2]) - 0.03, 3))

r = Rob("white_c")
q0, _ = r.joints()
for name, p in [("pregrasp", pregrasp), ("grasp", grasp)]:
    q, d = r.ik_near(p, R_S, seed=q0)
    print(name, "IK", None if q is None else np.round(q, 3).tolist(), d)
    if q is not None:
        L = fk_links(r, q)
        print("   " + " ".join(f"{k[-5:]}={np.round(v,3).tolist()}" for k, v in L.items()))
if "--dry" in sys.argv:
    rclpy.shutdown(); sys.exit()

r.gripper(0.04)
chk = lambda a, b: check(r, a, b)
q = r.move_tcp_cl(pregrasp, R_S, secs=8.0, check=chk)
assert q is not None
q = r.move_tcp_cl(grasp, R_S, secs=3.0, check=chk)
assert q is not None
gap = r.gripper(0.0)
print("GAP after close:", gap)
q = r.move_tcp_cl(lift, R_S, secs=4.0, check=chk, iters=1)
print("fingers after lift:", r.finger_gap())
print("DONE_C")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 66
timeout 300 python3 step_white_c.py --dry 2>&1 | grep -v "^\[WARN\]"

# openrua op 67
python3 - <<'EOF'
import re
s=open('step_white_c.py').read()
s=s.replace('''APP = -n
R_S = hand_R(APP, e1)''','''e2 = np.cross(n, e1)                       # down-ish, in the rim plane
TH = np.radians(float(sys.argv[sys.argv.index("--th") + 1]) if "--th" in sys.argv else 30.0)
APP = -n * np.cos(TH) + e2 * np.sin(TH)     # approach tilted TH from mug axis, about e1
R_S = hand_R(APP, e1)''')
s=s.replace('pregrasp = grasp - 0.05 * APP','pregrasp = grasp - 0.06 * APP')
open('step_white_c.py','w').write(s)
EOF
timeout 300 python3 step_white_c.py --dry 2>&1 | grep -v "^\[WARN\]"

# openrua op 68
timeout 900 python3 step_white_c.py 2>&1 | grep -v "^\[WARN\]"

# openrua op 69
timeout 120 python3 cam_cache.py agentview frontview sideview birdview 2>&1 | grep -v WARN

# openrua op 70
timeout 120 python3 - <<'EOF' 2>&1 | grep -v WARN
import numpy as np
from scipy.optimize import least_squares
from rob import *
r=Rob('m'); tcp,R=r.fk(); rclpy.shutdown()
hand=tcp-TCP_OFF*R[:,2]
print('tcp',np.round(tcp,4),'hand',np.round(hand,4))
allp=[]
for cam in ['agentview','frontview','sideview','birdview']:
    d=np.load(cam+'_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']
    H,W=depth.shape; u,v=np.meshgrid(np.arange(W),np.arange(H)); z=depth
    X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
    P=(np.stack([X,Y,z,np.ones_like(z)],-1)@T.T)[...,:3].reshape(-1,3)
    P=P[np.isfinite(P).all(1)]
    L=(P-hand)@R
    inhand=(L[:,2]>-0.12)&(L[:,2]<0.115)&(abs(L[:,0])<0.05)&(abs(L[:,1])<0.115)
    near=(np.linalg.norm(P[:,:2]-tcp[:2],axis=1)<0.17)&(P[:,2]>0.92)&(P[:,2]<tcp[2]+0.06)&~inhand
    allp.append(P[near])
P=np.vstack(allp); print('pts',len(P))
R0=0.0452; Lm=0.112
def axis(a,b): return np.array([np.cos(b)*np.cos(a),np.cos(b)*np.sin(a),np.sin(b)])
def resid(p):
    c=p[:3]; ax=axis(p[3],p[4]); d=P-c; t=d@ax
    rad=np.linalg.norm(d-np.outer(t,ax),axis=1)
    return np.concatenate([rad-R0, np.maximum(np.abs(t)-Lm/2,0)])
best=None
for a0 in np.radians(np.arange(0,360,45)):
  for b0 in np.radians([-60,-30,0,30]):
    p0=np.r_[tcp+[-0.03,-0.02,-0.05],a0,b0]
    s=least_squares(resid,p0,loss='soft_l1',f_scale=0.005)
    if best is None or s.cost<best.cost: best=s
p=best.x; c=p[:3]; ax=axis(p[3],p[4])
d=P-c; t=d@ax; rad=np.linalg.norm(d-np.outer(t,ax),axis=1)
inl=np.abs(rad-R0)<0.006
print('cost',best.cost,'inliers',inl.sum())
print('center',np.round(c,4),'axis',np.round(ax,3),'elev deg',np.degrees(np.arcsin(ax[2])))
print('ends',np.round(c-ax*Lm/2,3),np.round(c+ax*Lm/2,3))
print('tcp rel center: axial',round(float((tcp-c)@ax),4),' radial',round(float(np.linalg.norm((tcp-c)-((tcp-c)@ax)*ax)),4))
inter=(rad<0.03)&(np.abs(t)<Lm/2+0.01); print('interior/cap pts t:',np.round(np.percentile(t[inter],[5,50,95]),3) if inter.sum() else None)
np.save('mugfit2.npy',np.r_[c,ax])
EOF

# openrua op 71
mkdir -p "$(dirname /workspace/step_white_d.py)"
cat > /workspace/step_white_d.py <<'OPENRUA_EOF'
"""Mug held lying (rim pinch). Re-orient so mug axis -> +y (bottom forward,
level), carry to the microwave opening, insert lying on its side, release."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import *
from pathcheck import check, fk_links

f = np.load("mugfit2.npy"); c_mug, a_mug = f[:3], f[3:6]      # axis rim->bottom
r = Rob("white_d")
tcp0, R0 = r.fk()
e1 = R0[:, 1]                                             # finger axis (world)
print("tcp0", np.round(tcp0, 4), "mug axis", np.round(a_mug, 3), "e1", np.round(e1, 3))

# rotation Q: mug axis -> +y, finger axis -> -x (approximately)
e1p = e1 - (e1 @ a_mug) * a_mug; e1p /= np.linalg.norm(e1p)
Rs = np.column_stack([a_mug, e1p, np.cross(a_mug, e1p)])
Rt = np.column_stack([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
Q = Rt @ Rs.T
R_F = Q @ R0
print("final approach", np.round(R_F[:, 2], 3), "final finger axis", np.round(R_F[:, 1], 3))
# mug centre offset from TCP after rotation
off = Q @ (c_mug - tcp0)
print("mug centre rel tcp after rot", np.round(off, 4))

chk = lambda a, b: check(r, a, b)
X_MUG = -0.04
xt = X_MUG - off[0]
Y_IN, Z_IN = 0.28, 1.01
print("insert TCP x", round(xt, 4))
if "--dry" in sys.argv:
    rclpy.shutdown(); sys.exit()

def go(p, R, secs=4.0, iters=2):
    q = r.move_tcp_cl(np.asarray(p), R, secs=secs, check=chk, iters=iters)
    assert q is not None, f"move failed {p}"
    return q

# 1. up, 2. over to a free spot
go(tcp0 + [0, 0, 0.10], R0)
p_rot = np.array([0.0, -0.10, 1.25])
go(p_rot, R0, secs=5.0)
# 3. re-orient in steps (slerp)
rot0, rot1 = Rot.from_matrix(R0), Rot.from_matrix(R_F)
key = Rot.concatenate([rot0, rot1])
for s in [0.33, 0.66, 1.0]:
    Rk = (rot0 * (rot0.inv() * rot1) ** s).as_matrix()
    go(p_rot, Rk, secs=5.0)
print("fingers:", r.finger_gap())
# 4. pre-insert, 5. insert in steps
go([xt, 0.10, Z_IN], R_F, secs=6.0)
for y in [0.16, 0.22, Y_IN]:
    go([xt, y, Z_IN], R_F, secs=3.0)
go([xt, Y_IN, 1.0], R_F, secs=2.0)
r.gripper(0.04)
go([xt, 0.10, Z_IN], R_F, secs=4.0, iters=0)
go([xt, 0.10, 1.15], R_F, secs=3.0, iters=0)
print("DONE_D")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 72
timeout 300 python3 step_white_d.py --dry 2>&1 | grep -v "^\[WARN\]"

# openrua op 73
sed -i 's/Y_IN, Z_IN = 0.28, 1.01/Y_IN, Z_IN = 0.30, 1.0/; s/go(\[xt, Y_IN, 1.0\], R_F, secs=2.0)/go([xt, Y_IN, 0.985], R_F, secs=2.0)/' step_white_d.py && grep -n "Y_IN\|0.985" step_white_d.py && timeout 1500 python3 step_white_d.py 2>&1 | grep -v "^\[WARN\]"

# openrua op 74
python3 - <<'EOF'
s=open('step_white_d.py').read()
s=s.replace('off = Q @ (c_mug - tcp0)','TCP_FIT = np.array([0.0394, 0.0223, 1.1496])   # TCP when mugfit2 was measured\noff = Q @ (c_mug - TCP_FIT)')
s=s.replace('''key = Rot.concatenate([rot0, rot1])
for s in [0.33, 0.66, 1.0]:
    Rk = (rot0 * (rot0.inv() * rot1) ** s).as_matrix()''','''dvec = (rot0.inv() * rot1).as_rotvec()
print("total rotation deg", np.degrees(np.linalg.norm(dvec)))
for s in [0.33, 0.66, 1.0]:
    Rk = (rot0 * Rot.from_rotvec(s * dvec)).as_matrix()''')
s=s.replace('''go(tcp0 + [0, 0, 0.10], R0)
p_rot''','''if tcp0[2] < 1.2:
    go(tcp0 + [0, 0, 0.10], R0)
p_rot''')
open('step_white_d.py','w').write(s)
EOF
timeout 1500 python3 step_white_d.py 2>&1 | grep -v "^\[WARN\]"

# openrua op 75
timeout 120 python3 -c "
from rob import *
r=Rob('c'); q,_=r.joints(); print('q',np.round(q,3).tolist()); tcp,R=r.fk(); print('tcp',np.round(tcp,4).tolist()); print('R',np.round(R,3).tolist()); print('fingers',r.finger_gap())
rclpy.shutdown()" 2>&1 | grep -v WARN; timeout 120 python3 cam_cache.py agentview birdview 2>&1 | grep -v WARN

# openrua op 76
timeout 120 python3 - <<'EOF' 2>&1 | grep -v WARN
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import *
r=Rob('c'); q,_=r.joints(); tcp,R=r.fk()
R_F=np.array(hand_R([-0.163,0.898,-0.409],[-0.984,-0.179,0]))
Rrel=R.T@R_F
rv=Rot.from_matrix(Rrel).as_rotvec(); print('rel rotvec (hand frame)',np.round(rv,3),'deg',np.degrees(np.linalg.norm(rv)))
for sgn in [1,-1]:
    q2=q.copy(); q2[6]=q[6]+sgn*rv[2]
    print('candidate j7',round(q2[6],3), 'in limits', -2.9<q2[6]<2.9)
    if -2.9<q2[6]<2.9:
        t2,R2=r.fk(q2); ang=np.degrees(np.arccos(np.clip((np.trace(R_F@R2.T)-1)/2,-1,1)))
        print('   fk tcp',np.round(t2,4),'rot err to R_F',round(ang,1))
rclpy.shutdown()
EOF

# openrua op 77
timeout 600 python3 - <<'EOF' 2>&1 | grep -v WARN
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import *
r=Rob('c'); q,_=r.joints(); tcp,R=r.fk()
R_F=np.array(hand_R([-0.163,0.898,-0.409],[-0.984,-0.179,0]))
off=np.array([0.0323,0.0441,0.0167])
for rho in range(0,360,20):
    Ry=Rot.from_euler('y',rho,degrees=True).as_matrix()
    Rr=Ry@R_F
    qs,d=r.ik_near(tcp,Rr,seed=q,tries=4)
    o=Ry@off
    print(f"rho {rho:3d}: approach {np.round(Rr[:,2],2).tolist()} fingers {np.round(Rr[:,1],2).tolist()} mug off {np.round(o,3).tolist()}  ik dist {None if d is None else round(d,2)}  j7 {None if qs is None else round(qs[6],2)}")
rclpy.shutdown()
EOF

# openrua op 78
timeout 600 python3 - <<'EOF' 2>&1 | grep -v WARN
import numpy as np
from rob import *
from pathcheck import fk_links
r=Rob('c'); q,_=r.joints(); tcp,R=r.fk()
R_F=np.array(hand_R([-0.163,0.898,-0.409],[-0.984,-0.179,0]))
q2=q.copy(); q2[6]=q[6]+(-2.06+2*np.pi)
t2,R2=r.fk(q2); ang=np.degrees(np.arccos(np.clip((np.trace(R_F@R2.T)-1)/2,-1,1)))
print('j7',round(q2[6],3),'fk tcp',np.round(t2,4),'rot err',round(ang,2))
# insertion IK feasibility from q2
xt=-0.0723
for p in [[xt,0.10,1.01],[xt,0.16,1.0],[xt,0.22,1.0],[xt,0.30,1.0],[xt,0.30,0.985]]:
    qs,d=r.ik_near(np.array(p),R_F,seed=q2,tries=6)
    L=fk_links(r,qs) if qs is not None else {}
    print(p,'dist',None if d is None else round(d,2), 'q',None if qs is None else np.round(qs,2).tolist())
    if qs is not None: print('    '+' '.join(f'{k[-5:]}={np.round(v,2).tolist()}' for k,v in L.items() if k in ('panda_link5','panda_link7','panda_hand')))
    if qs is not None: q2=qs
rclpy.shutdown()
EOF

# openrua op 79
mkdir -p "$(dirname /workspace/step_white_e.py)"
cat > /workspace/step_white_e.py <<'OPENRUA_EOF'
"""Mug held; hand needs a -118 deg roll about its z which j7 cannot do
directly -> wrap j7 by +242 deg (slow). Then insert lying into the cavity."""
import sys
import numpy as np
from rob import *
from pathcheck import check

R_F = np.array(hand_R([-0.163, 0.898, -0.409], [-0.984, -0.179, 0]))
xt, Y_IN, Z_IN = -0.0723, 0.30, 1.0
r = Rob("white_e")
chk = lambda a, b: check(r, a, b)

def go(p, R, secs=4.0, iters=2):
    q = r.move_tcp_cl(np.asarray(p), R, secs=secs, check=chk, iters=iters)
    assert q is not None, f"move failed {p}"
    return q

q, _ = r.joints()
tcp, R = r.fk()
ang = np.degrees(np.arccos(np.clip((np.trace(R_F @ R.T) - 1) / 2, -1, 1)))
print("start tcp", np.round(tcp, 4), "rot err to R_F", round(ang, 1), "fingers", r.finger_gap())
if ang > 20:
    q2 = q.copy(); q2[6] = q[6] + (-2.06 + 2 * np.pi)
    assert -2.9 < q2[6] < 2.9
    print("wrapping j7 to", round(q2[6], 3))
    r.movej([q2], [20.0])
    tcp, R = r.fk()
    ang = np.degrees(np.arccos(np.clip((np.trace(R_F @ R.T) - 1) / 2, -1, 1)))
    print("after wrap tcp", np.round(tcp, 4), "rot err", round(ang, 1), "fingers", r.finger_gap())
if "--wrap-only" in sys.argv:
    rclpy.shutdown(); sys.exit()

go([xt, 0.10, Z_IN], R_F, secs=6.0)
print("fingers:", r.finger_gap())
if "--pre-only" in sys.argv:
    rclpy.shutdown(); sys.exit()
for y in [0.16, 0.22, Y_IN]:
    go([xt, y, Z_IN], R_F, secs=3.0)
go([xt, Y_IN, 0.985], R_F, secs=2.0)
r.gripper(0.04)
go([xt, 0.10, Z_IN], R_F, secs=4.0, iters=0)
go([xt, 0.10, 1.15], R_F, secs=3.0, iters=0)
print("DONE_E")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 80
timeout 900 python3 step_white_e.py --wrap-only 2>&1 | grep -v "^\[WARN\]" && timeout 120 python3 cam_cache.py agentview frontview sideview birdview 2>&1 | grep -v WARN

# openrua op 81
timeout 120 python3 - <<'EOF' 2>&1 | grep -v WARN
import numpy as np
from scipy.optimize import least_squares
from rob import *
r=Rob('m'); tcp,R=r.fk(); rclpy.shutdown()
hand=tcp-TCP_OFF*R[:,2]
print('tcp',np.round(tcp,4),'approach',np.round(R[:,2],3),'fingers',np.round(R[:,1],3))
allp=[]
for cam in ['agentview','frontview','sideview','birdview']:
    d=np.load(cam+'_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']
    H,W=depth.shape; u,v=np.meshgrid(np.arange(W),np.arange(H)); z=depth
    X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
    P=(np.stack([X,Y,z,np.ones_like(z)],-1)@T.T)[...,:3].reshape(-1,3)
    P=P[np.isfinite(P).all(1)]
    L=(P-hand)@R
    inhand=(L[:,2]>-0.25)&(L[:,2]<0.115)&(abs(L[:,0])<0.06)&(abs(L[:,1])<0.115)
    near=(np.linalg.norm(P[:,:2]-tcp[:2],axis=1)<0.17)&(P[:,2]>0.95)&(P[:,2]<tcp[2]+0.08)&~inhand&(P[:,1]>tcp[1]-0.03)
    allp.append(P[near])
P=np.vstack(allp); print('pts',len(P))
R0=0.0452; Lm=0.112
def axis(a,b): return np.array([np.cos(b)*np.cos(a),np.cos(b)*np.sin(a),np.sin(b)])
def resid(p):
    c=p[:3]; ax=axis(p[3],p[4]); d=P-c; t=d@ax
    rad=np.linalg.norm(d-np.outer(t,ax),axis=1)
    return np.concatenate([rad-R0, np.maximum(np.abs(t)-Lm/2,0)])
best=None
for a0 in np.radians(np.arange(0,360,45)):
  for b0 in np.radians([-30,0,30]):
    p0=np.r_[tcp+[0.03,0.045,0.0],a0,b0]
    s=least_squares(resid,p0,loss='soft_l1',f_scale=0.005)
    if best is None or s.cost<best.cost: best=s
p=best.x; c=p[:3]; ax=axis(p[3],p[4])
d=P-c; t=d@ax; rad=np.linalg.norm(d-np.outer(t,ax),axis=1)
inl=np.abs(rad-R0)<0.006
print('cost',best.cost,'inliers',inl.sum())
print('center',np.round(c,4),'axis',np.round(ax,3),'elev deg',round(np.degrees(np.arcsin(ax[2])),1),'azim deg',round(np.degrees(np.arctan2(ax[1],ax[0])),1))
print('center rel tcp',np.round(c-tcp,4))
inter=(rad<0.03)&(np.abs(t)<Lm/2+0.01); print('cap pts t:',np.round(np.percentile(t[inter],[5,50,95]),3) if inter.sum() else None)
out=(rad-R0>0.012)&(np.abs(t)<Lm/2); print('handle-ish pts',out.sum(),'mean rel center',np.round(P[out].mean(0)-c,3) if out.sum() else None)
np.save('mugfit3.npy',np.r_[c,ax,tcp])
EOF

# openrua op 82
timeout 120 python3 - <<'EOF' 2>&1 | grep -v WARN
import numpy as np
c=np.array([0.029,-0.039,1.171]); ax=np.array([0,1.0,0])
allp=[]
for cam in ['agentview','frontview','birdview','sideview']:
    d=np.load(cam+'_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']
    H,W=depth.shape; u,v=np.meshgrid(np.arange(W),np.arange(H)); z=depth
    X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
    P=(np.stack([X,Y,z,np.ones_like(z)],-1)@T.T)[...,:3].reshape(-1,3)
    P=P[np.isfinite(P).all(1)]
    d=P-c; t=d@ax; rad=np.linalg.norm(d-np.outer(t,ax),axis=1)
    m=(np.abs(t)<0.07)&(rad<0.10)&(P[:,2]>1.0)
    allp.append(P[m])
P=np.vstack(allp); d=P-c; t=d@ax; radv=d-np.outer(t,ax); rad=np.linalg.norm(radv,axis=1)
print('pts',len(P))
# angle around axis: 0 = +x, 90 = +z
ang=np.degrees(np.arctan2(radv[:,2],radv[:,0]))
for rlo in [0.03,0.04,0.045,0.05,0.055,0.06,0.07,0.08,0.09]:
    s=(rad>=rlo)&(rad<rlo+0.01)
    if s.sum(): 
        h=np.histogram(ang[s],bins=12,range=(-180,180))[0]
        print(f'rad {rlo:.3f}: n={s.sum():4d} t[{t[s].min():+.3f},{t[s].max():+.3f}] ang hist(-180..180 step30): {h.tolist()}')
# body extent along axis
body=(rad>0.035)&(rad<0.055)
print('body t range',np.percentile(t[body],[2,50,98]))
EOF

# openrua op 83
timeout 900 python3 step_white_e.py --pre-only 2>&1 | grep -v "^\[WARN\]" && timeout 120 python3 cam_cache.py agentview frontview 2>&1 | grep -v WARN

# openrua op 84
timeout 900 python3 step_white_e.py 2>&1 | grep -v "^\[WARN\]" && timeout 120 python3 cam_cache.py agentview frontview sideview birdview 2>&1 | grep -v WARN

# openrua op 85
python3 - <<'EOF'
import numpy as np
for cam in ['agentview','sideview','birdview']:
    d=np.load(cam+'_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']; col=d['color']
    H,W=depth.shape; u,v=np.meshgrid(np.arange(W),np.arange(H)); z=depth
    X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
    P=(np.stack([X,Y,z,np.ones_like(z)],-1)@T.T)[...,:3]
    m=(P[...,0]>-0.17)&(P[...,0]<0.08)&(P[...,1]>0.25)&(P[...,1]<0.47)&(P[...,2]>0.93)&(P[...,2]<1.1)&np.isfinite(z)
    # white pixels
    c=col.astype(int); white=(c.min(-1)>150)&(np.ptp(c,axis=-1)<40)
    mw=m&white
    print(cam,'cavity pts',m.sum(),'white pts',mw.sum())
    if mw.sum()>20:
        q=P[mw]; print('  white bbox min',np.round(q.min(0),3),'max',np.round(q.max(0),3),'mean',np.round(q.mean(0),3))
EOF

# openrua op 86
python3 - <<'EOF'
import numpy as np
d=np.load('birdview_cache.npz'); depth=d['depth'];K=d['K'];T=d['T']
H,W=depth.shape; u,v=np.meshgrid(np.arange(W),np.arange(H)); z=depth
X=(u-K[0,2])*z/K[0,0];Y=(v-K[1,2])*z/K[1,1]
P=(np.stack([X,Y,z,np.ones_like(z)],-1)@T.T)[...,:3].reshape(-1,3)
P=P[np.isfinite(P).all(1)]
m=(P[:,2]>0.95)&(P[:,2]<1.12)&(P[:,0]<-0.17)&(P[:,0]>-0.4)&(P[:,1]>-0.15)&(P[:,1]<0.3)
Q=P[m]; print('door pts',len(Q),'z range',Q[:,2].min(),Q[:,2].max())
# fit line in xy
c=Q[:,:2].mean(0); U,S,Vt=np.linalg.svd(Q[:,:2]-c,full_matrices=False); dirv=Vt[0]
t=(Q[:,:2]-c)@dirv; perp=(Q[:,:2]-c)@Vt[1]
print('line dir',np.round(dirv,3),'center',np.round(c,3),'t range',round(t.min(),3),round(t.max(),3),'perp spread',round(perp.std(),4),'perp range',round(perp.min(),3),round(perp.max(),3))
e0=c+t.min()*dirv; e1=c+t.max()*dirv; print('ends',np.round(e0,3),np.round(e1,3))
ang=np.degrees(np.arctan2(dirv[1],dirv[0])); print('angle deg',ang)
# top surface z hist
print('z hist',np.histogram(Q[:,2],bins=8)[0], np.round(np.histogram(Q[:,2],bins=8)[1],3))
# door thickness: perp extent at mid
mid=np.abs(t)<0.05; print('perp extent mid',round(perp[mid].min(),3),round(perp[mid].max(),3))
EOF

# openrua op 87
mkdir -p "$(dirname /workspace/step_door.py)"
cat > /workspace/step_door.py <<'OPENRUA_EOF'
"""Close the microwave door by pushing its outer face with closed fingers
along an arc about the hinge."""
import sys
import numpy as np
from rob import *
import pathcheck
from pathcheck import fk_links, RAD

HINGE = np.array([-0.19, 0.265])
TH0 = np.radians(-122.5)          # current door angle (tip at (-0.328,0.048))
RHO = 0.18                        # contact radius along the door
OFF = 0.0225                      # half door thickness + finger half width
Z = 1.0
LEAN = np.radians(30)

def door_pose(th):
    d = np.array([np.cos(th), np.sin(th), 0.0])
    n_out = np.array([np.sin(th), -np.cos(th), 0.0])
    tcp = np.array([HINGE[0], HINGE[1], Z]) + RHO * d + OFF * n_out
    app = -np.array([0, 0, 1.0]) * np.cos(LEAN) - n_out * np.sin(LEAN)
    return tcp, hand_R(app, d)

def check_no_door(r, q0, q1, n=6):
    bad = False
    for i in range(n + 1):
        q = q0 + (q1 - q0) * i / n
        L = fk_links(r, q)
        for k, p in L.items():
            h = [x for x in pathcheck.hazards(p, RAD[k]) if x != "DOOR"]
            if h:
                print(f"  hazard {k} {h} at {np.round(p,3).tolist()}"); bad = True
    return not bad

r = Rob("door")
chk = lambda a, b: check_no_door(r, a, b)
if "--dry" in sys.argv:
    q, _ = r.joints()
    for th in np.radians(np.arange(-122.5, 1, 15)):
        tcp, R = door_pose(th)
        qs, d = r.ik_near(tcp, R, seed=q, tries=6)
        print(f"th {np.degrees(th):6.1f} tcp {np.round(tcp,3).tolist()} ik {'ok' if qs is not None else 'FAIL'} dist {None if d is None else round(d,2)} j7 {None if qs is None else round(qs[6],2)}")
        if qs is not None: q = qs
    rclpy.shutdown(); sys.exit()

r.gripper(0.0)
tcp0, R0 = door_pose(TH0)
hi = tcp0 + [0, 0, 0.25]
assert r.move_tcp_cl(hi, R0, secs=8.0, check=chk, iters=1) is not None
assert r.move_tcp_cl(tcp0, R0, secs=4.0, check=chk, iters=1) is not None
angles = list(np.arange(np.degrees(TH0) + 10, 0, 10)) + [0.0, 2.0]
for a in angles:
    tcp, R = door_pose(np.radians(a))
    q = r.move_tcp_cl(tcp, R, secs=3.0, check=chk, iters=1)
    assert q is not None, f"failed at door angle {a}"
    print(f"--- door angle {a:.1f} done")
# retreat: back off -y a bit then up
tcp, R = door_pose(0.0)
r.move_tcp_cl(tcp + [0, -0.05, 0], R, secs=2.0, check=chk, iters=0)
r.move_tcp_cl(tcp + [0, -0.05, 0.25], R, secs=4.0, check=chk, iters=0)
print("DONE_DOOR")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 88
timeout 300 python3 step_door.py --dry 2>&1 | tail -20

# openrua op 89
sed -i 's/def door_pose(th):/def door_pose(th, sgn=1.0):/; s/    return tcp, hand_R(app, d)/    return tcp, hand_R(app, sgn * d)/' step_door.py && sed -i 's/        tcp, R = door_pose(th)\n/XX/' step_door.py && python3 - <<'EOF'
import re
s=open('step_door.py').read()
s=s.replace("""    for th in np.radians(np.arange(-122.5, 1, 15)):
        tcp, R = door_pose(th)""","""    SG = float(sys.argv[sys.argv.index("--sgn")+1]) if "--sgn" in sys.argv else 1.0
    for th in np.radians(np.arange(-122.5, 1, 15)):
        tcp, R = door_pose(th, SG)""")
open('step_door.py','w').write(s)
EOF
timeout 300 python3 step_door.py --dry --sgn -1 2>&1 | tail -12

# openrua op 90
python3 - <<'EOF'
s=open('step_door.py').read()
s=s.replace("""Z = 1.0
LEAN = np.radians(30)
""","""Z = 1.085                         # fingertips on the top 2 cm of the door; palm clears door top
LEAN = np.radians(30)
U = np.array([-0.875, -0.485, 0.0])   # fixed lean direction (palm displaced this way)
FING = np.array([0.485, -0.875, 0.0]) # fixed finger axis, perpendicular to U
R_FIX = np.array(hand_R(-np.array([0, 0, 1.0]) * np.cos(LEAN) - U * np.sin(LEAN), FING))
""")
s=s.replace("""    app = -np.array([0, 0, 1.0]) * np.cos(LEAN) - n_out * np.sin(LEAN)
    return tcp, hand_R(app, sgn * d)""","""    return tcp, R_FIX""")
s=s.replace("OFF = 0.0225 ","OFF = 0.0245 ")
open('step_door.py','w').write(s)
EOF
timeout 300 python3 step_door.py --dry 2>&1 | tail -12

# openrua op 91
timeout 120 python3 cam_cache.py birdview 2>&1 | tail -2; timeout 900 python3 step_door.py 2>&1 | grep -v "^\[ik_near\] try" | tail -60

# openrua op 92
timeout 120 python3 cam_cache.py birdview 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
d=np.load('birdview_cache.npz'); print(d.files)
P=d['pts'] if 'pts' in d.files else None
if P is None:
    for k in d.files: print(k, d[k].shape)
else:
    # door search: points at z in [1.0,1.12] outside microwave body footprint, y<0.27, x<0.1
    m=(P[:,2]>1.0)&(P[:,2]<1.12)&(P[:,1]<0.268)&(P[:,1]>0.0)&(P[:,0]>-0.4)&(P[:,0]<0.2)
    Q=P[m]; print("pts near front, high:",len(Q))
    if len(Q):
        print("x range",Q[:,0].min().round(3),Q[:,0].max().round(3),"y range",Q[:,1].min().round(3),Q[:,1].max().round(3))
        h,e=np.histogram(Q[:,1],bins=np.arange(0.0,0.28,0.01)); print(list(zip(e[:-1].round(2),h)))
    # anything left in the old open-door region
    m2=(P[:,2]>0.95)&(P[:,2]<1.12)&(P[:,0]<-0.2)&(P[:,1]<0.25)&(P[:,1]>-0.05)
    print("pts in old open-door region:",m2.sum())
EOF

# openrua op 93
grep -n "def \|import" px.py | head -20; grep -n "def " cam_cache.py measure.py | head

# openrua op 94
python3 - <<'EOF'
import numpy as np
from measure import cloud
P=cloud('birdview')
if isinstance(P,tuple): P=P[0]
print(P.shape)
m=(P[:,2]>1.0)&(P[:,2]<1.12)&(P[:,1]<0.268)&(P[:,1]>0.0)&(P[:,0]>-0.4)&(P[:,0]<0.2)
Q=P[m]; print("pts near front, high:",len(Q))
if len(Q):
    print("x range",Q[:,0].min().round(3),Q[:,0].max().round(3),"y range",Q[:,1].min().round(3),Q[:,1].max().round(3))
    h,e=np.histogram(Q[:,1],bins=np.arange(0.0,0.28,0.01)); print([(round(a,2),int(b)) for a,b in zip(e[:-1],h) if b])
    hx,ex=np.histogram(Q[Q[:,1]>0.2][:,0],bins=np.arange(-0.4,0.2,0.02)); print("x hist (y>0.2):",[(round(a,2),int(b)) for a,b in zip(ex[:-1],hx) if b])
m2=(P[:,2]>0.95)&(P[:,2]<1.12)&(P[:,0]<-0.2)&(P[:,1]<0.25)&(P[:,1]>-0.05)
print("pts in old open-door region:",m2.sum())
EOF

# openrua op 95
python3 - <<'EOF' 2>/dev/null | grep -v "^white\|^yellow\|^door\|^   "
import numpy as np
from measure import cloud
P=cloud('birdview')
m2=(P[:,2]>0.95)&(P[:,2]<1.12)&(P[:,0]<-0.2)&(P[:,1]<0.25)&(P[:,1]>-0.05)
Q=P[m2]; print(np.round(Q[:,0].min(),3),np.round(Q[:,0].max(),3),np.round(Q[:,1].min(),3),np.round(Q[:,1].max(),3),np.round(Q[:,2].min(),3),np.round(Q[:,2].max(),3))
print(np.round(np.percentile(Q,[10,50,90],axis=0),3))
# front face profile: for the door slab, points with y in [0.23,0.27], z>1.0: per-x max y and min y
m=(P[:,2]>1.0)&(P[:,2]<1.12)&(P[:,1]<0.268)&(P[:,1]>0.225)&(P[:,0]>-0.2)&(P[:,0]<0.08)
Q=P[m]
for x0 in np.arange(-0.2,0.08,0.04):
    s=Q[(Q[:,0]>=x0)&(Q[:,0]<x0+0.04)]
    if len(s): print(f"x {x0:+.2f}: n={len(s)} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
EOF

# openrua op 96
timeout 120 python3 cam_cache.py frontview 2>&1 | tail -1; timeout 120 python3 cam_cache.py agentview 2>&1 | tail -1; ls -la *.png | head

# openrua op 97
timeout 20 ros2 topic list 2>/dev/null | grep -iv "camera\|image\|depth\|param" | head -30; timeout 10 ros2 topic echo /joint_states --once 2>/dev/null | grep -A12 "^name"

# openrua op 98
timeout 120 python3 cam_cache.py sideview 2>&1 | tail -1

# openrua op 99
timeout 200 python3 - <<'EOF' 2>&1 | grep -v "^\[ik_near\] try" | tail -5
import numpy as np
from rob import *
r=Rob("park")
tcp,R=r.fk()
r.move_tcp_cl(np.array([-0.20,0.0,1.35]),R,secs=6.0,iters=0)
print("parked at",np.round(r.fk()[0],3),"fingers",r.finger_gap())
rclpy.shutdown()
EOF
