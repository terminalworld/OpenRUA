#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | head -50; echo ---; ros2 node list

# openrua op 3
ros2 topic echo /joint_states --once 2>&1 | head -40; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 4
ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo ---; timeout 20 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | head

# openrua op 5
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump world-frame point clouds / lookups for a camera.

Usage:
  python3 scene.py <cam> tf                 # print world->camera TF & intrinsics
  python3 scene.py <cam> px u v [u v ...]   # world xyz of pixels
  python3 scene.py <cam> save               # save <cam>_xyz.npy (HxWx3 world coords)
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam, mode = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float64)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 15
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    if mode == "tf":
        print("T", T, "\nR", R, "\nK", fx, fy, cx, cy, "size", W, H)
        print("depth range", np.nanmin(depth), np.nanmax(depth))
    elif mode == "px":
        vals = list(map(int, sys.argv[3:]))
        for u, v in zip(vals[::2], vals[1::2]):
            z = depth[v, u]
            p = R @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z]) + T
            print(f"({u},{v}) depth={z:.4f} world= {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    elif mode == "save":
        us, vs = np.meshgrid(np.arange(W), np.arange(H))
        pc = np.stack([(us - cx) * depth / fx, (vs - cy) * depth / fy, depth], -1)
        world = pc @ R.T + T
        np.save(f"{cam}_xyz.npy", world)
        print("saved", f"{cam}_xyz.npy", world.shape)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 6
timeout 120 python3 scene.py agentview tf && timeout 120 python3 scene.py agentview save && timeout 120 python3 scene.py birdview save && timeout 60 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8

# openrua op 7
python3 -c "
import numpy as np, cv2
W=np.load('agentview_xyz.npy'); img=cv2.imread('agentview.png')
# table height: median z over the lower part of the image
print('table z (bottom rows):', np.median(W[400:470, 100:540, 2]))
# bowl region ~ (335,215) in agentview
for (u,v) in [(335,215),(310,215),(360,215),(335,200),(335,232),(335,240)]:
    print('bowl px',u,v,W[v,u])
# cabinet top (left block) 
for (u,v) in [(100,180),(150,200),(60,250),(120,300),(180,180),(190,240),(230,250),(270,270),(200,290),(230,300),(280,300),(250,240)]:
    print('cab px',u,v,W[v,u], img[v,u])
"


# openrua op 8
python3 -c "
import numpy as np, cv2
W=np.load('birdview_xyz.npy'); img=cv2.imread('birdview.png')
Z=W[:,:,2]
print('z range', np.nanmin(Z), np.nanmax(Z))
# cabinet body: z>1.10
m=(Z>1.10)&(Z<1.2)
ys,xs=np.where(m); print('cabinet top px bbox', xs.min(),xs.max(),ys.min(),ys.max())
pts=W[m]; print('cabinet top world x', pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max(),'z',np.median(pts[:,2]))
# heights histogram in the table region
tab=(W[:,:,0]>-0.4)&(W[:,:,0]<0.5)&(np.abs(W[:,:,1])<0.6)
h,edges=np.histogram(Z[tab],bins=np.arange(0.88,1.25,0.01))
for c,e in zip(h,edges): print(f'{e:.2f} {c}')
"


# openrua op 9
python3 -c "
import numpy as np, cv2
W=np.load('birdview_xyz.npy'); img=cv2.imread('birdview.png')
Z=W[:,:,2]
def bbox(m,name):
    ys,xs=np.where(m); pts=W[m]
    print(name,'px u',xs.min(),xs.max(),'v',ys.min(),ys.max(),'| world x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
# cabinet top (z~1.12-1.13) and y<0
bbox((Z>1.115)&(Z<1.135)&(W[:,:,1]<0),'cabinet top')
# wooden shelf side y>0
bbox((Z>1.10)&(W[:,:,1]>0),'shelf')
# open drawer bottom / items at z 0.92-0.93 y<0
bbox((Z>0.915)&(Z<0.935),'z0.92 layer')
bbox((Z>0.965)&(Z<0.995)&(W[:,:,1]<0),'z0.97-0.99 y<0')
bbox((Z>0.995)&(Z<1.04)&(W[:,:,1]<0),'z1.0-1.04 y<0')
bbox((Z>1.04)&(Z<1.10)&(W[:,:,1]<0),'z1.04-1.10 y<0 (bottle top?)')
"


# openrua op 10
python3 -c "
import numpy as np, cv2
# orthographic top-down render: x in [-0.45,0.35], y in [-0.6,0.5], 2mm/px
res=0.002; x0,x1,y0,y1=-0.45,0.35,-0.6,0.5
Hh=int((x1-x0)/res); Ww=int((y1-y0)/res)
for cam in ['agentview','birdview']:
    W=np.load(f'{cam}_xyz.npy'); img=cv2.imread(f'{cam}.png')
    P=W.reshape(-1,3); C=img.reshape(-1,3)
    m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.89)&(P[:,2]<1.3)
    P=P[m]; C=C[m]
    r=((P[:,0]-x0)/res).astype(int); c=((P[:,1]-y0)/res).astype(int)
    Zmap=np.full((Hh,Ww),np.nan); Cmap=np.zeros((Hh,Ww,3),np.uint8)
    order=np.argsort(P[:,2])  # highest last wins
    Zmap[r[order],c[order]]=P[order,2]; Cmap[r[order],c[order]]=C[order]
    hm=np.nan_to_num((Zmap-0.89)/0.35*255,nan=0).clip(0,255).astype(np.uint8)
    hm=cv2.applyColorMap(hm,cv2.COLORMAP_JET); hm[np.isnan(Zmap)]=0
    for im in (hm,Cmap):
        for xv in np.arange(-0.4,0.35,0.1):
            rr=int((xv-x0)/res); cv2.line(im,(0,rr),(Ww,rr),(255,255,255),1); cv2.putText(im,f'x={xv:.1f}',(2,rr-2),0,0.4,(255,255,255),1)
        for yv in np.arange(-0.6,0.5,0.1):
            cc=int((yv-y0)/res); cv2.line(im,(cc,0),(cc,Hh),(255,255,255),1); cv2.putText(im,f'y={yv:.1f}',(cc+2,12),0,0.4,(255,255,255),1)
    cv2.imwrite(f'{cam}_topdown.png',np.hstack([Cmap,hm]))
print('ok')
"


# openrua op 11
python3 -c "
import numpy as np, cv2
W=np.load('agentview_xyz.npy'); img=cv2.imread('agentview.png')
Z=W[:,:,2]
m=(W[:,:,0]>-0.3)&(W[:,:,0]<-0.02)&(W[:,:,1]>-0.05)&(W[:,:,1]<0.12)&(Z>0.905)
pts=W[m]; ys,xs=np.where(m)
print('bowl pts n',len(pts))
print('x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
print('px bbox u',xs.min(),xs.max(),'v',ys.min(),ys.max())
# rim: z>1.0
r=pts[pts[:,2]>0.995]
print('rim x %.3f..%.3f y %.3f..%.3f'%(r[:,0].min(),r[:,0].max(),r[:,1].min(),r[:,1].max()), 'center', r[:,0].mean(), r[:,1].mean())
# highest
print('max z', pts[:,2].max())
h,e=np.histogram(pts[:,2],bins=np.arange(0.9,1.05,0.01))
print(list(zip(np.round(e,2),h)))
"


# openrua op 12
python3 -c "
import numpy as np, cv2
W=np.load('agentview_xyz.npy'); img=cv2.imread('agentview.png')
Z=W[:,:,2]
m=(W[:,:,0]>-0.26)&(W[:,:,0]<-0.02)&(W[:,:,1]>-0.05)&(W[:,:,1]<0.12)&(Z>0.905)&(Z<1.03)
pts=W[m]; ys,xs=np.where(m)
print('bowl pts n',len(pts))
print('x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
print('px bbox u',xs.min(),xs.max(),'v',ys.min(),ys.max())
r=pts[pts[:,2]>1.005]
print('rim x %.3f..%.3f y %.3f..%.3f'%(r[:,0].min(),r[:,0].max(),r[:,1].min(),r[:,1].max()), 'center', r[:,0].mean(), r[:,1].mean())
# fit circle to rim xy
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=r[:,0]**2+r[:,1]**2
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c+cx**2+cy**2)
print('circle fit center',cx,cy,'radius',R)
"


# openrua op 13
timeout 60 python3 -c "
import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node('tfdump'); b=Buffer(); TransformListener(b,n)
t=time.time()
while time.time()-t<5: rclpy.spin_once(n,timeout_sec=0.1)
print(b.all_frames_as_yaml())
"

# openrua op 14
timeout 120 python3 scene.py robot0_eye_in_hand tf && timeout 120 python3 scene.py robot0_eye_in_hand save && timeout 60 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A3 "At time" | head -5

# openrua op 15
python3 -c "
import numpy as np, cv2
W=np.load('robot0_eye_in_hand_xyz.npy'); img=cv2.imread('robot0_eye_in_hand.png')
Z=W[:,:,2]
print('table z', np.median(Z[100:300,40:150]))
# bowl: dark pixels around (285,200)
m=(np.abs(W[:,:,0]+0.14)<0.09)&(np.abs(W[:,:,1]-0.03)<0.09)&(Z>0.92)&(Z<1.1)
pts=W[m]
print('bowl x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
r=pts[pts[:,2]>pts[:,2].max()-0.01]
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=r[:,0]**2+r[:,1]**2
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c+cx**2+cy**2)
print('rim circle center',cx,cy,'radius',R,'rim z',np.median(r[:,2]))
h,e=np.histogram(pts[:,2],bins=np.arange(0.9,1.05,0.005)); print([(round(a,3),b) for a,b in zip(e,h) if b])
# drawer region: everything with y<-0.05 and z>0.905
m2=(W[:,:,1]<-0.04)&(Z>0.905)&(Z<1.2)&(W[:,:,0]>-0.4)
p2=W[m2]
h,e=np.histogram(p2[:,2],bins=np.arange(0.9,1.2,0.005)); print([(round(a,3),b) for a,b in zip(e,h) if b])
"


# openrua op 16
python3 -c "
import numpy as np, cv2
W=np.load('robot0_eye_in_hand_xyz.npy'); img=cv2.imread('robot0_eye_in_hand.png')
Z=W[:,:,2]
def bb(m,name):
    p=W[m]; ys,xs=np.where(m)
    if len(p)==0: print(name,'none'); return
    print(f'{name}: n={len(p)} x {p[:,0].min():.3f}..{p[:,0].max():.3f} y {p[:,1].min():.3f}..{p[:,1].max():.3f} z {p[:,2].min():.3f}..{p[:,2].max():.3f} | px u {xs.min()}-{xs.max()} v {ys.min()}-{ys.max()}')
reg=(W[:,:,1]<-0.04)&(W[:,:,0]>-0.4)
bb(reg&(Z>0.915)&(Z<0.925),'drawer floor 0.92')
bb(reg&(Z>0.975)&(Z<0.985),'0.98 layer')
bb(reg&(Z>1.01)&(Z<1.05),'1.01-1.05 layer')
bb(reg&(Z>1.05),'above 1.05')
# drawer floor detailed rows: for each x-slice, y range
m=reg&(Z>0.915)&(Z<0.925); p=W[m]
for x in np.arange(-0.25,0.06,0.02):
    s=p[np.abs(p[:,0]-x)<0.01]
    if len(s): print(f'x={x:.2f} floor y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"


# openrua op 17
python3 -c "
import numpy as np, cv2
W=np.load('robot0_eye_in_hand_xyz.npy'); img=cv2.imread('robot0_eye_in_hand.png')
res=0.001; x0,x1,y0,y1=-0.3,0.1,-0.3,0.15
Hh=int((x1-x0)/res); Ww=int((y1-y0)/res)
P=W.reshape(-1,3); C=img.reshape(-1,3)
m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.89)&(P[:,2]<1.25)
P=P[m]; C=C[m]
r=((P[:,0]-x0)/res).astype(int); c=((P[:,1]-y0)/res).astype(int)
Zmap=np.full((Hh,Ww),np.nan); Cmap=np.zeros((Hh,Ww,3),np.uint8)
order=np.argsort(P[:,2]); Zmap[r[order],c[order]]=P[order,2]; Cmap[r[order],c[order]]=C[order]
hm=np.nan_to_num((Zmap-0.89)/0.25*255,nan=0).clip(0,255).astype(np.uint8)
hm=cv2.applyColorMap(hm,cv2.COLORMAP_JET); hm[np.isnan(Zmap)]=0
for im in (hm,Cmap):
    for xv in np.arange(-0.3,0.1,0.05):
        rr=int((xv-x0)/res); cv2.line(im,(0,rr),(Ww,rr),(255,255,255),1); cv2.putText(im,f'x={xv:.2f}',(2,rr-2),0,0.4,(255,255,255),1)
    for yv in np.arange(-0.3,0.15,0.05):
        cc=int((yv-y0)/res); cv2.line(im,(cc,0),(cc,Hh),(255,255,255),1); cv2.putText(im,f'{yv:.2f}',(cc+2,12),0,0.4,(255,255,255),1)
cv2.imwrite('eih_topdown.png',np.hstack([Cmap,hm]))
# z profile along y at x=-0.1
zs=Zmap[int((-0.1-x0)/res)]
for yy in np.arange(-0.3,0.0,0.01):
    cc=int((yy-y0)/res); seg=zs[cc:cc+10]; seg=seg[~np.isnan(seg)]
    print(f'y={yy:.2f} z={np.median(seg) if len(seg) else float(\"nan\"):.3f}')
"


# openrua op 18
python3 -c "
import numpy as np, cv2
W=np.load('robot0_eye_in_hand_xyz.npy'); img=cv2.imread('robot0_eye_in_hand.png')
res=0.001; x0,x1,y0,y1=-0.3,0.1,-0.3,0.15
Hh=int(round((x1-x0)/res))+1; Ww=int(round((y1-y0)/res))+1
P=W.reshape(-1,3); C=img.reshape(-1,3)
m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>0.89)&(P[:,2]<1.25)
P=P[m]; C=C[m]
r=((P[:,0]-x0)/res).astype(int); c=((P[:,1]-y0)/res).astype(int)
Zmap=np.full((Hh,Ww),np.nan); Cmap=np.zeros((Hh,Ww,3),np.uint8)
order=np.argsort(P[:,2]); Zmap[r[order],c[order]]=P[order,2]; Cmap[r[order],c[order]]=C[order]
hm=np.nan_to_num((Zmap-0.89)/0.25*255,nan=0).clip(0,255).astype(np.uint8)
hm=cv2.applyColorMap(hm,cv2.COLORMAP_JET); hm[np.isnan(Zmap)]=0
for im in (hm,Cmap):
    for xv in np.arange(-0.3,0.1,0.05):
        rr=int((xv-x0)/res); cv2.line(im,(0,rr),(Ww,rr),(255,255,255),1); cv2.putText(im,f'x={xv:.2f}',(2,rr-2),0,0.4,(255,255,255),1)
    for yv in np.arange(-0.3,0.15,0.05):
        cc=int((yv-y0)/res); cv2.line(im,(cc,0),(cc,Hh),(255,255,255),1); cv2.putText(im,f'{yv:.2f}',(cc+2,12),0,0.4,(255,255,255),1)
cv2.imwrite('eih_topdown.png',np.hstack([Cmap,hm]))
zs=Zmap[int((-0.1-x0)/res)]
for yy in np.arange(-0.3,0.0,0.01):
    cc=int((yy-y0)/res); seg=zs[cc:cc+10]; seg=seg[~np.isnan(seg)]
    print(f'y={yy:.2f} z={np.median(seg) if len(seg) else float(\"nan\"):.3f}')
"


# openrua op 19
timeout 120 python3 scene.py sideview tf && timeout 120 python3 scene.py sideview save && timeout 120 python3 scene.py frontview tf && timeout 120 python3 scene.py frontview save

# openrua op 20
python3 -c "
import numpy as np, cv2
W=np.load('sideview_xyz.npy'); img=cv2.imread('sideview.png')
# elevation view: cols = x (world), rows = z (up). Include points with y in [-0.45, 0.15]
res=0.001; x0,x1,z0,z1=-0.3,0.15,0.88,1.2
Ww=int(round((x1-x0)/res))+1; Hh=int(round((z1-z0)/res))+1
P=W.reshape(-1,3); C=img.reshape(-1,3)
m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,2]>z0)&(P[:,2]<z1)&(P[:,1]>-0.45)&(P[:,1]<0.15)
P=P[m]; C=C[m]
c=((P[:,0]-x0)/res).astype(int); r=(Hh-1-(P[:,2]-z0)/res).astype(int)
Ymap=np.full((Hh,Ww),np.nan); Cmap=np.zeros((Hh,Ww,3),np.uint8)
order=np.argsort(-P[:,1])  # nearest to camera (largest y) drawn last
Ymap[r[order],c[order]]=P[order,1]; Cmap[r[order],c[order]]=C[order]
hm=np.nan_to_num((Ymap+0.45)/0.6*255,nan=0).clip(0,255).astype(np.uint8)
hm=cv2.applyColorMap(hm,cv2.COLORMAP_JET); hm[np.isnan(Ymap)]=0
for im in (hm,Cmap):
    for xv in np.arange(-0.3,0.15,0.05):
        cc=int((xv-x0)/res); cv2.line(im,(cc,0),(cc,Hh),(255,255,255),1); cv2.putText(im,f'{xv:.2f}',(cc+2,12),0,0.4,(255,255,255),1)
    for zv in np.arange(0.9,1.2,0.05):
        rr=int(Hh-1-(zv-z0)/res); cv2.line(im,(0,rr),(Ww,rr),(255,255,255),1); cv2.putText(im,f'z={zv:.2f}',(2,rr-2),0,0.4,(255,255,255),1)
cv2.imwrite('side_elev.png',np.vstack([Cmap,hm]))
print(Cmap.shape)
"


# openrua op 21
python3 -c "
import cv2; im=cv2.imread('side_elev.png'); im=cv2.resize(im,None,fx=2,fy=2,interpolation=cv2.INTER_NEAREST); cv2.imwrite('side_elev2.png',im)"

# openrua op 22
python3 -c "
import cv2; im=cv2.imread('sideview.png'); crop=im[200:320,230:420]; cv2.imwrite('side_crop.png',cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))"

# openrua op 23
python3 -c "
import numpy as np, cv2
W=np.load('frontview_xyz.npy'); img=cv2.imread('frontview.png')
res=0.001; y0,y1,z0,z1=-0.45,0.15,0.88,1.2
Ww=int(round((y1-y0)/res))+1; Hh=int(round((z1-z0)/res))+1
P=W.reshape(-1,3); C=img.reshape(-1,3)
m=(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]>z0)&(P[:,2]<z1)&(P[:,0]>-0.3)&(P[:,0]<0.12)
P=P[m]; C=C[m]
c=((P[:,1]-y0)/res).astype(int); r=(Hh-1-(P[:,2]-z0)/res).astype(int)
Xmap=np.full((Hh,Ww),np.nan); Cmap=np.zeros((Hh,Ww,3),np.uint8)
order=np.argsort(-P[:,0])
Xmap[r[order],c[order]]=P[order,0]; Cmap[r[order],c[order]]=C[order]
hm=np.nan_to_num((Xmap+0.3)/0.42*255,nan=0).clip(0,255).astype(np.uint8)
hm=cv2.applyColorMap(hm,cv2.COLORMAP_JET); hm[np.isnan(Xmap)]=0
for im in (hm,Cmap):
    for yv in np.arange(-0.45,0.15,0.05):
        cc=int((yv-y0)/res); cv2.line(im,(cc,0),(cc,Hh),(255,255,255),1); cv2.putText(im,f'{yv:.2f}',(cc+2,12),0,0.4,(255,255,255),1)
    for zv in np.arange(0.9,1.2,0.05):
        rr=int(Hh-1-(zv-z0)/res); cv2.line(im,(0,rr),(Ww,rr),(255,255,255),1); cv2.putText(im,f'z={zv:.2f}',(2,rr-2),0,0.4,(255,255,255),1)
out=np.vstack([Cmap,hm]); cv2.imwrite('front_elev.png',cv2.resize(out,None,fx=1.5,fy=1.5,interpolation=cv2.INTER_NEAREST))
# bowl profile: points with x in [-0.2,-0.05], y in [-0.04,0.09]
b=P[(P[:,0]>-0.2)&(P[:,0]<-0.05)&(P[:,1]>-0.04)&(P[:,1]<0.09)]
for z in np.arange(0.9,1.03,0.01):
    s=b[np.abs(b[:,2]-z)<0.005]
    if len(s): print(f'z={z:.2f} n={len(s)} y {s[:,1].min():.3f}..{s[:,1].max():.3f} x {s[:,0].min():.3f}..{s[:,0].max():.3f}')
"


# openrua op 24
python3 -c "
import numpy as np, cv2
W=np.load('frontview_xyz.npy'); img=cv2.imread('frontview.png')
u=330
for v in range(296,345,2):
    print(v, np.round(W[v,u],3), img[v,u])
"


# openrua op 25
python3 -c "
import numpy as np, cv2
W=np.load('robot0_eye_in_hand_xyz.npy'); img=cv2.imread('robot0_eye_in_hand.png')
D=np.load('robot0_eye_in_hand.npy') if False else None
v=200
for u in range(200,380,6):
    print(u, np.round(W[v,u],3), img[v,u])
"


# openrua op 26
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.70 2 && ros2 topic echo /joint_states --once | grep -A9 position && timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 120 python3 scene.py frontview save

# openrua op 27
python3 -c "
import numpy as np, cv2
W=np.load('frontview_xyz.npy'); img=cv2.imread('frontview.png')
P=W.reshape(-1,3)
b=P[(P[:,0]>-0.2)&(P[:,0]<-0.05)&(P[:,1]>-0.04)&(P[:,1]<0.09)&(P[:,2]>0.905)]
print('bowl z range', b[:,2].min(), b[:,2].max())
for z in np.arange(0.9,1.03,0.01):
    s=b[np.abs(b[:,2]-z)<0.005]
    if len(s): print(f'z={z:.2f} n={len(s)} y {s[:,1].min():.3f}..{s[:,1].max():.3f} x {s[:,0].min():.3f}..{s[:,0].max():.3f}')
"


# openrua op 28
timeout 120 python3 scene.py agentview save && python3 -c "
import numpy as np
W=np.load('agentview_xyz.npy'); Z=W[:,:,2]
m=(W[:,:,0]>-0.26)&(W[:,:,0]<-0.02)&(W[:,:,1]>-0.05)&(W[:,:,1]<0.12)&(Z>0.905)&(Z<0.97)
pts=W[m]
print('bowl x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].min(),pts[:,2].max()))
r=pts[pts[:,2]>pts[:,2].max()-0.012]
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=r[:,0]**2+r[:,1]**2
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c+cx**2+cy**2)
print('rim circle center',cx,cy,'radius',R,'rim z',np.median(r[:,2]))
"

# openrua op 29
python3 -c "
import numpy as np
W=np.load('agentview_xyz.npy'); Z=W[:,:,2]
d=np.hypot(W[:,:,0]+0.137,W[:,:,1]-0.026)
m=(d<0.07)&(Z>0.94)&(Z<0.97)
r=W[m]
print('n',len(r),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(r[:,0].min(),r[:,0].max(),r[:,1].min(),r[:,1].max(),r[:,2].min(),r[:,2].max()))
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=r[:,0]**2+r[:,1]**2
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c+cx**2+cy**2)
print('rim circle center',cx,cy,'radius',R,'rim z',np.median(r[:,2]))
"

# openrua op 30
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('urdfget')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.2)
open('robot.urdf','w').write(got[0]); print(len(got[0]))
" && grep -n -A12 'name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A10 'name="panda_leftfinger"' robot.urdf | head -40

# openrua op 31
python3 -c "
import numpy as np, struct
def stl_bbox(p):
    d=open(p,'rb').read()
    n=struct.unpack_from('<I',d,80)[0]
    v=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ['hand','finger']:
    print(f, stl_bbox(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'))
"

# openrua op 32
find / -name "hand.stl" -o -name "finger.stl" -o -name "hand*.obj" 2>/dev/null | head; find / -path "*panda*" -name "*.stl" 2>/dev/null | head

# openrua op 33
python3 -c "
import numpy as np
for cam in ['agentview','frontview']:
    W=np.load(f'{cam}_xyz.npy')
    m=(W[:,:,1]>-0.243)&(W[:,:,1]<-0.17)&(W[:,:,2]>0.99)&(W[:,:,2]<1.12)&(W[:,:,0]>-0.3)&(W[:,:,0]<0.1)
    p=W[m]
    print(cam,'n',len(p))
    for z in np.arange(0.99,1.12,0.01):
        s=p[np.abs(p[:,2]-z)<0.005]
        if len(s): print(f'  z={z:.2f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"

# openrua op 34
python3 -c "
import numpy as np
for cam in ['agentview','frontview','sideview']:
    W=np.load(f'{cam}_xyz.npy')
    m=(W[:,:,1]>-0.215)&(W[:,:,1]<-0.17)&(W[:,:,2]>0.99)&(W[:,:,2]<1.12)&(W[:,:,0]>-0.3)&(W[:,:,0]<0.1)
    p=W[m]
    print(cam,'handle pts n',len(p))
    if len(p): print(f'  x {p[:,0].min():.3f}..{p[:,0].max():.3f} y {p[:,1].min():.3f}..{p[:,1].max():.3f} z {p[:,2].min():.3f}..{p[:,2].max():.3f}')
# drawer front wall & handle from eih
W=np.load('robot0_eye_in_hand_xyz.npy')
m=(W[:,:,1]>-0.10)&(W[:,:,1]<-0.03)&(W[:,:,2]>0.93)&(W[:,:,2]<1.0)&(W[:,:,0]>-0.3)&(W[:,:,0]<0.1)
p=W[m]
for y in np.arange(-0.10,-0.03,0.005):
    s=p[np.abs(p[:,1]-y)<0.0025]
    if len(s): print(f'  y={y:.3f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}')
"

# openrua op 35
timeout 120 python3 scene.py sideview save && python3 -c "
import numpy as np
W=np.load('sideview_xyz.npy')
m=(W[:,:,2]>1.10)&(W[:,:,2]<1.30)&(np.abs(W[:,:,1])<0.2)&(W[:,:,0]>-0.4)&(W[:,:,0]<0.0)
p=W[m]
for z in np.arange(1.10,1.30,0.01):
    s=p[np.abs(p[:,2]-z)<0.005]
    if len(s): print(f'z={z:.2f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"

# openrua op 36
mkdir -p "$(dirname /workspace/notes.md)"
cat > /workspace/notes.md <<'OPENRUA_EOF'
# Scene notes (world frame, metres)

- Robot: Franka Panda, base panda_link0 at world (-0.66, 0, 0.912). Hand frame
  points down at start; fingers slide along hand-y. TCP = hand + 0.1034 hand-z.
  Hand housing ~0.2 (finger axis) x 0.06 x 0.06 tall.
- Table top z = 0.90.
- Black bowl (after settling): centre (-0.137, 0.025), rim radius 0.0525,
  rim z 0.955, bottom on table (height ~0.055). Too wide for the 0.08 gripper
  -> grasp the rim (one finger inside, one outside), fingers along world x.
- Cabinet body: x -0.215..0.025, y -0.41..-0.243, top z 1.127. Upper drawer
  fronts at y=-0.222; their handles protrude to y=-0.191, x -0.133..-0.044,
  z 1.01-1.02 and 1.084-1.099.
- Bottom drawer OPEN (slides along y, front faces +y): floor z 0.924,
  interior x -0.214..0.017, front wall inner face y~-0.088, outer ~-0.07,
  wall tops z 0.98. Drawer handle bar y -0.045..-0.035, x -0.131..-0.043,
  z 0.945-0.955. Closed position: front face at y=-0.222 (travel ~0.15).
- Bottle at (0.05, -0.08), radius ~0.03, z to 1.12 - keep x<0.0 near it.
- Cameras: agentview (+x side, looking -x), frontview, sideview (+y side),
  birdview (top), robot0_eye_in_hand (on hand, +0.05 hand-x offset).
OPENRUA_EOF

# openrua op 37
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable arm helper: joint state, IK (world-frame TCP poses), trajectories,
FK, gripper. Clients are built once per Arm instance."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = TRAJ["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation
TCP_OFF = float(M["hand"]["tcp_offset_m"])


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
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def hand_R(tilt_deg=0.0):
    """Hand pointing down, fingers along world x, flange tilted toward +y by
    tilt_deg (rotation about world x)."""
    R0 = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)
    t = math.radians(-tilt_deg)
    Rx = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)],
                   [0, math.sin(t), math.cos(t)]])
    return Rx @ R0


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 10)
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj_cli = ActionClient(self.node, FollowJointTrajectory,
                                     TRAJ["port"])
        self.grip_cli = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)
        self.traj_cli.wait_for_server(10)
        self.grip_cli.wait_for_server(10)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    def joints(self, fresh=True):
        if fresh:
            self._js = {}
        t = time.time()
        while not all(j in self._js for j in JOINTS) and time.time() - t < 20:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in JOINTS]

    def fingers(self):
        js = self.joints()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    # ---------- kinematics ----------
    def ik(self, tcp_world, R, seed=None, timeout=60):
        """IK for a TCP position (world) and hand rotation matrix R."""
        hand_world = np.asarray(tcp_world, float) - TCP_OFF * R[:, 2]
        p_base = hand_world - BASE
        q = R_to_quat(R)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        pose = req.ik_request.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = map(float, p_base)
        (pose.orientation.x, pose.orientation.y, pose.orientation.z,
         pose.orientation.w) = map(float, q)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            code = None if res is None else res.error_code.val
            raise RuntimeError(f"IK failed (code {code}) for tcp {tcp_world}")
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def fk(self, q=None, link="panda_hand"):
        """Return (pos_world, R) of link for joint vector q."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError("FK failed")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        R = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z,
                      p.orientation.w)
        return pos, R

    def tcp(self, q=None):
        pos, R = self.fk(q)
        return pos + TCP_OFF * R[:, 2], R

    # ---------- motion ----------
    def move_joints(self, waypoints, times):
        """waypoints: list of 7-vectors; times: cumulative seconds."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for q, t in zip(waypoints, times):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.traj_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        gh = send.result()
        if not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q_now = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q_now, waypoints[-1]))
        print(f"traj done error_code={code} max joint err={err:.4f}")
        return code, err

    def move_tcp(self, tcp_world, R, seconds=3.0, seed=None):
        q = self.ik(tcp_world, R, seed)
        return self.move_joints([q], [seconds]), q

    def move_tcp_line(self, p0, p1, R, n=6, seconds=4.0, seed=None):
        """Straight TCP line p0->p1 as IK waypoints in one trajectory."""
        qs = []
        seed = seed if seed is not None else self.arm_q()
        for i in range(1, n + 1):
            p = np.asarray(p0) + (np.asarray(p1) - np.asarray(p0)) * i / n
            seed = self.ik(p, R, seed)
            qs.append(seed)
        times = [seconds * i / n for i in range(1, n + 1)]
        return self.move_joints(qs, times), qs

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f
OPENRUA_EOF

# openrua op 38
timeout 300 python3 -c "
import numpy as np
from arm import Arm, hand_R, R_to_quat
a=Arm()
q=a.arm_q(); print('q',np.round(q,3))
pos,R=a.fk(q); print('hand world',np.round(pos,4)); print('R',np.round(R,3))
tcp,_=a.tcp(q); print('tcp',np.round(tcp,4))
print('fingers',a.fingers())
R20=hand_R(20); print('R20',np.round(R20,3),'quat',np.round(R_to_quat(R20),4))
# IK checks
for name,p,Rm in [('pregrasp',[-0.185,0.025,1.05],R20),('grasp',[-0.185,0.025,0.93],R20),('place',[-0.128,-0.155,0.957],R20),('push_start',[-0.165,-0.03,0.94],hand_R(45)),('push_end',[-0.165,-0.222,0.94],hand_R(45))]:
    try:
        s=a.ik(p,Rm); t,_=a.tcp(s); print(name,'ok q',np.round(s,3),'fk tcp',np.round(t,4))
    except Exception as e: print(name,'FAIL',e)
"

# openrua op 39
timeout 300 python3 -c "
import numpy as np, arm
from arm import Arm, hand_R
arm.BASE=np.zeros(3)
a=Arm()
q=a.arm_q()
pos,R=a.fk(q); print('hand (model frame)',np.round(pos,4))
for name,p,Rm in [('pregrasp',[-0.185,0.025,1.05],hand_R(20)),('grasp',[-0.185,0.025,0.93],hand_R(20)),('place',[-0.128,-0.155,0.957],hand_R(20)),('push_start',[-0.165,-0.03,0.94],hand_R(45)),('push_end',[-0.165,-0.222,0.94],hand_R(45))]:
    try:
        s=a.ik(p,Rm); t,_=a.tcp(s); print(name,'ok q',np.round(s,3),'fk tcp',np.round(t,4))
    except Exception as e: print(name,'FAIL',e)
"

# openrua op 40
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation", "new_string": "# Verified on this machine: /compute_fk with empty frame_id returns the hand\\n# at the same coordinates TF reports for world->panda_hand, and IK accepts\\n# world-frame poses. So the planner\'s model frame IS world here; no offset.\\nBASE = np.zeros(3)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 41
timeout 900 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm()
R20=hand_R(20)
print('fingers before', a.fingers())
# 1. pre-grasp above bowl -x rim
(code,err),q1=a.move_tcp([-0.185,0.025,1.05],R20,seconds=4)
t,_=a.tcp(); print('tcp now',np.round(t,4))
# 2. descend
(code,err),q2=a.move_tcp([-0.185,0.025,0.93],R20,seconds=3,seed=q1)
t,_=a.tcp(); print('tcp now',np.round(t,4))
" 2>&1 | tail -20 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 42
timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 43
timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 44
timeout 600 python3 -u -c "
import numpy as np
from arm import Arm
a=Arm()
f=a.gripper(0.0)
import time
for i in range(3):
    print('fingers', a.fingers())
" 2>&1 | tail -5

# openrua op 45
timeout 900 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm()
R20=hand_R(20)
(code,err),q=a.move_tcp([-0.185,0.025,1.08],R20,seconds=3)
t,_=a.tcp(); print('tcp now',np.round(t,4)); print('fingers',a.fingers())
" 2>&1 | tail -4 && timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 46
timeout 120 python3 scene.py frontview save && python3 -c "
import numpy as np
W=np.load('frontview_xyz.npy'); P=W.reshape(-1,3)
# bowl should be near tcp (-0.184,0.025,1.08); look at points x in [-0.3,-0.05], y in [-0.08,0.13], z in [0.95,1.12]
b=P[(P[:,0]>-0.3)&(P[:,0]<-0.05)&(P[:,1]>-0.08)&(P[:,1]<0.13)&(P[:,2]>0.95)&(P[:,2]<1.13)]
print('n',len(b))
for z in np.arange(0.95,1.13,0.01):
    s=b[np.abs(b[:,2]-z)<0.005]
    if len(s): print(f'z={z:.2f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"

# openrua op 47
timeout 900 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm()
R20=hand_R(20)
(code,err),q=a.move_tcp([-0.185,0.025,0.985],R20,seconds=3)
t,_=a.tcp(); print('tcp now',np.round(t,4))
a.gripper(0.04)
(code,err),q=a.move_tcp([-0.185,0.025,1.10],hand_R(0),seconds=3)
t,_=a.tcp(); print('tcp now',np.round(t,4))
" 2>&1 | tail -6 && timeout 120 python3 scene.py agentview save && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import numpy as np
W=np.load('agentview_xyz.npy'); Z=W[:,:,2]
m=(W[:,:,0]>-0.3)&(W[:,:,0]<0.0)&(W[:,:,1]>-0.03)&(W[:,:,1]<0.15)&(Z>0.905)&(Z<0.99)
p=W[m]; print('bowl-ish pts', len(p), 'z max', p[:,2].max())
d=np.hypot(W[:,:,0]-np.median(p[:,0]),W[:,:,1]-np.median(p[:,1]))
r=W[(d<0.08)&(Z>p[:,2].max()-0.012)&(Z<0.99)]
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=r[:,0]**2+r[:,1]**2
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c+cx**2+cy**2)
print('rim circle center',cx,cy,'radius',R,'rim z',np.median(r[:,2]))
"

# openrua op 48
timeout 1200 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm()
R0=hand_R(0)
bx,by=-0.177,0.043
gx=bx-0.048
(code,err),q1=a.move_tcp([gx,by,1.05],R0,seconds=4)
print('tcp',np.round(a.tcp()[0],4))
(code,err),q2=a.move_tcp([gx,by,0.93],R0,seconds=3,seed=q1)
print('tcp',np.round(a.tcp()[0],4))
" 2>&1 | tail -4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 49
timeout 900 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm()
a.gripper(0.0)
print('fingers', a.fingers())
(code,err),q=a.move_tcp([-0.225,0.043,1.08],hand_R(0),seconds=3)
print('tcp',np.round(a.tcp()[0],4),'fingers',a.fingers())
" 2>&1 | tail -4 && timeout 120 python3 scene.py frontview save && timeout 60 python3 tools/perception/cam_snap.py frontview && python3 -c "
import numpy as np
W=np.load('frontview_xyz.npy'); P=W.reshape(-1,3)
b=P[(P[:,0]>-0.35)&(P[:,0]<-0.08)&(P[:,1]>-0.03)&(P[:,1]<0.13)&(P[:,2]>0.96)&(P[:,2]<1.16)]
print('n',len(b))
for z in np.arange(0.96,1.16,0.01):
    s=b[np.abs(b[:,2]-z)<0.005]
    if len(s): print(f'z={z:.2f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"

# openrua op 50
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "def hand_R(tilt_deg=0.0):\\n    \\"\\"\\"Hand pointing down, fingers along world x, flange tilted toward +y by\\n    tilt_deg (rotation about world x).\\"\\"\\"\\n    R0 = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)\\n    t = math.radians(-tilt_deg)\\n    Rx = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)],\\n                   [0, math.sin(t), math.cos(t)]])\\n    return Rx @ R0", "new_string": "def hand_R(tilt_deg=0.0, yaw_deg=0.0):\\n    \\"\\"\\"Hand pointing down, fingers along world x, flange tilted toward +y by\\n    tilt_deg (rotation about world x). yaw_deg rotates the hand about world z\\n    first (180 -> hand +x points to world -y instead of +y).\\"\\"\\"\\n    R0 = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]], float)\\n    y = math.radians(yaw_deg)\\n    Rz = np.array([[math.cos(y), -math.sin(y), 0],\\n                   [math.sin(y), math.cos(y), 0], [0, 0, 1]])\\n    t = math.radians(-tilt_deg)\\n    Rx = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)],\\n                   [0, math.sin(t), math.cos(t)]])\\n    return Rx @ Rz @ R0", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 51
timeout 900 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm()
R=hand_R(0,180); print('R flipped',np.round(R,3))
q=a.ik([-0.225,0.043,1.10],R); print('ik q',np.round(q,3))
(code,err),q=a.move_tcp([-0.225,0.043,1.10],R,seconds=5)
p,Rn=a.tcp(); print('tcp',np.round(p,4)); print('R now',np.round(Rn,3)); print('fingers',a.fingers())
" 2>&1 | tail -12 && timeout 120 python3 scene.py frontview save && timeout 60 python3 tools/perception/cam_snap.py frontview && python3 -c "
import numpy as np
W=np.load('frontview_xyz.npy'); P=W.reshape(-1,3)
b=P[(P[:,0]>-0.35)&(P[:,0]<-0.08)&(P[:,1]>-0.10)&(P[:,1]<0.16)&(P[:,2]>0.98)&(P[:,2]<1.14)]
print('n',len(b))
for z in np.arange(0.98,1.14,0.01):
    s=b[np.abs(b[:,2]-z)<0.005]
    if len(s): print(f'z={z:.2f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"

# openrua op 52
timeout 300 python3 -u -c "
import numpy as np
from arm import Arm
a=Arm()
for i in range(2):
    q=a.arm_q(); print('q',np.round(q,3))
p,R=a.tcp(q); print('tcp',np.round(p,4)); print(np.round(R,3))
" 2>&1 | tail -8

# openrua op 53
timeout 900 python3 -u -c "
import numpy as np
from arm import Arm
a=Arm()
q=a.arm_q(); print('q before',np.round(q,3))
q2=list(q); q2[6]=1.615
a.move_joints([q2],[6.0])
print('q after',np.round(a.arm_q(),3),'fingers',a.fingers())
" 2>&1 | tail -4

# openrua op 54
timeout 1200 python3 -u -c "
import numpy as np
from arm import Arm
a=Arm()
q=a.arm_q(); print('q before',np.round(q,3))
q2=list(q); q2[6]=1.615
a.move_joints([q2],[12.0])
print('q after',np.round(a.arm_q(),3),'fingers',a.fingers())
" 2>&1 | tail -4

# openrua op 55
timeout 300 python3 -c "
import numpy as np
from arm import Arm
a=Arm(); p,R=a.tcp(); print('tcp',np.round(p,4)); print(np.round(R,3))
" 2>&1 | tail -4; timeout 120 python3 scene.py frontview save && timeout 60 python3 tools/perception/cam_snap.py frontview && python3 -c "
import numpy as np
W=np.load('frontview_xyz.npy'); P=W.reshape(-1,3)
b=P[(P[:,0]>-0.35)&(P[:,0]<-0.08)&(P[:,1]>-0.10)&(P[:,1]<0.16)&(P[:,2]>1.0)&(P[:,2]<1.14)]
print('n',len(b))
for z in np.arange(1.0,1.14,0.01):
    s=b[np.abs(b[:,2]-z)<0.005]
    if len(s): print(f'z={z:.2f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"

# openrua op 56
timeout 300 python3 -c "
import numpy as np
from arm import Arm
a=Arm(); q=a.arm_q()
for link in ['panda_link8','panda_hand']:
    p,R=a.fk(q,link); print(link,np.round(p,4)); print(np.round(R,3))
" 2>&1 | tail -8

# openrua op 57
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        hand_world = np.asarray(tcp_world, float) - TCP_OFF * R[:, 2]\\n        p_base = hand_world - BASE\\n        q = R_to_quat(R)", "new_string": "        hand_world = np.asarray(tcp_world, float) - TCP_OFF * R[:, 2]\\n        p_base = hand_world - BASE\\n        # the panda_arm group\'s tip is panda_link8; panda_hand is link8\\n        # rotated -45 deg about z (URDF panda_hand_joint), so ask IK for\\n        # R_link8 = R_hand * Rz(+45 deg)\\n        q = R_to_quat(R @ RZ45)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 58
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "TCP_OFF = float(M[\\"hand\\"][\\"tcp_offset_m\\"])", "new_string": "TCP_OFF = float(M[\\"hand\\"][\\"tcp_offset_m\\"])\\n_c45 = math.cos(math.pi / 4)\\nRZ45 = np.array([[_c45, -_c45, 0], [_c45, _c45, 0], [0, 0, 1]])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 59
timeout 600 python3 -c "
import numpy as np
from arm import Arm, hand_R
a=Arm(); q=a.arm_q()
# verify IK->FK roundtrip for hand orientation without moving
for tilt in [0,20,45]:
    R=hand_R(tilt)
    s=a.ik([-0.2,-0.05,1.05],R,seed=q)
    p,Rf=a.tcp(s)
    print('tilt',tilt,'tcp',np.round(p,4),'R err',np.abs(Rf-R).max().round(4),'q7',round(s[6],3))
" 2>&1 | tail -4

# openrua op 60
timeout 1200 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm()
p,R=a.tcp()
(code,err),q=a.move_tcp([-0.225,0.043,0.965],R,seconds=3)
print('tcp',np.round(a.tcp()[0],4))
a.gripper(0.04)
(code,err),q=a.move_tcp([-0.225,0.043,1.12],hand_R(0),seconds=8)
print('tcp',np.round(a.tcp()[0],4)); print(np.round(a.tcp()[1],3))
" 2>&1 | tail -9 && timeout 120 python3 scene.py agentview save && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import numpy as np
W=np.load('agentview_xyz.npy'); Z=W[:,:,2]
m=(W[:,:,0]>-0.4)&(W[:,:,0]<0.0)&(W[:,:,1]>-0.03)&(W[:,:,1]<0.2)&(Z>0.905)&(Z<0.99)
p=W[m]; print('bowl-ish pts', len(p), 'z max', p[:,2].max(), 'x %.3f..%.3f y %.3f..%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()))
d=np.hypot(W[:,:,0]-np.median(p[:,0]),W[:,:,1]-np.median(p[:,1]))
r=W[(d<0.08)&(Z>p[:,2].max()-0.012)&(Z<0.99)]
A=np.c_[2*r[:,0],2*r[:,1],np.ones(len(r))]; b=r[:,0]**2+r[:,1]**2
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; R=np.sqrt(c+cx**2+cy**2)
print('rim circle center',cx,cy,'radius',R,'rim z',np.median(r[:,2]))
"

# openrua op 61
timeout 600 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm()
q=a.ik([-0.225,0.043,1.12],hand_R(0))
cur=a.arm_q(); print('cur',np.round(cur,3)); print('tgt',np.round(q,3))
a.move_joints([q],[14])
print(np.round(a.arm_q(),3)); print(np.round(a.tcp()[1],3))
" 2>&1 | tail -8; python3 -c "
import numpy as np
W=np.load('agentview_xyz.npy'); Z=W[:,:,2]
d=np.hypot(W[:,:,0]+0.253,W[:,:,1]-0.023)
r=W[(d<0.065)&(d>0.04)&(Z>0.935)&(Z<0.99)]
import math
ang=np.degrees(np.arctan2(r[:,1]-0.023,r[:,0]+0.253))
for lo in range(-180,180,45):
    m=(ang>=lo)&(ang<lo+45)
    if m.sum(): print(lo, m.sum(), 'zmax %.3f'%r[m,2].max())
"

# openrua op 62
timeout 1200 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm(); R=hand_R(0)
a.move_tcp([-0.303,0.023,1.05],R,seconds=4)
a.move_tcp_line([-0.303,0.023,1.05],[-0.303,0.023,0.935],R,n=4,seconds=4)
print('tcp',np.round(a.tcp()[0],4))
a.gripper(0.0)
a.move_tcp_line([-0.303,0.023,0.935],[-0.303,0.023,1.08],R,n=4,seconds=4)
print('tcp',np.round(a.tcp()[0],4)); print(np.round(a.tcp()[1],3)); print('fingers',a.fingers())
" 2>&1 | grep -v '^$' | tail -12 && timeout 120 python3 scene.py agentview save && timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import numpy as np
W=np.load('agentview_xyz.npy'); Z=W[:,:,2]
m=(W[:,:,0]>-0.45)&(W[:,:,0]<-0.15)&(W[:,:,1]>-0.1)&(W[:,:,1]<0.15)&(Z>0.95)&(Z<1.075)
p=W[m]; print('pts',len(p),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
# bottom-most points: where is the lowest region
lo=p[p[:,2]<p[:,2].min()+0.01]; print('lowest region center', np.round(lo.mean(0),3), len(lo))
"

# openrua op 63
timeout 600 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm(); R=hand_R(0)
q=a.ik([-0.303,0.023,1.08],R); cur=a.arm_q()
print('diff',np.round(np.array(q)-np.array(cur),3))
a.move_joints([q],[10])
p,Rn=a.tcp(); print('tcp',np.round(p,4)); print(np.round(Rn,3)); print('fingers',a.fingers())
" 2>&1 | tail -7 && timeout 120 python3 scene.py agentview save && timeout 120 python3 scene.py sideview save && python3 -c "
import numpy as np
for cam in ['agentview','sideview']:
    W=np.load(cam+'_xyz.npy'); Z=W[:,:,2]
    m=(W[:,:,0]>-0.45)&(W[:,:,0]<-0.2)&(W[:,:,1]>-0.06)&(W[:,:,1]<0.11)&(Z>0.99)&(Z<1.085)
    p=W[m]
    print(cam,'pts',len(p),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
    lo=p[p[:,2]<p[:,2].min()+0.008]; print('  lowest region center', np.round(lo.mean(0),3), len(lo))
    # rim: highest band by angle around estimated center
    c=p[:,:2].mean(0); d=np.hypot(p[:,0]-c[0],p[:,1]-c[1]); rim=p[(d>0.04)]
    ang=np.degrees(np.arctan2(rim[:,1]-c[1],rim[:,0]-c[0]))
    print('  center',np.round(c,3),[ (lo_, round(float(rim[(ang>=lo_)&(ang<lo_+90),2].max()),3)) for lo_ in range(-180,180,90) if ((ang>=lo_)&(ang<lo_+90)).sum()])
"

# openrua op 64
timeout 1200 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm(); R=hand_R(20)
tgt=[-0.148,-0.159,1.08]
q=a.ik(tgt,R); print('q',np.round(q,3))
a.move_joints([q],[10])
d=np.array(q)-np.array(a.arm_q())
if abs(d).max()>0.01: a.move_joints([q],[8])
p,Rn=a.tcp(); print('tcp',np.round(p,4)); print('R err',np.round(abs(Rn-R).max(),3)); print('fingers',a.fingers())
" 2>&1 | tail -6 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 65
timeout 120 python3 scene.py sideview save && timeout 120 python3 scene.py agentview save && python3 -c "
import numpy as np
for cam in ['sideview','agentview']:
    W=np.load(cam+'_xyz.npy'); Z=W[:,:,2]
    m=(W[:,:,0]>-0.26)&(W[:,:,0]<-0.02)&(W[:,:,1]>-0.24)&(W[:,:,1]<-0.06)&(Z>0.985)&(Z<1.09)
    p=W[m]
    print(cam,'pts',len(p),'x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
    lo=p[p[:,2]<p[:,2].min()+0.01]; print('  lowest', np.round(lo.mean(0),3), len(lo))
    hi=p[p[:,2]>p[:,2].max()-0.01]; print('  highest', np.round(hi.mean(0),3), 'y range %.3f..%.3f'%(hi[:,1].min(),hi[:,1].max()), len(hi))
    # drawer front wall top (z~0.98) y position and drawer floor
    w=W[(W[:,:,0]>-0.2)&(W[:,:,0]<0.0)&(Z>0.97)&(Z<0.99)&(W[:,:,1]>-0.12)&(W[:,:,1]<-0.05)]
    if len(w): print('  wall top y %.3f..%.3f'%(w[:,1].min(),w[:,1].max()))
"

# openrua op 66
python3 -c "
import numpy as np
W=np.concatenate([np.load('sideview_xyz.npy').reshape(-1,3),np.load('agentview_xyz.npy').reshape(-1,3)])
m=(W[:,0]>-0.26)&(W[:,0]<0.0)&(W[:,1]>-0.25)&(W[:,1]<-0.04)&(W[:,2]>0.92)&(W[:,2]<1.13)
p=W[m]
# elevation grid 1cm
xs=np.arange(-0.26,0.0,0.01); ys=np.arange(-0.25,-0.04,0.01)
G=np.full((len(ys),len(xs)),np.nan)
for i,y in enumerate(ys):
  for j,x in enumerate(xs):
    s=p[(p[:,0]>=x)&(p[:,0]<x+0.01)&(p[:,1]>=y)&(p[:,1]<y+0.01)]
    if len(s): G[i,j]=s[:,2].max()
np.set_printoptions(linewidth=250)
print('cols x from -0.26 step 0.01; rows y from -0.25')
for i,y in enumerate(ys):
    print('%6.2f '%y+' '.join('  . ' if np.isnan(v) else '%4d'%round((v-0.9)*1000) for v in G[i]))
"

# openrua op 67
timeout 1200 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm(); R=hand_R(20)
p0=a.tcp()[0]; tgt=np.array([-0.153,-0.150,0.982])
(code,err),qs=a.move_tcp_line(p0,tgt,R,n=4,seconds=6)
if err>0.01: a.move_joints([qs[-1]],[6])
p,Rn=a.tcp(); print('tcp',np.round(p,4),'R err',np.round(abs(Rn-R).max(),3)); print('fingers',a.fingers())
" 2>&1 | tail -4 && timeout 60 python3 tools/perception/cam_snap.py sideview && timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 68
timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>&1 | head -20; timeout 120 python3 scene.py frontview save && python3 -c "
import numpy as np
W=np.load('frontview_xyz.npy').reshape(-1,3)
m=(W[:,0]>-0.26)&(W[:,0]<0.0)&(W[:,1]>-0.25)&(W[:,1]<-0.04)&(W[:,2]>0.92)&(W[:,2]<1.13)
p=W[m]
xs=np.arange(-0.26,0.0,0.01); ys=np.arange(-0.25,-0.04,0.01)
print('cols x from -0.26 step 0.01; rows y from -0.25 (min z in cell, mm above table)')
for y in ys:
    row=[]
    for x in xs:
        s=p[(p[:,0]>=x)&(p[:,0]<x+0.01)&(p[:,1]>=y)&(p[:,1]<y+0.01)]
        row.append('  . ' if not len(s) else '%4d'%round((s[:,2].min()-0.9)*1000))
    print('%6.2f '%y+' '.join(row))
"

# openrua op 69
cat >> /workspace/arm.py <<'EOF'


def wrench(a, n=3):
    """Average external wrench force (panda_link0 frame) over n samples."""
    from geometry_msgs.msg import WrenchStamped
    got = []
    sub = a.node.create_subscription(
        WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
        lambda m: got.append([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z]), 10)
    t = time.time()
    while len(got) < n and time.time() - t < 10:
        rclpy.spin_once(a.node, timeout_sec=0.2)
    a.node.destroy_subscription(sub)
    return np.mean(got, axis=0) if got else None
EOF
timeout 1200 python3 -u -c "
import numpy as np
from arm import Arm, hand_R, wrench
a=Arm(); R=hand_R(20)
print('F before',np.round(wrench(a),2))
for z in [0.982,0.974,0.966]:
    p0=a.tcp()[0]; tgt=np.array([-0.150,-0.150,z])
    a.move_tcp_line(p0,tgt,R,n=2,seconds=4)
    p,Rn=a.tcp(); print('tcp',np.round(p,4),'F',np.round(wrench(a),2),'fingers',np.round(a.fingers(),4))
" 2>&1 | grep -v '^$' | tail -8

# openrua op 70
timeout 1200 python3 -u -c "
import numpy as np
from arm import Arm, hand_R, wrench
a=Arm(); R=hand_R(20)
a.gripper(0.04)
print('F',np.round(wrench(a),2))
p0=a.tcp()[0]
a.move_tcp_line(p0,[p0[0],p0[1],1.09],R,n=3,seconds=5)
print('tcp',np.round(a.tcp()[0],4))
# move away toward +x/+y so cameras see the drawer
a.move_tcp([-0.25,0.10,1.15],hand_R(0),seconds=8)
print('tcp',np.round(a.tcp()[0],4))
" 2>&1 | grep -v '^$' | tail -6 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 120 python3 scene.py agentview save && python3 -c "
import numpy as np
W=np.load('agentview_xyz.npy').reshape(-1,3)
m=(W[:,0]>-0.26)&(W[:,0]<0.03)&(W[:,1]>-0.25)&(W[:,1]<-0.04)&(W[:,2]>0.92)&(W[:,2]<1.13)
p=W[m]
xs=np.arange(-0.26,0.03,0.01); ys=np.arange(-0.25,-0.04,0.01)
for y in ys:
    row=[]
    for x in xs:
        s=p[(p[:,0]>=x)&(p[:,0]<x+0.01)&(p[:,1]>=y)&(p[:,1]<y+0.01)]
        row.append('  . ' if not len(s) else '%4d'%round((s[:,2].max()-0.9)*1000))
    print('%6.2f '%y+' '.join(row))
"

# openrua op 71
timeout 1500 python3 -u -c "
import numpy as np
from arm import Arm, hand_R, wrench
a=Arm(); R=hand_R(45)
a.gripper(0.0)
q=a.ik([-0.165,0.0,1.05],R); a.move_joints([q],[10])
if abs(np.array(q)-np.array(a.arm_q())).max()>0.01: a.move_joints([q],[8])
p,Rn=a.tcp(); print('tcp',np.round(p,4),'R err',np.round(abs(Rn-R).max(),3))
a.move_tcp_line(p,[-0.165,-0.03,0.94],R,n=3,seconds=5)
p,Rn=a.tcp(); print('tcp',np.round(p,4),'R err',np.round(abs(Rn-R).max(),3),'F',np.round(wrench(a),2))
for y1 in [-0.10,-0.16,-0.20,-0.228]:
    p0=a.tcp()[0]
    a.move_tcp_line(p0,[-0.165,y1,0.94],R,n=3,seconds=4)
    p,Rn=a.tcp(); print('tcp',np.round(p,4),'R err',np.round(abs(Rn-R).max(),3),'F',np.round(wrench(a),2))
" 2>&1 | grep -v '^$' | grep -v 'traj done' && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 72
timeout 1200 python3 -u -c "
import numpy as np
from arm import Arm, hand_R
a=Arm(); R=hand_R(45)
p0=a.tcp()[0]
a.move_tcp_line(p0,[-0.165,-0.05,0.98],R,n=2,seconds=4)
a.move_tcp([-0.25,0.15,1.20],hand_R(0),seconds=10)
q=a.ik([-0.25,0.15,1.20],hand_R(0))
if abs(np.array(q)-np.array(a.arm_q())).max()>0.01: a.move_joints([q],[8])
print('tcp',np.round(a.tcp()[0],4))
" 2>&1 | grep -v '^$' | tail -1 && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py frontview && timeout 120 python3 scene.py agentview save && timeout 120 python3 scene.py frontview save && python3 -c "
import numpy as np
for cam in ['agentview','frontview']:
    W=np.load(cam+'_xyz.npy').reshape(-1,3)
    # drawer front region: x -0.2..0.0, z 0.93..0.97 (below handles) -> y of the front face
    m=(W[:,0]>-0.2)&(W[:,0]<0.0)&(W[:,2]>0.93)&(W[:,2]<0.975)&(W[:,1]>-0.3)&(W[:,1]<0.0)
    p=W[m]; print(cam,'bottom-drawer front band pts',len(p),'y max %.3f  (median of top 5%% = %.3f)'%(p[:,1].max(), np.percentile(p[:,1],95)))
    m2=(W[:,0]>-0.2)&(W[:,0]<0.0)&(W[:,2]>1.03)&(W[:,2]<1.07)&(W[:,1]>-0.3)&(W[:,1]<0.0)
    p2=W[m2]; print('   upper drawer front band y 95pct %.3f'%np.percentile(p2[:,1],95))
    # anything on the table in front of cabinet? (bowl left outside?)
    m3=(W[:,0]>-0.3)&(W[:,0]<0.1)&(W[:,1]>-0.24)&(W[:,1]<0.3)&(W[:,2]>0.905)&(W[:,2]<1.12)
    print('   objects in front of cabinet (z>0.905):',m3.sum())
"

# openrua op 73
python3 -c "
import numpy as np
for cam in ['agentview','frontview']:
    W=np.load(cam+'_xyz.npy').reshape(-1,3)
    m=(W[:,0]>-0.2)&(W[:,0]<-0.145)&(W[:,2]>0.93)&(W[:,2]<0.975)&(W[:,1]>-0.3)&(W[:,1]<0.0)
    p=W[m]; print(cam,'bottom drawer face (x -0.2..-0.145) y 95pct %.3f max %.3f'%(np.percentile(p[:,1],95),p[:,1].max()))
    m=(W[:,0]>-0.03)&(W[:,0]<0.02)&(W[:,2]>0.93)&(W[:,2]<0.975)&(W[:,1]>-0.3)&(W[:,1]<0.0)
    p=W[m]; print('   face x -0.03..0.02 y 95pct %.3f'%np.percentile(p[:,1],95))
    m3=(W[:,0]>-0.3)&(W[:,0]<0.1)&(W[:,1]>-0.19)&(W[:,1]<0.3)&(W[:,2]>0.905)&(W[:,2]<1.0)
    p=W[m3]; print('   low objects in front of cabinet:',len(p), '' if not len(p) else 'x %.2f..%.2f y %.2f..%.2f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()))
"
