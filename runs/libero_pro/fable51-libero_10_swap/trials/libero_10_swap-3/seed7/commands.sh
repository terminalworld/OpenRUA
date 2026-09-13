#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools; ls -la /workspace/.ros

# openrua op 2
cat tools/action/ik_move.py tools/action/fjt_send.py tools/action/gripper_cmd.py

# openrua op 3
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | head -40; echo ---; timeout 20 ros2 node list

# openrua op 5
mkdir -p snaps && for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 6
timeout 60 python3 tools/perception/px2world.py agentview 352 213; timeout 60 python3 tools/perception/px2world.py birdview 335 255; timeout 60 python3 tools/perception/px2world.py frontview 347 315

# openrua op 7
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ---; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 8
cat > snaps/pts.py <<'EOF'
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
cam = sys.argv[1]; pts = [tuple(map(int,p.split(','))) for p in sys.argv[2:]]
rclpy.init(); node = rclpy.create_node("pts"); tfbuf=Buffer(); TransformListener(tfbuf,node)
def grab(topic, T):
    got={}; s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
depth=grab(f"/{cam}/depth/image_raw", Image); info=grab(f"/{cam}/color/camera_info", CameraInfo)
D=np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
frame=f"{cam}_optical_frame"
while not tfbuf.can_transform("world", frame, rclpy.time.Time()): rclpy.spin_once(node, timeout_sec=0.2)
t=tfbuf.lookup_transform("world", frame, rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.eye(4); T[:3,:3]=R; T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
for u,v in pts:
    zz=D[v,u]; p=T@np.array([(u-cx)*zz/fx,(v-cy)*zz/fy,zz,1.0])
    print(f"({u},{v}) d={zz:.3f} -> {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
np.save(f"snaps/{cam}_depth.npy", D)
print("cam pos", T[:3,3])
EOF
timeout 60 python3 snaps/pts.py agentview 352,213 352,190 352,240 320,213 385,213 450,300 250,380 200,280 220,260 180,200 250,210 100,160 130,210 300,300

# openrua op 9
cat > snaps/seg.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("seg"); tfbuf=Buffer(); TransformListener(tfbuf,node)
def grab(topic, T):
    got={}; s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
depth=grab(f"/{cam}/depth/image_raw", Image); info=grab(f"/{cam}/color/camera_info", CameraInfo)
D=np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
frame=f"{cam}_optical_frame"
while not tfbuf.can_transform("world", frame, rclpy.time.Time()): rclpy.spin_once(node, timeout_sec=0.2)
t=tfbuf.lookup_transform("world", frame, rclpy.time.Time()); q=t.transform.rotation
x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.eye(4); T[:3,:3]=R; T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=D.shape; vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D,np.ones_like(D)],-1).reshape(-1,4)@T.T
P=P[:,:3].reshape(H,W,3)
np.save(f"snaps/{cam}_world.npy", P)
print("cam pos", T[:3,3], "R", R.round(3).tolist())
# report by ranges
zs=P[...,2]
print("z hist", np.histogram(zs[np.isfinite(zs)], bins=20)[0], np.histogram(zs[np.isfinite(zs)], bins=20)[1].round(3))
EOF
timeout 60 python3 snaps/seg.py birdview

# openrua op 10
cat > snaps/an.py <<'EOF'
import numpy as np
P=np.load("snaps/birdview_world.npy")
X,Y,Z=P[...,0],P[...,1],P[...,2]
# bowl region: around (-0.18,0.05); mask z>0.91 within a 0.15 radius
r=np.hypot(X+0.17,Y-0.05)
for name,m in [("bowl", (r<0.12)&(Z>0.905)&(Z<1.1))]:
    print(name, m.sum(), "x",X[m].min().round(3),X[m].max().round(3),"y",Y[m].min().round(3),Y[m].max().round(3),"z",Z[m].min().round(3),Z[m].max().round(3), "centroid", X[m].mean().round(4), Y[m].mean().round(4))
    # rim: highest points
    top=m&(Z>Z[m].max()-0.01)
    print(" rim centroid", X[top].mean().round(4), Y[top].mean().round(4), "n",top.sum())
# table
m=(Z>0.89)&(Z<0.905)&(np.abs(X)<0.6)&(np.abs(Y)<0.6)
print("table z", Z[m].mean().round(4), m.sum())
# cabinet region: y<-0.1, x in -0.35..0.15
m=(Y<-0.1)&(Y>-0.5)&(X>-0.4)&(X<0.2)&(Z>0.91)
print("cabinet pts", m.sum(), "x",X[m].min().round(3),X[m].max().round(3),"y",Y[m].min().round(3),Y[m].max().round(3),"z",Z[m].min().round(3),Z[m].max().round(3))
for zlo,zhi in [(0.91,0.95),(0.95,1.0),(1.0,1.05),(1.05,1.1),(1.1,1.15),(1.15,1.2)]:
    mm=m&(Z>=zlo)&(Z<zhi)
    if mm.sum(): print(f" z{zlo}-{zhi}: n={mm.sum()} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
EOF
python3 snaps/an.py

# openrua op 11
timeout 60 python3 snaps/seg.py agentview && cat > snaps/an2.py <<'EOF'
import numpy as np, cv2
P=np.load("snaps/agentview_world.npy")
img=cv2.imread("snaps/agentview.png")
X,Y,Z=P[...,0],P[...,1],P[...,2]
r=np.hypot(X+0.17,Y-0.05)
m=(r<0.13)&(Z>0.905)&(Z<1.1)
print("bowl", m.sum(), "x",X[m].min().round(3),X[m].max().round(3),"y",Y[m].min().round(3),Y[m].max().round(3),"z",Z[m].min().round(3),Z[m].max().round(3), "centroid", X[m].mean().round(4), Y[m].mean().round(4))
top=m&(Z>Z[m].max()-0.008)
print(" rim", X[top].min().round(3),X[top].max().round(3),Y[top].min().round(3),Y[top].max().round(3), "centroid", X[top].mean().round(4), Y[top].mean().round(4), top.sum())
vis=img.copy(); vis[m]=(0,0,255); cv2.imwrite("snaps/bowlmask.png", vis)
# drawer interior: points with z in 0.915..0.94, y<-0.1
m=(Y<-0.1)&(Y>-0.5)&(X>-0.4)&(X<0.2)&(Z>0.91)&(Z<0.945)
print("drawer floor", m.sum(), "x",X[m].min().round(3),X[m].max().round(3),"y",Y[m].min().round(3),Y[m].max().round(3),"z",Z[m].mean().round(4))
for zlo,zhi in [(0.945,1.0),(1.0,1.05),(1.05,1.1),(1.1,1.15),(1.15,1.25)]:
    mm=(Y<-0.05)&(Y>-0.5)&(X>-0.4)&(X<0.2)&(Z>=zlo)&(Z<zhi)
    if mm.sum(): print(f" z{zlo}-{zhi}: n={mm.sum()} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
EOF
python3 snaps/an2.py

# openrua op 12
timeout 60 python3 snaps/seg.py sideview && timeout 60 python3 snaps/seg.py frontview && cat > snaps/an3.py <<'EOF'
import numpy as np, cv2
for cam in ["sideview","frontview","birdview"]:
    P=np.load(f"snaps/{cam}_world.npy"); img=cv2.imread(f"snaps/{cam}.png")
    X,Y,Z=P[...,0],P[...,1],P[...,2]
    r=np.hypot(X+0.16,Y-0.04)
    m=(r<0.14)&(Z>0.905)&(Z<1.1)&np.isfinite(Z)
    if m.sum()==0: print(cam,"none"); continue
    print(cam, m.sum(), "x",X[m].min().round(3),X[m].max().round(3),"y",Y[m].min().round(3),Y[m].max().round(3),"z",Z[m].min().round(3),Z[m].max().round(3), "centroid", X[m].mean().round(4), Y[m].mean().round(4))
    top=m&(Z>Z[m].max()-0.008)
    print(" rim", X[top].min().round(3),X[top].max().round(3),Y[top].min().round(3),Y[top].max().round(3), "centroid", X[top].mean().round(4), Y[top].mean().round(4), top.sum())
    vis=img.copy(); vis[m]=(0,0,255); cv2.imwrite(f"snaps/{cam}_bowlmask.png", vis)
EOF
python3 snaps/an3.py

# openrua op 13
cat > snaps/an4.py <<'EOF'
import numpy as np
for cam in ["frontview","agentview"]:
    P=np.load(f"snaps/{cam}_world.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(np.hypot(X+0.143,Y-0.058)<0.09)&(Z>0.905)&(Z<1.1)
    print(cam)
    for zlo in np.arange(0.90,1.03,0.01):
        mm=m&(Z>=zlo)&(Z<zlo+0.01)
        if mm.sum(): print(f"  z{zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}] r={np.hypot(X[mm]+0.143,Y[mm]-0.058).mean():.3f}")
EOF
python3 snaps/an4.py

# openrua op 14
cat > snaps/an5.py <<'EOF'
import numpy as np
for cam in ["agentview","birdview","frontview"]:
    P=np.load(f"snaps/{cam}_world.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(Y<-0.05)&(Y>-0.5)&(X>-0.4)&(X<0.15)&(Z>0.905)&(Z<1.3)&np.isfinite(Z)
    print(cam)
    for ylo in np.arange(-0.45,-0.05,0.02):
        mm=m&(Y>=ylo)&(Y<ylo+0.02)
        if mm.sum()>5: print(f"  y{ylo:+.2f}: n={mm.sum():5d} x[{X[mm].min():.3f},{X[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]")
EOF
python3 snaps/an5.py

# openrua op 15
cat > snaps/an6.py <<'EOF'
import numpy as np
P=np.load("snaps/agentview_world.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
# tall stuff not cabinet body
m=(Z>1.05)&(Y>-0.22)&(Y<0.3)&(X>-0.4)&(X<0.4)&np.isfinite(Z)
print("tall pts", m.sum(), "x",X[m].min().round(3),X[m].max().round(3),"y",Y[m].min().round(3),Y[m].max().round(3),"z",Z[m].max().round(3))
for xlo in np.arange(-0.4,0.4,0.05):
    mm=m&(X>=xlo)&(X<xlo+0.05)
    if mm.sum()>5: print(f"  x{xlo:+.2f}: n={mm.sum():5d} y[{Y[mm].min():.3f},{Y[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]")
# drawer front panel: y in [-0.13,-0.06], z 0.92..1.05, by x
m=(Y>-0.13)&(Y<-0.06)&(Z>0.905)&(Z<1.05)
print("front panel region")
for xlo in np.arange(-0.3,0.1,0.02):
    mm=m&(X>=xlo)&(X<xlo+0.02)
    if mm.sum()>5: print(f"  x{xlo:+.2f}: n={mm.sum():5d} y[{Y[mm].min():.3f},{Y[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]")
EOF
python3 snaps/an6.py

# openrua op 16
cat > snaps/an7.py <<'EOF'
import numpy as np
for cam in ["agentview","frontview"]:
    P=np.load(f"snaps/{cam}_world.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(X>-0.05)&(X<0.2)&(Y>-0.2)&(Y<0.0)&(Z>0.905)&np.isfinite(Z)
    print(cam, m.sum())
    for zlo in np.arange(0.9,1.3,0.05):
        mm=m&(Z>=zlo)&(Z<zlo+0.05)
        if mm.sum()>5: print(f"  z{zlo:.2f}: n={mm.sum():5d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}] cx={X[mm].mean():.3f} cy={Y[mm].mean():.3f}")
EOF
python3 snaps/an7.py

# openrua op 17
cd snaps && python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[270:360,290:400]; cv2.imwrite('front_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); crop=im[170:260,290:410]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 18
cd snaps && python3 -c "
import numpy as np
P=np.load('frontview_world.npy')
for v in range(296,345,2):
    x,y,z=P[v,347]; print(v, round(float(x),3), round(float(y),3), round(float(z),3))
"

# openrua op 19
python3 -c "
import numpy as np
P=np.load('frontview_world.npy')
for v in range(296,345,2):
    x,y,z=P[v,347]; print(v, round(float(x),3), round(float(y),3), round(float(z),3))
"

# openrua op 20
python3 -c "
import numpy as np
P=np.load('sideview_world.npy')
for v in range(250,300,2):
    x,y,z=P[v,345]; print(v, round(float(x),3), round(float(y),3), round(float(z),3))
"

# openrua op 21
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_depth.png >/dev/null; python3 -c "
import numpy as np
D=np.load('snaps/eih_depth.npy'); print(D.shape)
for (u,v) in [(245,205),(150,300),(100,250),(245,140),(245,270),(180,205),(310,205),(300,330),(500,180),(550,120)]:
    print((u,v), round(float(D[v,u]),4))
"; timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A9 "^k:"

# openrua op 22
cd /workspace && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw snaps/eih_depth.png >/dev/null; python3 -c "
import numpy as np
D=np.load('snaps/eih_depth.npy'); print(D.shape)
for (u,v) in [(245,205),(150,300),(100,250),(245,140),(245,270),(180,205),(310,205),(300,330),(500,180),(550,120),(430,150),(470,250)]:
    print((u,v), round(float(D[v,u]),4))
"

# openrua op 23
python3 -c "
import numpy as np
P=np.load('snaps/agentview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
# points on the front face plane of the cabinet
m=(Y>-0.24)&(Y<-0.215)&(X>-0.25)&(X<0.05)&np.isfinite(Z)
print('front face pts', m.sum())
for zlo in np.arange(0.90,1.14,0.01):
    mm=m&(Z>=zlo)&(Z<zlo+0.01)
    print(f'  z{zlo:.2f}: n={mm.sum():4d}', (f'x[{X[mm].min():.3f},{X[mm].max():.3f}]' if mm.sum() else ''))
# points deeper inside cabinet (y<-0.24) at x in drawer range: visible interior => opening
m2=(Y<-0.24)&(Y>-0.42)&(X>-0.21)&(X<0.0)&np.isfinite(Z)&(Z>0.905)
print('interior pts', m2.sum())
for zlo in np.arange(0.90,1.14,0.01):
    mm=m2&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum(): print(f'  z{zlo:.2f}: n={mm.sum():4d} y[{Y[mm].min():.3f},{Y[mm].max():.3f}]')
"

# openrua op 24
python3 -c "
import numpy as np, cv2
P=np.load('snaps/sideview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(Y>-0.245)&(Y<-0.215)&(X>-0.25)&(X<0.05)&np.isfinite(Z)
print('front face pts', m.sum())
for zlo in np.arange(0.90,1.14,0.01):
    mm=m&(Z>=zlo)&(Z<zlo+0.01)
    print(f'  z{zlo:.2f}: n={mm.sum():4d}', (f'x[{X[mm].min():.3f},{X[mm].max():.3f}]' if mm.sum() else ''))
m2=(Y<-0.245)&(Y>-0.42)&(X>-0.23)&(X<0.02)&np.isfinite(Z)&(Z>0.905)
print('interior pts', m2.sum())
for zlo in np.arange(0.90,1.14,0.01):
    mm=m2&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum(): print(f'  z{zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]')
img=cv2.imread('snaps/sideview.png'); vis=img.copy(); vis[m]=(0,0,255); vis[m2]=(0,255,0); cv2.imwrite('snaps/side_cab.png', vis)
"

# openrua op 25
time timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854 2.0 && timeout 60 python3 tools/perception/cam_snap.py frontview snaps/frontview2.png && timeout 60 python3 snaps/seg.py frontview >/dev/null && python3 -c "
import numpy as np
P=np.load('snaps/frontview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(np.hypot(X+0.143,Y-0.058)<0.09)&(Z>0.905)&(Z<1.1)
print('bowl', m.sum(), 'z',Z[m].min().round(3),Z[m].max().round(3))
top=m&(Z>Z[m].max()-0.008); print(' rim centroid', X[top].mean().round(4), Y[top].mean().round(4), 'x',X[top].min().round(3),X[top].max().round(3),'y',Y[top].min().round(3),Y[top].max().round(3))
"

# openrua op 26
timeout 60 python3 snaps/seg.py agentview >/dev/null && timeout 60 python3 snaps/seg.py sideview > /dev/null && python3 -c "
import numpy as np
for cam in ['agentview','sideview']:
    P=np.load(f'snaps/{cam}_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
    print(cam)
    m=(Y<-0.05)&(Y>-0.5)&(X>-0.4)&(X<0.15)&(Z>0.905)&(Z<1.3)&np.isfinite(Z)
    for ylo in np.arange(-0.27,-0.05,0.02):
        mm=m&(Y>=ylo)&(Y<ylo+0.02)
        if mm.sum()>5: print(f'  y{ylo:+.2f}: n={mm.sum():5d} x[{X[mm].min():.3f},{X[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]')
    m=(X>0.0)&(X<0.2)&(Y>-0.2)&(Y<0.0)&(Z>0.905)&np.isfinite(Z)
    print('  bottle', m.sum(), 'x',X[m].min().round(3),X[m].max().round(3),'y',Y[m].min().round(3),Y[m].max().round(3),'ztop',Z[m].max().round(3))
    m=(Y>-0.245)&(Y<-0.215)&(X>-0.25)&(X<0.05)&np.isfinite(Z)
    for zlo in np.arange(0.96,1.06,0.01):
        mm=m&(Z>=zlo)&(Z<zlo+0.01)
        print(f'  face z{zlo:.2f}: n={mm.sum():4d}', (f'x[{X[mm].min():.3f},{X[mm].max():.3f}]' if mm.sum() else ''))
"

# openrua op 27
timeout 120 python3 tools/action/gripper_cmd.py 0.04 && time timeout 600 python3 tools/action/ik_move.py -0.20 0.059 1.00 0.7071 0.7071 0 0 4 --at tcp && timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position"

# openrua op 28
mkdir -p "$(dirname /workspace/mv.py)"
cat > /workspace/mv.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Move the TCP (fingertip midpoint) to a WORLD-frame pose via IK + trajectory.

Usage: python3 mv.py <x> <y> <z> <qx> <qy> <qz> <qw> [seconds]
       python3 mv.py fk          # just print the current TCP pose in world
World -> base: base is at world (-0.66, 0, 0.912) (from TF).
"""
import sys
import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE = np.array([-0.66, 0.0, 0.912])
TCP = 0.1034
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def joint_state(node):
    js = {}
    sub = node.create_subscription(JointState, "/joint_states",
                                   lambda m: js.setdefault("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return dict(zip(js["m"].name, js["m"].position))


def arm_seed(jsd):
    s = JointState()
    for j in JOINTS:
        s.name.append(j); s.position.append(jsd[j])
    return s


def fk_world(node, jsd):
    cli = node.create_client(GetPositionFK, "/compute_fk")
    cli.wait_for_service(timeout_sec=10)
    req = GetPositionFK.Request()
    req.fk_link_names = ["panda_hand"]
    req.robot_state.joint_state = arm_seed(jsd)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    res = fut.result()
    p = res.pose_stamped[0].pose
    q = p.orientation
    R = quat_R(q.x, q.y, q.z, q.w)
    hand = np.array([p.position.x, p.position.y, p.position.z]) + BASE
    tcp = hand + TCP * R[:, 2]
    return hand, tcp, (q.x, q.y, q.z, q.w)


def main():
    rclpy.init()
    node = rclpy.create_node("mv")
    jsd = joint_state(node)
    if sys.argv[1] == "fk":
        hand, tcp, q = fk_world(node, jsd)
        print("hand", hand.round(4), "tcp", tcp.round(4), "q", np.round(q, 4))
        print("fingers", round(jsd["panda_finger_joint1"], 4), round(jsd["panda_finger_joint2"], 4))
        return
    x, y, z, qx, qy, qz, qw = map(float, sys.argv[1:8])
    secs = float(sys.argv[8]) if len(sys.argv) > 8 else 3.0
    R = quat_R(qx, qy, qz, qw)
    hand_w = np.array([x, y, z]) - TCP * R[:, 2]
    hand_b = hand_w - BASE
    cli = node.create_client(GetPositionIK, "/compute_ik")
    cli.wait_for_service(timeout_sec=10)
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = hand_b
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    req.ik_request.robot_state.joint_state = arm_seed(jsd)
    req.ik_request.avoid_collisions = False
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
    res = fut.result()
    if res is None or res.error_code.val != 1:
        raise SystemExit(f"IK FAILED code={None if res is None else res.error_code.val}; no motion")
    sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    target = [sol[j] for j in JOINTS]
    print("target joints", np.round(target, 3))
    client = ActionClient(node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
    client.wait_for_server(timeout_sec=10)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINTS
    pt = JointTrajectoryPoint(positions=target)
    pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
    goal.trajectory.points = [pt]
    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    result = send.result().get_result_async()
    rclpy.spin_until_future_complete(node, result)
    print("traj error_code", result.result().result.error_code)
    jsd = joint_state(node)
    err = max(abs(jsd[j] - t) for j, t in zip(JOINTS, target))
    hand, tcp, q = fk_world(node, jsd)
    print("max joint err", round(err, 4))
    print("hand", hand.round(4), "tcp", tcp.round(4), "q", np.round(q, 4))
    print("fingers", round(jsd["panda_finger_joint1"], 4), round(jsd["panda_finger_joint2"], 4))


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 29
timeout 120 python3 mv.py fk

# openrua op 30
cat > /workspace/fktest.py <<'EOF'
import rclpy, numpy as np
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from sensor_msgs.msg import JointState
rclpy.init(); node=rclpy.create_node("t")
js={}
sub=node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
J=[f"panda_joint{i}" for i in range(1,8)]
d=dict(zip(js["m"].name, js["m"].position))
seed=JointState(); seed.name=J; seed.position=[d[j] for j in J]
cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=["panda_link0","panda_hand"]; req.robot_state.joint_state=seed
fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut)
for ps in fut.result().pose_stamped:
    print(ps.header.frame_id, ps.pose.position)
# IK test with the current hand pose as given by FK
hp=fut.result().pose_stamped[1].pose
ik=node.create_client(GetPositionIK,"/compute_ik"); ik.wait_for_service()
for label,off in [("as-is",0.0),("minus base",1)]:
    r=GetPositionIK.Request(); r.ik_request.group_name="panda_arm"; r.ik_request.pose_stamped.header.frame_id=""
    r.ik_request.pose_stamped.pose=hp
    if off:
        import copy; p=copy.deepcopy(hp); p.position.x-= -0.66; p.position.z-=0.912; r.ik_request.pose_stamped.pose=p
    r.ik_request.robot_state.joint_state=seed; r.ik_request.avoid_collisions=False
    f=ik.call_async(r); rclpy.spin_until_future_complete(node,f,timeout_sec=60); res=f.result()
    print(label, "code", res.error_code.val, np.round([dict(zip(res.solution.joint_state.name,res.solution.joint_state.position)).get(j,0) for j in J],3) if res.error_code.val==1 else "")
print("current", np.round(seed.position,3))
EOF
timeout 120 python3 /workspace/fktest.py

# openrua op 31
python3 - <<'EOF'
s=open("mv.py").read()
s=s.replace("BASE = np.array([-0.66, 0.0, 0.912])","BASE = np.array([0.0, 0.0, 0.0])  # FK/IK model frame == world on this machine")
s=s.replace("""    target = [sol[j] for j in JOINTS]
    print("target joints", np.round(target, 3))
""","""    target = [sol[j] for j in JOINTS]
    print("target joints", np.round(target, 3))
    # verify the IK solution actually reaches the requested pose
    hand_s, tcp_s, q_s = fk_world(node, sol)
    perr = np.linalg.norm(tcp_s - np.array([x, y, z]))
    Rs = quat_R(*q_s)
    aerr = np.degrees(np.arccos(np.clip((np.trace(Rs.T @ R) - 1) / 2, -1, 1)))
    print(f"IK check: tcp {tcp_s.round(4)} pos_err={perr*1000:.1f}mm ang_err={aerr:.2f}deg")
    if perr > 0.005 or aerr > 3:
        raise SystemExit("IK solution does not match target; no motion")
""")
open("mv.py","w").write(s)
EOF
timeout 120 python3 mv.py fk

# openrua op 32
time timeout 600 python3 mv.py -0.20 0.059 1.00 0.7071 0.7071 0 0 4

# openrua op 33
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/mv.py", "old_string": "    req.ik_request.group_name = \\"panda_arm\\"\\n    req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "    req.ik_request.group_name = \\"panda_arm\\"\\n    req.ik_request.ik_link_name = \\"panda_hand\\"  # default tip is link8 (45deg off)\\n    req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 34
time timeout 600 python3 mv.py -0.20 0.059 1.00 0.7071 0.7071 0 0 4

# openrua op 35
timeout 300 python3 tools/action/fjt_send.py 0.153,0.231,-0.024,-2.518,0.014,2.749,-0.668 3 && timeout 60 python3 mv.py fk

# openrua op 36
timeout 300 python3 tools/action/fjt_send.py 0.153,0.231,-0.024,-2.518,0.014,2.749,-0.668 3 && timeout 60 python3 mv.py fk && timeout 20 ros2 topic echo /joint_states --once | grep -A8 "^position"

# openrua op 37
python3 - <<'EOF'
s=open("mv.py").read()
s=s.replace("""    send = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send)
    result = send.result().get_result_async()
    rclpy.spin_until_future_complete(node, result)
    print("traj error_code", result.result().result.error_code)
    jsd = joint_state(node)
    err = max(abs(jsd[j] - t) for j, t in zip(JOINTS, target))
""","""    for attempt in range(3):
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, send)
        result = send.result().get_result_async()
        rclpy.spin_until_future_complete(node, result)
        print("traj error_code", result.result().result.error_code)
        jsd = joint_state(node)
        err = max(abs(jsd[j] - t) for j, t in zip(JOINTS, target))
        if err < 0.02:
            break
        print(f"joint err {err:.3f}, resending")
""")
open("mv.py","w").write(s)
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png

# openrua op 38
timeout 60 python3 snaps/seg.py agentview >/dev/null && python3 -c "
import numpy as np
P=np.load('snaps/agentview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
# fingers: points around z 0.99..1.03 near the hand
m=(Z>0.985)&(Z<1.03)&(np.abs(Y-0.059)<0.06)&(X>-0.3)&(X<-0.1)
print('finger pts', m.sum())
for xlo in np.arange(-0.27,-0.13,0.01):
    mm=m&(X>=xlo)&(X<xlo+0.01)
    if mm.sum(): print(f'  x{xlo:.2f}: n={mm.sum():3d} y[{Y[mm].min():.3f},{Y[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]')
m=(np.hypot(X+0.15,Y-0.059)<0.09)&(Z>0.905)&(Z<0.98)
top=m&(Z>Z[m].max()-0.008); print('rim', X[top].min().round(3),X[top].max().round(3),Y[top].min().round(3),Y[top].max().round(3),'z',Z[m].max().round(3))
"

# openrua op 39
timeout 600 python3 mv.py -0.20 0.059 0.933 0.7071 0.7071 0 0 2 && timeout 120 python3 tools/action/gripper_cmd.py 0.0 && timeout 60 python3 mv.py fk

# openrua op 40
timeout 600 python3 mv.py -0.20 0.059 1.13 0.7071 0.7071 0 0 3 && timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_lift.png

# openrua op 41
timeout 60 python3 snaps/seg.py agentview >/dev/null && timeout 60 python3 snaps/seg.py frontview >/dev/null && python3 -c "
import numpy as np
for cam in ['agentview','frontview']:
    P=np.load(f'snaps/{cam}_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(np.hypot(X+0.15,Y-0.059)<0.09)&(Z>1.05)&(Z<1.22)&np.isfinite(Z)
    print(cam, m.sum(), 'z',Z[m].min().round(3),Z[m].max().round(3))
    for zlo in np.arange(1.05,1.22,0.01):
        mm=m&(Z>=zlo)&(Z<zlo+0.01)
        if mm.sum(): print(f'  z{zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]')
"

# openrua op 42
python3 -c "
import cv2
im=cv2.imread('snaps/front_lift.png'); crop=im[180:300,280:420]; cv2.imwrite('snaps/lift_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 43
timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side2.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('snaps/side2.png'); crop=im[170:310,250:400]; cv2.imwrite('snaps/side_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 44
timeout 60 python3 snaps/seg.py sideview >/dev/null && python3 -c "
import numpy as np
P=np.load('snaps/sideview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.16)&(X<-0.03)&(Y<-0.20)&(Y>-0.30)&np.isfinite(Z)
for zlo in np.arange(0.95,1.08,0.005):
    mm=m&(Z>=zlo)&(Z<zlo+0.005)
    if mm.sum(): print(f'  z{zlo:.3f}: n={mm.sum():4d} y[{Y[mm].min():.3f},{Y[mm].max():.3f}] x[{X[mm].min():.3f},{X[mm].max():.3f}]')
# drawer front panel top & inner face
m=(X>-0.16)&(X<-0.03)&(Y<-0.06)&(Y>-0.20)&np.isfinite(Z)&(Z>0.95)
print('panel')
for zlo in np.arange(0.95,1.0,0.005):
    mm=m&(Z>=zlo)&(Z<zlo+0.005)
    if mm.sum(): print(f'  z{zlo:.3f}: n={mm.sum():4d} y[{Y[mm].min():.3f},{Y[mm].max():.3f}]')
"

# openrua op 45
timeout 60 python3 snaps/seg.py birdview >/dev/null && python3 -c "
import numpy as np
P=np.load('snaps/birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.16)&(X<-0.03)&(Y<-0.05)&(Y>-0.30)&np.isfinite(Z)&(Z>0.905)
for ylo in np.arange(-0.27,-0.05,0.005):
    mm=m&(Y>=ylo)&(Y<ylo+0.005)
    if mm.sum(): print(f'  y{ylo:+.3f}: n={mm.sum():4d} z[{Z[mm].min():.3f},{Z[mm].max():.3f}]')
"

# openrua op 46
timeout 60 python3 snaps/seg.py agentview >/dev/null && python3 -c "
import numpy as np
P=np.load('snaps/agentview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.32)&(X<-0.08)&(Y>-0.1)&(Y<0.2)&np.isfinite(Z)
for zlo in np.arange(1.13,1.36,0.01):
    mm=m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum(): print(f'  z{zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]')
"
python3 -c "
import numpy as np
P=np.load('snaps/birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.32)&(X<-0.08)&(Y>-0.1)&(Y<0.2)&np.isfinite(Z)&(Z>1.1)
print('birdview hand top-down: ')
for ylo in np.arange(-0.02,0.14,0.01):
    mm=m&(Y>=ylo)&(Y<ylo+0.01)
    if mm.sum(): print(f'  y{ylo:+.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]')
# top drawer handle
m=(X>-0.2)&(X<0.0)&(Y>-0.23)&(Y<-0.19)&np.isfinite(Z)&(Z>1.0)&(Z<1.12)
print('top handle', m.sum(), 'x',X[m].min().round(3),X[m].max().round(3),'y',Y[m].min().round(3),Y[m].max().round(3),'z',Z[m].min().round(3),Z[m].max().round(3))
"

# openrua op 47
timeout 600 python3 mv.py -0.177 -0.133 1.15 0.7071 0.7071 0 0 4 && timeout 600 python3 mv.py -0.177 -0.133 1.03 0.7071 0.7071 0 0 3 && timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_B.png && timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side_B.png

# openrua op 48
python3 -c "
import cv2
im=cv2.imread('snaps/side_B.png'); crop=im[160:300,250:420]; cv2.imwrite('snaps/sideB_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('snaps/front_B.png'); crop=im[200:340,150:350]; cv2.imwrite('snaps/frontB_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 49
timeout 600 python3 mv.py -0.177 -0.165 1.03 0.7071 0.7071 0 0 2 && timeout 600 python3 mv.py -0.177 -0.165 1.005 0.7071 0.7071 0 0 2 && timeout 60 python3 snaps/seg.py birdview >/dev/null && python3 -c "
import numpy as np
P=np.load('snaps/birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.25)&(X<0.02)&(Y>-0.26)&(Y<-0.05)&np.isfinite(Z)&(Z>0.93)&(Z<1.06)
print('bowl-ish pts in drawer area')
for zlo in np.arange(0.93,1.06,0.01):
    mm=m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum(): print(f'  z{zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]')
"

# openrua op 50
timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_D.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_D.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('snaps/front_D.png'); crop=im[200:340,150:350]; cv2.imwrite('snaps/frontD_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('snaps/agent_D.png'); crop=im[150:300,100:320]; cv2.imwrite('snaps/agentD_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 51
timeout 120 python3 tools/action/gripper_cmd.py 0.04 && timeout 600 python3 mv.py -0.177 -0.165 1.16 0.7071 0.7071 0 0 3 && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_E.png >/dev/null && timeout 60 python3 snaps/seg.py birdview >/dev/null && python3 -c "
import numpy as np
P=np.load('snaps/birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.25)&(X<0.02)&(Y>-0.26)&(Y<-0.05)&np.isfinite(Z)&(Z>0.93)&(Z<1.02)
for zlo in np.arange(0.93,1.02,0.005):
    mm=m&(Z>=zlo)&(Z<zlo+0.005)
    if mm.sum(): print(f'  z{zlo:.3f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]')
"

# openrua op 52
python3 -c "
import numpy as np
def R(x,y,z,w): return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
for q in [(-0.6533,0.6533,-0.2706,-0.2706),(0.6533,0.6533,-0.2706,0.2706)]:
    q=np.array(q); q/=np.linalg.norm(q); M=R(*q); print(q.round(4)); print(' hand x',M[:,0].round(3),' hand y (fingers)',M[:,1].round(3),' hand z (approach)',M[:,2].round(3))
"

# openrua op 53
timeout 120 python3 tools/action/gripper_cmd.py 0.0 && timeout 900 python3 mv.py -0.19 -0.02 1.05 -0.6533 0.6533 -0.2706 -0.2706 5

# openrua op 54
timeout 900 python3 mv.py -0.19 -0.02 0.955 -0.6533 0.6533 -0.2706 -0.2706 3 && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_F.png

# openrua op 55
cat > /workspace/drawer.py <<'EOF'
import numpy as np, subprocess
subprocess.run(["python3","snaps/seg.py","birdview"],capture_output=True,timeout=120)
P=np.load('snaps/birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.08)&(X<-0.01)&(Y>-0.30)&(Y<0.0)&np.isfinite(Z)&(Z>0.975)&(Z<0.995)
print('panel top pts', m.sum(), 'y range', Y[m].min().round(3) if m.sum() else None, Y[m].max().round(3) if m.sum() else None)
m2=(X>-0.08)&(X<-0.01)&(Y>-0.30)&(Y<0.0)&np.isfinite(Z)&(Z>0.915)&(Z<0.93)
print('drawer floor visible y range', Y[m2].min().round(3) if m2.sum() else None, Y[m2].max().round(3) if m2.sum() else None)
m3=(X>-0.20)&(X<-0.05)&(Y>-0.40)&(Y<-0.09)&np.isfinite(Z)&(Z>0.94)&(Z<0.98)
print('bowl-ish pts', m3.sum(), 'x',X[m3].min().round(3) if m3.sum() else None, X[m3].max().round(3) if m3.sum() else None,'y',Y[m3].min().round(3) if m3.sum() else None,Y[m3].max().round(3) if m3.sum() else None)
EOF
timeout 200 python3 /workspace/drawer.py

# openrua op 56
timeout 900 python3 mv.py -0.19 -0.10 0.955 -0.6533 0.6533 -0.2706 -0.2706 2 && timeout 200 python3 /workspace/drawer.py

# openrua op 57
timeout 900 python3 mv.py -0.19 -0.16 0.955 -0.6533 0.6533 -0.2706 -0.2706 2 && timeout 200 python3 /workspace/drawer.py && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 58
timeout 900 python3 mv.py -0.19 -0.215 0.955 -0.6533 0.6533 -0.2706 -0.2706 2 && timeout 200 python3 /workspace/drawer.py && timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 59
timeout 900 python3 mv.py -0.19 -0.223 0.955 -0.6533 0.6533 -0.2706 -0.2706 2; timeout 200 python3 /workspace/drawer.py; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_G.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side_G.png

# openrua op 60
timeout 900 python3 mv.py -0.19 -0.05 1.10 -0.6533 0.6533 -0.2706 -0.2706 3 && timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side_final.png && timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_final.png && timeout 200 python3 /workspace/drawer.py && python3 -c "
import numpy as np
P=np.load('snaps/birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
# anything bowl-like left outside the cabinet on the table?
m=(Z>0.93)&(Z<1.0)&(Y>-0.22)&(Y<0.5)&(X>-0.45)&(X<0.4)&np.isfinite(Z)
print('objects on table (z .93-1.0, outside cabinet):', m.sum())
for xlo in np.arange(-0.45,0.4,0.05):
    mm=m&(X>=xlo)&(X<xlo+0.05)
    if mm.sum(): print(f'  x{xlo:+.2f}: n={mm.sum():4d} y[{Y[mm].min():.3f},{Y[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]')
"

# openrua op 61
timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position"; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_H.png

# openrua op 62
python3 - <<'EOF'
s=open("mv.py").read()
s=s.replace("""    if perr > 0.005 or aerr > 3:
        raise SystemExit("IK solution does not match target; no motion")
""","""    if perr > 0.005 or aerr > 3:
        raise SystemExit("IK solution does not match target; no motion")
    import os
    maxd = float(os.environ.get("MAXDELTA", "99"))
    delta = max(abs(sol[j] - jsd[j]) for j in JOINTS)
    print(f"max joint delta from current: {delta:.3f}")
    if delta > maxd:
        raise SystemExit(f"IK solution reconfigures the arm (delta {delta:.2f} > {maxd}); no motion")
""")
open("mv.py","w").write(s)
EOF
MAXDELTA=0.6 timeout 600 python3 mv.py -0.19 -0.05 1.25 0.2588 0.8866 -0.368 0.1073 3

# openrua op 63
cat > /workspace/pathcheck.py <<'EOF'
import sys, numpy as np, rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
J=[f"panda_joint{i}" for i in range(1,8)]
cur=np.array([-1.6696,-0.4327,1.7811,-2.4396,-0.5233,2.3642,-1.2694])
tgt=np.array([float(v) for v in sys.argv[1].split(",")])
rclpy.init(); node=rclpy.create_node("pc")
cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service()
links=["panda_hand","panda_link7","panda_link6","panda_link5","panda_link4"]
for a in np.linspace(0,1,11):
    q=cur+(tgt-cur)*a
    req=GetPositionFK.Request(); req.fk_link_names=links
    req.robot_state.joint_state=JointState(name=J, position=list(q))
    f=cli.call_async(req); rclpy.spin_until_future_complete(node,f)
    out=[]
    for ps in f.result().pose_stamped:
        p=ps.pose.position; out.append(f"{ps.header.frame_id[6:]}=({p.x:.2f},{p.y:.2f},{p.z:.2f})")
    # TCP
    ps=f.result().pose_stamped[0]; o=ps.pose.orientation
    x,y,z,w=o.x,o.y,o.z,o.w
    zax=np.array([2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)])
    tcp=np.array([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z])+0.1034*zax
    print(f"a={a:.1f} tcp=({tcp[0]:.2f},{tcp[1]:.2f},{tcp[2]:.2f}) "+" ".join(out))
EOF
timeout 120 python3 /workspace/pathcheck.py -0.866,-0.296,0.969,-2.327,-0.605,2.008,-1.409

# openrua op 64
MAXDELTA=1.0 timeout 600 python3 mv.py -0.19 -0.05 1.25 0.2588 0.8866 -0.368 0.1073 3 && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_final.png && timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side_final.png && timeout 200 python3 /workspace/drawer.py

# openrua op 65
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-moveit-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-moveit-quirks.md <<'OPENRUA_EOF'
---
name: panda-moveit-quirks
description: Verified MoveIt/IK/controller quirks on the Franka Panda workstation (/workspace) that contradict or extend the shipped docs
metadata:
  type: project
---

Verified 2026-09-12 on the Panda + MoveIt machine in /workspace:

- `/compute_ik` default tip link is `panda_link8`, which is rotated 45° about z from `panda_hand`. Set `ik_request.ik_link_name = "panda_hand"` or every solution is 45° off in yaw.
- FK/IK model frame is `world` here (FK reports panda_link0 at (-0.66, 0, 0.912)), NOT the arm base as docs/30-action.md claims. Send world-frame poses with empty frame_id.
- `tools/action/ik_move.py --at tcp` is broken: the bare word `tcp` is counted as a positional arg → usage error. Use /workspace/mv.py (world-frame TCP pose, IK-link fix, FK check of the solution, auto-resend, MAXDELTA gate).
- joint7 tracks slowly: large yaw changes return FollowJointTrajectory error -5 with joint7 lagging; resending the same goal converges (2–3 sends).
- IK can return a wildly different configuration (e.g. joint1 swings 1.7 rad) for a small Cartesian step. Gate on max joint delta and FK-sample the interpolated path before executing.
- Gripper is binary open/closed with no force limit; finger gap on a held bowl wall shrinks over time (8.6 mm → 2 mm) but the grasp held.
- Scene starts unsettled: the bowl floated ~7 cm above the table until the first commanded motion stepped physics. Move (even a hold trajectory) before trusting object heights.

**Why:** each of these cost several turns of debugging; the docs are generic and partly wrong for this machine.
**How to apply:** start from mv.py, verify object heights after one motion, and always check IK joint deltas before executing.
OPENRUA_EOF

# openrua op 66
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md || touch MEMORY.md) && echo "- [Panda MoveIt quirks](panda-moveit-quirks.md) — IK tip link is link8 (45° off), model frame is world, ik_move --at tcp bug, joint7 lag, IK config jumps, unsettled scene" >> MEMORY.md && cat MEMORY.md
