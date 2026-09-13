#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 4
mkdir -p snaps && for c in agentview birdview frontview galleryview paperview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png & done; wait; ls -la snaps; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id" | sort | uniq -c | head -50; echo ---; timeout 10 ros2 topic echo /tf --once 2>/dev/null | grep -E "frame_id" | head

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def cb(m):
    for t in m.transforms:
        got[(t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
end=time.time()+5
while time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
for k,v in sorted(got.items()):
    print(k, f"t=({v.translation.x:.4f},{v.translation.y:.4f},{v.translation.z:.4f}) q=({v.rotation.x:.4f},{v.rotation.y:.4f},{v.rotation.z:.4f},{v.rotation.w:.4f})")
EOF
timeout 30 python3 tfdump.py

# openrua op 7
cat > px2w_batch.py <<'EOF'
"""Batch pixel->world for one camera using a fresh depth frame. usage: cam u,v u,v ..."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
cam = sys.argv[1]
pts = [tuple(int(x) for x in a.split(",")) for a in sys.argv[2:]]
rclpy.init(); node = rclpy.create_node("px2wb")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
def cb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
node.create_subscription(TFMessage, "/tf_static", cb, qos)
import time; end = time.time()+20
while not all(k in got for k in ("d","i","tf")) and time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
d, info, t = got["d"], got["i"], got["tf"]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
q = t.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T = np.eye(4); T[:3,:3]=R; T[:3,3]=[t.translation.x,t.translation.y,t.translation.z]
for (u,v) in pts:
    Z = depth[v,u]
    p = T @ np.array([(u-cx)*Z/fx, (v-cy)*Z/fy, Z, 1.0])
    print(f"({u},{v}) depth={Z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
EOF
timeout 40 python3 px2w_batch.py birdview 237,275 325,293 430,300 375,238 350,238 400,238 320,100 200,400 320,200

# openrua op 8
sed -i 's|node.create_subscription(TFMessage, "/tf_static", cb, qos)|node.create_subscription(TFMessage, "/tf_static", cb, qos); node.create_subscription(TFMessage, "/tf", cb, 100)|' px2w_batch.py && timeout 40 python3 px2w_batch.py birdview 237,275 325,293 430,300 375,238 350,238 400,238 320,100 200,400 320,200

# openrua op 9
cat > heightmap.py <<'EOF'
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = "birdview"
rclpy.init(); node = rclpy.create_node("hm")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def cb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
node.create_subscription(TFMessage, "/tf", cb, 100)
import time; end = time.time()+20
while not all(k in got for k in ("d","i","tf")) and time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
d, info, t = got["d"], got["i"], got["tf"]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
print("fx", fx, "fy", fy, "cx", cx, "cy", cy)
q = t.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tt = np.array([t.translation.x,t.translation.y,t.translation.z])
vv, uu = np.mgrid[0:d.height, 0:d.width]
P = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1) @ R.T + tt
np.save("birdview_world.npy", P)
Z = P[...,2]
# objects above table (0.90) but below robot arm region (x < -0.35 is robot)
mask = (Z > 0.93) & (P[...,0] > -0.35)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    m = lab == i
    pts = P[m]
    print(f"blob {i}: area={stats[i,4]} px, centroid px=({cents[i][0]:.0f},{cents[i][1]:.0f})  x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f}")
EOF
timeout 60 python3 heightmap.py

# openrua op 10
cat > analyze.py <<'EOF'
import numpy as np
P = np.load("birdview_world.npy")
X,Y,Z = P[...,0],P[...,1],P[...,2]
def region(name, m):
    pts = P[m]
    print(name, "n=",len(pts), f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
    h,e = np.histogram(pts[:,2], bins=np.arange(0.9,1.5,0.02))
    print("  z hist:", [(f"{e[i]:.2f}",int(h[i])) for i in range(len(h)) if h[i]>0])
# white mug
region("white mug", (Z>0.93)&(X>-0.2)&(X<0)&(Y<-0.15)&(Y>-0.4))
region("yellow mug", (Z>0.93)&(X>-0.15)&(X<0.1)&(Y>-0.1)&(Y<0.1))
# microwave region: y > 0.05, x > -0.3
region("microwave+door", (Z>0.93)&(X>-0.3)&(Y>0.05))
# microwave body top
mw = (Z>1.09)&(Z<1.12)&(X>-0.3)&(Y>0.05)
region("mw top plane", mw)
door = (Z>1.09)&(Z<1.12)&(X>-0.3)&(Y>0.05)&(Y<0.29)
region("door top", door)
# door footprint: in the x=-0.207 plane; find all points with x in [-0.23,-0.18] and y in [0.05,0.3]
region("door slab", (Z>0.93)&(X>-0.24)&(X<-0.17)&(Y>0.05)&(Y<0.30))
# body
region("body", (Z>0.93)&(X>-0.3)&(Y>0.30))
# hand
region("hand", (Z>1.0)&(X>-0.35)&(X<-0.1)&(Y>-0.1)&(Y<0.1))
EOF
python3 analyze.py

# openrua op 11
cat > hmimg.py <<'EOF'
import numpy as np, cv2
P = np.load("birdview_world.npy")
Z = P[...,2]
# crop to table region rows 150-420, cols 150-500, upscale
img = np.clip((Z-0.9)/0.6,0,1)
img8 = (img*255).astype(np.uint8)
col = cv2.applyColorMap(img8, cv2.COLORMAP_JET)
crop = col[150:420, 150:500]
crop = cv2.resize(crop, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
# grid lines every 0.1 m in world: world x = ? need mapping; just draw pixel grid every 20px (~7cm)
for i in range(0, crop.shape[1], 40): cv2.line(crop,(i,0),(i,crop.shape[0]-1),(255,255,255),1); cv2.putText(crop,str(150+i//2),(i,10),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
for j in range(0, crop.shape[0], 40): cv2.line(crop,(0,j),(crop.shape[1]-1,j),(255,255,255),1); cv2.putText(crop,str(150+j//2),(0,j+10),cv2.FONT_HERSHEY_SIMPLEX,0.3,(255,255,255),1)
cv2.imwrite("snaps/birdview_height.png", crop)
EOF
python3 hmimg.py

# openrua op 12
timeout 40 python3 px2w_batch.py robot0_robotview 600,150 620,180 630,210 600,220 560,100 500,200 450,160 400,300 610,100

# openrua op 13
timeout 40 python3 px2w_batch.py frontview 445,300 455,320 450,340 470,300 480,350 520,350 560,380 500,300 460,290

# openrua op 14
cat > analyze2.py <<'EOF'
import numpy as np
P = np.load("birdview_world.npy")
X,Y,Z = P[...,0],P[...,1],P[...,2]
top = (Z>1.09)&(Z<1.12)&(X>-0.15)&(Y>0.1)
pts = P[top]
print("body top: x[%.3f,%.3f] y[%.3f,%.3f]"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max()))
# per-row of x, min/max y (to see shape)
for x0 in np.arange(-0.2, 0.2, 0.02):
    m = top & (X>=x0)&(X<x0+0.02)
    if m.sum(): print(f"x={x0:.2f}: y[{Y[m].min():.3f},{Y[m].max():.3f}] n={m.sum()}")
# door slab: z in [1.09,1.12], x<-0.15, y in [0,0.3]
door = (Z>1.09)&(Z<1.12)&(X<-0.15)&(Y>0.0)&(Y<0.30)
pts = P[door]
print("door top: x[%.3f,%.3f] y[%.3f,%.3f] n=%d"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),len(pts)))
# fit line to door points
A = np.c_[pts[:,1], np.ones(len(pts))]
k,b = np.linalg.lstsq(A, pts[:,0], rcond=None)[0]
print(f"door line: x = {k:.3f}*y + {b:.3f}; angle from y axis = {np.degrees(np.arctan(k)):.1f} deg")
for y0 in np.arange(0.0, 0.32, 0.02):
    m = door & (Y>=y0)&(Y<y0+0.02)
    if m.sum(): print(f"y={y0:.2f}: x[{X[m].min():.3f},{X[m].max():.3f}] n={m.sum()}")
EOF
python3 analyze2.py

# openrua op 15
cat > analyze3.py <<'EOF'
import numpy as np
P = np.load("birdview_world.npy")
X,Y,Z = P[...,0],P[...,1],P[...,2]
for name,(x0,x1,y0,y1) in {"white":(-0.2,0.0,-0.4,-0.15),"yellow":(-0.15,0.1,-0.1,0.12)}.items():
    reg = (X>x0)&(X<x1)&(Y>y0)&(Y<y1)
    for zlo in [0.93, 1.0, 1.04, 1.06, 1.08]:
        m = reg & (Z>zlo)
        if m.sum()==0: continue
        pts = P[m]
        print(f"{name} z>{zlo}: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] (w={pts[:,0].max()-pts[:,0].min():.3f}) y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] (w={pts[:,1].max()-pts[:,1].min():.3f})")
    # inside floor
    m = reg & (Z>0.93)&(Z<1.0)
    pts=P[m]; print(f"{name} inner floor z mean {pts[:,2].mean():.3f}, center ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}), n={len(pts)}")
    # rim ring center
    m = reg & (Z>1.06)
    pts=P[m]; print(f"{name} rim center ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) zmax={pts[:,2].max():.3f}")
EOF
python3 analyze3.py

# openrua op 16
timeout 40 python3 px2w_batch.py robot0_robotview 340,95 300,130 355,240 340,225 420,140 20,80 10,200 250,300 && timeout 40 python3 px2w_batch.py frontview 180,290 175,345 195,320 330,300 335,355 300,330

# openrua op 17
python3 -c "
import cv2
im=cv2.imread('snaps/frontview.png'); c=im[250:400,120:400]; cv2.imwrite('snaps/front_crop.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('snaps/sideview.png'); c=im[180:300,240:360]; cv2.imwrite('snaps/side_crop.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 18
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 19
timeout 60 python3 heightmap.py >/dev/null && python3 analyze3.py && timeout 40 python3 px2w_batch.py birdview 320,100

# openrua op 20
cat > analyze4.py <<'EOF'
import numpy as np
P = np.load("birdview_world.npy")
X,Y,Z = P[...,0],P[...,1],P[...,2]
for name,(x0,x1,y0,y1) in {"white":(-0.2,0.0,-0.4,-0.15),"yellow":(-0.15,0.1,-0.1,0.12)}.items():
    reg = (X>x0)&(X<x1)&(Y>y0)&(Y<y1)&(Z>0.91)
    pts = P[reg]
    print(f"{name}: n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
    h,e = np.histogram(pts[:,2], bins=np.arange(0.9,1.1,0.01))
    print("  z hist:", [(f"{e[i]:.2f}",int(h[i])) for i in range(len(h)) if h[i]>0])
    zmax = pts[:,2].max()
    rim = reg & (Z > zmax-0.015)
    pr = P[rim]
    print(f"  rim: center ({pr[:,0].mean():.3f},{pr[:,1].mean():.3f}) x[{pr[:,0].min():.3f},{pr[:,0].max():.3f}] y[{pr[:,1].min():.3f},{pr[:,1].max():.3f}]")
    # body-only rim excluding handle: fit circle to rim points via least squares
    A = np.c_[2*pr[:,0], 2*pr[:,1], np.ones(len(pr))]
    b = pr[:,0]**2 + pr[:,1]**2
    cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
    r = np.sqrt(c + cx**2 + cy**2)
    print(f"  circle fit: center ({cx:.3f},{cy:.3f}) r={r:.3f}")
EOF
python3 analyze4.py

# openrua op 21
cat > profile.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
cam = sys.argv[1]
x0,x1,y0,y1 = map(float, sys.argv[2:6])
rclpy.init(); node = rclpy.create_node("prof")
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
def cb(m):
    for t in m.transforms:
        if t.child_frame_id == f"{cam}_optical_frame": got["tf"] = t.transform
node.create_subscription(TFMessage, "/tf", cb, 100)
import time; end = time.time()+20
while not all(k in got for k in ("d","i","tf")) and time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
d, info, t = got["d"], got["i"], got["tf"]
depth = np.frombuffer(d.data, dtype=np.float32).reshape(d.height, d.width)
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
q = t.rotation; x,y,z,w = q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tt = np.array([t.translation.x,t.translation.y,t.translation.z])
vv, uu = np.mgrid[0:d.height, 0:d.width]
P = np.stack([(uu-cx)*depth/fx, (vv-cy)*depth/fy, depth], -1) @ R.T + tt
np.save(f"{cam}_world.npy", P)
X,Y,Z = P[...,0],P[...,1],P[...,2]
m = (X>x0)&(X<x1)&(Y>y0)&(Y<y1)&(Z>0.905)
pts = P[m]
print(f"n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
for zlo in np.arange(0.90, 1.03, 0.01):
    mm = m & (Z>=zlo)&(Z<zlo+0.01)
    if mm.sum()<3: continue
    p = P[mm]
    print(f"z={zlo:.2f}: n={mm.sum():4d} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ywidth={p[:,1].max()-p[:,1].min():.3f} xwidth={p[:,0].max()-p[:,0].min():.3f}")
EOF
timeout 60 python3 profile.py frontview -0.2 0.0 -0.4 -0.15

# openrua op 22
timeout 60 python3 profile.py robot0_robotview -0.16 0.17 0.2 0.5 >/dev/null; python3 - <<'EOF'
import numpy as np
P = np.load("robot0_robotview_world.npy")
X,Y,Z = P[...,0],P[...,1],P[...,2]
m = (X>-0.16)&(X<0.17)&(Y>0.2)&(Y<0.6)&(Z>0.92)&(Z<1.09)
print("n", m.sum())
for xlo in np.arange(-0.16,0.17,0.03):
    for zlo in np.arange(0.92,1.09,0.03):
        mm = m&(X>=xlo)&(X<xlo+0.03)&(Z>=zlo)&(Z<zlo+0.03)
        if mm.sum()>2: print(f"x={xlo:.2f} z={zlo:.2f}: n={mm.sum():3d} y[{Y[mm].min():.3f},{Y[mm].max():.3f}] mean {Y[mm].mean():.3f}")
EOF

# openrua op 23
python3 - <<'EOF'
import numpy as np
P = np.load("robot0_robotview_world.npy")
X,Y,Z = P[...,0],P[...,1],P[...,2]
m = (X>-0.13)&(X<0.08)&(Z>0.95)&(Z<1.08)&(Y>0.2)
print("n", m.sum())
h,e = np.histogram(Y[m], bins=np.arange(0.2,0.7,0.02))
print([(f"{e[i]:.2f}",int(h[i])) for i in range(len(h)) if h[i]>0])
# interior back wall / floor
for ylo in np.arange(0.28,0.5,0.02):
    mm = m&(Y>=ylo)&(Y<ylo+0.02)
    if mm.sum()>5: print(f"y={ylo:.2f}: n={mm.sum()} x[{X[mm].min():.3f},{X[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]")
# cavity floor: points with y in [0.3,0.45], x in [-0.13,0.08], z<0.97
m2 = (X>-0.13)&(X<0.08)&(Y>0.29)&(Y<0.46)&(Z>0.9)&(Z<0.98)
print("floor candidates n", m2.sum(), "z mean", Z[m2].mean() if m2.sum() else None)
h,e = np.histogram(Z[m2], bins=np.arange(0.9,0.98,0.005)); print([(f"{e[i]:.3f}",int(h[i])) for i in range(len(h)) if h[i]>0])
EOF

# openrua op 24
python3 - <<'EOF'
import numpy as np
P = np.load("robot0_robotview_world.npy")
X,Y,Z = P[...,0],P[...,1],P[...,2]
# front face of microwave (y ~ 0.26-0.275), x in [-0.1,0.05]: which z are present -> opening top/bottom edges
m = (X>-0.1)&(X<0.05)&(Y>0.255)&(Y<0.28)
h,e = np.histogram(Z[m], bins=np.arange(0.9,1.12,0.005))
print("front face z:", [(f"{e[i]:.3f}",int(h[i])) for i in range(len(h)) if h[i]>0])
# opening x edges: front face points with z in [0.97,1.05]
m = (Y>0.255)&(Y<0.28)&(Z>0.97)&(Z<1.05)&(X>-0.3)&(X<0.3)
h,e = np.histogram(X[m], bins=np.arange(-0.3,0.3,0.01))
print("front face x (z .97-1.05):", [(f"{e[i]:.2f}",int(h[i])) for i in range(len(h)) if h[i]>0])
# door frame post at x=-0.16..-0.13: what's its y extent
m = (X>-0.17)&(X<-0.13)&(Z>0.97)&(Z<1.05)&(Y>0.2)&(Y<0.5)
h,e = np.histogram(Y[m], bins=np.arange(0.2,0.5,0.01))
print("post y:", [(f"{e[i]:.2f}",int(h[i])) for i in range(len(h)) if h[i]>0])
EOF

# openrua op 25
timeout 60 python3 profile.py frontview -0.3 0.3 0.2 0.5 > /dev/null; python3 - <<'EOF'
import numpy as np
for cam in ["frontview"]:
    P = np.load(f"{cam}_world.npy")
    X,Y,Z = P[...,0],P[...,1],P[...,2]
    m = (X>-0.1)&(X<0.05)&(Y>0.25)&(Y<0.285)
    h,e = np.histogram(Z[m], bins=np.arange(0.9,1.12,0.005))
    print(cam, "front face z:", [(f"{e[i]:.3f}",int(h[i])) for i in range(len(h)) if h[i]>0])
    # interior: points with y in [0.29,0.46], x in [-0.13,0.08]
    m = (X>-0.13)&(X<0.08)&(Y>0.285)&(Y<0.47)&(Z>0.9)&(Z<1.1)
    print("interior n", m.sum())
    h,e = np.histogram(Z[m], bins=np.arange(0.9,1.1,0.01)); print(" z:", [(f"{e[i]:.2f}",int(h[i])) for i in range(len(h)) if h[i]>0])
    h,e = np.histogram(Y[m], bins=np.arange(0.28,0.48,0.01)); print(" y:", [(f"{e[i]:.2f}",int(h[i])) for i in range(len(h)) if h[i]>0])
    h,e = np.histogram(X[m], bins=np.arange(-0.15,0.1,0.01)); print(" x:", [(f"{e[i]:.2f}",int(h[i])) for i in range(len(h)) if h[i]>0])
EOF

# openrua op 26
python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy")
X,Y,Z = P[...,0],P[...,1],P[...,2]
# white mug handle: y > -0.281+0.047
m = (X>-0.2)&(X<0.0)&(Y>-0.234)&(Y<-0.15)&(Z>0.91)
pts = P[m]
print(f"handle n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}]")
for ylo in np.arange(-0.235,-0.195,0.005):
    mm = m&(Y>=ylo)&(Y<ylo+0.005)
    if mm.sum(): print(f"y={ylo:.3f}: n={mm.sum():3d} x[{X[mm].min():.3f},{X[mm].max():.3f}] zmax={Z[mm].max():.3f} zmean={Z[mm].mean():.3f}")
EOF

# openrua op 27
cat > geturdf.py <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); node = rclpy.create_node("urdf")
got = []
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String, "/robot_description", got.append, qos)
import time; end=time.time()+10
while not got and time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
open("robot.urdf","w").write(got[0].data if got else "")
print(len(got[0].data) if got else "none")
EOF
timeout 30 python3 geturdf.py && grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A10 'link name="panda_leftfinger"' robot.urdf | head -40

# openrua op 28
python3 - <<'EOF'
import numpy as np, struct
def stl_bbox(path):
    data = open(path,'rb').read()
    if data[:5]==b'solid' and b'facet' in data[:300]:
        import re
        v = np.array([[float(x) for x in m] for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', data)])
    else:
        n = struct.unpack_from('<I', data, 80)[0]
        arr = np.frombuffer(data[84:84+n*50], dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v = arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ["hand","finger"]:
    lo,hi = stl_bbox(f"/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl")
    print(f, "min", np.round(lo,4), "max", np.round(hi,4), "size", np.round(hi-lo,4))
EOF

# openrua op 29
find / -name "hand.stl" -o -name "finger.stl" -o -name "hand.dae" 2>/dev/null | head; ls /opt/ros/jazzy/share/ 2>/dev/null | grep -i panda

# openrua op 30
cat > rob.py <<'EOF'
"""Shared helpers: IK/FK clients, world<->base frames, pose building, trajectory + gripper."""
import time, numpy as np, rclpy, yaml
from scipy.spatial.transform import Rotation as Rot
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"]=="joint_trajectory")
JOINTS = ARM["joints"]
BASE_W = np.array([-0.66, 0.0, 0.912])   # world -> panda_link0 translation (rotation identity)
TCP = M["hand"]["tcp_offset_m"]

def w2b(p): return np.asarray(p, float) - BASE_W
def b2w(p): return np.asarray(p, float) + BASE_W

def R_from_axes(approach, finger_axis):
    z = np.asarray(approach, float); z /= np.linalg.norm(z)
    y = np.asarray(finger_axis, float); y -= y.dot(z)*z; y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack([x, y, z])

def hand_pose_from_tcp(tcp_world, R):
    """panda_hand origin (world) given TCP world position and hand rotation matrix."""
    return np.asarray(tcp_world, float) - TCP * R[:, 2]

class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, ARM["port"])
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.spin(0.5)
    def _js_cb(self, m):
        self.js = dict(zip(m.name, m.position))
    def spin(self, sec):
        end = time.time()+sec
        while time.time() < end: rclpy.spin_once(self.node, timeout_sec=0.05)
    def joints(self):
        self.js = {}
        while len(self.js) < 7: rclpy.spin_once(self.node, timeout_sec=0.1)
        return [self.js[j] for j in JOINTS]
    def fingers(self):
        self.joints(); return self.js.get("panda_finger_joint1"), self.js.get("panda_finger_joint2")
    def _call(self, cli, req, timeout=60):
        cli.wait_for_service(timeout_sec=10)
        f = cli.call_async(req); rclpy.spin_until_future_complete(self.node, f, timeout_sec=timeout)
        return f.result()
    def solve_ik(self, hand_pos_world, R, seed=None, link="panda_hand", timeout=5.0):
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = M["planning"]["group"]; r.ik_link_name = link
        r.pose_stamped.header.frame_id = ""
        pb = w2b(hand_pos_world); q = Rot.from_matrix(R).as_quat()
        p = r.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        seed = seed if seed is not None else self.joints()
        r.robot_state.joint_state.name = list(JOINTS); r.robot_state.joint_state.position = [float(v) for v in seed]
        r.timeout.sec = int(timeout); r.timeout.nanosec = int((timeout%1)*1e9)
        r.avoid_collisions = False
        res = self._call(self.ik, req)
        if res is None or res.error_code.val != 1: return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]
    def solve_fk(self, q, link="panda_hand"):
        req = GetPositionFK.Request(); req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS); req.robot_state.joint_state.position = [float(v) for v in q]
        res = self._call(self.fk, req)
        p = res.pose_stamped[0].pose
        pos = b2w([p.position.x, p.position.y, p.position.z])
        R = Rot.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
        return pos, R
    def tcp(self):
        pos, R = self.solve_fk(self.joints()); return pos + TCP*R[:,2], R
    def move_joints(self, q, sec, via=None):
        """Send one FJT goal; via = list of (q, t) intermediate points."""
        self.fjt.wait_for_server(timeout_sec=10)
        g = FollowJointTrajectory.Goal(); g.trajectory.joint_names = list(JOINTS)
        pts = []
        for qq, t in (via or []) + [(q, sec)]:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t%1)*1e9)); pts.append(pt)
        g.trajectory.points = pts
        f = self.fjt.send_goal_async(g); rclpy.spin_until_future_complete(self.node, f)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf)
        code = rf.result().result.error_code
        err = np.abs(np.array(self.joints()) - np.array(q)).max()
        print(f"  move done code={code} max_joint_err={err:.4f}")
        return code, err
    def gripper(self, width):
        self.grip.wait_for_server(timeout_sec=10)
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
        f = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, f, timeout_sec=30)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={self.fingers()}")
        return r
EOF
cat > iktest.py <<'EOF'
import numpy as np
from rob import *
r = Robot("iktest")
q0 = r.joints(); print("q0", np.round(q0,3))
pos, R = r.solve_fk(q0); print("hand world", np.round(pos,3), "tcp", np.round(pos+TCP*R[:,2],3)); print(np.round(R,2))
tests = {
 "pregrasp(-y approach)": ((-0.080, -0.12, 0.945), R_from_axes([0,-1,0],[1,0,0])),
 "grasp(-y approach)":    ((-0.080, -0.207, 0.945), R_from_axes([0,-1,0],[1,0,0])),
 "lift":                  ((-0.080, -0.207, 1.15), R_from_axes([0,-1,0],[1,0,0])),
 "insert(+y approach)":   ((-0.035, 0.285, 1.02), R_from_axes([0,1,0],[1,0,0])),
 "pre-insert":            ((-0.035, 0.10, 1.02), R_from_axes([0,1,0],[1,0,0])),
 "topdown over mug":      ((-0.080, -0.281, 1.10), R_from_axes([0,0,-1],[0,1,0])),
}
for name,(tcp,R) in tests.items():
    for fa in ([1,0,0],[-1,0,0]) if "topdown" not in name else ([0,1,0],[1,0,0]):
        RR = R_from_axes(R[:,2], fa)
        q = r.solve_ik(hand_pose_from_tcp(tcp, RR), RR, seed=q0)
        print(f"{name} finger_axis={fa}: {'OK '+str(np.round(q,3)) if q else 'FAIL'}")
EOF
timeout 120 python3 iktest.py

# openrua op 31
cat > fkframe.py <<'EOF'
from rob import *
r = Robot("fkf")
req = GetPositionFK.Request(); req.fk_link_names = ["panda_link0","panda_hand"]
req.robot_state.joint_state.name = list(JOINTS); req.robot_state.joint_state.position = r.joints()
res = r._call(r.fk, req)
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
EOF
timeout 60 python3 fkframe.py

# openrua op 32
sed -i 's|^BASE_W = .*|BASE_W = np.array([0.0, 0.0, 0.0])   # FK/IK model frame == world on this machine (verified via /compute_fk)|' rob.py && timeout 120 python3 iktest.py

# openrua op 33
cat > look.py <<'EOF'
import sys, numpy as np
from rob import *
r = Robot("look")
x,y,z = map(float, sys.argv[1:4]); sec = float(sys.argv[4]) if len(sys.argv)>4 else 4.0
R = R_from_axes([0,0,-1],[0,1,0])   # straight down, fingers along world y
q = r.solve_ik(hand_pose_from_tcp((x,y,z), R), R)
print("ik", None if q is None else np.round(q,3))
if q is None: sys.exit(1)
r.move_joints(q, sec)
p, RR = r.tcp(); print("tcp now", np.round(p,3))
EOF
timeout 300 python3 look.py -0.08 -0.28 1.30 4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png

# openrua op 34
cat > ikbest.py <<'EOF'
import numpy as np
from rob import *
def best_ik(r, tcp, R, seeds=None, tries=8, weights=None):
    """Try several seeds, return the solution closest (weighted) to current joints."""
    cur = np.array(r.joints()); sols = []
    lim = np.array(ARM["limits_rad"])
    seeds = [cur] + (seeds or [])
    rng = np.random.default_rng(0)
    for i in range(tries):
        s = seeds[i] if i < len(seeds) else np.clip(cur + rng.normal(0, 0.6, 7), lim[:,0]+0.05, lim[:,1]-0.05)
        q = r.solve_ik(hand_pose_from_tcp(tcp, R), R, seed=list(s), timeout=2.0)
        if q is not None: sols.append(np.array(q))
    if not sols: return None
    w = np.array(weights if weights is not None else [1,1,1,1,0.5,0.5,0.3])
    d = [np.sum(w*np.abs(q-cur)) for q in sols]
    return list(sols[int(np.argmin(d))])
EOF
cat > look.py <<'EOF'
import sys, numpy as np
from rob import *; from ikbest import best_ik
r = Robot("look")
x,y,z = map(float, sys.argv[1:4]); sec = float(sys.argv[4]) if len(sys.argv)>4 else 4.0
print("now q", np.round(r.joints(),3))
R = R_from_axes([0,0,-1],[0,1,0])
q = best_ik(r, (x,y,z), R)
print("ik", None if q is None else np.round(q,3))
if q is None: sys.exit(1)
r.move_joints(q, sec)
p, RR = r.tcp(); print("tcp now", np.round(p,3))
EOF
timeout 300 python3 look.py -0.08 -0.28 1.30 4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png

# openrua op 35
timeout 20 ros2 topic echo /joint_states --once | grep -A7 "^position" ; cat .ros/log/*.log | grep -i -E "toler|joint7|clip|limit" | tail -5

# openrua op 36
cat > j7test.py <<'EOF'
import numpy as np, sys
from rob import *
r = Robot("j7")
q = r.joints(); q[6] = float(sys.argv[1])
r.move_joints(q, float(sys.argv[2]))
print("j7 now", round(r.joints()[6],3))
EOF
timeout 300 python3 j7test.py -1.5 3

# openrua op 37
timeout 300 python3 j7test.py -1.5 3; timeout 300 python3 j7test.py -2.5 6

# openrua op 38
python3 - <<'EOF'
import re
s = open("rob.py").read()
s = s.replace('''        self.fjt.wait_for_server(timeout_sec=10)
        g = FollowJointTrajectory.Goal()''','''        self.fjt.wait_for_server(timeout_sec=10)
        # sim joints move at most ~0.2 rad/s: stretch the duration so the goal is reachable
        cur = np.array(self.joints()); need = np.abs(np.array(q) - cur).max() / 0.15
        if need > sec:
            scale = need / sec; sec = need
            via = [(qq, t*scale) for qq, t in (via or [])]
            print(f"  (duration stretched to {sec:.1f}s)")
        g = FollowJointTrajectory.Goal()''')
open("rob.py","w").write(s)
EOF
echo ok

# openrua op 39
cat > scene.py <<'EOF'
"""Populate the MoveIt planning scene with measured obstacles (world frame)."""
import sys, numpy as np, rclpy
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def box(name, center, size, yaw=0.0):
    co = CollisionObject(); co.header.frame_id = "world"; co.id = name
    sp = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(s) for s in size])
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center)
    q = Rot.from_euler("z", yaw).as_quat(); p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    co.primitives = [sp]; co.primitive_poses = [p]; co.operation = CollisionObject.ADD
    return co

def cyl(name, center, radius, height):
    co = CollisionObject(); co.header.frame_id = "world"; co.id = name
    sp = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(height), float(radius)])
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center); p.orientation.w = 1.0
    co.primitives = [sp]; co.primitive_poses = [p]; co.operation = CollisionObject.ADD
    return co

def remove(name):
    co = CollisionObject(); co.header.frame_id = "world"; co.id = name; co.operation = CollisionObject.REMOVE
    return co

WHITE_MUG = dict(center=(-0.081, -0.281), r=0.047, h=0.112)
YELLOW_MUG = dict(center=(-0.022, 0.027), r=0.050, h=0.106)

def objects(include_white=True):
    objs = [
        box("table", (0.0, 0.0, 0.85), (1.6, 1.4, 0.10)),                       # top at 0.90
        box("microwave", (0.005, 0.365, 1.0035), (0.33, 0.20, 0.207)),          # x[-0.16,0.17] y[0.266,0.464]
        box("mw_panel", (0.125, 0.255, 1.0035), (0.09, 0.03, 0.207)),           # control panel lip
        box("mw_door", (-0.212, 0.18, 1.0035), (0.26, 0.03, 0.207), yaw=np.radians(68.2)),
        cyl("yellow_mug", (*YELLOW_MUG["center"], 0.90 + YELLOW_MUG["h"]/2), YELLOW_MUG["r"], YELLOW_MUG["h"]),
    ]
    if include_white:
        objs.append(cyl("white_mug", (*WHITE_MUG["center"], 0.90 + WHITE_MUG["h"]/2), WHITE_MUG["r"], WHITE_MUG["h"]))
    return objs

def apply(node, cos, attached=None, detach=None):
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene"); cli.wait_for_service(timeout_sec=10)
    ps = PlanningScene(); ps.is_diff = True; ps.robot_state.is_diff = True
    ps.world.collision_objects = cos
    if attached is not None: ps.robot_state.attached_collision_objects = [attached]
    if detach is not None:
        a = AttachedCollisionObject(); a.link_name = "panda_hand"; a.object.id = detach; a.object.operation = CollisionObject.REMOVE
        ps.robot_state.attached_collision_objects = [a]
    req = ApplyPlanningScene.Request(scene=ps)
    f = cli.call_async(req); rclpy.spin_until_future_complete(node, f, timeout_sec=30)
    return f.result().success if f.result() else None

if __name__ == "__main__":
    rclpy.init(); node = rclpy.create_node("scene")
    print("applied:", apply(node, objects(include_white="--no-white" not in sys.argv)))
EOF
timeout 60 python3 scene.py && timeout 30 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 4}}" 2>/dev/null | grep -E "^\s+id:" 

# openrua op 40
timeout 30 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 12}}" 2>/dev/null | grep -oE "id: [a-z_]+" 

# openrua op 41
timeout 30 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 1023}}" 2>/dev/null | grep -E "id:|frame_id" | head -20

# openrua op 42
cat > plan.py <<'EOF'
"""MoveIt planning helpers layered on rob.Robot: joint-goal planning and Cartesian straight lines."""
import numpy as np, rclpy
from scipy.spatial.transform import Rotation as Rot
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetCartesianPath
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
from rob import *

VMAX = 0.15   # rad/s the sim controller tracks reliably

def retime(traj, vmax=VMAX):
    """Rescale a JointTrajectory so no joint exceeds vmax; returns list of (positions, t)."""
    pts = [(list(p.positions), p.time_from_start.sec + p.time_from_start.nanosec*1e-9) for p in traj.points]
    out, t = [], 0.0
    prev = None
    for q, _ in pts:
        if prev is not None:
            t += max(np.abs(np.array(q)-np.array(prev)).max()/vmax, 0.05)
        out.append((q, t)); prev = q
    return out

class Planner(Robot):
    def __init__(self, name="planner"):
        super().__init__(name)
        self.mg = ActionClient(self.node, MoveGroup, "/move_action")
        self.cart = self.node.create_client(GetCartesianPath, "/compute_cartesian_path")
    def _start_state(self):
        rs = RobotState(); rs.joint_state.name = list(JOINTS); rs.joint_state.position = [float(v) for v in self.joints()]
        return rs
    def plan_joints(self, q_goal, attempts=3, time=10.0):
        self.mg.wait_for_server(timeout_sec=10)
        g = MoveGroup.Goal(); r = g.request
        r.group_name = M["planning"]["group"]; r.num_planning_attempts = attempts; r.allowed_planning_time = time
        r.max_velocity_scaling_factor = 0.1; r.max_acceleration_scaling_factor = 0.1
        r.start_state = self._start_state()
        c = Constraints()
        for j, v in zip(JOINTS, q_goal):
            c.joint_constraints.append(JointConstraint(joint_name=j, position=float(v), tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        r.goal_constraints = [c]
        g.planning_options.plan_only = True
        f = self.mg.send_goal_async(g); rclpy.spin_until_future_complete(self.node, f)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf)
        res = rf.result().result
        if res.error_code.val != 1:
            print(f"  plan FAILED code={res.error_code.val}"); return None
        traj = res.planned_trajectory.joint_trajectory
        print(f"  plan ok: {len(traj.points)} pts")
        return traj
    def execute(self, traj):
        pts = retime(traj)
        q, t = pts[-1]
        return self.move_joints(q, t, via=pts[:-1])
    def goto_joints(self, q_goal):
        traj = self.plan_joints(q_goal)
        if traj is None: return False
        code, err = self.execute(traj)
        return err < 0.05
    def cartesian(self, hand_targets, avoid=True, step=0.01):
        """hand_targets: list of (hand_pos_world, R). Returns (traj, fraction)."""
        self.cart.wait_for_service(timeout_sec=10)
        req = GetCartesianPath.Request()
        req.header.frame_id = "world"; req.group_name = M["planning"]["group"]; req.link_name = "panda_hand"
        req.start_state = self._start_state(); req.max_step = step; req.jump_threshold = 0.0
        req.avoid_collisions = avoid; req.max_velocity_scaling_factor = 0.1; req.max_acceleration_scaling_factor = 0.1
        for pos, R in hand_targets:
            p = Pose(); p.position.x, p.position.y, p.position.z = map(float, pos)
            q = Rot.from_matrix(R).as_quat(); p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
            req.waypoints.append(p)
        res = self._call(self.cart, req)
        if res is None: print("  cartesian: no answer"); return None, 0.0
        print(f"  cartesian: fraction={res.fraction:.2f} code={res.error_code.val} pts={len(res.solution.joint_trajectory.points)}")
        return res.solution.joint_trajectory, res.fraction
    def move_line_tcp(self, tcp_targets, R, avoid=True, min_fraction=0.99):
        traj, frac = self.cartesian([(hand_pose_from_tcp(t, R), R) for t in tcp_targets], avoid=avoid)
        if traj is None or frac < min_fraction: return False
        code, err = self.execute(traj)
        p, _ = self.tcp(); print("  tcp now", np.round(p,3))
        return err < 0.05
EOF
cat > step1.py <<'EOF'
"""Step 1: go to the pre-grasp pose beside the white mug handle (approach from +y, pitched 45 deg down)."""
import numpy as np, sys
from plan import *; from ikbest import best_ik
r = Planner("step1")
pitch = np.radians(float(sys.argv[1]) if len(sys.argv)>1 else 45)
approach = np.array([0, -np.cos(pitch), -np.sin(pitch)])   # toward -y and down
R = R_from_axes(approach, [1,0,0])
bar = np.array([-0.080, -0.207, 0.947])                     # handle outer bar centre (world)
pre = bar - 0.10*approach                                   # 10 cm back along the approach
print("pre-grasp tcp", np.round(pre,3), "hand", np.round(hand_pose_from_tcp(pre,R),3))
q = best_ik(r, pre, R)
print("ik", None if q is None else np.round(q,3))
if q is None: sys.exit(1)
ok = r.goto_joints(q)
p, RR = r.tcp(); print("tcp now", np.round(p,3), "approach now", np.round(RR[:,2],3), "ok", ok)
EOF
timeout 600 python3 step1.py 45

# openrua op 43
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih2.png & timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front2.png & wait

# openrua op 44
python3 -c "
import cv2; im=cv2.imread('snaps/front2.png'); c=im[200:380,80:330]; cv2.imwrite('snaps/front2_crop.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 45
cat > step2.py <<'EOF'
"""Step 2: straight-line approach to the handle bar and close the gripper."""
import numpy as np, sys
from plan import *
r = Planner("step2")
p0, R = r.tcp(); approach = R[:,2]
bar = np.array([-0.080, -0.207, 0.947])
target = bar + float(sys.argv[1] if len(sys.argv)>1 else 0.0)*approach   # optional extra depth along approach
print("tcp", np.round(p0,3), "-> target", np.round(target,3))
ok = r.move_line_tcp([target], R, avoid=True)
if not ok:
    print("retrying without collision checking"); ok = r.move_line_tcp([target], R, avoid=False)
print("approach ok", ok)
EOF
timeout 600 python3 step2.py 0.0

# openrua op 46
timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front3.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/side3.png >/dev/null; python3 -c "
import cv2; im=cv2.imread('snaps/front3.png'); c=im[230:380,100:300]; cv2.imwrite('snaps/front3_crop.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('snaps/side3.png'); c=im[150:290,250:400]; cv2.imwrite('snaps/side3_crop.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 47
cat > grasp.py <<'EOF'
import sys; from rob import *
r = Robot("grasp"); r.gripper(float(sys.argv[1]))
EOF
timeout 200 python3 grasp.py 0.0

# openrua op 48
cat > attach.py <<'EOF'
"""Attach/detach the white mug collision body to panda_hand in the planning scene."""
import sys, numpy as np, rclpy
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from rob import *; import scene

def attach_mug(r, mug_center_world, radius=0.05, height=0.12):
    hand_pos, R = r.solve_fk(r.joints())
    d_hand = R.T @ (np.asarray(mug_center_world) - hand_pos)       # mug centre in hand frame
    q = Rot.from_matrix(R.T).as_quat()                              # world-vertical axis expressed in hand frame
    a = AttachedCollisionObject(); a.link_name = "panda_hand"
    a.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
    co = a.object; co.header.frame_id = "panda_hand"; co.id = "white_mug"
    co.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[height, radius])]
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, d_hand)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    co.primitive_poses = [p]; co.operation = CollisionObject.ADD
    ok = scene.apply(r.node, [scene.remove("white_mug")], attached=a)
    print("attached white_mug at hand-frame offset", np.round(d_hand,3), "ok", ok)

def detach_mug(r, world_center):
    ok = scene.apply(r.node, [scene.cyl("white_mug", world_center, 0.047, 0.112)], detach="white_mug")
    print("detached, world obj re-added at", np.round(world_center,3), "ok", ok)

if __name__ == "__main__":
    r = Robot("attach")
    if sys.argv[1] == "attach": attach_mug(r, [float(v) for v in sys.argv[2:5]])
    else: detach_mug(r, [float(v) for v in sys.argv[2:5]])
EOF
timeout 60 python3 attach.py attach -0.081 -0.281 0.956 && cat > step3.py <<'EOF'
"""Step 3: lift straight up with the mug."""
import numpy as np, sys
from plan import *
r = Planner("step3")
p0, R = r.tcp(); dz = float(sys.argv[1]) if len(sys.argv)>1 else 0.12
target = p0 + np.array([0,0,dz])
ok = r.move_line_tcp([target], R, avoid=True)
if not ok: print("retry w/o collision check"); ok = r.move_line_tcp([target], R, avoid=False)
print("lift ok", ok, "fingers", r.fingers())
EOF
timeout 600 python3 step3.py 0.12

# openrua op 49
timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front4.png >/dev/null; python3 -c "
import cv2; im=cv2.imread('snaps/front4.png'); c=im[180:380,80:330]; cv2.imwrite('snaps/front4_crop.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 50
python3 - <<'EOF'
s = open("attach.py").read()
s = s.replace("    ok = scene.apply(r.node, [scene.remove(\"white_mug\")], attached=a)",
 "    ok0 = scene.apply(r.node, [scene.remove(\"white_mug\")])\n    ok = scene.apply(r.node, [], attached=a); print('remove', ok0)")
open("attach.py","w").write(s)
EOF
timeout 60 python3 attach.py attach -0.081 -0.281 1.076 && timeout 30 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 1023}}" 2>/dev/null | grep -oE "attached_collision_objects=\[moveit_msgs.msg.AttachedCollisionObject\(link_name='[a-z_]+'|id='[a-z_]+'" 

# openrua op 51
cat > step4.py <<'EOF'
"""Step 4: planned transit to the pre-insert pose in front of the microwave (hand yawed 180 deg, pitch kept)."""
import numpy as np, sys
from plan import *; from ikbest import best_ik
r = Planner("step4")
tcp = np.array([float(v) for v in sys.argv[1:4]]) if len(sys.argv) >= 4 else np.array([-0.035, 0.12, 1.10])
approach = np.array([0, np.cos(np.radians(45)), -np.sin(np.radians(45))])   # toward +y and down
best = None
for fa in ([-1,0,0],[1,0,0]):
    R = R_from_axes(approach, fa)
    q = best_ik(r, tcp, R)
    print("finger axis", fa, "ik", None if q is None else np.round(q,3))
    if q is None: continue
    traj = r.plan_joints(q)
    if traj is None: continue
    pts = retime(traj); dur = pts[-1][1]
    print("  duration", round(dur,1))
    if best is None or dur < best[1]: best = (traj, dur)
if best is None: sys.exit("no plan")
r.execute(best[0])
p, RR = r.tcp(); print("tcp now", np.round(p,3), "approach", np.round(RR[:,2],3), "fingers", r.fingers())
EOF
timeout 900 python3 step4.py -0.035 0.12 1.10

# openrua op 52
timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent5.png >/dev/null & timeout 60 python3 tools/perception/cam_snap.py robot0_robotview snaps/robot5.png >/dev/null & wait; echo done

# openrua op 53
timeout 60 python3 heightmap.py > /dev/null; python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
# everything above 0.93 in x[-0.2,0.15], y[-0.1,0.3] (hand+white mug region and yellow mug)
m = (Z>0.93)&(X>-0.25)&(X<0.15)&(Y>-0.15)&(Y<0.26)
pts = P[m]
print("n", len(pts))
for zlo in np.arange(0.93,1.35,0.02):
    mm = m&(Z>=zlo)&(Z<zlo+0.02)
    if mm.sum()>3: print(f"z={zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
EOF

# openrua op 54
for c in agentview frontview sideview robot0_robotview; do timeout 30 python3 tools/perception/cam_snap.py $c snaps/${c}_6.png; done

# openrua op 55
timeout 60 python3 profile.py robot0_robotview -0.25 0.15 0.0 0.25 > /dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P = np.load("robot0_robotview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.92)&(Z<1.16)&(X>-0.25)&(X<0.15)&(Y>0.0)&(Y<0.25)
print("n",m.sum())
for zlo in np.arange(0.92,1.16,0.01):
    mm = m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum()>3: print(f"z={zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
EOF

# openrua op 56
python3 - <<'EOF'
import numpy as np
P = np.load("robot0_robotview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.92)&(Z<1.12)&(X>-0.15)&(X<0.10)&(Y>0.0)&(Y<0.22)
print("n",m.sum())
for zlo in np.arange(0.92,1.12,0.01):
    mm = m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum()>3: print(f"z={zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]  ymed={np.median(Y[mm]):.3f} xmed={np.median(X[mm]):.3f}")
# also per-y slices
print()
for ylo in np.arange(0.0,0.22,0.01):
    mm = m&(Y>=ylo)&(Y<ylo+0.01)&(Z>0.99)
    if mm.sum()>3: print(f"y={ylo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]")
EOF

# openrua op 57
python3 - <<'EOF'
import numpy as np
P = np.load("robot0_robotview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.985)&(Z<1.125)&(X>-0.12)&(X<0.06)&(Y>0.03)&(Y<0.18)
def ascii(A,B,a0,a1,b0,b1,res=0.005,C=None):
    na=int((a1-a0)/res); nb=int((b1-b0)/res)
    img=np.full((nb,na),' ')
    ia=((A-a0)/res).astype(int); ib=((B-b0)/res).astype(int)
    ok=(ia>=0)&(ia<na)&(ib>=0)&(ib<nb)
    if C is None:
        img[ib[ok],ia[ok]]='#'
    else:
        # depth-coded: letter by C (e.g. x) quantile
        for i,j,c in zip(ib[ok],ia[ok],C[ok]):
            img[i,j]=chr(ord('a')+int(min(max((c+0.12)/0.18*20,0),19)))
    for row in img[::-1]: print(''.join(row))
print("TOP-DOWN (x right, y up), letter = x (a=-0.12 .. t=0.06):")
ascii(X[m],Y[m],-0.12,0.06,0.03,0.18)
print("\nSIDE from -x (y right, z up), letter=x depth (a near -0.12 ... t far 0.06):")
ascii(Y[m],Z[m],0.03,0.18,0.985,1.125,C=X[m])
EOF

# openrua op 58
python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.905)&(Z<0.999)&(X>-0.30)&(X<0.20)&(Y>-0.20)&(Y<0.26)
res=0.01
a0,a1,b0,b1=-0.30,0.20,-0.20,0.26
na=int((a1-a0)/res); nb=int((b1-b0)/res)
img=np.full((nb,na),'.')
ia=((X[m]-a0)/res).astype(int); ib=((Y[m]-b0)/res).astype(int)
for i,j,z in zip(ib,ia,Z[m]):
    img[i,j]=str(min(int((z-0.90)*100),9))
print("top-down, x right from -0.30 to 0.20, y up from -0.20 to 0.26, digit=cm above table (0.905-0.999)")
for k,row in enumerate(img[::-1]): print(f"{b1-(k+1)*res:+.2f} "+''.join(row))
print("      "+''.join(str(int(abs(a0+i*res)*10)%10) for i in range(na)))
EOF

# openrua op 59
timeout 30 python3 tfdump.py 2>/dev/null | grep -iE "robotview|agentview|frontview|sideview|birdview" | head

# openrua op 60
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/attach.py", "old_string": "def attach_mug(r, mug_center_world, radius=0.05, height=0.12):\\n    hand_pos, R = r.solve_fk(r.joints())\\n    d_hand = R.T @ (np.asarray(mug_center_world) - hand_pos)       # mug centre in hand frame\\n    q = Rot.from_matrix(R.T).as_quat()                              # world-vertical axis expressed in hand frame", "new_string": "def attach_mug(r, mug_center_world, radius=0.05, height=0.12, axis_world=(0, 0, 1)):\\n    hand_pos, R = r.solve_fk(r.joints())\\n    d_hand = R.T @ (np.asarray(mug_center_world) - hand_pos)       # mug centre in hand frame\\n    # cylinder z-axis -> axis_world: build a world rotation whose z is the mug axis, express in hand frame\\n    z = np.asarray(axis_world, float); z /= np.linalg.norm(z)\\n    x = np.cross([0, 1, 0] if abs(z[1]) < 0.9 else [1, 0, 0], z); x /= np.linalg.norm(x)\\n    Rm = np.column_stack([x, np.cross(z, x), z])\\n    q = Rot.from_matrix(R.T @ Rm).as_quat()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 61
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/attach.py", "old_string": "    if sys.argv[1] == \\"attach\\": attach_mug(r, [float(v) for v in sys.argv[2:5]])", "new_string": "    if sys.argv[1] == \\"attach\\":\\n        ax = [float(v) for v in sys.argv[5:8]] if len(sys.argv) >= 8 else (0, 0, 1)\\n        attach_mug(r, [float(v) for v in sys.argv[2:5]], axis_world=ax)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 62
timeout 60 python3 attach.py attach -0.034 0.103 1.035 0 1 0 && timeout 60 python3 - <<'EOF'
import rclpy, scene
from rob import Robot
r = Robot("upd")
ok = scene.apply(r.node, [scene.remove("yellow_mug"), scene.box("yellow_mug", (0.085, 0.01, 0.947), (0.10, 0.15, 0.095))])
print("yellow updated", ok)
EOF

# openrua op 63
timeout 60 python3 - <<'EOF'
import rclpy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
rclpy.init(); n = rclpy.create_node("chk")
c = n.create_client(GetPlanningScene, "/get_planning_scene"); c.wait_for_service()
req = GetPlanningScene.Request(); req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
f = c.call_async(req); rclpy.spin_until_future_complete(n, f); s = f.result().scene
for a in s.robot_state.attached_collision_objects:
    print("ATT", a.object.id, [ (p.primitives[0].dimensions, round(pp.position.x,3), round(pp.position.y,3), round(pp.position.z,3), [round(v,3) for v in (pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w)]) for p,pp in [(a.object, a.object.primitive_poses[0])]])
for o in s.world.collision_objects:
    print("WORLD", o.id, o.primitives[0].type, list(o.primitives[0].dimensions), round(o.pose.position.x,3), round(o.pose.position.y,3), round(o.pose.position.z,3))
EOF

# openrua op 64
timeout 60 python3 - <<'EOF'
import rclpy
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
rclpy.init(); n = rclpy.create_node("chk")
c = n.create_client(GetPlanningScene, "/get_planning_scene"); c.wait_for_service()
req = GetPlanningScene.Request(); req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
f = c.call_async(req); rclpy.spin_until_future_complete(n, f); s = f.result().scene
for a in s.robot_state.attached_collision_objects:
    p = a.object.pose
    print("ATT", a.object.id, a.link_name, a.object.header.frame_id, (round(p.position.x,3), round(p.position.y,3), round(p.position.z,3)), [round(v,3) for v in (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)])
EOF

# openrua op 65
cat > /workspace/step5.py <<'EOF'
"""Carry hanging mug to clear area: up, then over to (-0.08,-0.22,1.16), same orientation."""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
r = Planner("step5")
tcp, R = r.solve_fk(r.joints()); tcp = tcp + TCP*R[:,2]
print("tcp now", np.round(tcp,3), "approach", np.round(R[:,2],3))
up = tcp.copy(); up[2] = 1.16
ok = r.move_line_tcp([up], R, avoid=True)
print("up ok", ok)
goal = np.array([-0.08, -0.22, 1.16])
q = best_ik(r, goal, R)
if q is None: sys.exit("no IK")
print("goto", np.round(q,3)); r.goto_joints(q)
tcp2 = r.tcp(); print("tcp after", np.round(tcp2,3), "fingers", np.round(r.fingers(),4))
EOF
timeout 900 python3 -u step5.py 2>&1 | tail -20

# openrua op 66
timeout 60 python3 -c "
from rob import *; import numpy as np
r=Robot('c'); p,R=r.solve_fk(r.joints()); print('tcp',np.round(p+TCP*R[:,2],3),'appr',np.round(R[:,2],3),'fingers',np.round(r.fingers(),4))" 2>&1 | tail -2; timeout 30 python3 tools/perception/cam_snap.py frontview snaps/frontview_7.png

# openrua op 67
cat > /workspace/step6.py <<'EOF'
"""Yaw hand 180 deg in place (approach -> (0,-.707,-.707)), keeping TCP."""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
r = Planner("step6")
tcp, R0 = r.solve_fk(r.joints()); tcp = tcp + TCP*R0[:,2]
appr = np.array([0, -np.cos(np.radians(45)), -np.sin(np.radians(45))])
best = None
for fa in ([1,0,0], [-1,0,0]):
    R = R_from_axes(appr, fa)
    q = best_ik(r, tcp, R)
    if q is None: continue
    traj = r.plan_joints(q)
    if traj is None: continue
    L = sum(abs(np.array(traj.points[-1].positions) - np.array(traj.points[0].positions)))
    print("fa", fa, "q", np.round(q,3), "pathlen", round(L,3))
    if best is None or L < best[0]: best = (L, traj, q)
if best is None: sys.exit("no plan")
r.execute(best[1])
p, R = r.solve_fk(r.joints()); print("tcp after", np.round(p+TCP*R[:,2],3), "appr", np.round(R[:,2],3), "fa", np.round(R[:,1],3), "fingers", np.round(r.fingers(),4))
EOF
timeout 900 python3 -u step6.py 2>&1 | grep -v "^  plan" | tail

# openrua op 68
timeout 30 python3 tools/perception/cam_snap.py frontview snaps/frontview_8.png >/dev/null; timeout 60 python3 profile.py robot0_robotview -0.25 0.1 -0.4 -0.05 > /dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P = np.load("robot0_robotview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.95)&(Z<1.2)&(X>-0.25)&(X<0.1)&(Y>-0.4)&(Y<-0.05)
print("n",m.sum())
for zlo in np.arange(0.98,1.2,0.01):
    mm = m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum()>3: print(f"z={zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
print()
for ylo in np.arange(-0.4,-0.05,0.01):
    mm = m&(Y>=ylo)&(Y<ylo+0.01)&(Z<1.14)
    if mm.sum()>3: print(f"y={ylo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] z[{Z[mm].min():.3f},{Z[mm].max():.3f}]")
EOF

# openrua op 69
timeout 60 python3 profile.py frontview -0.25 0.1 -0.4 -0.05 > /dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P = np.load("frontview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.95)&(Z<1.25)&(X>-0.25)&(X<0.1)&(Y>-0.4)&(Y<-0.05)
print("n",m.sum())
for zlo in np.arange(0.96,1.25,0.01):
    mm = m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum()>3: print(f"z={zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
EOF

# openrua op 70
timeout 60 python3 attach.py attach -0.080 -0.295 1.163 0 0 1 2>&1 | tail -1; timeout 30 python3 tools/perception/cam_snap.py agentview snaps/agent_8.png >/dev/null

# openrua op 71
cat > /workspace/step7.py <<'EOF'
"""Lower to z=1.05, then yaw 180 deg to approach (0,+.707,-.707) (insertion orientation)."""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
r = Planner("step7")
tcp, R0 = r.solve_fk(r.joints()); tcp = tcp + TCP*R0[:,2]
low = tcp.copy(); low[2] = 1.05
print("lower ok", r.move_line_tcp([low], R0, avoid=True))
tcp, R0 = r.solve_fk(r.joints()); tcp = tcp + TCP*R0[:,2]
appr = np.array([0, np.cos(np.radians(45)), -np.sin(np.radians(45))])
best = None
for fa in ([1,0,0], [-1,0,0]):
    R = R_from_axes(appr, fa)
    q = best_ik(r, tcp, R)
    if q is None: continue
    traj = r.plan_joints(q)
    if traj is None: continue
    L = sum(abs(np.array(traj.points[-1].positions) - np.array(traj.points[0].positions)))
    print("fa", fa, "q", np.round(q,3), "pathlen", round(L,3))
    if best is None or L < best[0]: best = (L, traj, q)
if best is None: sys.exit("no plan")
r.execute(best[1])
p, R = r.solve_fk(r.joints()); print("tcp after", np.round(p+TCP*R[:,2],3), "appr", np.round(R[:,2],3), "fa", np.round(R[:,1],3), "fingers", np.round(r.fingers(),4))
EOF
timeout 900 python3 -u step7.py 2>&1 | grep -v "^  plan" | tail

# openrua op 72
timeout 30 python3 tools/perception/cam_snap.py agentview snaps/agent_9.png >/dev/null; timeout 60 python3 profile.py frontview -0.25 0.1 -0.4 0.0 > /dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P = np.load("frontview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.90)&(Z<1.2)&(X>-0.25)&(X<0.1)&(Y>-0.4)&(Y<0.0)
for zlo in np.arange(0.90,1.2,0.01):
    mm = m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum()>3: print(f"z={zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
EOF

# openrua op 73
cat > /workspace/setdown.py <<'EOF'
"""Lower TCP by dz (arg), open gripper, retreat up 0.10."""
import sys, numpy as np
from rob import *; from plan import Planner
dz = float(sys.argv[1]) if len(sys.argv) > 1 else 0.03
r = Planner("setdown")
tcp, R = r.solve_fk(r.joints()); tcp = tcp + TCP*R[:,2]
low = tcp.copy(); low[2] -= dz
print("lower ok", r.move_line_tcp([low], R, avoid=False))
r.gripper(0.04)
tcp, R = r.solve_fk(r.joints()); tcp = tcp + TCP*R[:,2]
up = tcp.copy(); up[2] += 0.10
print("up ok", r.move_line_tcp([up], R, avoid=False), "fingers", np.round(r.fingers(),4))
EOF
timeout 600 python3 -u setdown.py 0.03 2>&1 | grep -v "^  plan" | tail -6

# openrua op 74
timeout 60 python3 attach.py detach -0.084 -0.227 0.947 2>&1 | tail -1; timeout 60 python3 heightmap.py > /dev/null 2>&1; timeout 30 python3 tools/perception/cam_snap.py agentview snaps/agent_10.png > /dev/null; python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.905)&(Z<1.05)&(X>-0.25)&(X<0.05)&(Y>-0.40)&(Y<-0.10)
print("n", m.sum(), "zmax", Z[m].max())
res=0.01; a0,a1,b0,b1=-0.25,0.05,-0.40,-0.10
na=int((a1-a0)/res); nb=int((b1-b0)/res); img=np.full((nb,na),'.')
ia=((X[m]-a0)/res).astype(int); ib=((Y[m]-b0)/res).astype(int)
for i,j,z in zip(ib,ia,Z[m]): img[i,j]=str(min(int((z-0.90)*100),9))
for k,row in enumerate(img[::-1]): print(f"{b1-(k+1)*res:+.2f} "+''.join(row))
print("      "+''.join(str(int(abs(a0+i*res)*10)%10) for i in range(na)))
EOF

# openrua op 75
python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.91)&(X>-0.20)&(X<0.0)&(Y>-0.35)&(Y<-0.10)
body = m&(Z<0.996)
print("body y range", Y[body].min(), Y[body].max(), " x range", X[body].min(), X[body].max())
top = m&(Z>0.985)&(Z<0.996)
print("ridge x mean", X[top].mean(), "z max body", Z[body].max())
hd = m&(Z>=0.996)
print("handle: n", hd.sum(), "x", X[hd].min(), X[hd].max(), "y", Y[hd].min(), Y[hd].max(), "z", Z[hd].max())
# per-y max z along ridge
for ylo in np.arange(-0.29,-0.15,0.01):
    mm = m&(Y>=ylo)&(Y<ylo+0.01)
    if mm.sum(): print(f"y={ylo:.2f} zmax={Z[mm].max():.3f} x[{X[mm].min():.3f},{X[mm].max():.3f}]")
EOF

# openrua op 76
grep -n "def cyl\|def box\|def remove\|def apply" -A6 scene.py | head -60

# openrua op 77
python3 - <<'EOF'
import re
s = open("scene.py").read()
s = s.replace('''def cyl(name, center, radius, height):
    co = CollisionObject(); co.header.frame_id = "world"; co.id = name
    sp = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(height), float(radius)])
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center); p.orientation.w = 1.0''',
'''def cyl(name, center, radius, height, axis=(0, 0, 1)):
    co = CollisionObject(); co.header.frame_id = "world"; co.id = name
    sp = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[float(height), float(radius)])
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center)
    z = np.asarray(axis, float) / np.linalg.norm(axis)
    x = np.cross([0, 1, 0] if abs(z[1]) < 0.9 else [1, 0, 0], z); x /= np.linalg.norm(x)
    q = Rot.from_matrix(np.column_stack([x, np.cross(z, x), z])).as_quat()
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)''')
if "import numpy as np" not in s: s = "import numpy as np\n" + s
open("scene.py","w").write(s)
EOF
timeout 60 python3 - <<'EOF'
import scene
from rob import Robot
r = Robot("upd")
print(scene.apply(r.node, [scene.remove("white_mug"), scene.cyl("white_mug", (-0.090, -0.220, 0.949), 0.048, 0.115, axis=(0,1,0))]))
EOF

# openrua op 78
cat > /workspace/step8.py <<'EOF'
"""Rim grasp of lying mug: 45deg-pitched fingers from +y, pinch rim wall at its -x point.
Args: x_wall y_rim z_c  (wall centre x, rim plane y, mug axis height)."""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
xw, yr, zc = map(float, sys.argv[1:4])
r = Planner("step8")
appr = np.array([0, -np.cos(np.radians(45)), -np.sin(np.radians(45))])
R = R_from_axes(appr, [1, 0, 0])
target = np.array([xw, yr - 0.012, zc])
pre = target - 0.08 * appr
print("target", np.round(target,3), "pre", np.round(pre,3))
if abs(r.fingers()[0]) < 0.035: r.gripper(0.04)
q = best_ik(r, pre, R)
if q is None: sys.exit("no IK for pre")
r.goto_joints(q)
print("pre tcp", np.round(r.tcp(),3))
ok = r.move_line_tcp([target], R, avoid=False)
print("approach ok", ok, "tcp", np.round(r.tcp(),3))
r.gripper(0.0)
print("fingers", np.round(r.fingers(),4))
EOF
timeout 900 python3 -u step8.py -0.1335 -0.164 0.949 2>&1 | grep -v "^  plan" | tail -12

# openrua op 79
grep -n "def tcp" -A4 rob.py

# openrua op 80
sed -i 's/np.round(r.tcp(),3)/np.round(r.tcp()[0],3)/g' step8.py && cat > /workspace/step8b.py <<'EOF'
"""Resume: approach along line to target and close."""
import sys, numpy as np
from rob import *; from plan import Planner
xw, yr, zc = map(float, sys.argv[1:4])
r = Planner("step8b")
appr = np.array([0, -np.cos(np.radians(45)), -np.sin(np.radians(45))])
R = R_from_axes(appr, [1, 0, 0])
target = np.array([xw, yr - 0.012, zc])
print("pre tcp", np.round(r.tcp()[0],3), "fingers", np.round(r.fingers(),4))
ok = r.move_line_tcp([target], R, avoid=False)
print("approach ok", ok, "tcp", np.round(r.tcp()[0],3))
r.gripper(0.0)
print("fingers", np.round(r.fingers(),4))
EOF
timeout 600 python3 -u step8b.py -0.1335 -0.164 0.949 2>&1 | grep -v "^  plan" | tail -8

# openrua op 81
timeout 30 python3 tools/perception/cam_snap.py agentview snaps/agent_11.png > /dev/null; timeout 30 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_11.png > /dev/null; echo ok

# openrua op 82
cat > /workspace/lift.py <<'EOF'
import sys, numpy as np
from rob import *; from plan import Planner
dz = float(sys.argv[1])
r = Planner("lift")
tcp, R = r.tcp(); up = tcp.copy(); up[2] += dz
print("lift ok", r.move_line_tcp([up], R, avoid=False), "tcp", np.round(r.tcp()[0],3), "fingers", np.round(r.fingers(),4))
EOF
timeout 300 python3 -u lift.py 0.04 2>&1 | tail -1; timeout 60 python3 profile.py frontview -0.25 0.0 -0.35 -0.10 > /dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P = np.load("frontview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.905)&(Z<1.10)&(X>-0.25)&(X<0.0)&(Y>-0.35)&(Y<-0.10)
for zlo in np.arange(0.90,1.10,0.01):
    mm = m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum()>3: print(f"z={zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
EOF

# openrua op 83
grep -n "def cartesian\|def move_line_tcp" -A12 plan.py | head -50

# openrua op 84
cat > /workspace/step9.py <<'EOF'
"""Rim grasp of lying mug at its upper-left (45deg) rim point.
Args: x_c y_rim z_c. finger axis f=(-.707,0,.707), approach a=(-.5,-.707,-.5)."""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
xc, yr, zc = map(float, sys.argv[1:4])
r = Planner("step9")
f = np.array([-1, 0, 1]) / np.sqrt(2)
a = np.array([-0.5, -np.sqrt(0.5), -0.5])
R = R_from_axes(a, f)
target = np.array([xc - 0.0435*0.707, yr - 0.02, zc + 0.0435*0.707])
pre = target - 0.08 * a
print("target", np.round(target,3), "pre", np.round(pre,3))
if abs(r.fingers()[0]) < 0.035: r.gripper(0.04)
q = best_ik(r, pre, R)
if q is None: sys.exit("no IK for pre")
r.goto_joints(q)
print("pre tcp", np.round(r.tcp()[0],3))
ok = r.move_line_tcp([target], R, avoid=False)
print("approach ok", ok, "tcp", np.round(r.tcp()[0],3))
r.gripper(0.0)
print("fingers", np.round(r.fingers(),4))
EOF
timeout 900 python3 -u step9.py -0.0885 -0.165 0.949 2>&1 | grep -v "^  plan" | tail -10

# openrua op 85
timeout 300 python3 -u lift.py 0.04 2>&1 | tail -1; timeout 60 python3 profile.py frontview -0.25 0.0 -0.35 -0.10 > /dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P = np.load("frontview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.905)&(Z<1.10)&(X>-0.25)&(X<0.0)&(Y>-0.35)&(Y<-0.10)
for zlo in np.arange(0.90,1.10,0.01):
    mm = m&(Z>=zlo)&(Z<zlo+0.01)
    if mm.sum()>3: print(f"z={zlo:.2f}: n={mm.sum():4d} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
EOF

# openrua op 86
timeout 30 python3 tools/perception/cam_snap.py agentview snaps/agent_12.png > /dev/null; timeout 30 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_12.png > /dev/null; echo ok

# openrua op 87
cat > /workspace/moveto.py <<'EOF'
"""Cartesian TCP move keeping orientation: python3 moveto.py x y z [avoid=1]"""
import sys, numpy as np
from rob import *; from plan import Planner
r = Planner("moveto")
goal = np.array([float(v) for v in sys.argv[1:4]]); avoid = (sys.argv[4] != "0") if len(sys.argv) > 4 else True
tcp, R = r.tcp()
print("ok", r.move_line_tcp([goal], R, avoid=avoid), "tcp", np.round(r.tcp()[0],3))
EOF
timeout 300 python3 -u moveto.py -0.12 -0.05 1.25 0 2>&1 | tail -1; timeout 60 python3 heightmap.py > /dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.905)&(Z<1.10)&(X>-0.25)&(X<0.05)&(Y>-0.40)&(Y<-0.10)
res=0.005; a0,a1,b0,b1=-0.20,0.02,-0.36,-0.14
na=int((a1-a0)/res); nb=int((b1-b0)/res); img=np.full((nb,na),'.')
ia=((X[m]-a0)/res).astype(int); ib=((Y[m]-b0)/res).astype(int)
ok=(ia>=0)&(ia<na)&(ib>=0)&(ib<nb)
for i,j,z in zip(ib[ok],ia[ok],Z[m][ok]): img[i,j]=str(min(int((z-0.90)*100),9))
for k,row in enumerate(img[::-1]): print(f"{b1-(k+1)*res:+.3f} "+''.join(row))
print("       "+''.join(str(int(round(abs(a0+i*res)*100))%10) for i in range(na)))
EOF

# openrua op 88
python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.915)&(Z<1.05)&(X>-0.20)&(X<0.02)&(Y>-0.36)&(Y<-0.14)
pts = np.stack([X[m],Y[m]],1); c = pts.mean(0)
u,s,vt = np.linalg.svd(pts-c, full_matrices=False); ax = vt[0]; 
if ax[1] < 0: ax = -ax
perp = np.array([-ax[1], ax[0]])
t = (pts-c)@ax; w = (pts-c)@perp
print("centroid", c.round(3), "axis", ax.round(3), "yaw from +y deg", np.degrees(np.arctan2(ax[0], ax[1])).round(1))
print("along-axis range", t.min().round(3), t.max().round(3), " perp range", w.min().round(3), w.max().round(3))
# top ridge (highest points) -> axis line position
top = Z[m] > 0.985
print("ridge perp mean", w[top].mean().round(4), "ridge z", Z[m][top].mean().round(4))
# handle: points with perp offset beyond radius
h = np.abs(w - w[top].mean()) > 0.05
print("handle candidates n", h.sum(), "perp", w[h].min().round(3) if h.sum() else None, w[h].max().round(3) if h.sum() else None, "along", t[h].min().round(3) if h.sum() else None, t[h].max().round(3) if h.sum() else None)
# profile of half-width along axis
for tl in np.arange(t.min(), t.max(), 0.01):
    mm = (t>=tl)&(t<tl+0.01)
    if mm.sum(): print(f"t={tl:+.3f}: perp[{w[mm].min():+.3f},{w[mm].max():+.3f}] zmax={Z[m][mm].max():.3f}")
EOF

# openrua op 89
timeout 60 python3 profile.py frontview -0.25 0.05 -0.40 -0.10 > /dev/null 2>&1; timeout 30 python3 tools/perception/cam_snap.py frontview snaps/front_13.png >/dev/null; python3 - <<'EOF'
import numpy as np
P = np.load("frontview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.905)&(Z<1.04)&(X>-0.25)&(X<0.05)&(Y>-0.40)&(Y<-0.10)
c = np.array([-0.071,-0.243]); ax = np.array([0.442,0.897]); perp = np.array([-ax[1],ax[0]])
pts = np.stack([X[m],Y[m]],1); t=(pts-c)@ax; w=(pts-c)@perp; z=Z[m]
for tl in np.arange(-0.08,0.10,0.01):
    mm=(t>=tl)&(t<tl+0.01)
    if mm.sum()>3: print(f"t={tl:+.3f}: n={mm.sum():4d} perp[{w[mm].min():+.3f},{w[mm].max():+.3f}] z[{z[mm].min():.3f},{z[mm].max():.3f}]")
EOF

# openrua op 90
python3 - <<'EOF'
import numpy as np
c = np.array([-0.071,-0.243]); ax = np.array([0.442,0.897]); perp = np.array([-ax[1],ax[0]])
for cam in ["birdview","frontview"]:
    P = np.load(f"{cam}_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
    m = np.isfinite(Z)&(Z>1.008)&(Z<1.05)&(X>-0.20)&(X<0.02)&(Y>-0.36)&(Y<-0.14)
    pts = np.stack([X[m],Y[m]],1); t=(pts-c)@ax; w=(pts-c)@perp
    print(cam, "bar top pts n", m.sum(), "t[%.3f,%.3f]"%(t.min(),t.max()), "w[%.3f,%.3f] wmean %.4f"%(w.min(),w.max(),w.mean()), "zmax %.3f"%Z[m].max())
    # body top ridge (z in 0.985..1.0), exclude handle region
    mb = np.isfinite(Z)&(Z>0.985)&(Z<1.0)&(X>-0.20)&(X<0.02)&(Y>-0.36)&(Y<-0.14)
    pts = np.stack([X[mb],Y[mb]],1); t=(pts-c)@ax; w=(pts-c)@perp
    print(cam, "body ridge w mean %.4f"%w.mean(), "t range %.3f %.3f"%(t.min(),t.max()))
EOF

# openrua op 91
timeout 60 python3 - <<'EOF'
import scene
from rob import Robot
r = Robot("upd")
print(scene.apply(r.node, [scene.remove("white_mug"), scene.cyl("white_mug", (-0.071,-0.243,0.9465), 0.048, 0.115, axis=(0.442,0.897,0))]))
EOF
cat > /workspace/step10.py <<'EOF'
"""Grasp handle bar of lying mug from above. Args: bx by bz tx ty (bar centre, mug axis dir toward rim)."""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
bx, by, bz, tx, ty = map(float, sys.argv[1:6])
r = Planner("step10")
t = np.array([tx, ty, 0]); t /= np.linalg.norm(t)
perp = np.array([-t[1], t[0], 0])
lean = np.radians(20)
a = np.sin(lean)*t + np.array([0, 0, -np.cos(lean)])      # fingers point down and toward rim; hand leans toward base
R = R_from_axes(a, perp)
target = np.array([bx, by, bz]); pre = target - 0.07*a
print("target", target.round(3), "pre", pre.round(3), "a", a.round(3))
if abs(r.fingers()[0]) < 0.035: r.gripper(0.04)
q = best_ik(r, pre, R)
if q is None: sys.exit("no IK")
r.goto_joints(q); print("pre tcp", np.round(r.tcp()[0],3))
print("approach ok", r.move_line_tcp([target], R, avoid=False))
r.gripper(0.0); print("fingers", np.round(r.fingers(),4))
up = target.copy(); up[2] += 0.05
print("lift ok", r.move_line_tcp([up], R, avoid=False), "fingers", np.round(r.fingers(),4))
EOF
timeout 900 python3 -u step10.py -0.065 -0.2425 1.021 0.442 0.897 2>&1 | grep -v "^  plan" | tail -12

# openrua op 92
timeout 60 python3 heightmap.py > /dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P = np.load("birdview_world.npy"); X,Y,Z = P[...,0],P[...,1],P[...,2]
m = np.isfinite(Z)&(Z>0.905)&(Z<1.06)&(X>-0.25)&(X<0.05)&(Y>-0.40)&(Y<-0.12)
print("n", m.sum(), "z range", Z[m].min().round(3) if m.sum() else None, Z[m].max().round(3) if m.sum() else None)
if m.sum(): print("x", X[m].min().round(3), X[m].max().round(3), "y", Y[m].min().round(3), Y[m].max().round(3))
EOF

# openrua op 93
cat > /workspace/rotz.py <<'EOF'
"""Rotate hand about world z through the TCP by angle_deg (Cartesian, smooth)."""
import sys, numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import *; from plan import Planner
ang = np.radians(float(sys.argv[1])); n = int(sys.argv[2]) if len(sys.argv) > 2 else 8
r = Planner("rotz")
tcp, R0 = r.tcp()
targets = []
for k in range(1, n+1):
    Rk = Rot.from_rotvec([0, 0, ang*k/n]).as_matrix() @ R0
    targets.append((hand_pose_from_tcp(tcp, Rk), Rk))
traj, frac = r.cartesian(targets, avoid=False, step=0.005)
print("fraction", frac)
if traj is None or frac < 0.99: sys.exit("cartesian failed")
r.execute(traj)
p, R = r.tcp(); print("tcp", p.round(3), "finger axis", R[:,1].round(3), "approach", R[:,2].round(3), "fingers", np.round(r.fingers(),4))
EOF
timeout 600 python3 -u rotz.py -26.2 8 2>&1 | grep -v "^  plan" | tail -3

# openrua op 94
timeout 120 python3 heightmap.py 2>&1 | tail -2; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.2)&(P[:,0]<0.1)&(P[:,1]>-0.4)&(P[:,1]<-0.1)&(P[:,2]>0.95)
Q=P[m]; print(len(Q), Q.min(0).round(3), Q.max(0).round(3))
# principal direction in xy
c=Q[:,:2].mean(0); u,s,vt=np.linalg.svd(Q[:,:2]-c); print("center",c.round(3),"dir",vt[0].round(3), "spread",s.round(2))
EOF

# openrua op 95
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.2)&(P[:,0]<0.1)&(P[:,1]>-0.4)&(P[:,1]<-0.1)&(P[:,2]>0.95)&(P[:,2]<1.075)
Q=P[m]; print(len(Q), Q.min(0).round(3), Q.max(0).round(3))
c=Q[:,:2].mean(0); u,s,vt=np.linalg.svd(Q[:,:2]-c); print("center",c.round(3),"dir",vt[0].round(3), "spread",s.round(2))
# histogram along the principal direction
d=(Q[:,:2]-c)@vt[0]; print(np.histogram(d,bins=10)[0], d.min().round(3), d.max().round(3))
d2=(Q[:,:2]-c)@vt[1]; print("width", d2.min().round(3), d2.max().round(3))
EOF

# openrua op 96
timeout 120 python3 profile.py frontview -0.3 0.1 -0.4 -0.1 2>&1 | tail -3; python3 - <<'EOF'
import numpy as np
P=np.load('frontview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.2)&(P[:,0]<0.1)&(P[:,1]>-0.4)&(P[:,1]<-0.1)&(P[:,2]>0.93)&(P[:,2]<1.09)
Q=P[m]; print(len(Q), Q.min(0).round(3), Q.max(0).round(3))
# slice by z
for z0 in np.arange(0.93,1.09,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 97
timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird_now.png && timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_now.png && python3 -c "
import cv2; im=cv2.imread('snaps/bird_now.png'); print(im.shape)
# crop around the mug: birdview centroid px for x=-0.05,y=-0.23 ... find via world npy
import numpy as np; P=np.load('birdview_world.npy'); H,W=P.shape[:2]
m=(P[...,0]>-0.2)&(P[...,0]<0.1)&(P[...,1]>-0.4)&(P[...,1]<-0.1)&(P[...,2]>0.95)&(P[...,2]<1.075)
vs,us=np.where(m); print(us.min(),us.max(),vs.min(),vs.max())
crop=im[vs.min()-60:vs.max()+60, us.min()-60:us.max()+60]; cv2.imwrite('snaps/bird_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 98
python3 -c "
import cv2, numpy as np; im=cv2.imread('snaps/front_now.png'); P=np.load('frontview_world.npy'); print(P.shape, im.shape)
m=(P[...,0]>-0.2)&(P[...,0]<0.1)&(P[...,1]>-0.4)&(P[...,1]<-0.1)&(P[...,2]>0.93)&(P[...,2]<1.09)
vs,us=np.where(m); print(us.min(),us.max(),vs.min(),vs.max())
crop=im[max(vs.min()-80,0):vs.max()+40, us.min()-60:us.max()+60]; cv2.imwrite('snaps/front_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 99
python3 - <<'EOF'
import numpy as np
P=np.load('frontview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
for (x0,x1,y0,y1) in [(-0.2,-0.1,-0.4,-0.3),(-0.1,0.0,-0.4,-0.35),(0.0,0.1,-0.3,-0.2),(-0.1,0,-0.15,-0.1)]:
    m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1)&(P[:,2]<0.95)
    Q=P[m]; print((x0,x1,y0,y1), len(Q), np.percentile(Q[:,2],[5,50,95]).round(4) if len(Q) else None)
EOF

# openrua op 100
timeout 600 python3 -u rotz.py 52.4 12 2>&1 | grep -v "^  plan" | tail -3

# openrua op 101
timeout 120 python3 profile.py frontview -0.3 0.1 -0.45 -0.05 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
P=np.load('frontview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.2)&(P[:,0]<0.1)&(P[:,1]>-0.45)&(P[:,1]<-0.05)&(P[:,2]>0.93)&(P[:,2]<1.10)
Q=P[m]; print(len(Q), Q.min(0).round(3), Q.max(0).round(3))
for z0 in np.arange(0.93,1.10,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
print("--- by y slice (x,z extents)")
for y0 in np.arange(-0.34,-0.16,0.01):
    s=Q[(Q[:,1]>=y0)&(Q[:,1]<y0+0.01)]
    if len(s): print(f"y {y0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
EOF
python3 -c "
import cv2, numpy as np; im=cv2.imread('snaps/front_now.png')
import subprocess; subprocess.run(['python3','tools/perception/cam_snap.py','frontview','snaps/front_now.png'],capture_output=True)
im=cv2.imread('snaps/front_now.png'); P=np.load('frontview_world.npy')
m=(P[...,0]>-0.2)&(P[...,0]<0.1)&(P[...,1]>-0.45)&(P[...,1]<-0.05)&(P[...,2]>0.93)&(P[...,2]<1.09)
vs,us=np.where(m); crop=im[max(vs.min()-80,0):vs.max()+40, us.min()-60:us.max()+60]; cv2.imwrite('snaps/front_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 102
python3 - <<'EOF'
import numpy as np
P=np.load('frontview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.3)&(P[:,0]<0.2)&(P[:,2]>0.85)&(P[:,2]<0.95)
Q=P[m]; print("table-ish points y range", Q[:,1].min().round(3), Q[:,1].max().round(3))
for y0 in np.arange(-0.7,-0.3,0.05):
    s=Q[(Q[:,1]>=y0)&(Q[:,1]<y0+0.05)]
    print(f"y {y0:.2f}: n={len(s)} z med {np.median(s[:,2]).round(4) if len(s) else None}")
EOF

# openrua op 103
cat > /workspace/right.py <<'EOF'
"""Right the hanging mug: lower until base edge ~2mm above table, then move pinch on an arc about the base edge."""
import sys, numpy as np
from rob import *; from plan import Planner
a_len = float(sys.argv[1])      # pinch -> base plane distance along axis (m)
axis_z = float(sys.argv[2])     # current mug axis height
TABLE = 0.9017; R_MUG = 0.0465
r = Planner("right")
tcp, R = r.tcp(); x0, y0, z0 = tcp
E = np.array([x0, y0 - a_len, TABLE + 0.002])       # pivot: base bottom edge (after lowering)
drop = (axis_z - R_MUG) - E[2]
print("lower by", round(drop,4))
traj, f = r.move_line_tcp([tcp - [0, 0, drop]], R, avoid=False)
p1 = r.tcp()[0]; print("after lower tcp", p1.round(4), "fingers", np.round(r.fingers(),4))
rel = p1 - E; rad = np.hypot(rel[1], rel[2]); th0 = np.arctan2(rel[2], rel[1])
print("radius", round(rad,4), "theta0", round(np.degrees(th0),1))
pts = []
for k in range(1, 19):
    th = th0 + np.radians(90) * k / 18
    pts.append(E + [0, rad*np.cos(th), rad*np.sin(th)])
print("final pinch", pts[-1].round(4))
traj, f = r.move_line_tcp(pts, R, avoid=False, min_fraction=0.95)
p2 = r.tcp()[0]; print("after arc tcp", p2.round(4), "fingers", np.round(r.fingers(),4))
EOF
grep -n "def move_line_tcp" -A 12 plan.py

# openrua op 104
sed -i 's/traj, f = r.move_line_tcp(\[tcp - \[0, 0, drop\]\], R, avoid=False)/ok = r.move_line_tcp([tcp - [0, 0, drop]], R, avoid=False); print("lower ok", ok)/; s/traj, f = r.move_line_tcp(pts, R, avoid=False, min_fraction=0.95)/ok = r.move_line_tcp(pts, R, avoid=False, min_fraction=0.95); print("arc ok", ok)/' right.py && timeout 900 python3 -u right.py 0.062 0.999 2>&1 | grep -v "^  plan" | tail -12

# openrua op 105
timeout 120 python3 profile.py frontview -0.3 0.1 -0.6 -0.2 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_now.png >/dev/null; python3 - <<'EOF'
import numpy as np, cv2
P=np.load('frontview_world.npy'); im=cv2.imread('snaps/front_now.png')
m=(P[...,0]>-0.25)&(P[...,0]<0.15)&(P[...,1]>-0.6)&(P[...,1]<-0.25)&(P[...,2]>0.905)&(P[...,2]<1.06)
vs,us=np.where(m); print(us.min(),us.max(),vs.min(),vs.max())
crop=im[max(vs.min()-100,0):vs.max()+40, us.min()-60:us.max()+60]; cv2.imwrite('snaps/front_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
m=(Q[:,0]>-0.25)&(Q[:,0]<0.15)&(Q[:,1]>-0.6)&(Q[:,1]<-0.25)&(Q[:,2]>0.905)&(Q[:,2]<1.06); Q=Q[m]
for z0 in np.arange(0.905,1.06,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.3f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 106
cat > /workspace/carry.py <<'EOF'
"""Cartesian TCP moves keeping orientation: carry.py dx dy dz [dx dy dz ...] (relative steps)."""
import sys, numpy as np
from rob import *; from plan import Planner
r = Planner("carry"); v = [float(a) for a in sys.argv[1:]]
for i in range(0, len(v), 3):
    tcp, R = r.tcp(); tgt = tcp + np.array(v[i:i+3])
    ok = r.move_line_tcp([tgt], R, avoid=False); print("step", v[i:i+3], "ok", ok, "fingers", np.round(r.fingers(),4))
EOF
timeout 600 python3 -u carry.py 0 0 0.06  -0.03 0.19 0 2>&1 | grep -v "^  plan" | grep -v cartesian

# openrua op 107
timeout 120 python3 profile.py frontview -0.3 0.1 -0.45 -0.05 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
P=np.load('frontview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.2)&(P[:,0]<0.1)&(P[:,1]>-0.45)&(P[:,1]<-0.05)&(P[:,2]>0.93)&(P[:,2]<1.10)
Q=P[m]; print(len(Q), Q.min(0).round(3), Q.max(0).round(3))
for y0 in np.arange(-0.32,-0.14,0.01):
    s=Q[(Q[:,1]>=y0)&(Q[:,1]<y0+0.01)]
    if len(s): print(f"y {y0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
EOF

# openrua op 108
cat > /workspace/right2.py <<'EOF'
"""Right the hanging mug with pressing: right2.py a_len axis_z delta0 [n]."""
import sys, numpy as np
from rob import *; from plan import Planner
a_len, axis_z, d0 = map(float, sys.argv[1:4]); n = int(sys.argv[4]) if len(sys.argv) > 4 else 18
TABLE = 0.9017; R_MUG = 0.0465
r = Planner("right2")
tcp, R = r.tcp(); x0, y0, z0 = tcp
E = np.array([x0, y0 - a_len, TABLE])
drop = (axis_z - R_MUG) - TABLE
print("lower by", round(drop,4))
ok = r.move_line_tcp([tcp - [0, 0, drop]], R, avoid=False); print("lower ok", ok)
p1 = r.tcp()[0]; print("after lower tcp", p1.round(4), "fingers", np.round(r.fingers(),4))
rel = p1 - E; rad = np.hypot(rel[1], rel[2]); th0 = np.arctan2(rel[2], rel[1])
print("radius", round(rad,4), "theta0", round(np.degrees(th0),1))
pts = []
for k in range(1, n+1):
    th = th0 + np.radians(90) * k / n
    rk = rad - d0 * (1 - k / n)
    pts.append(E + [0, rk*np.cos(th), rk*np.sin(th)])
print("final pinch", pts[-1].round(4))
ok = r.move_line_tcp(pts, R, avoid=False, min_fraction=0.95); print("arc ok", ok)
p2 = r.tcp()[0]; print("after arc tcp", p2.round(4), "fingers", np.round(r.fingers(),4))
EOF
timeout 900 python3 -u right2.py 0.05 1.0015 0.012 2>&1 | grep -v "^  plan" | grep -v cartesian | tail -9

# openrua op 109
timeout 120 python3 profile.py frontview -0.3 0.1 -0.6 -0.2 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_now.png >/dev/null; python3 - <<'EOF'
import numpy as np, cv2
P=np.load('frontview_world.npy'); im=cv2.imread('snaps/front_now.png')
m=(P[...,0]>-0.25)&(P[...,0]<0.15)&(P[...,1]>-0.6)&(P[...,1]<-0.25)&(P[...,2]>0.905)&(P[...,2]<1.06)
vs,us=np.where(m)
crop=im[max(vs.min()-100,0):vs.max()+40, us.min()-60:us.max()+60]; cv2.imwrite('snaps/front_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
m=(Q[:,0]>-0.25)&(Q[:,0]<0.15)&(Q[:,1]>-0.6)&(Q[:,1]<-0.25)&(Q[:,2]>0.905)&(Q[:,2]<1.06); Q=Q[m]
for z0 in np.arange(0.905,1.06,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.3f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 110
cat > /workspace/release.py <<'EOF'
import sys, numpy as np
from rob import *; from plan import Planner
r = Planner("release"); r.gripper(0.04)
tcp, R = r.tcp(); a = R[:,2]
ok = r.move_line_tcp([tcp - 0.06*a], R, avoid=False); print("retreat ok", ok)
tcp, R = r.tcp(); ok = r.move_line_tcp([tcp + [0,0,0.08]], R, avoid=False); print("up ok", ok)
EOF
timeout 600 python3 -u release.py 2>&1 | grep -v "^  plan" | grep -v cartesian | tail -5

# openrua op 111
timeout 120 python3 heightmap.py 2>&1 | grep -i "blob" ; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.25)&(P[:,0]<0.15)&(P[:,1]>-0.6)&(P[:,1]<-0.2)&(P[:,2]>0.92)&(P[:,2]<1.05)
Q=P[m]; print(len(Q), Q.min(0).round(3), Q.max(0).round(3))
top=Q[Q[:,2]>1.0]; print("top ring n",len(top),"center",top[:,:2].mean(0).round(4),"z",np.percentile(top[:,2],[5,50,95]).round(4), "x[%.3f,%.3f] y[%.3f,%.3f]"%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max()))
# fit circle to rim points (z>1.005)
from numpy.linalg import lstsq
A=np.c_[2*top[:,0],2*top[:,1],np.ones(len(top))]; b=(top[:,:2]**2).sum(1); c=lstsq(A,b,rcond=None)[0]; print("circle center",c[:2].round(4),"r",np.sqrt(c[2]+c[0]**2+c[1]**2).round(4))
EOF

# openrua op 112
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.2)&(P[:,0]<0.05)&(P[:,1]>-0.5)&(P[:,1]<-0.2)&(P[:,2]>0.91)
Q=P[m]
for z0 in np.arange(0.91,1.02,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
hi=Q[Q[:,2]>0.99]; print("all >0.99: n",len(hi), "x[%.3f,%.3f] y[%.3f,%.3f]"%(hi[:,0].min(),hi[:,0].max(),hi[:,1].min(),hi[:,1].max()), "mean", hi[:,:2].mean(0).round(4))
EOF
timeout 120 python3 profile.py frontview -0.3 0.1 -0.5 -0.2 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
Q=np.load('frontview_world.npy').reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
m=(Q[:,0]>-0.25)&(Q[:,0]<0.15)&(Q[:,1]>-0.5)&(Q[:,1]<-0.2)&(Q[:,2]>0.905)&(Q[:,2]<1.03); Q=Q[m]
for z0 in np.arange(0.905,1.03,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.3f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]  ymid {(s[:,1].min()+s[:,1].max())/2:.4f}")
EOF

# openrua op 113
cat > /workspace/grasp45.py <<'EOF'
"""Grasp vertical handle bar from -y with 45deg lean: grasp45.py bx by bz [lift]"""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
bx, by, bz = map(float, sys.argv[1:4]); lift = float(sys.argv[4]) if len(sys.argv) > 4 else 0.03
r = Planner("grasp45")
a = np.array([0, np.sqrt(.5), -np.sqrt(.5)]); R = R_from_axes(a, [1,0,0])
target = np.array([bx, by, bz]); pre = target - 0.08*a
r.gripper(0.04)
q = best_ik(r, pre, R); assert q is not None, "no IK for pregrasp"
print("goto pregrasp"); r.goto_joints(q); print("tcp", r.tcp()[0].round(4))
ok = r.move_line_tcp([target], R, avoid=False); print("approach ok", ok)
r.gripper(0.0)
ok = r.move_line_tcp([target + [0,0,lift]], R, avoid=False); print("lift ok", ok, "fingers", np.round(r.fingers(),4))
EOF
timeout 900 python3 -u grasp45.py -0.065 -0.399 0.958 0.03 2>&1 | grep -v "^  plan" | grep -v cartesian | tail -8

# openrua op 114
cat ikbest.py; grep -n "def goto_joints" -A 8 plan.py

# openrua op 115
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^  plan"
import numpy as np
from rob import *; from ikbest import best_ik
r = Robot("iktest")
for lean in [45, 40, 35, 30]:
    b = np.radians(lean); a = np.array([0, np.sin(b), -np.cos(b)]); R = R_from_axes(a, [1,0,0])
    target = np.array([-0.065, -0.399, 0.958]); pre = target - 0.08*a
    q = best_ik(r, pre, R, tries=30)
    print(lean, "pre", pre.round(3), "IK", None if q is None else np.round(q,2))
    if q is not None:
        q2 = best_ik(r, target, R, seeds=[q], tries=10); print("   target IK", None if q2 is None else np.round(q2,2))
EOF

# openrua op 116
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^  plan"
import numpy as np
from rob import *; from ikbest import best_ik
r = Robot("iktest")
for lean in [25, 20, 15, 10, 0]:
    b = np.radians(lean); a = np.array([0, np.sin(b), -np.cos(b)]); R = R_from_axes(a, [1,0,0])
    target = np.array([-0.065, -0.399, 0.958]); pre = target - 0.08*a
    q = best_ik(r, pre, R, tries=30)
    print(lean, "pre", pre.round(3), "IK", None if q is None else np.round(q,2))
    if q is not None:
        q2 = best_ik(r, target, R, seeds=[q], tries=10); print("   target IK", None if q2 is None else np.round(q2,2))
EOF

# openrua op 117
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^  plan"
import numpy as np
from rob import *; from ikbest import best_ik
r = Robot("iktest")
b = np.radians(45); a = np.array([0, np.sin(b), -np.cos(b)]); R = R_from_axes(a, [1,0,0])
for name, tcp in [("insert", [-0.035,0.286,1.0]), ("preinsert", [-0.035,0.12,1.11]), ("regrasp y0.10", [-0.06,0.026,0.958]), ("regrasp pre", [-0.06,-0.03,1.015]), ("regrasp y0.15", [-0.06,0.076,0.958]), ("regrasp y0.18", [-0.06,0.106,0.958])]:
    q = best_ik(r, np.array(tcp), R, tries=30); print(name, tcp, None if q is None else np.round(q,2))
EOF

# openrua op 118
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.25)&(P[:,0]<0.25)&(P[:,1]>-0.25)&(P[:,1]<0.25)&(P[:,2]>0.91)
Q=P[m]; print(len(Q))
if len(Q):
    print(Q.min(0).round(3), Q.max(0).round(3))
    import collections
    # 5cm grid occupancy for z>0.91 (excluding yellow mug region)
    g=collections.Counter((tuple(np.floor(p[:2]/0.05).astype(int)), ) for p in Q)
    for k,v in sorted(g.items()): print(k, v)
EOF

# openrua op 119
cat > /workspace/graspbar.py <<'EOF'
"""Grasp vertical handle bar from -y with given lean (deg): graspbar.py bx by bz lean [lift]"""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
bx, by, bz, lean = map(float, sys.argv[1:5]); lift = float(sys.argv[5]) if len(sys.argv) > 5 else 0.03
r = Planner("graspbar")
b = np.radians(lean); a = np.array([0, np.sin(b), -np.cos(b)]); R = R_from_axes(a, [1,0,0])
target = np.array([bx, by, bz]); pre = target - 0.08*a
r.gripper(0.04)
q = best_ik(r, pre, R, tries=30); assert q is not None, "no IK for pregrasp"
print("goto pregrasp"); r.goto_joints(q); print("tcp", r.tcp()[0].round(4))
ok = r.move_line_tcp([target], R, avoid=False); print("approach ok", ok)
r.gripper(0.0)
ok = r.move_line_tcp([target + [0,0,lift]], R, avoid=False); print("lift ok", ok, "fingers", np.round(r.fingers(),4))
EOF
timeout 900 python3 -u graspbar.py -0.065 -0.399 0.958 20 0.03 2>&1 | grep -v "^  plan" | grep -v cartesian | tail -8

# openrua op 120
timeout 120 python3 profile.py frontview -0.3 0.1 -0.6 -0.2 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_now.png >/dev/null; python3 - <<'EOF'
import numpy as np, cv2
P=np.load('frontview_world.npy'); im=cv2.imread('snaps/front_now.png')
m=(P[...,0]>-0.25)&(P[...,0]<0.15)&(P[...,1]>-0.6)&(P[...,1]<-0.25)&(P[...,2]>0.905)&(P[...,2]<1.08)
vs,us=np.where(m)
crop=im[max(vs.min()-60,0):vs.max()+40, us.min()-60:us.max()+60]; cv2.imwrite('snaps/front_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
m=(Q[:,0]>-0.25)&(Q[:,0]<0.15)&(Q[:,1]>-0.6)&(Q[:,1]<-0.25)&(Q[:,2]>0.905)&(Q[:,2]<1.08); Q=Q[m]
for z0 in np.arange(0.905,1.08,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.3f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 121
sed -n 1,60p scene.py

# openrua op 122
cat > /workspace/mwscene.py <<'EOF'
"""Replace solid microwave box with hollow walls; attach white mug (upright) to hand."""
import numpy as np, rclpy
from rob import *; import scene; from attach import attach_mug
r = Robot("mwscene")
# cavity x[-0.15,0.08] y[0.266,0.444] z[0.944,1.09]; outer x[-0.16,0.17] y[0.266,0.464] z[0.90,1.107]
walls = [
    scene.remove("microwave"),
    scene.box("mw_floor",   (0.005, 0.365, 0.922), (0.33, 0.20, 0.044)),
    scene.box("mw_ceiling", (0.005, 0.365, 1.0985), (0.33, 0.20, 0.017)),
    scene.box("mw_back",    (0.005, 0.454, 1.0035), (0.33, 0.02, 0.207)),
    scene.box("mw_left",    (-0.155, 0.365, 1.0035), (0.01, 0.20, 0.207)),
    scene.box("mw_right",   (0.125, 0.365, 1.0035), (0.09, 0.20, 0.207)),
]
print("walls", scene.apply(r.node, walls))
tcp, R = r.tcp()
# mug body centre: bar is 0.074 in -y from axis, pinch at bar mid-height (0.056 above bottom)
center = tcp + np.array([0, 0.074, 0.0])
attach_mug(r, center, radius=0.048, height=0.114, axis_world=(0,0,1))
EOF
timeout 120 python3 mwscene.py 2>&1 | grep -v "^  plan"

# openrua op 123
cat > /workspace/plancheck.py <<'EOF'
import sys, numpy as np
from rob import *; from plan import Planner
r = Planner("plancheck")
tcp, R = r.tcp(); print("tcp", tcp.round(3))
wps = [tcp + [0,0,0.07], [-0.035, 0.12, tcp[2]+0.07], [-0.035, 0.12, 1.016], [-0.035, 0.286, 1.016], [-0.035, 0.286, 1.0]]
prev = tcp
for w in wps:
    traj, frac = r.cartesian([(hand_pose_from_tcp(np.array(w), R), R)], avoid=True, step=0.01)
    print("segment to", np.round(w,3), "fraction", frac)
    # note: cartesian() plans from the CURRENT robot state, so segments after the first are only indicative
    break
# plan whole path at once
traj, frac = r.cartesian([(hand_pose_from_tcp(np.array(w), R), R) for w in wps], avoid=True, step=0.01)
print("whole path fraction", frac)
EOF
timeout 300 python3 plancheck.py 2>&1 | grep -v "^  plan"

# openrua op 124
timeout 120 python3 - <<'EOF' 2>&1 | grep -v "^  plan"
import numpy as np, rclpy
from rob import *
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents, RobotState
r = Robot("chk")
cli = r.node.create_client(GetPlanningScene, "/get_planning_scene"); cli.wait_for_service(timeout_sec=10)
req = GetPlanningScene.Request(); req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
res = r._call(cli, req)
for co in res.scene.world.collision_objects:
    p = co.primitive_poses[0].position; print(co.id, co.primitives[0].type, np.round(co.primitives[0].dimensions,3), (round(p.x,3), round(p.y,3), round(p.z,3)))
for a in res.scene.robot_state.attached_collision_objects: print("attached", a.object.id, a.link_name)
# state validity at the failing pose
b = np.radians(20); a_ = np.array([0, np.sin(b), -np.cos(b)]); R = R_from_axes(a_, [1,0,0])
sv = r.node.create_client(GetStateValidity, "/check_state_validity"); sv.wait_for_service(timeout_sec=10)
for tcp in [[-0.035,0.12,1.03],[-0.035,0.12,1.016],[-0.035,0.2,1.016],[-0.035,0.286,1.016],[-0.035,0.286,1.0]]:
    q = r.solve_ik(hand_pose_from_tcp(np.array(tcp), R), R)
    if q is None: print(tcp, "no IK"); continue
    rq = GetStateValidity.Request(); rq.group_name = "panda_arm"
    rq.robot_state.joint_state.name = list(JOINTS); rq.robot_state.joint_state.position = [float(v) for v in q]
    rq.robot_state.attached_collision_objects = res.scene.robot_state.attached_collision_objects
    rq.robot_state.joint_state.name += ["panda_finger_joint1","panda_finger_joint2"]; rq.robot_state.joint_state.position += [0.0085, 0.0085]
    v = r._call(sv, rq)
    print(tcp, "valid", v.valid, [(c.contact_body_1, c.contact_body_2) for c in v.contacts])
EOF

# openrua op 125
timeout 120 python3 - <<'EOF' 2>&1 | grep -v "^  plan"
import numpy as np, rclpy
from rob import *
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents
r = Robot("chk")
cli = r.node.create_client(GetPlanningScene, "/get_planning_scene"); cli.wait_for_service(timeout_sec=10)
req = GetPlanningScene.Request(); req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
res = r._call(cli, req)
b = np.radians(20); a_ = np.array([0, np.sin(b), -np.cos(b)]); R = R_from_axes(a_, [1,0,0])
sv = r.node.create_client(GetStateValidity, "/check_state_validity"); sv.wait_for_service(timeout_sec=10)
for tcp in [[-0.035,0.12,1.05],[-0.035,0.12,1.03],[-0.035,0.12,1.016],[-0.035,0.2,1.016],[-0.035,0.286,1.016],[-0.035,0.286,1.0]]:
    q = r.solve_ik(hand_pose_from_tcp(np.array(tcp), R), R)
    if q is None: print(tcp, "no IK"); continue
    rq = GetStateValidity.Request(); rq.group_name = "panda_arm"
    rq.robot_state.joint_state.name = list(JOINTS) + ["panda_finger_joint1","panda_finger_joint2"]
    rq.robot_state.joint_state.position = [float(v) for v in q] + [0.0085, 0.0085]
    rq.robot_state.attached_collision_objects = res.scene.robot_state.attached_collision_objects
    v = r._call(sv, rq)
    print(tcp, "valid", v.valid, [(c.contact_body_1, c.contact_body_2) for c in v.contacts])
EOF

# openrua op 126
timeout 120 python3 - <<'EOF' 2>&1 | grep -v "^  plan"
import numpy as np, rclpy
from rob import *
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents
r = Robot("chk")
cli = r.node.create_client(GetPlanningScene, "/get_planning_scene"); cli.wait_for_service(timeout_sec=10)
req = GetPlanningScene.Request(); req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
res = r._call(cli, req)
sv = r.node.create_client(GetStateValidity, "/check_state_validity"); sv.wait_for_service(timeout_sec=10)
for lean in [20, 30, 45]:
  b = np.radians(lean); a_ = np.array([0, np.sin(b), -np.cos(b)]); R = R_from_axes(a_, [1,0,0])
  for y in [0.23, 0.245, 0.26, 0.275, 0.29]:
    for z in [1.016, 1.005]:
      tcp=[-0.035,y,z]
      q = r.solve_ik(hand_pose_from_tcp(np.array(tcp), R), R)
      if q is None: print(lean, tcp, "no IK"); continue
      rq = GetStateValidity.Request(); rq.group_name = "panda_arm"
      rq.robot_state.joint_state.name = list(JOINTS) + ["panda_finger_joint1","panda_finger_joint2"]
      rq.robot_state.joint_state.position = [float(v) for v in q] + [0.0085, 0.0085]
      rq.robot_state.attached_collision_objects = res.scene.robot_state.attached_collision_objects
      v = r._call(sv, rq)
      print(lean, tcp, "valid", v.valid, sorted(set((c.contact_body_1, c.contact_body_2) for c in v.contacts)))
EOF

# openrua op 127
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^  plan"
import numpy as np
from rob import *; from ikbest import best_ik
r = Robot("iktest")
R = R_from_axes([0,1,0], [1,0,0])   # horizontal, fingers pointing +y, pads along x
for name, tcp in [("insert", [-0.035,0.28,1.0]), ("preinsert", [-0.035,0.12,1.016]), ("preinsert high", [-0.035,0.12,1.06]),
                  ("regrasp here", [-0.065,-0.399,0.958]), ("regrasp pre here", [-0.065,-0.48,0.958]),
                  ("regrasp y0.10", [-0.06,0.026,0.958]), ("regrasp pre y0.10", [-0.06,-0.054,0.958]),
                  ("regrasp x0.2", [0.2,-0.3,0.958]), ("regrasp pre x0.2", [0.2,-0.38,0.958]),
                  ("regrasp x-0.25", [-0.25,-0.3,0.958]), ("regrasp pre x-0.25", [-0.25,-0.38,0.958])]:
    q = best_ik(r, np.array(tcp), R, tries=30); print(name, tcp, None if q is None else np.round(q,2))
EOF

# openrua op 128
python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
for (x0,x1,y0,y1) in [(-0.1,0.1,-0.1,0.1),(-0.15,-0.05,-0.1,0.0),(-0.15,0.0,-0.25,-0.1),(-0.1,0.0,0.0,0.15)]:
    m=(P[:,0]>x0)&(P[:,0]<x1)&(P[:,1]>y0)&(P[:,1]<y1); Q=P[m]
    print((x0,x1,y0,y1), len(Q), "z pct", np.percentile(Q[:,2],[1,50,99]).round(3) if len(Q) else None)
EOF

# openrua op 129
cat > /workspace/setdown2.py <<'EOF'
"""Carry held upright mug (pinch on bar) to pinch xy and set down: setdown2.py px py [transit_z] [bottom_offset]"""
import sys, numpy as np
from rob import *; from plan import Planner
px, py = float(sys.argv[1]), float(sys.argv[2]); tz = float(sys.argv[3]) if len(sys.argv) > 3 else 1.04
TABLE = 0.9017; PINCH_ABOVE_BOTTOM = 0.056
r = Planner("setdown2")
tcp, R = r.tcp(); a = R[:,2]
wps = [np.array([tcp[0], tcp[1], tz]), np.array([px, py, tz]), np.array([px, py, TABLE + PINCH_ABOVE_BOTTOM + 0.004])]
for w in wps:
    ok = r.move_line_tcp([w], R, avoid=False); print("to", w.round(3), "ok", ok, "fingers", np.round(r.fingers(),4))
    if not ok: sys.exit("move failed")
r.gripper(0.04)
tcp, R = r.tcp(); ok = r.move_line_tcp([tcp - 0.06*a], R, avoid=False); print("retreat ok", ok)
tcp, R = r.tcp(); ok = r.move_line_tcp([tcp + [0,0,0.06]], R, avoid=False); print("up ok", ok, r.tcp()[0].round(3))
EOF
timeout 900 python3 -u setdown2.py -0.06 0.026 1.04 2>&1 | grep -v "^  plan" | grep -v cartesian | grep -v "move done"

# openrua op 130
timeout 120 python3 heightmap.py 2>&1 | grep blob; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.2)&(P[:,0]<0.05)&(P[:,1]>-0.05)&(P[:,1]<0.25)&(P[:,2]>0.91)&(P[:,2]<1.05)
Q=P[m]; print(len(Q), Q.min(0).round(3), Q.max(0).round(3))
for z0 in np.arange(0.91,1.03,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
top=Q[Q[:,2]>1.0]; print("top: n",len(top), "x[%.3f,%.3f] y[%.3f,%.3f]"%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max()), "mean", top[:,:2].mean(0).round(4), "zmed", np.median(top[:,2]).round(4))
EOF

# openrua op 131
timeout 120 python3 profile.py frontview -0.2 0.05 -0.1 0.25 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
Q=np.load('frontview_world.npy').reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
m=(Q[:,0]>-0.15)&(Q[:,0]<0.03)&(Q[:,1]>-0.05)&(Q[:,1]<0.2)&(Q[:,2]>0.905)&(Q[:,2]<1.03); Q=Q[m]
for z0 in np.arange(0.905,1.03,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.3f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 132
timeout 60 python3 attach.py detach -0.063 0.0935 0.9577 2>&1 | grep -v "^  plan"
cat > /workspace/grasph.py <<'EOF'
"""Horizontal bar grasp from -y (fingers point +y, pads along x): grasph.py bx by bz [lift]"""
import sys, numpy as np
from rob import *; from plan import Planner; from ikbest import best_ik
bx, by, bz = map(float, sys.argv[1:4]); lift = float(sys.argv[4]) if len(sys.argv) > 4 else 0.03
r = Planner("grasph")
R = R_from_axes([0,1,0], [1,0,0]); a = R[:,2]
target = np.array([bx, by, bz]); pre = target - 0.08*a
r.gripper(0.04)
q = best_ik(r, pre, R, tries=30); assert q is not None, "no IK for pregrasp"
print("goto pregrasp"); ok = r.goto_joints(q); print("pregrasp ok", ok, "tcp", r.tcp()[0].round(4))
ok = r.move_line_tcp([target], R, avoid=False); print("approach ok", ok)
r.gripper(0.0)
ok = r.move_line_tcp([target + [0,0,lift]], R, avoid=False); print("lift ok", ok, "fingers", np.round(r.fingers(),4))
EOF
timeout 900 python3 -u grasph.py -0.063 0.021 0.958 0.03 2>&1 | grep -v "^  plan" | grep -v cartesian | grep -v "move done"

# openrua op 133
timeout 120 python3 profile.py frontview -0.2 0.05 -0.1 0.25 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_now.png >/dev/null; python3 - <<'EOF'
import numpy as np, cv2
P=np.load('frontview_world.npy'); im=cv2.imread('snaps/front_now.png')
m=(P[...,0]>-0.15)&(P[...,0]<0.03)&(P[...,1]>-0.1)&(P[...,1]<0.2)&(P[...,2]>0.905)&(P[...,2]<1.06)
vs,us=np.where(m)
crop=im[max(vs.min()-60,0):vs.max()+40, us.min()-80:us.max()+60]; cv2.imwrite('snaps/front_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
Q=P.reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
m=(Q[:,0]>-0.15)&(Q[:,0]<0.03)&(Q[:,1]>-0.1)&(Q[:,1]<0.2)&(Q[:,2]>0.905)&(Q[:,2]<1.06); Q=Q[m]
for z0 in np.arange(0.905,1.06,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.3f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 134
timeout 120 python3 - <<'EOF' 2>&1 | grep -v "^  plan"
import numpy as np
from rob import *; from attach import attach_mug; from plan import Planner
r = Planner("att2")
tcp, R = r.tcp(); print("tcp", tcp.round(4))
attach_mug(r, tcp + np.array([0, 0.0725, 0.0]), radius=0.048, height=0.114, axis_world=(0,0,1))
wps = [[-0.063, 0.021, 1.02], [-0.035, 0.12, 1.02], [-0.035, 0.29, 1.02], [-0.035, 0.29, 1.003]]
traj, frac = r.cartesian([(hand_pose_from_tcp(np.array(w), R), R) for w in wps], avoid=True, step=0.01)
print("whole path fraction", frac)
for w in wps[:3]:
    pass
EOF

# openrua op 135
timeout 120 python3 - <<'EOF' 2>&1 | grep -v "^  plan"
import numpy as np
from rob import *
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents
r = Robot("chk")
cli = r.node.create_client(GetPlanningScene, "/get_planning_scene"); cli.wait_for_service(timeout_sec=10)
req = GetPlanningScene.Request(); req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
res = r._call(cli, req)
sv = r.node.create_client(GetStateValidity, "/check_state_validity"); sv.wait_for_service(timeout_sec=10)
R = R_from_axes([0,1,0],[1,0,0])
def check(q, label):
    rq = GetStateValidity.Request(); rq.group_name = "panda_arm"
    rq.robot_state.joint_state.name = list(JOINTS) + ["panda_finger_joint1","panda_finger_joint2"]
    rq.robot_state.joint_state.position = [float(v) for v in q] + [0.0083, 0.0083]
    rq.robot_state.attached_collision_objects = res.scene.robot_state.attached_collision_objects
    v = r._call(sv, rq); print(label, "valid", v.valid, sorted(set((c.contact_body_1, c.contact_body_2) for c in v.contacts)))
check(r.joints(), "current")
for tcp in [[-0.063,0.021,1.02],[-0.035,0.12,1.02],[-0.035,0.2,1.02],[-0.035,0.29,1.02],[-0.035,0.29,1.003],[-0.035,0.30,1.003]]:
    q = r.solve_ik(hand_pose_from_tcp(np.array(tcp), R), R)
    if q is None: print(tcp, "no IK"); continue
    check(q, tcp)
EOF

# openrua op 136
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^  plan"
import numpy as np
from rob import *; from plan import Planner
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents
r = Planner("chk")
R = R_from_axes([0,1,0],[1,0,0])
tcp,_ = r.tcp(); ok = r.move_line_tcp([[tcp[0], tcp[1], 1.06]], R, avoid=False); print("up ok", ok)
cli = r.node.create_client(GetPlanningScene, "/get_planning_scene"); cli.wait_for_service(timeout_sec=10)
req = GetPlanningScene.Request(); req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
res = r._call(cli, req)
sv = r.node.create_client(GetStateValidity, "/check_state_validity"); sv.wait_for_service(timeout_sec=10)
def check(q, label):
    rq = GetStateValidity.Request(); rq.group_name = "panda_arm"
    rq.robot_state.joint_state.name = list(JOINTS) + ["panda_finger_joint1","panda_finger_joint2"]
    rq.robot_state.joint_state.position = [float(v) for v in q] + [0.0083, 0.0083]
    rq.robot_state.attached_collision_objects = res.scene.robot_state.attached_collision_objects
    v = r._call(sv, rq); print(label, "valid", v.valid, sorted(set((c.contact_body_1, c.contact_body_2) for c in v.contacts)))
check(r.joints(), "current")
for x in [-0.06, -0.08]:
  for tcp in [[x,0.12,1.02],[x,0.2,1.02],[x,0.29,1.02],[x,0.29,1.003]]:
    q = r.solve_ik(hand_pose_from_tcp(np.array(tcp), R), R)
    if q is None: print(tcp, "no IK"); continue
    check(q, tcp)
for x in [-0.06, -0.08]:
    wps = [[-0.063, 0.021, 1.06], [x, 0.12, 1.06], [x, 0.12, 1.02], [x, 0.29, 1.02], [x, 0.29, 1.003]]
    traj, frac = r.cartesian([(hand_pose_from_tcp(np.array(w), R), R) for w in wps], avoid=True, step=0.01)
    print("x", x, "whole path fraction", frac)
EOF

# openrua op 137
cat > /workspace/insert.py <<'EOF'
import sys, numpy as np
from rob import *; from plan import Planner
r = Planner("insert"); R = R_from_axes([0,1,0],[1,0,0])
wps = [[-0.06, 0.12, 1.06], [-0.06, 0.12, 1.02]]
for w in wps:
    ok = r.move_line_tcp([w], R, avoid=True); print("to", w, "ok", ok, "fingers", np.round(r.fingers(),4))
    if not ok: sys.exit("move failed")
EOF
timeout 600 python3 -u insert.py 2>&1 | grep -v "^  plan" | grep -v cartesian | grep -v "move done"
timeout 120 python3 profile.py frontview -0.2 0.05 0.0 0.3 2>&1 | tail -1; python3 - <<'EOF'
import numpy as np
Q=np.load('frontview_world.npy').reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
m=(Q[:,0]>-0.15)&(Q[:,0]<0.03)&(Q[:,1]>0.05)&(Q[:,1]<0.27)&(Q[:,2]>0.93)&(Q[:,2]<1.1); Q=Q[m]
for z0 in np.arange(0.93,1.1,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.3f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 138
python3 - <<'EOF'
import numpy as np
Q=np.load('frontview_world.npy').reshape(-1,3); Q=Q[np.isfinite(Q).all(1)]
m=(Q[:,0]>-0.12)&(Q[:,0]<0.0)&(Q[:,1]>0.05)&(Q[:,1]<0.26)&(Q[:,2]>0.93)&(Q[:,2]<1.1); Q=Q[m]
for z0 in np.arange(0.93,1.1,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(s): print(f"z {z0:.3f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 139
cat > /workspace/insert2.py <<'EOF'
import sys, numpy as np
from rob import *; from plan import Planner; from attach import attach_mug
r = Planner("insert2"); R = R_from_axes([0,1,0],[1,0,0])
tcp,_ = r.tcp(); attach_mug(r, tcp + np.array([0, 0.0725, -0.010]), radius=0.048, height=0.114, axis_world=(0,0,1))
for w, avoid in [([-0.06, 0.12, 1.032], True), ([-0.06, 0.29, 1.032], True)]:
    ok = r.move_line_tcp([w], R, avoid=avoid); print("to", w, "ok", ok, "fingers", np.round(r.fingers(),4))
    if not ok: sys.exit("move failed")
EOF
timeout 600 python3 -u insert2.py 2>&1 | grep -v "^  plan" | grep -v "move done"

# openrua op 140
timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_now.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_now.png >/dev/null; python3 -c "
import cv2; im=cv2.imread('snaps/front_now.png'); cv2.imwrite('snaps/front_crop.png', cv2.resize(im[150:400, 250:600],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))"

# openrua op 141
cat > /workspace/place.py <<'EOF'
import sys, numpy as np
from rob import *; from plan import Planner; from attach import detach_mug
r = Planner("place"); R = R_from_axes([0,1,0],[1,0,0])
tcp,_ = r.tcp(); print("tcp", tcp.round(4))
ok = r.move_line_tcp([[tcp[0], tcp[1], 1.009]], R, avoid=False); print("lower ok", ok, "fingers", np.round(r.fingers(),4))
r.gripper(0.04)
tcp,_ = r.tcp(); detach_mug(r, [tcp[0], tcp[1]+0.0725, 0.9017+0.0423+0.056])
ok = r.move_line_tcp([[tcp[0], 0.12, tcp[2]]], R, avoid=False); print("retreat ok", ok, r.tcp()[0].round(3))
EOF
timeout 600 python3 -u place.py 2>&1 | grep -v "^  plan" | grep -v "move done" | grep -v cartesian

# openrua op 142
timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_now.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_now.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_robotview snaps/robot_now.png >/dev/null; python3 -c "
import cv2; im=cv2.imread('snaps/front_now.png'); cv2.imwrite('snaps/front_crop.png', cv2.resize(im[120:420, 200:640],None,fx=1.6,fy=1.6,interpolation=cv2.INTER_CUBIC))"

# openrua op 143
timeout 30 ros2 topic list 2>/dev/null | grep -v -E "^/(panda|franka|move_group|moveit|servo|joint|parameter|rosout|planning|display|trajectory|attached|collision|recognized|execute|pickup|place|monitored)" | head -40; timeout 20 ros2 run tf2_ros tf2_echo world white_mug 2>&1 | head -3

# openrua op 144
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_now.png && timeout 60 python3 tools/perception/cam_snap.py galleryview snaps/gallery_now.png && timeout 60 python3 tools/perception/cam_snap.py paperview snaps/paper_now.png; python3 -c "
import cv2; im=cv2.imread('snaps/eih_now.png'); print(im.shape); cv2.imwrite('snaps/eih_big.png', cv2.resize(im,None,fx=1.5,fy=1.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 145
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^  plan"
import numpy as np
from rob import *; from ikbest import best_ik
r = Robot("iktest")
H = np.array([-0.165, 0.27]); rho = 0.12
def pose(th_deg, lean_deg, z=1.0):
    th = np.radians(th_deg); d = np.array([np.cos(th), np.sin(th), 0]); n1 = np.array([-np.sin(th), np.cos(th), 0])
    tcp = np.array([*H, 0]) + rho*d - 0.01*n1; tcp[2] = z
    b = np.radians(lean_deg); a = np.sin(b)*n1 + np.array([0,0,-np.cos(b)])
    return tcp, R_from_axes(a, d)
for lean in [90, 45, 30]:
    print("lean", lean)
    for th in [-114, -100, -80, -60, -40, -20, 0]:
        tcp, R = pose(th, lean)
        q = best_ik(r, tcp, R, tries=20)
        print("  th", th, "tcp", tcp.round(3), "IK", None if q is None else np.round(q,2))
EOF

# openrua op 146
sed -n 61,76p plan.py

# openrua op 147
grep -n "_start_state" -A 6 plan.py | head -12

# openrua op 148
python3 - <<'EOF'
import re
s=open('/workspace/plan.py').read()
s=s.replace("    def cartesian(self, hand_targets, avoid=True, step=0.01):", "    def cartesian(self, hand_targets, avoid=True, step=0.01, start_q=None):")
s=s.replace("req.start_state = self._start_state(); req.max_step = step", "req.start_state = self._start_state() if start_q is None else self._state_from(start_q); req.max_step = step")
s=s.replace("    def _start_state(self):", "    def _state_from(self, q):\n        rs = RobotState(); rs.joint_state.name = list(JOINTS); rs.joint_state.position = [float(v) for v in q]\n        return rs\n    def _start_state(self):")
open('/workspace/plan.py','w').write(s)
EOF
cat > /workspace/doorplan.py <<'EOF'
"""Test-plan door closing arcs: doorplan.py lean_deg rho [execute]"""
import sys, numpy as np, scene
from rob import *; from plan import Planner; from ikbest import best_ik
lean = float(sys.argv[1]); rho = float(sys.argv[2]); execute = len(sys.argv) > 3
H = np.array([-0.165, 0.27]); Z = 1.0
def pose(th_deg):
    th = np.radians(th_deg); d = np.array([np.cos(th), np.sin(th), 0]); n1 = np.array([-np.sin(th), np.cos(th), 0])
    tcp = np.array([*H, 0]) + rho*d - 0.01*n1; tcp[2] = Z
    b = np.radians(lean); a = np.sin(b)*n1 + np.array([0,0,-np.cos(b)])
    return tcp, R_from_axes(a, d)
r = Planner("doorplan")
scene.apply(r.node, [scene.remove("mw_door")])
ths = np.arange(-114, 2.1, 4.0)
targets = [(hand_pose_from_tcp(*pose(t)), pose(t)[1]) for t in ths]
best = None
for k in range(12):
    q0 = best_ik(r, *pose(ths[0]), tries=1 if k == 0 else 3, seeds=[np.random.default_rng(k).uniform(-1.5, 1.5, 7)]) if k else best_ik(r, *pose(ths[0]), tries=10)
    if q0 is None: continue
    traj, frac = r.cartesian(targets, avoid=True, step=0.01, start_q=q0)
    print("start", np.round(q0,2), "fraction", round(frac,3))
    if best is None or frac > best[1]: best = (q0, frac, traj)
    if frac >= 0.99: break
print("BEST fraction", best[1] if best else None)
EOF
timeout 900 python3 doorplan.py 90 0.12 2>&1 | grep -v "^  plan" | grep -v "cartesian:"

# openrua op 149
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "^  plan"
import numpy as np
from rob import *; from ikbest import best_ik
from moveit_msgs.srv import GetStateValidity
r = Robot("chk"); H = np.array([-0.165, 0.27])
sv = r.node.create_client(GetStateValidity, "/check_state_validity"); sv.wait_for_service(timeout_sec=10)
def pose(th_deg, lean, rho, Z=1.0):
    th = np.radians(th_deg); d = np.array([np.cos(th), np.sin(th), 0]); n1 = np.array([-np.sin(th), np.cos(th), 0])
    tcp = np.array([*H, 0]) + rho*d - 0.01*n1; tcp[2] = Z
    b = np.radians(lean); a = np.sin(b)*n1 + np.array([0,0,-np.cos(b)])
    return tcp, R_from_axes(a, d)
for lean, rho in [(90,0.12),(90,0.08),(60,0.12),(45,0.12)]:
  for th in [-30,-24,-18,-12,-6,0]:
    tcp, R = pose(th, lean, rho); q = best_ik(r, tcp, R, tries=8)
    if q is None: print(lean, rho, th, "no IK"); continue
    rq = GetStateValidity.Request(); rq.group_name = "panda_arm"
    rq.robot_state.joint_state.name = list(JOINTS) + ["panda_finger_joint1","panda_finger_joint2"]
    rq.robot_state.joint_state.position = [float(v) for v in q] + [0.0, 0.0]
    v = r._call(sv, rq); print(lean, rho, th, tcp.round(3), "valid", v.valid, sorted(set((c.contact_body_1, c.contact_body_2) for c in v.contacts)))
EOF

# openrua op 150
cat > /workspace/doorplan2.py <<'EOF'
"""Test-plan a door arc segment: doorplan2.py lean rho th0 th1"""
import sys, numpy as np, scene
from rob import *; from plan import Planner; from ikbest import best_ik
lean, rho, th0, th1 = map(float, sys.argv[1:5])
H = np.array([-0.165, 0.27]); Z = 1.0
def pose(th_deg):
    th = np.radians(th_deg); d = np.array([np.cos(th), np.sin(th), 0]); n1 = np.array([-np.sin(th), np.cos(th), 0])
    tcp = np.array([*H, 0]) + rho*d - 0.01*n1; tcp[2] = Z
    b = np.radians(lean); a = np.sin(b)*n1 + np.array([0,0,-np.cos(b)])
    return tcp, R_from_axes(a, d)
r = Planner("doorplan2")
ths = np.arange(th0, th1 + 0.1, 4.0)
targets = [(hand_pose_from_tcp(*pose(t)), pose(t)[1]) for t in ths]
best = None
for k in range(10):
    rng = np.random.default_rng(k)
    q0 = best_ik(r, *pose(ths[0]), tries=4, seeds=[rng.uniform(-1.5, 1.5, 7)])
    if q0 is None: continue
    traj, frac = r.cartesian(targets, avoid=True, step=0.01, start_q=q0)
    print("start", np.round(q0,2), "fraction", round(frac,3))
    if best is None or frac > best[1]: best = (q0, frac)
    if frac >= 0.99: break
print("BEST", best[1] if best else None, np.round(best[0],3) if best else None)
EOF
echo "phase2"; timeout 600 python3 doorplan2.py 90 0.12 -30 2 2>&1 | grep -v "^  plan" | grep -v "cartesian:"; echo "phase1"; timeout 600 python3 doorplan2.py 90 0.12 -114 -26 2>&1 | grep -v "^  plan" | grep -v "cartesian:"

# openrua op 151
timeout 120 python3 heightmap.py >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
P=np.load('birdview_world.npy').reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]<-0.14)&(P[:,0]>-0.4)&(P[:,1]>0.0)&(P[:,1]<0.30)&(P[:,2]>0.95)
Q=P[m]; print(len(Q), "z", np.percentile(Q[:,2],[5,50,95]).round(3))
# door top plane points: z>1.09
top=Q[Q[:,2]>1.09]; print("top n", len(top))
# fit line to the top points: PCA
c=top[:,:2].mean(0); u,s,vt=np.linalg.svd(top[:,:2]-c); d=vt[0]; n=vt[1]
print("center", c.round(4), "dir", d.round(3), "angle", np.degrees(np.arctan2(d[1],d[0])).round(1))
al=(top[:,:2]-c)@d; ac=(top[:,:2]-c)@n
print("along range", al.min().round(3), al.max().round(3), "across range", ac.min().round(3), ac.max().round(3))
# thickness histogram across
print(np.histogram(ac, bins=12)[0], np.histogram(ac,bins=12)[1].round(3))
# endpoints
e1=c+al.min()*d; e2=c+al.max()*d; print("ends", e1.round(3), e2.round(3))
# lower points (z between 0.95 and 1.09): handle?
lo=Q[(Q[:,2]<1.08)]
print("lower n", len(lo)); 
if len(lo):
    al2=(lo[:,:2]-c)@d; ac2=(lo[:,:2]-c)@n; print("lower along", al2.min().round(3), al2.max().round(3), "across", ac2.min().round(3), ac2.max().round(3), "z", np.percentile(lo[:,2],[5,50,95]).round(3))
EOF

# openrua op 152
timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird_now.png >/dev/null; python3 -c "
import cv2, numpy as np; im=cv2.imread('snaps/bird_now.png'); P=np.load('birdview_world.npy')
m=(P[...,0]<-0.1)&(P[...,0]>-0.45)&(P[...,1]>-0.05)&(P[...,1]<0.5)
vs,us=np.where(m); print(us.min(),us.max(),vs.min(),vs.max())
crop=im[vs.min():vs.max(), us.min():us.max()]; cv2.imwrite('snaps/bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 153
cat > /workspace/doorplan3.py <<'EOF'
"""Collect start configs giving full-fraction door arc: doorplan3.py lean rho th0 th1 nseeds"""
import sys, numpy as np, scene
from rob import *; from plan import Planner
lean, rho, th0, th1 = map(float, sys.argv[1:5]); nseeds = int(sys.argv[5])
H = np.array([-0.165, 0.27]); Z = 1.0
lim = np.array(ARM["limits_rad"])
def pose(th_deg):
    th = np.radians(th_deg); d = np.array([np.cos(th), np.sin(th), 0]); n1 = np.array([-np.sin(th), np.cos(th), 0])
    tcp = np.array([*H, 0]) + rho*d - 0.015*n1; tcp[2] = Z
    b = np.radians(lean); a = np.sin(b)*n1 + np.array([0,0,-np.cos(b)])
    return tcp, R_from_axes(a, d)
r = Planner("doorplan3")
scene.apply(r.node, [scene.remove("mw_door")])
ths = np.arange(th0, th1 + 0.1, 4.0)
targets = [(hand_pose_from_tcp(*pose(t)), pose(t)[1]) for t in ths]
tcp0, R0 = pose(ths[0]); good = []
rng = np.random.default_rng(1)
for k in range(nseeds):
    seed = rng.uniform(lim[:,0]+0.1, lim[:,1]-0.1)
    q0 = r.solve_ik(hand_pose_from_tcp(tcp0, R0), R0, seed=list(seed), timeout=1.0)
    if q0 is None: continue
    q0 = np.array(q0); margin = np.min(np.minimum(q0-lim[:,0], lim[:,1]-q0))
    if margin < 0.15: continue
    traj, frac = r.cartesian(targets, avoid=True, step=0.01, start_q=q0)
    if frac >= 0.99:
        qs = np.array([p.positions for p in traj.points]); m2 = np.min(np.minimum(qs-lim[:,0], lim[:,1]-qs))
        good.append((m2, q0)); print("GOOD start", np.round(q0,2), "min limit margin along path", round(m2,3))
print("found", len(good))
if good:
    best = max(good, key=lambda g: g[0]); print("BEST", np.round(best[1],3), "margin", round(best[0],3))
EOF
echo "phase1 -114..-26"; timeout 900 python3 doorplan3.py 90 0.12 -114 -26 40 2>&1 | grep -E "GOOD|found|BEST"

# openrua op 154
for lean in 90 60 45; do for seg in "-114 -60" "-60 -20" "-20 2" "-114 -40" "-40 2"; do echo "lean $lean seg $seg: $(timeout 600 python3 doorplan3.py $lean 0.12 $seg 25 2>&1 | grep -E 'found|BEST' | tr '\n' ' ')"; done; done

# openrua op 155
sed -i 's/^lean, rho, th0, th1 = map(float, sys.argv\[1:5\]); nseeds = int(sys.argv\[5\])/lean, rho, th0, th1 = map(float, sys.argv[1:5]); nseeds = int(sys.argv[5]); Z = float(sys.argv[6]) if len(sys.argv) > 6 else 1.0/; s/^H = np.array(\[-0.165, 0.27\]); Z = 1.0/H = np.array([-0.165, 0.27])/' doorplan3.py
for lean in 0 15; do for seg in "-114 2" "-114 -40" "-40 2"; do echo "lean $lean seg $seg: $(timeout 600 python3 doorplan3.py $lean 0.12 $seg 25 1.07 2>&1 | grep -E 'found|BEST' | tr '\n' ' ')"; done; done

# openrua op 156
cat heightmap.py | head -60; ls snaps | tail -5

# openrua op 157
mkdir -p "$(dirname /workspace/doorangle.py)"
cat > /workspace/doorangle.py <<'OPENRUA_EOF'
"""Measure the microwave door angle from the birdview heightmap (run heightmap.py first or pass --fresh)."""
import sys, subprocess, numpy as np
if "--fresh" in sys.argv: subprocess.run(["python3", "heightmap.py"], capture_output=True)
P = np.load("birdview_world.npy")
H = np.array([-0.165, 0.27])
X, Y, Z = P[..., 0], P[..., 1], P[..., 2]
# door top: tall thin object, z in [1.09, 1.12], west of the hinge region or in front of the cavity, not the arm (x > -0.35)
m = (Z > 1.085) & (Z < 1.125) & (X > -0.36) & (X < 0.16) & (Y < 0.262) & (Y > -0.1)
pts = np.stack([X[m], Y[m]], -1)
print("door-top points:", len(pts))
if len(pts) < 20: sys.exit()
c = pts.mean(0); u, s, vt = np.linalg.svd(pts - c); d = vt[0]
if d @ (c - H) < 0: d = -d
th = np.degrees(np.arctan2(d[1], d[0]))
rho = (pts - H) @ d
print(f"door dir {np.round(d,3)} theta={th:.1f} deg  centroid {np.round(c,3)}  rho range [{rho.min():.3f},{rho.max():.3f}]  perp spread {np.std((pts-H)@np.array([-d[1],d[0]])):.3f}")
OPENRUA_EOF

# openrua op 158
timeout 120 python3 doorangle.py --fresh

# openrua op 159
python3 -c "
import numpy as np, cv2
P=np.load('birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(Z>1.085)&(Z<1.125)&(X>-0.36)&(X<0.16)&(Y<0.262)&(Y>-0.1)
n,lab,st,ce=cv2.connectedComponentsWithStats(m.astype(np.uint8))
for i in range(1,n):
    if st[i,4]<15: continue
    p=P[lab==i]; print(i, st[i,4], 'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
"

# openrua op 160
python3 - <<'EOF'
import re
s=open('doorangle.py').read()
s=s.replace('''pts = np.stack([X[m], Y[m]], -1)''','''import cv2
n, lab, st, ce = cv2.connectedComponentsWithStats(m.astype(np.uint8))
# door = component whose points touch the hinge neighbourhood (within 6 cm of H)
best = None
for i in range(1, n):
    if st[i, 4] < 30: continue
    p = P[lab == i][:, :2]
    if np.linalg.norm(p - H, axis=1).min() < 0.06 and (best is None or st[i, 4] > st[best, 4]): best = i
pts = P[lab == best][:, :2] if best is not None else np.zeros((0, 2))''')
open('doorangle.py','w').write(s)
EOF
python3 doorangle.py

# openrua op 161
python3 - <<'EOF'
import numpy as np
exec(open('doorangle.py').read().split('print("door-top')[0])
n1=np.array([-d[1],d[0]]); off=(pts-H)@n1
print("perp offset of top points: mean %.4f min %.4f max %.4f"%(off.mean(),off.min(),off.max()))
EOF

# openrua op 162
cat >> doorangle.py <<'EOF'
n1 = np.array([-d[1], d[0]]); off = (pts - H) @ n1
print(f"perp offset of top points about H: mean {off.mean():.4f} min {off.min():.4f} max {off.max():.4f}")
EOF
python3 doorangle.py

# openrua op 163
cat doorplan3.py

# openrua op 164
sed -i 's/tcp = np.array(\[\*H, 0\]) + rho\*d - 0.015\*n1/tcp = np.array([*H, 0]) + rho*d + OFF*n1/; s/^H = np.array(\[-0.165, 0.27\])$/H = np.array([-0.165, 0.27]); OFF = -0.032/' doorplan3.py
for seg in "-114 -40" "-40 2" "-114 2"; do echo "lean 15 seg $seg: $(timeout 900 python3 doorplan3.py 15 0.12 $seg 40 1.07 2>&1 | grep -E 'found|BEST' | tr '\n' ' ')"; done

# openrua op 165
mkdir -p "$(dirname /workspace/doorpush.py)"
cat > /workspace/doorpush.py <<'OPENRUA_EOF'
"""Close the microwave door by pushing its outer face with the closed fingertips along an arc about the hinge.
usage: doorpush.py th0 th1 [--go]   (without --go: plan only)"""
import sys, numpy as np, rclpy, scene
from rob import *; from plan import Planner
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState

th0, th1 = float(sys.argv[1]), float(sys.argv[2]); GO = "--go" in sys.argv
LEAN, RHO, Z, OFF = 15.0, 0.12, 1.075, -0.032
H = np.array([-0.165, 0.27]); DOOR_OFF = -0.013          # door centre line offset from hinge line (birdview)
START_Q = [-1.032, -0.943, 1.63, -2.566, 1.209, 2.377, 0.879]   # doorplan3 lean15 z1.07 full-arc start (margin 0.50)
lim = np.array(ARM["limits_rad"])

def dn(th_deg):
    th = np.radians(th_deg)
    return np.array([np.cos(th), np.sin(th), 0]), np.array([-np.sin(th), np.cos(th), 0])
def pose(th_deg, back=0.0):
    d, n1 = dn(th_deg)
    tcp = np.array([*H, 0]) + RHO*d + (OFF - back)*n1; tcp[2] = Z
    b = np.radians(LEAN); a = np.sin(b)*n1 + np.array([0, 0, -np.cos(b)])
    return tcp, R_from_axes(a, d)
def door_box(th_deg, thick=0.035):
    d, n1 = dn(th_deg); c = np.array([*H, 1.0035]) + 0.15*d + DOOR_OFF*n1
    return scene.box("mw_door", c, (0.30, thick, 0.207), yaw=np.radians(th_deg))

r = Planner("doorpush")
sv = r.node.create_client(GetStateValidity, "/check_state_validity")
def valid(q):
    req = GetStateValidity.Request(); req.group_name = M["planning"]["group"]
    rs = RobotState(); rs.joint_state.name = list(JOINTS) + ["panda_finger_joint1", "panda_finger_joint2"]
    rs.joint_state.position = [float(v) for v in q] + [0.0, 0.0]; req.robot_state = rs
    res = r._call(sv, req)
    bad = [(c.contact_body_1, c.contact_body_2) for c in res.contacts]
    return res.valid, bad

# --- plan
tcp_c, Rc = pose(th0); tcp_p, _ = pose(th0, back=0.04)
q_c = r.solve_ik(hand_pose_from_tcp(tcp_c, Rc), Rc, seed=START_Q, timeout=2.0)
q_p = r.solve_ik(hand_pose_from_tcp(tcp_p, Rc), Rc, seed=q_c or START_Q, timeout=2.0)
print("contact q", np.round(q_c, 3) if q_c else None, "\npre q", np.round(q_p, 3) if q_p else None)
if q_c is None or q_p is None: sys.exit("no IK")
scene.apply(r.node, [door_box(th0, thick=0.03)])          # thin door: tests hand body vs door, not the 2 mm tip overlap
print("contact state valid (thin door in scene):", valid(q_c))
print("pre state valid:", valid(q_p))
scene.apply(r.node, [door_box(th0)])
ths = np.arange(th0, th1 + 0.1, 4.0)
arc = [(hand_pose_from_tcp(*pose(t)), pose(t)[1]) for t in ths]
scene.apply(r.node, [scene.remove("mw_door")])
traj, frac = r.cartesian(arc, avoid=True, start_q=q_c)
qs = np.array([p.positions for p in traj.points]); m2 = np.min(np.minimum(qs - lim[:, 0], lim[:, 1] - qs))
print(f"arc plan from contact q: fraction {frac:.2f}, limit margin {m2:.3f}")
scene.apply(r.node, [door_box(th0)])
if not GO or frac < 0.99: sys.exit("plan only" if not GO else "arc incomplete")

# --- execute
print("== closing gripper (fingertips as pusher)"); r.gripper(0.0)
p, R = r.tcp(); print("tcp now", np.round(p, 3))
print("== lifting clear of the opening")
if not r.move_line_tcp([p + [0, 0, 0.09], p + [0, -0.08, 0.09]], R, avoid=True, min_fraction=0.95): sys.exit("retreat failed")
print("== joint move to pre-push pose")
if not r.goto_joints(q_p): sys.exit("goto pre failed")
print("== approach to contact")
if not r.move_line_tcp([tcp_c], Rc, avoid=False): sys.exit("approach failed")
scene.apply(r.node, [scene.remove("mw_door")])
print("== pushing arc", th0, "->", th1)
traj, frac = r.cartesian(arc, avoid=True)
if frac < 0.99: sys.exit(f"arc fraction {frac}")
code, err = r.execute(traj); p, R = r.tcp(); print("tcp after arc", np.round(p, 3), "err", round(err, 4))
print("== retreat")
d1, n11 = dn(th1)
r.move_line_tcp([p - 0.05*n11, p - 0.05*n11 + [0, 0, 0.06]], R, avoid=False, min_fraction=0.9)
print("done; joints", np.round(r.joints(), 3))
OPENRUA_EOF

# openrua op 166
timeout 300 python3 doorpush.py -114 2 2>&1 | grep -v '^\s*$'

# openrua op 167
cat > /tmp/vt.py <<'EOF'
import sys; sys.argv=["x","-114","2"]
src=open('/workspace/doorpush.py').read().split("# --- plan")[0]
exec(src)
scene.apply(r.node, [door_box(th0)])
for back in [0.0,0.005,0.01,0.015,0.02,0.03]:
    t,R=pose(th0,back=back); q=r.solve_ik(hand_pose_from_tcp(t,R),R,seed=START_Q,timeout=2.0)
    print(back, valid(q) if q else "noik")
EOF
timeout 120 python3 /tmp/vt.py 2>&1 | grep -v '^\s*$'

# openrua op 168
timeout 120 python3 /tmp/vt.py 2>&1 | grep -v '^\s*$'

# openrua op 169
cp /tmp/vt.py /workspace/vt.py && timeout 120 python3 /workspace/vt.py 2>&1 | grep -v '^\s*$'

# openrua op 170
sed -i 's/LEAN, RHO, Z, OFF = 15.0, 0.12, 1.075, -0.032/LEAN, RHO, Z, OFF = 15.0, 0.12, 1.075, -0.040/; s/door_box(th0, thick=0.03)\])          # thin door: tests hand body vs door, not the 2 mm tip overlap/door_box(th0, thick=0.03)])          # thin door (face 2.5 mm back): tests hand body vs door, not the tip touch/' doorpush.py && timeout 300 python3 doorpush.py -114 2 2>&1 | grep -v '^\s*$'

# openrua op 171
timeout 1500 python3 doorpush.py -114 2 --go 2>&1 | grep -v '^\s*$'

# openrua op 172
timeout 120 python3 doorangle.py --fresh; python3 -c "
import numpy as np, cv2
P=np.load('birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(Z>1.085)&(Z<1.125)&(X>-0.36)&(X<0.2)&(Y<0.5)&(Y>-0.2)
n,lab,st,ce=cv2.connectedComponentsWithStats(m.astype(np.uint8))
for i in range(1,n):
    if st[i,4]<15: continue
    p=P[lab==i]; print(i, st[i,4], 'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
"

# openrua op 173
timeout 60 python3 tools/perception/cam_snap.py frontview snaps/front_closed.png >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/agent_closed.png >/dev/null 2>&1; ls -la snaps/front_closed.png snaps/agent_closed.png

# openrua op 174
for c in sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/${c}_closed.png >/dev/null 2>&1; done; ls snaps/*_closed.png

# openrua op 175
timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird_closed.png >/dev/null 2>&1; python3 -c "
import cv2; im=cv2.imread('snaps/bird_closed.png'); h,w=im.shape[:2]; print(h,w)
c=im[int(h*0.15):int(h*0.75), int(w*0.25):int(w*0.85)]; cv2.imwrite('snaps/bird_closed_crop.png', cv2.resize(c,None,fx=1.6,fy=1.6))"

# openrua op 176
python3 -c "
import numpy as np
P=np.load('birdview_world.npy'); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(Z>1.085)&(Z<1.115)&(X>-0.2)&(X<0.2)
for x0 in np.arange(-0.18,0.18,0.03):
    s=m&(X>=x0)&(X<x0+0.03)
    if s.sum(): print('x[%.2f,%.2f] front edge y_min=%.3f  n=%d'%(x0,x0+0.03,Y[s].min(),s.sum()))
# anything tall left of hinge / in front of the door (open-door region)?
o=(Z>0.95)&(Z<1.13)&(X>-0.36)&(X<-0.17)&(Y>-0.05)&(Y<0.25)
print('points in open-door region:', o.sum())
"

# openrua op 177
timeout 60 python3 -c "
from rob import *; import numpy as np, time
r=Robot('chk'); q1=np.array(r.joints()); r.spin(1.0); q2=np.array(r.joints())
print('joints', np.round(q2,3), 'drift', np.abs(q2-q1).max()); print('tcp', np.round(r.tcp()[0],3), 'fingers', np.round(r.fingers(),4))
" 2>&1 | grep -v '^\s*$'
