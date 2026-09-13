#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ---ACTIONS; ros2 action list; echo ---SERVICES; ros2 service list | head -50; echo ---NODES; ros2 node list

# openrua op 4
ros2 topic echo /joint_states --once; echo ---; ros2 node info /robot_bridge; echo ---; ros2 topic echo /pipeline_state --once 2>&1 | head -30

# openrua op 5
cat /tmp/claude-1000/-workspace/dcd9b893-c459-4201-a1d5-d46a48ae17c8/tasks/bcmbcwk4q.output

# openrua op 6
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand galleryview paperview; do timeout 30 python3 tools/perception/cam_snap.py $c || echo "FAIL $c"; done

# openrua op 7
timeout 60 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -20; echo ---; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | sort | uniq

# openrua op 8
cat > tfdump.py <<'EOF'
import rclpy, sys
from tf2_ros import Buffer, TransformListener
rclpy.init(); node = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, node)
import time
for _ in range(30): rclpy.spin_once(node, timeout_sec=0.2)
print(buf.all_frames_as_string())
for f in sys.argv[1:]:
    try:
        t = buf.lookup_transform("world", f, rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        print(f, "xyz=(%.4f %.4f %.4f) q=(%.4f %.4f %.4f %.4f)"%(tr.x,tr.y,tr.z,q.x,q.y,q.z,q.w))
    except Exception as e: print(f, "ERR", e)
rclpy.shutdown()
EOF
timeout 60 python3 tfdump.py panda_link0 panda_hand agentview_optical_frame birdview_optical_frame robot0_eye_in_hand_optical_frame

# openrua op 9
cat > cloud.py <<'EOF'
"""Grab depth+color+info+TF for a camera; save world-frame point cloud as npz."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub); return got["m"]

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
buf = Buffer(); TransformListener(buf, node)
depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float32)
color = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
while not buf.can_transform("world", frame, rclpy.time.Time()): rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; R = quat_R(q.x,q.y,q.z,q.w)
p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = depth.shape
v, u = np.mgrid[0:H, 0:W]
pc = np.stack([(u-cx)*depth/fx, (v-cy)*depth/fy, depth], -1)
pw = pc @ R.T + p0
np.savez(f"{cam}_cloud.npz", pw=pw, color=color, depth=depth)
print("saved", cam, H, W, "depth range", np.nanmin(depth), np.nanmax(depth))
rclpy.shutdown()
EOF
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview

# openrua op 10
cat > q.py <<'EOF'
import numpy as np, sys
d = np.load(sys.argv[1]); pw = d["pw"]; col = d["color"]
def at(u,v): 
    p = pw[v,u]; print(f"px({u},{v}) -> world ({p[0]:.3f},{p[1]:.3f},{p[2]:.3f}) bgr={col[v,u]}")
for a in sys.argv[2:]:
    u,v = map(int,a.split(",")); at(u,v)
EOF
# birdview: table, white mug, yellow mug, microwave body corners, door
python3 q.py birdview_cloud.npz 200,400 247,270 247,262 320,305 320,297 400,250 455,250 400,345 455,345 428,300 350,228 395,240 392,330 300,180 160,160 470,160

# openrua op 11
cat > seg.py <<'EOF'
import numpy as np
d = np.load("birdview_cloud.npz"); pw = d["pw"]; col = d["color"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
# microwave region: above table, y>0.1
m = (Z > 0.93) & (Y > 0.05) & (X > -0.5)
print("microwave-ish pts:", m.sum())
for zlo, zhi in [(0.93,1.0),(1.0,1.05),(1.05,1.09),(1.09,1.12)]:
    mm = m & (Z>=zlo) & (Z<zhi)
    if mm.sum(): print(f"z[{zlo},{zhi}) n={mm.sum()} x[{X[mm].min():.3f},{X[mm].max():.3f}] y[{Y[mm].min():.3f},{Y[mm].max():.3f}]")
# top face
top = m & (Z>1.09)
print("top face x", X[top].min(), X[top].max(), "y", Y[top].min(), Y[top].max())
# Door: thin thing - print a grid of z in region x[-0.3,0.2], y[0.05,0.5]
xs = np.arange(-0.30, 0.20, 0.025); ys = np.arange(0.05, 0.50, 0.025)
print("z grid (rows=x, cols=y)")
print("      " + " ".join(f"{y:5.2f}" for y in ys))
for x in xs:
    row = []
    for y in ys:
        mm = (np.abs(X-x)<0.0125)&(np.abs(Y-y)<0.0125)
        row.append(f"{Z[mm].max():5.2f}" if mm.sum() else "  -  ")
    print(f"{x:5.2f} " + " ".join(row))
EOF
python3 seg.py

# openrua op 12
cat > door.py <<'EOF'
import numpy as np
for cam in ["birdview","agentview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
    m = (Z>0.95)&(Z<1.13)&(X<-0.16)&(X>-0.6)&(Y>-0.1)&(Y<0.3)
    print(cam, "door-cand pts", m.sum())
    if m.sum()==0: continue
    xs, ys, zs = X[m], Y[m], Z[m]
    # bin by x
    for x in np.arange(-0.55,-0.15,0.02):
        mm = (xs>=x)&(xs<x+0.02)
        if mm.sum(): print(f"  x[{x:.2f}] n={mm.sum():4d} y[{ys[mm].min():.3f},{ys[mm].max():.3f}] z[{zs[mm].min():.3f},{zs[mm].max():.3f}]")
EOF
python3 door.py

# openrua op 13
cat > cav.py <<'EOF'
import numpy as np
d = np.load("agentview_cloud.npz"); pw = d["pw"]; col=d["color"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
# microwave body front region: x in [-0.17,0.17], y in [0.25,0.5]
m = (X>-0.18)&(X<0.18)&(Y>0.2)&(Y<0.6)&(Z>0.89)&(Z<1.15)
print("pts", m.sum())
xs,ys,zs = X[m],Y[m],Z[m]
print("y bins:")
for y in np.arange(0.2,0.6,0.02):
    mm=(ys>=y)&(ys<y+0.02)
    if mm.sum(): print(f"  y[{y:.2f}] n={mm.sum():5d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] z[{zs[mm].min():.3f},{zs[mm].max():.3f}]")
print("z bins (y>0.3):")
for z in np.arange(0.89,1.15,0.01):
    mm=(zs>=z)&(zs<z+0.01)&(ys>0.3)
    if mm.sum(): print(f"  z[{z:.2f}] n={mm.sum():5d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
EOF
python3 cav.py

# openrua op 14
cat > cav2.py <<'EOF'
import numpy as np
d = np.load("agentview_cloud.npz"); pw = d["pw"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
m = (X>-0.16)&(X<0.16)&(Y>0.29)&(Y<0.47)&(Z>0.905)&(Z<1.095)
xs,ys,zs = X[m],Y[m],Z[m]
print("interior pts", m.sum())
xb = np.arange(-0.16,0.16,0.02); yb=np.arange(0.29,0.47,0.02)
print("min z grid rows=x cols=y"); print("      "+" ".join(f"{y:5.2f}" for y in yb))
for x in xb:
    row=[]
    for y in yb:
        mm=(xs>=x)&(xs<x+0.02)&(ys>=y)&(ys<y+0.02)
        row.append(f"{zs[mm].min():5.3f}/{zs[mm].max():5.3f}" if mm.sum() else "     -     ")
    print(f"{x:5.2f} "+" ".join(row))
EOF
python3 cav2.py

# openrua op 15
cat > mug.py <<'EOF'
import numpy as np
d = np.load("birdview_cloud.npz"); pw = d["pw"]; col=d["color"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
for name,(x0,y0) in {"white":(-0.11,-0.25),"yellow":(0.01,0.0)}.items():
    m = (np.abs(X-x0)<0.09)&(np.abs(Y-y0)<0.09)&(Z>0.905)
    xs,ys,zs=X[m],Y[m],Z[m]
    print(name, "pts", m.sum(), "z range", zs.min(), zs.max())
    for z in np.arange(0.90,1.0,0.01):
        mm=(zs>=z)&(zs<z+0.01)
        if mm.sum(): print(f"  z[{z:.2f}] n={mm.sum():4d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
    top = zs>zs.max()-0.01
    print("  top-rim centroid", xs[top].mean(), ys[top].mean(), "extent x", xs[top].max()-xs[top].min(), "y", ys[top].max()-ys[top].min())
EOF
python3 mug.py

# openrua op 16
timeout 120 python3 cloud.py frontview && cat > door2.py <<'EOF'
import numpy as np
for cam in ["frontview","agentview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
    m = (Z>0.93)&(Z<1.13)&(X<-0.165)&(X>-0.6)&(Y>-0.2)&(Y<0.32)
    xs,ys,zs=X[m],Y[m],Z[m]
    # project onto door direction from hinge
    h = np.array([-0.17,0.28])
    dvec = np.stack([xs-h[0], ys-h[1]],1)
    ang = np.degrees(np.arctan2(dvec[:,1],dvec[:,0]))
    r = np.linalg.norm(dvec,axis=1)
    print(cam, "n", m.sum(), "angle pct", np.percentile(ang,[1,5,50,95,99]).round(1), "r max", r.max().round(3), "r pct", np.percentile(r,[50,90,99,99.9]).round(3))
    # tip: points with largest r
    idx = np.argsort(r)[-30:]
    print("  tip pts x,y,z:", np.c_[xs[idx],ys[idx],zs[idx]].mean(0).round(3))
    # thickness: perpendicular spread for a given angle
    a0 = np.median(ang); 
    perp = dvec @ np.array([-np.sin(np.radians(a0)), np.cos(np.radians(a0))])
    print("  median angle", a0.round(1), "perp spread pct", np.percentile(perp,[1,50,99]).round(3))
    print("  z range", zs.min().round(3), zs.max().round(3))
EOF
python3 door2.py

# openrua op 17
cat > door3.py <<'EOF'
import numpy as np
d = np.load("agentview_cloud.npz"); pw = d["pw"]; col=d["color"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
m = (Z>0.93)&(Z<1.13)&(X<-0.165)&(X>-0.5)&(Y>-0.2)&(Y<0.32)
xs,ys,zs=X[m],Y[m],Z[m]
h = np.array([-0.17,0.28])
dvec = np.stack([xs-h[0], ys-h[1]],1); r = np.linalg.norm(dvec,axis=1)
ang = np.degrees(np.arctan2(dvec[:,1],dvec[:,0]))
for r0 in np.arange(0.0,0.45,0.02):
    mm=(r>=r0)&(r<r0+0.02)
    if mm.sum(): print(f"r[{r0:.2f}] n={mm.sum():5d} ang[{np.percentile(ang[mm],5):.1f},{np.percentile(ang[mm],95):.1f}] z[{zs[mm].min():.3f},{zs[mm].max():.3f}] x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
EOF
python3 door3.py

# openrua op 18
cat > door4.py <<'EOF'
import numpy as np
d = np.load("agentview_cloud.npz"); pw = d["pw"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
m = (Z>0.95)&(Z<1.10)&(X<-0.18)&(X>-0.5)&(Y>-0.1)&(Y<0.30)
P = np.c_[X[m],Y[m]]
c = P.mean(0); u,s,vt = np.linalg.svd(P-c, full_matrices=False)
dirv = vt[0]; 
if dirv[0] > 0: dirv = -dirv   # point from hinge toward tip (toward -x)
t = (P-c)@dirv; perp=(P-c)@vt[1]
print("centroid", c.round(3), "dir", dirv.round(3), "angle", np.degrees(np.arctan2(dirv[1],dirv[0])).round(1))
print("t range", np.percentile(t,[0.5,99.5]).round(3), "perp spread", np.percentile(perp,[0.5,99.5]).round(3))
hinge_end = c + dirv*np.percentile(t,0.5); tip_end = c + dirv*np.percentile(t,99.5)
print("hinge end", hinge_end.round(3), "tip end", tip_end.round(3), "length", (np.percentile(t,99.5)-np.percentile(t,0.5)).round(3))
# frontview too
d = np.load("frontview_cloud.npz"); pw = d["pw"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
m = (Z>0.95)&(Z<1.10)&(X<-0.18)&(X>-0.5)&(Y>-0.1)&(Y<0.30)
P = np.c_[X[m],Y[m]]
t=(P-c)@dirv; print("frontview t range", np.percentile(t,[0.5,99.5]).round(3), "perp", np.percentile((P-c)@vt[1],[0.5,99.5]).round(3))
EOF
python3 door4.py

# openrua op 19
cat > urdf.py <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); node=rclpy.create_node("u")
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(String,"/robot_description",got.append,qos)
while not got: rclpy.spin_once(node,timeout_sec=0.5)
open("robot.urdf","w").write(got[0].data); print(len(got[0].data))
rclpy.shutdown()
EOF
timeout 60 python3 urdf.py && grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A6 'joint name="panda_finger_joint1"' robot.urdf; grep -n -A8 'link name="panda_leftfinger"' robot.urdf | head -30; grep -n -A6 'joint name="panda_hand_joint"\|joint name="panda_joint8"' robot.urdf

# openrua op 20
python3 - <<'EOF'
import re
s=open("robot.urdf").read()
for name in ["panda_hand","panda_leftfinger","panda_rightfinger"]:
    i=s.find(f'<link name="{name}"'); j=s.find("</link>",i); print(s[i:j][:600]); print()
for jn in ["panda_joint8","panda_hand_joint","panda_finger_joint1","panda_finger_joint2"]:
    i=s.find(f'<joint name="{jn}"'); j=s.find("</joint>",i); print(s[i:j]); print()
EOF
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/

# openrua op 21
find / -name "hand.stl" -o -name "finger.stl" -o -name "hand.dae" 2>/dev/null | head; find / -iname "*microwave*" 2>/dev/null | head

# openrua op 22
mkdir -p "$(dirname /workspace/lib.py)"
cat > /workspace/lib.py <<'OPENRUA_EOF'
"""Shared helpers: joint state, FK/IK (world frame, panda_hand), trajectories, gripper, snapshots."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import cv2

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation (TF, rotation identity)
FINGERS = ["panda_finger_joint1", "panda_finger_joint2"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    m = R
    t = np.trace(m)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def R_from_axes(x_hand, y_hand, z_hand):
    """Rotation whose columns are the hand axes expressed in world."""
    R = np.column_stack([x_hand, y_hand, z_hand])
    assert np.linalg.det(R) > 0.99, np.linalg.det(R)
    return R


R_DOWN = quat_to_R([1, 0, 0, 0])  # hand z pointing down (-Z world), fingers along world y


class Robot:
    def __init__(self, name="robot_lib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.bridge = CvBridge()
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10)
        self.fk_cli.wait_for_service(10)

    def _on_js(self, msg):
        self._js = msg

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None:
                self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger_gap(self):
        d = self.joints()
        return d[FINGERS[0]] - d[FINGERS[1]]  # joint2 reads negative here

    def fk(self, q=None, link="panda_hand"):
        """World pose (p, R) of link for arm config q (default: current)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        ps = res.pose_stamped[0].pose
        p = np.array([ps.position.x, ps.position.y, ps.position.z]) + BASE
        R = quat_to_R([ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w])
        return p, R

    def ik(self, p_world, R, seed=None, link="panda_hand", timeout=2.0):
        """Arm joints placing the hand frame at (p_world, R). None if no solution."""
        if seed is None:
            seed = self.arm_q()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.ik_link_name = link
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.pose_stamped.header.frame_id = ""
        pose = req.ik_request.pose_stamped.pose
        pb = np.asarray(p_world, float) - BASE
        pose.position.x, pose.position.y, pose.position.z = map(float, pb)
        q = R_to_quat(R)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK service timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---------- acting ----------
    def move(self, waypoints, times):
        """Send one trajectory through joint waypoints at cumulative times (s)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
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
        if rf.result() is None:
            raise RuntimeError("trajectory result timeout")
        code = rf.result().result.error_code
        q_now = np.array(self.arm_q())
        err = np.abs(q_now - np.array(waypoints[-1])).max()
        print(f"  move: error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        rf = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def snap(self, cam, out=None):
        topic = f"/{cam}/color/image_raw"
        got = []
        sub = self.node.create_subscription(Image, topic, got.append, 1)
        while not got:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, self.bridge.imgmsg_to_cv2(got[0], "bgr8"))
        return out

    # ---------- helpers ----------
    def cartesian_waypoints(self, p0, p1, R, n, seed):
        """IK along a straight line p0->p1 (n segments) at fixed R; returns joint list."""
        qs = []
        q = seed
        for i in range(1, n + 1):
            p = p0 + (p1 - p0) * i / n
            q = self.ik(p, R, seed=q)
            if q is None:
                raise RuntimeError(f"IK failed at waypoint {i}/{n}: {p}")
            qs.append(q)
        return qs

    def report(self, tag=""):
        p, R = self.fk()
        print(f"  [{tag}] hand p={np.round(p,4)} z_axis={np.round(R[:,2],3)} y_axis={np.round(R[:,1],3)} gap={self.finger_gap():.4f}", flush=True)
        return p, R
OPENRUA_EOF

# openrua op 23
cat > t0.py <<'EOF'
from lib import *
r = Robot("t0")
q = r.arm_q(); print("q", np.round(q,4), "gap", r.finger_gap())
p, R = r.fk(q); print("FK hand world", np.round(p,4)); print(np.round(R,3))
p8, R8 = r.fk(q, "panda_link8"); print("FK link8", np.round(p8,4)); print(np.round(R8,3))
# IK sanity: solve for the current pose
sol = r.ik(p, R, seed=[v+0.05 for v in q]); print("IK back:", None if sol is None else np.round(sol,4))
EOF
timeout 120 python3 t0.py

# openrua op 24
grep -o '<link name="[^"]*"' robot.urdf | head -3; grep -o '<joint name="[^"]*" type="fixed"><origin[^>]*><parent link="[^"]*"/><child link="panda_link0"' robot.urdf; python3 - <<'EOF'
s=open("robot.urdf").read()
i=s.find('<child link="panda_link0"'); print(s[i-300:i+40])
EOF

# openrua op 25
grep -c 'name="world"' robot.urdf; grep -o 'virtual_joint[^/]*' robot.urdf | head; ros2 param get /move_group robot_description_semantic 2>/dev/null | grep -i -o 'virtual_joint[^/]*' | head -3

# openrua op 26
sed -i 's/^BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.array([0.0, 0.0, 0.0])  # FK\/IK poses come back in world already (SRDF virtual joint world->panda_link0 is populated)/' lib.py && cat > t1.py <<'EOF'
from lib import *
r = Robot("t1")
q = r.arm_q()
p, R = r.fk(q); print("FK hand world", np.round(p,4))
# IK for a pose 10cm lower, straight down orientation; FK the result
target = p + np.array([0.1, 0, -0.1])
sol = r.ik(target, R_DOWN, seed=q); print("IK:", None if sol is None else np.round(sol,4))
if sol: 
    p2,R2 = r.fk(sol); print("FK(sol)", np.round(p2,4), "target", np.round(target,4)); print(np.round(R2,3))
# horizontal grasp orientation test near white mug pregrasp
R_H = R_from_axes(np.array([0,0,1.]), np.array([1.,0,0]), np.array([0,1.,0]))
for pt in [(-0.10,-0.40,0.96),(-0.10,-0.40,1.0),(0.0,0.10,1.10),(0.0,0.28,1.005),(0.0,0.20,1.005)]:
    s = r.ik(np.array(pt), R_H, seed=q); print("R_H", pt, "->", None if s is None else np.round(s,3))
R_H2 = R_from_axes(np.array([0,0,-1.]), np.array([-1.,0,0]), np.array([0,1.,0]))
for pt in [(-0.10,-0.40,0.96),(0.0,0.28,1.005)]:
    s = r.ik(np.array(pt), R_H2, seed=q); print("R_H2", pt, "->", None if s is None else np.round(s,3))
EOF
timeout 300 python3 t1.py

# openrua op 27
cat > t2.py <<'EOF'
from lib import *
import random
r = Robot("t2")
q0 = r.arm_q()
def Rpitch(pitch_deg, yaw_axis='+y'):
    # approach axis: +y rotated downward by pitch about x
    th = np.radians(pitch_deg)
    z_h = np.array([0, np.cos(th), -np.sin(th)])
    y_h = np.array([1.,0,0])
    x_h = np.cross(y_h, z_h)
    return R_from_axes(x_h, y_h, z_h)
seeds = [q0] + [[random.uniform(*l) for l in FJT["limits_rad"]] for _ in range(6)]
for pitch in [0, 15, 30, 45]:
    R = Rpitch(pitch)
    for pt in [(-0.10,-0.36,0.96),(-0.10,-0.36,1.0),(-0.10,-0.40,1.0),(-0.10,-0.34,0.97)]:
        ok=None
        for s in seeds:
            sol = r.ik(np.array(pt), R, seed=s, timeout=1.0)
            if sol: ok=sol; break
        print(f"pitch={pitch} pt={pt} -> {None if ok is None else np.round(ok,3)}", flush=True)
EOF
timeout 600 python3 t2.py

# openrua op 28
cat > mug2.py <<'EOF'
import numpy as np
for cam in ["agentview","frontview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
    m = (np.abs(X+0.10)<0.08)&(np.abs(Y+0.255)<0.08)&(Z>0.905)&(Z<1.0)
    xs,ys,zs=X[m],Y[m],Z[m]
    print(cam, "white mug pts", m.sum())
    for z in np.arange(0.90,1.0,0.01):
        mm=(zs>=z)&(zs<z+0.01)
        if mm.sum()>3: print(f"  z[{z:.2f}] n={mm.sum():4d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}] xc={(xs[mm].min()+xs[mm].max())/2:.3f}")
EOF
python3 mug2.py

# openrua op 29
cat > mug3.py <<'EOF'
import numpy as np
for cam in ["agentview","frontview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
    for name,(x0,y0) in {"white":(-0.09,-0.255),"yellow":(0.024,-0.003)}.items():
        m = (np.abs(X-x0)<0.08)&(np.abs(Y-y0)<0.08)&(Z>0.905)&(Z<1.25)
        xs,ys,zs=X[m],Y[m],Z[m]
        print(cam, name, "pts", m.sum())
        for z in np.arange(0.90,1.15,0.01):
            mm=(zs>=z)&(zs<z+0.01)
            if mm.sum()>3: print(f"  z[{z:.2f}] n={mm.sum():4d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}] xc={(xs[mm].min()+xs[mm].max())/2:.3f} yc={(ys[mm].min()+ys[mm].max())/2:.3f}")
EOF
python3 mug3.py

# openrua op 30
cat > warm.py <<'EOF'
from lib import *
r = Robot("warm")
print("gap before", r.finger_gap())
r.gripper(0.04)
r.report("after open")
EOF
timeout 600 python3 warm.py && timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview && python3 mug3.py 2>&1 | grep -A3 "birdview white\|agentview white" && python3 q.py birdview_cloud.npz 200,400 428,300 395,240

# openrua op 31
timeout 120 python3 cloud.py frontview >/dev/null; python3 mug3.py; python3 door4.py; python3 cav2.py | head -12

# openrua op 32
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
"""Planning-scene helpers: publish world collision boxes, validate joint states."""
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene, RobotState, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from shape_msgs.msg import SolidPrimitive

from lib import ARM, R_to_quat

# ---------- measured world geometry (metres, world frame) ----------
TABLE_Z = 0.90
MW = dict(x0=-0.175, x1=0.166, y0=0.27, y1=0.46, z0=TABLE_Z, z1=1.107)  # microwave exterior
CAV = dict(x0=-0.15, x1=0.14, y0=0.27, y1=0.44, z0=0.945, z1=1.07)  # cavity (z1 = guess until measured)
DOOR_HINGE = np.array([-0.182, 0.267])
DOOR_TIP = np.array([-0.304, 0.041])
DOOR_Z = (0.93, 1.108)
DOOR_THICK = 0.03
YELLOW = dict(x0=-0.03, x1=0.075, y0=-0.085, y1=0.05, z0=TABLE_Z, z1=1.0)
WHITE_MUG = dict(cx=-0.0985, cy=-0.2535, r=0.047, h=0.115)  # rim radius; tapers to ~0.03 at the base


def box(name, lo, hi, frame="world"):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, hi - lo)))
    pose = Pose()
    c = (lo + hi) / 2
    pose.position.x, pose.position.y, pose.position.z = map(float, c)
    pose.orientation.w = 1.0
    co.primitives = [prim]
    co.primitive_poses = [pose]
    co.operation = CollisionObject.ADD
    return co


def oriented_box(name, center, size, R, frame="world"):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(map(float, size)))
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, center)
    q = R_to_quat(R)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
    co.primitives = [prim]
    co.primitive_poses = [pose]
    co.operation = CollisionObject.ADD
    return co


def door_object(hinge=DOOR_HINGE, tip=DOOR_TIP):
    d = tip - hinge
    L = np.linalg.norm(d)
    ang = np.arctan2(d[1], d[0])
    R = np.array([[np.cos(ang), -np.sin(ang), 0], [np.sin(ang), np.cos(ang), 0], [0, 0, 1]])
    c2 = (hinge + tip) / 2
    center = [c2[0], c2[1], (DOOR_Z[0] + DOOR_Z[1]) / 2]
    return oriented_box("door", center, [L, DOOR_THICK, DOOR_Z[1] - DOOR_Z[0]], R)


def world_objects(with_white_mug=True, cav=CAV, door=True):
    objs = [
        box("table", [-0.55, -0.65, TABLE_Z - 0.05], [0.45, 0.60, TABLE_Z]),
        box("mw_floor", [MW["x0"], MW["y0"], MW["z0"]], [MW["x1"], MW["y1"], cav["z0"]]),
        box("mw_ceiling", [MW["x0"], MW["y0"], cav["z1"]], [MW["x1"], MW["y1"], MW["z1"]]),
        box("mw_left", [MW["x0"], MW["y0"], MW["z0"]], [cav["x0"], MW["y1"], MW["z1"]]),
        box("mw_right", [cav["x1"], MW["y0"], MW["z0"]], [MW["x1"], MW["y1"], MW["z1"]]),
        box("mw_back", [MW["x0"], cav["y1"], MW["z0"]], [MW["x1"], MW["y1"], MW["z1"]]),
        box("yellow_mug", [YELLOW["x0"], YELLOW["y0"], YELLOW["z0"]], [YELLOW["x1"], YELLOW["y1"], YELLOW["z1"]]),
    ]
    if door:
        objs.append(door_object())
    if with_white_mug:
        w = WHITE_MUG
        objs.append(box("white_mug", [w["cx"] - w["r"], w["cy"] - w["r"], TABLE_Z],
                        [w["cx"] + w["r"], w["cy"] + w["r"] + 0.03, TABLE_Z + w["h"]]))
    return objs


class Scene:
    def __init__(self, node):
        self.node = node
        self.apply = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.valid = node.create_client(GetStateValidity, "/check_state_validity")
        self.apply.wait_for_service(10)
        self.valid.wait_for_service(10)

    def publish(self, objects, remove=()):
        ps = PlanningScene()
        ps.is_diff = True
        for name in remove:
            co = CollisionObject()
            co.header.frame_id = "world"
            co.id = name
            co.operation = CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        ps.world.collision_objects.extend(objects)
        req = ApplyPlanningScene.Request(scene=ps)
        fut = self.apply.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        return fut.result().success

    def attach(self, name, lo, hi, link="panda_hand", detach=False):
        """Attach a box (given in hand-frame lo/hi) to the hand, or detach it."""
        ps = PlanningScene()
        ps.is_diff = True
        ps.robot_state.is_diff = True
        aco = AttachedCollisionObject()
        aco.link_name = link
        aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger"]
        if detach:
            aco.object.id = name
            aco.object.operation = CollisionObject.REMOVE
        else:
            aco.object = box(name, lo, hi, frame=link)
        ps.robot_state.attached_collision_objects.append(aco)
        if detach:  # also drop it from the world so it does not linger
            co = CollisionObject()
            co.header.frame_id = "world"
            co.id = name
            co.operation = CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        fut = self.apply.call_async(ApplyPlanningScene.Request(scene=ps))
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        return fut.result().success

    def check(self, q, verbose=True):
        req = GetStateValidity.Request()
        req.group_name = "panda_arm"
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.valid.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None:
            raise RuntimeError("validity service timeout")
        if verbose and not res.valid:
            pairs = {(c.contact_body_1, c.contact_body_2) for c in res.contacts}
            print("    INVALID contacts:", sorted(pairs), flush=True)
        return res.valid, [(c.contact_body_1, c.contact_body_2) for c in res.contacts]
OPENRUA_EOF

# openrua op 33
cat > t3.py <<'EOF'
from lib import *
from scene import *
r = Robot("t3"); sc = Scene(r.node)
print("publish:", sc.publish(world_objects()))
q0 = r.arm_q()
print("current valid:", sc.check(q0))
R_H  = R_from_axes(np.array([0,0,1.]),  np.array([1.,0,0]),  np.array([0,1.,0]))
R_H2 = R_from_axes(np.array([0,0,-1.]), np.array([-1.,0,0]), np.array([0,1.,0]))
for name,R in [("R_H",R_H),("R_H2",R_H2)]:
    for pt in [(-0.05,-0.12,1.04),(-0.05,-0.15,1.04),(-0.07,-0.15,1.0)]:
        s = r.ik(np.array(pt), R, seed=q0)
        if s is None: print(name, pt, "IK none"); continue
        v,_ = sc.check(s); print(name, pt, "q=",np.round(s,3), "valid",v)
EOF
timeout 300 python3 t3.py

# openrua op 34
cat > t4.py <<'EOF'
from lib import *
from scene import *
r = Robot("t4"); sc = Scene(r.node)
q0 = r.arm_q()
def R_look(zdir, xdir_hint=np.array([0,0,-1.])):
    z = np.asarray(zdir,float); z/=np.linalg.norm(z)
    x = xdir_hint - z*np.dot(xdir_hint,z); x/=np.linalg.norm(x)
    y = np.cross(z,x)
    return R_from_axes(x,y,z)
cands = [((0.0,0.05,1.06),(0,1,0)),((0.0,0.02,1.07),(0,1,0)),((0.05,0.0,1.07),(0,1,0)),((0.0,0.0,1.08),(0,1,0)),
         ((0.2,-0.1,1.03),(-0.406,0.914,0)),((0.2,-0.05,1.05),(-0.406,0.914,0)),((0.1,0.0,1.07),(-0.2,0.98,0))]
for pt, zd in cands:
    R = R_look(zd)
    s = r.ik(np.array(pt), R, seed=q0)
    if s is None: print(pt, zd, "IK none"); continue
    v,_ = sc.check(s); print(pt, zd, "q=",np.round(s,3), "valid",v, flush=True)
EOF
timeout 300 python3 t4.py

# openrua op 35
cat >> scene.py <<'EOF'


def path_valid(sc, q_from, q_to, steps=20, verbose=False):
    """Linear joint-space interpolation collision check."""
    q_from, q_to = np.asarray(q_from, float), np.asarray(q_to, float)
    bad = []
    for i in range(steps + 1):
        q = q_from + (q_to - q_from) * i / steps
        v, contacts = sc.check(q, verbose=False)
        if not v:
            bad.append((i, sorted({tuple(c) for c in contacts})))
    if verbose and bad:
        for b in bad:
            print(f"    step {b[0]}/{steps}: {b[1]}", flush=True)
    return not bad, bad
EOF
cat > look.py <<'EOF'
from lib import *
from scene import *
r = Robot("look"); sc = Scene(r.node)
sc.publish(world_objects())
q0 = r.arm_q()
R = R_from_axes(np.array([0,0,-1.]), np.array([-1.,0,0]), np.array([0,1.,0]))
target = np.array([0.0, 0.02, 1.07])
q1 = r.ik(target, R, seed=q0)
print("q1", np.round(q1,3))
ok, bad = path_valid(sc, q0, q1, steps=30, verbose=True)
print("direct path valid:", ok)
if not ok:
    # try via an intermediate high pose: same xy as target but z=1.35, pointing down
    qi = r.ik(np.array([-0.05, 0.0, 1.35]), R_DOWN, seed=q0)
    print("qi", np.round(qi,3), "valid", sc.check(qi)[0])
    ok1,_ = path_valid(sc, q0, qi, 30, True); ok2,_ = path_valid(sc, qi, q1, 30, True)
    print("via intermediate:", ok1, ok2)
    if ok1 and ok2:
        r.move([qi, q1], [4.0, 9.0])
else:
    r.move([q1], [5.0])
r.report("look pose")
EOF
timeout 900 python3 -u look.py 2>&1 | tee look.log

# openrua op 36
cat > st.py <<'EOF'
from lib import *
r = Robot("st")
q = r.arm_q(); print("q now", np.round(q,3))
tgt = np.array([0.035,0.915,-0.235,-1.539,1.606,1.358,1.479]); print("diff", np.round(np.array(q)-tgt,3))
r.report("now")
for c in ["agentview","frontview"]: r.snap(c)
EOF
timeout 120 python3 st.py

# openrua op 37
cat > resend.py <<'EOF'
from lib import *
import sys
r = Robot("resend")
tgt = [float(v) for v in sys.argv[1].split(",")]; T=float(sys.argv[2])
q0 = np.array(r.arm_q()); print("start diff", np.round(np.array(tgt)-q0,3))
r.move([tgt],[T])
q1 = np.array(r.arm_q()); print("end diff", np.round(np.array(tgt)-q1,3))
r.report("after")
EOF
timeout 900 python3 -u resend.py 0.035,0.915,-0.235,-1.539,1.606,1.358,1.479 4.0

# openrua op 38
timeout 120 python3 tfdump.py panda_hand robot0_eye_in_hand_optical_frame | tail -2 && timeout 120 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand_cloud.npz'); cv2.imwrite('eih.png', d['color'])
dep=d['depth']; print('depth center', dep[240,320], 'min', np.nanmin(dep), 'max', np.nanmax(dep))
"

# openrua op 39
cat > cav3.py <<'EOF'
import numpy as np
d = np.load("robot0_eye_in_hand_cloud.npz"); pw = d["pw"]; dep=d["depth"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(dep)&(dep>0.05)
# front face of microwave: y in [0.25,0.29]
m = ok&(Y>0.25)&(Y<0.295)&(X>-0.25)&(X<0.25)&(Z>0.9)&(Z<1.15)
print("front face pts", m.sum(), "y median", np.median(Y[m]).round(4))
# interior: y>0.30
mi = ok&(Y>0.30)&(X>-0.2)&(X<0.2)&(Z>0.9)&(Z<1.15)
print("interior pts", mi.sum(), "y range", Y[mi].min().round(3), Y[mi].max().round(3))
xs,ys,zs = X[mi],Y[mi],Z[mi]
print("interior z range", zs.min().round(4), zs.max().round(4), " x range", xs.min().round(4), xs.max().round(4))
# ceiling: points with z> 1.03 -> distribution
for z in np.arange(1.03,1.12,0.005):
    mm=(zs>=z)&(zs<z+0.005)
    if mm.sum(): print(f"  z[{z:.3f}] n={mm.sum():5d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
print("floor:")
for z in np.arange(0.93,0.97,0.005):
    mm=(zs>=z)&(zs<z+0.005)
    if mm.sum(): print(f"  z[{z:.3f}] n={mm.sum():5d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
print("walls (x hist for interior, z in 0.96..1.04):")
mm=(zs>0.96)&(zs<1.04)
for x in np.arange(-0.2,0.2,0.01):
    m2=mm&(xs>=x)&(xs<x+0.01)
    if m2.sum(): print(f"  x[{x:.2f}] n={m2.sum():5d} y[{ys[m2].min():.3f},{ys[m2].max():.3f}]")
print("back wall y:", np.percentile(ys, [95,99,99.9]).round(3))
EOF
python3 cav3.py

# openrua op 40
cat > cav4.py <<'EOF'
import numpy as np
d = np.load("robot0_eye_in_hand_cloud.npz"); pw = d["pw"]; dep=d["depth"]
X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
ok = np.isfinite(dep)&(dep>0.05)
m = ok&(Y>0.25)&(Y<0.299)&(X>-0.14)&(X<0.05)&(Z>0.9)&(Z<1.15)
xs,ys,zs=X[m],Y[m],Z[m]
print("front-plane pts within opening x-range:", m.sum())
for z in np.arange(0.90,1.12,0.01):
    mm=(zs>=z)&(zs<z+0.01)
    if mm.sum(): print(f"  z[{z:.2f}] n={mm.sum():5d} y[{ys[mm].min():.3f},{ys[mm].max():.3f}] x[{xs[mm].min():.3f},{xs[mm].max():.3f}]")
# x-extent of the opening at mid height: front plane points at z in [0.98,1.04]
m2 = ok&(Y>0.25)&(Y<0.299)&(Z>0.98)&(Z<1.04)&(X>-0.3)&(X<0.3)
print("front plane at mid-height, x hist:")
xs2=X[m2]
for x in np.arange(-0.3,0.3,0.02):
    mm=(xs2>=x)&(xs2<x+0.02)
    if mm.sum(): print(f"  x[{x:.2f}] n={mm.sum()}")
# also depth along the y direction between 0.267 and 0.30 : any points at all with x in cavity range?
m3 = ok&(Y>0.27)&(Y<0.30)&(X>-0.14)&(X<0.05)&(Z>0.9)&(Z<1.15)
print("pts between front plane and interior:", m3.sum(), "z", Z[m3].min() if m3.sum() else None, Z[m3].max() if m3.sum() else None)
EOF
python3 cav4.py

# openrua op 41
python3 - <<'EOF'
import re
s=open("scene.py").read()
s=s.replace('MW = dict(x0=-0.175, x1=0.166, y0=0.27, y1=0.46, z0=TABLE_Z, z1=1.107)  # microwave exterior',
            'MW = dict(x0=-0.175, x1=0.166, y0=0.267, y1=0.46, z0=TABLE_Z, z1=1.107)  # microwave exterior')
s=s.replace('CAV = dict(x0=-0.15, x1=0.14, y0=0.27, y1=0.44, z0=0.945, z1=1.07)  # cavity (z1 = guess until measured)',
            'CAV = dict(x0=-0.149, x1=0.055, y0=0.267, y1=0.435, z0=0.942, z1=1.087)  # cavity, measured with eye-in-hand')
open("scene.py","w").write(s)
EOF
grep -n "^MW\|^CAV" scene.py

# openrua op 42
cat > t5.py <<'EOF'
from lib import *
from scene import *
r = Robot("t5"); sc = Scene(r.node)
sc.publish(world_objects())
q0 = r.arm_q()
mx,my = -0.0985,-0.252
def R_pitch(phi):
    c,s=np.cos(phi),np.sin(phi)
    z=np.array([0,c,-s]); y=np.array([1.,0,0]); x=np.cross(y,z)
    return R_from_axes(x,y,z)
for deg in [0,10,15,20,25]:
    phi=np.radians(deg); R=R_pitch(phi); z=R[:,2]
    pad = np.array([mx,my,0.94])
    grasp = pad - 0.094*z
    pre = grasp - 0.07*z
    qg = r.ik(grasp,R,seed=q0,timeout=2.0)
    qp = None if qg is None else r.ik(pre,R,seed=qg,timeout=2.0)
    def fmt(q, allow_mug=False):
        if q is None: return "IK-none"
        v,c = sc.check(q, verbose=False)
        c = sorted({tuple(x) for x in c})
        if allow_mug: c=[x for x in c if not ('white_mug' in x and any(k in x[0]+x[1] for k in ['finger','hand']))]
        return f"valid={v and not c} contacts={c} q={np.round(q,3)}"
    print(f"pitch {deg}: grasp origin={np.round(grasp,3)} -> {fmt(qg, True)}")
    print(f"          pre   origin={np.round(pre,3)} -> {fmt(qp)}", flush=True)
EOF
timeout 600 python3 t5.py

# openrua op 43
cat > t6.py <<'EOF'
from lib import *
from scene import *
r = Robot("t6"); sc = Scene(r.node)
q0 = r.arm_q()
mx,my = -0.0985,-0.252
def R_py(phi, psi):
    # approach dir: horizontal (-sin psi, cos psi) tilted down by phi
    c,s=np.cos(phi),np.sin(phi)
    h = np.array([-np.sin(psi), np.cos(psi), 0.])
    z = c*h + np.array([0,0,-s])
    y = np.array([np.cos(psi), np.sin(psi), 0.])
    x = np.cross(y,z)
    return R_from_axes(x,y,z)
def fmt(q, allow_mug=False):
    if q is None: return "IK-none"
    v,c = sc.check(q, verbose=False)
    c = sorted({tuple(x) for x in c})
    if allow_mug: c=[x for x in c if not ('white_mug' in x and any(k in x[0]+x[1] for k in ['finger','hand']))]
    return f"ok={not c} contacts={c} q={np.round(q,3)}"
for deg,yaw in [(10,0),(12,0),(15,0),(10,15),(10,25),(15,20)]:
    R=R_py(np.radians(deg), np.radians(yaw)); z=R[:,2]
    pad=np.array([mx,my,0.94]); grasp=pad-0.094*z
    qg=r.ik(grasp,R,seed=q0)
    print(f"pitch {deg} yaw {yaw}: grasp {np.round(grasp,3)} -> {fmt(qg,True)}", flush=True)
    if qg is None: continue
    for back in [0.05,0.06]:
        pre=grasp-back*z; qp=r.ik(pre,R,seed=qg)
        print(f"     pre back={back} {np.round(pre,3)} -> {fmt(qp)}", flush=True)
EOF
timeout 600 python3 t6.py

# openrua op 44
ls *.npz *.py | head -50 && cat mug2.py 2>/dev/null | head -60

# openrua op 45
ls -la --time-style=+%T *_cloud.npz; python3 mug2.py 2>&1 | head -60

# openrua op 46
cat mug3.py; echo ----; python3 mug3.py 2>&1 | tail -40

# openrua op 47
cat > mug4.py <<'EOF'
import numpy as np
cx,cy=-0.0985,-0.2535
for cam in ["agentview","frontview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
    m = (np.abs(X-cx)<0.08)&(np.abs(Y-cy)<0.08)&(Z>0.905)&(Z<1.03)&(Y<cy+0.02)
    xs,ys,zs=X[m],Y[m],Z[m]
    r=np.hypot(xs-cx,ys-cy)
    print(cam)
    for z in np.arange(0.90,1.03,0.01):
        mm=(zs>=z)&(zs<z+0.01)
        if mm.sum()>3:
            rr=np.sort(r[mm])
            print(f"  z[{z:.2f}] n={mm.sum():4d} r_p50={rr[len(rr)//2]:.3f} r_p90={rr[int(len(rr)*0.9)]:.3f} r_max={rr[-1]:.3f}")
EOF
python3 mug4.py

# openrua op 48
grep -n -A3 "panda_hand\"\|collision" robot.urdf | grep -i "mesh\|box\|cylinder" | sort | uniq -c | head; find / -name "hand*.stl" -o -name "hand*.dae" 2>/dev/null | head

# openrua op 49
python3 - <<'EOF'
import numpy as np, struct
def stl(fn):
    b=open(fn,'rb').read()
    n=struct.unpack_from('<I',b,80)[0]
    arr=np.frombuffer(b[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    return arr['v'].reshape(-1,3)
for name in ["hand","finger","link7"]:
    v=stl(f"/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{name}.stl")
    print(name, "min",np.round(v.min(0),4),"max",np.round(v.max(0),4))
    if name=="hand":
        for z0 in np.arange(0,0.07,0.01):
            m=(v[:,2]>=z0)&(v[:,2]<z0+0.01)
            if m.sum(): print(f"  z[{z0:.2f}] x[{v[m,0].min():.3f},{v[m,0].max():.3f}] y[{v[m,1].min():.3f},{v[m,1].max():.3f}]")
EOF

# openrua op 50
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/ 2>&1; find / \( -name "hand.stl" -o -name "finger.stl" -o -name "hand.dae" \) 2>/dev/null | head; ros2 param get /move_group robot_description 2>/dev/null | grep -o 'panda_hand.\{0,600\}' | head -3

# openrua op 51
cat > handle.py <<'EOF'
import numpy as np
for cam in ["agentview","frontview","birdview"]:
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"]
    X,Y,Z = pw[...,0], pw[...,1], pw[...,2]
    m = (np.abs(X+0.0985)<0.08)&(Y>-0.205)&(Y<-0.12)&(Z>0.905)&(Z<1.02)
    xs,ys,zs=X[m],Y[m],Z[m]
    print(cam, "handle pts", m.sum())
    for z in np.arange(0.90,1.02,0.01):
        mm=(zs>=z)&(zs<z+0.01)
        if mm.sum()>2: print(f"  z[{z:.2f}] n={mm.sum():4d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] xc={np.median(xs[mm]):.3f} y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
    # y-profile at mid height
    mm=(zs>0.94)&(zs<0.97)
    for y in np.arange(-0.205,-0.17,0.005):
        k=mm&(ys>=y)&(ys<y+0.005)
        if k.sum()>0: print(f"    y[{y:.3f}] n={k.sum()} x[{xs[k].min():.3f},{xs[k].max():.3f}] z[{zs[k].min():.3f},{zs[k].max():.3f}]")
EOF
python3 handle.py

# openrua op 52
cat > ctl.py <<'EOF'
"""Higher-level motion helpers on top of lib/scene: collision-checked IK moves with retry."""
import numpy as np
from lib import *
from scene import *

class Ctl:
    def __init__(self, name="ctl"):
        self.r = Robot(name); self.sc = Scene(self.r.node)

    def ik_valid(self, p, R, seed=None, tries=6, ignore=()):
        """IK solution that passes the validity check (contacts involving `ignore` bodies are tolerated)."""
        seed = list(seed if seed is not None else self.r.arm_q())
        rng = np.random.default_rng(0)
        for i in range(tries):
            s = seed if i == 0 else list(np.array(seed) + rng.normal(0, 0.3, 7))
            q = self.r.ik(p, R, seed=s)
            if q is None: continue
            v, c = self.sc.check(q, verbose=False)
            c = [x for x in set(map(tuple, c)) if not any(g in x[0] or g in x[1] for g in ignore)]
            if not c: return q
            last = c
        print(f"  ik_valid: no valid IK for {np.round(p,3)}; last contacts {locals().get('last')}", flush=True)
        return None

    def exec(self, qs, times, tol=0.02, retries=3):
        code, err = self.r.move(qs, times)
        for _ in range(retries):
            if err <= tol: break
            code, err = self.r.move([qs[-1]], [max(2.0, times[-1] / 3)])
        if err > tol: raise RuntimeError(f"move did not converge (err {err:.3f})")
        return err

    def goto_q(self, q, t=4.0, ignore=(), steps=25):
        q0 = self.r.arm_q()
        ok, bad = path_valid(self.sc, q0, q, steps=steps)
        bad = [(i, [x for x in c if not any(g in x[0] or g in x[1] for g in ignore)]) for i, c in bad]
        bad = [b for b in bad if b[1]]
        if bad:
            print("  goto_q: path invalid:", bad[:4], flush=True); return False
        self.exec([q], [t]); return True

    def goto(self, p, R, t=4.0, ignore=(), seed=None):
        q = self.ik_valid(p, R, seed=seed, ignore=ignore)
        if q is None: return False
        return self.goto_q(q, t, ignore=ignore)

    def line(self, p1, R, n=6, t=4.0, ignore=(), check=True):
        """Straight Cartesian move from current hand position to p1 at fixed R."""
        p0, _ = self.r.fk()
        qs, q = [], self.r.arm_q()
        for i in range(1, n + 1):
            p = p0 + (np.asarray(p1) - p0) * i / n
            q = self.r.ik(p, R, seed=q)
            if q is None: print(f"  line: IK failed at {np.round(p,3)}"); return False
            if check:
                v, c = self.sc.check(q, verbose=False)
                c = [x for x in set(map(tuple, c)) if not any(g in x[0] or g in x[1] for g in ignore)]
                if c: print(f"  line: collision at {np.round(p,3)}: {c}"); return False
            qs.append(q)
        self.exec(qs, list(np.linspace(t / n, t, n)))
        return True
EOF
python3 -c "import ctl"; echo ok

# openrua op 53
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/scene.py", "old_string": "    if with_white_mug:\\n        w = WHITE_MUG\\n        objs.append(box(\\"white_mug\\", [w[\\"cx\\"] - w[\\"r\\"], w[\\"cy\\"] - w[\\"r\\"], TABLE_Z],\\n                        [w[\\"cx\\"] + w[\\"r\\"], w[\\"cy\\"] + w[\\"r\\"] + 0.03, TABLE_Z + w[\\"h\\"]]))\\n    return objs", "new_string": "    if with_white_mug:\\n        w = WHITE_MUG\\n        objs.append(box(\\"white_mug\\", [w[\\"cx\\"] - w[\\"r\\"], w[\\"cy\\"] - w[\\"r\\"], TABLE_Z],\\n                        [w[\\"cx\\"] + w[\\"r\\"], w[\\"cy\\"] + w[\\"r\\"], TABLE_Z + w[\\"h\\"]]))\\n        objs.append(box(\\"white_handle\\", [-0.106, -0.207, 0.915], [-0.088, -0.170, 0.995]))\\n    return objs\\n\\n\\n# yellow mug: body ~10 cm wide, handle on its -y side\\nYELLOW_BODY = dict(x0=-0.026, x1=0.074, y0=-0.052, y1=0.050, z0=TABLE_Z, z1=1.0)\\nYELLOW_HANDLE = dict(x0=0.018, x1=0.065, y0=-0.083, y1=-0.052, z0=0.92, z1=1.0)\\n\\n\\ndef yellow_objects(dx=0.0, dy=0.0):\\n    \\"\\"\\"Yellow mug body + handle boxes, optionally shifted (after pushing it).\\"\\"\\"\\n    b, h = YELLOW_BODY, YELLOW_HANDLE\\n    return [box(\\"yellow_mug\\", [b[\\"x0\\"] + dx, b[\\"y0\\"] + dy, b[\\"z0\\"]], [b[\\"x1\\"] + dx, b[\\"y1\\"] + dy, b[\\"z1\\"]]),\\n            box(\\"yellow_handle\\", [h[\\"x0\\"] + dx, h[\\"y0\\"] + dy, h[\\"z0\\"]], [h[\\"x1\\"] + dx, h[\\"y1\\"] + dy, h[\\"z1\\"]])]", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 54
cat > push_yellow.py <<'EOF'
import sys
from ctl import *
c = Ctl("push_yellow"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
sc.publish(world_objects() + yellow_objects())
r.report("start")
al = np.radians(30)
z = np.array([np.sin(al), 0, -np.cos(al)]); y = np.array([0, 1., 0]); x = np.cross(y, z)
R = R_from_axes(x, y, z)
def origin(tip): return np.asarray(tip) - 0.1034 * z
tip_hi  = np.array([-0.04, -0.002, 1.06])
tip_lo  = np.array([-0.04, -0.002, 0.955])
tip_end = np.array([ 0.11, -0.002, 0.955])
q0 = r.arm_q()
for name, tip in [("hi", tip_hi), ("lo", tip_lo), ("end", tip_end)]:
    q = c.ik_valid(origin(tip), R, seed=q0, ignore=("yellow",) if name == "end" else ())
    print(name, np.round(origin(tip), 3), None if q is None else np.round(q, 3), flush=True)
    if q is not None: q0 = q
if dry: sys.exit()
if r.finger_gap() > 0.01:
    r.gripper(0.0)
assert c.goto(origin(tip_hi), R, t=5.0), "goto hi failed"
r.report("hi")
assert c.line(origin(tip_lo), R, n=4, t=3.0), "descend failed"
r.report("lo")
# push: tolerate contacts with the yellow boxes (we are pushing it)
assert c.line(origin(tip_end), R, n=6, t=6.0, ignore=("yellow",)), "push failed"
r.report("end")
assert c.line(origin(tip_end + [0, 0, 0.12]), R, n=4, t=3.0, ignore=("yellow",)), "retract failed"
r.report("up")
r.snap("agentview"); r.snap("birdview")
EOF
timeout 600 python3 push_yellow.py

# openrua op 55
python3 - <<'EOF'
import re
s=open('ctl.py').read()
s=s.replace('''        rng = np.random.default_rng(0)
        for i in range(tries):
            s = seed if i == 0 else list(np.array(seed) + rng.normal(0, 0.3, 7))
            q = self.r.ik(p, R, seed=s)
            if q is None: continue
            v, c = self.sc.check(q, verbose=False)
            c = [x for x in set(map(tuple, c)) if not any(g in x[0] or g in x[1] for g in ignore)]
            if not c: return q
            last = c
        print(f"  ik_valid: no valid IK for {np.round(p,3)}; last contacts {locals().get('last')}", flush=True)
        return None''','''        rng = np.random.default_rng(0)
        sols, last = [], None
        for i in range(tries):
            s = seed if i < 2 else list(np.array(seed) + rng.normal(0, 0.15, 7))
            q = self.r.ik(p, R, seed=s)
            if q is None: continue
            v, c = self.sc.check(q, verbose=False)
            c = [x for x in set(map(tuple, c)) if not any(g in x[0] or g in x[1] for g in ignore)]
            if not c:
                sols.append(q)
                if np.abs(np.array(q) - np.array(seed)).max() < 0.5: break
            else: last = c
        if sols:
            return min(sols, key=lambda q: np.abs(np.array(q) - np.array(seed)).max())
        print(f"  ik_valid: no valid IK for {np.round(p,3)}; last contacts {last}", flush=True)
        return None''')
open('ctl.py','w').write(s)
EOF
timeout 600 python3 push_yellow.py

# openrua op 56
cat > t7.py <<'EOF'
from ctl import *
c = Ctl("t7"); r=c.r
al = np.radians(30)
z = np.array([np.sin(al), 0, -np.cos(al)]); y = np.array([0, 1., 0]); x = np.cross(y, z)
R = R_from_axes(x, y, z)
p0,R0 = r.fk([0,-0.161,0,-2.44,0,2.23,0.785]); print("home hand", np.round(p0,3), np.round(R0,2))
seed=[0.0,0.6,0.0,-1.9,0.0,2.5,0.785]
p,Rs=r.fk(seed); print("seed hand", np.round(p,3), np.round(Rs[:,2],2), np.round(Rs[:,1],2))
for tip in [[-0.04,-0.002,1.06],[-0.04,-0.002,0.955],[0.11,-0.002,0.955]]:
    o=np.array(tip)-0.1034*z
    q=c.ik_valid(o,R,seed=seed,tries=10,ignore=("yellow",))
    print(np.round(o,3), None if q is None else np.round(q,3))
    if q is not None: seed=q
EOF
timeout 300 python3 t7.py

# openrua op 57
sed -i 's/y = np.array(\[0, 1., 0\]); x = np.cross(y, z)/y = np.array([0, -1., 0]); x = np.cross(y, z)/' t7.py push_yellow.py && sed -i 's/seed=\[0.0,0.6,0.0,-1.9,0.0,2.5,0.785\]/seed=[0.0,0.6,0.0,-1.9,0.0,2.5,0.785]/' t7.py && timeout 300 python3 t7.py

# openrua op 58
cat > t8.py <<'EOF'
from ctl import *
c = Ctl("t8"); r=c.r
for deg in [0,10,15,20]:
    al=np.radians(deg)
    z = np.array([np.sin(al), 0, -np.cos(al)]); y = np.array([0, -1., 0]); x = np.cross(y, z)
    R = R_from_axes(x, y, z)
    seed=[0.0,0.6,0.0,-1.9,0.0,2.5,0.785]
    for tip in [[-0.04,-0.002,1.06],[-0.04,-0.002,0.955],[0.03,-0.002,0.955],[0.11,-0.002,0.955]]:
        o=np.array(tip)-0.1034*z
        q=c.ik_valid(o,R,seed=seed,tries=8,ignore=("yellow",))
        print(deg, np.round(o,3), None if q is None else np.round(q,3), flush=True)
        if q is not None: seed=q
EOF
timeout 600 python3 t8.py 2>&1 | grep -v "^$"

# openrua op 59
python3 - <<'EOF'
s=open('push_yellow.py').read()
s=s.replace("al = np.radians(30)","al = np.radians(20)")
s=s.replace("tip_hi  = np.array([-0.04, -0.002, 1.06])","tip_hi  = np.array([-0.05, -0.002, 1.05])")
s=s.replace("tip_lo  = np.array([-0.04, -0.002, 0.955])","tip_lo  = np.array([-0.05, -0.002, 0.94])")
s=s.replace("tip_end = np.array([ 0.11, -0.002, 0.955])","tip_end = np.array([ 0.10, -0.002, 0.94])")
s=s.replace("q0 = r.arm_q()","q0 = [0.0,0.6,0.0,-1.9,0.0,2.5,0.785]")
s=s.replace("assert c.goto(origin(tip_hi), R, t=5.0), \"goto hi failed\"",
 "qhi = c.ik_valid(origin(tip_hi), R, seed=[0.0,0.6,0.0,-1.9,0.0,2.5,0.785])\nassert c.goto_q(qhi, t=6.0), \"goto hi failed\"")
open('push_yellow.py','w').write(s)
EOF
timeout 900 python3 push_yellow.py --go

# openrua op 60
python3 - <<'EOF'
s=open('push_yellow.py').read()
s=s.replace("qhi = c.ik_valid(","p_now, R_now = r.fk()\nif p_now[2] < 1.2:\n    assert c.line(p_now + [0, 0, 1.25 - p_now[2]], R_now, n=3, t=3.0), 'lift failed'\n    r.report('lifted')\nqhi = c.ik_valid(")
open('push_yellow.py','w').write(s)
EOF
timeout 900 python3 push_yellow.py --go

# openrua op 61
python3 - <<'EOF'
s=open('ctl.py').read()
s=s.replace('''            q = self.r.ik(p, R, seed=q)
            if q is None: print(f"  line: IK failed at {np.round(p,3)}"); return False
            if check:
                v, c = self.sc.check(q, verbose=False)
                c = [x for x in set(map(tuple, c)) if not any(g in x[0] or g in x[1] for g in ignore)]
                if c: print(f"  line: collision at {np.round(p,3)}: {c}"); return False
            qs.append(q)''','''            q = self.ik_valid(p, R, seed=q, ignore=ignore) if check else self.r.ik(p, R, seed=q)
            if q is None: print(f"  line: no valid IK at {np.round(p,3)}"); return False
            if np.abs(np.array(q) - np.array(qs[-1] if qs else self.r.arm_q())).max() > 0.6:
                print(f"  line: joint jump at {np.round(p,3)}"); return False
            qs.append(q)''')
open('ctl.py','w').write(s)
s=open('push_yellow.py').read()
s=s.replace("if dry: sys.exit()","if dry: sys.exit()\nif r.fk()[0][2] > 1.2 or abs(r.fk()[0][0]+0.085) > 0.01:  # not yet at 'hi'")
s=s.replace("    p_now, R_now","    pass\np_now, R_now",1)
open('push_yellow.py','w').write(s)
EOF
grep -n "hi\b\|lift\|pass" push_yellow.py | head -20

# openrua op 62
sed -n 18,30p push_yellow.py

# openrua op 63
python3 - <<'EOF'
s=open('push_yellow.py').read()
s=s.replace("""if r.fk()[0][2] > 1.2 or abs(r.fk()[0][0]+0.085) > 0.01:  # not yet at 'hi'
if r.finger_gap() > 0.01:
    r.gripper(0.0)
p_now, R_now = r.fk()
if p_now[2] < 1.2:
    assert c.line(p_now + [0, 0, 1.25 - p_now[2]], R_now, n=3, t=3.0), 'lift failed'
    r.report('lifted')
qhi = c.ik_valid(origin(tip_hi), R, seed=[0.0,0.6,0.0,-1.9,0.0,2.5,0.785])
assert c.goto_q(qhi, t=6.0), "goto hi failed"
r.report("hi")
""","""if r.finger_gap() > 0.01:
    r.gripper(0.0)
p_now, R_now = r.fk()
at_hi = np.linalg.norm(p_now - origin(tip_hi)) < 0.01
if not at_hi:
    if p_now[2] < 1.2:
        assert c.line(p_now + [0, 0, 1.25 - p_now[2]], R_now, n=3, t=3.0), 'lift failed'
        r.report('lifted')
    qhi = c.ik_valid(origin(tip_hi), R, seed=[0.0,0.6,0.0,-1.9,0.0,2.5,0.785])
    assert c.goto_q(qhi, t=6.0), "goto hi failed"
    r.report("hi")
""")
open('push_yellow.py','w').write(s)
EOF
timeout 900 python3 push_yellow.py --go

# openrua op 64
timeout 120 python3 cloud.py birdview >/dev/null 2>&1; timeout 120 python3 cloud.py agentview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
for cam in ["birdview","agentview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
    m=(Z>0.93)&(Z<1.02)&(Y>-0.15)&(Y<0.12)&(X>-0.1)&(X<0.4)
    print(cam,"yellow-ish region pts",m.sum())
    if m.sum():
        xs,ys,zs=X[m],Y[m],Z[m]
        print(f"  x[{xs.min():.3f},{xs.max():.3f}] y[{ys.min():.3f},{ys.max():.3f}] z[{zs.min():.3f},{zs.max():.3f}]")
        for z in np.arange(0.93,1.02,0.01):
            mm=(zs>=z)&(zs<z+0.01)
            if mm.sum()>3: print(f"  z[{z:.2f}] n={mm.sum()} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
    m=(Z>0.905)&(Z<1.02)&(np.abs(X+0.1)<0.07)&(np.abs(Y+0.24)<0.08)
    if m.sum(): print("  white mug x[%.3f,%.3f] y[%.3f,%.3f] zmax=%.3f"%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max()))
EOF

# openrua op 65
cat > grasp.py <<'EOF'
import sys
from ctl import *
c = Ctl("grasp"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
sc.publish(world_objects() + yellow_objects(dx=0.126, dy=-0.013), remove=["yellow_handle"])
r.report("start")
HX, HY, HZ = -0.097, -0.192, 0.955          # fingertip target on the handle's outer bar
z = np.array([0, -1., 0])
cands = {}
for sgn in (+1, -1):
    y = np.array([sgn * 1., 0, 0]); x = np.cross(y, z); R = R_from_axes(x, y, z)
    o_hi = np.array([HX, HY, 1.10]) - 0.1034 * z
    o_lo = np.array([HX, HY, HZ]) - 0.1034 * z
    seed = [0.0, 0.6, 0.0, -1.9, 0.0, 2.5, 0.785]
    q_hi = c.ik_valid(o_hi, R, seed=seed, tries=8)
    q_lo = c.ik_valid(o_lo, R, seed=q_hi or seed, tries=8, ignore=("white_handle",))
    print("sgn", sgn, "hi", None if q_hi is None else np.round(q_hi, 3), "lo", None if q_lo is None else np.round(q_lo, 3), flush=True)
    if q_hi is not None and q_lo is not None: cands[sgn] = (R, o_hi, o_lo, q_hi, q_lo)
if dry: sys.exit()
sgn = max(cands, key=lambda s: -abs(cands[s][3][6]))   # prefer j7 near 0
R, o_hi, o_lo, q_hi, q_lo = cands[sgn]; print("using sgn", sgn, flush=True)
if r.finger_gap() < 0.07: r.gripper(0.08)
p_now, R_now = r.fk()
if p_now[2] < 1.15:
    assert c.line(p_now + [0, 0, 1.20 - p_now[2]], R_now, n=3, t=3.0), "lift failed"
assert c.goto_q(q_hi, t=6.0), "goto pre-grasp failed"
r.report("pre-grasp")
assert c.line(o_lo, R, n=5, t=4.0, ignore=("white_handle",)), "descend failed"
r.report("grasp pose")
gap = r.gripper(0.0)
print("GAP after close:", gap, flush=True)
r.snap("agentview"); r.snap("robot0_eye_in_hand")
EOF
timeout 900 python3 grasp.py

# openrua op 66
cat > t9.py <<'EOF'
from ctl import *
c = Ctl("t9"); r, sc = c.r, c.sc
sc.publish(world_objects() + yellow_objects(dx=0.126, dy=-0.013))
bar = np.array([-0.097, -0.1755, 0.955])
for deg in [30, 40, 50, 60]:
    ph = np.radians(deg)
    z = np.array([0, -np.cos(ph), -np.sin(ph)])
    for sgn in (+1, -1):
        y = np.array([sgn * 1., 0, 0]); x = np.cross(y, z); R = R_from_axes(x, y, z)
        tip = bar + 0.012 * z
        o_lo = tip - 0.1034 * z
        o_hi = o_lo + np.array([0, 0.04, 0.10])
        seed = [0.0, 0.6, 0.0, -1.9, 0.0, 2.5, 0.785]
        q_hi = c.ik_valid(o_hi, R, seed=seed, tries=8)
        q_lo = c.ik_valid(o_lo, R, seed=q_hi or seed, tries=8, ignore=("white_handle",))
        print(f"pitch {deg} sgn {sgn}: o_lo={np.round(o_lo,3)} hi={None if q_hi is None else np.round(q_hi,2)} lo={None if q_lo is None else np.round(q_lo,2)}", flush=True)
EOF
timeout 900 python3 t9.py 2>&1 | grep -v "^  ik_valid"

# openrua op 67
cat > grasp.py <<'EOF'
import sys
from ctl import *
c = Ctl("grasp"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
sc.publish(world_objects() + yellow_objects(dx=0.126, dy=-0.013))
r.report("start")
BAR = np.array([-0.097, -0.1755, 0.955])
PH = np.radians(30)
def hand_R(psi):
    d = np.array([np.cos(psi), np.sin(psi), 0])
    z = np.cos(PH) * d + np.array([0, 0, -np.sin(PH)])
    y = np.array([-np.sin(psi), np.cos(psi), 0]); x = np.cross(y, z)
    return R_from_axes(x, y, z)
R = hand_R(-np.pi / 2); z = R[:, 2]
tip = BAR + 0.012 * z
o_lo = tip - 0.1034 * z
o_hi = o_lo + np.array([0, 0.04, 0.10])
seed = [0.0, 0.6, 0.0, -1.9, 0.0, 2.5, 0.785]
q_hi = c.ik_valid(o_hi, R, seed=seed, tries=8)
q_lo = c.ik_valid(o_lo, R, seed=q_hi, tries=8, ignore=("white_handle",))
print("o_lo", np.round(o_lo, 3), "q_hi", np.round(q_hi, 3), "q_lo", np.round(q_lo, 3), flush=True)
if dry: sys.exit()
if r.finger_gap() < 0.07: r.gripper(0.08)
p_now, R_now = r.fk()
if p_now[2] < 1.15:
    assert c.line(p_now + [0, 0, 1.20 - p_now[2]], R_now, n=3, t=3.0), "lift failed"
assert c.goto_q(q_hi, t=6.0), "goto pre-grasp failed"
r.report("pre-grasp")
assert c.line(o_lo, hand_R(-np.pi / 2), n=5, t=4.0, ignore=("white_handle",)), "approach failed"
r.report("grasp pose")
gap = r.gripper(0.0)
print("GAP after close:", gap, flush=True)
r.snap("agentview"); r.snap("frontview")
EOF
timeout 900 python3 grasp.py --go

# openrua op 68
cat > lift.py <<'EOF'
from ctl import *
c = Ctl("lift"); r, sc = c.r, c.sc
sc.publish(world_objects(with_white_mug=False) + yellow_objects(dx=0.126, dy=-0.013), remove=["white_mug", "white_handle"])
p, R = r.fk(); r.report("before lift")
assert c.line(p + [0, 0, 0.15], R, n=4, t=4.0), "lift failed"
r.report("lifted")
print("gap", r.finger_gap())
r.snap("frontview"); r.snap("agentview")
EOF
timeout 600 python3 lift.py && timeout 120 python3 cloud.py frontview >/dev/null && python3 - <<'EOF'
import numpy as np
d=np.load("frontview_cloud.npz"); pw=d["pw"]; X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
m=(np.abs(X+0.1)<0.09)&(Y<-0.12)&(Y>-0.40)&(Z>0.95)&(Z<1.25)
xs,ys,zs=X[m],Y[m],Z[m]
print("pts",m.sum())
for z in np.arange(0.95,1.25,0.02):
    mm=(zs>=z)&(zs<z+0.02)
    if mm.sum()>5: print(f"z[{z:.2f}] n={mm.sum()} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
EOF

# openrua op 69
cat > carry.py <<'EOF'
import sys
from ctl import *
c = Ctl("carry"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
sc.publish(world_objects(with_white_mug=False) + yellow_objects(dx=0.126, dy=-0.013), remove=["white_mug", "white_handle"])
PH = np.radians(30)
def hand_R(psi):
    d = np.array([np.cos(psi), np.sin(psi), 0])
    z = np.cos(PH) * d + np.array([0, 0, -np.sin(PH)])
    y = np.array([-np.sin(psi), np.cos(psi), 0]); x = np.cross(y, z)
    return R_from_axes(x, y, z)
# mug box relative to the hand (computed from the grasp geometry, world -> hand at psi=-90deg)
R0 = hand_R(-np.pi / 2)
mug_c_rel = np.array([-0.0015, -0.1575, -0.0435])          # mug centre minus hand origin, world, at grasp
half = np.array([0.047, 0.062, 0.0575])                      # incl. handle margin on +y side (asymmetric ok)
corners = np.array([mug_c_rel + np.array([sx, sy, sz]) * half for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
ch = corners @ R0                                            # to hand frame: R0^T * v  == v @ R0
lo, hi = ch.min(0) - 0.005, ch.max(0) + 0.005
print("attached mug box (hand frame) lo", np.round(lo, 3), "hi", np.round(hi, 3), flush=True)
sc.attach("carried_mug", lo, hi)
p0, _ = r.fk(); r.report("start")
ORIG = np.array([-0.097, -0.096, 1.15])
# 1. yaw arc
psis = np.radians(np.arange(-90, -271, -30))
qs, q = [], r.arm_q()
for ps in psis:
    qq = c.ik_valid(ORIG, hand_R(ps), seed=q, tries=8)
    print(f"  psi {np.degrees(ps):.0f}: {None if qq is None else np.round(qq,2)}", flush=True)
    if qq is None: sys.exit("arc IK failed")
    if np.abs(np.array(qq) - np.array(q)).max() > 1.2: sys.exit("arc joint jump")
    qs.append(qq); q = qq
# 2. transit, 3. descend, 4. insert
Rf = hand_R(np.pi / 2)
P_tr = np.array([-0.047, 0.03, 1.15]); P_dn = np.array([-0.047, 0.03, 1.053]); P_in = np.array([-0.047, 0.1875, 1.053])
for name, P in [("transit", P_tr), ("descend", P_dn), ("insert", P_in)]:
    qq = c.ik_valid(P, Rf, seed=q, tries=8)
    print(f"  {name}: {None if qq is None else np.round(qq,2)}", flush=True)
    if qq is None: sys.exit(name + " IK failed")
    q = qq
if dry: sys.exit()
for i, qq in enumerate(qs[1:]):
    assert c.goto_q(qq, t=3.0), f"arc step {i} path invalid"
r.report("rotated")
assert c.line(P_tr, Rf, n=4, t=4.0), "transit failed"; r.report("transit")
assert c.line(P_dn, Rf, n=4, t=3.0), "descend failed"; r.report("descended")
r.snap("frontview")
assert c.line(P_in, Rf, n=6, t=5.0), "insert failed"; r.report("inserted")
r.snap("frontview"); r.snap("agentview")
print("gap", r.finger_gap())
EOF
timeout 900 python3 carry.py 2>&1 | grep -v "^  ik_valid"

# openrua op 70
timeout 900 python3 carry.py --go 2>&1 | grep -v "^  ik_valid\|^  psi\|^  transit\|^  descend:\|^  insert:"

# openrua op 71
cat > st2.py <<'EOF'
from lib import *
r=Robot("st2"); r.report("now"); print("gap", r.finger_gap())
r.snap("frontview"); r.snap("agentview"); r.snap("robot0_eye_in_hand")
EOF
timeout 120 python3 st2.py

# openrua op 72
timeout 120 python3 cloud.py robot0_eye_in_hand >/dev/null; timeout 120 python3 cloud.py sideview >/dev/null; python3 - <<'EOF'
import numpy as np
for cam in ["robot0_eye_in_hand","sideview"]:
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
    ok=np.isfinite(Z)
    print(cam, "finite", ok.sum())
    # mug region: near x=-0.05, y 0.15..0.45, z 0.9..1.1 ; the cavity floor near there
    m=ok&(np.abs(X+0.04)<0.09)&(Y>0.10)&(Y<0.46)&(Z>0.90)&(Z<1.12)
    xs,ys,zs=X[m],Y[m],Z[m]
    print(" pts",m.sum())
    for z in np.arange(0.90,1.12,0.01):
        mm=(zs>=z)&(zs<z+0.01)
        if mm.sum()>5: print(f"  z[{z:.2f}] n={mm.sum():5d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
EOF

# openrua op 73
cat > snapall.py <<'EOF'
from lib import *
import sys
r=Robot("snapall")
for cam in sys.argv[1:]: r.snap(cam)
EOF
timeout 120 python3 snapall.py galleryview paperview robot0_robotview sideview birdview

# openrua op 74
python3 - <<'EOF'
import numpy as np
d=np.load("robot0_eye_in_hand_cloud.npz"); pw=d["pw"]; col=d["color"]; X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
print(col.shape, col.dtype, col.max())
bright=(col.min(-1)>140)
m=bright&np.isfinite(Z)&(Y>0.12)&(Y<0.46)&(Z>0.85)&(Z<1.2)
xs,ys,zs=X[m],Y[m],Z[m]
print("white pts",m.sum())
for z in np.arange(0.90,1.15,0.01):
    mm=(zs>=z)&(zs<z+0.01)
    if mm.sum()>20: print(f"  z[{z:.2f}] n={mm.sum():5d} x[{xs[mm].min():.3f},{xs[mm].max():.3f}] y[{ys[mm].min():.3f},{ys[mm].max():.3f}]")
# dark points = microwave: front face location and cavity floor
dark=(col.max(-1)<60)&np.isfinite(Z)&(Z>0.85)&(Z<1.2)&(np.abs(X+0.04)<0.12)
xs,ys,zs=X[dark],Y[dark],Z[dark]
print("dark pts",dark.sum())
for z in np.arange(0.90,1.15,0.01):
    mm=(zs>=z)&(zs<z+0.01)
    if mm.sum()>20: print(f"  z[{z:.2f}] n={mm.sum():5d} y_min={ys[mm].min():.3f} y_p05={np.percentile(ys[mm],5):.3f}")
EOF

# openrua op 75
python3 - <<'EOF'
import numpy as np, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
d=np.load("robot0_eye_in_hand_cloud.npz"); pw=d["pw"]; col=d["color"]; X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
ok=np.isfinite(Z)&(np.abs(X+0.03)<0.06)&(Y>0.1)&(Y<0.5)&(Z>0.85)&(Z<1.2)
fig,ax=plt.subplots(1,2,figsize=(14,7))
ax[0].scatter(Y[ok],Z[ok],c=col[ok]/255.0,s=1); ax[0].set_xlabel("y"); ax[0].set_ylabel("z"); ax[0].set_title("side profile |x+0.03|<0.06"); ax[0].grid()
ok2=np.isfinite(Z)&(Y>0.1)&(Y<0.5)&(Z>0.85)&(Z<1.2)&(np.abs(X+0.04)<0.2)
ax[1].scatter(X[ok2],Z[ok2],c=col[ok2]/255.0,s=1); ax[1].set_xlabel("x"); ax[1].set_title("front profile"); ax[1].grid()
plt.savefig("prof.png",dpi=80)
EOF
echo done

# openrua op 76
python3 - <<'EOF'
import numpy as np, cv2
d=np.load("robot0_eye_in_hand_cloud.npz"); pw=d["pw"]; col=d["color"]; X,Y,Z=pw[...,0],pw[...,1],pw[...,2]
def render(u,v,c,ur,vr,fn,W=800,H=600):
    img=np.full((H,W,3),40,np.uint8)
    uu=((u-ur[0])/(ur[1]-ur[0])*(W-1)).astype(int); vv=((vr[1]-v)/(vr[1]-vr[0])*(H-1)).astype(int)
    k=(uu>=0)&(uu<W)&(vv>=0)&(vv<H)
    img[vv[k],uu[k]]=c[k][:,::-1]
    # grid every 5 cm
    for g in np.arange(ur[0],ur[1]+1e-9,0.05):
        x=int((g-ur[0])/(ur[1]-ur[0])*(W-1)); cv2.line(img,(x,0),(x,H-1),(90,90,90),1); cv2.putText(img,f"{g:.2f}",(x+2,12),cv2.FONT_HERSHEY_SIMPLEX,0.4,(200,200,0),1)
    for g in np.arange(vr[0],vr[1]+1e-9,0.05):
        y=int((vr[1]-g)/(vr[1]-vr[0])*(H-1)); cv2.line(img,(0,y),(W-1,y),(90,90,90),1); cv2.putText(img,f"{g:.2f}",(2,y-2),cv2.FONT_HERSHEY_SIMPLEX,0.4,(200,200,0),1)
    cv2.imwrite(fn,img)
ok=np.isfinite(Z)&(np.abs(X+0.03)<0.05)&(Y>0.1)&(Y<0.5)&(Z>0.85)&(Z<1.2)
render(Y[ok],Z[ok],col[ok],(0.1,0.5),(0.85,1.2),"prof_side.png")
ok2=np.isfinite(Z)&(Y>0.1)&(Y<0.5)&(Z>0.85)&(Z<1.2)&(np.abs(X+0.04)<0.2)
render(X[ok2],Z[ok2],col[ok2],(-0.25,0.15),(0.85,1.2),"prof_front.png")
EOF
echo done

# openrua op 77
cat ctl.py && cat carry.py && cat grasp.py | head -40

# openrua op 78
mkdir -p "$(dirname /workspace/back.py)"
cat > /workspace/back.py <<'OPENRUA_EOF'
import sys
from ctl import *
from carry import hand_R  # noqa  (carry.py exits early in dry mode... avoid)
OPENRUA_EOF

# openrua op 79
mkdir -p "$(dirname /workspace/common.py)"
cat > /workspace/common.py <<'OPENRUA_EOF'
"""Shared bits for the white-mug task: hand orientation, scene setup."""
import numpy as np
from ctl import *

PH = np.radians(30)
YDX, YDY = 0.126, -0.013  # yellow mug shift after pushing it


def hand_R(psi, ph=PH):
    """Hand approaching horizontally along direction psi (yaw), pitched ph down; fingers close perpendicular to d."""
    d = np.array([np.cos(psi), np.sin(psi), 0])
    z = np.cos(ph) * d + np.array([0, 0, -np.sin(ph)])
    y = np.array([-np.sin(psi), np.cos(psi), 0]); x = np.cross(y, z)
    return R_from_axes(x, y, z)


def scene_no_white(sc):
    sc.publish(world_objects(with_white_mug=False) + yellow_objects(dx=YDX, dy=YDY), remove=["white_mug", "white_handle"])


def attach_hanging_mug(sc, lo=(-0.127, -0.054, 0.071), hi=(0.045, 0.05, 0.246)):
    sc.attach("carried_mug", np.array(lo), np.array(hi))
OPENRUA_EOF

# openrua op 80
mkdir -p "$(dirname /workspace/back.py)"
cat > /workspace/back.py <<'OPENRUA_EOF'
import sys
from common import *
c = Ctl("back"); r, sc = c.r, c.sc
scene_no_white(sc)
r.report("start")
Rf = hand_R(np.pi / 2)
p, _ = r.fk()
# retreat straight back (-y) at current height, then settle at y=0.03
P1 = np.array([-0.047, 0.03, p[2] + 0.01])
assert c.line(P1, Rf, n=5, t=5.0, ignore=("carried_mug",)), "retreat failed"
r.report("retreated")
for cam in ("sideview", "frontview", "robot0_eye_in_hand", "agentview"):
    r.snap(cam)
print("gap", r.finger_gap())
OPENRUA_EOF

# openrua op 81
timeout 600 python3 back.py 2>&1 | tail -20

# openrua op 82
cat cloud.py | head -60; ls *.npz

# openrua op 83
timeout 120 python3 cloud.py frontview && timeout 120 python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
for cam in ("frontview","robot0_eye_in_hand"):
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
    ok=np.isfinite(pw).all(1)
    white=ok&(col.min(1)>150)&(np.ptp(col,axis=1)<30)
    # restrict to region in front of microwave, above table, away from robot links (x<0.05, y in [0.05,0.3])
    m=white&(pw[:,0]>-0.2)&(pw[:,0]<0.1)&(pw[:,1]>-0.05)&(pw[:,1]<0.3)&(pw[:,2]>0.9)&(pw[:,2]<1.12)
    P=pw[m]; print(cam, m.sum())
    if m.sum()>50:
        print(" min",np.round(P.min(0),3)," max",np.round(P.max(0),3))
        for z0 in np.arange(0.9,1.12,0.02):
            s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.02)]
            if len(s): print(f"  z {z0:.2f}: n={len(s):5d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 84
mkdir -p "$(dirname /workspace/place.py)"
cat > /workspace/place.py <<'OPENRUA_EOF'
"""Set the (horizontally hanging) mug down inside the cavity: insert high, lower until the rim touches,
then advance+descend so the mug rights itself onto the cavity floor."""
import sys
from common import *
c = Ctl("place"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
Rf = hand_R(np.pi / 2)
# hanging mug, measured in world relative to hand (hand at y=0.03,z=1.10): x[-0.096,-0.003] y[0.08,0.21] z[0.98,1.05]
p_now, _ = r.fk(); r.report("start")
lo_w = np.array([-0.096, 0.08, 0.98]) - p_now; hi_w = np.array([-0.003, 0.21, 1.05]) - p_now
corners = np.array([[lo_w[0] if sx < 0 else hi_w[0], lo_w[1] if sy < 0 else hi_w[1], lo_w[2] if sz < 0 else hi_w[2]]
                    for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
ch = corners @ Rf
sc.attach("carried_mug", None, None, detach=True)
sc.attach("carried_mug", ch.min(0) - 0.005, ch.max(0) + 0.005)
print("hanging mug box (hand)", np.round(ch.min(0), 3), np.round(ch.max(0), 3), flush=True)
X = -0.047
P1 = np.array([X, 0.19, 1.10])    # rim end inside the opening
P2 = np.array([X, 0.19, 1.062])   # rim about to touch the floor
P3 = np.array([X, 0.215, 1.043])  # bar 5.5 cm above the floor -> mug upright, centre y~0.37
q = r.arm_q()
for name, P, ign in [("P1", P1, ()), ("P2", P2, ()), ("P3", P3, ("carried_mug",))]:
    qq = c.ik_valid(P, Rf, seed=q, tries=8, ignore=ign)
    print(f"  {name}: {None if qq is None else np.round(qq, 2)}", flush=True)
    if qq is None: sys.exit(name + " invalid")
    q = qq
if dry: sys.exit()
assert c.line(P1, Rf, n=5, t=5.0), "insert failed"; r.report("P1")
r.snap("robot0_eye_in_hand", "/workspace/eih_P1.png")
assert c.line(P2, Rf, n=3, t=3.0), "lower failed"; r.report("P2")
r.snap("robot0_eye_in_hand", "/workspace/eih_P2.png")
try:
    c.line(P3, Rf, n=4, t=4.0, ignore=("carried_mug",))
except RuntimeError as e:
    print("P3:", e, flush=True)
r.report("P3")
r.snap("robot0_eye_in_hand", "/workspace/eih_P3.png")
print("gap", r.finger_gap())
OPENRUA_EOF

# openrua op 85
timeout 600 python3 place.py 2>&1 | tail -20

# openrua op 86
timeout 900 python3 place.py --go 2>&1 | tail -25

# openrua op 87
timeout 120 python3 st2.py 2>&1 | tail -5; timeout 60 python3 snapall.py robot0_eye_in_hand frontview 2>&1 | tail -2

# openrua op 88
timeout 120 python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
d=np.load("robot0_eye_in_hand_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
ok=np.isfinite(pw).all(1)
white=ok&(col.min(1)>140)&(np.ptp(col,axis=1)<30)&(pw[:,1]>0.2)&(pw[:,2]>0.93)&(pw[:,2]<1.09)
P=pw[white]; print(len(P), "min",np.round(P.min(0),3),"max",np.round(P.max(0),3))
for y0 in np.arange(0.2,0.44,0.02):
    s=P[(P[:,1]>=y0)&(P[:,1]<y0+0.02)]
    if len(s): print(f"  y {y0:.2f}: n={len(s):5d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
# handle (grey rod) points: darker grey near fingers
EOF

# openrua op 89
timeout 300 python3 - <<'EOF' 2>&1 | tail -30
from common import *
c = Ctl("probe"); r, sc = c.r, c.sc
scene_no_white(sc)
sc.attach("carried_mug", None, None, detach=True)
seed = r.arm_q()
for ph in (0, 15, 30):
    R = hand_R(np.pi/2, np.radians(ph))
    for x in (-0.047, -0.03, -0.06, -0.08, -0.02):
        for y, z in ((0.30, 1.0), (0.26, 1.02)):
            zz = z + 0.1034*np.sin(np.radians(ph))*0.5
            q = c.ik_valid([x, y, zz], R, seed=seed, tries=4)
            print(f"pitch {ph:2d} x {x:+.3f} y {y:.2f} z {zz:.3f}: {'OK' if q is not None else '--'}", flush=True)
EOF

# openrua op 90
timeout 300 python3 - <<'EOF' 2>&1 | grep -v ik_valid | head -30
from common import *
c = Ctl("probe"); r, sc = c.r, c.sc
seed = r.arm_q()
R = hand_R(np.pi/2, 0.0)
for x in (-0.047, -0.03, -0.06, -0.04, -0.055):
    for y, z in ((0.30, 1.0), (0.26, 1.02), (0.22, 1.02)):
        q = c.ik_valid([x, y, z], R, seed=seed, tries=4)
        print(f"pitch 0 x {x:+.3f} y {y:.2f} z {z:.3f}: {'OK' if q is not None else '--'}", flush=True)
EOF

# openrua op 91
grep -n -A12 'link name="panda_leftfinger"' robot.urdf | head -40; grep -n -B2 -A6 '<collision' robot.urdf | grep -A6 -i 'hand\|finger' | head -60

# openrua op 92
python3 - <<'EOF'
import numpy as np
d=np.load("robot0_eye_in_hand_cloud.npz"); pw=d["pw"]; dep=d["depth"]; col=d["color"].astype(int)
H,W,_=pw.shape
m=(dep<0.16)&np.isfinite(pw).all(2)
P=pw[m]; C=col[m]; print(len(P), "z range",P[:,2].min(),P[:,2].max())
# fingers hang below hand; hand at (-0.035,0.192,1.089) pitched 30 deg. Look at points in slabs of y
for y0 in np.arange(0.20,0.30,0.01):
    s=P[(P[:,1]>=y0)&(P[:,1]<y0+0.01)]
    if len(s):
        xs=np.sort(s[:,0]); print(f"y {y0:.2f}: n={len(s):5d} x[{xs[0]:.3f},{xs[-1]:.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]  x-hist:", np.histogram(s[:,0],bins=np.arange(-0.09,0.02,0.01))[0])
EOF

# openrua op 93
timeout 600 python3 back.py 2>&1 | grep -v "^  ik_valid" | tail -4 && for c in sideview frontview agentview birdview; do timeout 120 python3 cloud.py $c | tail -1; done

# openrua op 94
python3 - <<'EOF'
import numpy as np
# hand at (-0.0466,0.0304,1.0984), z_hand=(0,0.866,-0.5). fingertips at hand+0.1034*z = (-0.047,0.120,1.047); palm face at +0.0584*z=( -0.047,0.081,1.069)
h=np.array([-0.0466,0.0304,1.0984]); z=np.array([0,0.866,-0.5]); x=np.array([0,-0.5,-0.866])
for cam in ("sideview","frontview","agentview","birdview"):
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
    ok=np.isfinite(pw).all(1)
    rel=pw-h; s=rel@z; t=rel@x   # along hand z, along hand x
    # finger region: s in [0.065,0.105], |t|<0.012 (finger width 2cm), near x=-0.047
    m=ok&(s>0.062)&(s<0.106)&(np.abs(t)<0.015)&(np.abs(pw[:,0]+0.047)<0.06)
    P=pw[m]
    if len(P)==0: print(cam,"none"); continue
    xs=P[:,0]
    print(cam, len(P), "x-hist (mm bins):")
    hist,edges=np.histogram(xs,bins=np.arange(-0.10,0.0,0.004))
    print("  ", " ".join(f"{e*1000:+.0f}:{n}" for e,n in zip(edges[:-1],hist) if n>0))
EOF

# openrua op 95
python3 - <<'EOF'
import numpy as np
h=np.array([-0.0466,0.0304,1.0984]); z=np.array([0,0.866,-0.5]); x=np.array([0,-0.5,-0.866])
for cam in ("sideview","frontview","agentview"):
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"].reshape(-1,3)
    ok=np.isfinite(pw).all(1); rel=pw-h; s=rel@z; t=rel@x
    for s0,s1 in ((0.062,0.075),(0.075,0.09),(0.09,0.106)):
        m=ok&(s>s0)&(s<s1)&(np.abs(t)<0.015)&(np.abs(pw[:,0]+0.047)<0.05)
        xs=pw[m][:,0]
        hist,edges=np.histogram(xs,bins=np.arange(-0.10,0.0,0.004))
        print(f"{cam} s[{s0},{s1}] n={len(xs)}:", " ".join(f"{e*1000:+.0f}:{n}" for e,n in zip(edges[:-1],hist) if n>0))
EOF

# openrua op 96
mkdir -p "$(dirname /workspace/setdown.py)"
cat > /workspace/setdown.py <<'OPENRUA_EOF'
"""Lay the horizontally hanging mug down on the table in front of the microwave and release it."""
import sys
from common import *
c = Ctl("setdown"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
sc.attach("carried_mug", None, None, detach=True)  # mug pose unknown; move slowly, tolerate contacts
Rf = hand_R(np.pi / 2)
r.report("start")
X = -0.047
P1 = np.array([X, -0.08, 1.10])
P2 = np.array([X, -0.08, 1.035])   # mug underside ~0.915
P3 = np.array([X, -0.08, 1.015])   # expect contact with table before this
if dry:
    q = r.arm_q()
    for P in (P1, P2, P3):
        qq = c.ik_valid(P, Rf, seed=q, tries=6); print(np.round(P, 3), None if qq is None else np.round(qq, 2)); q = qq or q
    sys.exit()
assert c.line(P1, Rf, n=4, t=4.0), "move back failed"; r.report("P1")
assert c.line(P2, Rf, n=3, t=3.0), "lower failed"; r.report("P2")
try:
    c.line(P3, Rf, n=2, t=2.0)
except RuntimeError as e:
    print("P3:", e)
r.report("P3")
r.gripper(0.08)
p, _ = r.fk()
assert c.line(p + [0, 0, 0.10], Rf, n=3, t=3.0), "lift failed"
r.report("lifted")
for cam in ("agentview", "frontview", "sideview", "birdview"):
    r.snap(cam)
OPENRUA_EOF

# openrua op 97
timeout 300 python3 setdown.py 2>&1 | tail -20

# openrua op 98
timeout 600 python3 setdown.py --go 2>&1 | tail -20

# openrua op 99
timeout 120 python3 cloud.py agentview >/dev/null 2>&1; timeout 120 python3 cloud.py sideview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
for cam in ("agentview","sideview"):
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"]; col=d["color"].astype(int)
    m=(col.min(1)>140)&(np.ptp(col,1)<30)&(pw[:,2]>0.905)&(pw[:,2]<1.1)&(pw[:,1]<0.25)&(pw[:,1]>-0.2)&(np.abs(pw[:,0])<0.2)
    P=pw[m]; print(cam,len(P))
    if len(P)==0: continue
    print(" x",np.round(np.percentile(P[:,0],[1,50,99]),3)," y",np.round(np.percentile(P[:,1],[1,50,99]),3)," z",np.round(np.percentile(P[:,2],[1,50,99]),3))
    # slices along y
    for y0 in np.arange(-0.06,0.16,0.02):
        s=P[(P[:,1]>=y0)&(P[:,1]<y0+0.02)]
        if len(s)>5: print(f"  y[{y0:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
EOF

# openrua op 100
python3 - <<'EOF'
import numpy as np
for cam in ("agentview","sideview"):
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
    m=(col.min(1)>140)&(np.ptp(col,1)<30)&(pw[:,2]>0.905)&(pw[:,2]<1.1)&(pw[:,1]<0.25)&(pw[:,1]>-0.2)&(np.abs(pw[:,0])<0.2)
    P=pw[m]; print(cam,len(P))
    if len(P)==0: continue
    print(" x",np.round(np.percentile(P[:,0],[1,50,99]),3)," y",np.round(np.percentile(P[:,1],[1,50,99]),3)," z",np.round(np.percentile(P[:,2],[1,50,99]),3))
    for y0 in np.arange(-0.06,0.16,0.02):
        s=P[(P[:,1]>=y0)&(P[:,1]<y0+0.02)]
        if len(s)>5: print(f"  y[{y0:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
EOF

# openrua op 101
mkdir -p "$(dirname /workspace/roll.py)"
cat > /workspace/roll.py <<'OPENRUA_EOF'
"""Roll the lying mug by pushing its (upward) handle sideways toward -x with closed fingertips."""
import sys
from common import *
c = Ctl("roll"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
Rf = hand_R(np.pi / 2)
r.report("start")
z_h = Rf[:, 2]
def origin(tip): return np.asarray(tip) - 0.1034 * z_h
Y, Z = 0.03, 1.012
tips = [np.array([-0.04, Y, 1.10]), np.array([-0.04, Y, Z]), np.array([-0.13, Y, Z]), np.array([-0.13, Y, 1.10])]
q = r.arm_q()
for t in tips:
    qq = c.ik_valid(origin(t), Rf, seed=q, tries=6); print(np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
if r.finger_gap() > 0.01: r.gripper(0.0)
assert c.line(origin(tips[0]), Rf, n=4, t=4.0), "approach failed"; r.report("above")
assert c.line(origin(tips[1]), Rf, n=3, t=3.0), "descend failed"; r.report("down")
try:
    c.line(origin(tips[2]), Rf, n=5, t=5.0)
except RuntimeError as e:
    print("push:", e)
r.report("pushed")
assert c.line(origin(tips[3]), Rf, n=3, t=3.0), "lift failed"; r.report("up")
for cam in ("agentview", "frontview", "sideview"):
    r.snap(cam)
OPENRUA_EOF

# openrua op 102
timeout 300 python3 roll.py 2>&1 | tail -8

# openrua op 103
timeout 600 python3 roll.py --go 2>&1 | tail -12

# openrua op 104
timeout 120 python3 cloud.py agentview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
m=(col.min(1)>140)&(np.ptp(col,1)<30)&(pw[:,2]>0.905)&(pw[:,2]<1.1)&(pw[:,1]<0.25)&(pw[:,1]>-0.2)&(pw[:,0]<0.05)&(pw[:,0]>-0.25)
P=pw[m]; print(len(P))
print(" x",np.round(np.percentile(P[:,0],[1,50,99]),3)," y",np.round(np.percentile(P[:,1],[1,50,99]),3)," z",np.round(np.percentile(P[:,2],[1,50,99]),3))
for y0 in np.arange(-0.06,0.16,0.02):
    s=P[(P[:,1]>=y0)&(P[:,1]<y0+0.02)]
    if len(s)>5: print(f"  y[{y0:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
hi=P[P[:,2]>1.0]; print("high pts", len(hi), np.round(np.percentile(hi[:,0],[5,50,95]),3), np.round(np.percentile(hi[:,1],[5,50,95]),3))
EOF

# openrua op 105
python3 - <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
m=(col.min(1)>140)&(np.ptp(col,1)<30)&(pw[:,2]>0.905)&(pw[:,2]<1.1)&(pw[:,1]<0.25)&(pw[:,1]>-0.2)&(pw[:,0]<0.05)&(pw[:,0]>-0.25)
P=pw[m]
s=P[(P[:,1]>-0.01)&(P[:,1]<0.03)]
for z0 in np.arange(0.90,1.02,0.01):
    t=s[(s[:,2]>=z0)&(s[:,2]<z0+0.01)]
    if len(t): print(f"z[{z0:.2f}] n={len(t)} x[{t[:,0].min():.3f},{t[:,0].max():.3f}]")
EOF

# openrua op 106
sed -i 's/^Y, Z = 0.03, 1.012$/Y, Z = 0.02, 0.986/; s/^tips = .*/tips = [np.array([-0.135, Y, 1.10]), np.array([-0.135, Y, Z]), np.array([-0.205, Y, Z]), np.array([-0.205, Y, 1.10])]/' roll.py && timeout 300 python3 roll.py 2>&1 | tail -5

# openrua op 107
timeout 600 python3 roll.py --go 2>&1 | grep -E "^\s+\[|push" ; timeout 120 python3 cloud.py agentview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
m=(col.min(1)>140)&(np.ptp(col,1)<30)&(pw[:,2]>0.905)&(pw[:,2]<1.1)&(pw[:,1]<0.25)&(pw[:,1]>-0.2)&(pw[:,0]<0.05)&(pw[:,0]>-0.3)
P=pw[m]; print(len(P), " x",np.round(np.percentile(P[:,0],[1,50,99]),3)," y",np.round(np.percentile(P[:,1],[1,50,99]),3)," z",np.round(np.percentile(P[:,2],[1,50,99]),3))
s=P[(P[:,1]>-0.01)&(P[:,1]<0.03)]
for z0 in np.arange(0.90,1.03,0.01):
    t=s[(s[:,2]>=z0)&(s[:,2]<z0+0.01)]
    if len(t): print(f"z[{z0:.2f}] n={len(t)} x[{t[:,0].min():.3f},{t[:,0].max():.3f}]")
for y0 in np.arange(-0.08,0.16,0.02):
    s=P[(P[:,1]>=y0)&(P[:,1]<y0+0.02)]
    if len(s)>5: print(f"  y[{y0:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
EOF

# openrua op 108
timeout 120 python3 cloud.py birdview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
d=np.load("birdview_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
m=(col.min(1)>140)&(np.ptp(col,1)<30)&(pw[:,2]>0.905)&(pw[:,2]<1.1)&(pw[:,1]<0.25)&(pw[:,1]>-0.2)&(pw[:,0]<0.0)&(pw[:,0]>-0.3)
P=pw[m]; print(len(P), " x",np.round(np.percentile(P[:,0],[1,50,99]),3)," y",np.round(np.percentile(P[:,1],[1,50,99]),3)," z",np.round(np.percentile(P[:,2],[1,50,99]),3))
for x0 in np.arange(-0.26,-0.09,0.01):
    s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.01)]
    if len(s)>3: print(f"x[{x0:.2f}] n={len(s)} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
EOF

# openrua op 109
python3 -c "
from PIL import Image
im=Image.open('birdview.png'); print(im.size)
import numpy as np
d=np.load('birdview_cloud.npz'); pw=d['pw']; print(pw.shape)
# find pixel of world point (-0.15,0.02)
H,W=pw.shape[:2]
dd=np.linalg.norm(pw[:,:,:2]-np.array([-0.15,0.02]),axis=2); i=np.unravel_index(np.argmin(dd),dd.shape); print(i)
im.crop((i[1]-150,i[0]-150,i[1]+150,i[0]+150)).resize((600,600)).save('crop.png')
"

# openrua op 110
cat > park.py <<'EOF'
import sys
from common import *
c = Ctl("park"); r, sc = c.r, c.sc
scene_no_white(sc)
Rf = hand_R(np.pi / 2)
p, _ = r.fk()
P = np.array([0.05, -0.20, 1.35])
q = c.ik_valid(P, Rf, seed=r.arm_q(), tries=8)
assert q is not None and c.goto_q(q, t=5.0), "park failed"
r.report("parked")
for cam in sys.argv[1:] or ("birdview", "sideview", "agentview"):
    r.snap(cam)
EOF
timeout 300 python3 park.py 2>&1 | tail -2; timeout 120 python3 cloud.py birdview >/dev/null 2>&1; timeout 120 python3 cloud.py sideview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
for cam in ("birdview","sideview"):
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
    m=(col.min(1)>140)&(np.ptp(col,1)<30)&(pw[:,2]>0.905)&(pw[:,2]<1.05)&(pw[:,1]<0.12)&(pw[:,1]>-0.1)&(pw[:,0]<0.0)&(pw[:,0]>-0.26)
    P=pw[m]; print(cam,len(P))
    for x0 in np.arange(-0.26,-0.09,0.01):
        s=P[(P[:,0]>=x0)&(P[:,0]<x0+0.01)]
        if len(s)>3: print(f"x[{x0:.2f}] n={len(s)} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
EOF

# openrua op 111
mkdir -p "$(dirname /workspace/right.py)"
cat > /workspace/right.py <<'OPENRUA_EOF'
"""Stand the lying mug up: hook the rim's inner top edge with the closed fingertips (hand pointing +y,
pitched 45 deg) and sweep along an arc about the base's far bottom edge until the mug tips onto its base."""
import sys
from common import *
c = Ctl("right"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
R45 = hand_R(np.pi / 2, ph=np.pi / 4)
z_h = R45[:, 2]
def origin(tip): return np.asarray(tip) - 0.1034 * z_h
X = -0.15            # mug axis x
Y_RIM, Z_AX = -0.048, 0.947
P = np.array([0.06, 0.90])           # pivot (y,z): base's far bottom edge
c0 = np.array([Y_RIM, Z_AX + 0.041]) - P   # rim inner top edge rel. pivot
def tip(th, dy=0.0):
    th = np.radians(th)
    Rm = np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]])   # rim end up & +y
    cth = P + Rm @ c0
    n = np.array([np.sin(th), np.cos(th)])
    E = cth - 0.005 * n                       # finger top edge at the rim plane, 5 mm inside
    T = E - np.array([0, 0.0141]) + np.array([dy, 0])
    return np.array([X, T[0], T[1]])
T0 = tip(0)
pre = np.array([X, T0[1] - 0.018, 1.10])
down = np.array([X, T0[1] - 0.018, T0[2]])
seq = [("pre", pre), ("down", down), ("in", T0)] + [(f"th{th}", tip(th, dy=0.01 * th / 65)) for th in (15, 30, 45, 55, 65)]
q = r.arm_q()
for name, t in seq:
    qq = c.ik_valid(origin(t), R45, seed=q, tries=6); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
r.report("start")
if r.finger_gap() > 0.01: r.gripper(0.0)
for name, t in seq:
    try:
        c.line(origin(t), R45, n=3, t=3.0)
    except RuntimeError as e:
        print(name, e)
    r.report(name)
p, _ = r.fk()
c.line(p + [0, 0, 0.10], R45, n=2, t=2.0); r.report("up")
for cam in ("agentview", "frontview", "birdview"):
    r.snap(cam)
OPENRUA_EOF

# openrua op 112
timeout 300 python3 right.py 2>&1 | tail -10

# openrua op 113
sed -i 's/    T = E - np.array(\[0, 0.0141\]) + np.array(\[dy, 0\])/    T = E + np.array([0.0141, -0.0141]) + np.array([dy, 0])   # tip 1 cm along the finger inside the rim plane/' right.py && timeout 300 python3 right.py 2>&1 | tail -8

# openrua op 114
sed -i 's/T0\[1\] - 0.018/T0[1] - 0.03/g' right.py && timeout 900 python3 right.py --go 2>&1 | grep -E "^\s+\[|did not|^[a-z0-9]+ move"

# openrua op 115
grep -n "def line" -A 30 ctl.py | head -50

# openrua op 116
timeout 900 python3 right.py --go 2>&1 | grep -E "line:|did not" | head

# openrua op 117
python3 - <<'EOF'
s=open('right.py').read()
s=s.replace("""for name, t in seq:
    try:
        c.line(origin(t), R45, n=3, t=3.0)
    except RuntimeError as e:
        print(name, e)
    r.report(name)""","""q_pre = c.ik_valid(origin(pre), R45, seed=r.arm_q(), tries=8)
assert c.goto_q(q_pre, t=5.0), "pre failed"
r.report("pre")
for name, t in seq[1:]:
    try:
        ok = c.line(origin(t), R45, n=3, t=3.0)
        if not ok: sys.exit(name + ": line failed")
    except RuntimeError as e:
        print(name, e)
    r.report(name)""")
open('right.py','w').write(s)
EOF
timeout 900 python3 right.py --go 2>&1 | grep -E "^\s+\[|did not|line"

# openrua op 118
timeout 300 python3 park.py birdview agentview sideview 2>&1 | tail -1; for c in birdview agentview sideview; do timeout 120 python3 cloud.py $c >/dev/null 2>&1; done; python3 - <<'EOF'
import numpy as np
for cam in ("birdview","agentview","sideview"):
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
    m=(col.min(1)>140)&(np.ptp(col,1)<30)&(pw[:,2]>0.905)&(pw[:,2]<1.1)&(pw[:,1]<0.2)&(pw[:,1]>-0.1)&(pw[:,0]<0.0)&(pw[:,0]>-0.3)
    P=pw[m]; print(cam,len(P))
    if len(P)<20: continue
    print(" x",np.round(np.percentile(P[:,0],[1,50,99]),3)," y",np.round(np.percentile(P[:,1],[1,50,99]),3)," z",np.round(np.percentile(P[:,2],[1,50,99]),3))
    for z0 in np.arange(0.90,1.03,0.01):
        s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.01)]
        if len(s)>3: print(f"  z[{z0:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 119
python3 - <<'EOF'
import numpy as np
for cam in ("birdview","agentview"):
    d=np.load(f"{cam}_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
    m=(col.min(1)>140)&(np.ptp(col,1)<30)&(pw[:,2]>0.905)&(pw[:,2]<1.1)&(pw[:,1]<0.2)&(pw[:,1]>0.0)&(pw[:,0]<-0.0)&(pw[:,0]>-0.26)
    P=pw[m]
    H=P[(P[:,0]<-0.19)]
    print(cam, "handle pts", len(H))
    for x0 in np.arange(-0.26,-0.18,0.01):
        s=H[(H[:,0]>=x0)&(H[:,0]<x0+0.01)]
        if len(s)>2: print(f"  x[{x0:.2f}] n={len(s)} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
    # rim circle fit from top points
    T=P[P[:,2]>0.998]
    A=np.c_[2*T[:,0],2*T[:,1],np.ones(len(T))]; b=(T[:,0]**2+T[:,1]**2)
    cx,cy,k=np.linalg.lstsq(A,b,rcond=None)[0]; rr=np.sqrt(k+cx**2+cy**2)
    print("  rim fit centre",round(cx,3),round(cy,3),"r",round(rr,3), "n",len(T))
EOF

# openrua op 120
cat > look.py <<'EOF'
import sys
from common import *
c = Ctl("look"); r, sc = c.r, c.sc
scene_no_white(sc)
x, y, z = map(float, sys.argv[1:4])
q = c.ik_valid(np.array([x, y, z]), R_DOWN, seed=r.arm_q(), tries=8)
assert q is not None and c.goto_q(q, t=5.0), "look move failed"
r.report("look")
r.snap("robot0_eye_in_hand")
EOF
timeout 300 python3 look.py -0.15 0.13 1.32 2>&1 | tail -1; timeout 120 python3 cloud.py robot0_eye_in_hand >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
d=np.load("robot0_eye_in_hand_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
m=(col.min(1)>140)&(np.ptp(col,1)<30)&(pw[:,2]>0.905)&(pw[:,2]<1.1)&(pw[:,1]<0.25)&(pw[:,1]>0.0)&(pw[:,0]<0.0)&(pw[:,0]>-0.3)
P=pw[m]; print(len(P))
for z0 in np.arange(0.90,1.03,0.01):
    s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.01)]
    if len(s)>3: print(f"  z[{z0:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
T=P[P[:,2]>1.0]
A=np.c_[2*T[:,0],2*T[:,1],np.ones(len(T))]; b=(T[:,0]**2+T[:,1]**2)
cx,cy,k=np.linalg.lstsq(A,b,rcond=None)[0]; print("rim fit", round(cx,3), round(cy,3), "r", round(np.sqrt(k+cx**2+cy**2),3), len(T))
H=P[(P[:,2]<0.995)&(P[:,2]>0.93)]
# points outside the body radius
dd=np.hypot(H[:,0]-cx,H[:,1]-cy); Hh=H[dd>0.055]
print("handle pts",len(Hh), "x",np.round(np.percentile(Hh[:,0],[5,50,95]),3),"y",np.round(np.percentile(Hh[:,1],[5,50,95]),3))
ang=np.degrees(np.arctan2(Hh[:,1]-cy,Hh[:,0]-cx)); print("handle angle deg", np.round(np.percentile(ang,[5,50,95]),1))
EOF

# openrua op 121
mkdir -p "$(dirname /workspace/push_white.py)"
cat > /workspace/push_white.py <<'OPENRUA_EOF'
"""Slide the (upside-down) white mug along direction e with closed fingertips (hand pointing down)."""
import sys
from common import *
c = Ctl("push_white"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
cen = np.array([-0.15, 0.135])
e = np.array([0.5, -0.866])
Z = 0.96
start = np.r_[cen - 0.077 * e, Z]
end = np.r_[cen + 0.043 * e, Z]
def origin(tip): return np.asarray(tip) + [0, 0, 0.1034]
q = r.arm_q()
for name, t in (("start", start), ("end", end)):
    qq = c.ik_valid(origin(t), R_DOWN, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
if r.finger_gap() > 0.01: r.gripper(0.0)
q0 = c.ik_valid(origin(start) + [0, 0, 0.12], R_DOWN, seed=r.arm_q(), tries=8)
assert c.goto_q(q0, t=4.0), "pre failed"; r.report("pre")
assert c.line(origin(start), R_DOWN, n=3, t=3.0), "descend failed"; r.report("start")
try:
    c.line(origin(end), R_DOWN, n=5, t=5.0)
except RuntimeError as ex:
    print("push:", ex)
r.report("end")
p, _ = r.fk()
assert c.line(p + [0, 0, 0.15], R_DOWN, n=2, t=2.0), "lift failed"; r.report("up")
OPENRUA_EOF

# openrua op 122
timeout 300 python3 push_white.py 2>&1 | tail -3

# openrua op 123
python3 - <<'EOF'
import numpy as np
d=np.load("robot0_eye_in_hand_cloud.npz"); pw=d["pw"].reshape(-1,3); col=d["color"].reshape(-1,3).astype(int)
m=(col.min(1)>140)&(np.ptp(col,1)<30)&(pw[:,2]>0.905)&(pw[:,2]<1.1)&(pw[:,1]<0.25)&(pw[:,1]>0.0)&(pw[:,0]<-0.05)&(pw[:,0]>-0.2)
P=pw[m]
top=P[P[:,2]>1.005]; cx,cy=top[:,0].mean(),top[:,1].mean(); print("top centre",round(cx,3),round(cy,3),"n",len(top), "z", np.round(np.percentile(top[:,2],[5,50,95]),4))
for z0 in np.arange(0.95,1.02,0.005):
    s=P[(P[:,2]>=z0)&(P[:,2]<z0+0.005)]
    if len(s)>5:
        rr=np.hypot(s[:,0]-cx,s[:,1]-cy); print(f"z[{z0:.3f}] n={len(s)} r p50={np.percentile(rr,50):.3f} p90={np.percentile(rr,90):.3f} max={rr.max():.3f}")
EOF

# openrua op 124
mkdir -p "$(dirname /workspace/bodygrasp.py)"
cat > /workspace/bodygrasp.py <<'OPENRUA_EOF'
"""Vertical body pinch on the upside-down mug (near its base = top), yawed 45 deg to clear the door.
Then carry it to a free spot, set it down (still upside down), release."""
import sys
from common import *
c = Ctl("bodygrasp"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
yh = np.array([0.7071, 0.7071, 0]); zh = np.array([0, 0, -1.0]); xh = np.cross(yh, zh)
Rv = R_from_axes(xh, yh, zh)
def origin(tip): return np.asarray(tip) + [0, 0, 0.1034]
CEN = np.array([-0.151, 0.135])
DST = np.array([-0.05, 0.05])
Zt_hi, Zt_lo = 1.06, 0.978
seq = [("above", np.r_[CEN, Zt_hi]), ("down", np.r_[CEN, Zt_lo]), ("lift", np.r_[CEN, Zt_lo + 0.04]),
       ("carry", np.r_[DST, Zt_lo + 0.04]), ("set", np.r_[DST, Zt_lo])]
q = r.arm_q()
for name, t in seq:
    qq = c.ik_valid(origin(t), Rv, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
r.report("start")
if r.finger_gap() < 0.07: r.gripper(0.08)
q0 = c.ik_valid(origin(seq[0][1]), Rv, seed=r.arm_q(), tries=8)
assert c.goto_q(q0, t=5.0), "above failed"; r.report("above")
try:
    c.line(origin(seq[1][1]), Rv, n=4, t=4.0)
except RuntimeError as e:
    print("down:", e)
r.report("down")
gap = r.gripper(0.0)
print("gap after close", gap)
if gap < 0.05:
    sys.exit("grasp missed")
p, _ = r.fk()
assert c.line(p + [0, 0, 0.04], Rv, n=2, t=2.0), "lift failed"; r.report("lift")
r.snap("agentview", "/workspace/lift_a.png")
assert c.line(origin(seq[3][1]), Rv, n=4, t=4.0), "carry failed"; r.report("carry")
try:
    c.line(origin(seq[4][1]) + [0, 0, 0.004], Rv, n=3, t=3.0)
except RuntimeError as e:
    print("set:", e)
r.report("set")
r.gripper(0.08)
p, _ = r.fk()
assert c.line(p + [0, 0, 0.10], Rv, n=2, t=2.0), "retreat failed"; r.report("up")
r.snap("agentview"); r.snap("frontview")
OPENRUA_EOF

# openrua op 125
timeout 300 python3 bodygrasp.py 2>&1 | tail -6

# openrua op 126
timeout 900 python3 bodygrasp.py --go 2>&1 | grep -E "^\s+\[|gap|down:|set:|missed|failed"

# openrua op 127
timeout 900 python3 bodygrasp.py --go 2>&1 | grep -vE "^(above|down|lift|carry|set) \[" | tail -15

# openrua op 128
cat > scanmug.py <<'EOF'
"""Top-down eye-in-hand scan of the white mug: prints top centre, radius profile and handle direction."""
import sys, subprocess
from common import *
x, y = float(sys.argv[1]), float(sys.argv[2])
c = Ctl("scan"); r, sc = c.r, c.sc
scene_no_white(sc)
q = c.ik_valid(np.array([x, y, 1.32]), R_DOWN, seed=r.arm_q(), tries=8)
assert q is not None and c.goto_q(q, t=5.0), "look move failed"
r.snap("robot0_eye_in_hand")
subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], capture_output=True)
d = np.load("robot0_eye_in_hand_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (pw[:, 2] > 0.905) & (pw[:, 2] < 1.1) & (np.abs(pw[:, 0] - x) < 0.15) & (np.abs(pw[:, 1] - y) < 0.15)
P = pw[m]; print("pts", len(P))
zt = np.percentile(P[:, 2], 99); top = P[P[:, 2] > zt - 0.006]
cx, cy = top[:, 0].mean(), top[:, 1].mean(); print("top z", round(zt, 4), "top centre", round(cx, 4), round(cy, 4), "n", len(top))
for z0 in np.arange(0.91, zt, 0.01):
    s = P[(P[:, 2] >= z0) & (P[:, 2] < z0 + 0.01)]
    if len(s) > 5:
        rr = np.hypot(s[:, 0] - cx, s[:, 1] - cy); print(f"  z[{z0:.2f}] n={len(s)} r p50={np.percentile(rr,50):.3f} p90={np.percentile(rr,90):.3f} max={rr.max():.3f}")
rr = np.hypot(P[:, 0] - cx, P[:, 1] - cy); H = P[rr > 0.055]
if len(H) > 10:
    ang = np.degrees(np.arctan2(H[:, 1] - cy, H[:, 0] - cx))
    print("handle pts", len(H), "angle p5/50/95", np.round(np.percentile(ang, [5, 50, 95]), 1), "r p50/95", np.round(np.percentile(rr[rr > 0.055], [50, 95]), 3), "z", np.round(np.percentile(H[:, 2], [5, 95]), 3))
EOF
timeout 300 python3 scanmug.py -0.05 0.05 2>&1 | grep -v "^  move"

# openrua op 129
timeout 300 python3 st2.py 2>&1 | tail -3; timeout 300 python3 scanmug.py -0.05 0.05 2>&1 | grep -v "^  move"

# openrua op 130
mkdir -p "$(dirname /workspace/tip.py)"
cat > /workspace/tip.py <<'OPENRUA_EOF'
"""Tip the upside-down mug over toward +y by pushing its top from -y with closed fingertips."""
import sys
from common import *
c = Ctl("tip"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
cx, cy = float(sys.argv[1]), float(sys.argv[2])
Rf = hand_R(np.pi / 2)
z_h = Rf[:, 2]
def origin(tip): return np.asarray(tip) - 0.1034 * z_h
Z = 0.99
t0 = np.array([cx, cy - 0.037 - 0.03, Z]); t1 = np.array([cx, cy + 0.02, Z])
q = r.arm_q()
for name, t in (("t0", t0), ("t1", t1)):
    qq = c.ik_valid(origin(t), Rf, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
if r.finger_gap() > 0.01: r.gripper(0.0)
q0 = c.ik_valid(origin(t0) + [0, 0, 0.10], Rf, seed=r.arm_q(), tries=8)
assert c.goto_q(q0, t=5.0), "pre failed"; r.report("pre")
assert c.line(origin(t0), Rf, n=3, t=3.0), "descend failed"; r.report("t0")
try:
    c.line(origin(t1), Rf, n=5, t=5.0)
except RuntimeError as e:
    print("push:", e)
r.report("t1")
p, _ = r.fk()
assert c.line(p + [0, 0, 0.12], Rf, n=2, t=2.0), "lift failed"; r.report("up")
r.snap("agentview"); r.snap("frontview")
OPENRUA_EOF

# openrua op 131
timeout 900 python3 tip.py -0.054 0.047 --go 2>&1 | grep -E "^\s+\[|push:|failed|^t[01]"

# openrua op 132
timeout 300 python3 scanmug.py -0.05 0.08 2>&1 | grep -v "^  move"

# openrua op 133
python3 - <<'EOF'
s=open('tip.py').read()
s=s.replace("""Rf = hand_R(np.pi / 2)
z_h = Rf[:, 2]
def origin(tip): return np.asarray(tip) - 0.1034 * z_h
Z = 0.99
t0 = np.array([cx, cy - 0.037 - 0.03, Z]); t1 = np.array([cx, cy + 0.02, Z])""",
"""yh = np.array([1.0, 0, 0]); zh = np.array([0, 0, -1.0]); xh = np.cross(yh, zh)
Rf = R_from_axes(xh, yh, zh)     # vertical fingers, finger backs facing +-y
def origin(tip): return np.asarray(tip) + [0, 0, 0.1034]
Z = 0.99
t0 = np.array([cx, cy - 0.067, Z]); t1 = np.array([cx, cy + 0.03, Z])""")
open('tip.py','w').write(s)
EOF
timeout 900 python3 tip.py -0.0495 0.0582 --go 2>&1 | grep -E "^\s+\[|push:|failed|^t[01]"

# openrua op 134
timeout 300 python3 scanmug.py -0.05 0.12 2>&1 | grep -E "pts|top|handle"

# openrua op 135
mkdir -p "$(dirname /workspace/laydown.py)"
cat > /workspace/laydown.py <<'OPENRUA_EOF'
"""Body-pinch the upside-down mug near its base (top), rotate so the handle points -x, carry to a free
spot, then lower while dragging so the mug lays down on its side with the rim end toward the robot."""
import sys
from common import *
c = Ctl("laydown"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
cx, cy = float(sys.argv[1]), float(sys.argv[2])
def Rpin(ang):
    yh = np.array([np.cos(ang), np.sin(ang), 0]); zh = np.array([0, 0, -1.0]); xh = np.cross(yh, zh)
    return R_from_axes(xh, yh, zh)
A0, A1 = np.radians(-15), np.radians(-43)
R0, R1 = Rpin(A0), Rpin(A1)
def origin(tip): return np.asarray(tip) + [0, 0, 0.1034]
DST = np.array([-0.05, -0.06])
m = np.array([np.cos(A1 + np.pi / 2), np.sin(A1 + np.pi / 2)])   # (0.68,0.73): direction the pin end drifts; rim end goes -m
def lay(phi_deg, drift=0.10):
    ph = np.radians(phi_deg)
    zt = 0.89 + 0.095 * np.cos(ph) + 0.047 * np.sin(ph)
    return np.r_[DST + m * drift * phi_deg / 90, zt]
lays = [10, 25, 40, 55, 70, 85, 90]
q = r.arm_q()
for name, t, R in [("above", np.r_[cx, cy, 1.06], R0), ("down", np.r_[cx, cy, 0.978], R0), ("rot", np.r_[cx, cy, 1.0], R1),
                   ("dst", np.r_[DST, 1.0], R1)] + [(f"lay{p}", lay(p), R1) for p in lays]:
    qq = c.ik_valid(origin(t), R, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
r.report("start")
if r.finger_gap() < 0.07: r.gripper(0.08)
q0 = c.ik_valid(origin([cx, cy, 1.06]), R0, seed=r.arm_q(), tries=8)
assert c.goto_q(q0, t=5.0), "above failed"; r.report("above")
try:
    c.line(origin([cx, cy, 0.978]), R0, n=4, t=4.0)
except RuntimeError as e:
    print("down:", e)
r.report("down")
gap = r.gripper(0.0); print("gap after close", gap)
if gap < 0.05: sys.exit("grasp missed")
p, _ = r.fk()
assert c.line(p + [0, 0, 0.022], R0, n=2, t=2.0), "lift failed"; r.report("lift")
q1 = c.ik_valid(origin([cx, cy, 1.0]), R1, seed=r.arm_q(), tries=8)
assert q1 is not None and c.goto_q(q1, t=4.0), "rot failed"; r.report("rot")
assert c.line(origin(np.r_[DST, 1.0]), R1, n=4, t=4.0), "carry failed"; r.report("dst")
for p_ in lays:
    try:
        c.line(origin(lay(p_)), R1, n=2, t=2.5)
    except RuntimeError as e:
        print(f"lay{p_}:", e)
    r.report(f"lay{p_}")
r.gripper(0.08)
p, _ = r.fk()
assert c.line(p + [0, 0, 0.12], R1, n=2, t=2.0), "retreat failed"; r.report("up")
r.snap("agentview"); r.snap("frontview")
OPENRUA_EOF

# openrua op 136
timeout 300 python3 laydown.py -0.048 0.127 2>&1 | tail -20

# openrua op 137
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/laydown.py", "old_string": "DST = np.array([-0.05, -0.06])\\nm = np.array([np.cos(A1 + np.pi / 2), np.sin(A1 + np.pi / 2)])   # (0.68,0.73): direction the pin end drifts; rim end goes -m\\ndef lay(phi_deg, drift=0.10):\\n    ph = np.radians(phi_deg)\\n    zt = 0.89 + 0.095 * np.cos(ph) + 0.047 * np.sin(ph)\\n    return np.r_[DST + m * drift * phi_deg / 90, zt]\\nlays = [10, 25, 40, 55, 70, 85, 90]", "new_string": "DST = np.array([-0.10, -0.10])\\nm = np.array([np.cos(A1 + np.pi / 2), np.sin(A1 + np.pi / 2)])   # (0.68,0.73): direction the pin end drifts; rim end goes -m\\n# (drift along m, tip z): rim touches the table at tip z ~0.985; drag first, then descend so the mug tilts rim-first toward -m\\nLAYS = [(0.03, 0.986), (0.045, 0.98), (0.055, 0.97), (0.065, 0.96), (0.075, 0.95), (0.08, 0.94)]\\ndef lay(i):\\n    d, zt = LAYS[i]\\n    return np.r_[DST + m * d, zt]\\nlays = range(len(LAYS))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 138
timeout 300 python3 laydown.py -0.048 0.127 2>&1 | tail -20

# openrua op 139
timeout 600 python3 laydown.py -0.048 0.127 --go 2>&1 | grep -v "^ *ik_valid" | tail -30

# openrua op 140
timeout 300 python3 scanmug.py -0.05 -0.05 2>&1 | grep -v "^ *ik_valid\|move:" | tail -20

# openrua op 141
mkdir -p "$(dirname /workspace/flip.py)"
cat > /workspace/flip.py <<'OPENRUA_EOF'
"""Pinch the outer handle bar of the upside-down mug from above (vertical fingers straddling the loop plane),
lift: the COM is 7.8 cm off the pin so the mug swings to hang with its base pointing away from the bar.
Carry to L and lower: it lands base-edge first and lays down on its side with the handle bar on top."""
import sys
from common import *
c = Ctl("flip"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
mx, my, ang = float(sys.argv[1]), float(sys.argv[2]), np.radians(float(sys.argv[3]))
u = np.array([np.cos(ang), np.sin(ang), 0]); d = np.array([-u[1], u[0], 0])
zh = np.array([0, 0, -1.0]); Rb = R_from_axes(np.cross(d, zh), d, zh)      # fingers close along d (across the bar)
B = np.r_[mx, my, 0] + 0.075 * u
def origin(tip): return np.asarray(tip) + [0, 0, 0.1034]
L = np.array([-0.15, -0.15])
Z_GRIP, Z_HI, Z_LAND = 0.935, 1.10, 1.005
seq = [("above", np.r_[B[:2], 1.06]), ("down", np.r_[B[:2], Z_GRIP]), ("lift", np.r_[B[:2], Z_HI]),
       ("carry", np.r_[L, Z_HI]), ("land", np.r_[L, Z_LAND])]
q = r.arm_q()
for name, t in seq:
    qq = c.ik_valid(origin(t), Rb, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
r.report("start")
if r.finger_gap() < 0.07: r.gripper(0.08)
q0 = c.ik_valid(origin(seq[0][1]), Rb, seed=r.arm_q(), tries=8)
assert c.goto_q(q0, t=5.0), "above failed"; r.report("above")
try:
    c.line(origin(seq[1][1]), Rb, n=4, t=4.0)
except RuntimeError as e:
    print("down:", e)
r.report("down")
gap = r.gripper(0.0); print("gap after close", gap)
if not (0.008 < gap < 0.03): sys.exit("bar grasp missed")
assert c.line(origin(seq[2][1]), Rb, n=4, t=6.0), "lift failed"; r.report("lift")
r.snap("agentview", "/workspace/hang_a.png"); r.snap("sideview", "/workspace/hang_s.png")
assert c.line(origin(seq[3][1]), Rb, n=4, t=4.0), "carry failed"; r.report("carry")
try:
    c.line(origin(seq[4][1]), Rb, n=5, t=6.0)
except RuntimeError as e:
    print("land:", e)
r.report("land")
r.gripper(0.08)
p, _ = r.fk()
assert c.line(p + [0, 0, 0.12], Rb, n=2, t=2.0), "retreat failed"; r.report("up")
r.snap("agentview"); r.snap("frontview")
OPENRUA_EOF

# openrua op 142
timeout 300 python3 flip.py -0.076 -0.1016 173.5 2>&1 | tail -8

# openrua op 143
timeout 600 python3 flip.py -0.076 -0.1016 173.5 --go 2>&1 | grep -v "^ *ik_valid\|move: error_code=0" | tail -30

# openrua op 144
mkdir -p "$(dirname /workspace/lay.py)"
cat > /workspace/lay.py <<'OPENRUA_EOF'
"""Rigid bar pinch of the upside-down mug (vertical fingers across the outer bar, which is vertical),
lift, yaw so the body is at -y of the bar, then pitch the hand to horizontal (+y): the mug then lies with
the handle bar on top, base toward -y. Lower onto the table, release."""
import sys
from common import *
c = Ctl("lay"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
mx, my, ang = float(sys.argv[1]), float(sys.argv[2]), np.radians(float(sys.argv[3]))
u = np.array([np.cos(ang), np.sin(ang), 0]); d = np.array([-u[1], u[0], 0])
zh = np.array([0, 0, -1.0]); Rb = R_from_axes(np.cross(d, zh), d, zh)
Rc = R_from_axes(np.array([0, -1.0, 0]), np.array([-1.0, 0, 0]), zh)      # body (x_hand) toward -y
Rh = hand_R(np.pi / 2, 0.0)                                              # horizontal, pointing +y, body below
B = np.r_[mx, my, 0] + 0.075 * u
def origin(tip, R): return np.asarray(tip) - 0.1034 * R[:, 2]
L = np.array([-0.12, -0.12])
Z_GRIP, Z_HI, Z_LAY = 0.935, 1.15, 1.02
seq = [("above", np.r_[B[:2], 1.06], Rb), ("down", np.r_[B[:2], Z_GRIP], Rb), ("lift", np.r_[B[:2], Z_HI], Rb),
       ("yaw", np.r_[B[:2], Z_HI], Rc), ("pitch", np.r_[L, Z_HI], Rh), ("lay", np.r_[L, Z_LAY], Rh), ("out", np.r_[L, Z_LAY + 0.12], Rh)]
q = r.arm_q()
for name, t, R in seq:
    qq = c.ik_valid(origin(t, R), R, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
r.report("start")
if r.finger_gap() < 0.07: r.gripper(0.08)
q0 = c.ik_valid(origin(seq[0][1], Rb), Rb, seed=r.arm_q(), tries=8)
assert c.goto_q(q0, t=5.0), "above failed"; r.report("above")
try:
    c.line(origin(seq[1][1], Rb), Rb, n=4, t=4.0)
except RuntimeError as e:
    print("down:", e)
r.report("down")
gap = r.gripper(0.0); print("gap after close", gap)
if not (0.008 < gap < 0.03): sys.exit("bar grasp missed")
assert c.line(origin(seq[2][1], Rb), Rb, n=4, t=6.0), "lift failed"; r.report("lift")
for name, t, R in seq[3:5]:
    qq = c.ik_valid(origin(t, R), R, seed=r.arm_q(), tries=8)
    assert qq is not None and c.goto_q(qq, t=6.0), name + " failed"; r.report(name)
    if r.finger_gap() < 0.008: sys.exit("lost the bar")
r.snap("agentview", "/workspace/hang_a.png")
try:
    c.line(origin(seq[5][1], Rh), Rh, n=5, t=6.0)
except RuntimeError as e:
    print("lay:", e)
r.report("lay")
r.gripper(0.08)
assert c.line(origin(seq[6][1], Rh), Rh, n=3, t=3.0), "retreat failed"; r.report("out")
r.snap("agentview"); r.snap("frontview")
OPENRUA_EOF

# openrua op 145
timeout 300 python3 scanmug.py -0.15 -0.15 2>&1 | grep -v "^ *ik_valid\|move:" | tail -14

# openrua op 146
timeout 300 python3 lay.py -0.0871 -0.1542 172.7 2>&1 | tail -8

# openrua op 147
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/lay.py", "old_string": "       (\\"yaw\\", np.r_[B[:2], Z_HI], Rc), (\\"pitch\\", np.r_[L, Z_HI], Rh),", "new_string": "       (\\"yaw\\", np.r_[B[:2], Z_HI], Rc), (\\"pitch45\\", np.r_[L, Z_HI], hand_R(np.pi / 2, np.pi / 4)), (\\"pitch\\", np.r_[L, Z_HI], Rh),", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 148
sed -i 's/for name, t, R in seq\[3:5\]:/for name, t, R in seq[3:6]:/; s/c.line(origin(seq\[5\]\[1\], Rh), Rh, n=5, t=6.0)/c.line(origin(seq[6][1], Rh), Rh, n=5, t=6.0)/; s/c.line(origin(seq\[6\]\[1\], Rh), Rh, n=3, t=3.0), "retreat failed"/c.line(origin(seq[7][1], Rh), Rh, n=3, t=3.0), "retreat failed"/' lay.py && grep -n "seq\[" lay.py && timeout 300 python3 lay.py -0.0871 -0.1542 172.7 2>&1 | tail -8

# openrua op 149
timeout 900 python3 lay.py -0.0871 -0.1542 172.7 --go 2>&1 | grep -v "^ *ik_valid\|move: error_code=0" | tail -30

# openrua op 150
cat scanmug.py | head -40

# openrua op 151
mkdir -p "$(dirname /workspace/scanlying.py)"
cat > /workspace/scanlying.py <<'OPENRUA_EOF'
"""Top-down eye-in-hand scan of the lying white mug: bar (topmost points) centre/extent and body extents."""
import sys, subprocess
from common import *
x, y = float(sys.argv[1]), float(sys.argv[2])
c = Ctl("scan"); r, sc = c.r, c.sc
scene_no_white(sc)
q = c.ik_valid(np.array([x, y, 1.32]), R_DOWN, seed=r.arm_q(), tries=8)
assert q is not None and c.goto_q(q, t=5.0), "look move failed"
r.snap("robot0_eye_in_hand")
subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], capture_output=True)
d = np.load("robot0_eye_in_hand_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (pw[:, 2] > 0.905) & (pw[:, 2] < 1.1) & (np.abs(pw[:, 0] - x) < 0.15) & (np.abs(pw[:, 1] - y) < 0.15)
P = pw[m]; print("pts", len(P))
print("x range", np.round(np.percentile(P[:, 0], [1, 99]), 4), "y range", np.round(np.percentile(P[:, 1], [1, 99]), 4), "z max", round(P[:, 2].max(), 4))
zt = np.percentile(P[:, 2], 99.5)
for dz in (0.006, 0.012, 0.02):
    top = P[P[:, 2] > zt - dz]
    print(f"top-{dz}: n={len(top)} centre {top[:,0].mean():.4f} {top[:,1].mean():.4f} x[{top[:,0].min():.3f},{top[:,0].max():.3f}] y[{top[:,1].min():.3f},{top[:,1].max():.3f}]")
for z0 in np.arange(0.91, zt, 0.01):
    s = P[(P[:, 2] >= z0) & (P[:, 2] < z0 + 0.01)]
    if len(s) > 5:
        print(f"  z[{z0:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
OPENRUA_EOF

# openrua op 152
timeout 300 python3 scanlying.py -0.12 -0.12 2>&1 | grep -v "^ *ik_valid\|move:" | tail -20

# openrua op 153
mkdir -p "$(dirname /workspace/stand.py)"
cat > /workspace/stand.py <<'OPENRUA_EOF'
"""Lying mug (axis along y, rim +y, bar on top along y): pinch the bar from above (fingers close along x),
lift, rotate the hand about x by 90 deg -> hand points +y, mug hangs upright (base down, handle -y).
Carry into the microwave cavity, set down, release, retreat."""
import sys
from common import *
c = Ctl("stand"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
bx, by = float(sys.argv[1]), float(sys.argv[2])          # bar centre (top view)
def Rx(th):
    th = np.radians(th); return np.array([[1, 0, 0], [0, np.cos(th), -np.sin(th)], [0, np.sin(th), np.cos(th)]])
Rb = R_from_axes(np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), np.array([0, 0, -1.0]))   # vertical, close along x
R45, Rf = Rx(45) @ Rb, Rx(90) @ Rb                                                       # Rf: z=+y, x=+z, y=+x
def origin(tip, R): return np.asarray(tip) - 0.1034 * R[:, 2]
CX, CY = -0.03, 0.365          # target mug centre in the cavity
OFF = 0.066                    # mug axis is this far beyond the tips along z_hand once upright
Z_CARRY, Z_IN, Z_SET = 1.10, 1.012, 1.004
seq = [("above", np.r_[bx, by, 1.10], Rb), ("down", np.r_[bx, by, 1.013], Rb), ("lift", np.r_[bx, by, 1.15], Rb),
       ("rot45", np.r_[bx, by, 1.15], R45), ("rot90", np.r_[bx, by, 1.15], Rf),
       ("carry", np.r_[CX, 0.05, Z_CARRY], Rf), ("front", np.r_[CX, 0.15, Z_IN], Rf), ("in", np.r_[CX, CY - OFF, Z_IN], Rf),
       ("set", np.r_[CX, CY - OFF, Z_SET], Rf), ("back", np.r_[CX, 0.10, Z_IN], Rf), ("up", np.r_[CX, 0.05, 1.25], Rf)]
q = r.arm_q()
for name, t, R in seq:
    qq = c.ik_valid(origin(t, R), R, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q
if dry: sys.exit()
S = {n: (t, R) for n, t, R in seq}
def go(name, t=5.0):
    tt, R = S[name]; qq = c.ik_valid(origin(tt, R), R, seed=r.arm_q(), tries=8)
    assert qq is not None and c.goto_q(qq, t=t), name + " failed"; r.report(name)
def line(name, n=4, t=4.0, tolerant=False):
    tt, R = S[name]
    try:
        ok = c.line(origin(tt, R), R, n=n, t=t)
        if not ok: sys.exit(name + ": line failed")
    except RuntimeError as e:
        if not tolerant: raise
        print(name, e)
    r.report(name)
r.report("start")
if r.finger_gap() < 0.07: r.gripper(0.08)
go("above"); line("down", tolerant=True)
gap = r.gripper(0.0); print("gap after close", gap)
if not (0.008 < gap < 0.03): sys.exit("bar grasp missed")
line("lift", t=5.0); go("rot45", 6.0); go("rot90", 6.0)
if r.finger_gap() < 0.008: sys.exit("lost the bar")
r.snap("agentview", "/workspace/hang_a.png")
go("carry", 6.0); line("front", n=4, t=5.0); line("in", n=5, t=6.0, tolerant=True); line("set", n=2, t=2.0, tolerant=True)
r.gripper(0.06)
line("back", n=4, t=4.0); line("up", n=3, t=3.0)
r.snap("agentview"); r.snap("frontview")
OPENRUA_EOF

# openrua op 154
timeout 300 python3 stand.py -0.120 -0.105 2>&1 | tail -12

# openrua op 155
timeout 900 python3 stand.py -0.120 -0.105 --go 2>&1 | grep -v "^ *ik_valid\|move: error_code=0" | tail -30

# openrua op 156
mkdir -p "$(dirname /workspace/peek.py)"
cat > /workspace/peek.py <<'OPENRUA_EOF'
"""Look into the cavity with the eye-in-hand camera (hand horizontal pointing +y) and report white points."""
import sys, subprocess
from common import *
c = Ctl("peek"); r, sc = c.r, c.sc
scene_no_white(sc)
Rf = R_from_axes(np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]))
P0 = np.array([float(a) for a in sys.argv[1:4]]) if len(sys.argv) >= 4 else np.array([-0.04, 0.02, 1.03])
q = c.ik_valid(P0, Rf, seed=r.arm_q(), tries=8)
assert q is not None and c.goto_q(q, t=5.0), "look move failed"
r.snap("robot0_eye_in_hand")
subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], capture_output=True)
d = np.load("robot0_eye_in_hand_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (pw[:, 2] > 0.905) & (pw[:, 2] < 1.1) & (pw[:, 0] > -0.16) & (pw[:, 0] < 0.07) & (pw[:, 1] > 0.2) & (pw[:, 1] < 0.45)
P = pw[m]; print("white pts in cavity region", len(P))
if len(P):
    print("x", np.round(np.percentile(P[:, 0], [1, 50, 99]), 4), "y", np.round(np.percentile(P[:, 1], [1, 50, 99]), 4), "z", np.round(np.percentile(P[:, 2], [1, 50, 99]), 4))
    top = P[P[:, 2] > np.percentile(P[:, 2], 99) - 0.008]
    print("top ring centre", np.round(top[:, :2].mean(0), 4), "n", len(top))
    for z0 in np.arange(0.94, 1.08, 0.02):
        s = P[(P[:, 2] >= z0) & (P[:, 2] < z0 + 0.02)]
        if len(s) > 5: print(f"  z[{z0:.2f}] n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
OPENRUA_EOF

# openrua op 157
timeout 300 python3 peek.py 2>&1 | grep -v "^ *ik_valid\|move:" | tail -14

# openrua op 158
sed -n 1,80p grasp.py; grep -n "def gripper" -A25 lib.py | head -60

# openrua op 159
grep -n "GRIP" -B2 -A6 lib.py | head -40; ls /workspace; ls /workspace/docs 2>/dev/null

# openrua op 160
grep -n -i "effort\|force\|grip" machine.yaml docs/*.md | head -30

# openrua op 161
sed -n 45,62p machine.yaml

# openrua op 162
mkdir -p "$(dirname /workspace/stand3.py)"
cat > /workspace/stand3.py <<'OPENRUA_EOF'
"""Phases (python3 stand3.py <phase> [args] [--go]):
  pull  bx by bz        : lying mug in cavity, bar on top along y. Rigid bar pinch with hand pointing +y (fingers
                          close along x), drag it out along -y, carry to LAY, lower, release.
  stand bx by           : lying mug on table (axis along y, rim +y, bar on top). Pinch bar from above (close along
                          x), lift, rotate about x -> hand points +y, mug hangs upright (handle -y). Then hold.
  measure               : snap sideview/agentview clouds, report hanging mug extents and tilt.
  pitch  deg            : rotate the hand about its y_hand (x world) by deg (compensate droop), keep tips fixed.
  insert base_dz top_dz cy: carry into cavity (tips y = cy - OFF_Y), set down, release, retreat.
"""
import sys, subprocess
from common import *
c = Ctl("stand3"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
phase = sys.argv[1]; args = [float(a) for a in sys.argv[2:] if not a.startswith("--")]
Rf = R_from_axes(np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]))    # z=+y, y=+x, x=+z
Rb = R_from_axes(np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), np.array([0, 0, -1.0]))   # vertical, close along x
def Rx(th):
    th = np.radians(th); return np.array([[1, 0, 0], [0, np.cos(th), -np.sin(th)], [0, np.sin(th), np.cos(th)]])
def origin(tip, R): return np.asarray(tip) - 0.1034 * R[:, 2]
LAY = np.array([-0.05, -0.17])
def tips_now():
    p, R = r.fk(); return p + 0.1034 * R[:, 2], R
def plan(seq):
    q = r.arm_q(); ok = True
    for name, t, R in seq:
        qq = c.ik_valid(origin(t, R), R, seed=q, tries=8); print(name, np.round(t, 3), None if qq is None else np.round(qq, 2)); q = qq or q; ok &= qq is not None
    return ok
def go(name, t, R, T=5.0):
    qq = c.ik_valid(origin(t, R), R, seed=r.arm_q(), tries=8)
    assert qq is not None and c.goto_q(qq, t=T), name + " failed"; r.report(name)
def line(name, t, R, n=4, T=4.0, tolerant=False):
    try:
        ok = c.line(origin(t, R), R, n=n, t=T)
        if not ok: sys.exit(name + ": line failed")
    except RuntimeError as e:
        if not tolerant: raise
        print(name, e)
    r.report(name)

if phase == "pull":
    bx, by, bz = args[:3]
    seq = [("pre", np.r_[bx, 0.10, bz], Rf), ("in", np.r_[bx, by, bz], Rf), ("drag", np.r_[bx, 0.0, bz], Rf),
           ("carry", np.r_[LAY, bz], Rf), ("lower", np.r_[LAY, 1.018], Rf), ("up", np.r_[LAY, 1.15], Rf)]
    ok = plan(seq)
    if dry or not ok: sys.exit()
    if r.finger_gap() < 0.07: r.gripper(0.08)
    go(*seq[0]); line(*seq[1], n=5, T=5.0, tolerant=True)
    gap = r.gripper(0.0); print("gap", gap)
    if not (0.008 < gap < 0.03): sys.exit("bar grasp missed")
    line(*seq[2], n=6, T=7.0, tolerant=True); line(*seq[3], n=4, T=4.0); line(*seq[4], n=3, T=3.0, tolerant=True)
    r.gripper(0.08); line(*seq[5], n=3, T=3.0)
    r.snap("agentview")
elif phase == "stand":
    bx, by = args[:2]
    R45 = Rx(45) @ Rb
    seq = [("above", np.r_[bx, by, 1.10], Rb), ("down", np.r_[bx, by, 1.013], Rb), ("lift", np.r_[bx, by, 1.15], Rb),
           ("rot45", np.r_[bx, by, 1.15], R45), ("rot90", np.r_[bx, by, 1.15], Rf)]
    ok = plan(seq)
    if dry or not ok: sys.exit()
    if r.finger_gap() < 0.07: r.gripper(0.08)
    go(*seq[0]); line(*seq[1], tolerant=True)
    gap = r.gripper(0.0); print("gap", gap)
    if not (0.008 < gap < 0.03): sys.exit("bar grasp missed")
    line(*seq[2], T=5.0); go(*seq[3], T=7.0); go(*seq[4], T=7.0)
    print("gap now", r.finger_gap())
    r.snap("agentview", "/workspace/hang_a.png"); r.snap("sideview", "/workspace/hang_s.png")
elif phase == "measure":
    tips, R = tips_now(); print("tips", np.round(tips, 4), "z_hand", np.round(R[:, 2], 3), "x_hand", np.round(R[:, 0], 3))
    for cam in ("sideview", "agentview"):
        r.snap(cam); subprocess.run(["python3", "cloud.py", cam], capture_output=True)
        d = np.load(f"{cam}_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
        m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (np.abs(pw[:, 0] - tips[0]) < 0.08) & (pw[:, 1] > tips[1] - 0.02) & (pw[:, 1] < tips[1] + 0.16) & (pw[:, 2] > tips[2] - 0.12) & (pw[:, 2] < tips[2] + 0.12)
        P = pw[m]; print(cam, "pts", len(P))
        if len(P) < 50: continue
        print("  z rel tips: min %.4f max %.4f" % (P[:, 2].min() - tips[2], P[:, 2].max() - tips[2]))
        print("  y rel tips: min %.4f max %.4f" % (P[:, 1].min() - tips[1], P[:, 1].max() - tips[1]))
        zs = np.arange(P[:, 2].min(), P[:, 2].max(), 0.01)
        for z0 in zs:
            s = P[(P[:, 2] >= z0) & (P[:, 2] < z0 + 0.01)]
            if len(s) > 5: print(f"    z[{z0-tips[2]:+.3f}] n={len(s)} y[{s[:,1].min()-tips[1]:+.3f},{s[:,1].max()-tips[1]:+.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
elif phase == "pitch":
    deg = args[0]
    tips, R = tips_now(); Rn = Rx(deg) @ R
    ok = plan([("pitch", tips, Rn)])
    if dry or not ok: sys.exit()
    line("pitch", tips, Rn, n=3, T=3.0); print("gap now", r.finger_gap())
elif phase == "insert":
    base_dz, top_dz, cy = args[:3]          # mug bottom / top z relative to tips (from measure)
    OFF_Y = 0.0755
    zt = 1.0145 - (base_dz + top_dz) / 2    # centre the mug vertically in the cavity (0.942..1.087)
    print("insert tips z", round(zt, 4), "-> base", round(zt + base_dz, 4), "top", round(zt + top_dz, 4))
    tips, R = tips_now(); CX = -0.03
    seq = [("carry", np.r_[CX, 0.05, zt + 0.05], R), ("front", np.r_[CX, 0.15, zt], R), ("in", np.r_[CX, cy - OFF_Y, zt], R),
           ("set", np.r_[CX, cy - OFF_Y, 0.947 - base_dz], R), ("back", np.r_[CX, 0.10, zt], R), ("up", np.r_[CX, 0.05, 1.25], R)]
    ok = plan(seq)
    if dry or not ok: sys.exit()
    go(*seq[0], T=6.0); line(*seq[1], n=4, T=5.0); line(*seq[2], n=6, T=7.0, tolerant=True); line(*seq[3], n=2, T=2.0, tolerant=True)
    r.gripper(0.08); line(*seq[4], n=4, T=4.0); line(*seq[5], n=3, T=3.0)
    r.snap("agentview"); r.snap("frontview")
OPENRUA_EOF

# openrua op 163
timeout 300 python3 stand3.py pull 0.0 0.305 1.058 2>&1 | grep -v "^ *ik_valid" | tail -8

# openrua op 164
timeout 300 python3 stand3.py pull 0.0 0.305 1.058 2>&1 | grep "ik_valid" | head -5

# openrua op 165
timeout 300 python3 stand3.py pull -0.02 0.295 1.058 2>&1 | grep -v "^ *ik_valid" | tail -8

# openrua op 166
timeout 900 python3 stand3.py pull -0.02 0.295 1.058 --go 2>&1 | grep -v "^ *ik_valid\|move: error_code=0" | tail -16

# openrua op 167
timeout 300 python3 scanlying.py -0.05 -0.17 2>&1 | grep -v "^ *ik_valid\|move:" | tail -16

# openrua op 168
timeout 900 python3 stand3.py stand -0.050 -0.158 --go 2>&1 | grep -v "^ *ik_valid\|move: error_code=0" | tail -14

# openrua op 169
timeout 300 python3 stand3.py measure --go 2>&1 | grep -v "^ *ik_valid\|move:" | tail -40

# openrua op 170
cat >> stand3.py <<'EOF'
elif phase == "insert2":
    cy = args[0] if args else 0.36
    def top_ring():
        tips, R = tips_now()
        r.snap("sideview"); subprocess.run(["python3", "cloud.py", "sideview"], capture_output=True)
        d = np.load("sideview_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
        m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (np.abs(pw[:, 0] - tips[0]) < 0.08) & (pw[:, 1] > tips[1] - 0.02) & (pw[:, 1] < tips[1] + 0.16) & (pw[:, 2] > tips[2] - 0.12) & (pw[:, 2] < tips[2] + 0.12)
        P = pw[m]
        if len(P) < 30: print("top_ring: few pts", len(P)); return None
        zt = P[:, 2].max(); top = P[P[:, 2] > zt - 0.006]
        yc = (top[:, 1].min() + top[:, 1].max()) / 2 - tips[1]
        print(f"top ring: n={len(top)} top_dz={zt - tips[2]:+.4f} yc_rel={yc:+.4f} y[{top[:,1].min()-tips[1]:+.3f},{top[:,1].max()-tips[1]:+.3f}]")
        return zt - tips[2], yc
    tips, R = tips_now(); CX = -0.03
    line("carry", np.r_[CX, 0.05, tips[2]], R, n=8, T=10.0)
    m = top_ring()
    if m is None: sys.exit("cannot see mug")
    top_dz, yc = m
    alpha = np.degrees(np.arcsin(np.clip((yc - 0.0755) / 0.057, -1, 1))); print("droop est deg", round(alpha, 1))
    if abs(alpha) > 5:
        tips, R = tips_now(); Rn = Rx(alpha) @ R
        line("pitchfix", tips, Rn, n=3, T=4.0); print("gap now", r.finger_gap())
        m = top_ring(); top_dz, yc = m if m else (top_dz, yc)
    base_dz = top_dz - 0.107
    zt = 1.0145 - (base_dz + top_dz) / 2
    print("insert tips z", round(zt, 4), "-> base", round(zt + base_dz, 4), "top", round(zt + top_dz, 4))
    tips, R = tips_now()
    seq = [("front", np.r_[CX, 0.15, zt], R), ("in", np.r_[CX, cy - 0.0755, zt], R),
           ("set", np.r_[CX, cy - 0.0755, 0.947 - base_dz], R), ("back", np.r_[CX, 0.10, zt], R), ("up", np.r_[CX, 0.05, 1.25], R)]
    if not plan(seq): sys.exit("plan failed")
    line(*seq[0], n=4, T=5.0); line(*seq[1], n=6, T=8.0, tolerant=True); r.report("in"); line(*seq[2], n=2, T=2.0, tolerant=True)
    r.gripper(0.08); line(*seq[3], n=4, T=4.0); line(*seq[4], n=3, T=3.0)
    r.snap("agentview"); r.snap("frontview")
EOF
timeout 900 python3 stand3.py insert2 0.36 --go 2>&1 | grep -v "^ *ik_valid\|move: error_code=0" | tail -24

# openrua op 171
timeout 300 python3 peek.py 2>&1 | grep -v "^ *ik_valid\|move:" | tail -14

# openrua op 172
timeout 300 python3 stand3.py pull -0.027 0.29 1.058 2>&1 | grep -v "^ *ik_valid" | tail -7

# openrua op 173
timeout 900 python3 stand3.py pull -0.027 0.29 1.058 --go 2>&1 | grep -v "^ *ik_valid\|move: error_code=0" | tail -9 && timeout 300 python3 scanlying.py -0.05 -0.17 2>&1 | grep -v "^ *ik_valid\|move:" | grep "top-0.006\|x range"

# openrua op 174
cat >> stand3.py <<'EOF'
elif phase == "insert3":
    cy = args[0] if args else 0.36
    def top_ring(cam):
        tips, R = tips_now()
        r.snap(cam); subprocess.run(["python3", "cloud.py", cam], capture_output=True)
        d = np.load(f"{cam}_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
        m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (np.abs(pw[:, 0] - tips[0]) < 0.08) & (pw[:, 1] > tips[1] - 0.02) & (pw[:, 1] < tips[1] + 0.16) & (pw[:, 2] > tips[2] - 0.12) & (pw[:, 2] < tips[2] + 0.12)
        P = pw[m]
        if len(P) < 30: print(cam, "top_ring: few pts", len(P)); return None
        zt = P[:, 2].max(); top = P[P[:, 2] > zt - 0.006]
        w = top[:, 1].max() - top[:, 1].min(); yc = (top[:, 1].min() + top[:, 1].max()) / 2 - tips[1]
        print(f"{cam} top ring: n={len(top)} top_dz={zt - tips[2]:+.4f} yc_rel={yc:+.4f} width={w:.3f} zmin_rel={P[:,2].min()-tips[2]:+.4f}")
        if w < 0.08: print("  (partial ring, unreliable)"); return None
        return zt - tips[2], yc
    m0 = top_ring("sideview")
    tips, R = tips_now(); CX = -0.03
    line("carry", np.r_[CX, 0.05, tips[2]], R, n=8, T=10.0)
    ms = [top_ring(cam) for cam in ("sideview", "agentview")]
    ms = [m for m in ms if m] or ([m0] if m0 else [])
    if not ms: sys.exit("no reliable measurement")
    top_dz = np.mean([m[0] for m in ms]); yc = np.mean([m[1] for m in ms])
    alpha = np.degrees(np.arcsin(np.clip((yc - 0.0755) / 0.057, -1, 1))); print("droop est deg", round(alpha, 1), "top_dz", round(top_dz, 4))
    if abs(alpha) > 8: sys.exit("droop too large; aborting insertion")
    base_dz = top_dz - 0.107
    zt = 1.0145 - (base_dz + top_dz) / 2
    print("insert tips z", round(zt, 4), "-> base", round(zt + base_dz, 4), "top", round(zt + top_dz, 4))
    tips, R = tips_now()
    seq = [("front", np.r_[CX, 0.15, zt], R), ("in", np.r_[CX, cy - 0.0755, zt], R),
           ("set", np.r_[CX, cy - 0.0755, 0.947 - base_dz], R), ("back", np.r_[CX, 0.10, zt], R), ("up", np.r_[CX, 0.05, 1.25], R)]
    if not plan(seq): sys.exit("plan failed")
    line(*seq[0], n=4, T=6.0); line(*seq[1], n=6, T=10.0, tolerant=True); line(*seq[2], n=2, T=2.0, tolerant=True)
    r.gripper(0.08); line(*seq[3], n=4, T=4.0); line(*seq[4], n=3, T=3.0)
    r.snap("agentview"); r.snap("frontview")
EOF
timeout 900 python3 stand3.py stand -0.0497 -0.1674 --go 2>&1 | grep -v "^ *ik_valid\|move: error_code=0" | tail -5

# openrua op 175
timeout 900 python3 stand3.py insert3 0.36 --go 2>&1 | grep -v "^ *ik_valid\|move: error_code=0" | tail -20

# openrua op 176
timeout 120 python3 -c "
from common import *
c=Ctl('snap'); r=c.r
r.snap('agentview'); r.snap('sideview'); r.snap('frontview')
" 2>&1 | tail -2

# openrua op 177
cat >> stand3.py <<'EOF'
elif phase == "insert4":
    base_dz, top_dz, cy = args[:3]
    zt = 1.0145 - (base_dz + top_dz) / 2
    print("insert tips z", round(zt, 4), "-> base", round(zt + base_dz, 4), "top", round(zt + top_dz, 4))
    tips, R = tips_now(); CX = -0.03
    seq = [("front", np.r_[CX, 0.15, zt], R), ("in", np.r_[CX, cy - 0.0755, zt], R),
           ("set", np.r_[CX, cy - 0.0755, 0.947 - base_dz], R), ("back", np.r_[CX, 0.10, zt], R), ("up", np.r_[CX, 0.05, 1.25], R)]
    if not plan(seq): sys.exit("plan failed")
    if dry: sys.exit()
    line(*seq[0], n=5, T=8.0); line(*seq[1], n=6, T=10.0, tolerant=True); line(*seq[2], n=2, T=2.0, tolerant=True)
    r.gripper(0.08); line(*seq[3], n=4, T=4.0); line(*seq[4], n=3, T=3.0)
    r.snap("agentview"); r.snap("frontview")
EOF
timeout 900 python3 stand3.py insert4 -0.050 0.057 0.36 --go 2>&1 | grep -v "^ *ik_valid\|move: error_code=0" | tail -12

# openrua op 178
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "^ *ik_valid\|move:" | tail -30
import subprocess
from common import *
c = Ctl("peek2"); r, sc = c.r, c.sc
scene_no_white(sc)
Rf = R_from_axes(np.array([0, 0, 1.0]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]))
q = c.ik_valid(np.array([-0.04, 0.02, 1.06]), Rf, seed=r.arm_q(), tries=8)
assert q is not None and c.goto_q(q, t=5.0)
r.snap("robot0_eye_in_hand")
subprocess.run(["python3", "cloud.py", "robot0_eye_in_hand"], capture_output=True)
d = np.load("robot0_eye_in_hand_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
m = (pw[:, 0] > -0.14) & (pw[:, 0] < 0.05) & (pw[:, 1] > 0.25) & (pw[:, 1] < 0.5) & (pw[:, 2] > 0.9) & (pw[:, 2] < 1.15)
P = pw[m]; C = col[m]
white = (C.min(1) > 140) & (np.ptp(C, 1) < 30)
print("cavity pts", len(P), "white", white.sum())
for y0 in np.arange(0.25, 0.48, 0.02):
    s = P[(P[:, 1] >= y0) & (P[:, 1] < y0 + 0.02) & ~white[(P[:, 1] >= y0) & (P[:, 1] < y0 + 0.02)]] if False else None
    sel = (P[:, 1] >= y0) & (P[:, 1] < y0 + 0.02) & ~white
    s = P[sel]
    if len(s) > 5:
        lo = s[s[:, 2] < 1.0]; hi = s[s[:, 2] > 1.0]
        print(f"y[{y0:.2f}] n={len(s)} zmin={s[:,2].min():.3f} z_lo_p50={np.percentile(lo[:,2],50) if len(lo) else np.nan:.3f} zmax={s[:,2].max():.3f} z_hi_p50={np.percentile(hi[:,2],50) if len(hi) else np.nan:.3f} x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
w = P[white]
if len(w): print("white: x", np.round(np.percentile(w[:,0],[1,50,99]),3), "y", np.round(np.percentile(w[:,1],[1,50,99]),3), "z", np.round(np.percentile(w[:,2],[1,50,99]),3))
EOF

# openrua op 179
timeout 60 python3 - <<'EOF'
import numpy as np
d = np.load("robot0_eye_in_hand_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
white = (col.min(1) > 140) & (np.ptp(col, 1) < 30)
for xr in [(-0.14, -0.08), (0.02, 0.06)]:
    m = (pw[:, 0] > xr[0]) & (pw[:, 0] < xr[1]) & (pw[:, 1] > 0.2) & (pw[:, 1] < 0.5) & (pw[:, 2] > 0.85) & (pw[:, 2] < 1.15) & ~white
    P = pw[m]; C = col[m]
    print("x range", xr, "n", len(P))
    for y0 in np.arange(0.20, 0.46, 0.01):
        s = P[(P[:, 1] >= y0) & (P[:, 1] < y0 + 0.01)]
        cc = C[(P[:, 1] >= y0) & (P[:, 1] < y0 + 0.01)]
        if len(s) > 3:
            lo = s[:, 2] < 1.02
            print(f"  y[{y0:.2f}] n={len(s)} zmin={s[:,2].min():.3f} z<1.02 p50={np.percentile(s[lo,2],50) if lo.sum() else np.nan:.3f} zmax={s[:,2].max():.3f} col={cc.mean(0).astype(int)}")
EOF

# openrua op 180
grep -n "mw_\|box\|Box" scene.py | head -40

# openrua op 181
timeout 120 python3 - <<'EOF'
import subprocess, numpy as np
from common import *
c = Ctl("fv"); r = c.r
r.snap("frontview"); r.snap("agentview")
for cam in ("frontview", "agentview"):
    subprocess.run(["python3", "cloud.py", cam], capture_output=True)
    d = np.load(f"{cam}_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
    m = (pw[:, 0] > -0.14) & (pw[:, 0] < -0.08) & (pw[:, 1] > 0.20) & (pw[:, 1] < 0.40) & (pw[:, 2] > 0.90) & (pw[:, 2] < 1.03)
    P = pw[m]; print(cam, "n", len(P))
    for y0 in np.arange(0.20, 0.40, 0.01):
        s = P[(P[:, 1] >= y0) & (P[:, 1] < y0 + 0.01)]
        if len(s) > 3: print(f"  y[{y0:.2f}] n={len(s)} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]")
EOF

# openrua op 182
grep -n "phase\|^def \|^if \|^elif \|sys.argv" stand3.py | head -60; wc -l stand3.py

# openrua op 183
grep -o "top ring[^\\\\]*\\\\n[^\\\\]*" /claude-config/projects/-workspace/dcd9b893-c459-4201-a1d5-d46a48ae17c8.jsonl | head -20; grep -o "z rel tips[^\\\\]*" /claude-config/projects/-workspace/dcd9b893-c459-4201-a1d5-d46a48ae17c8.jsonl | head; grep -o "droop est[^\\\\]*" /claude-config/projects/-workspace/dcd9b893-c459-4201-a1d5-d46a48ae17c8.jsonl | head

# openrua op 184
ls -t /workspace/*.png | head -5

# openrua op 185
mkdir -p "$(dirname /workspace/insert5.py)"
cat > /workspace/insert5.py <<'OPENRUA_EOF'
"""Careful insertion of the hanging upright mug (after stand3.py stand). python3 insert5.py [cy] [--go]
1. carry level to y 0.05, descend to the 'front' pose, measure the mug (frontview/agentview clouds): base/top z, tilt.
2. re-centre the mug vertically in the opening (0.945 .. 1.089) if needed, re-measure.
3. advance in 3 cm steps, checking the hand tracks the commanded pose; stop and back off on deviation.
4. set down, release, retreat."""
import sys, subprocess
from common import *
c = Ctl("ins5"); r, sc = c.r, c.sc
dry = "--go" not in sys.argv
scene_no_white(sc)
args = [float(a) for a in sys.argv[1:] if not a.startswith("--")]
CY = args[0] if args else 0.36
CX, OFF_Y = -0.03, 0.0755
FLOOR, TOP = 0.945, 1.089
def origin(tip, R): return np.asarray(tip) - 0.1034 * R[:, 2]
def tips_now():
    p, R = r.fk(); return p + 0.1034 * R[:, 2], R
def line(name, t, R, n=4, T=4.0, tolerant=False):
    try:
        ok = c.line(origin(t, R), R, n=n, t=T)
        if not ok: sys.exit(name + ": line failed")
    except RuntimeError as e:
        if not tolerant: raise
        print(name, e)
    r.report(name)
def measure():
    tips, R = tips_now(); best = None
    for cam in ("frontview", "agentview", "sideview"):
        r.snap(cam); subprocess.run(["python3", "cloud.py", cam], capture_output=True)
        d = np.load(f"{cam}_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
        m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (pw[:, 0] > CX - 0.08) & (pw[:, 0] < CX + 0.08) & (pw[:, 1] > tips[1] - 0.01) & (pw[:, 1] < tips[1] + 0.17) & (pw[:, 2] > 0.90) & (pw[:, 2] < 1.13)
        P = pw[m]
        if len(P) < 50: print(cam, "few pts", len(P)); continue
        zmin, zmax = P[:, 2].min(), P[:, 2].max()
        print(f"{cam}: n={len(P)} z[{zmin:.4f},{zmax:.4f}] (rel tips {zmin-tips[2]:+.4f},{zmax-tips[2]:+.4f}) height {zmax-zmin:.4f}")
        for z0 in np.arange(zmin, zmax, 0.02):
            s = P[(P[:, 2] >= z0) & (P[:, 2] < z0 + 0.02)]
            if len(s) > 5: print(f"   z[{z0:.3f}] n={len(s)} y[{s[:,1].min()-tips[1]:+.3f},{s[:,1].max()-tips[1]:+.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
        if best is None or (zmax - zmin) > best[2] - best[1]: best = (cam, zmin, zmax)
    return best
tips, R = tips_now(); print("tips", np.round(tips, 4), "z_hand", np.round(R[:, 2], 3))
if dry: sys.exit()
if r.finger_gap() < 0.008: sys.exit("nothing held")
ZT = 1.014
line("carry", np.r_[CX, 0.05, tips[2]], R, n=6, T=8.0)
line("front", np.r_[CX, 0.15, ZT], R, n=6, T=8.0)
for it in range(3):
    b = measure()
    if b is None: sys.exit("cannot see the mug")
    cam, zmin, zmax = b
    if zmax - zmin < 0.095: print("mug not fully seen (height %.3f); trusting nominal base = top - 0.107" % (zmax - zmin)); zmin = zmax - 0.107
    shift = (FLOOR + TOP) / 2 - (zmin + zmax) / 2
    print(f"base {zmin:.4f} top {zmax:.4f} -> shift {shift:+.4f}")
    if abs(shift) < 0.004: break
    tips, R = tips_now(); ZT = tips[2] + shift
    line("recentre", np.r_[CX, 0.15, ZT], R, n=2, T=3.0)
base_dz = zmin - tips_now()[0][2]
print("gap", r.finger_gap(), "base_dz", round(base_dz, 4))
# advance in steps
y_goal = CY - OFF_Y
tips, R = tips_now(); y = tips[1]; ok = True
while y < y_goal - 0.002:
    y = min(y + 0.03, y_goal)
    tgt = np.r_[CX, y, ZT]
    try:
        c.line(origin(tgt, R), R, n=2, t=3.0)
    except RuntimeError as e:
        print("step", round(y, 3), e)
    tips2, R2 = tips_now(); dev = tips2 - tgt
    print(f"step y={y:.3f}: tips {np.round(tips2,4)} dev {np.round(dev,4)} gap {r.finger_gap():.4f}")
    if abs(dev[2]) > 0.006 or abs(dev[0]) > 0.006 or abs(dev[1]) > 0.006:
        print("DEVIATION - stopping"); ok = False
        r.snap("frontview", "/workspace/dev_f.png"); r.snap("agentview", "/workspace/dev_a.png")
        break
if ok:
    tips, R = tips_now()
    line("set", np.r_[CX, y_goal, FLOOR + 0.002 - base_dz], R, n=2, T=2.5, tolerant=True)
    r.gripper(0.08)
    line("back", np.r_[CX, 0.10, ZT], R, n=4, T=4.0); line("up", np.r_[CX, 0.05, 1.25], R, n=3, T=3.0)
else:
    tips, R = tips_now()
    line("backoff", np.r_[CX, tips[1] - 0.04, ZT], R, n=2, T=3.0, tolerant=True)
r.snap("agentview"); r.snap("frontview")
OPENRUA_EOF

# openrua op 186
timeout 60 python3 - <<'EOF'
import numpy as np
d = np.load("robot0_eye_in_hand_cloud.npz"); pw = d["pw"].reshape(-1, 3); col = d["color"].reshape(-1, 3).astype(int)
m = (col.min(1) > 140) & (np.ptp(col, 1) < 30) & (pw[:, 2] > 1.05) & (pw[:, 1] > 0.25) & (pw[:, 1] < 0.45) & (pw[:, 0] > -0.16) & (pw[:, 0] < 0.07)
P = pw[m]; print(len(P))
print("bar x", np.round(np.percentile(P[:,0],[2,50,98]),4), "y", np.round(np.percentile(P[:,1],[2,50,98]),4), "z", np.round(np.percentile(P[:,2],[2,50,98]),4))
EOF

# openrua op 187
timeout 300 python3 stand3.py pull -0.04 0.30 1.058 --go 2>&1 | tail -25
