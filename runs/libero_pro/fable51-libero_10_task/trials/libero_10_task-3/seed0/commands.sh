#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | head -50; echo ---; timeout 20 ros2 node list

# openrua op 3
for c in agentview birdview frontview sideview galleryview paperview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls *.png; timeout 20 ros2 topic echo /joint_states --once

# openrua op 4
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id" | head -40; echo ---; timeout 15 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | head; echo ---; timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:"

# openrua op 5
cat > /workspace/tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
seen = {}
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
end = time.time()+5
while time.time() < end: rclpy.spin_once(node, timeout_sec=0.2)
for (p,c),t in sorted(seen.items()):
    print(f"{p:>28} -> {c:<28} t=({t.translation.x:.4f},{t.translation.y:.4f},{t.translation.z:.4f}) q=({t.rotation.x:.4f},{t.rotation.y:.4f},{t.rotation.z:.4f},{t.rotation.w:.4f})")
EOF
timeout 40 python3 /workspace/tfdump.py

# openrua op 6
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && cat > /workspace/bird.py <<'EOF'
import numpy as np, sys
d = np.load("/workspace/birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
# camera at (-0.2,0,3.0), q=(0.7071,0.7071,0,0): R = rot 180deg about (1,1,0)/sqrt2
# R = [[0,1,0],[1,0,0],[0,0,-1]]
def px2w(u,v):
    z=d[v,u]; X=(u-cx)*z/fx; Y=(v-cy)*z/fy
    return np.array([Y-0.2, X, 3.0-z])
H,W=d.shape
uu,vv=np.meshgrid(np.arange(W),np.arange(H))
X=(uu-cx)*d/fx; Y=(vv-cy)*d/fy
wx=Y-0.2; wy=X; wz=3.0-d
np.save("/workspace/bird_world.npy", np.stack([wx,wy,wz],-1))
print("table height estimate (median z in center):", np.median(wz[350:450,150:500]))
for name,(u,v) in {"bottle_cap":(335,258),"bowl":(308,300),"drawer_handle":(345,300),"drawer_front_center":(365,300),"cabinet_top":(420,300),"shelf":(235,270),"table_free":(300,400)}.items():
    print(name, (u,v), px2w(u,v).round(4), "depth",d[v,u].round(4))
EOF
python3 /workspace/bird.py

# openrua op 7
timeout 60 python3 tools/perception/px2world.py birdview 335 258; timeout 60 python3 tools/perception/px2world.py agentview 348 172; timeout 60 python3 tools/perception/px2world.py robot0_robotview 375 90

# openrua op 8
cat > /workspace/seg.py <<'EOF'
import numpy as np, cv2
W = np.load("/workspace/bird_world.npy"); wx,wy,wz = W[...,0],W[...,1],W[...,2]
img = cv2.imread("/workspace/birdview.png")
# bottle: region around (335,258), height>0.95
def region(name, u0,u1,v0,v1, zmin, zmax=9):
    m = np.zeros(wz.shape,bool); m[v0:v1,u0:u1]=True
    m &= (wz>zmin)&(wz<zmax)
    if m.sum()==0: print(name,"empty"); return
    print(f"{name}: n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] z[{wz[m].min():.3f},{wz[m].max():.3f}] centroid=({wx[m].mean():.3f},{wy[m].mean():.3f})")
    vs,us = np.where(m); print("   px u[%d,%d] v[%d,%d]"%(us.min(),us.max(),vs.min(),vs.max()))
region("bottle", 315,360,235,285, 0.95)
region("bowl", 280,335,275,330, 0.92)
region("shelf", 190,285,220,320, 0.92)
# cabinet body and drawer
region("cabinet_all", 335,470,240,350, 0.92)
region("cabinet_top(>1.1)", 335,470,240,350, 1.1)
region("drawer_region(0.92-1.1)", 335,470,240,350, 0.92, 1.1)
# drawer interior floor: lower z between drawer walls
region("drawer_interior(<0.95)", 345,400,255,340, 0.905, 0.96)
EOF
python3 /workspace/seg.py

# openrua op 9
cat >> /workspace/seg.py <<'EOF'
print("----- refined")
region("bottle_body(0.95-1.15)", 315,360,235,285, 0.95, 1.15)
region("hand(>1.2)", 280,380,200,285, 1.2)
# drawer analysis by columns: for each v row through drawer, print z profile along u
row = 300
print("z profile along row v=300 (u 330..460):")
for u in range(330,460,4):
    print(f"  u={u} x={wx[row,u]:.3f} y={wy[row,u]:.3f} z={wz[row,u]:.3f}")
EOF
python3 /workspace/seg.py 2>&1 | sed -n '/refined/,$p'

# openrua op 10
cat > /workspace/seg2.py <<'EOF'
import numpy as np
W = np.load("/workspace/bird_world.npy"); wx,wy,wz = W[...,0],W[...,1],W[...,2]
for u in (350, 365, 380):
    print(f"column u={u} (y~{wy[300,u]:.3f}):")
    for v in range(240,345,3):
        print(f"  v={v} x={wx[v,u]:.3f} y={wy[v,u]:.3f} z={wz[v,u]:.3f}")
EOF
python3 /workspace/seg2.py

# openrua op 11
cat > /workspace/seg3.py <<'EOF'
import numpy as np
W = np.load("/workspace/bird_world.npy"); wx,wy,wz = W[...,0],W[...,1],W[...,2]
for lo,hi in [(0.95,1.0),(1.0,1.05),(1.05,1.1),(1.1,1.15)]:
    m=np.zeros(wz.shape,bool); m[235:290,315:365]=True; m&=(wz>lo)&(wz<hi)
    if m.sum(): print(f"z in ({lo},{hi}): n={m.sum()} x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] centroid=({wx[m].mean():.3f},{wy[m].mean():.3f})")
# row through the bottle
v=262
print("row v=262:", [(u, round(float(wx[v,u]),3), round(float(wy[v,u]),3), round(float(wz[v,u]),3)) for u in range(322,362,2)])
EOF
python3 /workspace/seg3.py

# openrua op 12
python3 -c "
import cv2
im=cv2.imread('birdview.png'); c=im[220:320,290:400]; cv2.imwrite('crop_bird.png', cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); c=im[140:260,300:420]; cv2.imwrite('crop_agent.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 13
timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png >/dev/null && cat > /workspace/agentworld.py <<'EOF'
import numpy as np
d=np.load("/workspace/agentview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
qx,qy,qz,qw=0.6380,0.6380,-0.3048,-0.3048
t=np.array([0.6586,0.0,1.6104])
x,y,z,w=qx,qy,qz,qw
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
H,W=d.shape; uu,vv=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1)
Wd=P@R.T+t
np.save("/workspace/agent_world.npy",Wd)
wx,wy,wz=Wd[...,0],Wd[...,1],Wd[...,2]
# column through the open drawer's +x side wall, u=420 (drawer box), v from 230 to 360
for u in (400,430):
  print("column u=",u)
  for v in range(230,370,4):
    print(f"  v={v} x={wx[v,u]:.3f} y={wy[v,u]:.3f} z={wz[v,u]:.3f}")
EOF
python3 /workspace/agentworld.py

# openrua op 14
timeout 20 ros2 node info /robot_bridge 2>&1 | head -60; echo ---; timeout 20 ros2 interface show robot_bridge/srv/LoadGeometryFromFile 2>/dev/null; timeout 20 ros2 service type /load_geometry_from_file; timeout 20 ros2 service type /get_urdf; timeout 20 ros2 service type /load_map

# openrua op 15
python3 -c "
import cv2
im=cv2.imread('robot0_robotview.png'); c=im[0:400,380:640]; cv2.imwrite('crop_robotview.png', cv2.resize(c,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('frontview.png'); c=im[240:420,320:600]; cv2.imwrite('crop_front.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 16
timeout 60 python3 tools/perception/cam_snap.py /robot0_robotview/depth/image_raw rv_depth.png >/dev/null && cat > /workspace/rvworld.py <<'EOF'
import numpy as np, subprocess
d=np.load("/workspace/rv_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
qx,qy,qz,qw=0.6531,0.6531,-0.2710,-0.2710
t=np.array([0.3400,0.0,1.3120])
x,y,z,w=qx,qy,qz,qw
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
H,W=d.shape; uu,vv=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1)
Wd=P@R.T+t; np.save("/workspace/rv_world.npy",Wd)
wx,wy,wz=Wd[...,0],Wd[...,1],Wd[...,2]
# the drawer's center column in image: u ~ 500 ; scan v from 140 to 400
for u in (470,500,530):
  print("column u=",u)
  for v in range(140,400,6):
    print(f"  v={v} x={wx[v,u]:.3f} y={wy[v,u]:.3f} z={wz[v,u]:.3f}")
EOF
python3 /workspace/rvworld.py

# openrua op 17
cat > /workspace/rvrow.py <<'EOF'
import numpy as np
Wd=np.load("/workspace/rv_world.npy"); wx,wy,wz=Wd[...,0],Wd[...,1],Wd[...,2]
for v in (250,300):
  print("row v=",v)
  for u in range(380,640,5):
    print(f"  u={u} x={wx[v,u]:.3f} y={wy[v,u]:.3f} z={wz[v,u]:.3f}")
EOF
python3 /workspace/rvrow.py

# openrua op 18
cat > /workspace/bottle_meas.py <<'EOF'
import numpy as np
Wd=np.load("/workspace/agent_world.npy"); wx,wy,wz=Wd[...,0],Wd[...,1],Wd[...,2]
# bottle pixels in agentview: roughly u 335-365, v 160-245. Restrict to region near bottle x,y
m=np.zeros(wz.shape,bool); m[150:250,325:375]=True
m&=(wx>-0.20)&(wx<-0.07)&(np.abs(wy-0.048)<0.06)&(wz>0.90)
print("n",m.sum(), "x",wx[m].min().round(3),wx[m].max().round(3),"y",wy[m].min().round(3),wy[m].max().round(3),"z",wz[m].min().round(3),wz[m].max().round(3))
for lo in np.arange(0.90,1.14,0.02):
    mm=m&(wz>=lo)&(wz<lo+0.02)
    if mm.sum(): print(f"z[{lo:.2f},{lo+0.02:.2f}] n={mm.sum()} y[{wy[mm].min():.3f},{wy[mm].max():.3f}] width={wy[mm].max()-wy[mm].min():.3f} xfront={wx[mm].min():.3f}")
EOF
python3 /workspace/bottle_meas.py

# openrua op 19
timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw fv_depth.png >/dev/null && cat > /workspace/fvworld.py <<'EOF'
import numpy as np
d=np.load("/workspace/fv_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
qx,qy,qz,qw=0.5608,0.5608,-0.4306,-0.4306
t=np.array([1.0,0.0,1.48])
x,y,z,w=qx,qy,qz,qw
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
H,W=d.shape; uu,vv=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1)
Wd=P@R.T+t; np.save("/workspace/fv_world.npy",Wd)
wx,wy,wz=Wd[...,0],Wd[...,1],Wd[...,2]
# cabinet front face (facing -y) pixels: y between 0.20 and 0.26, x in [-0.2,0.12], z>0.9
m=(wy>0.195)&(wy<0.26)&(wx>-0.2)&(wx<0.12)&(wz>0.905)
print("front-face-ish pixels:",m.sum())
for lo in np.arange(0.90,1.14,0.01):
    mm=m&(wz>=lo)&(wz<lo+0.01)
    if mm.sum(): print(f"z[{lo:.3f}] n={mm.sum():4d} y[{wy[mm].min():.3f},{wy[mm].max():.3f}] x[{wx[mm].min():.3f},{wx[mm].max():.3f}]")
EOF
python3 /workspace/fvworld.py

# openrua op 20
cat >> /workspace/fvworld.py <<'EOF'
print("---- front face only: y in [0.225,0.245], x in [-0.10,0.09]")
m=(wy>0.225)&(wy<0.245)&(wx>-0.10)&(wx<0.09)&(wz>0.905)
for lo in np.arange(0.90,1.14,0.005):
    mm=m&(wz>=lo)&(wz<lo+0.005)
    print(f"z[{lo:.3f}] n={mm.sum():4d}" + (f" y[{wy[mm].min():.3f},{wy[mm].max():.3f}] x[{wx[mm].min():.3f},{wx[mm].max():.3f}]" if mm.sum() else ""))
print("---- handles: y in [0.195,0.225]")
m=(wy>0.195)&(wy<0.225)&(wx>-0.10)&(wx<0.09)&(wz>0.905)
for lo in np.arange(0.90,1.14,0.01):
    mm=m&(wz>=lo)&(wz<lo+0.01)
    if mm.sum(): print(f"z[{lo:.3f}] n={mm.sum():4d} y[{wy[mm].min():.3f},{wy[mm].max():.3f}] x[{wx[mm].min():.3f},{wx[mm].max():.3f}]")
EOF
python3 /workspace/fvworld.py | sed -n '/front face only/,$p'

# openrua op 21
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small control helper for this Panda: joint state, FK/IK (MoveIt), trajectory,
gripper, and cartesian servo. All poses are WORLD frame unless noted; the
planner frame (panda_link0) is world shifted by BASE_T (from /tf)."""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_T = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
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
    return np.array([x, y, z, w])


def rot_from_axes(zaxis, xaxis_hint):
    """Rotation whose z column is zaxis and x column is closest to xaxis_hint."""
    z = np.asarray(zaxis, float); z /= np.linalg.norm(z)
    x = np.asarray(xaxis_hint, float); x = x - z * (x @ z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z], axis=1)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.05): rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None: self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def finger(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None: self.spin(0.2)
        f = self._wr.wrench.force; t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    # ---- kinematics (planner frame = panda_link0) ----
    def fk_hand(self, q=None):
        """world pose of panda_hand: (pos, quat, R)."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_T
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat, quat_to_R(quat)

    def tcp(self, q=None):
        pos, quat, R = self.fk_hand(q)
        return pos + TCP * R[:, 2], quat, R

    def ik_hand(self, pos_w, quat, seed=None, timeout=20.0):
        """IK for panda_hand at world pos/quat -> joint list or None."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos_w, float) - BASE_T
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed = self.arm_q() if seed is None else seed
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(map(float, seed))
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=90)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp(self, tcp_w, R, seed=None, timeout=20.0):
        pos = np.asarray(tcp_w, float) - TCP * R[:, 2]
        return self.ik_hand(pos, R_to_quat(R), seed, timeout)

    # ---- motion ----
    def move_q(self, q, seconds=3.0, via=None):
        """One trajectory to q (optionally through intermediate points `via`)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        allq = (via or []) + [q]
        n = len(allq)
        for i, qq in enumerate(allq):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=list(map(float, qq)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        r = rf.result()
        code = r.result.error_code if r else None
        qa = np.array(self.arm_q())
        err = np.abs(qa - np.array(q)).max()
        print(f"[move_q] code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_tcp(self, tcp_w, R, seconds=3.0, seed=None):
        q = self.ik_tcp(tcp_w, R, seed)
        if q is None:
            print(f"[move_tcp] IK FAILED for {np.round(tcp_w,3)}", flush=True)
            return None
        code, err = self.move_q(q, seconds)
        p, _, _ = self.tcp()
        print(f"[move_tcp] tcp now {np.round(p,4)} (target {np.round(tcp_w,4)})", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.finger()
        print(f"[gripper] target={width} reached={r.reached_goal} stalled={r.stalled} fingers={np.round(f,4)}", flush=True)
        return f

    def servo(self, lin, ang=(0, 0, 0), ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)


if __name__ == "__main__":
    r = Robot()
    q = r.arm_q()
    print("q:", np.round(q, 4))
    print("fingers:", r.finger())
    p, quat, R = r.fk_hand(q)
    print("hand world:", np.round(p, 4), "quat:", np.round(quat, 4))
    print("R:\n", np.round(R, 3))
    print("tcp world:", np.round(p + TCP * R[:, 2], 4))
    print("wrench:", r.wrench())
OPENRUA_EOF

# openrua op 22
timeout 120 python3 rob.py

# openrua op 23
cat > /workspace/urdf.py <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("u")
got=[]
n.create_subscription(String,"/robot_description",got.append,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
import time; t=time.time()
while not got and time.time()-t<10: rclpy.spin_once(n,timeout_sec=0.2)
s=got[0].data; open("/workspace/robot.urdf","w").write(s); print(len(s))
EOF
timeout 30 python3 /workspace/urdf.py; grep -n -E "<link name=|<joint name=|<origin|<parent|<child" /workspace/robot.urdf | head -40

# openrua op 24
cat > /workspace/fkcheck.py <<'EOF'
import rclpy, numpy as np
from rob import Robot, ARM
from moveit_msgs.srv import GetPositionFK
r=Robot("fkc")
q=r.arm_q()
req=GetPositionFK.Request(); req.header.frame_id=""
req.fk_link_names=["panda_link0","panda_link1","panda_hand","panda_leftfinger"]
req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=list(map(float,q))
fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
res=fut.result()
for n,ps in zip(res.fk_link_names,res.pose_stamped):
    p=ps.pose.position; o=ps.pose.orientation
    print(n, "frame=",repr(ps.header.frame_id), "pos=",round(p.x,4),round(p.y,4),round(p.z,4), "q=",round(o.x,4),round(o.y,4),round(o.z,4),round(o.w,4))
EOF
timeout 120 python3 /workspace/fkcheck.py

# openrua op 25
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE_T = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation", "new_string": "# Verified via /compute_fk: MoveIt\'s model frame here IS `world` (panda_link0\\n# is reported at (-0.66,0,0.912)), so poses need no base offset.\\nBASE_T = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 26
cat > /workspace/iktest.py <<'EOF'
import numpy as np
from rob import Robot, rot_from_axes, R_to_quat
r=Robot("ikt")
q=r.arm_q(); pos,quat,R=r.fk_hand(q)
print("hand", np.round(pos,4), np.round(quat,4))
sol=r.ik_hand(pos,quat)
print("ik roundtrip:", None if sol is None else np.round(sol,4)); print("current:", np.round(q,4))
# test IK for a top-down grasp pose above the bottle: tcp at (-0.14,0.049,1.05), z down, fingers along world y
Rg=rot_from_axes([0,0,-1],[1,0,0])
print("Rg\n",np.round(Rg,3), R_to_quat(Rg))
for z in (1.20,1.10,1.00):
    s=r.ik_tcp([-0.14,0.049,z],Rg)
    print("tcp z",z,"->", None if s is None else np.round(s,3))
    if s is not None:
        p,_,_=r.tcp(s); print("   fk tcp check", np.round(p,4))
EOF
timeout 300 python3 /workspace/iktest.py

# openrua op 27
cat > /workspace/iktest2.py <<'EOF'
import numpy as np
from rob import Robot, rot_from_axes, R_to_quat
r=Robot("ikt2")
Rg=rot_from_axes([0,0,-1],[1,0,0])
s=r.ik_tcp([-0.14,0.049,1.10],Rg)
p,quat,R=r.tcp(s)
print("sol",np.round(s,3)); print("tcp",np.round(p,4),"quat",np.round(quat,4)); print("R\n",np.round(R,3))
EOF
timeout 300 python3 /workspace/iktest2.py

# openrua op 28
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        # the group\'s default tip is panda_link8 (45 deg yaw off the hand)\\n        req.ik_request.ik_link_name = \\"panda_hand\\"\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 29
timeout 300 python3 /workspace/iktest2.py

# openrua op 30
cat > /workspace/step1_grasp.py <<'EOF'
import numpy as np
from rob import Robot, rot_from_axes
r=Robot("grasp")
BX,BY=-0.137,0.049
Rg=rot_from_axes([0,0,-1],[0,1,0])   # z down, fingers close along world x
r.gripper(0.04)
q=r.move_tcp([BX,BY,1.22],Rg,4.0)
q=r.move_tcp([BX,BY,1.00],Rg,3.0,seed=q)
print("wrench before close", np.round(r.wrench()[0],2))
f=r.gripper(0.0)
print("wrench after close", np.round(r.wrench()[0],2))
q=r.move_tcp([BX,BY,1.18],Rg,3.0,seed=q)
print("fingers after lift", np.round(r.finger(),4))
print("wrench after lift", np.round(r.wrench()[0],2))
EOF
timeout 900 python3 -u /workspace/step1_grasp.py 2>&1 | tee step1.log

# openrua op 31
for c in agentview robot0_robotview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null && cat > /workspace/handcheck.py <<'EOF'
import numpy as np
d=np.load("/workspace/birdview_depth.npy"); fx=579.4112549695428; cx=320; cy=240
H,W=d.shape; uu,vv=np.meshgrid(np.arange(W),np.arange(H))
wx=(vv-cy)*d/fx-0.2; wy=(uu-cx)*d/fx; wz=3.0-d
# region around the bottle/hand xy (-0.137,0.049): +-0.12
m=(np.abs(wx+0.137)<0.12)&(np.abs(wy-0.049)<0.12)
for lo in np.arange(0.90,1.5,0.02):
    mm=m&(wz>=lo)&(wz<lo+0.02)
    if mm.sum()>3: print(f"z[{lo:.2f}] n={mm.sum():4d} x[{wx[mm].min():.3f},{wx[mm].max():.3f}] y[{wy[mm].min():.3f},{wy[mm].max():.3f}]")
from rob import Robot
r=Robot("hc"); p,q,R=r.tcp(); ph,_,_=r.fk_hand()
print("reported tcp",np.round(p,4),"hand",np.round(ph,4))
EOF
timeout 120 python3 /workspace/handcheck.py

# openrua op 33
timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png >/dev/null; python3 - <<'EOF'
import numpy as np
d=np.load("/workspace/agentview_depth.npy")
fx=579.4112549695428; cx=320; cy=240
qx,qy,qz,qw=0.6380,0.6380,-0.3048,-0.3048; t=np.array([0.6586,0.0,1.6104])
x,y,z,w=qx,qy,qz,qw
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
H,W=d.shape; uu,vv=np.meshgrid(np.arange(W),np.arange(H))
P=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fx,d],-1); Wd=P@R.T+t
wx,wy,wz=Wd[...,0],Wd[...,1],Wd[...,2]
# bottle: dark pixels region u 330-365, v 130-200 ; take those with x in [-0.2,-0.08], y within 0.06 of 0.049
m=np.zeros(wz.shape,bool); m[120:210,325:370]=True
m&=(wx>-0.2)&(wx<-0.08)&(np.abs(wy-0.049)<0.05)
print("bottle-ish pixels z range", wz[m].min().round(3), wz[m].max().round(3))
for lo in np.arange(0.95,1.25,0.02):
    mm=m&(wz>=lo)&(wz<lo+0.02)
    if mm.sum()>3: print(f"z[{lo:.2f}] n={mm.sum():4d} x[{wx[mm].min():.3f},{wx[mm].max():.3f}] y[{wy[mm].min():.3f},{wy[mm].max():.3f}] width={wy[mm].max()-wy[mm].min():.3f}")
EOF

# openrua op 34
cat > /workspace/clockcheck.py <<'EOF'
import rclpy, time, numpy as np
from rosgraph_msgs.msg import Clock
from rob import Robot
r=Robot("cc")
clk=[]
r.node.create_subscription(Clock, "/clock", lambda m: clk.append(m.clock.sec+m.clock.nanosec*1e-9), 10)
def now():
    clk.clear(); t=time.time()
    while not clk and time.time()-t<3: r.spin(0.1)
    return clk[-1] if clk else None
print("clock:", now()); time.sleep(3); print("clock after 3s wall idle:", now())
# spin (listen) for 3 s
t=time.time()
while time.time()-t<3: r.spin(0.1)
print("clock after 3s spinning:", now())
EOF
timeout 60 python3 /workspace/clockcheck.py

# openrua op 35
cat > /workspace/lagtest.py <<'EOF'
import rclpy, time, numpy as np
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from rob import Robot, ARM, rot_from_axes
r=Robot("lag")
clk=[0.0]
r.node.create_subscription(Clock, "/clock", lambda m: clk.__setitem__(0, m.clock.sec+m.clock.nanosec*1e-9), 10)
log=[]
target=[None]
def on_js(m):
    d=dict(zip(m.name,m.position))
    q=np.array([d[j] for j in ARM])
    err = None if target[0] is None else float(np.abs(q-target[0]).max())
    log.append((time.time(), clk[0], err, d["panda_finger_joint1"]))
r.node.create_subscription(JointState,"/joint_states",on_js,10)
# small move: raise TCP by 3 cm at the same orientation
p,quat,R=r.tcp()
q=r.ik_tcp(p+[0,0,0.03],R)
target[0]=np.array(q)
t0=time.time()
code,err=r.move_q(q,2.0)
t1=time.time()
print("result after", round(t1-t0,2),"s wall; code",code,"err",err)
# keep listening 4 s
while time.time()-t1<4: r.spin(0.1)
for (tw,tc,e,f) in log[::max(1,len(log)//40)]:
    print(f"wall+{tw-t0:6.2f} clock={tc:7.3f} err={e if e is None else round(e,4)} finger={f:.4f}")
print("n msgs",len(log),"final clock",clk[0])
EOF
timeout 300 python3 -u /workspace/lagtest.py

# openrua op 36
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def move_q(self, q, seconds=3.0, via=None):\\n        \\"\\"\\"One trajectory to q (optionally through intermediate points `via`).\\"\\"\\"\\n        goal = FollowJointTrajectory.Goal()", "new_string": "    def move_q(self, q, seconds=None, via=None, retries=2, max_rate=0.2):\\n        \\"\\"\\"One trajectory to q (optionally through intermediate points `via`).\\n        Duration defaults to travel/max_rate (rad/s); re-sends on -5/off-target\\n        (the controller lags long goals; resending converges).\\"\\"\\"\\n        if seconds is None:\\n            dq = np.abs(np.array(q) - np.array(self.arm_q())).max()\\n            seconds = max(2.0, dq / max_rate)\\n        for attempt in range(retries + 1):\\n            code, err = self._send_traj(q, seconds, via if attempt == 0 else None)\\n            if code == 0 and err < 0.02:\\n                return code, err\\n            print(f\\"[move_q] retrying ({attempt+1}/{retries})\\", flush=True)\\n            seconds = max(2.0, err / max_rate * 1.5)\\n        return code, err\\n\\n    def _send_traj(self, q, seconds, via=None):\\n        goal = FollowJointTrajectory.Goal()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 37
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def move_tcp(self, tcp_w, R, seconds=3.0, seed=None):\\n        q = self.ik_tcp(tcp_w, R, seed)\\n        if q is None:\\n            print(f\\"[move_tcp] IK FAILED for {np.round(tcp_w,3)}\\", flush=True)\\n            return None\\n        code, err = self.move_q(q, seconds)\\n        p, _, _ = self.tcp()\\n        print(f\\"[move_tcp] tcp now {np.round(p,4)} (target {np.round(tcp_w,4)})\\", flush=True)\\n        return q", "new_string": "    def move_tcp(self, tcp_w, R, seconds=None, seed=None):\\n        q = self.ik_tcp(tcp_w, R, seed)\\n        if q is None:\\n            print(f\\"[move_tcp] IK FAILED for {np.round(tcp_w,3)}\\", flush=True)\\n            return None\\n        code, err = self.move_q(q, seconds)\\n        p, _, Rn = self.tcp()\\n        print(f\\"[move_tcp] tcp now {np.round(p,4)} z-axis {np.round(Rn[:,2],3)} (target {np.round(tcp_w,4)})\\", flush=True)\\n        return q", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 38
cat > /workspace/step2_place.py <<'EOF'
import numpy as np
from rob import Robot, rot_from_axes
r=Robot("place")
print("fingers", np.round(r.finger(),4))
p,quat,R0=r.tcp(); print("tcp start", np.round(p,4))
A=np.array([-0.107,0.095]); C=np.array([0.099,0.254])
dh=(C-A)/np.linalg.norm(C-A); print("diag dir", np.round(dh,3), "len", np.linalg.norm(C-A))
al=np.deg2rad(25)
d=np.array([np.cos(al)*dh[0], np.cos(al)*dh[1], -np.sin(al)])
hy=np.array([-dh[1], dh[0], 0.0])
Rp=rot_from_axes(d, np.cross(hy,d))
print("Rp\n", np.round(Rp,3))
bottom=np.array([0.062,0.228,0.949]); L=0.20
tcp_final=bottom-L*d; print("tcp_final", np.round(tcp_final,4))
# 1 lift
q=r.move_tcp([-0.137,0.049,1.30],R0)
# 2 rotate high
q=r.move_tcp([-0.10,0.06,1.30],Rp,seed=q)
if q is None: raise SystemExit("rotate IK failed")
# 3 above
q=r.move_tcp(tcp_final+[0,0,0.07],Rp,seed=q)
if q is None: raise SystemExit("above IK failed")
print("wrench", np.round(r.wrench()[0],2))
# 4 descend
q=r.move_tcp(tcp_final,Rp,seed=q)
if q is None: raise SystemExit("final IK failed")
print("wrench at bottom", np.round(r.wrench()[0],2), "fingers", np.round(r.finger(),4))
EOF
timeout 1500 python3 -u /workspace/step2_place.py 2>&1 | tee step2.log

# openrua op 39
python3 tools/perception/cam_snap.py agentview pre_release_agent.png >/dev/null 2>&1; python3 tools/perception/cam_snap.py frontview pre_release_front.png >/dev/null 2>&1; ls -la pre_release_*

# openrua op 40
cat > /workspace/step3_release.py <<'EOF'
import numpy as np
from rob import Robot
r=Robot("rel")
r.gripper(0.04)
print("wrench", np.round(r.wrench()[0],2))
q=r.arm_q()
p,_,R=r.tcp(); print("tcp", np.round(p,4))
# retreat straight back along -hand z (away from bottle) then up
q2=r.move_tcp(p-0.08*R[:,2]+[0,0,0.05],R,seed=q)
q3=r.move_tcp([-0.15,0.05,1.30],R,seed=q2)
EOF
timeout 900 python3 -u /workspace/step3_release.py 2>&1 | tee step3.log
for c in agentview frontview birdview robot0_robotview; do python3 tools/perception/cam_snap.py $c post_release_$c.png >/dev/null 2>&1; done; ls post_release_*

# openrua op 41
python3 tools/perception/cam_snap.py birdview/depth pr_bird_depth.png >/dev/null 2>&1; ls pr_bird* ; cat > chk_bottle.py <<'EOF'
import numpy as np
d=np.load("pr_bird_depth.npy") if __import__("os").path.exists("pr_bird_depth.npy") else np.load("pr_bird_depth_depth.npy")
fx=579.4112549695428; cx=320; cy=240
v,u=np.mgrid[0:480,0:640]
X=(u-cx)*d/fx; Y=(v-cy)*d/fx
wx=Y-0.2; wy=X; wz=3.0-d
# drawer region
m=(wx>-0.13)&(wx<0.13)&(wy>0.07)&(wy<0.27)
# points above floor 0.924 and below 0.99 are bottle or walls
sel=m&(wz>0.93)&(wz<1.0)
pts=np.stack([wx[sel],wy[sel],wz[sel]],1)
# exclude walls: interior x -0.107..0.098, y 0.095..0.254 with margin
inside=(pts[:,0]>-0.10)&(pts[:,0]<0.092)&(pts[:,1]>0.10)&(pts[:,1]<0.248)
b=pts[inside]
print("bottle pts",len(b))
print("x range",b[:,0].min(),b[:,0].max(),"y range",b[:,1].min(),b[:,1].max(),"z max",b[:,2].max())
# profile along diagonal
A=np.array([-0.107,0.095]); C=np.array([0.099,0.254]); dh=(C-A)/np.linalg.norm(C-A)
s=(b[:,:2]-A)@dh
for lo in np.arange(0,0.27,0.02):
    k=(s>=lo)&(s<lo+0.02)
    if k.sum(): print(f"s {lo:.2f}: n={k.sum()} zmax={b[k,2].max():.3f} zmed={np.median(b[k,2]):.3f}")
# anything higher than walls in the region (bottle sticking above 0.984)?
hi=m&(wz>0.99)&(wz<1.12)
print("pts above 0.99 in drawer region:",hi.sum())
if hi.sum(): print(" x",wx[hi].min(),wx[hi].max()," y",wy[hi].min(),wy[hi].max()," z",wz[hi].max())
EOF
python3 chk_bottle.py

# openrua op 42
grep -n "argv\|depth\|npy" tools/perception/cam_snap.py | head -20; ls *.npy | head

# openrua op 43
sed -n 17,28p tools/perception/cam_snap.py

# openrua op 44
timeout 120 python3 tools/perception/cam_snap.py /birdview/depth/image_raw pr_bird_depth.png 2>&1 | tail -1; sed -i 's|^d=np.load.*|d=np.load("pr_bird_depth.npy")|' chk_bottle.py; python3 chk_bottle.py

# openrua op 45
cat > chk_bottle.py <<'EOF'
import numpy as np, sys
d=np.load(sys.argv[1] if len(sys.argv)>1 else "pr_bird_depth.npy")
fx=579.4112549695428; cx=320; cy=240
v,u=np.mgrid[0:480,0:640]
X=(u-cx)*d/fx; Y=(v-cy)*d/fx
wx=Y-0.2; wy=X; wz=3.0-d
m=(wx>-0.13)&(wx<0.13)&(wy>0.05)&(wy<0.27)
sel=m&(wz>0.93)&(wz<1.0)
pts=np.stack([wx[sel],wy[sel],wz[sel]],1)
inside=(pts[:,0]>-0.10)&(pts[:,0]<0.092)&(pts[:,1]>0.10)&(pts[:,1]<0.248)
b=pts[inside]
print("bottle pts",len(b))
print("x range",b[:,0].min(),b[:,0].max(),"y range",b[:,1].min(),b[:,1].max(),"z max",b[:,2].max())
A=np.array([-0.107,0.095]); C=np.array([0.099,0.254]); dh=(C-A)/np.linalg.norm(C-A)
s=(b[:,:2]-A)@dh
for lo in np.arange(0,0.27,0.02):
    k=(s>=lo)&(s<lo+0.02)
    if k.sum(): print(f"s {lo:.2f}: n={k.sum()} zmax={b[k,2].max():.3f} zmed={np.median(b[k,2]):.3f}")
hi=m&(wz>0.99)&(wz<1.12)
print("pts above 0.99 in drawer region:",hi.sum())
if hi.sum(): print(" x",wx[hi].min(),wx[hi].max()," y",wy[hi].min(),wy[hi].max()," z",wz[hi].max())
# drawer front panel location: top of panel z~0.984 at y ~0.087-0.095
pan=(wx>-0.08)&(wx<0.08)&(wy>0.0)&(wy<0.12)&(wz>0.975)&(wz<0.995)
print("front panel y range", wy[pan].min() if pan.sum() else None, wy[pan].max() if pan.sum() else None)
EOF
python3 chk_bottle.py

# openrua op 46
cat > /workspace/step4_push.py <<'EOF'
import numpy as np
from rob import Robot, rot_from_axes
r=Robot("push")
r.gripper(0.0)
al=np.deg2rad(40)
z=np.array([0,np.cos(al),-np.sin(al)]); hy=np.array([1.0,0,0])
Rp=rot_from_axes(z, np.cross(hy,z)); print("Rp\n",np.round(Rp,3))
X=0.06; Z=0.955
q=r.move_tcp([X,-0.02,1.15],Rp)
if q is None: raise SystemExit("ik1")
q=r.move_tcp([X,0.02,Z],Rp,seed=q)
if q is None: raise SystemExit("ik2")
print("wrench pre", np.round(r.wrench()[0],2))
for y in [0.07,0.11,0.15,0.19,0.23,0.245]:
    q=r.move_tcp([X,y,Z],Rp,seed=q)
    if q is None: raise SystemExit(f"ik y={y}")
    print("wrench", np.round(r.wrench()[0],2), flush=True)
# retreat
p,_,R=r.tcp()
q=r.move_tcp(p-0.10*R[:,2],R,seed=q)
q=r.move_tcp([X,-0.05,1.20],R,seed=q)
EOF
timeout 1700 python3 -u /workspace/step4_push.py 2>&1 | tee step4.log

# openrua op 47
cat > /workspace/state.py <<'EOF'
import numpy as np
from rob import Robot
r=Robot("st")
q=r.arm_q(); print("q", np.round(q,3))
p,_,R=r.tcp(); print("tcp", np.round(p,4), "z", np.round(R[:,2],3), "y", np.round(R[:,1],3))
print("fingers", np.round(r.finger(),4)); print("wrench", np.round(r.wrench()[0],2))
EOF
timeout 120 python3 state.py 2>&1 | tail -4; timeout 60 python3 tools/perception/cam_snap.py agentview crash_agent.png >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py frontview crash_front.png > /dev/null 2>&1; echo done

# openrua op 48
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def move_tcp(self, tcp_w, R, seconds=None, seed=None):\\n        q = self.ik_tcp(tcp_w, R, seed)\\n        if q is None:\\n            print(f\\"[move_tcp] IK FAILED for {np.round(tcp_w,3)}\\", flush=True)\\n            return None", "new_string": "    def ik_tcp_near(self, tcp_w, R, seed=None, max_jump=1.0, tries=6):\\n        \\"\\"\\"IK solution whose joints stay within max_jump rad of seed (the IK\\n        service happily returns far-away branches the controller then cannot\\n        track and which sweep the hand through the table).\\"\\"\\"\\n        seed = self.arm_q() if seed is None else list(seed)\\n        best = None\\n        for i in range(tries):\\n            s = seed if i == 0 else list(np.array(seed) + np.random.uniform(-0.15, 0.15, len(seed)))\\n            q = self.ik_tcp(tcp_w, R, s, timeout=5.0)\\n            if q is None:\\n                continue\\n            jump = np.abs(np.array(q) - np.array(seed)).max()\\n            if best is None or jump < best[0]:\\n                best = (jump, q)\\n            if jump <= max_jump:\\n                return q\\n        if best is not None:\\n            print(f\\"[ik_tcp_near] closest branch still jumps {best[0]:.2f} rad\\", flush=True)\\n        return None\\n\\n    def move_tcp(self, tcp_w, R, seconds=None, seed=None, max_jump=1.0):\\n        q = self.ik_tcp_near(tcp_w, R, seed, max_jump)\\n        if q is None:\\n            print(f\\"[move_tcp] IK FAILED (or too far) for {np.round(tcp_w,3)}\\", flush=True)\\n            return None", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 49
cat > /workspace/recover.py <<'EOF'
import numpy as np
from rob import Robot
r=Robot("rec")
p,_,R=r.tcp(); print("tcp", np.round(p,4))
q=r.move_tcp(p+[0,0,0.12],R,max_jump=0.6)
print("wrench", np.round(r.wrench()[0],2))
EOF
timeout 600 python3 -u recover.py 2>&1 | tail -6

# openrua op 50
python3 -c "import scipy; print(scipy.__version__)" 2>&1; cat > /workspace/step4b_push.py <<'EOF'
import numpy as np
from rob import Robot, rot_from_axes, R_to_quat, quat_to_R
def slerp(q0,q1,t):
    q0=np.asarray(q0,float); q1=np.asarray(q1,float)
    if q0@q1<0: q1=-q1
    d=np.clip(q0@q1,-1,1); th=np.arccos(d)
    if th<1e-6: return q0
    return (np.sin((1-t)*th)*q0+np.sin(t*th)*q1)/np.sin(th)
r=Robot("push2")
al=np.deg2rad(40)
z=np.array([0,np.cos(al),-np.sin(al)]); hy=np.array([1.0,0,0])
Rp=rot_from_axes(z, np.cross(hy,z))
p,_,R0=r.tcp(); print("tcp", np.round(p,4))
q=r.arm_q()
# 1 go high
q=r.move_tcp([p[0],p[1],1.35],R0,seed=q,max_jump=0.8)
if q is None: raise SystemExit("ik high")
# 2 rotate in steps while translating toward (0.0,-0.05,1.35)
q0=R_to_quat(R0); q1=R_to_quat(Rp)
start=np.array([p[0],p[1],1.35]); goal=np.array([0.0,-0.05,1.35])
for t in [0.25,0.5,0.75,1.0]:
    Rt=quat_to_R(slerp(q0,q1,t)); pt=start+(goal-start)*t
    q=r.move_tcp(pt,Rt,seed=q,max_jump=0.9)
    if q is None: raise SystemExit(f"ik rot t={t}")
# 3 approach
X=0.06; Z=0.955
for pt in [[X,-0.02,1.15],[X,0.02,1.02],[X,0.02,Z]]:
    q=r.move_tcp(pt,Rp,seed=q,max_jump=0.9)
    if q is None: raise SystemExit(f"ik approach {pt}")
print("wrench pre", np.round(r.wrench()[0],2))
for y in [0.07,0.11,0.15,0.19,0.23,0.245]:
    q=r.move_tcp([X,y,Z],Rp,seed=q,max_jump=0.6)
    if q is None: raise SystemExit(f"ik y={y}")
    print("wrench", np.round(r.wrench()[0],2), flush=True)
p,_,R=r.tcp()
q=r.move_tcp(p-0.10*R[:,2],R,seed=q,max_jump=0.6)
q=r.move_tcp([X,-0.05,1.20],R,seed=q,max_jump=0.9)
EOF
timeout 1750 python3 -u /workspace/step4b_push.py 2>&1 | grep -v "^\[move_q\] code=0" | tee step4b.log

# openrua op 51
for c in agentview frontview robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c final_$c.png >/dev/null 2>&1; done; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw final_bird_depth.png >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw final_agent_depth.png >/dev/null 2>&1; ls final_*

# openrua op 52
cat > /workspace/chk_final.py <<'EOF'
import numpy as np
d=np.load("final_bird_depth.npy")
fx=579.4112549695428; cx=320; cy=240
v,u=np.mgrid[0:480,0:640]
X=(u-cx)*d/fx; Y=(v-cy)*d/fx
wx=Y-0.2; wy=X; wz=3.0-d
reg=(wx>-0.12)&(wx<0.12)&(wy>0.0)&(wy<0.40)
# anything between table and cabinet top in front of the cabinet face?
for lo,hi in [(0.905,0.93),(0.93,0.99),(0.99,1.11)]:
    k=reg&(wz>lo)&(wz<hi)
    print(f"z {lo}-{hi}: n={k.sum()}", "y range", (round(wy[k].min(),3), round(wy[k].max(),3)) if k.sum() else None)
k=reg&(wz>0.93)&(wz<0.99)
if k.sum():
    for ylo in np.arange(0.18,0.26,0.01):
        kk=k&(wy>=ylo)&(wy<ylo+0.01)
        if kk.sum(): print(f"  y {ylo:.2f}: n={kk.sum()} z {wz[kk].min():.3f}-{wz[kk].max():.3f} x {wx[kk].min():.3f}..{wx[kk].max():.3f}")
# cabinet top edge (front) y
top=reg&(wz>1.11)&(wz<1.14)
print("cabinet top y min", round(wy[top].min(),4))
# bottle anywhere on the table outside cabinet? table region z 0.91-1.0 excluding cabinet footprint and bowl
tab=(wx>-0.6)&(wx<0.4)&(wy>-0.5)&(wy<0.5)&(wz>0.915)&(wz<1.0)&~((wy>0.19)&(wx>-0.13)&(wx<0.13))
print("objects on table (z .915-1.0) n=",tab.sum(), "x",np.round([wx[tab].min(),wx[tab].max()],3),"y",np.round([wy[tab].min(),wy[tab].max()],3))
EOF
python3 chk_final.py

# openrua op 53
cat >> /workspace/chk_final.py <<'EOF'
print("--- points in front of cabinet, y 0..0.19, z 0.905-1.0")
k=(wx>-0.2)&(wx<0.2)&(wy>0.0)&(wy<0.19)&(wz>0.905)&(wz<1.0)
for ylo in np.arange(0.0,0.19,0.02):
    kk=k&(wy>=ylo)&(wy<ylo+0.02)
    if kk.sum(): print(f"  y {ylo:.2f}: n={kk.sum()} z {wz[kk].min():.3f}-{wz[kk].max():.3f} x {wx[kk].min():.3f}..{wx[kk].max():.3f}")
EOF
python3 chk_final.py | sed -n '/---/,$p'
