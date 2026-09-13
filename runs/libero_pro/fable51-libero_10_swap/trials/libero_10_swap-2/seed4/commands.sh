#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/README.md && cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; ros2 node list

# openrua op 3
ros2 topic echo /joint_states --once; ros2 topic echo /pipeline_state --once 2>&1 | head -30; ros2 node info /robot_bridge 2>&1 | head -60

# openrua op 4
cat /tmp/claude-1000/-workspace/482ac223-d5ee-4255-bc73-554929ae0167/tasks/bsu4c0pmi.output

# openrua op 5
for c in agentview frontview sideview birdview robot0_robotview; do timeout 40 python3 tools/perception/cam_snap.py $c; done; ros2 topic info /pipeline_state; ros2 node info /robot_bridge 2>&1 | head -60

# openrua op 6
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | sort -u; echo ---; timeout 30 python3 - <<'EOF'
import rclpy
from tf2_msgs.msg import TFMessage
rclpy.init(); n=rclpy.create_node("tfl")
frames=set()
def cb(m):
    for t in m.transforms: frames.add((t.header.frame_id,t.child_frame_id))
n.create_subscription(TFMessage,"/tf",cb,50)
import time
end=time.time()+8
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.2)
for f in sorted(frames): print(f)
EOF

# openrua op 7
timeout 60 python3 - <<'EOF'
import rclpy, time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node("tfq")
b=Buffer(); TransformListener(b,n)
end=time.time()+8
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.2)
for f in ["panda_link0","birdview_optical_frame","agentview_optical_frame","robot0_eye_in_hand_optical_frame"]:
    t=b.lookup_transform("world",f,rclpy.time.Time())
    tr=t.transform.translation; q=t.transform.rotation
    print(f, round(tr.x,4),round(tr.y,4),round(tr.z,4), "q", round(q.x,4),round(q.y,4),round(q.z,4),round(q.w,4))
EOF
ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:"

# openrua op 8
timeout 120 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 60 python3 - <<'EOF'
import numpy as np
d=np.load("birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
cam=np.array([-0.2,0,3.0])
R=np.array([[0,1,0],[1,0,0],[0,0,-1]])
def w(u,v):
    z=d[v,u]; p=np.array([(u-cx)*z/fx,(v-cy)*z/fy,z]); return cam+R@p
for name,(u,v) in {"moka":(243,283),"stove_center":(378,280),"knob":(378,237),"pan":(322,310),"table":(200,380),"pan_handle":(365,318)}.items():
    print(name,(u,v),np.round(w(u,v),4))
# table height: median of a region
print("table z stats", np.round(w(150,350),3), np.round(w(450,400),3))
EOF

# openrua op 9
timeout 60 python3 - <<'EOF'
import numpy as np
d=np.load("birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
cam=np.array([-0.2,0,3.0]); R=np.array([[0,1,0],[1,0,0],[0,0,-1]])
H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
z=d
X=cam[0]+ (vv-cy)*z/fy
Y=cam[1]+ (uu-cx)*z/fx
Z=cam[2]-z
def region(name,umin,umax,vmin,vmax,zmin=0.905):
    m=np.zeros_like(d,bool); m[vmin:vmax,umin:umax]=True
    m&=Z>zmin
    xs,ys,zs=X[m],Y[m],Z[m]
    print(f"{name}: n={m.sum()} x[{xs.min():.3f},{xs.max():.3f}] y[{ys.min():.3f},{ys.max():.3f}] zmax={zs.max():.3f} centroid=({xs.mean():.3f},{ys.mean():.3f})")
    # top-most part
    top=m&(Z>zs.max()-0.01)
    print(f"   top: x[{X[top].min():.3f},{X[top].max():.3f}] y[{Y[top].min():.3f},{Y[top].max():.3f}] c=({X[top].mean():.3f},{Y[top].mean():.3f})")
region("moka",210,275,255,310)
region("stove",345,415,250,315)
region("knob",355,400,220,255)
region("pan",285,375,275,345)
EOF

# openrua op 10
timeout 60 python3 - <<'EOF'
import cv2
im=cv2.imread("birdview.png"); crop=im[200:340,200:430]; cv2.imwrite("bird_crop.png",cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread("agentview.png"); crop=im[190:360,240:530]; cv2.imwrite("agent_crop.png",cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 11
timeout 60 python3 - <<'EOF'
import numpy as np
d=np.load("birdview_depth.npy")
fx=fy=579.4112549695428; cx=320; cy=240
cam=np.array([-0.2,0,3.0])
H,W=d.shape
vv,uu=np.mgrid[0:H,0:W]
X=cam[0]+(vv-cy)*d/fy; Y=cam[1]+(uu-cx)*d/fx; Z=cam[2]-d
# moka pot: region
m=np.zeros_like(d,bool); m[255:310,210:275]=True; m&=Z>0.905
for lo in np.arange(0.90,1.06,0.01):
    s=m&(Z>=lo)&(Z<lo+0.01)
    if s.sum(): print(f"moka z {lo:.2f}-{lo+.01:.2f}: n={s.sum()} x[{X[s].min():.3f},{X[s].max():.3f}] y[{Y[s].min():.3f},{Y[s].max():.3f}]")
print()
# knob
m=np.zeros_like(d,bool); m[222:256,360:395]=True; m&=(Z>0.905)&(Z<1.1)
print("knob overall", m.sum(), f"x[{X[m].min():.3f},{X[m].max():.3f}] y[{Y[m].min():.3f},{Y[m].max():.3f}] z[{Z[m].min():.3f},{Z[m].max():.3f}] c=({X[m].mean():.4f},{Y[m].mean():.4f})")
for lo in np.arange(0.90,0.98,0.005):
    s=m&(Z>=lo)&(Z<lo+0.005)
    if s.sum(): print(f"knob z {lo:.3f}: n={s.sum()} x[{X[s].min():.3f},{X[s].max():.3f}] y[{Y[s].min():.3f},{Y[s].max():.3f}]")
print()
# stove plate and burner heights
m=np.zeros_like(d,bool); m[250:315,345:415]=True; m&=(Z>0.905)&(Z<1.0)&(Y>0.2)  # avoid pan handle
print("stove", f"x[{X[m].min():.3f},{X[m].max():.3f}] y[{Y[m].min():.3f},{Y[m].max():.3f}] z[{Z[m].min():.3f},{Z[m].max():.3f}]")
for lo in np.arange(0.90,0.96,0.005):
    s=m&(Z>=lo)&(Z<lo+0.005)
    if s.sum(): print(f"stove z {lo:.3f}: n={s.sum()} x[{X[s].min():.3f},{X[s].max():.3f}] y[{Y[s].min():.3f},{Y[s].max():.3f}]")
EOF

# openrua op 12
timeout 120 python3 tools/perception/cam_snap.py /frontview/depth/image_raw frontview_depth.png; timeout 120 python3 tools/perception/cam_snap.py /sideview/depth/image_raw sideview_depth.png; timeout 60 python3 - <<'EOF'
import rclpy, time, numpy as np
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import CameraInfo
rclpy.init(); n=rclpy.create_node("tfq")
b=Buffer(); TransformListener(b,n)
infos={}
for c in ["frontview","sideview"]:
    n.create_subscription(CameraInfo,f"/{c}/color/camera_info",lambda m,c=c: infos.setdefault(c,m),1)
end=time.time()+8
while time.time()<end: rclpy.spin_once(n,timeout_sec=0.2)
for c in ["frontview","sideview"]:
    t=b.lookup_transform("world",f"{c}_optical_frame",rclpy.time.Time())
    tr=t.transform.translation; q=t.transform.rotation
    print(c, [tr.x,tr.y,tr.z], [q.x,q.y,q.z,q.w], list(infos[c].k))
EOF

# openrua op 13
timeout 60 python3 - <<'EOF'
import numpy as np
def R_of(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
fx=fy=579.4112549695428; cx=320; cy=240
def cloud(name,pos,q):
    d=np.load(f"{name}_depth.npy"); H,W=d.shape
    vv,uu=np.mgrid[0:H,0:W]
    P=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1).reshape(-1,3)
    Wp=(R_of(q)@P.T).T+np.array(pos)
    return Wp.reshape(H,W,3)
F=cloud("frontview",[1.0,0.0,1.48],[0.5608418947374366,0.5608418947374367,-0.4306464548876746,-0.4306464548876746])
S=cloud("sideview",[-0.05651774593317116,1.2761224129427358,1.4879572214102434],[0.009905065491771755,0.8064180940013643,-0.591222835289388,-0.0068779631569095825])
for nm,C in [("front",F),("side",S)]:
    X,Y,Z=C[...,0],C[...,1],C[...,2]
    m=(X>-0.15)&(X<0.0)&(Y>-0.40)&(Y<-0.15)&(Z>0.902)&(Z<1.1)
    print(nm,"moka pts",m.sum())
    for lo in np.arange(0.90,1.07,0.01):
        s=m&(Z>=lo)&(Z<lo+0.01)
        if s.sum()>3: print(f"  z {lo:.2f}: n={s.sum():4d} x[{X[s].min():.3f},{X[s].max():.3f}] y[{Y[s].min():.3f},{Y[s].max():.3f}]")
EOF

# openrua op 14
timeout 60 python3 - <<'EOF'
import cv2
im=cv2.imread("agentview.png"); crop=im[200:340,90:230]; cv2.imwrite("agent_moka.png",cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread("frontview.png"); crop=im[300:400,130:240]; cv2.imwrite("front_moka.png",cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 15
timeout 60 python3 - <<'EOF'
import rclpy, time
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("rd")
got={}
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,"/robot_description",lambda m: got.setdefault("m",m),qos)
end=time.time()+8
while "m" not in got and time.time()<end: rclpy.spin_once(n,timeout_sec=0.2)
open("robot.urdf","w").write(got["m"].data)
print(len(got["m"].data))
EOF
grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A10 'link name="panda_leftfinger"' robot.urdf | head -40; grep -n -A6 'joint name="panda_hand_joint"' robot.urdf; grep -n -A6 'joint name="panda_hand_tcp_joint"' robot.urdf

# openrua op 16
python3 - <<'EOF'
import re
s=open("robot.urdf").read()
for m in re.finditer(r'<joint name="(panda_hand_joint|panda_hand_tcp_joint|panda_finger_joint1|panda_joint7|panda_joint8)".*?</joint>', s, re.S):
    print(m.group(0)[:400]); print()
EOF
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ 2>/dev/null

# openrua op 17
cat > /workspace/rk.py <<'EOF'
"""Shared helpers: FK/IK clients, joint state, quaternion utils (reuse in scripts)."""
import time, math, numpy as np, rclpy, yaml
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState
from geometry_msgs.msg import PoseStamped

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation (no rotation)

def R_from_q(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def q_from_R(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t+1)*2; w = 0.25*s; x = (R[2,1]-R[1,2])/s; y = (R[0,2]-R[2,0])/s; z = (R[1,0]-R[0,1])/s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = math.sqrt(1+R[0,0]-R[1,1]-R[2,2])*2; w = (R[2,1]-R[1,2])/s; x = 0.25*s; y = (R[0,1]+R[1,0])/s; z = (R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = math.sqrt(1+R[1,1]-R[0,0]-R[2,2])*2; w = (R[0,2]-R[2,0])/s; x = (R[0,1]+R[1,0])/s; y = 0.25*s; z = (R[1,2]+R[2,1])/s
    else:
        s = math.sqrt(1+R[2,2]-R[0,0]-R[1,1])*2; w = (R[1,0]-R[0,1])/s; x = (R[0,2]+R[2,0])/s; y = (R[1,2]+R[2,1])/s; z = 0.25*s
    return np.array([x, y, z, w])

class Robot:
    def __init__(self, name="rk"):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def joints(self, fresh=True):
        if fresh: self._js.pop("m", None)
        end = time.time()+15
        while "m" not in self._js and time.time() < end: rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]; d = dict(zip(m.name, m.position))
        return d

    def arm_q(self):
        d = self.joints(); return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints(); return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def _seed(self, q):
        js = JointState(); js.name = list(ARM); js.position = [float(v) for v in q]; return js

    def fk_world(self, q=None):
        """hand pose in WORLD: (pos, quat)"""
        if q is None: q = self.arm_q()
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        f = self.fk.call_async(req); rclpy.spin_until_future_complete(self.node, f, timeout_sec=30)
        r = f.result()
        if r is None or r.error_code.val != 1: raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z]) + BASE, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik_world(self, pos, quat, seed=None, timeout=30):
        """pos in WORLD; returns joint list or None"""
        pos = np.asarray(pos, float) - BASE
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = "panda_arm"; r.pose_stamped.header.frame_id = ""
        r.ik_link_name = "panda_hand"
        p = r.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        r.robot_state.joint_state = self._seed(seed if seed is not None else self.arm_q())
        r.avoid_collisions = False
        r.timeout.sec = 2
        f = self.ik.call_async(req); rclpy.spin_until_future_complete(self.node, f, timeout_sec=timeout)
        res = f.result()
        if res is None: raise RuntimeError("IK timeout")
        if res.error_code.val != 1: return None
        d = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [d[j] for j in ARM]

def hand_quat(approach, closing):
    """world-frame quaternion for hand with z=approach, y=closing"""
    z = np.asarray(approach, float); z /= np.linalg.norm(z)
    y = np.asarray(closing, float); y -= z*np.dot(y, z); y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return q_from_R(np.column_stack([x, y, z]))
EOF
timeout 120 python3 -c "
from rk import *
r=Robot()
q=r.arm_q(); print('q',np.round(q,3))
p,qq=r.fk_world(q); print('hand world',np.round(p,4),np.round(qq,4)); print(np.round(R_from_q(*qq),3))
"

# openrua op 18
grep -o '<link name="world"[^>]*>' robot.urdf; python3 - <<'EOF'
import re
s=open("robot.urdf").read()
for m in re.finditer(r'<joint name="([^"]+)" type="fixed">.*?</joint>', s, re.S):
    if 'panda_link0' in m.group(0) or 'world' in m.group(0): print(m.group(0)[:300])
EOF

# openrua op 19
timeout 120 python3 - <<'EOF'
from rk import *
r=Robot()
q=r.arm_q()
req = GetPositionFK.Request(); req.fk_link_names = ["panda_link0","panda_link1","panda_hand"]
req.robot_state.joint_state = r._seed(q)
f = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, f, timeout_sec=30)
res=f.result()
for ps in res.pose_stamped:
    p=ps.pose; print(ps.header.frame_id, np.round([p.position.x,p.position.y,p.position.z],4))
# also FK with header frame panda_link0
req.header.frame_id="panda_link0"
f = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, f, timeout_sec=30)
res=f.result(); print("err",res.error_code.val)
for ps in res.pose_stamped:
    p=ps.pose; print(ps.header.frame_id, np.round([p.position.x,p.position.y,p.position.z],4))
EOF

# openrua op 20
timeout 120 python3 - <<'EOF'
from rk import *
r=Robot()
q=r.arm_q()
quat=np.array([0.9996,0,-0.0284,0])
for label,pos in [("world",[-0.203,0,1.2696]),("base",[0.457,0,0.3576])]:
    sol=r.ik_world(np.array(pos)+BASE, quat, seed=q)   # ik_world subtracts BASE -> sends pos as-is
    print(label, None if sol is None else np.round(sol,3))
EOF

# openrua op 21
python3 - <<'EOF'
s=open("rk.py").read()
s=s.replace("return np.array([p.position.x, p.position.y, p.position.z]) + BASE,","return np.array([p.position.x, p.position.y, p.position.z]),")
s=s.replace("        pos = np.asarray(pos, float) - BASE\n","        pos = np.asarray(pos, float)  # model frame == world on this machine (verified by FK/IK)\n")
open("rk.py","w").write(s)
EOF
grep -n "model frame\|p.position.z\])" rk.py

# openrua op 22
cat >> /workspace/rk.py <<'EOF'

# ---------------------------------------------------------------- motion
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class Mover(Robot):
    def __init__(self, name="mover"):
        super().__init__(name)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)

    def goto_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        n = len(wps)
        for i, w in enumerate(wps):
            t = seconds*(i+1)/n
            pt = JointTrajectoryPoint(positions=[float(v) for v in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1)*1e9)); pts.append(pt)
        goal.trajectory.points = pts
        f = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        h = f.result()
        rf = h.get_result_async(); rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        qa = np.array(self.arm_q()); err = np.abs(qa - np.array(q)).max()
        print(f"  traj error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def goto_pose(self, pos, quat, seconds=3.0, seed=None, max_delta=None):
        q = self.ik_world(pos, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED for {np.round(pos,3)}", flush=True); return None
        cur = np.array(self.arm_q()); d = np.abs(np.array(q)-cur)
        print(f"  IK ok, joint deltas {np.round(d,2)}", flush=True)
        if max_delta is not None and d.max() > max_delta:
            print("  delta too large, refusing", flush=True); return None
        self.goto_q(q, seconds)
        p, _ = self.fk_world(); print(f"  hand now at {np.round(p,4)}", flush=True)
        return q

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = 30.0
        f = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async(); rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result().result
        print(f"  gripper reached={res.reached_goal} stalled={res.stalled} fingers={np.round(self.fingers(),4)}", flush=True)
        return res

TOPDOWN = np.array([1.0, 0.0, 0.0, 0.0])  # hand x=+X, y=-Y, z=-Z (fingers close along world Y)
EOF
timeout 120 python3 - <<'EOF'
from rk import *
r=Robot()
q=r.arm_q()
# test IK feasibility of knob poses and moka poses (no motion)
for label,pos in [("knob_pre",[-0.205,0.204,1.14]),("knob_grasp",[-0.205,0.204,1.038]),("moka_above",[-0.058,-0.265,1.30])]:
    sol=r.ik_world(pos, TOPDOWN, seed=q)
    print(label, None if sol is None else np.round(sol,3))
    if sol is not None:
        p,qq=r.fk_world(sol); print("   fk",np.round(p,4), np.round(qq,3))
EOF

# openrua op 23
cat > /workspace/knob.py <<'EOF'
from rk import *
m = Mover("knob")
KX, KY = -0.205, 0.204
print("open gripper"); m.gripper(0.04)
print("pre-grasp above knob"); m.goto_pose([KX, KY, 1.14], TOPDOWN, 4.0)
print("descend to ridge"); m.goto_pose([KX, KY, 1.038], TOPDOWN, 2.0, max_delta=0.6)
print("close on ridge"); m.gripper(0.0)
q = m.arm_q(); print("q before turn", np.round(q, 3))
EOF
timeout 900 python3 -u knob.py 2>&1 | tee knob.log

# openrua op 24
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png

# openrua op 25
cat > /workspace/knob2.py <<'EOF'
from rk import *
import sys
m = Mover("knob2")
delta = float(sys.argv[1])
q = m.arm_q(); q2 = list(q); q2[6] = q[6] + delta
print("turn j7", round(q[6],3), "->", round(q2[6],3)); m.goto_q(q2, 3.0)
print("q after", np.round(m.arm_q(),3), "fingers", np.round(m.fingers(),4))
EOF
timeout 900 python3 -u knob2.py -1.57 2>&1 | tee -a knob.log; timeout 120 python3 tools/perception/cam_snap.py birdview bird2.png; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png

# openrua op 26
timeout 60 python3 - <<'EOF'
import cv2
im=cv2.imread("bird2.png"); crop=im[200:340,200:430]; cv2.imwrite("bird2_crop.png",cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 27
cat > /workspace/knob3.py <<'EOF'
from rk import *
m = Mover("knob3")
print("open"); m.gripper(0.04)
p,qq = m.fk_world(); print("hand", np.round(p,4), np.round(qq,3))
print("lift"); m.goto_pose([p[0], p[1], 1.20], qq, 2.5, max_delta=0.8)
EOF
timeout 900 python3 -u knob3.py 2>&1 | tee -a knob.log; timeout 120 python3 tools/perception/cam_snap.py agentview agent2.png

# openrua op 28
cat > /workspace/look.py <<'EOF'
from rk import *
import sys
m = Mover("look")
x,y,z = map(float, sys.argv[1:4]); secs = float(sys.argv[4]) if len(sys.argv)>4 else 4.0
m.goto_pose([x,y,z], TOPDOWN, secs)
EOF
timeout 900 python3 -u look.py -0.058 -0.265 1.32 2>&1 | tee -a knob.log; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_moka.png; timeout 120 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_moka_depth.png; ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:"

# openrua op 29
timeout 120 python3 - <<'EOF'
import numpy as np, rclpy, time
from rk import *
from tf2_ros import Buffer, TransformListener
r=Robot("tfc")
b=Buffer(); TransformListener(b,r.node)
end=time.time()+6
while time.time()<end: rclpy.spin_once(r.node,timeout_sec=0.2)
t=b.lookup_transform("world","robot0_eye_in_hand_optical_frame",rclpy.time.Time())
tr=t.transform.translation; q=t.transform.rotation
cam=np.array([tr.x,tr.y,tr.z]); R=R_from_q(q.x,q.y,q.z,q.w)
print("cam",np.round(cam,4)); print(np.round(R,3))
p,qq=r.fk_world(); print("hand",np.round(p,4))
d=np.load("eih_moka_depth.npy"); H,W=d.shape
fx=fy=312.77408948188935; cx=320; cy=240
vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-cx)*d/fx,(vv-cy)*d/fy,d],-1)
Wp=P@R.T+cam
X,Y,Z=Wp[...,0],Wp[...,1],Wp[...,2]
m=(Z>0.905)&(Z<1.1)&(X>-0.15)&(X<0.05)&(Y>-0.40)&(Y<-0.18)
print("pts",m.sum())
for lo in np.arange(0.90,1.07,0.01):
    s=m&(Z>=lo)&(Z<lo+0.01)
    if s.sum()>3: print(f"  z {lo:.2f}: n={s.sum():4d} x[{X[s].min():.3f},{X[s].max():.3f}] y[{Y[s].min():.3f},{Y[s].max():.3f}] cx={X[s].mean():.3f} cy={Y[s].mean():.3f}")
# lid center: highest points
top=m&(Z>Z[m].max()-0.008)
print("lid knob center", X[top].mean(), Y[top].mean(), Z[m].max())
# rim: z in 1.02-1.04, exclude handle (Y<-0.30) and spout (Y>-0.225)
s=m&(Z>1.015)&(Z<1.045)&(Y>-0.31)&(Y<-0.22)
print("rim x range",X[s].min(),X[s].max(),"y range",Y[s].min(),Y[s].max())
# table height check
tm=(Z>0.85)&(Z<0.905)&(X>-0.2)&(X<0.1)
print("table z median", np.median(Z[tm]))
EOF

# openrua op 30
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ && timeout 60 python3 - <<'EOF'
import numpy as np, struct
def stl_bounds(path):
    data=open(path,'rb').read()
    if data[:5]==b'solid' and b'facet' in data[:300]:
        import re
        v=np.array([list(map(float,m.groups())) for m in re.finditer(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)',data)])
    else:
        n=struct.unpack_from('<I',data,80)[0]
        arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ["hand.stl","finger.stl","link7.stl","link6.stl"]:
    lo,hi=stl_bounds(f"/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}")
    print(f, np.round(lo,4), np.round(hi,4))
EOF
grep -o '<link name="panda_hand">.*\?</link>' robot.urdf | head -c 800; echo; grep -o '<link name="panda_leftfinger">.*\?</link>' robot.urdf | head -c 600

# openrua op 31
find / -name "hand*.stl" -o -name "hand*.dae" -o -name "finger*.stl" -o -name "hand*.obj" 2>/dev/null | head; find / -iname "*moka*" 2>/dev/null | head; find / -path /proc -prune -o -iname "*robocasa*" -print 2>/dev/null | head -3

# openrua op 32
timeout 300 python3 - <<'EOF'
from rk import *
r=Robot("iktest")
C=np.array([-0.055,-0.263])
q0=r.arm_q()
for phi_deg in [45, 35, 55]:
  for psi_deg in [0, 10, 20]:
    phi=np.radians(phi_deg); psi=np.radians(psi_deg)
    d=np.array([np.cos(psi)*np.sin(phi), -np.cos(psi)*np.cos(phi), -np.sin(psi)])
    c=np.array([np.cos(phi), np.sin(phi), 0.0])
    for sign in (+1,-1):
        quat=hand_quat(d, sign*c)
        Rm=R_from_q(*quat); 
        # fingertip 1.5cm past centre, at z 0.965
        tip=np.array([C[0],C[1],0.965])+0.015*d
        hand=tip-0.1034*d
        pre=hand-0.08*d
        sol=r.ik_world(hand, quat, seed=q0)
        sol2=r.ik_world(pre, quat, seed=sol if sol is not None else q0)
        print(f"phi={phi_deg} psi={psi_deg} sign={sign:+d} handx={np.round(Rm[:,0],2)} grasp={'ok' if sol is not None else 'FAIL'} pre={'ok' if sol2 is not None else 'FAIL'}", None if sol is None else np.round(sol,2))
EOF

# openrua op 33
timeout 600 python3 - <<'EOF'
from rk import *
r=Robot("iktest2")
C=np.array([-0.055,-0.263])
LIM=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
links=["panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"]
def fk_links(q):
    req=GetPositionFK.Request(); req.fk_link_names=links; req.robot_state.joint_state=r._seed(q)
    f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30)
    return {ps.header.frame_id if False else n:(np.array([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z])) for n,ps in zip(links,f.result().pose_stamped)}
seeds=[[0,0.3,0,-2.0,0,2.3,0.78],[-0.3,0.5,0,-1.8,0,2.3,0.4],[0,0.8,0,-1.5,0,2.3,0.78],[-0.5,0.6,0.3,-1.9,0.5,2.6,0.0],[0.3,0.6,-0.5,-1.9,-0.5,2.6,1.2],[0,0,0,-1.57,0,1.57,0.78],[-0.4,0.4,0,-2.2,0,2.6,-0.7]]
best=[]
for phi_deg in [45,40,50]:
  for psi_deg in [0,10,20]:
    phi=np.radians(phi_deg); psi=np.radians(psi_deg)
    d=np.array([np.cos(psi)*np.sin(phi), -np.cos(psi)*np.cos(phi), -np.sin(psi)])
    c=np.array([np.cos(phi), np.sin(phi), 0.0])
    tip=np.array([C[0],C[1],0.965])+0.015*d
    hand=tip-0.1034*d
    for sign in (+1,-1):
        quat=hand_quat(d, sign*c)
        for s in seeds:
            sol=r.ik_world(hand, quat, seed=s)
            if sol is None: continue
            sol=np.array(sol)
            margin=np.min(np.minimum(sol-LIM[:,0],LIM[:,1]-sol))
            L=fk_links(sol)
            minz=min(v[2] for v in L.values())
            best.append((margin,phi_deg,psi_deg,sign,np.round(sol,2),round(minz,3),np.round(L["panda_link6"],3),np.round(L["panda_link4"],3)))
best.sort(key=lambda t:-t[0])
seen=set()
for b in best:
    key=(b[1],b[2],b[3],tuple(b[4]))
    if key in seen: continue
    seen.add(key)
    print(f"margin={b[0]:.2f} phi={b[1]} psi={b[2]} sign={b[3]:+d} q={b[4]} minz={b[5]} link6={b[6]} link4={b[7]}")
EOF

# openrua op 34
cat > /workspace/moka_plan.py <<'EOF'
from rk import *
C = np.array([-0.055, -0.263])          # moka pot centre (x,y) world
B = np.array([-0.057, 0.207])           # burner centre
GRASP_Z = 0.965                         # hand z at grasp (pads 0.955-0.975, just above collar)
TIP_PAST = 0.015                        # fingertip past pot centre along approach
PHI = np.radians(45.0)                  # approach yaw: hand comes from (-x,+y) side
D = np.array([np.sin(PHI), -np.cos(PHI), 0.0])   # approach direction (hand z)
CC = np.array([np.cos(PHI), np.sin(PHI), 0.0])   # closing direction
Q_GRASP = hand_quat(D, -CC)             # hand x up (camera on top)
HAND_GRASP = np.array([C[0], C[1], GRASP_Z]) + (TIP_PAST - 0.1034) * D
HAND_PRE = HAND_GRASP - 0.08 * D
SEED_GRASP = [0.53, 1.15, -0.8, -1.94, -2.26, 2.25, -2.05]
# place: approach from +y (phi=0) so the palm stays clear of the knob
PHI_P = 0.0
DP = np.array([np.sin(PHI_P), -np.cos(PHI_P), 0.0]); CP = np.array([np.cos(PHI_P), np.sin(PHI_P), 0.0])
Q_PLACE = hand_quat(DP, -CP)
POT_OFF = 0.1034 - TIP_PAST             # pot centre = hand + POT_OFF * d
def wrench(node_robot, n=3):
    from geometry_msgs.msg import WrenchStamped
    got = []
    sub = node_robot.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", got.append, 5)
    end = time.time() + 5
    while len(got) < n and time.time() < end: rclpy.spin_once(node_robot.node, timeout_sec=0.2)
    node_robot.node.destroy_subscription(sub)
    if not got: return None
    w = got[-1].wrench
    return np.round([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z], 2)
EOF
timeout 300 python3 - <<'EOF'
from moka_plan import *
print("Q_GRASP", np.round(Q_GRASP,4)); print(np.round(R_from_q(*Q_GRASP),3))
print("HAND_GRASP", np.round(HAND_GRASP,4), "HAND_PRE", np.round(HAND_PRE,4))
r=Robot("plan")
qg=r.ik_world(HAND_GRASP,Q_GRASP,seed=SEED_GRASP); print("grasp q",np.round(qg,3))
qp=r.ik_world(HAND_PRE,Q_GRASP,seed=qg); print("pre q",np.round(qp,3))
qh=r.ik_world([HAND_PRE[0],HAND_PRE[1],1.25],Q_GRASP,seed=qp); print("prehigh q",np.round(qh,3))
# place poses
hp=np.array([B[0],B[1],1.0]) - POT_OFF*DP
print("HAND_PLACE",np.round(hp,4))
for z,lab in [(1.13,"place_high"),(1.0,"place")]:
    s=r.ik_world([hp[0],hp[1],z],Q_PLACE,seed=SEED_GRASP); print(lab, None if s is None else np.round(s,3))
    if s is not None:
        req=GetPositionFK.Request(); req.fk_link_names=["panda_link6","panda_link7"]; req.robot_state.joint_state=r._seed(s)
        f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30)
        for ps in f.result().pose_stamped: print("   ",np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],3))
print("wrench now", wrench(r))
EOF

# openrua op 35
timeout 900 python3 - <<'EOF'
from moka_plan import *
r=Robot("plan2")
LIM=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
seeds=[SEED_GRASP,[0,0.3,0,-2.0,0,2.3,0.78],[-0.3,0.5,0,-1.8,0,2.3,0.4],[0.3,0.8,-0.3,-1.5,-0.5,2.3,0.78],[0.5,1.0,-0.5,-1.5,-1.5,2.5,0.0],[0.5,0.6,-0.5,-1.9,-0.5,2.6,1.2],[0.8,1.2,-1.0,-1.7,-2.2,2.2,-2.0],[1.0,1.0,-0.8,-1.6,-2.5,2.0,-1.5],[0.6,1.3,-0.4,-1.4,-2.0,2.4,-2.0]]
links=["panda_link4","panda_link6","panda_link7"]
def fkl(q):
    req=GetPositionFK.Request(); req.fk_link_names=links; req.robot_state.joint_state=r._seed(q)
    f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30)
    return [np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],3) for ps in f.result().pose_stamped]
res=[]
for phi_deg in [-30,-15,0,15,30]:
    phi=np.radians(phi_deg); d=np.array([np.sin(phi),-np.cos(phi),0]); c=np.array([np.cos(phi),np.sin(phi),0])
    quat=hand_quat(d,-c)
    hp=np.array([B[0],B[1],0])-POT_OFF*d
    for z in [1.0,1.13]:
        for s in seeds:
            sol=r.ik_world([hp[0],hp[1],z],quat,seed=s)
            if sol is None: continue
            sol=np.array(sol); margin=np.min(np.minimum(sol-LIM[:,0],LIM[:,1]-sol))
            res.append((margin,phi_deg,z,tuple(np.round(sol,2))))
res=sorted(set(res),key=lambda t:-t[0])
for m in res[:25]:
    print(f"margin={m[0]:.2f} phi={m[1]} z={m[2]} q={m[3]} links={fkl(list(m[3]))}")
EOF

# openrua op 36
timeout 900 python3 - <<'EOF'
from moka_plan import *
r=Robot("plan3")
LIM=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
seeds=[[0.38,1.03,-0.63,-1.94,-2.22,2.15,0.9],[0,0.3,0,-2.0,0,2.3,0.78],[-0.3,0.5,0,-1.8,0,2.3,0.4],[0.3,0.8,-0.3,-1.5,-0.5,2.3,0.78],[0.5,1.0,-0.5,-1.5,-1.5,2.5,0.0],[0.5,0.6,-0.5,-1.9,-0.5,2.6,1.2],[0.8,1.2,-1.0,-1.7,-2.2,2.2,1.0],[1.0,1.0,-0.8,-1.6,-2.5,2.0,1.5],[0.6,1.3,-0.4,-1.4,-2.0,2.4,0.5],[0.3,0.9,-0.3,-1.3,-2.3,2.0,0.8]]
links=["panda_link4","panda_link6","panda_link7"]
def fkl(q):
    req=GetPositionFK.Request(); req.fk_link_names=links; req.robot_state.joint_state=r._seed(q)
    f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30)
    return [np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],3) for ps in f.result().pose_stamped]
res=[]
for phi_deg in [-15,0,15,30,45]:
    phi=np.radians(phi_deg); d=np.array([np.sin(phi),-np.cos(phi),0]); c=np.array([np.cos(phi),np.sin(phi),0])
    quat=hand_quat(d,+c)   # hand x down
    hp=np.array([B[0],B[1],0])-POT_OFF*d
    for z in [1.0,1.13]:
        for s in seeds:
            sol=r.ik_world([hp[0],hp[1],z],quat,seed=s)
            if sol is None: continue
            sol=np.array(sol); margin=np.min(np.minimum(sol-LIM[:,0],LIM[:,1]-sol))
            res.append((margin,phi_deg,z,tuple(np.round(sol,2))))
res=sorted(set(res),key=lambda t:-t[0])
for m in res[:20]:
    print(f"margin={m[0]:.2f} phi={m[1]} z={m[2]} q={m[3]} links={fkl(list(m[3]))}")
EOF

# openrua op 37
timeout 900 python3 - <<'EOF'
from moka_plan import *
r=Robot("plan4")
LIM=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
links=["panda_link4","panda_link6","panda_link7"]
def fkl(q):
    req=GetPositionFK.Request(); req.fk_link_names=links; req.robot_state.joint_state=r._seed(q)
    f=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,f,timeout_sec=30)
    return [np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],3) for ps in f.result().pose_stamped]
for phi_deg,seed in [(30,[0.72,0.68,-0.08,-1.76,-1.44,1.53,0.09]),(15,[0.66,0.82,0.0,-1.51,-1.29,1.29,0.02]),(45,[1.04,0.62,-0.47,-1.95,-1.53,1.87,0.18])]:
    phi=np.radians(phi_deg); d=np.array([np.sin(phi),-np.cos(phi),0]); c=np.array([np.cos(phi),np.sin(phi),0])
    quat=hand_quat(d,+c)
    hp=np.array([B[0],B[1],0])-POT_OFF*d
    s=seed
    for z in [1.13,1.09,1.05,1.02,1.0]:
        sol=r.ik_world([hp[0],hp[1],z],quat,seed=s)
        if sol is None: print(phi_deg,z,"FAIL"); continue
        s=sol; sol=np.array(sol); margin=np.min(np.minimum(sol-LIM[:,0],LIM[:,1]-sol))
        print(f"phi={phi_deg} z={z} margin={margin:.2f} q={np.round(sol,2)} links={fkl(list(sol))}")
# grasp family sign=+1 at phi=45 for consistency
phi=PHI; quat=hand_quat(D,+CC)
for lab,p,s in [("grasp",HAND_GRASP,[0.38,1.03,-0.63,-1.94,-2.22,2.15,0.9]),("pre",HAND_PRE,None),("lift",HAND_GRASP+[0,0,0.165],None)]:
    sol=r.ik_world(p,quat,seed=s if s else sol)
    sol=np.array(sol); margin=np.min(np.minimum(sol-LIM[:,0],LIM[:,1]-sol))
    print(lab, f"margin={margin:.2f}", np.round(sol,2), fkl(list(sol)))
EOF

# openrua op 38
cat >> /workspace/rk.py <<'EOF'

def slerp(q0, q1, t):
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
    d = np.dot(q0, q1)
    if d < 0: q1 = -q1; d = -d
    if d > 0.9995:
        q = q0 + t*(q1-q0); return q/np.linalg.norm(q)
    th = math.acos(d)
    return (math.sin((1-t)*th)*q0 + math.sin(t*th)*q1)/math.sin(th)

def move_interp(m, pos, quat, n=6, seconds=4.0, max_jump=0.6, dry=False):
    """Cartesian-ish move: n IK waypoints between current hand pose and target, one trajectory."""
    p0, q0 = m.fk_world(); pos = np.asarray(pos, float)
    seed = m.arm_q(); wps = []
    for i in range(1, n+1):
        t = i/n
        q = m.ik_world(p0 + t*(pos-p0), slerp(q0, quat, t), seed=seed)
        if q is None:
            print(f"  IK failed at waypoint {i}/{n}", flush=True); return None
        jump = np.abs(np.array(q)-np.array(seed)).max()
        if jump > max_jump:
            print(f"  joint jump {jump:.2f} at waypoint {i}/{n}: {np.round(q,2)}", flush=True); return None
        wps.append(q); seed = q
    print(f"  {n} waypoints ok, final q {np.round(wps[-1],2)}", flush=True)
    if dry: return wps
    m.goto_q(wps[-1], seconds, via=wps[:-1])
    p, qq = m.fk_world()
    print(f"  hand now {np.round(p,4)} (target {np.round(pos,4)}) err={np.linalg.norm(p-pos)*1000:.1f}mm quat_err={1-abs(np.dot(qq,quat)):.4f}", flush=True)
    return wps
EOF
python3 - <<'EOF'
s=open("/workspace/moka_plan.py").read()
s=s.replace("Q_GRASP = hand_quat(D, -CC)             # hand x up (camera on top)","Q_GRASP = hand_quat(D, +CC)             # hand x DOWN (this IK family has good joint margins)")
s=s.replace("SEED_GRASP = [0.53, 1.15, -0.8, -1.94, -2.26, 2.25, -2.05]","SEED_GRASP = [0.38, 1.03, -0.65, -1.96, -2.33, 2.13, 0.99]")
s=s.replace("PHI_P = 0.0","PHI_P = np.radians(15.0)")
s=s.replace("Q_PLACE = hand_quat(DP, -CP)","Q_PLACE = hand_quat(DP, +CP)")
open("/workspace/moka_plan.py","w").write(s)
EOF
cat > /workspace/moka_a.py <<'EOF'
# stage A: reorient high, descend to pre-grasp
from moka_plan import *
m = Mover("moka_a")
print("wrench", wrench(m))
print("open gripper"); m.gripper(0.04)
pre_high = HAND_PRE + np.array([0, 0, 0.25])
print("-> pre_high", np.round(pre_high,3)); 
if move_interp(m, pre_high, Q_GRASP, n=8, seconds=6.0) is None: raise SystemExit("abort")
print("-> pre", np.round(HAND_PRE,3))
if move_interp(m, HAND_PRE, Q_GRASP, n=6, seconds=4.0) is None: raise SystemExit("abort")
print("wrench", wrench(m)); print("fingers", m.fingers())
EOF
timeout 1500 python3 -u moka_a.py 2>&1 | tee moka.log

# openrua op 39
cat > /workspace/moka_a2.py <<'EOF'
from moka_plan import *
m = Mover("moka_a2")
q=m.arm_q(); print("q now", np.round(q,3))
target=[0.08,0.77,-0.23,-2.23,-2.43,1.79,0.81]
print("resend pre q"); m.goto_q(target, 4.0)
q=m.arm_q(); print("q now", np.round(q,3), "target", target)
p,qq=m.fk_world(); print("hand", np.round(p,4), "quat err", 1-abs(np.dot(qq,Q_GRASP)))
print("wrench", wrench(m))
EOF
timeout 900 python3 -u moka_a2.py 2>&1 | tee -a moka.log

# openrua op 40
timeout 120 python3 tools/perception/cam_snap.py agentview agent3.png; timeout 120 python3 tools/perception/cam_snap.py frontview front3.png

# openrua op 41
cat > /workspace/moka_b.py <<'EOF'
import sys
from moka_plan import *
m = Mover("moka_b")
dist = float(sys.argv[1])
p0,_ = m.fk_world()
target = p0 + dist*D; target[2] = GRASP_Z
print("advance", dist, "->", np.round(target,4)); print("wrench before", wrench(m))
r = move_interp(m, target, Q_GRASP, n=3, seconds=2.5)
print("wrench after", wrench(m))
EOF
timeout 900 python3 -u moka_b.py 0.06 2>&1 | tee -a moka.log; timeout 120 python3 tools/perception/cam_snap.py birdview bird3.png

# openrua op 42
timeout 60 python3 - <<'EOF'
import cv2
im=cv2.imread("bird3.png"); crop=im[230:330,180:330]; cv2.imwrite("bird3_crop.png",cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 43
timeout 900 python3 -u moka_b.py 0.015 2>&1 | tee -a moka.log; timeout 120 python3 tools/perception/cam_snap.py sideview side4.png

# openrua op 44
timeout 120 python3 tools/perception/cam_snap.py birdview bird4.png; timeout 60 python3 - <<'EOF'
import cv2
im=cv2.imread("bird4.png"); crop=im[230:330,180:330]; cv2.imwrite("bird4_crop.png",cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread("side4.png"); crop=im[180:330,200:420]; cv2.imwrite("side4_crop.png",cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
EOF

# openrua op 45
cat > moka_c.py <<'EOF'
import sys, numpy as np, rclpy
from rk import *
from moka_plan import *
rclpy.init(); m = Mover()
print("wrench before", np.round(wrench(m),2))
m.gripper(0.0)
for _ in range(20): rclpy.spin_once(m, timeout_sec=0.05)
print("fingers", m.fingers())
print("wrench after", np.round(wrench(m),2))
p,q = m.fk_world(); print("hand", np.round(p,4))
EOF
timeout 120 python3 -u moka_c.py 2>&1 | tail -8

# openrua op 46
grep -n "class Mover\|class Robot\|def __init__\|super().__init__" rk.py | head; grep -n "rclpy.init\|Mover(" moka_b.py

# openrua op 47
sed -n 29,45p rk.py; sed -n 1,8p moka_b.py; grep -n "def wrench" -A8 moka_plan.py

# openrua op 48
cat > moka_c.py <<'EOF'
import sys, numpy as np, rclpy
from moka_plan import *
m = Mover("moka_c")
print("wrench before", np.round(wrench(m),2))
m.gripper(0.0)
print("fingers", m.fingers())
print("wrench after", np.round(wrench(m),2))
p,q = m.fk_world(); print("hand", np.round(p,4))
EOF
timeout 120 python3 -u moka_c.py 2>&1 | tail -8

# openrua op 49
cat > fing.py <<'EOF'
from moka_plan import *
m = Mover("fing")
for i in range(3):
    print("fingers", m.fingers(), "wrench", np.round(wrench(m,5),2))
EOF
timeout 60 python3 -u fing.py 2>&1 | tail -4

# openrua op 50
grep -n "def gripper" -A22 rk.py; grep -n "gripper" knob.py | head

# openrua op 51
grep -n "gripper" knob.log | head; grep -n "fingers" knob.log | head -5

# openrua op 52
timeout 120 python3 -u moka_c.py 2>&1 | tail -4; python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null 2>&1; python3 tools/perception/cam_snap.py sideview >/dev/null 2>&1; ls -la robot0_eye_in_hand.png sideview.png

# openrua op 53
python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw 2>&1 | tail -2; ls -t *.npy | head -3

# openrua op 54
python3 - <<'EOF'
import numpy as np
d = np.load("robot0_eye_in_hand.npy"); fx=312.77; cx=320; cy=240
print(d.shape, np.nanmin(d), np.nanmax(d))
for v in range(150, 420, 10):
    row = d[v]
    # pot pixels: depth < 0.3
    idx = np.where(row < 0.25)[0]
    if len(idx)==0: print(v, "none", np.round(row[320],3)); continue
    w = (idx[-1]-idx[0]) * row[320]/fx
    print(v, "cols", idx[0], idx[-1], "d_c", np.round(row[320],3), "width_m", np.round(w,3), "y_off(m)", np.round((v-cy)*row[320]/fx,3))
EOF

# openrua op 55
python3 - <<'EOF'
import numpy as np
d = np.load("birdview_depth.npy")
print("table depth samples", [np.round(float(d[v,u]),3) for v,u in [(100,100),(400,500),(240,150),(350,200)]])
sub = d[255:305, 220:275]
print("pot region min depth", np.round(sub.min(),3), "-> z", np.round(3.0-sub.min(),3))
# table z
tz = 3.0 - np.median(d[380:460, 40:140]); print("table z", np.round(tz,3))
# stove coil top
print("coil region z", np.round(3.0 - d[290:330, 330:400].min(),3))
EOF

# openrua op 56
cat > moka_d.py <<'EOF'
import sys
from moka_plan import *
m = Mover("moka_d")
TH = np.radians(12.0)
DT = D*np.cos(TH) - np.array([0,0,1.0])*np.sin(TH)      # approach tilted 12 deg down
Q_TILT = hand_quat(DT, +CC)
step = sys.argv[1]; dry = len(sys.argv) > 2 and sys.argv[2] == "dry"
p0,q0 = m.fk_world(); print("hand now", np.round(p0,4), "q", np.round(m.arm_q(),3))
print("wrench before", np.round(wrench(m),2))
if step == "back":
    t = p0 - 0.04*D; move_interp(m, t, Q_GRASP, n=2, seconds=2.5, dry=dry)
elif step == "tilt":
    t = p0.copy(); t[2] = 0.950; move_interp(m, t, Q_TILT, n=4, seconds=3.0, dry=dry)
elif step == "adv":
    dist = float(sys.argv[3]) if len(sys.argv) > 3 else None
    horiz = 0.1034*np.cos(TH)
    goal = np.array([C[0], C[1], 0.950]) + (TIP_PAST - horiz)*np.array([D[0], D[1], 0.0])
    if dist is not None: goal = p0 + dist*D; goal[2] = 0.950
    print("goal", np.round(goal,4)); move_interp(m, goal, Q_TILT, n=3, seconds=2.5, dry=dry)
elif step == "close":
    m.gripper(0.0)
elif step == "open":
    m.gripper(0.04)
p1,q1 = m.fk_world(); print("hand after", np.round(p1,4), "q", np.round(m.arm_q(),3), "fingers", np.round(m.fingers(),4))
print("wrench after", np.round(wrench(m),2))
EOF
timeout 120 python3 -u moka_d.py back dry 2>&1 | tail -6; timeout 120 python3 -u moka_d.py tilt dry 2>&1 | tail -6

# openrua op 57
timeout 200 python3 -u moka_d.py back 2>&1 | grep -v "^\s*[0-9.e+-]*\]$" | tail -5; timeout 200 python3 -u moka_d.py tilt 2>&1 | tail -5

# openrua op 58
for c in birdview sideview agentview; do python3 tools/perception/cam_snap.py $c >/dev/null 2>&1; done; python3 - <<'EOF'
import cv2
b = cv2.imread("birdview.png"); cv2.imwrite("bird5_crop.png", cv2.resize(b[230:330,180:330], None, fx=5, fy=5, interpolation=cv2.INTER_NEAREST))
s = cv2.imread("sideview.png"); cv2.imwrite("side5_crop.png", cv2.resize(s[180:330,200:420], None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 59
python3 tools/perception/cam_snap.py /birdview/depth/image_raw >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
d = np.load("birdview.npy")
z = 3.0 - d
# pot top: z within 1.03..1.07 region near pixel (247,280), excluding robot (which is brighter/higher). look in window
win = z[240:330, 200:300]
mask = (win > 1.03) & (win < 1.08)
vs, us = np.nonzero(mask)
print("n", len(vs))
v = vs.mean()+240; u = us.mean()+200
dist = 3.0 - 1.057
x = (v-240)*dist/579.41 - 0.2; y = (u-320)*dist/579.41
print("pot top centroid px", round(u,1), round(v,1), "-> world", round(x,4), round(y,4))
# top z stats
print("z max", np.round(win[mask].max(),3))
EOF

# openrua op 60
python3 - <<'EOF'
import numpy as np
for f in ["birdview_depth.npy","birdview.npy"]:
    d = np.load(f); z = 3.0 - d
    win = z[240:330, 200:300]; mask = (win > 1.03) & (win < 1.08)
    vs, us = np.nonzero(mask); v = vs.mean()+240; u = us.mean()+200
    dist = 3.0 - 1.057
    x = (v-240)*dist/579.41 - 0.2; y = (u-320)*dist/579.41
    # also the widest-body estimate: region z>0.92 (whole pot) but exclude robot: restrict cols<290
    m2 = (win > 0.92) & (win < 1.08)
    print(f, "top centroid", round(x,4), round(y,4), "n", len(vs), "| body n", m2.sum())
EOF

# openrua op 61
cat > moka_e.py <<'EOF'
import sys
from moka_plan import *
m = Mover("moka_e")
TH = np.radians(12.0); DT = D*np.cos(TH) - np.array([0,0,1.0])*np.sin(TH); Q_TILT = hand_quat(DT, +CC)
p0,q0 = m.fk_world(); print("hand now", np.round(p0,4)); print("wrench before", np.round(wrench(m),2))
dx,dy,dz = [float(a) for a in sys.argv[1:4]]; quat = Q_TILT if (len(sys.argv)>4 and sys.argv[4]=="tilt") else Q_GRASP
t = p0 + np.array([dx,dy,dz]); move_interp(m, t, quat, n=3, seconds=2.5)
p1,q1 = m.fk_world(); print("hand after", np.round(p1,4), "q", np.round(m.arm_q(),3), "fingers", np.round(m.fingers(),4))
print("wrench after", np.round(wrench(m),2))
EOF
timeout 200 python3 -u moka_e.py -0.02 0.02 0.05 tilt 2>&1 | tail -5

# openrua op 62
timeout 120 python3 -u moka_d.py close 2>&1 | grep -E "gripper|fingers" | tail -2; timeout 120 python3 -u moka_d.py open 2>&1 | grep -E "gripper" | tail -1

# openrua op 63
cat tools/action/gripper_cmd.py | head -60

# openrua op 64
timeout 60 python3 tools/action/gripper_cmd.py 0.0 2>&1 | tail -2; timeout 60 python3 -u fing.py 2>&1 | tail -1

# openrua op 65
cat > grip_tick.py <<'EOF'
import sys
from moka_plan import *
m = Mover("grip_tick")
w = float(sys.argv[1]); secs = float(sys.argv[2]) if len(sys.argv)>2 else 2.0
print("fingers before", np.round(m.fingers(),4))
m.gripper(w)
q = m.arm_q(); m.goto_q(q, secs)   # hold pose to advance the sim clock
print("fingers after", np.round(m.fingers(),4), "wrench", np.round(wrench(m),2))
EOF
timeout 120 python3 -u grip_tick.py 0.0 2.0 2>&1 | tail -4

# openrua op 66
timeout 20 ros2 action list 2>&1; timeout 20 ros2 topic list 2>&1 | grep -iv "camera\|image\|depth" 

# openrua op 67
timeout 30 python3 - <<'EOF' 2>&1 | tail -5
import rclpy, time
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node("jse"); got=[]
n.create_subscription(JointState, "/joint_states", got.append, 1)
end=time.time()+10
while not got and time.time()<end: rclpy.spin_once(n, timeout_sec=0.2)
m=got[0]
for nm,p,v,e in zip(m.name,m.position,m.velocity,m.effort):
    if "finger" in nm: print(nm, round(p,4), round(v,4), round(e,3))
print("stamp", m.header.stamp.sec, m.header.stamp.nanosec)
EOF
timeout 10 ros2 topic echo /clock --once 2>&1 | head -4

# openrua op 68
python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null 2>&1; python3 tools/perception/cam_snap.py agentview >/dev/null 2>&1; echo done

# openrua op 69
python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
d = np.load("robot0_eye_in_hand.npy")
for v in [5,30,60,100,150,200,240,280,320,360,400,440,475]:
    print(v, np.round(d[v,[100,320,540]],3))
EOF

# openrua op 70
ls -la --time-style=+%H:%M *.npy | head -20; python3 - <<'EOF'
import numpy as np
d = np.load("birdview_depth.npy"); z = 3.0-d
# print a coarse map of z>0.95 regions near the robot (top rows)
sub = z[0:260, 200:440]
ys,xs = np.nonzero((sub>1.0)&(sub<1.2))
print("hand-ish region rows", ys.min(), ys.max(), "cols", xs.min()+200, xs.max()+200) if len(ys) else print("none")
# histogram of z in that window
import collections
vals = np.round(sub[(sub>0.95)],2); print(collections.Counter(vals.flatten()).most_common(12))
EOF

# openrua op 71
find / -iname "*hand*.stl" -o -iname "*hand*.obj" -o -iname "*hand*.dae" -o -iname "*finger*.stl" -o -iname "*finger*.obj" 2>/dev/null | grep -v proc | head; grep -o 'filename="[^"]*hand[^"]*"' robot.urdf | head -3

# openrua op 72
ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ 2>&1 | head; python3 - <<'EOF'
import struct, numpy as np, os
p="/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/hand.stl"
if os.path.exists(p):
    b=open(p,'rb').read()
    n=struct.unpack('<I', b[80:84])[0]
    arr=np.frombuffer(b[84:84+n*50], dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    v=arr['v'].reshape(-1,3)
    print("hand.stl tris",n,"min",v.min(0),"max",v.max(0))
p2=p.replace("hand.stl","finger.stl")
if os.path.exists(p2):
    b=open(p2,'rb').read(); n=struct.unpack('<I', b[80:84])[0]
    arr=np.frombuffer(b[84:84+n*50], dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')])); v=arr['v'].reshape(-1,3)
    print("finger.stl min",v.min(0),"max",v.max(0))
EOF

# openrua op 73
cat > probe.py <<'EOF'
import sys
from moka_plan import *
m = Mover("probe")
p0,q0 = m.fk_world(); print("hand now", np.round(p0,4)); w0 = wrench(m); print("wrench before", np.round(w0,2))
tx,ty,tz = [float(a) for a in sys.argv[1:4]]
t = np.array([tx,ty,tz]); move_interp(m, t, Q_GRASP, n=3, seconds=3.0)
p1,q1 = m.fk_world(); print("hand after", np.round(p1,4), "q", np.round(m.arm_q(),3), "fingers", np.round(m.fingers(),4))
w1 = wrench(m); print("wrench after", np.round(w1,2), "delta", np.round(np.array(w1)-np.array(w0),2))
EOF
timeout 200 python3 -u probe.py -0.22 -0.10 0.985 2>&1 | tail -4

# openrua op 74
timeout 60 python3 tools/action/gripper_cmd.py 0.04 2>&1 | tail -1; timeout 60 python3 -u fing.py 2>&1 | tail -1; timeout 60 python3 tools/action/gripper_cmd.py 0.0 2>&1 | tail -1; timeout 60 python3 -u fing.py 2>&1 | tail -1

# openrua op 75
timeout 20 ros2 node list 2>&1; ps aux | grep -i "bridge\|sim\|mujoco\|robosuite" | grep -v grep | head

# openrua op 76
cat > gtest.py <<'EOF'
import sys, time, rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
rclpy.init(); n = rclpy.create_node("gtest")
clk = {}; js = {}
n.create_subscription(Clock, "/clock", lambda m: clk.__setitem__("t", m.clock.sec + m.clock.nanosec*1e-9), 10)
n.create_subscription(JointState, "/joint_states", lambda m: js.__setitem__("f", [p for nm,p in zip(m.name,m.position) if "finger" in nm]), 10)
c = ActionClient(n, GripperCommand, "/franka_gripper/gripper_action"); c.wait_for_server(10)
for _ in range(10): rclpy.spin_once(n, timeout_sec=0.1)
print("clock before", clk.get("t"), "fingers", js.get("f"))
g = GripperCommand.Goal(); g.command.position = float(sys.argv[1]); g.command.max_effort = 30.0
fb = []
f = c.send_goal_async(g, feedback_callback=lambda m: fb.append((m.feedback.position, m.feedback.reached_goal, m.feedback.stalled)))
rclpy.spin_until_future_complete(n, f, timeout_sec=30); h = f.result()
t0=time.time(); rf = h.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=120)
r = rf.result(); print("status", r.status, "result pos", r.result.position, "effort", r.result.effort, "reached", r.result.reached_goal, "stalled", r.result.stalled, "wall", round(time.time()-t0,2))
print("feedback n", len(fb), fb[:3], fb[-3:])
for _ in range(10): rclpy.spin_once(n, timeout_sec=0.1)
print("clock after", clk.get("t"), "fingers", js.get("f"))
EOF
timeout 120 python3 -u gtest.py 0.0 2>&1 | tail -5

# openrua op 77
python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null 2>&1; python3 tools/perception/cam_snap.py frontview >/dev/null 2>&1; python3 -c "
import cv2; im=cv2.imread('robot0_eye_in_hand.png'); cv2.imwrite('eih_crop.png', cv2.resize(im[300:480,0:640], None, fx=1.5, fy=1.5))"

# openrua op 78
python3 tools/perception/cam_snap.py birdview >/dev/null 2>&1; python3 -c "
import cv2; b=cv2.imread('birdview.png'); cv2.imwrite('bird6_crop.png', cv2.resize(b[200:340,150:350], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))"

# openrua op 79
timeout 200 python3 -u probe.py -0.214 -0.104 1.13 2>&1 | tail -3; python3 tools/perception/cam_snap.py /birdview/depth/image_raw >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np
d = np.load("birdview.npy"); z = 3.0 - d
win = z[150:330, 150:330]; mask = (win > 1.15) & (win < 1.26)
vs, us = np.nonzero(mask); print("n", len(vs), "zmax", np.round(win.max(),3))
if len(vs):
    zt = np.median(win[mask]); dist = 3.0 - zt
    v = vs.mean()+150; u = us.mean()+150
    print("pot top z", round(zt,3), "-> world", round((v-240)*dist/579.41-0.2,4), round((u-320)*dist/579.41,4))
EOF

# openrua op 80
python3 - <<'EOF'
import numpy as np
d = np.load("birdview.npy"); z = 3.0 - d
sub = z[235:285, 235:290]
np.set_printoptions(linewidth=200)
print(np.round(sub[::5, ::5],2))
mask = (sub>1.17)&(sub<1.23); vs,us = np.nonzero(mask)
if len(vs):
    zt = np.median(sub[mask]); dist=3.0-zt; v=vs.mean()+235; u=us.mean()+235
    print("n",len(vs),"pot top z", round(zt,3), "world", round((v-240)*dist/579.41-0.2,4), round((u-320)*dist/579.41,4))
EOF

# openrua op 81
cat > moka_t.py <<'EOF'
import sys
from moka_plan import *
m = Mover("moka_t")
POT_OFF2 = 0.109
dry = "dry" in sys.argv
p0,q0 = m.fk_world(); print("hand now", np.round(p0,4)); w0 = wrench(m)
z = float(sys.argv[1])
goal = np.array([B[0] - POT_OFF2*DP[0], B[1] - POT_OFF2*DP[1], z])
print("goal", np.round(goal,4))
r = move_interp(m, goal, Q_PLACE, n=8, seconds=6.0, dry=dry)
p1,q1 = m.fk_world(); print("hand after", np.round(p1,4), "q", np.round(m.arm_q(),3), "fingers", np.round(m.fingers(),4))
w1 = wrench(m); print("wrench after", np.round(w1,2), "delta", np.round(np.array(w1)-np.array(w0),2))
EOF
timeout 200 python3 -u moka_t.py 1.13 dry 2>&1 | grep -E "goal|waypoints|final|fail|jump" | tail -4

# openrua op 82
timeout 300 python3 -u moka_t.py 1.13 2>&1 | grep -v "^\s*[0-9.e+-]*\]$" | tail -5

# openrua op 83
timeout 300 python3 -u moka_t.py 1.13 2>&1 | grep -v "^\s*[0-9.e+-]*\]$" | tail -4

# openrua op 84
python3 tools/perception/cam_snap.py /birdview/depth/image_raw >/dev/null 2>&1; python3 tools/perception/cam_snap.py birdview >/dev/null 2>&1; python3 - <<'EOF'
import numpy as np, cv2
d = np.load("birdview.npy"); z = 3.0 - d
# expected pot top near B, z ~1.17: px
B=(-0.057,0.207); dist=3.0-1.17
u=320+B[1]*579.41/dist; v=240+(B[0]+0.2)*579.41/dist; print("expected px", round(u), round(v))
sub = z[int(v)-30:int(v)+30, int(u)-30:int(u)+30]
mask=(sub>1.15)&(sub<1.22); vs,us=np.nonzero(mask)
zt=np.median(sub[mask]); dd=3.0-zt; vv=vs.mean()+int(v)-30; uu=us.mean()+int(u)-30
print("n",len(vs),"pot top z",round(zt,3),"world",round((vv-240)*dd/579.41-0.2,4), round((uu-320)*dd/579.41,4))
# burner: what is under the pot? z map coarse
np.set_printoptions(linewidth=200); print(np.round(z[int(v)-40:int(v)+41:8, int(u)-40:int(u)+41:8],2))
b=cv2.imread("birdview.png"); cv2.imwrite("bird7_crop.png", cv2.resize(b[int(v)-60:int(v)+60, int(u)-80:int(u)+80], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 85
timeout 300 python3 -u moka_t.py 1.06 2>&1 | grep -v "^\s*[0-9.e+-]*\]$" | tail -3

# openrua op 86
timeout 300 python3 -u moka_t.py 1.045 2>&1 | grep -v "^\s*[0-9.e+-]*\]$" | tail -3

# openrua op 87
cat > pottop.py <<'EOF'
import numpy as np, subprocess
subprocess.run(["python3","tools/perception/cam_snap.py","/birdview/depth/image_raw"], capture_output=True)
d = np.load("birdview.npy"); z = 3.0 - d
B=(-0.057,0.207)
best=None
for zt_guess in np.arange(0.95, 1.25, 0.01):
    dist=3.0-zt_guess; u=int(320+B[1]*579.41/dist); v=int(240+(B[0]+0.2)*579.41/dist)
    sub=z[v-25:v+25, u-25:u+25]; mask=(np.abs(sub-zt_guess)<0.012)
    if mask.sum()>150 and (best is None or mask.sum()>best[0]): best=(mask.sum(), zt_guess, np.median(sub[mask]), u, v, sub, mask)
n,zg,zt,u,v,sub,mask=best; vs,us=np.nonzero(mask); dd=3.0-zt
print("pot top z", round(zt,3), "n", n, "world", round(((vs.mean()+v-25)-240)*dd/579.41-0.2,4), round(((us.mean()+u-25)-320)*dd/579.41,4))
EOF
python3 pottop.py

# openrua op 88
python3 tools/perception/cam_snap.py birdview >/dev/null 2>&1; python3 tools/perception/cam_snap.py agentview >/dev/null 2>&1; python3 tools/perception/cam_snap.py frontview >/dev/null 2>&1; python3 - <<'EOF'
import cv2
b=cv2.imread("birdview.png"); cv2.imwrite("bird8_crop.png", cv2.resize(b[225:345,306:466], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 89
cat > pottop2.py <<'EOF'
import numpy as np, subprocess
subprocess.run(["python3","tools/perception/cam_snap.py","/birdview/depth/image_raw"], capture_output=True)
d = np.load("birdview.npy"); z = 3.0 - d
P=(-0.0585,0.19); zt=1.07; dist=3.0-zt
u=int(320+P[1]*579.41/dist); v=int(240+(P[0]+0.2)*579.41/dist)
sub=z[v-8:v+9, u-8:u+9]
print("pot top region z: max", np.round(sub.max(),3), "median", np.round(np.median(sub),3), "p90", np.round(np.percentile(sub,90),3))
EOF
python3 pottop2.py

# openrua op 90
timeout 300 python3 -u moka_t.py 1.038 2>&1 | grep -v "^\s*[0-9.e+-]*\]$" | tail -2; python3 pottop2.py

# openrua op 91
timeout 300 python3 -u moka_t.py 1.032 2>&1 | grep -v "^\s*[0-9.e+-]*\]$" | tail -1; python3 pottop2.py

# openrua op 92
timeout 300 python3 -u moka_t.py 1.025 2>&1 | grep -v "^\s*[0-9.e+-]*\]$" | tail -2; python3 pottop2.py

# openrua op 93
python3 - <<'EOF'
import numpy as np
d = np.load("birdview.npy"); z = 3.0 - d
B=(-0.057,0.207); dist=3.0-0.93
u=int(320+B[1]*579.41/dist); v=int(240+(B[0]+0.2)*579.41/dist)
np.set_printoptions(linewidth=220)
print(np.round(z[v-36:v+37:6, u-36:u+37:6],3))
EOF

# openrua op 94
python3 - <<'EOF'
import numpy as np
d = np.load("birdview_depth.npy"); z = 3.0 - d
P=(-0.055,-0.263); dist=3.0-1.06
u=int(320+P[1]*579.41/dist); v=int(240+(P[0]+0.2)*579.41/dist)
sub=z[v-9:v+10, u-9:u+10]
np.set_printoptions(linewidth=220); print(np.round(sub[::2, ::2],3))
vals=sub[sub>1.0]; print("rim-ish median", np.round(np.median(vals),3), "max", np.round(vals.max(),3), "n", len(vals))
EOF

# openrua op 95
timeout 300 python3 -u moka_t.py 1.02 2>&1 | grep -v "^\s*[0-9.e+-]*\]$" | tail -2; python3 pottop2.py

# openrua op 96
timeout 120 python3 -u grip_tick.py 0.04 2.0 2>&1 | tail -3; python3 pottop2.py

# openrua op 97
cat > retreat.py <<'EOF'
import sys
from moka_plan import *
m = Mover("retreat")
p0,q0 = m.fk_world(); print("hand now", np.round(p0,4))
back = float(sys.argv[1]); up = float(sys.argv[2])
t = p0 - back*np.array(DP); t[2] += up
r = move_interp(m, t, Q_PLACE, n=3, seconds=3.0)
p1,q1 = m.fk_world(); print("hand after", np.round(p1,4), "fingers", np.round(m.fingers(),4), "wrench", np.round(wrench(m),2))
EOF
timeout 300 python3 -u retreat.py 0.09 0.0 2>&1 | grep -v "^\s*[0-9.e+-]*\]$" | tail -2; python3 pottop2.py

# openrua op 98
timeout 300 python3 -u retreat.py 0.05 0.15 2>&1 | grep -v "^\s*[0-9.e+-]*\]$" | tail -2; python3 pottop2.py; for c in birdview agentview frontview; do python3 tools/perception/cam_snap.py $c >/dev/null 2>&1; done; python3 - <<'EOF'
import numpy as np, cv2
d = np.load("birdview.npy"); z = 3.0 - d
B=(-0.057,0.207); dist=3.0-1.065
u=int(320+B[1]*579.41/dist); v=int(240+(B[0]+0.2)*579.41/dist)
sub=z[v-30:v+31, u-30:u+31]; mask=(sub>1.05)&(sub<1.08); vs,us=np.nonzero(mask)
vv=vs.mean()+v-30; uu=us.mean()+u-30; dd=3.0-np.median(sub[mask])
print("pot lid centroid world", round((vv-240)*dd/579.41-0.2,4), round((uu-320)*dd/579.41,4), "n", len(vs))
b=cv2.imread("birdview.png"); cv2.imwrite("bird9_crop.png", cv2.resize(b[v-60:v+60, u-80:u+80], None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
EOF
