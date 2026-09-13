#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la tools .ros

# openrua op 2
ls -la tools/action tools/perception; cat tools/perception/*.py tools/action/*.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iE "ik|fk|plan|scene" ; echo ---; timeout 20 ros2 topic echo /joint_states --once

# openrua op 4
mkdir -p snaps && for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40; echo ---; timeout 20 ros2 topic echo /agentview/color/camera_info --once | grep -A12 "^k:"

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
t0=time.time()
while time.time()-t0 < 4: rclpy.spin_once(node, timeout_sec=0.2)
for k,v in sorted(seen.items()):
    print(k, f"t=({v.translation.x:.4f},{v.translation.y:.4f},{v.translation.z:.4f}) q=({v.rotation.x:.4f},{v.rotation.y:.4f},{v.rotation.z:.4f},{v.rotation.w:.4f})")
EOF
timeout 40 python3 tfdump.py

# openrua op 7
cat > percep.py <<'EOF'
"""Grab color+depth+info from a camera, return world-frame point cloud helpers."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from cv_bridge import CvBridge
import cv2

def quat_R(x,y,z,w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def grab(node, topic, typ, timeout=20):
    got={}
    sub=node.create_subscription(typ, topic, lambda m: got.setdefault("m",m), 1)
    import time; t0=time.time()
    while "m" not in got and time.time()-t0<timeout: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got: raise SystemExit(f"no msg on {topic}")
    return got["m"]

def cam_T(node, cam):
    seen={}
    def cb(m):
        for t in m.transforms: seen[(t.header.frame_id,t.child_frame_id)]=t.transform
    qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    s=node.create_subscription(TFMessage,"/tf_static",cb,qos)
    import time; t0=time.time()
    key=("world",f"{cam}_optical_frame")
    while key not in seen and time.time()-t0<10: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s)
    tr=seen[key]; T=np.eye(4)
    T[:3,:3]=quat_R(tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w)
    T[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
    return T

def cloud(cam, node=None):
    own = node is None
    if own:
        rclpy.init(); node=rclpy.create_node("percep")
    color=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/color/image_raw",Image),"bgr8")
    depth=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/depth/image_raw",Image),"passthrough").astype(np.float32)
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    T=cam_T(node,cam)
    if own: rclpy.shutdown()
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    h,w=depth.shape
    u,v=np.meshgrid(np.arange(w),np.arange(h))
    X=(u-cx)*depth/fx; Y=(v-cy)*depth/fy
    P=np.stack([X,Y,depth,np.ones_like(depth)],-1) @ T.T
    return color, depth, P[...,:3]

if __name__=="__main__":
    cam=sys.argv[1]
    color,depth,P=cloud(cam)
    np.save(f"snaps/{cam}_P.npy",P); cv2.imwrite(f"snaps/{cam}.png",color)
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    for a in sys.argv[2:]:
        u,v=map(int,a.split(","))
        print(f"px({u},{v}) -> world {P[v,u]}")
EOF
timeout 60 python3 percep.py agentview 105,295 555,295 220,235 395,250 315,200 320,400

# openrua op 8
python3 - <<'EOF'
import re
s=open("percep.py").read()
s=s.replace('''    s=node.create_subscription(TFMessage,"/tf_static",cb,qos)
''','''    s=node.create_subscription(TFMessage,"/tf_static",cb,qos)
    s2=node.create_subscription(TFMessage,"/tf",cb,100)
''').replace("    node.destroy_subscription(s)\n    tr=seen[key]","    node.destroy_subscription(s); node.destroy_subscription(s2)\n    tr=seen[key]")
open("percep.py","w").write(s)
EOF
timeout 60 python3 percep.py agentview 105,295 555,295 220,235 395,250 315,200 320,400

# openrua op 9
cat > seg.py <<'EOF'
import numpy as np, cv2, sys
from percep import cloud
from scipy import ndimage
cam = sys.argv[1] if len(sys.argv)>1 else "birdview"
color,depth,P=cloud(cam)
z=P[...,2]
mask=(z>0.44)&(z<0.75)&(P[...,0]>-0.35)&(P[...,0]<0.5)&(np.abs(P[...,1])<0.6)
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<15: continue
    pts=P[m]; c=color[m].mean(0)[::-1]
    vs,us=np.where(m)
    print(f"blob{i}: n={m.sum()} px=({us.mean():.0f},{vs.mean():.0f}) xy=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) zmax={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} rgb={c.astype(int)} xr=({pts[:,0].min():.3f},{pts[:,0].max():.3f}) yr=({pts[:,1].min():.3f},{pts[:,1].max():.3f})")
vis=color.copy(); vis[mask]=(0,255,0)
cv2.imwrite(f"snaps/{cam}_seg.png",vis)
EOF
timeout 60 python3 seg.py birdview

# openrua op 10
timeout 60 python3 - <<'EOF'
import numpy as np
P=np.load("snaps/birdview_P.npy")
z=P[...,2]
# table surface samples at several pixels
for (u,v) in [(318,320),(200,240),(440,240),(318,200),(318,359),(150,300)]:
    print((u,v), P[v,u])
# histogram of z over table region
reg=(P[...,0]>-0.4)&(P[...,0]<0.5)&(np.abs(P[...,1])<0.5)
h,e=np.histogram(z[reg],bins=np.arange(0.40,0.80,0.01))
for hh,ee in zip(h,e): 
    if hh: print(f"{ee:.2f} {hh}")
EOF

# openrua op 11
timeout 60 python3 percep.py birdview 318,320 200,240 440,240 318,200 150,300 && timeout 60 python3 - <<'EOF'
import numpy as np
P=np.load("snaps/birdview_P.npy")
z=P[...,2]
reg=(P[...,0]>-0.4)&(P[...,0]<0.5)&(np.abs(P[...,1])<0.5)
h,e=np.histogram(z[reg],bins=np.arange(0.40,0.80,0.01))
for hh,ee in zip(h,e): 
    if hh: print(f"{ee:.2f} {hh}")
EOF

# openrua op 12
timeout 120 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position"

# openrua op 13
timeout 60 python3 seg.py birdview

# openrua op 14
timeout 60 python3 - <<'EOF'
import numpy as np
P=np.load("snaps/birdview_P.npy")
for name,(x0,x1,y0,y1) in {"white":(-0.14,-0.03,-0.20,-0.06),"yellow":(-0.09,0.03,0.05,0.16),"red":(-0.25,-0.17,-0.08,0.05)}.items():
    m=(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.435)
    z=P[m][:,2]
    h,e=np.histogram(z,bins=np.arange(0.43,0.80,0.01))
    print(name, "n",m.sum(), " ".join(f"{ee:.2f}:{hh}" for hh,ee in zip(h,e) if hh))
    top=(P[...,2]>0.49)&(P[...,2]<0.6)&m
    pts=P[top]
    if len(pts): print("  rim-level centroid", pts[:,:2].mean(0), "xr",pts[:,0].min(),pts[:,0].max(),"yr",pts[:,1].min(),pts[:,1].max())
EOF

# openrua op 15
timeout 60 python3 percep.py agentview 215,197 220,200 218,265 395,215 390,222 310,152 330,296 105,285 && timeout 60 python3 - <<'EOF'
import numpy as np
P=np.load("snaps/agentview_P.npy")
for name,(x0,x1,y0,y1) in {"white":(-0.14,-0.03,-0.20,-0.06),"yellow":(-0.09,0.03,0.05,0.16),"red":(-0.25,-0.17,-0.08,0.05),"lplate":(-0.09,0.06,-0.35,-0.21),"rplate":(-0.08,0.06,0.23,0.38)}.items():
    m=(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.43)
    z=P[m][:,2]
    h,e=np.histogram(z,bins=np.arange(0.43,0.80,0.01))
    print(name, "n",m.sum(), "zmax %.3f"%z.max() if len(z) else "", " ".join(f"{ee:.2f}:{hh}" for hh,ee in zip(h,e) if hh))
EOF

# openrua op 16
cat > fk.py <<'EOF'
import rclpy, sys, numpy as np
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
JOINTS=[f"panda_joint{i}" for i in range(1,8)]
def get_js(node):
    got={}
    s=node.create_subscription(JointState,"/joint_states",lambda m: got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node,timeout_sec=0.2)
    node.destroy_subscription(s)
    return dict(zip(got["m"].name,got["m"].position))
def fk(node, cli, q, links=("panda_hand","panda_link8")):
    req=GetPositionFK.Request()
    req.header.frame_id=""
    req.fk_link_names=list(links)
    req.robot_state.joint_state.name=JOINTS
    req.robot_state.joint_state.position=[float(x) for x in q]
    fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=30)
    r=fut.result()
    out={}
    for n,ps in zip(r.fk_link_names,r.pose_stamped):
        p,o=ps.pose.position,ps.pose.orientation
        out[n]=(np.array([p.x,p.y,p.z]),np.array([o.x,o.y,o.z,o.w]),ps.header.frame_id)
    return out, r.error_code.val
if __name__=="__main__":
    rclpy.init(); node=rclpy.create_node("fk")
    cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service(10)
    js=get_js(node)
    q=[js[j] for j in JOINTS]
    print("q",np.round(q,4), "fingers", js["panda_finger_joint1"], js["panda_finger_joint2"])
    out,code=fk(node,cli,q)
    print("code",code)
    for n,(p,o,f) in out.items(): print(n,"frame",f,"pos",np.round(p,4),"quat",np.round(o,4))
    rclpy.shutdown()
EOF
timeout 60 python3 fk.py

# openrua op 17
timeout 60 python3 - <<'EOF'
import numpy as np, rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from percep import quat_R
rclpy.init(); node=rclpy.create_node("x")
seen={}
def cb(m):
    for t in m.transforms: seen[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage,"/tf_static",cb,qos); node.create_subscription(TFMessage,"/tf",cb,100)
t0=time.time()
while time.time()-t0<3: rclpy.spin_once(node,timeout_sec=0.2)
def T(tr):
    M=np.eye(4); M[:3,:3]=quat_R(tr.rotation.x,tr.rotation.y,tr.rotation.z,tr.rotation.w); M[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]; return M
chain=["world","panda_link0"]+[f"panda_link{i}" for i in range(1,9)]+["panda_hand"]
M=np.eye(4)
for a,b in zip(chain[:-1],chain[1:]):
    M=M@T(seen[(a,b)])
    print(b, np.round(M[:3,3],4))
print("hand R\n",np.round(M[:3,:3],3))
EOF

# openrua op 18
timeout 60 python3 - <<'EOF'
import numpy as np
from percep import cloud
color,depth,P=cloud("robot0_eye_in_hand")
np.save("snaps/eih_P.npy",P)
for name,(x0,x1,y0,y1) in {"white":(-0.16,-0.01,-0.22,-0.04),"yellow":(-0.10,0.05,0.03,0.18)}.items():
    m=(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.435)&(P[...,2]<0.65)
    z=P[m][:,2]
    h,e=np.histogram(z,bins=np.arange(0.43,0.65,0.01))
    print(name,"n",m.sum()," ".join(f"{ee:.2f}:{hh}" for hh,ee in zip(h,e) if hh))
    rim=m&(P[...,2]>z.max()-0.015)
    pts=P[rim]
    print("  rim pts",len(pts),"centroid",np.round(pts[:,:2].mean(0),4),"xr",np.round([pts[:,0].min(),pts[:,0].max()],3),"yr",np.round([pts[:,1].min(),pts[:,1].max()],3))
    # at mid height
    for zl in (0.47,0.50,0.53):
        mm=m&(np.abs(P[...,2]-zl)<0.005); pts=P[mm]
        if len(pts): print(f"  z~{zl}: n={len(pts)} xr",np.round([pts[:,0].min(),pts[:,0].max()],3),"yr",np.round([pts[:,1].min(),pts[:,1].max()],3))
EOF

# openrua op 19
cat > ctl.py <<'EOF'
"""Control helpers: persistent node, IK -> trajectory, gripper, FK, joint state."""
import time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])

def quat_R(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

def topdown_quat(yaw):
    """Hand pointing straight down; yaw=0 -> fingers along world y, yaw=pi/2 -> along x."""
    return (np.cos(yaw/2), np.sin(yaw/2), 0.0, 0.0)

class Ctl:
    def __init__(self):
        rclpy.init(); self.node = rclpy.create_node("ctl")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(15); self.grip.wait_for_server(15)
        self.ik.wait_for_service(15); self.fk.wait_for_service(15)
        self.spin(0.5)
        while "pos" not in self.js: self.spin(0.2)

    def _js_cb(self, m):
        self.js["pos"] = dict(zip(m.name, m.position)); self.js["vel"] = dict(zip(m.name, m.velocity))

    def spin(self, t):
        end = time.time() + t
        while time.time() < end: rclpy.spin_once(self.node, timeout_sec=0.05)

    def q(self):
        self.spin(0.3); return np.array([self.js["pos"][j] for j in JOINTS])

    def fingers(self):
        self.spin(0.3); p = self.js["pos"]; return p["panda_finger_joint1"], p["panda_finger_joint2"]

    def fk_pose(self, q=None, link="panda_hand"):
        q = self.q() if q is None else q
        req = GetPositionFK.Request(); req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        ps = fut.result().pose_stamped[0].pose
        p = np.array([ps.position.x, ps.position.y, ps.position.z])
        o = np.array([ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w])
        return p, o

    def tcp_pose(self, q=None):
        p, o = self.fk_pose(q); R = quat_R(*o); return p + TCP*R[:, 2], o

    def solve_ik(self, xyz, quat, at_tcp=True, seed=None, tries=5):
        xyz = np.array(xyz, float)
        if at_tcp: xyz = xyz - TCP*quat_R(*quat)[:, 2]
        seed = self.q() if seed is None else np.array(seed)
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, xyz)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = JOINTS
            s = seed if k == 0 else seed + np.random.uniform(-0.3, 0.3, 7)
            req.ik_request.robot_state.joint_state.position = [float(x) for x in s]
            req.ik_request.timeout.sec = 1
            fut = self.ik.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                qs = np.array([sol[j] for j in JOINTS])
                # verify by FK
                pt, _ = self.tcp_pose(qs) if at_tcp else self.fk_pose(qs)
                tgt = xyz + (TCP*quat_R(*quat)[:, 2] if at_tcp else 0)
                err = np.linalg.norm(pt - tgt)
                if err < 0.005: return qs
                print(f"IK fk-mismatch {err:.4f}, retry")
            else:
                print("IK failed", None if r is None else r.error_code.val, "retry", k)
        return None

    def move_q(self, qt, seconds, tol=0.01):
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
        pt = JointTrajectoryPoint(positions=[float(x) for x in qt])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1)*1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(self.q() - qt).max()
        print(f"  traj code={code} max joint err={err:.4f}")
        return code == 0 and err < tol

    def move_tcp(self, xyz, yaw=0.0, seconds=3.0, seed=None):
        quat = topdown_quat(yaw)
        qs = self.solve_ik(xyz, quat, seed=seed)
        if qs is None: print("  NO IK for", xyz); return False
        ok = self.move_q(qs, seconds)
        p, _ = self.tcp_pose()
        print(f"  tcp now {np.round(p,4)} target {np.round(xyz,4)} err={np.linalg.norm(p-np.array(xyz)):.4f}")
        return ok

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def close(self): rclpy.shutdown()
EOF
cat > test_ik.py <<'EOF'
from ctl import *
c=Ctl()
print("q",np.round(c.q(),3)); p,o=c.tcp_pose(); print("tcp",np.round(p,4),np.round(o,4))
for tgt,yaw in [((-0.078,-0.189,0.65),0.0),((-0.078,-0.189,0.523),0.0),((-0.006,0.307,0.65),0.0),((-0.03,0.155,0.52),0.0)]:
    qs=c.solve_ik(tgt,topdown_quat(yaw))
    print(tgt, None if qs is None else np.round(qs,3))
c.close()
EOF
timeout 300 python3 test_ik.py

# openrua op 20
cat > step1.py <<'EOF'
from ctl import *
c=Ctl()
ok=c.move_tcp((-0.078,-0.187,0.65), yaw=0.0, seconds=3.0)
print("hover ok",ok)
c.close()
EOF
timeout 600 python3 step1.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png

# openrua op 21
timeout 60 python3 fk.py

# openrua op 22
timeout 600 python3 step1.py && timeout 60 python3 fk.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png

# openrua op 23
python3 - <<'EOF'
s=open("ctl.py").read()
s=s.replace('''def topdown_quat(yaw):
    """Hand pointing straight down; yaw=0 -> fingers along world y, yaw=pi/2 -> along x."""
    return (np.cos(yaw/2), np.sin(yaw/2), 0.0, 0.0)
''','''def qmul(a, b):
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)

# IK/FK tip link is panda_link8; panda_hand = link8 rotated -45 deg about z
Q_L8_FROM_HAND = (0.0, 0.0, np.sin(np.pi/8), np.cos(np.pi/8))

def topdown_quat(yaw):
    """HAND pointing straight down; yaw=0 -> fingers along world y, yaw=pi/2 -> along x.
    Returned as the panda_link8 orientation the IK service expects."""
    q_hand = (np.cos(yaw/2), np.sin(yaw/2), 0.0, 0.0)
    return qmul(q_hand, Q_L8_FROM_HAND)
''')
open("ctl.py","w").write(s)
EOF
cat > test_ik2.py <<'EOF'
from ctl import *
c=Ctl()
for yaw in (0.0, np.pi/2):
    qs=c.solve_ik((-0.078,-0.187,0.65),topdown_quat(yaw))
    p,o=c.fk_pose(qs); R=quat_R(*o)
    print("yaw",yaw,"q",np.round(qs,3),"hand quat",np.round(o,3),"finger axis (hand y) in world",np.round(R[:,1],3),"hand z",np.round(R[:,2],3))
c.close()
EOF
timeout 300 python3 test_ik2.py

# openrua op 24
timeout 600 python3 step1.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png

# openrua op 25
python3 - <<'EOF'
s=open("ctl.py").read()
s=s.replace('''    def move_q(self, qt, seconds, tol=0.01):
        goal''','''    def move_q(self, qt, seconds, tol=0.01, retries=2):
        for k in range(retries+1):
            ok = self._move_q_once(qt, seconds, tol)
            if ok: return True
            print("  retrying trajectory (controller lag)")
        return False

    def _move_q_once(self, qt, seconds, tol):
        goal''')
open("ctl.py","w").write(s)
EOF
timeout 600 python3 step1.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png

# openrua op 26
cat > step2.py <<'EOF'
from ctl import *
c=Ctl()
p,_=c.tcp_pose(); print("tcp before",np.round(p,4))
ok=c.move_tcp((-0.078,-0.187,0.518), yaw=0.0, seconds=2.5); print("descend ok",ok)
f=c.gripper(0.0)
print("fingers after close", f)
c.close()
EOF
timeout 900 python3 step2.py && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/av2.png

# openrua op 27
cat > step3.py <<'EOF'
from ctl import *
c=Ctl()
ok=c.move_tcp((-0.078,-0.187,0.66), yaw=0.0, seconds=2.5); print("lift ok",ok)
print("fingers", c.fingers())
c.close()
EOF
timeout 900 python3 step3.py && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/av3.png

# openrua op 28
cat > step4.py <<'EOF'
from ctl import *
c=Ctl()
# mug centre sits +0.046 y from the TCP (grasped on the -y wall)
PLATE=(-0.006,0.307); off=0.046
tx,ty=PLATE[0],PLATE[1]-off
print("raise", c.move_tcp((-0.078,-0.187,0.75), yaw=0.0, seconds=2.0))
print("over plate", c.move_tcp((tx,ty,0.75), yaw=0.0, seconds=3.5))
print("fingers", c.fingers())
print("lower", c.move_tcp((tx,ty,0.55), yaw=0.0, seconds=3.0))
print("fingers", c.fingers())
c.close()
EOF
timeout 1200 python3 step4.py && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/av4.png

# openrua op 29
cat > step5.py <<'EOF'
from ctl import *
c=Ctl()
c.gripper(0.04)
print("retreat", c.move_tcp((-0.006,0.261,0.72), yaw=0.0, seconds=2.5))
c.close()
EOF
timeout 900 python3 step5.py && timeout 60 python3 seg.py birdview && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/av5.png

# openrua op 30
cat > step6.py <<'EOF'
from ctl import *
c=Ctl()
print("hover yellow", c.move_tcp((-0.03,0.105,0.70), yaw=0.0, seconds=3.0))
c.close()
EOF
timeout 900 python3 step6.py && timeout 60 python3 - <<'EOF'
import numpy as np, cv2
from percep import cloud
color,depth,P=cloud("robot0_eye_in_hand")
cv2.imwrite("snaps/eih6.png",color)
m=(P[...,0]>-0.12)&(P[...,0]<0.06)&(P[...,1]>0.0)&(P[...,1]<0.2)&(P[...,2]>0.435)&(P[...,2]<0.65)
z=P[m][:,2]
h,e=np.histogram(z,bins=np.arange(0.43,0.65,0.01)); print(" ".join(f"{ee:.2f}:{hh}" for hh,ee in zip(h,e) if hh))
zmax=z.max(); print("zmax",zmax)
rim=m&(P[...,2]>zmax-0.012); pts=P[rim][:,:2]
# fit circle (algebraic)
A=np.c_[2*pts[:,0],2*pts[:,1],np.ones(len(pts))]; b=(pts**2).sum(1)
cx,cy,cc=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(cc+cx*cx+cy*cy)
print("rim circle fit centre",round(cx,4),round(cy,4),"r",round(r,4),"n",len(pts))
print("rim extents x",pts[:,0].min(),pts[:,0].max(),"y",pts[:,1].min(),pts[:,1].max())
# handle: points at mid height outside the body circle
mid=m&(P[...,2]>0.47)&(P[...,2]<0.52); pm=P[mid][:,:2]
d=np.hypot(pm[:,0]-cx,pm[:,1]-cy); out=pm[d>r+0.005]
if len(out): print("handle-ish pts",len(out),"mean",out.mean(0))
EOF

# openrua op 31
cat > step7.py <<'EOF'
from ctl import *
c=Ctl()
GX,GY=-0.030,0.148
print("hover", c.move_tcp((GX,GY,0.66), yaw=0.0, seconds=2.5))
print("descend", c.move_tcp((GX,GY,0.517), yaw=0.0, seconds=2.5))
f=c.gripper(0.0)
print("lift", c.move_tcp((GX,GY,0.66), yaw=0.0, seconds=2.5))
print("fingers after lift", c.fingers())
c.close()
EOF
timeout 1200 python3 step7.py && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/av7.png

# openrua op 32
cat > step8.py <<'EOF'
from ctl import *
c=Ctl()
PLATE=(-0.013,-0.281); off=0.046   # mug centre sits -0.046 y from the TCP (grasped on +y wall)
tx,ty=PLATE[0],PLATE[1]+off
print("raise", c.move_tcp((-0.03,0.148,0.75), yaw=0.0, seconds=2.0))
print("over plate", c.move_tcp((tx,ty,0.75), yaw=0.0, seconds=3.5))
print("fingers", c.fingers())
print("lower", c.move_tcp((tx,ty,0.55), yaw=0.0, seconds=3.0))
print("fingers", c.fingers())
c.gripper(0.04)
print("retreat", c.move_tcp((tx,ty,0.72), yaw=0.0, seconds=2.5))
c.close()
EOF
timeout 1500 python3 step8.py && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/av8.png

# openrua op 33
cat > step9.py <<'EOF'
from ctl import *
c=Ctl()
print("park", c.move_tcp((-0.15,0.0,0.75), yaw=0.0, seconds=3.0))
c.close()
EOF
timeout 900 python3 step9.py && timeout 60 python3 seg.py birdview && timeout 60 python3 - <<'EOF'
import numpy as np
P=np.load("snaps/birdview_P.npy")
for name,(cx,cy) in {"left plate":(-0.013,-0.281),"right plate":(-0.006,0.307)}.items():
    m=(np.hypot(P[...,0]-cx,P[...,1]-cy)<0.09)&(P[...,2]>0.47)&(P[...,2]<0.62)
    pts=P[m]
    print(name,"mug pts above plate:",len(pts),"centroid xy",np.round(pts[:,:2].mean(0),3),"offset from plate centre",np.round(np.hypot(*(pts[:,:2].mean(0)-[cx,cy])),3),"zmax",round(pts[:,2].max(),3))
EOF

# openrua op 34
timeout 60 python3 tools/perception/cam_snap.py birdview snaps/bird_final.png && timeout 60 python3 tools/perception/cam_snap.py agentview snaps/av_final.png
