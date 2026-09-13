#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ---; timeout 20 ros2 node list

# openrua op 3
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 4
cat > tfdump.py <<'EOF'
import rclpy, sys
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = rclpy.create_node("tfdump")
buf = Buffer(); TransformListener(buf, n)
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.2)
print(buf.all_frames_as_string())
pairs = [("world","panda_link0"),("world","birdview_optical_frame"),("world","agentview_optical_frame"),("world","sideview_optical_frame"),("world","frontview_optical_frame"),("panda_link0","panda_hand")]
for a,b in pairs:
    try:
        t = buf.lookup_transform(a,b,Time())
        tr=t.transform.translation; q=t.transform.rotation
        print(a,"->",b, f"t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
    except Exception as e: print(a,"->",b,"FAIL",e)
EOF
timeout 60 python3 tfdump.py

# openrua op 5
cat > depthmap.py <<'EOF'
"""Grab depth+info+TF for a camera once; save world-XYZ map as npy."""
import sys, numpy as np, rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
cam = sys.argv[1]
rclpy.init(); n = rclpy.create_node("dm"); buf=Buffer(); TransformListener(buf,n)
got={}
n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d",m),1)
n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i",m),1)
while len(got)<2 or not buf.can_transform("world", f"{cam}_optical_frame", Time()):
    rclpy.spin_once(n, timeout_sec=0.2)
d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
k = got["i"].k; fx,fy,cx,cy = k[0],k[4],k[2],k[5]
H,W = d.shape
u,v = np.meshgrid(np.arange(W), np.arange(H))
X = (u-cx)*d/fx; Y=(v-cy)*d/fy; Z=d
t = buf.lookup_transform("world", f"{cam}_optical_frame", Time())
q=t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
tr = np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
P = np.stack([X,Y,Z],-1) @ R.T + tr
np.save(f"{cam}_xyz.npy", P)
print(cam, d.shape, "fx",fx,"depth range",np.nanmin(d),np.nanmax(d))
EOF
timeout 120 python3 depthmap.py birdview && python3 - <<'EOF'
import numpy as np
P=np.load("birdview_xyz.npy")
for (u,v,name) in [(400,380,"table"),(328,293,"yellow mug"),(420,297,"grey mug"),(250,270,"microwave top"),(190,225,"door"),(320,100,"floor?")]:
    print(name,(u,v),np.round(P[v,u],3))
EOF

# openrua op 6
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_xyz.npy")
Z=P[...,2]
def ext(mask,name):
    pts=P[mask]
    if len(pts)==0: print(name,"none"); return
    print(f"{name}: n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
# things above the table (z>0.92) within table region
above = (Z>0.92)&(P[...,0]>-0.35)&(P[...,0]<0.5)&(np.abs(P[...,1])<0.6)
ext(above&(P[...,1]>-0.1)&(P[...,1]<0.15),"yellow mug")
ext(above&(P[...,1]>0.25)&(P[...,1]<0.45),"grey mug")
ext(above&(P[...,1]<-0.1)&(Z>1.05),"microwave+door top")
ext(above&(P[...,1]<-0.1)&(Z>1.05)&(P[...,0]>-0.2),"microwave body top (x>-0.2)")
ext(above&(P[...,1]<-0.1)&(Z>1.05)&(P[...,0]<-0.2),"door top (x<-0.2)")
# microwave body: pixel rows
mask=above&(P[...,1]<-0.1)&(Z>1.05)
vs,us=np.where(mask)
print("pixel bbox u",us.min(),us.max(),"v",vs.min(),vs.max())
# per-column analysis of body
for v in range(215,330,10):
    row=mask[v]; 
    if row.any():
        u=np.where(row)[0]; print(v, u.min(), u.max(), np.round(P[v,u.min()],3), np.round(P[v,u.max()],3))
EOF

# openrua op 7
cat > tf1.py <<'EOF'
import rclpy, sys
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init(); n = rclpy.create_node("tf1")
buf = Buffer(); TransformListener(buf, n)
for _ in range(15): rclpy.spin_once(n, timeout_sec=0.2)
for b in sys.argv[1:]:
    t = buf.lookup_transform("world",b,Time()); tr=t.transform.translation; q=t.transform.rotation
    print(b, f"t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
timeout 60 python3 tf1.py robot0_robotview_optical_frame robot0_eye_in_hand_optical_frame galleryview_optical_frame paperview_optical_frame panda_hand panda_leftfinger panda_rightfinger

# openrua op 8
sed -i 's/for _ in range(15): rclpy.spin_once(n, timeout_sec=0.2)/import time\nwhile not buf.can_transform("world","panda_hand",Time()) or not buf.can_transform("world","robot0_robotview_optical_frame",Time()): rclpy.spin_once(n, timeout_sec=0.2)/' tf1.py && timeout 60 python3 tf1.py robot0_robotview_optical_frame robot0_eye_in_hand_optical_frame galleryview_optical_frame paperview_optical_frame panda_hand panda_leftfinger panda_rightfinger

# openrua op 9
cat > urdf.py <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node("u")
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]
n.create_subscription(String,"/robot_description",got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open("robot.urdf","w").write(got[0].data); print(len(got[0].data))
EOF
timeout 60 python3 urdf.py && grep -n -A12 '<link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A10 'name="panda_hand_joint"\|name="panda_leftfinger"\|name="panda_hand_tcp' robot.urdf | head -80

# openrua op 10
python3 - <<'EOF'
import numpy as np, struct
def stl_bbox(fn):
    d=open(fn,'rb').read()
    n=struct.unpack_from('<I',d,80)[0]
    v=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ["hand","finger","link7"]:
    print(f, stl_bbox(f"/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl"))
EOF

# openrua op 11
find / -name "hand.stl" -o -name "hand*.obj" 2>/dev/null | grep -v proc | head

# openrua op 12
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small arm helper: FK/IK, trajectories, gripper, camera grabs. Reuses clients."""
import sys, time, math
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState, Image, CameraInfo
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (TF)
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()  # x,y,z,w


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.n = rclpy.create_node("arm_helper")
        self.js = None
        self.n.create_subscription(JointState, "/joint_states", self._js, 1)
        self.fjt = ActionClient(self.n, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.n, GripperCommand, GRIP["port"])
        self.ik = self.n.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.n.create_client(GetPositionFK, "/compute_fk")
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self.n)
        self.fjt.wait_for_server(10); self.gr.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        while self.js is None:
            rclpy.spin_once(self.n, timeout_sec=0.2)

    def _js(self, m):
        self.js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.n, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self.js = None
            while self.js is None:
                self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in JOINTS]

    def fingers(self, fresh=True):
        if fresh:
            self.js = None
            while self.js is None:
                self.spin(0.2)
        d = dict(zip(self.js.name, self.js.position))
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def seed_state(self, q):
        s = JointState(); s.name = list(JOINTS); s.position = [float(v) for v in q]
        return s

    def fk_hand(self, q=None):
        """Return (pos_world, quat) of panda_hand for joint vector q (default current)."""
        q = self.joints() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.seed_state(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if r is None else r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def ik_hand(self, pos_world, quat, seed=None, at_tcp=False, tries=1):
        """IK for panda_hand at world pose. If at_tcp, pos is the fingertip point."""
        pos = np.array(pos_world, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        pos_base = pos - BASE
        seed = self.joints() if seed is None else seed
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos_base)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state = self.seed_state(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 1
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
            seed = list(np.array(seed) + np.random.uniform(-0.3, 0.3, len(seed)))
        raise RuntimeError(f"IK failed at {pos_world} (code {None if r is None else r.error_code.val})")

    def move(self, waypoints, durations):
        """waypoints: list of joint vectors; durations: cumulative time_from_start per point."""
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(JOINTS)
        for q, t in zip(waypoints, durations):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            g.trajectory.points.append(pt)
        fut = self.fjt.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, fut)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res)
        code = res.result().result.error_code
        err = np.max(np.abs(np.array(self.joints()) - np.array(waypoints[-1])))
        print(f"move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.n, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"gripper -> pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def snap(self, cam, out=None):
        import cv2
        from cv_bridge import CvBridge
        got = {}
        sub = self.n.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("m", m), 1)
        while "m" not in got:
            self.spin(0.2)
        self.n.destroy_subscription(sub)
        out = out or f"{cam}.png"
        cv2.imwrite(out, CvBridge().imgmsg_to_cv2(got["m"], "bgr8"))
        return out

    def xyz_map(self, cam):
        """World-frame XYZ per pixel from the camera's depth."""
        from cv_bridge import CvBridge
        got = {}
        s1 = self.n.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
        s2 = self.n.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
        fr = f"{cam}_optical_frame"
        while len(got) < 2 or not self.tfbuf.can_transform("world", fr, Time()):
            self.spin(0.2)
        self.n.destroy_subscription(s1); self.n.destroy_subscription(s2)
        d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
        k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        H, W = d.shape
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
        t = self.tfbuf.lookup_transform("world", fr, Time())
        q = t.transform.rotation
        R = quat_to_R([q.x, q.y, q.z, q.w])
        tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        return pc @ R.T + tr

    def hand_tf(self):
        while not self.tfbuf.can_transform("world", "panda_hand", Time()):
            self.spin(0.2)
        t = self.tfbuf.lookup_transform("world", "panda_hand", Time())
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), np.array([q.x, q.y, q.z, q.w])


# handy orientations (hand frame -> world): quaternion x,y,z,w
Q_DOWN = np.array([1.0, 0.0, 0.0, 0.0])          # Z down, fingers along world y, hand X = +x
Q_FWD = np.array([0.0, 0.7071068, 0.0, 0.7071068])  # Z along +x (toward microwave), fingers along y
OPENRUA_EOF

# openrua op 13
timeout 120 python3 - <<'EOF'
from arm import *
a = Arm()
q = a.joints(); print("joints", np.round(q,3))
print("fingers", a.fingers())
pos, quat = a.fk_hand(q); print("FK hand world", np.round(pos,4), np.round(quat,4))
print("TF hand world", a.hand_tf())
# test IK for the look-in pose
for target in [(-0.45,-0.24,1.25),(-0.50,-0.24,1.05)]:
    try:
        sol = a.ik_hand(target, Q_FWD, seed=q, tries=3)
        p2,q2 = a.fk_hand(sol); print("IK ok", target, "->", np.round(sol,3), "FK", np.round(p2,3), np.round(q2,3))
    except Exception as e: print("IK", target, e)
EOF

# openrua op 14
timeout 120 python3 - <<'EOF'
from arm import *
import arm
a = Arm()
q = a.joints()
# world-frame hypothesis: pass world coords with BASE zeroed
arm.BASE = np.zeros(3)
try:
    sol = a.ik_hand((-0.203, 0.0, 1.2696), (0.9996,0,-0.0284,0), seed=q)
    print("world-frame IK ->", np.round(sol,3), "current", np.round(q,3))
except Exception as e: print("world-frame IK", e)
# base-frame hypothesis
try:
    sol = a.ik_hand((0.457, 0.0, 0.3576), (0.9996,0,-0.0284,0), seed=q)
    print("base-frame IK ->", np.round(sol,3))
except Exception as e: print("base-frame IK", e)
EOF

# openrua op 15
sed -i 's/^BASE = np.array(\[-0.66, 0.0, 0.912\]).*/BASE = np.zeros(3)  # FK\/IK services already return\/accept WORLD-frame poses (verified)/' arm.py && timeout 120 python3 - <<'EOF'
from arm import *
a = Arm()
q = a.joints()
sol = a.ik_hand((-0.203, 0.0, 1.2696), (0.9996,0,-0.0284,0), seed=q)
print("sol", np.round(sol,3)); print("FK(sol)", a.fk_hand(sol))
sol = a.ik_hand((-0.203, 0.0, 1.2696), Q_DOWN, seed=q)
print("sol Q_DOWN", np.round(sol,3)); print("FK(sol)", a.fk_hand(sol))
for target in [(-0.45,-0.24,1.25),(-0.50,-0.24,1.05)]:
    sol = a.ik_hand(target, Q_FWD, seed=q, tries=3)
    print("IK", target, np.round(sol,3), "FK", a.fk_hand(sol))
EOF

# openrua op 16
python3 - <<'EOF'
import re
s=open("arm.py").read()
s=s.replace('''            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
''','''            # IK targets panda_link8 = panda_hand rotated +45deg about hand Z (verified via FK)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, qmul(quat, ROTZ45))
''')
s=s.replace('''def R_to_quat(R):''','''def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return np.array([w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2,
                     w1*w2 - x1*x2 - y1*y2 - z1*z2])


ROTZ45 = np.array([0.0, 0.0, math.sin(math.pi/8), math.cos(math.pi/8)])


def R_to_quat(R):''')
open("arm.py","w").write(s)
EOF
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_xyz.npy"); Z=P[...,2]
m=(Z>0.91)&(Z<1.05)&(P[...,1]>-0.1)&(P[...,1]<0.15)&(P[...,0]>-0.15)&(P[...,0]<0.1)
pts=P[m]
print("yellow mug pts",len(pts),"x",pts[:,0].min(),pts[:,0].max(),"y",pts[:,1].min(),pts[:,1].max(),"z",pts[:,2].min(),pts[:,2].max())
# rim: highest points
rim=pts[pts[:,2]>0.965]
print("rim x",rim[:,0].min(),rim[:,0].max(),"y",rim[:,1].min(),rim[:,1].max(),"center",rim[:,0].mean(),rim[:,1].mean())
# body without handle: y> -0.02
body=pts[(pts[:,1]>-0.03)]
print("body(y>-0.03) x",body[:,0].min(),body[:,0].max(),"y",body[:,1].min(),body[:,1].max())
# histogram of z
h,e=np.histogram(pts[:,2],bins=14); print(list(zip(np.round(e,3),h)))
EOF

# openrua op 17
timeout 120 python3 depthmap.py frontview && timeout 120 python3 depthmap.py agentview && python3 - <<'EOF'
import numpy as np
for cam in ["frontview","agentview"]:
    P=np.load(f"{cam}_xyz.npy")
    m=(P[...,2]>0.905)&(P[...,2]<1.0)&(P[...,1]>-0.10)&(P[...,1]<0.12)&(P[...,0]>-0.15)&(P[...,0]<0.10)
    pts=P[m]
    print(cam,"mug pts",len(pts))
    for z0 in np.arange(0.905,0.985,0.01):
        s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
        if len(s)==0: continue
        body=s[s[:,1]>-0.005]
        print(f" z {z0:.3f}: n={len(s)} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] body y[{body[:,1].min() if len(body) else 0:.3f},{body[:,1].max() if len(body) else 0:.3f}] width={body[:,1].max()-body[:,1].min() if len(body) else 0:.3f} x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
EOF

# openrua op 18
timeout 120 python3 depthmap.py agentview 2>&1 | grep -v XMLPARSER && python3 - <<'EOF'
import numpy as np
for cam in ["frontview","agentview"]:
    P=np.load(f"{cam}_xyz.npy")
    m=(P[...,2]>0.905)&(P[...,2]<1.0)&(P[...,1]>-0.10)&(P[...,1]<0.12)&(P[...,0]>-0.15)&(P[...,0]<0.10)
    pts=P[m]
    print(cam,"mug pts",len(pts))
    for z0 in np.arange(0.905,0.985,0.01):
        s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
        if len(s)==0: continue
        body=s[s[:,1]>-0.005]
        print(f" z {z0:.3f}: n={len(s)} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] body y[{body[:,1].min() if len(body) else 0:.3f},{body[:,1].max() if len(body) else 0:.3f}] width={body[:,1].max()-body[:,1].min() if len(body) else 0:.3f} x[{s[:,0].min():.3f},{s[:,0].max():.3f}]")
EOF

# openrua op 19
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_xyz.npy"); Z=P[...,2]
# hand region: around (-0.203, 0), z between 1.15 and 1.35
m=(Z>1.15)&(Z<1.40)&(np.abs(P[...,0]+0.2)<0.2)&(np.abs(P[...,1])<0.2)
pts=P[m]
print("n",len(pts))
for z0 in np.arange(1.15,1.40,0.02):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.02)]
    if len(s): print(f"z {z0:.2f}-{z0+0.02:.2f}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 20
cat >> arm.py <<'EOF'


# ---------------- MoveIt planning with a populated scene ----------------
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (CollisionObject, PlanningScene, Constraints, JointConstraint,
                             PositionConstraint, OrientationConstraint, MotionPlanRequest, PlanningOptions)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


def _box(name, center, size, frame="world"):
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = name
    sp = SolidPrimitive(); sp.type = SolidPrimitive.BOX; sp.dimensions = [float(s) for s in size]
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center); p.orientation.w = 1.0
    co.primitives = [sp]; co.primitive_poses = [p]
    co.operation = CollisionObject.ADD
    return co


class Planner:
    def __init__(self, arm):
        self.a = arm
        self.n = arm.n
        self.mg = ActionClient(self.n, MoveGroup, M["planning"]["move_action"])
        self.aps = self.n.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.mg.wait_for_server(10); self.aps.wait_for_service(10)

    def set_scene(self, boxes, remove=()):
        ps = PlanningScene(); ps.is_diff = True
        for name, center, size in boxes:
            ps.world.collision_objects.append(_box(name, center, size))
        for name in remove:
            co = CollisionObject(); co.id = name; co.header.frame_id = "world"
            co.operation = CollisionObject.REMOVE
            ps.world.collision_objects.append(co)
        req = ApplyPlanningScene.Request(); req.scene = ps
        fut = self.aps.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        print("scene applied:", fut.result().success, flush=True)

    def go_joints(self, q, vel=0.3, plan_only=False):
        g = MoveGroup.Goal()
        r = g.request
        r.group_name = M["planning"]["group"]
        r.num_planning_attempts = 10
        r.allowed_planning_time = 10.0
        r.max_velocity_scaling_factor = vel
        r.max_acceleration_scaling_factor = 0.3
        r.start_state.joint_state = self.a.seed_state(self.a.joints())
        r.start_state.is_diff = True
        c = Constraints()
        for j, v in zip(JOINTS, q):
            jc = JointConstraint(joint_name=j, position=float(v), tolerance_above=0.005,
                                 tolerance_below=0.005, weight=1.0)
            c.joint_constraints.append(jc)
        r.goal_constraints = [c]
        g.planning_options.plan_only = plan_only
        g.planning_options.replan = False
        fut = self.mg.send_goal_async(g)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=60)
        gh = fut.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.n, res)
        rr = res.result().result
        err = np.max(np.abs(np.array(self.a.joints()) - np.array(q)))
        print(f"plan/exec code={rr.error_code.val} max_joint_err={err:.4f} "
              f"npts={len(rr.planned_trajectory.joint_trajectory.points)}", flush=True)
        return rr.error_code.val, err
EOF
timeout 300 python3 - <<'EOF'
from arm import *
a = Arm(); pl = Planner(a)
# scene: table, microwave body, door, grey mug, yellow mug
pl.set_scene([
  ("table", (0.0, 0.0, 0.85), (2.0, 2.0, 0.10)),
  ("microwave", (-0.085, -0.237, 1.005), (0.34, 0.23, 0.21)),
  ("door", (-0.267, -0.45, 1.0), (0.04, 0.25, 0.20)),
  ("grey_mug", (-0.006, 0.342, 0.94), (0.10, 0.13, 0.09)),
  ("yellow_mug", (-0.025, 0.03, 0.94), (0.09, 0.13, 0.09)),
])
q0 = a.joints()
import math
def pitched(theta_deg):
    t=math.radians(theta_deg)
    Zh=np.array([math.cos(t),0,-math.sin(t)]); Xh=np.array([math.sin(t),0,math.cos(t)]); Yh=np.cross(Zh,Xh)
    return R_to_quat(np.stack([Xh,Yh,Zh],1))
for pos,th in [((-0.60,-0.24,1.35),50),((-0.55,-0.24,1.30),50),((-0.53,-0.24,1.22),45)]:
    try:
        sol=a.ik_hand(pos, pitched(th), seed=q0, tries=5)
        print("cand",pos,th,np.round(sol,2), "FK", np.round(a.fk_hand(sol)[0],3))
    except Exception as e: print("fail",pos,th,e)
EOF

# openrua op 21
timeout 600 python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a = Arm(); pl = Planner(a)
q = [-0.84,-1.31,-0.63,-2.98,0.01,2.06,-0.36]
code, err = pl.go_joints(q, vel=0.5)
print("hand TF", a.hand_tf())
print(a.snap("robot0_eye_in_hand","look1.png"))
EOF

# openrua op 22
timeout 60 python3 -c "
from arm import *
a=Arm(); print(np.round(a.joints(),3)); print(a.fingers())
" 2>&1 | grep -v XMLPARSER

# openrua op 23
cat >> arm.py <<'EOF'

    def plan_exec(self, q, total_time=None, speed=0.6):
        """Plan collision-free path with MoveIt, then execute it through the FJT action.
        speed: rad/s bound used to set total duration (if total_time not given)."""
        code, _ = self.go_joints(q, plan_only=True)
        if code != 1:
            raise RuntimeError(f"planning failed code={code}")
        pts = self._last_traj.joint_trajectory.points
        wps = [list(p.positions) for p in pts]
        if len(wps) < 2:
            wps = [self.a.joints(), list(q)]
        # path length in joint space -> duration
        L = sum(np.max(np.abs(np.array(wps[i+1]) - np.array(wps[i]))) for i in range(len(wps)-1))
        T = total_time or max(2.0, L / speed)
        cum = [0.0]
        for i in range(len(wps)-1):
            cum.append(cum[-1] + np.max(np.abs(np.array(wps[i+1]) - np.array(wps[i]))))
        times = [0.5 + T * c / max(cum[-1], 1e-6) for c in cum]
        wps[-1] = list(q)
        return self.a.move(wps[1:], times[1:])
EOF
python3 - <<'EOF'
s=open("arm.py").read()
s=s.replace("""        rr = res.result().result
        err = np.max(np.abs(np.array(self.a.joints()) - np.array(q)))""","""        rr = res.result().result
        self._last_traj = rr.planned_trajectory
        err = np.max(np.abs(np.array(self.a.joints()) - np.array(q)))""")
open("arm.py","w").write(s)
EOF
timeout 600 python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a = Arm(); pl = Planner(a)
q = [-0.84,-1.31,-0.63,-2.98,0.01,2.06,-0.36]
pl.plan_exec(q)
print("joints", np.round(a.joints(),3))
print("hand TF", a.hand_tf())
print(a.snap("robot0_eye_in_hand","look1.png"))
EOF

# openrua op 24
timeout 600 python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a = Arm()
q = [-0.84,-1.31,-0.63,-2.98,0.01,2.06,-0.36]
a.move([q],[3.0])
print("joints", np.round(a.joints(),3))
print("hand TF", a.hand_tf())
print(a.snap("robot0_eye_in_hand","look1.png"))
EOF

# openrua op 25
timeout 120 python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
P=a.xyz_map("robot0_eye_in_hand"); np.save("eih_xyz.npy",P)
# probe a row across the front face at v=235 and a column at u=310
for u in range(240,390,10): print("u",u,np.round(P[235,u],3))
print("col")
for v in range(190,290,8): print("v",v,np.round(P[v,310],3))
EOF

# openrua op 26
python3 - <<'EOF'
import numpy as np
P=np.load("eih_xyz.npy")
for u in range(390,600,15): print("u",u,"v235",np.round(P[235,u],3),"v215",np.round(P[215,u],3))
# handle region
for (u,v) in [(560,215),(565,240),(545,260),(520,200)]: print("handle?",(u,v),np.round(P[v,u],3))
# scan interior of the window region: 
for v in range(200,280,10): print("v",v,"u470",np.round(P[v,470],3))
EOF

# openrua op 27
cat >> arm.py <<'EOF'


def look_quat(eye, target, up=(0, 0, 1)):
    """Hand quaternion with Z_h pointing from eye to target and Y_h horizontal."""
    z = np.array(target, float) - np.array(eye, float); z /= np.linalg.norm(z)
    y = np.cross(z, np.array(up, float));
    if np.linalg.norm(y) < 1e-6: y = np.array([0, 1.0, 0])
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return R_to_quat(np.stack([x, y, z], 1))
EOF
timeout 600 python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a = Arm(); pl = Planner(a)
q0=a.joints()
eye=(-0.45,-0.62,1.30); tgt=(-0.10,-0.34,0.98)
for e in [eye, (-0.40,-0.65,1.25), (-0.50,-0.58,1.35)]:
    try:
        sol=a.ik_hand(e, look_quat(e,tgt), seed=q0, tries=6)
        print("cand",e,np.round(sol,2)); break
    except Exception as ex: print("fail",e,ex)
pl.plan_exec(sol)
a.move([sol],[2.0])
print("hand TF", a.hand_tf())
print(a.snap("robot0_eye_in_hand","look2.png"))
P=a.xyz_map("robot0_eye_in_hand"); np.save("eih2_xyz.npy",P)
EOF

# openrua op 28
timeout 600 python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a = Arm()
sol=[1.07,-0.72,-2.18,-1.51,0.34,1.67,-1.13]
print("before", np.round(a.joints(),3))
a.move([sol],[5.0])
print("after", np.round(a.joints(),3))
print("hand TF", a.hand_tf())
print(a.snap("robot0_eye_in_hand","look2.png"))
P=a.xyz_map("robot0_eye_in_hand"); np.save("eih2_xyz.npy",P)
EOF

# openrua op 29
python3 - <<'EOF'
import numpy as np
P=np.load("eih2_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
# interior: points inside body footprint with y > -0.33 (behind the front face plane)
inside=(X>-0.27)&(X<0.09)&(Y>-0.33)&(Y<-0.10)&(Z>0.90)&(Z<1.12)
pts=P[inside]; print("interior pts",len(pts))
print("x",pts[:,0].min(),pts[:,0].max(),"y",pts[:,1].min(),pts[:,1].max(),"z",pts[:,2].min(),pts[:,2].max())
# floor of interior: low z points
fl=pts[pts[:,2]<0.96]; print("floor pts",len(fl),"z hist",np.histogram(fl[:,2],bins=6))
print("floor x",fl[:,0].min(),fl[:,0].max(),"y",fl[:,1].min(),fl[:,1].max())
# back wall (max y)
bw=pts[pts[:,1]>pts[:,1].max()-0.01]; print("back wall y~",bw[:,1].mean(),"x",bw[:,0].min(),bw[:,0].max(),"z",bw[:,2].min(),bw[:,2].max())
# front face plane: points with y in [-0.36,-0.33]
ff=P[(X>-0.27)&(X<0.10)&(Y>-0.37)&(Y<-0.33)&(Z>0.90)&(Z<1.12)]
print("front face pts",len(ff),"x",ff[:,0].min(),ff[:,0].max(),"z",ff[:,2].min(),ff[:,2].max(), "y mean",ff[:,1].mean())
# opening: where front-face plane is missing. histogram of x for front face points at z in 0.95-1.05
mid=ff[(ff[:,2]>0.95)&(ff[:,2]<1.05)]
print("front-face mid-height x hist",np.histogram(mid[:,0],bins=np.arange(-0.27,0.11,0.02)))
top=ff[(ff[:,2]>1.05)]; print("front face top strip z min", top[:,2].min())
# side walls of interior: x extremes at mid height
midi=pts[(pts[:,2]>0.95)&(pts[:,2]<1.05)]
print("interior mid-height x hist",np.histogram(midi[:,0],bins=np.arange(-0.27,0.11,0.02)))
print("interior z hist",np.histogram(pts[:,2],bins=np.arange(0.90,1.13,0.02)))
EOF

# openrua op 30
python3 - <<'EOF'
import numpy as np
P=np.load("eih2_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
ff=P[(X>-0.23)&(X<-0.03)&(Y>-0.37)&(Y<-0.32)&(Z>0.90)&(Z<1.12)]
print("front plane in opening x-range: z hist",np.histogram(ff[:,2],bins=np.arange(0.90,1.13,0.01)))
inside=P[(X>-0.23)&(X<-0.03)&(Y>-0.32)&(Y<-0.10)&(Z>0.90)&(Z<1.12)]
print("interior z hist",np.histogram(inside[:,2],bins=np.arange(0.90,1.13,0.01)))
fl=inside[(inside[:,2]>0.93)&(inside[:,2]<0.96)]
print("floor y range",fl[:,1].min(),fl[:,1].max(),"x",fl[:,0].min(),fl[:,0].max())
lo=inside[inside[:,2]<0.92]; print("low pts y",lo[:,1].min() if len(lo) else None,lo[:,1].max() if len(lo) else None, "x",lo[:,0].min() if len(lo) else None,lo[:,0].max() if len(lo) else None)
# ceiling: points at z 1.0-1.09 inside
ce=inside[(inside[:,2]>1.0)&(inside[:,2]<1.095)]; print("ceiling-ish pts",len(ce), "z", np.round(np.percentile(ce[:,2],[5,50,95]),3) if len(ce) else None, "y",ce[:,1].min() if len(ce) else None, ce[:,1].max() if len(ce) else None)
EOF

# openrua op 31
timeout 600 python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a = Arm(); pl = Planner(a)
q0=a.joints()
sol=a.ik_hand((-0.075, 0.036, 1.30), Q_DOWN, seed=[0,-0.3,0,-2.2,0,1.9,0.785], tries=6)
print("sol",np.round(sol,2))
pl.plan_exec(sol)
a.move([sol],[3.0])
print("hand TF", a.hand_tf())
print(a.snap("robot0_eye_in_hand","look3.png"))
P=a.xyz_map("robot0_eye_in_hand"); np.save("eih3_xyz.npy",P)
EOF

# openrua op 32
python3 - <<'EOF'
s=open("arm.py").read()
s=s.replace('''        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(JOINTS)
        for q, t in zip(waypoints, durations):''','''        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(JOINTS)
        # controller tracks at ~0.2 rad/s: stretch timing so each segment is feasible
        prev = np.array(self.joints()); tprev = 0.0; fixed = []
        for q, t in zip(waypoints, durations):
            need = np.max(np.abs(np.array(q) - prev)) / MAX_RATE + 0.3
            t = max(t, tprev + need)
            fixed.append(t); prev = np.array(q); tprev = t
        durations = fixed
        for q, t in zip(waypoints, durations):''')
s=s.replace("TCP = M[\"hand\"][\"tcp_offset_m\"]","TCP = M[\"hand\"][\"tcp_offset_m\"]\nMAX_RATE = 0.2  # rad/s the FJT controller actually achieves (measured)")
open("arm.py","w").write(s)
EOF
timeout 900 python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a = Arm()
sol=[0.02,0.15,0.04,-1.99,-0.01,2.14,0.85]
a.move([sol],[1.0])
print("joints",np.round(a.joints(),3)); print("hand TF", a.hand_tf())
print(a.snap("robot0_eye_in_hand","look3.png"))
P=a.xyz_map("robot0_eye_in_hand"); np.save("eih3_xyz.npy",P)
EOF

# openrua op 33
python3 - <<'EOF'
import numpy as np
P=np.load("eih3_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(Z>0.905)&(Z<1.0)&(np.abs(X+0.025)<0.09)&(np.abs(Y-0.03)<0.10)
pts=P[m]; print("mug pts",len(pts),"z max",pts[:,2].max())
rim=pts[pts[:,2]>0.97]; body=rim[rim[:,1]>-0.0]
print("rim all x",rim[:,0].min(),rim[:,0].max(),"y",rim[:,1].min(),rim[:,1].max())
print("rim body x",body[:,0].min(),body[:,0].max(),"y",body[:,1].min(),body[:,1].max())
cx=(body[:,0].min()+body[:,0].max())/2; cy=body[:,1].max()-(body[:,0].max()-body[:,0].min())/2
print("center est",cx,cy,"diam",body[:,0].max()-body[:,0].min())
# circle fit on rim body points (x,y)
A=np.c_[2*body[:,0],2*body[:,1],np.ones(len(body))]; b=body[:,0]**2+body[:,1]**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2); print("circle fit center",c[0],c[1],"r",r)
# inner floor of mug (looking inside): z hist
print(np.histogram(pts[:,2],bins=np.arange(0.90,1.0,0.01)))
# side wall visible? points at z between 0.91 and 0.96 outside the rim circle radius
side=pts[(pts[:,2]<0.965)]
d=np.hypot(side[:,0]-c[0],side[:,1]-c[1]); print("lower pts radial dist percentiles",np.percentile(d,[5,50,95]) if len(d) else None)
for z0 in np.arange(0.90,0.98,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s): 
        dd=np.hypot(s[:,0]-c[0],s[:,1]-c[1]); print(f"z {z0:.2f}: n={len(s)} r max {dd.max():.3f} r med {np.median(dd):.3f}")
EOF

# openrua op 34
python3 - <<'EOF'
import numpy as np
P=np.load("eih3_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
c=(-0.02272,0.03732)
m=(Z>0.905)&(Z<1.0)&(np.abs(X+0.025)<0.09)&(np.abs(Y-0.03)<0.10)
pts=P[m]
rim=pts[(pts[:,2]>0.97)&(pts[:,1]>0.0)]
d=np.hypot(rim[:,0]-c[0],rim[:,1]-c[1])
print("rim radial percentiles",np.round(np.percentile(d,[1,5,25,50,75,95,99]),4))
print("rim z percentiles",np.round(np.percentile(rim[:,2],[5,50,95]),4))
# pixel footprint: where is the mug in the image
vs,us=np.where(m); print("pixel bbox u",us.min(),us.max(),"v",vs.min(),vs.max())
# check depth scale: table z around the mug
tab=P[(np.abs(X+0.025)<0.15)&(np.abs(Y-0.03)<0.15)&(Z<0.905)]
print("table z",np.round(np.percentile(tab[:,2],[5,50,95]),4))
# also check the handle extent
h=pts[(pts[:,1]<0.0)&(pts[:,2]>0.93)]; print("handle x",h[:,0].min(),h[:,0].max(),"y",h[:,1].min(),h[:,1].max(),"z",h[:,2].min(),h[:,2].max())
EOF

# openrua op 35
python3 - <<'EOF'
import numpy as np
for cam in ["frontview","agentview","birdview"]:
    P=np.load(f"{cam}_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(Z>0.905)&(Z<1.05)&(np.abs(X+0.02)<0.08)&(np.abs(Y-0.03)<0.09)
    pts=P[m]
    print(cam,len(pts),np.histogram(pts[:,2],bins=np.arange(0.90,1.06,0.01)))
    top=pts[pts[:,2]>np.percentile(pts[:,2],90)]; print(" top z",np.round(np.percentile(pts[:,2],[50,90,99,100]),3))
EOF

# openrua op 36
python3 - <<'EOF'
import numpy as np
for cam in ["frontview","agentview"]:
    P=np.load(f"{cam}_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
    tab=P[(X>0.1)&(X<0.4)&(np.abs(Y)<0.3)&(Z<0.95)]; print(cam,"table z",np.round(np.percentile(tab[:,2],[5,50,95]),3))
    mw=P[(X>-0.2)&(X<0.05)&(Y>-0.33)&(Y<-0.15)&(Z>1.0)]; print(cam,"microwave top z",np.round(np.percentile(mw[:,2],[5,50,95]),3), "x range", mw[:,0].min(), mw[:,0].max())
    # grey mug
    g=P[(np.abs(X+0.006)<0.08)&(np.abs(Y-0.342)<0.08)&(Z>0.905)&(Z<1.2)]; print(cam,"grey mug z",np.round(np.percentile(g[:,2],[1,50,99]),3))
    m=P[(np.abs(X+0.02)<0.08)&(np.abs(Y-0.03)<0.09)&(Z>0.905)&(Z<1.2)]; print(cam,"yellow mug z",np.round(np.percentile(m[:,2],[1,50,99]),3),"x",np.round(np.percentile(m[:,0],[1,50,99]),3))
EOF

# openrua op 37
python3 - <<'EOF'
import numpy as np
P=np.load("frontview_xyz.npy")
for (u,v) in [(335,300),(335,320),(335,340),(335,355),(335,362),(335,370),(300,340),(370,340),(505,330),(505,300)]:
    print((u,v),np.round(P[v,u],3))
EOF

# openrua op 38
for c in birdview frontview robot0_eye_in_hand agentview; do echo "== $c"; timeout 30 ros2 topic echo /$c/color/camera_info --once 2>/dev/null | grep -A3 "^k:" ; timeout 30 ros2 topic echo /$c/color/camera_info --once 2>/dev/null | grep -E "^(height|width)"; done

# openrua op 39
python3 - <<'EOF'
import numpy as np
P=np.load("eih3_xyz.npy")
v=237
print("row v=237: u -> (x,y,z)")
for u in range(250,400,4): print(u, np.round(P[v,u],3))
EOF

# openrua op 40
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_xyz.npy")
# find row closest to x=-0.022 : v where P[v,328,0] ~ -0.022
col=P[:,328,0]; v=int(np.argmin(np.abs(col+0.022))); print("row",v)
for u in range(312,350,1): print(u, np.round(P[v,u],3))
EOF

# openrua op 41
python3 - <<'EOF'
import numpy as np
for cam in ["frontview","agentview"]:
    P=np.load(f"{cam}_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
    m=(Z>0.97)&(Z<1.09)&(np.abs(X+0.02)<0.08)&(Y>-0.005)&(Y<0.12)
    pts=P[m]
    print(cam)
    for z0 in np.arange(0.975,1.08,0.01):
        s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
        if len(s)>5:
            print(f"  z {z0:.3f} (h={z0-0.978:.3f}): n={len(s)} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] width={s[:,1].max()-s[:,1].min():.3f} xmax={s[:,0].max():.3f}")
EOF

# openrua op 42
python3 - <<'EOF'
import numpy as np
P=np.load("eih2_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
inside=(Y>-0.33)&(Y<-0.15)&(Z>0.90)&(Z<1.10)
L=P[inside&(X>-0.28)&(X<-0.24)]; R=P[inside&(X>-0.05)&(X<0.0)]
print("left wall pts",len(L),"x pct",np.round(np.percentile(L[:,0],[5,50,95]),3),"z range",L[:,2].min(),L[:,2].max(),"y range",L[:,1].min(),L[:,1].max())
print("right wall pts",len(R),"x pct",np.round(np.percentile(R[:,0],[5,50,95]),3),"z range",R[:,2].min(),R[:,2].max(),"y range",R[:,1].min(),R[:,1].max())
# ceiling? points with z>1.06 and y>-0.33 and x in (-0.24,-0.04)
C=P[inside&(X>-0.24)&(X<-0.04)&(Z>1.0)]; print("ceiling pts",len(C), np.round(np.percentile(C[:,2],[5,50,95]),3) if len(C) else None)
# floor extent
F=P[inside&(Z>0.93)&(Z<0.96)]; print("floor x",F[:,0].min(),F[:,0].max(),"y",F[:,1].min(),F[:,1].max(),"z med",np.median(F[:,2]))
# back wall: points with y > -0.20 inside
B=P[inside&(Y>-0.20)&(X>-0.24)&(X<-0.04)]; print("back pts",len(B),"y pct",np.round(np.percentile(B[:,1],[5,50,95]),3) if len(B) else None,"z",B[:,2].min() if len(B) else None,B[:,2].max() if len(B) else None)
EOF

# openrua op 43
timeout 600 python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a)
eye=np.array([-0.33,0.037,1.06]); tgt=np.array([-0.023,0.037,0.93])
q=look_quat(eye,tgt)
sol=a.ik_hand(eye,q,seed=a.joints(),tries=8)
print("sol",sol)
if sol is not None:
    print(pl.plan_exec(sol)); a.move([sol],[1.0])
    print("hand",a.fk_hand())
    a.snap("robot0_eye_in_hand","look4.png")
    np.save("eih4_xyz.npy",a.xyz_map("robot0_eye_in_hand"))
EOF

# openrua op 44
timeout 600 python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a)
for eye in [[-0.36,0.06,1.10],[-0.38,0.10,1.12],[-0.40,0.037,1.15],[-0.36,0.15,1.10]]:
    eye=np.array(eye); tgt=np.array([-0.023,0.037,0.93])
    q=look_quat(eye,tgt)
    try:
        sol=a.ik_hand(eye,q,seed=a.joints(),tries=8)
    except Exception as e:
        print(eye,"fail"); continue
    print("ok",eye,sol)
    print(pl.plan_exec(sol)); a.move([sol],[1.0])
    print("hand",a.fk_hand())
    a.snap("robot0_eye_in_hand","look4.png")
    np.save("eih4_xyz.npy",a.xyz_map("robot0_eye_in_hand"))
    break
EOF

# openrua op 45
python3 - <<'EOF'
import numpy as np
P=np.load("eih4_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.10)&(X<0.05)&(Y>-0.02)&(Y<0.12)&(Z>0.90)&(Z<1.05)&np.isfinite(Z)
pts=P[m]
print("n",len(pts),"z range",pts[:,2].min(),pts[:,2].max())
for z0 in np.arange(0.9025,1.01,0.005):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.005)]
    if len(s)<5: continue
    ymin,ymax=s[:,1].min(),s[:,1].max()
    # nearest x (camera side is -x): the point closest to camera gives x_c - r
    xmin=s[:,0].min()
    print(f"z {z0:.4f} h={z0-0.90:.3f} n={len(s):4d} y[{ymin:.4f},{ymax:.4f}] width={ymax-ymin:.4f} yc={(ymin+ymax)/2:.4f} xmin={xmin:.4f}")
EOF

# openrua op 46
python3 - <<'EOF'
import numpy as np
P=np.load("eih4_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.10)&(X<0.05)&(Y>-0.08)&(Y<0.0)&(Z>0.905)&(Z<1.05)&np.isfinite(Z)
pts=P[m]
print("handle-side n",len(pts))
for z0 in np.arange(0.9075,1.01,0.005):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.005)]
    if len(s)<3: continue
    print(f"z {z0:.4f} h={z0-0.90:.3f} n={len(s):4d} y[{s[:,1].min():.4f},{s[:,1].max():.4f}] x[{s[:,0].min():.4f},{s[:,0].max():.4f}]")
EOF

# openrua op 47
python3 - <<'EOF'
import numpy as np
P=np.load("eih3_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(Y>-0.06)&(Y<-0.012)&(Z>0.97)&(Z<1.0)&(X>-0.12)&(X<0.08)&np.isfinite(Z)
s=P[m]; print("top bar n",len(s),"x pct",np.round(np.percentile(s[:,0],[2,10,50,90,98]),4),"y",np.round(np.percentile(s[:,1],[2,50,98]),4),"z",np.round(np.percentile(s[:,2],[2,50,98]),4))
# rim center from eih3: points z>0.995 near mug
r=P[(Z>0.995)&(Z<1.01)&(X>-0.1)&(X<0.06)&(Y>-0.02)&(Y<0.1)]
print("rim n",len(r),"x",r[:,0].min(),r[:,0].max(),"y",r[:,1].min(),r[:,1].max())
EOF

# openrua op 48
grep -n "def R_to_quat\|def look_quat\|def set_scene\|def plan_exec\|def _box\|def go_joints" arm.py && sed -n '/def R_to_quat/,/^$/p' arm.py && sed -n '/def set_scene/,/def go_joints/p' arm.py

# openrua op 49
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a)
phi=np.deg2rad(20); c,s=np.cos(phi),np.sin(phi)
R=np.array([[0,1,0],[c,0,s],[s,0,-c]]); q=R_to_quat(R)
Zh=R[:,2]
tcp=np.array([-0.0235,-0.0415,0.95])
pre=tcp-0.06*Zh
print("pre tcp",pre,"grasp tcp",tcp, "Zh",Zh)
sol_pre=a.ik_hand(pre,q,seed=a.joints(),at_tcp=True,tries=10)
print("pre sol",sol_pre)
sol_g=a.ik_hand(tcp,q,seed=sol_pre,at_tcp=True,tries=10)
print("grasp sol",sol_g, "maxdiff",np.max(np.abs(np.array(sol_g)-np.array(sol_pre))))
np.save("sols_handle.npy",np.array([sol_pre,sol_g]))
EOF

# openrua op 50
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
phi=np.deg2rad(20); c,s=np.cos(phi),np.sin(phi)
R=np.array([[0,1,0],[c,0,s],[s,0,-c]]); q=R_to_quat(R); Zh=R[:,2]
tcp=np.array([-0.0235,-0.0415,0.95]); pre=tcp-0.06*Zh
seeds=[[0,-0.3,0,-2.2,0,1.9,0.8],[0.3,0.5,-0.3,-1.8,0.3,2.3,0.0],[-0.5,0.3,0.5,-2.0,-1.5,2.0,1.5],[0,0.6,0,-1.5,-1.57,1.6,0.8],[0.5,0.4,-0.9,-1.9,1.5,2.0,-1.0],[-0.3,0.8,0.2,-1.6,1.57,1.4,2.0]]
best=None
for sd in seeds:
    try: sol=a.ik_hand(pre,q,seed=sd,at_tcp=True,tries=4)
    except Exception as e: print("fail",sd); continue
    print(np.round(sol,3))
# also alternative orientation with Y_h=-x (X_h flipped)
R2=np.array([[0,-1,0],[-c,0,s],[-s,0,-c]]); print("det",np.linalg.det(R2)); q2=R_to_quat(R2)
for sd in seeds[:3]:
    try: sol=a.ik_hand(pre,q2,seed=sd,at_tcp=True,tries=4)
    except Exception as e: print("fail2",sd); continue
    print("alt",np.round(sol,3))
EOF

# openrua op 51
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a)
pl.set_scene([], remove=("yellow_mug",))
phi=np.deg2rad(20); c,s=np.cos(phi),np.sin(phi)
R=np.array([[0,1,0],[c,0,s],[s,0,-c]]); q=R_to_quat(R); Zh=R[:,2]
tcp=np.array([-0.0235,-0.0415,0.95]); pre=tcp-0.06*Zh
sol_pre=a.ik_hand(pre,q,seed=[-0.258,0.627,0.101,-1.801,0.394,2.324,-1.245],at_tcp=True,tries=6)
print("pre",np.round(sol_pre,3))
print(pl.plan_exec(sol_pre)); a.move([sol_pre],[1.0])
p,qq=a.fk_hand(); print("hand",p,qq,"tcp",p+0.1034*quat_to_R(qq)[:,2])
a.snap("robot0_eye_in_hand","pre1.png"); np.save("eih_pre1.npy",a.xyz_map("robot0_eye_in_hand"))
EOF

# openrua op 52
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a)
phi=np.deg2rad(20); c,s=np.cos(phi),np.sin(phi)
R=np.array([[0,1,0],[c,0,s],[s,0,-c]]); q=R_to_quat(R); Zh=R[:,2]
tcp=np.array([-0.0235,-0.0415,0.95]); pre=tcp+np.array([0,-0.01,0.05])
sol_pre=a.ik_hand(pre,q,seed=[-0.258,0.627,0.101,-1.801,0.394,2.324,-1.245],at_tcp=True,tries=6)
print("pre",np.round(sol_pre,3))
try:
    print(pl.plan_exec(sol_pre))
except Exception as e:
    print("ERR",e)
    # try a higher intermediate
    mid=tcp+np.array([0,-0.01,0.15])
    sol_mid=a.ik_hand(mid,q,seed=sol_pre,at_tcp=True,tries=6); print("mid",np.round(sol_mid,3))
    print(pl.plan_exec(sol_mid)); a.move([sol_mid],[1.0])
    print(pl.plan_exec(sol_pre))
a.move([sol_pre],[1.0])
p,qq=a.fk_hand(); print("hand",p,qq,"tcp",p+0.1034*quat_to_R(qq)[:,2])
a.snap("robot0_eye_in_hand","pre1.png"); np.save("eih_pre1.npy",a.xyz_map("robot0_eye_in_hand"))
EOF

# openrua op 53
timeout 120 python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
rclpy.init(); n=rclpy.create_node("qs")
c=n.create_client(GetPlanningScene,"/get_planning_scene"); c.wait_for_service(10)
req=GetPlanningScene.Request(); req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
f=c.call_async(req); rclpy.spin_until_future_complete(n,f,timeout_sec=20)
for co in f.result().scene.world.collision_objects:
    for p,s in zip(co.primitive_poses,co.primitives):
        print(co.id, "center",[round(p.position.x,3),round(p.position.y,3),round(p.position.z,3)],"size",[round(d,3) for d in s.dimensions])
EOF

# openrua op 54
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
phi=np.deg2rad(20); c,s=np.cos(phi),np.sin(phi)
R=np.array([[0,1,0],[c,0,s],[s,0,-c]]); q=R_to_quat(R); Zh=R[:,2]
tcp=np.array([-0.0235,-0.0415,0.95]); pre=tcp+np.array([0,-0.01,0.05])
cur=a.joints()
sol_pre=a.ik_hand(pre,q,seed=cur,at_tcp=True,tries=6)
print("cur",np.round(cur,3)); print("pre",np.round(sol_pre,3))
a.move([sol_pre],[2.0]); a.move([sol_pre],[1.0])
p,qq=a.fk_hand(); print("hand",p,qq,"tcp",p+0.1034*quat_to_R(qq)[:,2])
a.snap("robot0_eye_in_hand","pre1.png"); np.save("eih_pre1.npy",a.xyz_map("robot0_eye_in_hand"))
EOF

# openrua op 55
python3 - <<'EOF'
import numpy as np
P=np.load("eih_pre1.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.05)&(X<0.0)&(Y>-0.08)&(Y<-0.02)&(Z>0.92)&(Z<1.0)&np.isfinite(Z)
s=P[m]; print("handle n",len(s))
for z0 in np.arange(0.92,1.0,0.01):
    t=s[(s[:,2]>=z0)&(s[:,2]<z0+0.01)]
    if len(t)>3: print(f"z {z0:.2f} n={len(t)} y[{t[:,1].min():.4f},{t[:,1].max():.4f}] x[{t[:,0].min():.4f},{t[:,0].max():.4f}]")
EOF

# openrua op 56
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
phi=np.deg2rad(20); c,s=np.cos(phi),np.sin(phi)
R=np.array([[0,1,0],[c,0,s],[s,0,-c]]); q=R_to_quat(R)
tcp=np.array([-0.0245,-0.0425,0.95])
cur=a.joints()
sol=a.ik_hand(tcp,q,seed=cur,at_tcp=True,tries=6)
print("grasp sol",np.round(sol,3),"maxdiff",np.max(np.abs(np.array(sol)-np.array(cur))))
a.move([sol],[2.0]); a.move([sol],[1.0])
p,qq=a.fk_hand(); print("tcp",p+0.1034*quat_to_R(qq)[:,2])
print("fingers before",a.fingers())
a.gripper(0.0)
print("fingers after",a.fingers())
a.snap("robot0_eye_in_hand","grasp1.png")
EOF

# openrua op 57
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
p,qq=a.fk_hand(); tcp=p+0.1034*quat_to_R(qq)[:,2]
cur=a.joints()
sol=a.ik_hand(tcp+np.array([0,0,0.04]),qq,seed=cur,at_tcp=True,tries=6)
a.move([sol],[2.0]); a.move([sol],[1.0])
p,qq=a.fk_hand(); print("tcp",p+0.1034*quat_to_R(qq)[:,2],"fingers",a.fingers())
a.snap("frontview","lift_front.png"); a.snap("agentview","lift_agent.png"); a.snap("robot0_eye_in_hand","lift_eih.png")
EOF

# openrua op 58
timeout 300 python3 depthmap.py frontview 2>&1 | grep -v XMLPARSER | tail -2; python3 - <<'EOF'
import numpy as np
P=np.load("frontview_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.09)&(X<0.05)&(Y>-0.03)&(Y<0.11)&(Z>0.92)&(Z<1.12)&np.isfinite(Z)
s=P[m]; print("n",len(s),"z range",s[:,2].min(),s[:,2].max())
# rim: highest points per y-bin
for y0 in np.arange(-0.02,0.10,0.01):
    t=s[(s[:,1]>=y0)&(s[:,1]<y0+0.01)]
    if len(t)>3: print(f"y {y0:.2f} zmax={t[:,2].max():.4f} zmin={t[:,2].min():.4f} n={len(t)}")
EOF

# openrua op 59
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
p,qq=a.fk_hand(); Zh=quat_to_R(qq)[:,2]; tcp=p+0.1034*Zh
print("tcp now",tcp)
goal_xy=np.array([-0.12+0.005, 0.23-0.0858])
wps=[np.array([tcp[0],tcp[1],1.06]),
     np.array([(tcp[0]+goal_xy[0])/2,(tcp[1]+goal_xy[1])/2,1.06]),
     np.array([goal_xy[0],goal_xy[1],1.06]),
     np.array([goal_xy[0],goal_xy[1],0.982])]
seed=a.joints(); sols=[]
for w in wps:
    s=a.ik_hand(w,qq,seed=seed,at_tcp=True,tries=8); sols.append(s); seed=s
    print(np.round(w,3),np.round(s,3))
for s in sols:
    a.move([s],[2.0])
a.move([sols[-1]],[1.0])
p,qq=a.fk_hand(); print("tcp",p+0.1034*quat_to_R(qq)[:,2],"fingers",a.fingers())
EOF

# openrua op 60
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
a.gripper(0.04)
p,qq=a.fk_hand(); Zh=quat_to_R(qq)[:,2]; tcp=p+0.1034*Zh
s=a.ik_hand(tcp+np.array([0,0,0.10]),qq,seed=a.joints(),at_tcp=True,tries=6)
a.move([s],[2.0]); a.move([s],[1.0])
a.snap("agentview","placed_agent.png"); a.snap("frontview","placed_front.png")
EOF

# openrua op 61
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
# move to a top-down view above the new mug location
s=a.ik_hand(np.array([-0.16,0.20,1.28]),Q_DOWN,seed=[0.02,0.151,0.027,-1.989,-0.009,2.139,0.85],tries=8)
a.move([s],[3.0]); a.move([s],[1.0])
a.snap("robot0_eye_in_hand","top2.png"); P=a.xyz_map("robot0_eye_in_hand"); np.save("eih_top2.npy",P)
X,Y,Z=P[...,0],P[...,1],P[...,2]
r=P[(Z>0.99)&(Z<1.02)&(X>-0.25)&(X<0.0)&(Y>0.1)&(Y<0.35)&np.isfinite(Z)]
print("rim n",len(r),"x",r[:,0].min(),r[:,0].max(),"y",r[:,1].min(),r[:,1].max(),"z med",np.median(r[:,2]))
xc=(r[:,0].min()+r[:,0].max())/2; yc=(r[:,1].min()+r[:,1].max())/2; print("center",xc,yc)
h=P[(Z>0.97)&(Z<0.995)&(X>xc-0.03)&(X<xc+0.03)&(Y<yc-0.052)&(Y>yc-0.10)&np.isfinite(Z)]
print("handle n",len(h),"x",np.round(np.percentile(h[:,0],[2,50,98]),4),"y",np.round(np.percentile(h[:,1],[2,50,98]),4),"z",np.round(np.percentile(h[:,2],[2,50,98]),4))
EOF

# openrua op 62
python3 - <<'EOF'
import numpy as np
P=np.load("eih_top2.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
r=P[(Z>0.99)&(Z<1.02)&(X>-0.25)&(X<-0.05)&(Y>0.1)&(Y<0.30)&np.isfinite(Z)]
print("rim n",len(r),"x",r[:,0].min(),r[:,0].max(),"y",r[:,1].min(),r[:,1].max(),"z med",np.median(r[:,2]))
xc=(r[:,0].min()+r[:,0].max())/2; yc=(r[:,1].min()+r[:,1].max())/2; print("center",xc,yc)
h=P[(Z>0.97)&(Z<0.995)&(X>xc-0.03)&(X<xc+0.03)&(Y<yc-0.052)&(Y>yc-0.10)&np.isfinite(Z)]
print("handle n",len(h),"x",np.round(np.percentile(h[:,0],[2,50,98]),4),"y",np.round(np.percentile(h[:,1],[2,50,98]),4),"z",np.round(np.percentile(h[:,2],[2,50,98]),4))
EOF

# openrua op 63
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a)
phi=np.deg2rad(85); c,s=np.cos(phi),np.sin(phi)
R=np.array([[0,1,0],[c,0,s],[s,0,-c]]); q=R_to_quat(R); Zh=R[:,2]
tcp=np.array([-0.112,0.137,0.96]); pre=tcp-0.06*Zh
seeds=[a.joints(),[0,-0.3,0,-2.2,0,1.9,0.8],[0.3,0.5,-0.3,-1.8,0.3,2.3,0.0],[-0.5,0.3,0.5,-2.0,-1.5,2.0,1.5],[0,0.6,0,-1.5,-1.57,1.6,0.8]]
for sd in seeds:
    try: sol=a.ik_hand(pre,q,seed=sd,at_tcp=True,tries=4)
    except Exception as e: print("fail"); continue
    code,err=pl.go_joints(sol,plan_only=True)
    print(np.round(sol,3),"plan code",code)
EOF

# openrua op 64
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
a=Arm(); pl=Planner(a)
sv=a.n.create_client(GetStateValidity,"/check_state_validity"); print("svc",sv.wait_for_service(5))
def valid(q):
    req=GetStateValidity.Request(); rs=RobotState(); js=JointState(); js.name=JOINTS; js.position=list(map(float,q)); rs.joint_state=js
    req.robot_state=rs; req.group_name="panda_arm"
    f=sv.call_async(req); rclpy.spin_until_future_complete(a.n,f,timeout_sec=10); r=f.result()
    return r.valid,[(c.contact_body_1,c.contact_body_2) for c in r.contacts]
sols=[[0.098,0.938,-0.377,-1.83,1.532,1.323,-1.978],[-0.988,1.165,0.809,-1.899,0.685,2.062,-1.682],[-1.133,1.326,0.952,-1.922,0.441,2.163,-1.534]]
for s in sols: print(valid(s))
EOF

# openrua op 65
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm()
phi=np.deg2rad(20); c,s=np.cos(phi),np.sin(phi)
R=np.array([[0,1,0],[c,0,s],[s,0,-c]]); q=R_to_quat(R)
bar=np.array([-0.112,0.137,0.96])
pre=bar+np.array([0,-0.01,0.06])
seed=a.joints()
s_pre=a.ik_hand(pre,q,seed=seed,at_tcp=True,tries=8); print("pre",np.round(s_pre,3))
s_g=a.ik_hand(bar,q,seed=s_pre,at_tcp=True,tries=8); print("grasp",np.round(s_g,3))
a.move([s_pre],[3.0]); a.move([s_pre],[1.0])
a.snap("robot0_eye_in_hand","pre2.png")
a.move([s_g],[2.0]); a.move([s_g],[1.0])
p,qq=a.fk_hand(); print("tcp",p+0.1034*quat_to_R(qq)[:,2])
a.gripper(0.0); print("fingers",a.fingers())
EOF

# openrua op 66
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm()
p,qq=a.fk_hand(); R0=quat_to_R(qq); tcp=p+0.1034*R0[:,2]
R1=Rot.from_euler('z',-90,degrees=True).as_matrix()@R0; q1=R_to_quat(R1)
print("new Zh",np.round(R1[:,2],3),"new Yh",np.round(R1[:,1],3))
mug_goal=np.array([-0.09,0.21])
tcp_goal=np.array([mug_goal[0]-0.0858, mug_goal[1]+0.005])
wps=[(tcp+np.array([0,0,0.06]),qq),
     (np.array([tcp[0],tcp[1],tcp[2]+0.06]),q1),
     (np.array([tcp_goal[0],tcp_goal[1],tcp[2]+0.06]),q1),
     (np.array([tcp_goal[0],tcp_goal[1],0.970]),q1)]
seed=a.joints(); sols=[]
for w,qw in wps:
    s=a.ik_hand(w,qw,seed=seed,at_tcp=True,tries=8); sols.append(s); seed=s; print(np.round(w,3),np.round(s,3))
for s in sols: a.move([s],[2.5])
a.move([sols[-1]],[1.0])
p,qq=a.fk_hand(); print("tcp",p+0.1034*quat_to_R(qq)[:,2],"fingers",a.fingers())
a.gripper(0.04)
s=a.ik_hand(p+0.1034*quat_to_R(qq)[:,2]+np.array([0,0,0.12]),qq,seed=a.joints(),at_tcp=True,tries=6)
a.move([s],[2.0]); a.move([s],[1.0])
a.snap("agentview","placed2_agent.png"); a.snap("robot0_eye_in_hand","placed2_eih.png"); np.save("eih_placed2.npy",a.xyz_map("robot0_eye_in_hand"))
EOF

# openrua op 67
python3 - <<'EOF'
import numpy as np
P=np.load("eih_placed2.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
r=P[(Z>0.99)&(Z<1.02)&(X>-0.25)&(X<-0.03)&(Y>0.1)&(Y<0.30)&np.isfinite(Z)]
print("rim n",len(r),"x",r[:,0].min(),r[:,0].max(),"y",r[:,1].min(),r[:,1].max(),"z med",np.median(r[:,2]))
xc=(r[:,0].min()+r[:,0].max())/2; yc=(r[:,1].min()+r[:,1].max())/2; print("center",xc,yc)
h=P[(Z>0.93)&(Z<0.995)&(Y>yc-0.03)&(Y<yc+0.03)&(X<xc-0.052)&(X>xc-0.11)&np.isfinite(Z)]
print("handle n",len(h),"x",np.round(np.percentile(h[:,0],[2,50,98]),4),"y",np.round(np.percentile(h[:,1],[2,50,98]),4),"z",np.round(np.percentile(h[:,2],[2,50,98]),4))
EOF

# openrua op 68
python3 - <<'EOF'
import numpy as np
P=np.load("eih_placed2.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
r=P[(Z>0.99)&(Z<1.02)&(X>-0.25)&(X<-0.06)&(Y>0.12)&(Y<0.28)&np.isfinite(Z)]
print("rim n",len(r),"x",r[:,0].min(),r[:,0].max(),"y",r[:,1].min(),r[:,1].max())
print("center",(r[:,0].min()+r[:,0].max())/2,(r[:,1].min()+r[:,1].max())/2)
EOF

# openrua op 69
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a)
phi=np.deg2rad(85); c,s=np.cos(phi),np.sin(phi)
R=np.array([[c,0,s],[0,-1,0],[s,0,-c]]); q=R_to_quat(R); Zh=R[:,2]
bar=np.array([-0.1775,0.216,0.955]); pre=bar-0.06*Zh
print("pre",pre)
seeds=[a.joints(),[0,-0.3,0,-2.2,0,1.9,0.8],[0.3,0.5,-0.3,-1.8,0.3,2.3,0.0],[0.2,0.8,0,-1.8,0,2.6,0.8]]
ok=None
for sd in seeds:
    try: sol=a.ik_hand(pre,q,seed=sd,at_tcp=True,tries=4)
    except Exception: print("ikfail"); continue
    code,err=pl.go_joints(sol,plan_only=True); print(np.round(sol,3),"code",code)
    if code==1: ok=sol; break
if ok is not None:
    print(pl.plan_exec(ok)); a.move([ok],[1.0])
    p,qq=a.fk_hand(); print("tcp",p+0.1034*quat_to_R(qq)[:,2])
    a.snap("robot0_eye_in_hand","pre3.png"); np.save("eih_pre3.npy",a.xyz_map("robot0_eye_in_hand"))
EOF

# openrua op 70
python3 - <<'EOF'
import numpy as np
P=np.load("eih_pre3.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
h=P[(X>-0.20)&(X<-0.16)&(Y>0.19)&(Y<0.24)&(Z>0.92)&(Z<1.0)&np.isfinite(Z)]
print("handle n",len(h))
for z0 in np.arange(0.92,1.0,0.01):
    t=h[(h[:,2]>=z0)&(h[:,2]<z0+0.01)]
    if len(t)>3: print(f"z {z0:.2f} n={len(t)} x[{t[:,0].min():.4f},{t[:,0].max():.4f}] y[{t[:,1].min():.4f},{t[:,1].max():.4f}]")
EOF

# openrua op 71
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
p,qq=a.fk_hand()
tcp=np.array([-0.180,0.216,0.955])
s=a.ik_hand(tcp,qq,seed=a.joints(),at_tcp=True,tries=6)
print("maxdiff",np.max(np.abs(np.array(s)-np.array(a.joints()))))
a.move([s],[2.0]); a.move([s],[1.0])
p,qq=a.fk_hand(); print("tcp",p+0.1034*quat_to_R(qq)[:,2])
a.gripper(0.0); print("fingers",a.fingers())
a.snap("robot0_eye_in_hand","grasp3.png")
EOF

# openrua op 72
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
p,qq=a.fk_hand(); Zh=quat_to_R(qq)[:,2]; tcp=p+0.1034*Zh
s=a.ik_hand(tcp+np.array([0,0,0.05]),qq,seed=a.joints(),at_tcp=True,tries=6)
a.move([s],[2.0]); a.move([s],[1.0])
p,qq=a.fk_hand(); print("tcp",p+0.1034*quat_to_R(qq)[:,2],"fingers",a.fingers())
a.snap("agentview","lift3_agent.png"); a.snap("sideview","lift3_side.png")
EOF
timeout 300 python3 depthmap.py agentview 2>&1 | grep -v XMLPARSER | tail -1
python3 - <<'EOF'
import numpy as np
P=np.load("agentview_xyz.npy"); X,Y,Z=P[...,0],P[...,1],P[...,2]
m=(X>-0.16)&(X<-0.04)&(Y>0.15)&(Y<0.28)&(Z>0.92)&(Z<1.12)&np.isfinite(Z)
s=P[m]; print("n",len(s),"z",s[:,2].min(),s[:,2].max())
for x0 in np.arange(-0.16,-0.04,0.01):
    t=s[(s[:,0]>=x0)&(s[:,0]<x0+0.01)]
    if len(t)>3: print(f"x {x0:.2f} zmax={t[:,2].max():.4f} zmin={t[:,2].min():.4f} n={len(t)}")
EOF

# openrua op 73
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm()
p,qq=a.fk_hand(); R0=quat_to_R(qq); tcp=p+0.1034*R0[:,2]
R1=Rot.from_euler('z',90,degrees=True).as_matrix()@R0; q1=R_to_quat(R1)
print("Zh after yaw",np.round(R1[:,2],3))
wps=[(np.array([tcp[0],tcp[1],1.20]),qq),
     (np.array([tcp[0],tcp[1],1.20]),q1),
     (np.array([-0.15,0.05,1.20]),q1),
     (np.array([-0.123,-0.25,1.20]),q1),
     (np.array([-0.123,-0.49,1.20]),q1),
     (np.array([-0.123,-0.49,1.018]),q1)]
seed=a.joints(); sols=[]
for w,qw in wps:
    s=a.ik_hand(w,qw,seed=seed,at_tcp=True,tries=10)
    print(np.round(w,3),np.round(s,3),"maxdiff",round(float(np.max(np.abs(np.array(s)-np.array(seed)))),3))
    sols.append(s); seed=s
np.save("sols_carry.npy",np.array(sols))
EOF

# openrua op 74
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm()
p,qq=a.fk_hand(); R0=quat_to_R(qq); tcp=p+0.1034*R0[:,2]
# 1) find good staging solutions
Rst=Rot.from_euler('z',90,degrees=True).as_matrix()@R0; qst=R_to_quat(Rst)
seeds=[[-0.7,0.5,0,-1.8,0,2.3,0.0],[-0.7,0.8,0.2,-1.5,0.3,2.3,-0.5],[-0.5,0.3,-0.3,-2.0,1.5,2.0,-1.5],[-1.0,0.6,0.5,-1.9,-0.5,2.4,0.5],[0,0.6,-0.6,-1.8,1.57,1.6,-1.6],[-0.4,0.9,-0.9,-1.6,1.2,1.8,-2.0]]
for sd in seeds:
    for z in [1.20,1.018]:
        try: s=a.ik_hand(np.array([-0.123,-0.49,z]),qst,seed=sd,at_tcp=True,tries=4); print(z,np.round(s,3))
        except Exception as e: print(z,"fail",sd)
# 2) yaw continuity from current config
seed=a.joints()
for ang in range(15,91,15):
    R1=Rot.from_euler('z',ang,degrees=True).as_matrix()@R0
    try: s=a.ik_hand(np.array([tcp[0],tcp[1],1.20]),R_to_quat(R1),seed=seed,at_tcp=True,tries=6)
    except Exception as e: print("yaw",ang,"fail"); break
    print("yaw",ang,np.round(s,3),"maxdiff",round(float(np.max(np.abs(np.array(s)-np.array(seed)))),3)); seed=s
EOF

# openrua op 75
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
seeds=[[-0.7,0.5,0,-1.8,0,2.3,0.0],[-0.5,0.3,-0.3,-2.0,1.5,2.0,-1.5],[-1.0,0.6,0.5,-1.9,-0.5,2.4,0.5],[0,0.6,-0.6,-1.8,1.57,1.6,-1.6],[-0.4,0.9,-0.9,-1.6,1.2,1.8,-2.0],[-0.8,1.0,0.3,-1.4,0.5,2.4,-0.8]]
for phi in [85,90]:
    ph=np.deg2rad(phi); c,s=np.cos(ph),np.sin(ph)
    R=np.array([[0,1,0],[c,0,s],[s,0,-c]]); q=R_to_quat(R)
    for x in [-0.12,-0.15]:
        for y in [-0.38,-0.40,-0.42,-0.44,-0.46]:
            for z in [1.02,1.12]:
                found=None
                for sd in seeds:
                    try: found=a.ik_hand(np.array([x,y,z]),q,seed=sd,at_tcp=True,tries=2); break
                    except Exception: pass
                print(phi,x,y,z,"OK" if found is not None else "--", np.round(found,2) if found is not None else "")
EOF

# openrua op 76
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
ph=np.deg2rad(85); c,s=np.cos(ph),np.sin(ph)
R=np.array([[0,1,0],[c,0,s],[s,0,-c]]); q=R_to_quat(R)
seed=[0.098,0.938,-0.377,-1.83,1.532,1.323,-1.978]
for y in np.arange(0.08,-0.50,-0.04):
    try: sol=a.ik_hand(np.array([-0.12,y,1.02]),q,seed=seed,at_tcp=True,tries=6); seed=sol; print(round(y,2),np.round(sol,2))
    except Exception as e: print(round(y,2),"fail"); 
EOF

# openrua op 77
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm()
ph=np.deg2rad(85); c,s=np.cos(ph),np.sin(ph)
Rbase=np.array([[0,1,0],[c,0,s],[s,0,-c]])
Rflip=np.array([[0,-1,0],[-c,0,s],[-s,0,-c]])  # X_h=-z-ish, Y_h=-x
variants={"flip":Rflip}
for th in [-40,-20,20,40]:
    variants[f"yaw{th}"]=Rot.from_euler('z',th,degrees=True).as_matrix()@Rbase
    variants[f"flipyaw{th}"]=Rot.from_euler('z',th,degrees=True).as_matrix()@Rflip
seeds=[[0.098,0.938,-0.377,-1.83,1.532,1.323,-1.978],[-0.3,0.6,0,-1.8,0,2.3,0.8],[-0.5,0.8,0.5,-1.5,-1.5,2.0,1.5],[0.3,0.5,-0.3,-1.8,0.3,2.3,0.0],[-1,0.5,1,-2,0,2.5,0]]
for name,R in variants.items():
    q=R_to_quat(R); seed=None; last=None
    for sd in seeds:
        try: seed=a.ik_hand(np.array([-0.12,0.0,1.02]),q,seed=sd,at_tcp=True,tries=3); break
        except Exception: pass
    if seed is None: print(name,"no start"); continue
    for y in np.arange(-0.04,-0.60,-0.04):
        try: sol=a.ik_hand(np.array([-0.12,y,1.02]),q,seed=seed,at_tcp=True,tries=6); seed=sol; last=y
        except Exception: break
    print(name,"reaches y =",last, np.round(seed,2))
EOF

# openrua op 78
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm()
def orient(theta_deg, yh_sign):
    th=np.deg2rad(theta_deg); Zh=np.array([-np.sin(th),np.cos(th),0.0]); Yh=np.array([0,0,yh_sign]); Xh=np.cross(Yh,Zh)
    R=np.column_stack([Xh,Yh,Zh]); assert abs(np.linalg.det(R)-1)<1e-6; return R_to_quat(R)
seeds=[[0.098,0.938,-0.377,-1.83,1.532,1.323,-1.978],[-0.3,0.6,0,-1.8,0,2.3,0.8],[-0.5,0.8,0.5,-1.5,-1.5,2.0,1.5],[0.3,0.5,-0.3,-1.8,0.3,2.3,0.0],[-1,0.5,1,-2,0,2.5,0],[-0.7,0.9,0.3,-1.6,1.0,2.2,-0.6]]
for th in [0,-20,-40]:
    for sgn in [1,-1]:
        q=orient(th,sgn); seed=None; last=None
        for sd in seeds:
            try: seed=a.ik_hand(np.array([-0.14,0.0,1.015]),q,seed=sd,at_tcp=True,tries=3); break
            except Exception: pass
        if seed is None: print(th,sgn,"no start"); continue
        for y in np.arange(-0.04,-0.62,-0.04):
            try: sol=a.ik_hand(np.array([-0.14,y,1.015]),q,seed=seed,at_tcp=True,tries=6); seed=sol; last=y
            except Exception: break
        print("theta",th,"Yh sign",sgn,"reaches y =",round(last,2) if last else None, np.round(seed,2))
EOF

# openrua op 79
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
p,qq=a.fk_hand(); Zh=quat_to_R(qq)[:,2]; tcp=p+0.1034*Zh
s=a.ik_hand(np.array([tcp[0],tcp[1],0.958]),qq,seed=a.joints(),at_tcp=True,tries=6)
a.move([s],[2.0]); a.move([s],[1.0])
print("tcp",a.fk_hand()[0]+0.1034*Zh)
a.gripper(0.04)
s2=a.ik_hand(np.array([tcp[0]-0.06,tcp[1],0.965]),qq,seed=a.joints(),at_tcp=True,tries=6)
a.move([s2],[2.0]); a.move([s2],[1.0])
a.snap("agentview","down3_agent.png")
EOF

# openrua op 80
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
P=a.xyz_map("birdview"); np.save("bird_xyz.npy",P)
pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
# door: x<-0.2, y in [-0.60,-0.36], z>0.93
m=(pts[:,0]<-0.15)&(pts[:,0]>-0.45)&(pts[:,1]<-0.36)&(pts[:,1]>-0.65)&(pts[:,2]>0.93)
d=pts[m]; print("door pts",len(d))
for lo,hi in [(-0.40,-0.44),(-0.46,-0.50),(-0.52,-0.56),(-0.56,-0.60)]:
    s=d[(d[:,1]<lo)&(d[:,1]>hi)]
    if len(s): print(f"y {lo}..{hi}: x min {s[:,0].min():.3f} max {s[:,0].max():.3f} z max {s[:,2].max():.3f} n{len(s)}")
print("door y min",d[:,1].min(), "y of x<-0.2 pts min", d[d[:,0]<-0.24][:,1].min())
# mug check
m2=(pts[:,0]>-0.2)&(pts[:,0]<0.0)&(pts[:,1]>0.1)&(pts[:,1]<0.33)&(pts[:,2]>1.0)
mm=pts[m2]; print("mug rim center",mm[:,:2].mean(0),"zmax",mm[:,2].max(),len(mm))
EOF

# openrua op 81
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
a.gripper(0.0)
q=np.array(Q_DOWN,float)
def go(p,dur=3.0,tries=6):
    s=a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=tries)
    a.move([s],[dur]); a.move([s],[1.0])
    print("tcp",np.round(a.fk_hand()[0]+0.1034*quat_to_R(a.fk_hand()[1])[:,2],3))
go([-0.18,-0.56,1.15],4.0)
go([-0.18,-0.56,1.00],2.5)
go([-0.22,-0.56,1.00],1.5)
go([-0.26,-0.555,1.00],2.0)
P=a.xyz_map("birdview"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
m=(pts[:,0]<-0.15)&(pts[:,0]>-0.6)&(pts[:,1]<-0.36)&(pts[:,1]>-0.7)&(pts[:,2]>1.05)&(pts[:,2]<1.12)
d=pts[m]
for lo,hi in [(-0.40,-0.44),(-0.46,-0.50),(-0.52,-0.56),(-0.56,-0.60)]:
    s=d[(d[:,1]<lo)&(d[:,1]>hi)]
    if len(s): print(f"y {lo}..{hi}: x min {s[:,0].min():.3f} max {s[:,0].max():.3f} n{len(s)}")
a.snap("agentview","push1.png")
EOF

# openrua op 82
timeout 120 python3 -u -c "
from arm import *
a=Arm(); a.snap('birdview','push1_bird.png'); a.snap('frontview','push1_front.png')" 2>&1 | grep -v XMLPARSER

# openrua op 83
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
q=np.array(Q_DOWN,float)
s=a.ik_hand(np.array([-0.20,-0.60,1.25]),q,seed=a.joints(),at_tcp=True,tries=6)
a.move([s],[3.0]); a.move([s],[1.0])
P=a.xyz_map("birdview"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
m=(pts[:,0]<-0.15)&(pts[:,0]>-0.6)&(pts[:,1]<-0.36)&(pts[:,1]>-0.7)&(pts[:,2]>0.95)&(pts[:,2]<1.12)
d=pts[m]
for lo,hi in [(-0.36,-0.40),(-0.40,-0.44),(-0.44,-0.48),(-0.48,-0.52),(-0.52,-0.56),(-0.56,-0.60),(-0.60,-0.64)]:
    s=d[(d[:,1]<lo)&(d[:,1]>hi)]
    if len(s): print(f"y {lo}..{hi}: x min {s[:,0].min():.3f} max {s[:,0].max():.3f} zmax {s[:,2].max():.3f} n{len(s)}")
a.snap("birdview","push1b_bird.png")
EOF

# openrua op 84
timeout 120 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
import numpy as np
from arm import *
a=Arm()
P=a.xyz_map("birdview"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
top=pts[(pts[:,2]>1.09)&(pts[:,2]<1.13)&(pts[:,0]<0.1)&(pts[:,0]>-0.4)&(pts[:,1]>-0.36)&(pts[:,1]<0.0)]
print("microwave top x",top[:,0].min(),top[:,0].max(),"y",top[:,1].min(),top[:,1].max(),len(top))
old=np.load("bird_xyz.npy").reshape(-1,3); old=old[np.isfinite(old).all(1)]
t2=old[(old[:,2]>1.09)&(old[:,2]<1.13)&(old[:,0]<0.1)&(old[:,0]>-0.4)&(old[:,1]>-0.36)&(old[:,1]<0.0)]
print("old top x",t2[:,0].min(),t2[:,0].max(),"y",t2[:,1].min(),t2[:,1].max(),len(t2))
EOF

# openrua op 85
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
q=np.array(Q_DOWN,float)
hinge=np.array([-0.28,-0.35]); L=0.18
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,dur=2.0):
    s=a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=6)
    if np.abs(s-a.joints()).max()>1.0: print("big jump",np.round(s,2)); 
    a.move([s],[dur]); a.move([s],[1.0]); print("tcp",np.round(tcp(),3))
# start: inner side of door at current beta ~8deg
b=np.deg2rad(8); c=hinge+L*np.array([-np.sin(b),-np.cos(b)])+0.03*np.array([np.cos(b),-np.sin(b)])
go([c[0],c[1],1.15],3.0); go([c[0],c[1],1.00],2.0)
for bd in range(15,95,10):
    b=np.deg2rad(bd); c=hinge+L*np.array([-np.sin(b),-np.cos(b)])+0.005*np.array([np.cos(b),-np.sin(b)])
    try: go([c[0],c[1],1.00],1.5)
    except Exception as e: print("fail at",bd,e); break
go([tcp()[0],tcp()[1],1.20],2.0)
P=a.xyz_map("birdview"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
d=pts[(pts[:,0]<-0.28)&(pts[:,0]>-0.7)&(pts[:,1]<-0.2)&(pts[:,1]>-0.7)&(pts[:,2]>0.95)&(pts[:,2]<1.12)]
print("door pts x",d[:,0].min(),d[:,0].max(),"y",d[:,1].min(),d[:,1].max(),len(d))
a.snap("birdview","push2_bird.png")
EOF

# openrua op 86
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
q=np.array(Q_DOWN,float)
hinge=np.array([-0.28,-0.35]); L=0.18
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,dur=2.0):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=6))
    if np.abs(s-np.array(a.joints())).max()>1.0: print("big jump",np.round(s,2)); 
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); print("tcp",np.round(tcp(),3))
b=np.deg2rad(8); c=hinge+L*np.array([-np.sin(b),-np.cos(b)])+0.03*np.array([np.cos(b),-np.sin(b)])
go([c[0],c[1],1.15],3.0); go([c[0],c[1],1.00],2.0)
for bd in range(15,95,10):
    b=np.deg2rad(bd); c=hinge+L*np.array([-np.sin(b),-np.cos(b)])+0.005*np.array([np.cos(b),-np.sin(b)])
    try: go([c[0],c[1],1.00],1.5)
    except Exception as e: print("fail at",bd,e); break
go([tcp()[0],tcp()[1],1.20],2.0)
P=a.xyz_map("birdview"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
d=pts[(pts[:,0]<-0.28)&(pts[:,0]>-0.7)&(pts[:,1]<-0.2)&(pts[:,1]>-0.7)&(pts[:,2]>0.95)&(pts[:,2]<1.12)]
print("door pts x",d[:,0].min(),d[:,0].max(),"y",d[:,1].min(),d[:,1].max(),len(d))
a.snap("birdview","push2_bird.png")
EOF

# openrua op 87
timeout 120 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
P=a.xyz_map("birdview"); np.save("bird2.npy",P); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
d=pts[(pts[:,0]<-0.285)&(pts[:,0]>-0.7)&(pts[:,1]<-0.30)&(pts[:,1]>-0.7)&(pts[:,2]>1.05)&(pts[:,2]<1.12)]
print(len(d))
# PCA line
c=d[:,:2].mean(0); u,s,vt=np.linalg.svd(d[:,:2]-c); dirv=vt[0]; print("center",c,"dir",dirv,"sv",s)
t=(d[:,:2]-c)@dirv; n=(d[:,:2]-c)@vt[1]
print("t range",t.min(),t.max(),"n range",n.min(),n.max())
e1=c+t.min()*dirv; e2=c+t.max()*dirv; print("ends",e1,e2)
EOF

# openrua op 88
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
q=np.array(Q_DOWN,float)
H=np.array([-0.28,-0.335]); L=0.16
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,dur=2.0):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=6))
    if np.abs(s-np.array(a.joints())).max()>1.0: print("big jump",np.round(s,2))
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"err",np.round(np.linalg.norm(t-p),3)); return t
def pt(bd,off):
    b=np.deg2rad(bd); d=np.array([-np.sin(b),-np.cos(b)]); n=np.array([np.cos(b),-np.sin(b)])
    return H+L*d+off*n
c=pt(29,0.06); go([c[0],c[1],1.22],3.0); go([c[0],c[1],1.00],2.5)
c=pt(29,0.005); go([c[0],c[1],1.00],1.5)
for bd in range(38,95,8):
    c=pt(bd,0.005); t=go([c[0],c[1],1.00],1.5)
    if np.linalg.norm(t[:2]-c)>0.02: print("stalled at",bd); break
t=tcp(); go([t[0],t[1]+0.03,1.22],2.0)
P=a.xyz_map("birdview"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
d=pts[(pts[:,0]<-0.285)&(pts[:,0]>-0.7)&(pts[:,1]<-0.20)&(pts[:,1]>-0.7)&(pts[:,2]>1.05)&(pts[:,2]<1.12)]
c=d[:,:2].mean(0); u,s,vt=np.linalg.svd(d[:,:2]-c); dirv=vt[0]; tt=(d[:,:2]-c)@dirv
print("door ends",c+tt.min()*dirv,c+tt.max()*dirv,"dir",dirv,len(d))
a.snap("birdview","push3_bird.png")
EOF

# openrua op 89
timeout 120 python3 -u -c "
from arm import *
a=Arm(); a.snap('frontview','push3_front.png'); a.snap('sideview','push3_side.png')" 2>&1 | grep -v XMLPARSER

# openrua op 90
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
q=np.array(Q_DOWN,float)
H=np.array([-0.28,-0.335]); L=0.16; Z=1.07
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,dur=2.0):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=6))
    if np.abs(s-np.array(a.joints())).max()>1.0: print("big jump",np.round(s,2))
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"err",np.round(np.linalg.norm(t-p),3)); return t
def pt(bd,off):
    b=np.deg2rad(bd); d=np.array([-np.sin(b),-np.cos(b)]); n=np.array([np.cos(b),-np.sin(b)])
    return H+L*d+off*n
c=pt(26,0.06); go([c[0],c[1],1.22],3.0); go([c[0],c[1],Z],2.0)
c=pt(26,0.005); go([c[0],c[1],Z],1.5)
for bd in range(34,95,8):
    c=pt(bd,0.005); t=go([c[0],c[1],Z],1.5)
    if np.linalg.norm(t[:2]-c)>0.02: print("stalled at",bd); break
t=tcp(); go([t[0],t[1]+0.03,1.25],2.0)
P=a.xyz_map("birdview"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
d=pts[(pts[:,0]<-0.285)&(pts[:,0]>-0.7)&(pts[:,1]<-0.20)&(pts[:,1]>-0.7)&(pts[:,2]>1.05)&(pts[:,2]<1.12)]
c=d[:,:2].mean(0); u,s,vt=np.linalg.svd(d[:,:2]-c); dirv=vt[0]; tt=(d[:,:2]-c)@dirv
print("door ends",c+tt.min()*dirv,c+tt.max()*dirv,"dir",dirv,len(d))
a.snap("birdview","push4_bird.png")
EOF

# openrua op 91
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a)
# scene: microwave body + door (3 boxes along door line) 
door=[]
H=np.array([-0.283,-0.319]); d=np.array([-0.405,-0.914])
for i,t in enumerate([0.04,0.12,0.20]):
    c=H+t*d; door.append((f"door{i}",(c[0],c[1],1.0),(0.09,0.09,0.22)))
pl.set_scene([("microwave",(-0.105,-0.245,1.0),(0.38,0.24,0.22))]+door, remove=("yellow_mug",))
q=np.array(Q_DOWN,float)
s=a.ik_hand(np.array([-0.14,0.216,1.28]),q,seed=a.joints(),at_tcp=True,tries=6)
code,err=pl.go_joints(list(s),0.5,plan_only=True); print("plan",code)
if code==1: a.move([list(s)],[4.0]); a.move([list(s)],[1.0])
else: a.move([list(s)],[5.0]); a.move([list(s)],[1.0])
P=a.xyz_map("robot0_eye_in_hand"); np.save("eih_top4.npy",P); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
m=pts[(pts[:,2]>0.91)&(pts[:,2]<1.05)&(pts[:,0]>-0.25)&(pts[:,0]<0.0)&(pts[:,1]>0.12)&(pts[:,1]<0.32)]
rim=m[m[:,2]>0.99]; print("rim center",rim[:,:2].mean(0),"n",len(rim),"zmax",rim[:,2].max())
cx,cy=rim[:,:2].mean(0)
bar=m[(m[:,0]<cx-0.06)&(m[:,2]>0.92)&(m[:,2]<0.995)]
print("bar x",bar[:,0].min(),bar[:,0].max(),"y",bar[:,1].min(),bar[:,1].max(),"z",bar[:,2].min(),bar[:,2].max(),len(bar))
print("bar y mean",bar[:,1].mean())
a.snap("robot0_eye_in_hand","top4.png")
EOF

# openrua op 92
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a)
c,s_=np.cos(np.deg2rad(85)),np.sin(np.deg2rad(85))
R=np.array([[c,0,s_],[0,-1,0],[s_,0,-c]]); qg=R_to_quat(R)
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
a.gripper(0.04)
s=a.ik_hand(np.array([-0.18,0.218,1.10]),qg,seed=a.joints(),at_tcp=True,tries=8)
code,err=pl.go_joints(list(s),0.5,plan_only=True); print("plan",code)
if code!=1:
    s=a.ik_hand(np.array([-0.18,0.218,1.13]),qg,seed=a.joints(),at_tcp=True,tries=8); code,err=pl.go_joints(list(s),0.5,plan_only=True); print("plan2",code)
a.move([list(s)],[4.0]); a.move([list(s)],[1.0]); print("tcp",np.round(tcp(),3))
for z in [1.02,0.955]:
    s=a.ik_hand(np.array([-0.18,0.218,z]),qg,seed=a.joints(),at_tcp=True,tries=8)
    a.move([list(s)],[2.0]); a.move([list(s)],[1.0]); print("tcp",np.round(tcp(),3))
a.gripper(0.0)
s=a.ik_hand(np.array([-0.18,0.218,1.00]),qg,seed=a.joints(),at_tcp=True,tries=8)
a.move([list(s)],[2.0]); a.move([list(s)],[1.0]); print("tcp",np.round(tcp(),3), "fingers",a.fingers())
a.snap("agentview","grasp4.png")
EOF

# openrua op 93
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
p,qq=a.fk_hand(); qg=qq
s=a.ik_hand(np.array([-0.18,0.218,1.12]),qg,seed=a.joints(),at_tcp=True,tries=8)
a.move([list(s)],[2.5]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",a.fingers())
for cam in ["agentview","frontview"]:
    P=a.xyz_map(cam); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
    m=pts[(pts[:,0]>-0.16)&(pts[:,0]<0.0)&(pts[:,1]>0.14)&(pts[:,1]<0.30)&(pts[:,2]>0.95)&(pts[:,2]<1.25)]
    print(cam,"mug pts",len(m),"zmin",m[:,2].min(),"zmax",m[:,2].max())
    for lo,hi in [(1.0,1.03),(1.03,1.06),(1.06,1.09),(1.09,1.12),(1.12,1.15),(1.15,1.18)]:
        s_=m[(m[:,2]>lo)&(m[:,2]<hi)]
        if len(s_): print(f"  z {lo}-{hi}: x {s_[:,0].min():.3f}..{s_[:,0].max():.3f} y {s_[:,1].min():.3f}..{s_[:,1].max():.3f} n{len(s_)}")
a.snap("agentview","lift4.png")
EOF

# openrua op 94
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,q,dur=2.0):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=8))
    j=np.abs(s-np.array(a.joints())).max()
    if j>0.8: print("BIG JUMP",j,np.round(s,2)); return None
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",np.round(a.fingers()[0],4)); return t
p,qq=a.fk_hand(); R0=quat_to_R(qq)
go([-0.18,0.218,1.25],qq,2.5)
for ang in [-45,-90]:
    R=Rot.from_euler('z',ang,degrees=True).as_matrix()@R0; q=R_to_quat(R)
    if go([-0.18,0.218,1.25],q,2.5) is None: break
print("Zh now",np.round(quat_to_R(a.fk_hand()[1])[:,2],3))
q=a.fk_hand()[1]
for wp in [(-0.16,0.10),(-0.14,0.0),(-0.13,-0.095)]:
    if go([wp[0],wp[1],1.25],q,2.5) is None: break
a.snap("agentview","carry4.png")
EOF

# openrua op 95
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm()
print("joints",np.round(a.joints(),3))
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def ik_cont(p,q):
    seed=np.array(a.joints())
    best=None
    for k in range(12):
        sd=seed if k==0 else seed+np.random.normal(0,0.15,7)
        try: s=np.array(a.ik_hand(np.array(p),q,seed=list(sd),at_tcp=True,tries=1))
        except Exception as e: continue
        j=np.abs(s-seed).max()
        if best is None or j<best[0]: best=(j,s)
        if j<0.5: break
    return best
def go(p,q,dur=2.0):
    b=ik_cont(p,q)
    if b is None: print("IK fail"); return None
    j,s=b
    if j>0.8: print("BIG JUMP",j,np.round(s,2)); return None
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",np.round(a.fingers()[0],4)); return t
p,qq=a.fk_hand(); R0=quat_to_R(qq)
if go([-0.18,0.218,1.25],qq,2.5) is not None:
    for ang in [-30,-60,-90]:
        R=Rot.from_euler('z',ang,degrees=True).as_matrix()@R0; q=R_to_quat(R)
        if go([-0.18,0.218,1.25],q,2.5) is None: break
print("Zh now",np.round(quat_to_R(a.fk_hand()[1])[:,2],3),"joints",np.round(a.joints(),3))
EOF

# openrua op 96
grep -n "def \|class \|ApplyPlanningScene\|is_diff\|robot_state" /workspace/arm.py | head -60

# openrua op 97
cat >> /workspace/arm.py <<'EOF'


def attach_cylinder(pl, name, xyz_hand, quat_hand, height, radius, link="panda_hand"):
    """Attach a cylinder to the hand in the planning scene (pose in panda_hand frame)."""
    from moveit_msgs.msg import AttachedCollisionObject
    aco = AttachedCollisionObject()
    aco.link_name = link
    aco.touch_links = ["panda_hand", "panda_leftfinger", "panda_rightfinger", "panda_link8", "panda_link7"]
    co = CollisionObject(); co.header.frame_id = link; co.id = name
    sp = SolidPrimitive(); sp.type = SolidPrimitive.CYLINDER; sp.dimensions = [float(height), float(radius)]
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, xyz_hand)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat_hand)
    co.primitives = [sp]; co.primitive_poses = [p]; co.operation = CollisionObject.ADD
    aco.object = co
    ps = PlanningScene(); ps.is_diff = True; ps.robot_state.is_diff = True
    ps.robot_state.attached_collision_objects = [aco]
    req = ApplyPlanningScene.Request(); req.scene = ps
    fut = pl.aps.call_async(req)
    rclpy.spin_until_future_complete(pl.n, fut, timeout_sec=30)
    print("attach applied:", fut.result().success, flush=True)


def detach_all(pl, name, link="panda_hand"):
    from moveit_msgs.msg import AttachedCollisionObject
    aco = AttachedCollisionObject(); aco.link_name = link
    aco.object.id = name; aco.object.operation = CollisionObject.REMOVE
    ps = PlanningScene(); ps.is_diff = True; ps.robot_state.is_diff = True
    ps.robot_state.attached_collision_objects = [aco]
    # also drop it from the world in case it got re-added there
    co = CollisionObject(); co.id = name; co.header.frame_id = "world"; co.operation = CollisionObject.REMOVE
    ps.world.collision_objects.append(co)
    req = ApplyPlanningScene.Request(); req.scene = ps
    fut = pl.aps.call_async(req)
    rclpy.spin_until_future_complete(pl.n, fut, timeout_sec=30)
    print("detach applied:", fut.result().success, flush=True)
EOF
grep -n "^from geometry_msgs\|Pose" /workspace/arm.py | head -3

# openrua op 98
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm(); pl=Planner(a)
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
door=[]
H=np.array([-0.283,-0.319]); d=np.array([-0.405,-0.914])
for i,t in enumerate([0.04,0.12,0.20]):
    c=H+t*d; door.append((f"door{i}",(c[0],c[1],1.0),(0.09,0.09,0.22)))
pl.set_scene([("microwave",(-0.105,-0.245,1.0),(0.38,0.24,0.22)),("table",(0.0,0.0,0.85),(1.0,1.6,0.1)),("greymug",(0.02,0.33,0.955),(0.11,0.11,0.12))]+door, remove=("yellow_mug",))
attach_cylinder(pl,"mug",[-0.036,0.0,0.183],[0,0.7071068,0,0.7071068],0.12,0.056)
# check current state validity
from moveit_msgs.srv import GetStateValidity
cl=a.n.create_client(GetStateValidity,"/check_state_validity"); cl.wait_for_service(5)
req=GetStateValidity.Request(); req.robot_state.joint_state=a.seed_state(a.joints()); req.group_name="panda_arm"
fut=cl.call_async(req); rclpy.spin_until_future_complete(a.n,fut,timeout_sec=30); r=fut.result()
print("valid now:",r.valid,[ (c.contact_body_1,c.contact_body_2) for c in r.contacts])
p,qq=a.fk_hand(); R0=quat_to_R(qq)
R=Rot.from_euler('z',-90,degrees=True).as_matrix()@R0; qT=R_to_quat(R)
print("target Zh",np.round(R[:,2],3),"Xh",np.round(R[:,0],3))
np.save("qT.npy",qT)
# find IK solutions and try planning
sols=[]
for k in range(10):
    try: s=a.ik_hand(np.array([-0.13,-0.095,1.25]),qT,seed=list(np.array(a.joints())+np.random.normal(0,0.6,7)),at_tcp=True,tries=3)
    except Exception as e: continue
    sols.append(np.array(s))
print(len(sols),"ik sols")
done=False
for s in sols:
    req.robot_state.joint_state=a.seed_state(list(s)); fut=cl.call_async(req); rclpy.spin_until_future_complete(a.n,fut,timeout_sec=30)
    if not fut.result().valid: print("goal invalid",[ (c.contact_body_1,c.contact_body_2) for c in fut.result().contacts]); continue
    code,err=pl.go_joints(list(s),0.5,plan_only=True)
    if code==1:
        np.save("traj_A.npy",np.array([list(p.positions) for p in pl._last_traj.joint_trajectory.points])); print("planned, npts",len(pl._last_traj.joint_trajectory.points)); done=True; break
print("done",done)
EOF

# openrua op 99
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents
cl=a.n.create_client(GetPlanningScene,"/get_planning_scene"); cl.wait_for_service(5)
req=GetPlanningScene.Request(); req.components.components=PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS|PlanningSceneComponents.WORLD_OBJECT_NAMES
fut=cl.call_async(req); rclpy.spin_until_future_complete(a.n,fut,timeout_sec=30); sc=fut.result().scene
print("attached:",[o.object.id for o in sc.robot_state.attached_collision_objects],"world:",[o.id for o in sc.world.collision_objects])
wps=np.load("traj_A.npy"); wps=[list(w) for w in wps]
L=sum(np.max(np.abs(np.array(wps[i+1])-np.array(wps[i]))) for i in range(len(wps)-1))
cum=[0.0]
for i in range(len(wps)-1): cum.append(cum[-1]+np.max(np.abs(np.array(wps[i+1])-np.array(wps[i]))))
T=max(3.0,L/0.4); times=[0.5+T*c/cum[-1] for c in cum]
a.move(wps[1:],times[1:]); a.move([wps[-1]],[1.5])
p,qq=a.fk_hand(); t=p+0.1034*quat_to_R(qq)[:,2]; print("tcp",np.round(t,3),"Zh",np.round(quat_to_R(qq)[:,2],3),"fingers",np.round(a.fingers()[0],4))
a.snap("agentview","carryA.png")
EOF

# openrua op 100
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
print("joints",np.round(a.joints(),3))
p,qq=a.fk_hand(); R=quat_to_R(qq); t=p+0.1034*R[:,2]; print("tcp",np.round(t,3))
for cam in ["agentview","frontview"]:
    P=a.xyz_map(cam); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
    m=pts[(pts[:,0]>-0.25)&(pts[:,0]<0.05)&(pts[:,1]>-0.30)&(pts[:,1]<0.0)&(pts[:,2]>1.115)&(pts[:,2]<1.30)]
    print(cam,len(m))
    for lo in np.arange(1.12,1.30,0.02):
        s_=m[(m[:,2]>lo)&(m[:,2]<lo+0.02)]
        if len(s_)>5: print(f"  z {lo:.2f}: x {s_[:,0].min():.3f}..{s_[:,0].max():.3f} y {s_[:,1].min():.3f}..{s_[:,1].max():.3f} n{len(s_)}")
EOF

# openrua op 101
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
wps=np.load("traj_A.npy"); print("target",np.round(wps[-1],3)); print("actual",np.round(a.joints(),3)); print("diff",np.round(np.array(a.joints())-wps[-1],3))
EOF

# openrua op 102
timeout 120 python3 -u -c "
from arm import *
a=Arm(); a.snap('robot0_eye_in_hand','eihA.png'); a.snap('birdview','birdA.png'); a.snap('frontview','frontA.png')" 2>&1 | grep -v XMLPARSER

# openrua op 103
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
P=a.xyz_map("frontview"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
m=pts[(pts[:,0]>-0.30)&(pts[:,0]<0.05)&(pts[:,1]>-0.30)&(pts[:,1]<0.10)&(pts[:,2]>1.115)&(pts[:,2]<1.235)]
Hh,xe,ye=np.histogram2d(m[:,0],m[:,1],bins=[np.arange(-0.30,0.06,0.02),np.arange(-0.30,0.12,0.02)])
print("rows x from -0.30 step 0.02; cols y from -0.30 step 0.02")
for i,row in enumerate(Hh): print(f"x{xe[i]:+.2f} "+"".join(f"{int(v):4d}" if v>0 else "   ." for v in row))
# also the color image lookup: find yellow pixels in frontview color to isolate mug
import cv2
img=cv2.imread("frontA.png"); hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
mask=(hsv[:,:,0]>20)&(hsv[:,:,0]<35)&(hsv[:,:,1]>80)&(hsv[:,:,2]>80)
yl=P[mask]; yl=yl[np.isfinite(yl).all(1)]
print("yellow pts",len(yl)); 
if len(yl): print("x",yl[:,0].min(),yl[:,0].max(),"y",yl[:,1].min(),yl[:,1].max(),"z",yl[:,2].min(),yl[:,2].max(),"mean",yl.mean(0))
EOF

# openrua op 104
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def ik_cont(p,q,maxj=0.7):
    seed=np.array(a.joints()); best=None
    for k in range(15):
        sd=seed if k==0 else seed+np.random.normal(0,0.1*k,7)
        try: s=np.array(a.ik_hand(np.array(p),q,seed=list(sd),at_tcp=True,tries=1))
        except Exception: continue
        j=np.abs(s-seed).max()
        if best is None or j<best[0]: best=(j,s)
        if j<maxj: break
    return best
def go(p,q,dur=2.0):
    b=ik_cont(p,q)
    if b is None: print("IK fail",p); return None
    j,s=b
    if j>0.9: print("BIG JUMP",j,np.round(s,2)); return None
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",np.round(a.fingers()[0],4)); return t
p,qq=a.fk_hand()
q=qq
for wp in [(-0.09,0.0,1.26),(-0.09,0.08,1.26),(-0.09,0.13,1.26)]:
    if go(wp,q,2.5) is None: break
for z in [1.15,1.05,1.00,0.985]:
    if go((-0.09,0.13,z),q,2.0) is None: break
a.snap("frontview","setdownA.png")
EOF

# openrua op 105
timeout 120 python3 -u -c "
from arm import *
a=Arm(); print(np.round(a.joints(),3)); a.snap('frontview','setdownA.png'); a.snap('birdview','birdB.png')" 2>&1 | grep -v XMLPARSER

# openrua op 106
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from moveit_msgs.msg import AttachedCollisionObject
from moveit_msgs.srv import GetStateValidity
a=Arm(); pl=Planner(a)
detach_all(pl,"mug")
# attach sphere approximating hanging mug
aco=AttachedCollisionObject(); aco.link_name="panda_hand"; aco.touch_links=["panda_hand","panda_leftfinger","panda_rightfinger","panda_link8","panda_link7"]
co=CollisionObject(); co.header.frame_id="panda_hand"; co.id="mugs"
sp=SolidPrimitive(); sp.type=SolidPrimitive.SPHERE; sp.dimensions=[0.10]
p=Pose(); p.position.x=-0.05; p.position.z=0.15; p.orientation.w=1.0
co.primitives=[sp]; co.primitive_poses=[p]; co.operation=CollisionObject.ADD; aco.object=co
ps=PlanningScene(); ps.is_diff=True; ps.robot_state.is_diff=True; ps.robot_state.attached_collision_objects=[aco]
req=ApplyPlanningScene.Request(); req.scene=ps; fut=pl.aps.call_async(req); rclpy.spin_until_future_complete(pl.n,fut,timeout_sec=30); print("attach",fut.result().success)
cl=a.n.create_client(GetStateValidity,"/check_state_validity"); cl.wait_for_service(5)
def valid(q):
    r=GetStateValidity.Request(); r.robot_state.joint_state=a.seed_state(list(q)); r.group_name="panda_arm"
    f=cl.call_async(r); rclpy.spin_until_future_complete(a.n,f,timeout_sec=30); res=f.result()
    return res.valid,[(c.contact_body_1,c.contact_body_2) for c in res.contacts]
print("current valid:",valid(a.joints()))
qT=np.load("qT.npy")
goal=np.array([-0.09,0.13,1.10])
sols=[]
for k in range(12):
    try: s=a.ik_hand(goal,qT,seed=list(np.random.uniform(-1,1,7)*np.array([1,0.8,1,0.8,1,0.8,1])+np.array([0,0,0,-1.8,0,1.8,0])),at_tcp=True,tries=2)
    except Exception: continue
    sols.append(np.array(s))
print(len(sols),"sols")
ok=False
for s in sols:
    v=valid(s); print("goal valid",v, np.round(s,2))
    if not v[0]: continue
    code,err=pl.go_joints(list(s),0.5,plan_only=True)
    if code==1:
        np.save("traj_B.npy",np.array([list(p.positions) for p in pl._last_traj.joint_trajectory.points])); ok=True; break
print("ok",ok)
EOF

# openrua op 107
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
wps=np.load("traj_B.npy"); wps=[list(w) for w in wps]
cum=[0.0]
for i in range(len(wps)-1): cum.append(cum[-1]+np.max(np.abs(np.array(wps[i+1])-np.array(wps[i]))))
T=max(3.0,cum[-1]/0.3); times=[0.5+T*c/cum[-1] for c in cum]
a.move(wps[1:],times[1:]); a.move([wps[-1]],[1.5])
p,qq=a.fk_hand(); t=p+0.1034*quat_to_R(qq)[:,2]; print("tcp",np.round(t,3),"Zh",np.round(quat_to_R(qq)[:,2],3),"fingers",np.round(a.fingers()[0],4),"joints",np.round(a.joints(),2))
a.snap("frontview","unfoldB.png")
EOF

# openrua op 108
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
p,qq=a.fk_hand(); q=qq
def go(p,dur=2.0):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>0.8: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",np.round(a.fingers()[0],4)); return t
for z in [1.06,1.03,1.00,0.99]:
    if go((-0.09,0.128,z),1.5) is None: break
a.snap("frontview","setdownB.png")
EOF

# openrua op 109
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
p,qq=a.fk_hand(); q=qq
def go(p,dur=2.0):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=1))
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",np.round(a.fingers()[0],4)); return t
go((-0.08,0.16,1.03),1.5)
go((-0.08,0.16,1.02),1.5)
a.gripper(0.04)
go((-0.08,0.16,1.15),2.0)
go((-0.08,0.16,1.30),2.0)
a.snap("frontview","laidC.png")
P=a.xyz_map("frontview"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
m=pts[(pts[:,0]>-0.25)&(pts[:,0]<0.05)&(pts[:,1]>-0.05)&(pts[:,1]<0.28)&(pts[:,2]>0.905)&(pts[:,2]<1.10)]
print("mug pts",len(m),"z max",m[:,2].max())
for lo in np.arange(0.90,1.08,0.02):
    s_=m[(m[:,2]>lo)&(m[:,2]<lo+0.02)]
    if len(s_)>5: print(f"  z {lo:.2f}: x {s_[:,0].min():.3f}..{s_[:,0].max():.3f} y {s_[:,1].min():.3f}..{s_[:,1].max():.3f} n{len(s_)}")
EOF

# openrua op 110
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
q=np.array(Q_DOWN,float)
s=a.ik_hand(np.array([-0.04,0.16,1.28]),q,seed=a.joints(),at_tcp=True,tries=6)
a.move([list(s)],[3.0]); a.move([list(s)],[1.0])
P=a.xyz_map("robot0_eye_in_hand"); np.save("eih_top5.npy",P); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
m=pts[(pts[:,2]>0.91)&(pts[:,2]<1.05)&(pts[:,0]>-0.2)&(pts[:,0]<0.08)&(pts[:,1]>0.05)&(pts[:,1]<0.28)]
rim=m[m[:,2]>0.995]; print("rim center",rim[:,:2].mean(0),"x",rim[:,0].min(),rim[:,0].max(),"y",rim[:,1].min(),rim[:,1].max(),"n",len(rim),"zmax",rim[:,2].max())
cx,cy=rim[:,:2].mean(0)
bar=m[(m[:,0]>cx+0.055)&(m[:,2]>0.92)&(m[:,2]<0.995)]
print("bar x",bar[:,0].min(),bar[:,0].max(),"y",bar[:,1].min(),bar[:,1].max(),"z",bar[:,2].min(),bar[:,2].max(),len(bar),"y mean",bar[:,1].mean())
a.snap("robot0_eye_in_hand","top5.png")
EOF

# openrua op 111
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); a.snap("robot0_eye_in_hand","top5.png")
P=np.load("eih_top5.npy"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
m=pts[(pts[:,2]>0.915)&(pts[:,2]<0.995)&(pts[:,0]>-0.25)&(pts[:,0]<0.1)&(pts[:,1]>0.0)&(pts[:,1]<0.3)]
# exclude mug body radius <0.055 from center
c=np.array([-0.072,0.1465]); r=np.linalg.norm(m[:,:2]-c,axis=1); h=m[(r>0.055)&(r<0.12)]
print("handle-ish pts",len(h))
if len(h): print("x",h[:,0].min(),h[:,0].max(),"y",h[:,1].min(),h[:,1].max(),"z",h[:,2].min(),h[:,2].max(),"mean",h.mean(0))
EOF

# openrua op 112
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a)
detach_all(pl,"mugs")
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
c,s_=np.cos(np.deg2rad(20)),np.sin(np.deg2rad(20))
R=np.array([[0,1,0],[c,0,s_],[s_,0,-c]]); qg=R_to_quat(R)
print("Zh",R[:,2],"Xh",R[:,0])
def go(p,q,dur=2.0,maxj=0.9):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j,np.round(s,2)); return None
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",np.round(a.fingers()[0],4)); return t
a.gripper(0.04)
# plan to pre-grasp above with MoveIt (no mug attached)
s=a.ik_hand(np.array([-0.072,0.2245,1.12]),qg,seed=a.joints(),at_tcp=True,tries=8)
code,err=pl.go_joints(list(s),0.5,plan_only=True); print("plan",code)
if code==1: pl.plan_exec(list(s))
else: a.move([list(s)],[4.0])
a.move([list(s)],[1.0]); print("tcp",np.round(tcp(),3))
for z in [1.02,0.975,0.955]:
    if go((-0.072,0.2245,z),qg,1.5) is None: break
a.snap("robot0_eye_in_hand","pre6.png")
EOF

# openrua op 113
timeout 120 python3 -u -c "
from arm import *
a=Arm(); a.snap('frontview','pre6_front.png'); a.snap('sideview','pre6_side.png')" 2>&1 | grep -v XMLPARSER

# openrua op 114
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
f=a.gripper(0.0)
EOF

# openrua op 115
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
p,qq=a.fk_hand(); q=qq
def go(p,dur=2.0):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>0.9: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",np.round(a.fingers()[0],4)); return t
t=tcp()
go((t[0],t[1],1.07),3.0)
P=a.xyz_map("frontview"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
m=pts[(pts[:,0]>-0.16)&(pts[:,0]<0.0)&(pts[:,1]>0.05)&(pts[:,1]<0.30)&(pts[:,2]>0.905)&(pts[:,2]<1.13)]
print("pts",len(m))
for lo in np.arange(0.90,1.14,0.02):
    s_=m[(m[:,2]>lo)&(m[:,2]<lo+0.02)]
    if len(s_)>5: print(f"  z {lo:.2f}: x {s_[:,0].min():.3f}..{s_[:,0].max():.3f} y {s_[:,1].min():.3f}..{s_[:,1].max():.3f} n{len(s_)}")
a.snap("frontview","lift6.png")
EOF

# openrua op 116
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm(); pl=Planner(a)
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,q,dur=2.0,maxj=0.9):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j,np.round(s,2)); return None
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",np.round(a.fingers()[0],4)); return t
p,qq=a.fk_hand(); R0=quat_to_R(qq); t=tcp()
go((t[0],t[1],1.27),qq,3.0)
print("joints",np.round(a.joints(),2))
for ang in [45,90,135,180]:
    R=Rot.from_euler('z',ang,degrees=True).as_matrix()@R0; q=R_to_quat(R)
    if go((-0.066,0.237,1.27),q,3.0) is None: print("stopped at",ang); break
print("Zh",np.round(quat_to_R(a.fk_hand()[1])[:,2],3),"joints",np.round(a.joints(),2))
EOF

# openrua op 117
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.srv import GetStateValidity
a=Arm(); pl=Planner(a)
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
p,qq=a.fk_hand(); R=quat_to_R(qq); t=tcp()
h=R[:,2].copy(); h[2]=0; h/=np.linalg.norm(h)
d=-0.087*h+np.array([0,0,-0.023])
mug_h=R.T@(d+0.1034*R[:,2])   # from hand origin: TCP offset + d
ax=R.T@np.array([0,0,1.0])
# quaternion rotating hand-frame z to ax
def quat_from_z(ax):
    z=np.array([0,0,1.0]); v=np.cross(z,ax); c=np.dot(z,ax)
    if np.linalg.norm(v)<1e-6: return [0,0,0,1]
    s=np.sqrt((1+c)*2); return [v[0]/s,v[1]/s,v[2]/s,s/2]
qc=quat_from_z(ax); print("mug in hand",np.round(mug_h,3),"axis",np.round(ax,3))
attach_cylinder(pl,"mug",mug_h,qc,0.12,0.058)
cl=a.n.create_client(GetStateValidity,"/check_state_validity"); cl.wait_for_service(5)
def valid(q):
    r=GetStateValidity.Request(); r.robot_state.joint_state=a.seed_state(list(q)); r.group_name="panda_arm"
    f=cl.call_async(r); rclpy.spin_until_future_complete(a.n,f,timeout_sec=30); res=f.result()
    return res.valid,[(c.contact_body_1,c.contact_body_2) for c in res.contacts]
print("current valid",valid(a.joints()))
c,s_=np.cos(np.deg2rad(20)),np.sin(np.deg2rad(20))
R0=np.array([[0,1,0],[c,0,s_],[s_,0,-c]])
RT=Rot.from_euler('z',180,degrees=True).as_matrix()@R0; qT=R_to_quat(RT); np.save("qT2.npy",qT)
print("target Zh",np.round(RT[:,2],3))
sols=[]
for k in range(16):
    seed=list(np.array(a.joints())+np.random.normal(0,0.7,7)) if k else a.joints()
    try: s=np.array(a.ik_hand(np.array([-0.066,0.237,1.27]),qT,seed=seed,at_tcp=True,tries=2))
    except Exception: continue
    if not any(np.abs(s-x).max()<0.05 for x in sols): sols.append(s)
print(len(sols),"distinct sols")
sols.sort(key=lambda s: np.abs(s-np.array(a.joints())).max())
ok=False
for s in sols:
    v=valid(s)
    if not v[0]: print("invalid",v[1]); continue
    code,err=pl.go_joints(list(s),0.5,plan_only=True)
    if code==1:
        np.save("traj_C.npy",np.array([list(p.positions) for p in pl._last_traj.joint_trajectory.points])); ok=True; print("sol",np.round(s,2)); break
print("ok",ok)
EOF

# openrua op 118
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,q,dur=2.0,maxj=0.9):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j,np.round(s,2)); return None
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",np.round(a.fingers()[0],4),"j7",np.round(a.joints()[6],2)); return t
c,s_=np.cos(np.deg2rad(20)),np.sin(np.deg2rad(20))
R0=np.array([[0,1,0],[c,0,s_],[s_,0,-c]])
P=(-0.066,0.237,1.27)
for ang in [90,45,0,-45,-90,-135,-180]:
    R=Rot.from_euler('z',ang,degrees=True).as_matrix()@R0; q=R_to_quat(R)
    if go(P,q,3.0) is None: print("stopped at",ang); break
print("Zh",np.round(quat_to_R(a.fk_hand()[1])[:,2],3),"joints",np.round(a.joints(),2))
EOF

# openrua op 119
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,q,dur=2.0,maxj=1.2):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j,np.round(s,2)); return None
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",np.round(a.fingers()[0],4),"j7",np.round(a.joints()[6],2)); return t
c,s_=np.cos(np.deg2rad(20)),np.sin(np.deg2rad(20))
R0=np.array([[0,1,0],[c,0,s_],[s_,0,-c]])
P=(-0.066,0.237,1.27)
for ang in [-67.5,-90,-112.5,-135,-157.5,-180]:
    R=Rot.from_euler('z',ang,degrees=True).as_matrix()@R0; q=R_to_quat(R)
    if go(P,q,3.0) is None: print("stopped at",ang); break
print("Zh",np.round(quat_to_R(a.fk_hand()[1])[:,2],3),"joints",np.round(a.joints(),2))
a.snap("frontview","yaw6.png")
EOF

# openrua op 120
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
p,qq=a.fk_hand(); R=quat_to_R(qq); t=p+0.1034*R[:,2]; print("tcp",np.round(t,3),"Zh",np.round(R[:,2],3))
for cam in ["frontview","agentview"]:
    P=a.xyz_map(cam); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
    m=pts[(pts[:,0]>-0.20)&(pts[:,0]<0.05)&(pts[:,1]>0.10)&(pts[:,1]<0.45)&(pts[:,2]>1.10)&(pts[:,2]<1.36)]
    print(cam,len(m))
    for lo in np.arange(1.14,1.34,0.02):
        s_=m[(m[:,2]>lo)&(m[:,2]<lo+0.02)]
        if len(s_)>5: print(f"  z {lo:.2f}: x {s_[:,0].min():.3f}..{s_[:,0].max():.3f} ({s_[:,0].mean():.3f}) y {s_[:,1].min():.3f}..{s_[:,1].max():.3f} ({s_[:,1].mean():.3f}) n{len(s_)}")
EOF

# openrua op 121
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,q,dur=2.0,maxj=1.2):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j,np.round(s,2)); return None
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",np.round(a.fingers()[0],4)); return t
p,qq=a.fk_hand(); q=qq
for wp in [(-0.09,0.05,1.27),(-0.11,-0.15,1.27),(-0.13,-0.30,1.27),(-0.13,-0.397,1.27)]:
    if go(wp,q,3.5) is None: break
for z in [1.22,1.195,1.185,1.178]:
    if go((-0.13,-0.397,z),q,2.0) is None: break
a.snap("agentview","placeD.png"); a.snap("sideview","placeD_side.png")
EOF

# openrua op 122
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,q,dur=2.0,maxj=1.2):
    s=np.array(a.ik_hand(np.array(p),q,seed=a.joints(),at_tcp=True,tries=1))
    a.move([list(s)],[dur]); a.move([list(s)],[1.0]); t=tcp(); print("tcp",np.round(t,3),"fingers",np.round(a.fingers()[0],4)); return t
a.gripper(0.04)
p,qq=a.fk_hand(); q=qq; t=tcp()
go((t[0],t[1]-0.02,1.30),q,2.5)
go((-0.13,-0.50,1.40),q,3.0)
P=a.xyz_map("birdview"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
m=pts[(pts[:,0]>-0.25)&(pts[:,0]<0.0)&(pts[:,1]>-0.45)&(pts[:,1]<-0.15)&(pts[:,2]>1.115)&(pts[:,2]<1.25)]
print("mug pts",len(m),"zmax",m[:,2].max())
rim=m[m[:,2]>m[:,2].max()-0.015]; print("rim center",rim[:,:2].mean(0),"x",rim[:,0].min(),rim[:,0].max(),"y",rim[:,1].min(),rim[:,1].max())
for lo in np.arange(1.12,1.22,0.02):
    s_=m[(m[:,2]>lo)&(m[:,2]<lo+0.02)]
    if len(s_)>5: print(f"  z {lo:.2f}: x {s_[:,0].min():.3f}..{s_[:,0].max():.3f} y {s_[:,1].min():.3f}..{s_[:,1].max():.3f} n{len(s_)}")
a.snap("birdview","placedD_bird.png"); a.snap("agentview","placedD_agent.png")
EOF

# openrua op 123
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
a.snap("agentview","placedD_agent.png"); a.snap("birdview","placedD_bird.png")
P=a.xyz_map("birdview"); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
m=pts[(pts[:,0]>-0.35)&(pts[:,0]<0.1)&(pts[:,1]>-0.6)&(pts[:,1]<0.0)&(pts[:,2]>1.115)&(pts[:,2]<1.30)]
print("pts above microwave top",len(m))
if len(m): print("x",m[:,0].min(),m[:,0].max(),"y",m[:,1].min(),m[:,1].max(),"z",m[:,2].min(),m[:,2].max())
EOF

# openrua op 124
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
q=np.array(Q_DOWN,float)
s=a.ik_hand(np.array([-0.05,-0.20,1.45]),q,seed=a.joints(),at_tcp=True,tries=6)
a.move([list(s)],[3.0]); a.move([list(s)],[1.0])
for cam in ["birdview","agentview"]:
    P=a.xyz_map(cam); np.save(f"{cam}_fall.npy",P); pts=P.reshape(-1,3); pts=pts[np.isfinite(pts).all(1)]
    m=pts[(pts[:,0]>-0.30)&(pts[:,0]<0.05)&(pts[:,1]>-0.60)&(pts[:,1]<-0.36)&(pts[:,2]>0.905)&(pts[:,2]<1.10)]
    print(cam,"pts",len(m))
    if len(m):
        print(" x",m[:,0].min(),m[:,0].max(),"y",m[:,1].min(),m[:,1].max(),"z",m[:,2].min(),m[:,2].max())
        for lo in np.arange(0.90,1.02,0.02):
            s_=m[(m[:,2]>lo)&(m[:,2]<lo+0.02)]
            if len(s_)>5: print(f"  z {lo:.2f}: x {s_[:,0].min():.3f}..{s_[:,0].max():.3f} y {s_[:,1].min():.3f}..{s_[:,1].max():.3f} n{len(s_)}")
a.snap("agentview","fall_agent.png"); a.snap("birdview","fall_bird.png")
EOF

# openrua op 125
timeout 120 python3 -u -c "
from arm import *
a=Arm(); a.snap('frontview','fall_front.png'); a.snap('sideview','fall_side.png')" 2>&1 | grep -v XMLPARSER

# openrua op 126
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
print("joints",np.round(a.joints(),3),"tcp",np.round(tcp(),3),"fingers",a.fingers())
s=np.array(a.ik_hand(np.array([-0.13,-0.42,1.32]),Q_DOWN,seed=a.joints(),at_tcp=True,tries=1))
print("jump",np.abs(s-np.array(a.joints())).max())
a.move([list(s)],[3.0]); a.move([list(s)],[1.0])
print("tcp",np.round(tcp(),3))
a.snap("robot0_eye_in_hand","eih_fall.png")
X=a.xyz_map("robot0_eye_in_hand"); np.save("eih_fall.npy",X)
a.snap("birdview","bird_fall2.png")
B=a.xyz_map("birdview"); np.save("bird_fall2.npy",B)
# analyze region
for name,M in [("eih",X),("bird",B)]:
    P=M.reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.25)&(P[:,0]<-0.03)&(P[:,1]>-0.55)&(P[:,1]<-0.355)&(P[:,2]>0.905)&(P[:,2]<1.09)
    Q=P[m]; print(name,"n",len(Q))
    for z0 in np.arange(0.91,1.06,0.02):
        S=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
        if len(S): print(" z%.2f n%4d x %.3f..%.3f y %.3f..%.3f"%(z0,len(S),S[:,0].min(),S[:,0].max(),S[:,1].min(),S[:,1].max()))
EOF

# openrua op 127
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm()
def R_pitch(phi_deg, ysign=1):
    ph=np.radians(phi_deg)
    Zh=np.array([0,np.cos(ph),-np.sin(ph)]); Yh=np.array([ysign,0,0.]); Xh=np.cross(Yh,Zh)
    return np.column_stack([Xh,Yh,Zh])
seeds=[list(a.joints()), [0,-0.3,0,-2.2,0,1.9,0.8], [0.3,0.5,-0.5,-1.8,0.3,2.3,0.5], [-0.5,0.8,0.5,-1.5,-0.5,2.4,0.0]]
for ysign in (1,-1):
  for phi in (0,15,30):
    q=R_to_quat(R_pitch(phi,ysign))
    for y in (-0.33,-0.40,-0.47):
        ok=None
        for s in seeds:
            try:
                sol=a.ik_hand(np.array([-0.125,y,1.0]),q,seed=s,at_tcp=True,tries=1); ok=np.round(sol,2); break
            except Exception as e: pass
        print("ysign",ysign,"phi",phi,"y",y,"->",ok)
EOF

# openrua op 128
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
import numpy as np
X=np.load("eih_fall.npy"); P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.22)&(P[:,0]<-0.03)&(P[:,1]>-0.55)&(P[:,1]<-0.30)&(P[:,2]>0.905)&(P[:,2]<1.05)
Q=P[m]
for z0 in np.arange(0.91,1.03,0.01):
    S=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    if len(S):
        h,e=np.histogram(S[:,1],bins=np.arange(-0.55,-0.30,0.01))
        print("z%.2f n%4d x %.3f..%.3f y %.3f..%.3f"%(z0,len(S),S[:,0].min(),S[:,0].max(),S[:,1].min(),S[:,1].max()), " ".join("%d"%v for v in h))
# circle fit on rim points z 0.995..1.01 excluding y>-0.352 band
S=Q[(Q[:,2]>=0.993)&(Q[:,2]<1.012)&(Q[:,1]<-0.352)]
A=np.c_[2*S[:,0],2*S[:,1],np.ones(len(S))]; b=S[:,0]**2+S[:,1]**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print("rim fit center",np.round(c[:2],4),"r",round(r,4),"n",len(S))
# handle points: z 0.93..0.99, y< center_y-0.055
Hh=Q[(Q[:,2]>0.925)&(Q[:,2]<0.992)&(Q[:,1]<c[1]-0.058)]
print("handle pts n",len(Hh),"x %.3f..%.3f y %.3f..%.3f z %.3f..%.3f"%(Hh[:,0].min(),Hh[:,0].max(),Hh[:,1].min(),Hh[:,1].max(),Hh[:,2].min(),Hh[:,2].max()))
print("handle x mean",Hh[:,0].mean().round(4))
EOF

# openrua op 129
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
for cam in ("agentview","frontview"):
    M=a.xyz_map(cam); np.save(cam+"_now.npy",M)
    P=M.reshape(-1,3); P=P[np.isfinite(P).all(1)]
    m=(P[:,0]>-0.24)&(P[:,0]<-0.05)&(P[:,1]>-0.40)&(P[:,1]<-0.10)&(P[:,2]>0.93)&(P[:,2]<1.12)
    Q=P[m]; print(cam,"n",len(Q))
    for z0 in np.arange(0.93,1.12,0.01):
        S=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
        if len(S):
            h,e=np.histogram(S[:,1],bins=np.arange(-0.40,-0.10,0.02))
            print(" z%.2f n%4d y %.3f..%.3f"%(z0,len(S),S[:,1].min(),S[:,1].max())," ".join("%d"%v for v in h))
EOF

# openrua op 130
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
import numpy as np
X=np.load("eih_fall.npy"); P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.20)&(P[:,0]<-0.06)&(P[:,1]>-0.36)&(P[:,1]<-0.10)
Q=P[m]
F=Q[(Q[:,2]>0.935)&(Q[:,2]<0.96)]
print("floor pts n",len(F),"y %.3f..%.3f z %.3f..%.3f"%(F[:,1].min(),F[:,1].max(),F[:,2].min(),F[:,2].max()))
T=Q[(Q[:,2]>1.09)]
print("top pts n",len(T),"y %.3f..%.3f z %.3f..%.3f"%(T[:,1].min(),T[:,1].max(),T[:,2].min(),T[:,2].max()))
O=Q[(Q[:,2]>0.96)&(Q[:,2]<1.09)]
print("other pts n",len(O))
if len(O):
    for z0 in np.arange(0.96,1.09,0.01):
        S=O[(O[:,2]>=z0)&(O[:,2]<z0+0.01)]
        if len(S): print(" z%.2f n%4d y %.3f..%.3f"%(z0,len(S),S[:,1].min(),S[:,1].max()))
ymax=F[:,1].max(); cam=np.array([-0.13,-0.42,1.32])
zedge=cam[2]-(cam[2]-0.945)*((-0.33-cam[1])/(ymax-cam[1]))
print("frame bottom edge estimate z=%.3f (assuming front plane y=-0.33)"%zedge)
EOF

# openrua op 131
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
import numpy as np
X=np.load("eih_fall.npy"); P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
for xlo,xhi in [(-0.25,-0.19),(-0.19,-0.06),(-0.06,-0.03),(-0.03,0.05)]:
    m=(P[:,0]>xlo)&(P[:,0]<xhi)&(P[:,1]>-0.42)&(P[:,1]<-0.25)&(P[:,2]>0.89)&(P[:,2]<1.12)
    Q=P[m]; print("x %.2f..%.2f n %d"%(xlo,xhi,len(Q)))
    for y0 in np.arange(-0.42,-0.25,0.01):
        S=Q[(Q[:,1]>=y0)&(Q[:,1]<y0+0.01)]
        if len(S):
            zs=np.sort(S[:,2]); 
            # cluster z
            cl=[]; start=zs[0]; prev=zs[0]
            for z in zs[1:]:
                if z-prev>0.008: cl.append((start,prev)); start=z
                prev=z
            cl.append((start,prev))
            print("  y%.2f n%4d z: "%(y0,len(S))+" ".join("%.3f-%.3f"%c for c in cl))
EOF

# openrua op 132
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
import numpy as np
X=np.load("eih_fall.npy"); P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
face=P[(P[:,1]>-0.337)&(P[:,1]<-0.322)&(P[:,2]>0.96)&(P[:,2]<1.07)&(P[:,0]>-0.4)&(P[:,0]<0.2)]
h,e=np.histogram(face[:,0],bins=np.arange(-0.40,0.2,0.01))
print("front face pts (z .96-1.07) x-hist:"); print(" ".join("%.2f:%d"%(e[i],h[i]) for i in range(len(h)) if h[i]>0))
# interior floor points: z 0.94-0.95, y>-0.32
fl=P[(P[:,2]>0.938)&(P[:,2]<0.95)&(P[:,1]>-0.32)&(P[:,1]<-0.2)]
print("floor x %.3f..%.3f"%(fl[:,0].min(),fl[:,0].max()))
# interior side walls: points with y>-0.31, z 0.95-1.05
sw=P[(P[:,1]>-0.31)&(P[:,1]<-0.2)&(P[:,2]>0.95)&(P[:,2]<1.05)]
h,e=np.histogram(sw[:,0],bins=np.arange(-0.40,0.2,0.01))
print("interior wall pts x-hist:"); print(" ".join("%.2f:%d"%(e[i],h[i]) for i in range(len(h)) if h[i]>0))
# frame top edge: points y -0.337..-0.322, z>1.07
ft=P[(P[:,1]>-0.337)&(P[:,1]<-0.322)&(P[:,2]>1.07)&(P[:,2]<1.105)&(P[:,0]>-0.19)&(P[:,0]<-0.05)]
print("frame top face z %.3f..%.3f n %d"%(ft[:,2].min(),ft[:,2].max(),len(ft)))
EOF

# openrua op 133
grep -n "def \|class " arm.py | head -60

# openrua op 134
cat >> /workspace/arm.py <<'EOF'


class Validity:
    """Wrapper for /check_state_validity (collision + limits) for arm joint vectors."""
    def __init__(self, arm):
        from moveit_msgs.srv import GetStateValidity
        self.a = arm; self.n = arm.n
        self.srv = GetStateValidity
        self.cl = self.n.create_client(GetStateValidity, "/check_state_validity")
        self.cl.wait_for_service(10)

    def check(self, q, verbose=True):
        req = self.srv.Request()
        req.group_name = M["planning"]["group"]
        req.robot_state.joint_state = self.a.seed_state(q)
        req.robot_state.is_diff = True
        fut = self.cl.call_async(req)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=30)
        r = fut.result()
        if r is None:
            print("validity: no response", flush=True); return None, []
        pairs = [(c.contact_body_1, c.contact_body_2, round(c.depth, 4)) for c in r.contacts]
        if verbose:
            print("valid:", r.valid, pairs, flush=True)
        return r.valid, pairs
EOF
echo ok

# openrua op 135
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a); V=Validity(a)
detach_all(pl,"mug")
# hollow microwave: interior x -0.235..-0.03, y -0.33..-0.127, z 0.943..1.09 ; outer x -0.288..0.079, y -0.36..-0.128(back wall thin), z 0.90..1.107
boxes=[
 ("mw_top",   (-0.105,-0.2435,1.0985),(0.367,0.233,0.017)),
 ("mw_left",  (-0.2615,-0.2435,1.0),  (0.053,0.233,0.2)),
 ("mw_right", (0.0245,-0.2435,1.0),   (0.109,0.233,0.2)),
 ("mw_back",  (-0.105,-0.115,1.0),    (0.367,0.024,0.2)),
 ("mw_floor", (-0.105,-0.2435,0.916), (0.367,0.233,0.036)),
 ("table",(0,0,0.85),(1.0,1.6,0.1)),
 ("greymug",(0.02,0.33,0.955),(0.11,0.11,0.12)),
]
H=np.array([-0.283,-0.319]); d=np.array([-0.405,-0.914])
for i,t in enumerate((0.04,0.12,0.20)):
    c=H+t*d; boxes.append(("door%d"%i,(c[0],c[1],1.0),(0.09,0.09,0.22)))
pl.set_scene(boxes, remove=("microwave",))
q0=a.joints(); print("current valid?"); V.check(q0)
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
print("tcp now",np.round(tcp(),3))
G=np.array([-0.123,-0.462,0.948])
def chain(wps, seed):
    sols=[]; s=np.array(seed)
    for (p,phi) in wps:
        q=R_to_quat(Rp(phi))
        try:
            sol=np.array(a.ik_hand(np.array(p),q,seed=list(s),at_tcp=True,tries=1))
        except Exception as e:
            print("IK FAIL at",np.round(p,3),phi,e); return sols
        jump=np.abs(sol-s).max()
        v,pairs=V.check(list(sol),verbose=False)
        print("p",np.round(p,3),"phi",phi,"jump %.2f"%jump,"valid",v,pairs[:3],"q",np.round(sol,2))
        sols.append(sol); s=sol
    return sols
for phi in (25,30,35):
    print("=== phi",phi)
    Z=Rp(phi)[:,2]
    wps=[(G-0.08*Z,phi),(G,phi),(G+[0,0,0.05],phi)]
    for y in np.arange(-0.44,-0.339,0.02): wps.append((np.array([-0.123,y,0.998]),phi))
    wps.append((np.array([-0.123,-0.347,0.985]),phi))
    chain(wps,q0)
EOF

# openrua op 136
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
import numpy as np
X=np.load("eih_fall.npy"); P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
for name,xlo,xhi in [("leftframe",-0.27,-0.24),("rightframe",-0.03,0.0),("mid",-0.19,-0.06)]:
    S=P[(P[:,0]>xlo)&(P[:,0]<xhi)&(P[:,1]>-0.40)&(P[:,1]<-0.30)&(P[:,2]>0.95)]
    print(name)
    for z0 in np.arange(0.95,1.12,0.01):
        T=S[(S[:,2]>=z0)&(S[:,2]<z0+0.01)]
        if len(T): 
            h,e=np.histogram(T[:,1],bins=np.arange(-0.40,-0.30,0.005))
            print("  z%.2f n%4d y %.3f..%.3f "%(z0,len(T),T[:,1].min(),T[:,1].max())+" ".join("%d"%v for v in h))
EOF

# openrua op 137
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
from scipy.spatial.transform import Rotation as Rot
a=Arm(); pl=Planner(a); V=Validity(a)
detach_all(pl,"mug")
yc=-0.226; yl=0.198
boxes=[
 ("mw_top",   (-0.105,yc,1.0985),(0.367,yl,0.017)),
 ("mw_left",  (-0.2615,yc,1.0),  (0.053,yl,0.2)),
 ("mw_right", (0.0245,yc,1.0),   (0.109,yl,0.2)),
 ("mw_back",  (-0.105,-0.115,1.0),(0.367,0.024,0.2)),
 ("mw_floor", (-0.105,yc,0.915), (0.367,yl,0.03)),
]
pl.set_scene(boxes)
q0=a.joints()
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
G=np.array([-0.123,-0.462,0.948]); phi=25
def attach_mug(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    off=np.array([0,0.08,0.0035])  # world offset from TCP to mug center
    xh=np.array([0,c,s]); yh=np.array([1,0,0]); zh=np.array([0,s,-c])
    p=np.array([off@xh, off@yh, off@zh+0.1034])
    q=Rot.from_euler('y',180-phi,degrees=True).as_quat()
    attach_cylinder(pl,"mug",p,q,0.103,0.05)
def chain(wps, seed):
    sols=[]; s=np.array(seed)
    for (p,phi) in wps:
        q=R_to_quat(Rp(phi))
        try: sol=np.array(a.ik_hand(np.array(p),q,seed=list(s),at_tcp=True,tries=1))
        except Exception as e: print("IK FAIL at",np.round(p,3),phi,e); return sols
        jump=np.abs(sol-s).max(); v,pairs=V.check(list(sol),verbose=False)
        print("p",np.round(p,3),"phi",phi,"jump %.2f"%jump,"valid",v,pairs[:4],"q",np.round(sol,2))
        sols.append(sol); s=sol
    return sols
Z=Rp(phi)[:,2]
print("--- arm only, approach")
sols=chain([(G-0.08*Z,phi),(G,phi)],q0)
print("--- with mug attached, insertion")
attach_mug(phi)
wps=[(G+[0,0,0.05],phi)]
for y in np.arange(-0.44,-0.299,0.02): wps.append((np.array([-0.123,y,0.998]),phi))
wps.append((np.array([-0.123,-0.31,0.985]),phi))
sols2=chain(wps,sols[-1])
np.save("plan_sols.npy",np.array(sols+sols2))
detach_all(pl,"mug")
EOF

# openrua op 138
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a); V=Validity(a)
detach_all(pl,"mug")
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
phi=25; Q=R_to_quat(Rp(phi)); Z=Rp(phi)[:,2]
G=np.array([-0.123,-0.462,0.948])
def go(p,dur=1.5,maxj=0.6):
    s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    t=tcp(); print("tcp",np.round(t,3),"fingers %.4f"%a.fingers()[0]); return t
# 1. MoveIt to pre-grasp
pre=G-0.08*Z
s=np.array(a.ik_hand(pre,Q,seed=a.joints(),at_tcp=True,tries=1))
print("pregrasp q",np.round(s,2)); V.check(list(s))
pl.plan_exec(list(s),speed=0.4)
print("tcp",np.round(tcp(),3))
# 2. descend along Z to grasp in 2 steps
go(G-0.04*Z); go(G)
# 3. close
a.gripper(0.0); a.spin(0.3); print("fingers after close %.4f"%a.fingers()[0])
a.snap("robot0_eye_in_hand","grasp7.png")
EOF

# openrua op 139
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
phi=25; Q=R_to_quat(Rp(phi))
def go(p,dur=1.5,maxj=0.6):
    s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    t=tcp(); print("tcp",np.round(t,3),"fingers %.4f"%a.fingers()[0]); return t
go([-0.123,-0.462,0.975]); go([-0.123,-0.462,0.998])
a.snap("robot0_eye_in_hand","lift7.png")
X=a.xyz_map("robot0_eye_in_hand"); np.save("eih_lift7.npy",X)
P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.25)&(P[:,0]<0.0)&(P[:,1]>-0.50)&(P[:,1]<-0.33)&(P[:,2]>0.92)&(P[:,2]<1.09)
S=P[m]
for z0 in np.arange(0.92,1.09,0.02):
    T=S[(S[:,2]>=z0)&(S[:,2]<z0+0.02)]
    if len(T): print(" z%.2f n%4d x %.3f..%.3f y %.3f..%.3f"%(z0,len(T),T[:,0].min(),T[:,0].max(),T[:,1].min(),T[:,1].max()))
# rim fit
R=S[(S[:,2]>1.035)&(S[:,2]<1.075)]
if len(R)>20:
    A=np.c_[2*R[:,0],2*R[:,1],np.ones(len(R))]; b=R[:,0]**2+R[:,1]**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print("rim fit center",np.round(c[:2],4),"r %.4f n %d zmean %.3f"%(r,len(R),R[:,2].mean()))
EOF

# openrua op 140
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
a.snap("birdview","bird_lift7.png")
B=a.xyz_map("birdview"); P=B.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.25)&(P[:,0]<0.0)&(P[:,1]>-0.50)&(P[:,1]<-0.33)&(P[:,2]>0.92)&(P[:,2]<1.09)
S=P[m]
for z0 in np.arange(0.92,1.09,0.01):
    T=S[(S[:,2]>=z0)&(S[:,2]<z0+0.01)]
    if len(T): print(" z%.2f n%4d x %.3f..%.3f y %.3f..%.3f"%(z0,len(T),T[:,0].min(),T[:,0].max(),T[:,1].min(),T[:,1].max()))
R=S[(S[:,2]>1.04)&(S[:,2]<1.06)]
if len(R)>10:
    A=np.c_[2*R[:,0],2*R[:,1],np.ones(len(R))]; b=R[:,0]**2+R[:,1]**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print("rim fit center",np.round(c[:2],4),"r %.4f n %d zmean %.3f"%(r,len(R),R[:,2].mean()))
EOF

# openrua op 141
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
phi=25; Q=R_to_quat(Rp(phi))
def go(p,dur=1.5,maxj=0.6):
    s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    err=np.abs(np.array(a.joints())-s).max()
    t=tcp(); print("tcp",np.round(t,3),"fingers %.4f"%a.fingers()[0],"err %.4f"%err); return err
for y in [-0.44,-0.42,-0.40,-0.38,-0.36,-0.345]:
    e=go([-0.123,y,0.998])
    if e is None or e>0.03: print("STOP: contact/jump"); break
a.snap("robot0_eye_in_hand","ins7.png")
a.snap("birdview","bird_ins7.png")
EOF

# openrua op 142
timeout 600 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
B=a.xyz_map("birdview"); P=B.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.20)&(P[:,0]<-0.05)&(P[:,1]>-0.50)&(P[:,1]<-0.25)&(P[:,2]>0.92)&(P[:,2]<1.12)
S=P[m]
for z0 in np.arange(0.92,1.12,0.01):
    T=S[(S[:,2]>=z0)&(S[:,2]<z0+0.01)]
    if len(T): print(" z%.2f n%4d x %.3f..%.3f y %.3f..%.3f"%(z0,len(T),T[:,0].min(),T[:,0].max(),T[:,1].min(),T[:,1].max()))
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
print("tcp",np.round(tcp(),4),"joints",np.round(a.joints(),3))
EOF

# openrua op 143
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
a.snap("agentview","ins7_agent.png"); a.snap("frontview","ins7_front.png"); a.snap("sideview","ins7_side.png")
EOF

# openrua op 144
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER | tail -20
from arm import *
a=Arm()
for cam,out in [("agentview","ins7_agent.png"),("frontview","ins7_front.png"),("sideview","ins7_side.png")]:
    try: a.snap(cam,out); print("ok",cam)
    except Exception as e: print("fail",cam,e)
EOF
ls -la ins7_*.png

# openrua op 145
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
X=a.xyz_map("robot0_eye_in_hand"); np.save("eih_ins7.npy",X)
P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.20)&(P[:,0]<-0.05)&(P[:,1]>-0.50)&(P[:,1]<-0.25)&(P[:,2]>0.90)&(P[:,2]<1.085)
S=P[m]
for z0 in np.arange(0.90,1.085,0.01):
    T=S[(S[:,2]>=z0)&(S[:,2]<z0+0.01)]
    if len(T): print(" z%.2f n%5d x %.3f..%.3f y %.3f..%.3f"%(z0,len(T),T[:,0].min(),T[:,0].max(),T[:,1].min(),T[:,1].max()))
EOF

# openrua op 146
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
phi=25; Q=R_to_quat(Rp(phi))
def go(p,dur=1.5,maxj=0.6):
    s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    err=np.abs(np.array(a.joints())-s).max()
    t=tcp(); print("tcp",np.round(t,3),"fingers %.4f"%a.fingers()[0],"err %.4f"%err); return err
print("raise"); go([-0.123,-0.41,1.03])
print("advance"); 
for y in (-0.40,-0.39):
    e=go([-0.123,y,1.03])
    if e>0.04: print("STOP"); raise SystemExit
a.snap("robot0_eye_in_hand","ins7b.png")
EOF

# openrua op 147
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
phi=25; Q=R_to_quat(Rp(phi))
def go(p,dur=1.5,maxj=0.6):
    s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    err=np.abs(np.array(a.joints())-s).max()
    t=tcp(); print("tcp",np.round(t,3),"fingers %.4f"%a.fingers()[0],"err %.4f"%err); return err
print("lower")
for z in (1.02,1.01,1.005):
    e=go([-0.123,-0.39,z])
    if e>0.04: print("STOP lower"); raise SystemExit
print("advance")
for y in (-0.375,-0.36,-0.345):
    e=go([-0.123,y,1.005])
    if e>0.04: print("STOP adv"); break
a.snap("robot0_eye_in_hand","ins7c.png")
EOF

# openrua op 148
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
phi=25; Q=R_to_quat(Rp(phi))
def go(p,dur=1.5,maxj=0.6):
    s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    err=np.abs(np.array(a.joints())-s).max()
    t=tcp(); print("tcp",np.round(t,3),"fingers %.4f"%a.fingers()[0],"err %.4f"%err); return err
go([-0.123,-0.345,1.0]); go([-0.123,-0.345,0.995])
a.snap("robot0_eye_in_hand","ins7d.png")
X=a.xyz_map("robot0_eye_in_hand")
P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.20)&(P[:,0]<-0.05)&(P[:,1]>-0.45)&(P[:,1]<-0.15)&(P[:,2]>0.90)&(P[:,2]<1.085)
S=P[m]
for z0 in np.arange(0.94,1.085,0.01):
    T=S[(S[:,2]>=z0)&(S[:,2]<z0+0.01)]
    if len(T): print(" z%.2f n%5d x %.3f..%.3f y %.3f..%.3f"%(z0,len(T),T[:,0].min(),T[:,0].max(),T[:,1].min(),T[:,1].max()))
EOF

# openrua op 149
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
phi=25; Q=R_to_quat(Rp(phi)); Z=Rp(phi)[:,2]
def go(p,dur=1.5,maxj=0.6):
    s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    err=np.abs(np.array(a.joints())-s).max()
    t=tcp(); print("tcp",np.round(t,3),"fingers %.4f"%a.fingers()[0],"err %.4f"%err); return err
a.gripper(0.04); a.spin(0.3); print("fingers",a.fingers())
t=tcp()
go(t-0.03*Z); go(t-0.07*Z); go(t-0.07*Z+[0,0,0.05])
a.snap("robot0_eye_in_hand","rel7.png")
X=a.xyz_map("robot0_eye_in_hand"); np.save("eih_rel7.npy",X)
P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.22)&(P[:,0]<-0.03)&(P[:,1]>-0.40)&(P[:,1]<-0.15)&(P[:,2]>0.945)&(P[:,2]<1.085)
S=P[m]
for z0 in np.arange(0.95,1.085,0.01):
    T=S[(S[:,2]>=z0)&(S[:,2]<z0+0.01)]
    if len(T): print(" z%.2f n%5d x %.3f..%.3f y %.3f..%.3f"%(z0,len(T),T[:,0].min(),T[:,0].max(),T[:,1].min(),T[:,1].max()))
R=S[(S[:,2]>1.038)&(S[:,2]<1.055)]
if len(R)>20:
    A=np.c_[2*R[:,0],2*R[:,1],np.ones(len(R))]; b=R[:,0]**2+R[:,1]**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print("rim fit center",np.round(c[:2],4),"r %.4f n %d zmean %.3f"%(r,len(R),R[:,2].mean()))
EOF

# openrua op 150
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a); V=Validity(a)
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
print("tcp",np.round(tcp(),3),"q",np.round(a.joints(),2))
s=np.array(a.joints())
# push waypoints: tip target y_tip from -0.386 (just behind bar face -0.3785) to -0.3065 ; tip z 0.98
wps=[]
for i,ytip in enumerate(np.arange(-0.40,-0.305,0.015)):
    phi=45 if ytip<-0.36 else (52 if ytip<-0.335 else 60)
    Z=Rp(phi)[:,2]
    T=np.array([-0.123,ytip,0.98])-0.009*Z
    wps.append((T,phi))
for T,phi in wps:
    q=R_to_quat(Rp(phi))
    try: sol=np.array(a.ik_hand(T,q,seed=list(s),at_tcp=True,tries=1))
    except Exception as e: print("IK FAIL",np.round(T,3),phi,e); break
    v,pairs=V.check(list(sol),verbose=False)
    print("tcp",np.round(T,3),"phi",phi,"jump %.2f"%np.abs(sol-s).max(),"valid",v,pairs[:3],"q",np.round(sol,2))
    s=sol
EOF

# openrua op 151
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,phi,dur=1.5,maxj=0.7):
    Q=R_to_quat(Rp(phi))
    s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    err=np.abs(np.array(a.joints())-s).max()
    t=tcp(); print("tcp",np.round(t,3),"phi",phi,"err %.4f"%err); return err
a.gripper(0.0); a.spin(0.3); print("fingers",a.fingers())
first=True
for ytip in np.arange(-0.40,-0.305,0.015):
    phi=45 if ytip<-0.36 else (52 if ytip<-0.335 else 60)
    Z=Rp(phi)[:,2]; T=np.array([-0.123,ytip,0.98])-0.009*Z
    e=go(T,phi,dur=2.5 if first else 1.5); first=False
    if e is None or e>0.05: print("STOP"); break
a.snap("robot0_eye_in_hand","push7.png")
EOF

# openrua op 152
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,phi,dur=1.5,maxj=0.7):
    Q=R_to_quat(Rp(phi))
    s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    err=np.abs(np.array(a.joints())-s).max()
    t=tcp(); print("tcp",np.round(t,3),"phi",phi,"err %.4f"%err); return err
Z=Rp(60)[:,2]; t=tcp()
go(t-0.06*Z,60); go(t-0.12*Z,55); go([-0.123,-0.46,1.12],45); go([-0.123,-0.50,1.16],40)
a.snap("robot0_eye_in_hand","after_push7.png")
X=a.xyz_map("robot0_eye_in_hand"); np.save("eih_after_push7.npy",X)
P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.22)&(P[:,0]<-0.03)&(P[:,1]>-0.40)&(P[:,1]<-0.13)&(P[:,2]>0.945)&(P[:,2]<1.085)
S=P[m]
for z0 in np.arange(0.95,1.085,0.01):
    T=S[(S[:,2]>=z0)&(S[:,2]<z0+0.01)]
    if len(T): print(" z%.2f n%5d x %.3f..%.3f y %.3f..%.3f"%(z0,len(T),T[:,0].min(),T[:,0].max(),T[:,1].min(),T[:,1].max()))
R=S[(S[:,2]>1.038)&(S[:,2]<1.056)]
if len(R)>20:
    A=np.c_[2*R[:,0],2*R[:,1],np.ones(len(R))]; b=R[:,0]**2+R[:,1]**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print("rim fit center",np.round(c[:2],4),"r %.4f n %d zmean %.3f"%(r,len(R),R[:,2].mean()))
EOF

# openrua op 153
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,phi,dur=1.5,maxj=0.7):
    Q=R_to_quat(Rp(phi))
    try: s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    except Exception as e: print("IKFAIL",p,phi); return None
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    err=np.abs(np.array(a.joints())-s).max()
    t=tcp(); print("tcp",np.round(t,3),"phi",phi,"err %.4f"%err); return err
go([-0.123,-0.39,1.03],50); go([-0.123,-0.41,1.06],45); go([-0.123,-0.43,1.10],40); go([-0.123,-0.45,1.14],35)
a.snap("robot0_eye_in_hand","after_push7.png")
X=a.xyz_map("robot0_eye_in_hand"); np.save("eih_after_push7.npy",X)
P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.22)&(P[:,0]<-0.03)&(P[:,1]>-0.40)&(P[:,1]<-0.13)&(P[:,2]>0.945)&(P[:,2]<1.085)
S=P[m]
for z0 in np.arange(0.95,1.085,0.01):
    T=S[(S[:,2]>=z0)&(S[:,2]<z0+0.01)]
    if len(T): print(" z%.2f n%5d x %.3f..%.3f y %.3f..%.3f"%(z0,len(T),T[:,0].min(),T[:,0].max(),T[:,1].min(),T[:,1].max()))
R=S[(S[:,2]>1.038)&(S[:,2]<1.056)]
if len(R)>20:
    A=np.c_[2*R[:,0],2*R[:,1],np.ones(len(R))]; b=R[:,0]**2+R[:,1]**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
    print("rim fit center",np.round(c[:2],4),"r %.4f n %d zmean %.3f"%(r,len(R),R[:,2].mean()))
EOF

# openrua op 154
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
for c in ["agentview","frontview","sideview","birdview"]:
    try: a.snap(c,"chk_%s.png"%c)
    except Exception as e: print(c,e)
EOF

# openrua op 155
grep -n "def snap\|def xyz_map\|def cams\|camera" arm.py | head -30; ros2 topic list | grep image_raw

# openrua op 156
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER | tail -5
from arm import *
a=Arm()
for c in ["agentview","frontview","sideview","birdview"]:
    try: a.snap(c,"chk_%s.png"%c); print("ok",c)
    except Exception as e: print(c,repr(e))
EOF
ls -la chk_*

# openrua op 157
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
import numpy as np
X=np.load("eih_after_push7.npy"); P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
m=(P[:,0]>-0.22)&(P[:,0]<-0.03)&(P[:,1]>-0.40)&(P[:,1]<-0.13)&(P[:,2]>0.945)&(P[:,2]<1.085)
S=P[m]
# body columns away from handle x
for z0 in np.arange(0.95,1.04,0.01):
    T=S[(S[:,2]>=z0)&(S[:,2]<z0+0.01)]
    L=T[(T[:,0]<-0.14)]; R=T[(T[:,0]>-0.11)]; H=T[np.abs(T[:,0]+0.125)<0.012]
    f=lambda A: ("%.3f..%.3f n%d"%(A[:,1].min(),A[:,1].max(),len(A)) if len(A) else "-")
    print("z%.2f  left y %s | right y %s | handle-col y %s"%(z0,f(L),f(R),f(H)))
# circle fit of body front arc at z 0.96-1.02 excluding handle column
B=S[(S[:,2]>0.96)&(S[:,2]<1.02)&(np.abs(S[:,0]+0.125)>0.015)]
A=np.c_[2*B[:,0],2*B[:,1],np.ones(len(B))]; b=B[:,0]**2+B[:,1]**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print("body arc fit center",np.round(c[:2],4),"r %.4f n %d"%(r,len(B)))
EOF

# openrua op 158
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a); V=Validity(a)
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,phi,dur=1.5,maxj=0.7,check=True):
    Q=R_to_quat(Rp(phi))
    try: s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    except Exception as e: print("IKFAIL",p,phi); return None
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    if check:
        ok,pairs=V.check(list(s),verbose=False)
        bad=[p_ for p_ in pairs if p_[2]>0.006]
        if not ok and bad: print("INVALID",p,phi,pairs); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    err=np.abs(np.array(a.joints())-s).max()
    t=tcp(); print("tcp",np.round(t,3),"phi",phi,"err %.4f"%err,flush=True); return err
print("fingers",a.gripper_state() if hasattr(a,"gripper_state") else "?")
seq=[([-0.123,-0.43,1.10],40),([-0.123,-0.39,1.03],50),([-0.123,-0.353,0.985],60),
     ([-0.123,-0.335,0.985],60),([-0.123,-0.318,0.984],60),([-0.123,-0.31,0.983],68),([-0.123,-0.299,0.982],75)]
for p,phi in seq:
    e=go(p,phi)
    if e is None or e>0.02: print("STOP at",p,phi,e); break
a.snap("robot0_eye_in_hand","push8.png")
EOF

# openrua op 159
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
def go(p,phi,dur=1.5,maxj=0.7):
    Q=R_to_quat(Rp(phi))
    try: s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    except Exception as e: print("IKFAIL",p,phi); return None
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    err=np.abs(np.array(a.joints())-s).max()
    t=tcp(); print("tcp",np.round(t,3),"phi",phi,"err %.4f"%err,flush=True); return err
for p,phi in [([-0.123,-0.35,0.99],60),([-0.123,-0.39,1.03],50),([-0.123,-0.43,1.10],40),([-0.122,-0.45,1.14],35)]:
    if go(p,phi) is None: break
X=a.xyz_map("robot0_eye_in_hand"); np.save("eih_after_push8.npy",X)
P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
S=P[(P[:,0]>-0.22)&(P[:,0]<-0.03)&(P[:,1]>-0.40)&(P[:,1]<-0.13)&(P[:,2]>0.945)&(P[:,2]<1.085)]
for z0 in np.arange(0.95,1.04,0.01):
    T=S[(S[:,2]>=z0)&(S[:,2]<z0+0.01)]
    L=T[(T[:,0]<-0.14)]; R=T[(T[:,0]>-0.11)]; H=T[np.abs(T[:,0]+0.125)<0.012]
    f=lambda A: ("%.3f..%.3f n%d"%(A[:,1].min(),A[:,1].max(),len(A)) if len(A) else "-")
    print("z%.2f  left y %s | right y %s | handle-col y %s"%(z0,f(L),f(R),f(H)))
B=S[(S[:,2]>0.96)&(S[:,2]<1.02)&(np.abs(S[:,0]+0.125)>0.015)]
A=np.c_[2*B[:,0],2*B[:,1],np.ones(len(B))]; b=B[:,0]**2+B[:,1]**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c[2]+c[0]**2+c[1]**2)
print("body arc fit center",np.round(c[:2],4),"r %.4f n %d"%(r,len(B)))
EOF

# openrua op 160
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
print([n for n in dir(a) if not n.startswith("_")])
import arm as A; print([n for n in dir(A) if n.isupper() or n.startswith("Q")])
EOF

# openrua op 161
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
print("fingers",a.fingers())
# move to top view over the door via IK chain, stepping pitch back toward vertical
def goQ(p,Q,dur=2.0,maxj=1.0):
    try: s=np.array(a.ik_hand(np.array(p),Q,seed=a.joints(),at_tcp=True,tries=1))
    except Exception as e: print("IKFAIL",p); return None
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.8])
    t=tcp(); print("tcp",np.round(t,3),flush=True); return t
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
for p,phi in [([-0.15,-0.47,1.20],25),([-0.20,-0.48,1.26],12),([-0.30,-0.46,1.32],0)]:
    if goQ(p,R_to_quat(Rp(phi))) is None: break
a.snap("robot0_eye_in_hand","door_top.png")
X=a.xyz_map("robot0_eye_in_hand"); np.save("eih_door_top.npy",X)
P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
D=P[(P[:,2]>1.095)&(P[:,2]<1.13)&(P[:,1]<-0.33)&(P[:,0]<-0.05)]
print("door top pts",len(D))
if len(D)>10:
    c=D[:,:2].mean(0); u,s,vt=np.linalg.svd(D[:,:2]-c); d=vt[0]; nrm=vt[1]
    t=(D[:,:2]-c)@d; w=(D[:,:2]-c)@nrm
    print("center",np.round(c,3),"dir",np.round(d,3),"len %.3f..%.3f  width %.3f..%.3f z %.3f..%.3f"%(t.min(),t.max(),w.min(),w.max(),D[:,2].min(),D[:,2].max()))
EOF

# openrua op 162
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
HG=np.array([-0.276,-0.325])
def Rz(th): c,s=np.cos(th),np.sin(th); return np.array([[c,-s],[s,c]])
def orient(u,phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    X=np.array([c*u[0],c*u[1],s]); Z=np.array([s*u[0],s*u[1],-c]); Y=np.cross(Z,X)
    return np.column_stack([X,Y,Z])
def pose(thdeg,r=0.10,off=-0.015,phi=50,ztip=1.03):
    th=np.radians(thdeg)
    tip2=HG+Rz(th)@np.array([r,off]); u=np.array([-np.sin(th),np.cos(th)])
    Rm=orient(u,phi); Z=Rm[:,2]
    tcp=np.array([tip2[0],tip2[1],ztip])-0.009*Z
    return tcp,R_to_quat(Rm)
q=list(a.joints()); sols=[]
for th in list(np.arange(-120,1,5))+[2]:
    for off in [-0.05 if th==-120 else -0.015]:
        p,Q=pose(th,off=off)
        try: s=a.ik_hand(p,Q,seed=q,at_tcp=True,tries=1)
        except Exception as e: print("IKFAIL th",th,np.round(p,3)); s=None
        if s is None: break
        j=np.abs(np.array(s)-np.array(q)).max(); print("th %4d tcp %s jump %.2f q %s"%(th,np.round(p,3),j,np.round(s,2)))
        q=list(s); sols.append((th,s))
np.save("door_sols.npy",np.array([[t]+list(s) for t,s in sols]))
EOF

# openrua op 163
grep -n "def \|class " arm.py | sed -n '1,80p'

# openrua op 164
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a); V=Validity(a)
c=np.array([-0.34,-0.435]); d=np.array([-0.501,-0.865]); n_out=np.array([-0.866,0.5])
boxes=[]
for i,t in enumerate(np.arange(-0.11,0.111,0.04)):
    p=c+t*d; boxes.append(("dr%d"%i,(p[0],p[1],1.0),(0.05,0.05,0.22)))
hb=c+0.10*d+0.03*n_out; boxes.append(("drhandle",(hb[0],hb[1],1.0),(0.06,0.06,0.16)))
pl.set_scene(boxes,remove=["door","door0","door1","door2"])
HG=np.array([-0.276,-0.325])
def Rz(th): c_,s=np.cos(th),np.sin(th); return np.array([[c_,-s],[s,c_]])
def orient(u,phi):
    ph=np.radians(phi); c_,s=np.cos(ph),np.sin(ph)
    X=np.array([c_*u[0],c_*u[1],s]); Z=np.array([s*u[0],s*u[1],-c_]); Y=np.cross(Z,X)
    return np.column_stack([X,Y,Z])
def pose(thdeg,r=0.10,off=-0.015,phi=50,ztip=1.03):
    th=np.radians(thdeg)
    tip2=HG+Rz(th)@np.array([r,off]); u=np.array([-np.sin(th),np.cos(th)])
    Rm=orient(u,phi); Z=Rm[:,2]
    return np.array([tip2[0],tip2[1],ztip])-0.009*Z, R_to_quat(Rm)
p,Q=pose(-120,off=-0.08)
s=a.ik_hand(p,Q,seed=list(a.joints()),at_tcp=True,tries=1); print("target q",np.round(s,3))
V.check(list(s))
EOF

# openrua op 165
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); V=Validity(a)
HG=np.array([-0.276,-0.325])
def Rz(th): c_,s=np.cos(th),np.sin(th); return np.array([[c_,-s],[s,c_]])
def orient(u,phi):
    ph=np.radians(phi); c_,s=np.cos(ph),np.sin(ph)
    X=np.array([c_*u[0],c_*u[1],s]); Z=np.array([s*u[0],s*u[1],-c_]); Y=np.cross(Z,X)
    return np.column_stack([X,Y,Z])
def pose(thdeg,r=0.10,off=-0.015,phi=50,ztip=1.03):
    th=np.radians(thdeg)
    tip2=HG+Rz(th)@np.array([r,off]); u=np.array([-np.sin(th),np.cos(th)])
    Rm=orient(u,phi); Z=Rm[:,2]
    return np.array([tip2[0],tip2[1],ztip])-0.009*Z, R_to_quat(Rm)
p,Q=pose(-120,off=-0.08); print("tcp",np.round(p,3))
rng=np.random.default_rng(0)
lims=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
found=[]
for k in range(40):
    seed=list(rng.uniform(lims[:,0],lims[:,1]))
    try: s=np.array(a.ik_hand(p,Q,seed=seed,at_tcp=True,tries=1))
    except Exception: continue
    margin=np.minimum(s-lims[:,0],lims[:,1]-s).min()
    found.append((margin,s))
found.sort(key=lambda x:-x[0])
for m,s in found[:8]: print("margin %.2f q %s"%(m,np.round(s,2)))
EOF

# openrua op 166
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); V=Validity(a)
HG=np.array([-0.276,-0.325])
lims=np.array([[-2.9,2.9],[-1.76,1.76],[-2.9,2.9],[-3.07,-0.07],[-2.9,2.9],[-0.02,3.75],[-2.9,2.9]])
def Rz(th): c_,s=np.cos(th),np.sin(th); return np.array([[c_,-s],[s,c_]])
def orient(u,phi):
    ph=np.radians(phi); c_,s=np.cos(ph),np.sin(ph)
    X=np.array([c_*u[0],c_*u[1],s]); Z=np.array([s*u[0],s*u[1],-c_]); Y=np.cross(Z,X)
    return np.column_stack([X,Y,Z])
def pose(thdeg,r=0.10,off=-0.015,phi=50,ztip=1.03):
    th=np.radians(thdeg)
    tip2=HG+Rz(th)@np.array([r,off]); u=np.array([-np.sin(th),np.cos(th)])
    Rm=orient(u,phi); Z=Rm[:,2]
    return np.array([tip2[0],tip2[1],ztip])-0.009*Z, R_to_quat(Rm)
q=[-0.04,0.14,-0.94,-2.66,2.35,2.53,-1.88]
plan=[(-120,-0.08),(-120,-0.05),(-120,-0.03),(-120,-0.015)]+[(t,-0.015) for t in list(np.arange(-115,1,5))+[2]]
sols=[]
for th,off in plan:
    p,Q=pose(th,off=off)
    try: s=np.array(a.ik_hand(p,Q,seed=list(q),at_tcp=True,tries=1))
    except Exception: print("IKFAIL",th,off); break
    j=np.abs(s-np.array(q)).max(); m=np.minimum(s-lims[:,0],lims[:,1]-s).min()
    ok,pairs=V.check(list(s),verbose=False) if th==-120 else (None,[])
    print("th %4d off %.3f jump %.2f margin %.2f q %s valid %s %s"%(th,off,j,m,np.round(s,2),ok,pairs))
    q=s; sols.append([th,off]+list(s))
np.save("door_sols2.npy",np.array(sols))
EOF

# openrua op 167
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm(); pl=Planner(a)
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
q0=[-0.04,0.14,-0.94,-2.66,2.35,2.53,-1.88]
pl.plan_exec(q0,speed=0.5)
print("q",np.round(a.joints(),3)); print("tcp",np.round(tcp(),3))
a.snap("robot0_eye_in_hand","door_start.png")
EOF

# openrua op 168
timeout 1200 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
S=np.load("door_sols2.npy")
for row in S[1:]:
    th,off=row[0],row[1]; s=list(row[2:])
    a.move([s],[0.9]); a.move([s],[0.5])
    err=np.abs(np.array(a.joints())-np.array(s)).max()
    print("th %4d off %.3f err %.4f tcp %s"%(th,off,err,np.round(tcp(),3)),flush=True)
    if err>0.03: print("STALL"); break
a.snap("robot0_eye_in_hand","door_end.png")
EOF

# openrua op 169
timeout 900 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER
from arm import *
a=Arm()
def tcp(): p,qq=a.fk_hand(); return p+0.1034*quat_to_R(qq)[:,2]
HG=np.array([-0.276,-0.325])
def Rz(th): c_,s=np.cos(th),np.sin(th); return np.array([[c_,-s],[s,c_]])
def orient(u,phi):
    ph=np.radians(phi); c_,s=np.cos(ph),np.sin(ph)
    X=np.array([c_*u[0],c_*u[1],s]); Z=np.array([s*u[0],s*u[1],-c_]); Y=np.cross(Z,X)
    return np.column_stack([X,Y,Z])
def pose(thdeg,r=0.10,off=-0.015,phi=50,ztip=1.03):
    th=np.radians(thdeg)
    tip2=HG+Rz(th)@np.array([r,off]); u=np.array([-np.sin(th),np.cos(th)])
    Rm=orient(u,phi); Z=Rm[:,2]
    return np.array([tip2[0],tip2[1],ztip])-0.009*Z, R_to_quat(Rm)
def goQ(p,Q,dur=1.5,maxj=1.0):
    s=np.array(a.ik_hand(np.array(p),Q,seed=list(a.joints()),at_tcp=True,tries=1))
    j=np.abs(s-np.array(a.joints())).max()
    if j>maxj: print("BIG JUMP",j); return None
    a.move([list(s)],[dur]); a.move([list(s)],[0.6])
    err=np.abs(np.array(a.joints())-s).max(); t=tcp(); print("tcp",np.round(t,3),"err %.4f"%err,flush=True); return t
for off,z in [(-0.04,1.03),(-0.08,1.06),(-0.12,1.12)]:
    p,Q=pose(0,off=off,ztip=z); goQ(p,Q)
# top view over the door area
def Rp(phi):
    ph=np.radians(phi); c,s=np.cos(ph),np.sin(ph)
    return np.column_stack([[0,c,s],[1,0,0],[0,s,-c]])
for p,phi in [([-0.17,-0.50,1.22],20),([-0.17,-0.50,1.30],5),([-0.17,-0.45,1.34],0)]:
    goQ(p,R_to_quat(Rp(phi)))
a.snap("robot0_eye_in_hand","door_closed_top.png")
X=a.xyz_map("robot0_eye_in_hand"); np.save("eih_door_closed.npy",X)
P=X.reshape(-1,3); P=P[np.isfinite(P).all(1)]
D=P[(P[:,2]>1.095)&(P[:,2]<1.13)&(P[:,1]<-0.33)&(P[:,1]>-0.45)&(P[:,0]<-0.02)&(P[:,0]>-0.32)]
print("door top pts",len(D))
if len(D)>10:
    c=D[:,:2].mean(0); u,s,vt=np.linalg.svd(D[:,:2]-c); d=vt[0]; nrm=vt[1]
    t=(D[:,:2]-c)@d; w=(D[:,:2]-c)@nrm
    print("center",np.round(c,3),"dir",np.round(d,3),"len %.3f..%.3f width %.3f..%.3f z %.3f..%.3f"%(t.min(),t.max(),w.min(),w.max(),D[:,2].min(),D[:,2].max()))
    print("y range of door top:",round(D[:,1].min(),3),round(D[:,1].max(),3)," x range",round(D[:,0].min(),3),round(D[:,0].max(),3))
EOF

# openrua op 170
timeout 300 python3 -u - <<'EOF' 2>&1 | grep -v XMLPARSER | tail -3
from arm import *
a=Arm()
for c in ["sideview","frontview"]: a.snap(c,"final_%s.png"%c)
print("joints",np.round(a.joints(),3),"fingers",a.fingers())
EOF
