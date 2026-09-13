#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
ros2 topic list 2>&1; echo ---; ros2 action list 2>&1; echo ---; ros2 service list 2>&1 | head -40; echo ---; ros2 node list 2>&1

# openrua op 2
ros2 topic echo /joint_states --once 2>&1 | head -40; echo ---; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview galleryview paperview; do python3 tools/perception/cam_snap.py $c 2>&1; done

# openrua op 3
for p in "205 230" "328 228" "420 250" "315 400" "315 380" "315 420" "270 400" "360 400"; do echo -n "agentview $p -> "; python3 tools/perception/px2world.py agentview $p 2>&1; done; ros2 topic echo /agentview/color/camera_info --once 2>&1 | grep -A12 "^k:"

# openrua op 4
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; echo ---; timeout 15 ros2 topic echo /tf_static --once 2>&1 | head -60

# openrua op 5
cat > wlib.py <<'EOF'
"""Shared helpers: TF, depth->world cloud, FK/IK, trajectory, gripper."""
import struct, sys, time
import numpy as np
import rclpy, yaml
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")["joints"]
BASE_OFF = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 translation (identity rotation)

def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2,1]-R[1,2])/s, (R[0,2]-R[2,0])/s, (R[1,0]-R[0,1])/s, 0.25*s])
    i = np.argmax(np.diag(R))
    if i == 0:
        s = np.sqrt(1 + R[0,0] - R[1,1] - R[2,2]) * 2
        return np.array([0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s])
    if i == 1:
        s = np.sqrt(1 + R[1,1] - R[0,0] - R[2,2]) * 2
        return np.array([(R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s])
    s = np.sqrt(1 + R[2,2] - R[0,0] - R[1,1]) * 2
    return np.array([(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s])

def topdown_quat(yaw):
    """Hand pointing straight down (hand Z = -world Z), rotated by yaw about world Z.
    yaw=0 -> hand X = world X, fingers close along world Y."""
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    R0 = np.diag([1.0, -1.0, -1.0])
    return R_to_quat(Rz @ R0)

class W(Node):
    def __init__(self):
        super().__init__("wlib")
        self.tfbuf = Buffer(); TransformListener(self.tfbuf, self)
        self.bridge = CvBridge()
        self.js = None
        self.create_subscription(JointState, "/joint_states", self._js, 1)
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self, GripperCommand, "/franka_gripper/gripper_action")

    def _js(self, m): self.js = m

    def spin(self, t=0.2): rclpy.spin_once(self, timeout_sec=t)

    def joints(self):
        self.js = None
        while self.js is None: self.spin()
        d = dict(zip(self.js.name, self.js.position))
        return d

    def arm_q(self):
        d = self.joints(); return [d[j] for j in ARM]

    def grab(self, topic, typ, timeout=30.0):
        got = {}
        sub = self.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
        t0 = time.time()
        while "m" not in got and time.time() - t0 < timeout: self.spin()
        self.destroy_subscription(sub)
        if "m" not in got: raise RuntimeError(f"no msg on {topic}")
        return got["m"]

    def tf(self, target, source):
        t0 = time.time()
        while time.time() - t0 < 10:
            self.spin()
            if self.tfbuf.can_transform(target, source, rclpy.time.Time()): break
        t = self.tfbuf.lookup_transform(target, source, rclpy.time.Time())
        q = t.transform.rotation; tr = t.transform.translation
        T = np.eye(4); T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w); T[:3, 3] = [tr.x, tr.y, tr.z]
        return T

    def cloud(self, cam):
        """World-frame point cloud (H,W,3) + color image from a camera."""
        depth_msg = self.grab(f"/{cam}/depth/image_raw", Image)
        color_msg = self.grab(f"/{cam}/color/image_raw", Image)
        info = self.grab(f"/{cam}/color/camera_info", CameraInfo)
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
        color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        H, Wd = depth.shape
        u, v = np.meshgrid(np.arange(Wd), np.arange(H))
        pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth, np.ones_like(depth)], -1)
        T = self.tf("world", f"{cam}_optical_frame")
        pw = pc @ T.T
        return pw[..., :3], color

    def fk_pose(self, q=None):
        """Hand pose in WORLD frame (T 4x4) for arm joints q (default current)."""
        if q is None: q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1: raise RuntimeError(f"FK failed {r}")
        p = r.pose_stamped[0].pose
        T = np.eye(4); T[:3, :3] = quat_to_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        T[:3, 3] = [p.position.x, p.position.y, p.position.z]
        T[:3, 3] += BASE_OFF  # planner frame is panda_link0
        return T

    def ik(self, pos_world, quat, seed=None, tries=1):
        """IK for hand at world pos/quat. Returns joint list or None."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.asarray(pos_world) - BASE_OFF
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(x) for x in (seed or self.arm_q())]
        req.ik_request.timeout.sec = 2
        for _ in range(tries):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def move(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, vq in enumerate(via):
                pt = JointTrajectoryPoint(positions=[float(x) for x in vq])
                t = seconds * (i + 1) / n
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9)); pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9)); pts.append(pt)
        goal.trajectory.points = pts
        self.fjt.wait_for_server(timeout_sec=10)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"move: error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width); goal.command.max_effort = 30.0
        self.grip.wait_for_server(timeout_sec=10)
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=120)
        r = res.result().result
        d = self.joints()
        print(f"gripper: reached={r.reached_goal} stalled={r.stalled} fingers={d['panda_finger_joint1']:.4f},{d['panda_finger_joint2']:.4f}", flush=True)
        return d['panda_finger_joint1']

    def move_pose(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik(pos, quat, seed=seed, tries=3)
        if q is None:
            print(f"IK FAILED for {pos}", flush=True); return None
        return self.move(q, seconds)

def start():
    rclpy.init(); return W()
EOF
cat > scan.py <<'EOF'
import sys, numpy as np, cv2
sys.path.insert(0, "/workspace")
from wlib import start
w = start()
cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
pw, color = w.cloud(cam)
np.save(f"{cam}_cloud.npy", pw)
z = pw[..., 2]
ok = np.isfinite(z)
print("z range", np.nanmin(z[ok]), np.nanmax(z[ok]))
# table height: mode of z in the central region
hist, edges = np.histogram(z[ok], bins=400, range=(0.3, 1.0))
table_z = edges[np.argmax(hist)]
print("table z ~", table_z)
mask = ok & (z > table_z + 0.008) & (z < table_z + 0.3)
mask8 = mask.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask8)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 30: continue
    m = lab == i
    P = pw[m]
    c = color[m].mean(0)
    print(f"blob {i}: area={stats[i,4]} px centroid_px=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
          f"x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] "
          f"z[{P[:,2].min():.3f},{P[:,2].max():.3f}] mean=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) color(bgr)={c.astype(int)}")
w.destroy_node()
EOF
python3 scan.py birdview 2>&1

# openrua op 6
python3 scan.py robot0_eye_in_hand 2>&1; echo ---; python3 scan.py frontview 2>&1

# openrua op 7
python3 - <<'EOF'
import numpy as np
for cam in ["birdview","robot0_eye_in_hand","agentview" ]:
    try: pw=np.load(f"{cam}_cloud.npy")
    except Exception as e: print(cam, e); continue
    z=pw[...,2]; ok=np.isfinite(z)&(np.abs(pw[...,0])<0.4)&(np.abs(pw[...,1])<0.5)
    h,e=np.histogram(z[ok],bins=200,range=(0.4,0.6)); print(cam,"table mode",e[np.argmax(h)], "count", h.max())
    # pudding region
    sel = ok & (pw[...,0]>-0.3)&(pw[...,0]<-0.15)&(np.abs(pw[...,1]-0.017)<0.06)
    zz=z[sel]; print("  pudding region z pct", np.percentile(zz,[5,50,95,99]) if zz.size else None)
EOF
python3 - <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start
w=start()
pw,color=w.cloud("agentview"); np.save("agentview_cloud.npy",pw)
z=pw[...,2]; ok=np.isfinite(z)&(np.abs(pw[...,0])<0.4)&(np.abs(pw[...,1])<0.5)
h,e=np.histogram(z[ok],bins=200,range=(0.4,0.6)); print("agentview table mode",e[np.argmax(h)])
sel = ok & (pw[...,0]>-0.3)&(pw[...,0]<-0.15)&(np.abs(pw[...,1]-0.017)<0.06)&(z>0.489)
P=pw[sel]; print("pudding pts", P.shape, "x",P[:,0].min(),P[:,0].max(),"y",P[:,1].min(),P[:,1].max(),"z",P[:,2].min(),P[:,2].max())
# white mug
sel = ok & (pw[...,0]>-0.2)&(pw[...,0]<0.0)&(pw[...,1]<-0.05)&(pw[...,1]>-0.25)&(z>0.5)
P=pw[sel]; print("white mug pts", P.shape, "x",P[:,0].min(),P[:,0].max(),"y",P[:,1].min(),P[:,1].max(),"z",P[:,2].min(),P[:,2].max())
top = P[P[:,2]>P[:,2].max()-0.01]; print(" rim pts x",top[:,0].min(),top[:,0].max(),"y",top[:,1].min(),top[:,1].max(), "mean", top.mean(0))
w.destroy_node()
EOF

# openrua op 8
python3 - <<'EOF'
import numpy as np
pw=np.load("agentview_cloud.npy")
for (u,v) in [(100,420),(500,300),(315,330),(315,460),(240,400),(390,400),(315,300),(60,250),(580,250)]:
    print((u,v), np.round(pw[v,u],4))
pe=np.load("robot0_eye_in_hand_cloud.npy")
for (u,v) in [(320,240),(320,60),(320,20),(320,100),(100,100),(500,150),(300,392),(300,385),(300,398)]:
    print("eih",(u,v), np.round(pe[v,u],4))
EOF

# openrua op 9
python3 - <<'EOF'
import sys; sys.path.insert(0,"/workspace")
from wlib import start
w=start()
w.gripper(0.04)
q=w.arm_q(); print(q)
w.move(q, 1.0)
w.destroy_node()
EOF
python3 scan.py robot0_eye_in_hand 2>&1 | tail -5

# openrua op 10
python3 - <<'EOF'
import sys; sys.path.insert(0,"/workspace")
from wlib import start
w=start()
q=w.arm_q()
for i in range(3): w.move(q, 1.0)
w.destroy_node()
EOF
python3 scan.py robot0_eye_in_hand 2>&1 | tail -5; python3 scan.py agentview 2>&1 | tail -6

# openrua op 11
python3 - <<'EOF'
import numpy as np
pw=np.load("agentview_cloud.npy")
z=pw[...,2]; ok=np.isfinite(z)
sel=ok&(pw[...,0]>-0.2)&(pw[...,0]<0.0)&(pw[...,1]<-0.05)&(pw[...,1]>-0.25)&(z>0.53)&(z<0.6)
P=pw[sel]; print("rim pts",len(P),"z",P[:,2].min(),P[:,2].max())
# fit circle in xy
A=np.c_[2*P[:,0],2*P[:,1],np.ones(len(P))]; b=P[:,0]**2+P[:,1]**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print("rim circle center",cx,cy,"r",r)
# handle: points with z in 0.47-0.52 and y > cy + r
sel2=ok&(pw[...,0]>-0.2)&(pw[...,0]<0.0)&(pw[...,1]<-0.05)&(pw[...,1]>-0.25)&(z>0.44)&(z<0.53)
Q=pw[sel2]; print("body/handle y range",Q[:,1].min(),Q[:,1].max(),"x",Q[:,0].min(),Q[:,0].max())
# red mug rim
sel=ok&(pw[...,0]>-0.15)&(pw[...,0]<0.05)&(pw[...,1]>0.03)&(pw[...,1]<0.25)&(z>0.555)&(z<0.6)
P=pw[sel]; A=np.c_[2*P[:,0],2*P[:,1],np.ones(len(P))]; b=P[:,0]**2+P[:,1]**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; print("red mug rim center",c[0],c[1],"r",np.sqrt(c[2]+c[0]**2+c[1]**2))
# pudding
sel=ok&(pw[...,0]>-0.3)&(pw[...,0]<-0.15)&(np.abs(pw[...,1]-0.017)<0.06)&(z>0.455)
P=pw[sel]; print("pudding top pts",len(P),"x",P[:,0].min(),P[:,0].max(),"y",P[:,1].min(),P[:,1].max(),"mean",P.mean(0))
EOF

# openrua op 12
cat > step1.py <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
q0=topdown_quat(0.0)
print("quat", q0)
T=w.fk_pose(); print("current hand (world):", np.round(T[:3,3],4)); print(np.round(T[:3,:3],3))
# 1. hover above white mug grasp point (rim, -y side)
grasp=np.array([-0.093,-0.204,0.525])
hover=grasp+[0,0,0.10]
r=w.move_pose(tcp_to_hand(hover,q0), q0, 3.0)
print("hover result", r)
T=w.fk_pose(); print("hand now:", np.round(T[:3,3],4), "tcp:", np.round(T[:3,3]+OFF*T[:3,2],4))
w.destroy_node()
EOF
python3 step1.py 2>&1

# openrua op 13
timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854]}}}" 2>&1 | tail -30

# openrua op 14
python3 - <<'EOF'
import numpy as np
def T(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854]
a=[0,0,0,0.0825,-0.0825,0,0.088]; d=[0.333,0,0.316,0,0.384,0,0]; al=[0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2]
M=np.eye(4)
for i in range(7): M=M@T(a[i],d[i],al[i],q[i])
M=M@T(0,0.107,0,0)  # flange
print("hand pos in base frame:",np.round(M[:3,3],4))
print("+base offset:",np.round(M[:3,3]+[-0.51,0,0.42],4))
EOF

# openrua op 15
python3 - <<'EOF'
import numpy as np
def T(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854]
a=[0,0,0,0.0825,-0.0825,0,0.088]; d=[0.333,0,0.316,0,0.384,0,0]; al=[0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2]
M=np.eye(4)
for i in range(7): M=M@T(a[i],d[i],al[i],q[i])
M=M@T(0,0.107,0,0)  # flange
print("hand pos in base frame:",np.round(M[:3,3],4))
print("+base offset:",np.round(M[:3,3]+[-0.51,0,0.42],4))
EOF

# openrua op 16
python3 - <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
import wlib
from wlib import start
w=start()
w.BASE_OFF = np.zeros(3); wlib.BASE_OFF=np.zeros(3)
q=w.arm_q(); print("current", np.round(q,3))
w.fk.wait_for_service(timeout_sec=10)
sol=w.ik([-0.053,0.0,0.7776],[0.9996,0,-0.0284,0])
print("IK world-frame interpretation:", None if sol is None else np.round(sol,3))
sol=w.ik([-0.053+0.51,0.0,0.7776-0.42],[0.9996,0,-0.0284,0])
print("IK base-frame interpretation:", None if sol is None else np.round(sol,3))
w.destroy_node()
EOF

# openrua op 17
sed -i 's/self.ik = self.create_client(GetPositionIK/self.ik_cli = self.create_client(GetPositionIK/; s/self.fk = self.create_client(GetPositionFK/self.fk_cli = self.create_client(GetPositionFK/; s/fut = self.fk.call_async(req)/self.fk_cli.wait_for_service(timeout_sec=10); fut = self.fk_cli.call_async(req)/; s/fut = self.ik.call_async(req)/self.ik_cli.wait_for_service(timeout_sec=10); fut = self.ik_cli.call_async(req)/' wlib.py && grep -n "_cli" wlib.py
python3 - <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
import wlib
from wlib import start
w=start()
wlib.BASE_OFF=np.zeros(3)
q=w.arm_q(); print("current", np.round(q,3))
sol=w.ik([-0.053,0.0,0.7776],[0.9996,0,-0.0284,0])
print("IK world-frame interpretation:", None if sol is None else np.round(sol,3))
sol=w.ik([-0.053+0.51,0.0,0.7776-0.42],[0.9996,0,-0.0284,0])
print("IK base-frame interpretation:", None if sol is None else np.round(sol,3))
w.destroy_node()
EOF

# openrua op 18
python3 - <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
import wlib
from wlib import start, R_to_quat
w=start()
wlib.BASE_OFF=np.zeros(3)
for q in [[0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009],[0,-0.161,0,-2.445,0,2.227,0.785]]:
    T=w.fk_pose(q); print(np.round(T[:3,3],4), "quat", np.round(R_to_quat(T[:3,:3]),4)); print(np.round(T[:3,:3],3))
w.destroy_node()
EOF

# openrua op 19
sed -i 's/req.ik_request.group_name = "panda_arm"/req.ik_request.group_name = "panda_arm"\n        req.ik_request.ik_link_name = "panda_hand"/' wlib.py
sed -i 's/^BASE_OFF = np.array(\[-0.51, 0.0, 0.42\]).*/BASE_OFF = np.zeros(3)  # IK\/FK on this machine already work in WORLD coords (verified vs TF and manual FK)/' wlib.py
grep -n "BASE_OFF\|ik_link_name" wlib.py
python3 - <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, R_to_quat
w=start()
sol=w.ik([-0.053,0.0,0.7776],[0.9996,0,-0.0284,0])
print("IK:", None if sol is None else np.round(sol,3))
if sol is not None:
    T=w.fk_pose(sol); print(np.round(T[:3,3],4), "quat", np.round(R_to_quat(T[:3,:3]),4))
w.destroy_node()
EOF

# openrua op 20
python3 step1.py 2>&1 && python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 tools/perception/cam_snap.py agentview

# openrua op 21
cat > step2.py <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
q0=topdown_quat(0.0)
grasp=np.array([-0.093,-0.204,0.525])
cur=w.arm_q()
sol=w.ik(tcp_to_hand(grasp,q0), q0, seed=cur, tries=3)
print("descend sol", np.round(sol,3), "diff", np.round(np.array(sol)-np.array(cur),3))
if np.abs(np.array(sol)-np.array(cur)).max() > 1.0: raise SystemExit("IK branch jump; abort")
w.move(sol, 2.5)
T=w.fk_pose(); print("tcp:", np.round(T[:3,3]+OFF*T[:3,2],4))
w.gripper(0.0)
w.destroy_node()
EOF
python3 step2.py 2>&1 && python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 tools/perception/cam_snap.py agentview

# openrua op 22
cat > step3.py <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
def go(tcp, yaw=0.0, secs=2.5, maxjump=1.0):
    q0=topdown_quat(yaw); cur=w.arm_q()
    sol=w.ik(tcp_to_hand(tcp,q0), q0, seed=cur, tries=3)
    if sol is None: raise SystemExit(f"IK failed {tcp}")
    d=np.abs(np.array(sol)-np.array(cur)).max()
    if d>maxjump: raise SystemExit(f"IK branch jump {d:.2f}; abort")
    w.move(sol, secs)
    T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
    return T
go([-0.093,-0.204,0.68], secs=2.5)
print("fingers", {k:round(v,4) for k,v in w.joints().items() if "finger" in k})
w.destroy_node()
EOF
python3 step3.py 2>&1 && python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py sideview

# openrua op 23
cat > step4.py <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
def go(tcp, yaw=0.0, secs=2.5, maxjump=1.2):
    q0=topdown_quat(yaw); cur=w.arm_q()
    sol=w.ik(tcp_to_hand(tcp,q0), q0, seed=cur, tries=3)
    if sol is None: raise SystemExit(f"IK failed {tcp}")
    d=np.abs(np.array(sol)-np.array(cur)).max()
    if d>maxjump: raise SystemExit(f"IK branch jump {d:.2f}; abort")
    w.move(sol, secs)
    T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
    return T
go([0.172,-0.048,0.68], secs=3.5)
print("fingers", {k:round(v,4) for k,v in w.joints().items() if "finger" in k})
go([0.172,-0.048,0.60], secs=2.5)
print("fingers", {k:round(v,4) for k,v in w.joints().items() if "finger" in k})
w.destroy_node()
EOF
python3 step4.py 2>&1 && python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 24
cat > step5.py <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
def go(tcp, yaw=0.0, secs=2.5, maxjump=1.2):
    q0=topdown_quat(yaw); cur=w.arm_q()
    sol=w.ik(tcp_to_hand(tcp,q0), q0, seed=cur, tries=3)
    if sol is None: raise SystemExit(f"IK failed {tcp}")
    d=np.abs(np.array(sol)-np.array(cur)).max()
    if d>maxjump: raise SystemExit(f"IK branch jump {d:.2f}; abort")
    w.move(sol, secs)
    T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
    return T
w.gripper(0.04)
go([0.172,-0.048,0.70], secs=2.5)
w.destroy_node()
EOF
python3 step5.py 2>&1 && python3 tools/perception/cam_snap.py agentview && python3 scan.py agentview 2>&1 | tail -5

# openrua op 25
python3 - <<'EOF'
import numpy as np
pw=np.load("agentview_cloud.npy"); z=pw[...,2]; ok=np.isfinite(z)
# white mug rim now: points with z in 0.53-0.56 near plate
sel=ok&(pw[...,0]>0.05)&(pw[...,0]<0.3)&(np.abs(pw[...,1])<0.15)&(z>0.53)&(z<0.56)
P=pw[sel]; A=np.c_[2*P[:,0],2*P[:,1],np.ones(len(P))]; b=P[:,0]**2+P[:,1]**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; print("white mug rim center",np.round(c[:2],4),"r",round(float(np.sqrt(c[2]+c[0]**2+c[1]**2)),4),"top z",P[:,2].max())
# plate rim: points z 0.45-0.46 in the region, excluding mug
sel=ok&(pw[...,0]>0.05)&(pw[...,0]<0.3)&(np.abs(pw[...,1])<0.15)&(z>0.446)&(z<0.462)
P=pw[sel]; print("plate-ish pts x",P[:,0].min(),P[:,0].max(),"y",P[:,1].min(),P[:,1].max(), "mean", P.mean(0))
EOF

# openrua op 26
cat > step6.py <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
def go(tcp, yaw=0.0, secs=2.5, maxjump=1.2):
    q0=topdown_quat(yaw); cur=w.arm_q()
    sol=w.ik(tcp_to_hand(tcp,q0), q0, seed=cur, tries=3)
    if sol is None: raise SystemExit(f"IK failed {tcp}")
    d=np.abs(np.array(sol)-np.array(cur)).max()
    if d>maxjump: raise SystemExit(f"IK branch jump {d:.2f}; abort")
    w.move(sol, secs)
    T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
    return T
q0=topdown_quat(0.0)
# feasibility check first
for tcp in ([-0.212,0.017,0.60],[-0.212,0.017,0.445]):
    s=w.ik(tcp_to_hand(tcp,q0),q0,tries=3); print("IK", tcp, None if s is None else np.round(s,3))
go([-0.212,0.017,0.62], secs=3.5)
w.destroy_node()
EOF
python3 step6.py 2>&1 && python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 tools/perception/cam_snap.py agentview

# openrua op 27
cat > step6.py <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
def report():
    T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
q0=topdown_quat(0.0)
cur=w.arm_q()
grasp=[-0.212,0.017,0.445]; hover=[-0.212,0.017,0.62]; way=[-0.03,-0.02,0.78]
qg=w.ik(tcp_to_hand(grasp,q0),q0,seed=cur,tries=3); print("grasp q",np.round(qg,3))
qh=w.ik(tcp_to_hand(hover,q0),q0,seed=qg,tries=3); print("hover q",np.round(qh,3), "diff to grasp", np.round(np.abs(np.array(qh)-np.array(qg)).max(),3))
qw=w.ik(tcp_to_hand(way,q0),q0,seed=cur,tries=3); print("way q",np.round(qw,3))
w.move(qw, 3.0); report()
w.move(qh, 3.5); report()
w.destroy_node()
EOF
python3 step6.py 2>&1 && python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 tools/perception/cam_snap.py agentview

# openrua op 28
cat > retry.py <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start
w=start()
OFF=0.1034
q=[float(x) for x in sys.argv[1].split(",")]; secs=float(sys.argv[2])
for i in range(3):
    code,err=w.move(q,secs)
    if err<0.01: break
T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
w.destroy_node()
EOF
python3 retry.py "0.213,-0.597,-0.197,-2.999,-0.164,2.404,0.957" 3.0 2>&1 && python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 tools/perception/cam_snap.py agentview

# openrua op 29
cat > step7.py <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
def report():
    T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
q0=topdown_quat(0.0); cur=w.arm_q()
qg=w.ik(tcp_to_hand([-0.212,0.017,0.445],q0),q0,seed=cur,tries=3); print("grasp q",np.round(qg,3),"jump",np.round(np.abs(np.array(qg)-np.array(cur)).max(),3))
for i in range(3):
    code,err=w.move(qg,3.0)
    if err<0.01: break
report()
w.gripper(0.0)
w.destroy_node()
EOF
python3 step7.py 2>&1 && python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 tools/perception/cam_snap.py agentview

# openrua op 30
cat > step8.py <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
def report():
    T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
    return T
def fingers(): return {k:round(v,4) for k,v in w.joints().items() if "finger" in k}
def go(tcp, secs=3.0, seed=None):
    q0=topdown_quat(0.0); cur=w.arm_q()
    sol=w.ik(tcp_to_hand(tcp,q0),q0,seed=seed or cur,tries=3)
    if sol is None: raise SystemExit(f"IK failed {tcp}")
    print("target",tcp,"jump",np.round(np.abs(np.array(sol)-np.array(cur)).max(),3), flush=True)
    for i in range(3):
        code,err=w.move(sol,secs)
        if err<0.01: break
    report(); print("fingers",fingers(), flush=True)
    return sol
go([-0.212,0.017,0.65], 2.5)          # lift straight up
go([0.0,0.09,0.70], 3.0)              # waypoint, high over the red mug region
go([0.19,0.15,0.62], 3.0)             # above the destination
w.destroy_node()
EOF
python3 step8.py 2>&1 && python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 31
cat > step9.py <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start, topdown_quat, quat_to_R
w=start()
OFF=0.1034
def tcp_to_hand(tcp, q):
    R=quat_to_R(*q); return np.asarray(tcp)-OFF*R[:,2]
def report():
    T=w.fk_pose(); print("tcp now:", np.round(T[:3,3]+OFF*T[:3,2],4), flush=True)
def fingers(): return {k:round(v,4) for k,v in w.joints().items() if "finger" in k}
def go(tcp, secs=3.0):
    q0=topdown_quat(0.0); cur=w.arm_q()
    sol=w.ik(tcp_to_hand(tcp,q0),q0,seed=cur,tries=3)
    if sol is None: raise SystemExit(f"IK failed {tcp}")
    print("target",tcp,"jump",np.round(np.abs(np.array(sol)-np.array(cur)).max(),3), flush=True)
    for i in range(3):
        code,err=w.move(sol,secs)
        if err<0.01: break
    report(); print("fingers",fingers(), flush=True)
go([0.19,0.15,0.455], 2.5)
w.gripper(0.04)
go([0.19,0.15,0.65], 2.5)
w.destroy_node()
EOF
python3 step9.py 2>&1 && python3 tools/perception/cam_snap.py agentview && python3 scan.py agentview 2>&1 | grep -v "^blob [123]:"

# openrua op 32
cat > step10.py <<'EOF'
import sys; sys.path.insert(0,"/workspace")
import numpy as np
from wlib import start
w=start()
home=[0.0,-0.161,0.0,-2.4446,0.0,2.2268,0.7854]
for i in range(3):
    code,err=w.move(home,3.5)
    if err<0.01: break
T=w.fk_pose(); print("hand:", np.round(T[:3,3],4))
w.destroy_node()
EOF
python3 step10.py 2>&1 && for c in agentview birdview frontview; do python3 tools/perception/cam_snap.py $c; done && python3 scan.py agentview 2>&1 | grep -v "^blob [123]:"

# openrua op 33
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Verified quirks of the Franka Panda sim workstation in /workspace (frames, IK tip link, settling, table height) that contradict or extend the shipped docs
metadata:
  type: project
---

Verified 2026-09-12 on the /workspace Panda (LIBERO-style living-room table scene):

- `/compute_ik` and `/compute_fk` operate in the WORLD frame (world->panda_link0 is (-0.51, 0, 0.42) and FK output already includes it), despite docs/30-action.md saying the planner frame is the arm base. Leave frame_id empty, pass world coords directly.
- The `panda_arm` group tip is panda_link8, which is yawed +45° from panda_hand. Set `ik_request.ik_link_name = "panda_hand"` or IK "succeeds" with the hand rotated 45°.
- At session start objects float ~6 cm above the table until the sim advances; run a few no-op trajectories (move to current joints) and rescan before trusting object heights. Table top is z≈0.425 world.
- Trajectory goals often return error_code -5 and stop short on large moves; resending the same goal converges (loop until max joint err < 0.01).
- Gripper is binary open/closed; finger stop position after closing is reliable grasp evidence (e.g. 0.004 on a mug rim wall, 0.023 on a 4.6 cm box).
- Mug body diameters (~9 cm) exceed the 7.7 cm gripper opening; grasp mugs by the rim wall (one finger inside), TCP ~2.5 cm below the rim.

**Why:** each of these cost a debugging cycle; the docs are generic and partly wrong for this machine.
**How to apply:** reuse `/workspace/wlib.py` (W class: cloud(), fk_pose(), ik(), move(), gripper()) if it still exists; otherwise rebuild with the facts above.
OPENRUA_EOF

# openrua op 34
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index yet)"); echo "- [Panda workstation quirks](panda-workstation-quirks.md) — IK/FK are world-frame, IK tip is link8 (set ik_link_name=panda_hand), objects float until sim advances, -5 goals converge on resend" >> MEMORY.md
