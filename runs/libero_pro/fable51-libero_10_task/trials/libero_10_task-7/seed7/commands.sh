#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 node list

# openrua op 4
timeout 60 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 60 ros2 topic echo /tf_static --once 2>&1 | head -150; echo ----; timeout 30 ros2 topic echo /birdview/color/camera_info --once

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
while time.time()-t0 < 5: rclpy.spin_once(node, timeout_sec=0.2)
for (p,c),t in sorted(seen.items()):
    print(f"{p} -> {c}: t=({t.translation.x:.4f},{t.translation.y:.4f},{t.translation.z:.4f}) q=({t.rotation.x:.4f},{t.rotation.y:.4f},{t.rotation.z:.4f},{t.rotation.w:.4f})")
EOF
timeout 60 python3 tfdump.py

# openrua op 7
cat > cloud.py <<'EOF'
"""Grab color+depth+info from a camera, produce world-frame point cloud; save npz."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(node, topic, T, timeout=30):
    got={}
    sub=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    import time; t0=time.time()
    while "m" not in got and time.time()-t0<timeout: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

def main():
    cam=sys.argv[1]
    rclpy.init(); node=rclpy.create_node("cloud")
    tfs={}
    qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    def cb(m):
        for t in m.transforms: tfs[(t.header.frame_id,t.child_frame_id)]=t.transform
    node.create_subscription(TFMessage,"/tf_static",cb,qos)
    node.create_subscription(TFMessage,"/tf",cb,100)
    depth=grab(node,f"/{cam}/depth/image_raw",Image)
    color=grab(node,f"/{cam}/color/image_raw",Image)
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    import time; t0=time.time()
    while ("world",f"{cam}_optical_frame") not in tfs and time.time()-t0<10: rclpy.spin_once(node,timeout_sec=0.2)
    t=tfs[("world",f"{cam}_optical_frame")]
    R=quat_R(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w); p=np.array([t.translation.x,t.translation.y,t.translation.z])
    br=CvBridge()
    D=br.imgmsg_to_cv2(depth,"passthrough").astype(np.float64)
    C=br.imgmsg_to_cv2(color,"bgr8")
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    H,W=D.shape
    u,v=np.meshgrid(np.arange(W),np.arange(H))
    X=(u-cx)*D/fx; Y=(v-cy)*D/fy
    P=np.stack([X,Y,D],-1)@R.T+p
    np.savez(f"{cam}_cloud.npz",P=P,C=C,D=D)
    print("saved", f"{cam}_cloud.npz", "depth range", np.nanmin(D), np.nanmax(D))
    rclpy.shutdown()
main()
EOF
timeout 120 python3 cloud.py birdview

# openrua op 8
cat > objs.py <<'EOF'
import numpy as np, cv2
d=np.load("birdview_cloud.npz"); P=d["P"]; C=d["C"]; D=d["D"]
Z=P[...,2]
# table height: mode of z in the table region
hist,edges=np.histogram(Z[np.isfinite(Z)],bins=300)
zt=edges[np.argmax(hist)]
print("dominant z (table?)",zt, "floor", np.nanmin(Z))
# find table height more precisely: values within the table region around center pixel
print("z at center pixel", Z[240,320], "z at (200,450)", Z[200,450])
table=Z[(Z>0.7)&(Z<1.0)]
print("table z stats", table.min(), table.max(), np.median(table))
zt=np.median(table)
mask=(Z>zt+0.01)&(Z<zt+0.5)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<15: continue
    m=lab==i
    pts=P[m]
    col=C[m].mean(0)
    print(f"comp {i}: px area {stats[i,4]}, centroid px {cent[i]}, world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} zmin {pts[:,2].min():.3f} meanBGR {col.round()}")
EOF
python3 objs.py

# openrua op 9
cat > objs.py <<'EOF'
import numpy as np, cv2
d=np.load("birdview_cloud.npz"); P=d["P"]; C=d["C"]; D=d["D"]
Z=P[...,2]
zt=0.425
mask=((Z>zt+0.008)&(Z<zt+0.6)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<10: continue
    m=lab==i
    pts=P[m]
    col=C[m].mean(0)
    print(f"comp {i}: px area {stats[i,4]}, centroid px {cent[i].round(1)}, world cx {pts[:,0].mean():.3f} cy {pts[:,1].mean():.3f} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} zmin {pts[:,2].min():.3f} meanBGR {col.round()}")
EOF
python3 objs.py

# openrua op 10
timeout 120 python3 cloud.py agentview && cat > objs2.py <<'EOF'
import numpy as np, cv2
d=np.load("agentview_cloud.npz"); P=d["P"]; C=d["C"]; D=d["D"]
Z=P[...,2]
zt=0.425
mask=((Z>zt+0.01)&(Z<zt+0.25)&(P[...,0]>-0.45)&(P[...,0]<0.4)&(np.abs(P[...,1])<0.6)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<10: continue
    m=lab==i
    pts=P[m]
    col=C[m].mean(0)
    print(f"comp {i}: px area {stats[i,4]}, centroid px {cent[i].round(1)}, world cx {pts[:,0].mean():.3f} cy {pts[:,1].mean():.3f} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} zmin {pts[:,2].min():.3f} meanBGR {col.round()}")
EOF
python3 objs2.py

# openrua op 11
python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[220:300,320:400]; cv2.imwrite('crop_box.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
crop=im[170:270,200:290]; cv2.imwrite('crop_ketchup.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 12
cat > robot.py <<'EOF'
"""Small controller: IK (MoveIt) -> FollowJointTrajectory, gripper, joint state, FK.
All public poses are in WORLD frame; converted to panda_link0 for MoveIt."""
import time, sys
import numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])   # from /tf world->panda_link0
TCP_OFF = float(M["hand"]["tcp_offset_m"])

def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)

class Robot:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(20), "no fjt"
        assert self.grip.wait_for_server(20), "no gripper"
        assert self.ik.wait_for_service(20), "no ik"
        self.fk.wait_for_service(5)

    def _on_js(self, m): self._js["m"] = m

    def spin(self, t=0.05): rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 20: self.spin(0.1)
        m = self._js["m"]
        d = dict(zip(m.name, m.position))
        return d

    def arm_q(self):
        d = self.joints(); return np.array([d[j] for j in JOINTS])

    def fingers(self):
        d = self.joints(); return d["panda_finger_joint1"], d["panda_finger_joint2"]

    # ---- kinematics ----
    @staticmethod
    def quat_topdown(yaw_deg=0.0):
        """hand z pointing down (world -z); fingers open along world y rotated by yaw about z."""
        r = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])
        return r.as_quat()  # x,y,z,w

    def ik_world(self, xyz_world, quat, at_tcp=True, seed=None, attempts=3):
        xyz = np.array(xyz_world, float) - BASE_IN_WORLD
        R = Rot.from_quat(quat).as_matrix()
        if at_tcp:
            xyz = xyz - TCP_OFF * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.timeout.sec = 2
        req.ik_request.avoid_collisions = False
        q0 = self.arm_q() if seed is None else np.array(seed)
        for k in range(attempts):
            seedjs = JointState(); seedjs.name = list(JOINTS); seedjs.position = [float(v) for v in q0]
            req.ik_request.robot_state.joint_state = seedjs
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return np.array([sol[j] for j in JOINTS])
            log(f"IK attempt {k} failed code={None if res is None else res.error_code.val}")
            q0 = q0 + np.random.uniform(-0.3, 0.3, 7)
        return None

    def fk_world(self, q=None):
        """TCP pose in world via /compute_fk of panda_hand; returns (xyz_tcp, quat)."""
        if q is None: q = self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        js = JointState(); js.name = list(JOINTS); js.position = [float(v) for v in q]
        req.robot_state.joint_state = js
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        p = res.pose_stamped[0].pose
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = Rot.from_quat(quat).as_matrix()
        return xyz + TCP_OFF * R[:, 2], quat

    # ---- motion ----
    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        wps = (waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        h = send.result()
        if h is None or not h.accepted:
            log("FJT goal not accepted"); return False
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code if rf.result() else None
        err = np.abs(self.arm_q() - np.array(q)).max()
        log(f"move done code={code} max joint err={err:.4f}")
        return code == 0 and err < 0.05

    def move_world(self, xyz, quat, seconds=3.0, at_tcp=True, seed=None):
        q = self.ik_world(xyz, quat, at_tcp=at_tcp, seed=seed)
        if q is None:
            log("IK failed for", xyz); return None
        ok = self.move_q(q, seconds)
        return q if ok else None

    def gripper(self, width):
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        log(f"gripper({width}) reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f
EOF
cat > test_fk.py <<'EOF'
from robot import *
r = Robot()
q = r.arm_q(); log("q", q.round(3))
log("fingers", r.fingers())
fk = r.fk_world(q); log("tcp world", fk)
# check IK round trip for a pre-grasp pose above ketchup
quat = Robot.quat_topdown(90)
log("quat", quat)
sol = r.ik_world([-0.22, -0.125, 0.62], quat)
log("ik sol", None if sol is None else sol.round(3))
if sol is not None: log("fk of sol", r.fk_world(sol))
EOF
timeout 300 python3 test_fk.py

# openrua op 13
sed -i 's/^BASE_IN_WORLD = np.array(\[-0.51, 0.0, 0.42\]).*/BASE_IN_WORLD = np.zeros(3)   # FK check: planner model frame == world on this machine/' robot.py && timeout 300 python3 test_fk.py

# openrua op 14
cat > test_fk2.py <<'EOF'
from robot import *
r = Robot()
quat = Robot.quat_topdown(90)
sol = r.ik_world([-0.22, -0.125, 0.62], quat)
for link in ["panda_link8", "panda_hand"]:
    req = GetPositionFK.Request(); req.header.frame_id=""; req.fk_link_names=[link]
    js = JointState(); js.name=list(JOINTS); js.position=[float(v) for v in sol]
    req.robot_state.joint_state=js
    fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    p=fut.result().pose_stamped[0].pose
    print(link, [round(v,4) for v in (p.position.x,p.position.y,p.position.z)], [round(v,4) for v in (p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)])
print("requested", quat.round(4))
EOF
timeout 120 python3 test_fk2.py

# openrua op 15
python3 - <<'EOF'
import re
s=open("robot.py").read()
s=s.replace('''        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)''',
'''        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        # IK tip link is panda_link8 = panda_hand rotated +45deg about z (checked via /compute_fk)
        q8 = (Rot.from_quat(quat) * Rot.from_euler("z", 45, degrees=True)).as_quat()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)''')
open("robot.py","w").write(s)
EOF
cat > test_fk3.py <<'EOF'
from robot import *
r = Robot()
for yaw in (0, 90):
    quat = Robot.quat_topdown(yaw)
    sol = r.ik_world([-0.22, -0.125, 0.62], quat)
    fk = r.fk_world(sol)
    Rh = Rot.from_quat(fk[1]).as_matrix()
    print(f"yaw {yaw}: tcp {fk[0].round(4)} hand y-axis (finger open dir) in world {Rh[:,1].round(3)} hand z {Rh[:,2].round(3)}")
EOF
timeout 120 python3 test_fk3.py

# openrua op 16
python3 - <<'EOF'
import numpy as np
d=np.load("agentview_cloud.npz"); P=d["P"]
Z=P[...,2]
m=(Z>0.448)&(Z<0.47)&(P[...,0]>-0.25)&(P[...,0]<-0.05)&(P[...,1]>0.0)&(P[...,1]<0.15)
pts=P[m]; print("box top face n",len(pts),"center",pts.mean(0).round(4),"x",pts[:,0].min().round(3),pts[:,0].max().round(3),"y",pts[:,1].min().round(3),pts[:,1].max().round(3))
# ketchup body from birdview
b=np.load("birdview_cloud.npz")["P"]
m=(b[...,2]>0.44)&(b[...,2]<0.6)&(b[...,0]>-0.3)&(b[...,0]<-0.15)&(b[...,1]>-0.2)&(b[...,1]<-0.05)
pts=b[m]; print("ketchup n",len(pts),"center",pts.mean(0).round(4),"x",pts[:,0].min().round(3),pts[:,0].max().round(3),"y",pts[:,1].min().round(3),pts[:,1].max().round(3))
m2=m&(b[...,2]>0.53); pts=b[m2]; print("ketchup cap center",pts.mean(0).round(4))
EOF

# openrua op 17
cat > pick_ketchup.py <<'EOF'
from robot import *
r = Robot()
KX, KY = -0.224, -0.126
YAW = 90            # fingers open along world x (bottle's thin axis)
Q = Robot.quat_topdown(YAW)
BASKET = (0.01, 0.26)

log("open gripper"); r.gripper(GRIP["open_m"])
log("pre-grasp"); q1 = r.move_world([KX, KY, 0.64], Q, 4.0); assert q1 is not None
log("tcp", r.fk_world()[0].round(3))
log("descend"); q2 = r.move_world([KX, KY, 0.475], Q, 3.0, seed=q1); assert q2 is not None
log("tcp", r.fk_world()[0].round(3))
log("close"); f = r.gripper(GRIP["closed_m"])
log("lift"); q3 = r.move_world([KX, KY, 0.75], Q, 3.0, seed=q2); assert q3 is not None
log("fingers after lift", r.fingers())
log("over basket"); q4 = r.move_world([BASKET[0], BASKET[1], 0.78], Q, 4.0, seed=q3); assert q4 is not None
log("tcp", r.fk_world()[0].round(3))
log("release"); r.gripper(GRIP["open_m"])
log("retreat up"); q5 = r.move_world([BASKET[0], BASKET[1], 0.85], Q, 2.0, seed=q4)
log("DONE")
EOF
nohup python3 -u pick_ketchup.py > pick_ketchup.log 2>&1 &
sleep 60; cat pick_ketchup.log

# openrua op 18
cat > state.py <<'EOF'
from robot import *
r = Robot()
q = r.arm_q(); log("q", q.round(3)); log("fingers", r.fingers()); log("tcp", r.fk_world(q))
EOF
timeout 60 python3 state.py; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 19
timeout 60 python3 state.py; timeout 30 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 20
python3 - <<'EOF'
s=open("robot.py").read()
s=s.replace('''    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()''','''    def move_q(self, q, seconds=3.0, waypoints=None, retries=2):
        # pace by the largest joint displacement (~0.6 rad/s max) and retry slower on tolerance violations
        dist = np.abs(self.arm_q() - np.array(q)).max()
        seconds = max(seconds, dist / 0.6)
        for attempt in range(retries + 1):
            ok = self._move_q_once(q, seconds, waypoints)
            if ok: return True
            seconds *= 1.6
            log(f"retrying slower: {seconds:.1f}s")
        return False

    def _move_q_once(self, q, seconds, waypoints=None):
        goal = FollowJointTrajectory.Goal()''')
open("robot.py","w").write(s)
EOF
sed -i 's/^log("open gripper"); r.gripper(GRIP\["open_m"\])/f = r.fingers()\nif f[0] < 0.035: r.gripper(GRIP["open_m"])/' pick_ketchup.py
nohup python3 -u pick_ketchup.py > pick_ketchup.log 2>&1 &
sleep 90; cat pick_ketchup.log

# openrua op 21
timeout 60 python3 state.py; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 30 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 22
cat >> robot.py <<'EOF'

def _cont(seed, q):
    """wrap-free joint distance check"""
    return np.abs(np.array(q) - np.array(seed)).max()

def move_line(self, xyz_target, quat, step=0.02, speed=0.06, at_tcp=True, max_jump=0.35):
    """Straight Cartesian line from the current TCP to xyz_target as one multi-point trajectory."""
    start, _ = self.fk_world()
    target = np.array(xyz_target, float)
    n = max(1, int(np.ceil(np.linalg.norm(target - start) / step)))
    seed = self.arm_q()
    wps = []
    t = 0.0
    dt = max(0.3, (np.linalg.norm(target - start) / n) / speed)
    for i in range(1, n + 1):
        p = start + (target - start) * i / n
        q = self.ik_world(p, quat, at_tcp=at_tcp, seed=seed, attempts=1)
        if q is None or _cont(seed, q) > max_jump:
            log(f"line: IK jump/fail at waypoint {i}/{n} {p.round(3)} " +
                ("" if q is None else f"jump={_cont(seed, q):.2f}"))
            return False
        t += dt
        wps.append((q, t))
        seed = q
    qf, tf = wps[-1]
    ok = self.move_q(qf, tf, waypoints=wps[:-1])
    pos, _ = self.fk_world()
    log(f"line end tcp {pos.round(4)} (target {target.round(4)}) err {np.linalg.norm(pos-target)*1000:.1f} mm")
    return ok
Robot.move_line = move_line
EOF
cat > recover.py <<'EOF'
from robot import *
r = Robot()
pos, quat = r.fk_world()
log("tcp", pos.round(3))
r.move_line([pos[0], pos[1], 0.66], quat, speed=0.05)
log("fingers", r.fingers())
EOF
timeout 600 python3 -u recover.py 2>&1 | tee recover.log

# openrua op 23
cat > iktest.py <<'EOF'
from robot import *
r = Robot()
pos, quat = r.fk_world(); q0 = r.arm_q()
print("q0", q0.round(3))
for i in range(4):
    q = r.ik_world(pos, quat, seed=q0, attempts=1)
    print("same pose:", None if q is None else (q.round(3), f"jump {np.abs(q-q0).max():.3f}"))
for i in range(3):
    q = r.ik_world(pos+[0,0,0.02], quat, seed=q0, attempts=1)
    print("+2cm:", None if q is None else (q.round(3), f"jump {np.abs(q-q0).max():.3f}"))
EOF
timeout 300 python3 iktest.py

# openrua op 24
cat > recover2.py <<'EOF'
from robot import *
r = Robot()
Q = Robot.quat_topdown(90)
q_home = np.array([0, -0.161037389, 0, -2.44459747, 0, 2.2267522, 0.7853981633974483])
q1 = r.ik_world([-0.224, -0.126, 0.64], Q, seed=q_home, attempts=1)
log("q1", q1.round(3), "current", r.arm_q().round(3))
ok = r.move_q(q1, 5.0)
log("tcp", r.fk_world()[0].round(3), "fingers", r.fingers())
EOF
timeout 600 python3 -u recover2.py 2>&1 | tee recover2.log

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py agentview && python3 -c "
import cv2; im=cv2.imread('agentview.png'); cv2.imwrite('crop.png', cv2.resize(im[120:300,150:330],None,fx=3,fy=3))"

# openrua op 26
cat >> robot.py <<'EOF'

LIMITS = np.array(FJT["limits_rad"])

def plan_descent(self, xyz_grasp, quat, z_top, step=0.02, n_seeds=25, max_jump=0.35, margin=0.12):
    """Find a grasp IK solution + a joint-continuous straight vertical path from z_top down to it.
    Returns (chain) where chain[0] is at z_top and chain[-1] at the grasp, or None."""
    xyz_grasp = np.array(xyz_grasp, float)
    rng = np.random.default_rng(0)
    cands = []
    seeds = [self.arm_q()] + [rng.uniform(LIMITS[:, 0] + 0.2, LIMITS[:, 1] - 0.2) for _ in range(n_seeds)]
    for s in seeds:
        q = self.ik_world(xyz_grasp, quat, seed=s, attempts=1)
        if q is None: continue
        if any(np.abs(q - c).max() < 0.05 for c in cands): continue
        if (q - LIMITS[:, 0] < margin).any() or (LIMITS[:, 1] - q < margin).any(): continue
        cands.append(q)
    log(f"plan_descent: {len(cands)} distinct grasp solutions")
    best = None
    n = int(np.ceil((z_top - xyz_grasp[2]) / step))
    for q in cands:
        chain = [q]; ok = True; seed = q
        for i in range(1, n + 1):
            p = xyz_grasp + [0, 0, (z_top - xyz_grasp[2]) * i / n]
            qi = self.ik_world(p, quat, seed=seed, attempts=1)
            if qi is None or np.abs(qi - seed).max() > max_jump: ok = False; break
            chain.append(qi); seed = qi
        if not ok: continue
        length = sum(np.abs(chain[i + 1] - chain[i]).sum() for i in range(len(chain) - 1))
        lim = min((q - LIMITS[:, 0]).min(), (LIMITS[:, 1] - q).min())
        log(f"  candidate q={q.round(2)} pathlen={length:.2f} limit-margin={lim:.2f}")
        score = length - 0.5 * lim
        if best is None or score < best[0]: best = (score, chain[::-1])
    return None if best is None else best[1]
Robot.plan_descent = plan_descent

def run_chain(self, chain, speed=0.05, step=0.02):
    """Execute a joint chain as one trajectory (first element should be near the current config)."""
    dt = max(0.4, step / speed)
    wps = [(q, dt * (i + 1)) for i, q in enumerate(chain[1:])]
    qf, tf = wps[-1]
    return self.move_q(qf, tf, waypoints=wps[:-1])
Robot.run_chain = run_chain
EOF
cat > plan_k.py <<'EOF'
from robot import *
import pickle
r = Robot()
for yaw in (90, -90):
    Q = Robot.quat_topdown(yaw)
    chain = r.plan_descent([-0.224, -0.126, 0.475], Q, 0.64)
    if chain is not None:
        log(f"yaw {yaw}: top q {chain[0].round(3)} grasp q {chain[-1].round(3)} len {len(chain)}")
        pickle.dump((yaw, chain), open(f"chain_k_{yaw}.pkl", "wb"))
EOF
timeout 900 python3 -u plan_k.py 2>&1 | tee plan_k.log

# openrua op 27
cat > scene.py <<'EOF'
"""Publish table + basket as collision boxes into the MoveIt planning scene."""
import rclpy, time
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

def box(name, center, size):
    co = CollisionObject(); co.id = name; co.header.frame_id = "world"
    sp = SolidPrimitive(); sp.type = SolidPrimitive.BOX; sp.dimensions = [float(s) for s in size]
    p = Pose(); p.position.x, p.position.y, p.position.z = map(float, center); p.orientation.w = 1.0
    co.primitives = [sp]; co.primitive_poses = [p]; co.operation = CollisionObject.ADD
    return co

rclpy.init(); node = rclpy.create_node("scene")
cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
assert cli.wait_for_service(20)
ps = PlanningScene(); ps.is_diff = True
ps.world.collision_objects = [
    box("table", (-0.245, 0.0, 0.375), (1.15, 1.8, 0.10)),   # top at z=0.425
    box("basket", (0.01, 0.26, 0.53), (0.17, 0.18, 0.21)),   # rim at z~0.635
]
req = ApplyPlanningScene.Request(); req.scene = ps
fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
print("applied:", fut.result().success)
g = node.create_client(GetPlanningScene, "/get_planning_scene"); g.wait_for_service(10)
from moveit_msgs.msg import PlanningSceneComponents
rq = GetPlanningScene.Request(); rq.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
fut = g.call_async(rq); rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
print("scene objects:", [c.id for c in fut.result().scene.world.collision_objects])
EOF
timeout 120 python3 scene.py

# openrua op 28
sed -i 's/req.ik_request.avoid_collisions = False/req.ik_request.avoid_collisions = True/' robot.py
cat > plan_k.py <<'EOF'
from robot import *
import pickle
r = Robot()
# sanity: current pose IK still valid with collision checking
log("cur ok:", r.ik_world(*r.fk_world(), attempts=1) is not None)
for yaw in (90, -90):
    Q = Robot.quat_topdown(yaw)
    chain = r.plan_descent([-0.224, -0.126, 0.475], Q, 0.64, n_seeds=40, margin=0.10)
    if chain is not None:
        log(f"yaw {yaw}: top q {chain[0].round(3)} grasp q {chain[-1].round(3)} len {len(chain)}")
        pickle.dump((yaw, chain), open(f"chain_k_{yaw}.pkl", "wb"))
EOF
timeout 900 python3 -u plan_k.py 2>&1 | tee plan_k.log

# openrua op 29
python3 - <<'EOF'
s=open("robot.py").read()
s=s.replace('''        score = length - 0.5 * lim
        if best is None or score < best[0]: best = (score, chain[::-1])''','''        approach = np.abs(chain[-1] - self.arm_q()).max()   # how far the chain top is from where we are
        score = length + 1.5 * approach - 0.5 * lim
        log(f"    approach-jump={approach:.2f} score={score:.2f}")
        if best is None or score < best[0]: best = (score, chain[::-1])''')
open("robot.py","w").write(s)
EOF
cat > plan_k.py <<'EOF'
from robot import *
import pickle
r = Robot()
log("current q", r.arm_q().round(3))
for yaw in (90, -90):
    Q = Robot.quat_topdown(yaw)
    chain = r.plan_descent([-0.224, -0.126, 0.475], Q, 0.64, n_seeds=40, margin=0.10)
    if chain is not None:
        log(f"yaw {yaw}: top q {chain[0].round(3)} grasp q {chain[-1].round(3)} len {len(chain)}")
        pickle.dump((yaw, chain), open(f"chain_k_{yaw}.pkl", "wb"))
EOF
timeout 900 python3 -u plan_k.py 2>&1 | grep -v "^.*candidate" | tee plan_k.log

# openrua op 30
timeout 120 python3 cloud.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
P=np.load("agentview_cloud.npz")["P"]
m=(P[...,2]>0.445)&(P[...,2]<0.53)&(P[...,0]>-0.30)&(P[...,0]<-0.15)&(P[...,1]>-0.2)&(P[...,1]<-0.05)
pts=P[m]; print("ketchup body n",len(pts),"y center",((pts[:,1].min()+pts[:,1].max())/2).round(4),"y",pts[:,1].min().round(3),pts[:,1].max().round(3),"x front",pts[:,0].max().round(3), "x back", pts[:,0].min().round(3))
m=(P[...,2]>0.535)&(P[...,2]<0.58)&(P[...,0]>-0.30)&(P[...,0]<-0.15)&(P[...,1]>-0.2)&(P[...,1]<-0.05)
pts=P[m]; print("cap n",len(pts),"center",pts.mean(0).round(4), "y",pts[:,1].min().round(3),pts[:,1].max().round(3))
EOF

# openrua op 31
cat > grasp_k.py <<'EOF'
from robot import *
r = Robot()
KX, KY = -0.215, -0.125
Q = Robot.quat_topdown(90)
chain = r.plan_descent([KX, KY, 0.475], Q, 0.64, n_seeds=40, margin=0.10)
assert chain is not None
log("top", chain[0].round(3), "grasp", chain[-1].round(3))
assert r.move_q(chain[0], 3.0), "move to chain top failed"
log("tcp top", r.fk_world()[0].round(4))
ok = r.run_chain(chain, speed=0.04)
log("descent ok", ok, "tcp", r.fk_world()[0].round(4))
f = r.gripper(GRIP["closed_m"])
log("fingers after close", f)
EOF
nohup python3 -u grasp_k.py > grasp_k.log 2>&1 &
sleep 75; grep -v candidate grasp_k.log | grep -v approach

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; python3 -c "
import cv2; im=cv2.imread('agentview.png'); cv2.imwrite('crop.png', cv2.resize(im[100:300,130:330],None,fx=3,fy=3))"

# openrua op 33
cat > lift.py <<'EOF'
from robot import *
import sys
r = Robot()
z = float(sys.argv[1]) if len(sys.argv) > 1 else 0.70
if r.fingers()[0] < 0.035: r.gripper(GRIP["open_m"])
pos, quat = r.fk_world()
r.move_line([pos[0], pos[1], z], quat, speed=0.05, max_jump=0.6)
log("tcp", r.fk_world()[0].round(3), "q", r.arm_q().round(3))
EOF
timeout 600 python3 -u lift.py 0.70 2>&1 | tee lift.log; timeout 120 python3 cloud.py birdview; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 34
python3 - <<'EOF'
import numpy as np, cv2
d=np.load("birdview_cloud.npz"); P=d["P"]; C=d["C"]
Z=P[...,2]
m=(Z>0.435)&(Z<0.6)&(P[...,0]>-0.30)&(P[...,0]<0.0)&(P[...,1]>-0.25)&(P[...,1]<-0.03)
mask=m.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<10: continue
    pts=P[lab==i]; col=C[lab==i].mean(0)
    print(f"comp {i}: n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} BGR {col.round()}")
    # print z profile along x
    xs=np.linspace(pts[:,0].min(),pts[:,0].max(),12)
    for a,b in zip(xs[:-1],xs[1:]):
        s=pts[(pts[:,0]>=a)&(pts[:,0]<b)]
        if len(s): print(f"   x~{(a+b)/2:.3f}: y[{s[:,1].min():.3f},{s[:,1].max():.3f}] zmax {s[:,2].max():.3f}")
EOF

# openrua op 35
python3 -c "
import cv2; im=cv2.imread('agentview.png'); cv2.imwrite('crop.png', cv2.resize(im[210:320,170:300],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('birdview.png'); cv2.imwrite('crop_b.png', cv2.resize(im[200:330,240:340],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null; python3 -c "
import cv2; im=cv2.imread('birdview.png'); cv2.imwrite('crop_b.png', cv2.resize(im[200:330,240:340],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 36
cat > park.py <<'EOF'
from robot import *
import sys
r = Robot()
x, y, z = map(float, sys.argv[1:4])
Q = Robot.quat_topdown(float(sys.argv[4]) if len(sys.argv) > 4 else 90)
q = r.move_world([x, y, z], Q, 4.0)
log("tcp", r.fk_world()[0].round(3), "q", r.arm_q().round(3))
EOF
timeout 600 python3 -u park.py -0.35 0.12 0.70 2>&1 | tail -2; timeout 120 python3 cloud.py birdview; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null; python3 -c "
import cv2; im=cv2.imread('birdview.png'); cv2.imwrite('crop_b.png', cv2.resize(im[200:330,240:340],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 37
python3 - <<'EOF'
import numpy as np, cv2
d=np.load("birdview_cloud.npz"); P=d["P"]; C=d["C"]
Z=P[...,2]
m=(Z>0.432)&(Z<0.6)&(P[...,0]>-0.30)&(P[...,0]<-0.02)&(P[...,1]>-0.25)&(P[...,1]<-0.03)
pts=P[m]; col=C[m]
print(f"n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f}")
xs=np.arange(pts[:,0].min(),pts[:,0].max()+0.01,0.01)
for a,b in zip(xs[:-1],xs[1:]):
    s=pts[(pts[:,0]>=a)&(pts[:,0]<b)]
    if len(s): print(f"   x~{(a+b)/2:.3f}: n={len(s):3d} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] ymid {(s[:,1].min()+s[:,1].max())/2:.3f} zmax {s[:,2].max():.3f}")
EOF

# openrua op 38
python3 - <<'EOF'
import numpy as np, cv2
d=np.load("birdview_cloud.npz"); P=d["P"]; C=d["C"].astype(int)
B,G,R=C[...,0],C[...,1],C[...,2]
orange=(R>110)&(B<90)&(R-B>60)
roi=(P[...,0]>-0.30)&(P[...,0]<-0.02)&(P[...,1]>-0.25)&(P[...,1]<-0.03)
m=orange&roi
vs,us=np.nonzero(m)
print("orange px count",m.sum(),"u range",us.min(),us.max(),"v range",vs.min(),vs.max())
# per-row (v) column extents
for v in range(vs.min(),vs.max()+1):
    row=us[vs==v]
    if len(row): print(v, row.min(), row.max(), "width px", row.max()-row.min()+1, "world x %.3f"%(-0.2+(v-240)*2.575/579.41))
# cap: grey pixels below the orange
grey=(abs(R-G)<15)&(abs(G-B)<15)&(R>90)&(R<200)&roi
vs2,us2=np.nonzero(grey); print("grey n",grey.sum(),"u",us2.min(),us2.max(),"v",vs2.min(),vs2.max())
EOF

# openrua op 39
python3 - <<'EOF'
s=open("robot.py").read()
s=s.replace('''        for attempt in range(retries + 1):
            ok = self._move_q_once(q, seconds, waypoints)
            if ok: return True
            seconds *= 1.6
            log(f"retrying slower: {seconds:.1f}s")
        return False''','''        for attempt in range(retries + 1):
            ok = self._move_q_once(q, seconds, waypoints)
            if ok: return True
            seconds *= 1.6
            if waypoints:
                # drop waypoints already passed so the retry does not replay the path from its start
                cur = self.arm_q()
                pts = [w[0] for w in waypoints] + [np.array(q)]
                k = int(np.argmin([np.abs(p - cur).max() for p in pts]))
                rem = waypoints[k + 1:]
                if rem:
                    t0 = waypoints[k][1] if k < len(waypoints) else 0.0
                    scale = seconds / max(1e-6, (rem[-1][1] if rem else 1.0) - t0 + (waypoints[-1][1] - waypoints[-2][1] if len(waypoints) > 1 else 1.0))
                    waypoints = [(w[0], (w[1] - t0) * scale) for w in rem]
                else:
                    waypoints = None
            log(f"retrying slower: {seconds:.1f}s with {0 if not waypoints else len(waypoints)} waypoints")
        return False''')
s=s.replace('''        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)''','''        # settle point: hold the final target a moment so the goal-tolerance check sees a converged arm
        settle = JointTrajectoryPoint(positions=[float(v) for v in q])
        ts = seconds + 1.0
        settle.time_from_start = Duration(sec=int(ts), nanosec=int((ts % 1) * 1e9))
        pts.append(settle)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)''')
open("robot.py","w").write(s)
EOF
cat > grasp_k2.py <<'EOF'
from robot import *
r = Robot()
KX, KY = -0.150, -0.133
Q = Robot.quat_topdown(0)          # fingers open along world y (across the lying bottle)
if r.fingers()[0] < 0.035: r.gripper(GRIP["open_m"])
chain = r.plan_descent([KX, KY, 0.44], Q, 0.60, n_seeds=40, margin=0.10)
assert chain is not None
log("top", chain[0].round(3), "grasp", chain[-1].round(3))
assert r.move_q(chain[0], 4.0), "move to chain top failed"
log("tcp top", r.fk_world()[0].round(4))
ok = r.run_chain(chain, speed=0.03)
log("descent ok", ok, "tcp", r.fk_world()[0].round(4))
f = r.gripper(GRIP["closed_m"])
log("fingers after close", f)
EOF
nohup python3 -u grasp_k2.py > grasp_k2.log 2>&1 &
sleep 100; grep -v -e candidate -e approach grasp_k2.log

# openrua op 40
cat > lift2.py <<'EOF'
from robot import *
r = Robot()
pos, quat = r.fk_world()
r.move_line([pos[0], pos[1], 0.70], quat, speed=0.04, max_jump=0.6)
log("tcp", r.fk_world()[0].round(3), "fingers", r.fingers())
EOF
timeout 600 python3 -u lift2.py 2>&1 | tail -2; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null

# openrua op 41
cat > place_k.py <<'EOF'
from robot import *
r = Robot()
BX, BY, BZ = 0.01, 0.26, 0.74
Q45 = Robot.quat_topdown(45)
pos, _ = r.fk_world()
log("rotate in place to yaw 45")
assert r.move_world(pos, Q45, 3.0) is not None
log("line to basket")
ok = r.move_line([BX, BY, BZ], Q45, speed=0.06, step=0.03, max_jump=0.6)
log("over basket ok", ok, "tcp", r.fk_world()[0].round(4), "fingers", r.fingers())
r.gripper(GRIP["open_m"])
log("released; fingers", r.fingers())
r.move_line([BX, BY, 0.85], Q45, speed=0.06, step=0.03, max_jump=0.6)
log("retreated tcp", r.fk_world()[0].round(3))
EOF
nohup python3 -u place_k.py > place_k.log 2>&1 &
sleep 120; cat place_k.log

# openrua op 42
timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; python3 -c "
import cv2; im=cv2.imread('agentview.png'); cv2.imwrite('crop.png', cv2.resize(im[150:400,420:640],None,fx=2,fy=2))"

# openrua op 43
timeout 120 python3 cloud.py birdview >/dev/null && python3 - <<'EOF'
import numpy as np, cv2
d=np.load("birdview_cloud.npz"); P=d["P"]; C=d["C"]
Z=P[...,2]
m=(Z>0.435)&(Z<0.5)&(P[...,0]>-0.25)&(P[...,0]<-0.05)&(P[...,1]>0.0)&(P[...,1]<0.15)
pts=P[m]; print(f"box n={len(pts)} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop {pts[:,2].max():.3f} center ({(pts[:,0].min()+pts[:,0].max())/2:.4f},{(pts[:,1].min()+pts[:,1].max())/2:.4f}) mean {pts.mean(0).round(4)}")
# color-based check (bluish box)
Ci=C.astype(int); B,G,R=Ci[...,0],Ci[...,1],Ci[...,2]
blue=(B>R+20)&(P[...,0]>-0.25)&(P[...,0]<-0.05)&(P[...,1]>0.0)&(P[...,1]<0.15)
vs,us=np.nonzero(blue); print("blue px", blue.sum(), "u",us.min(),us.max(),"v",vs.min(),vs.max())
print("blue center world approx x %.4f y %.4f"%(-0.2+((vs.min()+vs.max())/2-240)*2.545/579.41, ((us.min()+us.max())/2-320)*2.545/579.41))
EOF

# openrua op 44
cat > grasp_c.py <<'EOF'
from robot import *
r = Robot()
CX, CY = -0.158, 0.061
Q = Robot.quat_topdown(0)          # fingers open along world y (box's 4 cm dimension)
if r.fingers()[0] < 0.035: r.gripper(GRIP["open_m"])
chain = r.plan_descent([CX, CY, 0.437], Q, 0.60, n_seeds=40, margin=0.10)
assert chain is not None
log("top", chain[0].round(3), "grasp", chain[-1].round(3))
assert r.move_q(chain[0], 4.0), "move to chain top failed"
log("tcp top", r.fk_world()[0].round(4))
ok = r.run_chain(chain, speed=0.03)
log("descent ok", ok, "tcp", r.fk_world()[0].round(4))
f = r.gripper(GRIP["closed_m"])
log("fingers after close", f)
EOF
nohup python3 -u grasp_c.py > grasp_c.log 2>&1 &
sleep 110; grep -v -e candidate -e approach grasp_c.log

# openrua op 45
cat > place_c.py <<'EOF'
from robot import *
r = Robot()
BX, BY, BZ = 0.01, 0.26, 0.72
pos, quat = r.fk_world()
ok = r.move_line([pos[0], pos[1], 0.70], quat, speed=0.04, max_jump=0.6)
log("lifted", ok, "tcp", r.fk_world()[0].round(3), "fingers", r.fingers())
ok = r.move_line([BX, BY, BZ], quat, speed=0.06, step=0.03, max_jump=0.6)
if not ok:
    log("line failed; falling back to IK move")
    r.move_world([BX, BY, BZ], quat, 5.0)
log("over basket tcp", r.fk_world()[0].round(4), "fingers", r.fingers())
r.gripper(GRIP["open_m"])
log("released; fingers", r.fingers())
r.move_line([BX, BY, 0.85], quat, speed=0.06, step=0.03, max_jump=0.6)
log("retreated tcp", r.fk_world()[0].round(3))
EOF
nohup python3 -u place_c.py > place_c.log 2>&1 &
sleep 120; cat place_c.log

# openrua op 46
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 600 python3 -u park.py -0.35 -0.15 0.75 0 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 120 python3 cloud.py birdview >/dev/null

# openrua op 47
python3 - <<'EOF'
import numpy as np
d=np.load("birdview_cloud.npz"); P=d["P"]; C=d["C"].astype(int)
B,G,R=C[...,0],C[...,1],C[...,2]
orange=(R>110)&(B<90)&(R-B>60)
blue=(B>R+20)&(B>70)
table=(P[...,2]>0.40)&(P[...,2]<0.9)&(np.abs(P[...,1])<0.9)&(P[...,0]>-0.85)&(P[...,0]<0.35)
inb=(P[...,0]>-0.08)&(P[...,0]<0.10)&(P[...,1]>0.165)&(P[...,1]<0.355)
for name,m in (("ketchup(orange)",orange),("cream cheese(blue)",blue)):
    tot=(m&table).sum(); ib=(m&table&inb).sum()
    pts=P[m&table&inb]
    print(f"{name}: {tot} px on table, {ib} inside basket footprint; z range in basket {pts[:,2].min():.3f}-{pts[:,2].max():.3f}" if len(pts) else f"{name}: {tot} px on table, none in basket")
    out=P[m&table&~inb]
    print("   outside-basket px:", len(out), "" if not len(out) else f"at x~{out[:,0].mean():.2f} y~{out[:,1].mean():.2f}")
EOF
