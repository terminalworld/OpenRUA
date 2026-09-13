#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 node list

# openrua op 4
timeout 60 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 60 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80; echo; timeout 30 ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:"

# openrua op 6
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
def cb(m, which):
    for t in m.transforms:
        got[(which, t.header.frame_id, t.child_frame_id)] = t.transform
node.create_subscription(TFMessage, "/tf_static", lambda m: cb(m, "static"), qos)
node.create_subscription(TFMessage, "/tf", lambda m: cb(m, "dyn"), 100)
import time
end = time.time() + 8
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
for k, t in sorted(got.items()):
    print(k, f"t=({t.translation.x:.4f},{t.translation.y:.4f},{t.translation.z:.4f}) q=({t.rotation.x:.4f},{t.rotation.y:.4f},{t.rotation.z:.4f},{t.rotation.w:.4f})")
EOF
timeout 60 python3 tfdump.py

# openrua op 7
cat > locate.py <<'EOF'
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import time
cam = sys.argv[1] if len(sys.argv)>1 else "birdview"
rclpy.init(); node = rclpy.create_node("loc")
tfbuf = Buffer(); TransformListener(tfbuf, node)
got = {}
node.create_subscription(Image, f"/{cam}/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(Image, f"/{cam}/color/image_raw", lambda m: got.setdefault("c", m), 1)
node.create_subscription(CameraInfo, f"/{cam}/color/camera_info", lambda m: got.setdefault("i", m), 1)
end=time.time()+20
while time.time()<end and not ("d" in got and "c" in got and "i" in got and tfbuf.can_transform("world", f"{cam}_optical_frame", rclpy.time.Time())):
    rclpy.spin_once(node, timeout_sec=0.2)
br = CvBridge()
depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
color = br.imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx,fy,cx,cy = k[0],k[4],k[2],k[5]
t = tfbuf.lookup_transform("world", f"{cam}_optical_frame", rclpy.time.Time())
q = t.transform.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T = np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
H,W = depth.shape
vv,uu = np.mgrid[0:H,0:W]
P = np.stack([(uu-cx)*depth/fx,(vv-cy)*depth/fy,depth],-1) @ R.T + T
np.save(f"{cam}_world.npy", P)
print("depth range", np.nanmin(depth), np.nanmax(depth))
# table height estimate: median z of central region
zc = P[...,2]
print("median z center", np.median(zc[200:400, 200:450]))
for (u,v) in [(328,318),(283,268),(353,270),(320,284),(320,200),(320,400)]:
    print((u,v), "depth", depth[v,u], "world", P[v,u].round(4), "bgr", color[v,u])
EOF
timeout 90 python3 locate.py birdview

# openrua op 8
cat > locate2.py <<'EOF'
import sys, numpy as np
cam=sys.argv[1]
P=np.load(f"{cam}_world.npy")
import cv2
color=cv2.imread(f"{cam}.png")
pts=[tuple(map(int,a.split(","))) for a in sys.argv[2:]]
for u,v in pts:
    print((u,v), P[v,u].round(4), color[v,u])
EOF
timeout 90 python3 locate.py agentview >/dev/null; python3 locate2.py agentview 340,375 210,230 210,190 415,240 415,190 328,230 328,215 100,400 500,400 320,300

# openrua op 9
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 2.0 && timeout 90 python3 locate.py birdview > /dev/null && timeout 90 python3 locate.py agentview >/dev/null && python3 locate2.py agentview 340,375 210,230 415,240 320,300 && python3 locate2.py birdview 328,318 283,268 353,270

# openrua op 10
timeout 60 python3 tools/perception/cam_snap.py birdview && python3 - <<'EOF'
import cv2, numpy as np
img=cv2.imread("birdview.png")
P=np.load("birdview_world.npy")
crop=img[220:360, 240:420]
cv2.imwrite("bird_crop.png", cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
# height-based mask of objects: z between 0.43 and 0.60 (above table, below arm)
z=P[...,2]
mask=(z>0.432)&(z<0.62)
mask[:, :200]=False; mask[:, 440:]=False; mask[:200]=False; mask[380:]=False
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    xyz=P[m]
    print(f"comp {i}: px centroid {cent[i].round(1)}, area {stats[i,4]}, world xy mean {xyz[:,:2].mean(0).round(4)}, z range {xyz[:,2].min():.3f}-{xyz[:,2].max():.3f}, bgr mean {img[m].mean(0).round(0)}")
EOF

# openrua op 11
timeout 90 python3 locate.py robot0_eye_in_hand > /dev/null && python3 - <<'EOF'
import cv2, numpy as np
img=cv2.imread("robot0_eye_in_hand.png")
P=np.load("robot0_eye_in_hand_world.npy")
z=P[...,2]
# pudding: region around (305,390); print world coords of a grid
for v in range(370,410,10):
    print([tuple(P[v,u,:].round(3)) for u in range(280,340,15)])
mask=(z>0.44)&(z<0.50)
mask[:300]=False  # only lower part
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<50: continue
    m=lab==i; xyz=P[m]
    print(f"comp {i}: px {cent[i].round(1)}, area {stats[i,4]}, xy {xyz[:,:2].mean(0).round(4)}, xrange {xyz[:,0].min():.3f}-{xyz[:,0].max():.3f} yrange {xyz[:,1].min():.3f}-{xyz[:,1].max():.3f} z {xyz[:,2].min():.3f}-{xyz[:,2].max():.3f} bgr {img[m].mean(0).round(0)}")
EOF

# openrua op 12
cat > fk.py <<'EOF'
"""FK of panda_hand and TCP in world from current /joint_states via TF chain (numpy)."""
import numpy as np, rclpy, time, sys
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
from scipy.spatial.transform import Rotation as Rot
def get_chain():
    rclpy.init(); node = rclpy.create_node("fk")
    got = {}
    qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def cb(m):
        for t in m.transforms: got[(t.header.frame_id, t.child_frame_id)] = t.transform
    node.create_subscription(TFMessage, "/tf_static", cb, qos)
    node.create_subscription(TFMessage, "/tf", cb, 100)
    end=time.time()+5
    while time.time()<end: rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown(); return got
def T_of(t):
    M=np.eye(4); M[:3,:3]=Rot.from_quat([t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w]).as_matrix()
    M[:3,3]=[t.translation.x,t.translation.y,t.translation.z]; return M
if __name__=="__main__":
    g=get_chain()
    chain=["world","panda_link0","panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_link8","panda_hand"]
    M=np.eye(4)
    for a,b in zip(chain,chain[1:]): M=M@T_of(g[(a,b)])
    tcp=M@np.array([0,0,0.1034,1])
    print("hand pos world", M[:3,3].round(4), "quat xyzw", Rot.from_matrix(M[:3,:3]).as_quat().round(4))
    print("hand axes in world: x", M[:3,0].round(3), "y", M[:3,1].round(3), "z", M[:3,2].round(3))
    print("TCP world", tcp[:3].round(4))
    for i in range(1,8):
        t=g[(f"panda_link{i-1}",f"panda_link{i}")]
EOF
timeout 60 python3 fk.py

# openrua op 13
timeout 600 python3 tools/action/ik_move.py 0.10 0.0 0.60 1 0 0 0 3 && timeout 60 python3 fk.py && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 14
cat > arm.py <<'EOF'
"""Reusable arm helpers: joints, IK (prints solution), FJT, gripper, FK."""
import sys, time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
ARM = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
JOINTS = ARM["joints"]
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])
TCP_OFF = M["hand"]["tcp_offset_m"]

class Arm:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, ARM["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
    def spin(self, fut, timeout=600):
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout); return fut.result()
    def joints(self, fresh=True):
        if fresh: self._js.pop("m", None)
        end = time.time()+10
        while "m" not in self._js and time.time()<end: rclpy.spin_once(self.node, timeout_sec=0.1)
        m = self._js["m"]; d = dict(zip(m.name, m.position))
        return d
    def arm_q(self):
        d = self.joints(); return [d[j] for j in JOINTS]
    def fingers(self):
        d = self.joints(); return d["panda_finger_joint1"], d["panda_finger_joint2"]
    def fk_hand(self):
        """hand pose in planner frame via /compute_fk; returns (pos, quat xyzw)"""
        self.fk.wait_for_service(5)
        req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = self.arm_q()
        res = self.spin(self.fk.call_async(req), 60)
        p = res.pose_stamped[0].pose
        return res.pose_stamped[0].header.frame_id, np.array([p.position.x,p.position.y,p.position.z]), np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
    def solve_ik(self, pos, quat, seed=None, tries=3):
        self.ik.wait_for_service(5)
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = M["planning"]["group"]; r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = map(float,pos)
        r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y, r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w = map(float,quat)
        r.robot_state.joint_state.name = JOINTS
        r.robot_state.joint_state.position = list(seed) if seed is not None else self.arm_q()
        r.timeout.sec = 2
        for _ in range(tries):
            res = self.spin(self.ik.call_async(req), 60)
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in JOINTS]
        print("IK failed", None if res is None else res.error_code.val); return None
    def move_joints(self, q, seconds=3.0, retries=2):
        self.fjt.wait_for_server(5)
        for attempt in range(retries+1):
            goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = JOINTS
            pt = JointTrajectoryPoint(positions=[float(x) for x in q])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds%1)*1e9))
            goal.trajectory.points = [pt]
            gh = self.spin(self.fjt.send_goal_async(goal))
            res = self.spin(gh.get_result_async())
            code = res.result.error_code
            cur = np.array(self.arm_q()); err = np.abs(cur-np.array(q)).max()
            print(f"fjt code={code} max joint err={err:.4f}")
            if err < 0.02: return True
            seconds = max(seconds, 2.0)
        return err < 0.05
    def gripper(self, width):
        self.grip.wait_for_server(5)
        g = GripperCommand.Goal(); g.command.position = float(width); g.command.max_effort = float(GRIP["max_effort"])
        gh = self.spin(self.grip.send_goal_async(g)); res = self.spin(gh.get_result_async(), 300)
        f = self.fingers(); print(f"gripper reached={res.result.reached_goal} stalled={res.result.stalled} fingers={f}")
        return f
def down_quat(yaw_deg=0.0):
    """hand pointing down (z down), fingers along world y when yaw=0; yaw rotates about world z."""
    return Rot.from_euler("z", yaw_deg, degrees=True).__mul__(Rot.from_quat([1,0,0,0])).as_quat()
EOF
cat > probe.py <<'EOF'
from arm import *
a = Arm()
print("joints", np.round(a.arm_q(),4))
print("fk", a.fk_hand())
q = a.solve_ik([0.10,0,0.60],[1,0,0,0])
print("ik sol for (0.10,0,0.60):", None if q is None else np.round(q,4))
EOF
timeout 120 python3 probe.py

# openrua op 15
timeout 90 python3 locate.py birdview > /dev/null && python3 - <<'EOF'
import cv2, numpy as np
img=cv2.imread("birdview.png")
P=np.load("birdview_world.npy")
z=P[...,2]
mask=(z>0.432)&(z<0.62)
mask[:, :200]=False; mask[:, 440:]=False; mask[:200]=False; mask[380:]=False
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i; xyz=P[m]
    print(f"comp {i}: px {cent[i].round(1)}, area {stats[i,4]}, xy {xyz[:,:2].mean(0).round(4)}, x {xyz[:,0].min():.3f}..{xyz[:,0].max():.3f} y {xyz[:,1].min():.3f}..{xyz[:,1].max():.3f} z {xyz[:,2].min():.3f}..{xyz[:,2].max():.3f} bgr {img[m].mean(0).round(0)}")
    # rim: highest points
    top = m & (z > xyz[:,2].max()-0.01)
    if top.sum()>5: print("   top-ring xy mean", P[top][:,:2].mean(0).round(4), "n", top.sum())
crop=img[220:360, 240:420]
cv2.imwrite("bird_crop.png", cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 16
python3 - <<'EOF'
import numpy as np, cv2
P=np.load("birdview_world.npy"); z=P[...,2]
img=cv2.imread("birdview.png")
def fit_circle(pts):
    x,y=pts[:,0],pts[:,1]
    A=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]
    r=np.sqrt(c[2]+c[0]**2+c[1]**2); return c[0],c[1],r
# white mug rim: z > 0.535 within region
for name,(x0,x1,y0,y1) in {"white":(-0.16,-0.03,-0.22,-0.06),"red":(-0.13,-0.01,0.09,0.19)}.items():
    reg=(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)
    for zt in [0.53, 0.54]:
        m=reg&(z>zt)
        pts=P[m][:,:2]
        cx,cy,r=fit_circle(pts)
        # also inner-bottom
        print(name, f"z>{zt}: n={m.sum()} circle center ({cx:.4f},{cy:.4f}) r={r:.4f}  ptmean {pts.mean(0).round(4)}")
    inner=reg&(z>0.44)&(z<0.47)
    print(name,"inner floor pts", inner.sum(), "mean", P[inner][:,:2].mean(0).round(4), "z", P[inner][:,2].mean().round(4))
    body=reg&(z>0.5)
    print(name, "handle-ish pts", (reg&(z>0.47)&(z<0.53)).sum(), P[reg&(z>0.47)&(z<0.53)][:,:2].mean(0).round(4))
EOF

# openrua op 17
cat >> arm.py <<'EOF'

def tcp_to_hand(tcp, yaw_deg=0.0):
    q = down_quat(yaw_deg); R = Rot.from_quat(q).as_matrix()
    return np.array(tcp) - TCP_OFF * R[:, 2], q

def move_tcp(a, tcp, yaw_deg=0.0, seconds=3.0, seed=None):
    pos, q = tcp_to_hand(tcp, yaw_deg)
    sol = a.solve_ik(pos, q, seed=seed)
    if sol is None: return False
    ok = a.move_joints(sol, seconds)
    fr, p, qq = a.fk_hand()
    R = Rot.from_quat(qq).as_matrix(); tcp_now = p + TCP_OFF * R[:, 2]
    print(f"TCP now {tcp_now.round(4)} target {np.round(tcp,4)}  hand y-axis {R[:,1].round(3)}")
    return ok
EOF
cat > step1.py <<'EOF'
from arm import *
a = Arm()
a.gripper(0.04)
move_tcp(a, [-0.094, -0.195, 0.66], 0.0, 4.0)
EOF
timeout 900 python3 -u step1.py 2>&1 | tee step1.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 18
timeout 60 python3 fk.py; timeout 60 ros2 topic echo /joint_states --once | grep -A9 "^position"

# openrua op 19
cat > iktest.py <<'EOF'
from arm import *
a = Arm()
def fk_of(q):
    req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand","panda_link8"]
    req.robot_state.joint_state.name = JOINTS; req.robot_state.joint_state.position = list(q)
    res = a.spin(a.fk.call_async(req), 60)
    out=[]
    for ps in res.pose_stamped:
        p=ps.pose; qq=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
        out.append((np.round([p.position.x,p.position.y,p.position.z],4), np.round(qq,4), Rot.from_quat(qq).as_matrix()[:,1].round(3)))
    return out
cur = a.arm_q()
print("current q", np.round(cur,4)); print("  fk hand/link8:", fk_of(cur))
for yaw in [0, 45, -45, 90]:
    pos, q = tcp_to_hand([-0.094,-0.195,0.66], yaw)
    sol = a.solve_ik(pos, q, seed=cur)
    print(f"yaw {yaw}: req quat {np.round(q,4)}  sol {None if sol is None else np.round(sol,3)}")
    if sol: print("   fk:", fk_of(sol))
EOF
timeout 300 python3 iktest.py

# openrua op 20
sed -i 's/req.fk_link_names = \["panda_hand","panda_link8"\]/req.fk_link_names = ["panda_hand"]/' iktest.py && timeout 400 python3 iktest.py

# openrua op 21
sed -i 's/    req = GetPositionFK.Request(); req.fk_link_names = \["panda_hand"\]/    a.fk.wait_for_service(10); req = GetPositionFK.Request(); req.fk_link_names = ["panda_hand"]/' iktest.py && timeout 400 python3 iktest.py

# openrua op 22
python3 - <<'EOF'
s=open("arm.py").read()
s=s.replace("""    pos, q = tcp_to_hand(tcp, yaw_deg)
    sol = a.solve_ik(pos, q, seed=seed)""","""    pos, _ = tcp_to_hand(tcp, yaw_deg)
    # machine fact: this IK solver's hand yaw comes out +45 deg from the request
    _, q_req = tcp_to_hand(tcp, yaw_deg - 45.0)
    sol = a.solve_ik(pos, q_req, seed=seed)""")
open("arm.py","w").write(s)
EOF
cat > step2.py <<'EOF'
from arm import *
a = Arm()
move_tcp(a, [-0.094, -0.195, 0.66], 0.0, 2.0)
EOF
timeout 600 python3 -u step2.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 23
cat > step3.py <<'EOF'
from arm import *
a = Arm()
ok1 = move_tcp(a, [-0.094, -0.195, 0.575], 0.0, 2.0)
ok2 = move_tcp(a, [-0.094, -0.195, 0.529], 0.0, 2.0)
print("descent ok", ok1, ok2)
EOF
timeout 600 python3 -u step3.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 24
cat > step4.py <<'EOF'
from arm import *
a = Arm()
a.gripper(0.0)
import time
for i in range(3):
    a.move_joints(a.arm_q(), 0.5)   # tick the sim so finger readings settle
    print("fingers", a.fingers())
EOF
timeout 600 python3 -u step4.py

# openrua op 25
cat > step5.py <<'EOF'
from arm import *
a = Arm()
move_tcp(a, [-0.094, -0.195, 0.72], 0.0, 3.0)
print("fingers", a.fingers())
EOF
timeout 600 python3 -u step5.py && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 26
timeout 90 python3 locate.py birdview > /dev/null && python3 - <<'EOF'
import numpy as np, cv2
P=np.load("birdview_world.npy"); z=P[...,2]; img=cv2.imread("birdview.png")
def fit_circle(pts):
    x,y=pts[:,0],pts[:,1]
    A=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]
    return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)
reg=(P[...,0]>0.02)&(P[...,0]<0.26)&(P[...,1]>-0.12)&(P[...,1]<0.16)
m=reg&(z>0.436)&(z<0.50)
pts=P[m][:,:2]
print("plate pts", m.sum(), "mean", pts.mean(0).round(4), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3))
# edge points: boundary of mask
er=cv2.erode(m.astype(np.uint8), np.ones((3,3),np.uint8))
edge=m&(er==0)
print("edge circle (cx,cy,r)", np.round(fit_circle(P[edge][:,:2]),4))
print("z stats", np.percentile(P[m][:,2],[5,50,95]).round(4))
# brightness-based mask as cross-check
b=img.mean(2)
mb=reg&(b>120)
print("bright pts", mb.sum(), "mean", P[mb][:,:2].mean(0).round(4))
EOF

# openrua op 27
cat > step6.py <<'EOF'
from arm import *
a = Arm()
move_tcp(a, [0.135, -0.0195, 0.72], 0.0, 4.0)
print("fingers", a.fingers())
move_tcp(a, [0.135, -0.0195, 0.60], 0.0, 2.5)
move_tcp(a, [0.135, -0.0195, 0.556], 0.0, 2.0)
print("fingers", a.fingers())
EOF
timeout 900 python3 -u step6.py && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 28
cat > step7.py <<'EOF'
from arm import *
a = Arm()
a.gripper(0.04)
move_tcp(a, [0.135, -0.0195, 0.60], 0.0, 1.5)
move_tcp(a, [0.135, -0.0195, 0.75], 0.0, 2.0)
print("fingers", a.fingers())
EOF
timeout 900 python3 -u step7.py && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 29
timeout 90 python3 locate.py birdview > /dev/null && python3 - <<'EOF'
import numpy as np, cv2
P=np.load("birdview_world.npy"); z=P[...,2]; img=cv2.imread("birdview.png")
def fit_circle(pts):
    x,y=pts[:,0],pts[:,1]
    A=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]
    return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)
reg=(P[...,0]>0.02)&(P[...,0]<0.26)&(P[...,1]>-0.12)&(P[...,1]<0.16)
rim=reg&(z>0.55)
print("mug rim pts", rim.sum(), "circle", np.round(fit_circle(P[rim][:,:2]),4), "z max", P[rim][:,2].max().round(4))
# pudding region
reg2=(P[...,0]>-0.25)&(P[...,0]<-0.10)&(P[...,1]>-0.06)&(P[...,1]<0.08)
m=reg2&(z>0.435)&(z<0.60)
pts=P[m]
print("pudding pts", m.sum(), "mean xy", pts[:,:2].mean(0).round(4), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "z", np.percentile(pts[:,2],[5,50,95]).round(4))
# orientation of the box via PCA of top points
top=m&(z>0.455)
xy=P[top][:,:2]; c=xy.mean(0); u,s,vt=np.linalg.svd(xy-c)
print("top pts", top.sum(), "center", c.round(4), "axes", vt.round(3), "extent along axes", (2*np.sqrt(s**2/len(xy))).round(3) if len(xy) else None)
proj=(xy-c)@vt.T
print("extent along principal axes (min..max):", proj.min(0).round(3), proj.max(0).round(3))
crop=img[250:300, 290:360]
cv2.imwrite("pud_crop.png", cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 30
cat > park.py <<'EOF'
from arm import *
import sys
a = Arm()
x,y,z = map(float, sys.argv[1:4]); yaw = float(sys.argv[4]) if len(sys.argv)>4 else 0.0
move_tcp(a, [x,y,z], yaw, float(sys.argv[5]) if len(sys.argv)>5 else 3.0)
EOF
timeout 600 python3 -u park.py -0.15 -0.32 0.70 && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 31
timeout 90 python3 locate.py birdview > /dev/null && python3 - <<'EOF'
import numpy as np, cv2
P=np.load("birdview_world.npy"); z=P[...,2]; img=cv2.imread("birdview.png")
def fit_circle(pts):
    x,y=pts[:,0],pts[:,1]
    A=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]
    return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)
reg=(P[...,0]>0.02)&(P[...,0]<0.26)&(P[...,1]>-0.12)&(P[...,1]<0.16)
rim=reg&(z>0.55)
print("mug rim pts", rim.sum(), "circle", np.round(fit_circle(P[rim][:,:2]),4), "z max", P[rim][:,2].max().round(4))
plate=reg&(z>0.436)&(z<0.50)
er=cv2.erode(plate.astype(np.uint8), np.ones((3,3),np.uint8)); edge=plate&(er==0)
# outer edge only: points far from mug center
pe=P[edge][:,:2]
print("plate visible pts", plate.sum(), "outer-edge circle", np.round(fit_circle(pe[np.linalg.norm(pe-[0.135,0.0225],axis=1)>0.055]),4))
reg2=(P[...,0]>-0.25)&(P[...,0]<-0.10)&(P[...,1]>-0.06)&(P[...,1]<0.08)
m=reg2&(z>0.435)&(z<0.60)
pts=P[m]
print("pudding pts", m.sum(), "mean xy", pts[:,:2].mean(0).round(4), "x", pts[:,0].min().round(3), pts[:,0].max().round(3), "y", pts[:,1].min().round(3), pts[:,1].max().round(3), "z", np.percentile(pts[:,2],[5,50,95]).round(4))
top=m&(z>0.455)
xy=P[top][:,:2]; c=xy.mean(0); u,s,vt=np.linalg.svd(xy-c)
proj=(xy-c)@vt.T
print("top pts", top.sum(), "center", c.round(4), "axes", vt.round(3), "extent (min..max):", proj.min(0).round(3), proj.max(0).round(3))
crop=img[255:300, 295:350]
cv2.imwrite("pud_crop.png", cv2.resize(crop,None,fx=8,fy=8,interpolation=cv2.INTER_NEAREST))
EOF

# openrua op 32
python3 - <<'EOF'
import numpy as np
P=np.load("birdview_world.npy"); z=P[...,2]
# hand region around TCP (-0.15,-0.32); hand body between z 0.74 and 0.85 (TCP 0.70, palm bottom ~0.745)
reg=(P[...,0]>-0.30)&(P[...,0]<0.0)&(P[...,1]>-0.50)&(P[...,1]<-0.15)
for lo,hi in [(0.70,0.745),(0.745,0.80),(0.80,0.86)]:
    m=reg&(z>lo)&(z<hi); pts=P[m]
    if m.sum(): print(f"z {lo}-{hi}: n={m.sum()} x {pts[:,0].min():.3f}..{pts[:,0].max():.3f} y {pts[:,1].min():.3f}..{pts[:,1].max():.3f}")
EOF

# openrua op 33
cat > step8.py <<'EOF'
from arm import *
a = Arm()
print("fingers", a.fingers())
move_tcp(a, [-0.200, 0.013, 0.60], 0.0, 3.0)
move_tcp(a, [-0.200, 0.013, 0.50], 0.0, 2.0)
EOF
timeout 900 python3 -u step8.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 34
cat > step9.py <<'EOF'
from arm import *
a = Arm()
move_tcp(a, [-0.200, 0.013, 0.448], 0.0, 2.0)
a.gripper(0.0)
for i in range(2):
    a.move_joints(a.arm_q(), 0.5); print("fingers", a.fingers())
move_tcp(a, [-0.200, 0.013, 0.60], 0.0, 2.0)
print("fingers after lift", a.fingers())
EOF
timeout 900 python3 -u step9.py && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 35
cat > step10.py <<'EOF'
from arm import *
a = Arm()
move_tcp(a, [-0.200, 0.013, 0.72], 0.0, 2.0)
move_tcp(a, [0.135, 0.155, 0.72], 90.0, 4.0)
print("fingers", a.fingers())
move_tcp(a, [0.135, 0.155, 0.56], 90.0, 2.5)
print("fingers", a.fingers())
EOF
timeout 900 python3 -u step10.py && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 36
cat > step11.py <<'EOF'
from arm import *
a = Arm()
move_tcp(a, [0.135, 0.155, 0.456], 90.0, 2.0)
a.gripper(0.04)
move_tcp(a, [0.135, 0.155, 0.60], 90.0, 2.0)
move_tcp(a, [-0.10, -0.30, 0.75], 0.0, 3.0)
print("fingers", a.fingers())
EOF
timeout 900 python3 -u step11.py && timeout 60 python3 tools/perception/cam_snap.py agentview && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 37
timeout 90 python3 locate.py birdview > /dev/null && python3 - <<'EOF'
import numpy as np, cv2
P=np.load("birdview_world.npy"); z=P[...,2]; img=cv2.imread("birdview.png")
def fit_circle(pts):
    x,y=pts[:,0],pts[:,1]
    A=np.c_[2*x,2*y,np.ones(len(x))]; b=x**2+y**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]
    return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)
reg=(P[...,0]>0.02)&(P[...,0]<0.26)&(P[...,1]>-0.12)&(P[...,1]<0.10)
rim=reg&(z>0.55); cx,cy,r=fit_circle(P[rim][:,:2])
plate=reg&(z>0.436)&(z<0.50); er=cv2.erode(plate.astype(np.uint8), np.ones((3,3),np.uint8)); edge=plate&(er==0)
pe=P[edge][:,:2]; px,py,pr=fit_circle(pe[np.linalg.norm(pe-[0.135,0.0225],axis=1)>0.055])
print(f"WHITE MUG rim center ({cx:.4f},{cy:.4f}) r={r:.3f} top z={P[rim][:,2].max():.3f}  (n={rim.sum()})")
print(f"PLATE center ({px:.4f},{py:.4f}) r={pr:.3f}")
print(f"  mug-plate horizontal offset = {np.hypot(cx-px,cy-py)*100:.1f} cm  (plate radius {pr*100:.1f} cm) -> mug ON plate: {np.hypot(cx-px,cy-py) < pr-0.02}")
reg2=(P[...,0]>0.05)&(P[...,0]<0.25)&(P[...,1]>0.10)&(P[...,1]<0.25)
m=reg2&(z>0.435)&(z<0.50); pts=P[m]
print(f"PUDDING pts={m.sum()} center ({pts[:,0].mean():.4f},{pts[:,1].mean():.4f}) x {pts[:,0].min():.3f}..{pts[:,0].max():.3f} y {pts[:,1].min():.3f}..{pts[:,1].max():.3f} top z {np.percentile(pts[:,2],95):.3f}")
print(f"  pudding is {(pts[:,1].mean()-py)*100:.1f} cm in +y (agentview-right) of plate center; gap to plate rim {(pts[:,1].min()-(py+pr))*100:.1f} cm; x offset {(pts[:,0].mean()-px)*100:.1f} cm")
# anything else displaced? red mug
reg3=(P[...,0]>-0.15)&(P[...,0]<0.0)&(P[...,1]>0.08)&(P[...,1]<0.20); rm=reg3&(z>0.53)
print("RED MUG rim center", np.round(fit_circle(P[rm][:,:2])[:2],4), "(was (-0.065,0.126))")
EOF
