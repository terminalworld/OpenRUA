#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -v parameter | head -40; echo ----; ros2 node list

# openrua op 4
cd /workspace; for c in agentview birdview frontview sideview robot0_eye_in_hand; do python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
cd /workspace; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12; timeout 10 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | sort | uniq

# openrua op 6
cd /workspace; cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
def cb(m):
    for t in m.transforms:
        got[(t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(TFMessage, "/tf_static", cb, qos)
node.create_subscription(TFMessage, "/tf", cb, 100)
import time
for _ in range(30): rclpy.spin_once(node, timeout_sec=0.2)
for k, t in sorted(got.items()):
    print(k, f"t=({t.translation.x:.3f},{t.translation.y:.3f},{t.translation.z:.3f}) q=({t.rotation.x:.3f},{t.rotation.y:.3f},{t.rotation.z:.3f},{t.rotation.w:.3f})")
EOF
python3 tfdump.py

# openrua op 7
cd /workspace; cat > pxw.py <<'EOF'
"""pxw.py <camera> u,v [u,v ...] : print world xyz for several pixels using one grab."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy

def R_of(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def grab(node, topic, T):
    got={}
    sub=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub); return got["m"]

def cam_pose(node, frame):
    got={}
    def cb(m):
        for t in m.transforms:
            if t.child_frame_id==frame and t.header.frame_id=="world": got["t"]=t.transform
    qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    s1=node.create_subscription(TFMessage,"/tf_static",cb,qos); s2=node.create_subscription(TFMessage,"/tf",cb,100)
    while "t" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    t=got["t"]; T=np.eye(4); T[:3,:3]=R_of((t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w))
    T[:3,3]=[t.translation.x,t.translation.y,t.translation.z]; return T

def main():
    cam=sys.argv[1]; pts=[tuple(map(int,a.split(","))) for a in sys.argv[2:]]
    rclpy.init(); node=rclpy.create_node("pxw")
    depth=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
    T=cam_pose(node,f"{cam}_optical_frame")
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    for u,v in pts:
        z=float(D[v,u]); p=T@np.array([(u-cx)*z/fx,(v-cy)*z/fy,z,1])
        print(f"({u},{v}) depth={z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()
main()
EOF
python3 pxw.py birdview 316,320 316,300 316,340 300,320 332,320 275,262 262,262 288,262 345,280 200,200 316,150 100,240

# openrua op 8
cd /workspace; cat > heightmap.py <<'EOF'
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
sys.argv=[sys.argv[0],"birdview"]
exec(open("pxw.py").read().split("def main")[0])
rclpy.init(); node=rclpy.create_node("hm")
cam="birdview"
depth=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
T=cam_pose(node,f"{cam}_optical_frame")
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=D.shape
vv,uu=np.mgrid[0:H,0:W]
X=(uu-cx)*D/fx; Y=(vv-cy)*D/fy
P=np.stack([X,Y,D,np.ones_like(D)],-1)@T.T
Z=P[...,2]
np.save("bird_world.npy",P[...,:3])
print("table z mode:", np.median(Z[(Z>0.3)&(Z<0.5)]))
mask=((Z>0.44)&(Z<0.75)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    pts=P[m][:,:3]
    print(f"blob{i}: px area={stats[i,4]} centroid px=({cent[i][0]:.0f},{cent[i][1]:.0f}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
cv2.imwrite("bird_mask.png", mask*255)
rclpy.shutdown()
EOF
python3 heightmap.py

# openrua op 9
cd /workspace; python3 pxw.py robot0_eye_in_hand 290,395 280,395 300,395 290,385 340,60 340,20 340,100 2>/dev/null; python3 pxw.py agentview 330,215 335,225 325,225 200,230 385,240 300,380 300,350 300,410 250,380 350,380 2>/dev/null

# openrua op 10
cd /workspace; cat > fk.py <<'EOF'
import rclpy, yaml, sys
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); node=rclpy.create_node("fk")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
arm=[f"panda_joint{i}" for i in range(1,8)]
cur=dict(zip(js["m"].name,js["m"].position))
print("joints:", [round(cur[j],4) for j in arm], "fingers:", [round(cur[f"panda_finger_joint{i}"],4) for i in (1,2)])
cli=node.create_client(GetPositionFK,"/compute_fk"); cli.wait_for_service()
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand"]
req.robot_state.joint_state.name=arm; req.robot_state.joint_state.position=[cur[j] for j in arm]
fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=30)
r=fut.result()
p=r.pose_stamped[0].pose
print("frame:", r.pose_stamped[0].header.frame_id, "err", r.error_code.val)
print(f"hand (base frame): {p.position.x:.4f} {p.position.y:.4f} {p.position.z:.4f}  q {p.orientation.x:.4f} {p.orientation.y:.4f} {p.orientation.z:.4f} {p.orientation.w:.4f}")
print(f"hand (world): {p.position.x-0.51:.4f} {p.position.y:.4f} {p.position.z+0.42:.4f}")
rclpy.shutdown()
EOF
python3 fk.py 2>/dev/null

# openrua op 11
cd /workspace; cat > tfchain.py <<'EOF'
import numpy as np, rclpy
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy
exec(open("pxw.py").read().split("def grab")[0])
rclpy.init(); node=rclpy.create_node("tfc")
got={}
def cb(m):
    for t in m.transforms: got[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
node.create_subscription(TFMessage,"/tf_static",cb,qos); node.create_subscription(TFMessage,"/tf",cb,100)
for _ in range(20): rclpy.spin_once(node,timeout_sec=0.2)
def M(t):
    T=np.eye(4); T[:3,:3]=R_of((t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)); T[:3,3]=[t.translation.x,t.translation.y,t.translation.z]; return T
chain=["world","panda_link0"]+[f"panda_link{i}" for i in range(1,9)]+["panda_hand"]
T=np.eye(4)
for a,b in zip(chain,chain[1:]):
    T=T@M(got[(a,b)])
    print(b, np.round(T[:3,3],4))
print("hand R:\n",np.round(T[:3,:3],3))
rclpy.shutdown()
EOF
python3 tfchain.py 2>/dev/null

# openrua op 12
cd /workspace; cat > iktest.py <<'EOF'
import rclpy, sys, numpy as np
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState
rclpy.init(); node=rclpy.create_node("iktest")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
arm=[f"panda_joint{i}" for i in range(1,8)]
cur=dict(zip(js["m"].name,js["m"].position))
cli=node.create_client(GetPositionIK,"/compute_ik"); cli.wait_for_service()
def ik(x,y,z,q=(0.9996,0,-0.0284,0),frame=""):
    req=GetPositionIK.Request(); r=req.ik_request
    r.group_name="panda_arm"; r.pose_stamped.header.frame_id=frame
    r.pose_stamped.pose.position.x=x; r.pose_stamped.pose.position.y=y; r.pose_stamped.pose.position.z=z
    r.pose_stamped.pose.orientation.x,r.pose_stamped.pose.orientation.y,r.pose_stamped.pose.orientation.z,r.pose_stamped.pose.orientation.w=q
    r.robot_state.joint_state.name=arm; r.robot_state.joint_state.position=[cur[j] for j in arm]
    r.ik_link_name="panda_hand"; r.avoid_collisions=False
    fut=cli.call_async(req); rclpy.spin_until_future_complete(node,fut,timeout_sec=30)
    res=fut.result()
    if res is None: return None, "timeout"
    if res.error_code.val!=1: return None, res.error_code.val
    sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
    return [round(sol[j],3) for j in arm], 1
print("current:", [round(cur[j],3) for j in arm])
print("world coords :", ik(-0.053,0.0,0.7776))
print("base coords  :", ik(0.457,0.0,0.3576))
rclpy.shutdown()
EOF
python3 iktest.py 2>/dev/null

# openrua op 13
cd /workspace; python3 iktest.py 2>&1 | grep -v XMLPARSER | tail -20

# openrua op 14
cd /workspace; sed -i 's/def ik(x,y,z,q=(0.9996,0,-0.0284,0),frame=""):/def ik(x,y,z,q=(0.9996,0.0,-0.0284,0.0),frame=""):\n    x,y,z=float(x),float(y),float(z); q=tuple(float(v) for v in q)/' iktest.py; python3 iktest.py 2>&1 | grep -v XMLPARSER | tail

# openrua op 15
cd /workspace; python3 -c "
import cv2
im=cv2.imread('birdview.png'); c=im[220:350,230:400]; c=cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC); cv2.imwrite('bird_crop.png',c)
im=cv2.imread('agentview.png'); c=im[150:320,130:450]; c=cv2.resize(c,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC); cv2.imwrite('agent_crop.png',c)
"

# openrua op 16
cd /workspace; cat > arm.py <<'EOF'
"""Small helper library: joint state, FK/IK (world frame on this machine), trajectories, gripper."""
import time, numpy as np, rclpy, yaml
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped

ARM = [f"panda_joint{i}" for i in range(1, 8)]
Q_DOWN = (1.0, 0.0, 0.0, 0.0)          # hand z down, fingers along world y
Q_DOWN_X = (0.7071, 0.7071, 0.0, 0.0)  # hand z down, fingers along world x

def R_of(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

class Arm:
    def __init__(self):
        if not rclpy.ok(): rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.traj = ActionClient(self.node, FollowJointTrajectory, "/panda_arm_controller/follow_joint_trajectory")
        self.grip = ActionClient(self.node, GripperCommand, "/franka_gripper/gripper_action")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m):
        self._js = dict(zip(m.name, m.position))
        self._js_stamp = time.time()

    def joints(self, fresh=True):
        t0 = time.time()
        if fresh:
            self._js = {}
        while len(self._js) < 9 and time.time() - t0 < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request(); req.fk_link_names = [link]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z]), (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik(self, xyz, quat=Q_DOWN, seed=None, tcp=False):
        """World-frame hand pose -> joint list, or None. tcp=True: xyz is the fingertip centre."""
        xyz = np.array(xyz, dtype=float)
        if tcp:
            xyz = xyz - 0.1034 * R_of(quat)[:, 2]
        seed = seed if seed is not None else self.arm_q()
        req = GetPositionIK.Request(); r = req.ik_request
        r.group_name = "panda_arm"; r.ik_link_name = "panda_hand"; r.avoid_collisions = False
        r.pose_stamped.header.frame_id = ""
        r.pose_stamped.pose.position.x, r.pose_stamped.pose.position.y, r.pose_stamped.pose.position.z = map(float, xyz)
        (r.pose_stamped.pose.orientation.x, r.pose_stamped.pose.orientation.y,
         r.pose_stamped.pose.orientation.z, r.pose_stamped.pose.orientation.w) = map(float, quat)
        r.robot_state.joint_state.name = ARM
        r.robot_state.joint_state.position = [float(v) for v in seed]
        r.timeout = Duration(sec=2)
        fut = self.ik_cli.call_async(req); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK failed for {np.round(xyz,3)}: {None if res is None else res.error_code.val}")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, waypoints, seconds, verbose=True):
        """waypoints: list of 7-lists (or one), seconds: total (evenly spaced)."""
        if not isinstance(waypoints[0], (list, tuple, np.ndarray)):
            waypoints = [waypoints]
        goal = FollowJointTrajectory.Goal(); goal.trajectory.joint_names = ARM
        n = len(waypoints)
        for i, wp in enumerate(waypoints):
            t = seconds * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.traj.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q = self.arm_q(); err = np.abs(np.array(q) - np.array(waypoints[-1])).max()
        if verbose: print(f"traj done code={code} max joint err={err:.4f}")
        return code, err

    def move_to(self, xyz, quat=Q_DOWN, seconds=3.0, tcp=False, via=None):
        """IK + trajectory. via: optional list of intermediate xyz (same quat)."""
        pts = list(via or []) + [xyz]
        seed = self.arm_q(); wps = []
        for p in pts:
            q = self.ik(p, quat, seed=seed, tcp=tcp)
            if q is None: return None
            wps.append(q); seed = q
        code, err = self.move_joints(wps, seconds)
        pos, _ = self.fk()
        print(f"hand now at {np.round(pos,4)}")
        return pos

    def gripper(self, width):
        goal = GripperCommand.Goal(); goal.command.position = float(width); goal.command.max_effort = 30.0
        fut = self.grip.send_goal_async(goal); rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async(); rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def servo(self, lin, ticks, ang=(0, 0, 0)):
        msg = TwistStamped(); msg.header.frame_id = "panda_link0"
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg); rclpy.spin_once(self.node, timeout_sec=0.05)
EOF
cat > step0.py <<'EOF'
from arm import *
a = Arm()
print("fingers before:", a.fingers())
a.gripper(0.04)
print(a.fk())
EOF
python3 step0.py 2>&1 | grep -v XMLPARSER

# openrua op 17
cd /workspace; cat > step1.py <<'EOF'
from arm import *
a = Arm()
a.move_to((-0.30, -0.25, 0.95), Q_DOWN, seconds=3.0)
print("q:", np.round(a.arm_q(),3))
EOF
python3 step1.py 2>&1 | grep -v XMLPARSER && for c in birdview agentview sideview; do python3 tools/perception/cam_snap.py $c 2>/dev/null; done && python3 heightmap.py 2>/dev/null

# openrua op 18
cd /workspace; cat > step2.py <<'EOF'
from arm import *
a = Arm()
c = np.array([-0.110, -0.176]); r = 0.045
pre = (c[0], c[1] - r, 0.62)
a.move_to(pre, Q_DOWN, seconds=3.0, tcp=True)
print("fk hand:", np.round(a.fk()[0],4))
EOF
python3 step2.py 2>&1 | grep -v XMLPARSER && python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>/dev/null && python3 tools/perception/cam_snap.py agentview 2>/dev/null

# openrua op 19
cd /workspace; cat > step2b.py <<'EOF'
from arm import *
a = Arm()
c = np.array([-0.110, -0.176]); r = 0.045
pre = (c[0], c[1] - r, 0.62)
q = a.ik(pre, Q_DOWN, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 3.0)
    if err < 0.01: break
print("fk hand:", np.round(a.fk()[0],4), "q:", np.round(a.arm_q(),3))
EOF
python3 step2b.py 2>&1 | grep -v XMLPARSER && python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>/dev/null

# openrua op 20
cd /workspace; cat > rim.py <<'EOF'
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
exec(open("pxw.py").read().split("def main")[0])
cam=sys.argv[1]; zlo=float(sys.argv[2]); zhi=float(sys.argv[3])
rclpy.init(); node=rclpy.create_node("rim")
depth=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
T=cam_pose(node,f"{cam}_optical_frame")
print("cam at", np.round(T[:3,3],4))
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=D.shape; vv,uu=np.mgrid[0:H,0:W]
P=np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D,np.ones_like(D)],-1)@T.T
Z=P[...,2]
mask=((Z>zlo)&(Z<zhi)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    pts=P[lab==i][:,:3]
    print(f"blob{i}: area={stats[i,4]} px=({cent[i][0]:.0f},{cent[i][1]:.0f}) x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
    # circle fit (algebraic) on xy
    x,y=pts[:,0],pts[:,1]; A=np.c_[2*x,2*y,np.ones_like(x)]; b=x*x+y*y
    (cx_,cy_,c_),*_=np.linalg.lstsq(A,b,rcond=None); r=np.sqrt(c_+cx_**2+cy_**2)
    print(f"   circle fit: center=({cx_:.4f},{cy_:.4f}) r={r:.4f}")
cv2.imwrite("rim_mask.png",mask*255)
rclpy.shutdown()
EOF
python3 rim.py robot0_eye_in_hand 0.50 0.56 2>/dev/null

# openrua op 21
cd /workspace; python3 rim.py robot0_eye_in_hand 0.542 0.56 2>/dev/null

# openrua op 22
cd /workspace; cat > step3.py <<'EOF'
from arm import *
a = Arm()
g = (-0.1066, -0.2214)
# descend in two steps for accuracy
for z in (0.57, 0.524):
    q = a.ik((g[0], g[1], z), Q_DOWN, tcp=True)
    for i in range(3):
        code, err = a.move_joints(q, 2.0)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4))
f = a.gripper(0.0)
EOF
python3 step3.py 2>&1 | grep -v XMLPARSER && python3 tools/perception/cam_snap.py agentview 2>/dev/null

# openrua op 23
cd /workspace; cat > step4.py <<'EOF'
from arm import *
a = Arm()
g = (-0.1066, -0.2214)
q = a.ik((g[0], g[1], 0.68), Q_DOWN, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 2.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4), "fingers:", a.fingers())
EOF
python3 step4.py 2>&1 | grep -v XMLPARSER && python3 tools/perception/cam_snap.py agentview 2>/dev/null && python3 heightmap.py 2>/dev/null | grep -v blob1

# openrua op 24
cd /workspace; cat > step5.py <<'EOF'
from arm import *
a = Arm()
plate = np.array([0.147, -0.019])
tcp_xy = (plate[0], plate[1] - 0.045)
for z, secs in ((0.68, 3.5), (0.575, 2.0)):
    q = a.ik((tcp_xy[0], tcp_xy[1], z), Q_DOWN, tcp=True)
    if q is None: raise SystemExit("ik fail")
    for i in range(3):
        code, err = a.move_joints(q, secs)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4), "fingers:", np.round(a.fingers(),4))
EOF
python3 step5.py 2>&1 | grep -v XMLPARSER && python3 tools/perception/cam_snap.py agentview 2>/dev/null

# openrua op 25
cd /workspace; cat > step6.py <<'EOF'
from arm import *
a = Arm()
a.gripper(0.04)
pos,_ = a.fk()
q = a.ik((pos[0], pos[1], pos[2] + 0.15), Q_DOWN)
for i in range(3):
    code, err = a.move_joints(q, 2.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4))
EOF
python3 step6.py 2>&1 | grep -v XMLPARSER && python3 tools/perception/cam_snap.py agentview 2>/dev/null && python3 heightmap.py 2>/dev/null | grep -v blob1

# openrua op 26
cd /workspace; cat > step7.py <<'EOF'
from arm import *
a = Arm()
pre = (-0.215, 0.022, 0.60)
q = a.ik(pre, Q_DOWN, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 3.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4))
EOF
python3 step7.py 2>&1 | grep -v XMLPARSER && python3 rim.py robot0_eye_in_hand 0.45 0.48 2>/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>/dev/null && python3 heightmap.py 2>/dev/null | grep -v blob1

# openrua op 27
cd /workspace; cat > step8.py <<'EOF'
from arm import *
a = Arm()
g = (-0.210, 0.025)
for z in (0.50, 0.442):
    q = a.ik((g[0], g[1], z), Q_DOWN, tcp=True)
    for i in range(3):
        code, err = a.move_joints(q, 2.0)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4))
a.gripper(0.0)
EOF
python3 step8.py 2>&1 | grep -v XMLPARSER && python3 tools/perception/cam_snap.py agentview 2>/dev/null

# openrua op 28
cd /workspace; cat > step9.py <<'EOF'
from arm import *
a = Arm()
g = (-0.210, 0.025)
tgt = (0.147, 0.116)
q = a.ik((g[0], g[1], 0.68), Q_DOWN, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 2.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4), "fingers:", np.round(a.fingers(),4))
for z, secs in ((0.68, 3.5), (0.455, 2.0)):
    q = a.ik((tgt[0], tgt[1], z), Q_DOWN, tcp=True)
    if q is None: raise SystemExit("ik fail")
    for i in range(3):
        code, err = a.move_joints(q, secs)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4), "fingers:", np.round(a.fingers(),4))
EOF
python3 step9.py 2>&1 | grep -v XMLPARSER && python3 tools/perception/cam_snap.py agentview 2>/dev/null

# openrua op 29
cd /workspace; cat > step10.py <<'EOF'
from arm import *
a = Arm()
a.gripper(0.04)
pos,_ = a.fk()
q = a.ik((pos[0], pos[1], pos[2] + 0.20), Q_DOWN)
for i in range(3):
    code, err = a.move_joints(q, 2.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4))
EOF
python3 step10.py 2>&1 | grep -v XMLPARSER && for c in agentview sideview birdview frontview; do python3 tools/perception/cam_snap.py $c 2>/dev/null; done; python3 heightmap.py 2>/dev/null | grep -v blob1

# openrua op 30
cd /workspace; cat > step11.py <<'EOF'
from arm import *
a = Arm()
q = a.ik((-0.30, -0.25, 0.95), Q_DOWN)
for i in range(3):
    code, err = a.move_joints(q, 3.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4))
EOF
python3 step11.py 2>&1 | grep -v XMLPARSER && for c in agentview birdview; do python3 tools/perception/cam_snap.py $c 2>/dev/null; done; python3 heightmap.py 2>/dev/null | grep -v blob1; python3 -c "
import cv2
im=cv2.imread('birdview.png'); c=im[280:360,260:400]; c=cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC); cv2.imwrite('bird_crop.png',c)"

# openrua op 31
cd /workspace; python3 -c "
from arm import *
a=Arm(); print(np.round(a.arm_q(),3)); print(np.round(a.fk()[0],4), a.fk()[1])
" 2>&1 | grep -v XMLPARSER

# openrua op 32
cd /workspace; cat > step12.py <<'EOF'
from arm import *
a = Arm()
home = [0.0, -0.161, 0.0, -2.445, 0.0, 2.227, 0.785]
cur = np.array(a.arm_q())
zs = []
for s in np.linspace(0, 1, 15):
    q = cur + s * (np.array(home) - cur)
    p, _ = a.fk(list(q))
    zs.append((round(s,2), np.round(p,3)))
for s, p in zs: print(s, p)
EOF
python3 step12.py 2>&1 | grep -v XMLPARSER

# openrua op 33
cd /workspace; cat > step13.py <<'EOF'
from arm import *
a = Arm()
home = [0.0, -0.161, 0.0, -2.445, 0.0, 2.227, 0.785]
for i in range(4):
    code, err = a.move_joints(home, 6.0)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4), np.round(a.arm_q(),3))
EOF
python3 step13.py 2>&1 | grep -v XMLPARSER && for c in agentview birdview; do python3 tools/perception/cam_snap.py $c 2>/dev/null; done; python3 heightmap.py 2>/dev/null | grep -v blob1

# openrua op 34
cd /workspace; python3 -c "
import cv2
im=cv2.imread('birdview.png'); c=im[285:355,250:400]; c=cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC); cv2.imwrite('bird_crop.png',c)
im=cv2.imread('agentview.png'); c=im[280:470,180:480]; c=cv2.resize(c,None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC); cv2.imwrite('agent_crop.png',c)"

# openrua op 35
cd /workspace; cat > step14.py <<'EOF'
from arm import *
a = Arm()
q = a.ik((0.14, -0.04, 0.72), Q_DOWN_X, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 3.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4), np.round(a.arm_q(),3))
EOF
python3 step14.py 2>&1 | grep -v XMLPARSER && python3 rim.py robot0_eye_in_hand 0.50 0.60 2>/dev/null && python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>/dev/null

# openrua op 36
cd /workspace; cat > rim2.py <<'EOF'
import sys, numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
exec(open("pxw.py").read().split("def main")[0])
cam="robot0_eye_in_hand"
rclpy.init(); node=rclpy.create_node("rim2")
depth=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
D=np.frombuffer(depth.data,dtype=np.float32).reshape(depth.height,depth.width)
T=cam_pose(node,f"{cam}_optical_frame")
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=D.shape; vv,uu=np.mgrid[0:H,0:W]
P=(np.stack([(uu-cx)*D/fx,(vv-cy)*D/fy,D,np.ones_like(D)],-1)@T.T)[...,:3]
sel=(P[...,0]>0.05)&(P[...,0]<0.25)&(P[...,1]>-0.15)&(P[...,1]<0.03)&(P[...,2]>0.47)
pts=P[sel]
print("mug pts:",len(pts),"z range",pts[:,2].min(),pts[:,2].max())
# top-most points (rim highest)
for zlo in (0.56,0.54,0.52,0.50):
    s=pts[pts[:,2]>zlo]
    print(f"z>{zlo}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] mean=({s[:,0].mean():.3f},{s[:,1].mean():.3f})")
# rim extreme points in x among top z>0.52
s=pts[pts[:,2]>0.50]
i=s[:,0].argmax(); j=s[:,0].argmin()
print("x-max rim point:",np.round(s[i],3)," x-min rim point:",np.round(s[j],3))
k=s[:,1].argmax(); l=s[:,1].argmin()
print("y-max point:",np.round(s[k],3)," y-min point:",np.round(s[l],3))
# for x-max side: points with x > xmax-0.01
e=s[s[:,0]>s[:,0].max()-0.012]; print("x-max edge: mean",np.round(e.mean(0),3),"zmax",e[:,2].max())
e=s[s[:,0]<s[:,0].min()+0.012]; print("x-min edge: mean",np.round(e.mean(0),3),"zmax",e[:,2].max())
rclpy.shutdown()
EOF
python3 rim2.py 2>/dev/null

# openrua op 37
cd /workspace; sed -i 's/(P\[...,2\]>0.47)/(P[...,2]>0.47)\&(P[...,2]<0.60)/' rim2.py; python3 rim2.py 2>/dev/null

# openrua op 38
cd /workspace; python3 tools/perception/cam_snap.py frontview 2>/dev/null; python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[170:280,230:400]; c=cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC); cv2.imwrite('front_crop.png',c)"

# openrua op 39
cd /workspace; cat > step15.py <<'EOF'
from arm import *
a = Arm()
g = (0.186, -0.055)
for z, secs in ((0.60, 3.0), (0.532, 2.0)):
    q = a.ik((g[0], g[1], z), Q_DOWN_X, tcp=True)
    if q is None: raise SystemExit("ik fail")
    for i in range(3):
        code, err = a.move_joints(q, secs)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4))
a.gripper(0.0)
EOF
python3 step15.py 2>&1 | grep -v XMLPARSER && python3 tools/perception/cam_snap.py agentview 2>/dev/null

# openrua op 40
cd /workspace; cat > step16.py <<'EOF'
from arm import *
a = Arm()
g = (0.186, -0.055)
q = a.ik((g[0], g[1], 0.66), Q_DOWN_X, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 2.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4), "fingers:", np.round(a.fingers(),4))
EOF
python3 step16.py 2>&1 | grep -v XMLPARSER && python3 tools/perception/cam_snap.py agentview 2>/dev/null && python3 tools/perception/cam_snap.py frontview 2>/dev/null && python3 -c "
import cv2
im=cv2.imread('frontview.png'); c=im[120:280,200:420]; c=cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC); cv2.imwrite('front_crop.png',c)"

# openrua op 41
cd /workspace; cat > step17.py <<'EOF'
from arm import *
a = Arm()
plate = np.array([0.147, -0.019])
tcp = (plate[0] + 0.045, plate[1])
for z, secs in ((0.66, 2.5), (0.545, 2.0)):
    q = a.ik((tcp[0], tcp[1], z), Q_DOWN_X, tcp=True)
    if q is None: raise SystemExit("ik fail")
    for i in range(3):
        code, err = a.move_joints(q, secs)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4), "fingers:", np.round(a.fingers(),4))
a.gripper(0.04)
q = a.ik((tcp[0], tcp[1], 0.75), Q_DOWN_X, tcp=True)
for i in range(3):
    code, err = a.move_joints(q, 2.5)
    if err < 0.01: break
print("hand:", np.round(a.fk()[0],4))
EOF
python3 step17.py 2>&1 | grep -v XMLPARSER && python3 tools/perception/cam_snap.py agentview 2>/dev/null && python3 tools/perception/cam_snap.py frontview 2>/dev/null

# openrua op 42
cd /workspace; cat > step18.py <<'EOF'
from arm import *
a = Arm()
home = [0.0, -0.161, 0.0, -2.445, 0.0, 2.227, 0.785]
cur = np.array(a.arm_q())
ok = True
for s in np.linspace(0, 1, 12):
    q = cur + s * (np.array(home) - cur)
    p, _ = a.fk(list(q)); print(round(s,2), np.round(p,3))
    if p[2] - 0.11 < 0.60: ok = False
print("path clear:", ok)
if ok:
    for i in range(3):
        code, err = a.move_joints(home, 5.0)
        if err < 0.01: break
    print("hand:", np.round(a.fk()[0],4))
EOF
python3 step18.py 2>&1 | grep -v XMLPARSER

# openrua op 43
cd /workspace; for c in agentview birdview frontview; do python3 tools/perception/cam_snap.py $c 2>/dev/null; done; python3 heightmap.py 2>/dev/null | grep -v blob1; python3 -c "
import cv2
im=cv2.imread('birdview.png'); c=im[285:355,250:400]; c=cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC); cv2.imwrite('bird_crop.png',c)"
