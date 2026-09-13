#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/README.md && echo ---- && cat tools/perception/cam_snap.py && echo ---- && cat tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py && echo ---- && cat tools/action/gripper_cmd.py && echo ---- && cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -iv parameter | head -40; echo ---; timeout 20 ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | head -80; timeout 10 ros2 topic echo /tf --once 2>&1 | head -30

# openrua op 6
cat > tf_dump.py <<'EOF'
import rclpy
from tf2_msgs.msg import TFMessage
rclpy.init(); node = rclpy.create_node("tfdump")
got = []
node.create_subscription(TFMessage, "/tf", got.append, 10)
import time
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.3)
seen = {}
for m in got:
    for t in m.transforms:
        tr, q = t.transform.translation, t.transform.rotation
        seen[(t.header.frame_id, t.child_frame_id)] = (round(tr.x,4), round(tr.y,4), round(tr.z,4), round(q.x,4), round(q.y,4), round(q.z,4), round(q.w,4))
for k, v in sorted(seen.items()):
    print(k, v)
EOF
timeout 60 python3 tf_dump.py

# openrua op 7
cat > scene.py <<'EOF'
"""Segment birdview depth into objects above table; print world centroids + heights."""
import numpy as np, rclpy, struct
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2
from cv_bridge import CvBridge

def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]

def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

rclpy.init(); node=rclpy.create_node("scene")
buf=Buffer(); TransformListener(buf,node)
cam="birdview"
depth=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/depth/image_raw",Image),"passthrough").astype(np.float64)
color=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/color/image_raw",Image),"bgr8")
info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
while not buf.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time())
q=t.transform.rotation; R=quat_R(q.x,q.y,q.z,q.w); o=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
H,W=depth.shape
v,u=np.mgrid[0:H,0:W]
pc=np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth],-1)
pw=pc@R.T+o
Z=pw[...,2]
np.save("birdview_world.npy",pw)
# table height: mode of Z in the center region
zc=Z[150:450,150:490]
hist,edges=np.histogram(zc[np.isfinite(zc)],bins=200)
table_z=edges[np.argmax(hist)]
print("table_z ~",table_z)
mask=(Z>table_z+0.01)&np.isfinite(Z)
# restrict to table region (x range) to exclude robot
mask&=(pw[...,0]>-0.45)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    P=pw[m]
    print(f"blob {i}: px centroid=({cent[i][0]:.0f},{cent[i][1]:.0f}) area={stats[i,4]} world x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] ztop={P[:,2].max():.3f} centroid=({P[:,0].mean():.3f},{P[:,1].mean():.3f})")
cv2.imwrite("birdview_mask.png",(mask*255).astype(np.uint8))
EOF
timeout 120 python3 scene.py

# openrua op 8
cat > potprofile.py <<'EOF'
import numpy as np
pw=np.load("birdview_world.npy"); Z=pw[...,2]
table=0.894
for name,(x0,x1,y0,y1) in {"potA":(-0.26,-0.14,-0.30,-0.12),"potB":(-0.12,0.0,0.14,0.32),"knob":(-0.03,0.1,-0.03,0.09),"stove":(0.08,0.30,-0.08,0.15)}.items():
    m=(pw[...,0]>x0)&(pw[...,0]<x1)&(pw[...,1]>y0)&(pw[...,1]<y1)&np.isfinite(Z)
    print(name)
    for lo in np.arange(0.01,0.18,0.01):
        mm=m&(Z>table+lo)
        if mm.sum()==0: continue
        P=pw[mm]
        print(f"  z>{lo:.2f}: n={mm.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] cx={P[:,0].mean():.3f} cy={P[:,1].mean():.3f}")
EOF
python3 potprofile.py

# openrua op 9
cat > sidecloud.py <<'EOF'
import numpy as np, rclpy, sys
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
def grab(node, topic, T):
    got={}
    s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m), 1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
def quat_R(x,y,z,w):
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
rclpy.init(); node=rclpy.create_node("cloud")
buf=Buffer(); TransformListener(buf,node)
for cam in sys.argv[1:]:
    depth=CvBridge().imgmsg_to_cv2(grab(node,f"/{cam}/depth/image_raw",Image),"passthrough").astype(np.float64)
    info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
    while not buf.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
    t=buf.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time())
    q=t.transform.rotation; R=quat_R(q.x,q.y,q.z,q.w); o=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
    H,W=depth.shape; v,u=np.mgrid[0:H,0:W]
    pc=np.stack([(u-cx)*depth/fx,(v-cy)*depth/fy,depth],-1)
    np.save(f"{cam}_world.npy",pc@R.T+o)
    print(cam,"saved")
EOF
timeout 120 python3 sidecloud.py sideview frontview agentview
cat > sideprofile.py <<'EOF'
import numpy as np
table=0.894
for cam in ["sideview","frontview","agentview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    print(cam)
    for name,(x0,x1,y0,y1) in {"potA":(-0.26,-0.14,-0.30,-0.12),"potB":(-0.12,0.0,0.14,0.32)}.items():
        m=(pw[...,0]>x0)&(pw[...,0]<x1)&(pw[...,1]>y0)&(pw[...,1]<y1)&np.isfinite(Z)
        print(" ",name)
        for lo in np.arange(0.0,0.17,0.01):
            mm=m&(Z>table+lo)&(Z<=table+lo+0.01)
            if mm.sum()<3: continue
            P=pw[mm]
            print(f"   z {lo:.2f}-{lo+0.01:.2f}: n={mm.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
EOF
python3 sideprofile.py

# openrua op 10
python3 -c "
import cv2
im=cv2.imread('agentview.png'); c=im[160:290,150:260]; cv2.imwrite('potA_zoom.png',cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('sideview.png'); c=im[270:380,260:350]; cv2.imwrite('potA_side_zoom.png',cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 11
cat > getdesc.py <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("rd")
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,"/robot_description",got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open("robot.urdf","w").write(got[0].data)
print(len(got[0].data))
EOF
timeout 60 python3 getdesc.py && grep -n -A12 'name="panda_hand"\|name="panda_leftfinger"\|name="panda_rightfinger"' robot.urdf | grep -i 'link name\|origin\|box\|mesh\|cylinder' | head -40

# openrua op 12
python3 -c "
import numpy as np,struct
def stl(path):
    d=open(path,'rb').read()
    n=struct.unpack_from('<I',d,80)[0]
    arr=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))
    return arr['v'].reshape(-1,3)
for f in ['hand','finger']:
    v=stl(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl')
    print(f, v.min(0), v.max(0))
v=stl('/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/finger.stl')
for z0 in np.arange(0,0.06,0.005):
    m=(v[:,2]>=z0)&(v[:,2]<z0+0.005)
    if m.sum(): print(f'finger z {z0:.3f}: x[{v[m,0].min():.4f},{v[m,0].max():.4f}] y[{v[m,1].min():.4f},{v[m,1].max():.4f}]')
"


# openrua op 13
find / -iname "*hand*.stl" -o -iname "*finger*.stl" 2>/dev/null | head; find / -iname "*moka*" 2>/dev/null | head

# openrua op 14
ls /opt/ros/jazzy/share/ 2>/dev/null | grep -i panda; ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/ 2>/dev/null; python3 -c "import robosuite" 2>&1; pip list 2>/dev/null | grep -i -E "robosuite|robocasa|mujoco"

# openrua op 15
timeout 30 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; timeout 30 ros2 interface show control_msgs/action/GripperCommand

# openrua op 16
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
"""Reusable robot helpers: joint state, FK, seeded IK, trajectories, gripper.

Build one Robot() and reuse it (clients are expensive to rebuild).
All world-frame poses are converted to panda_link0 for MoveIt.
"""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation (identity rotation)
TCP_OFF = float(M["hand"]["tcp_offset_m"])


def hand_pose_from_tcp(tcp_xyz, R):
    """Hand-frame origin given the TCP point and rotation matrix R (hand->world)."""
    return np.asarray(tcp_xyz) - TCP_OFF * R[:, 2]


def side_grasp_R(yaw_deg, pitch_deg):
    """Hand rotation for a side approach: hand z points horizontally along
    `yaw` (deg, from +x, ccw) and is pitched down by `pitch`; hand y (finger
    axis) stays horizontal."""
    yaw, pitch = math.radians(yaw_deg), math.radians(pitch_deg)
    z = np.array([math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), -math.sin(pitch)])
    y = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


def top_down_R(yaw_deg):
    """Hand z straight down, finger axis (hand y) along world direction yaw."""
    yaw = math.radians(yaw_deg)
    z = np.array([0, 0, -1.0])
    y = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    x = np.cross(y, z)
    return np.column_stack([x, y, z])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("rob_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk.wait_for_service(10); self.ik.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))
        self._js_t = time.time()

    def joints(self, fresh=True):
        if fresh:
            self._js = {}
        while not self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in ARM]

    def finger_gap(self):
        js = self.joints()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    def _call(self, cli, req, timeout=60):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def fk_hand(self, q=None):
        """World-frame (xyz, R) of panda_hand for joint vector q (default current)."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        res = self._call(self.fk, req)
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        R = Rot.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
        return xyz, R

    def tcp(self, q=None):
        xyz, R = self.fk_hand(q)
        return xyz + TCP_OFF * R[:, 2], R

    def solve_ik(self, hand_xyz_world, R, seed=None, attempts=3):
        """Seeded IK for a hand pose in world frame. Returns joint list or None."""
        seed = self.arm_q() if seed is None else seed
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pl = np.asarray(hand_xyz_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, pl)
        qx, qy, qz, qw = Rot.from_matrix(R).as_quat()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, (qx, qy, qz, qw))
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(map(float, seed))
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 1
        for _ in range(attempts):
            res = self._call(self.ik, req)
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def ik_tcp(self, tcp_xyz, R, seed=None):
        return self.solve_ik(hand_pose_from_tcp(tcp_xyz, R), R, seed)

    def move_joints(self, waypoints, seconds, timeout=600):
        """Send a multi-point trajectory (list of joint vectors, list of times)."""
        if not isinstance(seconds, (list, tuple)):
            n = len(waypoints)
            seconds = [seconds * (i + 1) / n for i in range(n)]
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        for q, t in zip(waypoints, seconds):
            pt = JointTrajectoryPoint(positions=list(map(float, q)))
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=timeout)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        code = res.result().result.error_code
        q = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q, waypoints[-1]))
        print(f"  traj error_code={code} max_joint_err={err:.4f}")
        return code, err

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap
OPENRUA_EOF

# openrua op 17
cat > t1.py <<'EOF'
from rob import *
r = Robot()
print("joints", r.joints())
xyz, R = r.fk_hand()
print("hand", xyz.round(4)); print(R.round(3))
t, _ = r.tcp(); print("tcp", t.round(4))
print("open gripper"); r.gripper(GRIP["open_m"])
print(r.joints())
EOF
timeout 300 python3 -u t1.py

# openrua op 18
cat > t2.py <<'EOF'
from rob import *
r = Robot()
q = r.arm_q()
req = GetPositionFK.Request(); req.fk_link_names=["panda_link0","panda_hand"]
req.robot_state.joint_state.name=ARM; req.robot_state.joint_state.position=q
res = r._call(r.fk, req)
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
# IK at the raw returned pose (no offset)
p=res.pose_stamped[1].pose
xyz=np.array([p.position.x,p.position.y,p.position.z]); R=Rot.from_quat([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]).as_matrix()
sol=r.solve_ik(xyz+BASE_IN_WORLD, R, seed=q)   # solve_ik subtracts BASE -> raw
print("IK raw-frame:", None if sol is None else np.round(sol,3))
print("current     :", np.round(q,3))
EOF
timeout 120 python3 -u t2.py

# openrua op 19
sed -i 's/^BASE_IN_WORLD = np.array(\[-0.66, 0.0, 0.912\]).*/BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # MoveIt model frame IS world here (FK header says world; verified)/' rob.py && grep -n BASE_IN_WORLD rob.py | head -2
cat > t3.py <<'EOF'
from rob import *
r = Robot()
q = r.arm_q()
# pre-grasp test pose for pot A: side approach from -x, pitch 12
R = side_grasp_R(0, 12)
tcp = np.array([-0.199-0.008-0.10, -0.200, 0.894+0.04+0.05])
sol = r.ik_tcp(tcp, R, seed=q)
print("sol", None if sol is None else np.round(sol,3))
if sol is not None:
    t, R2 = r.tcp(sol); print("fk tcp of sol", t.round(4), "target", tcp.round(4)); print(R2.round(3)); print(R.round(3))
EOF
timeout 120 python3 -u t3.py

# openrua op 20
python3 - <<'EOF'
import re
s=open("rob.py").read()
s=s.replace('''        qx, qy, qz, qw = Rot.from_matrix(R).as_quat()''','''        # the IK tip link is panda_link8; panda_hand = link8 * Rz(-45deg)
        R8 = np.asarray(R) @ Rot.from_euler("z", 45, degrees=True).as_matrix()
        qx, qy, qz, qw = Rot.from_matrix(R8).as_quat()''')
s=s.replace("All world-frame poses are converted to panda_link0 for MoveIt.","MoveIt's model frame here is `world` (FK header confirms), so poses go in as-is.")
open("rob.py","w").write(s)
EOF
timeout 120 python3 -u t3.py

# openrua op 21
python3 - <<'EOF'
s=open("rob.py").read()
s=s.replace("req.ik_request.avoid_collisions = False","req.ik_request.avoid_collisions = True")
s+='''

    def fk_links(self, q, links=("panda_link3", "panda_link4", "panda_link5", "panda_link6", "panda_link7", "panda_hand")):
        req = GetPositionFK.Request()
        req.fk_link_names = list(links)
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(map(float, q))
        res = self._call(self.fk, req)
        return {n: np.array([ps.pose.position.x, ps.pose.position.y, ps.pose.position.z])
                for n, ps in zip(links, res.pose_stamped)}

    def best_ik_tcp(self, tcp_xyz, R, seeds, min_link_z=0.95, verbose=True):
        """Try several seeds; keep solutions whose arm links stay above min_link_z
        (world) and whose wrist is not flipped; return the one closest to seeds[0]."""
        best, best_cost = None, 1e9
        for sd in seeds:
            sol = self.ik_tcp(tcp_xyz, R, seed=sd)
            if sol is None:
                continue
            links = self.fk_links(sol)
            lowest = min(v[2] for k, v in links.items() if k != "panda_hand")
            if lowest < min_link_z:
                if verbose: print(f"   reject: link z {lowest:.3f}")
                continue
            cost = sum((a - b) ** 2 for a, b in zip(sol, seeds[0])) + 2.0 * abs(sol[4])
            if verbose: print(f"   cand cost={cost:.2f} q={np.round(sol, 2)} lowest_link_z={lowest:.3f}")
            if cost < best_cost:
                best, best_cost = sol, cost
        return best
'''
open("rob.py","w").write(s)
EOF
cat > t4.py <<'EOF'
from rob import *
r = Robot()
q0 = r.arm_q()
R = side_grasp_R(0, 10)
TABLE=0.894
potA=np.array([-0.199,-0.200]); d=0.01
seeds=[q0,[0,-0.785,0,-2.356,0,1.571,0.785],[-0.4,0.3,0,-2.2,0,2.5,0.4],[-0.3,0.6,0.1,-1.8,0,2.4,0.5],[0.0,0.0,0.0,-1.5,0.0,1.5,0.8]]
P1=np.array([potA[0]-d-0.08, potA[1], TABLE+0.15])
sol=r.best_ik_tcp(P1,R,seeds)
print("P1 best", np.round(sol,3))
P2=np.array([potA[0]-d-0.08, potA[1], TABLE+0.04])
sol2=r.best_ik_tcp(P2,R,[sol]+seeds)
print("P2 best", np.round(sol2,3))
P3=np.array([potA[0]-d, potA[1], TABLE+0.04])
sol3=r.best_ik_tcp(P3,R,[sol2]+seeds)
print("P3 best", np.round(sol3,3))
EOF
timeout 300 python3 -u t4.py

# openrua op 22
cat > t5.py <<'EOF'
from rob import *
r = Robot()
for name,q in [("P1",[-0.609,0.191,-0.201,-2.851,2.317,1.789,0.889]),("P3",[-0.871,0.675,0.169,-2.502,2.425,1.794,0.686])]:
    L=r.fk_links(q,("panda_link1","panda_link2","panda_link3","panda_link4","panda_link5","panda_link6","panda_link7","panda_hand"))
    print(name); [print("  ",k,v.round(3)) for k,v in L.items()]
    t,R=r.tcp(q); print("   tcp",t.round(3))
EOF
timeout 120 python3 -u t5.py

# openrua op 23
mkdir -p "$(dirname /workspace/pick_approach.py)"
cat > /workspace/pick_approach.py <<'OPENRUA_EOF'
"""Approach a pot from -x (or given yaw) with a pitched side grasp, stop with the
pads around the lower chamber, gripper still open. Saves state to a json.

Usage: python3 pick_approach.py <pot_x> <pot_y> [yaw_deg]
"""
import json
import sys

from rob import *

TABLE = 0.894
PITCH = 10.0
D = 0.010          # pads stop this far short of the pot centre (along approach)
GRASP_Z = 0.040    # TCP height above table at grasp
BACK = 0.08        # stand-off behind the pot before sliding in


def cart_line(r, R, a, b, step, seed):
    """IK waypoints along the straight TCP segment a->b (inclusive of b)."""
    a, b = np.asarray(a), np.asarray(b)
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
    out = []
    for i in range(1, n + 1):
        p = a + (b - a) * i / n
        sol = r.ik_tcp(p, R, seed=seed)
        if sol is None:
            raise SystemExit(f"IK failed at {p}")
        out.append(sol)
        seed = sol
    return out


def main():
    px, py = float(sys.argv[1]), float(sys.argv[2])
    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    r = Robot()
    R = side_grasp_R(yaw, PITCH)
    ap = np.array([math.cos(math.radians(yaw)), math.sin(math.radians(yaw)), 0.0])  # approach dir
    pot = np.array([px, py, 0.0])
    P3 = pot - D * ap + np.array([0, 0, TABLE + GRASP_Z])
    P2 = P3 - BACK * ap
    P1 = P2 + np.array([0, 0, 0.11])
    P0 = P1 + np.array([0, 0, 0.10])
    print("P0", P0.round(3), "P1", P1.round(3), "P2", P2.round(3), "P3", P3.round(3))

    q0 = r.arm_q()
    seeds = [q0, [0, -0.785, 0, -2.356, 0, 1.571, 0.785], [-0.4, 0.3, 0, -2.2, 0, 2.5, 0.4],
             [-0.9, 0.65, 0.0, -2.65, 2.3, 1.65, 0.65]]
    s0 = r.best_ik_tcp(P0, R, seeds)
    if s0 is None:
        raise SystemExit("no IK for P0")
    print("-> P0"); r.move_joints([s0], 4.0)
    s1 = r.ik_tcp(P1, R, seed=s0); print("-> P1"); r.move_joints([s1], 2.5)
    print("-> P2"); r.move_joints(cart_line(r, R, P1, P2, 0.03, s1), 3.0)
    print("-> P3 (slide in)"); r.move_joints(cart_line(r, R, P2, P3, 0.02, r.arm_q()), 3.0)
    t, _ = r.tcp()
    print("TCP now", t.round(4), "target", P3.round(4))
    json.dump({"pot": [px, py], "yaw": yaw, "P3": P3.tolist(), "P1": P1.tolist(), "q": r.arm_q()},
              open("approach_state.json", "w"))


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 24
timeout 900 python3 -u pick_approach.py -0.199 -0.200 0 2>&1 | tail -30

# openrua op 25
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent1.png; timeout 60 python3 tools/perception/cam_snap.py sideview side1.png

# openrua op 26
cat > converge.py <<'EOF'
"""Re-send the TCP target from approach_state.json until the arm converges."""
import json, sys
from rob import *
st=json.load(open("approach_state.json"))
r=Robot(); R=side_grasp_R(st["yaw"],10.0)
target=np.array(st["P3"])
for i in range(4):
    sol=r.ik_tcp(target,R,seed=r.arm_q())
    code,err=r.move_joints([sol],2.0)
    t,_=r.tcp(); print("TCP",t.round(4),"err",np.round(t-target,4))
    if err<0.01: break
EOF
timeout 600 python3 -u converge.py

# openrua op 27
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png; timeout 60 python3 tools/perception/cam_snap.py frontview front2.png

# openrua op 28
cat > grasp.py <<'EOF'
from rob import *
r=Robot()
print("gap before", round(r.finger_gap(),4))
gap=r.gripper(GRIP["closed_m"])
print("gap after close", round(gap,4))
EOF
timeout 300 python3 -u grasp.py

# openrua op 29
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 30
timeout 120 python3 sidecloud.py birdview sideview && python3 - <<'EOF'
import numpy as np
table=0.894
for cam in ["birdview","sideview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    m=(pw[...,0]>-0.30)&(pw[...,0]<-0.10)&(pw[...,1]>-0.30)&(pw[...,1]<-0.12)&np.isfinite(Z)
    print(cam)
    for lo in np.arange(0.0,0.20,0.01):
        mm=m&(Z>table+lo)&(Z<=table+lo+0.01)
        if mm.sum()<3: continue
        P=pw[mm]
        print(f"   z {lo:.2f}-{lo+0.01:.2f}: n={mm.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
EOF

# openrua op 31
python3 - <<'EOF'
s=open("rob.py").read()
s+='''
    def move_converged(self, waypoints, seconds, tol=0.01, retries=3):
        """Send the trajectory, then re-send the final point until converged."""
        code, err = self.move_joints(waypoints, seconds)
        for _ in range(retries):
            if err < tol:
                break
            print("   re-sending final point to converge")
            code, err = self.move_joints([waypoints[-1]], 2.0)
        return code, err
'''
open("rob.py","w").write(s)
p=open("pick_approach.py").read()
p=p.replace('print("-> P0"); r.move_joints([s0], 4.0)','print("-> P0"); r.move_converged([s0], 4.0)')
p=p.replace('print("-> P1"); r.move_joints([s1], 2.5)','print("-> P1"); r.move_converged([s1], 2.5)')
p=p.replace('print("-> P2"); r.move_joints(cart_line(r, R, P1, P2, 0.03, s1), 3.0)','print("-> P2"); r.move_converged(cart_line(r, R, P1, P2, 0.03, s1), 3.0)')
p=p.replace('print("-> P3 (slide in)"); r.move_joints(cart_line(r, R, P2, P3, 0.02, r.arm_q()), 3.0)','print("-> P3 (slide in)"); r.move_converged(cart_line(r, R, P2, P3, 0.02, r.arm_q()), 5.0)')
open("pick_approach.py","w").write(p)
EOF
grep -n "move_converged" pick_approach.py
cat > backoff.py <<'EOF'
"""Open gripper and retreat along -approach by 6 cm, then up 5 cm."""
import json, sys
from rob import *
st=json.load(open("approach_state.json")); yaw=st["yaw"]
r=Robot(); R=side_grasp_R(yaw,10.0)
r.gripper(GRIP["open_m"])
ap=np.array([math.cos(math.radians(yaw)),math.sin(math.radians(yaw)),0.0])
t,_=r.tcp()
for tgt in [t-0.06*ap, t-0.06*ap+np.array([0,0,0.05])]:
    sol=r.ik_tcp(tgt,R,seed=r.arm_q()); r.move_converged([sol],2.5)
print("TCP",r.tcp()[0].round(4))
EOF
timeout 600 python3 -u backoff.py

# openrua op 32
cat > potfind.py <<'EOF'
"""Locate a pot: knob (z>0.15 above table) centroid + lower-chamber centre, from given cams.
Usage: python3 potfind.py x0 x1 y0 y1 [cams...]"""
import sys, subprocess, numpy as np
x0,x1,y0,y1=map(float,sys.argv[1:5]); cams=sys.argv[5:] or ["birdview","sideview","frontview","agentview"]
subprocess.run(["python3","sidecloud.py",*cams],check=True,timeout=300)
table=0.894
for cam in cams:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    m=(pw[...,0]>x0)&(pw[...,0]<x1)&(pw[...,1]>y0)&(pw[...,1]<y1)&np.isfinite(Z)
    k=m&(Z>table+0.15)&(Z<table+0.175)
    lo=m&(Z>table+0.015)&(Z<table+0.05)
    P=pw[k]; L=pw[lo]
    if len(P): print(f"{cam}: knob n={len(P)} centre=({P[:,0].mean():.4f},{P[:,1].mean():.4f}) top z={P[:,2].max():.3f}")
    if len(L): print(f"{cam}: lower n={len(L)} x[{L[:,0].min():.3f},{L[:,0].max():.3f}] y[{L[:,1].min():.3f},{L[:,1].max():.3f}]")
EOF
timeout 400 python3 potfind.py -0.26 -0.10 -0.30 -0.14

# openrua op 33
timeout 900 python3 -u pick_approach.py -0.168 -0.216 0 2>&1 | grep -v cand

# openrua op 34
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png && timeout 300 python3 potfind.py -0.26 -0.10 -0.30 -0.14 sideview frontview

# openrua op 35
timeout 300 python3 -u grasp.py

# openrua op 36
cat > lift.py <<'EOF'
import json, sys
from rob import *
st=json.load(open("approach_state.json")); yaw=st["yaw"]
dz=float(sys.argv[1]) if len(sys.argv)>1 else 0.15
r=Robot(); R=side_grasp_R(yaw,10.0)
t,_=r.tcp()
wps=[]; seed=r.arm_q()
n=max(1,int(dz/0.03))
for i in range(1,n+1):
    tgt=t+np.array([0,0,dz*i/n]); sol=r.ik_tcp(tgt,R,seed=seed); wps.append(sol); seed=sol
r.move_converged(wps,4.0)
print("TCP",r.tcp()[0].round(4),"gap",round(r.finger_gap(),4))
EOF
timeout 600 python3 -u lift.py 0.15 && timeout 300 python3 potfind.py -0.26 -0.10 -0.30 -0.14 sideview frontview && timeout 60 python3 tools/perception/cam_snap.py sideview side5.png

# openrua op 37
python3 - <<'EOF'
import numpy as np
for cam in ["sideview","frontview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    m=(pw[...,0]>-0.20)&(pw[...,0]<-0.10)&(pw[...,1]>-0.27)&(pw[...,1]<-0.16)&np.isfinite(Z)
    for lo in np.arange(0.90,1.25,0.02):
        mm=m&(Z>lo)&(Z<=lo+0.02)
        if mm.sum()<3: continue
        P=pw[mm]; print(f"{cam} z {lo:.2f}: n={mm.sum():4d} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}]")
EOF

# openrua op 38
mkdir -p "$(dirname /workspace/place.py)"
cat > /workspace/place.py <<'OPENRUA_EOF'
"""Carry the held pot to a target and set it down.

Usage: python3 place.py <pot_target_x> <pot_target_y> <carry_z> <place_tcp_z> [yaw_deg] [fwd_off] [side_off]
pot_target: where the pot centre should end up (world xy).
carry_z:    TCP height during transport.  place_tcp_z: TCP height at release.
fwd_off:    pot centre offset ahead of TCP along approach dir (measured, m).
side_off:   pot centre offset along hand +y from the TCP (m).
"""
import json
import sys

from rob import *

PITCH = 10.0


def cart_line(r, R, a, b, step, seed):
    a, b = np.asarray(a), np.asarray(b)
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
    out = []
    for i in range(1, n + 1):
        p = a + (b - a) * i / n
        sol = r.ik_tcp(p, R, seed=seed)
        if sol is None:
            raise SystemExit(f"IK failed at {p}")
        out.append(sol); seed = sol
    return out


def main():
    tx, ty, carry_z, place_z = map(float, sys.argv[1:5])
    yaw = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0
    fwd = float(sys.argv[6]) if len(sys.argv) > 6 else 0.02
    side = float(sys.argv[7]) if len(sys.argv) > 7 else 0.0
    r = Robot()
    R = side_grasp_R(yaw, PITCH)
    ap = R[:, 2].copy(); ap[2] = 0; ap /= np.linalg.norm(ap)
    sd = R[:, 1]
    tcp_xy = np.array([tx, ty, 0.0]) - fwd * ap - side * sd
    t0, _ = r.tcp()
    print("start TCP", t0.round(3), "gap", round(r.finger_gap(), 4))
    A = np.array([t0[0], t0[1], carry_z])
    B = np.array([tcp_xy[0], tcp_xy[1], carry_z])
    C = np.array([tcp_xy[0], tcp_xy[1], place_z])
    print("A", A.round(3), "B", B.round(3), "C", C.round(3))
    seed = r.arm_q()
    # sanity: IK reachable at C with good posture before moving
    sC = r.best_ik_tcp(C, R, [seed, [-0.2, 0.9, 0.0, -1.9, 2.3, 1.9, 0.6], [0, 0.6, 0, -2.0, 0, 2.6, 0.8]])
    if sC is None:
        raise SystemExit("place pose unreachable; nothing moved")
    print("-> up"); r.move_converged(cart_line(r, R, t0, A, 0.03, seed), 2.5)
    print("-> over target"); r.move_converged(cart_line(r, R, A, B, 0.04, r.arm_q()), 5.0)
    print("-> down"); r.move_converged(cart_line(r, R, B, C, 0.03, r.arm_q()), 4.0)
    t, _ = r.tcp(); print("TCP at release", t.round(4), "gap", round(r.finger_gap(), 4))
    json.dump({"yaw": yaw, "C": C.tolist(), "target": [tx, ty]}, open("place_state.json", "w"))


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 39
timeout 900 python3 -u place.py 0.23 -0.012 1.10 0.972 0 0.02 0.007 2>&1 | grep -v "cand\|reject"

# openrua op 40
python3 - <<'EOF'
s=open("rob.py").read()
s+='''

SEEDS = [[0, -0.785, 0, -2.356, 0, 1.571, 0.785], [-0.9, 0.65, 0.0, -2.65, 2.3, 1.65, 0.65],
         [-0.2, 0.9, 0.0, -1.9, 2.3, 1.9, 0.6], [0, 0.6, 0, -2.0, 0, 2.6, 0.8],
         [0.3, 0.8, -0.3, -2.0, 2.4, 1.8, 0.9], [-0.5, 0.4, 0.3, -2.4, 2.2, 1.7, 0.5]]


def cart_line(r, R, a, b, step, seed, max_jump=1.2):
    """IK waypoints along the straight TCP segment a->b (inclusive of b),
    keeping joint-space continuity (falls back to other seeds if needed)."""
    a, b = np.asarray(a), np.asarray(b)
    n = max(1, int(np.ceil(np.linalg.norm(b - a) / step)))
    out = []
    for i in range(1, n + 1):
        p = a + (b - a) * i / n
        sol = r.ik_tcp(p, R, seed=seed)
        if sol is None or max(abs(x - y) for x, y in zip(sol, seed)) > max_jump:
            cands = []
            for sd in [seed] + SEEDS:
                s2 = r.ik_tcp(p, R, seed=sd)
                if s2 is not None:
                    cands.append((max(abs(x - y) for x, y in zip(s2, seed)), s2))
            if not cands:
                raise SystemExit(f"IK failed at {p}")
            jump, sol = min(cands, key=lambda c: c[0])
            print(f"   (re-seeded IK at {p.round(3)}, jump {jump:.2f})")
        out.append(sol); seed = sol
    return out
'''
open("rob.py","w").write(s)
for f in ["place.py","pick_approach.py"]:
    p=open(f).read()
    import re
    # remove local cart_line definitions
    p=re.sub(r'\ndef cart_line\(.*?\n    return out\n', '\n', p, flags=re.S)
    open(f,"w").write(p)
EOF
grep -n "def cart_line" rob.py place.py pick_approach.py
timeout 900 python3 -u place.py 0.23 -0.012 1.10 0.972 0 0.02 0.007 2>&1 | grep -v "cand\|reject"

# openrua op 41
cat > ikdbg.py <<'EOF'
from rob import *
r=Robot(); R=side_grasp_R(0,10)
print("gap",round(r.finger_gap(),4),"tcp",r.tcp()[0].round(3))
for p in [[0.068,-0.091,1.10],[0.0,-0.13,1.10],[0.13,-0.05,1.10],[0.21,-0.019,1.10],[0.21,-0.019,0.972]]:
    for ac in [True, False]:
        req = GetPositionIK.Request(); req.ik_request.group_name="panda_arm"; req.ik_request.pose_stamped.header.frame_id=""
        h=hand_pose_from_tcp(p,R); R8=R@Rot.from_euler("z",45,degrees=True).as_matrix(); q=Rot.from_matrix(R8).as_quat()
        pp=req.ik_request.pose_stamped.pose; pp.position.x,pp.position.y,pp.position.z=map(float,h); pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=map(float,q)
        req.ik_request.robot_state.joint_state.name=ARM; req.ik_request.robot_state.joint_state.position=r.arm_q(); req.ik_request.avoid_collisions=ac; req.ik_request.timeout.sec=2
        res=r._call(r.ik,req); print(p,"avoid",ac,"code",res.error_code.val, None if res.error_code.val!=1 else np.round([res.solution.joint_state.position[res.solution.joint_state.name.index(j)] for j in ARM],2))
EOF
timeout 300 python3 -u ikdbg.py

# openrua op 42
timeout 60 python3 tools/perception/cam_snap.py sideview side6.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent6.png; python3 -c "
import cv2; im=cv2.imread('side6.png'); c=im[120:280,280:440]; cv2.imwrite('side6z.png',cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 43
cat > reach.py <<'EOF'
from rob import *
r=Robot(); 
q=r.arm_q()
for z in [1.02,1.06,1.10]:
  for yaw in [0]:
    R=side_grasp_R(yaw,10)
    row=[]
    for x in [-0.1,0.0,0.07,0.13,0.21]:
        y=-0.217+(x+0.18)*(0.198/0.39)
        ok=any(r.ik_tcp([x,y,z],R,seed=s) is not None for s in [q]+SEEDS)
        row.append(f"{x:+.2f}:{'ok' if ok else '--'}")
    print(f"z={z} yaw={yaw}", " ".join(row))
EOF
timeout 600 python3 -u reach.py

# openrua op 44
timeout 120 python3 sidecloud.py sideview frontview && python3 - <<'EOF'
import numpy as np
for cam in ["sideview","frontview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    m=(pw[...,0]>-0.20)&(pw[...,0]<-0.10)&(pw[...,1]>-0.27)&(pw[...,1]<-0.16)&np.isfinite(Z)&(Z>0.95)
    P=pw[m]; print(cam,"lowest pot z",np.sort(P[:,2])[:5].round(3),"knob z",P[:,2].max().round(3))
    for lo in np.arange(1.02,1.26,0.02):
        mm=m&(Z>lo)&(Z<=lo+0.02)
        if mm.sum()<3: continue
        Q=pw[mm]; print(f"   z {lo:.2f}: n={mm.sum():4d} x[{Q[:,0].min():.3f},{Q[:,0].max():.3f}] y[{Q[:,1].min():.3f},{Q[:,1].max():.3f}]")
EOF

# openrua op 45
cat > setdown.py <<'EOF'
"""Lower the held pot straight down to TCP z given, open, back off, rise."""
import sys
from rob import *
z=float(sys.argv[1]); yaw=float(sys.argv[2]) if len(sys.argv)>2 else 0.0
pitch=float(sys.argv[3]) if len(sys.argv)>3 else 10.0
r=Robot(); R=side_grasp_R(yaw,pitch)
t,_=r.tcp(); print("TCP",t.round(3),"gap",round(r.finger_gap(),4))
tgt=np.array([t[0],t[1],z])
r.move_converged(cart_line(r,R,t,tgt,0.03,r.arm_q()),3.0)
print("TCP",r.tcp()[0].round(3),"gap",round(r.finger_gap(),4))
r.gripper(GRIP["open_m"])
ap=R[:,2].copy(); ap[2]=0; ap/=np.linalg.norm(ap)
t,_=r.tcp()
r.move_converged(cart_line(r,R,t,t-0.07*ap,0.03,r.arm_q()),2.5)
t,_=r.tcp()
r.move_converged(cart_line(r,R,t,t+np.array([0,0,0.08]),0.03,r.arm_q()),2.5)
print("TCP",r.tcp()[0].round(3))
EOF
timeout 600 python3 -u setdown.py 0.932 0 10 2>&1 | grep -v "re-seeded"

# openrua op 46
timeout 300 python3 potfind.py -0.26 -0.05 -0.30 -0.14 birdview sideview frontview; timeout 60 python3 tools/perception/cam_snap.py agentview agent7.png

# openrua op 47
python3 - <<'EOF'
p=open("pick_approach.py").read()
p=p.replace('''    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    r = Robot()
    R = side_grasp_R(yaw, PITCH)''','''    yaw = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
    pitch = float(sys.argv[4]) if len(sys.argv) > 4 else PITCH
    d = float(sys.argv[5]) if len(sys.argv) > 5 else D
    grasp_z = float(sys.argv[6]) if len(sys.argv) > 6 else GRASP_Z
    r = Robot()
    R = side_grasp_R(yaw, pitch)''')
p=p.replace("P3 = pot - D * ap + np.array([0, 0, TABLE + GRASP_Z])","P3 = pot - d * ap + np.array([0, 0, TABLE + grasp_z])")
p=p.replace('json.dump({"pot": [px, py], "yaw": yaw,','json.dump({"pot": [px, py], "yaw": yaw, "pitch": pitch, "d": d, "grasp_z": grasp_z,')
p=p.replace('Usage: python3 pick_approach.py <pot_x> <pot_y> [yaw_deg]','Usage: python3 pick_approach.py <pot_x> <pot_y> [yaw_deg] [pitch_deg] [d] [grasp_z]')
open("pick_approach.py","w").write(p)
for f in ["converge.py","backoff.py","lift.py"]:
    s=open(f).read().replace('side_grasp_R(st["yaw"],10.0)','side_grasp_R(st["yaw"],st.get("pitch",10.0))').replace('side_grasp_R(yaw,10.0)','side_grasp_R(yaw,st.get("pitch",10.0))')
    open(f,"w").write(s)
EOF
grep -n "side_grasp_R" converge.py backoff.py lift.py
timeout 900 python3 -u pick_approach.py -0.154 -0.213 0 0 0.0 0.045 2>&1 | grep -v "cand\|reject"

# openrua op 48
python3 - <<'EOF' 2>&1 | grep -v "cand\|reject"
import json, numpy as np
from rob import *
r=Robot()
st=json.load(open("approach_state.json"))
q=r.arm_q(); print("q now", np.round(q,3))
lim=M["actuators"][0]["limits_rad"]
for i,(v,l) in enumerate(zip(q,lim)): print(i+1, round(v,3), l, "NEAR LIMIT" if min(abs(v-l[0]),abs(v-l[1]))<0.05 else "")
R=side_grasp_R(0,0)
P3=np.array(st["P3"])
s=r.ik_tcp(P3,R,seed=q); print("ik P3", np.round(s,3) if s else None)
if s: print("diff", np.round(np.array(s)-np.array(q),3))
links=r.fk_links(q, ("panda_link5","panda_link6","panda_link7","panda_hand","panda_leftfinger","panda_rightfinger"))
for k,v in links.items(): print(k, v.round(3))
EOF

# openrua op 49
ls tools/perception/; ros2 topic list | grep color/image_raw

# openrua op 50
head -20 tools/perception/cam_snap.py | grep -i usage -A3; python3 tools/perception/cam_snap.py agentview agent8.png && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih8.png && python3 tools/perception/cam_snap.py sideview side8.png

# openrua op 51
timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench | head -30

# openrua op 52
timeout 900 python3 -u pick_approach.py -0.154 -0.213 0 10 0.005 0.04 2>&1 | grep -v "cand\|reject"

# openrua op 53
python3 tools/perception/cam_snap.py robot0_eye_in_hand eih9.png >/dev/null && python3 tools/perception/cam_snap.py sideview side9.png >/dev/null; echo ok

# openrua op 54
cat grasp.py; timeout 300 python3 -u grasp.py 2>&1 | tail -3

# openrua op 55
cat lift.py; timeout 600 python3 -u lift.py 0.10 2>&1 | tail -5

# openrua op 56
cat sidecloud.py | head -30; cat potfind.py | head -40

# openrua op 57
cat > heldpot.py <<'EOF'
"""Measure the held pot relative to the TCP: knob centre, base z, chamber extents.
Usage: python3 heldpot.py [cams...]"""
import sys, subprocess, numpy as np
from rob import *
cams=sys.argv[1:] or ["sideview","frontview","birdview"]
subprocess.run(["python3","sidecloud.py",*cams],check=True,timeout=300)
r=Robot(); t,R=r.tcp(); print("TCP",t.round(4),"gap",round(r.finger_gap(),4))
ap=R[:,2].copy(); ap[2]=0; ap/=np.linalg.norm(ap); sd=R[:,1]
for cam in cams:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    # box around the TCP, excluding the hand region behind the TCP
    rel=pw-t; f=rel@ap; s=rel@sd
    m=np.isfinite(Z)&(f>-0.02)&(f<0.12)&(np.abs(s)<0.09)&(Z>t[2]-0.12)&(Z<t[2]+0.20)
    P=pw[m]
    if not len(P): print(cam,"nothing"); continue
    zt=P[:,2].max(); knob=P[P[:,2]>zt-0.02]
    fk=(knob-t)@ap; sk=(knob-t)@sd
    print(f"{cam}: n={len(P)} top z={zt:.3f} base z={P[:,2].min():.3f} knob fwd={fk.mean():.4f} side={sk.mean():.4f}")
    # lower chamber slab at TCP height
    lc=P[np.abs(P[:,2]-t[2])<0.01]; fl=(lc-t)@ap; sl=(lc-t)@sd
    if len(lc): print(f"   at TCP z: fwd[{fl.min():.3f},{fl.max():.3f}] side[{sl.min():.3f},{sl.max():.3f}] n={len(lc)}")
EOF
timeout 600 python3 heldpot.py 2>&1 | grep -v saved

# openrua op 58
cat > carry.py <<'EOF'
"""Move the TCP along straight world segments, keeping orientation from approach_state.
Usage: python3 carry.py dx dy dz [seconds]  (relative TCP move)"""
import json, sys
from rob import *
st=json.load(open("approach_state.json"))
dx,dy,dz=map(float,sys.argv[1:4]); secs=float(sys.argv[4]) if len(sys.argv)>4 else 4.0
r=Robot(); R=side_grasp_R(st["yaw"],st.get("pitch",10.0))
t,_=r.tcp(); g0=r.finger_gap()
tgt=t+np.array([dx,dy,dz])
r.move_converged(cart_line(r,R,t,tgt,0.03,r.arm_q()),secs)
t2,_=r.tcp(); print("TCP",t2.round(4),"target",tgt.round(4),"gap",round(g0,4),"->",round(r.finger_gap(),4))
EOF
timeout 600 python3 -u carry.py 0.10 0 0.02 4 2>&1 | grep -v "^ *(re"

# openrua op 59
timeout 600 python3 heldpot.py sideview frontview 2>&1 | grep -v saved

# openrua op 60
cat setdown.py | head -20; timeout 600 python3 -u setdown.py 0.930 0 10 2>&1 | grep -v "^ *(re" | tail -8

# openrua op 61
timeout 600 python3 potfind.py -0.15 0.05 -0.30 -0.12 birdview sideview frontview 2>&1 | grep -v saved; python3 tools/perception/cam_snap.py agentview agent10.png >/dev/null

# openrua op 62
python3 - <<'EOF'
import numpy as np
for cam in ["sideview","frontview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    m=np.isfinite(Z)&(pw[...,0]>-0.12)&(pw[...,0]<0.05)&(pw[...,1]>-0.32)&(pw[...,1]<-0.12)&(Z>0.896)&(Z<1.07)
    P=pw[m]; print(cam, len(P))
    for z0 in np.arange(0.0,0.17,0.01):
        s=P[(P[:,2]>0.894+z0)&(P[:,2]<0.894+z0+0.01)]
        if len(s): print(f"  h {z0:.2f}-{z0+0.01:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 63
timeout 900 python3 -u pick_approach.py -0.033 -0.212 0 5 0.005 0.068 2>&1 | grep -v "cand\|reject" && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih11.png >/dev/null && python3 tools/perception/cam_snap.py sideview side11.png >/dev/null

# openrua op 64
timeout 300 python3 -u grasp.py 2>&1 | tail -2; timeout 600 python3 -u lift.py 0.06 2>&1 | tail -2

# openrua op 65
timeout 600 python3 heldpot.py sideview frontview 2>&1 | grep -v saved; python3 tools/perception/cam_snap.py sideview side12.png >/dev/null

# openrua op 66
python3 - <<'EOF'
import numpy as np
t=np.array([-0.0397,-0.2141,1.0212])
for cam in ["sideview","frontview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    m=np.isfinite(Z)&(np.abs(pw[...,0]-t[0]+0.01)<0.08)&(np.abs(pw[...,1]-t[1])<0.08)&(Z>0.90)&(Z<1.13)
    P=pw[m]
    for z0 in np.arange(0.90,1.13,0.01):
        s=P[(P[:,2]>z0)&(P[:,2]<z0+0.01)]
        if len(s): print(f"{cam} z {z0:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 67
timeout 600 python3 -u carry.py 0.13 0.10 0 5 2>&1 | grep -v "^ *(re" | tail -2

# openrua op 68
timeout 600 python3 heldpot.py frontview sideview 2>&1 | grep -v saved

# openrua op 69
timeout 600 python3 -u carry.py 0.131 0.100 0 6 2>&1 | grep -v "^ *(re" | tail -2

# openrua op 70
timeout 600 python3 heldpot.py frontview 2>&1 | grep -v saved; python3 - <<'EOF'
import numpy as np
pw=np.load("frontview_world.npy"); Z=pw[...,2]
m=np.isfinite(Z)&(pw[...,0]>0.12)&(pw[...,0]<0.30)&(np.abs(pw[...,1]+0.01)<0.08)&(Z>0.935)&(Z<1.0)
P=pw[m]
for z0 in np.arange(0.935,1.0,0.005):
    s=P[(P[:,2]>z0)&(P[:,2]<z0+0.005)]
    if len(s): print(f"z {z0:.3f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
EOF

# openrua op 71
timeout 600 python3 -u carry.py -0.015 0 -0.021 4 2>&1 | grep -v "^ *(re" | tail -1; timeout 20 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench | grep -A3 force

# openrua op 72
timeout 600 python3 -u setdown.py 0.9944 0 5 2>&1 | grep -v "^ *(re" | tail -5; timeout 600 python3 potfind.py 0.09 0.30 -0.10 0.13 birdview frontview sideview 2>&1 | grep -v saved; python3 tools/perception/cam_snap.py agentview agent13.png >/dev/null

# openrua op 73
python3 - <<'EOF'
import numpy as np
for cam in ["frontview","birdview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    m=np.isfinite(Z)&(pw[...,0]>0.09)&(pw[...,0]<0.30)&(pw[...,1]>-0.10)&(pw[...,1]<0.13)&(Z>0.90)&(Z<1.10)
    P=pw[m]; print(cam)
    for z0 in np.arange(0.90,1.10,0.01):
        s=P[(P[:,2]>z0)&(P[:,2]<z0+0.01)]
        if len(s): print(f"  z {z0:.2f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
# plate surface height away from pot
pw=np.load("birdview_world.npy"); Z=pw[...,2]
m=np.isfinite(Z)&(pw[...,0]>0.10)&(pw[...,0]<0.16)&(pw[...,1]>0.05)&(pw[...,1]<0.12)
print("plate z (free area) median", np.median(Z[m]).round(4), "min", Z[m].min().round(4), "max", Z[m].max().round(4))
EOF

# openrua op 74
python3 - <<'EOF'
import numpy as np
for cam in ["frontview","birdview","sideview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    m=np.isfinite(Z)&(pw[...,0]>0.19)&(pw[...,0]<0.30)&(pw[...,1]>-0.06)&(pw[...,1]<0.05)&(Z>1.05)
    P=pw[m]
    print(cam, "top z", P[:,2].max().round(4), "knob xy", P[P[:,2]>P[:,2].max()-0.01][:,:2].mean(0).round(4))
EOF

# openrua op 75
timeout 600 python3 potfind.py -0.15 0.05 0.15 0.35 birdview frontview sideview 2>&1 | grep -v saved; python3 - <<'EOF'
import numpy as np
for cam in ["birdview","frontview","sideview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    m=np.isfinite(Z)&(pw[...,0]>-0.15)&(pw[...,0]<0.05)&(pw[...,1]>0.15)&(pw[...,1]<0.35)&(Z>0.90)&(Z<1.10)
    P=pw[m]
    for z0 in [0.91,0.93,0.95,0.96,0.97,0.98,1.03]:
        s=P[(P[:,2]>z0)&(P[:,2]<z0+0.01)]
        if len(s): print(f"{cam} h {z0-0.894:.3f} n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
    print(cam,"top",P[:,2].max().round(4))
EOF

# openrua op 76
timeout 900 python3 -u pick_approach.py -0.056 0.241 -45 5 0.005 0.068 2>&1 | grep -v "cand\|reject" && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih14.png >/dev/null && python3 tools/perception/cam_snap.py agentview agent14.png >/dev/null

# openrua op 77
cat converge.py

# openrua op 78
sed -i 's/if err<0.01: break/if err<0.003: break/' converge.py; timeout 300 python3 -u converge.py 2>&1 | tail -4; python3 tools/perception/cam_snap.py robot0_eye_in_hand eih15.png >/dev/null

# openrua op 79
timeout 600 python3 potfind.py -0.15 0.05 0.15 0.35 frontview sideview 2>&1 | grep -v saved; python3 - <<'EOF'
import numpy as np
from rob import *
r=Robot(); t,R=r.tcp(); print("TCP",t.round(4)); sd=R[:,1]; ap=R[:,2].copy(); ap[2]=0; ap/=np.linalg.norm(ap)
for cam in ["frontview","sideview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    m=np.isfinite(Z)&(pw[...,0]>-0.15)&(pw[...,0]<0.05)&(pw[...,1]>0.15)&(pw[...,1]<0.35)&(Z>0.905)&(Z<0.945)
    P=pw[m]; rel=P-t; f=rel@ap; s=rel@sd
    print(cam, "lower chamber rel TCP: fwd[%.3f,%.3f] side[%.3f,%.3f]"%(f.min(),f.max(),s.min(),s.max()))
EOF

# openrua op 80
cat backoff.py; python3 - <<'EOF'
p=open("rob.py").read()
p=p.replace('''            print("   re-sending final point to converge")
            code, err = self.move_joints([waypoints[-1]], 2.0)''','''            print("   re-sending final point to converge")
            code, err = self.move_joints([waypoints[-1]], max(2.0, 4.0 * err))''')
p=p.replace('''    def move_converged(self, waypoints, seconds, tol=0.01, retries=3):''','''    def move_converged(self, waypoints, seconds, tol=0.005, retries=6):''')
open("rob.py","w").write(p)
EOF
grep -n "retries\|4.0 \* err" rob.py

# openrua op 81
timeout 600 python3 -u backoff.py 2>&1 | tail -3

# openrua op 82
timeout 600 python3 potfind.py -0.10 0.10 0.15 0.35 birdview frontview sideview 2>&1 | grep -v saved

# openrua op 83
timeout 900 python3 -u pick_approach.py -0.009 0.248 -45 5 0.005 0.068 2>&1 | grep -v "reject" && python3 tools/perception/cam_snap.py robot0_eye_in_hand eih16.png >/dev/null

# openrua op 84
timeout 300 python3 -u grasp.py 2>&1 | tail -1; timeout 600 python3 -u lift.py 0.06 2>&1 | tail -1; timeout 600 python3 heldpot.py frontview sideview 2>&1 | grep -v saved

# openrua op 85
timeout 600 python3 -u carry.py 0.07 -0.08 0 5 2>&1 | grep -v "^ *(re" | tail -1

# openrua op 86
timeout 600 python3 heldpot.py frontview sideview 2>&1 | grep -v saved

# openrua op 87
timeout 600 python3 -u carry.py 0.064 -0.0757 0 5 2>&1 | grep -v "^ *(re" | tail -1; timeout 600 python3 -u carry.py 0 0 -0.024 4 2>&1 | grep -v "^ *(re" | tail -1

# openrua op 88
python3 tools/perception/cam_snap.py agentview agent17.png >/dev/null; python3 tools/perception/cam_snap.py frontview front17.png >/dev/null; echo done

# openrua op 89
timeout 600 python3 sidecloud.py birdview frontview sideview >/dev/null; python3 - <<'EOF'
import numpy as np
for cam in ["birdview","frontview","sideview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    for name,(x0,x1,y0,y1) in {"potA":(0.17,0.30,-0.07,0.03),"potB":(0.08,0.20,0.04,0.16)}.items():
        m=np.isfinite(Z)&(pw[...,0]>x0)&(pw[...,0]<x1)&(pw[...,1]>y0)&(pw[...,1]<y1)&(Z>1.06)&(Z<1.12)
        P=pw[m]
        if len(P):
            top=P[:,2].max(); k=P[P[:,2]>top-0.008]
            print(f"{cam} {name}: top z={top:.4f} knob xy=({k[:,0].mean():.4f},{k[:,1].mean():.4f}) n={len(k)}")
        lo=np.isfinite(Z)&(pw[...,0]>x0)&(pw[...,0]<x1)&(pw[...,1]>y0)&(pw[...,1]<y1)&(Z>0.94)&(Z<0.975)
        L=pw[lo]
        if len(L): print(f"    lower chamber x[{L[:,0].min():.3f},{L[:,0].max():.3f}] y[{L[:,1].min():.3f},{L[:,1].max():.3f}]")
EOF

# openrua op 90
timeout 600 python3 -u carry.py -0.0566 0.0566 0.10 4 2>&1 | grep -v "^ *(re" | tail -1; python3 -c "
from rob import *
r=Robot(); r.gripper(GRIP['open_m'])" 2>&1 | tail -1; python3 tools/perception/cam_snap.py agentview agent18.png >/dev/null; python3 tools/perception/cam_snap.py birdview bird18.png >/dev/null

# openrua op 91
timeout 600 python3 sidecloud.py birdview frontview >/dev/null; python3 - <<'EOF'
import numpy as np
plate=dict(x=(0.09,0.28),y=(-0.061,0.129),z=0.925)
for cam in ["birdview","frontview"]:
    pw=np.load(f"{cam}_world.npy"); Z=pw[...,2]
    for name,(x0,x1,y0,y1) in {"potA":(0.17,0.30,-0.07,0.03),"potB":(0.08,0.20,0.04,0.16)}.items():
        box=np.isfinite(Z)&(pw[...,0]>x0)&(pw[...,0]<x1)&(pw[...,1]>y0)&(pw[...,1]<y1)
        P=pw[box&(Z>1.06)]; L=pw[box&(Z>0.935)&(Z<0.985)]
        top=P[:,2].max(); k=P[P[:,2]>top-0.006]
        cx,cy=k[:,0].mean(),k[:,1].mean()
        inside = plate["x"][0]+0.037<cx<plate["x"][1]-0.037 and plate["y"][0]+0.037<cy<plate["y"][1]-0.037
        print(f"{cam} {name}: top z={top:.4f} (upright height {top-plate['z']:.3f}) centre=({cx:.3f},{cy:.3f}) footprint x[{L[:,0].min():.3f},{L[:,0].max():.3f}] y[{L[:,1].min():.3f},{L[:,1].max():.3f}] fully_on_plate={inside}")
EOF
