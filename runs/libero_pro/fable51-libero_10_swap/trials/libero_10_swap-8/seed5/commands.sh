#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -v parameter | head -50; echo ----; timeout 20 ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview galleryview paperview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls *.png; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|translation|x:|y:|z:" | head -80; echo ----; timeout 10 ros2 topic echo /birdview/color/camera_info --once

# openrua op 6
cat > tf_dump.py <<'EOF'
import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n = rclpy.create_node("tfd")
seen = {}
def cb(m):
    for t in m.transforms:
        seen[(t.header.frame_id, t.child_frame_id)] = t.transform
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(TFMessage, "/tf_static", cb, qos)
n.create_subscription(TFMessage, "/tf", cb, 100)
t0=time.time()
while time.time()-t0 < 3: rclpy.spin_once(n, timeout_sec=0.2)
for k,v in sorted(seen.items()):
    tr, q = v.translation, v.rotation
    print(f"{k[0]:>28} -> {k[1]:<28} t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
timeout 30 python3 tf_dump.py

# openrua op 7
cat > bird.py <<'EOF'
import numpy as np, sys
d = np.load("birdview.npy")
fx=fy=579.4112549695428; cx=320; cy=240
# world->cam: t=(-0.2,0,3.0), q=(0.7071,0.7071,0,0)
x,y,z,w = 0.7071,0.7071,0,0
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
t = np.array([-0.2,0,3.0])
def w(u,v):
    Z = d[v,u]; p = np.array([(u-cx)*Z/fx,(v-cy)*Z/fy,Z])
    return R@p + t
for u,v in [(262,238),(385,280),(330,348),(330,302),(200,400),(450,200),(160,160),(478,430)]:
    print((u,v), d[v,u], np.round(w(u,v),4))
print("depth min/max", np.nanmin(d), np.nanmax(d))
EOF
python3 bird.py

# openrua op 8
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && mv birdview_depth.npy birdview.npy && python3 bird.py

# openrua op 9
cat > seg.py <<'EOF'
import numpy as np, cv2
d = np.load("birdview.npy")
fx=fy=579.4112549695428; cx=320; cy=240
t = np.array([-0.2,0,3.0])
H,W = d.shape
vv,uu = np.mgrid[0:H,0:W]
Z = d
X = (uu-cx)*Z/fx; Y=(vv-cy)*Z/fy
# R for q=(0.7071,0.7071,0,0): world = R@p + t
x,y,z,w = 0.7071,0.7071,0,0
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
P = np.stack([X,Y,Z],-1) @ R.T + t
wz = P[...,2]
mask = (wz > 0.905) & (vv>150)&(vv<470)&(uu>150)&(uu<480)
mask = mask.astype(np.uint8)
n, lab, stats, cents = cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA] < 30: continue
    m = lab==i
    pts = P[m]
    print(f"comp {i}: area={stats[i,cv2.CC_STAT_AREA]} px centroid=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
          f"x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] zmax={pts[:,2].max():.3f} mean=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})")
EOF
python3 seg.py

# openrua op 10
cat > pot.py <<'EOF'
import numpy as np
d = np.load("birdview.npy")
fx=fy=579.4112549695428; cx=320; cy=240
t = np.array([-0.2,0,3.0])
H,W = d.shape
vv,uu = np.mgrid[0:H,0:W]
X = (uu-cx)*d/fx; Y=(vv-cy)*d/fy
x,y,z,w = 0.7071,0.7071,0,0
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
P = np.stack([X,Y,d],-1) @ R.T + t
def region(xr, yr, name):
    m = (P[...,0]>xr[0])&(P[...,0]<xr[1])&(P[...,1]>yr[0])&(P[...,1]<yr[1])
    pts = P[m]
    print(name)
    for lo in np.arange(0.90, 1.07, 0.01):
        s = pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.01)]
        if len(s): print(f"  z[{lo:.2f},{lo+0.01:.2f}) n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
    top = pts[pts[:,2]>1.0]
    print("  body(z>1.0) mean", top[:,0].mean().round(4), top[:,1].mean().round(4))
region((-0.26,-0.15),(-0.30,-0.12),"potA")
region((-0.13,-0.01),(0.13,0.31),"potB")
region((0.08,0.30),(-0.08,0.14),"stove")
EOF
python3 pot.py

# openrua op 11
timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw sideview_depth.png; timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw frontview_depth.png; timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png; ls *.npy

# openrua op 12
cat > cloud.py <<'EOF'
import numpy as np, yaml, subprocess, sys
CAMS = {
 "birdview": ((-0.2,0,3.0),(0.7071,0.7071,0,0)),
 "sideview": ((-0.0565,1.2761,1.4880),(0.0099,0.8064,-0.5912,-0.0069)),
 "frontview": ((1.0,0,1.48),(0.5608,0.5608,-0.4306,-0.4306)),
 "agentview": ((0.6586,0,1.6104),(0.6380,0.6380,-0.3048,-0.3048)),
}
def qR(x,y,z,w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def cloud(cam, npy, fx, cx=320, cy=240):
    d = np.load(npy); H,W = d.shape
    vv,uu = np.mgrid[0:H,0:W]
    X = (uu-cx)*d/fx; Y=(vv-cy)*d/fx
    t,q = CAMS[cam]; R = qR(*q)
    P = np.stack([X,Y,d],-1) @ R.T + np.array(t)
    return P
if __name__ == "__main__":
    import re
    cam = sys.argv[1]; npy = sys.argv[2]
    info = subprocess.run(["ros2","topic","echo",f"/{cam}/color/camera_info","--once"],capture_output=True,text=True,timeout=30).stdout
    k = [float(v) for v in re.findall(r"^- ([\d.e+-]+)", info.split("k:")[1].split("r:")[0], re.M)]
    fx = k[0]; print("fx",fx)
    P = cloud(cam,npy,fx)
    np.save(f"{cam}_cloud.npy", P)
    flat = P.reshape(-1,3)
    # table-plane check
    tab = flat[(np.abs(flat[:,2]-0.90)<0.005)]
    print("table pts", len(tab), "z mean", tab[:,2].mean())
    def region(xr, yr, name):
        m = (P[...,0]>xr[0])&(P[...,0]<xr[1])&(P[...,1]>yr[0])&(P[...,1]<yr[1])&(P[...,2]>0.905)&(P[...,2]<1.2)
        pts = P[m]; print(name, len(pts))
        for lo in np.arange(0.90, 1.07, 0.01):
            s = pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.01)]
            if len(s): print(f"  z[{lo:.2f},{lo+0.01:.2f}) n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}")
    region((-0.26,-0.15),(-0.30,-0.12),"potA")
    region((-0.13,-0.01),(0.13,0.31),"potB")
    region((0.08,0.30),(-0.08,0.14),"stove")
EOF
python3 cloud.py sideview sideview_depth.npy

# openrua op 13
cat > urdf.py <<'EOF'
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n = rclpy.create_node("u")
got=[]
n.create_subscription(String, "/robot_description", got.append, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
while not got: rclpy.spin_once(n, timeout_sec=0.5)
open("robot.urdf","w").write(got[0].data)
EOF
timeout 30 python3 urdf.py; grep -n -A30 '<link name="panda_hand"' robot.urdf | head -60; grep -n -A25 '<link name="panda_leftfinger"' robot.urdf | head -40; grep -n -B2 -A12 'finger_joint1' robot.urdf | head -40

# openrua op 14
python3 -c "
import cv2
im = cv2.imread('birdview.png')
crop = im[200:320, 220:440]
cv2.imwrite('bird_crop.png', cv2.resize(crop, None, fx=4, fy=4, interpolation=cv2.INTER_NEAREST))
im = cv2.imread('agentview.png')
cv2.imwrite('agent_crop.png', cv2.resize(im[160:320, 150:520], None, fx=2, fy=2))
"

# openrua op 15
mkdir -p "$(dirname /workspace/lib.py)"
cat > /workspace/lib.py <<'OPENRUA_EOF'
"""Shared robot helpers: FK/IK, trajectory, gripper, joint state, servo.

World frame → panda_link0 offset comes from TF (world→panda_link0).
IK/FK are done in panda_link0 (leave frame_id empty per machine.yaml).
"""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (TF)
TCP = M["hand"]["tcp_offset_m"]


def down_quat(yaw_deg=0.0):
    """Hand z pointing world -z; yaw_deg rotates about world z.
    yaw=0: fingers open along world y; yaw=90: fingers along world x."""
    r = Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])
    return r.as_quat()  # x,y,z,w


class Robot:
    def __init__(self, name="lib"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 10:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def fk_world(self, q=None, link="panda_hand"):
        """Return (pos_world, quat) of link for joint config q (default current)."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def tcp_world(self, q=None):
        pos, quat = self.fk_world(q)
        R = Rot.from_quat(quat).as_matrix()
        return pos + TCP * R[:, 2], quat

    # ---------------- planning ----------------
    def ik_world(self, pos, quat, at_tcp=True, seed=None, tries=3):
        """IK for a world pose of the TCP (or hand). Returns joint list or None."""
        pos = np.array(pos, dtype=float)
        if at_tcp:
            R = Rot.from_quat(quat).as_matrix()
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE_IN_WORLD
        seed = seed if seed is not None else self.arm_q()
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 1
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    # ---------------- acting ----------------
    def move_q(self, q, seconds=3.0, via=None):
        """Send a trajectory (optionally through via points) and wait."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        n = len(wps)
        for i, wp in enumerate(wps):
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            t = seconds * (i + 1) / n
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        cur = np.array(self.arm_q())
        err = np.abs(cur - np.array(q)).max()
        print(f"  move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik_world(pos, quat, seed=seed)
        if q is None:
            print(f"  IK FAILED for {np.round(pos,3)}")
            return None
        self.move_q(q, seconds)
        p, _ = self.tcp_world()
        print(f"  tcp now {np.round(p,4)} (target {np.round(pos,4)}) err={np.linalg.norm(p-pos):.4f}")
        return q

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap

    def servo(self, v, n=20, frame=None):
        """Stream n twist messages (linear m/s in base frame)."""
        msg = TwistStamped()
        msg.header.frame_id = frame or TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
OPENRUA_EOF

# openrua op 16
cat > t1.py <<'EOF'
from lib import *
r = Robot("t1")
print("q", np.round(r.arm_q(),3))
pos, quat = r.fk_world()
print("hand world", np.round(pos,4), np.round(quat,4))
print("tcp world", np.round(r.tcp_world()[0],4))
print("finger gap", r.finger_gap())
for yaw in (0, 90):
    q = down_quat(yaw); print("down_quat", yaw, np.round(q,4))
# IK feasibility probes
for name, p in [("aboveA", (-0.206,-0.195,1.20)), ("graspA", (-0.206,-0.195,1.00)),
                ("stoveC", (0.188,0.03,1.00)), ("stoveNear", (0.14,0.03,1.00)), ("stoveFar", (0.24,0.03,1.00)),
                ("stoveC_hi", (0.188,0.03,1.10)), ("aboveB", (-0.068,0.233,1.20)), ("graspB", (-0.068,0.233,1.00))]:
    for yaw in (90,):
        q = r.ik_world(p, down_quat(yaw))
        print(name, yaw, None if q is None else np.round(q,3))
EOF
timeout 300 python3 t1.py

# openrua op 17
cat > t2.py <<'EOF'
from lib import *
import lib
r = Robot("t2")
req = GetPositionFK.Request(); req.fk_link_names=["panda_hand","panda_link0","panda_link1"]
req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=r.arm_q()
fut = r.fk.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
for ps in fut.result().pose_stamped:
    print(ps.header.frame_id, np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],4))
# try IK without base offset
lib.BASE_IN_WORLD = np.zeros(3)
for name, p in [("cur_hand", (-0.2035,0,1.2701))]:
    q = r.ik_world(p, (0.9996,0,-0.0284,0), at_tcp=False)
    print(name, None if q is None else np.round(q,3))
for name, p in [("aboveA", (-0.206,-0.195,1.20)), ("graspA", (-0.206,-0.195,1.00)),("stoveC", (0.188,0.03,1.00))]:
    q = r.ik_world(p, down_quat(90))
    print(name, None if q is None else np.round(q,3))
EOF
timeout 300 python3 t2.py

# openrua op 18
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/lib.py", "old_string": "BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (TF)", "new_string": "# Verified empirically: /compute_fk and /compute_ik on this machine work in\\n# the WORLD frame (FK of panda_link0 returns (-0.66, 0, 0.912)), so no offset.\\nBASE_IN_WORLD = np.zeros(3)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 19
cat > t3.py <<'EOF'
from lib import *
r = Robot("t3")
def tilted(yaw, pitch):
    return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat(down_quat(yaw))).as_quat()
seed = r.arm_q()
for x in (0.10, 0.12, 0.14, 0.16, 0.19, 0.22, 0.24):
    row = []
    for pitch in (0, -15, -30, -45):
        for yaw in (90, 0):
            q = r.ik_world((x, 0.03, 1.00), tilted(yaw, pitch), seed=seed, tries=2)
            row.append(f"p{pitch}y{yaw}:{'ok' if q else '--'}")
    print(x, " ".join(row))
EOF
timeout 600 python3 t3.py

# openrua op 20
cat > m1.py <<'EOF'
from lib import *
r = Robot("m1")
q = r.move_tcp((-0.206,-0.195,1.25), down_quat(90), seconds=3)
print("q", np.round(r.arm_q(),3))
EOF
timeout 900 python3 m1.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih1_depth.png && timeout 30 python3 tf_dump.py | grep eye_in_hand

# openrua op 21
cat > t4.py <<'EOF'
from lib import *
r = Robot("t4")
q = r.arm_q(); print("q", np.round(q,3))
pos, quat = r.fk_world(); print("hand", np.round(pos,4), "quat", np.round(quat,4))
print("euler xyz deg", np.round(Rot.from_quat(quat).as_euler("xyz", degrees=True),1))
print("target quat", np.round(down_quat(90),4), np.round(Rot.from_quat(down_quat(90)).as_euler("xyz",degrees=True),1))
print("gap", r.finger_gap())
EOF
timeout 120 python3 t4.py

# openrua op 22
cat > m2.py <<'EOF'
from lib import *
r = Robot("m2")
q = r.ik_world((-0.206,-0.195,1.25), down_quat(90))
print("ik", np.round(q,3))
r.move_q(q, 4)
print("q now", np.round(r.arm_q(),3))
pos, quat = r.fk_world(); print("hand", np.round(pos,4), "euler", np.round(Rot.from_quat(quat).as_euler("xyz", degrees=True),1))
EOF
timeout 900 python3 m2.py

# openrua op 23
cat > m3.py <<'EOF'
from lib import *
import sys
r = Robot("m3")
q = [0.197,-0.199,-0.575,-2.231,-0.122,2.06,-1.882]
r.move_q(q, float(sys.argv[1]) if len(sys.argv)>1 else 8)
print("q now", np.round(r.arm_q(),3))
pos, quat = r.fk_world(); print("hand", np.round(pos,4), "euler", np.round(Rot.from_quat(quat).as_euler("xyz", degrees=True),1))
EOF
timeout 900 python3 m3.py 8

# openrua op 24
cat > t5.py <<'EOF'
from lib import *
r = Robot("t5")
q = r.arm_q(); print("q", np.round(q,3))
pos, quat = r.fk_world()
R = Rot.from_quat(quat).as_matrix()
print("hand", np.round(pos,4)); print("hand x axis", np.round(R[:,0],3), "y axis", np.round(R[:,1],3), "z axis", np.round(R[:,2],3))
Rt = Rot.from_quat(down_quat(90)).as_matrix()
print("target x", np.round(Rt[:,0],3), "y", np.round(Rt[:,1],3), "z", np.round(Rt[:,2],3))
print("angle diff deg", np.degrees((Rot.from_quat(quat).inv()*Rot.from_quat(down_quat(90))).magnitude()))
EOF
timeout 120 python3 t5.py

# openrua op 25
cat > t6.py <<'EOF'
from lib import *
r = Robot("t6")
for link in ("panda_link8","panda_hand"):
    pos, quat = r.fk_world(link=link)
    R = Rot.from_quat(quat).as_matrix()
    print(link, np.round(pos,4), "y axis", np.round(R[:,1],3))
EOF
timeout 120 python3 t6.py

# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/lib.py", "old_string": "        pos = np.array(pos, dtype=float)\\n        if at_tcp:\\n            R = Rot.from_quat(quat).as_matrix()\\n            pos = pos - TCP * R[:, 2]\\n        pos = pos - BASE_IN_WORLD", "new_string": "        pos = np.array(pos, dtype=float)\\n        if at_tcp:\\n            R = Rot.from_quat(quat).as_matrix()\\n            pos = pos - TCP * R[:, 2]\\n        pos = pos - BASE_IN_WORLD\\n        # The IK group\'s tip link is panda_link8 (verified via FK); panda_hand\\n        # = link8 rotated -45 deg about z.  Convert the hand quaternion.\\n        quat = (Rot.from_quat(quat) * Rot.from_euler(\\"z\\", 45, degrees=True)).as_quat()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/lib.py", "old_string": "    def move_q(self, q, seconds=3.0, via=None):\\n        \\"\\"\\"Send a trajectory (optionally through via points) and wait.\\"\\"\\"\\n        goal = FollowJointTrajectory.Goal()", "new_string": "    def move_q(self, q, seconds=3.0, via=None, retries=3, tol=0.02):\\n        \\"\\"\\"Send a trajectory and wait; re-send until within tol (this\\n        machine\'s joint7 tops out near 0.2 rad/s, so long yaw moves need\\n        several passes / long durations).\\"\\"\\"\\n        cur = np.array(self.arm_q())\\n        delta = np.abs(cur - np.array(q)).max()\\n        seconds = max(seconds, delta / 0.18)\\n        code, err = self._send_traj(q, seconds, via)\\n        for _ in range(retries):\\n            if err <= tol:\\n                break\\n            code, err = self._send_traj(q, max(2.0, err / 0.15), None)\\n        return code, err\\n\\n    def _send_traj(self, q, seconds, via):\\n        goal = FollowJointTrajectory.Goal()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 28
cat > m4.py <<'EOF'
from lib import *
r = Robot("m4")
q = r.move_tcp((-0.206,-0.195,1.25), down_quat(90), seconds=4)
pos, quat = r.fk_world(); R = Rot.from_quat(quat).as_matrix()
print("hand y axis", np.round(R[:,1],3), "z axis", np.round(R[:,2],3))
print("q", np.round(r.arm_q(),3), "gap", r.finger_gap())
EOF
timeout 900 python3 m4.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih1_depth.png && timeout 30 python3 tf_dump.py | grep eye_in_hand && timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:"

# openrua op 29
cat > eih.py <<'EOF'
import numpy as np, sys, cv2
from scipy.spatial.transform import Rotation as Rot
npy = sys.argv[1]; t = np.array([float(v) for v in sys.argv[2:5]]); q = [float(v) for v in sys.argv[5:9]]
fx = 312.77408948188935; cx=320; cy=240
d = np.load(npy); H,W = d.shape
vv,uu = np.mgrid[0:H,0:W]
P = np.stack([(uu-cx)*d/fx,(vv-cy)*d/fx,d],-1) @ Rot.from_quat(q).as_matrix().T + t
np.save("eih_cloud.npy", P)
z = P[...,2]
print("table z median (central band):", np.median(z[(np.abs(z-0.9)<0.03)]))
for lo in np.arange(0.90, 1.08, 0.01):
    m = (z>=lo)&(z<lo+0.01)&(vv<360)
    if m.sum()>5:
        s = P[m]; print(f"z[{lo:.2f},{lo+0.01:.2f}) n={m.sum():5d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]")
# object above table near center
m = (z>0.95)&(vv<360)&(np.abs(P[...,0]-P[...,0][240,320])<0.12)&(np.abs(P[...,1]-P[...,1][240,320])<0.12)
s = P[m]
print("object pts", len(s), "z range", s[:,2].min(), s[:,2].max())
top = s[s[:,2]>1.025]
print("top plateau: x[%.4f,%.4f] y[%.4f,%.4f] center (%.4f,%.4f)"%(top[:,0].min(),top[:,0].max(),top[:,1].min(),top[:,1].max(), (top[:,0].min()+top[:,0].max())/2,(top[:,1].min()+top[:,1].max())/2))
# per-slab in y (exclude handle/spout) -> body x extent
body = top[(np.abs(top[:,1]-np.median(top[:,1]))<0.02)]
print("body(|dy|<2cm) x extent", body[:,0].min(), body[:,0].max(), "width", body[:,0].max()-body[:,0].min())
knob = s[s[:,2]>1.045]
if len(knob): print("knob center", knob[:,0].mean(), knob[:,1].mean(), "zmax", knob[:,2].max(), "n", len(knob))
# pixel of the top plateau centroid
ys, xs = np.where(m & (z>1.025))
print("top centroid px", xs.mean(), ys.mean())
EOF
python3 eih.py eih1_depth.npy -0.2063 -0.1451 1.3539 1.0 0.0015 -0.0001 0.0004

# openrua op 30
cat > fing.py <<'EOF'
import numpy as np
P = np.load("eih_cloud.npy"); d = np.load("eih1_depth.npy")
z = P[...,2]
# fingers: in lower band v>360, points much closer than table (z world > 1.1)
m = (z>1.15)
vs,us = np.where(m)
print("finger px u range", us.min(), us.max(), "v range", vs.min(), vs.max(), "n", m.sum())
s = P[m]
print("finger pts z range", s[:,2].min(), s[:,2].max())
left = s[s[:,0] < -0.2062]; right = s[s[:,0] > -0.2062]
print("left finger x range", left[:,0].min(), left[:,0].max(), "right finger x range", right[:,0].min(), right[:,0].max())
print("inner gap (right.min - left.max)", right[:,0].min()-left[:,0].max())
for lo in np.arange(1.15,1.36,0.02):
    mm = (s[:,2]>=lo)&(s[:,2]<lo+0.02)
    if mm.sum()>3:
        ss = s[mm]; l = ss[ss[:,0]<-0.2062]; rr = ss[ss[:,0]>-0.2062]
        if len(l) and len(rr): print(f"z[{lo:.2f}] n={mm.sum()} inner gap {rr[:,0].min()-l[:,0].max():.4f}  y range {ss[:,1].min():.3f},{ss[:,1].max():.3f}")
EOF
python3 fing.py

# openrua op 31
cat > handle.py <<'EOF'
import numpy as np
P = np.load("eih_cloud.npy"); z = P[...,2]
m = (z>0.93)&(z<1.1)&(P[...,1]<-0.236)&(np.abs(P[...,0]+0.205)<0.05)
s = P[m]; print("handle pts", len(s))
for lo in np.arange(-0.275,-0.235,0.005):
    mm = (s[:,1]>=lo)&(s[:,1]<lo+0.005)
    if mm.sum(): ss=s[mm]; print(f"y[{lo:.3f}] n={mm.sum():4d} x[{ss[:,0].min():.4f},{ss[:,0].max():.4f}] w={ss[:,0].max()-ss[:,0].min():.4f} z[{ss[:,2].min():.3f},{ss[:,2].max():.3f}]")
# spout side
m = (z>0.93)&(z<1.1)&(P[...,1]>-0.166)&(np.abs(P[...,0]+0.205)<0.05)
s = P[m]; print("spout pts", len(s))
for lo in np.arange(-0.166,-0.13,0.005):
    mm = (s[:,1]>=lo)&(s[:,1]<lo+0.005)
    if mm.sum(): ss=s[mm]; print(f"y[{lo:.3f}] n={mm.sum():4d} x[{ss[:,0].min():.4f},{ss[:,0].max():.4f}] w={ss[:,0].max()-ss[:,0].min():.4f} z[{ss[:,2].min():.3f},{ss[:,2].max():.3f}]")
# body width per y slice at top
m = (z>1.02)&(np.abs(P[...,0]+0.205)<0.06)&(P[...,1]>-0.25)&(P[...,1]<-0.15)
s = P[m]
for lo in np.arange(-0.245,-0.155,0.005):
    mm = (s[:,1]>=lo)&(s[:,1]<lo+0.005)
    if mm.sum(): ss=s[mm]; print(f"top y[{lo:.3f}] n={mm.sum():4d} x[{ss[:,0].min():.4f},{ss[:,0].max():.4f}] w={ss[:,0].max()-ss[:,0].min():.4f}")
EOF
python3 handle.py

# openrua op 32
python3 -c "
import numpy as np
P=np.load('eih_cloud.npy'); z=P[...,2]
for lo in (1.036,1.040,1.045,1.050):
    m=(z>lo)&(z<1.1)&(np.abs(P[...,0]+0.205)<0.03)&(np.abs(P[...,1]+0.195)<0.03)
    s=P[m]
    if len(s): print(lo, len(s), 'x[%.4f,%.4f] y[%.4f,%.4f] cx=%.4f cy=%.4f'%(s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max(),(s[:,0].min()+s[:,0].max())/2,(s[:,1].min()+s[:,1].max())/2))
"

# openrua op 33
cat > graspA.py <<'EOF'
from lib import *
r = Robot("graspA")
kx, ky = -0.2052, -0.1946
Q = down_quat(90)
print("gap before", r.finger_gap())
if r.finger_gap() < 0.078: r.gripper(0.04)
print("-- above knob")
r.move_tcp((kx, ky, 1.12), Q, 3)
print("-- descend to knob")
r.move_tcp((kx, ky, 1.039), Q, 3)
print("-- close")
gap = r.gripper(0.0)
print("gap after close", gap)
q = r.arm_q(); print("q", np.round(q,3))
EOF
timeout 1200 python3 graspA.py

# openrua op 34
cat > liftA.py <<'EOF'
from lib import *
r = Robot("liftA")
kx, ky = -0.2052, -0.1946
Q = down_quat(90)
r.move_tcp((kx, ky, 1.10), Q, 3)
print("gap after lift", r.finger_gap())
EOF
timeout 1200 python3 liftA.py && for c in sideview agentview; do timeout 60 python3 tools/perception/cam_snap.py $c ${c}_2.png; done

# openrua op 35
cat > t7.py <<'EOF'
from lib import *
r = Robot("t7")
seed = r.arm_q()
for yaw in (0, 180, 90):
    for x in (0.13, 0.14, 0.15):
        q = r.ik_world((x, 0.08, 1.084), down_quat(yaw), seed=seed, tries=2)
        q2 = r.ik_world((x, -0.02, 1.084), down_quat(yaw), seed=seed, tries=2)
        print(f"yaw{yaw} x{x}: y=0.08 {'ok '+str(np.round(q,2)) if q else '--'} | y=-0.02 {'ok '+str(np.round(q2,2)) if q2 else '--'}")
EOF
timeout 600 python3 t7.py

# openrua op 36
python3 -c "
import numpy as np
P=np.load('eih_cloud.npy'); z=P[...,2]
m=(P[...,0]>0.07)&(P[...,0]<0.30)&(P[...,1]>-0.09)&(P[...,1]<0.14)&(z>0.91)&(z<1.0)
s=P[m]; print(len(s))
import collections
for lo in np.arange(0.915,0.96,0.0025):
    mm=(s[:,2]>=lo)&(s[:,2]<lo+0.0025)
    if mm.sum()>20: ss=s[mm]; print(f'z[{lo:.4f}] n={mm.sum():5d} x[{ss[:,0].min():.3f},{ss[:,0].max():.3f}] y[{ss[:,1].min():.3f},{ss[:,1].max():.3f}]')
# radial profile around burner center
c=np.array([0.19,0.03])
rr=np.hypot(s[:,0]-c[0],s[:,1]-c[1])
for lo in np.arange(0,0.10,0.01):
    mm=(rr>=lo)&(rr<lo+0.01)
    if mm.sum(): print(f'r[{lo:.2f}] n={mm.sum():5d} z median {np.median(s[mm,2]):.4f} max {s[mm,2].max():.4f}')
"

# openrua op 37
cat > t8.py <<'EOF'
from lib import *
r = Robot("t8")
def tilted(yaw, pitch):
    return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat(down_quat(yaw))).as_quat()
seed = r.arm_q()
for z in (1.075, 1.10):
  for pitch in (0, -5, -10, -15):
    row=[]
    for x in (0.14, 0.15, 0.16, 0.17, 0.18):
        q = r.ik_world((x, 0.03, z), tilted(180, pitch), seed=seed, tries=2)
        row.append(f"x{x}:{'ok' if q else '--'}")
    print(f"z{z} pitch{pitch}: "+" ".join(row))
EOF
timeout 600 python3 t8.py

# openrua op 38
cat > placeA.py <<'EOF'
from lib import *
r = Robot("placeA")
def tilted(yaw, pitch):
    return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat(down_quat(yaw))).as_quat()
POT_DROP = 0.139   # knob-grasp TCP to pot base
tx, ty = 0.178, 0.075  # desired pot base center
pitch = -10
h = Rot.from_quat(tilted(180, pitch)).as_matrix()[:, 2]
print("hand z axis", np.round(h,4))
base_z = 0.930 + 0.004 + 0.0335*np.sin(np.radians(10))
tcp_rel = np.array([tx, ty, base_z]) - POT_DROP*h
print("release TCP", np.round(tcp_rel,4))
print("gap", r.finger_gap())
print("-- lift")
p, _ = r.tcp_world()
r.move_tcp((p[0], p[1], 1.25), down_quat(90), 3)
print("-- traverse")
q = r.move_tcp((tcp_rel[0], tcp_rel[1], 1.22), tilted(180, pitch), 5)
if q is None:
    q = r.move_tcp((tcp_rel[0]-0.03, tcp_rel[1], 1.22), tilted(180, pitch), 5)
print("gap", r.finger_gap())
EOF
timeout 1500 python3 placeA.py

# openrua op 39
timeout 60 python3 tools/perception/cam_snap.py agentview agentview_3.png

# openrua op 40
cat > placeA2.py <<'EOF'
from lib import *
r = Robot("placeA2")
def tilted(yaw, pitch):
    return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat(down_quat(yaw))).as_quat()
tcp_rel = (0.1539, 0.075, 1.0767)
print("-- descend")
r.move_tcp((tcp_rel[0], tcp_rel[1], 1.13), tilted(180, -10), 3)
r.move_tcp(tcp_rel, tilted(180, -10), 3)
print("-- release")
r.gripper(0.04)
print("-- retreat")
r.move_tcp((tcp_rel[0], tcp_rel[1], 1.20), tilted(180, -10), 3)
EOF
timeout 1500 python3 placeA2.py && timeout 60 python3 tools/perception/cam_snap.py agentview agentview_4.png && timeout 60 python3 tools/perception/cam_snap.py sideview sideview_4.png

# openrua op 41
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && mv birdview_depth.npy birdview.npy && python3 - <<'EOF'
import numpy as np
exec(open("pot.py").read().split("region((-0.26")[0])
region((0.08,0.30),(-0.08,0.14),"stove area")
EOF

# openrua op 42
cat > m5.py <<'EOF'
from lib import *
r = Robot("m5")
r.move_tcp((-0.068, 0.233, 1.25), down_quat(90), 5)
pos, quat = r.fk_world(); R = Rot.from_quat(quat).as_matrix()
print("hand y axis", np.round(R[:,1],3), "z axis", np.round(R[:,2],3), "gap", r.finger_gap())
EOF
timeout 1500 python3 m5.py && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih2_depth.png && timeout 30 python3 tf_dump.py | grep eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && mv birdview_depth.npy birdview.npy && timeout 60 python3 tools/perception/cam_snap.py birdview birdview_5.png

# openrua op 43
python3 - <<'EOF'
import numpy as np
exec(open("pot.py").read().split("region((-0.26")[0])
region((0.08,0.30),(-0.08,0.14),"stove area")
EOF
python3 eih.py eih2_depth.npy -0.0681 0.2833 1.3540 1.0 0.0015 -0.0006 -0.0002 2>&1 | tail -6
python3 -c "
import numpy as np
P=np.load('eih_cloud.npy'); z=P[...,2]
for lo in (1.036,1.045):
    m=(z>lo)&(z<1.1)&(np.abs(P[...,0]+0.068)<0.03)&(np.abs(P[...,1]-0.233)<0.03)
    s=P[m]
    if len(s): print(lo, len(s), 'x[%.4f,%.4f] y[%.4f,%.4f] cx=%.4f cy=%.4f zmax=%.4f'%(s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max(),(s[:,0].min()+s[:,0].max())/2,(s[:,1].min()+s[:,1].max())/2, s[:,2].max()))
"

# openrua op 44
python3 - <<'EOF'
import numpy as np
exec(open("pot.py").read().split("def region")[0])
m=(P[...,0]>0.08)&(P[...,0]<0.30)&(P[...,1]>-0.08)&(P[...,1]<0.14)&(P[...,2]>1.0)
s=P[m]
for lo in np.arange(1.04,1.10,0.01):
    mm=(s[:,2]>=lo)&(s[:,2]<lo+0.01)
    if mm.sum(): ss=s[mm]; print(f"z[{lo:.2f}] n={mm.sum():4d} x[{ss[:,0].min():.3f},{ss[:,0].max():.3f}] y[{ss[:,1].min():.3f},{ss[:,1].max():.3f}]")
top=s[(s[:,2]>1.06)&(s[:,2]<1.075)]
# body only: exclude handle by taking points with |y - median| small and x-extent
print("top y center", (top[:,1].min()+top[:,1].max())/2)
knob=s[s[:,2]>=1.075]
if len(knob): print("knob", knob[:,0].mean(), knob[:,1].mean(), len(knob))
EOF

# openrua op 45
cat > graspB.py <<'EOF'
from lib import *
r = Robot("graspB")
kx, ky = -0.0671, 0.2331
Q = down_quat(90)
if r.finger_gap() < 0.078: r.gripper(0.04)
print("-- above knob"); r.move_tcp((kx, ky, 1.12), Q, 3)
print("-- descend"); r.move_tcp((kx, ky, 1.039), Q, 3)
print("-- close"); gap = r.gripper(0.0)
print("-- lift"); r.move_tcp((kx, ky, 1.25), Q, 3)
print("gap after lift", r.finger_gap())
EOF
timeout 1500 python3 graspB.py && timeout 60 python3 tools/perception/cam_snap.py sideview sideview_6.png

# openrua op 46
cat > placeB.py <<'EOF'
from lib import *
r = Robot("placeB")
def tilted(yaw, pitch):
    return (Rot.from_euler("y", pitch, degrees=True) * Rot.from_quat(down_quat(yaw))).as_quat()
POT_DROP = 0.139
tx, ty = 0.185, -0.015   # pot A landed ~9 mm short in x; compensate
pitch = -10
Qp = tilted(180, pitch)
h = Rot.from_quat(Qp).as_matrix()[:, 2]
base_z = 0.930 + 0.004 + 0.0335*np.sin(np.radians(10))
tcp_rel = np.array([tx, ty, base_z]) - POT_DROP*h
print("release TCP", np.round(tcp_rel,4))
print("-- traverse")
q = r.move_tcp((tcp_rel[0], tcp_rel[1], 1.22), Qp, 5)
if q is None:
    raise SystemExit("traverse IK failed")
print("gap", r.finger_gap())
print("-- descend")
r.move_tcp((tcp_rel[0], tcp_rel[1], 1.13), Qp, 3)
r.move_tcp(tcp_rel, Qp, 3)
print("-- release"); r.gripper(0.04)
print("-- retreat"); r.move_tcp((tcp_rel[0], tcp_rel[1], 1.20), Qp, 3)
EOF
timeout 1500 python3 placeB.py && timeout 60 python3 tools/perception/cam_snap.py agentview agentview_7.png

# openrua op 47
cat > park.py <<'EOF'
from lib import *
r = Robot("park")
r.move_tcp((-0.15, 0.0, 1.30), down_quat(90), 5)
print("gap", r.finger_gap())
EOF
timeout 1500 python3 park.py && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && mv birdview_depth.npy birdview.npy && for c in birdview agentview sideview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c ${c}_final.png; done
python3 - <<'EOF'
import numpy as np, cv2
exec(open("pot.py").read().split("def region")[0])
m=(P[...,0]>0.06)&(P[...,0]<0.32)&(P[...,1]>-0.10)&(P[...,1]<0.16)&(P[...,2]>0.905)
s=P[m]
for lo in np.arange(0.92,1.10,0.01):
    mm=(s[:,2]>=lo)&(s[:,2]<lo+0.01)
    if mm.sum(): ss=s[mm]; print(f"z[{lo:.2f}] n={mm.sum():4d} x[{ss[:,0].min():.3f},{ss[:,0].max():.3f}] y[{ss[:,1].min():.3f},{ss[:,1].max():.3f}]")
# separate the two pots by y sign relative to burner centre
for name, sel in (("pot at +y", s[:,1]>0.03), ("pot at -y", s[:,1]<0.03)):
    p = s[sel & (s[:,2]>1.05)]
    knob = p[p[:,2]>=1.075]
    print(name, "top n", len(p), "top x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f"%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].max()),
          "knob centre", np.round(knob[:,:2].mean(0),3) if len(knob) else None)
# anything else above the table elsewhere (pots left behind / knocked over)?
m2=(P[...,2]>0.905)&(P[...,0]>-0.5)&(P[...,0]<0.5)&(np.abs(P[...,1])<0.6)&~((P[...,0]>0.06)&(P[...,0]<0.32)&(P[...,1]>-0.10)&(P[...,1]<0.16))
lab_n, lab, st, cen = cv2.connectedComponentsWithStats(m2.astype(np.uint8))
for i in range(1,lab_n):
    if st[i,cv2.CC_STAT_AREA]>30:
        pts=P[lab==i]; print("other object:", st[i,cv2.CC_STAT_AREA], "px  x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f"%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),pts[:,2].max()))
EOF

# openrua op 48
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-quirks
description: Verified quirks of the /workspace Franka Panda sim (IK tip link, frames, joint7 speed, gripper opening, moka-pot knob grasp)
metadata:
  type: project
---

Facts verified empirically on the /workspace Panda workstation (2026-09-13):

- `/compute_ik` and `/compute_fk` answer in the WORLD frame (FK of panda_link0
  returns (-0.66, 0, 0.912)), despite docs/30-action.md saying the base frame.
- IK group tip link is `panda_link8`, which is rotated -45° about z relative to
  `panda_hand`; convert hand quaternions with `q_hand * Rz(+45°)` or the yaw
  lands 45° off. Positions coincide (TCP offset 0.1034 along z).
- joint7 tops out near 0.2 rad/s: large yaw changes return error_code=-5
  (goal tolerance) with the other joints on target; re-sending converges.
- Gripper fully open = 0.080 m between finger inner faces (measured).
- Moka pots are ~7.7 cm wide at the top (too wide to straddle); they are single
  rigid bodies, so a pinch grasp on the 1.5 cm lid knob (TCP ~6 mm above the
  lid) lifts the whole pot reliably; closed gap on the knob ≈ 0.0114.
- Straight-down reach at TCP z≈1.0–1.1 ends around world x=0.14; pitching the
  hand -10° about y extends it to x≈0.18 (stove burner centre ≈ (0.178, 0.03)).

**Why:** these contradict or go beyond the shipped docs and cost a lot of
probing to discover.
**How to apply:** reuse `/workspace/lib.py` (Robot class) if still present;
otherwise apply the frame/tip-link corrections before trusting IK results.
OPENRUA_EOF

# openrua op 49
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no MEMORY.md yet)")

# openrua op 50
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda sim quirks](panda-sim-quirks.md) — IK tip is panda_link8 (45° yaw offset), IK/FK in world frame, slow joint7, knob grasp for moka pots
OPENRUA_EOF
