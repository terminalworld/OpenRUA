#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace; cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
cd /workspace; ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -iE "ik|fk|plan|scene" ; echo ----; ros2 topic echo /joint_states --once

# openrua op 4
cd /workspace; mkdir -p img; for c in agentview frontview birdview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c img/$c.png & done; wait; ls -la img

# openrua op 5
cd /workspace; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ----; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12; echo ----; ros2 topic echo /birdview/color/camera_info --once | head -20

# openrua op 6
cd /workspace; cat > tools/perception/depth_world.py <<'EOF'
#!/usr/bin/env python3
"""Grab depth+intrinsics+TF for a camera; save world-XYZ point cloud as npy.
Usage: depth_world.py <camera> -> img/<camera>_xyz.npy (H,W,3) world coords
"""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

def grab(node, topic, T, timeout=30):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time; end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("depth_world")
buf = Buffer(); TransformListener(buf, node)
depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
import time; end = time.time() + 20
while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; x, y, z, w = q.x, q.y, q.z, q.w
R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = depth.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
pc = np.stack([(u-cx)*depth/fx, (v-cy)*depth/fy, depth], -1)
xyz = pc @ R.T + tr
np.save(f"img/{cam}_xyz.npy", xyz)
print("saved", xyz.shape, "depth range", np.nanmin(depth), np.nanmax(depth))
rclpy.shutdown()
EOF
timeout 90 python3 tools/perception/depth_world.py birdview

# openrua op 7
cd /workspace; python3 - <<'EOF'
import numpy as np
xyz = np.load("img/birdview_xyz.npy")
z = xyz[...,2]
# table height: mode of z in the table region
tab = z[200:450, 160:480]
print("table z median", np.median(tab))
tz = np.median(tab)
for name,(u,v) in {"stove":(262,285),"nearpot":(333,308),"farpot":(388,283),"knob":(262,238)}.items():
    print(name, xyz[v,u].round(3))
# segment objects above table by height
mask = (z > tz+0.01) & (np.arange(480)[:,None] > 200)
from scipy import ndimage
lab, n = ndimage.label(mask)
for i in range(1,n+1):
    m = lab==i
    if m.sum() < 30: continue
    pts = xyz[m]
    print(f"blob{i}: n={m.sum()} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} centroid={pts.mean(0).round(3)}")
EOF

# openrua op 8
cd /workspace; python3 - <<'EOF'
import numpy as np
xyz = np.load("img/birdview_xyz.npy")
z = xyz[...,2]
# near pot region
for lo,hi in [(0.91,0.95),(0.95,0.99),(0.99,1.02),(1.02,1.04),(1.04,1.06)]:
    m = (z>lo)&(z<=hi)&(xyz[...,0]>-0.02)&(xyz[...,0]<0.09)&(xyz[...,1]>-0.05)&(xyz[...,1]<0.12)
    if m.sum()==0: print(lo,hi,"none"); continue
    p = xyz[m]
    print(f"z({lo},{hi}] n={m.sum()} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] cen={p[:,:2].mean(0).round(3)}")
# top-down: all points of near pot above 0.95, x/y extents per ring
m = (z>0.95)&(xyz[...,0]>-0.02)&(xyz[...,0]<0.09)&(xyz[...,1]>-0.05)&(xyz[...,1]<0.12)
p = xyz[m]
print("above .95: x",p[:,0].min().round(3),p[:,0].max().round(3)," y",p[:,1].min().round(3),p[:,1].max().round(3))
EOF

# openrua op 9
cd /workspace; timeout 90 python3 tools/perception/depth_world.py agentview && timeout 90 python3 tools/perception/depth_world.py sideview && python3 - <<'EOF'
import numpy as np
for cam in ["agentview","sideview"]:
    xyz = np.load(f"img/{cam}_xyz.npy")
    x,y,z = xyz[...,0],xyz[...,1],xyz[...,2]
    print("==",cam)
    # near pot: x in [-0.03,0.11], y in [-0.06,0.13]
    for lo in np.arange(0.90,1.06,0.01):
        m=(z>lo)&(z<=lo+0.01)&(x>-0.03)&(x<0.11)&(y>-0.06)&(y<0.13)
        if m.sum()<3: continue
        p=xyz[m]
        print(f"z {lo:.2f}: n={m.sum():4d} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}]")
EOF

# openrua op 10
cd /workspace; python3 - <<'EOF'
import numpy as np
xyz = np.load("img/birdview_xyz.npy")
x,y,z = xyz[...,0],xyz[...,1],xyz[...,2]
m=(z>0.93)&(z<1.02)&(x>-0.03)&(x<0.11)&(y>-0.06)&(y<0.13)
p=xyz[m]
for q in p[np.argsort(p[:,1])]: print(q.round(3))
EOF

# openrua op 11
cd /workspace; python3 - <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node("rd")
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,"/robot_description",got.append,qos)
import time; t=time.time()
while not got and time.time()-t<15: rclpy.spin_once(n,timeout_sec=0.5)
open("img/robot.urdf","w").write(got[0].data if got else "")
print(len(got[0].data) if got else "none")
EOF
grep -n -A6 -iE "finger_joint1|finger_joint2" img/robot.urdf | grep -iE "joint name|limit|origin|axis|mimic" | head -20; grep -n -iE "<link name" img/robot.urdf | tail -8

# openrua op 12
cd /workspace; python3 -c "
import cv2
im=cv2.imread('img/agentview.png'); crop=im[230:380,280:420]; cv2.imwrite('img/crop_agent.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('img/robot0_robotview.png'); crop=im[150:380,250:480]; cv2.imwrite('img/crop_robot.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 13
cd /workspace; python3 - <<'EOF'
import numpy as np, cv2
xyz=np.load("img/agentview_xyz.npy")
im=cv2.imread("img/agentview.png")
# lower chamber rows in agentview: find rows where z ~0.92 near pot
z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
# For each image row v in pot area, find columns where the pot is (x < 0.09 & x>-0.03 & y in [-0.06,0.13] & z>0.905)
for v in range(255,360,5):
    cols=[u for u in range(280,420) if z[v,u]>0.905 and -0.03<x[v,u]<0.11 and -0.06<y[v,u]<0.13]
    if cols:
        u0,u1=min(cols),max(cols)
        print(v, u0,u1, "y:",round(y[v,u0],3), round(y[v,u1],3), "z:",round(z[v,u0],3),round(z[v,(u0+u1)//2],3), "width_m", round(y[v,u1]-y[v,u0],3))
EOF

# openrua op 14
cd /workspace; python3 - <<'EOF'
import numpy as np, cv2
im=cv2.imread("img/birdview.png"); xyz=np.load("img/birdview_xyz.npy")
crop=im[280:340,300:370]; cv2.imwrite("img/crop_bird.png", cv2.resize(crop,None,fx=8,fy=8,interpolation=cv2.INTER_NEAREST))
# print depth z along a column through pot center (u=333) and row (v=308)
z=xyz[...,2]
print("row v=308 z:", [round(z[308,u],3) for u in range(318,350)])
print("col u=333 z:", [round(z[v,333],3) for v in range(292,325)])
print("x at v:", [round(xyz[v,333,0],3) for v in range(292,325)])
print("y at u:", [round(xyz[308,u,1],3) for u in range(318,350)])
EOF

# openrua op 15
cd /workspace; timeout 20 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 Translation | head -4; timeout 20 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A3 Translation | head -4; ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A3 "^k:"

# openrua op 16
cd /workspace; timeout 120 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2

# openrua op 17
mkdir -p "$(dirname /workspace/tools/action/act.py)"
cat > /workspace/tools/action/act.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""World-frame motion helper (IK -> FollowJointTrajectory), reusable clients.

Usage:
  act.py pose                          print hand + TCP world pose from TF
  act.py move X Y Z YAWDEG [secs]      TCP to world (X,Y,Z), hand pointing
                                       down, fingers closing along world
                                       axis rotated YAWDEG from +y
  act.py line X Y Z YAWDEG [secs] [n]  straight TCP line from current pose
                                       to target in n IK waypoints
  act.py joints J1,...,J7 [secs]       raw joint target
"""
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

ROOT = Path(__file__).resolve().parents[2]
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
JOINTS = FJT["joints"]
BASE_IN_WORLD = np.array([-0.660, 0.0, 0.912])  # from tf world->panda_link0
TCP = float(M["hand"]["tcp_offset_m"])


def quat_down(yaw_deg):
    """Hand z down; yaw_deg=0 -> hand y (closing) along world y."""
    psi = math.radians(yaw_deg)
    # q = qz(psi) * qx(pi)
    return (math.cos(psi / 2), math.sin(psi / 2), 0.0, 0.0)  # (x, y, z, w)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("act")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.update(zip(m.name, m.position)), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.buf = Buffer()
        TransformListener(self.buf, self.node)
        t0 = time.time()
        while time.time() - t0 < 10 and not all(j in self.js for j in JOINTS):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if not self.ik.wait_for_service(5) or not self.fjt.wait_for_server(5):
            raise SystemExit("IK service / FJT server missing")

    def spin(self, s=0.2):
        rclpy.spin_once(self.node, timeout_sec=s)

    def arm_q(self):
        return [self.js[j] for j in JOINTS]

    def hand_tf(self):
        t0 = time.time()
        while time.time() - t0 < 10:
            self.spin()
            if self.buf.can_transform("world", "panda_hand", rclpy.time.Time()):
                break
        t = self.buf.lookup_transform("world", "panda_hand", rclpy.time.Time()).transform
        p = np.array([t.translation.x, t.translation.y, t.translation.z])
        q = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        return p, q

    def tcp_pose(self):
        p, q = self.hand_tf()
        R = quat_to_R(*q)
        return p + TCP * R[:, 2], q

    def solve_ik(self, tcp_world, q, seed):
        R = quat_to_R(*q)
        hand_world = np.asarray(tcp_world) - TCP * R[:, 2]
        hb = hand_world - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        pp = req.ik_request.pose_stamped.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, hb)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = q
        req.ik_request.robot_state.joint_state.name = JOINTS
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"IK failed for {tcp_world}: "
                               f"{None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def send_traj(self, points, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        n = len(points)
        for i, q in enumerate(points):
            t = secs * (i + 1) / n
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        # settle + fresh joint state
        for _ in range(5):
            self.spin(0.1)
        err = max(abs(a - b) for a, b in zip(self.arm_q(), points[-1]))
        print(f"traj error_code={code} max_joint_err={err:.4f}")
        return code

    def move(self, tcp, yaw, secs):
        q = quat_down(yaw)
        sol = self.solve_ik(tcp, q, self.arm_q())
        return self.send_traj([sol], secs)

    def line(self, tcp, yaw, secs, n):
        q = quat_down(yaw)
        p0, _ = self.tcp_pose()
        seed = self.arm_q()
        pts = []
        for i in range(1, n + 1):
            p = p0 + (np.asarray(tcp) - p0) * i / n
            seed = self.solve_ik(p, q, seed)
            pts.append(seed)
        return self.send_traj(pts, secs)

    def report(self):
        p, q = self.hand_tf()
        t, _ = self.tcp_pose()
        print(f"hand xyz={p.round(4)} quat(xyzw)={np.round(q, 4)}")
        print(f"tcp  xyz={t.round(4)}")
        print("fingers", round(self.js.get("panda_finger_joint1", -1), 4),
              round(self.js.get("panda_finger_joint2", -1), 4))


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    arm = Arm()
    cmd = a[0]
    if cmd == "pose":
        arm.report()
    elif cmd == "move":
        x, y, z, yaw = map(float, a[1:5])
        secs = float(a[5]) if len(a) > 5 else 3.0
        arm.move((x, y, z), yaw, secs)
        arm.report()
    elif cmd == "line":
        x, y, z, yaw = map(float, a[1:5])
        secs = float(a[5]) if len(a) > 5 else 2.0
        n = int(a[6]) if len(a) > 6 else 4
        arm.line((x, y, z), yaw, secs, n)
        arm.report()
    elif cmd == "joints":
        qs = [float(v) for v in a[1].split(",")]
        secs = float(a[2]) if len(a) > 2 else 3.0
        arm.send_traj([qs], secs)
        arm.report()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 18
cd /workspace; timeout 60 python3 tools/action/act.py pose

# openrua op 19
cd /workspace; timeout 300 python3 tools/action/act.py move 0.1 -0.35 0.95 0 4 && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_fingers.png && timeout 90 python3 tools/perception/depth_world.py agentview

# openrua op 20
cd /workspace; timeout 300 python3 tools/action/act.py move -0.1 -0.38 0.95 0 4 && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_fingers.png && timeout 90 python3 tools/perception/depth_world.py agentview

# openrua op 21
cd /workspace; timeout 120 python3 - <<'EOF'
import sys; sys.path.insert(0,"tools/action")
from act import *
arm=Arm()
p,q=arm.tcp_pose(); print("tcp",p,q)
for tgt,qq in [(p,q),(p,quat_down(0)),(p+np.array([0,0,-0.1]),quat_down(0)),(p+np.array([0.1,0,-0.1]),quat_down(0)),((-0.1,-0.3,1.0),quat_down(0)),((-0.1,-0.3,1.0),q)]:
    try:
        s=arm.solve_ik(tgt,qq,arm.arm_q()); print("OK",np.round(tgt,3),np.round(s,3))
    except Exception as e: print("FAIL",np.round(tgt,3),e)
EOF

# openrua op 22
cd /workspace; python3 - <<'EOF'
import re
s=open("tools/action/act.py").read()
s=s.replace('        req.ik_request.group_name = M["planning"]["group"]\n',
 '        req.ik_request.group_name = M["planning"]["group"]\n        req.ik_request.ik_link_name = "panda_hand"\n        req.ik_request.timeout.sec = 2\n')
open("tools/action/act.py","w").write(s)
EOF
timeout 120 python3 - <<'EOF'
import sys; sys.path.insert(0,"tools/action")
from act import *
arm=Arm()
p,q=arm.tcp_pose(); print("tcp",p,q)
for tgt,qq in [(p,q),(p,quat_down(0)),(p+np.array([0.1,0,-0.1]),quat_down(0)),((-0.1,-0.3,1.0),quat_down(0)),((-0.1,-0.38,0.95),quat_down(0)),((0.035,0.048,1.1),quat_down(90)),((-0.04,-0.21,1.1),quat_down(90))]:
    try:
        s=arm.solve_ik(tgt,qq,arm.arm_q()); print("OK",np.round(tgt,3),np.round(s,3))
    except Exception as e: print("FAIL",np.round(tgt,3),e)
EOF

# openrua op 23
cd /workspace; timeout 120 python3 - <<'EOF'
import sys; sys.path.insert(0,"tools/action")
from act import *
from moveit_msgs.srv import GetPositionFK
arm=Arm()
fk=arm.node.create_client(GetPositionFK,"/compute_fk"); fk.wait_for_service(5)
req=GetPositionFK.Request(); req.fk_link_names=["panda_hand","panda_link8"]
req.robot_state.joint_state.name=JOINTS; req.robot_state.joint_state.position=arm.arm_q()
fut=fk.call_async(req); rclpy.spin_until_future_complete(arm.node,fut,timeout_sec=30)
r=fut.result(); print("fk err",r.error_code.val)
for n,ps in zip(r.fk_link_names,r.pose_stamped):
    p=ps.pose.position;o=ps.pose.orientation; print(n,ps.header.frame_id,round(p.x,4),round(p.y,4),round(p.z,4),"|",round(o.x,4),round(o.y,4),round(o.z,4),round(o.w,4))
print("tf hand", arm.hand_tf())
# now IK straight with FK pose of panda_hand
ps=r.pose_stamped[0]
for link in ["panda_hand","","panda_link8"]:
    req=GetPositionIK.Request(); req.ik_request.group_name="panda_arm"
    if link: req.ik_request.ik_link_name=link
    req.ik_request.pose_stamped.header.frame_id=""
    req.ik_request.pose_stamped.pose = r.pose_stamped[0 if link!="panda_link8" else 1].pose
    req.ik_request.robot_state.joint_state.name=JOINTS; req.ik_request.robot_state.joint_state.position=arm.arm_q()
    fut=arm.ik.call_async(req); rclpy.spin_until_future_complete(arm.node,fut,timeout_sec=30)
    print(link or "(none)", fut.result().error_code.val, np.round(fut.result().solution.joint_state.position[:7],3))
EOF

# openrua op 24
cd /workspace; sed -i 's/^BASE_IN_WORLD = np.array(\[-0.660, 0.0, 0.912\]).*/BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK\/IK poses are already world-frame here (checked vs \/compute_fk)/' tools/action/act.py; timeout 120 python3 - <<'EOF'
import sys; sys.path.insert(0,"tools/action")
from act import *
arm=Arm()
p,q=arm.tcp_pose(); print("tcp",p,q)
for tgt,qq in [(p,q),(p,quat_down(0)),((-0.1,-0.38,0.95),quat_down(0)),((0.035,0.048,1.1),quat_down(90)),((0.035,0.048,1.02),quat_down(90)),((-0.04,-0.21,1.1),quat_down(90)),((-0.04,-0.21,1.0),quat_down(90))]:
    try:
        s=arm.solve_ik(tgt,qq,arm.arm_q()); print("OK",np.round(tgt,3),np.round(s,3))
    except Exception as e: print("FAIL",np.round(tgt,3),e)
EOF

# openrua op 25
cd /workspace; timeout 300 python3 tools/action/act.py move -0.1 -0.38 0.95 0 4 && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_fingers.png && timeout 90 python3 tools/perception/depth_world.py agentview && cp img/agentview_xyz.npy img/agent_fingers_xyz.npy

# openrua op 26
cd /workspace; timeout 300 python3 tools/action/act.py move -0.1 -0.38 0.95 0 5

# openrua op 27
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_fingers.png && timeout 90 python3 tools/perception/depth_world.py agentview && cp img/agentview_xyz.npy img/agent_fingers_xyz.npy && python3 - <<'EOF'
import numpy as np, cv2
xyz=np.load("img/agent_fingers_xyz.npy"); z=xyz[...,2]; x=xyz[...,0]; y=xyz[...,1]
# finger region: near (-0.1,-0.38), z 0.95-1.0
for lo in np.arange(0.95,1.06,0.01):
    m=(z>lo)&(z<=lo+0.01)&(x>-0.2)&(x<0.0)&(y>-0.5)&(y<-0.26)
    if m.sum()<3: continue
    p=xyz[m]; ys=np.sort(p[:,1])
    # find gap: largest jump in sorted y
    d=np.diff(ys); i=d.argmax()
    print(f"z {lo:.2f}: n={m.sum():4d} y[{ys[0]:.4f},{ys[-1]:.4f}] x[{p[:,0].min():.3f},{p[:,0].max():.3f}] biggest gap {d[i]:.4f} between {ys[i]:.4f} and {ys[i+1]:.4f}")
im=cv2.imread("img/agent_fingers.png"); 
EOF

# openrua op 28
cd /workspace; python3 -c "
import cv2
im=cv2.imread('img/agent_fingers.png'); 
import numpy as np
xyz=np.load('img/agent_fingers_xyz.npy')
# find pixel of tcp
d=np.linalg.norm(xyz-np.array([-0.1,-0.38,0.95]),axis=2); v,u=np.unravel_index(d.argmin(),d.shape); print(u,v)
crop=im[max(0,v-120):v+40, u-110:u+110]; cv2.imwrite('img/crop_fingers.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 29
cd /workspace; python3 -c "
import cv2
im=cv2.imread('img/agent_fingers.png'); crop=im[130:310, 0:200]; cv2.imwrite('img/crop_fingers.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 30
cd /workspace; python3 - <<'EOF'
import numpy as np
xyz=np.load("img/agentview_xyz.npy")  # this is the fingers snapshot, pot unchanged
# pot near-face profile: for pot column (y in [0.043,0.053]), list (z, x_front) per row
rows=[]
for v in range(250,360):
    for u in range(300,400):
        X,Y,Z=xyz[v,u]
        if 0.040<Y<0.056 and -0.03<X<0.12 and Z>0.903:
            rows.append((v,round(Z,3),round(X,3)))
            break
for r in rows: print(r)
EOF

# openrua op 31
cd /workspace; timeout 300 python3 tools/action/act.py move 0.036 0.048 1.30 90 5 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih1.png && timeout 90 python3 tools/perception/depth_world.py robot0_eye_in_hand && cp img/robot0_eye_in_hand_xyz.npy img/eih1_xyz.npy

# openrua op 32
cd /workspace; python3 - <<'EOF'
import numpy as np
xyz=np.load("img/eih1_xyz.npy"); x,y,z=xyz[...,0],xyz[...,1],xyz[...,2]
sel=(x>-0.03)&(x<0.11)&(y>-0.06)&(y<0.13)
for lo in np.arange(1.00,1.06,0.005):
    m=sel&(z>lo)&(z<=lo+0.005)
    if m.sum()<3: continue
    p=xyz[m]
    # body-only: restrict y to [0.02,0.08] for x-extent
    b=m&(y>0.025)&(y<0.07)
    pb=xyz[b] if b.sum() else None
    print(f"z {lo:.3f}: n={m.sum():4d} y[{p[:,1].min():.4f},{p[:,1].max():.4f}]  body x[{pb[:,0].min():.4f},{pb[:,0].max():.4f}] w={pb[:,0].max()-pb[:,0].min():.4f}" if pb is not None and len(pb)>0 else f"z {lo:.3f}: n={m.sum()}")
# knob
m=sel&(z>1.045); p=xyz[m]; print("knob n",m.sum(),"center",p[:,:2].mean(0).round(4),"x",p[:,0].min().round(4),p[:,0].max().round(4),"y",p[:,1].min().round(4),p[:,1].max().round(4),"ztop",p[:,2].max().round(4))
# lid top ring center estimate: points z in (1.03,1.045)
m=sel&(z>1.03)&(z<1.045)&(y>0.012)&(y<0.084); p=xyz[m]; print("lid pts",m.sum(),"x",p[:,0].min().round(4),p[:,0].max().round(4),"cx",((p[:,0].min()+p[:,0].max())/2).round(4),"y",p[:,1].min().round(4),p[:,1].max().round(4))
EOF

# openrua op 33
cd /workspace; python3 - <<'EOF'
import numpy as np
xyz=np.load("img/eih1_xyz.npy"); x,y,z=xyz[...,0],xyz[...,1],xyz[...,2]
sel=(x>0.0)&(x<0.08)&(y>0.01)&(y<0.09)
for lo in np.arange(1.035,1.065,0.005):
    m=sel&(z>lo)&(z<=lo+0.005)
    if m.sum()<1: print(lo,"none"); continue
    p=xyz[m]; print(f"z {lo:.3f}: n={m.sum():4d} x[{p[:,0].min():.4f},{p[:,0].max():.4f}] y[{p[:,1].min():.4f},{p[:,1].max():.4f}] cen={p[:,:2].mean(0).round(4)}")
# handle arc and bracket: y<0.012, z>1.0
m=(x>0.0)&(x<0.08)&(y>-0.05)&(y<0.012)&(z>0.99); p=xyz[m]
for lo in np.arange(-0.04,0.012,0.004):
    mm=m&(y>lo)&(y<=lo+0.004)
    if mm.sum(): q=xyz[mm]; print(f"handle y {lo:.3f}: n={mm.sum()} x[{q[:,0].min():.4f},{q[:,0].max():.4f}] z[{q[:,2].min():.4f},{q[:,2].max():.4f}]")
EOF

# openrua op 34
cd /workspace; cat > tools/perception/knob.py <<'EOF'
#!/usr/bin/env python3
"""Locate the moka-pot lid knob near (x0,y0) from the eye-in-hand cloud.
Usage: knob.py x0 y0 -> prints knob center xyz, lid top z, and TCP delta."""
import sys, subprocess, numpy as np
x0, y0 = float(sys.argv[1]), float(sys.argv[2])
subprocess.run(["python3", "tools/perception/depth_world.py", "robot0_eye_in_hand"], check=True, timeout=120)
xyz = np.load("img/robot0_eye_in_hand_xyz.npy"); x, y, z = xyz[..., 0], xyz[..., 1], xyz[..., 2]
sel = (np.abs(x - x0) < 0.03) & (np.abs(y - y0) < 0.03) & (z > 1.0) & (z < 1.1)
p = xyz[sel]
ztop = p[:, 2].max()
knob = p[p[:, 2] > ztop - 0.008]
lid = p[(p[:, 2] > 1.03) & (p[:, 2] < ztop - 0.012)]
print(f"knob n={len(knob)} center=({knob[:,0].mean():.4f}, {knob[:,1].mean():.4f}) "
      f"x[{knob[:,0].min():.4f},{knob[:,0].max():.4f}] y[{knob[:,1].min():.4f},{knob[:,1].max():.4f}] "
      f"ztop={ztop:.4f}  lid z max={lid[:,2].max() if len(lid) else float('nan'):.4f} n={len(lid)}")
EOF
timeout 300 python3 tools/action/act.py move 0.0384 0.0512 1.15 90 4; timeout 200 python3 tools/perception/knob.py 0.0384 0.0512

# openrua op 35
cd /workspace; timeout 300 python3 tools/action/act.py line 0.0384 0.0509 1.039 90 3 4; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand img/eih_pregrasp.png

# openrua op 36
cd /workspace; timeout 120 python3 tools/action/gripper_cmd.py 0.0; ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 37
cd /workspace; timeout 300 python3 tools/action/act.py line 0.0384 0.0509 1.12 90 3 3; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force; ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2; timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_lift.png

# openrua op 38
cd /workspace; timeout 90 python3 tools/perception/depth_world.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
xyz=np.load("img/agentview_xyz.npy"); x,y,z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(x>-0.01)&(x<0.09)&(y>0.0)&(y<0.1)&(z>0.905)&(z<1.2)
p=xyz[m]; print("pot pts", m.sum(), "z min", p[:,2].min().round(4), "z max", p[:,2].max().round(4))
# stove plate / burner from birdview
b=np.load("img/birdview_xyz.npy"); bx,by,bz=b[...,0],b[...,1],b[...,2]
m=(bz>0.92)&(bz<0.94)&(bx>-0.15)&(bx<0.06)&(by>-0.31)&(by<-0.11); q=b[m]
print("stove top n",m.sum(),"x",q[:,0].min().round(3),q[:,0].max().round(3),"y",q[:,1].min().round(3),q[:,1].max().round(3),"z",q[:,2].mean().round(4), "center", ((q[:,0].min()+q[:,0].max())/2).round(4), ((q[:,1].min()+q[:,1].max())/2).round(4))
import cv2
im=cv2.imread("img/birdview.png"); hsv=cv2.cvtColor(im,cv2.COLOR_BGR2HSV)
red=((hsv[...,0]<10)|(hsv[...,0]>170))&(hsv[...,1]>120)&(hsv[...,2]>80)
vs,us=np.nonzero(red); print("red px", len(us), "centroid uv", us.mean().round(1), vs.mean().round(1), "world", b[int(vs.mean()),int(us.mean())].round(4))
EOF

# openrua op 39
cd /workspace; timeout 300 python3 tools/action/act.py line 0.0384 0.0509 1.20 90 2 2 && timeout 300 python3 tools/action/act.py line -0.046 -0.208 1.20 90 6 6 && ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2

# openrua op 40
cd /workspace; timeout 90 python3 tools/perception/depth_world.py agentview >/dev/null && python3 - <<'EOF'
import numpy as np
xyz=np.load("img/agentview_xyz.npy"); x,y,z=xyz[...,0],xyz[...,1],xyz[...,2]
m=(x>-0.15)&(x<0.06)&(y>-0.31)&(y<-0.10)&(z>1.0)&(z<1.19)
p=xyz[m]; print("pot pts", m.sum(), "z min", p[:,2].min().round(4), "x", p[:,0].min().round(3), p[:,0].max().round(3), "y", p[:,1].min().round(3), p[:,1].max().round(3))
# lower chamber slice (z 1.07-1.11): center
s=p[(p[:,2]>1.07)&(p[:,2]<1.11)]; print("lower chamber y range", s[:,1].min().round(4), s[:,1].max().round(4), "center y", ((s[:,1].min()+s[:,1].max())/2).round(4), "front x", s[:,0].max().round(4))
EOF

# openrua op 41
cd /workspace; timeout 300 python3 tools/action/act.py line -0.046 -0.208 1.068 90 4 4 && ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 42
cd /workspace; timeout 120 python3 tools/action/gripper_cmd.py 0.04 && timeout 300 python3 tools/action/act.py line -0.046 -0.208 1.20 90 3 3 && timeout 60 python3 tools/perception/cam_snap.py agentview img/agent_final.png && timeout 90 python3 tools/perception/depth_world.py birdview > /dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview img/bird_final.png

# openrua op 43
cd /workspace; python3 - <<'EOF'
import numpy as np
from scipy import ndimage
b=np.load("img/birdview_xyz.npy"); z=b[...,2]
mask=(z>0.94)&(np.arange(480)[:,None]>200)
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<100: continue
    p=b[m]; print(f"blob n={m.sum()} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} centroid={p[:,:2].mean(0).round(3)}")
# stove-region pot: knob location and lid height
m=(z>1.05)&(np.abs(b[...,0]+0.046)<0.06)&(np.abs(b[...,1]+0.208)<0.06); p=b[m]
print("pot on stove: knob top z", p[:,2].max().round(4), "knob xy", p[:,:2].mean(0).round(4), "(stove plate center -0.046,-0.208; plate x[-0.139,0.047] y[-0.301,-0.115])")
EOF

# openrua op 44
cd /workspace; timeout 300 python3 tools/action/act.py move -0.21 0.0 1.17 0 5 >/dev/null; timeout 90 python3 tools/perception/depth_world.py birdview > /dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview img/bird_final.png >/dev/null && python3 - <<'EOF'
import numpy as np
from scipy import ndimage
b=np.load("img/birdview_xyz.npy"); z=b[...,2]
mask=(z>0.94)&(np.arange(480)[:,None]>200)
lab,n=ndimage.label(mask)
for i in range(1,n+1):
    m=lab==i
    if m.sum()<100: continue
    p=b[m]; print(f"blob n={m.sum()} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} centroid={p[:,:2].mean(0).round(3)}")
m=(z>1.045)&(np.abs(b[...,0]+0.046)<0.06)&(np.abs(b[...,1]+0.208)<0.06); p=b[m]
print("pot on stove: knob top z", p[:,2].max().round(4), "knob xy", p[:,:2].mean(0).round(4))
print("expected knob top if resting on stove top (0.9265): ", round(0.9265+(1.053-0.90),4))
EOF
