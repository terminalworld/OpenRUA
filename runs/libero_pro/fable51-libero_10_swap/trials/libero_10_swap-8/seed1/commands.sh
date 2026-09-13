#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list; echo ----; ros2 action list; echo ----; ros2 service list | grep -iv parameter | head -40; echo ----; ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls *.png; ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" | head -40

# openrua op 6
cd /workspace; for p in "258 238" "388 278" "330 347" "330 303" "200 400" "330 240"; do echo -n "px $p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 7
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png; ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:"; python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
# table depth
print('table', d[400,200], 'pot1', d[238,258], 'pot2', d[278,388], 'stove', d[347,330])
"

# openrua op 8
cd /workspace; python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
# camera is 2.1m above table at ~z=0.9 → camera z≈3.0; pixel scale at table: 2.1/579 = 3.6mm/px
mask = d < 2.09  # above table by >1cm
import scipy.ndimage as ndi
lab, n = ndi.label(mask)
for i in range(1,n+1):
    ys,xs = np.where(lab==i)
    if len(ys)<20: continue
    print(i, 'n',len(ys),'u',xs.min(),xs.max(),'v',ys.min(),ys.max(),'centroid',xs.mean().round(1),ys.mean().round(1),'mindepth',d[lab==i].min().round(3))
"

# openrua op 9
cd /workspace; python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
reg = d[220:260, 230:285]
np.set_printoptions(linewidth=250, precision=2)
# print depth relative to camera: heights above table (table depth 2.10)
h = (2.10 - reg)
h[h<0.005]=0
for row in range(0,40,2):
    print(220+row, ' '.join(f'{v*100:3.0f}' for v in h[row,::2]))
"

# openrua op 10
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py /frontview/depth/image_raw frontview_depth.png; python3 -c "
import numpy as np
d=np.load('frontview_depth.npy')
np.set_printoptions(linewidth=250, precision=2)
reg = d[290:400, 380:480]
bg = np.median(d[290:400, 480:520])
print('bg',bg)
for row in range(0,110,3):
    r=reg[row]
    print(290+row, ''.join('#' if v<bg-0.05 else '.' for v in r))
"

# openrua op 11
cd /workspace; ros2 topic echo /frontview/color/camera_info --once | grep -A3 "^k:" | head -4; python3 -c "
import numpy as np
d=np.load('frontview_depth.npy')
fx=None
import subprocess
bg=1.3368
for row in [311,317,323,329,335,341,347,353,356,360]:
    r=d[row]
    cols=[u for u in range(400,480) if r[u]<bg-0.03]
    if not cols: continue
    # find contiguous main body segment: largest run
    runs=[];s=cols[0];p=cols[0]
    for u in cols[1:]:
        if u!=p+1: runs.append((s,p)); s=u
        p=u
    runs.append((s,p))
    a,b=max(runs,key=lambda t:t[1]-t[0])
    za,zb=r[a],r[b]
    print(row, a,b,'px',b-a+1,'depth',round(float(za),3),round(float(zb),3), 'width_m ~', round((b-a+1)*(za+zb)/2/579.4,3))
"

# openrua op 12
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw sideview_depth.png; ros2 topic echo /sideview/color/camera_info --once | grep -A1 "^k:" | tail -1; python3 -c "
import numpy as np
d=np.load('sideview_depth.npy')
for row in range(280,375,3):
    r=d[row]
    seg=r[280:350]
    bg=np.median(r[380:420])
    cols=[u for u in range(280,350) if r[u]<bg-0.15]
    line=''.join('#' if r[u]<bg-0.15 else '.' for u in range(280,350))
    if cols:
        a,b=cols[0],cols[-1]
        print(row,line,a,b,round(float(r[a]),3),round(float(r[b]),3),'w~',round((b-a+1)*(r[a]+r[b])/2/579.4,3))
    else: print(row,line)
"

# openrua op 13
cd /workspace; python3 -c "
import numpy as np
d=np.load('sideview_depth.npy')
for row in range(334,380,3):
    r=d[row]
    print(row, ' '.join(f'{v:.2f}' for v in r[290:345:2]))
"

# openrua op 14
cd /workspace; timeout 120 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 position | tail -2

# openrua op 15
cd /workspace; python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
for name,(v0,v1,u0,u1) in {'pot1':(222,258,232,290),'pot2':(258,296,356,412)}.items():
    reg=d[v0:v1,u0:u1]
    h=2.10-reg
    print(name)
    for row in range(reg.shape[0]):
        print(v0+row, ''.join('#' if x>0.10 else ('+' if x>0.02 else '.') for x in h[row]))
"

# openrua op 16
cd /workspace; timeout 30 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A2 Translation | head -3; timeout 15 ros2 topic echo /tf --once 2>/dev/null | grep -E "frame_id" | sort | uniq -c; ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A9 "^k:" | tr -d '\n'; echo; ros2 interface show moveit_msgs/srv/GetPositionFK | head -30

# openrua op 17
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library for this Panda: joint state, FK/IK, trajectories,
gripper, servo bursts, camera grabs. One rclpy node per process."""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import CameraInfo, Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (TF)
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def down_quat(yaw):
    """Hand Z pointing down (-world Z), hand X rotated by yaw about world Z.
    yaw=0 -> hand X along +world X (fingers close along world Y)."""
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    Rflip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # 180 deg about X
    return R_to_quat(Rz @ Rflip)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.bridge = CvBridge()

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---- sensing ----
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def fk_world(self, q=None, link="panda_hand"):
        """Return (pos_world, quat xyzw) of link for arm config q."""
        if q is None:
            q = self.arm_q()
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])

    def tcp_world(self, q=None):
        pos, quat = self.fk_world(q)
        R = quat_to_R(*quat)
        return pos + TCP * R[:, 2], quat

    def grab(self, topic, msg_type=Image, timeout=30.0):
        got = {}
        sub = self.node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
        t0 = time.time()
        while "m" not in got and time.time() - t0 < timeout:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        if "m" not in got:
            raise RuntimeError(f"no msg on {topic}")
        return got["m"]

    def color(self, cam):
        return self.bridge.imgmsg_to_cv2(self.grab(f"/{cam}/color/image_raw"), "bgr8")

    def depth(self, cam):
        return self.bridge.imgmsg_to_cv2(self.grab(f"/{cam}/depth/image_raw"), "passthrough")

    def K(self, cam):
        return np.array(self.grab(f"/{cam}/color/camera_info", CameraInfo).k).reshape(3, 3)

    # ---- acting ----
    def ik_world(self, pos_world, quat, seed=None, tcp=False, tries=1):
        """IK for the hand (or TCP if tcp=True) at a world pose; returns q or None."""
        pos = np.array(pos_world, float)
        if tcp:
            pos = pos - TCP * quat_to_R(*quat)[:, 2]
        pos = pos - BASE_IN_WORLD
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in (seed or self.arm_q())]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        for _ in range(tries):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def move_q(self, q_or_list, seconds, wait=True):
        """Send one or several waypoints (list of q) as one trajectory."""
        pts = q_or_list if isinstance(q_or_list[0], (list, tuple, np.ndarray)) else [q_or_list]
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        n = len(pts)
        for i, q in enumerate(pts):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            t = seconds * (i + 1) / n
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q_now = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q_now, pts[-1]))
        print(f"[move] error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        self.grip.wait_for_server(10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"[grip] reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def servo(self, v, ticks, w=(0, 0, 0)):
        """Stream a base-frame twist (m/s) for `ticks` messages."""
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, w)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)
OPENRUA_EOF

# openrua op 18
cd /workspace; timeout 120 python3 -c "
from rob import *
r=Robot()
q=r.arm_q(); print('q',np.round(q,3))
pos,quat=r.fk_world(q); print('hand world',np.round(pos,4),np.round(quat,3))
print('tcp',np.round(r.tcp_world(q)[0],4))
print('gap',r.finger_gap())
# test IK feasibility at candidate poses (hand pointing down), tcp positions
for name,p in [('pot1_top',(-0.207,-0.208,1.10)),('pot2_top',(-0.0725,0.228,1.10)),('stove_c_hi',(0.182,0.036,1.05)),('stove_near',(0.12,0.036,1.05)),('stove_c_lo',(0.182,0.036,1.0)),('stove_far',(0.24,0.036,1.05))]:
    for yaw in [0.0, math.pi/2]:
        sol=r.ik_world(p, down_quat(yaw), tcp=True, tries=3)
        print(name, 'yaw',round(yaw,2), None if sol is None else np.round(sol,3))
"

# openrua op 19
cd /workspace; timeout 120 python3 -c "
from rob import *
r=Robot()
q=r.arm_q()
pos,quat=r.fk_world(q); pos=pos-BASE_IN_WORLD
print('hand (FK raw)',np.round(pos,4),np.round(quat,3))
# try IK at raw FK pose (i.e. world coords) => ik_world subtracts base, so pass pos+BASE to get raw
for label,p in [('raw as-is', pos+BASE_IN_WORLD), ('raw minus base', pos)]:
    sol=r.ik_world(p, quat, tries=2)
    print(label, None if sol is None else np.round(sol,3))
"

# openrua op 20
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 (TF)", "new_string": "# Verified empirically: /compute_fk and /compute_ik poses are already in\\n# the same frame as px2world\'s \\"world\\" (FK of the current config matches\\n# TF world->panda_hand), so no base offset is applied.\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 21
cd /workspace; timeout 300 python3 -c "
from rob import *
r=Robot()
for name,p in [('pot1_top',(-0.207,-0.208,1.10)),('pot2_top',(-0.0725,0.228,1.10)),('stove_c_hi',(0.182,0.036,1.05)),('stove_near',(0.12,0.036,1.05)),('stove_c_lo',(0.182,0.036,1.0)),('stove_far',(0.24,0.036,1.05)),('stove_c_hi2',(0.182,0.036,1.10))]:
    for yaw in [0.0, math.pi/2]:
        sol=r.ik_world(p, down_quat(yaw), tcp=True, tries=3)
        print(name, 'yaw',round(yaw,2), None if sol is None else np.round(sol,3), flush=True)
"

# openrua op 22
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
q=r.ik_world((-0.203,-0.20,1.30), down_quat(math.pi/2), tcp=True, tries=3)
print('sol',np.round(q,3))
r.move_q(q, 4.0)
print('tcp now',np.round(r.tcp_world()[0],4))
import cv2
cv2.imwrite('hand1.png', r.color('robot0_eye_in_hand'))
np.save('hand1_depth.npy', r.depth('robot0_eye_in_hand'))
" 2>&1 | grep -v Warn

# openrua op 23
cd /workspace; timeout 120 python3 -c "
from rob import *
r=Robot(); print(np.round(r.arm_q(),3)); print(r.finger_gap())
"

# openrua op 24
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
q=[0.034,-0.204,-0.416,-2.122,-0.088,1.933,-1.914]
r.move_q(q, 3.0)
print(np.round(r.arm_q(),3))
" 2>&1 | grep -v Warn

# openrua op 25
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
q=[0.034,-0.204,-0.416,-2.122,-0.088,1.933,-1.914]
r.move_q(q, 8.0)
print(np.round(r.arm_q(),3))
" 2>&1 | grep -v Warn

# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        pts = q_or_list if isinstance(q_or_list[0], (list, tuple, np.ndarray)) else [q_or_list]\\n        self.fjt.wait_for_server(10)", "new_string": "        pts = q_or_list if isinstance(q_or_list[0], (list, tuple, np.ndarray)) else [q_or_list]\\n        # this controller tracks at most ~0.17 rad/s per joint (measured);\\n        # stretch the duration so the goal is reachable in time\\n        q0 = np.array(self.arm_q())\\n        dmax = 0.0\\n        for q in pts:\\n            dmax += float(np.max(np.abs(np.array(q) - q0)))\\n            q0 = np.array(q)\\n        seconds = max(seconds, dmax / 0.12)\\n        self.fjt.wait_for_server(10)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
q=[0.034,-0.204,-0.416,-2.122,-0.088,1.933,-1.914]
r.move_q(q, 2.0)
print(np.round(r.arm_q(),3))
pos,quat=r.tcp_world(); print('tcp',np.round(pos,4), 'R', np.round(quat_to_R(*quat),2))
import cv2
cv2.imwrite('hand1.png', r.color('robot0_eye_in_hand'))
np.save('hand1_depth.npy', r.depth('robot0_eye_in_hand'))
" 2>&1 | grep -v Warn

# openrua op 28
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.ik_link_name = \\"panda_hand\\"  # group tip is link8 (45 deg off)\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 29
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
q=r.ik_world((-0.203,-0.20,1.30), down_quat(math.pi/2), tcp=True, tries=3)
print('sol',np.round(q,3))
pos,quat=r.tcp_world(q); print('predicted tcp',np.round(pos,4), 'R', np.round(quat_to_R(*quat),2))
r.move_q(q, 2.0)
pos,quat=r.tcp_world(); print('tcp',np.round(pos,4), 'R', np.round(quat_to_R(*quat),2))
import cv2
cv2.imwrite('hand1.png', r.color('robot0_eye_in_hand'))
np.save('hand1_depth.npy', r.depth('robot0_eye_in_hand'))
" 2>&1 | grep -v Warn

# openrua op 30
cd /workspace; python3 -c "
import numpy as np
d=np.load('hand1_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
# camera z: hand at 1.30+0.1034=1.4034, camera ~ same height -> table depth should be ~0.50
print('center depth',d[240,320], 'table sample', d[200,200], d[100,400])
reg=d[230:350,270:370]
tbl=0.503
for row in range(0,120,3):
    print(230+row, ''.join('#' if (tbl-x)>0.12 else ('+' if (tbl-x)>0.06 else ('-' if (tbl-x)>0.01 else '.')) for x in reg[row]))
"

# openrua op 31
cd /workspace; python3 -c "
import numpy as np
d=np.load('hand1_depth.npy')
reg=d[230:350,270:370]
np.set_printoptions(linewidth=300)
for row in range(24,80,4):
    print(230+row, ' '.join(f'{(0.503-x)*100:4.1f}' for x in reg[row,14:90:3]))
"

# openrua op 32
cd /workspace; python3 -c "
import cv2
im=cv2.imread('hand1.png'); crop=im[220:360,250:400]; cv2.imwrite('hand1_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 33
cd /workspace; python3 -c "
import numpy as np
d=np.load('hand1_depth.npy')
tbl=0.503
H=tbl-d
mask=H>0.10
# horizontal extents for rows 270-300
for row in range(266,304,4):
    cols=np.where(mask[row])[0]; print('row',row,'cols',cols.min(),cols.max(),'w',cols.max()-cols.min()+1)
for col in range(300,344,4):
    rows=np.where(mask[:,col])[0]; print('col',col,'rows',rows.min(),rows.max(),'h',rows.max()-rows.min()+1)
"

# openrua op 34
cd /workspace; python3 -c "
import numpy as np, cv2
d=np.load('hand1_depth.npy'); im=cv2.imread('hand1.png')
H=0.503-d
for row in range(252,316,2):
    a=''.join('#' if H[row,c]>0.10 else '.' for c in range(284,360))
    g=im[row,284:360].mean(axis=1)
    b=''.join('#' if v>150 else ('+' if v>110 else '.') for v in g)
    print(row,a,' ',b)
"

# openrua op 35
cd /workspace; python3 -c "
import numpy as np, cv2
d=np.load('hand1_depth.npy'); im=cv2.imread('hand1.png')
H=0.503-d
for row in range(248,264,1):
    print(row,' '.join(f'{H[row,c]*100:4.1f}' for c in range(292,352,3)))
print()
for row in range(300,316,1):
    print(row,' '.join(f'{H[row,c]*100:4.1f}' for c in range(292,352,3)))
"

# openrua op 36
find / -iname "*moka*" 2>/dev/null | head; find / -iname "*.xml" -path "*libero*" 2>/dev/null | head -5; find / -iname "*stove*" 2>/dev/null | head -5; pip list 2>/dev/null | grep -i -E "robosuite|libero|mujoco"

# openrua op 37
cd /workspace; cat >> rob.py <<'EOF'


def pitched_quat(p, yaw=0.0):
    """Hand approach axis tilted by p rad from straight-down toward the
    +X direction (rotated by yaw about world Z); fingers close along the
    horizontal axis perpendicular to the approach. p=0 == down_quat(yaw)."""
    Z = np.array([math.sin(p), 0.0, -math.cos(p)])
    Y = np.array([0.0, -1.0, 0.0])
    X = np.cross(Y, Z)
    R = np.column_stack([X, Y, Z])
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return R_to_quat(Rz @ R)
EOF
timeout 600 python3 -u -c "
from rob import *
r=Robot()
P1=(-0.2026,-0.2004); P2=(-0.0725,0.228)
tests={'pick1':(P1[0],P1[1],0.93),'pick2':(P2[0],P2[1],0.93),'pre1':(P1[0]-0.08,P1[1],1.01),
 'placeA':(0.13,0.036,0.965),'placeB':(0.23,0.036,0.965),'placeA_hi':(0.13,0.036,1.06),'placeB_hi':(0.23,0.036,1.06),'placeA2':(0.13,-0.005,0.965),'placeB2':(0.13,0.077,0.965)}
for p in [math.radians(45), math.radians(60), math.radians(30)]:
    for name,pos in tests.items():
        sol=r.ik_world(pos, pitched_quat(p), tcp=True, tries=3)
        print(f'pitch {math.degrees(p):.0f} {name:10s}', None if sol is None else np.round(sol,2), flush=True)
" 2>&1 | grep -v Warn

# openrua op 38
cd /workspace; cat >> rob.py <<'EOF'


def nice_seed(pos_world):
    """Elbow-up ready pose facing the target azimuth (base at x=-0.66)."""
    az = math.atan2(pos_world[1], pos_world[0] + 0.66)
    return [az, 0.2, 0.0, -2.0, 0.0, 2.2, 0.785 + az]


def ik_nice(r, pos_world, quat, tcp=True, tries=6):
    """IK from several seeds around the nominal ready pose; returns the
    solution closest to the nominal seed (avoids wrapped-around configs)."""
    seed0 = nice_seed(pos_world)
    best = None
    rng = np.random.default_rng(0)
    for i in range(tries):
        seed = seed0 if i == 0 else list(np.array(seed0) + rng.normal(0, 0.3, 7))
        sol = r.ik_world(pos_world, quat, seed=seed, tcp=tcp, tries=1)
        if sol is None:
            continue
        d = float(np.linalg.norm(np.array(sol) - np.array(seed0)))
        if best is None or d < best[0]:
            best = (d, sol)
    return None if best is None else best[1]
EOF
timeout 900 python3 -u -c "
from rob import *
r=Robot()
P1=(-0.2026,-0.2004); P2=(-0.0725,0.228)
tests={'pick1':(P1[0],P1[1],0.93),'pick2':(P2[0],P2[1],0.93),'pre1':(P1[0]-0.08,P1[1],1.01),'pre2':(P2[0]-0.08,P2[1],1.01),
 'placeA':(0.13,0.036,0.965),'placeB':(0.23,0.036,0.965),'placeA_hi':(0.13,0.036,1.06),'placeB_hi':(0.23,0.036,1.06),'carry':(0.0,0.0,1.10)}
for p in [math.radians(45), math.radians(60)]:
    for name,pos in tests.items():
        sol=ik_nice(r,pos, pitched_quat(p))
        print(f'pitch {math.degrees(p):.0f} {name:10s}', None if sol is None else np.round(sol,2), flush=True)
" 2>&1 | grep -v Warn

# openrua op 39
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
q=r.ik_world((-0.0725,0.228,1.30), down_quat(math.pi/2), tcp=True, tries=3)
print('sol',np.round(q,3))
r.move_q(q, 3.0)
pos,quat=r.tcp_world(); print('tcp',np.round(pos,4))
import cv2
cv2.imwrite('hand2.png', r.color('robot0_eye_in_hand'))
d=r.depth('robot0_eye_in_hand'); np.save('hand2_depth.npy', d)
tbl=np.median(d[100:140,100:140]); print('table depth',tbl)
H=tbl-d; m=H>0.10
ys,xs=np.where(m[150:400,200:450]); ys+=150; xs+=200
print('mask bbox',xs.min(),xs.max(),ys.min(),ys.max())
# lid center: use widest rows
w=[(row, np.where(m[row,200:450])[0]) for row in range(150,400)]
w=[(row,c.min()+200,c.max()+200) for row,c in w if len(c)>0]
best=max(w,key=lambda t:t[2]-t[1]); print('widest row',best)
" 2>&1 | grep -v Warn

# openrua op 40
cd /workspace; python3 -c "
import numpy as np, cv2
d=np.load('hand2_depth.npy'); H=0.503-d
im=cv2.imread('hand2.png'); crop=im[200:360,240:400]; cv2.imwrite('hand2_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
for row in range(220,330,3):
    print(row, ''.join('#' if H[row,c]>0.10 else ('+' if H[row,c]>0.03 else '.') for c in range(260,390)))
"

# openrua op 41
cd /workspace; cat >> rob.py <<'EOF'


def line_move(r, p_from, p_to, quat, n, seconds, seed=None):
    """Straight Cartesian TCP line as one multi-waypoint trajectory."""
    pts = []
    q_prev = seed or r.arm_q()
    for i in range(1, n + 1):
        p = np.array(p_from) + (np.array(p_to) - np.array(p_from)) * i / n
        q = r.ik_world(p, quat, seed=q_prev, tcp=True, tries=3)
        if q is None:
            raise RuntimeError(f"IK failed at waypoint {i}: {p}")
        if np.max(np.abs(np.array(q) - np.array(q_prev))) > 1.0:
            raise RuntimeError(f"IK branch jump at waypoint {i}: {np.round(q,2)} vs {np.round(q_prev,2)}")
        pts.append(q)
        q_prev = q
    return r.move_q(pts, seconds)
EOF
timeout 900 python3 -u -c "
from rob import *
import cv2
r=Robot()
P2=np.array([-0.0746,0.2351,0.93])
p=math.radians(45); quat=pitched_quat(p); Z=quat_to_R(*quat)[:,2]; print('Z_h',np.round(Z,3))
pre=P2-0.10*Z; print('pre',np.round(pre,4))
q=ik_nice(r,pre,quat); print('q_pre',np.round(q,3))
r.move_q(q,4.0)
pos,qq=r.tcp_world(); print('tcp',np.round(pos,4),'R',np.round(quat_to_R(*qq),2))
print('gap',r.finger_gap())
cv2.imwrite('hand_pre2.png', r.color('robot0_eye_in_hand'))
" 2>&1 | grep -v Warn

# openrua op 42
cd /workspace; timeout 120 python3 tools/perception/cam_snap.py agentview agent_now.png >/dev/null; echo ok

# openrua op 43
cd /workspace; cat >> rob.py <<'EOF'


LIMITS = np.array(FJT["limits_rad"])


def margin_ok(q, m=0.12):
    q = np.array(q)
    return bool(np.all(q > LIMITS[:, 0] + m) and np.all(q < LIMITS[:, 1] - m))


def pitched_quat2(p, flip=False, yaw=0.0):
    """Like pitched_quat; flip=True rotates the hand 180 deg about its
    approach axis (same grasp, fingers swapped, other IK branches)."""
    Z = np.array([math.sin(p), 0.0, -math.cos(p)])
    Y = np.array([0.0, 1.0 if flip else -1.0, 0.0])
    X = np.cross(Y, Z)
    R = np.column_stack([X, Y, Z])
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    return R_to_quat(Rz @ R)


def ik_best(r, pos_world, quat, seed0=None, tries=8, tcp=True):
    """Several seeds; keep solutions with joint-limit margin; return the one
    closest to seed0 (default: nice_seed)."""
    seed0 = np.array(seed0 if seed0 is not None else nice_seed(pos_world))
    rng = np.random.default_rng(1)
    best = None
    for i in range(tries):
        seed = seed0 if i == 0 else seed0 + rng.normal(0, 0.4, 7)
        seed = np.clip(seed, LIMITS[:, 0] + 0.1, LIMITS[:, 1] - 0.1)
        sol = r.ik_world(pos_world, quat, seed=list(seed), tcp=tcp, tries=1)
        if sol is None or not margin_ok(sol):
            continue
        d = float(np.linalg.norm(np.array(sol) - seed0))
        if best is None or d < best[0]:
            best = (d, sol)
    return None if best is None else best[1]
EOF
timeout 1200 python3 -u -c "
from rob import *
r=Robot()
P1=np.array([-0.202,-0.201,0.93]); P2=np.array([-0.0746,0.2351,0.93])
A=np.array([0.138,0.036,0.963]); B=np.array([0.228,0.036,0.963])
p=math.radians(45)
for flip in [False, True]:
    quat=pitched_quat2(p, flip); Z=quat_to_R(*quat)[:,2]
    for name,pos in [('pick2',P2),('pre2',P2-0.10*Z),('lift2',P2+[0,0,0.15]),('placeB',B),('preB',B-0.10*Z),('hiB',B-0.10*Z+[0,0,0.10]),('pick1',P1),('pre1',P1-0.10*Z),('lift1',P1+[0,0,0.15]),('placeA',A),('preA',A-0.10*Z),('hiA',A-0.10*Z+[0,0,0.10]),('carry',(0.0,0.0,1.15))]:
        sol=ik_best(r,pos,quat)
        print(f'flip={flip} {name:7s}', None if sol is None else np.round(sol,2), flush=True)
" 2>&1 | grep -v Warn

# openrua op 44
cd /workspace; timeout 1200 python3 -u -c "
from rob import *
r=Robot()
P1=np.array([-0.202,-0.201,0.93]); A=np.array([0.138,0.036,0.963])
p=math.radians(45); quat=pitched_quat2(p, True); Z=quat_to_R(*quat)[:,2]
seedA=[0.02,1.08,-0.15,-1.29,-1.41,3.27,-0.89]
for name,pos in [('pick1',P1),('pre1',P1-0.10*Z),('lift1',P1+[0,0,0.15]),('mid',(-0.05,-0.1,1.10)),('hiA',A-0.10*Z+[0,0,0.10])]:
    sol=ik_best(r,pos,quat,seed0=seedA,tries=12)
    print(f'{name:7s}', None if sol is None else np.round(sol,2), flush=True)
" 2>&1 | grep -v Warn

# openrua op 45
cd /workspace; timeout 1200 python3 -u -c "
from rob import *
r=Robot()
P1=np.array([-0.202,-0.201,0.93]); A=np.array([0.138,0.036,0.963])
p=math.radians(45); quat=pitched_quat2(p, True); Z=quat_to_R(*quat)[:,2]
seedL=[-0.35,0.08,-0.04,-2.49,2.13,2.82,1.5]
for name,pos in [('lift1',P1+[0,0,0.15]),('mid1',(-0.10,-0.10,1.12)),('mid2',(0.0,0.0,1.12)),('hiA',A-0.10*Z+[0,0,0.10]),('preA',A-0.10*Z),('placeA',A)]:
    sol=ik_best(r,pos,quat,seed0=seedL,tries=12)
    print(f'{name:7s}', None if sol is None else np.round(sol,2), flush=True)
    if sol is not None: seedL=sol
" 2>&1 | grep -v Warn

# openrua op 46
cd /workspace; timeout 1200 python3 -u -c "
from rob import *
r=Robot()
P2=np.array([-0.0746,0.2351,0.93]); B=np.array([0.228,0.036,0.963])
p=math.radians(45); quat=pitched_quat2(p, True); Z=quat_to_R(*quat)[:,2]
seed=[0.28,0.37,0.05,-2.09,-1.9,2.88,-0.23]
for name,pos in [('lift2',P2+[0,0,0.15]),('mid1',(0.0,0.15,1.12)),('mid2',(0.08,0.06,1.12)),('hiB',B-0.10*Z+[0,0,0.10]),('preB',B-0.10*Z),('placeB',B)]:
    sol=ik_best(r,pos,quat,seed0=seed,tries=12)
    print(f'{name:7s}', None if sol is None else np.round(sol,2), flush=True)
    if sol is not None: seed=sol
" 2>&1 | grep -v Warn

# openrua op 47
cd /workspace; timeout 1500 python3 -u -c "
from rob import *
import cv2
r=Robot()
print('q now',np.round(r.arm_q(),3))
hover=[0.256,0.224,0.128,-1.623,-0.029,1.845,-0.396]
r.move_q(hover,4.0)
P2=np.array([-0.0746,0.2351,0.93])
quat=pitched_quat2(math.radians(45), True); Z=quat_to_R(*quat)[:,2]
pre=P2-0.10*Z
q_pre=ik_best(r,pre,quat,seed0=[0.45,0.42,-0.02,-2.22,-2.24,2.77,0.24]); print('q_pre',np.round(q_pre,3))
q_hi=ik_best(r,pre+[0,0,0.12],quat,seed0=q_pre); print('q_hi',np.round(q_hi,3))
r.move_q(q_hi,5.0)
line_move(r,pre+[0,0,0.12],pre,quat,3,3.0,seed=q_hi)
pos,qq=r.tcp_world(); print('tcp',np.round(pos,4),'want',np.round(pre,4),'R',np.round(quat_to_R(*qq),2))
print('gap',r.finger_gap())
cv2.imwrite('hand_pre2.png', r.color('robot0_eye_in_hand'))
" 2>&1 | grep -v Warn; timeout 120 python3 tools/perception/cam_snap.py agentview agent_now.png >/dev/null

# openrua op 48
cd /workspace; timeout 1500 python3 -u -c "
from rob import *
import cv2
r=Robot()
P2=np.array([-0.0746,0.2351,0.93])
quat=pitched_quat2(math.radians(45), True); Z=quat_to_R(*quat)[:,2]
pre=P2-0.10*Z
line_move(r,pre,P2-0.03*Z,quat,3,3.0)
pos,qq=r.tcp_world(); print('tcp',np.round(pos,4),'want',np.round(P2-0.03*Z,4))
print('gap',r.finger_gap())
cv2.imwrite('hand_app2.png', r.color('robot0_eye_in_hand'))
" 2>&1 | grep -v Warn; timeout 120 python3 tools/perception/cam_snap.py agentview agent_now.png >/dev/null

# openrua op 49
cd /workspace; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A7 wrench; timeout 600 python3 -u -c "
from rob import *
r=Robot()
for l in ['panda_hand','panda_leftfinger','panda_rightfinger','panda_link8']:
    print(l, np.round(r.fk_world(link=l)[0],4))
print('tcp',np.round(r.tcp_world()[0],4))
" 2>&1 | grep -v Warn

# openrua op 50
cd /workspace; timeout 900 python3 -u -c "
from rob import *
from geometry_msgs.msg import WrenchStamped
r=Robot()
def wrench():
    m=r.grab('/franka_robot_state_broadcaster/external_wrench', WrenchStamped)
    f=m.wrench.force; t=m.wrench.torque
    return np.round([f.x,f.y,f.z,t.x,t.y,t.z],2)
print('wrench now',wrench())
P2=np.array([-0.0746,0.2351,0.93])
quat=pitched_quat2(math.radians(45), True); Z=quat_to_R(*quat)[:,2]
cur=r.tcp_world()[0]
line_move(r,cur,P2-0.06*Z,quat,2,2.0)
print('tcp',np.round(r.tcp_world()[0],4),'wrench',wrench())
line_move(r,P2-0.06*Z,P2-0.03*Z,quat,2,2.0)
print('tcp',np.round(r.tcp_world()[0],4),'want',np.round(P2-0.03*Z,4),'wrench',wrench())
" 2>&1 | grep -v Warn

# openrua op 51
cd /workspace; for c in sideview frontview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c ${c}_now.png >/dev/null & done; wait; python3 -c "
import cv2
im=cv2.imread('sideview_now.png'); cv2.imwrite('side_crop.png', cv2.resize(im[150:330,250:450],None,fx=3,fy=3,interpolation=cv2.INTER_LINEAR))
im=cv2.imread('frontview_now.png'); cv2.imwrite('front_crop.png', cv2.resize(im[180:400,330:530],None,fx=3,fy=3,interpolation=cv2.INTER_LINEAR))
"

# openrua op 52
cd /workspace; timeout 60 python3 -c "
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n=rclpy.create_node('urdf')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<20: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0] if got else '')
print(len(got[0]) if got else 'none')
"; grep -n -A12 'link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A14 'link name="panda_leftfinger"' robot.urdf | head -50

# openrua op 53
cd /workspace; python3 -c "
import numpy as np, struct
def stl_bounds(fn):
    data=open(fn,'rb').read()
    n=struct.unpack('<I',data[80:84])[0]
    arr=np.frombuffer(data[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
    v=arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ['hand','finger']:
    lo,hi=stl_bounds(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl')
    print(f, 'min',np.round(lo,4),'max',np.round(hi,4))
"

# openrua op 54
find / -iname "hand*.stl" -o -iname "finger*.stl" -o -iname "hand*.dae" 2>/dev/null | head

# openrua op 55
cd /workspace; timeout 300 python3 -u -c "
from rob import *
from tf2_ros import Buffer, TransformListener
r=Robot()
tfbuf=Buffer(); TransformListener(tfbuf, r.node)
import time
t0=time.time()
while time.time()-t0<10 and not tfbuf.can_transform('world','sideview_optical_frame',rclpy.time.Time()): r.spin(0.2)
t=tfbuf.lookup_transform('world','sideview_optical_frame',rclpy.time.Time())
q=t.transform.rotation; R=quat_to_R(q.x,q.y,q.z,q.w); T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
print('cam pos',np.round(T,3))
d=r.depth('sideview'); K=r.K('sideview'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
np.save('side_depth_now.npy',d)
# hand pose
hp,hq=r.fk_world(); Rh=quat_to_R(*hq)
print('hand',np.round(hp,3))
# all pixels -> world, keep those within 0.16 m of hand origin and y within +-0.12 of hand y
vs,us=np.mgrid[0:480,0:640]
z=d
X=(us-cx)*z/fx; Y=(vs-cy)*z/fy
P=np.stack([X,Y,z],-1).reshape(-1,3)@R.T+T
P=P.reshape(480,640,3)
rel=P-hp
dist=np.linalg.norm(rel,axis=-1)
m=(dist<0.16)&(np.abs(rel[...,1])<0.13)
pts=rel[m]
loc=pts@Rh  # coords in hand frame (x,y,z)
print('n',len(pts))
print('hand-frame extents: X',np.round(loc[:,0].min(),3),np.round(loc[:,0].max(),3),' Y',np.round(loc[:,1].min(),3),np.round(loc[:,1].max(),3),' Z',np.round(loc[:,2].min(),3),np.round(loc[:,2].max(),3))
# X extent as function of Z bins
for z0 in np.arange(-0.04,0.12,0.01):
    sel=(loc[:,2]>=z0)&(loc[:,2]<z0+0.01)
    if sel.sum()>0: print(f'Z {z0:+.2f}..{z0+0.01:+.2f}: X {loc[sel,0].min():+.3f} .. {loc[sel,0].max():+.3f}  n={sel.sum()}')
" 2>&1 | grep -v Warn

# openrua op 56
cd /workspace; timeout 300 python3 -u -c "
from rob import *
from geometry_msgs.msg import WrenchStamped
r=Robot()
q=r.arm_q(); p,qt=r.tcp_world(); R=quat_to_R(*qt)
print('tcp',np.round(p,4),'Z',np.round(R[:,2],3))
back=p-0.09*R[:,2]
res=line_move(r, p, back, qt, 4, 5.0, seed=q)
p2,_=r.tcp_world(); print('tcp now',np.round(p2,4))
w=r.grab('/franka_robot_state_broadcaster/external_wrench', WrenchStamped, 5).wrench
print('wrench',round(w.force.x,2),round(w.force.y,2),round(w.force.z,2))
print('q',np.round(r.arm_q(),3), 'gap',round(r.finger_gap(),4))
" 2>&1 | grep -v Warn

# openrua op 57
cd /workspace; timeout 300 python3 -u -c "
from rob import *
from tf2_ros import Buffer, TransformListener
r=Robot()
tfbuf=Buffer(); TransformListener(tfbuf, r.node)
import time
t0=time.time()
while time.time()-t0<10 and not tfbuf.can_transform('world','birdview_optical_frame',rclpy.time.Time()): r.spin(0.2)
t=tfbuf.lookup_transform('world','birdview_optical_frame',rclpy.time.Time())
q=t.transform.rotation; R=quat_to_R(q.x,q.y,q.z,q.w); T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
d=r.depth('birdview'); K=r.K('birdview'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
vs,us=np.mgrid[0:480,0:640]
P=(np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1).reshape(-1,3)@R.T+T).reshape(480,640,3)
for name,c in [('pot1',(-0.202,-0.201)),('pot2',(-0.0746,0.2351))]:
    m=(np.abs(P[...,0]-c[0])<0.07)&(np.abs(P[...,1]-c[1])<0.07)&(P[...,2]>1.025)
    pts=P[m]; print(name,'n',m.sum(),'lid-top centroid',np.round(pts[:,:2].mean(0),4),'zmax',round(pts[:,2].max(),3))
    m2=(np.abs(P[...,0]-c[0])<0.07)&(np.abs(P[...,1]-c[1])<0.07)&(P[...,2]>1.045)
    if m2.sum(): print('   knob centroid',np.round(P[m2][:,:2].mean(0),4), m2.sum())
" 2>&1 | grep -v Warn

# openrua op 58
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
q0=r.arm_q()
def test(name,pos,p_deg,yaw_deg,seed=None):
    qt=pitched_quat2(math.radians(p_deg),False,math.radians(yaw_deg))
    sol=ik_best(r,np.array(pos),qt,seed0=seed,tries=10,tcp=True)
    if sol is None: print(f'{name:28s} p={p_deg} yaw={yaw_deg}: NO IK'); return None
    print(f'{name:28s} p={p_deg} yaw={yaw_deg}: q={np.round(sol,2)}')
    return sol
P2=np.array([-0.069,0.221,0.935]); P1=np.array([-0.202,-0.201,0.935])
for p in (80,):
    for yaw in (0,):
        Z=quat_to_R(*pitched_quat2(math.radians(p),False,math.radians(yaw)))[:,2]
        s=test('pot2 pre',P2-0.10*Z,p,yaw)
        s=test('pot2 grasp',P2,p,yaw,s)
        s=test('pot2 lift',P2+[0,0,0.15],p,yaw,s)
for p in (80,):
    for yaw in (0,-45):
        Z=quat_to_R(*pitched_quat2(math.radians(p),False,math.radians(yaw)))[:,2]
        s=test('pot1 pre',P1-0.10*Z,p,yaw)
        s=test('pot1 grasp',P1,p,yaw,s)
        s=test('pot1 lift',P1+[0,0,0.15],p,yaw,s)
# placements
for name,pos,yaw in [('B',(0.23,0.06),0),('B',(0.22,0.065),45),('A',(0.15,-0.01),0),('A',(0.14,0.09),0),('A',(0.14,0.0),-45),('A2',(0.16,0.0),0),('B2',(0.24,0.07),0)]:
    for p in (80,):
        test(f'place {name} high',(pos[0],pos[1],1.10),p,yaw)
        test(f'place {name} low',(pos[0],pos[1],0.98),p,yaw)
" 2>&1 | grep -v Warn

# openrua op 59
cd /workspace; timeout 900 python3 -u -c "
from rob import *
import itertools
r=Robot()
def search(pos,qt,az):
    sols=[]
    for a,b,c in itertools.product((0.2,0.5,0.8,1.1,1.4),(-1.0,-1.6,-2.2,-2.8),(1.2,1.8,2.4,3.0)):
        seed=np.array([az,a,0,b,0,c,0.785+az])
        s=r.ik_world(np.array(pos),qt,seed=seed,tcp=True)
        if s is not None and margin_ok(s,0.1): sols.append(s)
    if not sols: return None
    sols=np.array(sols)
    # niceness: small |j3|,|j5|, j1 near az
    score=np.abs(sols[:,2])+np.abs(sols[:,4])+np.abs(sols[:,0]-az)
    i=np.argmin(score); return sols[i], len(sols)
P2=np.array([-0.069,0.221,0.935]); P1=np.array([-0.202,-0.201,0.935])
tests=[('pot2 grasp',P2,80,0),('pot2 pre',None,80,0),('pot1 grasp',P1,80,0),('pot1 grasp',P1,80,-45),
       ('B (0.23,0.06,0.98)',(0.23,0.06,0.98),80,0),('B (0.23,0.06,1.08)',(0.23,0.06,1.08),80,0),
       ('A (0.15,-0.01,0.98)',(0.15,-0.01,0.98),80,0),('A (0.15,-0.01,1.08)',(0.15,-0.01,1.08),80,0),
       ('A (0.14,0.09,0.98)',(0.14,0.09,0.98),80,0),('A (0.14,0.0,0.98) y-45',(0.14,0.0,0.98),80,-45),
       ('B (0.22,0.065,0.98) y45',(0.22,0.065,0.98),80,45)]
for name,pos,p,yaw in tests:
    qt=pitched_quat2(math.radians(p),False,math.radians(yaw))
    if pos is None:
        Z=quat_to_R(*qt)[:,2]; pos=P2-0.10*Z
    az=math.atan2(pos[1],pos[0]+0.66)
    res=search(pos,qt,az)
    if res is None: print(f'{name:26s}: NO IK'); continue
    s,n=res; print(f'{name:26s}: n={n:3d} q={np.round(s,2)}')
" 2>&1 | grep -v Warn

# openrua op 60
cd /workspace; grep -n "seed or self.arm_q()" rob.py; sed -i 's/(seed or self.arm_q())/(seed if seed is not None else self.arm_q())/' rob.py; sed -n 150,175p rob.py

# openrua op 61
cd /workspace; cat > iksearch.py <<'EOF'
from rob import *
import itertools
def search(r,pos,qt,az=None,m=0.1):
    pos=np.array(pos,float)
    if az is None: az=math.atan2(pos[1],pos[0]+0.66)
    sols=[]
    for a,b,c in itertools.product((0.2,0.5,0.8,1.1,1.4),(-1.0,-1.6,-2.2,-2.8),(1.2,1.8,2.4,3.0)):
        seed=np.array([az,a,0,b,0,c,0.785+az])
        s=r.ik_world(pos,qt,seed=seed,tcp=True)
        if s is not None and margin_ok(s,m): sols.append(s)
    if not sols: return None
    sols=np.array(sols)
    score=np.abs(sols[:,2])+np.abs(sols[:,4])+np.abs(sols[:,0]-az)
    i=np.argmin(score); return sols[i], len(sols)
EOF
timeout 900 python3 -u -c "
from iksearch import *
r=Robot()
P2=np.array([-0.069,0.221,0.935]); P1=np.array([-0.202,-0.201,0.935])
tests=[('pot2 grasp',P2,80,0),('pot2 pre',None,80,0),('pot1 grasp',P1,80,0),('pot1 grasp',P1,80,-45),
       ('B (0.23,0.06,0.98)',(0.23,0.06,0.98),80,0),('B (0.23,0.06,1.08)',(0.23,0.06,1.08),80,0),
       ('A (0.15,-0.01,0.98)',(0.15,-0.01,0.98),80,0),('A (0.15,-0.01,1.08)',(0.15,-0.01,1.08),80,0),
       ('A (0.14,0.09,0.98)',(0.14,0.09,0.98),80,0),('A (0.14,0.0,0.98) y-45',(0.14,0.0,0.98),80,-45),
       ('B (0.22,0.065,0.98) y45',(0.22,0.065,0.98),80,45)]
for name,pos,p,yaw in tests:
    qt=pitched_quat2(math.radians(p),False,math.radians(yaw))
    if pos is None:
        Z=quat_to_R(*qt)[:,2]; pos=P2-0.10*Z
    res=search(r,pos,qt)
    if res is None: print(f'{name:26s}: NO IK'); continue
    s,n=res; print(f'{name:26s}: n={n:3d} q={np.round(s,2)}')
" 2>&1 | grep -v Warn

# openrua op 62
cd /workspace; grep -n "def fk_world" -A 22 rob.py

# openrua op 63
cd /workspace; cat >> rob.py <<'EOF'


LINKS = ["panda_link1", "panda_link2", "panda_link3", "panda_link4", "panda_link5",
         "panda_link6", "panda_link7", "panda_link8", "panda_hand"]


def links_world(r, q):
    """Positions of all arm link origins in world for config q -> dict."""
    r.fk.wait_for_service(10)
    req = GetPositionFK.Request()
    req.fk_link_names = list(LINKS)
    req.robot_state.joint_state.name = list(ARM)
    req.robot_state.joint_state.position = [float(v) for v in q]
    fut = r.fk.call_async(req)
    rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
    res = fut.result()
    out = {}
    for name, ps in zip(res.fk_link_names, res.pose_stamped):
        p = ps.pose.position
        out[name] = np.array([p.x, p.y, p.z]) + BASE_IN_WORLD
    return out


def arm_clearance(r, q, verbose=False):
    """Min z of link origins 3..hand and the hand-frame sanity; crude table check."""
    L = links_world(r, q)
    zs = {k: round(v[2], 3) for k, v in L.items()}
    if verbose:
        for k in LINKS:
            print(f"  {k:12s} {np.round(L[k],3)}")
    return min(L[k][2] for k in LINKS[2:])
EOF
timeout 600 python3 -u -c "
from iksearch import *
r=Robot()
for name,q in [('pot2 grasp',[0.44,0.78,0.04,-2.13,-2.64,1.94,-2.5]),('pot1 grasp',[-0.59,0.66,-0.05,-2.5,2.51,1.71,-2.35]),('pot1 grasp45',[-0.19,0.66,0,-2.51,-2.55,1.72,-2.34]),('B low',[0.28,1.46,-0.5,-0.65,-2.76,2.86,-2.19]),('B high',[0.35,1.47,-0.68,-0.94,0.19,3.65,1.33]),('A(0.14,0.09)',[0.68,1.17,-0.68,-1.44,-2.8,2.54,-1.96]),('B y45',[-0.11,1.42,0.17,-0.74,1.72,2.34,-1.43]),('current',r.arm_q())]:
    print(name); mz=arm_clearance(r,q,True); print('  min z',round(mz,3))
" 2>&1 | grep -v Warn

# openrua op 64
cd /workspace; cat >> rob.py <<'EOF'


def path_ik(r, p_from, p_to, q_from, q_to, n, seed, m=0.1, maxjump=0.6):
    """IK along a straight TCP line with quaternion slerp-ish (linear yaw), seeded consecutively.
    Returns list of q or raises."""
    from scipy.spatial.transform import Rotation as Rot, Slerp
    key = Rot.from_quat([q_from, q_to])
    sl = Slerp([0, 1], key)
    pts = []
    prev = np.array(seed, float)
    for i in range(1, n + 1):
        t = i / n
        pos = np.array(p_from) * (1 - t) + np.array(p_to) * t
        qt = sl([t])[0].as_quat()
        s = r.ik_world(pos, qt, seed=prev, tcp=True, tries=3)
        if s is None:
            raise RuntimeError(f"IK fail at t={t:.2f} pos={np.round(pos,3)}")
        s = np.array(s)
        jump = np.abs(s - prev).max()
        if jump > maxjump:
            raise RuntimeError(f"joint jump {jump:.2f} at t={t:.2f} q={np.round(s,2)} prev={np.round(prev,2)}")
        if not margin_ok(s, m):
            raise RuntimeError(f"margin violated at t={t:.2f} q={np.round(s,2)}")
        pts.append(s)
        prev = s
    return pts
EOF
timeout 900 python3 -u -c "
from iksearch import *
r=Robot()
P2=np.array([-0.069,0.221,0.935])
q80=pitched_quat2(math.radians(80),False,0.0); Z=quat_to_R(*q80)[:,2]
pre=P2-0.10*Z; lift=P2+[0,0,0.15]
qg,n=search(r,P2,q80); print('grasp',np.round(qg,2),n)
# pre seeded from grasp
qp=r.ik_world(pre,q80,seed=qg,tcp=True,tries=3); print('pre',np.round(qp,2), margin_ok(qp,0.1))
# path from current pose to pre? check the pre/grasp/lift path continuity
path=path_ik(r,pre,P2,q80,q80,qp,qp,5,qp); print('pre->grasp ok', np.round(path[-1],2))
path2=path_ik(r,P2,lift,q80,q80,path[-1],path[-1],5,path[-1]); print('grasp->lift ok', np.round(path2[-1],2))
print('clearance pre',round(arm_clearance(r,qp),3),'grasp',round(arm_clearance(r,path[-1]),3))
# current -> pre: joint-space; print current
print('cur',np.round(r.arm_q(),2))
" 2>&1 | grep -v Warn

# openrua op 65
cd /workspace; timeout 900 python3 -u -c "
from iksearch import *
r=Robot()
P2=np.array([-0.069,0.221,0.935])
q80=pitched_quat2(math.radians(80),False,0.0); Z=quat_to_R(*q80)[:,2]
pre=P2-0.10*Z; lift=P2+[0,0,0.15]
qg=np.array([0.45,0.78,0.03,-2.13,-2.63,1.94,-2.49])
qp=np.array(r.ik_world(pre,q80,seed=qg,tcp=True,tries=3)); print('pre',np.round(qp,2))
path=path_ik(r,pre,P2,q80,q80,5,qp); print('pre->grasp ok', np.round(path[-1],2))
path2=path_ik(r,P2,lift,q80,q80,5,path[-1]); print('grasp->lift ok', np.round(path2[-1],2))
print('clearance pre',round(arm_clearance(r,qp),3),'grasp',round(arm_clearance(r,path[-1]),3))
# joint-space move from current to pre: check intermediate FK heights (interpolate)
cur=r.arm_q()
for t in np.linspace(0,1,6):
    q=cur*(1-t)+qp*t; L=links_world(r,q); tcp,_=r.tcp_world(q)
    print(f't={t:.1f} hand={np.round(L[\"panda_hand\"],3)} tcp={np.round(tcp,3)} link7={np.round(L[\"panda_link7\"],3)}')
" 2>&1 | grep -v Warn

# openrua op 66
cd /workspace; timeout 900 python3 -u -c "
from iksearch import *
r=Robot()
qp=np.array([0.52,0.64,0.09,-2.42,-2.53,1.79,-2.45])
cur=np.array(r.arm_q())
for t in np.linspace(0,1,8):
    q=cur*(1-t)+qp*t; L=links_world(r,q); tcp,_=r.tcp_world(q)
    print(f't={t:.2f} hand={np.round(L[\"panda_hand\"],3)} tcp={np.round(tcp,3)} link7={np.round(L[\"panda_link7\"],3)} l5={np.round(L[\"panda_link5\"],3)}')
" 2>&1 | grep -v Warn

# openrua op 67
cd /workspace; timeout 900 python3 -u -c "
from iksearch import *
import cv2
r=Robot()
qp=np.array([0.52,0.64,0.09,-2.42,-2.53,1.79,-2.45])
r.move_q(qp, 8.0)
tcp,qt=r.tcp_world(); print('tcp',np.round(tcp,4),'Z',np.round(quat_to_R(*qt)[:,2],3),'X',np.round(quat_to_R(*qt)[:,0],3))
img=r.color('robot0_eye_in_hand'); d=r.depth('robot0_eye_in_hand')
cv2.imwrite('hand_pre80.png',img); np.save('hand_pre80_depth.npy',d)
print('depth center',d[240,320], 'min',np.nanmin(d))
" 2>&1 | grep -v Warn

# openrua op 68
cd /workspace; timeout 300 python3 -u -c "
from rob import *
r=Robot()
d=np.load('hand_pre80_depth.npy'); K=r.K('robot0_eye_in_hand'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
hp,hq=r.fk_world(); R=quat_to_R(*hq)
cam=hp+0.05*R[:,0]
# camera axes: right=Y_h, down=-X_h, forward=Z_h
Rc=np.stack([R[:,1],-R[:,0],R[:,2]],1)
vs,us=np.mgrid[0:480,0:640]
P=(np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1).reshape(-1,3)@Rc.T+cam).reshape(480,640,3)
for zlo,zhi in [(0.905,0.925),(0.925,0.945),(0.945,0.955),(0.96,0.99),(1.0,1.03)]:
    m=(P[...,2]>zlo)&(P[...,2]<zhi)&(d<0.30)&(P[...,0]>-0.16)
    pts=P[m]
    if len(pts): print(f'z {zlo}-{zhi}: n={len(pts)} x {pts[:,0].min():.3f}..{pts[:,0].max():.3f}  y {pts[:,1].min():.3f}..{pts[:,1].max():.3f}  ymid {(pts[:,1].min()+pts[:,1].max())/2:.4f}')
# table
m=(d<0.5)&(P[...,2]<0.905); print('table z', np.round(P[m][:,2].mean(),3) if m.sum() else None)
print('cam',np.round(cam,3))
" 2>&1 | grep -v Warn

# openrua op 69
cd /workspace; timeout 300 python3 -u -c "
from rob import *
r=Robot()
d=np.load('hand_pre80_depth.npy'); K=r.K('robot0_eye_in_hand'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
hp,hq=r.fk_world(); R=quat_to_R(*hq)
cam=hp+0.05*R[:,0]
Rc=np.stack([R[:,1],-R[:,0],R[:,2]],1)
vs,us=np.mgrid[0:480,0:640]
P=(np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1).reshape(-1,3)@Rc.T+cam).reshape(480,640,3)
for zlo,zhi in [(0.905,0.925),(0.925,0.945),(0.945,0.955),(0.96,0.99),(1.0,1.03),(1.03,1.06)]:
    m=(P[...,2]>zlo)&(P[...,2]<zhi)&(d<0.30)&(P[...,0]>-0.16)&(P[...,0]<-0.03)
    pts=P[m]
    if len(pts): print(f'z {zlo}-{zhi}: n={len(pts)} x {pts[:,0].min():.3f}..{pts[:,0].max():.3f}  y {pts[:,1].min():.3f}..{pts[:,1].max():.3f}  ymid {(pts[:,1].min()+pts[:,1].max())/2:.4f}')
" 2>&1 | grep -v Warn

# openrua op 70
cd /workspace; timeout 900 python3 -u -c "
from rob import *
from geometry_msgs.msg import WrenchStamped
r=Robot()
P2=np.array([-0.073,0.216,0.935])
q80=pitched_quat2(math.radians(80),False,0.0); Z=quat_to_R(*q80)[:,2]
pre=P2-0.10*Z
cur=np.array(r.arm_q()); tcp,_=r.tcp_world()
pts=path_ik(r,tcp,pre,q80,q80,3,cur)
r.move_q(pts,3.0)
pts=path_ik(r,pre,P2,q80,q80,6,pts[-1])
code,err=r.move_q(pts,6.0)
tcp,qt=r.tcp_world(); print('tcp',np.round(tcp,4),'target',P2)
w=r.grab('/franka_robot_state_broadcaster/external_wrench', WrenchStamped, 5).wrench
print('wrench',round(w.force.x,2),round(w.force.y,2),round(w.force.z,2))
print('q',np.round(r.arm_q(),3))
" 2>&1 | grep -v Warn

# openrua op 71
cd /workspace; timeout 300 python3 -u -c "
from rob import *
import cv2
r=Robot()
cv2.imwrite('hand_grasp80.png', r.color('robot0_eye_in_hand'))
cv2.imwrite('front_grasp80.png', r.color('frontview'))
" 2>&1 | grep -v Warn

# openrua op 72
cd /workspace; timeout 300 python3 -u -c "
from rob import *
from geometry_msgs.msg import WrenchStamped
r=Robot()
gap=r.gripper(0.0)
r.spin(1.0)
print('gap',round(r.finger_gap(),4))
w=r.grab('/franka_robot_state_broadcaster/external_wrench', WrenchStamped, 5).wrench
print('wrench',round(w.force.x,2),round(w.force.y,2),round(w.force.z,2))
" 2>&1 | grep -v Warn

# openrua op 73
cd /workspace; timeout 600 python3 -u -c "
from rob import *
from geometry_msgs.msg import WrenchStamped
import cv2
r=Robot()
P2=np.array([-0.073,0.216,0.935])
q80=pitched_quat2(math.radians(80),False,0.0)
cur=np.array(r.arm_q())
pts=path_ik(r,P2,P2+[0,0,0.05],q80,q80,3,cur)
r.move_q(pts,3.0)
r.spin(0.5)
print('gap',round(r.finger_gap(),4))
w=r.grab('/franka_robot_state_broadcaster/external_wrench', WrenchStamped, 5).wrench
print('wrench',round(w.force.x,2),round(w.force.y,2),round(w.force.z,2))
cv2.imwrite('front_lift5.png', r.color('frontview')); cv2.imwrite('hand_lift5.png', r.color('robot0_eye_in_hand'))
" 2>&1 | grep -v Warn

# openrua op 74
cd /workspace; timeout 300 python3 -u -c "
from rob import *
from tf2_ros import Buffer, TransformListener
r=Robot()
tfbuf=Buffer(); TransformListener(tfbuf, r.node)
import time
t0=time.time()
while time.time()-t0<10 and not tfbuf.can_transform('world','birdview_optical_frame',rclpy.time.Time()): r.spin(0.2)
t=tfbuf.lookup_transform('world','birdview_optical_frame',rclpy.time.Time())
q=t.transform.rotation; R=quat_to_R(q.x,q.y,q.z,q.w); T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
d=r.depth('birdview'); K=r.K('birdview'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
vs,us=np.mgrid[0:480,0:640]
P=(np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1).reshape(-1,3)@R.T+T).reshape(480,640,3)
c=(-0.073,0.216)
m=(np.abs(P[...,0]-c[0])<0.05)&(np.abs(P[...,1]-c[1])<0.05)&(P[...,2]>1.04)
pts=P[m]; print('pot2 region n',m.sum(),'zmax',round(pts[:,2].max(),3),'centroid of z>1.09:',np.round(P[m&(P[...,2]>1.09)][:,:2].mean(0),4) if (m&(P[...,2]>1.09)).sum() else None)
" 2>&1 | grep -v Warn

# openrua op 75
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
P2=np.array([-0.073,0.216,0.935])
q80=pitched_quat2(math.radians(80),False,0.0)
q45=pitched_quat2(math.radians(80),False,math.radians(45))
B=np.array([0.235,0.045,0.968])
cur=np.array(r.arm_q()); tcp,_=r.tcp_world(); print('tcp',np.round(tcp,3))
lift=P2+[0,0,0.15]
p1=path_ik(r,tcp,lift,q80,q80,4,cur); print('lift ok',np.round(p1[-1],2))
high=np.array([B[0],B[1],1.10])
p2=path_ik(r,lift,high,q80,q45,10,p1[-1]); print('carry ok',np.round(p2[-1],2))
p3=path_ik(r,high,B,q45,q45,8,p2[-1]); print('descend ok',np.round(p3[-1],2))
for name,q in [('high',p2[-1]),('low',p3[-1])]:
    print(name); arm_clearance(r,q,True)
np.save('plan_pot2.npy',np.array(p1+p2+p3)); print(len(p1),len(p2),len(p3))
" 2>&1 | grep -v Warn

# openrua op 76
cd /workspace; timeout 900 python3 -u -c "
from iksearch import *
r=Robot()
q80=pitched_quat2(math.radians(80),False,0.0)
q45=pitched_quat2(math.radians(80),False,math.radians(45))
ql=np.array([0.44,0.37,0.02,-2.3,-2.59,2.16,-2.61])
rng=np.random.default_rng(0)
def allsols(pos,qt,ref,n=40):
    sols=[]
    for i in range(n):
        seed=ref+rng.normal(0,0.5,7)
        s=r.ik_world(np.array(pos),qt,seed=seed,tcp=True)
        if s is not None and margin_ok(s,0.1): sols.append(np.array(s))
    return sols
for pos,qt,name in [((-0.011,0.182,1.088),q80,'t0.2 yaw0'),((-0.011,0.182,1.088),q45,'t0.2 yaw45'),((0.05,0.15,1.10),q45,'mid'),((0.235,0.045,1.10),q45,'high45'),((0.235,0.045,1.10),q80,'high0')]:
    sols=allsols(pos,qt,ql)
    if not sols: print(name,'none'); continue
    ds=[np.abs(s-ql).max() for s in sols]
    i=int(np.argmin(ds)); print(name,'n',len(sols),'closest',np.round(sols[i],2),'maxdiff',round(ds[i],2))
" 2>&1 | grep -v Warn

# openrua op 77
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
P2=np.array([-0.073,0.216,0.935])
q80=pitched_quat2(math.radians(80),False,0.0)
ql=np.array([0.44,0.37,0.02,-2.3,-2.59,2.16,-2.61])
lift=P2+[0,0,0.15]
for Bxy in [(0.235,0.045),(0.23,0.05),(0.225,0.045)]:
  for hz in (1.03,1.06):
    try:
        high=np.array([Bxy[0],Bxy[1],hz]); low=np.array([Bxy[0],Bxy[1],0.978])
        p2=path_ik(r,lift,high,q80,q80,10,ql)
        p3=path_ik(r,high,low,q80,q80,5,p2[-1])
        print(Bxy,hz,'OK high',np.round(p2[-1],2),'low',np.round(p3[-1],2))
        L=links_world(r,p3[-1]); print('   link7',np.round(L['panda_link7'],3),'hand',np.round(L['panda_hand'],3))
        np.save('plan_pot2_carry.npy',np.array(p2+p3)); break
    except Exception as e: print(Bxy,hz,'FAIL',e)
" 2>&1 | grep -v Warn

# openrua op 78
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
P2=np.array([-0.073,0.216,0.935])
q80=pitched_quat2(math.radians(80),False,0.0)
ql=np.array([0.44,0.37,0.02,-2.3,-2.59,2.16,-2.61])
lift=P2+[0,0,0.15]
for Bxy in [(0.235,0.045)]:
  for hz in (1.03,1.10,1.15):
    try:
        high=np.array([Bxy[0],Bxy[1],hz]); low=np.array([Bxy[0],Bxy[1],0.978])
        p2=path_ik(r,lift,high,q80,q80,10,ql,m=0.03)
        p3=path_ik(r,high,low,q80,q80,5,p2[-1],m=0.03)
        P=np.array(p2+p3); print(Bxy,hz,'OK; j5 range',round(P[:,4].min(),3),round(P[:,4].max(),3),'j6 max',round(P[:,5].max(),3))
        print('  high',np.round(p2[-1],2),'low',np.round(p3[-1],2))
        L=links_world(r,p3[-1]); print('   link7',np.round(L['panda_link7'],3),'hand',np.round(L['panda_hand'],3))
        np.save(f'plan_pot2_carry_{hz}.npy',P)
    except Exception as e: print(Bxy,hz,'FAIL',e)
" 2>&1 | grep -v Warn

# openrua op 79
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
P2=np.array([-0.073,0.216,0.935])
q80=pitched_quat2(math.radians(80),False,0.0)
ql=np.array([0.44,0.37,0.02,-2.3,-2.59,2.16,-2.61])
lift=P2+[0,0,0.15]
for Bxy in [(0.185,0.085),(0.2,0.05),(0.2,0.085),(0.19,0.06)]:
  for hz in (1.03,):
    try:
        high=np.array([Bxy[0],Bxy[1],hz]); low=np.array([Bxy[0],Bxy[1],0.978])
        p2=path_ik(r,lift,high,q80,q80,10,ql,m=0.05)
        p3=path_ik(r,high,low,q80,q80,5,p2[-1],m=0.05)
        P=np.array(p2+p3); print(Bxy,hz,'OK; j5 range',round(P[:,4].min(),3),round(P[:,4].max(),3),'j6 max',round(P[:,5].max(),3))
        print('  high',np.round(p2[-1],2),'low',np.round(p3[-1],2))
        L=links_world(r,p3[-1]); print('   link7',np.round(L['panda_link7'],3),'hand',np.round(L['panda_hand'],3))
        np.save(f'plan_pot2_carry_{Bxy[0]}_{Bxy[1]}.npy',P)
    except Exception as e: print(Bxy,hz,'FAIL',e)
" 2>&1 | grep -v Warn

# openrua op 80
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
P2=np.array([-0.073,0.216,0.935])
q80=pitched_quat2(math.radians(80),False,0.0)
ql=np.array([0.44,0.37,0.02,-2.3,-2.59,2.16,-2.61])
lift=P2+[0,0,0.15]
rng=np.random.default_rng(3)
sols=[]
for i in range(80):
    seed=ql+rng.normal(0,0.7,7)
    s=r.ik_world(lift,q80,seed=seed,tcp=True)
    if s is not None and margin_ok(s,0.15): sols.append(np.round(s,2))
sols=np.unique(np.array(sols),axis=0)
print(len(sols))
for s in sols[np.argsort(sols[:,4])]: print(s)
" 2>&1 | grep -v Warn

# openrua op 81
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
P2=np.array([-0.073,0.216,0.935])
q80=pitched_quat2(math.radians(80),False,0.0)
ql=np.array([0.44,0.37,0.02,-2.3,-2.59,2.16,-2.61])
lift=P2+[0,0,0.15]
for pdeg in (70,60,50):
  qp=pitched_quat2(math.radians(pdeg),False,0.0)
  for Bxy in [(0.235,0.045),(0.2,0.06)]:
    hz=1.05
    try:
        high=np.array([Bxy[0],Bxy[1],hz]); low=np.array([Bxy[0],Bxy[1],0.98])
        p2=path_ik(r,lift,high,q80,qp,10,ql,m=0.05)
        p3=path_ik(r,high,low,qp,qp,5,p2[-1],m=0.05)
        P=np.array(p2+p3); print(pdeg,Bxy,'OK; j5 range',round(P[:,4].min(),3),round(P[:,4].max(),3),'j6 max',round(P[:,5].max(),3))
        print('  high',np.round(p2[-1],2),'low',np.round(p3[-1],2))
        L=links_world(r,p3[-1]); print('   link7',np.round(L['panda_link7'],3),'hand',np.round(L['panda_hand'],3))
        np.save(f'plan_pot2_carry_p{pdeg}_{Bxy[0]}.npy',P)
    except Exception as e: print(pdeg,Bxy,'FAIL',e)
" 2>&1 | grep -v Warn

# openrua op 82
cd /workspace; timeout 1200 python3 -u -c "
from rob import *
import itertools
r=Robot()
rng=np.random.default_rng(5)
ql=np.array([0.44,0.37,0.02,-2.3,-2.59,2.16,-2.61])
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
best={}
for x,y,p,yaw in itertools.product((0.14,0.19,0.235),(0.0,0.05,0.09),(75,80,85,90),(-30,0,30)):
    qt=pitched_quat2(math.radians(p),False,math.radians(yaw))
    pos=np.array([x,y,0.98])
    sols=[]
    for i in range(12):
        seed=ql+rng.normal(0,0.8,7) if i%2 else nice_seed(pos)+rng.normal(0,0.5,7)
        s=r.ik_world(pos,qt,seed=seed,tcp=True)
        if s is not None: sols.append(np.array(s))
    if not sols: continue
    ms=[margin(s) for s in sols]; i=int(np.argmax(ms))
    print(f'x{x} y{y} p{p} yaw{yaw}: n={len(sols):2d} best margin {ms[i]:.2f} q={np.round(sols[i],2)}')
" 2>&1 | grep -v Warn | sort -t'n' -k4 | tail -40

# openrua op 83
cd /workspace; timeout 1200 python3 -u -c "
from rob import *
import itertools
r=Robot()
rng=np.random.default_rng(5)
ql=np.array([0.44,0.37,0.02,-2.3,-2.59,2.16,-2.61])
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
for x,y,p,yaw in itertools.product((0.14,0.19),(0.0,0.05,0.09),(80,85,90),(30,45,60)):
    qt=pitched_quat2(math.radians(p),False,math.radians(yaw))
    pos=np.array([x,y,0.98])
    sols=[]
    for i in range(12):
        seed=ql+rng.normal(0,0.8,7) if i%2 else nice_seed(pos)+rng.normal(0,0.5,7)
        s=r.ik_world(pos,qt,seed=seed,tcp=True)
        if s is not None: sols.append(np.array(s))
    if not sols: print(f'x{x} y{y} p{p} yaw{yaw}: none'); continue
    ms=[margin(s) for s in sols]; i=int(np.argmax(ms))
    print(f'x{x} y{y} p{p} yaw{yaw}: n={len(sols):2d} best margin {ms[i]:.2f} q={np.round(sols[i],2)}')
" 2>&1 | grep -v Warn

# openrua op 84
cd /workspace; timeout 1200 python3 -u -c "
from iksearch import *
r=Robot()
P2=np.array([-0.073,0.216,0.935])
q45=pitched_quat2(math.radians(80),False,math.radians(45)); Z=quat_to_R(*q45)[:,2]
res=search(r,P2,q45); print('grasp45',res and (np.round(res[0],2),res[1]))
rng=np.random.default_rng(1)
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
sols=[]
for i in range(30):
    s=r.ik_world(P2,q45,seed=nice_seed(P2)+rng.normal(0,0.6,7),tcp=True)
    if s is not None: sols.append(np.array(s))
ms=[margin(s) for s in sols]; i=int(np.argmax(ms)); qg=sols[i]; print('best-margin grasp45',np.round(qg,2),round(ms[i],2))
pre=P2-0.10*Z; lift=P2+[0,0,0.15]
qp=np.array(r.ik_world(pre,q45,seed=qg,tcp=True,tries=3)); print('pre',np.round(qp,2))
p1=path_ik(r,pre,P2,q45,q45,5,qp,m=0.12); p2=path_ik(r,P2,lift,q45,q45,5,p1[-1],m=0.12); print('lift',np.round(p2[-1],2))
for Bxy in [(0.235,0.045),(0.22,0.05),(0.2,0.06)]:
    try:
        high=np.array([Bxy[0],Bxy[1],1.04]); low=np.array([Bxy[0],Bxy[1],0.972])
        p3=path_ik(r,lift,high,q45,q45,10,p2[-1],m=0.12); p4=path_ik(r,high,low,q45,q45,5,p3[-1],m=0.12)
        P=np.array(p3+p4); print(Bxy,'OK margin min',round(min(margin(q) for q in P),2),'low',np.round(p4[-1],2))
        L=links_world(r,p4[-1]); print('   link7',np.round(L['panda_link7'],3),'hand',np.round(L['panda_hand'],3),'l5',np.round(L['panda_link5'],3))
    except Exception as e: print(Bxy,'FAIL',e)
print('clearance grasp',round(arm_clearance(r,qg,True),3))
" 2>&1 | grep -v Warn

# openrua op 85
cd /workspace; timeout 1200 python3 -u -c "
from iksearch import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
P1=np.array([-0.202,-0.201,0.935])
q0=pitched_quat2(math.radians(80),False,0.0); Z0=quat_to_R(*q0)[:,2]
q45=pitched_quat2(math.radians(80),False,math.radians(45))
rng=np.random.default_rng(2)
sols=[]
for i in range(30):
    s=r.ik_world(P1,q0,seed=nice_seed(P1)+rng.normal(0,0.6,7),tcp=True)
    if s is not None: sols.append(np.array(s))
ms=[margin(s) for s in sols]; i=int(np.argmax(ms)); qg=sols[i]; print('pot1 grasp yaw0',np.round(qg,2),round(ms[i],2))
pre=P1-0.10*Z0; lift=P1+[0,0,0.15]
qp=np.array(r.ik_world(pre,q0,seed=qg,tcp=True,tries=3)); print('pre',np.round(qp,2))
p1=path_ik(r,pre,P1,q0,q0,5,qp,m=0.12); p2=path_ik(r,P1,lift,q0,q0,5,p1[-1],m=0.12); print('lift',np.round(p2[-1],2))
for Axy in [(0.125,-0.01),(0.13,0.0),(0.13,-0.02)]:
    try:
        high=np.array([Axy[0],Axy[1],1.04]); low=np.array([Axy[0],Axy[1],0.975])
        p3=path_ik(r,lift,high,q0,q45,12,p2[-1],m=0.12); p4=path_ik(r,high,low,q45,q45,5,p3[-1],m=0.12)
        P=np.array(p3+p4); print(Axy,'OK margin min',round(min(margin(q) for q in P),2),'low',np.round(p4[-1],2))
        L=links_world(r,p4[-1]); print('   link7',np.round(L['panda_link7'],3),'hand',np.round(L['panda_hand'],3),'l5',np.round(L['panda_link5'],3))
    except Exception as e: print(Axy,'FAIL',e)
arm_clearance(r,qg,True)
" 2>&1 | grep -v Warn

# openrua op 86
cd /workspace; timeout 900 python3 -u -c "
from rob import *
from geometry_msgs.msg import WrenchStamped
r=Robot()
P2=np.array([-0.073,0.216,0.935])
q0=pitched_quat2(math.radians(80),False,0.0); Z0=quat_to_R(*q0)[:,2]
cur=np.array(r.arm_q()); tcp,_=r.tcp_world(); print('tcp',np.round(tcp,4))
down=np.array([P2[0],P2[1],0.937])
pts=path_ik(r,tcp,down,q0,q0,4,cur,m=0.05); r.move_q(pts,4.0)
tcp,_=r.tcp_world(); print('tcp down',np.round(tcp,4))
gap=r.gripper(0.08); r.spin(1.0); print('gap',round(r.finger_gap(),4))
back=down-0.15*Z0
pts=path_ik(r,down,back,q0,q0,5,pts[-1],m=0.05); r.move_q(pts,5.0)
tcp,_=r.tcp_world(); print('tcp back',np.round(tcp,4))
w=r.grab('/franka_robot_state_broadcaster/external_wrench', WrenchStamped, 5).wrench
print('wrench',round(w.force.x,2),round(w.force.y,2),round(w.force.z,2))
print('q',np.round(r.arm_q(),3))
" 2>&1 | grep -v Warn

# openrua op 87
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
cur=np.array(r.arm_q())
hover=np.array([0.256,0.224,0.128,-1.623,-0.029,1.845,-0.396])
pre45=np.array([-0.45,0.72,0.43,-2.36,2.26,2.03,-2.45])
pots=[np.array([-0.073,0.216]),np.array([-0.202,-0.201])]
def sweep(qa,qb,n=15):
    out=[]
    for t in np.linspace(0,1,n):
        q=qa*(1-t)+qb*t; L=links_world(r,q); tcp,qt=r.tcp_world(q); R=quat_to_R(*qt)
        # finger tip extremes
        tips=[tcp+s*0.05*R[:,1] for s in (-1,1)]
        minz=min(L[k][2] for k in LINKS[2:]); minz=min(minz,tcp[2])
        dpot=min(np.linalg.norm(p[:2]-c) for p in tips+[tcp,L['panda_hand']] for c in pots if p[2]<1.08)
        out.append((round(t,2),np.round(tcp,3),round(minz,3),round(dpot,3) if np.isfinite(dpot) else None))
    return out
print('cur->hover'); [print(o) for o in sweep(cur,hover)]
print('hover->pre45'); [print(o) for o in sweep(hover,pre45)]
" 2>&1 | grep -v Warn

# openrua op 88
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
cur=np.array(r.arm_q())
hover=np.array([0.256,0.224,0.128,-1.623,-0.029,1.845,-0.396])
pre45=np.array([-0.45,0.72,0.43,-2.36,2.26,2.03,-2.45])
pots=[np.array([-0.073,0.216]),np.array([-0.202,-0.201])]
def sweep(qa,qb,n=15):
    out=[]
    for t in np.linspace(0,1,n):
        q=qa*(1-t)+qb*t; L=links_world(r,q); tcp,qt=r.tcp_world(q); R=quat_to_R(*qt)
        tips=[tcp+s*0.05*R[:,1] for s in (-1,1)]
        minz=min(L[k][2] for k in LINKS[2:]); minz=min(minz,tcp[2])
        ds=[np.linalg.norm(p[:2]-c) for p in tips+[tcp,L['panda_hand']] for c in pots if p[2]<1.08]
        out.append((round(t,2),np.round(tcp,3),round(minz,3),round(min(ds),3) if ds else None))
    return out
print('cur->hover'); [print(o) for o in sweep(cur,hover)]
print('hover->pre45'); [print(o) for o in sweep(hover,pre45)]
" 2>&1 | grep -v Warn

# openrua op 89
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
cur=np.array(r.arm_q()); tcp,_=r.tcp_world()
q0=pitched_quat2(math.radians(80),False,0.0)
q45=pitched_quat2(math.radians(80),False,math.radians(45))
pre45=np.array([-0.45,0.72,0.43,-2.36,2.26,2.03,-2.45])
pots=[np.array([-0.073,0.216]),np.array([-0.202,-0.201])]
S=np.array([-0.30,0.12,1.22])
pA=path_ik(r,tcp,S,q0,q0,6,cur,m=0.05); qa=pA[-1]; print('S branch A',np.round(qa,2))
qb=np.array(r.ik_world(S,q45,seed=pre45,tcp=True,tries=3)); print('S branch B',np.round(qb,2), margin_ok(qb,0.1))
def sweep(qa,qb,n=15):
    for t in np.linspace(0,1,n):
        q=qa*(1-t)+qb*t; L=links_world(r,q); tcp,qt=r.tcp_world(q); R=quat_to_R(*qt)
        tips=[tcp+s*0.05*R[:,1] for s in (-1,1)]
        minz=min(L[k][2] for k in LINKS[2:]); minz=min(minz,tcp[2])
        ds=[np.linalg.norm(p[:2]-c) for p in tips+[tcp,L['panda_hand']] for c in pots if p[2]<1.08]
        print(round(t,2),np.round(tcp,3),round(minz,3),round(min(ds),3) if ds else None)
print('A->B at S'); sweep(qa,qb)
print('B(S)->pre45'); sweep(qb,pre45)
np.save('transition.npy',np.array(pA+[qb]))
" 2>&1 | grep -v Warn

# openrua op 90
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
cur=np.array(r.arm_q()); tcp,_=r.tcp_world()
q0=pitched_quat2(math.radians(80),False,0.0)
q45=pitched_quat2(math.radians(80),False,math.radians(45))
pre45=np.array([-0.45,0.72,0.43,-2.36,2.26,2.03,-2.45])
pots=[np.array([-0.073,0.216]),np.array([-0.202,-0.201])]
def sweep(qa,qb,n=13):
    worst=(9,9)
    for t in np.linspace(0,1,n):
        q=qa*(1-t)+qb*t; L=links_world(r,q); tcp,qt=r.tcp_world(q); R=quat_to_R(*qt)
        tips=[tcp+s*0.05*R[:,1] for s in (-1,1)]
        minz=min(L[k][2] for k in LINKS[2:]); minz=min(minz,tcp[2])
        ds=[np.linalg.norm(p[:2]-c) for p in tips+[tcp,L['panda_hand']] for c in pots if p[2]<1.08]
        d=min(ds) if ds else 9
        worst=(min(worst[0],minz),min(worst[1],d))
    return worst
for S in [tcp, tcp+[0,0,0.1], tcp+[-0.03,-0.05,0.12], tcp+[0,-0.1,0.15]]:
    for qt,name in [(q45,'yaw45'),(q0,'yaw0')]:
        qb=r.ik_world(S,qt,seed=pre45,tcp=True,tries=3)
        if qb is None or not margin_ok(qb,0.1): print(np.round(S,3),name,'no B sol'); continue
        qb=np.array(qb)
        print(np.round(S,3),name,'B',np.round(qb,2),'sweep cur->B (minz,dpot)',sweep(cur,qb),' B->pre45',sweep(qb,pre45))
" 2>&1 | grep -v Warn

# openrua op 91
cd /workspace; timeout 900 python3 -u -c "
from rob import *
import cv2
r=Robot()
qB=np.array([-0.56,0.05,0.7,-2.86,2.45,1.96,-2.22])
pre45=np.array([-0.45,0.72,0.43,-2.36,2.26,2.03,-2.45])
r.move_q(qB,10.0)
print('tcp',np.round(r.tcp_world()[0],3))
r.move_q(pre45,8.0)
tcp,qt=r.tcp_world(); print('tcp',np.round(tcp,4),'Z',np.round(quat_to_R(*qt)[:,2],3),'gap',round(r.finger_gap(),4))
img=r.color('robot0_eye_in_hand'); d=r.depth('robot0_eye_in_hand')
cv2.imwrite('hand_pre45.png',img); np.save('hand_pre45_depth.npy',d)
" 2>&1 | grep -v Warn

# openrua op 92
cd /workspace; timeout 300 python3 -u -c "
from rob import *
r=Robot()
d=np.load('hand_pre45_depth.npy'); K=r.K('robot0_eye_in_hand'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
hp,hq=r.fk_world(); R=quat_to_R(*hq)
cam=hp+0.05*R[:,0]
Rc=np.stack([R[:,1],-R[:,0],R[:,2]],1)
vs,us=np.mgrid[0:480,0:640]
P=(np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1).reshape(-1,3)@Rc.T+cam).reshape(480,640,3)
# pot-frame coords: along Z_h (a) and perpendicular horizontal (b)
Zh=R[:,2].copy(); Zh[2]=0; Zh/=np.linalg.norm(Zh); Bh=np.array([-Zh[1],Zh[0],0])
rel=P-hp
a=rel@Zh; b=rel@Bh
for zlo,zhi in [(0.905,0.925),(0.925,0.945),(0.945,0.955)]:
    m=(P[...,2]>zlo)&(P[...,2]<zhi)&(d<0.30)&(a>0.05)&(a<0.20)&(np.abs(b)<0.1)
    pts=P[m]
    if len(pts): print(f'z {zlo}-{zhi}: n={len(pts)} a {a[m].min():.3f}..{a[m].max():.3f}  b {b[m].min():.3f}..{b[m].max():.3f} bmid {(b[m].min()+b[m].max())/2:.4f}; xy of near-most: {np.round(P[m][np.argmin(a[m])][:2],3)}')
print('hand',np.round(hp,3),'Zh',np.round(Zh,3),'Bh',np.round(Bh,3))
" 2>&1 | grep -v Warn

# openrua op 93
cd /workspace; timeout 900 python3 -u -c "
from rob import *
from geometry_msgs.msg import WrenchStamped
import cv2
r=Robot()
P2=np.array([-0.072,0.214,0.935])
q45=pitched_quat2(math.radians(80),False,math.radians(45))
cur=np.array(r.arm_q()); tcp,_=r.tcp_world()
pts=path_ik(r,tcp,P2,q45,q45,6,cur,m=0.12); code,err=r.move_q(pts,6.0)
tcp,_=r.tcp_world(); print('tcp',np.round(tcp,4))
w=r.grab('/franka_robot_state_broadcaster/external_wrench', WrenchStamped, 5).wrench
print('wrench',round(w.force.x,2),round(w.force.y,2),round(w.force.z,2))
cv2.imwrite('hand_grasp45.png', r.color('robot0_eye_in_hand'))
r.gripper(0.0); r.spin(1.0); print('gap',round(r.finger_gap(),4))
lift=P2+[0,0,0.15]
pts=path_ik(r,P2,lift,q45,q45,5,pts[-1],m=0.12); r.move_q(pts,5.0)
tcp,_=r.tcp_world(); print('tcp lift',np.round(tcp,4),'gap',round(r.finger_gap(),4))
print('q',np.round(r.arm_q(),3))
" 2>&1 | grep -v Warn

# openrua op 94
cd /workspace; cat > potcheck.py <<'EOF'
from rob import *
from tf2_ros import Buffer, TransformListener
import time
def bird_points(r):
    tfbuf=Buffer(); TransformListener(tfbuf, r.node)
    t0=time.time()
    while time.time()-t0<10 and not tfbuf.can_transform('world','birdview_optical_frame',rclpy.time.Time()): r.spin(0.2)
    t=tfbuf.lookup_transform('world','birdview_optical_frame',rclpy.time.Time())
    q=t.transform.rotation; R=quat_to_R(q.x,q.y,q.z,q.w); T=np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    d=r.depth('birdview'); K=r.K('birdview'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
    vs,us=np.mgrid[0:480,0:640]
    return (np.stack([(us-cx)*d/fx,(vs-cy)*d/fy,d],-1).reshape(-1,3)@R.T+T).reshape(480,640,3)
def pot_report(P,c,rad=0.06,zmin=1.02):
    m=(np.abs(P[...,0]-c[0])<rad)&(np.abs(P[...,1]-c[1])<rad)&(P[...,2]>zmin)
    if m.sum()==0: return None
    pts=P[m]; top=pts[pts[:,2]>pts[:,2].max()-0.03]
    return dict(n=int(m.sum()), zmax=round(float(pts[:,2].max()),3), top_xy=np.round(top[:,:2].mean(0),4))
if __name__=='__main__':
    import sys
    r=Robot(); P=bird_points(r)
    for arg in sys.argv[1:]:
        x,y=map(float,arg.split(',')); print(arg, pot_report(P,(x,y)))
EOF
timeout 300 python3 -u potcheck.py -0.072,0.214 -0.202,-0.201 2>&1 | grep -v Warn

# openrua op 95
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
q45=pitched_quat2(math.radians(80),False,math.radians(45)); Z=quat_to_R(*q45)[:,2]
cur=np.array(r.arm_q()); tcp,_=r.tcp_world()
B=np.array([0.21,0.055,0.975]); high=np.array([0.21,0.055,1.04])
p1=path_ik(r,tcp,high,q45,q45,10,cur,m=0.12); p2=path_ik(r,high,B,q45,q45,4,p1[-1],m=0.12)
back=B-0.12*Z; up=back+[0,0,0.12]
p3=path_ik(r,B,back,q45,q45,4,p2[-1],m=0.12); p4=path_ik(r,back,up,q45,q45,3,p3[-1],m=0.12)
print('margins',round(min(margin(q) for q in p1+p2),2),round(min(margin(q) for q in p3+p4),2))
L=links_world(r,p2[-1]); print('at B: link7',np.round(L['panda_link7'],3),'hand',np.round(L['panda_hand'],3),'l5',np.round(L['panda_link5'],3))
# pot bottom clearance along carry: tcp z min
zs=[r.tcp_world(q)[0][2] for q in p1]; print('carry tcp z min',round(min(zs),3))
np.save('pot2_place.npy',np.array(p1+p2)); np.save('pot2_retreat.npy',np.array(p3+p4)); print(len(p1),len(p2),len(p3),len(p4))
" 2>&1 | grep -v Warn

# openrua op 96
cd /workspace; timeout 900 python3 -u -c "
from rob import *
from geometry_msgs.msg import WrenchStamped
import cv2
r=Robot()
P=np.load('pot2_place.npy'); Q=np.load('pot2_retreat.npy')
r.move_q(list(P[:10]),10.0)
print('tcp high',np.round(r.tcp_world()[0],4),'gap',round(r.finger_gap(),4))
r.move_q(list(P[10:]),4.0)
tcp,_=r.tcp_world(); print('tcp B',np.round(tcp,4),'gap',round(r.finger_gap(),4))
w=r.grab('/franka_robot_state_broadcaster/external_wrench', WrenchStamped, 5).wrench
print('wrench',round(w.force.x,2),round(w.force.y,2),round(w.force.z,2))
cv2.imwrite('front_B.png', r.color('frontview'))
r.gripper(0.08); r.spin(1.0); print('gap',round(r.finger_gap(),4))
r.move_q(list(Q[:4]),4.0); r.move_q(list(Q[4:]),3.0)
print('tcp after retreat',np.round(r.tcp_world()[0],4))
print('q',np.round(r.arm_q(),3))
" 2>&1 | grep -v Warn

# openrua op 97
cd /workspace; timeout 300 python3 -u -c "
from rob import *
import cv2
r=Robot()
cv2.imwrite('agent_B.png', r.color('agentview')); cv2.imwrite('bird_B.png', r.color('birdview'))
" 2>&1 | grep -v Warn; timeout 300 python3 -u potcheck.py 0.21,0.055 -0.072,0.214 -0.202,-0.201 2>&1 | grep -v Warn

# openrua op 98
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
cur=np.array(r.arm_q())
pre1=np.array([-0.76,0.57,-0.11,-2.72,2.29,1.61,-2.43])
pots=[np.array([0.231,0.07]),np.array([-0.202,-0.201])]
def sweep(qa,qb,n=15):
    for t in np.linspace(0,1,n):
        q=qa*(1-t)+qb*t; L=links_world(r,q); tcp,qt=r.tcp_world(q); R=quat_to_R(*qt)
        tips=[tcp+s*0.05*R[:,1] for s in (-1,1)]
        minz=min(L[k][2] for k in LINKS[2:]); minz=min(minz,tcp[2])
        ds=[np.linalg.norm(p[:2]-c) for p in tips+[tcp,L['panda_hand']] for c in pots if p[2]<1.08]
        print(round(t,2),np.round(tcp,3),round(minz,3),round(min(ds),3) if ds else None, 'margin ok',margin_ok(q,0.1))
sweep(cur,pre1)
" 2>&1 | grep -v Warn

# openrua op 99
cd /workspace; timeout 900 python3 -u -c "
from rob import *
import cv2
r=Robot()
q0=pitched_quat2(math.radians(80),False,0.0); Z0=quat_to_R(*q0)[:,2]
q45=pitched_quat2(math.radians(80),False,math.radians(45))
P1=np.array([-0.202,-0.201,0.935]); pre1=P1-0.10*Z0
cur=np.array(r.arm_q()); tcp,_=r.tcp_world()
mid=np.array([pre1[0],pre1[1],1.15])
p1=path_ik(r,tcp,mid,q45,q0,10,cur,m=0.12); p2=path_ik(r,mid,pre1,q0,q0,5,p1[-1],m=0.12)
print('pre1 q',np.round(p2[-1],2))
r.move_q(list(p1),10.0); r.move_q(list(p2),5.0)
tcp,qt=r.tcp_world(); print('tcp',np.round(tcp,4),'Z',np.round(quat_to_R(*qt)[:,2],3),'gap',round(r.finger_gap(),4))
img=r.color('robot0_eye_in_hand'); d=r.depth('robot0_eye_in_hand')
cv2.imwrite('hand_pre1.png',img); np.save('hand_pre1_depth.npy',d)
" 2>&1 | grep -v Warn

# openrua op 100
cd /workspace; timeout 900 python3 -u -c "
from rob import *
import cv2
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
q0=pitched_quat2(math.radians(80),False,0.0); Z0=quat_to_R(*q0)[:,2]
q45=pitched_quat2(math.radians(80),False,math.radians(45))
P1=np.array([-0.202,-0.201,0.935]); pre1=P1-0.10*Z0
cur=np.array(r.arm_q()); tcp,_=r.tcp_world()
w1=np.array([-0.05,-0.33,1.05]); w2=np.array([-0.30,-0.32,1.0])
p1=path_ik(r,tcp,w1,q45,q0,8,cur,m=0.12); p2=path_ik(r,w1,w2,q0,q0,6,p1[-1],m=0.12); p3=path_ik(r,w2,pre1,q0,q0,4,p2[-1],m=0.12)
print('margins',[round(min(margin(q) for q in p),2) for p in (p1,p2,p3)],'pre1 q',np.round(p3[-1],2))
for q in (p1[-1],p2[-1],p3[-1]): print(' clearance',round(arm_clearance(r,q),3))
r.move_q(list(p1),8.0); r.move_q(list(p2),6.0); r.move_q(list(p3),4.0)
tcp,qt=r.tcp_world(); print('tcp',np.round(tcp,4),'Z',np.round(quat_to_R(*qt)[:,2],3),'gap',round(r.finger_gap(),4))
img=r.color('robot0_eye_in_hand'); d=r.depth('robot0_eye_in_hand')
cv2.imwrite('hand_pre1.png',img); np.save('hand_pre1_depth.npy',d)
" 2>&1 | grep -v Warn

# openrua op 101
cd /workspace; timeout 900 python3 -u -c "
from rob import *
from geometry_msgs.msg import WrenchStamped
r=Robot()
w=r.grab('/franka_robot_state_broadcaster/external_wrench', WrenchStamped, 5).wrench
print('wrench',round(w.force.x,2),round(w.force.y,2),round(w.force.z,2))
print('q',np.round(r.arm_q(),3))
target=np.array([-1.07,0.58,0.14,-2.7,2.22,1.73,-2.51])
print('diff',np.round(target-np.array(r.arm_q()),3))
" 2>&1 | grep -v Warn

# openrua op 102
cd /workspace; timeout 300 python3 -u -c "
from rob import *
import cv2
r=Robot()
cv2.imwrite('agent_now.png', r.color('agentview')); cv2.imwrite('side_now.png', r.color('sideview')); cv2.imwrite('front_now.png', r.color('frontview'))
" 2>&1 | grep -v Warn

# openrua op 103
cd /workspace; timeout 900 python3 -u -c "
from rob import *
from geometry_msgs.msg import WrenchStamped
import cv2
r=Robot()
cur=np.array(r.arm_q()); tcp,qt=r.tcp_world(); R=quat_to_R(*qt)
q0=pitched_quat2(math.radians(80),False,0.0)
up=tcp+np.array([-0.05,0,0.12])
pts=path_ik(r,tcp,up,qt,q0,4,cur,m=0.1); r.move_q(pts,4.0)
w=r.grab('/franka_robot_state_broadcaster/external_wrench', WrenchStamped, 5).wrench
print('tcp',np.round(r.tcp_world()[0],3),'wrench',round(w.force.x,2),round(w.force.y,2),round(w.force.z,2))
cv2.imwrite('agent_now.png', r.color('agentview')); cv2.imwrite('bird_now.png', r.color('birdview'))
np.save('bird_now_depth.npy', r.depth('birdview'))
" 2>&1 | grep -v Warn

# openrua op 104
cd /workspace; timeout 300 python3 -u -c "
from potcheck import *
r=Robot(); P=bird_points(r)
m=(P[...,0]>-0.45)&(P[...,0]<0.0)&(P[...,1]>-0.4)&(P[...,1]<-0.05)&(P[...,2]>0.905)&(P[...,2]<1.0)
pts=P[m]; print('n',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'zmax',pts[:,2].max().round(3))
# profile along x: for x bins, y-range and zmax
for x0 in np.arange(pts[:,0].min(),pts[:,0].max(),0.01):
    s=(pts[:,0]>=x0)&(pts[:,0]<x0+0.01)
    if s.sum()>3: print(f'x {x0:+.3f}: y {pts[s,1].min():+.3f}..{pts[s,1].max():+.3f}  ymid {(pts[s,1].min()+pts[s,1].max())/2:+.3f} zmax {pts[s,2].max():.3f} n={s.sum()}')
" 2>&1 | grep -v Warn

# openrua op 105
cd /workspace; timeout 1200 python3 -u -c "
from iksearch import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
rng=np.random.default_rng(7)
def best(pos,qt,ref=None,n=30):
    sols=[]
    for i in range(n):
        seed=(ref if ref is not None else nice_seed(pos))+rng.normal(0,0.7,7)
        s=r.ik_world(np.array(pos),qt,seed=seed,tcp=True)
        if s is not None: sols.append(np.array(s))
    if not sols: return None
    ms=[margin(s) for s in sols]; i=int(np.argmax(ms)); return sols[i],ms[i],len(sols)
G=np.array([-0.275,-0.182,0.935]); Lf=np.array([-0.275,-0.182,1.15])
qg=pitched_quat2(math.radians(-10),False,math.radians(180))
print('grasp quat X',np.round(quat_to_R(*qg)[:,0],3),'Y',np.round(quat_to_R(*qg)[:,1],3),'Z',np.round(quat_to_R(*qg)[:,2],3))
res=best(G,qg); print('grasp',res and (np.round(res[0],2),round(res[1],2),res[2]))
qs=res[0]
p1=path_ik(r,G,Lf,qg,qg,4,qs,m=0.1); print('lift',np.round(p1[-1],2))
prev=p1[-1]; ok=True
for p in (10,30,50,70,90):
    qt=pitched_quat2(math.radians(p),False,math.radians(180))
    try:
        seg=path_ik(r,Lf,Lf,pitched_quat2(math.radians(p-20 if p>10 else -10),False,math.radians(180)),qt,4,prev,m=0.1); prev=seg[-1]
        print(f'p={p}: q={np.round(prev,2)} margin={margin(prev):.2f}')
    except Exception as e: print(f'p={p}: FAIL {e}'); ok=False; break
if ok:
    for yaw in (135,90,45,0):
        qt=pitched_quat2(math.radians(90),False,math.radians(yaw))
        try:
            seg=path_ik(r,Lf,Lf,pitched_quat2(math.radians(90),False,math.radians(yaw+45)),qt,4,prev,m=0.1); prev=seg[-1]
            print(f'yaw={yaw}: q={np.round(prev,2)} margin={margin(prev):.2f}')
        except Exception as e: print(f'yaw={yaw}: FAIL {e}'); break
" 2>&1 | grep -v Warn

# openrua op 106
cd /workspace; cat >> rob.py <<'EOF'


def plan_chain(r, wps, n_per=4, m=0.1, tries=25, seed_ref=None, rng=None, maxjump=0.6):
    """wps: list of (pos, quat). Try many IK starts at wps[0]; follow with path_ik.
    Returns list of segments (each a list of q) or None."""
    rng = rng or np.random.default_rng(0)
    pos0, qt0 = wps[0]
    starts = []
    for i in range(tries):
        ref = seed_ref if seed_ref is not None else nice_seed(pos0)
        s = r.ik_world(np.array(pos0), qt0, seed=ref + rng.normal(0, 0.7, 7), tcp=True)
        if s is not None and margin_ok(np.array(s), m):
            starts.append(np.array(s))
    best = None
    for s in starts:
        segs = []
        prev = s
        try:
            for (pa, qa), (pb, qb) in zip(wps[:-1], wps[1:]):
                seg = path_ik(r, pa, pb, qa, qb, n_per, prev, m=m, maxjump=maxjump)
                segs.append(seg)
                prev = seg[-1]
        except RuntimeError as e:
            continue
        allq = np.array([q for seg in segs for q in seg])
        mg = float(np.min(np.minimum(allq - LIMITS[:, 0], LIMITS[:, 1] - allq)))
        if best is None or mg > best[0]:
            best = (mg, s, segs)
    return best
EOF
timeout 1500 python3 -u -c "
from rob import *
r=Robot()
G=np.array([-0.275,-0.182,0.935]); Lf=np.array([-0.275,-0.182,1.15])
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
wps=[(G,Q(-10,180)),(Lf,Q(-10,180))]
for p in (10,30,50,70,90): wps.append((Lf,Q(p,180)))
for yaw in (135,90,45,0): wps.append((Lf,Q(90,yaw)))
res=plan_chain(r,wps,n_per=3,m=0.1,tries=25)
if res is None: print('NO CHAIN')
else:
    mg,s,segs=res; print('chain margin',round(mg,2),'start',np.round(s,2))
    for i,seg in enumerate(segs): print(i,np.round(seg[-1],2))
    np.save('pot1_right_chain.npy',np.array([q for seg in segs for q in seg]))
" 2>&1 | grep -v Warn

# openrua op 107
cd /workspace; timeout 1500 python3 -u -c "
from rob import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
G=np.array([-0.275,-0.182,0.935]); Lf=np.array([-0.275,-0.182,1.06])
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
rng=np.random.default_rng(11)
starts=[]
for i in range(40):
    s=r.ik_world(G,Q(10,180),seed=nice_seed(G)+rng.normal(0,0.7,7),tcp=True)
    if s is not None and margin_ok(np.array(s),0.15): starts.append(np.array(s))
print('starts',len(starts))
best=None
for s in starts:
    try:
        seg1=path_ik(r,G,Lf,Q(10,180),Q(10,180),3,s,m=0.15)
    except RuntimeError: continue
    q2=seg1[-1]
    for dj in (math.pi,-math.pi):
        q3=q2.copy(); q3[6]+=dj
        if not margin_ok(q3,0.15): continue
        # verify FK orientation matches Q(-10,0)
        pos,qt=r.tcp_world(q3); Rq=quat_to_R(*qt); Rt=quat_to_R(*Q(-10,0))
        if np.abs(Rq-Rt).max()>0.05: print('orientation mismatch',np.round(Rq,2)); continue
        try:
            seg2=path_ik(r,Lf,Lf,Q(-10,0),Q(35,0),3,q3,m=0.15); seg3=path_ik(r,Lf,Lf,Q(35,0),Q(80,0),3,seg2[-1],m=0.15)
        except RuntimeError as e: continue
        allq=np.array(seg1+[q3]+seg2+seg3); mg=min(margin(q) for q in allq)
        print('OK start',np.round(s,2),'q3',np.round(q3,2),'end',np.round(seg3[-1],2),'margin',round(mg,2))
        if best is None or mg>best[0]: best=(mg,s,seg1,q3,seg2+seg3)
if best:
    mg,s,seg1,q3,seg23=best; np.save('pot1_plan_start.npy',s); np.save('pot1_plan_seg1.npy',np.array(seg1)); np.save('pot1_plan_q3.npy',q3); np.save('pot1_plan_seg23.npy',np.array(seg23)); print('BEST margin',round(mg,2))
" 2>&1 | grep -v Warn

# openrua op 108
cd /workspace; timeout 1500 python3 -u -c "
from rob import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
G=np.array([-0.275,-0.182,0.935]); Lf=np.array([-0.275,-0.182,1.06])
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
rng=np.random.default_rng(11)
starts=[]
for i in range(40):
    s=r.ik_world(G,Q(10,180),seed=nice_seed(G)+rng.normal(0,0.7,7),tcp=True)
    if s is not None and margin_ok(np.array(s),0.15): starts.append(np.array(s))
for s in starts[:8]:
    print('start',np.round(s,2))
    try: seg1=path_ik(r,G,Lf,Q(10,180),Q(10,180),3,s,m=0.15)
    except RuntimeError as e: print('  lift fail',e); continue
    q2=seg1[-1]; print('  q2',np.round(q2,2))
    for dj in (math.pi,-math.pi):
        q3=q2.copy(); q3[6]+=dj
        if not margin_ok(q3,0.15): print('  j7 spin',dj,'out of range'); continue
        pos,qt=r.tcp_world(q3); Rq=quat_to_R(*qt); Rt=quat_to_R(*Q(-10,0))
        print('  q3',np.round(q3,2),'orient err',round(float(np.abs(Rq-Rt).max()),3))
        try:
            seg2=path_ik(r,Lf,Lf,Q(-10,0),Q(35,0),3,q3,m=0.15); print('  seg2 ok',np.round(seg2[-1],2))
            seg3=path_ik(r,Lf,Lf,Q(35,0),Q(80,0),3,seg2[-1],m=0.15); print('  seg3 ok',np.round(seg3[-1],2))
        except RuntimeError as e: print('  rot fail',e)
" 2>&1 | grep -v Warn

# openrua op 109
cd /workspace; timeout 1700 python3 -u -c "
from rob import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
G=np.array([-0.275,-0.182,0.935]); Lf=np.array([-0.275,-0.182,1.06]); Rr=np.array([0.05,-0.10,1.10])
Ah=np.array([0.125,-0.01,1.04]); Al=np.array([0.125,-0.01,0.975])
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
rng=np.random.default_rng(12)
starts=[]
for i in range(40):
    s=r.ik_world(G,Q(10,180),seed=nice_seed(G)+rng.normal(0,0.4,7),tcp=True)
    if s is not None and margin_ok(np.array(s),0.12): starts.append(np.array(s))
print('starts',len(starts))
best=None
for s in starts:
    try: seg1=path_ik(r,G,Lf,Q(10,180),Q(10,180),3,s,m=0.12)
    except RuntimeError as e: continue
    q2=seg1[-1]
    for dj in (math.pi,-math.pi):
        q3=q2.copy(); q3[6]+=dj
        if not margin_ok(q3,0.12): continue
        try:
            seg2=path_ik(r,Lf,Rr,Q(-10,0),Q(-10,0),6,q3,m=0.12)
            seg3=path_ik(r,Rr,Rr,Q(-10,0),Q(35,0),3,seg2[-1],m=0.12)
            seg4=path_ik(r,Rr,Rr,Q(35,0),Q(80,0),3,seg3[-1],m=0.12)
            seg5=path_ik(r,Rr,Ah,Q(80,0),Q(80,45),6,seg4[-1],m=0.12)
            seg6=path_ik(r,Ah,Al,Q(80,45),Q(80,45),3,seg5[-1],m=0.12)
        except RuntimeError as e: print('  fail',str(e)[:80]); continue
        allq=np.array(seg1+[q3]+seg2+seg3+seg4+seg5+seg6); mg=min(margin(q) for q in allq)
        print('OK start',np.round(s,2),'margin',round(mg,2),'end',np.round(seg6[-1],2))
        if best is None or mg>best[0]: best=(mg,s,seg1,q3,seg2,seg3+seg4,seg5+seg6)
if best:
    mg,s,seg1,q3,seg2,seg34,seg56=best
    np.savez('pot1_plan.npz',start=s,seg1=np.array(seg1),q3=q3,seg2=np.array(seg2),seg34=np.array(seg34),seg56=np.array(seg56)); print('BEST margin',round(mg,2),'start',np.round(s,2))
    for nm,q in [('grasp',s),('lift',seg1[-1]),('spun',q3),('R',seg2[-1]),('upright',seg34[-1]),('A low',seg56[-1])]:
        L=links_world(r,q); print(nm,'hand',np.round(L['panda_hand'],3),'link7',np.round(L['panda_link7'],3),'l5',np.round(L['panda_link5'],3),'margin',round(margin(q),2))
" 2>&1 | grep -v Warn

# openrua op 110
cd /workspace; timeout 1700 python3 -u -c "
from rob import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
rng=np.random.default_rng(3)
for Rr in [np.array([0.05,-0.10,1.10]),np.array([0.0,-0.15,1.12]),np.array([-0.05,-0.25,1.10]),np.array([0.10,-0.05,1.12])]:
    ok=0; tried=0
    for i in range(30):
        s=r.ik_world(Rr,Q(-10,0),seed=nice_seed(Rr)+rng.normal(0,0.5,7),tcp=True)
        if s is None or not margin_ok(np.array(s),0.12): continue
        tried+=1
        try:
            seg=path_ik(r,Rr,Rr,Q(-10,0),Q(80,0),12,np.array(s),m=0.12,maxjump=0.5)
            ok+=1; print('OK',Rr,'start',np.round(s,2),'end',np.round(seg[-1],2),'mg',round(min(margin(q) for q in seg),2))
        except RuntimeError as e: print('  fail',str(e)[:70])
    print(Rr,'tried',tried,'ok',ok)
" 2>&1 | grep -v Warn

# openrua op 111
cd /workspace; timeout 1700 python3 -u -c "
from rob import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
rng=np.random.default_rng(5)
good=np.array([0.31,0.79,-0.89,-1.56,0.54,1.9,0.1])
res=[]
for Rr in [np.array([-0.05,-0.25,1.10]),np.array([0.0,-0.2,1.12]),np.array([0.05,-0.15,1.12])]:
  for yawe in (0,45):
    ok=0; tried=0
    for i in range(40):
        seed=(good if i%2 else nice_seed(Rr))+rng.normal(0,0.5,7)
        s=r.ik_world(Rr,Q(-10,0),seed=seed,tcp=True)
        if s is None or not margin_ok(np.array(s),0.12): continue
        tried+=1
        try:
            seg=path_ik(r,Rr,Rr,Q(-10,0),Q(80,yawe),12,np.array(s),m=0.12,maxjump=0.5)
            ok+=1; mg=min(margin(q) for q in seg); print('OK',Rr,yawe,'start',np.round(s,2),'end',np.round(seg[-1],2),'mg',round(mg,2)); res.append((Rr,yawe,np.array(s),np.array(seg),mg))
        except RuntimeError as e: pass
    print(Rr,yawe,'tried',tried,'ok',ok)
import pickle; pickle.dump(res,open('rot_ok.pkl','wb'))
" 2>&1 | grep -v Warn

# openrua op 112
cd /workspace; timeout 1700 python3 -u -c "
from rob import *
import pickle
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
G=np.array([-0.275,-0.182,0.935]); Lf=np.array([-0.275,-0.182,1.06]); Rr=np.array([0.0,-0.2,1.12])
Ah=np.array([0.125,-0.01,1.04]); Al=np.array([0.125,-0.01,0.975])
res=pickle.load(open('rot_ok.pkl','rb'))
goods=[x[2] for x in res if x[1]==45 and x[4]>0.6]
print('good seeds',len(goods))
rng=np.random.default_rng(1)
best=None
for gs in goods:
    for k in range(3):
        seed=gs.copy(); seed[6]-=math.pi; seed=seed+rng.normal(0,0.15,7)*(k>0)
        s=r.ik_world(G,Q(10,180),seed=seed,tcp=True)
        if s is None: continue
        s=np.array(s)
        if not margin_ok(s,0.15): continue
        try:
            seg1=path_ik(r,G,Lf,Q(10,180),Q(10,180),3,s,m=0.15)
            q3=seg1[-1].copy(); q3[6]+=math.pi
            if not margin_ok(q3,0.15): raise RuntimeError('spin margin')
            seg2=path_ik(r,Lf,Rr,Q(-10,0),Q(-10,0),6,q3,m=0.15,maxjump=0.5)
            seg3=path_ik(r,Rr,Rr,Q(-10,0),Q(80,45),12,seg2[-1],m=0.15,maxjump=0.5)
            seg4=path_ik(r,Rr,Ah,Q(80,45),Q(80,45),6,seg3[-1],m=0.15,maxjump=0.5)
            seg5=path_ik(r,Ah,Al,Q(80,45),Q(80,45),3,seg4[-1],m=0.15,maxjump=0.5)
        except RuntimeError as e: print('  fail',str(e)[:90]); continue
        allq=np.array(seg1+[q3]+seg2+seg3+seg4+seg5); mg=min(margin(q) for q in allq)
        print('OK start',np.round(s,2),'margin',round(mg,2))
        if best is None or mg>best[0]: best=(mg,s,seg1,q3,seg2,seg3,seg4,seg5)
if best:
    mg,s,seg1,q3,seg2,seg3,seg4,seg5=best
    np.savez('pot1_plan.npz',start=s,seg1=np.array(seg1),q3=q3,seg2=np.array(seg2),seg3=np.array(seg3),seg4=np.array(seg4),seg5=np.array(seg5))
    print('BEST margin',round(mg,2),'start',np.round(s,3))
    for nm,q in [('grasp',s),('lift',seg1[-1]),('spun',q3),('R',seg2[-1]),('upright',seg3[-1]),('A high',seg4[-1]),('A low',seg5[-1])]:
        L=links_world(r,q); print(nm,np.round(q,2),'hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3),'l5',np.round(L['panda_link5'],3),'l3',np.round(L['panda_link3'],3),'mg',round(margin(q),2))
" 2>&1 | grep -v Warn

# openrua op 113
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
P=np.load('pot1_plan.npz')
q0=r.arm_q(); print('current q',np.round(q0,2),'tcp',np.round(r.tcp_world(q0),3),'gap',r.finger_gap())
lift=P['seg1'][-1]; print('lift q',np.round(lift,2))
Lf=np.array([-0.275,-0.182,1.06]); H=np.array([-0.275,-0.182,1.16])
hov=path_ik(r,Lf,H,Q(10,180),Q(10,180),3,lift,m=0.15)
print('hover q',np.round(hov[-1],2))
np.save('hover_q.npy',hov[-1])
# sweep check current -> hover
for t in np.linspace(0,1,11):
    q=q0+(hov[-1]-q0)*t; L=links_world(r,q)
    print(round(t,1),'tcp',np.round(r.tcp_world(q),3),'hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3),'l6',np.round(L['panda_link6'],3),'l5',np.round(L['panda_link5'],3))
" 2>&1 | grep -v Warn

# openrua op 114
cd /workspace; grep -n "def arm_q\|def joints\|def tcp_world\|def finger_gap" -A6 rob.py | head -60

# openrua op 115
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
P=np.load('pot1_plan.npz')
q0=np.array(r.arm_q()); print('current q',np.round(q0,2),'tcp',np.round(r.tcp_world(q0)[0],3),'gap',round(r.finger_gap(),4))
lift=P['seg1'][-1]; print('lift q',np.round(lift,2))
Lf=np.array([-0.275,-0.182,1.06]); H=np.array([-0.275,-0.182,1.16])
hov=path_ik(r,Lf,H,Q(10,180),Q(10,180),3,lift,m=0.15)
print('hover q',np.round(hov[-1],2))
np.save('hover_q.npy',hov[-1])
for t in np.linspace(0,1,11):
    q=q0+(hov[-1]-q0)*t; L=links_world(r,q)
    print(round(t,1),'tcp',np.round(r.tcp_world(q)[0],3),'hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3),'l6',np.round(L['panda_link6'],3),'l5',np.round(L['panda_link5'],3))
" 2>&1 | grep -v Warn

# openrua op 116
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
hov=np.load('hover_q.npy')
print(r.move_q(hov,12))
q=np.array(r.arm_q()); print('q',np.round(q,2),'tcp',np.round(r.tcp_world(q)[0],3))
import cv2
img=r.color('robot0_eye_in_hand'); d=r.depth('robot0_eye_in_hand')
cv2.imwrite('hand_hover1.png',img); np.save('hand_hover1_depth.npy',d)
print(img.shape, np.nanmin(d), np.nanmax(d))
" 2>&1 | grep -v Warn

# openrua op 117
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
d=np.load('hand_hover1_depth.npy'); import cv2
img=cv2.imread('hand_hover1.png')
q=np.array(r.arm_q()); pos,quat=r.fk_world(q); R=quat_to_R(*quat)
K=r.K('robot0_eye_in_hand'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]; print('K',fx,fy,cx,cy)
cam=pos+0.05*R[:,0]
Rc=np.stack([R[:,1],-R[:,0],R[:,2]],axis=1)
v,u=np.mgrid[0:480,0:640]
pc=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],axis=-1)
P=pc@Rc.T+cam
print('table z sample (row 200,col 100):',P[200,100], ' (row 170, col 550):',P[170,550])
# pot mask: white-ish pixels
m=(img.min(axis=2)>150)&(v>150)&(v<400)
pts=P[m]; print('white pts',len(pts),'x range',pts[:,0].min(),pts[:,0].max(),'y range',pts[:,1].min(),pts[:,1].max(),'zmax',pts[:,2].max())
# lower chamber: rows 300-370
for row in range(200,390,20):
    mm=m[row]; 
    if mm.sum()==0: continue
    cols=np.where(mm)[0]; print('row',row,'x',round(P[row,cols[0],0],3),'y',round(P[row,cols[0],1],3),'->',round(P[row,cols[-1],1],3),'zmax',round(P[row,cols,2].max(),3))
# dark handle
mh=(img.max(axis=2)<60)&(v>200)&(v<320)&(u>240)&(u<300)
ph=P[mh]; print('handle pts',len(ph),'x',ph[:,0].min(),ph[:,0].max(),'y',ph[:,1].min(),ph[:,1].max(),'z',ph[:,2].min(),ph[:,2].max())
np.save('hover_P.npy',P)
" 2>&1 | grep -v Warn

# openrua op 118
cd /workspace; timeout 600 python3 -u -c "
import numpy as np
P=np.load('hover_P.npy')
v,u=np.mgrid[0:480,0:640]
m=(P[:,:,2]>0.92)&(v<400)&(v>140)
pts=P[m]; print('n',len(pts),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'zmax',pts[:,2].max().round(3))
# slices along x
for x0 in np.arange(-0.40,-0.22,0.01):
    s=pts[(pts[:,0]>=x0)&(pts[:,0]<x0+0.01)]
    if len(s)==0: continue
    hi=s[s[:,2]>0.95]
    print(round(x0,2),'n',len(s),'y',s[:,1].min().round(3),s[:,1].max().round(3),'zmax',s[:,2].max().round(3),'ymid(z>.95)',(hi[:,1].min()+hi[:,1].max()).round(3)/2 if len(hi) else None, 'ymin/max hi',hi[:,1].min().round(3) if len(hi) else None,hi[:,1].max().round(3) if len(hi) else None)
"


# openrua op 119
cd /workspace; timeout 1200 python3 -u -c "
from rob import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
G=np.array([-0.272,-0.184,0.935]); Lf=np.array([-0.272,-0.184,1.06]); H=np.array([-0.272,-0.184,1.16]); Rr=np.array([0.0,-0.2,1.12])
Ah=np.array([0.125,-0.01,1.04]); Al=np.array([0.125,-0.01,0.975])
P=np.load('pot1_plan.npz'); s0=P['start']
qh=np.array(r.arm_q())
s=np.array(r.ik_world(G,Q(10,180),seed=s0,tcp=True)); print('grasp q',np.round(s,2),'mg',round(margin(s),2),'tcp',np.round(r.tcp_world(s)[0],3))
seg1=path_ik(r,G,Lf,Q(10,180),Q(10,180),3,s,m=0.15)
segH=path_ik(r,Lf,H,Q(10,180),Q(10,180),3,seg1[-1],m=0.15)
print('hover new',np.round(segH[-1],2),'current',np.round(qh,2),'maxdiff',np.abs(segH[-1]-qh).max().round(3))
q3=seg1[-1].copy(); q3[6]+=math.pi
seg2=path_ik(r,Lf,Rr,Q(-10,0),Q(-10,0),6,q3,m=0.15,maxjump=0.5)
seg3=path_ik(r,Rr,Rr,Q(-10,0),Q(80,45),12,seg2[-1],m=0.15,maxjump=0.5)
seg4=path_ik(r,Rr,Ah,Q(80,45),Q(80,45),6,seg3[-1],m=0.15,maxjump=0.5)
seg5=path_ik(r,Ah,Al,Q(80,45),Q(80,45),3,seg4[-1],m=0.15,maxjump=0.5)
allq=np.array(seg1+[q3]+seg2+seg3+seg4+seg5); print('chain margin',round(min(margin(q) for q in allq),2))
# descend path: hover -> Lf -> G  (reverse)
down=list(reversed(segH))[1:]+list(reversed(seg1))[1:]
np.savez('pot1_plan2.npz',start=s,down=np.array(down),seg1=np.array(seg1),q3=q3,seg2=np.array(seg2),seg3=np.array(seg3),seg4=np.array(seg4),seg5=np.array(seg5))
for nm,q in [('G',s),('spun',q3),('R',seg2[-1]),('up',seg3[-1]),('Ah',seg4[-1]),('Al',seg5[-1])]:
    L=links_world(r,q); print(nm,np.round(q,2),'tcp',np.round(r.tcp_world(q)[0],3),'l7',np.round(L['panda_link7'],3),'mg',round(margin(q),2))
# check seg3 sweep: pot extents
for q in seg3:
    pos,quat=r.tcp_world(q); R=quat_to_R(*quat)
    top=pos+0.12*R[:,0]; base=pos-0.03*R[:,0]
    print('  rot: tcp',np.round(pos,3),'top',np.round(top,3),'base',np.round(base,3))
" 2>&1 | grep -v Warn

# openrua op 120
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
P=np.load('pot1_plan2.npz'); down=list(P['down'])
w0=r.wrench(); print('wrench before',np.round(w0,1))
for i,q in enumerate(down):
    code,err=r.move_q(q,4)
    w=r.wrench(); print(i,'code',code,'err',round(err,4),'tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'wrench',np.round(w,1))
print('gap',round(r.finger_gap(),4))
" 2>&1 | grep -v Warn

# openrua op 121
cd /workspace; grep -n "wrench\|def " rob.py | head -60

# openrua op 122
cd /workspace; grep -rn "external_wrench\|WrenchStamped" --include=*.py . | grep -v tools/ | head; sed -n 124,135p rob.py

# openrua op 123
cd /workspace; cat >> rob.py <<'EOF'


def wrench(r):
    """External wrench (fx,fy,fz,tx,ty,tz) from the robot state broadcaster."""
    from geometry_msgs.msg import WrenchStamped
    m = r.grab("/franka_robot_state_broadcaster/external_wrench", WrenchStamped)
    f, t = m.wrench.force, m.wrench.torque
    return np.array([f.x, f.y, f.z, t.x, t.y, t.z])
EOF
timeout 600 python3 -u -c "
from rob import *
r=Robot()
P=np.load('pot1_plan2.npz'); down=list(P['down'])
print('wrench before',np.round(wrench(r),1))
for i,q in enumerate(down):
    code,err=r.move_q(q,4)
    print(i,'code',code,'err',round(err,4),'tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'wrench',np.round(wrench(r),1))
print('gap',round(r.finger_gap(),4))
" 2>&1 | grep -v Warn

# openrua op 124
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
P=np.load('pot1_plan2.npz'); s=P['start']
code,err=r.move_q(s,4); print('code',code,'err',round(err,4),'tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'wrench',np.round(wrench(r),1))
import cv2; cv2.imwrite('hand_G.png',r.color('robot0_eye_in_hand')); cv2.imwrite('side_G.png',r.color('sideview'))
" 2>&1 | grep -v Warn

# openrua op 125
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
q=np.array(r.arm_q()); pos=r.tcp_world(q)[0]
up=np.array(r.ik_world(pos+[0,0,0.02],Q(10,180),seed=q,tcp=True)); print('jump',np.abs(up-q).max())
code,err=r.move_q(up,3); print(code,err,'tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'wrench',np.round(wrench(r),1))
d=r.depth('robot0_eye_in_hand'); np.save('hand_G_depth.npy',d)
import cv2; cv2.imwrite('hand_G2.png',r.color('robot0_eye_in_hand'))
q=np.array(r.arm_q()); hp,hq=r.fk_world(q); R=quat_to_R(*hq)
K=r.K('robot0_eye_in_hand'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
cam=hp+0.05*R[:,0]; Rc=np.stack([R[:,1],-R[:,0],R[:,2]],axis=1)
v,u=np.mgrid[0:480,0:640]
P=(np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],axis=-1))@Rc.T+cam
tcp=r.tcp_world(q)[0]; print('tcp',np.round(tcp,3))
m=(P[:,:,2]>0.92)&(P[:,:,2]<1.0)&(v<380)
pts=P[m]
for x0 in np.arange(-0.33,-0.22,0.01):
    s=pts[(pts[:,0]>=x0)&(pts[:,0]<x0+0.01)]
    if len(s)<5: continue
    print(round(x0,2),'n',len(s),'y',s[:,1].min().round(3),s[:,1].max().round(3),'ymid',((s[:,1].min()+s[:,1].max())/2).round(3),'zmax',s[:,2].max().round(3))
" 2>&1 | grep -v Warn

# openrua op 126
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
q=np.array(r.arm_q())
Hl=np.array([-0.258,-0.185,0.985])
qh=np.array(r.ik_world(Hl,Q(10,180),seed=q,tcp=True)); print('jump',np.abs(qh-q).max().round(3),'q',np.round(qh,2))
code,err=r.move_q(qh,4); print(code,round(err,4),'tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'wrench',np.round(wrench(r),1))
import cv2, time; time.sleep(0.5)
d=r.depth('robot0_eye_in_hand'); img=r.color('robot0_eye_in_hand'); cv2.imwrite('hand_lowhover.png',img)
q=np.array(r.arm_q()); hp,hq=r.fk_world(q); R=quat_to_R(*hq)
K=r.K('robot0_eye_in_hand'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
cam=hp+0.05*R[:,0]; Rc=np.stack([R[:,1],-R[:,0],R[:,2]],axis=1)
v,u=np.mgrid[0:480,0:640]
P=(np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],axis=-1))@Rc.T+cam
tcp=r.tcp_world(q)[0]; print('tcp',np.round(tcp,3))
m=(P[:,:,2]>0.93)&(P[:,:,2]<0.99)
pts=P[m]
for x0 in np.arange(-0.32,-0.23,0.01):
    s=pts[(pts[:,0]>=x0)&(pts[:,0]<x0+0.01)]
    if len(s)<5: continue
    print(round(x0,2),'n',len(s),'y',s[:,1].min().round(4),s[:,1].max().round(4),'ymid',((s[:,1].min()+s[:,1].max())/2).round(4),'zmax',s[:,2].max().round(3))
np.save('lowhover_P.npy',P)
" 2>&1 | grep -v Warn

# openrua op 127
cd /workspace; timeout 600 python3 -u -c "
import numpy as np
P=np.load('lowhover_P.npy'); v,u=np.mgrid[0:480,0:640]
m=(P[:,:,2]>0.93)&(P[:,:,2]<0.98)
pts=P[m]
for x0 in np.arange(-0.29,-0.24,0.005):
    s=pts[(pts[:,0]>=x0)&(pts[:,0]<x0+0.005)]
    if len(s)<5: continue
    # width at various heights
    hi=s[s[:,2]>0.955]
    print(round(x0,3),'n',len(s),'y',s[:,1].min().round(4),s[:,1].max().round(4),'ymid',((s[:,1].min()+s[:,1].max())/2).round(4),'zmax',s[:,2].max().round(3),'w(z>.955)',(hi[:,1].max()-hi[:,1].min()).round(4) if len(hi) else None)
# finger pixels
mf=(P[:,:,2]>0.98); f=P[mf]; print('finger pts y',f[:,1].min().round(3),f[:,1].max().round(3),'x',f[:,0].min().round(3),f[:,0].max().round(3))
"


# openrua op 128
cd /workspace; timeout 1200 python3 -u -c "
from rob import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
G=np.array([-0.292,-0.182,0.937]); Hl=np.array([-0.292,-0.182,0.985]); Lf=np.array([-0.292,-0.182,1.06]); Rr=np.array([0.0,-0.2,1.12])
Ah=np.array([0.135,-0.015,1.05]); Al=np.array([0.135,-0.015,0.99])
q=np.array(r.arm_q())
qh=np.array(r.ik_world(Hl,Q(10,180),seed=q,tcp=True)); print('hover2 jump',np.abs(qh-q).max().round(3))
down=path_ik(r,Hl,G,Q(10,180),Q(10,180),3,qh,m=0.15)
s=down[-1]; print('grasp q',np.round(s,2),'mg',round(margin(s),2))
seg1=path_ik(r,G,Lf,Q(10,180),Q(10,180),3,s,m=0.15)
q3=seg1[-1].copy(); q3[6]+=math.pi
seg2=path_ik(r,Lf,Rr,Q(-10,0),Q(-10,0),6,q3,m=0.15,maxjump=0.5)
seg3=path_ik(r,Rr,Rr,Q(-10,0),Q(80,45),12,seg2[-1],m=0.15,maxjump=0.5)
seg4=path_ik(r,Rr,Ah,Q(80,45),Q(80,45),6,seg3[-1],m=0.15,maxjump=0.5)
seg5=path_ik(r,Ah,Al,Q(80,45),Q(80,45),3,seg4[-1],m=0.15,maxjump=0.5)
allq=np.array(seg1+[q3]+seg2+seg3+seg4+seg5); print('chain margin',round(min(margin(q) for q in allq),2))
np.savez('pot1_plan3.npz',hover=qh,down=np.array(down),seg1=np.array(seg1),q3=q3,seg2=np.array(seg2),seg3=np.array(seg3),seg4=np.array(seg4),seg5=np.array(seg5))
for nm,q in [('G',s),('spun',q3),('R',seg2[-1]),('up',seg3[-1]),('Ah',seg4[-1]),('Al',seg5[-1])]:
    L=links_world(r,q); print(nm,np.round(q,2),'tcp',np.round(r.tcp_world(q)[0],3),'hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3),'mg',round(margin(q),2))
" 2>&1 | grep -v Warn

# openrua op 129
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
P=np.load('pot1_plan3.npz')
code,err=r.move_q(P['hover'],4); print('hover',code,round(err,4),'tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'wrench',np.round(wrench(r),1))
import time; time.sleep(0.5)
d=r.depth('robot0_eye_in_hand')
q=np.array(r.arm_q()); hp,hq=r.fk_world(q); R=quat_to_R(*hq)
K=r.K('robot0_eye_in_hand'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
cam=hp+0.05*R[:,0]; Rc=np.stack([R[:,1],-R[:,0],R[:,2]],axis=1)
v,u=np.mgrid[0:480,0:640]
Pw=(np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],axis=-1))@Rc.T+cam
m=(Pw[:,:,2]>0.93)&(Pw[:,:,2]<0.98); pts=Pw[m]
for x0 in np.arange(-0.31,-0.27,0.005):
    s=pts[(pts[:,0]>=x0)&(pts[:,0]<x0+0.005)]
    if len(s)<5: continue
    print(round(x0,3),'n',len(s),'y',s[:,1].min().round(4),s[:,1].max().round(4),'ymid',((s[:,1].min()+s[:,1].max())/2).round(4),'zmax',s[:,2].max().round(3))
" 2>&1 | grep -v Warn

# openrua op 130
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
P=np.load('pot1_plan3.npz')
for i,q in enumerate(P['down']):
    code,err=r.move_q(q,3)
    w=wrench(r); print(i,'code',code,'err',round(err,4),'tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'wrench',np.round(w,1))
    if abs(w[2]+5)>4 or abs(w[1]-0.9)>4 or abs(w[0])>4: print('CONTACT - stop'); break
import cv2; cv2.imwrite('hand_G3.png',r.color('robot0_eye_in_hand'))
" 2>&1 | grep -v Warn

# openrua op 131
cd /workspace; timeout 300 python3 -c "
import cv2, numpy as np
for f in ['hand_lowhover.png','hand_G3.png']:
    img=cv2.imread(f); g=img.mean(axis=2)
    print(f)
    for row in [300,330,350,370,385,395]:
        line=g[row]
        white=np.where(line>150)[0]; dark=np.where(line<60)[0]
        # pot: contiguous white block near center
        wc=white[(white>150)&(white<500)]
        print(' row',row,'white',wc.min() if len(wc) else None,wc.max() if len(wc) else None,'center',(wc.min()+wc.max())/2 if len(wc) else None,'dark cols(<640):',[ (d.min(),d.max()) for d in np.split(dark,np.where(np.diff(dark)>1)[0]+1) if len(d)>3])
"


# openrua op 132
cd /workspace; timeout 300 python3 -c "
import cv2, numpy as np
for f in ['hand_lowhover.png','hand_G3.png']:
    img=cv2.imread(f).astype(int); g=img.mean(axis=2); sat=img.max(axis=2)-img.min(axis=2)
    pot=(sat<25)&(g>120)
    print(f)
    for row in [300,330,350,360,370,380,390,398]:
        cols=np.where(pot[row,150:500])[0]+150
        if len(cols)==0: continue
        blocks=[b for b in np.split(cols,np.where(np.diff(cols)>3)[0]+1) if len(b)>20]
        print(' row',row,[(b.min(),b.max(),(b.min()+b.max())/2) for b in blocks])
"


# openrua op 133
cd /workspace; timeout 300 python3 -c "
import numpy as np
P=np.load('lowhover_P.npy')
for row in [300,320,340,360,375,385,395]:
    z=P[row,:,2]; cols=np.where((z>0.93)&(z<0.985))[0]
    blocks=[b for b in np.split(cols,np.where(np.diff(cols)>2)[0]+1) if len(b)>10]
    print(row,[(b.min(),b.max(),(b.min()+b.max())/2, round(P[row,b.min(),1],4), round(P[row,b.max(),1],4), round(P[row,b.min(),0],3)) for b in blocks])
fz=P[:,:,2]>0.985
for row in [380,390,398]:
    cols=np.where(fz[row])[0]; blocks=[b for b in np.split(cols,np.where(np.diff(cols)>2)[0]+1) if len(b)>3]
    print('fingers row',row,[(b.min(),b.max()) for b in blocks])
"


# openrua op 134
cd /workspace; timeout 300 python3 -c "
import numpy as np
P=np.load('lowhover_P.npy')
row=360
for c in range(226,250,2): print(c, np.round(P[row,c],4))
print('...')
for c in range(390,450,2): print(c, np.round(P[row,c],4))
"


# openrua op 135
cd /workspace; timeout 300 python3 -c "
import cv2, numpy as np
img=cv2.imread('hand_lowhover.png')
crop=img[300:410,140:520]
big=cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST)
# draw column ticks every 20 px
for c in range(140,521,20):
    x=(c-140)*3; cv2.line(big,(x,0),(x,15),(0,0,255),1); cv2.putText(big,str(c),(x-10,30),cv2.FONT_HERSHEY_SIMPLEX,0.35,(0,0,255),1)
cv2.imwrite('crop_lowhover.png',big)
"


# openrua op 136
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
q=np.array(r.arm_q()); print('now tcp',np.round(r.tcp_world(q)[0],3))
# retreat up first
up=np.array(r.ik_world(np.array([-0.292,-0.182,1.0]),Q(10,180),seed=q,tcp=True)); print(r.move_q(up,3)[0], 'wrench',np.round(wrench(r),1))
Ht=np.array([-0.385,-0.185,1.0])
qt=np.array(r.ik_world(Ht,Q(10,180),seed=np.array(r.arm_q()),tcp=True)); print('jump',np.abs(qt-np.array(r.arm_q())).max().round(3))
print(r.move_q(qt,4)[0],'tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'wrench',np.round(wrench(r),1))
import cv2,time; time.sleep(0.5)
img=r.color('robot0_eye_in_hand'); d=r.depth('robot0_eye_in_hand'); cv2.imwrite('hand_top.png',img)
q=np.array(r.arm_q()); hp,hq=r.fk_world(q); R=quat_to_R(*hq)
K=r.K('robot0_eye_in_hand'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
cam=hp+0.05*R[:,0]; Rc=np.stack([R[:,1],-R[:,0],R[:,2]],axis=1)
v,u=np.mgrid[0:480,0:640]
Pw=(np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],axis=-1))@Rc.T+cam; np.save('top_P.npy',Pw)
m=(Pw[:,:,2]>0.91)&(Pw[:,:,2]<0.99); pts=Pw[m]
for x0 in np.arange(-0.42,-0.33,0.005):
    s=pts[(pts[:,0]>=x0)&(pts[:,0]<x0+0.005)]
    if len(s)<5: continue
    print(round(x0,3),'n',len(s),'y',s[:,1].min().round(4),s[:,1].max().round(4),'w',(s[:,1].max()-s[:,1].min()).round(4),'ymid',((s[:,1].min()+s[:,1].max())/2).round(4),'zmax',s[:,2].max().round(3))
" 2>&1 | grep -v Warn

# openrua op 137
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
Ht=np.array([-0.34,-0.17,1.0])
qt=np.array(r.ik_world(Ht,Q(10,180),seed=np.array(r.arm_q()),tcp=True)); print('jump',np.abs(qt-np.array(r.arm_q())).max().round(3))
print(r.move_q(qt,4)[0],'tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'wrench',np.round(wrench(r),1))
import cv2,time; time.sleep(0.5)
img=r.color('robot0_eye_in_hand'); d=r.depth('robot0_eye_in_hand'); cv2.imwrite('hand_top2.png',img)
q=np.array(r.arm_q()); hp,hq=r.fk_world(q); R=quat_to_R(*hq)
K=r.K('robot0_eye_in_hand'); fx,fy,cx,cy=K[0,0],K[1,1],K[0,2],K[1,2]
cam=hp+0.05*R[:,0]; Rc=np.stack([R[:,1],-R[:,0],R[:,2]],axis=1)
v,u=np.mgrid[0:480,0:640]
Pw=(np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],axis=-1))@Rc.T+cam; np.save('top2_P.npy',Pw)
m=(Pw[:,:,2]>0.905)&(Pw[:,:,2]<0.99); pts=Pw[m]
for x0 in np.arange(-0.41,-0.36,0.0025):
    s=pts[(pts[:,0]>=x0)&(pts[:,0]<x0+0.0025)]
    if len(s)<5: continue
    print(round(x0,4),'n',len(s),'y',s[:,1].min().round(4),s[:,1].max().round(4),'w',(s[:,1].max()-s[:,1].min()).round(4),'ymid',((s[:,1].min()+s[:,1].max())/2).round(4),'zmin',s[:,2].min().round(3),'zmax',s[:,2].max().round(3))
" 2>&1 | grep -v Warn

# openrua op 138
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
q=np.array(r.arm_q())
Hk=np.array([-0.397,-0.167,0.99]); Gk=np.array([-0.397,-0.167,0.937])
qh=np.array(r.ik_world(Hk,Q(0,180),seed=q,tcp=True)); print('hover q',np.round(qh,2),'mg',round(margin(qh),2),'jump',np.abs(qh-q).max().round(3))
down=path_ik(r,Hk,Gk,Q(0,180),Q(0,180),3,qh,m=0.15)
print('grasp q',np.round(down[-1],2),'mg',round(margin(down[-1]),2))
L=links_world(r,down[-1]); print('hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3))
np.savez('knob_plan.npz',hover=qh,down=np.array(down))
print(r.move_q(qh,4)[0],'tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'wrench',np.round(wrench(r),1))
import cv2,time; time.sleep(0.5); cv2.imwrite('hand_knobhover.png',r.color('robot0_eye_in_hand'))
" 2>&1 | grep -v Warn

# openrua op 139
cd /workspace; timeout 1200 python3 -u -c "
from rob import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
Gk=np.array([-0.397,-0.167,0.937])
rng=np.random.default_rng(7)
for p in (-10,0,10,20):
  for yaw in (0,180):
    best=None
    for i in range(40):
        s=r.ik_world(Gk,Q(p,yaw),seed=nice_seed(Gk)+rng.normal(0,0.6,7),tcp=True)
        if s is None: continue
        s=np.array(s); mg=margin(s)
        if abs(s[0])>1.5 or abs(s[2])>1.8: continue
        if best is None or mg>best[0]: best=(mg,s)
    print(p,yaw,'best margin',None if best is None else (round(best[0],2),np.round(best[1],2)))
" 2>&1 | grep -v Warn

# openrua op 140
cd /workspace; cat > knobplan.py <<'EOF'
from rob import *
import sys
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
Gk=np.array([-0.397,-0.167,0.937]); Hk=Gk+[0,0,0.05]; Lf=Gk+[0,0,0.15]
Rr=np.array([0.0,-0.2,1.17]); Ah=np.array([0.15,0.005,1.13]); Al=np.array([0.15,0.005,1.09])
PE=85
rng=np.random.default_rng(int(sys.argv[1]) if len(sys.argv)>1 else 0)
best=None; n_ok=0
for p in (-10,0,10):
  for yaw in (0,180):
    for i in range(25):
        s=r.ik_world(Gk,Q(p,yaw),seed=nice_seed(Gk)+rng.normal(0,0.6,7),tcp=True)
        if s is None: continue
        s=np.array(s)
        if not margin_ok(s,0.15) or abs(s[0])>1.6 or abs(s[2])>2.0: continue
        # the end yaw after spin: orientation Q(-p, yaw+180)
        yaw2=(yaw+180)%360
        try:
            hov=path_ik(r,Gk,Hk,Q(p,yaw),Q(p,yaw),2,s,m=0.15)
            seg1=path_ik(r,Gk,Lf,Q(p,yaw),Q(p,yaw),3,s,m=0.15)
            ok=False
            for dj in (math.pi,-math.pi):
                q3=seg1[-1].copy(); q3[6]+=dj
                if not margin_ok(q3,0.15): continue
                try:
                    seg2=path_ik(r,Lf,Rr,Q(-p,yaw2),Q(-p,yaw2),6,q3,m=0.15,maxjump=0.5)
                    seg3=path_ik(r,Rr,Rr,Q(-p,yaw2),Q(PE,yaw2+45),14,seg2[-1],m=0.15,maxjump=0.5)
                    seg4=path_ik(r,Rr,Ah,Q(PE,yaw2+45),Q(PE,yaw2+45),6,seg3[-1],m=0.15,maxjump=0.5)
                    seg5=path_ik(r,Ah,Al,Q(PE,yaw2+45),Q(PE,yaw2+45),2,seg4[-1],m=0.15,maxjump=0.5)
                    ok=True; break
                except RuntimeError as e: pass
            if not ok: raise RuntimeError('no spin/rot')
        except RuntimeError as e:
            continue
        allq=np.array(list(hov)+list(seg1)+[q3]+list(seg2)+list(seg3)+list(seg4)+list(seg5)); mg=min(margin(q) for q in allq)
        n_ok+=1
        print('OK p',p,'yaw',yaw,'dj',round(dj,2),'start',np.round(s,2),'margin',round(mg,2),flush=True)
        if best is None or mg>best[0]: best=(mg,p,yaw,dj,s,hov,seg1,q3,seg2,seg3,seg4,seg5)
print('n_ok',n_ok)
if best:
    mg,p,yaw,dj,s,hov,seg1,q3,seg2,seg3,seg4,seg5=best
    np.savez('knob_plan.npz',p=p,yaw=yaw,dj=dj,start=s,hover=np.array(hov),seg1=np.array(seg1),q3=q3,seg2=np.array(seg2),seg3=np.array(seg3),seg4=np.array(seg4),seg5=np.array(seg5))
    print('BEST',mg,p,yaw,dj)
    for nm,q in [('hover',hov[-1]),('G',s),('lift',seg1[-1]),('spun',q3),('R',seg2[-1]),('up',seg3[-1]),('Ah',seg4[-1]),('Al',seg5[-1])]:
        L=links_world(r,q); print(nm,np.round(q,2),'tcp',np.round(r.tcp_world(q)[0],3),'hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3),'mg',round(margin(q),2))
EOF
timeout 1750 python3 -u knobplan.py 0 2>&1 | grep -v Warn

# openrua op 141
cd /workspace; cat > knobplan2.py <<'EOF'
from rob import *
import sys
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
Gk=np.array([-0.397,-0.167,0.937]); Hk=Gk+[0,0,0.05]; Lf=Gk+[0,0,0.085]; M=np.array([-0.20,-0.20,1.10])
Rr=np.array([0.0,-0.2,1.17]); Ah=np.array([0.15,0.005,1.13]); Al=np.array([0.15,0.005,1.09])
PE=85
rng=np.random.default_rng(int(sys.argv[1]) if len(sys.argv)>1 else 0)
s_ref=np.array([-0.35,0.22,-0.2,-2.85,-0.22,2.92,-2.7])
best=None; n_ok=0
for p in (10,15,20):
    yaw=180
    for i in range(30):
        seed=(s_ref if i%2 else nice_seed(Gk))+rng.normal(0,0.4,7)
        s=r.ik_world(Gk,Q(p,yaw),seed=seed,tcp=True)
        if s is None: continue
        s=np.array(s)
        if not margin_ok(s,0.15) or abs(s[0])>1.6 or abs(s[2])>2.0: continue
        yaw2=(yaw+180)%360
        try:
            hov=path_ik(r,Gk,Hk,Q(p,yaw),Q(p,yaw),2,s,m=0.15)
            seg1=path_ik(r,Gk,Lf,Q(p,yaw),Q(p,yaw),3,s,m=0.15)
            seg1b=path_ik(r,Lf,M,Q(p,yaw),Q(p,yaw),6,seg1[-1],m=0.15,maxjump=0.5)
            ok=False
            for dj in (math.pi,-math.pi):
                q3=seg1b[-1].copy(); q3[6]+=dj
                if not margin_ok(q3,0.15): continue
                try:
                    seg2=path_ik(r,M,Rr,Q(-p,yaw2),Q(-p,yaw2),6,q3,m=0.15,maxjump=0.5)
                    seg3=path_ik(r,Rr,Rr,Q(-p,yaw2),Q(PE,yaw2+45),14,seg2[-1],m=0.15,maxjump=0.5)
                    seg4=path_ik(r,Rr,Ah,Q(PE,yaw2+45),Q(PE,yaw2+45),6,seg3[-1],m=0.15,maxjump=0.5)
                    seg5=path_ik(r,Ah,Al,Q(PE,yaw2+45),Q(PE,yaw2+45),2,seg4[-1],m=0.15,maxjump=0.5)
                    ok=True; break
                except RuntimeError as e: pass
            if not ok: raise RuntimeError('no spin/rot')
        except RuntimeError as e:
            continue
        allq=np.array(list(hov)+list(seg1)+list(seg1b)+[q3]+list(seg2)+list(seg3)+list(seg4)+list(seg5)); mg=min(margin(q) for q in allq)
        n_ok+=1
        print('OK p',p,'dj',round(dj,2),'start',np.round(s,2),'margin',round(mg,2),flush=True)
        if best is None or mg>best[0]: best=(mg,p,yaw,dj,s,hov,seg1,seg1b,q3,seg2,seg3,seg4,seg5)
print('n_ok',n_ok)
if best:
    mg,p,yaw,dj,s,hov,seg1,seg1b,q3,seg2,seg3,seg4,seg5=best
    np.savez('knob_plan2.npz',p=p,yaw=yaw,dj=dj,start=s,hover=np.array(hov),seg1=np.array(seg1),seg1b=np.array(seg1b),q3=q3,seg2=np.array(seg2),seg3=np.array(seg3),seg4=np.array(seg4),seg5=np.array(seg5))
    print('BEST',round(mg,3),p,yaw,dj)
    for nm,q in [('hover',hov[-1]),('G',s),('lift',seg1[-1]),('M',seg1b[-1]),('spun',q3),('R',seg2[-1]),('up',seg3[-1]),('Ah',seg4[-1]),('Al',seg5[-1])]:
        L=links_world(r,q); print(nm,np.round(q,2),'tcp',np.round(r.tcp_world(q)[0],3),'hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3),'mg',round(margin(q),2))
EOF
timeout 1750 python3 -u knobplan2.py 1 2>&1 | grep -v Warn

# openrua op 142
cd /workspace; cat > knobplan3.py <<'EOF'
from rob import *
import sys
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
Gk=np.array([-0.397,-0.167,0.937]); Hk=Gk+[0,0,0.05]; Lf=Gk+[0,0,0.085]; M=np.array([-0.20,-0.20,1.10])
Rr=np.array([0.0,-0.2,1.17]); Ah=np.array([0.15,0.005,1.13]); Al=np.array([0.15,0.005,1.09])
PE=85
rng=np.random.default_rng(int(sys.argv[1]) if len(sys.argv)>1 else 0)
refs={(-10,180):np.array([-0.82,0.35,0.38,-2.6,2.37,3.21,1.08]),(10,0):np.array([-1.12,0.39,0.24,-2.61,-2.34,3.21,2.23]),(-20,180):None,(20,0):None}
best=None; n_ok=0
for (p,yaw),ref in refs.items():
    for i in range(40):
        seed=(ref if (ref is not None and i%2) else nice_seed(Gk))+rng.normal(0,0.5,7)
        s=r.ik_world(Gk,Q(p,yaw),seed=seed,tcp=True)
        if s is None: continue
        s=np.array(s)
        if not margin_ok(s,0.15) or abs(s[0])>1.6 or abs(s[2])>2.0: continue
        yaw2=(yaw+180)%360
        try:
            hov=path_ik(r,Gk,Hk,Q(p,yaw),Q(p,yaw),2,s,m=0.15)
            seg1=path_ik(r,Gk,Lf,Q(p,yaw),Q(p,yaw),3,s,m=0.15)
            seg1b=path_ik(r,Lf,M,Q(p,yaw),Q(p,yaw),6,seg1[-1],m=0.15,maxjump=0.5)
            ok=False
            for dj in (math.pi,-math.pi):
                q3=seg1b[-1].copy(); q3[6]+=dj
                if not margin_ok(q3,0.15): continue
                try:
                    seg2=path_ik(r,M,Rr,Q(-p,yaw2),Q(-p,yaw2),6,q3,m=0.15,maxjump=0.5)
                    seg3=path_ik(r,Rr,Rr,Q(-p,yaw2),Q(PE,yaw2+45),14,seg2[-1],m=0.15,maxjump=0.5)
                    seg4=path_ik(r,Rr,Ah,Q(PE,yaw2+45),Q(PE,yaw2+45),6,seg3[-1],m=0.15,maxjump=0.5)
                    seg5=path_ik(r,Ah,Al,Q(PE,yaw2+45),Q(PE,yaw2+45),2,seg4[-1],m=0.15,maxjump=0.5)
                    ok=True; break
                except RuntimeError as e: last=str(e)
            if not ok: raise RuntimeError('no spin/rot '+last[:60])
        except RuntimeError as e:
            print('  fail',p,yaw,str(e)[:80]); continue
        allq=np.array(list(hov)+list(seg1)+list(seg1b)+[q3]+list(seg2)+list(seg3)+list(seg4)+list(seg5)); mg=min(margin(q) for q in allq)
        n_ok+=1
        print('OK p',p,'yaw',yaw,'dj',round(dj,2),'start',np.round(s,2),'margin',round(mg,2),flush=True)
        if best is None or mg>best[0]: best=(mg,p,yaw,dj,s,hov,seg1,seg1b,q3,seg2,seg3,seg4,seg5)
print('n_ok',n_ok)
if best:
    mg,p,yaw,dj,s,hov,seg1,seg1b,q3,seg2,seg3,seg4,seg5=best
    np.savez('knob_plan3.npz',p=p,yaw=yaw,dj=dj,start=s,hover=np.array(hov),seg1=np.array(seg1),seg1b=np.array(seg1b),q3=q3,seg2=np.array(seg2),seg3=np.array(seg3),seg4=np.array(seg4),seg5=np.array(seg5))
    print('BEST',round(mg,3),p,yaw,dj)
    for nm,q in [('hover',hov[-1]),('G',s),('lift',seg1[-1]),('M',seg1b[-1]),('spun',q3),('R',seg2[-1]),('up',seg3[-1]),('Ah',seg4[-1]),('Al',seg5[-1])]:
        L=links_world(r,q); print(nm,np.round(q,2),'tcp',np.round(r.tcp_world(q)[0],3),'hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3),'mg',round(margin(q),2))
EOF
timeout 1750 python3 -u knobplan3.py 2 2>&1 | grep -v Warn | grep -v "^  fail" 

# openrua op 143
cd /workspace; timeout 1750 python3 -u knobplan3.py 3 2>&1 | grep "^  fail" | sed 's/q=.*//' | sort | uniq -c | sort -rn | head -20

# openrua op 144
cd /workspace; timeout 1200 python3 -u -c "
from rob import *
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
q0=np.array(r.arm_q()); print('cur',np.round(q0,2),'tcp',np.round(r.tcp_world(q0)[0],3))
S0=np.array([-0.415,-0.167,0.99]); S1=np.array([-0.415,-0.167,0.922]); S2=np.array([-0.30,-0.167,0.922])
rng=np.random.default_rng(4); best=None
for i in range(40):
    s=r.ik_world(S0,Q(10,0),seed=nice_seed(S0)+rng.normal(0,0.6,7),tcp=True)
    if s is None: continue
    s=np.array(s)
    if not margin_ok(s,0.2) or abs(s[0])>1.6 or abs(s[2])>2.0: continue
    try:
        d1=path_ik(r,S0,S1,Q(10,0),Q(10,0),3,s,m=0.2)
        d2=path_ik(r,S1,S2,Q(10,0),Q(10,0),8,d1[-1],m=0.2,maxjump=0.4)
    except RuntimeError as e: continue
    mg=min(margin(q) for q in [s]+d1+d2)
    if best is None or mg>best[0]: best=(mg,s,d1,d2)
print('best',None if best is None else round(best[0],2))
mg,s,d1,d2=best
print('S0 q',np.round(s,2)); print('S1 q',np.round(d1[-1],2)); print('S2 q',np.round(d2[-1],2))
np.savez('push_plan.npz',s0=s,d1=np.array(d1),d2=np.array(d2))
for t in np.linspace(0,1,11):
    q=q0+(s-q0)*t; L=links_world(r,q)
    print(round(t,1),'tcp',np.round(r.tcp_world(q)[0],3),'hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3),'l6',np.round(L['panda_link6'],3))
" 2>&1 | grep -v Warn

# openrua op 145
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot(); P=np.load('push_plan.npz')
w0=wrench(r); print('w0',np.round(w0,1))
print('to S0',r.move_q(P['s0'],20)); print('tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3))
r.gripper(0.0); import time; time.sleep(1.5); print('gap',round(r.finger_gap(),4))
for q in P['d1']:
    print('desc',r.move_q(q,4), np.round(r.tcp_world(np.array(r.arm_q()))[0],3), 'dw',np.round(wrench(r)[:3]-w0[:3],1))
" 2>&1 | grep -v Warn

# openrua op 146
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot(); P=np.load('push_plan.npz')
w0=np.array([0,0.9,-5.0])
for i,q in enumerate(P['d2']):
    c=r.move_q(q,3); t=r.tcp_world(np.array(r.arm_q()))[0]; dw=wrench(r)[:3]-w0
    print(i,c[0],round(c[1],3),np.round(t,3),'dw',np.round(dw,1))
    if np.abs(dw).max()>12: print('ABORT'); break
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py birdview bird_push.png; python3 tools/perception/cam_snap.py sideview side_push.png

# openrua op 147
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
q=np.array(r.arm_q()); Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
t=r.tcp_world(q)[0]
up=r.ik_world(t+np.array([0,0,0.13]),Q(10,0),seed=q,tcp=True)
print('margin',margin_ok(np.array(up),0.15)); print(r.move_q(up,5)); print(np.round(r.tcp_world(np.array(r.arm_q()))[0],3))
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py birdview bird_push2.png; python3 tools/perception/cam_snap.py robot0_eye_in_hand hand_push.png

# openrua op 148
cd /workspace; timeout 300 python3 -u -c "
from rob import *
r=Robot(); q=np.array(r.arm_q())
pos,quat=r.fk_world(q,'panda_hand'); R=quat_to_R(quat) if 'quat_to_R' in dir() else None
" 2>&1 | grep -v Warn; grep -n "def .*quat\|def fk_world\|def depth\|def color\|def K" rob.py | head -20

# openrua op 149
cd /workspace; grep -n "hand_cloud\|def cloud\|Rc\b" *.py | head; timeout 300 python3 -u -c "
from rob import *
r=Robot(); q=np.array(r.arm_q())
pos,quat=r.fk_world(q,'panda_hand'); R=quat_to_R(*quat); X,Y,Z=R[:,0],R[:,1],R[:,2]
cam=np.array(pos)+0.05*X; Rc=np.stack([Y,-X,Z],1)
D=r.depth('robot0_eye_in_hand'); D=np.array(D,dtype=float)
h,w=D.shape; u,v=np.meshgrid(np.arange(w),np.arange(h))
pc=np.stack([(u-320)*D/312.77,(v-240)*D/312.77,D],-1).reshape(-1,3)
P=(Rc@pc.T).T+cam; ok=np.isfinite(P).all(1); P=P[ok]
np.save('push_P.npy',P)
tab=P[(P[:,2]>0.85)&(P[:,2]<0.905)]; print('table z',np.round(np.median(tab[:,2]),4))
S=P[(P[:,2]>0.907)&(P[:,0]<-0.05)&(P[:,0]>-0.6)]
print('n obj',len(S),'x',np.round([S[:,0].min(),S[:,0].max()],3),'y',np.round([S[:,1].min(),S[:,1].max()],3),'z max',round(S[:,2].max(),3))
# slices along x
for x0 in np.arange(S[:,0].min(),S[:,0].max(),0.01):
    s=S[(S[:,0]>=x0)&(S[:,0]<x0+0.01)]
    if len(s)<5: continue
    print(round(x0,3),'y',np.round([s[:,1].min(),s[:,1].max()],3),'w',round(s[:,1].max()-s[:,1].min(),3),'zmax',round(s[:,2].max(),3),'ytop',np.round(np.median(s[s[:,2]>s[:,2].max()-0.01][:,1]),3))
" 2>&1 | grep -v Warn

# openrua op 150
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot(); q=np.array(r.arm_q()); Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
up=r.ik_world(np.array([-0.20,-0.19,1.10]),Q(10,0),seed=q,tcp=True); print(np.round(up,2),margin_ok(np.array(up),0.15)); print(r.move_q(up,6))
q=np.array(r.arm_q()); pos,quat=r.fk_world(q,'panda_hand'); R=quat_to_R(*quat); X,Y,Z=R[:,0],R[:,1],R[:,2]
cam=np.array(pos)+0.05*X; Rc=np.stack([Y,-X,Z],1)
D=np.array(r.depth('robot0_eye_in_hand'),dtype=float); h,w=D.shape; u,v=np.meshgrid(np.arange(w),np.arange(h))
pc=np.stack([(u-320)*D/312.77,(v-240)*D/312.77,D],-1).reshape(-1,3); P=(Rc@pc.T).T+cam; P=P[np.isfinite(P).all(1)]
np.save('push2_P.npy',P)
S=P[(P[:,2]>0.907)&(P[:,2]<1.0)&(P[:,0]<-0.05)&(P[:,0]>-0.6)]
print('n obj',len(S),'x',np.round([S[:,0].min(),S[:,0].max()],3),'y',np.round([S[:,1].min(),S[:,1].max()],3),'z max',round(S[:,2].max(),3))
for x0 in np.arange(S[:,0].min(),S[:,0].max(),0.01):
    s=S[(S[:,0]>=x0)&(S[:,0]<x0+0.01)]
    if len(s)<5: continue
    hi=s[s[:,2]>s[:,2].max()-0.008]
    print(round(x0,3),'y',np.round([s[:,1].min(),s[:,1].max()],3),'w',round(s[:,1].max()-s[:,1].min(),3),'zmax',round(s[:,2].max(),3),'ytop',np.round(np.median(hi[:,1]),3),'n',len(s))
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py robot0_eye_in_hand hand_push2.png

# openrua op 151
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot(); q0=np.array(r.arm_q()); Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
T=np.array([-0.20,-0.19,1.10]); rng=np.random.default_rng(1); best=None
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
for i in range(30):
    s=r.ik_world(T,Q(10,180),seed=nice_seed(T)+rng.normal(0,0.5,7),tcp=True)
    if s is None: continue
    s=np.array(s); m=margin(s)
    if best is None or m>best[0]: best=(m,s)
print('best',round(best[0],2),np.round(best[1],2)); s=best[1]
lo=1e9
for t in np.linspace(0,1,21):
    q=q0+(s-q0)*t; L=links_world(r,q); tc=r.tcp_world(q)[0]
    lo=min(lo,tc[2],L['panda_hand'][2],L['panda_link7'][2],L['panda_link6'][2])
print('min z along sweep',round(lo,3))
print(r.move_q(s,10))
q=np.array(r.arm_q()); pos,quat=r.fk_world(q,'panda_hand'); R=quat_to_R(*quat); X,Y,Z=R[:,0],R[:,1],R[:,2]
cam=np.array(pos)+0.05*X; Rc=np.stack([Y,-X,Z],1)
D=np.array(r.depth('robot0_eye_in_hand'),dtype=float); h,w=D.shape; u,v=np.meshgrid(np.arange(w),np.arange(h))
pc=np.stack([(u-320)*D/312.77,(v-240)*D/312.77,D],-1).reshape(-1,3); P=(Rc@pc.T).T+cam; P=P[np.isfinite(P).all(1)]
np.save('push3_P.npy',P)
S=P[(P[:,2]>0.907)&(P[:,2]<1.0)&(P[:,0]<-0.05)&(P[:,0]>-0.6)]
print('n obj',len(S),'x',np.round([S[:,0].min(),S[:,0].max()],3),'y',np.round([S[:,1].min(),S[:,1].max()],3),'z max',round(S[:,2].max(),3))
for x0 in np.arange(S[:,0].min(),S[:,0].max(),0.01):
    s_=S[(S[:,0]>=x0)&(S[:,0]<x0+0.01)]
    if len(s_)<5: continue
    hi=s_[s_[:,2]>s_[:,2].max()-0.008]
    print(round(x0,3),'y',np.round([s_[:,1].min(),s_[:,1].max()],3),'w',round(s_[:,1].max()-s_[:,1].min(),3),'zmax',round(s_[:,2].max(),3),'ytop',np.round(np.median(hi[:,1]),3),'n',len(s_))
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py robot0_eye_in_hand hand_push3.png

# openrua op 152
cd /workspace; cat > knobplan4.py <<'EOF'
from rob import *
import sys
r=Robot()
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw))
YG=167.3
Gk=np.array([-0.309,-0.168,0.937]); Hk=Gk+[0,0,0.05]; Lf=Gk+[0,0,0.085]; M=np.array([-0.20,-0.20,1.10])
Rr=np.array([0.0,-0.2,1.17]); PE=85
rng=np.random.default_rng(int(sys.argv[1]) if len(sys.argv)>1 else 0)
best=None; n_ok=0
for p in (-10,0,-20):
    for i in range(24):
        seed=nice_seed(Gk)+rng.normal(0,0.5,7)
        s=r.ik_world(Gk,Q(p,YG),seed=seed,tcp=True)
        if s is None: continue
        s=np.array(s)
        if not margin_ok(s,0.15) or abs(s[0])>1.6 or abs(s[2])>2.0: continue
        try:
            hov=path_ik(r,Gk,Hk,Q(p,YG),Q(p,YG),2,s,m=0.15)
            seg1=path_ik(r,Gk,Lf,Q(p,YG),Q(p,YG),3,s,m=0.15)
            seg1b=path_ik(r,Lf,M,Q(p,YG),Q(p,YG),6,seg1[-1],m=0.15,maxjump=0.5)
        except RuntimeError as e:
            continue
        for dj in (0.0,math.pi,-math.pi):
            q3=seg1b[-1].copy(); q3[6]+=dj
            if not margin_ok(q3,0.15): continue
            for ye in (45,90,0,135):
                try:
                    seg2=path_ik(r,M,Rr,Q(p,YG),Q(p,YG),6,q3,m=0.15,maxjump=0.5)
                    seg3=path_ik(r,Rr,Rr,Q(p,YG),Q(PE,ye),16,seg2[-1],m=0.15,maxjump=0.5)
                    Xh=quat_to_R(*Q(PE,ye))[:,0]
                    Al=np.array([0.141,-0.004,0.941])+0.15*Xh; Ah=Al+[0,0,0.04]
                    seg4=path_ik(r,Rr,Ah,Q(PE,ye),Q(PE,ye),6,seg3[-1],m=0.15,maxjump=0.5)
                    seg5=path_ik(r,Ah,Al,Q(PE,ye),Q(PE,ye),2,seg4[-1],m=0.15,maxjump=0.5)
                except RuntimeError as e:
                    continue
                allq=np.array(list(hov)+list(seg1)+list(seg1b)+[q3]+list(seg2)+list(seg3)+list(seg4)+list(seg5)); mg=min(margin(q) for q in allq)
                n_ok+=1
                print('OK p',p,'dj',round(dj,2),'ye',ye,'start',np.round(s,2),'margin',round(mg,2),flush=True)
                if best is None or mg>best[0]: best=(mg,p,dj,ye,s,hov,seg1,seg1b,q3,seg2,seg3,seg4,seg5)
print('n_ok',n_ok)
if best:
    mg,p,dj,ye,s,hov,seg1,seg1b,q3,seg2,seg3,seg4,seg5=best
    np.savez('knob_plan4.npz',p=p,ye=ye,dj=dj,start=s,hover=np.array(hov),seg1=np.array(seg1),seg1b=np.array(seg1b),q3=q3,seg2=np.array(seg2),seg3=np.array(seg3),seg4=np.array(seg4),seg5=np.array(seg5))
    print('BEST',round(mg,3),p,dj,ye)
    for nm,q in [('hover',hov[-1]),('G',s),('lift',seg1[-1]),('M',seg1b[-1]),('spun',q3),('R',seg2[-1]),('up',seg3[-1]),('Ah',seg4[-1]),('Al',seg5[-1])]:
        L=links_world(r,q); print(nm,np.round(q,2),'tcp',np.round(r.tcp_world(q)[0],3),'hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3),'mg',round(margin(q),2))
EOF
timeout 1700 python3 -u knobplan4.py 2>&1 | grep -v Warn

# openrua op 153
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot(); P=np.load('knob_plan4.npz'); q0=np.array(r.arm_q())
hov=P['hover'][-1]
print('sweep cur->hover')
for t in np.linspace(0,1,11):
    q=q0+(hov-q0)*t; L=links_world(r,q); tc=r.tcp_world(q)[0]
    print(round(t,1),'tcp',np.round(tc,3),'hand',np.round(L['panda_hand'],3),'l7',np.round(L['panda_link7'],3),'l6',np.round(L['panda_link6'],3),'l5',np.round(L['panda_link5'],3))
print('chain: pot base positions & link clearance')
names=['hover','start','seg1','seg1b','q3','seg2','seg3','seg4','seg5']
for nm in names:
    arr=P[nm]; arr=arr if arr.ndim==2 else arr[None]
    for q in arr:
        pos,quat=r.tcp_world(q); R=quat_to_R(*quat); base=pos-0.15*R[:,0]; hb=pos-0.08*R[:,0]-0.07*R[:,1]
        L=links_world(r,q); lows=min(L[k][2] for k in L if k not in ('panda_link0','panda_link1'))
        print(nm,'tcp',np.round(pos,3),'base',np.round(base,3),'handle',np.round(hb,3),'minlinkz',round(lows,3),'clr',round(arm_clearance(r,q),3) if 'arm_clearance' in dir() else '')
" 2>&1 | grep -v Warn

# openrua op 154
cd /workspace; timeout 900 python3 -u -c "
from rob import *
import time
r=Robot(); P=np.load('knob_plan4.npz')
r.gripper(0.08); time.sleep(1.5); print('gap',round(r.finger_gap(),4))
w0=wrench(r)[:3]; print('w0',np.round(w0,1))
print('hover',r.move_q(P['hover'][-1],10)); print('tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'dw',np.round(wrench(r)[:3]-w0,1))
for q in [P['hover'][0],P['start']]:
    c=r.move_q(q,4); dw=wrench(r)[:3]-w0
    print('desc',c[0],round(c[1],3),np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'dw',np.round(dw,1))
    if np.abs(dw).max()>4: print('CONTACT - stop'); break
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py robot0_eye_in_hand hand_G4.png; python3 tools/perception/cam_snap.py sideview side_G4.png

# openrua op 155
cd /workspace; timeout 300 python3 -u -c "
from rob import *
import time
r=Robot(); w0=np.array([0,0.0,-5.1])
r.gripper(0.0); time.sleep(2); print('gap',round(r.finger_gap(),4)); print('w',np.round(wrench(r)[:3],1))
print('tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3))
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py sideview side_grasp4.png

# openrua op 156
cd /workspace; timeout 120 python3 -c "
import numpy as np
P=np.load('push3_P.npy')
K=P[(P[:,0]<-0.293)&(P[:,0]>-0.33)&(P[:,2]>0.905)&(P[:,1]>-0.20)&(P[:,1]<-0.13)]
print(len(K))
for x0 in np.arange(-0.312,-0.292,0.002):
    s=K[(K[:,0]>=x0)&(K[:,0]<x0+0.002)]
    if len(s)==0: continue
    print(round(x0,3),len(s),'y',np.round([s[:,1].min(),s[:,1].max()],3),'z',np.round([s[:,2].min(),s[:,2].max()],3))
"

# openrua op 157
cd /workspace; timeout 900 python3 -u -c "
from rob import *
import time
r=Robot(); P=np.load('knob_plan4.npz'); Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw)); YG=167.3
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
G2=np.array([-0.300,-0.168,0.932]); H2=G2+[0,0,0.05]; L2=G2+[0,0,0.085]; M=np.array([-0.20,-0.20,1.10])
q0=np.array(r.arm_q())
g2=np.array(r.ik_world(G2,Q(0,YG),seed=q0,tcp=True)); print('g2',np.round(g2,2),'jump',round(np.abs(g2-q0).max(),3),'mg',round(margin(g2),2))
hov2=path_ik(r,G2,H2,Q(0,YG),Q(0,YG),2,g2,m=0.15)
seg1=path_ik(r,G2,L2,Q(0,YG),Q(0,YG),3,g2,m=0.15)
seg1b=path_ik(r,L2,M,Q(0,YG),Q(0,YG),6,seg1[-1],m=0.15,maxjump=0.5)
print('M new',np.round(seg1b[-1],2),'vs plan q3',np.round(P['q3'],2),'jump',round(np.abs(seg1b[-1]-P['q3']).max(),3))
np.savez('knob_plan4b.npz',start=g2,hover=np.array(hov2),seg1=np.array(seg1),seg1b=np.array(seg1b))
# execute: open, up to hover2 (via current +3cm), then descend
r.gripper(0.08); time.sleep(2); print('gap',round(r.finger_gap(),4))
w0=wrench(r)[:3]
print('hover2',r.move_q(hov2[-1],5), np.round(r.tcp_world(np.array(r.arm_q()))[0],3))
for q in [hov2[0],g2]:
    c=r.move_q(q,4); dw=wrench(r)[:3]-w0
    print('desc',c[0],round(c[1],3),np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'dw',np.round(dw,1))
    if np.abs(dw).max()>4: print('CONTACT'); break
r.gripper(0.0); time.sleep(2); print('gap',round(r.finger_gap(),4),'w',np.round(wrench(r)[:3],1))
" 2>&1 | grep -v Warn

# openrua op 158
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot(); P=np.load('knob_plan4b.npz')
for q in P['seg1']:
    c=r.move_q(q,4); print('lift',c[0],round(c[1],3),np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'gap',round(r.finger_gap(),4),'w',np.round(wrench(r)[:3],1))
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py sideview side_lift4.png; python3 tools/perception/cam_snap.py agentview agent_lift4.png

# openrua op 159
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot(); Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw)); YG=167.3
q0=np.array(r.arm_q()); t0=r.tcp_world(q0)[0]
seg=path_ik(r,t0,np.array([-0.30,-0.168,1.10]),Q(0,YG),Q(0,YG),3,q0,m=0.15,maxjump=0.5)
for q in seg:
    c=r.move_q(q,4); print('up',c[0],round(c[1],3),np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'gap',round(r.finger_gap(),4),'w',np.round(wrench(r)[:3],1))
np.save('cur_q.npy',np.array(r.arm_q()))
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py sideview side_lift5.png; python3 tools/perception/cam_snap.py agentview agent_lift5.png; python3 tools/perception/cam_snap.py frontview front_lift5.png

# openrua op 160
cd /workspace; cat > tools_cloud.py <<'EOF'
import struct, sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
def _grab(node, topic, msg_type, timeout=15.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds/1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds/1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub); return got["m"]
def cloud(cam):
    rclpy.init(); node = rclpy.create_node("cloud"); tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth = _grab(node, f"/{cam}/depth/image_raw", Image); info = _grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width).astype(float)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    u, v = np.meshgrid(np.arange(depth.width), np.arange(depth.height))
    pc = np.stack([(u-cx)*D/fx, (v-cy)*D/fy, D], -1).reshape(-1,3)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds/1e9 + 10
    while node.get_clock().now().nanoseconds/1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()): break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time()); q = t.transform.rotation
    x,y,z,w = q.x,q.y,q.z,q.w
    R = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    T = np.array([t.transform.translation.x,t.transform.translation.y,t.transform.translation.z])
    P = (R@pc.T).T + T; ok = np.isfinite(P).all(1)
    rclpy.shutdown(); return P[ok], (u.reshape(-1)[ok], v.reshape(-1)[ok])
if __name__ == "__main__":
    P,_ = cloud(sys.argv[1]); np.save(sys.argv[2], P); print(P.shape)
EOF
timeout 120 python3 -c "
import numpy as np
from tools_cloud import cloud
P,_=cloud('sideview'); np.save('side_P.npy',P)
S=P[(P[:,0]>-0.42)&(P[:,0]<-0.18)&(P[:,1]>-0.30)&(P[:,1]<-0.05)&(P[:,2]>0.905)&(P[:,2]<1.09)]
print('n',len(S),'zmin',round(S[:,2].min(),3),'zmax',round(S[:,2].max(),3))
for z0 in np.arange(0.93,1.09,0.02):
    s=S[(S[:,2]>=z0)&(S[:,2]<z0+0.02)]
    if len(s)<3: continue
    print(round(z0,2),len(s),'x',np.round([s[:,0].min(),s[:,0].max()],3),'y',np.round([s[:,1].min(),s[:,1].max()],3))
" 2>&1 | grep -v Warn

# openrua op 161
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot(); Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw)); YG=167.3
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
q0=np.array(r.arm_q()); t0=r.tcp_world(q0)[0]
A=np.array([-0.301,-0.167,1.13]); B=np.array([0.117,-0.020,1.13]); C=np.array([0.117,-0.020,1.080])
s1=path_ik(r,t0,A,Q(0,YG),Q(0,YG),2,q0,m=0.15,maxjump=0.5)
s2=path_ik(r,A,B,Q(0,YG),Q(0,YG),12,s1[-1],m=0.15,maxjump=0.5)
s3=path_ik(r,B,C,Q(0,YG),Q(0,YG),3,s2[-1],m=0.15,maxjump=0.5)
print('margins',round(min(margin(q) for q in s1+s2+s3),2))
for q in s2: print(np.round(r.tcp_world(q)[0],3),np.round(q,2))
np.savez('carry4.npz',s1=np.array(s1),s2=np.array(s2),s3=np.array(s3))
" 2>&1 | grep -v Warn

# openrua op 162
cd /workspace; timeout 1500 python3 -u -c "
from rob import *
r=Robot(); Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw)); YG=167.3
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
q0=np.array(r.arm_q()); t0=r.tcp_world(q0)[0]
A=np.array([-0.301,-0.167,1.13]); B=np.array([0.117,-0.020,1.13]); C=np.array([0.117,-0.020,1.080])
s1=path_ik(r,t0,A,Q(0,YG),Q(0,YG),2,q0,m=0.15,maxjump=0.5)
best=None
for dj in (-math.pi/2,math.pi/2,math.pi,-math.pi,-math.pi/4,math.pi/4,-3*math.pi/4,3*math.pi/4):
    qs=s1[-1].copy(); qs[6]+=dj
    if not margin_ok(qs,0.15): print(dj,'spin out of range'); continue
    pos,quat=r.tcp_world(qs)
    try:
        s2=path_ik(r,A,B,quat,quat,12,qs,m=0.15,maxjump=0.5)
        s3=path_ik(r,B,C,quat,quat,3,s2[-1],m=0.15,maxjump=0.5)
    except RuntimeError as e: print(round(dj,2),'fail',str(e)[:60]); continue
    mg=min(margin(q) for q in s2+s3); print(round(dj,2),'OK margin',round(mg,2),'B q',np.round(s2[-1],2))
    if best is None or mg>best[0]: best=(mg,dj,qs,s2,s3)
mg,dj,qs,s2,s3=best; print('BEST dj',round(dj,2),round(mg,2))
np.savez('carry4.npz',s1=np.array(s1),qs=qs,s2=np.array(s2),s3=np.array(s3),dj=dj)
" 2>&1 | grep -v Warn

# openrua op 163
cd /workspace; timeout 1500 python3 -u -c "
from rob import *
r=Robot(); Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw)); YG=167.3
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
q0=np.array(r.arm_q()); t0=r.tcp_world(q0)[0]; print('q0',np.round(q0,2),t0)
A=np.array([-0.301,-0.167,1.20]); B=np.array([0.117,-0.020,1.13]); C=np.array([0.117,-0.020,1.080])
s1=path_ik(r,t0,A,Q(0,YG),Q(0,YG),3,q0,m=0.15,maxjump=0.5)
qa=s1[-1]; print('qa',np.round(qa,2))
rng=np.random.default_rng(3); cands=[]
for yaw in (YG, YG-180, YG-90, YG+90):
  for i in range(25):
    s=r.ik_world(A,Q(0,yaw),seed=nice_seed(A)+rng.normal(0,0.5,7),tcp=True)
    if s is None: continue
    s=np.array(s)
    if not margin_ok(s,0.2) or abs(s[0])>1.2: continue
    try:
        s2=path_ik(r,A,B,Q(0,yaw),Q(0,yaw),12,s,m=0.15,maxjump=0.5)
        s3=path_ik(r,B,C,Q(0,yaw),Q(0,yaw),3,s2[-1],m=0.15,maxjump=0.5)
    except RuntimeError as e: continue
    # sweep check qa->s
    worst_z=9; worst_tilt=0
    for t in np.linspace(0,1,21):
        q=qa+(s-qa)*t; pos,quat=r.tcp_world(q); R=quat_to_R(*quat)
        worst_z=min(worst_z,pos[2]); worst_tilt=max(worst_tilt,math.degrees(math.acos(-R[2,2])))
    mg=min(margin(q) for q in s2+s3)
    print('yaw',yaw,'s',np.round(s,2),'mg',round(mg,2),'sweep minz',round(worst_z,3),'maxtilt',round(worst_tilt,1),'jump',round(np.abs(s-qa).max(),2))
    cands.append((worst_tilt,-mg,yaw,s,s2,s3,worst_z))
cands.sort(key=lambda c:(c[0]>25, -c[1]*0+c[0]))
print('n',len(cands))
if cands:
    c=cands[0]; np.savez('carry4.npz',s1=np.array(s1),yaw=c[2],s=c[3],s2=np.array(c[4]),s3=np.array(c[5]))
    print('chosen yaw',c[2],'tilt',round(c[0],1),'mg',round(-c[1],2),'minz',round(c[6],3))
" 2>&1 | grep -v Warn

# openrua op 164
cd /workspace; timeout 1500 python3 -u -c "
from rob import *
r=Robot(); Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw)); YG=167.3
def margin(q): return float(np.min(np.minimum(q-LIMITS[:,0],LIMITS[:,1]-q)))
q0=np.array(r.arm_q()); t0=r.tcp_world(q0)[0]
A=np.array([-0.301,-0.167,1.30]); B=np.array([0.117,-0.020,1.13]); C=np.array([0.117,-0.020,1.080])
s1=path_ik(r,t0,A,Q(0,YG),Q(0,YG),4,q0,m=0.15,maxjump=0.5); qa=s1[-1]; print('qa',np.round(qa,2),round(margin(qa),2))
s=np.array(r.ik_world(A,Q(0,YG),seed=np.array([-0.52,-0.42,0.09,-2.61,0.04,2.19,-2.6]),tcp=True)); print('s',np.round(s,2),round(margin(s),2))
s2=path_ik(r,A,B,Q(0,YG),Q(0,YG),12,s,m=0.15,maxjump=0.5); s3=path_ik(r,B,C,Q(0,YG),Q(0,YG),3,s2[-1],m=0.15,maxjump=0.5)
print('carry margin',round(min(margin(q) for q in s2+s3),2))
for t in np.linspace(0,1,21):
    q=qa+(s-qa)*t; pos,quat=r.tcp_world(q); R=quat_to_R(*quat); L=links_world(r,q)
    print(round(t,2),'tcp',np.round(pos,3),'tilt',round(math.degrees(math.acos(max(-1,min(1,-R[2,2])))),1),'Zh',np.round(R[:,2],2),'minlink',round(min(L[k][2] for k in L if k not in('panda_link0','panda_link1')),3))
np.savez('carry4.npz',s1=np.array(s1),s=s,s2=np.array(s2),s3=np.array(s3))
" 2>&1 | grep -v Warn

# openrua op 165
cd /workspace; timeout 1200 python3 -u -c "
from rob import *
r=Robot(); P=np.load('carry4.npz')
for q in P['s1']:
    c=r.move_q(q,4); print('up',c[0],round(c[1],3),np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'gap',round(r.finger_gap(),4))
c=r.move_q(P['s'],25); print('switch',c[0],round(c[1],3),np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'gap',round(r.finger_gap(),4),'w',np.round(wrench(r)[:3],1))
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py sideview side_sw.png; python3 tools/perception/cam_snap.py agentview agent_sw.png

# openrua op 166
cd /workspace; sleep 3; timeout 1200 python3 -u -c "
from rob import *
r=Robot(); P=np.load('carry4.npz')
print('cur',np.round(r.tcp_world(np.array(r.arm_q()))[0],3))
for q in P['s1']:
    c=r.move_q(q,4); print('up',c[0],round(c[1],3),np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'gap',round(r.finger_gap(),4))
c=r.move_q(P['s'],25); print('switch',c[0],round(c[1],3),np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'gap',round(r.finger_gap(),4),'w',np.round(wrench(r)[:3],1))
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py sideview side_sw.png; python3 tools/perception/cam_snap.py agentview agent_sw.png

# openrua op 167
cd /workspace; timeout 1200 python3 -u -c "
from rob import *
import time
r=Robot(); P=np.load('carry4.npz')
for q in P['s2']:
    c=r.move_q(q,4); print('carry',c[0],round(c[1],3),np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'gap',round(r.finger_gap(),4),'w',np.round(wrench(r)[:3],1))
time.sleep(2)
w0=wrench(r)[:3]
for q in P['s3']:
    c=r.move_q(q,4); dw=wrench(r)[:3]-w0; print('lower',c[0],round(c[1],3),np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'dw',np.round(dw,1))
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py agentview agent_low.png; python3 tools/perception/cam_snap.py frontview front_low.png

# openrua op 168
cd /workspace; timeout 600 python3 -u -c "
from rob import *
import time
r=Robot(); Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw)); YG=167.3
r.gripper(0.08); time.sleep(2.5); print('gap',round(r.finger_gap(),4))
q0=np.array(r.arm_q()); t0=r.tcp_world(q0)[0]
seg=path_ik(r,t0,t0+np.array([0,0,0.10]),Q(0,YG),Q(0,YG),3,q0,m=0.15,maxjump=0.5)
for q in seg:
    c=r.move_q(q,4); print('up',c[0],round(c[1],3),np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'w',np.round(wrench(r)[:3],1))
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py agentview agent_rel.png; python3 tools/perception/cam_snap.py birdview bird_rel.png

# openrua op 169
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot(); Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw)); YG=167.3
q0=np.array(r.arm_q()); t0=r.tcp_world(q0)[0]
seg=path_ik(r,t0,np.array([-0.15,-0.15,1.25]),Q(0,YG),Q(0,YG),4,q0,m=0.15,maxjump=0.5)
for q in seg: c=r.move_q(q,4)
print('tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'w',np.round(wrench(r)[:3],1))
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py birdview bird_final.png; python3 tools/perception/cam_snap.py agentview agent_final.png; timeout 120 python3 -c "
import numpy as np
from tools_cloud import cloud
P,_=cloud('birdview'); np.save('bird_final_P.npy',P)
S=P[(P[:,2]>1.0)&(P[:,2]<1.2)&(P[:,0]>-0.1)&(P[:,0]<0.4)&(P[:,1]>-0.2)&(P[:,1]<0.3)]
print('high points',len(S))
from collections import Counter
# cluster by rounding
for c,n in Counter([tuple(np.round(p[:2]/0.05)*0.05) for p in S]).most_common(8): print(np.round(c,2),n)
for cx,cy in [(0.15,-0.01),(0.231,0.07)]:
    s=S[(np.abs(S[:,0]-cx)<0.05)&(np.abs(S[:,1]-cy)<0.05)]
    if len(s): print('pot near',cx,cy,'n',len(s),'top z',round(s[:,2].max(),3),'xy of top',np.round(s[s[:,2]>s[:,2].max()-0.01][:,:2].mean(0),3))
" 2>&1 | grep -v Warn

# openrua op 170
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot(); Q=lambda p,yaw: pitched_quat2(math.radians(p),False,math.radians(yaw)); YG=167.3
q0=np.array(r.arm_q()); t0=r.tcp_world(q0)[0]
seg=path_ik(r,t0,np.array([-0.20,-0.20,1.25]),Q(0,YG),Q(0,YG),10,q0,m=0.12,maxjump=0.7)
for q in seg: c=r.move_q(q,4)
print('tcp',np.round(r.tcp_world(np.array(r.arm_q()))[0],3),'q',np.round(r.arm_q(),2),'w',np.round(wrench(r)[:3],1))
" 2>&1 | grep -v Warn; python3 tools/perception/cam_snap.py birdview bird_final.png; python3 tools/perception/cam_snap.py agentview agent_final.png; python3 tools/perception/cam_snap.py frontview front_final.png; timeout 120 python3 -c "
import numpy as np
from tools_cloud import cloud
P,_=cloud('birdview'); np.save('bird_final_P.npy',P)
S=P[(P[:,2]>1.0)&(P[:,2]<1.2)&(P[:,0]>-0.1)&(P[:,0]<0.4)&(P[:,1]>-0.2)&(P[:,1]<0.3)]
print('high points',len(S))
for cx,cy in [(0.15,-0.01),(0.231,0.07)]:
    s=S[(np.abs(S[:,0]-cx)<0.06)&(np.abs(S[:,1]-cy)<0.06)]
    if len(s): print('pot near',cx,cy,'n',len(s),'top z',round(s[:,2].max(),3),'xy of top',np.round(s[s[:,2]>s[:,2].max()-0.01][:,:2].mean(0),3),'lid extent x',np.round([s[:,0].min(),s[:,0].max()],3),'y',np.round([s[:,1].min(),s[:,1].max()],3))
" 2>&1 | grep -v Warn
