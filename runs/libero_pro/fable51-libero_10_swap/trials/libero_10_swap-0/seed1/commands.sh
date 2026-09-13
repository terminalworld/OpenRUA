#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la tools .ros

# openrua op 2
ls -la tools/action tools/perception; cat tools/perception/*.py tools/action/*.py

# openrua op 3
ros2 topic list && echo --- && ros2 action list && echo --- && ros2 service list | grep -v parameter && echo --- && ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('robot0_robotview.png')
cv2.imwrite('crop1.png', cv2.resize(im[70:190,20:130],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop2.png', cv2.resize(im[0:130,150:260],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im2=cv2.imread('agentview.png')
cv2.imwrite('crop3.png', cv2.resize(im2[190:300,190:270],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png; ros2 topic echo /birdview/color/camera_info --once | head -30; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12

# openrua op 7
timeout 15 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -A2 Translation | head -3; ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|child" | head -40

# openrua op 8
cat > /workspace/bird_locate.py <<'EOF'
import numpy as np, cv2
d = np.load('birdview_depth.npy')
fx=fy=579.4112549695428; cx=320; cy=240
def w(u,v):
    z=d[v,u]; X=(u-cx)*z/fx; Y=(v-cy)*z/fy
    return np.array([Y-0.2, X, 3.0-z])
# table height: sample an empty table area
print("table", w(200,200), w(450,300))
# find pixels above table: height > table+0.005 within table region
H = 3.0 - d
tab = np.median(H[180:330, 200:460])
print("table z median", tab)
mask = (H > tab+0.01) & (H < tab+0.4)
mask[:, :] &= np.arange(640)[None,:] > 200
mask[:, :] &= np.arange(480)[:,None] > 200
mask[:, :] &= np.arange(480)[:,None] < 340
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4] < 10: continue
    m = lab==i
    hmax = H[m].max()
    u,v = cent[i]
    print(f"comp {i}: px=({u:.0f},{v:.0f}) area={stats[i,4]} bbox={stats[i,:4]} top_z={hmax:.3f} world_xy={w(int(u),int(v))[:2]}")
EOF
python3 /workspace/bird_locate.py

# openrua op 9
timeout 60 python3 tools/perception/px2world.py agentview 530 300; timeout 60 python3 tools/perception/px2world.py agentview 135 300; timeout 60 python3 tools/perception/px2world.py agentview 232 215

# openrua op 10
ros2 interface show moveit_msgs/srv/GetPositionFK | head -20; ros2 interface show moveit_msgs/srv/GetPositionIK | grep -v "^\s*#" | head -40

# openrua op 11
mkdir -p "$(dirname /workspace/robo.py)"
cat > /workspace/robo.py <<'OPENRUA_EOF'
"""Helper: persistent clients for joint state, FK, IK, trajectory, gripper, servo.

Base frame panda_link0 sits at world (-0.51, 0, 0.42); helpers take WORLD
coordinates and convert. Fingertip (TCP) is 0.1034 m along hand +Z.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE_W = np.array([-0.51, 0.0, 0.42])
TCP = float(M["hand"]["tcp_offset_m"])
# hand pointing straight down, fingers opening along world Y
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down_quat(yaw):
    """Hand pointing down (180 deg about X) then yawed about world Z."""
    # q = qz(yaw) * qx(pi)
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    # qx(pi) = (1,0,0,0); qz = (0,0,sz,cz); product qz*qx:
    # w = cz*0 - sz*0 = 0 ... compute generally
    ax, ay, az, aw = 0.0, 0.0, sz, cz
    bx, by, bz, bw = 1.0, 0.0, 0.0, 0.0
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("robo_helper")
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.update(m=m), 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            lambda m: self._wr.update(m=m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.clear()
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

    def wrench(self):
        self._wr.clear()
        end = time.time() + 5
        while "m" not in self._wr and time.time() < end:
            self.spin(0.2)
        if "m" not in self._wr:
            return None
        f = self._wr["m"].wrench.force
        return np.array([f.x, f.y, f.z])

    def hand_pose_world(self, q=None):
        """FK of panda_hand -> (xyz world, quat xyzw)."""
        q = q or self.arm_q()
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = q
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z,
                     p.orientation.w)

    def tcp_world(self, q=None):
        xyz, quat = self.hand_pose_world(q)
        R = quat_to_R(*quat)
        return xyz + TCP * R[:, 2], quat

    # ---------- acting ----------
    def solve_ik(self, xyz_world, quat=Q_DOWN, at_tcp=True, seed=None):
        xyz = np.array(xyz_world, dtype=float)
        if at_tcp:
            xyz = xyz - TCP * quat_to_R(*quat)[:, 2]
        xyz = xyz - BASE_W
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        (p.orientation.x, p.orientation.y,
         p.orientation.z, p.orientation.w) = quat
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = seed or self.arm_q()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_joints(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via, 1):
                t = seconds * i / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  traj error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def move_to(self, xyz_world, quat=Q_DOWN, seconds=3.0, at_tcp=True):
        q = self.solve_ik(xyz_world, quat, at_tcp)
        if q is None:
            print(f"  IK FAILED for {xyz_world}", flush=True)
            return None
        code, err = self.move_joints(q, seconds)
        tcp, _ = self.tcp_world()
        print(f"  tcp now {tcp.round(4)} (target {np.round(xyz_world,4)})", flush=True)
        return tcp

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def servo(self, vx=0.0, vy=0.0, vz=0.0, ticks=20, hz=20):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(1.0 / hz)
        return self.tcp_world()[0]
OPENRUA_EOF

# openrua op 12
timeout 120 python3 -u -c "
from robo import *
r = Robot()
print('q', np.round(r.arm_q(),3))
xyz, quat = r.hand_pose_world()
print('hand world', xyz.round(4), np.round(quat,4))
print('tcp world', r.tcp_world()[0].round(4))
print('gap', r.finger_gap())
print('wrench', r.wrench())
# IK test for above alphabet soup
q = r.solve_ik([-0.239, -0.147, 0.60])
print('ik above soup', q and np.round(q,3))
"

# openrua op 13
python3 -c "
import numpy as np
def dh(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
def fk(q):
    P=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
    T=np.eye(4)
    for (a,d,al),th in zip(P,q): T=T@dh(a,d,al,th)
    T=T@dh(0,0.107,0,0)  # flange
    T=T@dh(0,0.1034,0,-np.pi/4) # hand (rotated -45deg) + tcp... approx
    return T
q=[0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483]
T=fk(q); print('tcp base-frame', T[:3,3].round(4)); print(T[:3,:3].round(3))
"


# openrua op 14
python3 - <<'EOF'
import re
s=open('robo.py').read()
s=s.replace("BASE_W = np.array([-0.51, 0.0, 0.42])","BASE_W = np.array([0.0, 0.0, 0.0])  # FK/IK services already work in world frame (verified)")
open('robo.py','w').write(s)
EOF
timeout 120 python3 -u -c "
from robo import *
r = Robot()
print('tcp world', r.tcp_world()[0].round(4))
q = r.solve_ik([-0.239, -0.147, 0.60])
print('ik above soup', q and np.round(q,3))
if q: print('fk of sol', r.tcp_world(q)[0].round(4))
q = r.solve_ik([-0.011, -0.245, 0.60])
print('ik above sauce', q and np.round(q,3))
q = r.solve_ik([-0.005, 0.277, 0.70])
print('ik above basket', q and np.round(q,3))
"

# openrua op 15
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); H=3.0-d
for name,(u0,v0,u1,v1) in {'soup':(275,220,300,245),'sauce':(250,272,280,298),'ketchup':(276,252,300,272),'basket':(360,262,410,310)}.items():
    sub=H[v0:v1,u0:u1]; top=sub.max()
    m=sub>top-0.012
    z=d[v0:v1,u0:u1][m].mean()
    n=m.sum(); px=z/579.41
    print(name,'top',round(top,3),'npx',n,'equiv diam m',round(2*np.sqrt(n/np.pi)*px,4),'px size',round(px,4))
    vs,us=np.nonzero(m); print('   top-surface centroid world', round((vs.mean()+v0-240)*z/579.41-0.2,4), round((us.mean()+u0-320)*z/579.41,4))
"


# openrua op 16
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); H=3.0-d
np.set_printoptions(linewidth=250, precision=2)
print(H[262:312:3, 358:412:3])
"


# openrua op 17
mkdir -p "$(dirname /workspace/pick_soup.py)"
cat > /workspace/pick_soup.py <<'OPENRUA_EOF'
from robo import *
r = Robot()
SOUP = np.array([-0.2387, -0.1451])
HOVER, GRASP = 0.62, 0.455

print("open gripper"); r.gripper(0.04)
print("hover above soup"); r.move_to([*SOUP, HOVER], seconds=4)
print("descend");
for z in (0.56, 0.50, GRASP):
    r.move_to([*SOUP, z], seconds=1.5)
print("wrench before", r.wrench())
print("close"); gap = r.gripper(0.0)
print("wrench after", r.wrench())
print("GRASP_GAP", gap)
if gap > 0.03:
    print("lift"); r.move_to([*SOUP, 0.70], seconds=3)
    print("gap after lift", r.finger_gap())
print("DONE")
OPENRUA_EOF

# openrua op 18
nohup python3 -u pick_soup.py > pick_soup.log 2>&1 &
sleep 45; cat /workspace/pick_soup.log

# openrua op 19
sleep 30; cat /workspace/pick_soup.log

# openrua op 20
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 -u -c "
from robo import *
r=Robot(); print('tcp', r.tcp_world()[0].round(4), 'gap', round(r.finger_gap(),4), 'wrench', r.wrench().round(2))"

# openrua op 21
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robo.py", "old_string": "        code = res.result().result.error_code\\n        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()\\n        print(f\\"  traj error_code={code} max_joint_err={err:.4f}\\", flush=True)\\n        return code, err", "new_string": "        code = res.result().result.error_code\\n        err = self._settle_err(q)\\n        print(f\\"  traj error_code={code} max_joint_err={err:.4f}\\", flush=True)\\n        if err > 0.02 and retries > 0:\\n            print(\\"  resending (controller lag)\\", flush=True)\\n            return self.move_joints(q, max(seconds, 2.0), retries=retries - 1)\\n        return code, err\\n\\n    def _settle_err(self, q):\\n        \\"\\"\\"Re-read joints until they stop changing; return max |q - target|.\\"\\"\\"\\n        prev = np.array(self.arm_q())\\n        for _ in range(20):\\n            cur = np.array(self.arm_q())\\n            if np.abs(cur - prev).max() < 1e-4:\\n                break\\n            prev = cur\\n        return float(np.abs(cur - np.array(q)).max())", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 22
python3 - <<'EOF'
s=open('robo.py').read()
s=s.replace("def move_joints(self, q, seconds=3.0, via=None):","def move_joints(self, q, seconds=3.0, via=None, retries=1):")
open('robo.py','w').write(s)
EOF
cat > place_soup.py <<'EOF'
from robo import *
r = Robot()
BASKET = np.array([-0.013, 0.252])
tcp = r.tcp_world()[0]
print("gap", r.finger_gap())
print("raise"); r.move_to([tcp[0], tcp[1], 0.76], seconds=2.5)
print("over basket"); r.move_to([*BASKET, 0.76], seconds=4)
print("gap", r.finger_gap())
print("lower"); r.move_to([*BASKET, 0.63], seconds=2.5)
print("gap", r.finger_gap(), "wrench", r.wrench())
print("release"); r.gripper(0.04)
print("retreat"); r.move_to([*BASKET, 0.80], seconds=2.5)
print("DONE")
EOF
nohup python3 -u place_soup.py > place_soup.log 2>&1 &
sleep 60; cat place_soup.log

# openrua op 23
sleep 40; cat place_soup.log

# openrua op 24
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png; python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); H=3.0-d
np.set_printoptions(linewidth=250, precision=2)
print(H[262:312:3, 358:412:3])"

# openrua op 25
cat > pick_sauce.py <<'EOF'
from robo import *
r = Robot()
SAUCE = np.array([-0.0072, -0.2464])
BASKET2 = np.array([-0.013, 0.295])
print("open"); r.gripper(0.04)
print("hover"); r.move_to([*SAUCE, 0.66], seconds=4)
print("descend")
for z in (0.58, 0.52, 0.485):
    r.move_to([*SAUCE, z], seconds=1.5)
print("wrench", r.wrench())
print("close"); gap = r.gripper(0.0)
print("GRASP_GAP", gap)
if gap < 0.03:
    print("GRASP FAILED"); raise SystemExit(1)
print("lift"); r.move_to([*SAUCE, 0.76], seconds=3)
print("gap", r.finger_gap())
print("over basket"); r.move_to([*BASKET2, 0.76], seconds=4)
print("gap", r.finger_gap())
print("lower"); r.move_to([*BASKET2, 0.68], seconds=2)
print("gap", r.finger_gap(), "wrench", r.wrench())
print("release"); r.gripper(0.04)
print("retreat"); r.move_to([-0.15, 0.10, 0.85], seconds=3)
print("DONE")
EOF
nohup python3 -u pick_sauce.py > pick_sauce.log 2>&1 &
sleep 75; cat pick_sauce.log

# openrua op 26
sleep 60; cat pick_sauce.log

# openrua op 27
sleep 20; tail -3 pick_sauce.log; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py birdview; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png

# openrua op 28
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); H=3.0-d
np.set_printoptions(linewidth=250, precision=2)
print(H[262:312:3, 358:412:3])
# anything above table outside basket bbox in the right half?
mask=(H>0.435)&(H<0.9); mask[:, :355]=False; mask[:262]=False; mask[315:]=False
print('above-table px near basket (incl. basket):', mask.sum())
im=cv2.imread('birdview.png'); cv2.imwrite('bird_crop.png', cv2.resize(im[240:330,340:430],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 29
timeout 200 python3 -u -c "
from robo import *
r=Robot()
r.move_to([-0.15,0.10,0.90], seconds=2); r.move_to([-0.15,0.10,0.85], seconds=2)
print('gap', r.finger_gap())
" && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth2.png && python3 -c "
import numpy as np
a=np.load('birdview_depth.npy'); b=np.load('birdview_depth2.npy')
H=3.0-b
np.set_printoptions(linewidth=250, precision=2)
print(H[268:306:3, 364:406:3])
print('max change in basket region (m):', np.abs(a-b)[262:315,358:412].max().round(3))
"

# openrua op 30
python3 -c "
import numpy as np
a=np.load('birdview_depth.npy'); b=np.load('birdview_depth2.npy')
diff=np.abs(a-b)[262:315,358:412]
vs,us=np.nonzero(diff>0.01); print('changed px:', len(vs), 'rows', vs.min()+262 if len(vs) else None, vs.max()+262 if len(vs) else None, 'cols', us.min()+358 if len(vs) else None, us.max()+358 if len(vs) else None)
print('max diff excluding first 8 rows/cols:', diff[8:,8:].max().round(4))
"
