#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md

# openrua op 2
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo --- && ros2 action list && echo --- && ros2 service list | grep -iv parameter && echo --- && ros2 node list

# openrua op 4
for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_cans.png', cv2.resize(im[220:320,200:400],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('robot0_robotview.png'); cv2.imwrite('crop_rv.png', cv2.resize(im[30:140,160:440],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 60 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; timeout 60 python3 tools/perception/px2world.py agentview 340 355; timeout 60 python3 tools/perception/px2world.py agentview 236 258; timeout 60 python3 tools/perception/px2world.py agentview 520 260

# openrua op 7
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a camera's depth frame as a world-frame point cloud (.npz) plus
a height-above-table mask image. Usage: python3 cloud.py <camera>"""
import struct, sys
import numpy as np, rclpy, cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge


def grab(node, topic, T):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
buf = Buffer(); TransformListener(buf, node)
d = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
frame = f"{cam}_optical_frame"
while not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation
R = quat_R(q.x, q.y, q.z, q.w)
p0 = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
h, w = d.shape
u, v = np.meshgrid(np.arange(w), np.arange(h))
pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
P = pc @ R.T + p0
np.savez(f"{cam}_cloud.npz", P=P, depth=d)
print("cam pos", p0, "R", R.round(3).tolist())
z = P[..., 2]
print("z percentiles", np.nanpercentile(z[np.isfinite(z)], [1, 5, 25, 50, 75, 95, 99]).round(3))
rclpy.shutdown()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview

# openrua op 9
python3 -c "
import numpy as np, cv2
D=np.load('birdview_cloud.npz'); P=D['P']; z=P[...,2]
tab=np.isfinite(z)&(z>0.415)&(z<0.435)
print('table z median', np.median(z[tab]))
m=((z>0.436)&(z<0.75)).astype('uint8')
n,lab,st,cen=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    if st[i,4]<4: continue
    sel=lab==i; pts=P[sel]
    print(i,'px',cen[i].round(0),'area',st[i,4],'x[%.3f %.3f] y[%.3f %.3f] ztop %.3f'%(pts[:,0].min(),pts[:,0].max(),pts[:,1].min(),pts[:,1].max(),np.percentile(pts[:,2],95)))
"


# openrua op 10
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small pick/place driver for this Panda. Poses are given in the WORLD
frame for the TCP (fingertip point), converted to the arm base for IK.

Usage: python3 arm.py <cmd> [args] ...   (several commands in sequence)
  goto X Y Z [T]      TCP to world (X,Y,Z), hand pointing down, fingers
                      along world y; T seconds (default 3)
  gotoyaw X Y Z YAW [T]  same with the hand yawed YAW rad about world z
  open | close        gripper
  js                  print arm joints + finger gap
"""
import math, sys, time
import numpy as np, rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GR = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE = np.array([-0.510, 0.0, 0.420])  # world -> panda_link0 (tf2_echo)


class Arm:
    def __init__(self):
        self.node = rclpy.create_node("arm")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self.js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GR["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)

    def joints(self):
        self.js.pop("m", None)
        while "m" not in self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self.js["m"]
        d = dict(zip(m.name, m.position))
        return [d[j] for j in JOINTS], d

    def gap(self):
        _, d = self.joints()
        return abs(d["panda_finger_joint1"]) + abs(d["panda_finger_joint2"])

    def solve(self, xyz_world, yaw=0.0):
        # hand pointing down: 180deg about x, then yaw about world z
        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
        # q = qz(yaw) * qx(pi)  -> (x,y,z,w)
        q = (cy, sy, 0.0, 0.0)  # derived: qz*qx with qx=(1,0,0,0)
        R = _R(*q)
        hand = np.array(xyz_world) - TCP * R[:, 2]
        p = hand - BASE
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = q
        cur, _ = self.joints()
        seed = JointState(); seed.name = list(JOINTS); seed.position = cur
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"IK failed for {xyz_world}: "
                             f"{None if res is None else res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def move(self, positions, secs):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(x) for x in positions])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        cur, _ = self.joints()
        err = max(abs(a - b) for a, b in zip(cur, positions))
        print(f"  move: error_code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def goto(self, xyz, yaw=0.0, secs=3.0):
        print(f"goto {xyz} yaw={yaw:.2f}", flush=True)
        sol = self.solve(xyz, yaw)
        code, err = self.move(sol, secs)
        if err > 0.02:
            print("  retrying same goal", flush=True)
            self.move(sol, secs)

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GR["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        print(f"gripper {width}: reached={r.reached_goal} stalled={r.stalled} "
              f"gap={self.gap():.4f}", flush=True)


def _R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    rclpy.init()
    arm = Arm()
    a = sys.argv[1:]
    while a:
        c = a.pop(0)
        if c == "goto":
            x, y, z = map(float, a[:3]); a = a[3:]
            t = float(a.pop(0)) if a and _isnum(a[0]) else 3.0
            arm.goto((x, y, z), 0.0, t)
        elif c == "gotoyaw":
            x, y, z, yaw = map(float, a[:4]); a = a[4:]
            t = float(a.pop(0)) if a and _isnum(a[0]) else 3.0
            arm.goto((x, y, z), yaw, t)
        elif c == "open":
            arm.gripper(GR["open_m"])
        elif c == "close":
            arm.gripper(GR["closed_m"])
        elif c == "js":
            j, d = arm.joints()
            print("joints", [round(v, 3) for v in j], "gap", round(arm.gap(), 4))
        else:
            raise SystemExit(f"unknown cmd {c}")
    rclpy.shutdown()


def _isnum(s):
    try: float(s); return True
    except ValueError: return False


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 11
nohup python3 -u arm.py js open goto -0.095 -0.128 0.60 4 js > step1.log 2>&1 &
sleep 45; cat step1.log

# openrua op 12
cat > fk.py <<'EOF'
import rclpy, sys
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node("fk")
c=n.create_client(GetPositionFK,"/compute_fk"); c.wait_for_service(10)
r=GetPositionFK.Request(); r.fk_link_names=["panda_hand","panda_link8"]
r.robot_state.joint_state=JointState(name=[f"panda_joint{i}" for i in range(1,8)],position=[0.0,-0.161,0.0,-2.445,0.0,2.227,0.785])
f=c.call_async(r); rclpy.spin_until_future_complete(n,f,timeout_sec=30)
res=f.result()
print(res.error_code.val)
for nm,ps in zip(res.fk_link_names,res.pose_stamped):
    p=ps.pose.position;q=ps.pose.orientation
    print(nm, ps.header.frame_id, round(p.x,3),round(p.y,3),round(p.z,3), [round(v,3) for v in (q.x,q.y,q.z,q.w)])
EOF
timeout 60 python3 fk.py

# openrua op 13
sed -i 's/^BASE = np.array(\[-0.510, 0.0, 0.420\]).*/BASE = np.array([0.0, 0.0, 0.0])  # FK\/IK model frame already == world (checked with \/compute_fk)/' arm.py && grep -n "^BASE" arm.py
nohup python3 -u arm.py goto -0.095 -0.128 0.60 4 js > step1.log 2>&1 &
sleep 60; cat step1.log

# openrua op 14
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 fk.py 2>/dev/null | head -0; python3 - <<'EOF'
import rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node("fk")
got={}
n.create_subscription(JointState,"/joint_states",lambda m: got.setdefault("m",m),1)
while "m" not in got: rclpy.spin_once(n,timeout_sec=0.2)
m=got["m"]; d=dict(zip(m.name,m.position))
c=n.create_client(GetPositionFK,"/compute_fk"); c.wait_for_service(10)
r=GetPositionFK.Request(); r.fk_link_names=["panda_hand"]
r.robot_state.joint_state=JointState(name=[f"panda_joint{i}" for i in range(1,8)],position=[d[f"panda_joint{i}"] for i in range(1,8)])
f=c.call_async(r); rclpy.spin_until_future_complete(n,f,timeout_sec=30)
ps=f.result().pose_stamped[0]; p=ps.pose.position; q=ps.pose.orientation
print("hand", round(p.x,3),round(p.y,3),round(p.z,3), [round(v,3) for v in (q.x,q.y,q.z,q.w)])
EOF

# openrua op 15
python3 - <<'EOF'
import re
s=open('/workspace/arm.py').read()
s=s.replace("        cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)\n",
"        # IK tip link is panda_link8, which sits 45deg (about z) off\n        # panda_hand; compensate so `yaw` is the HAND's world yaw\n        yaw8 = yaw - math.pi / 4\n        cy, sy = math.cos(yaw8 / 2), math.sin(yaw8 / 2)\n")
open('/workspace/arm.py','w').write(s)
EOF
grep -n "yaw8" arm.py

# openrua op 16
nohup python3 -u arm.py goto -0.095 -0.128 0.60 2 goto -0.095 -0.128 0.465 3 close js > step2.log 2>&1 &
sleep 75; cat step2.log

# openrua op 17
nohup python3 -u arm.py goto -0.095 -0.128 0.72 3 js goto -0.013 0.25 0.72 4 js > step3.log 2>&1 &
sleep 80; cat step3.log

# openrua op 18
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 19
nohup python3 -u arm.py open js > step4.log 2>&1 &
sleep 30; cat step4.log; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 20
nohup python3 -u arm.py goto 0.049 0.027 0.60 4 js > step5.log 2>&1 &
sleep 60; cat step5.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 21
nohup python3 -u arm.py goto 0.049 0.027 0.50 3 goto 0.049 0.027 0.44 3 js close js > step6.log 2>&1 &
sleep 80; cat step6.log

# openrua op 22
nohup python3 -u arm.py goto 0.049 0.027 0.65 3 js goto -0.013 0.25 0.72 4 js > step7.log 2>&1 &
sleep 75; cat step7.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 23
nohup python3 -u arm.py open goto -0.10 0.10 0.75 3 js > step8.log 2>&1 &
sleep 50; cat step8.log; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 24
timeout 120 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np
D=np.load('birdview_cloud.npz'); P=D['P']; z=P[...,2]
# basket interior footprint from earlier segmentation
sel=np.isfinite(z)&(P[...,0]>-0.08)&(P[...,0]<0.055)&(P[...,1]>0.18)&(P[...,1]<0.32)
zz=z[sel]; print('inside-basket z percentiles', np.percentile(zz,[5,25,50,75,95]).round(3))
# anything left on the table at the old butter / soup spots?
for name,(x0,x1,y0,y1) in {'butter spot':(0.0,0.10,0.0,0.06),'soup spot':(-0.14,-0.05,-0.17,-0.09)}.items():
    s=np.isfinite(z)&(P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)
    print(name,'max z',z[s].max().round(3))
"; python3 -c "
import cv2; im=cv2.imread('birdview.png'); cv2.imwrite('crop_basket.png', cv2.resize(im[250:320,340:420],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))"

# openrua op 25
nohup python3 -u arm.py goto -0.013 0.25 0.85 3 > step9.log 2>&1 & sleep 40; cat step9.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 26
python3 -c "
import cv2; im=cv2.imread('robot0_eye_in_hand.png'); cv2.imwrite('crop_b2.png', cv2.resize(im[200:350,230:400],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 27
for c in frontview sideview robot0_robotview galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 28
python3 -c "
import numpy as np, cv2
D=np.load('birdview_cloud.npz'); P=D['P']
print('px(580,268) world', P[268,580].round(3), 'px(575,262)', P[262,575].round(3))
im=cv2.imread('birdview.png'); cv2.imwrite('crop_floor.png', cv2.resize(im[250:290,560:600],None,fx=8,fy=8,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('galleryview.png'); cv2.imwrite('crop_floor2.png', cv2.resize(im[375:405,420:455],None,fx=8,fy=8,interpolation=cv2.INTER_CUBIC))
"

# openrua op 29
ros2 topic list | grep -iE "cmd_vel|odom" ; echo "no base topics: $?"; timeout 60 python3 -c "
import sys; sys.argv=['arm.py']; sys.path.insert(0,'/workspace')
import rclpy, arm
rclpy.init(); a=arm.Arm()
for p in [(-0.06,1.34,0.03),(-0.06,1.34,0.20),(-0.3,1.0,0.1)]:
    try: a.solve(p); print('IK ok', p)
    except SystemExit as e: print(e)
"

# openrua op 30
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Non-obvious facts about the /workspace Panda arm sim (IK frame is world, tip link is panda_link8, high drops bounce objects out of the basket)
metadata:
  type: project
---

Franka Panda workstation sim in /workspace (Sept 2026 sessions):

- `/compute_ik` and `/compute_fk` with empty `frame_id` work in the WORLD
  frame here (world→panda_link0 = (-0.51, 0, 0.42)); docs/30-action.md's
  "model frame is the arm base" note does NOT apply — base-frame coords
  give IK error -31.
- The IK tip link is `panda_link8`, which is 45° about z off `panda_hand`;
  command link8 yaw = desired hand yaw − π/4 to get the fingers where you
  expect.
- Table top is at world z≈0.425; the wicker basket rim is ~0.62 (20 cm
  deep). Releasing a light object (butter box) from TCP z=0.72 over the
  basket made it bounce off the can already inside and fly ~1.1 m off
  the table, out of reach forever (fixed base, no odom/cmd_vel).

**Why:** cost a full task once — the butter ended on the floor at
(-0.06, 1.34, 0).
**How to apply:** lower the TCP to just above the basket floor / contents
(≤5 cm) before opening the gripper; first trajectory attempt often
returns error -5 with big joint error, resend converges ([[arm-driver-script]]).
OPENRUA_EOF

# openrua op 31
cd /claude-config/projects/-workspace/memory && (cat MEMORY.md 2>/dev/null; true) && echo "- [Panda workstation quirks](panda-workstation-quirks.md) — IK frame is world, link8 45° off hand, release low into basket" >> MEMORY.md && cat MEMORY.md
