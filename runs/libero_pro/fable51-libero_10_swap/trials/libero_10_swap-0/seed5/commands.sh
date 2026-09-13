#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 3
for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 30 ros2 topic echo /joint_states --once

# openrua op 4
python3 -c "
import cv2
im=cv2.imread('robot0_robotview.png')
cv2.imwrite('crop1.png', cv2.resize(im[0:280, 20:280], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
im2=cv2.imread('agentview.png')
cv2.imwrite('crop2.png', cv2.resize(im2[170:360, 90:300], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
im3=cv2.imread('robot0_eye_in_hand.png')
cv2.imwrite('crop3.png', cv2.resize(im3[130:380, 380:640], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
"

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('sideview.png')
cv2.imwrite('crop4.png', cv2.resize(im[380:460, 230:420], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
python3 -c "
import cv2
im=cv2.imread('birdview.png')
c=im[190:330, 230:420]
c=cv2.resize(c, None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC)
# draw grid every 10px original
for i in range(0, c.shape[1], 40): cv2.line(c,(i,0),(i,c.shape[0]),(0,255,0),1); cv2.putText(c,str(230+i//4),(i,12),0,0.35,(0,255,255),1)
for j in range(0, c.shape[0], 40): cv2.line(c,(0,j),(c.shape[1],j),(0,255,0),1); cv2.putText(c,str(190+j//4),(0,j+12),0,0.35,(0,255,255),1)
cv2.imwrite('crop5.png', c)
"

# openrua op 7
for p in "279 222" "265 286" "383 285" "383 260" "250 250" "383 330"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p 2>&1 | tail -1; done

# openrua op 8
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -12; echo ---; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -40

# openrua op 9
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
print(d.shape, np.nanmin(d), np.nanmax(d))
# table depth near objects
print('table depth', d[250,250], d[330,383])
for name,(u,v) in {'soup':(279,222),'sauce':(265,286),'basket_c':(383,285)}.items():
    print(name, d[v,u])
# soup extent: pixels with depth < table-0.03 in a window
def extent(u,v,thr):
    w=d[v-20:v+21,u-20:u+21]
    m=w<thr
    ys,xs=np.where(m)
    print(' cols',xs.min()+u-20, xs.max()+u-20,' rows',ys.min()+v-20,ys.max()+v-20, 'center', xs.mean()+u-20, ys.mean()+v-20, 'min depth', w[m].min())
extent(279,222,d[250,250]-0.03)
extent(265,286,d[250,250]-0.03)
# basket rim: profile along row 285
row=d[285,340:420]; print('row285', np.round(row,3))
col=d[250:320,383]; print('col383', np.round(col,3))
"

# openrua op 10
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
def extent(u,v,thr,r=20):
    w=d[v-r:v+r+1,u-r:u+r+1]
    m=w<thr
    ys,xs=np.where(m)
    print(' cols',xs.min()+u-r, xs.max()+u-r,' rows',ys.min()+v-r,ys.max()+v-r, 'center', xs.mean()+u-r, ys.mean()+v-r, 'n',m.sum())
extent(265,286,2.49)
extent(279,222,2.51)
# also what's around the sauce: print depth map coarse
np.set_printoptions(linewidth=250)
print(np.round(d[262:300:2, 250:292:2],2))
"

# openrua op 11
timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A3 "^k:"; timeout 20 ros2 topic echo /clock --once

# openrua op 12
for p in "280 224" "265 289"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p 2>&1 | tail -1; done

# openrua op 13
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Persistent-client helper: run a sequence of commands in one process.

Usage: python3 -u robot.py "<cmd>" "<cmd>" ...
Commands (world-frame metres; TCP = fingertip point; top-down grasp):
  goto X Y Z [yaw=0] [secs=3]   IK to TCP pose, then trajectory; verifies
  grip open|close               gripper action, prints finger gap after
  pose                          FK -> TCP world pose + finger gap
  joints                        print arm joint positions
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
# world -> panda_link0 (from tf_static): identity rotation, this offset
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self.js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        assert self.fjt.wait_for_server(10), "no fjt server"
        assert self.grip.wait_for_server(10), "no gripper server"
        assert self.ik.wait_for_service(10), "no ik"
        assert self.fk.wait_for_service(10), "no fk"
        self.wait_js()

    def _on_js(self, m):
        self.js = m

    def wait_js(self):
        self.js = None
        while self.js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(zip(self.js.name, self.js.position))

    def arm_state(self):
        d = self.wait_js()
        s = JointState()
        s.name = list(ARM)
        s.position = [d[j] for j in ARM]
        return s, d

    def finger_gap(self, d):
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def call(self, cli, req, timeout=60):
        f = cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=timeout)
        return f.result()

    def tcp_pose(self):
        seed, d = self.arm_state()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = seed
        res = self.call(self.fk, req)
        p = res.pose_stamped[0].pose
        R = quat_R(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        hand = np.array([p.position.x, p.position.y, p.position.z])
        tcp = hand + TCP * R[:, 2]
        return tcp + BASE_IN_WORLD, R, d

    def goto(self, x, y, z, yaw=0.0, secs=3.0):
        q = (math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0)  # Rz(yaw)*Rx(pi)
        R = quat_R(*q)
        tcp_b = np.array([x, y, z]) - BASE_IN_WORLD
        hand_b = tcp_b - TCP * R[:, 2]
        seed, _ = self.arm_state()
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        res = self.call(self.ik, req)
        if res is None or res.error_code.val != 1:
            print(f"IK FAILED code={None if res is None else res.error_code.val}")
            return False
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        target = [sol[j] for j in ARM]
        cur = [seed.position[i] for i in range(len(ARM))]
        jump = max(abs(a - b) for a, b in zip(target, cur))
        print(f"IK ok, max joint jump {jump:.2f} rad")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=target)
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        t0 = time.time()
        f = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code if rf.result() else "timeout"
        _, d = self.arm_state()
        err = max(abs(d[j] - t) for j, t in zip(ARM, target))
        tcp, _, _ = self.tcp_pose()
        print(f"traj code={code} ({time.time()-t0:.0f}s) joint err {err:.3f} "
              f"TCP world {tcp.round(3)}")
        return True

    def gripper(self, open_):
        goal = GripperCommand.Goal()
        goal.command.position = GRIP["open_m"] if open_ else GRIP["closed_m"]
        goal.command.max_effort = float(GRIP["max_effort"])
        f = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=60)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result if rf.result() else None
        _, d = self.arm_state()
        print(f"gripper {'open' if open_ else 'close'}: reached={getattr(r,'reached_goal',None)} "
              f"stalled={getattr(r,'stalled',None)} gap={self.finger_gap(d):.4f}")


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    r = Robot()
    for cmd in sys.argv[1:]:
        parts = cmd.split()
        print(f">>> {cmd}", flush=True)
        if parts[0] == "goto":
            args = [float(v) for v in parts[1:]]
            x, y, z = args[:3]
            yaw = args[3] if len(args) > 3 else 0.0
            secs = args[4] if len(args) > 4 else 3.0
            if not r.goto(x, y, z, yaw, secs):
                print("ABORT sequence"); break
        elif parts[0] == "grip":
            r.gripper(parts[1] == "open")
        elif parts[0] == "pose":
            tcp, R, d = r.tcp_pose()
            print(f"TCP world {tcp.round(4)} hand z-axis {R[:,2].round(3)} "
                  f"gap={r.finger_gap(d):.4f}")
        elif parts[0] == "joints":
            _, d = r.arm_state()
            print({j: round(d[j], 4) for j in ARM})
        sys.stdout.flush()
    print("DONE", flush=True)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 14
timeout 120 python3 -u robot.py pose joints 2>&1 | tail -5

# openrua op 15
cat > /tmp/fktest.py <<'EOF'
import rclpy, sys
sys.path.insert(0,'/workspace')
from robot import *
r = Robot()
seed, d = r.arm_state()
req = GetPositionFK.Request(); req.fk_link_names=["panda_link0","panda_hand"]; req.robot_state.joint_state = seed
res = r.call(r.fk, req)
for ps in res.pose_stamped: print(ps.header.frame_id, ps.pose.position)
# IK test: hand pose in base frame vs world frame
for label, pos in [("base",(0.457,0.0,0.358)),("world",(-0.053,0.0,0.778))]:
    req = GetPositionIK.Request()
    req.ik_request.group_name="panda_arm"; req.ik_request.pose_stamped.header.frame_id=""
    p=req.ik_request.pose_stamped.pose; p.position.x,p.position.y,p.position.z=pos
    p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=(1.0,0.0,-0.028,0.0)
    req.ik_request.robot_state.joint_state=seed; req.ik_request.timeout.sec=2
    res=r.call(r.ik,req)
    sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position))
    print(label, res.error_code.val, {j:round(sol.get(j,9),3) for j in ARM})
rclpy.shutdown()
EOF
timeout 120 python3 -u /tmp/fktest.py 2>&1 | tail -8

# openrua op 16
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "# world -> panda_link0 (from tf_static): identity rotation, this offset\\nBASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])", "new_string": "# verified empirically: /compute_fk and /compute_ik on this machine work\\n# in the WORLD frame (FK of panda_link0 = (-0.51, 0, 0.42)), so no offset\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 17
timeout 900 python3 -u robot.py "grip open" "goto -0.269 -0.173 0.62 0 3" pose > run1.log 2>&1; tail -8 run1.log

# openrua op 18
timeout 600 python3 -u robot.py joints "goto -0.269 -0.173 0.62 0 4" joints pose > run2.log 2>&1; tail -8 run2.log

# openrua op 19
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png

# openrua op 20
timeout 900 python3 -u robot.py "goto -0.269 -0.173 0.50 0 3" "goto -0.269 -0.173 0.45 0 2" pose > run3.log 2>&1; tail -8 run3.log

# openrua op 21
timeout 120 python3 -u robot.py joints 2>&1 | grep panda; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A7 "^wrench"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png

# openrua op 22
timeout 900 python3 -u robot.py "goto -0.269 -0.173 0.45 0 2" joints > run4.log 2>&1; tail -5 run4.log

# openrua op 23
timeout 900 python3 -u robot.py "goto -0.269 -0.173 0.50 0 2" pose > run5.log 2>&1; tail -3 run5.log; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 "force"

# openrua op 24
timeout 20 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 Translation | head -5; timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A9 "^k:"

# openrua op 25
timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_d.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png >/dev/null && python3 -c "
import numpy as np
d=np.load('eih_d.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
np.set_printoptions(linewidth=250)
print(np.round(d[200:400:10, 100:560:20],3))
"

# openrua op 26
python3 -c "
import numpy as np, cv2
d=np.load('eih_d.npy')
m=((d>0.08)&(d<0.12)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    x,y,w,h,a=stats[i]
    if a>500: print(i, 'bbox',x,y,w,h,'area',a,'centroid',cent[i].round(1), 'mindepth', d[lab==i].min().round(4))
"

# openrua op 27
timeout 900 python3 -u robot.py "goto -0.269 -0.175 0.48 0 2" "grip close" pose > run6.log 2>&1; tail -6 run6.log; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 "force"

# openrua op 28
timeout 900 python3 -u robot.py "goto -0.269 -0.175 0.68 0 3" pose > run7.log 2>&1; tail -4 run7.log; timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png

# openrua op 29
timeout 1200 python3 -u robot.py "goto -0.236 -0.199 0.76 0 3" "goto -0.10 0.05 0.76 0 4" "goto 0.006 0.277 0.76 0 4" pose > run8.log 2>&1; grep -E "traj|TCP|IK" run8.log

# openrua op 30
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent4.png

# openrua op 31
timeout 1200 python3 -u robot.py "goto 0.006 0.277 0.62 0 3" pose "grip open" "goto 0.006 0.277 0.76 0 3" pose > run9.log 2>&1; grep -E "traj|TCP|IK|gripper" run9.log; timeout 60 python3 tools/perception/cam_snap.py agentview agent5.png; timeout 60 python3 tools/perception/cam_snap.py birdview bird5.png

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird5_depth.png >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('bird5_depth.npy')
# basket interior region rows 270-305, cols 363-402
b=d[272:303, 366:400]
print('basket interior min depth', b.min().round(4), '-> z', (0.425+2.575-b.min()).round(4))
ys,xs=np.where(b<2.46); print('object in basket cols',xs.min()+366,xs.max()+366,'rows',ys.min()+272,ys.max()+272)
# tomato sauce still there?
w=d[270:305,250:285]; ys,xs=np.where(w<2.50); print('sauce cols',xs.min()+250,xs.max()+250,'rows',ys.min()+270,ys.max()+270,'center',(xs.mean()+250).round(1),(ys.mean()+270).round(1),'top z',(0.425+2.575-w.min()).round(4))
im=cv2.imread('bird5.png'); cv2.imwrite('crop6.png', cv2.resize(im[190:330,230:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 33
timeout 1200 python3 -u robot.py "goto 0.0 0.0 0.76 0 4" "goto 0.010 -0.236 0.76 0 4" "goto 0.010 -0.236 0.62 0 3" pose > run10.log 2>&1; grep -E "traj|TCP|IK|gripper" run10.log; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_d2.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih5.png

# openrua op 34
python3 -c "
import numpy as np, cv2
d=np.load('eih_d2.npy')
tab=np.median(d[50:150,300:400]); print('table depth', tab.round(4), 'cam z ->', (0.425+tab).round(4))
m=((d>0.05)&(d<tab-0.04)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    x,y,w,h,a=stats[i]
    if a>300: print(i,'bbox',x,y,w,h,'area',a,'centroid',cent[i].round(1),'mindepth',d[lab==i].min().round(4))
"

# openrua op 35
timeout 600 python3 -u robot.py "goto 0.0136 -0.2324 0.60 0 2" pose > run11.log 2>&1; grep -E "traj|TCP" run11.log; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_d3.png >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('eih_d3.npy')
tab=np.median(d[50:150,300:400])
m=((d>0.05)&(d<tab-0.04)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    x,y,w,h,a=stats[i]
    if a>300 and 200<cent[i][0]<440 and cent[i][1]<360: 
        Z=d[lab==i].min(); print('can bbox',x,y,w,h,'centroid',cent[i].round(1),'rim depth',Z.round(4),'pred TCP v', (240+0.05*312.77/Z).round(1))
"

# openrua op 36
timeout 900 python3 -u robot.py "goto 0.0136 -0.2324 0.48 0 3" pose "grip close" pose > run12.log 2>&1; grep -E "traj|TCP|gripper" run12.log; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 37
timeout 1500 python3 -u robot.py "goto 0.0136 -0.2324 0.76 0 4" "goto 0.0 0.0 0.76 0 4" "goto 0.006 0.277 0.76 0 4" pose > run13.log 2>&1; grep -E "traj|TCP|gripper" run13.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih6.png; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_d4.png > /dev/null

# openrua op 38
timeout 1200 python3 -u robot.py "goto -0.0185 0.2495 0.70 0 3" "goto -0.0185 0.2495 0.625 0 2" pose > run14.log 2>&1; grep -E "traj|TCP|gripper" run14.log; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih7.png

# openrua op 39
timeout 1200 python3 -u robot.py "grip open" "goto -0.0185 0.2495 0.78 0 3" pose > run15.log 2>&1; grep -E "traj|TCP|gripper" run15.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih8.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent8.png

# openrua op 40
timeout 900 python3 -u robot.py "goto -0.20 0.0 0.80 0 4" pose > run16.log 2>&1; grep -E "traj|TCP" run16.log; for c in agentview birdview frontview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c final_$c.png; done; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw final_bird_depth.png

# openrua op 41
python3 -c "
import numpy as np, cv2
d=np.load('final_bird_depth.npy')
# basket rim region rows 267-306, cols 362-403 ; objects inside: depth between floor(2.505) and rim(2.376)
b=d[262:312, 356:410]
inside=(b>2.30)&(b<2.50)
ys,xs=np.where(inside)
print('pixels above floor inside basket bbox:', inside.sum(), 'min depth', b.min().round(3), '-> top z', (0.425+2.575-b.min()).round(3))
# check original object spots are empty (table depth 2.575)
print('soup spot depth', d[224,280].round(3), 'sauce spot depth', d[289,265].round(3), '(table=2.575)')
# any object outside the basket but off the table plane near the basket? scan whole table for depth<2.55 excluding known items
im=cv2.imread('final_birdview.png'); cv2.imwrite('final_crop.png', cv2.resize(im[240:330,340:430],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"
