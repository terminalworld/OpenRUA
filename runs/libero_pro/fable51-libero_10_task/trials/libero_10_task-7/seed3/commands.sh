#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; find /workspace/tools -type f; ls /workspace/.ros

# openrua op 2
cat tools/action/ik_move.py tools/action/fjt_send.py tools/action/gripper_cmd.py

# openrua op 3
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 4
ros2 topic list && echo --- && ros2 action list && echo --- && ros2 service list | grep -v -E 'parameter|describe|get_type' && echo --- && ros2 node list

# openrua op 5
for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ros2 topic echo /joint_states --once

# openrua op 6
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12; echo ---; ros2 topic echo /tf_static --once 2>&1 | grep -E 'frame_id|child' | head -40

# openrua op 7
timeout 20 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -8; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -4; echo ---; ros2 topic echo /birdview/color/camera_info --once | grep -A12 '^k:'

# openrua op 8
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); f=579.4112549695428
print(d.shape, np.nanmin(d), np.nanmax(d))
# world coords
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
# table height: mode of Z in central region
tab=np.median(Z[200:400,200:450]); print('table z ~',tab)
mask=(Z>tab+0.01)&(Z<tab+0.5)
# exclude robot area roughly x<-0.3
mask &= (X>-0.35)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    print(f'blob {i}: px={stats[i,4]} u,v=({cent[i][0]:.0f},{cent[i][1]:.0f}) X=[{X[m].min():.3f},{X[m].max():.3f}] Y=[{Y[m].min():.3f},{Y[m].max():.3f}] Ztop={Z[m].max():.3f} Zmed={np.median(Z[m]):.3f}')
"


# openrua op 9
timeout 300 python3 tools/action/fjt_send.py 0,-0.785,0,-2.356,0,1.571,0.785 3 && ros2 topic echo /joint_states --once | grep -A7 '^position'

# openrua op 10
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
mask=(Z>0.435)&(Z<1.0)&(X>-0.35)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    print(f'blob {i}: px={stats[i,4]} u,v=({cent[i][0]:.0f},{cent[i][1]:.0f}) X=[{X[m].min():.3f},{X[m].max():.3f}] Y=[{Y[m].min():.3f},{Y[m].max():.3f}] Xc={X[m].mean():.3f} Yc={Y[m].mean():.3f} Ztop={Z[m].max():.3f} Zmed={np.median(Z[m]):.3f}')
"


# openrua op 11
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
def tab(x0,x1,y0,y1):
    m=(X>x0)&(X<x1)&(Y>y0)&(Y<y1); return np.median(Z[m])
print('table near ketchup', tab(-0.26,-0.22,-0.2,-0.1), 'near box', tab(-0.13,-0.09,0.08,0.14), 'near basket', tab(-0.15,-0.1,0.2,0.3))
# ketchup profile
m=(X>-0.22)&(X<-0.14)&(Y>-0.21)&(Y<-0.11)&(Z>0.44)
for z0 in [0.44,0.47,0.50,0.53,0.56]:
    mm=m&(Z>z0)
    if mm.sum(): print(f'ketchup Z>{z0}: X[{X[mm].min():.3f},{X[mm].max():.3f}] Y[{Y[mm].min():.3f},{Y[mm].max():.3f}] n={mm.sum()}')
m=(X>-0.19)&(X<-0.07)&(Y>0.0)&(Y<0.08)&(Z>0.435)
print('box: X[%.3f,%.3f] Y[%.3f,%.3f] Zmax %.3f Zmed %.3f' % (X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max(),np.median(Z[m])))
# basket interior floor
m=(X>-0.05)&(X<0.05)&(Y>0.22)&(Y<0.30)
print('basket center z median', np.median(Z[m]), 'min', Z[m].min())
"


# openrua op 12
mkdir -p "$(dirname /workspace/tcp_pose.py)"
cat > /workspace/tcp_pose.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Print hand and TCP pose in the WORLD frame plus finger positions.

Uses TF panda_link0->panda_hand (the arm chain is disconnected from world,
so world->base offset comes from machine facts observed via tf2_echo).
"""
import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

WORLD_T_BASE = np.array([-0.51, 0.0, 0.42])
TCP = 0.1034


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("tcp_pose")
    buf = Buffer()
    TransformListener(buf, node)
    js = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: js.__setitem__("m", m), 1)
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.2)
        if "m" in js and buf.can_transform("panda_link0", "panda_hand", Time()):
            break
    t = buf.lookup_transform("panda_link0", "panda_hand", Time())
    tr, q = t.transform.translation, t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    hand = np.array([tr.x, tr.y, tr.z]) + WORLD_T_BASE
    tcp = hand + TCP * R[:, 2]
    print(f"hand_world  {hand[0]:.4f} {hand[1]:.4f} {hand[2]:.4f}")
    print(f"tcp_world   {tcp[0]:.4f} {tcp[1]:.4f} {tcp[2]:.4f}")
    print(f"quat xyzw   {q.x:.3f} {q.y:.3f} {q.z:.3f} {q.w:.3f}")
    print(f"finger_axis_world {R[:, 1].round(3)}")
    m = js["m"]
    d = dict(zip(m.name, m.position))
    print("joints", [round(d[f"panda_joint{i}"], 3) for i in range(1, 8)])
    print("fingers", round(d["panda_finger_joint1"], 4), round(d["panda_finger_joint2"], 4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 13
timeout 60 python3 tcp_pose.py; python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
m=(X>-0.19)&(X<-0.07)&(Y>0.0)&(Y<0.08)&(Z>0.435)&(Z<0.6)
print('box: X[%.3f,%.3f] Y[%.3f,%.3f] center (%.3f,%.3f) Zmax %.3f n=%d' % (X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),X[m].mean(),Y[m].mean(),Z[m].max(),m.sum()))
m=(X>-0.22)&(X<-0.14)&(Y>-0.21)&(Y<-0.11)&(Z>0.44)
print('ketchup center (%.3f,%.3f)'%(X[m].mean(),Y[m].mean()))
"

# openrua op 14
timeout 300 python3 tools/action/ik_move.py 0.329 -0.155 0.23 0.7071 0.7071 0 0 3 --at tcp && timeout 60 python3 tcp_pose.py

# openrua op 15
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/tools/action/ik_move.py", "old_string": "    args = [a for a in sys.argv[1:] if not a.startswith(\\"--\\")]", "new_string": "    args = [a for a in sys.argv[1:]\\n            if not a.startswith(\\"--\\") and a not in (\\"tcp\\", \\"hand\\")]", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 16
timeout 300 python3 tools/action/ik_move.py 0.329 -0.155 0.23 0.7071 0.7071 0 0 3 --at tcp && timeout 60 python3 tcp_pose.py

# openrua op 17
timeout 300 python3 tools/action/ik_move.py 0.329 -0.155 0.23 0.7071 -0.7071 0 0 3 --at tcp && timeout 60 python3 tcp_pose.py

# openrua op 18
mkdir -p "$(dirname /workspace/ik_probe.py)"
cat > /workspace/ik_probe.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Probe IK feasibility for a list of TCP poses (base frame) without moving.

Usage: python3 ik_probe.py "x y z qx qy qz qw" ["x y z qx qy qz qw" ...]
Prints the joint solution (manifest order) or the error code per pose.
"""
import sys

import numpy as np
import rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

sys.path.insert(0, "/workspace/tools/action")
from ik_move import _quat_to_R  # noqa: E402

TCP = 0.1034
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


def main():
    rclpy.init()
    node = rclpy.create_node("ik_probe")
    js = {}
    node.create_subscription(JointState, "/joint_states",
                             lambda m: js.setdefault("m", m), 1)
    while "m" not in js:
        rclpy.spin_once(node, timeout_sec=0.2)
    cli = node.create_client(GetPositionIK, "/compute_ik")
    cli.wait_for_service(timeout_sec=10)
    seed = JointState()
    for n_, p_ in zip(js["m"].name, js["m"].position):
        if n_ in JOINTS:
            seed.name.append(n_)
            seed.position.append(p_)
    for spec in sys.argv[1:]:
        x, y, z, qx, qy, qz, qw = map(float, spec.split())
        R = _quat_to_R(qx, qy, qz, qw)
        hx, hy, hz = np.array([x, y, z]) - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = hx, hy, hz
        p.orientation.x, p.orientation.y = qx, qy
        p.orientation.z, p.orientation.w = qz, qw
        req.ik_request.robot_state.joint_state = seed
        req.ik_request.timeout.sec = 2
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=60)
        res = fut.result()
        if res is None:
            print(spec, "-> no answer")
            continue
        if res.error_code.val != 1:
            print(spec, f"-> FAIL {res.error_code.val}")
            continue
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        print(spec, "->", ",".join(f"{sol[j]:.4f}" for j in JOINTS))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 19
timeout 200 python3 ik_probe.py "0.329 -0.155 0.23 1 0 0 0" "0.329 -0.155 0.23 0.7071 0.7071 0 0" "0.329 -0.155 0.23 0.7071 -0.7071 0 0" "0.329 -0.155 0.23 0.9239 0.3827 0 0" "0.329 -0.155 0.23 0.9239 -0.3827 0 0" "0.329 -0.155 0.23 0.3827 0.9239 0 0" "0.51 0.264 0.30 1 0 0 0" "0.384 0.039 0.23 1 0 0 0"

# openrua op 20
python3 - <<'EOF'
import rclpy, numpy as np, sys
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
rclpy.init(); node=rclpy.create_node('t')
js={}
node.create_subscription(JointState,'/joint_states',lambda m: js.setdefault('m',m),1)
while 'm' not in js: rclpy.spin_once(node,timeout_sec=0.2)
J=[f'panda_joint{i}' for i in range(1,8)]
seed=JointState(); 
for n,p in zip(js['m'].name,js['m'].position):
    if n in J: seed.name.append(n); seed.position.append(p)
fk=node.create_client(GetPositionFK,'/compute_fk'); fk.wait_for_service(5)
r=GetPositionFK.Request(); r.fk_link_names=['panda_hand']; r.robot_state.joint_state=seed
f=fk.call_async(r); rclpy.spin_until_future_complete(node,f,timeout_sec=30); res=f.result()
print('FK err',res.error_code.val, res.pose_stamped[0].header.frame_id if res.pose_stamped else None)
if res.pose_stamped:
    pp=res.pose_stamped[0].pose; print('FK hand',pp.position, pp.orientation)
ik=node.create_client(GetPositionIK,'/compute_ik'); ik.wait_for_service(5)
def tryik(pose, frame='', with_seed=True, tmo=None, link=''):
    q=GetPositionIK.Request(); q.ik_request.group_name='panda_arm'
    q.ik_request.pose_stamped.header.frame_id=frame
    q.ik_request.pose_stamped.pose=pose
    if link: q.ik_request.ik_link_name=link
    if with_seed: q.ik_request.robot_state.joint_state=seed
    if tmo: q.ik_request.timeout.sec=tmo
    f=ik.call_async(q); rclpy.spin_until_future_complete(node,f,timeout_sec=60); r=f.result()
    if r is None: return 'none'
    if r.error_code.val!=1: return f'FAIL {r.error_code.val}'
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
    return [round(sol[j],3) for j in J]
if res.pose_stamped:
    print('IK current pose, seed:', tryik(pp))
    print('IK current pose, no seed:', tryik(pp, with_seed=False))
    print('IK current pose, link panda_hand:', tryik(pp, link='panda_hand'))
    print('IK current pose, link panda_link8:', tryik(pp, link='panda_link8'))
rclpy.shutdown()
EOF

# openrua op 21
python3 - <<'EOF'
import re
for f in ['tools/action/ik_move.py','ik_probe.py']:
    s=open(f).read()
    s=s.replace('req.ik_request.group_name = planning.get("group", "panda_arm")',
                'req.ik_request.group_name = planning.get("group", "panda_arm")\n    # machine fact: default tip is panda_link8; aim the HAND frame\n    req.ik_request.ik_link_name = m["frames"]["hand"]')
    s=s.replace('req.ik_request.group_name = "panda_arm"',
                'req.ik_request.group_name = "panda_arm"\n        req.ik_request.ik_link_name = "panda_hand"')
    open(f,'w').write(s)
EOF
grep -n ik_link_name tools/action/ik_move.py ik_probe.py
timeout 200 python3 ik_probe.py "-0.181 -0.155 0.65 0.7071 0.7071 0 0" "-0.181 -0.155 0.49 0.7071 0.7071 0 0" "-0.126 0.039 0.65 1 0 0 0" "-0.126 0.039 0.442 1 0 0 0" "0.0 0.264 0.72 1 0 0 0" "0.0 0.264 0.72 0.7071 0.7071 0 0"

# openrua op 22
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/tools/action/ik_move.py", "old_string": "    # machine fact: leave frame_id empty; poses are interpreted in the\\n    # planner\'s model frame (the arm base), see machine.yaml planning", "new_string": "    # machine fact: leave frame_id empty; poses are interpreted in the\\n    # planner\'s model frame. On THIS machine /compute_fk reports panda_hand\\n    # in WORLD coordinates (base sits at world (-0.51, 0, 0.42)), so pass\\n    # world-frame targets here.", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 23
timeout 300 python3 tools/action/ik_move.py -0.181 -0.155 0.65 0.7071 0.7071 0 0 3 --at tcp; timeout 60 python3 tcp_pose.py

# openrua op 24
timeout 300 python3 tools/action/fjt_send.py -0.5503,-0.4212,0.1178,-2.7795,0.0682,2.3597,-1.2766 4; timeout 60 python3 tcp_pose.py

# openrua op 25
timeout 300 python3 tools/action/fjt_send.py -0.5503,-0.4212,0.1178,-2.7795,0.0682,2.3597,-1.2766 6; timeout 60 python3 tcp_pose.py

# openrua op 26
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 27
timeout 100 python3 ik_probe.py "-0.181 -0.155 0.49 0.7071 0.7071 0 0" "-0.181 -0.155 0.50 0.7071 0.7071 0 0"

# openrua op 28
timeout 300 python3 tools/action/fjt_send.py -0.6006,0.1054,0.1421,-2.8269,-0.0712,2.9307,-1.1750 4; timeout 60 python3 tcp_pose.py

# openrua op 29
ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 -E 'force|torque'; timeout 300 python3 tools/action/fjt_send.py -0.6006,0.1054,0.1421,-2.8269,-0.0712,2.9307,-1.1750 6; timeout 60 python3 tcp_pose.py

# openrua op 30
ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 -E 'force'; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 31
timeout 300 python3 tools/action/fjt_send.py -0.5503,-0.4212,0.1178,-2.7795,0.0682,2.3597,-1.2766 4; timeout 60 python3 tcp_pose.py | head -2

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
mask=(Z>0.435)&(Z<0.62)&(X>-0.35)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    print(f'blob {i}: px={stats[i,4]} X=[{X[m].min():.3f},{X[m].max():.3f}] Y=[{Y[m].min():.3f},{Y[m].max():.3f}] Xc={X[m].mean():.3f} Yc={Y[m].mean():.3f} Ztop={Z[m].max():.3f} Zmed={np.median(Z[m]):.3f}')
"

# openrua op 33
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
m=(Z>0.435)&(Z<0.62)&(X>-0.17)&(X<0.0)&(Y>-0.2)&(Y<-0.12)
for x0 in np.arange(-0.16,-0.01,0.01):
    mm=m&(X>=x0)&(X<x0+0.01)
    if mm.sum(): print(f'X {x0:.2f}: Y[{Y[mm].min():.3f},{Y[mm].max():.3f}] Yc={Y[mm].mean():.3f} Ztop={Z[mm].max():.3f} n={mm.sum()}')
"

# openrua op 34
mkdir -p "$(dirname /workspace/goto.py)"
cat > /workspace/goto.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""IK to a WORLD-frame TCP pose, then send the trajectory until it converges.

Usage: python3 goto.py <x> <y> <z> <qx> <qy> <qz> <qw> [seconds=4] [--joints j1,...,j7]
Resends the same joint goal (up to 4 times) while any joint is more than
TOL rad from target, since this controller lags long goals. Prints the
final TCP pose in world (via /compute_fk) so the caller can verify.
"""
import sys
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, "/workspace/tools/action")
from ik_move import _quat_to_R  # noqa: E402

TCP = 0.1034
TOL = 0.02
JOINTS = [f"panda_joint{i}" for i in range(1, 8)]


class Robot:
    def __init__(self):
        self.node = rclpy.create_node("goto")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory,
                                "/panda_arm_controller/follow_joint_trajectory")
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.fjt.wait_for_server(10)

    def _on_js(self, m):
        self.js = dict(zip(m.name, m.position))

    def joints(self):
        self.js = {}
        while not self.js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return [self.js[j] for j in JOINTS]

    def seed(self):
        s = JointState()
        s.name = list(JOINTS)
        s.position = self.joints()
        return s

    def solve(self, x, y, z, qx, qy, qz, qw):
        R = _quat_to_R(qx, qy, qz, qw)
        hx, hy, hz = np.array([x, y, z]) - TCP * R[:, 2]
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = hx, hy, hz
        p.orientation.x, p.orientation.y = qx, qy
        p.orientation.z, p.orientation.w = qz, qw
        req.ik_request.robot_state.joint_state = self.seed()
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise SystemExit(f"IK FAILED ({None if res is None else res.error_code.val})")
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def send(self, target, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(v) for v in target])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        f = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, f)
        r = f.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, r)
        return r.result().result.error_code

    def move(self, target, seconds, tries=4):
        for i in range(tries):
            code = self.send(target, seconds)
            cur = self.joints()
            err = np.abs(np.array(cur) - np.array(target)).max()
            print(f"  try {i+1}: error_code={code} max_joint_err={err:.4f}",
                  flush=True)
            if err < TOL:
                return True
            seconds = max(seconds, 4) + 2
        return False

    def tcp(self):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self.seed()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        p = fut.result().pose_stamped[0].pose
        q = p.orientation
        R = _quat_to_R(q.x, q.y, q.z, q.w)
        hand = np.array([p.position.x, p.position.y, p.position.z])
        return hand + TCP * R[:, 2], (q.x, q.y, q.z, q.w), R[:, 1]


def main():
    argv = sys.argv[1:]
    joints = None
    if "--joints" in argv:
        i = argv.index("--joints")
        joints = [float(v) for v in argv[i + 1].split(",")]
        del argv[i:i + 2]
    rclpy.init()
    rb = Robot()
    if joints is None:
        x, y, z, qx, qy, qz, qw = map(float, argv[:7])
        seconds = float(argv[7]) if len(argv) > 7 else 4.0
        joints = rb.solve(x, y, z, qx, qy, qz, qw)
        print("IK:", ",".join(f"{v:.4f}" for v in joints))
    else:
        seconds = float(argv[0]) if argv else 4.0
    ok = rb.move(joints, seconds)
    tcp, q, fa = rb.tcp()
    print(f"converged={ok} tcp_world {tcp[0]:.4f} {tcp[1]:.4f} {tcp[2]:.4f} "
          f"quat {np.round(q,3)} finger_axis {np.round(fa,3)}")
    print("fingers", round(rb.js.get("panda_finger_joint1", 0), 4),
          round(rb.js.get("panda_finger_joint2", 0), 4))
    rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 35
timeout 600 python3 goto.py -0.115 -0.165 0.60 1 0 0 0 4

# openrua op 36
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 37
timeout 20 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A2 -E 'Translation|Quaternion' | head -4; ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A9 '^k:' | tr '\n' ' '; echo; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png

# openrua op 38
python3 -c "
import numpy as np
d=np.load('eih_depth.npy'); f=312.77408948188935
# hand pose: TCP at (-0.1155,-0.1650,0.5999), quat ~(1,0,0,0) -> hand z down; hand x -> world x? R=diag(1,-1,-1)
hand=np.array([-0.1155,-0.1650,0.5999+0.1034])
v,u=np.mgrid[0:480,0:640]
cx=(u-320)*d/f; cy=(v-240)*d/f; cz=d
# cam->hand: hand_x = 0.05 - cam_y ; hand_y = cam_x ; hand_z = cam_z
hx=0.05-cy; hy=cx; hz=cz
# hand->world with R=diag(1,-1,-1)
X=hand[0]+hx; Y=hand[1]-hy; Z=hand[2]-hz
print('table z median', np.median(Z[(Z<0.44)&(Z>0.40)]))
m=(Z>0.44)&(Z<0.60)&(Y>-0.22)&(Y<-0.11)&(X>-0.2)&(X<0.02)
print('bottle: X[%.3f,%.3f] Y[%.3f,%.3f] Ztop %.3f n=%d'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),Z[m].max(),m.sum()))
for x0 in np.arange(-0.16,0.0,0.01):
    mm=m&(X>=x0)&(X<x0+0.01)
    if mm.sum()>3: print(f'X {x0:.2f}: Y[{Y[mm].min():.3f},{Y[mm].max():.3f}] Yc={Y[mm].mean():.3f} Ztop={Z[mm].max():.3f}')
mm=m&(Z>0.46)
print('above 0.46: X[%.3f,%.3f] Y[%.3f,%.3f] Yc %.3f'%(X[mm].min(),X[mm].max(),Y[mm].min(),Y[mm].max(),Y[mm].mean()))
"

# openrua op 39
timeout 600 python3 goto.py -0.11 -0.160 0.50 1 0 0 0 4

# openrua op 40
timeout 600 python3 goto.py -0.11 -0.160 0.45 1 0 0 0 3; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 41
timeout 300 python3 tools/action/gripper_cmd.py 0.0; ros2 topic echo /joint_states --once | grep -A9 '^position' | tail -2; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 42
timeout 600 python3 goto.py -0.11 -0.160 0.70 1 0 0 0 4

# openrua op 43
timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 44
timeout 600 python3 goto.py -0.02 0.244 0.72 0.9239 0.3827 0 0 5

# openrua op 45
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 46
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 '^position' | tail -2; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 47
timeout 600 python3 goto.py -0.134 0.037 0.60 1 0 0 0 5 && timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 48
python3 -c "
import numpy as np
d=np.load('eih_depth.npy'); f=312.77408948188935
hand=np.array([-0.1339,0.0372,0.6002+0.1034])
v,u=np.mgrid[0:480,0:640]
cx=(u-320)*d/f; cy=(v-240)*d/f
X=hand[0]+0.05-cy; Y=hand[1]-cx; Z=hand[2]-d
m=(Z>0.44)&(Z<0.50)&(np.abs(X-hand[0])<0.12)&(np.abs(Y-hand[1])<0.10)
print('box: X[%.3f,%.3f] Y[%.3f,%.3f] center (%.4f,%.4f) Ztop %.3f n=%d'%(X[m].min(),X[m].max(),Y[m].min(),Y[m].max(),X[m].mean(),Y[m].mean(),Z[m].max(),m.sum()))
print('table', np.median(Z[(Z>0.40)&(Z<0.44)]))
"

# openrua op 49
timeout 600 python3 goto.py -0.13 0.039 0.50 1 0 0 0 4 && timeout 600 python3 goto.py -0.13 0.039 0.442 1 0 0 0 3; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 50
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 51
timeout 300 python3 tools/action/gripper_cmd.py 0.0; ros2 topic echo /joint_states --once | grep -A9 '^position' | tail -2

# openrua op 52
timeout 600 python3 goto.py -0.13 0.039 0.65 1 0 0 0 4 && timeout 600 python3 goto.py 0.0 0.264 0.70 1 0 0 0 5

# openrua op 53
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 54
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once | grep -A9 '^position' | tail -2

# openrua op 55
timeout 600 python3 goto.py -0.15 0.15 0.80 1 0 0 0 4 | tail -2; for c in agentview birdview robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c >/dev/null; done; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); f=579.4112549695428
v,u=np.mgrid[0:480,0:640]
X=(v-240)*d/f-0.2; Y=(u-320)*d/f; Z=3.0-d
mask=(Z>0.435)&(Z<0.75)&(X>-0.35)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    print(f'blob {i}: px={stats[i,4]} X=[{X[m].min():.3f},{X[m].max():.3f}] Y=[{Y[m].min():.3f},{Y[m].max():.3f}] Ztop={Z[m].max():.3f}')
# anything left on the table outside the basket footprint and cans?
m=(Z>0.435)&(Z<0.75)&(X>-0.35)&~((X>-0.09)&(X<0.08)&(Y>0.16)&(Y<0.35))&(Y<0.9)&(Y>-0.85)&(X<0.34)
print('non-basket raised pixels:', m.sum())
"

# openrua op 56
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Machine-specific facts for the /workspace Franka Panda sim that contradict or extend the starter docs (IK frame, tip link, controller lag, camera geometry)
metadata:
  type: project
---

On the /workspace Panda workstation (machine.yaml, MoveIt, paused-clock sim), learned 2026-09-12:

- `/compute_ik` and `/compute_fk` work in the **WORLD** frame, not the arm base as docs/30-action.md claims. world→panda_link0 = (-0.51, 0, 0.42); table top is world z≈0.425.
- IK's default tip link is `panda_link8`; set `ik_request.ik_link_name = "panda_hand"` or orientations are 45° off and solutions differ.
- Hand quaternion (1,0,0,0) points down with fingers along world Y; (0.7071,0.7071,0,0) puts fingers along X; (0.9239,0.3827,0,0) is 45°.
- The trajectory controller lags long goals (error_code -5, joint 7 especially). Resending the same goal with a longer duration converges; `/workspace/goto.py` (IK on TCP + resend-until-converged + FK readback) was written for this. Never descend on a goal that hasn't converged — an off-target descent knocked over the ketchup bottle.
- Eye-in-hand camera: optical frame is +0.05 m along hand X, rotated 90° about hand Z, f=312.77, 640x480. Birdview is at world (-0.2,0,3.0) looking down, f=579.41; image-down = +X, image-right = +Y.
- Gripper is open/closed only; finger readings after closing give width evidence (0.0285 → ~6 cm bottle, 0.021 → 4 cm box).

**Why:** the generic docs are wrong about the IK frame on this machine and silent about the tip link; each cost several failed IK calls.
**How to apply:** feed world-frame TCP targets to goto.py/ik_move.py, verify convergence before any descent, measure objects with wrist depth once above them.
OPENRUA_EOF

# openrua op 57
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index yet)")

# openrua op 58
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda workstation quirks](panda-workstation-quirks.md) — IK/FK are world-frame, set ik_link_name=panda_hand, controller lags (resend), camera geometry, goto.py helper
OPENRUA_EOF
