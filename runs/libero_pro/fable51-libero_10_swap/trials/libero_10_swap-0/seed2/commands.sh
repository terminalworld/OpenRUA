#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
ros2 topic list 2>&1; echo ----; ros2 action list 2>&1; echo ----; ros2 service list 2>&1 | head -40; echo ----; ros2 node list 2>&1

# openrua op 4
ros2 topic echo /joint_states --once 2>&1 | head -40; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('agentview.png')
crop=im[180:420,90:300]
cv2.imwrite('crop_cans.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im2=cv2.imread('robot0_robotview.png')
crop2=im2[0:280,20:280]
cv2.imwrite('crop_cans2.png', cv2.resize(crop2,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id|x:|y:|z:|w:" | head -80

# openrua op 7
cat > tfdump.py <<'EOF'
import rclpy, yaml
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
rclpy.init(); node = rclpy.create_node("tfdump")
got = {}
qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
def cb(m, tag):
    for t in m.transforms:
        got[(tag, t.header.frame_id, t.child_frame_id)] = (t.transform.translation, t.transform.rotation)
node.create_subscription(TFMessage, "/tf_static", lambda m: cb(m,"static"), qos)
node.create_subscription(TFMessage, "/tf", lambda m: cb(m,"dyn"), 10)
import time
for _ in range(40): rclpy.spin_once(node, timeout_sec=0.1)
for k,(tr,q) in sorted(got.items()):
    print(k, f"t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
EOF
python3 tfdump.py

# openrua op 8
python3 -c "
import cv2
im=cv2.imread('birdview.png')
crop=im[200:330,240:420]
cv2.imwrite('crop_bird.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 9
for p in "281 232" "280 260" "267 289" "279 306" "300 296" "329 296" "386 290" "386 250" "386 330" "345 290" "425 290" "200 200"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 10
for p in "366 290" "406 290" "386 272" "386 308" "375 280" "270 289" "264 289" "281 226" "287 232"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -8

# openrua op 11
timeout 300 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once 2>&1 | grep -A9 position | tail -3

# openrua op 12
timeout 600 python3 tools/action/ik_move.py 0.275 -0.169 0.20 1 0 0 0 4 --at tcp; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3

# openrua op 13
timeout 600 python3 tools/action/ik_move.py 0.275 -0.169 0.3034 1 0 0 0 4; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3

# openrua op 14
for z in 0.32 0.30 0.34; do for q in "1 0 0 0" "0.9239 0.3827 0 0" "0.9239 -0.3827 0 0"; do echo "z=$z q=$q"; timeout 600 python3 tools/action/ik_move.py 0.275 -0.169 $z $q 4 && break 2; done; done; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3

# openrua op 15
cat > kin.py <<'EOF'
import sys, rclpy
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
ARM=[f"panda_joint{i}" for i in range(1,8)]
rclpy.init(); node=rclpy.create_node("kin")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.setdefault("m",m),1)
while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
seed=JointState()
for n,p in zip(js["m"].name, js["m"].position):
    if n in ARM: seed.name.append(n); seed.position.append(p)
print("seed", dict(zip(seed.name,[round(p,3) for p in seed.position])))
fk=node.create_client(GetPositionFK,"/compute_fk"); fk.wait_for_service()
r=GetPositionFK.Request(); r.header.frame_id=""; r.fk_link_names=["panda_hand"]; r.robot_state.joint_state=seed
f=fk.call_async(r); rclpy.spin_until_future_complete(node,f,timeout_sec=30)
res=f.result(); print("FK err", res.error_code.val)
for ps in res.pose_stamped:
    p=ps.pose.position; o=ps.pose.orientation
    print("FK frame", ps.header.frame_id, "pos", round(p.x,4),round(p.y,4),round(p.z,4), "q", round(o.x,4),round(o.y,4),round(o.z,4),round(o.w,4))
ik=node.create_client(GetPositionIK,"/compute_ik"); ik.wait_for_service()
def try_ik(x,y,z,qx,qy,qz,qw,frame=""):
    q=GetPositionIK.Request(); q.ik_request.group_name="panda_arm"
    q.ik_request.pose_stamped.header.frame_id=frame
    pp=q.ik_request.pose_stamped.pose
    pp.position.x,pp.position.y,pp.position.z=x,y,z
    pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=qx,qy,qz,qw
    q.ik_request.robot_state.joint_state=seed
    q.ik_request.timeout.sec=1
    f=ik.call_async(q); rclpy.spin_until_future_complete(node,f,timeout_sec=60)
    r=f.result()
    if r is None: print("  no answer"); return None
    if r.error_code.val!=1: print("  IK err", r.error_code.val); return None
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
    print("  sol", [round(sol[j],4) for j in ARM]); return [sol[j] for j in ARM]
for a in sys.argv[1:]:
    v=[float(t) for t in a.split(",")]
    print("IK", v); try_ik(*v)
EOF
python3 kin.py 0.457,0,0.358,1,0,0,0 -0.053,0,0.778,1,0,0,0 0.457,0,0.358,1,0,-0.028,0

# openrua op 16
timeout 600 python3 tools/action/ik_move.py -0.235 -0.169 0.70 1 0 0 0 4; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3; ros2 topic echo /joint_states --once 2>&1 | grep -A9 "^position" 

# openrua op 17
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png

# openrua op 18
timeout 600 python3 tools/action/ik_move.py -0.235 -0.169 0.566 1 0 0 0 3; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3

# openrua op 19
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent2.png

# openrua op 20
timeout 300 python3 tools/action/gripper_cmd.py 0.0; ros2 topic echo /joint_states --once 2>&1 | grep -A9 "^position" | tail -2

# openrua op 21
timeout 600 python3 tools/action/ik_move.py -0.235 -0.169 0.75 1 0 0 0 3; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3; ros2 topic echo /joint_states --once 2>&1 | grep -A9 "^position" | tail -2; timeout 60 python3 tools/perception/cam_snap.py agentview agent3.png

# openrua op 22
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 600 python3 tools/action/ik_move.py -0.235 -0.169 0.70 1 0 0 0 3; timeout 600 python3 tools/action/ik_move.py -0.235 -0.169 0.552 1 0 0 0 3; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3; ros2 topic echo /joint_states --once 2>&1 | grep -A9 "^position" | tail -2; timeout 60 python3 tools/perception/cam_snap.py agentview agent4.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih4.png

# openrua op 23
timeout 60 python3 tools/perception/cam_snap.py sideview side4.png; python3 -c "
import cv2
im=cv2.imread('side4.png'); crop=im[280:480,200:480]
cv2.imwrite('side4c.png', cv2.resize(crop,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agent4.png'); crop=im[130:300,120:320]
cv2.imwrite('agent4c.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 24
timeout 600 python3 tools/action/ik_move.py -0.235 -0.169 0.552 1 0 0 0 3; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3

# openrua op 25
ros2 topic echo /joint_states --once 2>&1 | grep -A9 "^position"; python3 kin.py -0.235,-0.169,0.552,1,0,0,0 2>&1 | tail -3; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A7 wrench

# openrua op 26
timeout 600 python3 tools/action/ik_move.py -0.235 -0.169 0.70 0.9239 -0.3827 0 0 3; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3

# openrua op 27
timeout 600 python3 tools/action/ik_move.py -0.235 -0.169 0.552 0.9239 -0.3827 0 0 3; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force

# openrua op 28
timeout 600 python3 tools/action/ik_move.py -0.235 -0.169 0.70 0.3827 0.9239 0 0 4; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3; ros2 topic echo /joint_states --once 2>&1 | grep -A7 "^position"

# openrua op 29
python3 kin.py -0.235,-0.169,0.70,0.3827,0.9239,0,0 2>&1 | tail -2; timeout 600 python3 tools/action/ik_move.py -0.235 -0.169 0.70 0.3827 0.9239 0 0 4; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3; ros2 topic echo /joint_states --once 2>&1 | grep -A7 "^position"

# openrua op 30
timeout 600 python3 tools/action/fjt_send.py -1.5193,-0.7961,1.1481,-2.8763,0.9498,2.212,2.7107 4; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3; ros2 topic echo /joint_states --once 2>&1 | grep -A7 "^position" | tail -1

# openrua op 31
timeout 600 python3 tools/action/fjt_send.py -1.5193,-0.7961,1.1481,-2.8763,0.9498,2.212,2.7107 4; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 "^At time" | head -3; ros2 topic echo /joint_states --once 2>&1 | grep -A7 "^position" | tail -1

# openrua op 32
cat > goto.py <<'EOF'
#!/usr/bin/env python3
"""goto.py x y z qx qy qz qw [secs] [--joints j1,...,j7]  (world frame, link8 orientation)
IK, then FollowJointTrajectory resent until joints converge; prints FK of panda_hand."""
import sys, rclpy
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
ARM=[f"panda_joint{i}" for i in range(1,8)]
args=[a for a in sys.argv[1:] if not a.startswith("--")]
joints_arg = sys.argv[sys.argv.index("--joints")+1] if "--joints" in sys.argv else None
rclpy.init(); node=rclpy.create_node("goto")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.__setitem__("m",m),1)
def cur():
    js.pop("m",None)
    while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
    d=dict(zip(js["m"].name,js["m"].position)); return [d[j] for j in ARM], d
def seed_state():
    s=JointState(); p,_=cur(); s.name=list(ARM); s.position=p; return s
fk=node.create_client(GetPositionFK,"/compute_fk"); fk.wait_for_service()
def do_fk():
    r=GetPositionFK.Request(); r.fk_link_names=["panda_hand"]; r.robot_state.joint_state=seed_state()
    f=fk.call_async(r); rclpy.spin_until_future_complete(node,f,timeout_sec=30)
    ps=f.result().pose_stamped[0]; p=ps.pose.position; o=ps.pose.orientation
    return (round(p.x,4),round(p.y,4),round(p.z,4)),(round(o.x,3),round(o.y,3),round(o.z,3),round(o.w,3))
if joints_arg:
    target=[float(t) for t in joints_arg.split(",")]; secs=float(args[0]) if args else 4.0
else:
    x,y,z,qx,qy,qz,qw=map(float,args[:7]); secs=float(args[7]) if len(args)>7 else 4.0
    ik=node.create_client(GetPositionIK,"/compute_ik"); ik.wait_for_service()
    q=GetPositionIK.Request(); q.ik_request.group_name="panda_arm"
    pp=q.ik_request.pose_stamped.pose
    pp.position.x,pp.position.y,pp.position.z=x,y,z
    pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=qx,qy,qz,qw
    q.ik_request.robot_state.joint_state=seed_state(); q.ik_request.timeout.sec=1
    f=ik.call_async(q); rclpy.spin_until_future_complete(node,f,timeout_sec=60)
    r=f.result()
    if r is None or r.error_code.val!=1:
        print("IK FAILED", None if r is None else r.error_code.val); sys.exit(2)
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position)); target=[sol[j] for j in ARM]
print("target joints", [round(t,4) for t in target])
client=ActionClient(node,FollowJointTrajectory,"/panda_arm_controller/follow_joint_trajectory"); client.wait_for_server()
for attempt in range(5):
    g=FollowJointTrajectory.Goal(); g.trajectory.joint_names=list(ARM)
    pt=JointTrajectoryPoint(positions=target); pt.time_from_start=Duration(sec=int(secs),nanosec=int((secs%1)*1e9))
    g.trajectory.points=[pt]
    s=client.send_goal_async(g); rclpy.spin_until_future_complete(node,s)
    rf=s.result().get_result_async(); rclpy.spin_until_future_complete(node,rf)
    code=rf.result().result.error_code
    p,_=cur(); err=max(abs(a-b) for a,b in zip(p,target))
    print(f"attempt {attempt} code={code} max_joint_err={err:.4f}")
    if err<0.01: break
pos,ori=do_fk(); print("hand", pos, ori)
_,d=cur(); print("fingers", round(d["panda_finger_joint1"],4), round(d["panda_finger_joint2"],4))
EOF
timeout 900 python3 goto.py -0.235 -0.169 0.552 0.3827 0.9239 0 0 3; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force

# openrua op 33
timeout 60 python3 tools/perception/cam_snap.py agentview agent5.png; timeout 60 python3 tools/perception/cam_snap.py sideview side5.png; python3 -c "
import cv2
im=cv2.imread('agent5.png'); crop=im[100:330,80:340]; cv2.imwrite('agent5c.png', cv2.resize(crop,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('side5.png'); crop=im[250:480,180:500]; cv2.imwrite('side5c.png', cv2.resize(crop,None,fx=2.2,fy=2.2,interpolation=cv2.INTER_CUBIC))"

# openrua op 34
python3 - <<'EOF'
import re
s=open('goto.py').read()
old=s[s.index("    q=GetPositionIK.Request()"):s.index("print(\"target joints\"")]
new='''    import random
    best=None; p0,_=cur()
    for k in range(12):
        q=GetPositionIK.Request(); q.ik_request.group_name="panda_arm"
        pp=q.ik_request.pose_stamped.pose
        pp.position.x,pp.position.y,pp.position.z=x,y,z
        pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=qx,qy,qz,qw
        s=JointState(); s.name=list(ARM)
        s.position=[v+(random.uniform(-0.15,0.15) if k else 0.0) for v in p0]
        q.ik_request.robot_state.joint_state=s; q.ik_request.timeout.sec=1
        f=ik.call_async(q); rclpy.spin_until_future_complete(node,f,timeout_sec=60)
        r=f.result()
        if r is None or r.error_code.val!=1: continue
        sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position)); t=[sol[j] for j in ARM]
        d=sum(abs(a-b) for a,b in zip(t,p0))
        if best is None or d<best[0]: best=(d,t)
        if d<0.5: break
    if best is None: print("IK FAILED"); sys.exit(2)
    print(f"IK joint-distance {best[0]:.3f}"); target=best[1]
'''
s=s.replace(old,new); open('goto.py','w').write(s)
EOF
timeout 900 python3 goto.py -0.235 -0.169 0.70 0.3827 0.9239 0 0 3 2>&1 | grep -v XMLPARSER

# openrua op 35
timeout 900 python3 goto.py -0.235 -0.169 0.552 0.3827 0.9239 0 0 3 2>&1 | grep -v XMLPARSER; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force

# openrua op 36
timeout 900 python3 goto.py -0.235 -0.169 0.70 0.3827 0.9239 0 0 3 2>&1 | grep -v XMLPARSER
cat > pose.py <<'EOF'
#!/usr/bin/env python3
"""pose.py tcp_x tcp_y tcp_z tilt_deg [yaw180=1] -> prints 'x y z qx qy qz qw' for panda_link8 (world).
Hand points down, fingers open along world Y; tilt>0 leans the approach toward -x (base side)."""
import sys, numpy as np
from scipy.spatial.transform import Rotation as R
tx,ty,tz,tilt=map(float,sys.argv[1:5]); yaw180=int(sys.argv[5]) if len(sys.argv)>5 else 1
q0 = R.from_quat([0,1,0,0]) if yaw180 else R.from_quat([1,0,0,0])     # hand Z down
hand = R.from_rotvec(np.deg2rad(tilt)*np.array([0,1,0])) * q0
zh = hand.apply([0,0,1])                                             # hand approach axis in world
flange = np.array([tx,ty,tz]) - 0.1034*zh
l8 = hand * R.from_quat([0,0,0.3827,0.9239])                         # hand = l8 * (0,0,-0.3827,0.9239)
q = l8.as_quat()
print(" ".join(f"{v:.4f}" for v in [*flange,*q]))
EOF
python3 pose.py -0.235 -0.169 0.46 30

# openrua op 37
P=$(python3 pose.py -0.235 -0.169 0.58 30); echo $P; timeout 900 python3 goto.py $P 3 2>&1 | grep -v XMLPARSER

# openrua op 38
P=$(python3 pose.py -0.235 -0.169 0.462 30); echo $P; timeout 900 python3 goto.py $P 3 2>&1 | grep -v XMLPARSER; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force

# openrua op 39
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih6.png; timeout 60 python3 tools/perception/cam_snap.py sideview side6.png; python3 -c "
import cv2
im=cv2.imread('side6.png'); crop=im[250:480,180:500]; cv2.imwrite('side6c.png', cv2.resize(crop,None,fx=2.2,fy=2.2,interpolation=cv2.INTER_CUBIC))"

# openrua op 40
timeout 60 python3 tools/perception/cam_snap.py birdview bird6.png; python3 -c "
import cv2
im=cv2.imread('bird6.png'); crop=im[200:330,240:420]; cv2.imwrite('bird6c.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 41
for p in "281 225" "281 222" "281 228" "278 225" "284 225"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 42
P=$(python3 pose.py -0.235 -0.169 0.58 30); timeout 900 python3 goto.py $P 3 2>&1 | grep -v XMLPARSER | tail -2; P=$(python3 pose.py -0.2645 -0.168 0.58 30); timeout 900 python3 goto.py $P 3 2>&1 | grep -v XMLPARSER | tail -2; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih7.png

# openrua op 43
P=$(python3 pose.py -0.2645 -0.168 0.468 30); timeout 900 python3 goto.py $P 3 2>&1 | grep -v XMLPARSER | tail -3; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force; timeout 60 python3 tools/perception/cam_snap.py sideview side7.png; timeout 60 python3 tools/perception/cam_snap.py birdview bird7.png; python3 -c "
import cv2
im=cv2.imread('side7.png'); crop=im[250:480,180:500]; cv2.imwrite('side7c.png', cv2.resize(crop,None,fx=2.2,fy=2.2,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('bird7.png'); crop=im[200:330,240:420]; cv2.imwrite('bird7c.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 44
timeout 60 python3 tools/perception/cam_snap.py agentview agent7.png; python3 -c "
import cv2
im=cv2.imread('agent7.png'); crop=im[100:330,60:340]; cv2.imwrite('agent7c.png', cv2.resize(crop,None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"; timeout 20 ros2 run tf2_ros tf2_echo world panda_leftfinger 2>&1 | grep -A1 "^At time" | head -2; timeout 20 ros2 run tf2_ros tf2_echo world panda_rightfinger 2>&1 | grep -A1 "^At time" | head -2

# openrua op 45
cat > goto2.py <<'EOF'
#!/usr/bin/env python3
"""goto2.py x y z qx qy qz qw [secs] [--steps N]  (world frame, panda_link8 pose)
Straight Cartesian path from the current link8 pose: N waypoints, IK per waypoint seeded
from the previous solution (nearest branch), sent as ONE trajectory; resent until converged."""
import sys, random, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation as R
ARM=[f"panda_joint{i}" for i in range(1,8)]
args=[a for a in sys.argv[1:] if not a.startswith("--")]
steps=int(sys.argv[sys.argv.index("--steps")+1]) if "--steps" in sys.argv else 1
x,y,z,qx,qy,qz,qw=map(float,args[:7]); secs=float(args[7]) if len(args)>7 else 4.0
rclpy.init(); node=rclpy.create_node("goto2")
js={}
node.create_subscription(JointState,"/joint_states",lambda m: js.__setitem__("m",m),1)
def cur():
    js.pop("m",None)
    while "m" not in js: rclpy.spin_once(node,timeout_sec=0.2)
    d=dict(zip(js["m"].name,js["m"].position)); return [d[j] for j in ARM], d
fk=node.create_client(GetPositionFK,"/compute_fk"); fk.wait_for_service()
ik=node.create_client(GetPositionIK,"/compute_ik"); ik.wait_for_service()
def do_fk(joints, link):
    r=GetPositionFK.Request(); r.fk_link_names=[link]; s=JointState(); s.name=list(ARM); s.position=list(joints)
    r.robot_state.joint_state=s
    f=fk.call_async(r); rclpy.spin_until_future_complete(node,f,timeout_sec=30)
    ps=f.result().pose_stamped[0]; p=ps.pose.position; o=ps.pose.orientation
    return np.array([p.x,p.y,p.z]), np.array([o.x,o.y,o.z,o.w])
def do_ik(pos, quat, seed, tries=12):
    best=None
    for k in range(tries):
        q=GetPositionIK.Request(); q.ik_request.group_name="panda_arm"
        pp=q.ik_request.pose_stamped.pose
        pp.position.x,pp.position.y,pp.position.z=map(float,pos)
        pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=map(float,quat)
        s=JointState(); s.name=list(ARM)
        s.position=[v+(random.uniform(-0.1,0.1) if k else 0.0) for v in seed]
        q.ik_request.robot_state.joint_state=s; q.ik_request.timeout.sec=1
        f=ik.call_async(q); rclpy.spin_until_future_complete(node,f,timeout_sec=60)
        r=f.result()
        if r is None or r.error_code.val!=1: continue
        sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position)); t=[sol[j] for j in ARM]
        d=sum(abs(a-b) for a,b in zip(t,seed))
        if best is None or d<best[0]: best=(d,t)
        if d<0.3: break
    return best
p0,_=cur()
start_pos,start_q=do_fk(p0,"panda_link8")
target_pos=np.array([x,y,z]); target_q=np.array([qx,qy,qz,qw])
if np.dot(start_q,target_q)<0: start_q=-start_q
key=R.from_quat([start_q,target_q])
from scipy.spatial.transform import Slerp
slerp=Slerp([0,1],key)
seed=p0; wps=[]
for i in range(1,steps+1):
    a=i/steps; pos=start_pos*(1-a)+target_pos*a; quat=slerp([a]).as_quat()[0]
    best=do_ik(pos,quat,seed)
    if best is None: print(f"IK FAILED at step {i}"); sys.exit(2)
    print(f"step {i}: dist {best[0]:.3f} joints {[round(v,3) for v in best[1]]}")
    seed=best[1]; wps.append(best[1])
target=wps[-1]
client=ActionClient(node,FollowJointTrajectory,"/panda_arm_controller/follow_joint_trajectory"); client.wait_for_server()
for attempt in range(5):
    g=FollowJointTrajectory.Goal(); g.trajectory.joint_names=list(ARM)
    pts=[]
    if attempt==0:
        for i,w in enumerate(wps,1):
            t=secs*i/steps; pt=JointTrajectoryPoint(positions=w); pt.time_from_start=Duration(sec=int(t),nanosec=int((t%1)*1e9)); pts.append(pt)
    else:
        pt=JointTrajectoryPoint(positions=target); t=max(1.5,secs/2); pt.time_from_start=Duration(sec=int(t),nanosec=int((t%1)*1e9)); pts=[pt]
    g.trajectory.points=pts
    s=client.send_goal_async(g); rclpy.spin_until_future_complete(node,s)
    rf=s.result().get_result_async(); rclpy.spin_until_future_complete(node,rf)
    code=rf.result().result.error_code
    p,_=cur(); err=max(abs(a-b) for a,b in zip(p,target))
    print(f"attempt {attempt} code={code} max_joint_err={err:.4f}")
    if err<0.01: break
p,d=cur(); pos,ori=do_fk(p,"panda_hand")
print("hand", np.round(pos,4).tolist(), np.round(ori,3).tolist())
print("fingers", round(d["panda_finger_joint1"],4), round(d["panda_finger_joint2"],4))
EOF
P=$(python3 pose.py -0.2645 -0.168 0.60 30); timeout 900 python3 goto2.py $P 4 --steps 6 2>&1 | grep -v XMLPARSER

# openrua op 46
timeout 60 python3 tools/perception/cam_snap.py birdview bird8.png; python3 -c "
import cv2
im=cv2.imread('bird8.png'); crop=im[200:330,240:420]; cv2.imwrite('bird8c.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 47
for p in "281 220" "278 222" "284 222" "281 224"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 48
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird_depth.png >/dev/null; python3 - <<'EOF'
import numpy as np
d=np.load('bird_depth.npy')
# birdview: camera at z=3.0 looking straight down; world z = 3.0 - depth (approx, optical axis vertical)
# intrinsics: from px2world scale: 0.2/45 m per px approx; compute from camera_info instead
import subprocess, re
info=subprocess.run("ros2 topic echo /birdview/color/camera_info --once",shell=True,capture_output=True,text=True).stdout
k=re.search(r"k:\n((?:- .*\n){9})",info).group(1); k=[float(v[2:]) for v in k.strip().split("\n")]
fx,fy,cx,cy=k[0],k[4],k[2],k[5]
def world(u,v):
    z=d[v,u]; X=(u-cx)*z/fx; Y=(v-cy)*z/fy
    # optical frame q=(0.7071,0.7071,0,0): rotation 180deg about (1,1,0)/sqrt2 -> maps (X,Y,Z)->(Y,X,-Z)
    return np.array([-0.2+Y, 0.0+X, 3.0-z])
# can region: pixels near (281,224) with world z>0.49
ys,xs=np.mgrid[205:245,265:300]
mask=(3.0-d[205:245,265:300])>0.49
pts=np.array([world(u,v) for v,u in zip(ys[mask],xs[mask])])
print("n",len(pts),"x range",pts[:,0].min(),pts[:,0].max(),"y range",pts[:,1].min(),pts[:,1].max(),"z",pts[:,2].min(),pts[:,2].max())
print("centroid",pts.mean(axis=0))
# check world() against px2world value at (281,224): expected -0.2688 -0.1677 0.5086
print(world(281,224))
EOF

# openrua op 49
python3 - <<'EOF'
import numpy as np, subprocess, re
d=np.load('bird_depth.npy')
info=subprocess.run("ros2 topic echo /birdview/color/camera_info --once",shell=True,capture_output=True,text=True).stdout
k=re.search(r"k:\n((?:- .*\n){9})",info).group(1); k=[float(v[2:]) for v in k.strip().split("\n")]
fx,fy,cx,cy=k[0],k[4],k[2],k[5]
def world(u,v):
    z=d[v,u]; X=(u-cx)*z/fx; Y=(v-cy)*z/fy
    return np.array([-0.2+Y, 0.0+X, 3.0-z])
ys,xs=np.mgrid[205:245,265:300]
wz=3.0-d[205:245,265:300]
mask=(wz>0.49)&(wz<0.52)
pts=np.array([world(u,v) for v,u in zip(ys[mask],xs[mask])])
print("n",len(pts),"x range",pts[:,0].min(),pts[:,0].max(),"y range",pts[:,1].min(),pts[:,1].max(),"z",pts[:,2].min(),pts[:,2].max())
print("centroid",pts.mean(axis=0))
for v in range(214,234):
    print(v, "".join("#" if (wz[v-205,u-265]>0.49 and wz[v-205,u-265]<0.52) else ("A" if wz[v-205,u-265]>=0.52 else ".") for u in range(265,300)))
EOF

# openrua op 50
P=$(python3 pose.py -0.263 -0.165 0.47 30); timeout 900 python3 goto2.py $P 4 --steps 6 2>&1 | grep -v XMLPARSER; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force

# openrua op 51
P=$(python3 pose.py -0.263 -0.161 0.60 30); timeout 900 python3 goto2.py $P 4 --steps 6 2>&1 | grep -v XMLPARSER | tail -3

# openrua op 52
P=$(python3 pose.py -0.263 -0.161 0.47 30); timeout 900 python3 goto2.py $P 5 --steps 8 2>&1 | grep -v XMLPARSER | grep -v "^step"; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force

# openrua op 53
timeout 60 python3 tools/perception/cam_snap.py agentview agent9.png; timeout 60 python3 tools/perception/cam_snap.py frontview front9.png; python3 -c "
import cv2
im=cv2.imread('agent9.png'); crop=im[80:330,40:340]; cv2.imwrite('agent9c.png', cv2.resize(crop,None,fx=2.4,fy=2.4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('front9.png'); crop=im[150:300,180:380]; cv2.imwrite('front9c.png', cv2.resize(crop,None,fx=3.5,fy=3.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 54
timeout 60 python3 tools/perception/cam_snap.py sideview side9.png; python3 -c "
import cv2
im=cv2.imread('side9.png'); crop=im[300:440,300:480]; cv2.imwrite('side9c.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 55
python3 - <<'EOF' 2>&1 | grep -v XMLPARSER
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node("urdf")
got=[]
n.create_subscription(String,"/robot_description",lambda m: got.append(m.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
for _ in range(50):
    rclpy.spin_once(n,timeout_sec=0.2)
    if got: break
open("robot.urdf","w").write(got[0]); print(len(got[0]))
EOF
grep -n -A25 'link name="panda_leftfinger"' robot.urdf | head -60; grep -n -B2 -A12 'link name="panda_hand"' robot.urdf | head -60

# openrua op 56
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih9.png

# openrua op 57
P=$(python3 pose.py -0.263 -0.161 0.62 30); timeout 900 python3 goto2.py $P 4 --steps 6 2>&1 | grep -v XMLPARSER | tail -2
timeout 900 python3 goto2.py -0.10 -0.42 0.65 0.3827 0.9239 0 0 4 --steps 4 2>&1 | grep -v XMLPARSER | tail -2

# openrua op 58
for z in 0.56 0.545 0.535 0.525 0.515; do echo "== hand z $z"; timeout 900 python3 goto2.py -0.10 -0.42 $z 0.3827 0.9239 0 0 2 --steps 2 2>&1 | grep -v XMLPARSER | grep -E "attempt 0|hand"; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force | tail -1; done

# openrua op 59
timeout 60 python3 tools/perception/cam_snap.py agentview agent10.png; timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agent10_depth.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('agent10.png'); print(im.shape)
crop=im[150:400,200:520]; cv2.imwrite('agent10c.png', cv2.resize(crop,None,fx=2.2,fy=2.2,interpolation=cv2.INTER_CUBIC))"

# openrua op 60
python3 -c "
import cv2
im=cv2.imread('agent10.png'); crop=im[200:290,0:120]; cv2.imwrite('agent10c.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 61
for p in "8 282" "68 282" "66 282" "70 282" "10 282" "6 282" "68 275" "68 270"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py agentview $p; done

# openrua op 62
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih10.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih10_depth.png >/dev/null; python3 - <<'EOF'
import numpy as np, subprocess, re, cv2
d=np.load('eih10_depth.npy')
info=subprocess.run("ros2 topic echo /robot0_eye_in_hand/color/camera_info --once",shell=True,capture_output=True,text=True).stdout
k=re.search(r"k:\n((?:- .*\n){9})",info).group(1); k=[float(v[2:]) for v in k.strip().split("\n")]
fx,fy,cx,cy=k[0],k[4],k[2],k[5]; print("fx",fx,"cx",cx,"cy",cy, d.shape)
# print depth profile along row 385 and 395
for row in (370,380,390,400):
    print(row, " ".join(f"{d[row,u]:.3f}" for u in range(100,540,20)))
im=cv2.imread('eih10.png'); cv2.imwrite('eih10c.png', cv2.resize(im[330:420,80:560],None,fx=2,fy=2))
EOF

# openrua op 63
python3 - <<'EOF'
import numpy as np
d=np.load('eih10_depth.npy'); fx=312.774; cx=320; cy=240
for row in (360,375,390,405):
    r=d[row]; 
    # inner edges: last finger pixel left of centre, first finger pixel right of centre
    fing=np.where(r<0.115)[0]
    L=fing[fing<320].max(); Rr=fing[fing>320].min()
    XL=(L-cx)*r[L]/fx; XR=(Rr-cx)*r[Rr]/fx
    print(row, L, Rr, round(r[L],3), round(r[Rr],3), "X_L",round(XL,4),"X_R",round(XR,4),"gap",round(XR-XL,4))
EOF

# openrua op 64
python3 - <<'EOF'
import numpy as np
d=np.load('eih10_depth.npy'); fx=312.774; cx=320; cy=240
for row in (370,380,390,400,410):
    r=d[row]; fing=np.where(r<0.115)[0]
    l=fing[fing<320]; rr=fing[fing>320]
    if len(l)==0 or len(rr)==0: print(row,"n/a"); continue
    L=l.max(); Rr=rr.min()
    XL=(L-cx)*r[L]/fx; XR=(Rr-cx)*r[Rr]/fx
    print(row, L, Rr, round(float(r[L]),3), round(float(r[Rr]),3), "X_L",round(XL,4),"X_R",round(XR,4),"gap",round(XR-XL,4))
EOF

# openrua op 65
sed -i 's/0.1034\*zh/0.1175*zh/' pose.py && grep -n 0.1175 pose.py
cat > lid.py <<'EOF'
#!/usr/bin/env python3
"""lid.py zmin zmax [cam] -> centroid (world) of wrist-camera depth pixels whose world z is in [zmin,zmax]."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
zmin,zmax=float(sys.argv[1]),float(sys.argv[2]); cam=sys.argv[3] if len(sys.argv)>3 else "robot0_eye_in_hand"
rclpy.init(); node=rclpy.create_node("lid"); buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m: got.setdefault("d",m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m: got.setdefault("i",m),1)
while not("d" in got and "i" in got and buf.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time())):
    rclpy.spin_once(node,timeout_sec=0.2)
d=np.frombuffer(got["d"].data,dtype=np.float32).reshape(got["d"].height,got["d"].width)
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
t=buf.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time())
q=t.transform.rotation; tr=t.transform.translation
Rm=R.from_quat([q.x,q.y,q.z,q.w]).as_matrix(); T=np.array([tr.x,tr.y,tr.z])
v,u=np.mgrid[0:d.shape[0],0:d.shape[1]]
ok=np.isfinite(d)&(d>0.01)
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)[ok]@Rm.T+T
sel=(P[:,2]>zmin)&(P[:,2]<zmax)
pts=P[sel]
print("n",sel.sum())
if sel.sum():
    print("centroid",np.round(pts.mean(0),4).tolist(),"x",np.round([pts[:,0].min(),pts[:,0].max()],4).tolist(),"y",np.round([pts[:,1].min(),pts[:,1].max()],4).tolist(),"z",np.round([pts[:,2].min(),pts[:,2].max()],4).tolist())
EOF
P=$(python3 pose.py -0.265 -0.160 0.56 30); echo $P; timeout 900 python3 goto2.py $P 4 --steps 4 2>&1 | grep -v XMLPARSER | tail -2; timeout 120 python3 lid.py 0.495 0.53

# openrua op 66
python3 - <<'EOF'
import re
s=open('lid.py').read()
s=s.replace("sel=(P[:,2]>zmin)&(P[:,2]<zmax)","sel=(P[:,2]>zmin)&(P[:,2]<zmax)&(abs(P[:,0]-float(sys.argv[4]))<0.06)&(abs(P[:,1]-float(sys.argv[5]))<0.06)")
s=s.replace('cam=sys.argv[3] if len(sys.argv)>3 else "robot0_eye_in_hand"','cam="robot0_eye_in_hand"')
open('lid.py','w').write(s)
EOF
timeout 120 python3 lid.py 0.495 0.53 - -0.265 -0.16

# openrua op 67
P=$(python3 pose.py -0.262 -0.161 0.56 30); timeout 900 python3 goto2.py $P 3 --steps 2 2>&1 | grep -v XMLPARSER | tail -2; P=$(python3 pose.py -0.262 -0.161 0.455 30); echo $P; timeout 900 python3 goto2.py $P 5 --steps 8 2>&1 | grep -v XMLPARSER | grep -v "^step"; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force

# openrua op 68
sed -i 's/cam="robot0_eye_in_hand"/cam=sys.argv[3] if len(sys.argv)>3 and sys.argv[3]!="-" else "robot0_eye_in_hand"/' lid.py; for zr in "0.43 0.45" "0.45 0.47" "0.47 0.49" "0.49 0.505" "0.505 0.53"; do timeout 120 python3 lid.py $zr agentview -0.262 -0.161 | tail -1; done

# openrua op 69
P=$(python3 pose.py -0.262 -0.161 0.60 30); timeout 900 python3 goto2.py $P 4 --steps 6 2>&1 | grep -v XMLPARSER | tail -2; timeout 900 python3 goto2.py -0.15 -0.40 0.80 0.3827 0.9239 0 0 4 --steps 4 2>&1 | grep -v XMLPARSER | tail -2; for zr in "0.43 0.45" "0.45 0.47" "0.47 0.49" "0.49 0.505" "0.505 0.53"; do echo "agentview $zr: $(timeout 120 python3 lid.py $zr agentview -0.262 -0.161 | tail -1)"; echo "robotview $zr: $(timeout 120 python3 lid.py $zr robot0_robotview -0.262 -0.161 | tail -1)"; done

# openrua op 70
P=$(python3 pose.py -0.260 -0.1605 0.60 30); timeout 900 python3 goto2.py $P 4 --steps 4 2>&1 | grep -v XMLPARSER | tail -2; P=$(python3 pose.py -0.260 -0.1605 0.525 30); timeout 900 python3 goto2.py $P 3 --steps 4 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand"; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih11_depth.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih11.png >/dev/null; python3 - <<'EOF'
import numpy as np
d=np.load('eih11_depth.npy'); fx=312.774; cx=320; cy=240
def X(u,z): return (u-cx)*z/fx
# finger inner edges
for row in (375,390):
    r=d[row]; fing=np.where(r<0.112)[0]; l=fing[fing<320]; rr=fing[fing>320]
    if len(l) and len(rr):
        L=l.max(); Rr=rr.min(); print("fingers row",row,"X_L",round(X(L,r[L]),4),"X_R",round(X(Rr,r[Rr]),4))
# lid: pixels with depth between (lid distance) -> find rows with a plateau of depth < table
for row in range(200,370,10):
    r=d[row]; lid=np.where((r>0.02)&(r<0.16))[0]   # lid is closer than the table (~0.24 away at this height)
    if len(lid)>5:
        print("row",row,"lid u",lid.min(),lid.max(),"depth",round(float(np.median(r[lid])),3),"X",round(X(lid.min(),r[lid.min()]),4),round(X(lid.max(),r[lid.max()]),4))
EOF

# openrua op 71
P=$(python3 pose.py -0.260 -0.155 0.525 30); timeout 900 python3 goto2.py $P 2 --steps 2 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand"; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih12_depth.png >/dev/null; python3 - <<'EOF'
import numpy as np
d=np.load('eih12_depth.npy'); fx=312.774; cx=320; cy=240
def X(u,z): return (u-cx)*z/fx
for row in (375,390):
    r=d[row]; fing=np.where(r<0.112)[0]; l=fing[fing<320]; rr=fing[fing>320]
    if len(l) and len(rr):
        L=l.max(); Rr=rr.min(); print("fingers row",row,"X_L",round(X(L,r[L]),4),"X_R",round(X(Rr,r[Rr]),4))
for row in range(330,372,6):
    r=d[row]; lid=np.where((r>0.02)&(r<0.16))[0]
    if len(lid)>5:
        print("row",row,"lid X",round(X(lid.min(),r[lid.min()]),4),round(X(lid.max(),r[lid.max()]),4),"centre",round((X(lid.min(),r[lid.min()])+X(lid.max(),r[lid.max()]))/2,4))
EOF

# openrua op 72
P=$(python3 pose.py -0.260 -0.155 0.455 30); timeout 900 python3 goto2.py $P 4 --steps 6 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand"; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>&1 | grep -A3 force | tail -1

# openrua op 73
python3 - <<'EOF'
s=open('goto2.py').read()
s=s.replace('target=wps[-1]\n','target=wps[-1]\nif "--dry" in sys.argv: print("dry run, no motion"); sys.exit(0)\n')
open('goto2.py','w').write(s)
EOF
P=$(python3 pose.py -0.260 -0.155 0.60 30); timeout 900 python3 goto2.py $P 4 --steps 4 2>&1 | grep -v XMLPARSER | grep hand
for t in -20 -30 -40; do P=$(python3 pose.py -0.260 -0.155 0.455 $t); echo "tilt $t -> $P"; timeout 600 python3 goto2.py $P 4 --steps 1 --dry 2>&1 | grep -v XMLPARSER; done

# openrua op 74
cat > pose2.py <<'EOF'
#!/usr/bin/env python3
"""pose2.py tx ty tz  ax ay az  fx fy fz -> link8 pose (world) for TCP at t, hand approach axis a (points from hand
toward fingertips), finger-opening axis f (perpendicular to a). Fingertip offset 0.1175 m."""
import sys, numpy as np
from scipy.spatial.transform import Rotation as R
v=list(map(float,sys.argv[1:10])); t=np.array(v[0:3]); a=np.array(v[3:6]); f=np.array(v[6:9])
a=a/np.linalg.norm(a); f=f-np.dot(f,a)*a; f=f/np.linalg.norm(f); x=np.cross(f,a)
hand=R.from_matrix(np.column_stack([x,f,a]))
flange=t-0.1175*a
q=(hand*R.from_quat([0,0,0.3827,0.9239])).as_quat()
print(" ".join(f"{v:.4f}" for v in [*flange,*q]))
EOF
S=0.5; C=0.866
for cand in "0 $S -$C 1 0 0" "0 -$S -$C 1 0 0" "0 $S -$C -1 0 0" "0 -$S -$C -1 0 0" "0.2588 0 -0.9659 0 1 0" "0.2588 0 -0.9659 0 -1 0"; do P=$(python3 pose2.py -0.260 -0.155 0.455 $cand); echo "cand [$cand] -> $P"; timeout 600 python3 goto2.py $P 4 --steps 1 --dry 2>&1 | grep -v XMLPARSER | grep step; done

# openrua op 75
python3 tools/perception/cam_snap.py birdview bird2.png 2>&1 | grep -v XMLPARSER; python3 tools/perception/cam_snap.py /birdview/depth/image_raw bird2_depth.png 2>&1 | grep -v XMLPARSER
echo "--- soup lid"; python3 lid.py 0.495 0.520 birdview -0.26 -0.155 2>&1 | grep -v XMLPARSER
echo "--- ketchup cap"; python3 lid.py 0.535 0.60 birdview -0.14 -0.155 2>&1 | grep -v XMLPARSER
echo "--- ketchup body at z 0.50-0.53"; python3 lid.py 0.50 0.53 birdview -0.14 -0.155 2>&1 | grep -v XMLPARSER

# openrua op 76
cat lid.py | head -40

# openrua op 77
cat > scan.py <<'EOF'
#!/usr/bin/env python3
"""scan.py cam zmin zmax xmin xmax ymin ymax [grid=0.02] -> occupancy grid of world points in the box (from cam depth)."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation as R
cam=sys.argv[1]; zmin,zmax,xmin,xmax,ymin,ymax=map(float,sys.argv[2:8]); g=float(sys.argv[8]) if len(sys.argv)>8 else 0.02
rclpy.init(); node=rclpy.create_node("scan"); buf=Buffer(); TransformListener(buf,node)
got={}
node.create_subscription(Image,f"/{cam}/depth/image_raw",lambda m: got.setdefault("d",m),1)
node.create_subscription(CameraInfo,f"/{cam}/color/camera_info",lambda m: got.setdefault("i",m),1)
while not("d" in got and "i" in got and buf.can_transform("world",f"{cam}_optical_frame",rclpy.time.Time())):
    rclpy.spin_once(node,timeout_sec=0.2)
d=np.frombuffer(got["d"].data,dtype=np.float32).reshape(got["d"].height,got["d"].width)
k=got["i"].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
t=buf.lookup_transform("world",f"{cam}_optical_frame",rclpy.time.Time())
q=t.transform.rotation; tr=t.transform.translation
Rm=R.from_quat([q.x,q.y,q.z,q.w]).as_matrix(); T=np.array([tr.x,tr.y,tr.z])
v,u=np.mgrid[0:d.shape[0],0:d.shape[1]]
ok=np.isfinite(d)&(d>0.01)
P=np.stack([(u-cx)*d/fx,(v-cy)*d/fy,d],-1)[ok]@Rm.T+T
sel=(P[:,2]>zmin)&(P[:,2]<zmax)&(P[:,0]>xmin)&(P[:,0]<xmax)&(P[:,1]>ymin)&(P[:,1]<ymax)
pts=P[sel]; print("n",len(pts))
xs=np.arange(xmin,xmax,g); ys=np.arange(ymin,ymax,g)
H=np.zeros((len(ys),len(xs))); Z=np.zeros_like(H)
for p in pts:
    i=int((p[1]-ymin)/g); j=int((p[0]-xmin)/g); H[i,j]+=1; Z[i,j]=max(Z[i,j],p[2])
print("rows=y (top=ymin) cols=x (left=xmin); cell = max z (cm) or . if empty")
print("      "+" ".join(f"{x*100:4.0f}" for x in xs))
for i,y in enumerate(ys):
    print(f"{y*100:5.0f} "+" ".join(f"{Z[i,j]*100:4.1f}" if H[i,j]>0 else "   ." for j in range(len(xs))))
EOF
python3 scan.py birdview 0.44 0.70 -0.36 -0.02 -0.26 -0.06 0.02 2>&1 | grep -v XMLPARSER

# openrua op 78
for c in agentview sideview frontview; do echo "== $c"; python3 scan.py $c 0.44 0.60 -0.36 -0.02 -0.26 -0.06 0.02 2>&1 | grep -v XMLPARSER; done

# openrua op 79
python3 scan.py agentview 0.50 0.60 -0.31 -0.08 -0.24 -0.10 0.01 2>&1 | grep -v XMLPARSER

# openrua op 80
for t in 0 10 15; do P=$(python3 pose.py -0.255 -0.155 0.48 $t); echo "tilt $t -> $P"; timeout 600 python3 goto2.py $P 4 --steps 1 --dry 2>&1 | grep -v XMLPARSER | grep step; done; ros2 topic echo /joint_states --once 2>/dev/null | grep -A9 position | head -10

# openrua op 81
P=$(python3 pose.py -0.255 -0.155 0.60 0); echo $P; timeout 900 python3 goto2.py $P 5 --steps 6 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"

# openrua op 82
python3 tools/perception/cam_snap.py robot0_eye_in_hand eih20.png 2>&1 | grep -v XMLPARSER; python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih20_depth.png 2>&1 | grep -v XMLPARSER
python3 lid.py 0.50 0.53 - -0.255 -0.155 2>&1 | grep -v XMLPARSER
python3 - <<'EOF'
import numpy as np
d=np.load("eih20_depth.npy"); fx=312.774; cx=320; cy=240
# finger inner edges: near-depth pixels in rows 370-395
for r in (375,385,395):
    row=d[r]; near=np.where(np.isfinite(row)&(row<0.12))[0]
    if near.size:
        # find the gap between the two fingers: near pixels left/right of centre
        L=near[near<320]; Rr=near[near>320]
        print(r, "left finger inner edge px", L.max() if L.size else None, "right finger inner edge px", Rr.min() if Rr.size else None,
              "-> X at can-lid depth:", None)
# lid pixels (depth ~ cam height - lid) : camera z ~ 0.7175+? print depth stats center
c=d[200:280,260:380]; print("centre depth range", np.nanmin(c), np.nanmax(c))
EOF

# openrua op 83
python3 - <<'EOF'
import numpy as np
d=np.load("eih20_depth.npy")
# lid = pixels with depth within the lid band: camera height above lid; find band around min depth in the central region excluding fingers
m=np.isfinite(d)&(d>0.15)&(d<0.40)
vals=d[m]; print("depth hist (m):", np.round(np.percentile(vals,[0,5,25,50,75,95,100]),3))
lidband=(d>0.19)&(d<0.225)
v,u=np.nonzero(lidband); print("lid px n",len(u),"u centre",u.mean().round(1),"u range",u.min(),u.max(),"v centre",v.mean().round(1),"v range",v.min(),v.max())
EOF

# openrua op 84
python3 - <<'EOF'
import numpy as np
d=np.load("eih20_depth.npy")
lid=(d>0.19)&(d<0.207)
v,u=np.nonzero(lid); print("lid px n",len(u),"u centre",u.mean().round(1),"u range",u.min(),u.max(),"v centre",v.mean().round(1),"v range",v.min(),v.max())
# finger tip rows: near pixels
near=np.isfinite(d)&(d<0.13); vv,uu=np.nonzero(near); print("finger px v range",vv.min(),vv.max(),"u ranges", np.unique(uu//20*20))
EOF

# openrua op 85
python3 - <<'EOF'
import numpy as np
d=np.load("eih20_depth.npy")
sub=d[240:380,240:420]; lid=(sub>0.19)&(sub<0.21)
v,u=np.nonzero(lid); u=u+240; v=v+240
print("lid px n",len(u),"u centre",u.mean().round(1),"u range",u.min(),u.max(),"v centre",v.mean().round(1),"v range",v.min(),v.max())
print("lid depth", np.round(np.percentile(sub[lid],[0,50,100]),4))
EOF

# openrua op 86
P=$(python3 pose.py -0.250 -0.1465 0.60 0); timeout 900 python3 goto2.py $P 3 --steps 2 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih21_depth.png 2>&1 | grep -v XMLPARSER
python3 - <<'EOF'
import numpy as np
d=np.load("eih21_depth.npy")
sub=d[240:380,240:420]; lid=(sub>0.19)&(sub<0.21)
v,u=np.nonzero(lid); u=u+240; v=v+240
print("lid u centre",(u.min()+u.max())/2,"range",u.min(),u.max(),"v centre",(v.min()+v.max())/2,"range",v.min(),v.max())
for r in (375,385,395):
    row=d[r]; near=np.where(np.isfinite(row)&(row<0.12))[0]; L=near[near<320]; Rr=near[near>320]
    print(r,"finger inner edges",L.max(),Rr.min(),"gap centre",(L.max()+Rr.min())/2)
EOF
python3 lid.py 0.50 0.53 - -0.25 -0.1465 2>&1 | grep -v XMLPARSER

# openrua op 87
for z in 0.53 0.475; do P=$(python3 pose.py -0.250 -0.1468 $z 0); echo "== TCP z $z"; timeout 900 python3 goto2.py $P 4 --steps 4 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>/dev/null | grep -A3 force: | tail -3 | tr '\n' ' '; echo; done

# openrua op 88
timeout 120 python3 tools/action/gripper_cmd.py 0.0 2>&1 | grep -v XMLPARSER | tail -2
P=$(python3 pose.py -0.250 -0.1468 0.485 0); timeout 600 python3 goto2.py $P 3 --steps 1 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>/dev/null | grep -A3 force: | tail -3 | tr '\n' ' '; echo

# openrua op 89
P=$(python3 pose.py -0.250 -0.1468 0.62 0); timeout 600 python3 goto2.py $P 4 --steps 3 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>/dev/null | grep -A3 force: | tail -3 | tr '\n' ' '; echo
python3 tools/perception/cam_snap.py agentview agent20.png 2>&1 | grep -v XMLPARSER
python3 scan.py agentview 0.44 0.60 -0.31 -0.08 -0.24 -0.10 0.02 2>&1 | grep -v XMLPARSER

# openrua op 90
python3 scan.py agentview 0.43 0.75 -0.14 0.20 0.12 0.44 0.02 2>&1 | grep -v XMLPARSER

# openrua op 91
python3 scan.py agentview 0.40 0.75 -0.10 0.12 0.16 0.38 0.01 2>&1 | grep -v XMLPARSER

# openrua op 92
python3 scan.py birdview 0.40 0.75 -0.10 0.12 0.16 0.38 0.01 2>&1 | grep -v XMLPARSER

# openrua op 93
P=$(python3 pose.py -0.25 -0.147 0.70 0); timeout 600 python3 goto2.py $P 4 --steps 3 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
P=$(python3 pose.py -0.015 0.25 0.70 0); timeout 900 python3 goto2.py $P 8 --steps 8 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"

# openrua op 94
ros2 topic echo /joint_states --once 2>/dev/null | grep -A7 position | tail -7 | tr '\n' ' '; echo; P=$(python3 pose.py -0.015 0.25 0.70 0); timeout 300 python3 goto2.py $P 4 --steps 1 --dry 2>&1 | grep -v XMLPARSER | grep step

# openrua op 95
P=$(python3 pose.py -0.015 0.25 0.70 0); timeout 900 python3 goto2.py $P 4 --steps 1 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
P=$(python3 pose.py -0.015 0.25 0.60 0); timeout 900 python3 goto2.py $P 4 --steps 3 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>/dev/null | grep -A3 force: | tail -3 | tr '\n' ' '; echo

# openrua op 96
timeout 120 python3 tools/action/gripper_cmd.py 0.08 2>&1 | grep -v XMLPARSER | tail -1
P=$(python3 pose.py -0.015 0.25 0.70 0); timeout 600 python3 goto2.py $P 3 --steps 2 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
python3 scan.py birdview 0.40 0.75 -0.10 0.12 0.16 0.38 0.01 2>&1 | grep -v XMLPARSER | sed -n '1,20p'

# openrua op 97
P=$(python3 pose.py -0.20 0.02 0.70 0); timeout 900 python3 goto2.py $P 5 --steps 4 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
python3 scan.py birdview 0.40 0.75 -0.10 0.12 0.16 0.38 0.01 2>&1 | grep -v XMLPARSER
python3 tools/perception/cam_snap.py agentview agent21.png 2>&1 | grep -v XMLPARSER

# openrua op 98
python3 scan.py agentview 0.44 0.70 -0.06 0.16 -0.30 -0.04 0.01 2>&1 | grep -v XMLPARSER

# openrua op 99
P=$(python3 pose.py 0.02 -0.23 0.60 0); timeout 900 python3 goto2.py $P 6 --steps 6 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
python3 tools/perception/cam_snap.py robot0_eye_in_hand eih22.png 2>&1 | grep -v XMLPARSER; python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih22_depth.png 2>&1 | grep -v XMLPARSER
python3 - <<'EOF'
import numpy as np
d=np.load("eih22_depth.npy")
sub=d[220:380,220:440]; lid=(sub>0.185)&(sub<0.215)
v,u=np.nonzero(lid); u=u+220; v=v+220
print("lid depth",np.round(np.percentile(sub[lid],[0,50,100]),4))
print("lid u centre",(u.min()+u.max())/2,"range",u.min(),u.max(),"v centre",(v.min()+v.max())/2,"range",v.min(),v.max())
for r in (375,385,395):
    row=d[r]; near=np.where(np.isfinite(row)&(row<0.12))[0]; L=near[near<320]; Rr=near[near>320]
    print(r,"finger inner edges",L.max(),Rr.min(),"gap centre",(L.max()+Rr.min())/2)
EOF
python3 lid.py 0.50 0.53 - 0.02 -0.23 2>&1 | grep -v XMLPARSER

# openrua op 100
for z in 0.53 0.472; do P=$(python3 pose.py 0.02 -0.2297 $z 0); echo "== TCP z $z"; timeout 900 python3 goto2.py $P 4 --steps 4 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand"; ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>/dev/null | grep -A3 force: | tail -3 | tr '\n' ' '; echo; done
timeout 120 python3 tools/action/gripper_cmd.py 0.0 2>&1 | grep -v XMLPARSER | tail -1
P=$(python3 pose.py 0.02 -0.2297 0.485 0); timeout 600 python3 goto2.py $P 3 --steps 1 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
P=$(python3 pose.py 0.02 -0.2297 0.70 0); timeout 600 python3 goto2.py $P 4 --steps 3 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
python3 scan.py agentview 0.44 0.60 -0.06 0.16 -0.30 -0.04 0.02 2>&1 | grep -v XMLPARSER | head -12

# openrua op 101
P=$(python3 pose.py 0.015 0.295 0.70 0); timeout 900 python3 goto2.py $P 8 --steps 8 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"

# openrua op 102
P=$(python3 pose.py 0.015 0.295 0.60 0); timeout 900 python3 goto2.py $P 4 --steps 3 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once 2>/dev/null | grep -A3 force: | tail -3 | tr '\n' ' '; echo
timeout 120 python3 tools/action/gripper_cmd.py 0.08 2>&1 | grep -v XMLPARSER | tail -1
P=$(python3 pose.py 0.015 0.295 0.70 0); timeout 600 python3 goto2.py $P 3 --steps 2 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
P=$(python3 pose.py -0.20 0.02 0.70 0); timeout 900 python3 goto2.py $P 5 --steps 4 2>&1 | grep -v XMLPARSER | grep -E "attempt|hand|fingers"
python3 scan.py birdview 0.40 0.75 -0.10 0.12 0.16 0.38 0.01 2>&1 | grep -v XMLPARSER
python3 tools/perception/cam_snap.py agentview agent22.png 2>&1 | grep -v XMLPARSER; python3 tools/perception/cam_snap.py sideview side22.png 2>&1 | grep -v XMLPARSER

# openrua op 103
python3 tools/perception/cam_snap.py frontview front22.png 2>&1 | grep -v XMLPARSER; python3 -c "
import cv2; im=cv2.imread('agent22.png'); cv2.imwrite('agent22c.png', cv2.resize(im[190:330,440:630],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 104
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-kinematics-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-kinematics-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-kinematics-quirks
description: Verified frame/geometry facts for the /workspace Panda sim that contradict its docs (world-frame IK, panda_link8 tip, 0.1175 fingertip offset, j7 lag)
metadata:
  type: project
---

Measured on the /workspace Franka Panda LIBERO-style sim (Sept 2026), where docs/machine.yaml are wrong or silent:
- /compute_ik and /compute_fk with empty frame_id use the **world** frame (not panda_link0); world→panda_link0 = (-0.51, 0, 0.42). IK tip is **panda_link8**, yawed +45° from panda_hand (link8 = hand ⊗ (0,0,0.3827,0.9239)).
- Fingertip (TCP) is **0.1175 m** below the flange (manifest says 0.1034 — wrong); open finger inner gap 0.0776 m; wrist camera protrudes ~5 cm on hand +X.
- Joint 7 lags: FJT returns -5 but re-sending the same goal converges; single-point joint goals bow in Cartesian space — use stepped waypoints (goto2.py). Clock is paused between commands: verify by re-reading sensors after a further command, never by waiting.
- Wrist-cam trick: finger inner edges sit at u≈201/438 (rows 375–395); centre a lid between them before descending. Tilted grasps fail on ~7 cm cans (hand-body corner hits the lid) — go vertical.
- Basket interior at floor ≈ 10×11 cm (rim z 0.578, floor 0.445); two cans only fit diagonally/leaning.

**Why:** the starter docs mislead on frames and TCP offset; rediscovering cost most of the session.
**How to apply:** trust these over docs/10-machine.md; helpers pose.py/goto2.py/scan.py/lid.py in /workspace already encode them.
OPENRUA_EOF

# openrua op 105
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true); echo "- [Panda sim kinematics quirks](panda-sim-kinematics-quirks.md) — world-frame IK, link8 tip, 0.1175 TCP offset, j7 lag, basket size" >> MEMORY.md
