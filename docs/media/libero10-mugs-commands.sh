#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
/bin/bash -lc "pwd && rg --files -g 'AGENTS.md' -g '*README*' -g '*.py' -g '*.md' -g '*.yaml' -g '*.json' /workspace"

# openrua op 1
/bin/bash -lc 'cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md'

# openrua op 2
/bin/bash -lc 'ros2 topic list'

# openrua op 3
/bin/bash -lc 'cat tools/action/ik_move.py tools/perception/px2world.py'

# openrua op 4
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview /workspace/start.png'

# openrua op 5
/bin/bash -lc 'python3 tools/perception/cam_snap.py birdview /workspace/bird.png'

# openrua op 6
/bin/bash -lc 'ros2 service list && ros2 topic echo /joint_states --once'

# openrua op 7
/bin/bash -lc 'ros2 topic echo /tf_static --once'

# openrua op 8
/bin/bash -lc 'python3 tools/perception/px2world.py agentview 389 220 && python3 tools/perception/px2world.py agentview 192 220 && python3 tools/perception/px2world.py birdview 246 281 && python3 tools/perception/px2world.py birdview 389 289'

# openrua op 9
/bin/bash -lc "ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK \"{header: {frame_id: ''}, fk_link_names: [panda_link0, panda_hand], robot_state: {joint_state: {name: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], position: [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]}}}\""

# openrua op 10
/bin/bash -lc 'cat tools/action/fjt_send.py tools/action/gripper_cmd.py'

# openrua op 11
/bin/bash -lc "cat > /workspace/control.py <<'PY'
import sys,json,time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK,GetPositionFK
from control_msgs.action import FollowJointTrajectory,GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation
rclpy.init(); n=rclpy.create_node('task_control'); js=None
names=['panda_joint'+str(i) for i in range(1,8)]
def cb(m):
 global js
 js=m
n.create_subscription(JointState,'/joint_states',cb,1)
def fresh():
 global js
 js=None
 while js is None:rclpy.spin_once(n,timeout_sec=.2)
 return dict(zip(js.name,js.position))
def call(c,r):
 c.wait_for_service(); f=c.call_async(r); rclpy.spin_until_future_complete(n,f); return f.result()
ik=n.create_client(GetPositionIK,'/compute_ik'); fk=n.create_client(GetPositionFK,'/compute_fk')
def state(d):
 s=JointState();s.name=names;s.position=[d[k] for k in names];return s
def pose(d):
 r=GetPositionFK.Request();r.fk_link_names=['panda_hand'];r.robot_state.joint_state=state(d)
 p=call(fk,r).pose_stamped[0].pose
 xyz=np.array([p.position.x,p.position.y,p.position.z]);q=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
 return xyz, q, xyz+Rotation.from_quat(q).apply([0,0,.1034])
def act(typ,port,g):
 c=ActionClient(n,typ,port);c.wait_for_server();f=c.send_goal_async(g);rclpy.spin_until_future_complete(n,f);f=f.result().get_result_async();rclpy.spin_until_future_complete(n,f);print('result',f.result().result,flush=True)
a=sys.argv[1];d=fresh();print('before',pose(d),'fingers',[d[k] for k in d if 'finger' in k],flush=True)
if a in ['move','probe']:
 target=np.array(list(map(float,sys.argv[2:5])));q=list(map(float,sys.argv[5:9])) if len(sys.argv)>5 else [1.,0.,0.,0.]
 hand=target-Rotation.from_quat(q).apply([0,0,.1034])
 r=GetPositionIK.Request();r.ik_request.group_name='panda_arm';r.ik_request.robot_state.joint_state=state(d)
 p=r.ik_request.pose_stamped.pose;p.position.x,p.position.y,p.position.z=hand;p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=q
 res=call(ik,r);print('ik',res.error_code.val,flush=True)
 if res.error_code.val"'!=1:sys.exit(1)
 sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position));actual=pose(sol);print('"'solution fk',actual,flush=True)
 if np.linalg.norm(actual[2]-target)>.015:raise RuntimeError('coordinate mismatch')
 if a=='move':
  g=FollowJointTrajectory.Goal();g.trajectory.joint_names=names;pt=JointTrajectoryPoint();pt.positions=[sol[k] for k in names];pt.time_from_start.sec=4;g.trajectory.points=[pt];act(FollowJointTrajectory,'/panda_arm_controller/follow_joint_trajectory',g)
  d=fresh();print('after',pose(d),'joint error',max(abs(d[k]-sol[k]) for k in names),flush=True)
elif a=='grip':
 g=GripperCommand.Goal();g.command.position=float(sys.argv[2]);g.command.max_effort=30.;act(GripperCommand,'/franka_gripper/gripper_action',g);d=fresh();print('fingers',[d[k] for k in d if 'finger' in k],flush=True)
rclpy.shutdown()
PY
python3 -u control.py probe -0.05 0.0 0.8"

# openrua op 12
/bin/bash -lc 'python3 -u control.py move -0.05 0 0.8'

# openrua op 13
/bin/bash -lc "cp tools/perception/px2world.py /workspace/points.py
python3 - <<'PY'
p='points.py'
s=open(p).read().replace('if len(sys.argv) "'!= 4:'"', 'if len(sys.argv) < 4:').replace('cam, u, v = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])','cam, u, v = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])')
s=s.replace('p = T @ p_cam\\n    print(f\"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\")','''for i in range(2,len(sys.argv),2):
        u,v=map(int,sys.argv[i:i+2])
        z=struct.unpack_from(\"<f\",depth.data,(v*depth.width+u)*4)[0]
        p=T @ np.array([(u-cx)*z/fx,(v-cy)*z/fy,z,1.])
        print(u,v, np.round(p[:3],4))''')
open(p,'w').write(s)
PY"

# openrua op 14
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview /workspace/current.png && python3 tools/perception/cam_snap.py birdview /workspace/bird2.png'

# openrua op 15
/bin/bash -lc 'python3 -u control.py grip 0.04'

# openrua op 16
/bin/bash -lc 'python3 points.py agentview 355 256 334 261 346 247 389 238 356 237 421 242 193 229 161 231 225 230 239 255 88 315 546 330 && python3 points.py birdview 246 281 387 290 277 265 285 264 269 264'

# openrua op 17
/bin/bash -lc 'python3 tools/perception/cam_snap.py paperview /workspace/paper.png && python3 tools/perception/cam_snap.py sideview /workspace/side.png'

# openrua op 18
/bin/bash -lc 'python3 -u control.py move -0.075 0.025 0.67 0.9238795 0.3826834 0 0'

# openrua op 19
/bin/bash -lc 'python3 -u control.py move -0.075 0.025 0.51 0.9238795 0.3826834 0 0'

# openrua op 20
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview /workspace/grasp.png && python3 tools/perception/cam_snap.py robot0_eye_in_hand /workspace/wrist.png'

# openrua op 21
/bin/bash -lc 'python3 -u control.py grip 0'

# openrua op 22
/bin/bash -lc 'python3 -u control.py move -0.075 0.025 0.69 0.9238795 0.3826834 0 0'

# openrua op 23
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview /workspace/lift.png'

# openrua op 24
/bin/bash -lc 'python3 -u control.py grip 0.04 && python3 -u control.py move -0.025 0.10 0.65 0.9238795 0.3826834 0 0'

# openrua op 25
/bin/bash -lc 'python3 points.py agentview 388 256 390 222 355 237 420 242 388 302'

# openrua op 26
/bin/bash -lc 'python3 -u control.py move -0.026 0.095 0.515 0.9238795 0.3826834 0 0'

# openrua op 27
/bin/bash -lc 'python3 -u control.py grip 0'

# openrua op 28
/bin/bash -lc 'python3 -u control.py move -0.026 0.095 0.70 0.9238795 0.3826834 0 0'

# openrua op 29
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview /workspace/lift2.png'

# openrua op 30
/bin/bash -lc 'python3 -u control.py move 0.026 -0.326 0.72 0.9238795 0.3826834 0 0'

# openrua op 31
/bin/bash -lc 'python3 -u control.py move 0.026 -0.326 0.545 0.9238795 0.3826834 0 0'

# openrua op 32
/bin/bash -lc 'python3 -u control.py grip 0.04 && python3 -u control.py move 0.026 -0.326 0.73 0.9238795 0.3826834 0 0'

# openrua op 33
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview /workspace/placed1.png && python3 tools/perception/cam_snap.py birdview /workspace/placed1top.png'

# openrua op 34
/bin/bash -lc 'python3 points.py agentview 195 244 194 212 162 229 225 229 195 299'

# openrua op 35
/bin/bash -lc 'python3 -u control.py move -0.032 -0.18 0.70 0.9238795 0.3826834 0 0'

# openrua op 36
/bin/bash -lc 'python3 -u control.py move -0.044 -0.178 0.522 0.9238795 0.3826834 0 0'

# openrua op 37
/bin/bash -lc 'python3 -u control.py grip 0'

# openrua op 38
/bin/bash -lc 'python3 -u control.py move -0.044 -0.178 0.74 0.9238795 0.3826834 0 0'

# openrua op 39
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview /workspace/white_lift.png'

# openrua op 40
/bin/bash -lc 'python3 -u control.py move 0.065 0.296 0.76 0.9238795 0.3826834 0 0'

# openrua op 41
/bin/bash -lc 'python3 -u control.py move 0.065 0.296 0.551 0.9238795 0.3826834 0 0'

# openrua op 42
/bin/bash -lc 'python3 -u control.py grip 0.04 && python3 -u control.py move 0.065 0.296 0.76 0.9238795 0.3826834 0 0'

# openrua op 43
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview /workspace/final.png && python3 tools/perception/cam_snap.py sideview /workspace/final_side.png'
