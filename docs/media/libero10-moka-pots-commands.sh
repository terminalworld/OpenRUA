#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
/bin/bash -lc "pwd && rg --files -g 'AGENTS.md' -g '*README*' -g '*.md' -g '*.py' -g '*.json' -g '*.sh'"

# openrua op 1
/bin/bash -lc 'cat README.md docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md'

# openrua op 2
/bin/bash -lc 'cat machine.yaml tools/action/ik_move.py tools/perception/px2world.py'

# openrua op 3
/bin/bash -lc 'ros2 topic list && ros2 topic echo /joint_states --once && python3 tools/perception/cam_snap.py agentview'

# openrua op 4
/bin/bash -lc 'python3 tools/perception/cam_snap.py birdview && python3 tools/perception/cam_snap.py frontview'

# openrua op 5
/bin/bash -lc 'ros2 service list && cat tools/action/fjt_send.py tools/action/gripper_cmd.py'

# openrua op 6
/bin/bash -lc 'python3 tools/perception/px2world.py birdview 260 240 && python3 tools/perception/px2world.py birdview 388 277 && python3 tools/perception/px2world.py birdview 334 356 && python3 tools/perception/px2world.py birdview 310 340 && ros2 topic echo /tf_static --once'

# openrua op 7
/bin/bash -lc "ros2 topic echo /tf --once && ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK \"{header: {frame_id: ''}, fk_link_names: [panda_hand, panda_link0], robot_state: {joint_state: {name: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], position: [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]}}}\""

# openrua op 8
/bin/bash -lc "cat > /workspace/control.py <<'PY'
import sys, json, rclpy, numpy as np
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
rclpy.init(); n=rclpy.create_node('task_control'); got=[]
n.create_subscription(JointState,'/joint_states',got.append,1)
def spin(f):
 rclpy.spin_until_future_complete(n,f); return f.result()
def state():
 got.clear()
 while not got:rclpy.spin_once(n,timeout_sec=.2)
 return got[-1]
names=['panda_joint'+str(i) for i in range(1,8)]
def arm():
 s=state(); d=dict(zip(s.name,s.position)); j=JointState();j.name=names;j.position=[d[k] for k in names];return j
def fk(j):
 c=n.create_client(GetPositionFK,'/compute_fk');c.wait_for_service();q=GetPositionFK.Request();q.fk_link_names=['panda_hand'];q.robot_state.joint_state=j;return spin(c.call_async(q)).pose_stamped[0].pose
cmd=sys.argv[1]
if cmd=='state':
 j=arm();print('HAND',fk(j));s=state();print('JOINTS',dict(zip(s.name,s.position)))
elif cmd in ['move','solve']:
 xyz=list(map(float,sys.argv[2:5]));quat=list(map(float,sys.argv[5:9])) if len(sys.argv)>5 else [1.,0.,0.,0.]
 c=n.create_client(GetPositionIK,'/compute_ik');c.wait_for_service();q=GetPositionIK.Request();q.ik_request.group_name='panda_arm';q.ik_request.robot_state.joint_state=arm();p=q.ik_request.pose_stamped.pose;p.position.x,p.position.y,p.position.z=xyz;p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w=quat
 r=spin(c.call_async(q));print('IK',r.error_code.val,flush=True)
 if r.error_code.val"'!=1:sys.exit(1)
 j=JointState();d=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position));j.name=names;j.position=[d[k] for k in names];p=fk(j);print('"'FK',p,flush=True)
 if cmd=='move':
  assert np.linalg.norm(np.array([p.position.x,p.position.y,p.position.z])-xyz)<.01
  c=ActionClient(n,FollowJointTrajectory,'/panda_arm_controller/follow_joint_trajectory');c.wait_for_server();q=FollowJointTrajectory.Goal();q.trajectory.joint_names=names;q.trajectory.points=[JointTrajectoryPoint(positions=j.position,time_from_start=Duration(sec=4))];h=spin(c.send_goal_async(q));print('RESULT',spin(h.get_result_async()).result,flush=True);print('ACTUAL',fk(arm()),flush=True)
rclpy.shutdown()
PY
python3 control.py solve -0.2 -0.20 1.24"

# openrua op 9
/bin/bash -lc 'python3 tools/action/gripper_cmd.py 0.04 && python3 control.py move -0.20 -0.265 1.24 0.92387953 0.38268343 0 0'

# openrua op 10
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py birdview'

# openrua op 11
/bin/bash -lc 'python3 tools/perception/px2world.py birdview 240 240 && python3 tools/perception/px2world.py birdview 247 240 && python3 tools/perception/px2world.py birdview 321 340 && python3 tools/perception/px2world.py birdview 346 370'

# openrua op 12
/bin/bash -lc 'ros2 topic echo /joint_states --once'

# openrua op 13
/bin/bash -lc 'python3 control.py move -0.20 -0.265 1.24 0.92387953 0.38268343 0 0'

# openrua op 14
/bin/bash -lc 'python3 tools/perception/px2world.py birdview 240 240'

# openrua op 15
/bin/bash -lc "cp tools/perception/px2world.py measure.py
python3 - <<'PY'
p='measure.py'
s=open(p).read().replace('if len(sys.argv) "'!= 4:'"', 'if len(sys.argv) < 4:')
s=s.replace('cam, u, v = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])','cam, u, v = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])')
s=s.replace('print(f\"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\")','''for i in range(2,len(sys.argv),2):
        u,v=map(int,sys.argv[i:i+2])
        z=struct.unpack_from(\"<f\",depth.data,(v*depth.width+u)*4)[0]
        p=T @ np.array([(u-cx)*z/fx,(v-cy)*z/fy,z,1.])
        print(u,v, np.round(p[:3],4))''')
open(p,'w').write(s)
PY"

# openrua op 16
/bin/bash -lc "sed -i 's/Duration(sec=4)/Duration(sec=8)/' control.py
python3 control.py move -0.20 -0.265 1.24 0.92387953 0.38268343 0 0"

# openrua op 17
/bin/bash -lc 'python3 measure.py birdview 240 240 246 240 260 240 367 277 388 277 321 340 346 370 333 313 && python3 tools/perception/cam_snap.py birdview'

# openrua op 18
/bin/bash -lc "sed -i 's/Duration(sec=8)/Duration(sec=4)/' control.py
python3 control.py move -0.20 -0.265 1.125 0.92387953 0.38268343 0 0"

# openrua op 19
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py robot0_eye_in_hand'

# openrua op 20
/bin/bash -lc 'python3 tools/action/gripper_cmd.py 0.0 && python3 control.py state'

# openrua op 21
/bin/bash -lc "python3 - <<'PY'
p='control.py';s=open(p).read().replace('import sys, json, rclpy, numpy as np','import sys, json, rclpy, numpy as np, os').replace('Duration(sec=4)','Duration(sec=int(os.environ.get(\"MOVE_SECONDS\",\"4\")))');open(p,'w').write(s)
PY"

# openrua op 22
/bin/bash -lc 'python3 control.py move -0.20 -0.265 1.25 0.92387953 0.38268343 0 0'

# openrua op 23
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview && python3 control.py solve 0.115 0.0 1.15 0.92387953 -0.38268343 0 0'

# openrua op 24
/bin/bash -lc 'MOVE_SECONDS=8 python3 control.py move 0.115 0.0 1.25 0.92387953 -0.38268343 0 0'

# openrua op 25
/bin/bash -lc 'python3 control.py move 0.115 0.0 1.25 0.92387953 -0.38268343 0 0'

# openrua op 26
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py birdview'

# openrua op 27
/bin/bash -lc 'MOVE_SECONDS=3 python3 control.py move 0.115 0.0 1.153 0.92387953 -0.38268343 0 0'

# openrua op 28
/bin/bash -lc 'python3 tools/perception/cam_snap.py sideview'

# openrua op 29
/bin/bash -lc 'MOVE_SECONDS=2 python3 control.py move 0.115 0.0 1.15 0.92387953 -0.38268343 0 0'

# openrua op 30
/bin/bash -lc 'python3 tools/action/gripper_cmd.py 0.04 && MOVE_SECONDS=3 python3 control.py move 0.11 0.0 1.28 0.92387953 -0.38268343 0 0'

# openrua op 31
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py birdview'

# openrua op 32
/bin/bash -lc 'python3 measure.py birdview 367 277 371 277 388 277 386 281'

# openrua op 33
/bin/bash -lc 'MOVE_SECONDS=10 python3 control.py move -0.075 0.163 1.25 0.92387953 0.38268343 0 0'

# openrua op 34
/bin/bash -lc 'MOVE_SECONDS=3 python3 control.py move -0.075 0.163 1.125 0.92387953 0.38268343 0 0'

# openrua op 35
/bin/bash -lc 'python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 tools/perception/cam_snap.py agentview'

# openrua op 36
/bin/bash -lc 'python3 tools/action/gripper_cmd.py 0.0 && python3 control.py state'

# openrua op 37
/bin/bash -lc 'MOVE_SECONDS=3 python3 control.py move -0.075 0.163 1.27 0.92387953 0.38268343 0 0'

# openrua op 38
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview'

# openrua op 39
/bin/bash -lc 'MOVE_SECONDS=10 python3 control.py move 0.115 0.10 1.27 0.92387953 -0.38268343 0 0'

# openrua op 40
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview'

# openrua op 41
/bin/bash -lc 'MOVE_SECONDS=3 python3 control.py move 0.115 0.10 1.166 0.92387953 -0.38268343 0 0'

# openrua op 42
/bin/bash -lc 'python3 tools/action/gripper_cmd.py 0.04 && MOVE_SECONDS=3 python3 control.py move 0.11 0.10 1.29 0.92387953 -0.38268343 0 0'

# openrua op 43
/bin/bash -lc 'MOVE_SECONDS=4 python3 control.py move -0.20 0 1.30 0.92387953 -0.38268343 0 0'

# openrua op 44
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview && python3 tools/perception/cam_snap.py birdview && python3 tools/perception/cam_snap.py sideview'
