#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
/bin/bash -lc "pwd && rg --files -g 'AGENTS.md' -g '*README*' -g '*.md' -g '*.py' -g '*.json' -g '*.sh'"

# openrua op 1
/bin/bash -lc 'cat README.md docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md'

# openrua op 2
/bin/bash -lc 'cat machine.yaml tools/action/ik_move.py tools/perception/px2world.py && ros2 topic list && ros2 service list'

# openrua op 3
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview /workspace/agent.png && python3 tools/perception/cam_snap.py birdview /workspace/bird.png && ros2 topic echo /joint_states --once'

# openrua op 4
/bin/bash -lc 'python3 tools/perception/px2world.py birdview 373 237 && python3 tools/perception/px2world.py birdview 308 281 && python3 tools/perception/px2world.py birdview 373 281 && ros2 topic echo /tf_static --once'

# openrua op 5
/bin/bash -lc "ros2 topic echo /tf --once && ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK \"{header: {frame_id: ''}, fk_link_names: [panda_hand], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981634]}}}\""

# openrua op 6
/bin/bash -lc 'cat tools/action/fjt_send.py tools/action/gripper_cmd.py'

# openrua op 7
/bin/bash -lc 'python3 tools/action/gripper_cmd.py 0.04 && python3 tools/action/ik_move.py -0.2106 0.1867 1.20 0.7071068 0.7071068 0 0 3'

# openrua op 8
/bin/bash -lc 'python3 tools/perception/cam_snap.py agentview /workspace/agent.png && python3 tools/perception/px2world.py birdview 373 237'

# openrua op 9
/bin/bash -lc "cat > /workspace/state.py <<'PY'
import rclpy
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
rclpy.init(); n=rclpy.create_node('read_state'); got=[]
n.create_subscription(JointState,'/joint_states',got.append,1)
while not got:rclpy.spin_once(n,timeout_sec=.2)
m=got[-1]; print(dict(zip(m.name,m.position)))
r=GetPositionFK.Request(); r.fk_link_names=['panda_hand']; r.robot_state.joint_state.name=m.name[:7]; r.robot_state.joint_state.position=m.position[:7]
c=n.create_client(GetPositionFK,'/compute_fk');c.wait_for_service();f=c.call_async(r);rclpy.spin_until_future_complete(n,f); print(f.result().pose_stamped)
PY
python3 state.py && python3 tools/action/ik_move.py -0.2106 0.1867 1.09 1 0 0 0 3"

# openrua op 10
/bin/bash -lc 'python3 tools/perception/cam_snap.py sideview /workspace/side.png'

# openrua op 11
/bin/bash -lc "python3 - <<'PY'
p='tools/action/ik_move.py'
s=open(p).read().replace('req.ik_request.group_name = planning.get(\"group\", \"panda_arm\")','req.ik_request.group_name = planning.get(\"group\", \"panda_arm\")\\n    req.ik_request.ik_link_name = \"panda_hand\"')
open(p,'w').write(s)
PY"

# openrua op 12
/bin/bash -lc 'python3 state.py && python3 tools/action/ik_move.py -0.2106 0.1867 1.035 1 0 0 0 2'

# openrua op 13
/bin/bash -lc 'python3 tools/action/ik_move.py -0.2106 0.1867 1.035 1 0 0 0 2 && python3 state.py && python3 tools/perception/cam_snap.py agentview /workspace/agent.png'

# openrua op 14
/bin/bash -lc "cp tools/perception/px2world.py /workspace/measure.py
python3 - <<'PY'
p='/workspace/measure.py';s=open(p).read();s=s.replace('if len(sys.argv) "'!= 4:'"', 'if len(sys.argv) < 4:');s=s.replace('cam, u, v = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])','cam, u, v = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])');a=s.index('    p = T @ p_cam');b=s.index('    rclpy.shutdown()',a);s=s[:a]+'''    for i in range(2,len(sys.argv),2):
        u,v=map(int,sys.argv[i:i+2]);z=struct.unpack_from(\"<f\", depth.data,(v*depth.width+u)*4)[0]
        p=T @ np.array([(u-cx)*z/fx,(v-cy)*z/fy,z,1.])
        print(u,v,':',*[round(float(k),4) for k in p[:3]])
'''+s[b:];open(p,'w').write(s)
PY"

# openrua op 15
/bin/bash -lc 'python3 tools/action/ik_move.py -0.2106 0.1867 1.035 1 0 0 0 3 && python3 tools/action/gripper_cmd.py 0.0 && python3 state.py'

# openrua op 16
/bin/bash -lc 'python3 tools/action/ik_move.py -0.2106 0.1867 1.035 0.7071068 0.7071068 0 0 5 && python3 state.py && python3 tools/perception/cam_snap.py agentview /workspace/agent.png'

# openrua op 17
/bin/bash -lc 'python3 measure.py birdview 254 281 300 281 308 281 319 281 373 281'

# openrua op 18
/bin/bash -lc 'python3 tools/action/gripper_cmd.py 0.04 && python3 tools/action/ik_move.py -0.2106 0.1867 1.22 0.8585 0.5128 0 0 3 && python3 tools/perception/cam_snap.py birdview /workspace/bird.png && python3 state.py'

# openrua op 19
/bin/bash -lc 'python3 tools/action/ik_move.py -0.055 -0.0708 1.15 0.7071068 0.7071068 0 0 4 && python3 state.py'

# openrua op 20
/bin/bash -lc 'python3 tools/action/ik_move.py -0.055 -0.0708 1.037 0.7071068 0.7071068 0 0 3 && python3 state.py && python3 tools/perception/cam_snap.py agentview /workspace/agent.png'

# openrua op 21
/bin/bash -lc 'python3 tools/action/gripper_cmd.py 0.0 && python3 state.py'

# openrua op 22
/bin/bash -lc 'python3 tools/action/ik_move.py -0.055 -0.0708 1.23 0.7071068 0.7071068 0 0 3 && python3 tools/perception/cam_snap.py agentview /workspace/agent.png && python3 state.py'

# openrua op 23
/bin/bash -lc 'python3 tools/perception/cam_snap.py birdview /workspace/bird.png && python3 tools/perception/cam_snap.py sideview /workspace/side.png'

# openrua op 24
/bin/bash -lc 'python3 measure.py birdview 249 285 373 281 && python3 tools/action/ik_move.py -0.055 0.356 1.25 0.7071068 0.7071068 0 0 5'

# openrua op 25
/bin/bash -lc 'python3 state.py && python3 tools/perception/cam_snap.py birdview /workspace/bird.png && python3 tools/perception/cam_snap.py agentview /workspace/agent.png'

# openrua op 26
/bin/bash -lc 'python3 measure.py birdview 378 285 && python3 tools/action/ik_move.py -0.055 0.356 1.085 0.7071068 0.7071068 0 0 3'

# openrua op 27
/bin/bash -lc 'python3 tools/perception/cam_snap.py sideview /workspace/side.png && python3 state.py && ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once'

# openrua op 28
/bin/bash -lc 'python3 tools/action/gripper_cmd.py 0.04 && python3 tools/action/ik_move.py -0.055 0.356 1.28 0.7071068 0.7071068 0 0 3 && python3 tools/perception/cam_snap.py agentview /workspace/final_agent.png && python3 tools/perception/cam_snap.py birdview /workspace/final_bird.png && python3 tools/perception/cam_snap.py sideview /workspace/final_side.png && python3 state.py'
