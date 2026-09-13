#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
ros2 topic list 2>&1; echo ---; ros2 action list 2>&1; echo ---; ros2 service list 2>&1 | head -40; echo ---; ros2 node list 2>&1

# openrua op 3
for c in agentview birdview frontview sideview galleryview paperview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls -la *.png; ros2 topic echo /joint_states --once 2>&1 | head -40

# openrua op 4
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12; echo ---; ros2 topic echo /birdview/color/camera_info --once 2>&1 | grep -A12 '^k:'

# openrua op 5
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
fx=579.4112549695428; cx=320; cy=240
def w(u,v):
    z=d[v,u]; X=(u-cx)*z/fx; Y=(v-cy)*z/fx
    # cam->world: wx = Y + (-0.2)? Matrix: world = R*p + t ; R=[[0,1,0],[1,0,0],[0,0,-1]]
    return (Y-0.2, X+0.0, 3.0-z)
for name,(u,v) in {'table':(320,400),'book':(315,265),'mug':(360,240),'caddyL':(240,180),'caddyR':(320,180),'caddy_top_edge':(280,160),'caddy_bot_edge':(280,205)}.items():
    print(name, (u,v), ['%.3f'%x for x in w(u,v)])
"


# openrua op 6
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
X=(us-cx)*d/fx; Y=(vs-cy)*d/fx
wx=Y-0.2; wy=X; wz=3.0-d
# print a height map of the caddy/book region with coarse sampling
np.set_printoptions(linewidth=250)
print('rows v=150..280 step 5, cols u=200..380 step 5; value = (wz-0.88)*100 cm')
print('     '+' '.join('%3d'%u for u in range(200,381,5)))
for v in range(150,281,5):
    print('%3d  '%v + ' '.join('%3d'%round((wz[v,u]-0.88)*100) for u in range(200,381,5)))
"


# openrua op 7
find / -iname "*caddy*" 2>/dev/null | head; find / -iname "*libero*" -maxdepth 4 2>/dev/null | head; ls /opt /home 2>/dev/null

# openrua op 8
ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 1023}}" 2>&1 | head -60; echo ---; ros2 topic echo /pipeline_state --once 2>&1 | head; ls /workspace/.ros/log; grep -il "caddy\|book" /workspace/.ros/log/* 2>/dev/null

# openrua op 9
ls -la /workspace/.ros/log; grep -il "caddy\|book" /workspace/.ros/log/* 2>/dev/null; tail -5 /workspace/.ros/log/python3_63_1789245694503.log

# openrua op 10
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
m=(wz>0.95)&(vs>240)&(vs<300)&(us>290)&(us<350)
print('book pixels',m.sum())
print('wx range',wx[m].min(),wx[m].max(),'wy range',wy[m].min(),wy[m].max(),'wz',wz[m].min(),wz[m].max())
print('centroid',wx[m].mean(),wy[m].mean())
# PCA for orientation
P=np.stack([wx[m],wy[m]],1); P=P-P.mean(0); w,v=np.linalg.eigh(P.T@P); print('axes',v, 'ext',np.sqrt(w/m.sum())*2*np.sqrt(3))
# project along axes
pr=P@v; print('extent along minor',pr[:,0].min(),pr[:,0].max(),'major',pr[:,1].min(),pr[:,1].max())
print('major axis angle deg', np.degrees(np.arctan2(v[1,1],v[0,1])))
"


# openrua op 11
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
for thr in (1.0,1.05):
  m=(wz>thr)&(vs>240)&(vs<300)&(us>290)&(us<345)
  print('thr',thr,'book pixels',m.sum())
  print(' wx range',wx[m].min(),wx[m].max(),'wy range',wy[m].min(),wy[m].max(),'wz',wz[m].min(),wz[m].max())
  print(' centroid',wx[m].mean(),wy[m].mean())
  P=np.stack([wx[m],wy[m]],1); P=P-P.mean(0); w,v=np.linalg.eigh(P.T@P)
  pr=P@v; print(' extent along minor',pr[:,0].min(),pr[:,0].max(),'major',pr[:,1].min(),pr[:,1].max())
  print(' major axis angle deg', np.degrees(np.arctan2(v[1,1],v[0,1])))
  print(' pixel bbox u',us[m].min(),us[m].max(),'v',vs[m].min(),vs[m].max())
"


# openrua op 12
python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[140:300,190:390]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png'); crop=im[130:400,100:470]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_LINEAR))
"

# openrua op 13
python3 -c "
from scipy.spatial.transform import Rotation as R
import numpy as np
q=(R.from_euler('z',0,degrees=True)*R.from_euler('x',180,degrees=True)).as_quat(); print('yaw0',q)
q=(R.from_euler('z',-17,degrees=True)*R.from_euler('x',180,degrees=True)).as_quat(); print('yaw-17',q)
"


# openrua op 14
time timeout 300 python3 tools/action/ik_move.py 0.40 0.35 0.338 1 0 0 0 3 --at tcp 2>&1 | tail -5

# openrua op 15
time timeout 300 python3 tools/action/ik_move.py 0.40 0.35 0.4414 1 0 0 0 3 2>&1 | tail -5

# openrua op 16
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Helper library: IK / FK / trajectory / gripper / servo for this Panda.

World frame <-> base frame: base (panda_link0) sits at world (-0.75, 0, 0.912).
All public functions take WORLD coordinates.
"""
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

BASE_W = np.array([-0.75, 0.0, 0.912])
TCP = 0.1034

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, msg):
        self._js["m"] = msg

    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in JOINTS]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    # ---- kinematics -------------------------------------------------
    @staticmethod
    def quat_down(yaw_deg):
        """Hand pointing straight down, fingers closing along direction
        (sin yaw, -cos yaw) in world; yaw=0 -> fingers close along world y."""
        return (R.from_euler("z", yaw_deg, degrees=True)
                * R.from_euler("x", 180, degrees=True)).as_quat()

    def hand_pose_world(self):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        seed = JointState()
        for n, p in zip(JOINTS, self.arm_q()):
            seed.name.append(n); seed.position.append(p)
        req.robot_state.joint_state = seed
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_W
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q

    def tcp_world(self):
        pos, q = self.hand_pose_world()
        return pos + R.from_quat(q).apply([0, 0, TCP]), q

    def solve_ik(self, tcp_w, quat, seed_q=None):
        """tcp_w: fingertip-centre target in world. Returns joint list or None."""
        hand_w = np.asarray(tcp_w, float) - R.from_quat(quat).apply([0, 0, TCP])
        hand_b = hand_w - BASE_W
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_b)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        seed = JointState()
        for n, v in zip(JOINTS, seed_q or self.arm_q()):
            seed.name.append(n); seed.position.append(float(v))
        req.ik_request.robot_state.joint_state = seed
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print(f"IK failed code={None if res is None else res.error_code.val}", flush=True)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    # ---- motion -----------------------------------------------------
    def move_joints(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        pts = []
        if via:
            n = len(via) + 1
            for i, vq in enumerate(via, 1):
                t = seconds * i / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in vq])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"traj done code={code} max_joint_err={err:.4f}", flush=True)
        return code

    def move_tcp(self, tcp_w, quat, seconds=3.0, seed_q=None):
        q = self.solve_ik(tcp_w, quat, seed_q)
        if q is None:
            return None
        self.move_joints(q, seconds)
        pos, _ = self.tcp_world()
        print(f"tcp now {pos.round(4)} target {np.round(tcp_w,4)}", flush=True)
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={self.finger_gap()}", flush=True)
        return r

    def servo(self, vx=0.0, vy=0.0, vz=0.0, ticks=20):
        msg = TwistStamped(); msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(ticks):
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def close(self):
        self.node.destroy_node(); rclpy.shutdown()
OPENRUA_EOF

# openrua op 17
timeout 200 python3 -u -c "
from arm import *
a=Arm()
print('q',np.round(a.arm_q(),3))
print('hand',a.hand_pose_world()); print('tcp',a.tcp_world())
for yaw in (0,45,-45,90):
    for pos in ([-0.35,0.35,1.25],[-0.10,-0.01,1.20],[-0.44,-0.14,1.25]):
        q=a.solve_ik(pos,Arm.quat_down(yaw)); print(yaw,pos,None if q is None else np.round(q,2))
a.close()
"

# openrua op 18
timeout 200 python3 -u -c "
from arm import *
a=Arm()
import arm as A
# FK raw
req = GetPositionFK.Request(); req.fk_link_names=['panda_hand','panda_link0']
seed=JointState()
for n,p in zip(JOINTS,a.arm_q()): seed.name.append(n); seed.position.append(p)
req.robot_state.joint_state=seed
fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=30)
for ps in fut.result().pose_stamped: print(ps.header.frame_id, ps.pose.position)
# try IK in world coords
A.BASE_W[:]=0
q=a.solve_ik([-0.35,0.35,1.25],Arm.quat_down(0)); print(np.round(q,3) if q is not None else None)
if q is not None:
    seed=JointState()
    for n,p in zip(JOINTS,q): seed.name.append(n); seed.position.append(p)
    req.robot_state.joint_state=seed; fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=30)
    for ps in fut.result().pose_stamped: print(ps.header.frame_id, ps.pose.position)
a.close()
"

# openrua op 19
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE_W = np.array([-0.75, 0.0, 0.912])", "new_string": "# Verified: /compute_ik and /compute_fk on this machine use WORLD coordinates\\n# (FK of panda_link0 returns (-0.75, 0, 0.912)); no base offset needed.\\nBASE_W = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 20
timeout 400 python3 -u -c "
from arm import *
a=Arm()
print('tcp before',a.tcp_world()[0].round(3))
a.move_tcp([-0.35,0.35,1.25],Arm.quat_down(0),4)
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 21
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
np.set_printoptions(linewidth=250)
print('cols u=210..360 step 3; value=(wz-0.88)*100 cm')
print('     '+' '.join('%3d'%u for u in range(210,361,3)))
for v in range(152,210,2):
    print('%3d  '%v + ' '.join('%3d'%round((wz[v,u]-0.88)*100) for u in range(210,361,3)))
"


# openrua op 22
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
def ext(m,label):
    print(f'{label}: n={m.sum()} wx[{wx[m].min():.4f},{wx[m].max():.4f}] wy[{wy[m].min():.4f},{wy[m].max():.4f}] wz[{wz[m].min():.3f},{wz[m].max():.3f}]')
reg=(us>=262)&(us<=296)&(vs>=150)&(vs<=200)
ext(reg&(wz>1.03)&(vs<162),'back wall top (middle col)')
ext(reg&(wz>0.97)&(wz<1.01)&(vs>=175)&(vs<=183),'middle divider top')
ext(reg&(wz<0.92)&(wz>0.885)&(vs<178),'back compartment floor')
ext(reg&(wz<0.92)&(wz>0.885)&(vs>178),'front compartment floor')
col=(vs>=164)&(vs<=176)
ext(col&(us>=255)&(us<=263)&(wz>0.93)&(wz<1.0),'left ledge/divider (u~258-261)')
ext(col&(us>=295)&(us<=302)&(wz>1.03),'right tall divider top')
# book
m=(wz>1.04)&(vs>240)&(vs<300)&(us>295)&(us<345)
ext(m,'book top')
P=np.stack([wx[m],wy[m]],1); c=P.mean(0); Q=P-c; w,v=np.linalg.eigh(Q.T@Q); pr=Q@v
print('book centre',c.round(4),'minor ext',pr[:,0].min().round(4),pr[:,0].max().round(4),'major ext',pr[:,1].min().round(4),pr[:,1].max().round(4),'major angle',np.degrees(np.arctan2(v[1,1],v[0,1])).round(1))
"


# openrua op 23
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
m=(wz>0.95)&(vs>205)&(vs<400)&(us>150)&(us<500)
print('pixels above 0.95 in table area:',m.sum(),'u',us[m].min(),us[m].max(),'v',vs[m].min(),vs[m].max())
for thr in (1.0,1.04):
  mm=(wz>thr)&(vs>205)&(vs<400)&(us>150)&(us<500)&(us<340)
  print(thr, mm.sum(), 'u',us[mm].min(),us[mm].max(),'v',vs[mm].min(),vs[mm].max(), 'wz max',wz[mm].max())
im=cv2.imread('birdview.png'); crop=im[140:300,190:390]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"


# openrua op 24
timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[130:400,100:470]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2,fy=2,interpolation=cv2.INTER_LINEAR))"

# openrua op 25
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
for thr in (0.95,0.99,1.01):
  m=(wz>thr)&(vs>240)&(vs<300)&(us>295)&(us<340)
  P=np.stack([wx[m],wy[m]],1); c=P.mean(0); Q=P-c; w,v=np.linalg.eigh(Q.T@Q); pr=Q@v
  print(thr,'n',m.sum(),'wz',wz[m].min().round(3),wz[m].max().round(3),'centre',c.round(4),'minor',pr[:,0].min().round(4),pr[:,0].max().round(4),'major',pr[:,1].min().round(4),pr[:,1].max().round(4),'angle',np.degrees(np.arctan2(v[1,1],v[0,1])).round(1))
# table height near book
print('table z near book', wz[300:320,300:340].mean().round(4))
# mug
m=(wz>0.95)&(vs>205)&(vs<300)&(us>340)&(us<400); print('mug', wx[m].mean().round(3), wy[m].mean().round(3), wz[m].max().round(3))
"


# openrua op 26
timeout 60 python3 -c "
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('rd')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data); print(len(got[0].data))
"; grep -n -A12 'joint name="panda_finger_joint1"\|link name="panda_leftfinger"\|joint name="panda_hand_joint"\|joint name="panda_hand_tcp_joint"\|link name="panda_hand"' robot.urdf | head -120

# openrua op 27
time timeout 600 python3 -u -c "
from arm import *
a=Arm()
a.gripper(0.04)
q=Arm.quat_down(-17)
print('pregrasp'); a.move_tcp([-0.106,-0.011,1.15],q,4)
print('descend'); a.move_tcp([-0.106,-0.011,1.006],q,2.5)
print('fingers',a.finger_gap())
a.close()
"

# openrua op 28
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; python3 -c "
import cv2
im=cv2.imread('agentview.png'); crop=im[100:400,150:500]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2,fy=2))"

# openrua op 29
timeout 300 python3 -u -c "
from arm import *
a=Arm()
a.gripper(0.0)
print('fingers',a.finger_gap())
a.close()
"

# openrua op 30
timeout 300 python3 -u -c "
from arm import *
a=Arm()
print('fingers',a.finger_gap())
q=Arm.quat_down(-17)
a.move_tcp([-0.106,-0.011,1.10],q,2.5)
print('fingers',a.finger_gap())
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 31
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null; ros2 topic echo /joint_states --once 2>/dev/null | grep -A9 position | tail -2; python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
# region around book position: anything between 1.08 and 1.14 (book top) 
m=(wz>1.05)&(wz<1.16)&(vs>220)&(vs<320)&(us>270)&(us<370)
print('n',m.sum())
if m.sum(): print('wx',wx[m].min().round(3),wx[m].max().round(3),'wy',wy[m].min().round(3),wy[m].max().round(3),'wz',wz[m].min().round(3),wz[m].max().round(3))
m2=(wz>0.9)&(wz<1.05)&(vs>220)&(vs<320)&(us>270)&(us<340); print('low stuff n',m2.sum())
"

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview >/dev/null; python3 -c "
import cv2
im=cv2.imread('sideview.png'); crop=im[150:350,250:500]; cv2.imwrite('side_crop.png', cv2.resize(crop,None,fx=3,fy=3))
im=cv2.imread('frontview.png'); crop=im[200:400,200:450]; cv2.imwrite('front_crop.png', cv2.resize(crop,None,fx=3,fy=3))"

# openrua op 33
timeout 600 python3 -u -c "
from arm import *
a=Arm()
a.move_tcp([-0.106,-0.011,1.20],Arm.quat_down(-17),2.5)
print('fingers',a.finger_gap())
r=a.move_tcp([-0.439,-0.144,1.20],Arm.quat_down(-90),4)
print('q',np.round(a.arm_q(),3))
print('fingers',a.finger_gap())
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 34
timeout 600 python3 -u -c "
from arm import *
a=Arm()
for i in range(3):
    r=a.move_tcp([-0.439,-0.144,1.20],Arm.quat_down(-90),4)
    pos,_=a.tcp_world()
    if np.abs(pos-np.array([-0.439,-0.144,1.20])).max()<0.004: break
print('q',np.round(a.arm_q(),3)); print('fingers',a.finger_gap())
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 35
timeout 100 python3 -u -c "
from arm import *
a=Arm()
pos,q=a.hand_pose_world(); print('hand',pos.round(4),'quat',q.round(4))
print('desired',Arm.quat_down(-90).round(4))
print('hand y axis in world',R.from_quat(q).apply([0,1,0]).round(3),'hand z',R.from_quat(q).apply([0,0,1]).round(3))
a.close()
"; python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
np.set_printoptions(linewidth=250)
print('     '+' '.join('%3d'%u for u in range(230,330,3)))
for v in range(140,200,2):
    print('%3d  '%v + ' '.join('%3d'%round((wz[v,u]-0.88)*100) for u in range(230,330,3)))
im=cv2.imread('birdview.png'); crop=im[130:230,200:360]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
"

# openrua op 36
timeout 300 python3 -u -c "
from arm import *
a=Arm()
def yaw_now():
    pos,q=a.hand_pose_world(); y=R.from_quat(q).apply([0,1,0])
    # fingers close along hand y; quat_down(theta) gives hand y = (sin th, -cos th)
    return np.degrees(np.arctan2(y[0], -y[1])), pos
for it in range(3):
    yaw,pos=yaw_now(); print('yaw',round(yaw,2),'pos',pos.round(4))
    err=yaw-(-90)
    if abs(err)<1: break
    q=a.arm_q(); q[6]+=np.radians(err); print('new q7',round(q[6],3))
    a.move_joints(q,2.0)
yaw,pos=yaw_now(); print('final yaw',round(yaw,2),'tcp',a.tcp_world()[0].round(4),'fingers',a.finger_gap())
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null

# openrua op 37
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
np.set_printoptions(linewidth=250)
print('     '+' '.join('%3d'%u for u in range(240,320,2)))
for v in range(150,200,2):
    print('%3d  '%v + ' '.join('%3d'%round((wz[v,u]-0.88)*100) for u in range(240,320,2)))
# book top candidates: z ~ 1.19..1.23
m=(wz>1.17)&(wz<1.24)&(vs>140)&(vs<200)&(us>240)&(us<320)
print('n',m.sum())
if m.sum(): print('book top wx',wx[m].min().round(4),wx[m].max().round(4),'wy',wy[m].min().round(4),wy[m].max().round(4),'wz',wz[m].min().round(3),wz[m].max().round(3),'centre',wx[m].mean().round(4),wy[m].mean().round(4))
im=cv2.imread('birdview.png'); crop=im[130:230,200:360]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
"

# openrua op 38
timeout 300 python3 -u -c "
from arm import *
a=Arm()
def fk_q(q):
    req = GetPositionFK.Request(); req.fk_link_names=['panda_hand']
    seed=JointState()
    for n,p in zip(JOINTS,q): seed.name.append(n); seed.position.append(float(p))
    req.robot_state.joint_state=seed
    fut=a.fk.call_async(req); rclpy.spin_until_future_complete(a.node,fut,timeout_sec=30)
    p=fut.result().pose_stamped[0].pose
    pos=np.array([p.position.x,p.position.y,p.position.z]); quat=np.array([p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w])
    y=R.from_quat(quat).apply([0,1,0]); z=R.from_quat(quat).apply([0,0,1])
    return pos, np.degrees(np.arctan2(y[0],-y[1])), z
target=[-0.4395,-0.144,1.065]
q=a.solve_ik(target,Arm.quat_down(-90))
pos,yaw,z=fk_q(q); print('ik sol hand',pos.round(4),'yaw',round(yaw,2),'z axis',z.round(3),'q',np.round(q,3))
for it in range(3):
    err=yaw-(-90)
    if abs(err)<0.5: break
    q[6]+=np.radians(err); pos,yaw,z=fk_q(q); print('corrected yaw',round(yaw,2),'q7',round(q[6],3))
a.move_joints(q,3.0)
tcp,_=a.tcp_world(); pos,yaw,z=fk_q(a.arm_q()); print('tcp',tcp.round(4),'yaw',round(yaw,2),'fingers',a.finger_gap())
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null

# openrua op 39
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
m=(wz>1.05)&(wz<1.10)&(vs>140)&(vs<200)&(us>240)&(us<320)
print('n',m.sum())
if m.sum(): print('book top wx',wx[m].min().round(4),wx[m].max().round(4),'wy',wy[m].min().round(4),wy[m].max().round(4),'wz',wz[m].min().round(3),wz[m].max().round(3))
im=cv2.imread('sideview.png'); crop=im[150:350,300:550]; cv2.imwrite('side_crop.png', cv2.resize(crop,None,fx=3,fy=3))
"

# openrua op 40
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
m=(wz>1.062)&(wz<1.10)&(vs>140)&(vs<200)&(us>240)&(us<320)
print('n',m.sum())
if m.sum(): print('book top wx',wx[m].min().round(4),wx[m].max().round(4),'wy',wy[m].min().round(4),wy[m].max().round(4),'wz',wz[m].min().round(3),wz[m].max().round(3))
np.set_printoptions(linewidth=250)
print('     '+' '.join('%3d'%u for u in range(250,310,2)))
for v in range(156,186,2):
    print('%3d  '%v + ' '.join('%3d'%round((wz[v,u]-0.88)*100) for u in range(250,310,2)))
"

# openrua op 41
timeout 300 python3 -u -c "
from arm import *
a=Arm()
a.gripper(0.04)
print('fingers',a.finger_gap())
q=a.arm_q()
# retreat straight up: IK for higher tcp, keep q7 as is (yaw unchanged when only z changes ~ small); verify
sol=a.solve_ik([-0.4395,-0.144,1.25],Arm.quat_down(-90),q)
sol[6]=q[6]+(sol[0]-q[0])  # keep hand yaw: compensate base yaw change
a.move_joints(sol,3.0)
print('tcp',a.tcp_world()[0].round(4))
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null && timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null

# openrua op 42
python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
np.set_printoptions(linewidth=250)
print('     '+' '.join('%3d'%u for u in range(250,310,2)))
for v in range(154,200,2):
    print('%3d  '%v + ' '.join('%3d'%round((wz[v,u]-0.88)*100) for u in range(250,310,2)))
m=(wz>1.02)&(wz<1.06)&(vs>160)&(vs<182)&(us>262)&(us<296)
print('book top n',m.sum())
if m.sum(): print('wx',wx[m].min().round(4),wx[m].max().round(4),'wy',wy[m].min().round(4),wy[m].max().round(4),'wz',wz[m].min().round(3),wz[m].max().round(3))
im=cv2.imread('agentview.png'); crop=im[100:400,100:450]; cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=2,fy=2))
im=cv2.imread('sideview.png'); crop=im[150:350,300:550]; cv2.imwrite('side_crop.png', cv2.resize(crop,None,fx=3,fy=3))
im=cv2.imread('birdview.png'); crop=im[130:230,200:360]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
"

# openrua op 43
timeout 400 python3 -u -c "
from arm import *
a=Arm()
a.move_tcp([-0.30,0.30,1.30],Arm.quat_down(0),4)
print('fingers',a.finger_gap())
a.close()
" && timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png >/dev/null && timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy')
fx=579.4112549695428; cx=320; cy=240
vs,us=np.mgrid[0:480,0:640]
wz=3.0-d; wx=(vs-cy)*d/fx-0.2; wy=(us-cx)*d/fx
m=(wz>1.005)&(wz<1.06)&(vs>160)&(vs<182)&(us>262)&(us<296)
print('book top n',m.sum())
if m.sum(): print('wx',wx[m].min().round(4),wx[m].max().round(4),'wy',wy[m].min().round(4),wy[m].max().round(4),'wz',wz[m].min().round(3),wz[m].max().round(3),'centre',wx[m].mean().round(4),wy[m].mean().round(4))
print('slot interior: wx [-0.473,-0.4155] wy [-0.207,-0.081]; floor z 0.90')
# anything left at the original book location?
m2=(wz>0.9)&(vs>240)&(vs<300)&(us>295)&(us<340); print('objects at old book spot:',m2.sum())
im=cv2.imread('birdview.png'); crop=im[130:230,200:360]; cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
"
