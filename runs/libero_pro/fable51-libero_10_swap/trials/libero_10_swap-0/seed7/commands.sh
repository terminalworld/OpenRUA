#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -v parameter | head -40; echo ----; timeout 20 ros2 node list

# openrua op 4
timeout 60 ros2 topic echo /joint_states --once; for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ls *.png

# openrua op 5
python3 -c "
import cv2
img=cv2.imread('agentview.png')
crop=img[180:420,90:400]; crop=cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop_agent.png',crop)
img=cv2.imread('robot0_eye_in_hand.png')
crop=img[150:300,480:640]; crop=cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop_eih.png',crop)
"

# openrua op 6
python3 -c "
import cv2
img=cv2.imread('birdview.png')
crop=img[200:330,230:430]; crop=cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC); cv2.imwrite('crop_bird.png',crop)
"

# openrua op 7
timeout 60 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame_id" | head -40; echo ----; timeout 30 ros2 topic echo /birdview/color/camera_info --once

# openrua op 8
cat > tfdump.py <<'EOF'
import rclpy, time
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); node=rclpy.create_node('tfdump')
frames={}
def cb(msg):
    for t in msg.transforms:
        frames[(t.header.frame_id,t.child_frame_id)]=t.transform
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
node.create_subscription(TFMessage,'/tf_static',cb,qos)
node.create_subscription(TFMessage,'/tf',cb,10)
t0=time.time()
while time.time()-t0<5: rclpy.spin_once(node,timeout_sec=0.2)
for k,v in sorted(frames.items()):
    print(k, f"t=({v.translation.x:.4f},{v.translation.y:.4f},{v.translation.z:.4f}) q=({v.rotation.x:.4f},{v.rotation.y:.4f},{v.rotation.z:.4f},{v.rotation.w:.4f})")
EOF
timeout 60 python3 tfdump.py

# openrua op 9
cat > cloud.py <<'EOF'
"""cloud.py <camera> : save world-frame point cloud (HxWx3) as <camera>_xyz.npy plus color png"""
import sys, numpy as np, rclpy, time
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from cv_bridge import CvBridge
import cv2
cam=sys.argv[1]
rclpy.init(); node=rclpy.create_node('cloud')
got={}
node.create_subscription(Image,f'/{cam}/depth/image_raw',lambda m:got.setdefault('d',m),1)
node.create_subscription(Image,f'/{cam}/color/image_raw',lambda m:got.setdefault('c',m),1)
node.create_subscription(CameraInfo,f'/{cam}/color/camera_info',lambda m:got.setdefault('i',m),1)
qos=QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
tfs={}
def cb(msg):
    for t in msg.transforms: tfs[t.child_frame_id]=t.transform
node.create_subscription(TFMessage,'/tf_static',cb,qos)
node.create_subscription(TFMessage,'/tf',cb,10)
t0=time.time()
while (len(got)<3 or f'{cam}_optical_frame' not in tfs) and time.time()-t0<30: rclpy.spin_once(node,timeout_sec=0.2)
b=CvBridge()
depth=b.imgmsg_to_cv2(got['d'],'passthrough').astype(np.float64)
color=b.imgmsg_to_cv2(got['c'],'bgr8')
k=got['i'].k; fx,fy,cx,cy=k[0],k[4],k[2],k[5]
H,W=depth.shape
u,v=np.meshgrid(np.arange(W),np.arange(H))
X=(u-cx)*depth/fx; Y=(v-cy)*depth/fy; Z=depth
t=tfs[f'{cam}_optical_frame']; q=t.rotation; x,y,z,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
T=np.array([t.translation.x,t.translation.y,t.translation.z])
P=np.stack([X,Y,Z],-1)@R.T+T
np.save(f'{cam}_xyz.npy',P); cv2.imwrite(f'{cam}.png',color)
print('saved',cam, 'depth range',np.nanmin(depth),np.nanmax(depth))
EOF
timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy')
for name,(u,v) in {'alphabet':(285,230),'tomato':(266,284),'ketchup':(288,260),'basket':(384,290),'table':(200,200)}.items():
    print(name, P[v,u].round(4))
"

# openrua op 10
python3 -c "
import numpy as np, cv2
P=np.load('birdview_xyz.npy')
h=P[...,2]-0.425
mask=(h>0.02)&np.isfinite(h)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<10: continue
    m=lab==i
    xyz=P[m]
    print(i,'px',stats[i,:4],'n',stats[i,4],'x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].max()), 'centroid', xyz[:,:2].mean(0).round(3))
"

# openrua op 11
python3 -c "
import numpy as np
P=np.load('birdview_xyz.npy')
z=P[...,2]
sel=z[(z>0.3)&(z<0.6)]
hist,edges=np.histogram(sel,bins=np.arange(0.40,0.56,0.005))
for h,e in zip(hist,edges): print('%.3f %d'%(e,h))
# basket interior
m=(P[...,0]>-0.05)&(P[...,0]<0.05)&(P[...,1]>0.2)&(P[...,1]<0.32)
print('basket interior z min/median', z[m].min(), np.median(z[m]))
"

# openrua op 12
timeout 20 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30; timeout 20 ros2 interface show control_msgs/action/GripperCommand

# openrua op 13
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
"""Shared robot helper: one node, reusable IK/FK/FJT/gripper clients.

Coordinates given to move_* are WORLD frame; converted to the arm base
(planning) frame using the static world->panda_link0 transform.
"""
import time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])  # from /tf_static world->panda_link0
TCP = float(M["hand"]["tcp_offset_m"])
# hand pointing straight down, fingers closing along world y
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_down_yaw(yaw):
    """hand z down, then rotated about world z by yaw (rad)."""
    c, s = np.cos(yaw / 2), np.sin(yaw / 2)
    return quat_mul((0.0, 0.0, s, c), Q_DOWN)


class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("rob")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"
        assert self.fjt.wait_for_server(10), "no FJT"
        assert self.grip.wait_for_server(10), "no gripper"

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 20:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def hand_pose(self, q=None):
        """world-frame (xyz, quat) of panda_hand for arm config q (default current)."""
        if q is None:
            q = self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return xyz, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp_pose(self):
        xyz, q = self.hand_pose()
        return xyz + TCP * quat_to_R(*q)[:, 2], q

    # ---------- IK ----------
    def solve_ik(self, xyz_world, quat, at_tcp=True, seed=None, timeout=60):
        xyz = np.array(xyz_world, float)
        if at_tcp:
            xyz = xyz - TCP * quat_to_R(*quat)[:, 2]
        xyz = xyz - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            raise RuntimeError(f"IK failed code={res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if waypoints:
            for wq, wt in waypoints:
                pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
                pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        cur = np.array(self.arm_q())
        err = np.abs(cur - np.array(q)).max()
        print(f"  fjt code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, xyz, quat=Q_DOWN, seconds=3.0, seed=None):
        q = self.solve_ik(xyz, quat, at_tcp=True, seed=seed)
        code, err = self.move_q(q, seconds)
        p, _ = self.tcp_pose()
        print(f"  tcp now {p.round(4)} target {np.array(xyz).round(4)} d={np.linalg.norm(p-xyz):.4f}")
        return p

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        g = self.finger_gap()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} fingers={g}")
        return g

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
OPENRUA_EOF

# openrua op 14
timeout 120 python3 -c "
from rob import *
r=Robot()
print('arm q', np.round(r.arm_q(),4))
p,q=r.hand_pose(); print('hand world', p.round(4), np.round(q,4))
p,q=r.tcp_pose(); print('tcp world', p.round(4))
print('fingers', r.finger_gap())
r.close()
"

# openrua op 15
timeout 120 python3 -c "
from rob import *
r=Robot()
import rob
rob.BASE_IN_WORLD=np.zeros(3)  # test: treat model frame as world
q0=r.arm_q()
p,q=r.hand_pose(); print('hand (model frame)', p.round(4), np.round(q,4))
req_hdr=None
try:
    s=r.solve_ik(p,q,at_tcp=False); print('IK model=world -> dq', np.round(np.array(s)-q0,4))
except Exception as e: print('IK err',e)
rob.BASE_IN_WORLD=np.array([-0.51,0,0.42])
try:
    s=r.solve_ik(p+rob.BASE_IN_WORLD,q,at_tcp=False); print('IK model=base -> dq', np.round(np.array(s)-q0,4))
except Exception as e: print('IK err',e)
r.close()
"

# openrua op 16
timeout 120 python3 -c "
from rob import *
import rob
rob.BASE_IN_WORLD=np.zeros(3)
r=Robot()
q0=r.arm_q()
p,q=r.hand_pose()
s=r.solve_ik(p,q,at_tcp=False)
p2,q2=r.hand_pose(s); print('FK of IK sol', p2.round(4), np.round(q2,4), 'target', p.round(4), np.round(q,4))
# test IK above alphabet soup, hand down
for z in (0.65,0.55,0.50):
    try:
        s=r.solve_ik([-0.246,-0.151,z],Q_DOWN,at_tcp=True); print(z,'ok', np.round(s,3)); print(' FK tcp', (r.hand_pose(s)[0]+TCP*quat_to_R(*Q_DOWN)[:,2]).round(4))
    except Exception as e: print(z,e)
r.close()
"

# openrua op 17
timeout 120 python3 -c "
from rob import *
import rob
rob.BASE_IN_WORLD=np.zeros(3)
r=Robot()
for z in (0.65,0.55):
    s=r.solve_ik([-0.246,-0.151,z],Q_DOWN,at_tcp=True)
    p,q=r.hand_pose(s); print(z, 'hand', p.round(4), 'quat', np.round(q,4), 'Rz col', quat_to_R(*q)[:,2].round(3))
r.close()
"

# openrua op 18
sed -i 's/^BASE_IN_WORLD = np.array(\[-0.51, 0.0, 0.42\]).*/BASE_IN_WORLD = np.zeros(3)  # MoveIt model frame == world (verified by FK vs camera TF)/' rob.py && grep -n BASE_IN_WORLD rob.py | head -2 && timeout 300 python3 -c "
from rob import *
r=Robot()
r.gripper(0.04)
r.close()
"

# openrua op 19
timeout 900 python3 -u -c "
from rob import *
r=Robot()
r.move_tcp([-0.246,-0.151,0.65], Q_DOWN, 4.0)
r.close()
" 2>&1 | tee step1.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 20
timeout 120 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np, cv2
P=np.load('robot0_eye_in_hand_xyz.npy')
print('cam pixel(320,240)->', P[240,320].round(3))
h=P[...,2]-0.42
mask=((h>0.03)&np.isfinite(h)).astype(np.uint8)
mask[380:]=0
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<50: continue
    xyz=P[lab==i]
    print(i,'px',stats[i,:4],'n',stats[i,4],'x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].max()),'centroid',xyz[:,:2].mean(0).round(3))
"; timeout 60 python3 -c "
from rob import *
r=Robot(); print(np.round(r.arm_q(),3)); p,q=r.hand_pose(); print(p.round(4),np.round(q,4)); r.close()"

# openrua op 21
timeout 900 python3 -u -c "
from rob import *
r=Robot()
q=r.arm_q(); print('before', np.round(q,3))
q[6]=-0.703
r.move_q(q, 3.0)
print('after', np.round(r.arm_q(),3))
r.close()
" 2>&1 | tee step2.log

# openrua op 22
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "        code = res.result().result.error_code\\n        cur = np.array(self.arm_q())\\n        err = np.abs(cur - np.array(q)).max()\\n        print(f\\"  fjt code={code} max_joint_err={err:.4f}\\")\\n        return code, err", "new_string": "        code = res.result().result.error_code\\n        cur = np.array(self.arm_q())\\n        err = np.abs(cur - np.array(q)).max()\\n        print(f\\"  fjt code={code} max_joint_err={err:.4f}\\")\\n        if err > tol and retries > 0:\\n            # joint7 on this machine is slow; re-send with time for the residual\\n            return self.move_q(q, max(2.0, err / 0.15), retries=retries - 1, tol=tol)\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 23
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "    def move_q(self, q, seconds=3.0, waypoints=None):", "new_string": "    def move_q(self, q, seconds=3.0, waypoints=None, retries=4, tol=0.01):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 24
timeout 900 python3 -u -c "
from rob import *
r=Robot()
p,qh=r.hand_pose(); print('hand', p.round(4), np.round(qh,4))
q8=quat_mul(qh,(0,0,0.3827,0.9239))
tgt=np.array([-0.243,-0.150,0.47])
s=r.solve_ik(tgt,q8,at_tcp=True)
p2,q2=r.hand_pose(s); print('IK sol FK hand', p2.round(4), np.round(q2,4), 'tcp', (p2+TCP*quat_to_R(*q2)[:,2]).round(4))
print('dq', np.round(np.array(s)-np.array(r.arm_q()),3))
r.close()
"

# openrua op 25
timeout 1200 python3 -u -c "
from rob import *
r=Robot()
p,qh=r.hand_pose()
q8=quat_mul(qh,(0,0,0.3827,0.9239))
r.move_tcp([-0.243,-0.150,0.56], q8, 3.0)
r.move_tcp([-0.243,-0.150,0.47], q8, 3.0)
r.gripper(0.0)
print('fingers after close', r.finger_gap())
r.close()
" 2>&1 | tee step3.log; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 26
timeout 1500 python3 -u -c "
from rob import *
r=Robot()
p,qh=r.hand_pose()
q8=quat_mul(qh,(0,0,0.3827,0.9239))
r.move_tcp([-0.243,-0.150,0.75], q8, 3.0)
print('fingers', r.finger_gap())
r.move_tcp([0.007,0.265,0.75], q8, 5.0)
print('fingers', r.finger_gap())
r.move_tcp([0.007,0.265,0.60], q8, 3.0)
r.gripper(0.04)
r.move_tcp([0.007,0.265,0.78], q8, 3.0)
r.close()
" 2>&1 | tee step4.log; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 27
timeout 1500 python3 -u -c "
from rob import *
r=Robot()
p,qh=r.hand_pose()
q8=quat_mul(qh,(0,0,0.3827,0.9239))
r.move_tcp([-0.013,-0.234,0.66], q8, 4.0)
r.close()
" 2>&1 | tee step5.log; timeout 120 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np, cv2
P=np.load('robot0_eye_in_hand_xyz.npy')
h=P[...,2]-0.42
mask=((h>0.03)&np.isfinite(h)).astype(np.uint8)
mask[380:]=0
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<50: continue
    xyz=P[lab==i]
    print(i,'px',stats[i,:4],'n',stats[i,4],'x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].max()),'centroid',xyz[:,:2].mean(0).round(3))
"

# openrua op 28
timeout 1500 python3 -u -c "
from rob import *
r=Robot()
p,qh=r.hand_pose()
q8=quat_mul(qh,(0,0,0.3827,0.9239))
r.move_tcp([-0.010,-0.233,0.56], q8, 3.0)
r.move_tcp([-0.010,-0.233,0.47], q8, 3.0)
r.gripper(0.0)
r.move_tcp([-0.010,-0.233,0.75], q8, 3.0)
print('fingers after lift', r.finger_gap())
r.close()
" 2>&1 | tee step6.log; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 29
timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np, cv2
P=np.load('birdview_xyz.npy')
m=(P[...,0]>-0.09)&(P[...,0]<0.10)&(P[...,1]>0.16)&(P[...,1]<0.36)
z=P[...,2]
sub=m&(z>0.50)&(z<0.60)
xyz=P[sub]
print('points in basket 0.50<z<0.60:',len(xyz))
if len(xyz): print('x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f centroid %s'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].max(), xyz[:,:2].mean(0).round(3)))
"

# openrua op 30
python3 -c "
import numpy as np, cv2
P=np.load('birdview_xyz.npy')
m=(P[...,0]>-0.06)&(P[...,0]<0.07)&(P[...,1]>0.19)&(P[...,1]<0.33)
z=P[...,2]
for lo,hi in [(0.49,0.51),(0.51,0.53),(0.53,0.55),(0.55,0.57),(0.57,0.59)]:
    sub=m&(z>=lo)&(z<hi); xyz=P[sub]
    if len(xyz): print(lo,hi,len(xyz),'x[%.3f,%.3f] y[%.3f,%.3f]'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max()), xyz[:,:2].mean(0).round(3))
img=cv2.imread('birdview.png'); crop=img[250:330,340:420]; cv2.imwrite('crop_basket.png',cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"

# openrua op 31
timeout 1500 python3 -u -c "
from rob import *
r=Robot()
p,qh=r.hand_pose()
q8=quat_mul(qh,(0,0,0.3827,0.9239))
r.move_tcp([0.040,0.295,0.75], q8, 5.0)
print('fingers', r.finger_gap())
r.move_tcp([0.040,0.295,0.645], q8, 3.0)
r.gripper(0.04)
r.move_tcp([0.040,0.295,0.80], q8, 3.0)
r.close()
" 2>&1 | tee step7.log; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 120 python3 cloud.py birdview

# openrua op 32
python3 -c "
import numpy as np, cv2
P=np.load('birdview_xyz.npy')
m=(P[...,0]>-0.09)&(P[...,0]<0.10)&(P[...,1]>0.16)&(P[...,1]<0.36)
z=P[...,2]
sub=m&(z>0.50)
xyz=P[sub]
print('all pts above 0.50 in basket footprint: x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].max()))
sub=m&(z>0.64); xyz=P[sub]
print('pts above rim (0.64):',len(xyz), 'x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].max()) if len(xyz) else '')
img=cv2.imread('birdview.png'); crop=img[250:330,340:420]; cv2.imwrite('crop_basket.png',cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"

# openrua op 33
timeout 1500 python3 -u -c "
from rob import *
r=Robot()
p,qh=r.hand_pose()
q8=quat_mul(qh,(0,0,0.3827,0.9239))
r.move_tcp([-0.15,0.05,0.80], q8, 4.0)
r.close()
" 2>&1 | tee step8.log; timeout 120 python3 cloud.py birdview && python3 -c "
import numpy as np, cv2
P=np.load('birdview_xyz.npy')
m=(P[...,0]>-0.09)&(P[...,0]<0.10)&(P[...,1]>0.16)&(P[...,1]<0.36)
z=P[...,2]
sub=m&(z>0.50); xyz=P[sub]
print('pts above 0.50: x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].max()))
sub=m&(z>0.635); xyz=P[sub]
print('pts above rim (0.635):',len(xyz), 'x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(xyz[:,0].min(),xyz[:,0].max(),xyz[:,1].min(),xyz[:,1].max(),xyz[:,2].max()) if len(xyz) else '')
img=cv2.imread('birdview.png'); crop=img[250:330,340:420]; cv2.imwrite('crop_basket.png',cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py sideview
