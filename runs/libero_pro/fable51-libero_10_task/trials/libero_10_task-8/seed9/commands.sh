#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -100

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la .ros tools

# openrua op 2
ls -la tools/action tools/perception; cat tools/action/* tools/perception/*

# openrua op 3
cat tools/action/fjt_send.py tools/action/gripper_cmd.py; sed -n 1,80p tools/action/ik_move.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | head -50; echo ---; timeout 20 ros2 node list

# openrua op 5
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 60 python3 tools/perception/px2world.py agentview 355 330; timeout 60 python3 tools/perception/px2world.py agentview 480 275; timeout 60 python3 tools/perception/px2world.py agentview 205 290; timeout 60 python3 tools/perception/px2world.py agentview 300 400

# openrua op 7
timeout 20 ros2 topic echo /tf_static --once 2>&1 | head -80; timeout 10 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12

# openrua op 8
cd /workspace; for p in "338 313" "398 277" "263 280" "263 235" "300 350"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 9
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png; timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A12 '^k:'; python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
# print depth around pot A (338,313)
sub=d[295:335, 318:360]
np.set_printoptions(linewidth=250, precision=3)
print(sub[::2, ::2])
"

# openrua op 10
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py /agentview/depth/image_raw agentview_depth.png; timeout 20 ros2 topic echo /agentview/color/camera_info --once | grep -A3 '^k:' | head -3; python3 -c "
import numpy as np
d=np.load('agentview_depth.npy')
fx=579.41
# scan rows across pot A (cols 300-420); pot A roughly at cols 320-400, rows 270-360
for v in range(268, 366, 4):
    row=d[v,300:420]
    # object pixels: depth noticeably less than neighbors' background; find contiguous minimum region
    bg=np.median(np.concatenate([d[v,280:300], d[v,420:440]]))
    mask=row < bg-0.03
    if mask.any():
        idx=np.where(mask)[0]
        u0,u1=idx.min()+300, idx.max()+300
        z=np.median(row[mask])
        print(v, u0, u1, 'width_px', u1-u0+1, 'z', round(float(z),3), 'width_m', round((u1-u0+1)*z/fx,4))
    else: print(v,'none')
"

# openrua op 11
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library for this Panda: joint read, FK, IK, trajectory,
gripper, servo. Poses are in the ARM BASE frame (panda_link0) unless
stated; world = base + (-0.66, 0, 0.912) per tf_static.
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
TCP = float(M["hand"]["tcp_offset_m"])
WORLD_TO_BASE = np.array([-0.66, 0.0, 0.912])  # position of panda_link0 in world


def w2b(p):
    return np.asarray(p, float) - WORLD_TO_BASE


def b2w(p):
    return np.asarray(p, float) + WORLD_TO_BASE


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_about_z(theta):
    return np.array([0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2)])


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 10)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---- sensing ----
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        end = time.time() + 15
        while self._js is None and time.time() < end:
            self.spin(0.1)
        if self._js is None:
            raise RuntimeError("no /joint_states")
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] - d["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        end = time.time() + 5
        while self._wr is None and time.time() < end:
            self.spin(0.1)
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        return np.array([f.x, f.y, f.z])

    def fk(self, q=None, link="panda_hand"):
        """hand pose in base frame: (pos[3], quat[4] xyzw)"""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(5)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y,
                          p.orientation.z, p.orientation.w]))

    def tcp(self, q=None):
        """fingertip-centre position in base frame"""
        p, quat = self.fk(q)
        return p + TCP * quat_to_R(quat)[:, 2], quat

    def ik(self, pos, quat, seed=None, at_tcp=False, timeout=10.0):
        """pos/quat in base frame -> arm joint array, or None"""
        pos = np.asarray(pos, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        if seed is None:
            seed = self.arm_q()
        self.ik_cli.wait_for_service(5)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        (p.orientation.x, p.orientation.y,
         p.orientation.z, p.orientation.w) = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.timeout = Duration(sec=int(timeout),
                                          nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK failed code={r and r.error_code.val}", file=sys.stderr)
            return None
        d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return np.array([d[j] for j in ARM])

    # ---- action ----
    def move(self, q, seconds=3.0, waypoints=None):
        """one FJT goal; waypoints = list of (q, t) before the final q"""
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        for wq, wt in (waypoints or []):
            pt = JointTrajectoryPoint(positions=[float(v) for v in wq])
            pt.time_from_start = Duration(sec=int(wt), nanosec=int((wt % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q)).max()
        print(f"move done code={code} max_joint_err={err:.4f}")
        return code, err

    def gripper(self, width):
        self.grip.wait_for_server(10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        print(f"gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} "
              f"gap={self.finger_gap():.4f}")
        return r

    def servo(self, lin, ang=(0, 0, 0), ticks=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            self.spin(dt)
OPENRUA_EOF

# openrua op 12
cd /workspace; timeout 120 python3 -c "
from rob import *
r=Robot()
q=r.arm_q(); print('q',q)
p,quat=r.fk(q); print('hand base',p,quat,'world',b2w(p))
t,_=r.tcp(q); print('tcp base',t,'world',b2w(t))
print('R hand:\n',quat_to_R(quat).round(3))
print('gap',r.finger_gap(), 'wrench', r.wrench())
"

# openrua op 13
cd /workspace; timeout 120 python3 -c "
from rob import *
r=Robot()
import numpy as np
for q in [[0,-0.785,0,-2.356,0,1.571,0.785],[0,0,0,-1.5,0,1.5,0.785]]:
    p,quat=r.fk(np.array(q)); print(q,'->',p.round(3),quat.round(3))
for link in ['panda_link0','panda_link1','panda_link4','panda_link8','panda_hand']:
    p,quat=r.fk(link=link); print(link,p.round(3),quat.round(3))
"; timeout 20 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0,-0.785,0,-2.356,0,1.571,0.785]}}}" | tail -20

# openrua op 14
cd /workspace; timeout 300 python3 -c "
from rob import *
r=Robot()
q0=r.arm_q()
p,quat=r.fk(q0)
print('hand (world per FK):',p.round(3))
for label,pos in [('world',p),('base',p-WORLD_TO_BASE)]:
    q=r.ik(pos,quat,seed=q0,timeout=3)
    print(label, None if q is None else (q.round(3), 'diff', np.abs(q-q0).max().round(4)))
"

# openrua op 15
cd /workspace; timeout 300 python3 -c "
from rob import *
r=Robot()
q0=r.arm_q()
p,quat=r.fk(q0)
q=np.array([ 0.058, -0.161, -0.058, -2.445, -0.012,  2.227,  0.009])
p2,quat2=r.fk(q)
print('target',p.round(4),quat.round(4)); print('ik sol',p2.round(4),quat2.round(4))
print('R target\n',quat_to_R(quat).round(3)); print('R sol\n',quat_to_R(quat2).round(3))
"

# openrua op 16
cd /workspace; python3 -c "
import numpy as np, cv2
d=np.load('birdview_depth.npy'); fx=579.41; cx,cy=320,240
img=cv2.imread('birdview.png')
# camera pose: top-down; table depth 2.1 at z=0.90 -> cam z=3.0. Use px2world calibration: pixel(338,313)->(0.047,0.061) at depth 1.963
# Instead compute direct: object pixels near pot A
reg=d[290:340, 310:370]
mask=reg<2.05
ys,xs=np.where(mask)
print('bbox rows',ys.min()+290,ys.max()+290,'cols',xs.min()+310,xs.max()+310)
# per-row extents
for v in range(ys.min()+290, ys.max()+291):
    row=np.where(d[v,310:370]<2.05)[0]
    print(v, row.min()+310, row.max()+310, len(row), round(float(d[v,310:370][row].min()),3))
"

# openrua op 17
cd /workspace; timeout 60 python3 tools/perception/px2world.py birdview 338 316; timeout 60 python3 tools/perception/px2world.py birdview 338 311; timeout 60 python3 tools/perception/px2world.py birdview 338 322; echo stove; timeout 60 python3 tools/perception/px2world.py birdview 263 281; python3 -c "
import numpy as np
d=np.load('birdview_depth.npy')
reg=d[250:310, 235:295]
m=(reg>2.0)&(reg<2.09)
ys,xs=np.where(m); print('stove plate bbox rows',ys.min()+250,ys.max()+250,'cols',xs.min()+235,xs.max()+235, 'center',(xs.mean()+235).round(1),(ys.mean()+250).round(1), 'depth',np.median(reg[m]).round(3))
"

# openrua op 18
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw sideview_depth.png; timeout 20 ros2 topic echo /sideview/color/camera_info --once | grep -A1 '^k:' | tail -1

# openrua op 19
cd /workspace; python3 -c "
import numpy as np
d=np.load('sideview_depth.npy'); fx=579.41
for v in range(292, 384, 3):
    row=d[v,270:360]
    bg=np.median(np.concatenate([d[v,250:270], d[v,360:380]]))
    mask=row < bg-0.05
    if mask.any():
        idx=np.where(mask)[0]; u0,u1=idx.min()+270, idx.max()+270
        z=np.min(row[mask])
        print(v, u0, u1, 'w_px', u1-u0+1, 'zmin', round(float(z),3), 'w_m', round((u1-u0+1)*float(np.median(row[mask]))/fx,4))
    else: print(v,'none', round(float(bg),3))
"

# openrua op 20
cd /workspace; for p in "313 304" "313 312" "313 325" "313 340" "313 350" "313 364" "313 367"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py sideview $p; done

# openrua op 21
cd /workspace; sed -i 's/^Poses are in the ARM BASE frame (panda_link0) unless$/Poses for FK\/IK are in the WORLD frame (verified: FK(panda_link0) = (-0.66,0,0.912));/; s/^stated; world = base + (-0.66, 0, 0.912) per tf_static.$/leave frame_id empty. IK ignores yaw about hand z -> fix joint7 afterwards./' rob.py; head -6 rob.py; timeout 300 python3 -c "
from rob import *
r=Robot()
q0=r.arm_q()
POT=np.array([0.058,0.061]); 
qd=np.array([0.7071,0.7071,0,0])   # hand down, fingers along world x
for name,z in [('pre',1.13),('grasp',0.985)]:
    q=r.ik([POT[0],POT[1],z],qd,seed=q0,at_tcp=True,timeout=5)
    if q is None: continue
    t,quat=r.tcp(q); R=quat_to_R(quat)
    print(name,'q',q.round(3),'tcp',t.round(4),'hand x',R[:,0].round(3),'hand y',R[:,1].round(3),'hand z',R[:,2].round(3))
    # try fixing yaw with j7
    for j7 in [q[6]+np.pi/2, q[6]-np.pi/2]:
        if -2.9<j7<2.9:
            q2=q.copy(); q2[6]=j7; t2,quat2=r.tcp(q2); R2=quat_to_R(quat2)
            print('   j7=%.3f'%j7,'tcp',t2.round(4),'hand y',R2[:,1].round(3))
"

# openrua op 22
cd /workspace; timeout 300 python3 -c "
from rob import *
r=Robot()
qpre=np.array([ 0.112,0.641,-0.036,-1.421,0.025,2.062,-1.499])
qg=np.array([ 0.075,0.852,0.012,-1.453,-0.012,2.304,-1.479])
for name,q in [('pre',qpre),('grasp',qg)]:
    for d in [np.pi/4,-np.pi/4]:
        q2=q.copy(); q2[6]+=d; t,quat=r.tcp(q2); R=quat_to_R(quat)
        print(name,'j7=%.3f'%q2[6],'tcp',t.round(4),'hand y',R[:,1].round(3),'hand z',R[:,2].round(3))
"

# openrua op 23
cd /workspace; timeout 600 python3 -c "
from rob import *
r=Robot()
r.gripper(0.04)
print('fingers', {k:round(v,4) for k,v in r.joints().items() if 'finger' in k})
qpre=np.array([ 0.112,0.641,-0.036,-1.421,0.025,2.062,-0.714])
r.move(qpre, 4.0)
print('q now', r.arm_q().round(3)); t,quat=r.tcp(); print('tcp world', t.round(4), 'hand z', quat_to_R(quat)[:,2].round(3), 'hand y', quat_to_R(quat)[:,1].round(3))
"

# openrua op 24
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py agentview a1.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview s1.png

# openrua op 25
cd /workspace; timeout 600 python3 -c "
from rob import *
r=Robot()
qpre=np.array([ 0.112,0.641,-0.036,-1.421,0.025,2.062,-0.714])
for i in range(3):
    code,err=r.move(qpre, 4.0)
    if err<0.02: break
print('q now', r.arm_q().round(3)); t,quat=r.tcp(); print('tcp world', t.round(4), 'hand z', quat_to_R(quat)[:,2].round(3), 'hand y', quat_to_R(quat)[:,1].round(3))
"

# openrua op 26
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png

# openrua op 27
cd /workspace; timeout 900 python3 -c "
from rob import *
r=Robot()
qg=np.array([ 0.075,0.852,0.012,-1.453,-0.012,2.304,-0.694])
for i in range(3):
    code,err=r.move(qg, 3.0)
    if err<0.02: break
t,quat=r.tcp(); print('tcp world', t.round(4), 'hand z', quat_to_R(quat)[:,2].round(3), 'hand y', quat_to_R(quat)[:,1].round(3))
print('wrench before', r.wrench())
r.gripper(0.0)
print('fingers', {k:round(v,4) for k,v in r.joints().items() if 'finger' in k})
print('wrench after', r.wrench())
"

# openrua op 28
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py agentview a2.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview s2.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png

# openrua op 29
cd /workspace; timeout 900 python3 -c "
from rob import *
r=Robot()
q0=r.arm_q(); t0,quat0=r.tcp(q0); print('tcp now',t0.round(4))
qd=np.array([0.7071,0.7071,0,0])
q=r.ik([t0[0],t0[1],1.09],qd,seed=q0,at_tcp=True,timeout=5)
print('ik',q.round(3))
# fix yaw to match current j7 offset: find j7 giving hand y ~ (1,0,0)
best=None
for d in [0,np.pi/4,-np.pi/4,np.pi/2,-np.pi/2,3*np.pi/4,-3*np.pi/4]:
    q2=q.copy(); q2[6]=q[6]+d
    if not -2.9<q2[6]<2.9: continue
    t,qq=r.tcp(q2); hy=quat_to_R(qq)[:,1]
    if best is None or abs(hy[0])>best[0]: best=(abs(hy[0]),q2,hy)
print('chosen',best[1].round(3),'hand y',best[2].round(3), 'j7 delta from now', round(best[1][6]-q0[6],3))
np.save('q_lift1.npy',best[1])
"

# openrua op 30
cd /workspace; timeout 900 python3 -c "
from rob import *
r=Robot()
t0,_=r.tcp(); print('tcp before',t0.round(4), 'gap', round(r.finger_gap(),4))
r.servo([0,0,0.05],ticks=20)
t1,_=r.tcp(); print('tcp after 20 ticks',t1.round(4), 'dz', round(t1[2]-t0[2],4), 'gap', round(r.finger_gap(),4))
"

# openrua op 31
cd /workspace; timeout 900 python3 -c "
from rob import *
r=Robot()
t0,_=r.tcp()
r.servo([0,0,0.05],ticks=300,dt=0.02)
t1,_=r.tcp(); print('tcp after',t1.round(4), 'dz', round(t1[2]-t0[2],4), 'gap', round(r.finger_gap(),4), 'wrench', r.wrench().round(2))
"; timeout 60 python3 tools/perception/cam_snap.py sideview s3.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview a3.png

# openrua op 32
cd /workspace; timeout 120 python3 -c "
from rob import *
r=Robot()
t1,_=r.tcp(); print('tcp',t1.round(4), 'gap', round(r.finger_gap(),4), 'wrench', r.wrench().round(2))
"

# openrua op 33
cd /workspace; cat >> rob.py <<'EOF'

    def servo_to(self, target, speed=0.1, tol=0.004, burst=50, dt=0.02,
                 max_bursts=200, watch_gap=None):
        """closed-loop straight-line TCP move (world frame) via twist bursts"""
        target = np.asarray(target, float)
        for i in range(max_bursts):
            t, _ = self.tcp()
            err = target - t
            n = np.linalg.norm(err)
            if n < tol:
                print(f"servo_to reached {t.round(4)} (err {n*1000:.1f} mm)")
                return True
            if watch_gap is not None:
                g = self.finger_gap()
                if abs(g - watch_gap) > 0.004:
                    print(f"servo_to: gap changed {watch_gap:.4f}->{g:.4f}, stopping")
                    return False
            v = err / n * min(speed, max(0.02, n * 2))
            self.servo(v, ticks=burst, dt=dt)
        print(f"servo_to: budget exhausted at {t.round(4)}, err {n*1000:.1f} mm")
        return False
EOF
timeout 1200 python3 -c "
from rob import *
r=Robot()
g=r.finger_gap()
r.servo_to([0.058,0.061,1.16],speed=0.1,watch_gap=g)
print('gap',round(r.finger_gap(),4),'wrench',r.wrench().round(2))
"

# openrua op 34
cat /tmp/claude-1000/-workspace/f2a86f29-9a34-4849-af45-df926790e45e/tasks/bk11kof5n.output

# openrua op 35
cd /workspace; timeout 120 python3 -c "
from rob import *
r=Robot('peek')
t1,_=r.tcp(); print('tcp',t1.round(4), 'gap', round(r.finger_gap(),4))
"

# openrua op 36
cd /workspace; timeout 60 python3 tools/perception/cam_snap.py agentview a4.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview s4.png >/dev/null;  timeout 60 python3 tools/perception/cam_snap.py birdview b4.png; timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw b4_depth.png

# openrua op 37
cd /workspace; python3 -c "
import numpy as np
d=np.load('b4_depth.npy')
reg=d[280:350, 300:380]
m=(reg<1.98)
ys,xs=np.where(m)
print('lid-level bbox rows',ys.min()+280,ys.max()+280,'cols',xs.min()+300,xs.max()+300)
for v in range(ys.min()+280, ys.max()+281):
    row=np.where(d[v,300:380]<1.98)[0]
    print(v, row.min()+300, row.max()+300, len(row), round(float(d[v,300:380][row].min()),3))
"

# openrua op 38
cd /workspace; timeout 600 python3 -u -c "
from rob import *
r=Robot()
r.gripper(0.04)
t,_=r.tcp(); print('tcp',t.round(4))
" ; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png; timeout 60 python3 tools/perception/cam_snap.py /robot0_eye_in_hand/depth/image_raw eih3_depth.png; timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A9 '^k:'

# openrua op 39
cd /workspace; python3 -c "
import numpy as np, cv2
d=np.load('eih3_depth.npy'); fx=312.77; cx,cy=320,240
print(d.shape, np.nanmin(d), np.nanmax(d))
# pot A region: around (355,300)
reg=d[200:400, 250:450]
print('depth hist near pot', np.percentile(reg, [0,5,25,50,75,95,100]).round(3))
# lid pixels: depth close to min + 1.5cm, near center
zmin=np.nanmin(d[230:380, 280:430]); print('zmin',zmin)
m=np.zeros_like(d,bool); m[230:380,280:430]=(d[230:380,280:430]<zmin+0.03)
ys,xs=np.where(m); print('n',len(xs),'centroid px',xs.mean().round(1),ys.mean().round(1),'bbox',xs.min(),xs.max(),ys.min(),ys.max())
zl=np.median(d[m]); print('lid depth',zl.round(4),'px->m',zl/fx)
# widths along image x and y through centroid, and min-width direction
cxp,cyp=int(xs.mean()),int(ys.mean())
pts=np.stack([xs-xs.mean(),ys-ys.mean()],1)*zl/fx
for ang in range(0,180,15):
    a=np.deg2rad(ang); u=np.array([np.cos(a),np.sin(a)])
    proj=pts@u; print(ang, 'extent m', (proj.max()-proj.min()).round(4))
"

# openrua op 40
cd /workspace; python3 -c "
import numpy as np
d=np.load('eih3_depth.npy'); fx=312.77
sub=d[200:360, 280:440]
vals=np.sort(sub.ravel()); print(np.percentile(vals,[0,2,5,10,20,30,40,50,60,70,80,90,100]).round(3))
# histogram bins
h,e=np.histogram(sub,bins=np.arange(0.2,0.42,0.01)); 
for c,lo in zip(h,e[:-1]): print(round(lo,2),c)
"

# openrua op 41
cd /workspace; python3 -c "
import numpy as np
d=np.load('eih3_depth.npy'); fx=312.77
m=np.zeros_like(d,bool); m[200:360,280:440]=(d[200:360,280:440]<0.25)
ys,xs=np.where(m); print('n',len(xs),'centroid px',xs.mean().round(1),ys.mean().round(1),'bbox x',xs.min(),xs.max(),'y',ys.min(),ys.max())
z=d[m]; 
# use per-pixel depth to convert to metric camera-plane coords
X=(xs-320)*z/fx; Y=(ys-240)*z/fx
pts=np.stack([X,Y],1); c=pts.mean(0)
print('metric centroid',c.round(4))
for ang in range(0,180,15):
    a=np.deg2rad(ang); u=np.array([np.cos(a),np.sin(a)])
    proj=(pts-c)@u; print(ang, 'extent m', (proj.max()-proj.min()).round(4), 'lo',proj.min().round(4),'hi',proj.max().round(4))
# remove handle/spout: keep points within radius r of centroid and re-evaluate extents of the 'disc'
rr=np.linalg.norm(pts-c,axis=1)
for R in [0.035,0.04,0.045]:
    print('frac within',R, (rr<R).mean().round(3))
"

# openrua op 42
cd /workspace; python3 -c "
import numpy as np
d=np.load('eih3_depth.npy')
m=np.zeros_like(d,bool); m[200:360,280:440]=(d[200:360,280:440]<0.25)
ys,xs=np.where(m)
# exclude handle/spout: iteratively keep points within 3.6cm of the centroid (metric)
fx=312.77; z=d[m]; X=(xs-320)*z/fx; Y=(ys-240)*z/fx; pts=np.stack([X,Y],1); c=pts.mean(0)
for _ in range(5):
    keep=np.linalg.norm(pts-c,axis=1)<0.037; c=pts[keep].mean(0)
sel=np.where(keep)[0]
u=int(round(xs[sel].mean())); v=int(round(ys[sel].mean())); print('rim disc centre px',u,v,'depth',d[v,u])
# knob pixel: min depth
k=np.argmin(np.where(m,d,9)); kv,ku=divmod(k,d.shape[1]); print('knob px',ku,kv,d[kv,ku])
" ; timeout 60 python3 tools/perception/px2world.py robot0_eye_in_hand 360 296; timeout 20 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A2 Translation | head -3

# openrua op 43
cd /workspace; timeout 60 python3 -c "
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node('urdf_get')
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
got=[]
n.create_subscription(String,'/robot_description',lambda m: got.append(m.data),qos)
import time; t=time.time()
while not got and time.time()-t<15: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0] if got else '')
print(len(got[0]) if got else 'none')
"; grep -n -A12 '<link name="panda_hand"' robot.urdf | head -60; grep -n -B2 -A10 'name="panda_leftfinger"' robot.urdf | head -50; grep -n -i 'camera\|eye_in_hand' robot.urdf | head

# openrua op 44
cd /workspace; python3 -c "
import numpy as np, struct
def stl_bbox(fn):
    b=open(fn,'rb').read()
    if b[:5]==b'solid' and b'facet' in b[:300]:
        import re
        v=np.array([[float(x) for x in m] for m in re.findall(rb'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',b)])
    else:
        n=struct.unpack('<I',b[80:84])[0]
        arr=np.frombuffer(b[84:84+n*50],dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))
        v=arr['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ['hand','finger']:
    lo,hi=stl_bbox(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'); print(f,'min',lo.round(4),'max',hi.round(4),'size',(hi-lo).round(4))
"

# openrua op 45
find / -iname 'hand*.stl' -o -iname 'finger*.stl' 2>/dev/null | head; find / -iname '*moka*' 2>/dev/null | head

# openrua op 46
cd /workspace; python3 -c "
import numpy as np
d=np.load('eih3_depth.npy'); fx=312.77
# fingers: depth < 0.15 region
m=d<0.16
ys,xs=np.where(m); print('finger px bbox x',xs.min(),xs.max(),'y',ys.min(),ys.max(), 'n',len(xs))
# per row, find the two finger blobs' inner edges
for v in range(ys.min(), ys.max()+1, 6):
    row=np.where(d[v]<0.16)[0]
    if len(row)<2: continue
    # split into left/right blobs
    gaps=np.where(np.diff(row)>20)[0]
    if len(gaps)==0: print(v,'single blob',row.min(),row.max()); continue
    L=row[:gaps[0]+1]; R=row[gaps[0]+1:]
    zl=d[v,L].mean(); zr=d[v,R].mean()
    inner_l=(L.max()-320)*zl/fx; inner_r=(R.min()-320)*zr/fx
    print(v,'L',L.min(),L.max(),'R',R.min(),R.max(),'zL',zl.round(3),'zR',zr.round(3),'inner gap m',(inner_r-inner_l).round(4),'mid px',(L.max()+R.min())/2)
"

# openrua op 47
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Depth image -> world-frame point cloud for one camera (fresh frame).

Usage as a module:  pts, rgb = cloud.grab("agentview")  # (N,3) world xyz
"""
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def _quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def grab(cam, node=None, timeout=20.0):
    own = node is None
    if own:
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node("cloud_" + cam)
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    got = {}
    subs = [
        node.create_subscription(Image, f"/{cam}/depth/image_raw",
                                 lambda m: got.setdefault("d", m), 1),
        node.create_subscription(Image, f"/{cam}/color/image_raw",
                                 lambda m: got.setdefault("c", m), 1),
        node.create_subscription(CameraInfo, f"/{cam}/color/camera_info",
                                 lambda m: got.setdefault("i", m), 1),
    ]
    frame = f"{cam}_optical_frame"
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if len(got) == 3 and tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    for s in subs:
        node.destroy_subscription(s)
    if len(got) < 3:
        raise RuntimeError(f"missing data for {cam}: {list(got)}")
    br = CvBridge()
    depth = br.imgmsg_to_cv2(got["d"], "passthrough").astype(np.float64)
    rgb = br.imgmsg_to_cv2(got["c"], "rgb8")
    k = got["i"].k
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = _quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y,
                  t.transform.translation.z])
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    z = depth
    X = (u - cx) * z / fx
    Y = (v - cy) * z / fy
    P = np.stack([X, Y, z], -1).reshape(-1, 3) @ R.T + T
    P = P.reshape(h, w, 3)
    if own:
        node.destroy_node()
    return P, rgb, depth


if __name__ == "__main__":
    import sys
    P, rgb, depth = grab(sys.argv[1])
    np.save(f"{sys.argv[1]}_cloud.npy", P)
    print(P.shape, "saved")
OPENRUA_EOF

# openrua op 48
cd /workspace; timeout 120 python3 -c "
import cloud, numpy as np
P,rgb,depth=cloud.grab('agentview')
np.save('agentview_cloud.npy',P)
# pot A points: world box around (0.0485,0.068), z 0.9-1.07 (above table), exclude robot (x< -0.1 not needed)
m=(np.abs(P[...,0]-0.0485)<0.09)&(np.abs(P[...,1]-0.068)<0.12)&(P[...,2]>0.905)&(P[...,2]<1.08)
pts=P[m]; print('n',len(pts))
# profile: for z bins, min/max y and min/max x
for z0 in np.arange(0.905,1.07,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s)==0: print(round(z0,3),'-'); continue
    print(round(z0,3), 'n',len(s), 'y[%.3f %.3f]'%(s[:,1].min(),s[:,1].max()), 'x[%.3f %.3f]'%(s[:,0].min(),s[:,0].max()))
"

# openrua op 49
cd /workspace; timeout 900 python3 -u -c "
from rob import *
r=Robot()
q_home=np.array([0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483])
for i in range(3):
    code,err=r.move(q_home, 4.0)
    if err<0.02: break
t,_=r.tcp(); print('tcp',t.round(4))
"; timeout 120 python3 -c "
import cloud, numpy as np
P,rgb,depth=cloud.grab('birdview'); np.save('birdview_cloud.npy',P)
m=(np.abs(P[...,0]-0.05)<0.1)&(np.abs(P[...,1]-0.065)<0.1)&(P[...,2]>1.02)&(P[...,2]<1.06)
pts=P[m]; print('lid-level pts',len(pts))
# rim disc: iteratively trim to within 3.6cm of centroid
c=pts[:,:2].mean(0)
for _ in range(6):
    keep=np.linalg.norm(pts[:,:2]-c,axis=1)<0.037; c=pts[keep][:,:2].mean(0)
print('birdview rim centre',c.round(4),'z med',np.median(pts[keep][:,2]).round(4))
# octagon orientation: extents by angle
q=pts[keep][:,:2]-c
for ang in range(0,90,5):
    a=np.deg2rad(ang); u=np.array([np.cos(a),np.sin(a)]); pr=q@u; print(ang,(pr.max()-pr.min()).round(4))
"

# openrua op 50
mkdir -p "$(dirname /workspace/align.py)"
cat > /workspace/align.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Measure pot rim vs fingers from the wrist camera (world frame via TF).

Returns dict with rim centre (x,y), rim top z, flat-normal angle (deg,
world, in [0,45)), finger gap centre and finger axis angle, plus FK tcp.
"""
import numpy as np

import cloud
from rob import quat_to_R


def extents(q, ang_deg):
    a = np.deg2rad(ang_deg)
    u = np.array([np.cos(a), np.sin(a)])
    pr = q @ u
    return pr.min(), pr.max()


def measure(r, pot_guess, verbose=True):
    P, rgb, depth = cloud.grab("robot0_eye_in_hand", node=None)
    tcp, quat = r.tcp()
    R = quat_to_R(quat)
    out = {"tcp": tcp, "hand_y": R[:, 1]}
    # ---- fingers: points within 12 cm of tcp and above tcp-1cm, at |hand-y| ~ 3.5..5 cm
    rel = P - tcp
    hy = rel @ R[:, 1]
    hx = rel @ R[:, 0]
    hz = rel @ R[:, 2]
    fm = (np.abs(hx) < 0.02) & (hz > -0.06) & (hz < 0.015) & (np.abs(hy) > 0.03) & (np.abs(hy) < 0.06)
    fm &= np.isfinite(P).all(-1)
    fp = P[fm]
    if len(fp) > 50:
        hyf = (fp - tcp) @ R[:, 1]
        left = hyf[hyf < 0]
        right = hyf[hyf > 0]
        # inner faces = max of left blob, min of right blob (robust percentiles)
        li = np.percentile(left, 99) if len(left) else np.nan
        ri = np.percentile(right, 1) if len(right) else np.nan
        out["finger_inner_hy"] = (li, ri)
        out["finger_gap"] = ri - li
        out["finger_centre_hy"] = (li + ri) / 2
    # ---- rim
    m = (np.abs(P[..., 0] - pot_guess[0]) < 0.08) & (np.abs(P[..., 1] - pot_guess[1]) < 0.08)
    m &= np.isfinite(P).all(-1)
    pts = P[m]
    ztop = np.percentile(pts[:, 2], 99.5)
    out["z_top"] = ztop
    rim = pts[(pts[:, 2] > ztop - 0.035) & (pts[:, 2] < ztop - 0.012)]  # rim + top wall, below knob
    c = rim[:, :2].mean(0)
    for _ in range(8):
        keep = np.linalg.norm(rim[:, :2] - c, axis=1) < 0.0415
        c = rim[keep][:, :2].mean(0)
    q = rim[keep][:, :2] - c
    angs = np.arange(0, 45, 0.5)
    ext = np.array([np.subtract(*extents(q, a)[::-1]) for a in angs])
    amin = angs[np.argmin(ext)]
    # refine centre as midpoint along flat normals amin and amin+90 (and 45, 135)
    cs = []
    for a in (amin, amin + 45, amin + 90, amin + 135):
        lo, hi = extents(q, a)
        u = np.array([np.cos(np.deg2rad(a)), np.sin(np.deg2rad(a))])
        cs.append(((lo + hi) / 2) * u)
    c_ref = c + np.sum(cs, 0) / 2  # each pair of perpendicular normals gives full 2D shift
    out["rim_centre"] = c_ref
    out["flat_normal_deg"] = amin
    out["ftf"] = ext.min()
    out["vtv"] = ext.max()
    out["ext_profile"] = list(zip(angs[::10], ext[::10].round(4)))
    if verbose:
        for k, v in out.items():
            if k != "ext_profile":
                print(f"  {k}: {np.round(v, 4) if not isinstance(v, tuple) else tuple(np.round(v, 4))}")
    return out
OPENRUA_EOF

# openrua op 51
cd /workspace; timeout 1200 python3 -u -c "
from rob import *
import align
r=Robot()
q0=r.arm_q()
qd=np.array([0.7071,0.7071,0,0])
POT=[0.045,0.063]
q=r.ik([POT[0],POT[1],1.075],qd,seed=q0,at_tcp=True,timeout=5)
print('ik',q.round(3))
# fix yaw -> fingers along world x
best=None
for d in np.arange(-np.pi, np.pi, np.pi/4):
    q2=q.copy(); q2[6]=q[6]+d
    if not -2.85<q2[6]<2.85: continue
    t,qq=r.tcp(q2); hy=quat_to_R(qq)[:,1]
    if best is None or hy[0]>best[0]: best=(hy[0],q2)
q=best[1]; print('yaw-fixed',q.round(3))
for i in range(4):
    code,err=r.move(q, 4.0)
    if err<0.01: break
t,quat=r.tcp(); print('tcp',t.round(4),'hand y',quat_to_R(quat)[:,1].round(3),'hand z',quat_to_R(quat)[:,2].round(3))
m=align.measure(r, POT)
"

# openrua op 52
cd /workspace; timeout 120 python3 -c "
import cloud, numpy as np
P,rgb,depth=cloud.grab('robot0_eye_in_hand'); np.save('eih_cloud.npy',P); np.save('eih_depth_now.npy',depth)
import cv2; cv2.imwrite('eih_now.png', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
tcp=np.array([0.0449,0.0629,1.075])
rel=P-tcp
# finger points: z between tcp-0.005 and tcp+0.03 (fingertips ~ tcp level), |dy|<2cm, |dx| 2.5..6cm
fm=(rel[...,2]>-0.005)&(rel[...,2]<0.03)&(np.abs(rel[...,1])<0.02)&(np.abs(rel[...,0])>0.025)&(np.abs(rel[...,0])<0.06)
fp=rel[fm]; print('finger pts',len(fp))
for side,s in [('L',fp[fp[:,0]<0]),('R',fp[fp[:,0]>0])]:
    if len(s): print(side,'n',len(s),'x range',s[:,0].min().round(4),s[:,0].max().round(4),'z range',s[:,2].min().round(4),s[:,2].max().round(4), 'y range', s[:,1].min().round(4), s[:,1].max().round(4))
# histogram of x for finger pts
h,e=np.histogram(fp[:,0],bins=np.arange(-0.06,0.061,0.002)); 
for c,lo in zip(h,e[:-1]):
    if c: print(round(lo,3),c)
"

# openrua op 53
cd /workspace; python3 -c "
import numpy as np
P=np.load('eih_cloud.npy'); tcp=np.array([0.0449,0.0629,1.075])
guess=np.array([0.045,0.063])
m=(np.abs(P[...,0]-guess[0])<0.08)&(np.abs(P[...,1]-guess[1])<0.08)&(P[...,2]<tcp[2]-0.01)&np.isfinite(P).all(-1)
pts=P[m]; ztop=np.percentile(pts[:,2],99.8); print('ztop',ztop.round(4), 'n',len(pts))
h,e=np.histogram(pts[:,2],bins=np.arange(0.90,1.07,0.005))
for c,lo in zip(h,e[:-1]):
    if c: print(round(lo,3),c)
"

# openrua op 54
cd /workspace; python3 -c "
import numpy as np
P=np.load('eih_cloud.npy'); tcp=np.array([0.0449,0.0629,1.075]); guess=np.array([0.045,0.063])
m=(np.abs(P[...,0]-guess[0])<0.08)&(np.abs(P[...,1]-guess[1])<0.08)&(P[...,2]>1.022)&(P[...,2]<1.045)&np.isfinite(P).all(-1)
pts=P[m][:,:2]; c=pts.mean(0)
for _ in range(8):
    keep=np.linalg.norm(pts-c,axis=1)<0.043; c=pts[keep].mean(0)
q=pts[keep]-c; print('n',len(q),'centre',c.round(4))
def ext(q,a):
    u=np.array([np.cos(np.deg2rad(a)),np.sin(np.deg2rad(a))]); pr=q@u; return pr.min(),pr.max()
angs=np.arange(0,45,0.5); e=np.array([np.subtract(*ext(q,a)[::-1]) for a in angs])
amin=angs[np.argmin(e)]; print('flat normal angle',amin,'ftf',e.min().round(4),'vtv',e.max().round(4))
for a in range(0,180,15): lo,hi=ext(q,a); print(a,'lo',lo.round(4),'hi',hi.round(4),'ext',(hi-lo).round(4))
# refined centre from flat normals amin, amin+90 (choose the ones not near the handle-spout axis ~90deg)
cs=np.zeros(2)
for a in (amin, amin+90):
    lo,hi=ext(q,a); u=np.array([np.cos(np.deg2rad(a)),np.sin(np.deg2rad(a))]); cs+=((lo+hi)/2)*u
print('centre refined',(c+cs).round(4), 'rel to tcp',(c+cs-tcp[:2]).round(4))
# also the handle/spout direction: points beyond 4.3cm
far=pts[np.linalg.norm(pts-c,axis=1)>=0.043]-c
ang=np.degrees(np.arctan2(far[:,1],far[:,0])); h,_=np.histogram(ang,bins=np.arange(-180,181,15)); print('far pts angle hist',list(zip(np.arange(-180,180,15),h)))
"

# openrua op 55
cd /workspace; python3 -c "
import numpy as np, cv2
P=np.load('eih_cloud.npy'); tcp=np.array([0.0449,0.0629,1.075]); guess=np.array([0.045,0.063])
m=(np.abs(P[...,0]-guess[0])<0.08)&(np.abs(P[...,1]-guess[1])<0.08)&(P[...,2]<tcp[2]-0.01)&(P[...,2]>0.95)&np.isfinite(P).all(-1)
pts=P[m]
# top-down height map at 0.5mm resolution, 16x16cm, x right, y up
res=0.0005; W=int(0.16/res)
img=np.zeros((W,W),np.float32)
ix=((pts[:,0]-(guess[0]-0.08))/res).astype(int); iy=((guess[1]+0.08-pts[:,1])/res).astype(int)
ok=(ix>=0)&(ix<W)&(iy>=0)&(iy<W)
np.maximum.at(img,(iy[ok],ix[ok]),pts[ok,2])
vis=np.zeros((W,W,3),np.uint8)
z=img; 
vis[(z>0.95)&(z<1.0)]=(80,80,80)
vis[(z>=1.0)&(z<1.022)]=(0,128,255)
vis[(z>=1.022)&(z<1.040)]=(255,255,255)
vis[(z>=1.040)]=(0,0,255)
# tcp marker and finger faces (x = tcp +- 0.040)
def px(x,y): return int((x-(guess[0]-0.08))/res), int((guess[1]+0.08-y)/res)
cv2.circle(vis, px(tcp[0],tcp[1]), 4, (0,255,0), 1)
for s in (-0.040,0.040):
    x0,y0=px(tcp[0]+s, tcp[1]-0.05); x1,y1=px(tcp[0]+s, tcp[1]+0.05); cv2.line(vis,(x0,y0),(x1,y1),(0,255,0),1)
vis=cv2.resize(vis,(640,640),interpolation=cv2.INTER_NEAREST)
cv2.imwrite('topdown.png',vis)
"

# openrua op 56
cd /workspace; cat > knob.py <<'EOF'
import numpy as np
import cloud

def knob_and_flats(tcp, guess, P=None):
    """pot centre from knob-top centroid; flat-normal angle from lid octagon"""
    if P is None:
        P, _, _ = cloud.grab("robot0_eye_in_hand")
    m = (np.abs(P[..., 0] - guess[0]) < 0.08) & (np.abs(P[..., 1] - guess[1]) < 0.08) \
        & (P[..., 2] < tcp[2] - 0.008) & np.isfinite(P).all(-1)
    pts = P[m]
    ztop = np.percentile(pts[:, 2], 99.5)
    knob = pts[pts[:, 2] > ztop - 0.006]
    kc = knob[:, :2].mean(0)
    # knob is a disc: refine with the extents midpoint (robust to partial occlusion? use max-extent)
    lid = pts[(pts[:, 2] > ztop - 0.032) & (pts[:, 2] < ztop - 0.012)]  # lid top, below knob
    q = lid[:, :2] - kc
    def ext(a):
        u = np.array([np.cos(np.deg2rad(a)), np.sin(np.deg2rad(a))]); pr = q @ u
        return pr.min(), pr.max()
    angs = np.arange(0, 45, 0.5)
    e = np.array([np.subtract(*ext(a)[::-1]) for a in angs])
    amin = angs[np.argmin(e)]
    # extents along x (vertex axis) for centre check: midpoint along 0 deg
    lo, hi = ext(0.0)
    return dict(knob_centre=kc, z_top=ztop, flat_normal_deg=float(amin), ftf=float(e.min()),
                vtv=float(e.max()), x_ext=(lo, hi), x_mid=kc[0] + (lo + hi) / 2, n_knob=len(knob))
EOF
python3 -c "
import numpy as np, knob
P=np.load('eih_cloud.npy'); tcp=np.array([0.0449,0.0629,1.075])
r=knob.knob_and_flats(tcp,[0.045,0.063],P)
for k,v in r.items(): print(k, np.round(v,4) if not isinstance(v,tuple) else tuple(np.round(v,4)))
print('knob rel tcp', (r['knob_centre']-tcp[:2]).round(4))
"

# openrua op 57
cd /workspace; python3 -u - <<'EOF'
import numpy as np, rob
r = rob.Robot()
q = r.arm_q(); print("q", q.round(3))
t, quat = r.tcp(); R = rob.quat_to_R(quat)
print("tcp", t.round(4), "hand y ang", np.degrees(np.arctan2(R[1,1], R[0,1])).round(2))
for d in (+0.3927, -0.3927):
    q2 = q.copy(); q2[6] += d
    t2, quat2 = r.tcp(q2); R2 = rob.quat_to_R(quat2)
    print(f"dj7={d:+.4f}: tcp {t2.round(4)} hand y ang {np.degrees(np.arctan2(R2[1,1], R2[0,1])).round(2)} z-axis {R2[:,2].round(3)}")
EOF

# openrua op 58
cd /workspace; python3 -u - <<'EOF'
import numpy as np, rob
r = rob.Robot()
q = r.arm_q(); q2 = q.copy(); q2[6] += np.deg2rad(-(21.5 + 0.12))
for i in range(4):
    code, err = r.move(q2, seconds=2.0)
    if err < 0.005: break
t, quat = r.tcp(); R = rob.quat_to_R(quat)
print("tcp", t.round(4), "hand y ang", np.degrees(np.arctan2(R[1,1], R[0,1])).round(2), "gap", round(r.finger_gap(),4))
EOF

# openrua op 59
cd /workspace; python3 -u - <<'EOF'
import numpy as np, rob, cloud, knob
r = rob.Robot()
t, quat = r.tcp(); R = rob.quat_to_R(quat)
P, rgb, depth = cloud.grab("robot0_eye_in_hand")
np.save("eih_cloud2.npy", P)
k = knob.knob_and_flats(t, t[:2], P)
print("knob rel tcp", (k["knob_centre"]-t[:2]).round(4), "ztop", round(k["z_top"],4), "flat", k["flat_normal_deg"], "ftf", round(k["ftf"],4), "vtv", round(k["vtv"],4))
hy_ang = np.degrees(np.arctan2(R[1,1], R[0,1]))
print("hand y ang", hy_ang.round(2), "-> misalignment", round(((k['flat_normal_deg']-hy_ang+22.5)%45)-22.5, 2), "deg")
# finger faces in hand frame
rel = P - t; hx = rel@R[:,0]; hy = rel@R[:,1]; hz = rel@R[:,2]
fm = (np.abs(hx)<0.02)&(hz>-0.06)&(hz<0.015)&(np.abs(hy)>0.03)&(np.abs(hy)<0.06)&np.isfinite(P).all(-1)
hyf = hy[fm]
print("finger faces hy:", np.percentile(hyf[hyf<0],99).round(4), np.percentile(hyf[hyf>0],1).round(4), "n", fm.sum())
# extent of lid along hand y (the closing direction), relative to knob centre
lid = P[(np.abs(P[...,0]-t[0])<0.08)&(np.abs(P[...,1]-t[1])<0.08)&(P[...,2]>k["z_top"]-0.032)&(P[...,2]<k["z_top"]-0.012)&np.isfinite(P).all(-1)]
pr = (lid[:,:2]-k["knob_centre"])@R[:2,1]
print("lid extent along hand y:", pr.min().round(4), pr.max().round(4))
EOF

# openrua op 60
cd /workspace; python3 -u - <<'EOF'
import numpy as np, rob
P=np.load("eih_cloud2.npy")
t=np.array([0.045,0.0629,1.075]); ang=np.deg2rad(21.37)
R=np.array([[np.cos(ang),-np.sin(ang),0],[np.sin(ang),np.cos(ang),0],[0,0,1]])  # hand x,y in world (z ignored)
rel=P-t; hx=rel@R[:,0]; hy=rel@R[:,1]; hz=-(rel@np.array([0,0,1]))  # hz positive = below tcp
m=(np.abs(hx)<0.02)&(hz>-0.06)&(hz<0.015)&np.isfinite(P).all(-1)
for lo in np.arange(-0.06,0.06,0.005):
    s=m&(hy>=lo)&(hy<lo+0.005)
    if s.sum(): print(f"hy {lo:+.3f}: n={s.sum():5d} hz {hz[s].min():+.3f}..{hz[s].max():+.3f}  hx {hx[s].min():+.3f}..{hx[s].max():+.3f}")
EOF

# openrua op 61
cd /workspace; python3 -u - <<'EOF'
import numpy as np, rob
r=rob.Robot(); t,quat=r.tcp(); R=rob.quat_to_R(quat)
print("hand x",R[:,0].round(3),"hand y",R[:,1].round(3))
P=np.load("eih_cloud2.npy")
rel=P-t; hx=rel@R[:,0]; hy=rel@R[:,1]; hz=rel@R[:,2]
m=(np.abs(hx)<0.02)&(hz>-0.06)&(hz<0.015)&np.isfinite(P).all(-1)
for lo in np.arange(-0.06,0.06,0.005):
    s=m&(hy>=lo)&(hy<lo+0.005)
    if s.sum(): print(f"hy {lo:+.3f}: n={s.sum():5d} hz {hz[s].min():+.3f}..{hz[s].max():+.3f}  hx {hx[s].min():+.3f}..{hx[s].max():+.3f}")
EOF

# openrua op 62
cd /workspace; python3 -u - <<'EOF'
import numpy as np, rob
r=rob.Robot(); t,quat=r.tcp(); R=rob.quat_to_R(quat)
P=np.load("eih_cloud2.npy")
rel=P-t; hx=rel@R[:,0]; hy=rel@R[:,1]; hz=rel@R[:,2]
m=(np.abs(hx)<0.017)&(hz>-0.06)&(hz<0.015)&np.isfinite(P).all(-1)&(np.abs(hy)>0.02)
for side,s in (("neg",m&(hy<0)),("pos",m&(hy>0))):
    v=np.abs(hy[s]); print(side, "n",s.sum(),"inner face percentiles 0.5/1/2/5%:",np.percentile(v,[0.5,1,2,5]).round(4), "hz range",hz[s].min().round(3),hz[s].max().round(3))
print("gap",round(r.finger_gap(),4), r.joints()["panda_finger_joint1"], r.joints()["panda_finger_joint2"])
EOF

# openrua op 63
cd /workspace; timeout 300 python3 -u - <<'EOF'
import numpy as np, rob
r=rob.Robot(); t,quat=r.tcp()
target=t+np.array([-0.0003,0.0016,0.0])
ok=r.servo_to(target, speed=0.03, tol=0.0006, burst=100, dt=0.02, max_bursts=15)
t2,_=r.tcp(); print("tcp now",t2.round(4),"wrench",r.wrench().round(2),"gap",round(r.finger_gap(),4))
EOF

# openrua op 64
cd /workspace; python3 - <<'EOF'
s=open("rob.py").read()
old='''    def joints(self, fresh=True):
        if fresh:
            self._js = None
        end = time.time() + 15
        while self._js is None and time.time() < end:
            self.spin(0.1)
        if self._js is None:
            raise RuntimeError("no /joint_states")
        d = dict(zip(self._js.name, self._js.position))
        return d'''
new='''    def joints(self, fresh=True, wait=3.0):
        """latest joint state; with fresh=True wait up to `wait` s for a new
        message (sim may be paused -> fall back to the last cached one)"""
        last = self._js
        if fresh:
            self._js = None
        end = time.time() + (wait if last is not None else 15)
        while self._js is None and time.time() < end:
            self.spin(0.1)
        if self._js is None:
            if last is None:
                raise RuntimeError("no /joint_states")
            self._js = last
        d = dict(zip(self._js.name, self._js.position))
        return d'''
assert old in s; open("rob.py","w").write(s.replace(old,new)); print("ok")
EOF
timeout 300 python3 -u - <<'EOF'
import numpy as np, rob
r=rob.Robot(); t,quat=r.tcp()
target=t+np.array([-0.0003,0.0016,0.0])
ok=r.servo_to(target, speed=0.03, tol=0.0006, burst=100, dt=0.02, max_bursts=15)
t2,_=r.tcp(); print("tcp now",t2.round(4),"wrench",r.wrench().round(2),"gap",round(r.finger_gap(),4))
EOF

# openrua op 65
cd /workspace; timeout 20 ros2 topic hz /joint_states --window 20 2>&1 | head -3; timeout 10 ros2 topic echo --once /joint_states 2>&1 | grep -A9 position | head -12; ros2 node list 2>&1 | head -20

# openrua op 66
cd /workspace; ros2 topic list 2>&1 | head -40; echo ---; ros2 node info /robot_bridge 2>&1 | head -60

# openrua op 67
cd /workspace; (timeout 25 ros2 topic echo /joint_states --field header.stamp 2>&1 | head -12) ; echo ---; timeout 8 ros2 topic echo --once /clock 2>&1 | head -5; grep -n -i -B2 -A6 "clock\|paused\|publish" /workspace/docs/*.md | head -60

# openrua op 68
cd /workspace; sed -n 1,80p docs/10-machine.md | grep -n -A12 "§4\|## 4\|clock" | head -50; echo ---; timeout 10 ros2 topic hz /clock --window 5 2>&1 | head -2; timeout 10 ros2 topic hz /agentview/color/image_raw --window 5 2>&1 | head -2

# openrua op 69
ps aux --sort=-%cpu | head -8; echo; timeout 15 ros2 topic hz /joint_states --window 5 2>&1 | head -2

# openrua op 70
cd /workspace; timeout 60 python3 -u - <<'EOF'
import rclpy, time, yaml
from geometry_msgs.msg import TwistStamped
tw = next(a for a in yaml.safe_load(open("machine.yaml"))["actuators"] if a["kind"] == "cartesian_twist")
rclpy.init(); node = rclpy.create_node("stopper")
pub = node.create_publisher(TwistStamped, tw["port"], 10)
msg = TwistStamped(); msg.header.frame_id = tw["frame"]
for _ in range(5):
    msg.header.stamp = node.get_clock().now().to_msg(); pub.publish(msg); rclpy.spin_once(node, timeout_sec=0.05)
print("sent zero twists")
EOF
for i in 1 2 3; do timeout 20 ros2 topic echo --once /joint_states --field position 2>&1 | head -3; date +%T; done

# openrua op 71
cd /workspace; timeout 20 ros2 param list /robot_bridge 2>&1 | head; echo "rc=$?"; timeout 20 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1], position: [0.0]}}}" 2>&1 | tail -3

# openrua op 72
cd /workspace; timeout 60 python3 -u - <<'EOF'
import rclpy, time
from sensor_msgs.msg import JointState, Image
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
rclpy.init(); n=rclpy.create_node("probe")
c={"js":0,"clock":0,"tf":0,"img":0}; last={}
n.create_subscription(JointState,"/joint_states",lambda m:(c.__setitem__("js",c["js"]+1), last.__setitem__("js",m)),10)
n.create_subscription(Clock,"/clock",lambda m:(c.__setitem__("clock",c["clock"]+1), last.__setitem__("clock",m)),10)
n.create_subscription(TFMessage,"/tf",lambda m:c.__setitem__("tf",c["tf"]+1),10)
n.create_subscription(Image,"/agentview/color/image_raw",lambda m:c.__setitem__("img",c["img"]+1),1)
t0=time.time()
while time.time()-t0<40:
    rclpy.spin_once(n,timeout_sec=0.2)
    if int(time.time()-t0)%10==0 and c["js"]+c["clock"]>0: pass
print(c)
if "clock" in last: print("clock", last["clock"].clock.sec, last["clock"].clock.nanosec)
if "js" in last: print("js", dict(zip(last["js"].name, [round(p,4) for p in last["js"].position])))
EOF

# openrua op 73
cd /workspace; timeout 60 python3 -u - <<'EOF'
import rclpy, time, numpy as np
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
rclpy.init(); n=rclpy.create_node("probe2")
st={}
n.create_subscription(JointState,"/joint_states",lambda m:st.__setitem__("js",m),10)
n.create_subscription(Clock,"/clock",lambda m:st.__setitem__("clk",m),10)
t0=time.time(); lastp=None
while time.time()-t0<20:
    rclpy.spin_once(n,timeout_sec=0.2)
    if "js" in st and "clk" in st:
        p=np.array(st["js"].position[:7]).round(4)
        if lastp is None or (p!=lastp).any():
            print(f"{time.time()-t0:5.1f}s clk={st['clk'].clock.sec}.{st['clk'].clock.nanosec//100000000} q={p}")
            lastp=p
EOF

# openrua op 74
cd /workspace; timeout 60 python3 -u - <<'EOF'
import numpy as np, rob
r=rob.Robot(); q=r.arm_q(); print("q",q.round(4))
t,quat=r.tcp(); R=rob.quat_to_R(quat)
print("tcp",t.round(4),"hand y ang",np.degrees(np.arctan2(R[1,1],R[0,1])).round(2),"z-axis",R[:,2].round(3))
print("wrench",r.wrench(),"gap",round(r.finger_gap(),4))
EOF

# openrua op 75
cd /workspace; timeout 120 python3 -u - <<'EOF'
import rclpy, time, numpy as np, yaml
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
tw = next(a for a in yaml.safe_load(open("machine.yaml"))["actuators"] if a["kind"] == "cartesian_twist")
rclpy.init(); n=rclpy.create_node("cal")
st={"js":[], "clk":[]}
n.create_subscription(JointState,"/joint_states",lambda m:st["js"].append((time.time(),np.array(m.position[:7]))),10)
n.create_subscription(Clock,"/clock",lambda m:st["clk"].append((time.time(),m.clock.sec+m.clock.nanosec*1e-9)),10)
pub=n.create_publisher(TwistStamped,tw["port"],10)
def settle(quiet=6.0):
    """spin until no clock message for `quiet` seconds"""
    last=time.time()
    while time.time()-last<quiet:
        k=len(st["clk"]); rclpy.spin_once(n,timeout_sec=0.2)
        if len(st["clk"])>k: last=time.time()
settle(4)
def test(nmsg, dt, vz):
    c0=st["clk"][-1][1] if st["clk"] else None; j0=st["js"][-1][1] if st["js"] else None
    nc=len(st["clk"])
    msg=TwistStamped(); msg.header.frame_id=tw["frame"]; msg.twist.linear.z=vz
    t0=time.time()
    for _ in range(nmsg):
        msg.header.stamp=n.get_clock().now().to_msg(); pub.publish(msg); rclpy.spin_once(n,timeout_sec=dt)
    settle()
    c1=st["clk"][-1][1]; j1=st["js"][-1][1]
    print(f"{nmsg} msgs @dt={dt}: clock +{c1-c0:.3f}s, {len(st['clk'])-nc} clock msgs, wall {time.time()-t0:.1f}s, dq={np.round(j1-j0,4)}")
test(1,0.05,0.02)
test(1,0.05,0.02)
test(5,0.05,0.02)
test(20,0.02,0.02)
EOF

# openrua op 76
cd /workspace; timeout 60 python3 -u - <<'EOF'
import rclpy, time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
rclpy.init(); n=rclpy.create_node("clk")
v=[]; js=[]
n.create_subscription(Clock,"/clock",lambda m:v.append((round(time.time(),2),m.clock.sec+m.clock.nanosec*1e-9)),10)
n.create_subscription(JointState,"/joint_states",lambda m:js.append((round(time.time(),2),m.header.stamp.sec+m.header.stamp.nanosec*1e-9)),10)
t0=time.time()
while time.time()-t0<12: rclpy.spin_once(n,timeout_sec=0.2)
print("clock:",[(round(a-t0,1),round(b,2)) for a,b in v][:30])
print("js stamps:",[(round(a-t0,1),round(b,2)) for a,b in js][:10])
EOF

# openrua op 77
cd /workspace; cat > servocal.py <<'EOF'
import rclpy, time, numpy as np, yaml
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TwistStamped
tw = next(a for a in yaml.safe_load(open("/workspace/machine.yaml"))["actuators"] if a["kind"] == "cartesian_twist")
rclpy.init(); n=rclpy.create_node("cal")
st={"js":None, "clk":None}
n.create_subscription(JointState,"/joint_states",lambda m:st.__setitem__("js",np.array(m.position[:7])),10)
n.create_subscription(Clock,"/clock",lambda m:st.__setitem__("clk",m.clock.sec+m.clock.nanosec*1e-9),10)
pub=n.create_publisher(TwistStamped,tw["port"],10)
def settle(quiet=3.0):
    """spin until the sim clock value has not changed for `quiet` wall seconds"""
    last=time.time(); c=st["clk"]
    while time.time()-last<quiet:
        rclpy.spin_once(n,timeout_sec=0.2)
        if st["clk"]!=c: c=st["clk"]; last=time.time()
    return c
def test(nmsg, dt, vz):
    c0=settle(); j0=st["js"].copy()
    msg=TwistStamped(); msg.header.frame_id=tw["frame"]; msg.twist.linear.z=vz
    t0=time.time()
    for _ in range(nmsg):
        msg.header.stamp=n.get_clock().now().to_msg(); pub.publish(msg); rclpy.spin_once(n,timeout_sec=dt)
    c1=settle(); j1=st["js"]
    print(f"{nmsg} msgs @dt={dt} vz={vz}: clock +{c1-c0:.3f}s, wall {time.time()-t0:.1f}s, dq={np.round(j1-j0,4)}")
if __name__=="__main__":
    import sys
    for spec in sys.argv[1:]:
        a,b,c=spec.split(","); test(int(a),float(b),float(c))
EOF
timeout 300 python3 -u servocal.py 1,0.05,0.02 1,0.05,0.02 5,0.05,0.02 20,0.02,0.02

# openrua op 78
cd /workspace; timeout 300 python3 -u servocal.py 20,0.05,0.02 20,0.05,0.05 40,0.05,0.05 20,0.05,-0.05

# openrua op 79
cd /workspace; python3 - <<'EOF'
s=open("rob.py").read()
s=s.replace('''from sensor_msgs.msg import JointState
''','''from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
''')
s=s.replace('''        self._js = None
        self._wr = None
''','''        self._js = None
        self._wr = None
        self._clk = None
        self.node.create_subscription(Clock, "/clock", self._on_clk, 10)
''')
s=s.replace('''    def _on_wr(self, m):
        self._wr = m
''','''    def _on_wr(self, m):
        self._wr = m

    def _on_clk(self, m):
        self._clk = m.clock.sec + m.clock.nanosec * 1e-9

    def settle(self, quiet=1.5):
        """spin until the (paused) sim clock has stopped changing for `quiet`
        wall seconds, i.e. the bridge has drained all queued commands"""
        last = time.time()
        c = self._clk
        while time.time() - last < quiet:
            self.spin(0.2)
            if self._clk != c:
                c = self._clk
                last = time.time()
        return c
''')
s=s.replace('''    def servo_to(self, target, speed=0.1, tol=0.004, burst=50, dt=0.02,
                 max_bursts=200, watch_gap=None):
        """closed-loop straight-line TCP move (world frame) via twist bursts"""
        target = np.asarray(target, float)
        for i in range(max_bursts):
            t, _ = self.tcp()
''','''    def burst(self, lin, ang=(0, 0, 0), n=20, dt=0.08):
        """n twist msgs (each = 0.05 s sim) at a rate the bridge keeps up
        with, then wait for the sim to drain so sensor reads are exact"""
        self.servo(lin, ang, ticks=n, dt=dt)
        self.settle()

    def servo_to(self, target, speed=0.05, tol=0.001, n=20, max_bursts=40,
                 watch_gap=None, max_force=None, gain=1.0):
        """closed-loop straight-line TCP move (world frame): bursts with
        exact state read in between. gain = m of motion per (m/s * s) cmd"""
        target = np.asarray(target, float)
        for i in range(max_bursts):
            t, _ = self.tcp()
            if max_force is not None:
                f = self.wrench()
                if f is not None and np.linalg.norm(f - self._f0) > max_force:
                    print(f"servo_to: force {np.round(f,2)} exceeds limit, stopping at {t.round(4)}")
                    return False
''')
s=s.replace('''            v = err / n * min(speed, max(0.02, n * 2))
            self.servo(v, ticks=burst, dt=dt)
        print(f"servo_to: budget exhausted at {t.round(4)}, err {n*1000:.1f} mm")
        return False''','''            # expected motion per burst ~ gain*speed*n*0.05; shrink the
            # burst so we do not overshoot the target
            per = gain * speed * 0.05
            k = int(max(1, min(n, round(dist / per))))
            v = err / dist * speed
            self.burst(v, n=k)
            print(f"  burst {i}: k={k} at {t.round(4)} err {dist*1000:.1f} mm")
        t, _ = self.tcp()
        print(f"servo_to: budget exhausted at {t.round(4)}")
        return False''')
s=s.replace('''            err = target - t
            n = np.linalg.norm(err)
            if n < tol:
                print(f"servo_to reached {t.round(4)} (err {n*1000:.1f} mm)")
                return True''','''            err = target - t
            dist = np.linalg.norm(err)
            if dist < tol:
                print(f"servo_to reached {t.round(4)} (err {dist*1000:.1f} mm)")
                return True''')
s=s.replace('''        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
''','''        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self._f0 = np.zeros(3)  # wrench baseline for servo_to max_force
''')
open("rob.py","w").write(s); print("ok")
EOF
timeout 300 python3 -u - <<'EOF'
import numpy as np, rob
r=rob.Robot(); r.settle()
t0,_=r.tcp(); print("tcp0",t0.round(4))
r.burst([0,0,0.05], n=20)          # 1 s sim at 5 cm/s up
t1,_=r.tcp(); print("tcp1",t1.round(4),"moved",((t1-t0)*1000).round(1),"mm")
r.burst([0,0,-0.05], n=20)
t2,_=r.tcp(); print("tcp2",t2.round(4),"moved",((t2-t1)*1000).round(1),"mm")
r.burst([0.05,0,0], n=10)
t3,_=r.tcp(); print("tcp3",t3.round(4),"moved",((t3-t2)*1000).round(1),"mm")
EOF

# openrua op 80
cd /workspace; python3 - <<'EOF'
s=open("rob.py").read()
s=s.replace('''        self.servo(lin, ang, ticks=n, dt=dt)
        self.settle()''','''        self.servo(lin, ang, ticks=n, dt=dt)
        self.servo((0, 0, 0), ticks=zeros, dt=dt)  # flush servo velocity
        self.settle()''')
s=s.replace("def burst(self, lin, ang=(0, 0, 0), n=20, dt=0.08):","def burst(self, lin, ang=(0, 0, 0), n=20, dt=0.08, zeros=10):")
open("rob.py","w").write(s)
EOF
timeout 400 python3 -u - <<'EOF'
import numpy as np, rob
r=rob.Robot(); r.settle()
t0,_=r.tcp(); print("tcp0",t0.round(4))
for n,v in ((20,[0,0,0.05]),(20,[0,0,-0.05]),(40,[0,0,0.05]),(40,[0,0,-0.05]),(10,[0.05,0,0]),(10,[-0.05,0,0])):
    r.burst(v,n=n); t1,_=r.tcp(); print(f"n={n} v={v}: moved {((t1-t0)*1000).round(1)} mm -> tcp {t1.round(4)}"); t0=t1
EOF

# openrua op 81
cd /workspace; timeout 500 python3 -u - <<'EOF'
import numpy as np, rob
r=rob.Robot(); r.settle()
t0,_=r.tcp(); print("tcp0",t0.round(4))
# flush any pending motion first
r.burst([0,0,0],n=1,zeros=60); t1,_=r.tcp(); print("flush moved",((t1-t0)*1000).round(1)); t0=t1
for n,v,z in ((20,[0,0,0.05],40),(20,[0,0,-0.05],40),(20,[0,0,0.05],60),(20,[0,0,-0.05],60),(10,[0.05,0,0],60),(10,[-0.05,0,0],60)):
    r.burst(v,n=n,zeros=z); t1,_=r.tcp(); print(f"n={n} zeros={z} v={v}: moved {((t1-t0)*1000).round(1)} mm"); t0=t1
EOF

# openrua op 82
cd /workspace; cat > precise.py <<'EOF'
import numpy as np, rob

BASE_Q = np.array([0.7071068, 0.7071068, 0.0, 0.0])  # hand down, hand-y along world x

def hand_quat(yaw_deg):
    return rob.quat_mul(rob.quat_about_z(np.deg2rad(yaw_deg)), BASE_Q)

def yaw_of(quat):
    R = rob.quat_to_R(quat)
    return np.degrees(np.arctan2(R[1, 1], R[0, 1]))

def solve(r, tcp_target, yaw_deg, seed=None):
    """IK for TCP pose, then fix joint7 so hand-y yaw matches"""
    q = r.ik(tcp_target, hand_quat(yaw_deg), seed=seed, at_tcp=True)
    if q is None:
        return None
    for _ in range(3):
        _, quat = r.tcp(q)
        d = ((yaw_of(quat) - yaw_deg + 180) % 360) - 180
        if abs(d) < 0.05:
            break
        q[6] += np.deg2rad(d)  # dj7 = +x rotates hand-y by -x deg
        if q[6] > 2.85: q[6] -= np.pi / 2
        if q[6] < -2.85: q[6] += np.pi / 2
    return q

def goto(r, tcp_target, yaw_deg, seconds=3.0, tol=0.001, tries=6, seed=None):
    """iterate IK + trajectory until the FK TCP is within tol of target"""
    tcp_target = np.asarray(tcp_target, float)
    aim = tcp_target.copy()
    for i in range(tries):
        q = solve(r, aim, yaw_deg, seed=seed)
        if q is None:
            print("IK failed"); return False
        r.move(q, seconds=seconds)
        t, quat = r.tcp()
        err = tcp_target - t
        print(f"  goto {i}: tcp {t.round(4)} err {(err*1000).round(1)} mm yaw {yaw_of(quat):.2f}")
        if np.linalg.norm(err) < tol and abs(((yaw_of(quat) - yaw_deg + 180) % 360) - 180) < 0.3:
            return True
        aim = aim + err * 0.8  # compensate steady-state tracking offset
    return False
EOF
timeout 600 python3 -u - <<'EOF'
import numpy as np, rob, precise
r=rob.Robot(); r.settle()
ok=precise.goto(r,[0.0447,0.0645,1.075],21.5,seconds=4.0)
print("ok",ok,"gap",round(r.finger_gap(),4),"wrench",r.wrench().round(2))
EOF

# openrua op 83
cd /workspace; cat > measure.py <<'EOF'
import numpy as np, rob, cloud, knob, precise

def measure(r, P=None):
    t, quat = r.tcp(); R = rob.quat_to_R(quat)
    if P is None:
        P, _, _ = cloud.grab("robot0_eye_in_hand")
    k = knob.knob_and_flats(t, t[:2], P)
    yaw = precise.yaw_of(quat)
    mis = ((k["flat_normal_deg"] - yaw + 22.5) % 45) - 22.5
    rel = P - t; hx = rel @ R[:, 0]; hy = rel @ R[:, 1]; hz = rel @ R[:, 2]
    fm = (np.abs(hx) < 0.017) & (hz > -0.06) & (hz < 0.015) & np.isfinite(P).all(-1) & (np.abs(hy) > 0.02)
    fl = -np.percentile(np.abs(hy[fm & (hy < 0)]), 1); fr = np.percentile(np.abs(hy[fm & (hy > 0)]), 1)
    lid = P[(np.abs(P[..., 0] - t[0]) < 0.08) & (np.abs(P[..., 1] - t[1]) < 0.08) & (P[..., 2] > k["z_top"] - 0.032) & (P[..., 2] < k["z_top"] - 0.012) & np.isfinite(P).all(-1)]
    pr = (lid[:, :2] - t[:2]) @ R[:2, 1]   # lid extent along closing axis, rel TCP
    px = (lid[:, :2] - t[:2]) @ R[:2, 0]
    off = k["knob_centre"] - t[:2]
    print(f"tcp {t.round(4)} yaw {yaw:.2f}  knob-TCP {(off*1000).round(1)} mm (hand x {1000*off@R[:2,0]:.1f}, hand y {1000*off@R[:2,1]:.1f})")
    print(f"knob top z {k['z_top']:.4f}  flat normal {k['flat_normal_deg']} -> misalign {mis:.2f} deg  ftf {k['ftf']*1000:.1f} vtv {k['vtv']*1000:.1f} mm")
    print(f"fingers hy {fl*1000:.1f}..{fr*1000:.1f}  lid along hy {pr.min()*1000:.1f}..{pr.max()*1000:.1f}  clearance {1000*(pr.min()-fl):.1f}/{1000*(fr-pr.max()):.1f} mm   lid along hx {px.min()*1000:.1f}..{px.max()*1000:.1f}")
    return dict(tcp=t, yaw=yaw, off=off, mis=mis, k=k, fl=fl, fr=fr, pr=(pr.min(), pr.max()))
EOF
timeout 120 python3 -u -c "
import rob, measure; r=rob.Robot(); r.settle(); measure.measure(r)"

# openrua op 84
cd /workspace; timeout 900 python3 -u - <<'EOF'
import numpy as np, rob, precise
r=rob.Robot(); r.settle()
f0=r.wrench(); print("baseline wrench",f0.round(2))
x,y=0.0446,0.0641   # keep the achieved xy (knob 0.5 mm off; negligible)
for z in (1.050, 1.038, 1.028, 1.018):
    ok=precise.goto(r,[x,y,z],21.5,seconds=3.0,tol=0.0012)
    f=r.wrench(); t,_=r.tcp()
    print(f"z={z}: ok={ok} tcp {t.round(4)} dF {(f-f0).round(2)} gap {r.finger_gap():.4f}")
    if np.linalg.norm(f-f0)>4: print("CONTACT - stopping"); break
EOF

# openrua op 85
cd /workspace; timeout 300 python3 -u - <<'EOF'
import numpy as np, rob
r=rob.Robot(); r.settle()
res=r.gripper(0.0)
r.settle()
j=r.joints(); print("fingers", round(j["panda_finger_joint1"],4), round(j["panda_finger_joint2"],4), "gap", round(r.finger_gap(),4))
print("wrench", r.wrench().round(2)); t,_=r.tcp(); print("tcp",t.round(4))
EOF

# openrua op 86
cd /workspace; timeout 600 python3 -u - <<'EOF'
import numpy as np, rob, precise
r=rob.Robot(); r.settle()
g0=r.finger_gap()
ok=precise.goto(r,[0.0446,0.0643,1.048],21.5,seconds=3.0,tol=0.0015)
r.settle(); t,_=r.tcp()
print("tcp",t.round(4),"gap",round(r.finger_gap(),4),"(was",round(g0,4),") wrench",r.wrench().round(2))
EOF
python3 tools/perception/cam_snap.py sideview >/dev/null && python3 tools/perception/cam_snap.py agentview >/dev/null && echo snaps

# openrua op 87
cd /workspace; timeout 120 python3 -u - <<'EOF'
import numpy as np, cloud
P,rgb,d=cloud.grab("birdview")
m=np.isfinite(P).all(-1)&(np.abs(P[...,0]+0.054)<0.15)&(np.abs(P[...,1]+0.204)<0.15)
pts=P[m]; z=pts[:,2]
for lo in np.arange(0.89,1.0,0.01):
    s=(z>=lo)&(z<lo+0.01)
    if s.sum()>50: print(f"z {lo:.2f}-{lo+.01:.2f}: n={s.sum():6d} x {pts[s,0].min():+.3f}..{pts[s,0].max():+.3f} y {pts[s,1].min():+.3f}..{pts[s,1].max():+.3f}")
# stove top plate: points between 0.915 and 0.945
s=(z>0.915)&(z<0.945); sp=pts[s]
print("plate centre (mean)",sp[:,:2].mean(0).round(4),"extent x",sp[:,0].min().round(3),sp[:,0].max().round(3),"y",sp[:,1].min().round(3),sp[:,1].max().round(3))
c=rgb[m][s]; red=(c[:,0]>120)&(c[:,1]<80)&(c[:,2]<80)
if red.sum(): print("red ring centre",sp[red][:,:2].mean(0).round(4),"n",red.sum(), "z",sp[red][:,2].mean().round(4))
EOF

# openrua op 88
cd /workspace; timeout 900 python3 -u - <<'EOF'
import numpy as np, rob, precise
r=rob.Robot(); r.settle()
for tgt,sec in (([0.0445,0.0643,1.10],3.0), ([-0.058,-0.204,1.10],5.0)):
    ok=precise.goto(r,tgt,21.5,seconds=sec,tol=0.0015)
    r.settle(); t,_=r.tcp()
    print("->",t.round(4),"gap",round(r.finger_gap(),4),"wrench",r.wrench().round(2))
EOF
python3 tools/perception/cam_snap.py sideview >/dev/null && python3 tools/perception/cam_snap.py birdview >/dev/null && echo snaps

# openrua op 89
cd /workspace; timeout 900 python3 -u - <<'EOF'
import numpy as np, rob, precise
r=rob.Robot(); r.settle()
x,y=-0.0579,-0.2036
for z in (1.075, 1.062, 1.056, 1.051, 1.046, 1.041, 1.036):
    ok=precise.goto(r,[x,y,z],21.5,seconds=2.5,tol=0.0015,tries=3)
    r.settle(); f=r.wrench(); t,_=r.tcp()
    print(f"z={z}: tcp {t.round(4)} wrench {f.round(2)} gap {r.finger_gap():.4f}")
    if f[2] > -4.6: print("pot supported by burner (Fz rose) - stop"); break
EOF

# openrua op 90
cd /workspace; timeout 600 python3 -u - <<'EOF'
import numpy as np, rob, precise
r=rob.Robot(); r.settle()
r.gripper(0.08); r.settle()
print("gap",round(r.finger_gap(),4),"wrench",r.wrench().round(2))
ok=precise.goto(r,[-0.058,-0.2035,1.12],21.5,seconds=3.0,tol=0.002,tries=3)
r.settle(); t,_=r.tcp(); print("retreated tcp",t.round(4),"wrench",r.wrench().round(2))
ok=precise.goto(r,[-0.058,-0.2035,1.20],21.5,seconds=3.0,tol=0.003,tries=2)
r.settle(); t,_=r.tcp(); print("up tcp",t.round(4))
EOF
for c in sideview agentview birdview frontview; do python3 tools/perception/cam_snap.py $c >/dev/null; done; echo snaps

# openrua op 91
cd /workspace; timeout 120 python3 -u - <<'EOF'
import numpy as np, cloud, rob
P,rgb,d=cloud.grab("birdview")
m=np.isfinite(P).all(-1)&(np.abs(P[...,0]+0.058)<0.10)&(np.abs(P[...,1]+0.204)<0.10)&(P[...,2]>0.95)&(P[...,2]<1.12)
pts=P[m]; print("pot on stove: n",len(pts),"top z",pts[:,2].max().round(4))
lid=pts[pts[:,2]>pts[:,2].max()-0.045]; print("lid+knob centre xy",lid[:,:2].mean(0).round(4),"z range",lid[:,2].min().round(3),lid[:,2].max().round(3))
# pot B still at original place?
m2=np.isfinite(P).all(-1)&(P[...,2]>0.95)&(P[...,2]<1.12)&(P[...,1]>0.0)
print("other pot top z",P[m2][:,2].max().round(3),"centre",P[m2][:,:2].mean(0).round(3))
r=rob.Robot(); c=r.settle(); print("sim clock settled at",c,"arm q",r.arm_q().round(3),"gap",round(r.finger_gap(),4))
EOF

# openrua op 92
cd /workspace; timeout 120 python3 -u - <<'EOF'
import numpy as np, rob
r=rob.Robot(); r.joints(); c=r.settle(3.0); q=r.arm_q(); t,_=r.tcp()
print("clock",c,"q",q.round(3),"tcp",t.round(4),"gap",round(r.finger_gap(),4),"wrench",r.wrench().round(2))
EOF
