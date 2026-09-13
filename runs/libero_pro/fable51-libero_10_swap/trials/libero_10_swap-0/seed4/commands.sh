#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 service list | grep -v parameter | head -40; echo ---; timeout 20 ros2 node list

# openrua op 4
timeout 30 ros2 topic echo /joint_states --once; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('robot0_robotview.png')
cv2.imwrite('crop1.png', cv2.resize(im[80:200,10:130],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop2.png', cv2.resize(im[0:130,150:240],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('agentview.png')
cv2.imwrite('crop3.png', cv2.resize(im[190:300,190:260],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
timeout 60 python3 tools/perception/px2world.py birdview 385 290; timeout 60 python3 tools/perception/px2world.py birdview 283 232; timeout 30 ros2 topic echo /birdview/color/camera_info --once | head -20; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8

# openrua op 7
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a world-frame point cloud from a camera's depth frame.

Usage: python3 cloud.py <camera>  -> <camera>_cloud.npz (xyz: HxWx3 world, rgb)
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=20.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("cloud")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    rgb = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "rgb8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1) @ R.T + T
    np.savez(f"{cam}_cloud.npz", xyz=pc, rgb=rgb, depth=depth)
    print("cam pos", T, "R", R.round(3).tolist())
    z = pc[..., 2]
    ok = np.isfinite(z)
    print("z range", np.nanmin(z[ok]), np.nanmax(z[ok]))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); z=d['xyz'][...,2]
import collections
h=np.round(z,2); vals,counts=np.unique(h[np.isfinite(h)],return_counts=True)
top=sorted(zip(counts,vals),reverse=True)[:8]; print('birdview common z:',top)
"

# openrua op 9
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']; z=xyz[...,2]
mask=(np.isfinite(z)&(z>0.445)&(z<1.0)).astype('uint8')
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i; p=xyz[m]
    print(f'blob{i} px={stats[i,4]} pixc=({cent[i][0]:.0f},{cent[i][1]:.0f}) x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} zmed={np.median(p[:,2]):.3f} rgb={rgb[m].mean(0).astype(int)}')
"

# openrua op 10
timeout 120 python3 cloud.py robot0_robotview; python3 -c "
import numpy as np, cv2
for cam in ['agentview','robot0_robotview']:
    d=np.load(cam+'_cloud.npz'); xyz=d['xyz']; rgb=d['rgb']; z=xyz[...,2]
    mask=(np.isfinite(z)&(z>0.46)&(z<1.0)&(xyz[...,0]>-0.45)).astype('uint8')
    n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
    print('==',cam)
    for i in range(1,n):
        if stats[i,4]<30: continue
        m=lab==i; p=xyz[m]
        print(f'blob{i} px={stats[i,4]} pixc=({cent[i][0]:.0f},{cent[i][1]:.0f}) x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} rgb={rgb[m].mean(0).astype(int)}')
"

# openrua op 11
python3 -c "
import numpy as np
for cam in ['agentview','robot0_robotview','birdview']:
    d=np.load(cam+'_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
    m=np.isfinite(z)&(z>0.46)&(xyz[...,0]>-0.30)&(xyz[...,0]<-0.19)&(xyz[...,1]>-0.22)&(xyz[...,1]<-0.10)
    p=xyz[m]
    if len(p): print(cam,'alphabet soup pts',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'ztop',p[:,2].max().round(3))
    # top slice for cans
    for name,(x0,x1,y0,y1) in {'soup':(-0.30,-0.19,-0.22,-0.10),'sauce':(-0.05,0.06,-0.30,-0.20)}.items():
        m=np.isfinite(z)&(xyz[...,0]>x0)&(xyz[...,0]<x1)&(xyz[...,1]>y0)&(xyz[...,1]<y1)
        p=xyz[m]; 
        if not len(p): continue
        top=p[p[:,2]>p[:,2].max()-0.01]
        print('  ',name,'top slice n',len(top),'center',top[:,:2].mean(0).round(3),'ztop',p[:,2].max().round(3))
"

# openrua op 12
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable arm helpers: one node, persistent clients (IK, FK, FJT, gripper).

World <-> base: panda_link0 sits at world (-0.51, 0, 0.42), identity rotation.
All public functions take WORLD coordinates for the TCP (fingertip point).
"""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM_JOINTS = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])
# top-down grasp: hand z down, hand x along world +x (fingers close along world y)
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def yaw_q(yaw):
    """Top-down orientation rotated by yaw about world z (fingers close along
    the axis at yaw+90deg)."""
    # q = Rz(yaw) * (1,0,0,0)
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    # Rz(yaw) = (0,0,s,c); product (0,0,s,c)*(1,0,0,0)
    return (c, s, 0.0, 0.0)


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_ctl")
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        assert self.ik.wait_for_service(10), "no IK"
        assert self.fk.wait_for_service(10), "no FK"
        assert self.fjt.wait_for_server(10), "no FJT"
        assert self.grip.wait_for_server(10), "no gripper"
        self.spin_until(lambda: self._js is not None, 10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin_until(self, cond, timeout):
        end = time.time() + timeout
        while not cond() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return cond()

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            self.spin_until(lambda: self._js is not None, 10)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM_JOINTS]

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"] + abs(d["panda_finger_joint2"])

    def wrench(self):
        self._wr = None
        self.spin_until(lambda: self._wr is not None, 5)
        f = self._wr.wrench.force
        return (f.x, f.y, f.z)

    # ---- kinematics -------------------------------------------------
    def fk_hand_world(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM_JOINTS
        req.robot_state.joint_state.position = q or self.arm_q()
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w), r.pose_stamped[0].header.frame_id

    def ik_tcp_world(self, xyz, quat=Q_DOWN, seed=None):
        """IK for the TCP at world xyz with the given hand orientation.
        Returns joint list or None."""
        R = quat_R(*quat)
        hand = np.array(xyz) - TCP_OFF * R[:, 2]   # TCP is +Z of hand
        base = hand - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        req.ik_request.robot_state.joint_state.name = ARM_JOINTS
        req.ik_request.robot_state.joint_state.position = seed or self.arm_q()
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print("IK failed", None if r is None else r.error_code.val, "for", xyz)
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM_JOINTS]

    # ---- motion -----------------------------------------------------
    def move_joints(self, waypoints, seconds):
        """waypoints: list of joint lists; seconds: list of cumulative times."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        for q, t in zip(waypoints, seconds):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = max(abs(a - b) for a, b in zip(self.arm_q(), waypoints[-1]))
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, xyz, quat=Q_DOWN, seconds=3.0, via=None):
        """IK then trajectory. via: optional list of intermediate TCP xyz."""
        seed = self.arm_q()
        wps, ts = [], []
        pts = (via or []) + [list(xyz)]
        for i, p in enumerate(pts):
            q = self.ik_tcp_world(p, quat, seed)
            if q is None:
                return None
            wps.append(q)
            seed = q
            ts.append(seconds * (i + 1) / len(pts))
        code, err = self.move_joints(wps, ts)
        pos, _, _ = self.fk_hand_world()
        R = quat_R(*quat)
        tcp = pos + TCP_OFF * R[:, 2]
        print(f"  tcp now {tcp.round(4)} target {np.array(xyz).round(4)}")
        return tcp

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        gap = self.finger_gap()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}")
        return gap


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
OPENRUA_EOF

# openrua op 13
timeout 120 python3 -u -c "
from arm import *
a=Arm()
print('arm q', np.round(a.arm_q(),3), 'gap', a.finger_gap())
pos,q,fid=a.fk_hand_world()
print('FK frame', repr(fid), 'hand world', pos.round(4), 'quat', np.round(q,4))
R=quat_R(*q); print('hand z axis in world', R[:,2].round(3), 'hand x', R[:,0].round(3))
print('tcp world', (pos+TCP_OFF*R[:,2]).round(4))
print('wrench', a.wrench())
# test IK for pre-grasp above the soup can
q1=a.ik_tcp_world((-0.237,-0.161,0.62))
print('IK soup pregrasp', None if q1 is None else np.round(q1,3))
q2=a.ik_tcp_world((0.008,-0.251,0.64))
print('IK sauce pregrasp', None if q2 is None else np.round(q2,3))
q3=a.ik_tcp_world((0.005,0.25,0.75))
print('IK basket', None if q3 is None else np.round(q3,3))
" 2>&1 | grep -v '^\[' 

# openrua op 14
timeout 120 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v '^\['; timeout 20 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A2 Translation | head -3; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A2 Translation | head -3

# openrua op 15
timeout 120 python3 -u -c "
from arm import *
import arm
a=Arm()
cur=a.arm_q()
# bypass TCP/base offsets: test raw hand pose
arm.BASE_IN_WORLD=np.zeros(3); arm.TCP_OFF=0.0
for label,p in [('world',(-0.053,0,0.778)),('base',(0.457,0,0.358))]:
    q=a.ik_tcp_world(p,(1.0,0,-0.028,0))
    print(label, None if q is None else np.round(q,3), 'cur', np.round(cur,3))
" 2>&1 | grep -v '^\['

# openrua op 16
timeout 120 python3 -u -c "
from arm import *
import arm
a=Arm()
cur=a.arm_q()
arm.BASE_IN_WORLD=np.zeros(3); arm.TCP_OFF=0.0
for label,p in [('world',(-0.053,0,0.778)),('base',(0.457,0,0.358))]:
    q=a.ik_tcp_world(p,(1.0,0.0,-0.028,0.0))
    print(label, None if q is None else np.round(q,3), 'cur', np.round(cur,3))
" 2>&1 | grep -v '^\['

# openrua op 17
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])", "new_string": "# Verified live: /compute_fk and /compute_ik (empty frame_id) both use the\\n# WORLD frame on this machine (FK matched tf world->panda_hand), so no\\n# base offset is applied. panda_link0 sits at world (-0.51, 0, 0.42).\\nBASE_IN_WORLD = np.zeros(3)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 18
timeout 180 python3 -u -c "
from arm import *
a=Arm()
for name,p in [('soup pre',(-0.237,-0.161,0.62)),('soup grasp',(-0.237,-0.161,0.465)),('sauce pre',(0.008,-0.251,0.64)),('sauce grasp',(0.008,-0.251,0.475)),('basket',(0.005,0.25,0.75))]:
    q=a.ik_tcp_world(p)
    print(name, None if q is None else np.round(q,3))
    if q is not None:
        pos,quat,_=a.fk_hand_world(q); R=quat_R(*quat); print('   check tcp', (pos+TCP_OFF*R[:,2]).round(4), 'hand z', R[:,2].round(3))
" 2>&1 | grep -v '^\['

# openrua op 19
timeout 180 python3 -u -c "
from arm import *
a=Arm()
print('gap before', a.finger_gap())
a.gripper(0.04)
print('gap after open', a.finger_gap())
" 2>&1 | grep -v '^\['

# openrua op 20
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick one object top-down and place it in the basket, verifying each step.

Usage: python3 -u pick_place.py <name> <x> <y> <z_top> <grasp_tcp_z> [--stop-after-pre]
"""
import subprocess
import sys

import numpy as np

from arm import Arm, Q_DOWN

BASKET = (0.005, 0.25)
CARRY_Z = 0.76           # TCP height while carrying (basket rim is 0.63)
BASKET_DROP_Z = 0.72     # TCP height when releasing over basket


def snap(cam, out):
    subprocess.run([sys.executable, "/workspace/tools/perception/cam_snap.py", cam, out],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120)


def main():
    name, x, y, z_top, gz = sys.argv[1], *map(float, sys.argv[2:6])
    stop_pre = "--stop-after-pre" in sys.argv
    a = Arm()
    pre_z = z_top + 0.13
    print(f"[{name}] target ({x:.3f},{y:.3f}) top={z_top:.3f} grasp_z={gz:.3f}")

    print("1. open gripper")
    gap = a.gripper(0.04)
    assert gap > 0.07, "gripper did not open"

    print("2. move to pre-grasp")
    tcp = a.move_tcp((x, y, pre_z), Q_DOWN, seconds=4.0)
    assert tcp is not None and np.linalg.norm(tcp - (x, y, pre_z)) < 0.01, "pre-grasp not reached"
    snap("robot0_eye_in_hand", f"{name}_pre_eih.png")
    print("   saved", f"{name}_pre_eih.png")
    if stop_pre:
        return

    print("3. descend")
    steps = [(x, y, z) for z in np.linspace(pre_z, gz, 4)[1:-1]]
    tcp = a.move_tcp((x, y, gz), Q_DOWN, seconds=3.0, via=steps)
    assert tcp is not None and np.linalg.norm(tcp - (x, y, gz)) < 0.01, "grasp pose not reached"
    print("   wrench", np.round(a.wrench(), 2))

    print("4. close gripper")
    gap = a.gripper(0.0)
    print(f"   gap={gap:.4f} (>0.02 means something is held)")
    assert gap > 0.02, "closed on air"

    print("5. lift")
    tcp = a.move_tcp((x, y, CARRY_Z), Q_DOWN, seconds=3.0, via=[(x, y, gz + 0.08)])
    gap = a.finger_gap()
    print(f"   gap after lift={gap:.4f}")
    assert gap > 0.02, "object dropped during lift"
    snap("agentview", f"{name}_lifted.png")

    print("6. move over basket")
    tcp = a.move_tcp((BASKET[0], BASKET[1], CARRY_Z), Q_DOWN, seconds=4.0)
    assert tcp is not None and np.linalg.norm(tcp - (*BASKET, CARRY_Z)) < 0.01, "basket pose not reached"
    gap = a.finger_gap()
    print(f"   gap over basket={gap:.4f}")
    assert gap > 0.02, "object dropped in transit"
    snap("robot0_eye_in_hand", f"{name}_over_basket_eih.png")

    print("7. lower a bit and release")
    a.move_tcp((BASKET[0], BASKET[1], BASKET_DROP_Z), Q_DOWN, seconds=2.0)
    gap = a.gripper(0.04)
    print(f"   gap after open={gap:.4f}")

    print("8. retreat up")
    a.move_tcp((BASKET[0], BASKET[1], 0.85), Q_DOWN, seconds=2.0)
    snap("agentview", f"{name}_placed.png")
    snap("robot0_eye_in_hand", f"{name}_placed_eih.png")
    print(f"[{name}] DONE")


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 21
timeout 600 python3 -u pick_place.py sauce 0.008 -0.251 0.522 0.45 --stop-after-pre 2>&1 | grep -v '^\['

# openrua op 22
timeout 120 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v '^\[' && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
m=np.isfinite(z)&(z>0.505)&(z<0.54)&(xyz[...,0]>-0.06)&(xyz[...,0]<0.08)&(xyz[...,1]>-0.32)&(xyz[...,1]<-0.18)
p=xyz[m]; print('can top n',len(p),'center',p[:,:2].mean(0).round(4),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].mean().round(4))
vv,uu=np.where(m); print('pix center',uu.mean().round(0),vv.mean().round(0))
"

# openrua op 23
timeout 900 python3 -u pick_place.py sauce 0.008 -0.250 0.511 0.45 2>&1 | grep -v '^\['

# openrua op 24
timeout 600 python3 -u -c "
from arm import *
a=Arm()
print('wrench', np.round(a.wrench(),2), 'gap', round(a.finger_gap(),4))
pos,q,_=a.fk_hand_world(); R=quat_R(*q); print('tcp', (pos+TCP_OFF*R[:,2]).round(4), 'hand x', R[:,0].round(3))
tcp=a.move_tcp((0.008,-0.25,0.45), Q_DOWN, seconds=2.0)
print('wrench', np.round(a.wrench(),2))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand sauce_grasp_eih.png; timeout 60 python3 tools/perception/cam_snap.py agentview sauce_grasp_agent.png

# openrua op 25
timeout 120 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v '^\[' && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]; dep=d['depth']
m=np.isfinite(z)&(z>0.50)&(z<0.53)&(xyz[...,0]>-0.08)&(xyz[...,0]<0.09)&(xyz[...,1]>-0.33)&(xyz[...,1]<-0.17)
p=xyz[m]; print('can top n',len(p),'center',p[:,:2].mean(0).round(4),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].mean().round(4))
# fingers: points very close to camera
f=np.isfinite(dep)&(dep<0.15); pf=xyz[f]; print('near pts n',len(pf), 'z range', pf[:,2].min().round(3) if len(pf) else None, pf[:,2].max().round(3) if len(pf) else None)
import cv2
lab=(f).astype('uint8'); n,l,s,c=cv2.connectedComponentsWithStats(lab)
for i in range(1,n):
    if s[i,4]<50: continue
    q=xyz[l==i]; print(' finger blob px',s[i,4],'xy',q[:,:2].mean(0).round(4),'zmin',q[:,2].min().round(3))
"

# openrua op 26
timeout 600 python3 -u -c "
from arm import *
a=Arm()
tcp=a.move_tcp((0.008,-0.25,0.65), Q_DOWN, seconds=2.5)
print('wrench', np.round(a.wrench(),2))
" 2>&1 | grep -v '^\['; timeout 120 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v '^\[' && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
m=np.isfinite(z)&(z>0.50)&(z<0.53)&(xyz[...,0]>-0.08)&(xyz[...,0]<0.09)&(xyz[...,1]>-0.33)&(xyz[...,1]<-0.17)
p=xyz[m]; print('can top n',len(p),'center',p[:,:2].mean(0).round(4),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].mean().round(4))
"; timeout 30 ros2 service call /get_urdf std_srvs/srv/Trigger 2>/dev/null | head -c 300; echo; timeout 30 ros2 service type /get_urdf

# openrua op 27
timeout 60 python3 -c "
import rclpy
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
rclpy.init(); n=rclpy.create_node('urdf')
got=[]
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(String,'/robot_description',got.append,qos)
while not got: rclpy.spin_once(n,timeout_sec=0.5)
open('robot.urdf','w').write(got[0].data); print(len(got[0].data))
" 2>&1 | grep -v '^\['; grep -n -A12 'joint name="panda_hand_joint"\|joint name="panda_finger_joint1"\|joint name="panda_joint8"\|link name="panda_leftfinger"\|link name="panda_hand"' robot.urdf | grep -v '^\s*$' | head -120

# openrua op 28
timeout 120 python3 -u -c "
from arm import *
a=Arm()
q=a.arm_q(); print('q', np.round(q,3))
pos,quat,_=a.fk_hand_world(); R=quat_R(*quat)
print('FK hand pos', pos.round(4), 'quat', np.round(quat,4)); print('hand x', R[:,0].round(3), 'hand y', R[:,1].round(3), 'hand z', R[:,2].round(3))
" 2>&1 | grep -v '^\['; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | grep -A1 Translation | head -2; timeout 20 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 Translation | head -4;  timeout 20 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 Translation | head -4

# openrua op 29
timeout 300 python3 -u -c "
from arm import *
a=Arm()
import math
for yaw in [0, math.pi/4, -math.pi/4, math.pi/2]:
    quat=yaw_q(yaw)
    for tgt in [(0.008,-0.25,0.65), (-0.237,-0.161,0.62)]:
        q=a.ik_tcp_world(tgt, quat)
        if q is None: continue
        pos,fq,_=a.fk_hand_world(q); R=quat_R(*fq)
        print(f'yaw={yaw:+.3f} tgt={tgt} req={np.round(quat,3)} got={np.round(fq,3)} handx={R[:,0].round(2)} j7={q[6]:.3f}')
" 2>&1 | grep -v '^\['

# openrua op 30
for c in agentview frontview sideview; do timeout 120 python3 cloud.py $c 2>&1 | grep -v '^\[' | head -1; done; python3 -c "
import numpy as np
for cam in ['agentview','frontview','sideview']:
    d=np.load(cam+'_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
    m=np.isfinite(z)&(z>0.53)&(z<0.80)&(np.abs(xyz[...,0]-0.008)<0.06)&(np.abs(xyz[...,1]+0.25)<0.06)
    p=xyz[m]
    if len(p)==0: print(cam,'none'); continue
    p=p[np.argsort(p[:,2])]
    print(cam,'n',len(p),'lowest z',p[:10,2].round(3),'at xy',p[:3,:2].round(3))
"

# openrua op 31
timeout 600 python3 -u -c "
from arm import *
a=Arm()
print('wrench', np.round(a.wrench(),2))
a.move_tcp((0.10,-0.33,0.60), Q_DOWN, seconds=3.0)
for z in [0.52,0.49,0.47,0.45,0.435]:
    tcp=a.move_tcp((0.10,-0.33,z), Q_DOWN, seconds=1.5)
    print('   z target',z,'wrench', np.round(a.wrench(),2))
" 2>&1 | grep -v '^\['

# openrua op 32
timeout 120 python3 cloud.py agentview 2>&1 | grep -v '^\[' | head -1; timeout 120 python3 cloud.py frontview 2>&1 | grep -v '^\[' | head -1; python3 -c "
import numpy as np
for cam in ['agentview','frontview']:
    d=np.load(cam+'_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
    m=np.isfinite(z)&(z>0.43)&(z<0.80)&(np.abs(xyz[...,0]-0.115)<0.07)&(np.abs(xyz[...,1]+0.343)<0.07)
    p=xyz[m]; p=p[np.argsort(p[:,2])]
    print(cam,'n',len(p),'lowest z',p[:8,2].round(3))
    # table height near there
    t=np.isfinite(z)&(z<0.43)&(np.abs(xyz[...,0]-0.115)<0.1)&(np.abs(xyz[...,1]+0.343)<0.1)
    print('  table z median', np.median(xyz[t][:,2]).round(3))
"

# openrua op 33
timeout 60 python3 tools/perception/cam_snap.py frontview floor_front.png >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py agentview floor_agent.png >/dev/null 2>&1; python3 -c "
import cv2
im=cv2.imread('floor_agent.png'); cv2.imwrite('floor_agent_crop.png', cv2.resize(im[200:420,0:260],None,fx=2.5,fy=2.5,interpolation=cv2.INTER_CUBIC))"

# openrua op 34
timeout 300 python3 -u -c "
from arm import *
a=Arm()
cur=a.arm_q(); print('current q', np.round(cur,3))
q=a.ik_tcp_world((0.10,-0.33,0.435), Q_DOWN, seed=cur); print('IK z=0.435 q', np.round(q,3))
q=a.ik_tcp_world((0.10,-0.33,0.47), Q_DOWN, seed=cur); print('IK z=0.47  q', np.round(q,3))
" 2>&1 | grep -v '^\['

# openrua op 35
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        R = quat_R(*quat)\\n        hand = np.array(xyz) - TCP_OFF * R[:, 2]   # TCP is +Z of hand\\n        base = hand - BASE_IN_WORLD", "new_string": "        R = quat_R(*quat)\\n        hand = np.array(xyz) - TCP_OFF * R[:, 2]   # TCP is +Z of hand\\n        base = hand - BASE_IN_WORLD\\n        # Verified live: /compute_ik solves for a tip link yawed -45deg from\\n        # panda_hand (link8), so the resulting panda_hand orientation comes\\n        # out +45deg about its z from the request. Pre-rotate by -45deg.\\n        quat = quat_mul(quat, (0.0, 0.0, -math.sin(math.pi / 8), math.cos(math.pi / 8)))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 36
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "def quat_R(x, y, z, w):", "new_string": "def quat_mul(a, b):\\n    \\"\\"\\"Hamilton product a*b, quaternions as (x, y, z, w).\\"\\"\\"\\n    ax, ay, az, aw = a\\n    bx, by, bz, bw = b\\n    return (aw * bx + ax * bw + ay * bz - az * by,\\n            aw * by - ax * bz + ay * bw + az * bx,\\n            aw * bz + ax * by - ay * bx + az * bw,\\n            aw * bw - ax * bx - ay * by - az * bz)\\n\\n\\ndef quat_R(x, y, z, w):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 37
timeout 300 python3 -u -c "
from arm import *
a=Arm()
q=a.ik_tcp_world((0.10,-0.33,0.60), Q_DOWN)
pos,fq,_=a.fk_hand_world(q); R=quat_R(*fq); print('req Q_DOWN -> hand quat', np.round(fq,3), 'hand x', R[:,0].round(2), 'hand y', R[:,1].round(2))
a.move_tcp((0.10,-0.33,0.60), Q_DOWN, seconds=2.5)
" 2>&1 | grep -v '^\['

# openrua op 38
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        quat = quat_mul(quat, (0.0, 0.0, -math.sin(math.pi / 8), math.cos(math.pi / 8)))", "new_string": "        quat = quat_mul(quat, (0.0, 0.0, math.sin(math.pi / 8), math.cos(math.pi / 8)))", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 39
timeout 300 python3 -u -c "
from arm import *
a=Arm()
for yaw in [0, math.pi/2]:
    q=a.ik_tcp_world((0.10,-0.33,0.60), yaw_q(yaw))
    pos,fq,_=a.fk_hand_world(q); R=quat_R(*fq); print(f'yaw {yaw:.2f} -> hand quat', np.round(fq,3), 'hand x', R[:,0].round(2), 'hand y', R[:,1].round(2))
a.move_tcp((0.10,-0.33,0.60), Q_DOWN, seconds=2.5)
" 2>&1 | grep -v '^\['

# openrua op 40
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        code = res.result().result.error_code\\n        err = max(abs(a - b) for a, b in zip(self.arm_q(), waypoints[-1]))\\n        print(f\\"  traj done code={code} max_joint_err={err:.4f}\\")\\n        return code, err", "new_string": "        code = res.result().result.error_code\\n        # the result can arrive while the arm is still settling: re-read\\n        # until two consecutive joint reads agree\\n        prev = self.arm_q()\\n        for _ in range(20):\\n            cur = self.arm_q()\\n            if max(abs(a - b) for a, b in zip(cur, prev)) < 1e-3:\\n                break\\n            prev = cur\\n        err = max(abs(a - b) for a, b in zip(cur, waypoints[-1]))\\n        print(f\\"  traj done code={code} max_joint_err={err:.4f}\\")\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 41
timeout 900 python3 -u pick_place.py sauce 0.0086 -0.2498 0.511 0.48 2>&1 | grep -v '^\['

# openrua op 42
timeout 300 python3 -u -c "
from arm import *
a=Arm()
pos,fq,_=a.fk_hand_world(); R=quat_R(*fq); print('tcp', (pos+TCP_OFF*R[:,2]).round(4), 'gap', round(a.finger_gap(),4), 'wrench', np.round(a.wrench(),2))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py agentview transit.png >/dev/null 2>&1

# openrua op 43
mkdir -p "$(dirname /workspace/place.py)"
cat > /workspace/place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Finish a place: carry the held object over the basket, release, retreat.
Usage: python3 -u place.py <name>"""
import subprocess
import sys

import numpy as np

from arm import Arm, Q_DOWN

BASKET = (0.005, 0.25)
CARRY_Z, DROP_Z = 0.76, 0.72


def snap(cam, out):
    subprocess.run([sys.executable, "/workspace/tools/perception/cam_snap.py", cam, out],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=120)


def goto(a, xyz, seconds, tries=3):
    for i in range(tries):
        tcp = a.move_tcp(xyz, Q_DOWN, seconds=seconds)
        if tcp is not None and np.linalg.norm(tcp - xyz) < 0.01:
            return tcp
        print(f"   retry {i + 1}: off by {np.linalg.norm(tcp - xyz):.3f}")
    raise SystemExit("could not reach " + str(xyz))


def main():
    name = sys.argv[1]
    a = Arm()
    assert a.finger_gap() > 0.02, "nothing held"
    print("6. move over basket")
    goto(a, (*BASKET, CARRY_Z), 4.0)
    gap = a.finger_gap()
    print(f"   gap over basket={gap:.4f}")
    assert gap > 0.02, "object dropped in transit"
    snap("robot0_eye_in_hand", f"{name}_over_basket_eih.png")
    print("7. lower and release")
    goto(a, (*BASKET, DROP_Z), 2.0)
    gap = a.gripper(0.04)
    print(f"   gap after open={gap:.4f}")
    print("8. retreat up")
    goto(a, (*BASKET, 0.86), 2.5)
    snap("agentview", f"{name}_placed.png")
    snap("robot0_eye_in_hand", f"{name}_placed_eih.png")
    print(f"[{name}] PLACED")


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 44
timeout 900 python3 -u place.py sauce 2>&1 | grep -v '^\['

# openrua op 45
timeout 600 python3 -u pick_place.py soup -0.237 -0.161 0.501 0.48 --stop-after-pre 2>&1 | grep -v '^\['; timeout 120 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v '^\[' | head -1; python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
m=np.isfinite(z)&(z>0.49)&(z<0.515)&(xyz[...,0]>-0.30)&(xyz[...,0]<-0.18)&(xyz[...,1]>-0.23)&(xyz[...,1]<-0.10)
p=xyz[m]; print('soup top n',len(p),'center',p[:,:2].mean(0).round(4),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].mean().round(4))
# ketchup extents
m=np.isfinite(z)&(z>0.46)&(xyz[...,0]>-0.20)&(xyz[...,0]<-0.08)&(xyz[...,1]>-0.22)&(xyz[...,1]<-0.09)
p=xyz[m]; print('ketchup n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'ztop',p[:,2].max().round(3))
"

# openrua op 46
timeout 600 python3 -u -c "
from arm import *
a=Arm()
tgt=np.array([-0.237,-0.161,0.631])
for i in range(3):
    tcp=a.move_tcp(tgt, Q_DOWN, seconds=5.0)
    print('  q', np.round(a.arm_q(),3))
    if tcp is not None and np.linalg.norm(tcp-tgt)<0.01: break
" 2>&1 | grep -v '^\['

# openrua op 47
timeout 120 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v '^\[' | head -1; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand soup_pre_eih.png >/dev/null 2>&1; python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
m=np.isfinite(z)&(z>0.49)&(z<0.515)&(xyz[...,0]>-0.30)&(xyz[...,0]<-0.18)&(xyz[...,1]>-0.23)&(xyz[...,1]<-0.10)
p=xyz[m]; print('soup top n',len(p),'center',p[:,:2].mean(0).round(4),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].mean().round(4))
m=np.isfinite(z)&(z>0.46)&(xyz[...,0]>-0.20)&(xyz[...,0]<-0.08)&(xyz[...,1]>-0.22)&(xyz[...,1]<-0.09)
p=xyz[m]; print('ketchup n',len(p),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'ztop',p[:,2].max().round(3))
"

# openrua op 48
timeout 900 python3 -u -c "
from arm import *
import subprocess
a=Arm()
Q=yaw_q(math.pi)
x,y=-0.2317,-0.1613
def goto(t, s, tries=3):
    for i in range(tries):
        tcp=a.move_tcp(t, Q, seconds=s)
        if tcp is not None and np.linalg.norm(tcp-np.array(t))<0.01: return tcp
        print('   retry')
    raise SystemExit('unreached '+str(t))
print('rotate at pre-grasp'); goto((x,y,0.631), 4.0)
pos,fq,_=a.fk_hand_world(); R=quat_R(*fq); print('  hand x', R[:,0].round(2), 'hand y', R[:,1].round(2), 'j7', round(a.arm_q()[6],3))
print('descend'); goto((x,y,0.56), 2.0); goto((x,y,0.48), 2.0)
print('  wrench', np.round(a.wrench(),2))
subprocess.run(['python3','/workspace/tools/perception/cam_snap.py','robot0_eye_in_hand','soup_grasp_eih.png'],capture_output=True)
print('close'); gap=a.gripper(0.0)
assert gap>0.02, 'closed on air'
print('lift'); goto((x,y,0.56), 2.0); goto((x,y,0.76), 2.5)
print('  gap after lift', round(a.finger_gap(),4))
subprocess.run(['python3','/workspace/tools/perception/cam_snap.py','agentview','soup_lifted.png'],capture_output=True)
" 2>&1 | grep -v '^\['

# openrua op 49
timeout 300 python3 -u -c "
from arm import *
a=Arm()
import time
for i in range(3):
    pos,fq,_=a.fk_hand_world(); R=quat_R(*fq); print('tcp', (pos+TCP_OFF*R[:,2]).round(4), 'hand x', R[:,0].round(2), 'gap', round(a.finger_gap(),4), 'wrench', np.round(a.wrench(),2), 'q', np.round(a.arm_q(),2))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py agentview soup_state.png >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand soup_state_eih.png >/dev/null 2>&1

# openrua op 50
timeout 600 python3 -u -c "
from arm import *
a=Arm()
Q=yaw_q(math.pi)
for i in range(3):
    tcp=a.move_tcp((-0.2317,-0.1613,0.70), Q, seconds=4.0)
    if tcp is not None and np.linalg.norm(tcp-np.array([-0.2317,-0.1613,0.70]))<0.01: break
pos,fq,_=a.fk_hand_world(); R=quat_R(*fq); print('hand x', R[:,0].round(2), 'wrench', np.round(a.wrench(),2), 'q', np.round(a.arm_q(),2))
" 2>&1 | grep -v '^\['; timeout 60 python3 tools/perception/cam_snap.py agentview soup_state2.png >/dev/null 2>&1; timeout 120 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v '^\[' | head -1

# openrua op 51
python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
m=np.isfinite(z)&(z>0.49)&(z<0.515)&(xyz[...,0]>-0.32)&(xyz[...,0]<-0.15)&(xyz[...,1]>-0.25)&(xyz[...,1]<-0.08)
p=xyz[m]; print('soup top n',len(p),'center',p[:,:2].mean(0).round(4),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].mean().round(4))
mask=(np.isfinite(z)&(z>0.44)&(z<0.62)).astype('uint8')
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<200: continue
    q=xyz[lab==i]; print('blob px',stats[i,4],'x',q[:,0].min().round(3),q[:,0].max().round(3),'y',q[:,1].min().round(3),q[:,1].max().round(3),'ztop',q[:,2].max().round(3))
"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand soup_state2_eih.png >/dev/null 2>&1

# openrua op 52
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        code = res.result().result.error_code\\n        # the result can arrive while the arm is still settling: re-read\\n        # until two consecutive joint reads agree\\n        prev = self.arm_q()\\n        for _ in range(20):\\n            cur = self.arm_q()\\n            if max(abs(a - b) for a, b in zip(cur, prev)) < 1e-3:\\n                break\\n            prev = cur\\n        err = max(abs(a - b) for a, b in zip(cur, waypoints[-1]))\\n        print(f\\"  traj done code={code} max_joint_err={err:.4f}\\")\\n        return code, err", "new_string": "        code = res.result().result.error_code\\n        err = max(abs(a - b) for a, b in zip(self.arm_q(), waypoints[-1]))\\n        print(f\\"  traj done code={code} max_joint_err={err:.4f}\\")\\n        # The sim arm can lag the trajectory clock: the result comes back\\n        # (code -5) with the arm frozen short of the goal. Re-send the final\\n        # point alone, slow enough for the remaining error, until it converges.\\n        for _ in range(4):\\n            if err < 0.01:\\n                break\\n            goal.trajectory.points = [JointTrajectoryPoint(\\n                positions=[float(v) for v in waypoints[-1]],\\n                time_from_start=Duration(sec=int(max(2.0, err / 0.4)) + 1))]\\n            send = self.fjt.send_goal_async(goal)\\n            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)\\n            res = send.result().get_result_async()\\n            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)\\n            code = res.result().result.error_code\\n            err = max(abs(a - b) for a, b in zip(self.arm_q(), waypoints[-1]))\\n            print(f\\"  resend code={code} max_joint_err={err:.4f}\\")\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 53
timeout 900 python3 -u -c "
from arm import *
import subprocess
a=Arm()
Q=yaw_q(math.pi)
x,y=-0.2336,-0.1611
def goto(t, s):
    tcp=a.move_tcp(t, Q, seconds=s)
    assert tcp is not None and np.linalg.norm(tcp-np.array(t))<0.01, 'unreached '+str(t)
    return tcp
print('pre-grasp'); goto((x,y,0.63), 3.0)
pos,fq,_=a.fk_hand_world(); R=quat_R(*fq); print('  hand x', R[:,0].round(2))
print('descend'); goto((x,y,0.56), 2.0); goto((x,y,0.48), 2.0)
print('  wrench', np.round(a.wrench(),2))
subprocess.run(['python3','/workspace/tools/perception/cam_snap.py','robot0_eye_in_hand','soup_grasp_eih.png'],capture_output=True)
print('close'); gap=a.gripper(0.0)
assert gap>0.02, 'closed on air'
print('lift'); goto((x,y,0.56), 2.0); goto((x,y,0.76), 2.5)
print('  gap after lift', round(a.finger_gap(),4))
subprocess.run(['python3','/workspace/tools/perception/cam_snap.py','agentview','soup_lifted.png'],capture_output=True)
" 2>&1 | grep -v '^\['

# openrua op 54
timeout 900 python3 -u -c "
from arm import *
import subprocess
a=Arm()
Q=yaw_q(math.pi)
B=(0.005,0.25)
def goto(t, s):
    tcp=a.move_tcp(t, Q, seconds=s)
    assert tcp is not None and np.linalg.norm(tcp-np.array(t))<0.01, 'unreached '+str(t)
    return tcp
assert a.finger_gap()>0.02
print('over basket'); goto((*B,0.78), 5.0)
gap=a.finger_gap(); print('  gap', round(gap,4)); assert gap>0.02, 'dropped'
subprocess.run(['python3','/workspace/tools/perception/cam_snap.py','robot0_eye_in_hand','soup_over_basket_eih.png'],capture_output=True)
print('lower + release'); goto((*B,0.72), 2.0)
print('  gap after open', round(a.gripper(0.04),4))
print('retreat'); goto((*B,0.86), 2.5)
subprocess.run(['python3','/workspace/tools/perception/cam_snap.py','agentview','soup_placed.png'],capture_output=True)
subprocess.run(['python3','/workspace/tools/perception/cam_snap.py','robot0_eye_in_hand','soup_placed_eih.png'],capture_output=True)
" 2>&1 | grep -v '^\['

# openrua op 55
timeout 120 python3 cloud.py robot0_eye_in_hand 2>&1 | grep -v '^\[' | head -1; python3 -c "
import numpy as np, cv2
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]; rgb=d['rgb']
# inside basket footprint, below rim
m=np.isfinite(z)&(xyz[...,0]>-0.06)&(xyz[...,0]<0.07)&(xyz[...,1]>0.18)&(xyz[...,1]<0.32)&(z>0.44)&(z<0.62)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m.astype('uint8'))
for i in range(1,n):
    if stats[i,4]<300: continue
    q=xyz[lab==i]; c=rgb[lab==i].mean(0).astype(int)
    print('inside-basket blob px',stats[i,4],'x',q[:,0].min().round(3),q[:,0].max().round(3),'y',q[:,1].min().round(3),q[:,1].max().round(3),'z',q[:,2].min().round(3),q[:,2].max().round(3),'rgb',c)
# anything can-like left on the table outside the basket?
m2=np.isfinite(z)&(z>0.46)&(z<0.60)&((xyz[...,1]<0.16)|(xyz[...,1]>0.34)|(xyz[...,0]<-0.08)|(xyz[...,0]>0.09))
print('table objects seen outside basket px', int(m2.sum()))
"

# openrua op 56
python3 -c "
import cv2
im=cv2.imread('soup_placed.png'); cv2.imwrite('basket_zoom.png', cv2.resize(im[190:380,430:630],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 57
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Verified-live quirks of the /workspace Panda sim workstation (IK frame/yaw, effective floor, trajectory lag) that contradict or extend the shipped docs
metadata:
  type: project
---

Facts verified on 2026-09-12 on the Franka Panda sim workstation in /workspace (LIBERO-style scene, MoveIt + robot_bridge):

- `/compute_ik` and `/compute_fk` use the WORLD frame with empty `frame_id` (docs/30-action.md claims the arm base; FK matched `tf world->panda_hand`). panda_link0 is at world (-0.51, 0, 0.42); table top z≈0.42.
- `/compute_ik` solves for a tip yawed −45° from `panda_hand` (link8): the resulting hand comes out +45° about z from the request. Pre-multiply the request by a −45° local-z rotation (see `/workspace/arm.py` `ik_tcp_world`).
- `tcp_offset_m: 0.1034` is correct (fingertips at nominal TCP), but the arm hits a hard invisible floor at nominal TCP z≈0.474 (5.4 cm above the rendered table) everywhere; +z wrench rises when pushed. Grasp low objects by their upper part at TCP z≈0.48.
- FJT results often return code −5 with the sim arm frozen short of the goal (paused-clock lag), especially on big joint-7 rotations. Re-send the final point until joint error <0.01 rad — `arm.py move_joints` does this. Never chain moves on a −5 without converging first.
- Gripper is binary; open gap measures ~0.078–0.080 m. Cans are ~6.7 cm wide, so center within ~5 mm (wrist-cam depth at pre-grasp gives ~2 mm).
- Wrist cam `robot0_eye_in_hand` is 5 cm along hand +x from panda_hand, looking down; image x = hand y = finger closing axis.

**Why:** the shipped docs are generic and partly wrong for this machine; rediscovering these cost many turns.
**How to apply:** reuse `/workspace/arm.py`, `cloud.py`, `pick_place.py`, `place.py` if still present; otherwise re-verify each fact with one FK/IK probe before trusting it.
OPENRUA_EOF

# openrua op 58
cat /claude-config/projects/-workspace/memory/MEMORY.md 2>/dev/null; echo "- [Panda workstation quirks](panda-workstation-quirks.md) — IK is world-frame & +45° yaw off, floor at TCP z≈0.474, re-send FJT on code −5" >> /claude-config/projects/-workspace/memory/MEMORY.md
