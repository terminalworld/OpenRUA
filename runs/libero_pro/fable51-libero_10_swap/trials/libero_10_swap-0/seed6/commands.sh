#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | head -50; echo ---; timeout 30 ros2 node list

# openrua op 4
timeout 30 ros2 topic echo /joint_states --once | head -40; for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c img_$c.png; done

# openrua op 5
python3 -c "
import cv2
im=cv2.imread('img_agentview.png')
crop=im[180:420,90:420]
cv2.imwrite('crop_agent.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 6
python3 -c "
import cv2
im=cv2.imread('img_birdview.png')
crop=im[200:340,230:420]
cv2.imwrite('crop_bird.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('img_agentview.png')
crop=im[180:300,190:280]
cv2.imwrite('crop_agent2.png', cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
"

# openrua op 7
python3 -c "
import cv2
im=cv2.imread('img_sideview.png')
crop=im[380:480,220:400]
cv2.imwrite('crop_side.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('img_frontview.png')
crop=im[200:290,200:370]
cv2.imwrite('crop_front.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('img_robot0_eye_in_hand.png')
crop=im[150:280,500:640]
cv2.imwrite('crop_eih.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 8
timeout 60 python3 tools/perception/px2world.py agentview 137 305; timeout 60 python3 tools/perception/px2world.py agentview 232 215; timeout 60 python3 tools/perception/px2world.py agentview 530 300; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12

# openrua op 9
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a camera's depth frame as a world-frame point cloud (npz) plus a
color frame. Usage: python3 cloud.py <camera> [out_prefix]"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, typ):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    while "m" not in got:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else cam
    rclpy.init()
    node = rclpy.create_node("cloud")
    buf = Buffer()
    TransformListener(buf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "bgr8")
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y,
                  t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    P = np.stack([X, Y, depth], -1) @ R.T + T
    np.savez(out + "_cloud.npz", P=P, color=color, depth=depth)
    import cv2
    cv2.imwrite(out + "_color.png", color)
    print("saved", out, P.shape)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 10
timeout 120 python3 cloud.py birdview bird && timeout 120 python3 cloud.py agentview agent && python3 -c "
import numpy as np
d=np.load('bird_cloud.npz'); P=d['P']
z=P[...,2]
print('z percentiles', np.percentile(z[np.isfinite(z)],[1,5,25,50,75,95,99]))
# table height: mode of z in table region
zz=z[np.isfinite(z)]
hist,edges=np.histogram(zz,bins=200)
i=np.argsort(hist)[-5:]
for j in i: print(edges[j],hist[j])
"

# openrua op 11
python3 -c "
import numpy as np, cv2
d=np.load('bird_cloud.npz'); P=d['P']; col=d['color']
z=P[...,2]
mask=(np.isfinite(z)&(z>0.445)&(z<0.75)&(P[...,0]>-0.3)&(P[...,0]<0.6)&(np.abs(P[...,1])<0.5)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    pts=P[m]
    print(f'blob {i}: px centroid {cent[i].round(0)}, n={stats[i,4]}, x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} zmed={np.median(pts[:,2]):.3f} meanBGR={col[m].mean(0).round(0)}')
"

# openrua op 12
python3 -c "
import numpy as np, cv2
d=np.load('agent_cloud.npz'); P=d['P']; col=d['color']
z=P[...,2]
mask=(np.isfinite(z)&(z>0.445)&(z<0.75)&(P[...,0]>-0.4)&(P[...,0]<0.6)&(np.abs(P[...,1])<0.5)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    pts=P[m]
    print(f'blob {i}: px centroid {cent[i].round(0)}, n={stats[i,4]}, x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} meanBGR={col[m].mean(0).round(0)}')
"

# openrua op 13
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library: one node, reusable clients for FK / IK /
trajectory / gripper / servo, joint-state and wrench readers.
World frame = panda_link0 + (-0.51, 0, 0.42) (from tf2_echo)."""
import math
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
WORLD_T_BASE = np.array([-0.51, 0.0, 0.42])
TCP = float(M["hand"]["tcp_offset_m"])

# top-down grasp orientation (hand z down, fingers along base y)
Q_DOWN = (1.0, 0.0, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def q_yaw_down(yaw):
    """Hand pointing down, fingers rotated by yaw about vertical."""
    # R = Rz(yaw) * Rx(pi)
    qz = (0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
    qx = (1, 0, 0, 0)
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.node.create_subscription(
            WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
            self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        assert self.fjt.wait_for_server(10), "no fjt"
        assert self.grip.wait_for_server(10), "no gripper"
        assert self.ik.wait_for_service(10), "no ik"
        assert self.fk.wait_for_service(10), "no fk"
        while self._js is None:
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, n=3):
        for _ in range(n):
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self.spin()
        d = dict(zip(self._js.name, self._js.position))
        return [d[j] for j in ARM]

    def fingers(self):
        self.spin()
        d = dict(zip(self._js.name, self._js.position))
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def wrench(self):
        self.spin()
        if self._wr is None:
            return None
        f = self._wr.wrench.force
        return np.array([f.x, f.y, f.z])

    def _seed(self, q=None):
        js = JointState()
        js.name = list(ARM)
        js.position = list(q if q is not None else self.joints())
        return js

    def fk_pose(self, q=None):
        """Hand pose in base frame: (xyz, quat xyzw)."""
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                (p.orientation.x, p.orientation.y, p.orientation.z,
                 p.orientation.w))

    def tcp_world(self, q=None):
        p, quat = self.fk_pose(q)
        R = quat_R(*quat)
        return p + R[:, 2] * TCP + WORLD_T_BASE

    def ik_hand_base(self, xyz, quat, seed=None, tries=3):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        (p.orientation.x, p.orientation.y, p.orientation.z,
         p.orientation.w) = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=1)
        for _ in range(tries):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name,
                               res.solution.joint_state.position))
                return [sol[j] for j in ARM]
        return None

    def ik_tcp_world(self, xyz_world, quat=Q_DOWN, seed=None):
        """IK for the TCP (fingertip centre) at a WORLD position."""
        R = quat_R(*quat)
        hand_base = np.array(xyz_world) - WORLD_T_BASE - R[:, 2] * TCP
        return self.ik_hand_base(hand_base, quat, seed)

    def move(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = seconds * (i + 1) / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.max(np.abs(np.array(self.joints()) - np.array(q)))
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        return r.reached_goal, r.stalled, self.fingers()

    def servo(self, v, n=20):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            pub_ok = self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
OPENRUA_EOF

# openrua op 14
timeout 300 python3 -u -c "
from rob import *
r=Robot('probe')
print('joints', np.round(r.joints(),3))
print('fingers', r.fingers())
p,q=r.fk_pose(); print('hand base', p.round(3), np.round(q,3))
print('tcp world', r.tcp_world().round(3))
print('wrench', r.wrench())
# IK feasibility for the targets
for name,xyz in [('sauce_pre',(-0.01,-0.242,0.62)),('sauce_grasp',(-0.01,-0.242,0.47)),('soup_pre',(-0.252,-0.146,0.60)),('soup_grasp',(-0.252,-0.146,0.455)),('basket',(0.0,0.26,0.72))]:
    sol=r.ik_tcp_world(xyz)
    print(name, xyz, None if sol is None else np.round(sol,3))
    if sol is not None: print('   check tcp', r.tcp_world(sol).round(3))
"

# openrua op 15
timeout 300 python3 -u -c "
from rob import *
import numpy as np
d=np.load('bird_cloud.npz'); P=d['P']
print('bird px(300,265) world', P[265,300].round(3), 'px(320,150)', P[150,320].round(3))
r=Robot('probe2')
p,q=r.fk_pose()
print('ik of current pose:', r.ik_hand_base(p,q))
print('ik of current pose, exact quat (1,0,0,0):', r.ik_hand_base(p,(1,0,0,0)))
print('ik ready pose:', r.ik_hand_base((0.307,0,0.59),(1,0,0,0)))
print('ik lower:', r.ik_hand_base((0.4,-0.2,0.3),(1,0,0,0)))
" 2>&1 | grep -v XMLPARSER

# openrua op 16
timeout 300 python3 -u -c "
from rob import *
import numpy as np
r=Robot('probe3')
sol=r.ik_hand_base(*r.fk_pose())
print('fk of ik sol:', r.fk_pose(sol))
ready=[0,-0.785,0,-2.356,0,1.571,0.785]
print('fk ready:', r.fk_pose(ready))
print('ik ready seeded ready:', r.ik_hand_base(*r.fk_pose(ready), seed=ready))
print('ik ready seeded current:', r.ik_hand_base(*r.fk_pose(ready)))
" 2>&1 | grep -v XMLPARSER

# openrua op 17
python3 - <<'EOF'
import re
s=open('rob.py').read()
s=s.replace('WORLD_T_BASE = np.array([-0.51, 0.0, 0.42])','WORLD_T_BASE = np.array([-0.51, 0.0, 0.42])  # tf world->panda_link0\n# NOTE: /compute_fk and /compute_ik on this machine report/accept poses in\n# the WORLD frame already (verified: FK of the ready pose = base pose + offset).')
s=s.replace('        return p + R[:, 2] * TCP + WORLD_T_BASE','        return p + R[:, 2] * TCP')
s=s.replace('        hand_base = np.array(xyz_world) - WORLD_T_BASE - R[:, 2] * TCP\n        return self.ik_hand_base(hand_base, quat, seed)','        hand = np.array(xyz_world) - R[:, 2] * TCP\n        return self.ik_hand_base(hand, quat, seed)')
s=s.replace('        req.ik_request.pose_stamped.header.frame_id = ""\n','        req.ik_request.pose_stamped.header.frame_id = ""\n        req.ik_request.ik_link_name = "panda_hand"\n')
open('rob.py','w').write(s)
EOF
timeout 300 python3 -u -c "
from rob import *
r=Robot('probe4')
print('tcp world now', r.tcp_world().round(3))
sol=r.ik_hand_base(*r.fk_pose())
print('ik cur -> fk:', r.fk_pose(sol))
for name,xyz in [('sauce_pre',(-0.01,-0.242,0.62)),('sauce_grasp',(-0.01,-0.242,0.47)),('soup_pre',(-0.252,-0.146,0.60)),('soup_grasp',(-0.252,-0.146,0.455)),('basket',(0.0,0.26,0.72))]:
    sol=r.ik_tcp_world(xyz)
    print(name, xyz, None if sol is None else np.round(sol,3))
    if sol is not None: print('   check tcp', r.tcp_world(sol).round(3), 'quat', np.round(r.fk_pose(sol)[1],3))
" 2>&1 | grep -v XMLPARSER

# openrua op 18
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick a can (top-down grasp) and drop it into the basket.
Usage: python3 -u pick_place.py <x> <y> <top_z> <name>
All coordinates world frame (table top z=0.42)."""
import sys
import numpy as np
from rob import Robot, Q_DOWN

BASKET = np.array([0.0, 0.26])
RIM_Z = 0.627


def log(*a):
    print(*a, flush=True)


def cart_path(r, start_q, pts, quat=Q_DOWN):
    """IK for a chain of TCP points, each seeded with the previous one."""
    qs, seed = [], start_q
    for p in pts:
        q = r.ik_tcp_world(p, quat, seed=seed)
        if q is None:
            raise SystemExit(f"IK failed at {p}")
        qs.append(q)
        seed = q
    return qs


def go(r, qs, seconds):
    code, err = r.move(qs[-1], seconds, via=qs[:-1])
    tcp = r.tcp_world()
    log(f"  move code={code} joint_err={err:.4f} tcp={tcp.round(3)}")
    return tcp


def main():
    x, y, top, name = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    r = Robot("pick_" + name)
    log(f"[{name}] start tcp={r.tcp_world().round(3)} fingers={r.fingers()}")

    grasp_z = 0.42 + (top - 0.42) * 0.45      # a bit below mid-height
    pre_z = top + 0.10
    lift_z = 0.76
    log(f"[{name}] target ({x},{y}) top={top} grasp_z={grasp_z:.3f}")

    log("open gripper"); log(" ", r.gripper(0.04))

    q0 = r.joints()
    pre = cart_path(r, q0, [(x, y, lift_z), (x, y, pre_z)])
    log("-> pre-grasp"); go(r, pre, 4.0)

    # descend in 3 steps
    zs = np.linspace(pre_z, grasp_z, 4)[1:]
    down = cart_path(r, pre[-1], [(x, y, z) for z in zs])
    log("-> descend"); tcp = go(r, down, 2.5)
    if np.linalg.norm(tcp[:2] - np.array([x, y])) > 0.01:
        log("  WARNING: xy off by", np.round(tcp[:2] - [x, y], 3))

    log("close gripper"); res = r.gripper(0.0); log(" ", res)
    f1, f2 = res[2]
    gap = abs(f1) + abs(f2)
    log(f"  finger gap={gap:.4f}")
    if gap < 0.01:
        log("  GRASP FAILED (closed on air)"); sys.exit(2)

    up = cart_path(r, down[-1], [(x, y, grasp_z + 0.08), (x, y, lift_z)])
    log("-> lift"); go(r, up, 3.0)
    log(f"  fingers after lift={r.fingers()}")

    over = cart_path(r, up[-1], [(BASKET[0], BASKET[1], lift_z)])
    log("-> over basket"); go(r, over, 4.0)
    log(f"  fingers over basket={r.fingers()}")
    lower = cart_path(r, over[-1], [(BASKET[0], BASKET[1], 0.70)])
    log("-> lower into basket"); go(r, lower, 2.0)

    log("open gripper"); log(" ", r.gripper(0.04))
    back = cart_path(r, lower[-1], [(BASKET[0], BASKET[1], lift_z + 0.04)])
    log("-> retreat"); go(r, back, 2.0)
    log(f"[{name}] DONE fingers={r.fingers()}")


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 19
nohup python3 -u pick_place.py -0.010 -0.242 0.521 sauce > sauce.log 2>&1 &
sleep 45; grep -v XMLPARSER /workspace/sauce.log

# openrua op 20
sleep 40; grep -v XMLPARSER /workspace/sauce.log | tail -12

# openrua op 21
timeout 60 python3 tools/perception/cam_snap.py agentview after1_agent.png; timeout 60 python3 tools/perception/cam_snap.py birdview after1_bird.png; python3 -c "
import cv2
im=cv2.imread('after1_bird.png'); cv2.imwrite('after1_bird_crop.png', cv2.resize(im[200:340,230:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 22
timeout 120 python3 cloud.py birdview bird2 >/dev/null 2>&1; python3 -c "
import numpy as np, cv2
d=np.load('bird2_cloud.npz'); P=d['P']
z=P[...,2]
mask=(np.isfinite(z)&(z>0.445)&(z<0.75)&(P[...,0]>-0.35)&(P[...,0]<0.2)&(P[...,1]<0)&(P[...,1]>-0.5)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    pts=P[lab==i]
    print(f'blob {i}: n={stats[i,4]} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} centre=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})')
"

# openrua op 23
nohup python3 -u pick_place.py -0.249 -0.148 0.51 soup > soup.log 2>&1 &
sleep 60; grep -v XMLPARSER /workspace/soup.log

# openrua op 24
sleep 30; grep -v XMLPARSER /workspace/soup.log | tail -8; pgrep -f pick_place.py || echo finished

# openrua op 25
sleep 5; pgrep -f pick_place.py || echo finished; timeout 60 python3 tools/perception/cam_snap.py agentview after2_agent.png >/dev/null 2>&1; timeout 120 python3 cloud.py birdview bird3 >/dev/null 2>&1; python3 -c "
import numpy as np, cv2
im=cv2.imread('bird3_color.png'); cv2.imwrite('bird3_crop.png', cv2.resize(im[200:340,230:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
d=np.load('bird3_cloud.npz'); P=d['P']
z=P[...,2]
mask=(np.isfinite(z)&(z>0.445)&(z<0.75)&(P[...,0]>-0.4)&(P[...,0]<0.2)&(P[...,1]<0)&(P[...,1]>-0.5)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    pts=P[lab==i]
    print(f'blob {i}: n={stats[i,4]} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} centre=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})')
"

# openrua op 26
python3 - <<'EOF'
s=open('pick_place.py').read()
s=s.replace('''def go(r, qs, seconds):
    code, err = r.move(qs[-1], seconds, via=qs[:-1])
    tcp = r.tcp_world()
    log(f"  move code={code} joint_err={err:.4f} tcp={tcp.round(3)}")
    return tcp
''','''def go(r, qs, seconds, tol=0.02):
    code, err = r.move(qs[-1], seconds, via=qs[:-1])
    tcp = r.tcp_world()
    log(f"  move code={code} joint_err={err:.4f} tcp={tcp.round(3)}")
    # controller lag shows up as tolerance violations: resend the final
    # point until the joints actually converge
    for i in range(4):
        if err <= tol:
            break
        code, err = r.move(qs[-1], max(2.0, seconds / 2))
        tcp = r.tcp_world()
        log(f"  retry{i} code={code} joint_err={err:.4f} tcp={tcp.round(3)}")
    return tcp
''')
s=s.replace('''    log("-> descend"); tcp = go(r, down, 2.5)
    if np.linalg.norm(tcp[:2] - np.array([x, y])) > 0.01:
        log("  WARNING: xy off by", np.round(tcp[:2] - [x, y], 3))
''','''    log("-> descend"); tcp = go(r, down, 2.5)
    for i in range(3):
        off = np.linalg.norm(tcp[:2] - np.array([x, y]))
        if off <= 0.006 and abs(tcp[2] - grasp_z) <= 0.01:
            break
        log(f"  correcting: off={off:.4f} dz={tcp[2]-grasp_z:.4f}")
        fix = cart_path(r, r.joints(), [(x, y, grasp_z)])
        tcp = go(r, fix, 2.0)
    if np.linalg.norm(tcp[:2] - np.array([x, y])) > 0.01:
        log("  ABORT: xy still off", np.round(tcp[:2] - [x, y], 3)); sys.exit(3)
''')
s=s.replace('log("-> pre-grasp"); go(r, pre, 4.0)','log("-> pre-grasp"); go(r, pre, 6.0)')
s=s.replace('log("-> over basket"); go(r, over, 4.0)','log("-> over basket"); go(r, over, 6.0)')
open('pick_place.py','w').write(s)
EOF
nohup python3 -u pick_place.py -0.250 -0.149 0.51 soup > soup2.log 2>&1 &
sleep 75; grep -v XMLPARSER /workspace/soup2.log

# openrua op 27
pkill -f pick_place.py; sleep 2; grep -v XMLPARSER soup2.log | tail -4; timeout 300 python3 -u -c "
from rob import *
r=Robot('insp')
q=r.joints(); print('cur joints', np.round(q,3)); print('tcp', r.tcp_world().round(3), 'fingers', r.fingers())
x,y=-0.25,-0.149
seed=q
for z in [0.61,0.56,0.51,0.461]:
    s=r.ik_tcp_world((x,y,z), seed=seed)
    print(z, None if s is None else np.round(s,3))
    if s is not None: seed=s
print('--- with yaw 90deg')
seed=q
for z in [0.61,0.56,0.51,0.461]:
    s=r.ik_tcp_world((x,y,z), quat=q_yaw_down(np.pi/2), seed=seed)
    print(z, None if s is None else np.round(s,3))
    if s is not None: seed=s
print('--- with yaw -45deg')
seed=q
for z in [0.61,0.56,0.51,0.461]:
    s=r.ik_tcp_world((x,y,z), quat=q_yaw_down(-np.pi/4), seed=seed)
    print(z, None if s is None else np.round(s,3))
    if s is not None: seed=s
" 2>&1 | grep -v XMLPARSER

# openrua op 28
pgrep -f pick_place.py || echo "no picker"; grep -v XMLPARSER /workspace/soup2.log | tail -4

# openrua op 29
sleep 2; ps aux | grep -c "[p]ick_place.py"; cat > insp.py <<'EOF'
from rob import *
r=Robot('insp')
q=r.joints(); print('cur joints', np.round(q,3)); print('tcp', r.tcp_world().round(3), 'fingers', r.fingers())
x,y=-0.25,-0.149
for label,quat in [('yaw0',Q_DOWN),('yaw90',q_yaw_down(np.pi/2)),('yaw-45',q_yaw_down(-np.pi/4)),('yaw45',q_yaw_down(np.pi/4))]:
    print('---',label)
    seed=q
    for z in [0.61,0.56,0.51,0.461]:
        s=r.ik_tcp_world((x,y,z), quat=quat, seed=seed)
        print(z, None if s is None else np.round(s,3))
        if s is not None: seed=s
EOF
timeout 300 python3 -u insp.py 2>&1 | grep -v XMLPARSER

# openrua op 30
cat > insp2.py <<'EOF'
from rob import *
from scipy.spatial.transform import Rotation as Rot
r=Robot('insp2')
q=r.joints(); print('cur joints', np.round(q,3), 'tcp', r.tcp_world().round(3))
x,y=-0.25,-0.149
def q_tilt(theta, yaw=0.0):
    c,s=np.cos(theta),np.sin(theta)
    R=np.array([[c,0,-s],[0,-1,0],[-s,0,-c]])
    R=Rot.from_euler('z',yaw).as_matrix()@R
    return tuple(Rot.from_matrix(R).as_quat())  # xyzw
for deg in [20,30,40,50]:
    quat=q_tilt(np.radians(deg))
    print('--- tilt',deg)
    seed=q
    for z in [0.66,0.61,0.56,0.51,0.461]:
        s=r.ik_tcp_world((x,y,z), quat=quat, seed=seed)
        print(z, None if s is None else np.round(s,3), '' if s is None else r.tcp_world(s).round(3))
        if s is not None: seed=s
EOF
timeout 300 python3 -u insp2.py 2>&1 | grep -v XMLPARSER

# openrua op 31
cat > insp3.py <<'EOF'
from rob import *
from moveit_msgs.srv import GetPositionFK
r=Robot('insp3')
def links(q):
    req=GetPositionFK.Request(); req.fk_link_names=[f'panda_link{i}' for i in range(1,9)]+['panda_hand','panda_leftfinger','panda_rightfinger']
    req.robot_state.joint_state=r._seed(q)
    fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
    for n,ps in zip(req.fk_link_names,fut.result().pose_stamped):
        p=ps.pose.position; print(f'  {n}: ({p.x:.3f},{p.y:.3f},{p.z:.3f})')
print('grasp cfg tilt40'); links([-1.206,0.448,0.741,-2.532,-0.64,2.059,0.915])
print('pre cfg tilt40'); links([-1.22,-0.153,0.853,-2.575,-0.15,1.822,0.46])
EOF
timeout 120 python3 -u insp3.py 2>&1 | grep -v XMLPARSER

# openrua op 32
cat > insp4.py <<'EOF'
from rob import *
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.srv import GetPositionFK
r=Robot('insp4')
def q_tilt(theta, yaw=0.0):
    c,s=np.cos(theta),np.sin(theta)
    R=np.array([[c,0,-s],[0,-1,0],[-s,0,-c]])
    R=Rot.from_euler('z',yaw).as_matrix()@R
    return tuple(Rot.from_matrix(R).as_quat())
def links(q, names=('panda_link5','panda_link7','panda_hand')):
    req=GetPositionFK.Request(); req.fk_link_names=list(names)
    req.robot_state.joint_state=r._seed(q)
    fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
    return {n:np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],3) for n,ps in zip(names,fut.result().pose_stamped)}
x,y=-0.25,-0.149
q=r.joints()
for deg,yawdeg in [(35,-45),(40,-45),(35,-60),(40,-30)]:
    quat=q_tilt(np.radians(deg),np.radians(yawdeg)); print('--- tilt',deg,'yaw',yawdeg)
    seed=q
    for z in [0.66,0.61,0.56,0.51,0.461]:
        s=r.ik_tcp_world((x,y,z), quat=quat, seed=seed)
        if s is None: print(z,None); continue
        print(z, np.round(s,3), links(s) if z in (0.461,0.61) else '')
        seed=s
EOF
timeout 300 python3 -u insp4.py 2>&1 | grep -v XMLPARSER

# openrua op 33
python3 - <<'EOF'
s=open('pick_place.py').read()
s=s.replace('''import sys
import numpy as np
from rob import Robot, Q_DOWN
''','''import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import Robot, Q_DOWN


def q_tilt(theta, yaw=0.0):
    """Hand approach tilted by theta from vertical; the wrist is displaced
    from the fingertips along the xy direction given by yaw (0 = +x)."""
    c, s = np.cos(theta), np.sin(theta)
    R = np.array([[c, 0, -s], [0, -1, 0], [-s, 0, -c]])
    R = Rot.from_euler('z', yaw).as_matrix() @ R
    return tuple(Rot.from_matrix(R).as_quat())
''')
s=s.replace('''    x, y, top, name = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
''','''    x, y, top, name = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
    tilt = np.radians(float(sys.argv[5])) if len(sys.argv) > 5 else 0.0
    yaw = np.radians(float(sys.argv[6])) if len(sys.argv) > 6 else 0.0
    QG = q_tilt(tilt, yaw) if tilt else Q_DOWN
''')
s=s.replace('pre = cart_path(r, q0, [(x, y, lift_z), (x, y, pre_z)])','pre = cart_path(r, q0, [(x, y, lift_z), (x, y, pre_z)], QG)')
s=s.replace('down = cart_path(r, pre[-1], [(x, y, z) for z in zs])','down = cart_path(r, pre[-1], [(x, y, z) for z in zs], QG)')
s=s.replace('fix = cart_path(r, r.joints(), [(x, y, grasp_z)])','fix = cart_path(r, r.joints(), [(x, y, grasp_z)], QG)')
s=s.replace('up = cart_path(r, down[-1], [(x, y, grasp_z + 0.08), (x, y, lift_z)])','up = cart_path(r, r.joints(), [(x, y, grasp_z + 0.08), (x, y, lift_z)], QG)')
s=s.replace('zs = np.linspace(pre_z, grasp_z, 4)[1:]','zs = np.linspace(pre_z, grasp_z, 5)[1:]')
open('pick_place.py','w').write(s)
EOF
grep -n "QG\|q_tilt" pick_place.py | head; nohup python3 -u pick_place.py -0.250 -0.149 0.51 soup 35 -45 > soup3.log 2>&1 &
sleep 90; grep -v XMLPARSER /workspace/soup3.log

# openrua op 34
pkill -f pick_place.py; sleep 3; grep -v XMLPARSER soup3.log | grep -v "^ *File\|^Traceback\|^rclpy\|^\s*\^" | tail -3; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_now.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent_now.png; timeout 60 python3 tools/perception/cam_snap.py sideview side_now.png

# openrua op 35
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_now.png; timeout 60 python3 tools/perception/cam_snap.py agentview agent_now.png; timeout 60 python3 tools/perception/cam_snap.py sideview side_now.png

# openrua op 36
timeout 60 python3 -c "
from rob import *
r=Robot('st'); print('joints',np.round(r.joints(),3)); print('tcp',r.tcp_world().round(3)); print('fingers',r.fingers())
p,q=r.fk_pose(); print('hand',p.round(3),np.round(q,3))
" 2>&1 | grep -v XMLPARSER; timeout 20 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | grep -A3 Translation | head -4; timeout 60 python3 cloud.py robot0_eye_in_hand eih >/dev/null 2>&1 && python3 -c "
import numpy as np
d=np.load('eih_cloud.npz'); P=d['P']; z=P[...,2]
print('eih z range', np.nanmin(z), np.nanmax(z))
m=np.isfinite(z)&(z>0.44)&(z<0.6)
print('pts above table', m.sum())
pts=P[m]; print('x',pts[:,0].min(),pts[:,0].max(),'y',pts[:,1].min(),pts[:,1].max())
"

# openrua op 37
cat > retreat.py <<'EOF'
from rob import *
from pick_place import q_tilt, go, cart_path
r=Robot('retreat')
QG=q_tilt(np.radians(35),np.radians(-45))
tcp=r.tcp_world()
# straight up first, then to pre-grasp
qs=cart_path(r, r.joints(), [(tcp[0],tcp[1],0.62),(-0.25,-0.149,0.61)], QG)
go(r, qs, 4.0)
print('fingers', r.fingers(), 'tcp', r.tcp_world().round(3))
EOF
timeout 300 python3 -u retreat.py 2>&1 | grep -v XMLPARSER; timeout 60 python3 cloud.py robot0_eye_in_hand eih2 >/dev/null 2>&1; timeout 60 python3 cloud.py birdview bird4 >/dev/null 2>&1; python3 -c "
import numpy as np, cv2
for name in ['eih2','bird4']:
    d=np.load(name+'_cloud.npz'); P=d['P']; z=P[...,2]
    m=(np.isfinite(z)&(z>0.47)&(z<0.56)&(P[...,0]>-0.32)&(P[...,0]<-0.19)&(P[...,1]>-0.22)&(P[...,1]<-0.08)).astype(np.uint8)
    n,lab,stats,cent=cv2.connectedComponentsWithStats(m)
    for i in range(1,n):
        if stats[i,4]<20: continue
        pts=P[lab==i]
        print(name, f'blob n={stats[i,4]} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} centre=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) px={cent[i].round(0)}')
"

# openrua op 38
python3 -c "
import numpy as np, cv2
d=np.load('eih2_cloud.npz'); P=d['P']; col=d['color']; z=P[...,2]
cv2.imwrite('eih2_color.png', col)
# color of the blob region
m=(np.isfinite(z)&(z>0.47)&(z<0.56)&(P[...,0]>-0.32)&(P[...,0]<-0.19)&(P[...,1]>-0.22)&(P[...,1]<-0.08))
vis=col.copy(); vis[m]=(0,255,0); cv2.imwrite('eih2_mask.png', vis)
# distribution of z in whole image
zz=z[np.isfinite(z)]; print(np.percentile(zz,[0,5,50,95,100]))
# where are the points with z<0.3 (fingers/hand very near cam?) 
print('near pts', (d['depth']<0.15).sum())
"

# openrua op 39
timeout 60 python3 tools/perception/cam_snap.py agentview agent_now2.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview front_now2.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('agent_now2.png'); cv2.imwrite('agent_now2_crop.png', cv2.resize(im[150:330,120:330],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
im=cv2.imread('front_now2.png'); cv2.imwrite('front_now2_crop.png', cv2.resize(im[170:290,180:380],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 40
cat > home.py <<'EOF'
from rob import *
from pick_place import go
r=Robot('home')
HOME=[0.0,-0.161,0.0,-2.445,0.0,2.227,0.785]
go(r,[HOME],5.0)
print('tcp',r.tcp_world().round(3))
EOF
timeout 300 python3 -u home.py 2>&1 | grep -v XMLPARSER; timeout 120 python3 cloud.py birdview bird5 >/dev/null 2>&1; python3 -c "
import numpy as np, cv2
d=np.load('bird5_cloud.npz'); P=d['P']; col=d['color']; z=P[...,2]
cv2.imwrite('bird5_crop.png', cv2.resize(col[180:340,200:420],None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
m=(np.isfinite(z)&(z>0.445)&(z<0.75)&(P[...,0]>-0.45)&(P[...,0]<0.2)&(P[...,1]<0)&(P[...,1]>-0.5)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m)
for i in range(1,n):
    if stats[i,4]<15: continue
    pts=P[lab==i]
    print(f'blob n={stats[i,4]} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} centre=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) px={cent[i].round(0)}')
"

# openrua op 41
python3 -c "
import numpy as np, cv2
d=np.load('bird5_cloud.npz'); P=d['P']; col=d['color']; z=P[...,2]
m=(np.isfinite(z)&(z>0.445)&(z<0.51)&(P[...,0]>-0.45)&(P[...,0]<0.0)&(P[...,1]<-0.05)&(P[...,1]>-0.3))
pts=P[m]
print('lying-can region pts', m.sum(), 'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'z',pts[:,2].max().round(3))
# highest ridge = can axis line: points with z>0.475
r=pts[pts[:,2]>0.475]; print('ridge', len(r), 'x',r[:,0].min().round(3),r[:,0].max().round(3),'y',r[:,1].min().round(3),r[:,1].max().round(3),'centre',r.mean(0).round(3))
# fit axis via PCA on xy
c=r[:,:2].mean(0); u,s,vt=np.linalg.svd(r[:,:2]-c); print('axis dir', vt[0].round(3), 'spread', s.round(3))
# ketchup body pts z>0.52
k=P[np.isfinite(z)&(z>0.52)&(z<0.7)&(P[...,0]>-0.3)&(P[...,0]<0.0)&(P[...,1]<-0.05)&(P[...,1]>-0.3)]
print('ketchup x',k[:,0].min().round(3),k[:,0].max().round(3),'y',k[:,1].min().round(3),k[:,1].max().round(3))
"

# openrua op 42
python3 -c "
import numpy as np
d=np.load('bird5_cloud.npz'); P=d['P']; z=P[...,2]
m=(np.isfinite(z)&(z>0.445)&(z<0.51)&(P[...,0]>-0.30)&(P[...,0]<-0.135)&(P[...,1]<-0.05)&(P[...,1]>-0.3))
pts=P[m]
print('can pts', len(pts), 'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'ztop',pts[:,2].max().round(3))
for zt in [0.47,0.48]:
    r=pts[pts[:,2]>zt]; print(f'z>{zt}: n={len(r)} x[{r[:,0].min():.3f},{r[:,0].max():.3f}] y[{r[:,1].min():.3f},{r[:,1].max():.3f}] centre {r.mean(0).round(3)}')
# per-x-slice y centre
xs=np.arange(-0.21,-0.13,0.01)
for a in xs:
    s=pts[(pts[:,0]>=a)&(pts[:,0]<a+0.01)]
    if len(s): print(f'x~{a:.2f}: y[{s[:,1].min():.3f},{s[:,1].max():.3f}] yc={s[:,1].mean():.3f} zmax={s[:,2].max():.3f}')
"

# openrua op 43
python3 -c "
import numpy as np
d=np.load('bird5_cloud.npz'); P=d['P']; z=P[...,2]
m=(np.isfinite(z)&(z>0.43)&(z<0.51)&(P[...,0]>-0.30)&(P[...,0]<-0.135)&(P[...,1]<-0.05)&(P[...,1]>-0.3))
pts=P[m]
for a in np.arange(-0.19,-0.11,0.008):
    s=pts[(pts[:,1]>=a)&(pts[:,1]<a+0.008)&(pts[:,0]>-0.2)&(pts[:,0]<-0.15)]
    if len(s): print(f'y~{a:.3f}: n={len(s)} zmed={np.median(s[:,2]):.3f} zmax={s[:,2].max():.3f}')
"; python3 -c "
import numpy as np
d=np.load('eih2_cloud.npz'); P=d['P']; z=P[...,2]
m=(np.isfinite(z)&(z>0.43)&(z<0.51)&(P[...,0]>-0.30)&(P[...,0]<-0.135)&(P[...,1]<-0.05)&(P[...,1]>-0.3))
pts=P[m]; print('eih2 can-region', len(pts), 'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'ztop',pts[:,2].max().round(3))
for a in np.arange(-0.19,-0.11,0.008):
    s=pts[(pts[:,1]>=a)&(pts[:,1]<a+0.008)]
    if len(s): print(f'y~{a:.3f}: n={len(s)} zmed={np.median(s[:,2]):.3f} zmax={s[:,2].max():.3f} x[{s[:,0].min():.3f},{s[:,0].max():.3f}]')
"

# openrua op 44
cat > insp5.py <<'EOF'
from rob import *
from scipy.spatial.transform import Rotation as Rot
from moveit_msgs.srv import GetPositionFK
r=Robot('insp5')
def links(q, names=('panda_link5','panda_link7','panda_hand')):
    req=GetPositionFK.Request(); req.fk_link_names=list(names)
    req.robot_state.joint_state=r._seed(q)
    fut=r.fk.call_async(req); rclpy.spin_until_future_complete(r.node,fut,timeout_sec=30)
    return {n:np.round([ps.pose.position.x,ps.pose.position.y,ps.pose.position.z],3) for n,ps in zip(names,fut.result().pose_stamped)}
def q_roll(alpha):  # wrist displaced toward -y, fingers tilted in y-z plane
    R=Rot.from_euler('x',alpha).as_matrix()@np.diag([1,-1,-1])
    return tuple(Rot.from_matrix(R).as_quat())
def q_pitch(theta):  # wrist toward +x (theta>0) or -x (theta<0), fingers along y
    c,s=np.cos(theta),np.sin(theta)
    R=np.array([[c,0,-s],[0,-1,0],[-s,0,-c]])
    return tuple(Rot.from_matrix(R).as_quat())
x,y=-0.175,-0.154
q=r.joints()
for label,quat in [('vertical',Q_DOWN),('roll25',q_roll(np.radians(25))),('roll35',q_roll(np.radians(35))),('pitch-25',q_pitch(np.radians(-25))),('pitch20',q_pitch(np.radians(20)))]:
    print('---',label)
    seed=q
    for z in [0.65,0.60,0.55,0.50,0.45]:
        s=r.ik_tcp_world((x,y,z), quat=quat, seed=seed)
        if s is None: print(z,None); continue
        print(z, np.round(s,3), links(s) if z in (0.45,) else '')
        seed=s
EOF
timeout 300 python3 -u insp5.py 2>&1 | grep -v XMLPARSER

# openrua op 45
mkdir -p "$(dirname /workspace/pick_lying.py)"
cat > /workspace/pick_lying.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grasp the lying alphabet-soup can with a rolled hand (fingers close
across the can axis in the y-z plane, wrist displaced toward -y), then
drop it into the basket."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from rob import Robot, Q_DOWN
from pick_place import go, cart_path, log, BASKET

X, Y, ZC = -0.18, -0.154, 0.454          # can centre (world)
ROLL = np.radians(25)
GRASP_Z = 0.458
PRE_Z = 0.60
LIFT_Z = 0.76


def q_roll(alpha):
    R = Rot.from_euler('x', alpha).as_matrix() @ np.diag([1, -1, -1])
    return tuple(Rot.from_matrix(R).as_quat())


def main():
    r = Robot("pick_lying")
    QG = q_roll(ROLL)
    log("start", r.tcp_world().round(3), r.fingers())
    log("open", r.gripper(0.04))

    pre = cart_path(r, r.joints(), [(X, Y, LIFT_Z), (X, Y, PRE_Z)], QG)
    log("-> pre"); go(r, pre, 6.0)

    zs = np.linspace(PRE_Z, GRASP_Z, 5)[1:]
    down = cart_path(r, pre[-1], [(X, Y, z) for z in zs], QG)
    log("-> descend"); tcp = go(r, down, 3.0)
    err = np.linalg.norm(tcp - np.array([X, Y, GRASP_Z]))
    log(f"  tcp err={err:.4f}")
    if err > 0.012:
        log("  ABORT: descent blocked, lifting")
        go(r, cart_path(r, r.joints(), [(tcp[0], tcp[1], PRE_Z)], QG), 3.0)
        sys.exit(3)

    log("close"); res = r.gripper(0.0); log(" ", res)
    gap = abs(res[2][0]) + abs(res[2][1])
    log(f"  gap={gap:.4f}")
    if gap < 0.01:
        log("  GRASP FAILED"); sys.exit(2)

    up = cart_path(r, r.joints(), [(X, Y, GRASP_Z + 0.08), (X, Y, LIFT_Z)], QG)
    log("-> lift"); go(r, up, 3.0)
    log("  fingers", r.fingers())
    if abs(r.fingers()[0]) + abs(r.fingers()[1]) < 0.01:
        log("  DROPPED"); sys.exit(2)

    over = cart_path(r, r.joints(), [(BASKET[0], BASKET[1], LIFT_Z)], Q_DOWN)
    log("-> over basket"); go(r, over, 6.0)
    log("  fingers", r.fingers())
    lower = cart_path(r, r.joints(), [(BASKET[0], BASKET[1], 0.70)], Q_DOWN)
    log("-> lower"); go(r, lower, 2.0)
    log("open", r.gripper(0.04))
    back = cart_path(r, r.joints(), [(BASKET[0], BASKET[1], LIFT_Z + 0.04)], Q_DOWN)
    log("-> retreat"); go(r, back, 2.0)
    log("DONE", r.fingers())


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 46
nohup python3 -u pick_lying.py > lying.log 2>&1 &
sleep 80; grep -v XMLPARSER /workspace/lying.log

# openrua op 47
sleep 45; grep -v XMLPARSER /workspace/lying.log | tail -8; pgrep -f pick_lying.py >/dev/null && echo running || echo finished

# openrua op 48
sleep 5; pgrep -f pick_lying.py >/dev/null && echo running || echo finished; timeout 60 python3 tools/perception/cam_snap.py agentview final_agent.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py frontview final_front.png >/dev/null; timeout 120 python3 cloud.py birdview final_bird >/dev/null 2>&1; python3 -c "
import numpy as np, cv2
im=cv2.imread('final_agent.png'); cv2.imwrite('final_agent_crop.png', cv2.resize(im[170:360,430:640],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
d=np.load('final_bird_cloud.npz'); P=d['P']; z=P[...,2]
# anything left on the table outside the basket?
m=(np.isfinite(z)&(z>0.445)&(z<0.75)&(P[...,0]>-0.45)&(P[...,0]<0.25)&(np.abs(P[...,1])<0.45)&~((P[...,0]>-0.1)&(P[...,0]<0.1)&(P[...,1]>0.16)&(P[...,1]<0.36))).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(m)
print('objects outside basket:')
for i in range(1,n):
    if stats[i,4]<15: continue
    pts=P[lab==i]
    print(f'  x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={pts[:,2].max():.3f} centre=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f})')
# inside basket: points below rim height
b=(np.isfinite(z)&(P[...,0]>-0.07)&(P[...,0]<0.07)&(P[...,1]>0.19)&(P[...,1]<0.33))
zb=z[b]; print('basket interior z: min',zb.min().round(3),'median',np.median(zb).round(3),'max',zb.max().round(3))
"

# openrua op 49
pgrep -f pick_lying.py >/dev/null && echo still-running || echo finished; timeout 300 python3 -u home.py 2>&1 | grep -v XMLPARSER | tail -2; timeout 120 python3 cloud.py birdview final2_bird >/dev/null 2>&1; timeout 60 python3 tools/perception/cam_snap.py agentview final2_agent.png >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('final2_bird_cloud.npz'); P=d['P']; col=d['color']; z=P[...,2]
cv2.imwrite('final2_bird_crop.png', cv2.resize(col[240:330,340:420],None,fx=6,fy=6,interpolation=cv2.INTER_CUBIC))
b=(np.isfinite(z)&(P[...,0]>-0.06)&(P[...,0]<0.06)&(P[...,1]>0.20)&(P[...,1]<0.32))
zb=z[b]; print('basket interior z: min',zb.min().round(3),'median',np.median(zb).round(3),'max',zb.max().round(3), 'n',b.sum())
inside=(b&(z>0.44)&(z<0.62)); pts=P[inside]
print('contents pts',inside.sum(),'x',pts[:,0].min().round(3),pts[:,0].max().round(3),'y',pts[:,1].min().round(3),pts[:,1].max().round(3),'ztop',pts[:,2].max().round(3))
im=cv2.imread('final2_agent.png'); cv2.imwrite('final2_agent_crop.png', cv2.resize(im[170:360,430:640],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 50
pgrep -af "pick_|home.py|cloud.py" || echo "no pending processes"; tail -1 lying.log
