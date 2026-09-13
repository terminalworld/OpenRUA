#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
ls -la /workspace/tools /workspace/tools/* /workspace/.ros; cat /workspace/.gitignore

# openrua op 2
cat /workspace/tools/action/gripper_cmd.py /workspace/tools/perception/cam_snap.py /workspace/tools/perception/px2world.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview birdview sideview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" ; echo ---; timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; echo ---; timeout 30 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -8

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a world-frame height map from a camera; locate objects above the table."""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, typ, timeout=30.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
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
    cam = sys.argv[1] if len(sys.argv) > 1 else "birdview"
    rclpy.init()
    node = rclpy.create_node("scene")
    buf = Buffer(); TransformListener(buf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    v, u = np.mgrid[0:H, 0:W]
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    pw = pc @ R.T + T
    np.save(f"{cam}_pw.npy", pw)
    cv2.imwrite(f"{cam}_color.png", color)
    z = pw[..., 2]
    print("depth range", np.nanmin(depth), np.nanmax(depth))
    print("z percentiles", np.nanpercentile(z, [1, 5, 50, 95, 99]))
    # table height = mode of z
    hist, edges = np.histogram(z[np.isfinite(z)], bins=200)
    table = edges[np.argmax(hist)]
    print("table z ~", table)
    mask = (z > table + 0.02) & np.isfinite(z)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8))
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 30:
            continue
        m = lab == i
        pts = pw[m]
        print(f"blob {i}: px centroid ({cents[i][0]:.0f},{cents[i][1]:.0f}) area {stats[i,4]} "
              f"world xy min {pts[:,0].min():.3f},{pts[:,1].min():.3f} max {pts[:,0].max():.3f},{pts[:,1].max():.3f} "
              f"mean {pts[:,0].mean():.3f},{pts[:,1].mean():.3f} zmax {pts[:,2].max():.3f} "
              f"color {color[m].mean(0).astype(int)}")
    rclpy.shutdown()


main()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 scene.py birdview

# openrua op 8
python3 -c "
import numpy as np, cv2
pw=np.load('birdview_pw.npy'); color=cv2.imread('birdview_color.png')
z=pw[...,2]
zz=z[(z>0.5)&np.isfinite(z)]
hist,edges=np.histogram(zz,bins=300); table=edges[np.argmax(hist)]; print('table z',table)
mask=(z>table+0.015)&np.isfinite(z)
n,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8))
for i in range(1,n):
    if stats[i,4]<20: continue
    m=lab==i; pts=pw[m]
    print(f'blob {i}: px ({cents[i][0]:.0f},{cents[i][1]:.0f}) area {stats[i,4]} x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] mean {pts[:,0].mean():.3f},{pts[:,1].mean():.3f} zmax {pts[:,2].max():.3f} col {color[m].mean(0).astype(int)}')
"


# openrua op 9
python3 -c "
import numpy as np, cv2
pw=np.load('birdview_pw.npy'); color=cv2.imread('birdview_color.png')
z=pw[...,2]
m=(pw[...,0]>-0.02)&(pw[...,0]<0.12)&(pw[...,1]>-0.08)&(pw[...,1]<0.13)&(z>0.915)
pts=pw[m]
# height layers
for lo,hi in [(0.915,0.95),(0.95,0.98),(0.98,1.01),(1.01,1.04),(1.04,1.07)]:
    s=pts[(pts[:,2]>=lo)&(pts[:,2]<hi)]
    if len(s): print(f'z[{lo},{hi}) n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] mean {s[:,0].mean():.3f},{s[:,1].mean():.3f}')
# top-view visible surface: print the highest points region (lid)
top=pts[pts[:,2]>1.04]; print('top center',top[:,0].mean(),top[:,1].mean())
# crop
ys,xs=np.where(m); crop=color[ys.min()-10:ys.max()+10, xs.min()-10:xs.max()+10]
cv2.imwrite('pot_crop.png', cv2.resize(crop,None,fx=6,fy=6,interpolation=cv2.INTER_NEAREST))
"


# openrua op 10
timeout 120 python3 scene.py frontview 2>&1 | tail -3; timeout 120 python3 scene.py sideview 2>&1 | tail -3; python3 -c "
import numpy as np
for cam in ['frontview','sideview']:
    pw=np.load(cam+'_pw.npy'); z=pw[...,2]
    m=(pw[...,0]>-0.03)&(pw[...,0]<0.13)&(pw[...,1]>-0.09)&(pw[...,1]<0.14)&(z>0.905)&np.isfinite(z)
    pts=pw[m]; print(cam, len(pts))
    for lo in np.arange(0.905,1.07,0.01):
        s=pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.01)]
        if len(s): print(f'  z[{lo:.3f}) n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] w={s[:,0].max()-s[:,0].min():.3f}  y[{s[:,1].min():.3f},{s[:,1].max():.3f}] w={s[:,1].max()-s[:,1].min():.3f}')
"


# openrua op 11
python3 -c "
import numpy as np, cv2
for cam,(x0,x1,y0,y1) in {'frontview':(300,380,320,420),'sideview':(240,340,290,400),'agentview':(290,400,250,370)}.items():
    c=cv2.imread(cam+'.png')[y0:y1,x0:x1]
    cv2.imwrite(cam+'_crop.png', cv2.resize(c,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 12
mkdir -p "$(dirname /workspace/rb.py)"
cat > /workspace/rb.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small robot helper library: joint state, FK, IK, trajectory, gripper."""
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
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.660, 0.000, 0.912])  # from tf2_echo world panda_link0
TCP = M["hand"]["tcp_offset_m"]


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def down_quat(yaw):
    """Hand pointing straight down (hand +Z = world -Z), fingers closing
    along an axis rotated by `yaw` about world Z (yaw=0: fingers along
    world Y for the Panda hand convention, roughly)."""
    # Rotation: 180 deg about X, then yaw about Z
    # q = qz(yaw) * qx(pi)
    cx, sx = math.cos(math.pi / 2), math.sin(math.pi / 2)
    qx = np.array([sx, 0, 0, cx])
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    qz = np.array([0, 0, sz, cz])
    return quat_mul(qz, qx)


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


class Robot:
    def __init__(self, name="rb"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)

    def _on_js(self, m):
        self._js = m

    def _on_wr(self, m):
        self._wr = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin(0.2)
        f = self._wr.wrench.force
        t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def _seed(self, q=None):
        js = JointState()
        q = q if q is not None else self.arm_q()
        js.name = list(ARM)
        js.position = [float(v) for v in q]
        return js

    def fk_hand(self, q=None, link="panda_hand"):
        """Hand pose in the planning (base) frame -> returned in WORLD."""
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE_IN_WORLD
        quat = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, quat

    def ik_hand(self, pos_world, quat, seed=None, timeout=60):
        """IK for the hand frame at a WORLD pose. Returns joint list or None."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        pb = np.array(pos_world) - BASE_IN_WORLD
        p.position.x, p.position.y, p.position.z = map(float, pb)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK no answer")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp(self, tcp_world, quat, **kw):
        R = quat_R(*quat)
        hand = np.array(tcp_world) - TCP * R[:, 2]
        return self.ik_hand(hand, quat, **kw)

    def tcp_pose(self, q=None):
        pos, quat = self.fk_hand(q)
        R = quat_R(*quat)
        return pos + TCP * R[:, 2], quat

    def move(self, q, seconds=3.0, waypoints=None):
        """Send a trajectory to joint config q (optionally via waypoints list of (q, t))."""
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
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        qa = np.array(self.arm_q())
        err = np.abs(qa - np.array(q)).max()
        print(f"move done code={code} max_joint_err={err:.4f}", flush=True)
        return code, err

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f
OPENRUA_EOF

# openrua op 13
timeout 200 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
print('q',np.round(r.arm_q(),4))
pos,quat=r.fk_hand(); print('hand world',np.round(pos,4),'quat',np.round(quat,4))
tp,_=r.tcp_pose(); print('tcp world',np.round(tp,4))
print('R',np.round(rb.quat_R(*quat),3))
print('wrench',r.wrench())
print('fingers',r.fingers())
"

# openrua op 14
timeout 30 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A2 "At time" | head -4; timeout 30 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -3; timeout 20 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id|child_frame" | head; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child_frame" 

# openrua op 15
timeout 300 python3 -u -c "
import rb, numpy as np
rb.BASE_IN_WORLD=np.zeros(3)
r=rb.Robot()
q0=np.array(r.arm_q())
pos,quat=r.fk_hand(); print('fk raw',np.round(pos,4))
for label,p in [('world',pos),('base',pos-np.array([-0.66,0,0.912]))]:
    q=r.ik_hand(p,quat)
    print(label, None if q is None else np.round(np.array(q)-q0,3))
"

# openrua op 16
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rb.py", "old_string": "BASE_IN_WORLD = np.array([-0.660, 0.000, 0.912])  # from tf2_echo world panda_link0", "new_string": "# FK/IK services on this machine answer in the WORLD frame (verified:\\n# FK of the current config matches tf world->panda_hand), so no offset.\\nBASE_IN_WORLD = np.zeros(3)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 17
timeout 300 python3 -u -c "
import rb
r=rb.Robot()
r.gripper(0.04)
print(r.fingers())
"

# openrua op 18
python3 -c "
import numpy as np
pw=np.load('birdview_pw.npy'); z=pw[...,2]
m=(pw[...,0]>0.0)&(pw[...,0]<0.1)&(pw[...,1]>-0.07)&(pw[...,1]<-0.0)&(z>0.905)&np.isfinite(z)
pts=pw[m]
for y0 in np.arange(-0.06,0.0,0.005):
    s=pts[(pts[:,1]>=y0)&(pts[:,1]<y0+0.005)]
    if len(s): print(f'y[{y0:.3f}) n={len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] xmean {s[:,0].mean():.3f} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
"


# openrua op 19
timeout 300 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
q0=r.arm_q()
for yaw in [np.pi/2, -np.pi/2]:
    quat=rb.down_quat(yaw)
    print('yaw',yaw,'R',np.round(rb.quat_R(*quat),2).tolist())
    qa=r.ik_tcp([0.049,-0.028,1.15],quat,seed=q0)
    print(' pre',None if qa is None else np.round(qa,3))
    if qa is None: continue
    qb=r.ik_tcp([0.049,-0.028,1.02],quat,seed=qa)
    print(' grasp',None if qb is None else np.round(qb,3))
    if qb is not None:
        tp,tq=r.tcp_pose(qb); print(' fk tcp',np.round(tp,4))
"

# openrua op 20
timeout 600 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
q0=r.arm_q()
quat=rb.down_quat(-np.pi/2)
qa=r.ik_tcp([0.049,-0.028,1.15],quat,seed=q0)
print('target',np.round(qa,3))
r.move(qa,4.0)
print('q now',np.round(r.arm_q(),3))
tp,_=r.tcp_pose(); print('tcp',np.round(tp,4))
np.save('q_pre.npy',np.array(qa))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 21
timeout 600 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
qa=np.load('q_pre.npy')
r.move(qa,3.0)
print('q now',np.round(r.arm_q(),3))
tp,_=r.tcp_pose(); print('tcp',np.round(tp,4))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 22
timeout 120 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
pos,quat=r.fk_hand(); print('hand',np.round(pos,4),'quat',np.round(quat,4)); print(np.round(rb.quat_R(*quat),3))
print('target quat',np.round(rb.down_quat(-np.pi/2),4))
pos,quat=r.fk_hand(link='panda_link8'); print('link8 quat',np.round(quat,4))
"; timeout 20 ros2 run tf2_ros tf2_echo panda_link8 panda_hand 2>&1 | grep -A3 "At time"

# openrua op 23
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rb.py", "old_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "new_string": "        req.ik_request.group_name = M[\\"planning\\"][\\"group\\"]\\n        # default tip is panda_link8 (45 deg yaw off the hand); be explicit\\n        req.ik_request.ik_link_name = \\"panda_hand\\"\\n        req.ik_request.pose_stamped.header.frame_id = \\"\\"", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 24
timeout 600 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
q0=r.arm_q()
quat=rb.down_quat(-np.pi/2)
qa=r.ik_tcp([0.049,-0.028,1.15],quat,seed=q0)
print('target',np.round(qa,3))
pos,q=r.fk_hand(qa); print('fk hand quat',np.round(q,3), 'R', np.round(rb.quat_R(*q),2).tolist())
r.move(qa,3.0)
print('q now',np.round(r.arm_q(),3))
tp,_=r.tcp_pose(); print('tcp',np.round(tp,4))
np.save('q_pre.npy',np.array(qa))
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 25
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rb.py", "old_string": "        code = res.result().result.error_code\\n        qa = np.array(self.arm_q())\\n        err = np.abs(qa - np.array(q)).max()\\n        print(f\\"move done code={code} max_joint_err={err:.4f}\\", flush=True)\\n        return code, err", "new_string": "        code = res.result().result.error_code\\n        qa = np.array(self.arm_q())\\n        err = np.abs(qa - np.array(q)).max()\\n        print(f\\"move done code={code} max_joint_err={err:.4f}\\", flush=True)\\n        # controller lag on long goals: resend the (short) remainder\\n        if err > 0.01 and retries > 0:\\n            return self.move(q, max(1.5, seconds / 2), retries=retries - 1)\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 26
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rb.py", "old_string": "    def move(self, q, seconds=3.0, waypoints=None):", "new_string": "    def move(self, q, seconds=3.0, waypoints=None, retries=2):", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 27
timeout 600 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
qa=np.load('q_pre.npy')
r.move(qa,2.0)
tp,q=r.tcp_pose(); print('tcp',np.round(tp,4),'R',np.round(rb.quat_R(*q),2).tolist())
" 2>/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>/dev/null

# openrua op 28
timeout 120 python3 scene.py frontview 2>/dev/null | grep -E "blob (5|6|7)"; python3 -c "
import numpy as np
pw=np.load('frontview_pw.npy'); z=pw[...,2]
m=(pw[...,0]>0.0)&(pw[...,0]<0.12)&(pw[...,1]>-0.08)&(pw[...,1]<0.0)&(z>0.95)&(z<1.05)&np.isfinite(z)
pts=pw[m]
for y0 in np.arange(-0.06,0.0,0.005):
    s=pts[(pts[:,1]>=y0)&(pts[:,1]<y0+0.005)]
    if len(s): print(f'y[{y0:.3f}) n={len(s):3d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
print('by z:')
for z0 in np.arange(0.95,1.05,0.005):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.005)&(pts[:,1]<-0.012)]
    if len(s): print(f'z[{z0:.3f}) n={len(s):3d} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] x[{s[:,0].min():.3f},{s[:,0].max():.3f}]')
"

# openrua op 29
timeout 600 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
q0=np.array(r.arm_q())
quat=rb.down_quat(-np.pi/2)
qg=r.ik_tcp([0.050,-0.030,1.022],quat,seed=q0)
print('grasp q',np.round(qg,3),'delta',np.round(np.array(qg)-q0,3))
assert np.abs(np.array(qg)-q0).max()<0.6
r.move(qg,2.5)
tp,q=r.tcp_pose(); print('tcp',np.round(tp,4))
print('wrench',np.round(r.wrench()[0],2))
np.save('q_grasp.npy',np.array(qg))
" 2>/dev/null && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand 2>/dev/null && timeout 60 python3 tools/perception/cam_snap.py frontview 2>/dev/null

# openrua op 30
python3 -c "
import cv2
c=cv2.imread('frontview.png')[280:420,260:420]; cv2.imwrite('frontview_crop.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 31
timeout 600 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
f=r.gripper(0.0)
print('gap m',f[0]-f[1])
print('wrench',np.round(r.wrench()[0],2))
" 2>/dev/null && timeout 60 python3 tools/perception/cam_snap.py frontview 2>/dev/null && python3 -c "
import cv2
c=cv2.imread('frontview.png')[280:420,260:420]; cv2.imwrite('frontview_crop.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 32
timeout 600 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
q0=np.array(r.arm_q())
quat=rb.down_quat(-np.pi/2)
ql=r.ik_tcp([0.050,-0.030,1.20],quat,seed=q0)
print('delta',np.round(np.array(ql)-q0,3))
assert np.abs(np.array(ql)-q0).max()<0.6
r.move(ql,2.5)
tp,q=r.tcp_pose(); print('tcp',np.round(tp,4))
print('fingers',r.fingers()); print('wrench',np.round(r.wrench()[0],2))
np.save('q_lift.npy',np.array(ql))
" 2>/dev/null && timeout 60 python3 tools/perception/cam_snap.py frontview 2>/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview 2>/dev/null

# openrua op 33
python3 -c "
import numpy as np, cv2
pw=np.load('birdview_pw.npy'); c=cv2.imread('birdview_color.png')
b,g,r=c[...,0].astype(int),c[...,1].astype(int),c[...,2].astype(int)
red=(r>150)&(g<90)&(b<90)
pts=pw[red]; print('red n',len(pts),'center',np.round(pts[:,:2].mean(0),4),'z',np.round(pts[:,2].mean(),3))
# stove plate (blob) top
m=(pw[...,0]>-0.15)&(pw[...,0]<0.045)&(pw[...,1]>-0.3)&(pw[...,1]<-0.1)&(pw[...,2]>0.91)
s=pw[m]; print('plate x',s[:,0].min(),s[:,0].max(),'y',s[:,1].min(),s[:,1].max(),'z pct',np.percentile(s[:,2],[10,50,90,99]))
"

# openrua op 34
python3 -c "
import numpy as np, cv2
pw=np.load('birdview_pw.npy'); c=cv2.imread('birdview_color.png')
hsv=cv2.cvtColor(c,cv2.COLOR_BGR2HSV)
red=((hsv[...,0]<10)|(hsv[...,0]>170))&(hsv[...,1]>120)&(hsv[...,2]>80)
ys,xs=np.where(red); print('red n',len(ys),'px center',xs.mean(),ys.mean())
pts=pw[red]; print('world center',np.round(pts[:,:2].mean(0),4),'z',np.round(pts[:,2].mean(),3))
print('plate center', np.round([(-0.146+0.040)/2, (-0.294-0.107)/2],4))
"

# openrua op 35
python3 -c "
import numpy as np, cv2
c=cv2.imread('birdview_color.png'); print(c.shape, c[281,263], c[275:290:3,255:275:4].reshape(-1,3))
c2=cv2.imread('birdview.png'); print(c2[281,263])
"

# openrua op 36
python3 -c "
import numpy as np, cv2
pw=np.load('birdview_pw.npy'); c=cv2.imread('birdview_color.png').astype(int)
red=(c[...,2]>200)&(c[...,0]<190)&(c[...,2]-c[...,0]>50)
ys,xs=np.where(red); print('red n',len(ys),'px center',xs.mean(),ys.mean(), 'px range',xs.min(),xs.max(),ys.min(),ys.max())
pts=pw[red]; print('coil world center',np.round(pts[:,:2].mean(0),4),'z',np.round(np.percentile(pts[:,2],[10,50,90]),3))
"

# openrua op 37
timeout 120 python3 scene.py birdview 2>/dev/null | grep blob; python3 -c "
import numpy as np
pw=np.load('birdview_pw.npy'); z=pw[...,2]
m=(pw[...,0]>-0.05)&(pw[...,0]<0.15)&(pw[...,1]>-0.02)&(pw[...,1]<0.15)&(z>1.2)&(z<1.26)&np.isfinite(z)
pts=pw[m]; print('n',len(pts))
for z0 in np.arange(1.2,1.26,0.01):
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.01)]
    if len(s): print(f'z[{z0:.2f}) n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] mean {s[:,0].mean():.3f},{s[:,1].mean():.3f}')
"

# openrua op 38
timeout 900 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
quat=rb.down_quat(-np.pi/2)
q0=np.array(r.arm_q())
tx,ty=-0.050,-0.2655
q1=r.ik_tcp([tx,ty,1.20],quat,seed=q0); print('delta1',np.round(np.array(q1)-q0,3))
assert np.abs(np.array(q1)-q0).max()<1.0
r.move(q1,3.0)
tp,_=r.tcp_pose(); print('tcp',np.round(tp,4)); print('fingers',r.fingers())
q2=r.ik_tcp([tx,ty,1.10],quat,seed=np.array(r.arm_q())); print('delta2',np.round(np.array(q2)-np.array(q1),3))
assert np.abs(np.array(q2)-np.array(q1)).max()<0.6
r.move(q2,2.5)
tp,_=r.tcp_pose(); print('tcp',np.round(tp,4)); print('fingers',r.fingers()); print('wrench',np.round(r.wrench()[0],2))
" 2>/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview 2>/dev/null

# openrua op 39
timeout 900 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
quat=rb.down_quat(-np.pi/2)
q0=np.array(r.arm_q())
tx,ty=-0.050,-0.2655
q3=r.ik_tcp([tx,ty,1.062],quat,seed=q0); print('delta',np.round(np.array(q3)-q0,3))
assert np.abs(np.array(q3)-q0).max()<0.6
r.move(q3,2.0)
tp,_=r.tcp_pose(); print('tcp',np.round(tp,4)); print('fingers',r.fingers()); print('wrench',np.round(r.wrench()[0],2))
r.gripper(0.04)
print('wrench after release',np.round(r.wrench()[0],2))
# retreat straight up
q4=r.ik_tcp([tx,ty,1.20],quat,seed=np.array(r.arm_q()))
r.move(q4,2.5)
tp,_=r.tcp_pose(); print('tcp',np.round(tp,4))
" 2>/dev/null && timeout 60 python3 tools/perception/cam_snap.py agentview 2>/dev/null && timeout 60 python3 tools/perception/cam_snap.py frontview 2>/dev/null

# openrua op 40
timeout 120 python3 scene.py birdview 2>/dev/null >/dev/null; python3 -c "
import numpy as np
pw=np.load('birdview_pw.npy'); z=pw[...,2]
m=(pw[...,0]>-0.15)&(pw[...,0]<0.05)&(pw[...,1]>-0.30)&(pw[...,1]<-0.10)&(z>0.94)&(z<1.15)&np.isfinite(z)
pts=pw[m]; print('pot-on-stove points',len(pts),'zmax',pts[:,2].max())
top=pts[pts[:,2]>pts[:,2].max()-0.015]; print('knob center',np.round(top[:,:2].mean(0),4),'z',np.round(top[:,2].mean(),3))
lid=pts[(pts[:,2]>1.03)&(pts[:,2]<1.07)]; print('lid/top layer x',np.round([lid[:,0].min(),lid[:,0].max()],3),'y',np.round([lid[:,1].min(),lid[:,1].max()],3))
print('coil center was (-0.0533,-0.2015); stove plate x[-0.146,0.040] y[-0.294,-0.107]')
"; timeout 120 python3 scene.py frontview 2>/dev/null >/dev/null; python3 -c "
import numpy as np
pw=np.load('frontview_pw.npy'); z=pw[...,2]
m=(pw[...,0]>-0.15)&(pw[...,0]<0.06)&(pw[...,1]>-0.30)&(pw[...,1]<-0.10)&(z>0.935)&(z<1.10)&np.isfinite(z)
pts=pw[m]
print('frontview pot z range',np.round([pts[:,2].min(),pts[:,2].max()],3))
for z0 in [0.94,0.96,0.98,1.00,1.02,1.04,1.06]:
    s=pts[(pts[:,2]>=z0)&(pts[:,2]<z0+0.02)]
    if len(s): print(f' z[{z0:.2f}) xface {s[:,0].max():.3f} y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
"

# openrua op 41
timeout 900 python3 -u -c "
import rb, numpy as np
r=rb.Robot()
quat=rb.down_quat(-np.pi/2)
q0=np.array(r.arm_q())
q=r.ik_tcp([-0.25,-0.10,1.30],quat,seed=q0); print('delta',np.round(np.array(q)-q0,3))
r.move(q,3.0)
tp,_=r.tcp_pose(); print('tcp',np.round(tp,4))
" 2>/dev/null && timeout 120 python3 scene.py birdview 2>/dev/null | grep blob && python3 -c "
import numpy as np
pw=np.load('birdview_pw.npy'); z=pw[...,2]
m=(pw[...,0]>-0.15)&(pw[...,0]<0.05)&(pw[...,1]>-0.30)&(pw[...,1]<-0.10)&(z>0.94)&(z<1.15)&np.isfinite(z)
pts=pw[m]; print('pot-on-stove points',len(pts),'zmax',round(pts[:,2].max(),3))
top=pts[pts[:,2]>pts[:,2].max()-0.012]; print('knob center',np.round(top[:,:2].mean(0),4),'z',np.round(top[:,2].mean(),3),'(coil center -0.0533,-0.2015)')
" && timeout 60 python3 tools/perception/cam_snap.py agentview 2>/dev/null
