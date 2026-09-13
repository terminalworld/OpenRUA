#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
ros2 topic list && echo ---- && ros2 action list && echo ---- && ros2 service list | grep -iv parameter && echo ---- && ros2 node list

# openrua op 3
for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 4
python3 -c "
import cv2
im=cv2.imread('robot0_robotview.png')
cv2.imwrite('crop_cans.png', cv2.resize(im[0:120,130:430], None, fx=3, fy=3, interpolation=cv2.INTER_CUBIC))
cv2.imwrite('crop_box.png', cv2.resize(im[180:290,40:140], None, fx=4, fy=4, interpolation=cv2.INTER_CUBIC))
"

# openrua op 5
timeout 30 ros2 topic echo /tf_static --once 2>&1 | head -80; echo ----; timeout 20 ros2 topic echo /agentview/color/camera_info --once

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Build a world-frame point cloud from a camera and segment objects above the table.

Usage: python3 scene.py <camera>
Saves <camera>_cloud.npz (points Nx3 world, pixel indices) and prints
world→panda_link0 and clusters of above-table points.
"""
import struct
import sys

import numpy as np
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
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


def tf_mat(tfbuf, node, target, source):
    import time
    end = time.time() + 15
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform(target, source, Time()):
            break
    t = tfbuf.lookup_transform(target, source, Time())
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    H, W = depth_msg.height, depth_msg.width
    depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(H, W)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    T = tf_mat(tfbuf, node, "world", f"{cam}_optical_frame")
    try:
        Tb = tf_mat(tfbuf, node, "world", "panda_link0")
        print("world<-panda_link0:\n", np.round(Tb, 4))
    except Exception as e:
        print("no world<-panda_link0 tf:", e)
    vs, us = np.mgrid[0:H, 0:W]
    z = depth
    ok = np.isfinite(z) & (z > 0)
    X = (us - cx) * z / fx
    Y = (vs - cy) * z / fy
    P = np.stack([X, Y, z, np.ones_like(z)], -1) @ T.T
    P = P[..., :3]
    np.savez(f"{cam}_cloud.npz", P=P, ok=ok)
    print("camera pose world:", np.round(T[:3, 3], 3))
    pts = P[ok]
    # table height estimate: most common z
    hist, edges = np.histogram(pts[:, 2], bins=400)
    table_z = edges[np.argmax(hist)]
    print(f"table z ~ {table_z:.3f}")
    above = ok & (P[..., 2] > table_z + 0.015)
    # connected components in the image on the above mask
    import cv2
    n, lab = cv2.connectedComponents(above.astype(np.uint8))
    for i in range(1, n):
        m = lab == i
        if m.sum() < 30:
            continue
        p = P[m]
        vv, uu = np.where(m)
        print(f"cluster {i}: n={m.sum()} px u[{uu.min()}-{uu.max()}] v[{vv.min()}-{vv.max()}] "
              f"x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] "
              f"z[{p[:,2].min():.3f},{p[:,2].max():.3f}] centroid={np.round(p.mean(0),3)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 scene.py birdview

# openrua op 8
python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); P=d['P']; ok=d['ok']
pts=P[ok]
h,e=np.histogram(pts[:,2],bins=np.arange(-0.1,1.3,0.01))
for hh,ee in zip(h,e):
    if hh>200: print(f'{ee:.2f} {hh}')
"

# openrua op 9
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); P=d['P']; ok=d['ok']
table_z=0.42
above = ok & (P[...,2]>table_z+0.012) & (P[...,2]<0.9)
n,lab=cv2.connectedComponents(above.astype(np.uint8))
for i in range(1,n):
    m=lab==i
    if m.sum()<15: continue
    p=P[m]; vv,uu=np.where(m)
    print(f'cluster {i}: n={m.sum()} u[{uu.min()}-{uu.max()}] v[{vv.min()}-{vv.max()}] x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] c={np.round(p.mean(0),3)}')
"

# openrua op 10
timeout 120 python3 scene.py agentview 2>&1 | head -3; python3 -c "
import numpy as np, cv2
d=np.load('agentview_cloud.npz'); P=d['P']; ok=d['ok']
table_z=0.42
above = ok & (P[...,2]>table_z+0.012) & (P[...,2]<0.75) & (P[...,0]>-0.4)
n,lab=cv2.connectedComponents(above.astype(np.uint8))
for i in range(1,n):
    m=lab==i
    if m.sum()<15: continue
    p=P[m]; vv,uu=np.where(m)
    print(f'cluster {i}: n={m.sum()} u[{uu.min()}-{uu.max()}] v[{vv.min()}-{vv.max()}] x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] z[{p[:,2].min():.3f},{p[:,2].max():.3f}] c={np.round(p.mean(0),3)}')
"

# openrua op 11
python3 -c "
import numpy as np, cv2
for cam in ['birdview','agentview']:
    d=np.load(cam+'_cloud.npz'); P=d['P']; ok=d['ok']
    m = ok & (P[...,2]>0.432) & (P[...,2]<0.47) & (P[...,0]>0.05) & (P[...,0]<0.17) & (P[...,1]>-0.25) & (P[...,1]<-0.18)
    p=P[m][:,:2]; c=p.mean(0); 
    w,v=np.linalg.eigh(np.cov((p-c).T))
    ax=v[:,1]; yaw=np.degrees(np.arctan2(ax[1],ax[0]))
    proj=(p-c)@v
    print(cam, 'box center',np.round(c,3),'yaw deg',round(yaw,1),'extent', np.round(proj.max(0)-proj.min(0),3))
    m = ok & (P[...,2]>0.44) & (P[...,2]<0.52) & (P[...,0]>-0.26) & (P[...,0]<-0.15) & (P[...,1]>-0.23) & (P[...,1]<-0.12)
    p=P[m][:,:2]; print(cam,'can center', np.round(p.mean(0),3), 'extent', np.round(p.max(0)-p.min(0),3), 'ztop', P[m][:,2].max().round(3))
"


# openrua op 12
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable controller: joint state, FK, IK, trajectory, gripper for the Panda.

World frame -> panda_link0 frame: base sits at world (-0.51, 0, 0.42).
TCP targets are given in WORLD coordinates with the hand pointing down.
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])
TCP_OFF = float(M["hand"]["tcp_offset_m"])


def log(*a):
    print(time.strftime("%H:%M:%S"), *a, flush=True)


def down_quat(yaw=0.0):
    """Hand Z pointing down (-world z), fingers separate along world y when yaw=0.
    q = Rz(yaw) * Rx(pi)."""
    # Rx(pi) = (1,0,0,0); Rz(yaw) = (0,0,sin(yaw/2),cos(yaw/2))
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    # quaternion product Rz * Rx : (w1,x1,y1,z1)*(w2,x2,y2,z2)
    w1, x1, y1, z1 = cz, 0.0, 0.0, sz
    w2, x2, y2, z2 = 0.0, 1.0, 0.0, 0.0
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


class Ctl:
    def __init__(self, name="ctl"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 10)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.traj = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik.wait_for_service(10)
        self.fk.wait_for_service(10)
        self.traj.wait_for_server(10)
        self.grip.wait_for_server(10)
        self.joints()

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    def spin(self, t=0.3):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self._js = {}
        while not self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in ARM]

    def finger_gap(self):
        js = self.joints()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    def _seed(self, q=None):
        q = q if q is not None else self.arm_q()
        s = JointState()
        s.name = list(ARM)
        s.position = [float(v) for v in q]
        return s

    def fk_hand(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]),
                r.pose_stamped[0].header.frame_id, r.error_code.val)

    def tcp_world(self, q=None):
        """World position of the fingertip point (hand + TCP_OFF along hand z)."""
        p, quat, frame, code = self.fk_hand(q)
        x, y, z, w = quat
        zaxis = np.array([2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)])
        return p + TCP_OFF * zaxis + BASE_IN_WORLD, quat

    def ik_tcp_world(self, xyz, yaw=0.0, seed=None, tries=5):
        """IK for TCP at world xyz with hand pointing down. Returns arm q or None."""
        qx, qy, qz, qw = down_quat(yaw)
        R = quat_R(qx, qy, qz, qw)
        hand = np.array(xyz, dtype=float) - TCP_OFF * R[:, 2] - BASE_IN_WORLD
        for i in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, hand)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
            req.ik_request.robot_state.joint_state = self._seed(seed)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                q = [sol[j] for j in ARM]
                return q
            log(f"IK attempt {i} failed code={None if r is None else r.error_code.val}")
            # perturb seed
            base = seed if seed is not None else self.arm_q()
            seed = list(np.array(base) + np.random.uniform(-0.3, 0.3, 7))
        return None

    def move_q(self, q, secs=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        if via:
            n = len(via) + 1
            for i, v in enumerate(via):
                t = secs * (i + 1) / n
                pt = JointTrajectoryPoint(positions=[float(x) for x in v])
                pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
                pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        fut = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        res = rf.result()
        code = res.result.error_code if res else None
        self.spin(0.3)
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        log(f"move done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, xyz, yaw=0.0, secs=3.0, seed=None):
        q = self.ik_tcp_world(xyz, yaw, seed)
        if q is None:
            log(f"IK FAILED for {xyz}")
            return None
        code, err = self.move_q(q, secs)
        tcp, _ = self.tcp_world()
        log(f"tcp now {np.round(tcp, 4)} target {np.round(xyz, 4)}")
        return q

    def gripper(self, width, timeout=300):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=timeout)
        res = rf.result()
        self.spin(0.5)
        gap = self.finger_gap()
        log(f"gripper({width}) reached={res.result.reached_goal if res else None} "
            f"stalled={res.result.stalled if res else None} gap={gap:.4f}")
        return gap


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


if __name__ == "__main__":
    c = Ctl("ctl_cli")
    cmd = sys.argv[1]
    if cmd == "fk":
        p, q, f, code = c.fk_hand()
        print("hand in", f, p, q, code)
        print("tcp world", c.tcp_world()[0])
        print("joints", c.joints())
    elif cmd == "ik":
        print(c.ik_tcp_world([float(v) for v in sys.argv[2:5]],
                             float(sys.argv[5]) if len(sys.argv) > 5 else 0.0))
    elif cmd == "go":
        c.move_tcp([float(v) for v in sys.argv[2:5]],
                   float(sys.argv[5]) if len(sys.argv) > 5 else 0.0,
                   float(sys.argv[6]) if len(sys.argv) > 6 else 3.0)
    elif cmd == "grip":
        c.gripper(float(sys.argv[2]))
    elif cmd == "q":
        c.move_q([float(v) for v in sys.argv[2].split(",")],
                 float(sys.argv[3]) if len(sys.argv) > 3 else 3.0)
OPENRUA_EOF

# openrua op 13
timeout 120 python3 ctl.py fk

# openrua op 14
timeout 200 python3 -c "
import ctl, numpy as np, rclpy
from moveit_msgs.srv import GetPositionIK
from builtin_interfaces.msg import Duration
c=ctl.Ctl('iktest')
p,q,f,code=c.fk_hand()
cur=np.array(c.arm_q())
def solve(pos):
    req=GetPositionIK.Request(); req.ik_request.group_name='panda_arm'
    req.ik_request.pose_stamped.header.frame_id=''
    pp=req.ik_request.pose_stamped.pose
    pp.position.x,pp.position.y,pp.position.z=map(float,pos)
    pp.orientation.x,pp.orientation.y,pp.orientation.z,pp.orientation.w=map(float,q)
    req.ik_request.robot_state.joint_state=c._seed()
    req.ik_request.timeout=Duration(sec=1)
    fut=c.ik.call_async(req); rclpy.spin_until_future_complete(c.node,fut,timeout_sec=60)
    r=fut.result()
    if r.error_code.val!=1: return 'fail', r.error_code.val
    sol=dict(zip(r.solution.joint_state.name,r.solution.joint_state.position))
    return np.round(np.array([sol[j] for j in ctl.ARM])-cur,3)
print('as-is (world coords):', solve(p))
print('base coords:', solve(p-ctl.BASE_IN_WORLD))
"

# openrua op 15
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "BASE_IN_WORLD = np.array([-0.51, 0.0, 0.42])", "new_string": "# Verified empirically: /compute_fk and /compute_ik on this machine work in\\n# the WORLD frame (model root is `world`), so no base offset is applied.\\nBASE_IN_WORLD = np.array([0.0, 0.0, 0.0])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 16
timeout 300 python3 -c "
import ctl, numpy as np
c=ctl.Ctl('ikcheck')
print('tcp now', np.round(c.tcp_world()[0],3))
W = {
 'can_above': ([-0.21,-0.174,0.62],0.0),
 'can_grasp': ([-0.21,-0.174,0.46],0.0),
 'basket_above': ([0.01,0.255,0.78],0.0),
 'box_above': ([0.103,-0.213,0.60],-0.262),
 'box_grasp': ([0.103,-0.213,0.44],-0.262),
}
seed=None
for k,(p,yaw) in W.items():
    q=c.ik_tcp_world(p,yaw,seed)
    print(k, None if q is None else np.round(q,3))
    if q is not None:
        tcp,_=c.tcp_world(q); print('   fk check', np.round(tcp,4))
"

# openrua op 17
mkdir -p "$(dirname /workspace/pick.py)"
cat > /workspace/pick.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick an object with a top-down grasp and drop it into the basket.

Usage: python3 -u pick.py <x> <y> <grasp_z> <yaw_rad> <drop_x> <drop_y>
"""
import subprocess
import sys

import numpy as np

import ctl
from ctl import log

x, y, gz, yaw, dx, dy = map(float, sys.argv[1:7])
ABOVE_Z = 0.62
CARRY_Z = 0.78
DROP_Z = 0.78

c = ctl.Ctl("pick")
log("start tcp", np.round(c.tcp_world()[0], 3), "gap", round(c.finger_gap(), 4))

# 1. open
c.gripper(ctl.GRIP["open_m"])

# 2. above the object
q_above = c.move_tcp([x, y, ABOVE_Z], yaw, 4.0)
if q_above is None:
    sys.exit("IK above failed")

# 3. descend in two steps, seeded for branch continuity
q_prev = q_above
for z in (gz + 0.08, gz):
    q = c.ik_tcp_world([x, y, z], yaw, seed=q_prev)
    if q is None:
        sys.exit(f"IK failed at z={z}")
    jump = np.abs(np.array(q) - np.array(q_prev)).max()
    log(f"descend to z={z:.3f} max joint jump {jump:.3f}")
    if jump > 1.0:
        sys.exit("branch flip on descent; aborting for safety")
    c.move_q(q, 2.0)
    q_prev = q
tcp, _ = c.tcp_world()
log("at grasp height tcp", np.round(tcp, 4))
subprocess.run([sys.executable, "tools/perception/cam_snap.py", "robot0_eye_in_hand",
                "eih_pregrasp.png"], timeout=120)

# 4. close and check
gap = c.gripper(ctl.GRIP["closed_m"])
if gap < 0.01:
    log("WARNING: gripper closed on air (gap %.4f)" % gap)

# 5. lift straight up, then carry to the basket
q_lift = c.ik_tcp_world([x, y, ABOVE_Z], yaw, seed=q_prev)
c.move_q(q_lift, 2.5)
log("after lift gap", round(c.finger_gap(), 4))
q_carry = c.ik_tcp_world([x, y, CARRY_Z], yaw, seed=q_lift)
c.move_q(q_carry, 2.0)
q_drop = c.ik_tcp_world([dx, dy, DROP_Z], 0.0, seed=q_carry)
if q_drop is None:
    sys.exit("IK drop failed")
c.move_q(q_drop, 4.0)
tcp, _ = c.tcp_world()
log("over basket tcp", np.round(tcp, 4), "gap", round(c.finger_gap(), 4))

# 6. release
c.gripper(ctl.GRIP["open_m"])
log("released; tcp", np.round(c.tcp_world()[0], 3))
subprocess.run([sys.executable, "tools/perception/cam_snap.py", "agentview",
                "after_drop.png"], timeout=120)
log("DONE")
OPENRUA_EOF

# openrua op 18
nohup python3 -u pick.py -0.21 -0.174 0.46 0.0 -0.01 0.245 > pick_can.log 2>&1 &
sleep 45; cat /workspace/pick_can.log

# openrua op 19
sleep 40; cat /workspace/pick_can.log | tail -12

# openrua op 20
timeout 200 python3 -u -c "
import ctl, numpy as np
c=ctl.Ctl('settle')
q=c.ik_tcp_world([0.0,0.23,0.85],0.0)
c.move_q(q,2.0)
" && timeout 60 python3 tools/perception/cam_snap.py agentview after_settle.png

# openrua op 21
nohup python3 -u pick.py 0.103 -0.213 0.44 -0.262 0.04 0.28 > pick_box.log 2>&1 &
sleep 50; cat /workspace/pick_box.log

# openrua op 22
sleep 35; tail -8 /workspace/pick_box.log

# openrua op 23
timeout 200 python3 -u -c "
import ctl, numpy as np
c=ctl.Ctl('settle2')
q=c.ik_tcp_world([-0.1,0.1,0.85],0.0)
c.move_q(q,3.0)
" && for cam in agentview frontview birdview; do timeout 60 python3 tools/perception/cam_snap.py $cam final_$cam.png; done

# openrua op 24
python3 -c "
import cv2
im=cv2.imread('final_birdview.png')
cv2.imwrite('crop_basket.png', cv2.resize(im[255:325,340:420], None, fx=6, fy=6, interpolation=cv2.INTER_CUBIC))
"; timeout 120 python3 scene.py birdview > /dev/null; python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); P=d['P']; ok=d['ok']
# basket interior footprint
m = ok & (P[...,0]>-0.05)&(P[...,0]<0.07)&(P[...,1]>0.19)&(P[...,1]<0.32)
z=P[m][:,2]
print('inside-basket z range', z.min().round(3), z.max().round(3))
h,e=np.histogram(z,bins=np.arange(0.4,0.7,0.01))
for hh,ee in zip(h,e):
    if hh>5: print(f'{ee:.2f} {hh}')
# anything left on table near the original box / can spots?
for name,(x0,x1,y0,y1) in {'box_spot':(0.05,0.17,-0.26,-0.16),'can_spot':(-0.26,-0.15,-0.23,-0.12)}.items():
    m = ok & (P[...,0]>x0)&(P[...,0]<x1)&(P[...,1]>y0)&(P[...,1]<y1)&(P[...,2]>0.435)
    print(name,'above-table px:', m.sum())
"

# openrua op 25
pgrep -af "pick.py|ctl.py" || echo "no pending processes"
