#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -v -E "parameter|describe|get_type" ; echo ----; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
for p in "330 305" "420 300" "245 270" "180 222" "320 240" "290 160" "300 300"; do echo -n "birdview $p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 6
timeout 30 ros2 topic echo /birdview/color/camera_info --once | head -30; timeout 30 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12

# openrua op 7
python3 -c "
import numpy as np
d=np.load('birdview.npy'); f=579.4112549695428
H=3.0-d  # height in world
v,u=np.mgrid[0:480,0:640]
X=-0.2+(v-240)*d/f; Y=(u-320)*d/f
# table height
print('table z sample', H[400,320])
# objects above table by >2cm, within table region
mask=(H>0.93)&(X>-0.3)&(X<0.6)
# print ASCII map at 2cm resolution
xs=np.arange(-0.6,0.5,0.02); ys=np.arange(-0.8,0.6,0.02)
print('     y=', ' '.join(f'{y:+.2f}'[1:4] if i%5==0 else '   ' for i,y in enumerate(ys)))
for x in xs:
    row=''
    for y in ys:
        m=(np.abs(X-x)<0.01)&(np.abs(Y-y)<0.01)
        if not m.any(): row+=' .'; continue
        h=H[m].max()
        if h<0.905: row+=' .'
        elif h<0.95: row+=' -'
        elif h<1.0: row+=' o'
        elif h<1.05: row+=' O'
        elif h<1.15: row+=' #'
        else: row+=' @'
    print(f'x={x:+.2f}',row)
"


# openrua op 8
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && ls *.npy

# openrua op 9
python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); f=579.4112549695428
H=3.0-d  # height in world
v,u=np.mgrid[0:480,0:640]
X=-0.2+(v-240)*d/f; Y=(u-320)*d/f
print('table z sample', H[400,320], H[300,300])
xs=np.arange(-0.6,0.5,0.02); ys=np.arange(-0.8,0.6,0.02)
print('        ', ''.join(f'{y:+.2f}'[1:4].ljust(2) if i%5==0 else '  ' for i,y in enumerate(ys)))
for x in xs:
    row=''
    for y in ys:
        m=(np.abs(X-x)<0.01)&(np.abs(Y-y)<0.01)
        if not m.any(): row+=' .'; continue
        h=H[m].max()
        if h<0.905: row+=' .'
        elif h<0.95: row+=' -'
        elif h<1.0: row+=' o'
        elif h<1.05: row+=' O'
        elif h<1.15: row+=' #'
        else: row+=' @'
    print(f'x={x:+.2f}',row)
"


# openrua op 10
for c in galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 11
timeout 15 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; timeout 15 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -8; timeout 15 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | head -8

# openrua op 12
python3 -c "
import numpy as np
np.set_printoptions(linewidth=250, precision=3, suppress=True)
d=np.load('birdview_depth.npy'); f=579.4112549695428
H=3.0-d
# yellow mug region: pixel u 300-360, v 280-330
print('yellow mug heights (rows v=280..330 step 2, cols u=300..360 step 2)')
print(H[280:332:2, 300:362:2])
"


# openrua op 13
timeout 60 python3 tools/perception/cam_snap.py /sideview/depth/image_raw sideview_depth.png && timeout 20 ros2 topic echo /sideview/color/camera_info --once | grep -A9 "^k:" && timeout 15 ros2 run tf2_ros tf2_echo world sideview_optical_frame 2>&1 | grep -A5 Matrix

# openrua op 14
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""World-frame point cloud from a camera's depth + intrinsics + TF.

Usage: python3 cloud.py <camera> <out.npz>
Saves arrays X, Y, Z (world metres, HxW) and the color image.
"""
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam, out = sys.argv[1], sys.argv[2]
    rclpy.init()
    node = rclpy.create_node("cloud")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "bgr8")
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y,
                  t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    v, u = np.mgrid[0:h, 0:w]
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    pw = pc @ R.T + T
    np.savez(out, X=pw[..., 0], Y=pw[..., 1], Z=pw[..., 2], depth=depth,
             color=color)
    print(out, "saved; cam at", T)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 15
timeout 90 python3 cloud.py sideview side.npz && timeout 90 python3 cloud.py agentview agent.npz && python3 -c "
import numpy as np
for f in ['side.npz','agent.npz']:
    d=np.load(f); X,Y,Z=d['X'],d['Y'],d['Z']
    # yellow mug region: x in [-0.05,0.1], y in [-0.08,0.1], z>0.905
    m=(X>-0.06)&(X<0.11)&(Y>-0.09)&(Y<0.11)&(Z>0.902)&(Z<1.2)&np.isfinite(Z)
    print(f, 'yellow mug pts', m.sum(), 'z range', Z[m].min(), Z[m].max(), 'x', X[m].min(), X[m].max(), 'y', Y[m].min(), Y[m].max())
    # histogram of z
    hist,edges=np.histogram(Z[m],bins=np.arange(0.9,1.1,0.01)); print(list(zip(np.round(edges[:-1],2),hist)))
    # width at several heights
    for z0 in [0.92,0.95,0.98,1.01,1.04,1.06]:
        mm=m&(np.abs(Z-z0)<0.005)
        if mm.sum()>5: print(' z',z0,'x',round(X[mm].min(),3),round(X[mm].max(),3),'y',round(Y[mm].min(),3),round(Y[mm].max(),3))
    # gray mug
    m=(X>-0.06)&(X<0.11)&(Y>0.25)&(Y<0.45)&(Z>0.902)&(Z<1.2)&np.isfinite(Z)
    print(' gray mug z', Z[m].min(), Z[m].max(), 'x', X[m].min(), X[m].max(), 'y', Y[m].min(), Y[m].max())
"


# openrua op 16
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 17
timeout 90 python3 cloud.py birdview bird.npz && timeout 90 python3 cloud.py agentview agent.npz && python3 -c "
import numpy as np
for f in ['bird.npz','agent.npz']:
    d=np.load(f); X,Y,Z=d['X'],d['Y'],d['Z']
    m=(X>-0.08)&(X<0.13)&(Y>-0.1)&(Y<0.13)&(Z>0.902)&(Z<1.2)&np.isfinite(Z)
    print(f, 'yellow mug pts', m.sum(), 'z range', Z[m].min(), Z[m].max(), 'x', X[m].min(), X[m].max(), 'y', Y[m].min(), Y[m].max())
    for z0 in [0.91,0.92,0.93,0.94,0.95,0.96,0.97,0.98,0.99]:
        mm=m&(np.abs(Z-z0)<0.004)
        if mm.sum()>5: print(' z',z0,'n',mm.sum(),'x',round(X[mm].min(),3),round(X[mm].max(),3),'y',round(Y[mm].min(),3),round(Y[mm].max(),3))
    m=(X>-0.08)&(X<0.13)&(Y>0.25)&(Y<0.45)&(Z>0.902)&(Z<1.2)&np.isfinite(Z)
    print(' gray mug z', Z[m].min(), Z[m].max())
"


# openrua op 18
timeout 15 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | grep -A6 "Translation" | head -8; timeout 20 ros2 topic echo /robot0_eye_in_hand/color/camera_info --once | grep -A9 "^k:"

# openrua op 19
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable control helpers for this Panda: IK, trajectories, gripper, TF.

Import from scripts; all poses are WORLD frame unless noted.
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
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_T = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 (measured via TF)


def log(*a):
    print(*a, flush=True)


class Robot:
    def __init__(self, name="ctl"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self.js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.wait_js()

    def _on_js(self, m):
        self.js = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self, timeout=20):
        end = time.time() + timeout
        self.js = None
        while self.js is None and time.time() < end:
            self.spin(0.2)
        if self.js is None:
            raise RuntimeError("no /joint_states")
        return self.js

    def joints(self, fresh=True):
        js = self.wait_js() if fresh else self.js
        d = dict(zip(js.name, js.position))
        return [d[j] for j in ARM]

    def fingers(self, fresh=True):
        js = self.wait_js() if fresh else self.js
        d = dict(zip(js.name, js.position))
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    # ---------- kinematics ----------
    def hand_pose(self):
        """world -> panda_hand as (xyz, quat xyzw) via TF."""
        end = time.time() + 10
        while time.time() < end:
            self.spin(0.1)
            if self.tfbuf.can_transform("world", "panda_hand", rclpy.time.Time()):
                break
        t = self.tfbuf.lookup_transform("world", "panda_hand", rclpy.time.Time())
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), np.array([q.x, q.y, q.z, q.w])

    def fk(self, joints):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in joints]
        self.fk_cli.wait_for_service(timeout_sec=10)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        xyz = np.array([p.position.x, p.position.y, p.position.z]) + BASE_T
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return xyz, q

    def ik(self, xyz_world, quat, seed=None, at_tcp=False, timeout=30, attempts=3):
        """IK for the panda_hand frame (or fingertip point when at_tcp)."""
        xyz = np.array(xyz_world, dtype=float)
        if at_tcp:
            R = Rot.from_quat(quat).as_matrix()
            xyz = xyz - TCP * R[:, 2]
        xyz_base = xyz - BASE_T
        seed = self.joints() if seed is None else list(seed)
        self.ik_cli.wait_for_service(timeout_sec=10)
        for k in range(attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, xyz_base)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1, nanosec=0)
            fut = self.ik_cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
            code = None if res is None else res.error_code.val
            log(f"  ik attempt {k+1} failed code={code} for {np.round(xyz,3)}")
            # perturb seed slightly for another try
            seed = [s + np.random.uniform(-0.2, 0.2) for s in seed]
        return None

    # ---------- motion ----------
    def move_joints(self, waypoints, times):
        """waypoints: list of 7-lists; times: cumulative seconds per point."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for wp, t in zip(waypoints, times):
            for j, (v, lim) in enumerate(zip(wp, LIMITS)):
                if not (lim[0] - 1e-6 <= v <= lim[1] + 1e-6):
                    raise RuntimeError(f"joint {j+1} target {v:.3f} outside {lim}")
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=900)
        if rf.result() is None:
            log("  trajectory result timeout (client side)")
            return None
        code = rf.result().result.error_code
        err = np.array(self.joints()) - np.array(waypoints[-1])
        log(f"  traj done code={code} max_joint_err={np.abs(err).max():.4f}")
        return code

    def move_pose(self, xyz, quat, seconds=4.0, at_tcp=False, seed=None):
        q = self.ik(xyz, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            raise RuntimeError(f"IK failed for {np.round(xyz,3)}")
        code = self.move_joints([q], [seconds])
        return q, code

    def move_path(self, poses, quat, seconds_per=2.0, at_tcp=False, first_seconds=None):
        """Sequence of cartesian waypoints -> one joint trajectory."""
        seed = self.joints()
        wps, times = [], []
        t = 0.0
        for i, xyz in enumerate(poses):
            q = self.ik(xyz, quat, seed=seed, at_tcp=at_tcp)
            if q is None:
                raise RuntimeError(f"IK failed for waypoint {i} {np.round(xyz,3)}")
            # continuity check
            jump = np.abs(np.array(q) - np.array(seed)).max()
            if jump > 1.5:
                log(f"  warning: big joint jump {jump:.2f} at waypoint {i}")
            wps.append(q)
            t += (first_seconds if (i == 0 and first_seconds) else seconds_per)
            times.append(t)
            seed = q
        return self.move_joints(wps, times)

    def gripper(self, width):
        if not self.grip.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result().result
        f = self.fingers()
        log(f"  gripper -> reached={res.reached_goal} stalled={res.stalled} fingers={f}")
        return f

    def servo(self, lin, ang=(0, 0, 0), ticks=20, frame=None):
        msg = TwistStamped()
        msg.header.frame_id = frame or TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(ticks):
            self.twist_pub.publish(msg)
            self.spin(0.05)


def quat_down(yaw_deg=0.0):
    """Hand pointing straight down; yaw rotates the finger axis about world z.
    yaw=0: fingers close along world y (hand x = world x)."""
    return (Rot.from_euler("z", yaw_deg, degrees=True) * Rot.from_quat([1, 0, 0, 0])).as_quat()


def quat_from_axes(approach, finger_axis):
    """Quaternion for a hand whose z (approach) and y (finger closing) axes are given in world."""
    z = np.array(approach, float); z /= np.linalg.norm(z)
    y = np.array(finger_axis, float); y -= z * (y @ z); y /= np.linalg.norm(y)
    x = np.cross(y, z)
    R = np.stack([x, y, z], axis=1)
    return Rot.from_matrix(R).as_quat()
OPENRUA_EOF

# openrua op 20
timeout 120 python3 -c "
from ctl import *
r=Robot('test')
j=r.joints(); print('joints',np.round(j,3))
print('tf hand',r.hand_pose())
print('fk hand',r.fk(j))
q=r.ik([-0.203,0,1.27],[1,0,-0.028,0]); print('ik back',np.round(q,3) if q else q)
print('quat_down(0)',np.round(quat_down(0),3),'quat_down(90)',np.round(quat_down(90),3))
print('side approach -y, fingers x:',np.round(quat_from_axes([0,-1,0],[1,0,0]),3))
"


# openrua op 21
timeout 120 python3 -c "
import ctl
from ctl import *
ctl.BASE_T[:]=0
r=Robot('test')
j=r.joints()
print('fk hand',r.fk(j))
q=r.ik([-0.203,0,1.27],[1,0,-0.028,0]); print('ik back',np.round(q,3) if q else q)
"


# openrua op 22
timeout 120 python3 -c "
import ctl
from ctl import *
ctl.BASE_T[:]=0
r=Robot('test')
j=r.joints()
q=r.ik([-0.203,0,1.27],[1,0,-0.028,0]); print('ik',np.round(q,3))
print('fk of ik',r.fk(q))
q=r.ik([-0.203,0,1.27],[1,0,0,0]); print('ik2',np.round(q,3)); print('fk of ik2',r.fk(q))
"


# openrua op 23
python3 - <<'EOF'
import re
s=open('ctl.py').read()
s=s.replace('BASE_T = np.array([-0.660, 0.0, 0.912])  # world -> panda_link0 (measured via TF)',
 'BASE_T = np.array([0.0, 0.0, 0.0])  # measured: IK/FK services here work in WORLD coords\n# IK tip link is panda_link8, which is panda_hand rotated +45deg about z\nHAND_TO_LINK8 = Rot.from_euler("z", 45, degrees=True)')
s=s.replace('''            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)''',
'''            q8 = (Rot.from_quat(quat) * HAND_TO_LINK8).as_quat()
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)''')
open('ctl.py','w').write(s)
EOF
timeout 120 python3 -c "
from ctl import *
r=Robot('test')
q=r.ik([-0.203,0,1.27],[1,0,0,0]); print('ik',np.round(q,3)); print('fk of ik',r.fk(q))
q=r.ik([-0.13,-0.45,1.05],quat_down(90)); print('ik',np.round(q,3)); print('fk of ik',r.fk(q), 'want', quat_down(90))
"

# openrua op 24
timeout 300 python3 -u -c "
from ctl import *
r=Robot('look')
q=quat_from_axes([0,1,0],[1,0,0]); print('quat',np.round(q,3))
for y in [-0.62,-0.58,-0.55]:
    sol=r.ik([-0.13,y,1.0],q)
    print(y, None if sol is None else np.round(sol,3))
    if sol: break
r.move_joints([sol],[4.0])
print('hand',r.hand_pose())
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand look1.png

# openrua op 25
timeout 600 python3 -u -c "
from ctl import *
r=Robot('iktest')
import itertools
seed=r.joints()
tests={
 'horiz +y fingers x': quat_from_axes([0,1,0],[1,0,0]),
 'horiz +y fingers -x': quat_from_axes([0,1,0],[-1,0,0]),
 'horiz +y fingers z': quat_from_axes([0,1,0],[0,0,1]),
 'tilt45 +y/down fingers x': quat_from_axes([0,0.707,-0.707],[1,0,0]),
 'tilt30 +y/down fingers x': quat_from_axes([0,0.5,-0.866],[1,0,0]),
 'down fingers x': quat_down(90),
}
for name,q in tests.items():
    for pos in [[-0.13,-0.55,1.0],[-0.13,-0.5,1.0],[-0.13,-0.45,1.05],[-0.2,-0.5,1.0]]:
        sol=r.ik(pos,q,seed=seed,attempts=2)
        print(name,pos,'OK' if sol else 'fail', np.round(sol,2) if sol else '')
"


# openrua op 26
timeout 600 python3 -u -c "
from ctl import *
r=Robot('look')
seed=r.joints()
for ap,pos in [([0,0.5,-0.866],[-0.13,-0.55,1.08]),([0,0.5,-0.866],[-0.13,-0.52,1.05]),([0,0.6,-0.8],[-0.13,-0.5,1.05]),([0,0.5,-0.866],[-0.13,-0.5,1.0])]:
    q=quat_from_axes(ap,[1,0,0])
    sol=r.ik(pos,q,seed=seed,attempts=2)
    if sol:
        print('using',ap,pos,np.round(sol,2))
        # sanity: avoid elbow-flipped weird configs
        r.move_joints([sol],[4.0])
        print('hand',r.hand_pose())
        break
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand look1.png

# openrua op 27
timeout 60 python3 -c "
from ctl import *
r=Robot('chk'); print(np.round(r.joints(),3)); print(r.hand_pose())
"; timeout 60 python3 tools/perception/cam_snap.py agentview a2.png

# openrua op 28
timeout 600 python3 -u -c "
from ctl import *
r=Robot('look')
sol=[-0.62,1.23,-0.41,-0.86,0.72,1.58,-1.82]
r.move_joints([sol],[6.0])
print('joints',np.round(r.joints(),3)); print('hand',r.hand_pose())
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand look1.png

# openrua op 29
timeout 600 python3 -u -c "
from ctl import *
r=Robot('look')
sol=[-0.62,1.23,-0.41,-0.86,0.72,1.58,-1.82]
for i in range(3):
    code=r.move_joints([sol],[6.0])
    j=r.joints(); print('joints',np.round(j,3))
    if np.abs(np.array(j)-np.array(sol)).max()<0.02: break
print('hand',r.hand_pose())
" && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand look1.png

# openrua op 30
timeout 90 python3 cloud.py robot0_eye_in_hand eih.npz && python3 -c "
import numpy as np
d=np.load('eih.npz'); X,Y,Z=d['X'],d['Y'],d['Z']
ok=np.isfinite(Z)&(d['depth']>0.05)
# points inside microwave footprint x in [-0.25,0.0], y in [-0.34,-0.13]
m=ok&(X>-0.25)&(X<-0.01)&(Y>-0.33)&(Y<-0.14)
print('cavity pts',m.sum())
hist,e=np.histogram(Z[m],bins=np.arange(0.85,1.15,0.01)); print([(round(a,2),b) for a,b in zip(e[:-1],hist) if b])
# floor: lowest cluster
zf=Z[m]; print('floor z candidates', np.percentile(zf,[1,5,10,25,50]))
# opening edge: points near y=-0.34
m2=ok&(X>-0.25)&(X<-0.01)&(Y>-0.37)&(Y<-0.31)
hist,e=np.histogram(Z[m2],bins=np.arange(0.85,1.15,0.01)); print('front region',[(round(a,2),b) for a,b in zip(e[:-1],hist) if b])
# table in front
m3=ok&(X>-0.25)&(X<-0.01)&(Y>-0.45)&(Y<-0.37)
print('table front z', np.percentile(Z[m3],[5,50,95]) if m3.sum() else None)
# x extent of cavity floor
mf=m&(Z<0.96)&(Z>0.9)
print('floor x range',X[mf].min(),X[mf].max(),'y range',Y[mf].min(),Y[mf].max(), 'z', Z[mf].mean())
# side walls: points with Z between 0.95 and 1.05 and X near edges
mw=ok&(Y>-0.33)&(Y<-0.15)&(Z>0.95)&(Z<1.05)
print('wall x hist', np.histogram(X[mw],bins=np.arange(-0.3,0.15,0.01)))
"


# openrua op 31
timeout 900 python3 -u -c "
from ctl import *
r=Robot('iktest')
seed=r.joints()
bar=np.array([-0.13,-0.367,1.01])
for th in [0,15,20,25,30,35,40,45,50,60]:
    t=np.radians(th); a=np.array([0,np.cos(t),-np.sin(t)])
    for fing,name in [([1,0,0],'fx'),([0,np.sin(t),np.cos(t)],'fz')]:
        q=quat_from_axes(a,fing)
        hand=bar-0.1034*a
        sol=r.ik(hand,q,seed=seed,attempts=2)
        print(f'theta={th} fingers={name} hand={np.round(hand,3)}', 'OK '+str(np.round(sol,2)) if sol else 'fail')
"


# openrua op 32
python3 -c "
import numpy as np
d=np.load('bird.npz'); X,Y,Z=d['X'],d['Y'],d['Z']
ok=np.isfinite(Z)
# door: tall (z>1.0) points with y<-0.35
m=ok&(Y<-0.345)&(Y>-0.7)&(Z>0.95)&(X>-0.5)&(X<0.2)
print('door pts',m.sum(),'z',Z[m].min(),Z[m].max())
for y0 in np.arange(-0.36,-0.66,-0.03):
    mm=m&(np.abs(Y-y0)<0.01)
    if mm.sum(): print(f' y={y0:.2f} x range {X[mm].min():.3f}..{X[mm].max():.3f} n={mm.sum()} zmax={Z[mm].max():.3f}')
# microwave body top outline
m=ok&(Z>1.09)&(X>-0.4)&(X<0.2)&(Y>-0.345)&(Y<0.0)
print('body x',X[m].min(),X[m].max(),'y',Y[m].min(),Y[m].max(),'ztop',Z[m].max())
# mug now
m=ok&(X>-0.08)&(X<0.13)&(Y>-0.1)&(Y<0.13)&(Z>0.905)&(Z<1.2)
print('mug x',X[m].min(),X[m].max(),'y',Y[m].min(),Y[m].max(),'z',Z[m].min(),Z[m].max())
# rim: z>0.995
mr=m&(Z>0.995); print('rim x',X[mr].min(),X[mr].max(),'y',Y[mr].min(),Y[mr].max(),'center',(X[mr].min()+X[mr].max())/2,(Y[mr].min()+Y[mr].max())/2)
# handle: y< rim ymin
mh=m&(Y<Y[mr].min()-0.003)
print('handle pts',mh.sum(),'x',X[mh].min(),X[mh].max(),'y',Y[mh].min(),Y[mh].max(),'z',Z[mh].min(),Z[mh].max())
"


# openrua op 33
timeout 900 python3 -u -c "
from ctl import *
r=Robot('push')
r.gripper(0.0)   # close fingers to make a pusher
q=quat_down(90)  # vertical, fingers along x
tip_hi=[0.015,-0.075,1.06]; tip_lo=[0.015,-0.075,0.925]; tip_end=[0.015,0.045,0.925]; tip_up=[0.015,0.045,1.06]
seed=r.joints()
sols=[]
for p in [tip_hi,tip_lo,tip_end,tip_up]:
    s=r.ik(p,q,seed=seed,at_tcp=True); print(p, np.round(s,2) if s else None); 
    if s is None: raise SystemExit('ik fail')
    sols.append(s); seed=s
print('fk check', r.fk(sols[1]))
r.move_joints([sols[0]],[5.0])
r.move_joints([sols[1]],[3.0])
r.move_joints([sols[2]],[4.0])
r.move_joints([sols[3]],[3.0])
print('hand',r.hand_pose())
" 2>&1 | tail -20

# openrua op 34
ls -la /workspace/.ros/log/; tail -5 /workspace/.ros/log/*.log | head -80

# openrua op 35
timeout 600 python3 -u -c "
from ctl import *
from rosgraph_msgs.msg import Clock
r=Robot('diag')
clk={}
r.node.create_subscription(Clock, '/clock', lambda m: clk.__setitem__('t', m.clock.sec+m.clock.nanosec*1e-9), 10)
for _ in range(10): r.spin(0.1)
print('clock', clk.get('t'))
j0=r.joints(); print('j0',np.round(j0,3))
tgt=list(j0); tgt[0]+=0.3
import time
goal=FollowJointTrajectory.Goal(); goal.trajectory.joint_names=list(ARM)
pt=JointTrajectoryPoint(positions=[float(v) for v in tgt]); pt.time_from_start=Duration(sec=3)
goal.trajectory.points=[pt]
fb=[]
def on_fb(m): fb.append((clk.get('t'), m.feedback.actual.positions[0], m.feedback.desired.positions[0]))
send=r.fjt.send_goal_async(goal, feedback_callback=on_fb)
rclpy.spin_until_future_complete(r.node, send, timeout_sec=30)
gh=send.result(); rf=gh.get_result_async()
t0=time.time()
while not rf.done():
    r.spin(0.2)
    if int((time.time()-t0)*5)%10==0: pass
print('wall',time.time()-t0,'code',rf.result().result.error_code,'clock',clk.get('t'))
print('fb samples',len(fb)); 
for s in fb[::max(1,len(fb)//15)]: print(np.round(s,3))
print('j now',np.round(r.joints(),3))
for i in range(5):
    r.spin(0.5); time.sleep(0.5)
    print('clock',clk.get('t'),'j1',round(r.joints()[0],3))
"


# openrua op 36
timeout 600 python3 -u -c "
from ctl import *
r=Robot('diag')
j0=r.joints(); print('j0',np.round(j0,3))
tgt=list(j0); tgt[0]=0.593
code=r.move_joints([tgt,tgt],[3.0,8.0])
print('j now',np.round(r.joints(),3), 'target', np.round(tgt,3))
"


# openrua op 37
python3 - <<'EOF'
s=open('/workspace/ctl.py').read()
s=s.replace('''    def move_joints(self, waypoints, times):
        """waypoints: list of 7-lists; times: cumulative seconds per point."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for wp, t in zip(waypoints, times):''','''    def move_joints(self, waypoints, times, hold=4.0):
        """waypoints: list of 7-lists; times: cumulative seconds per point.
        The controller lags; a `hold` segment at the final target lets it settle
        while the sim clock runs (without it goals end with tolerance errors)."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        waypoints = list(waypoints) + [waypoints[-1]]
        times = list(times) + [times[-1] + hold]
        for wp, t in zip(waypoints, times):''')
s=s.replace('''    def move_pose(self, xyz, quat, seconds=4.0, at_tcp=False, seed=None):
        q = self.ik(xyz, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            raise RuntimeError(f"IK failed for {np.round(xyz,3)}")
        code = self.move_joints([q], [seconds])
        return q, code''','''    def move_pose(self, xyz, quat, seconds=None, at_tcp=False, seed=None, hold=4.0):
        q = self.ik(xyz, quat, seed=seed, at_tcp=at_tcp)
        if q is None:
            raise RuntimeError(f"IK failed for {np.round(xyz,3)}")
        if seconds is None:
            dist = np.abs(np.array(q) - np.array(self.joints())).max()
            seconds = max(2.0, dist / 0.5)
        code = self.move_joints([q], [seconds], hold=hold)
        return q, code''')
open('/workspace/ctl.py','w').write(s)
EOF
grep -n "hold" /workspace/ctl.py | head

# openrua op 38
cat > /workspace/scene.py <<'EOF'
#!/usr/bin/env python3
"""Snapshot birdview and report mug / door / robot positions."""
import subprocess, sys
import numpy as np
subprocess.run([sys.executable, "/workspace/cloud.py", "birdview", "/workspace/bird.npz"], check=True, capture_output=True)
d = np.load("/workspace/bird.npz"); X, Y, Z = d["X"], d["Y"], d["Z"]
ok = np.isfinite(Z)

def region(name, xr, yr, zr=(0.905, 1.03)):
    m = ok & (X > xr[0]) & (X < xr[1]) & (Y > yr[0]) & (Y < yr[1]) & (Z > zr[0]) & (Z < zr[1])
    if m.sum() < 5:
        print(f"{name}: not found"); return None
    mr = m & (Z > Z[m].max() - 0.012)   # rim points
    cx, cy = (X[mr].min() + X[mr].max()) / 2, (Y[mr].min() + Y[mr].max()) / 2
    print(f"{name}: rim center=({cx:.3f},{cy:.3f}) z=[{Z[m].min():.3f},{Z[m].max():.3f}] "
          f"x=[{X[m].min():.3f},{X[m].max():.3f}] y=[{Y[m].min():.3f},{Y[m].max():.3f}] n={m.sum()}")
    # handle = points outside rim circle radius 0.05
    mh = m & (np.hypot(X - cx, Y - cy) > 0.052)
    if mh.sum() > 5:
        hx, hy = X[mh].mean(), Y[mh].mean()
        ang = np.degrees(np.arctan2(hy - cy, hx - cx))
        print(f"   handle: mean=({hx:.3f},{hy:.3f}) dir={ang:.0f}deg (0=+x, -90=-y) z=[{Z[mh].min():.3f},{Z[mh].max():.3f}] n={mh.sum()}")
    return cx, cy

yl = region("yellow mug", (-0.3, 0.3), (-0.12, 0.25))
gr = region("gray mug", (-0.2, 0.3), (0.25, 0.5))
# door: points with y<-0.35 near x~-0.25, z>0.95
m = ok & (Y < -0.355) & (Y > -0.75) & (Z > 0.95) & (Z < 1.12) & (X > -0.45) & (X < 0.15)
if m.sum() > 20:
    print(f"door(open part): x=[{X[m].min():.3f},{X[m].max():.3f}] y=[{Y[m].min():.3f},{Y[m].max():.3f}] n={m.sum()}")
    for y0 in (-0.40, -0.50, -0.58):
        mm = m & (np.abs(Y - y0) < 0.01)
        if mm.sum(): print(f"   y={y0}: x=[{X[mm].min():.3f},{X[mm].max():.3f}]")
else:
    print("door: nothing beyond y<-0.355 (closed?)  n=", m.sum())
# front face of microwave region
m = ok & (Y < -0.33) & (Y > -0.40) & (Z > 0.95) & (Z < 1.12) & (X > -0.3) & (X < 0.1)
print(f"front strip y[-0.40,-0.33]: n={m.sum()} y=[{Y[m].min() if m.sum() else 0:.3f},{Y[m].max() if m.sum() else 0:.3f}]")
EOF
timeout 120 python3 /workspace/scene.py; timeout 60 python3 -c "
from ctl import *
r=Robot('c'); print('hand',r.hand_pose()); print('fingers',r.fingers()); print('joints',np.round(r.joints(),3))"

# openrua op 39
timeout 60 python3 tools/perception/cam_snap.py agentview a3.png && timeout 60 python3 tools/perception/cam_snap.py birdview b3.png && python3 -c "
import cv2; a=cv2.imread('a3.png'); b=cv2.imread('b3.png'); print(a.shape,b.shape)"

# openrua op 40
for c in agentview sideview frontview; do timeout 60 python3 cloud.py $c ${c}3.npz; done; python3 - <<'EOF'
import numpy as np
for c in ["agentview3","sideview3","frontview3","bird"]:
    d=np.load(f"/workspace/{c}.npz"); X,Y,Z=d["X"],d["Y"],d["Z"]; ok=np.isfinite(Z)
    m=ok&(Y<-0.33)&(Y>-0.5)&(X>-0.3)&(X<0.15)&(Z>0.9)&(Z<1.12)
    print(c,"front pts",m.sum())
    if m.sum()<20: continue
    # histogram of y among front points
    ys=Y[m]; h,e=np.histogram(ys,bins=np.arange(-0.50,-0.32,0.01)); print("  y hist",[(round(e[i],2),int(h[i])) for i in range(len(h)) if h[i]>0])
    # protruding parts: y<-0.365
    mp=m&(Y<-0.365)
    if mp.sum()>5:
        print(f"  protruding: x=[{X[mp].min():.3f},{X[mp].max():.3f}] y=[{Y[mp].min():.3f},{Y[mp].max():.3f}] z=[{Z[mp].min():.3f},{Z[mp].max():.3f}] n={mp.sum()}")
        for z0 in np.arange(0.92,1.10,0.02):
            mm=mp&(np.abs(Z-z0)<0.01)
            if mm.sum(): print(f"    z={z0:.2f}: x=[{X[mm].min():.3f},{X[mm].max():.3f}] y=[{Y[mm].min():.3f},{Y[mm].max():.3f}] n={mm.sum()}")
EOF

# openrua op 41
cat > /workspace/door_open.py <<'EOF'
#!/usr/bin/env python3
"""Open the microwave door: pinch the handle bar (fingers along the door's
x axis, hand tilted THETA below horizontal) and swing it about the hinge,
rotating the hand with the door."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from ctl import Robot, quat_from_axes, log

H = np.array([-0.262, -0.35])           # hinge axis (world x,y)
B0 = np.array([-0.015, -0.399, 1.01])   # handle bar centre, door closed
THETA = np.radians(45)
A0 = np.array([0.0, np.cos(THETA), -np.sin(THETA)])   # approach: +y and down
FX0 = np.array([1.0, 0.0, 0.0])                       # finger closing axis
PAD = 0.012        # tip goes this far past bar centre so pads sit on the bar
PHI_MAX = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0


def pose(phi_deg, back=0.0, dz=0.0):
    Rz = Rot.from_euler("z", -phi_deg, degrees=True)
    a, fx = Rz.apply(A0), Rz.apply(FX0)
    rel = Rz.apply([B0[0] - H[0], B0[1] - H[1], 0.0])
    bar = np.array([H[0] + rel[0], H[1] + rel[1], B0[2] + dz])
    return bar + (PAD - back) * a, quat_from_axes(a, fx), a


r = Robot("door")
dry = len(sys.argv) > 1 and sys.argv[1] == "dry"

# ---- plan everything first ----
seed = r.joints()
plan = []
pre = [(0.0, 0.16, 0.20), (0.0, 0.16, 0.0), (0.0, 0.08, 0.0), (0.0, 0.03, 0.0), (0.0, 0.0, 0.0)]
for phi, back, dz in pre:
    tip, q, a = pose(phi, back, dz)
    sol = r.ik(tip, q, seed=seed, at_tcp=True)
    log(f"pre phi={phi} back={back} dz={dz} tip={np.round(tip,3)} -> {None if sol is None else np.round(sol,2)}")
    if sol is None:
        sys.exit("IK failed in approach")
    plan.append(sol); seed = sol
swing = []
phis = np.arange(10.0, PHI_MAX + 0.1, 10.0)
for phi in phis:
    tip, q, a = pose(phi)
    sol = r.ik(tip, q, seed=seed, at_tcp=True)
    jump = None if sol is None else np.abs(np.array(sol) - np.array(seed)).max()
    log(f"swing phi={phi} tip={np.round(tip,3)} -> {None if sol is None else np.round(sol,2)} jump={jump}")
    if sol is None:
        log("stopping swing plan here"); phis = phis[:len(swing)]; break
    swing.append(sol); seed = sol
if dry:
    sys.exit(0)

# ---- execute ----
log("== move to high pre-pose")
r.move_joints([plan[0]], [max(3.0, np.abs(np.array(plan[0]) - np.array(r.joints())).max() / 0.4)])
log("== open gripper"); r.gripper(0.04)
log("== descend to pre-grasp"); r.move_joints([plan[1]], [4.0])
log("== approach"); r.move_joints(plan[2:], [2.0, 4.0, 6.0])
log("hand", np.round(r.hand_pose()[0], 3))
log("== close gripper"); f = r.gripper(0.0)
if f[0] < 0.004:
    log("!! fingers closed fully - missed the bar"); sys.exit(1)
log("== swing")
times = [3.0 + 2.0 * i for i in range(len(swing))]
r.move_joints(swing, times, hold=3.0)
log("== release"); r.gripper(0.04)
tip, q, a = pose(phis[-1], back=0.10)
sol = r.ik(tip, q, at_tcp=True)
if sol is not None:
    r.move_joints([sol], [3.0])
tip, q, a = pose(phis[-1], back=0.16, dz=0.20)
sol = r.ik(tip, q, at_tcp=True)
if sol is not None:
    r.move_joints([sol], [3.0])
log("done; fingers", r.fingers())
EOF
timeout 600 python3 /workspace/door_open.py dry 90

# openrua op 42
sed -i 's/pre = \[(0.0, 0.16, 0.20), (0.0, 0.16, 0.0), (0.0, 0.08, 0.0), (0.0, 0.03, 0.0), (0.0, 0.0, 0.0)\]/pre = [(0.0, 0.10, 0.12), (0.0, 0.10, 0.0), (0.0, 0.06, 0.0), (0.0, 0.03, 0.0), (0.0, 0.0, 0.0)]/' door_open.py && sed -i 's/tip, q, a = pose(phis\[-1\], back=0.16, dz=0.20)/tip, q, a = pose(phis[-1], back=0.10, dz=0.12)/' door_open.py && timeout 600 python3 door_open.py dry 90

# openrua op 43
python3 - <<'EOF'
import re
s=open('door_open.py').read()
s=s.replace('THETA = np.radians(45)','THETA = np.radians(float(sys.argv[3]) if len(sys.argv) > 3 else 80.0)')
s=s.replace('B0 = np.array([-0.015, -0.399, 1.01])','B0 = np.array([-0.015, -0.399, 0.975])')
s=s.replace('pre = [(0.0, 0.10, 0.12), (0.0, 0.10, 0.0), (0.0, 0.06, 0.0), (0.0, 0.03, 0.0), (0.0, 0.0, 0.0)]',
            'pre = [(0.0, 0.08, 0.10), (0.0, 0.08, 0.0), (0.0, 0.05, 0.0), (0.0, 0.025, 0.0), (0.0, 0.0, 0.0)]')
s=s.replace('tip, q, a = pose(phis[-1], back=0.10, dz=0.12)','tip, q, a = pose(phis[-1], back=0.08, dz=0.10)')
open('door_open.py','w').write(s)
EOF
for th in 90 80 70; do echo "== theta $th"; timeout 600 python3 door_open.py dry 90 $th 2>&1 | grep -v "ik attempt"; done

# openrua op 44
python3 - <<'EOF'
s=open('door_open.py').read()
s=s.replace('B0 = np.array([-0.015, -0.399, 0.975])','B0 = np.array([-0.015, -0.399, 0.992])')
s=s.replace('pre = [(0.0, 0.08, 0.10),','pre = [(0.0, 0.08, 0.15),')
s=s.replace('''log("== move to high pre-pose")''','''log("== lift straight up first")
hx, hq = r.hand_pose()
lift = r.ik(hx + [0, 0, 0.12], hq)
if lift is None:
    sys.exit("lift IK failed")
r.move_joints([lift], [4.0])
log("== move to high pre-pose")''')
open('door_open.py','w').write(s)
EOF
timeout 900 python3 door_open.py run 90 80 2>&1 | grep -v "ik attempt"

# openrua op 45
sed -i 's/pre = \[(0.0, 0.08, 0.15),/pre = [(0.0, 0.08, 0.12),/' door_open.py && timeout 900 python3 door_open.py run 90 80 2>&1 | grep -v "ik attempt"

# openrua op 46
timeout 120 python3 scene.py; timeout 60 python3 tools/perception/cam_snap.py agentview a4.png

# openrua op 47
python3 - <<'EOF'
s=open('door_open.py').read()
s=s.replace('B0 = np.array([-0.015, -0.399, 0.992])','B0 = np.array([-0.015, -0.399, 1.062])   # pinch near the top so the housing clears the bar top')
s=s.replace('if f[0] < 0.004:','if f[0] < 0.007:')
open('door_open.py','w').write(s)
EOF
timeout 900 python3 door_open.py run 90 90 2>&1 | grep -v "ik attempt"

# openrua op 48
timeout 120 python3 scene.py; timeout 60 python3 tools/perception/cam_snap.py agentview a5.png

# openrua op 49
cat > /workspace/push_mug.py <<'EOF'
#!/usr/bin/env python3
"""Push the yellow mug +y with the closed, vertical gripper (fingers along x)."""
import sys
import numpy as np
from ctl import Robot, quat_down, log

r = Robot("push")
Q = quat_down(90)          # vertical, finger axis along world x
MX = 0.015                 # mug/handle x
Y0, Y1 = -0.07, 0.08       # tip start / end y
ZP = 0.925

def ik_or_die(p, seed=None):
    s = r.ik(p, Q, seed=seed, at_tcp=True)
    if s is None:
        sys.exit(f"IK failed at {p}")
    return s

seed = r.joints()
approach = []
for p in [(-0.15, -0.45, 1.30), (-0.05, -0.25, 1.22), (MX, Y0, 1.12)]:
    s = ik_or_die(p, seed)
    log(f"wp {p} -> {np.round(s,2)} jump={np.abs(np.array(s)-np.array(seed)).max():.2f}")
    approach.append(s); seed = s
down = ik_or_die((MX, Y0, ZP), seed)
push = []
seed = down
for y in np.linspace(Y0, Y1, 6)[1:]:
    s = ik_or_die((MX, y, ZP), seed); push.append(s); seed = s
up = ik_or_die((MX, Y1, 1.10), seed)
if "dry" in sys.argv:
    sys.exit(0)
log("== close gripper"); r.gripper(0.0)
log("== approach"); r.move_joints(approach, [5.0, 9.0, 13.0])
log("hand", np.round(r.hand_pose()[0], 3))
log("== descend"); r.move_joints([down], [4.0])
log("== push"); r.move_joints(push, [2.0 + 1.5 * i for i in range(len(push))], hold=3.0)
log("hand", np.round(r.hand_pose()[0], 3))
log("== up"); r.move_joints([up], [3.0])
EOF
timeout 900 python3 push_mug.py 2>&1 | grep -v "ik attempt" && timeout 120 python3 scene.py

# openrua op 50
timeout 300 python3 -c "
from ctl import *
r=Robot('m'); s=r.ik((0.10,0.0,1.30), quat_down(90), at_tcp=True); print(s); r.move_joints([s],[4.0])
" 2>&1 | grep -v attempt; timeout 120 python3 scene.py

# openrua op 51
python3 - <<'EOF'
s=open('/workspace/ctl.py').read()
s=s.replace('''        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")''','''        if any(wp is None for wp in waypoints):
            raise RuntimeError("move_joints got a None waypoint (IK failed)")
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")''')
open('/workspace/ctl.py','w').write(s)
EOF
timeout 300 python3 -c "
from ctl import *
r=Robot('m'); s=r.ik((-0.10,0.0,1.30), quat_down(90), at_tcp=True); print(s); r.move_joints([s],[4.0])
" 2>&1 | grep -v attempt; timeout 120 python3 scene.py

# openrua op 52
python3 - <<'EOF'
import numpy as np
d=np.load('/workspace/bird.npz'); X,Y,Z=d['X'],d['Y'],d['Z']; ok=np.isfinite(Z)
cx,cy=0.022,0.171
m=ok&(np.abs(X-cx)<0.06)&(Y>0.07)&(Y<0.13)&(Z>0.915)&(Z<1.0)&(np.hypot(X-cx,Y-cy)>0.05)
print("handle pts",m.sum(), "x",X[m].min(),X[m].max(),"y",Y[m].min(),Y[m].max(),"z",Z[m].min(),Z[m].max())
for y0 in np.arange(0.084,0.126,0.005):
    mm=m&(np.abs(Y-y0)<0.0025)
    if mm.sum(): print(f" y={y0:.3f}: x=[{X[mm].min():.3f},{X[mm].max():.3f}] z=[{Z[mm].min():.3f},{Z[mm].max():.3f}] n={mm.sum()}")
# body edge on -y side at several z
mb=ok&(np.abs(X-cx)<0.02)&(Y>0.10)&(Y<0.17)&(Z>0.905)&(Z<1.01)
for z0 in (0.93,0.95,0.97,0.99,1.0):
    mm=mb&(np.abs(Z-z0)<0.006)
    if mm.sum(): print(f" body z={z0}: ymin={Y[mm].min():.3f}")
EOF

# openrua op 53
cat > /workspace/mug_in.py <<'EOF'
#!/usr/bin/env python3
"""Grasp the yellow mug by its handle (fingers along x, hand tilted 45deg,
approaching from -y/above) and put it into the microwave cavity."""
import sys
import numpy as np
from ctl import Robot, quat_from_axes, log

TH = np.radians(45)
A = np.array([0.0, np.cos(TH), -np.sin(TH)])
Q = quat_from_axes(A, [1.0, 0.0, 0.0])
G = np.array([0.0215, 0.099, 0.958])        # fingertip at grasp
CARRY_Z = 1.18                              # tip z carrying over the microwave
FRONT_Y = -0.48                             # tip y in front of the door opening
CAV_X = -0.10                               # tip/mug x inside cavity
IN_Y = -0.317                               # tip y when mug centre is at -0.245
IN_Z = 1.015

r = Robot("mug")

def ik(p, seed=None):
    s = r.ik(p, Q, seed=seed, at_tcp=True)
    if s is None:
        sys.exit(f"IK failed at {np.round(p,3)}")
    return s

def chain(points, seed):
    out = []
    for p in points:
        s = ik(p, seed)
        log(f"  wp {np.round(p,3)} jump={np.abs(np.array(s)-np.array(seed)).max():.2f}")
        out.append(s); seed = s
    return out

seed = r.joints()
pre = chain([G - 0.10 * A + [0, 0, 0.12], G - 0.10 * A], seed)
app = chain([G - 0.06 * A, G - 0.03 * A, G], pre[-1])
lift = chain([G + [0, 0, 0.07], G + [0, 0, CARRY_Z - G[2]]], app[-1])
carry = chain([(-0.04, -0.10, CARRY_Z), (CAV_X, -0.30, CARRY_Z), (CAV_X, FRONT_Y, CARRY_Z)], lift[-1])
down = chain([(CAV_X, FRONT_Y, 1.10), (CAV_X, FRONT_Y, IN_Z)], carry[-1])
ins = chain([(CAV_X, -0.42, IN_Z), (CAV_X, -0.37, IN_Z), (CAV_X, IN_Y, IN_Z)], down[-1])
ret = chain([(CAV_X, -0.38, IN_Z), (CAV_X, -0.44, IN_Z), np.array([CAV_X, -0.44, IN_Z]) - 0.10 * A], ins[-1])
if "dry" in sys.argv:
    sys.exit(0)

log("== open"); r.gripper(0.04)
log("== pre-grasp"); r.move_joints(pre, [6.0, 10.0])
log("== approach"); r.move_joints(app, [2.0, 4.0, 6.0])
log("hand", np.round(r.hand_pose()[0], 3))
log("== close"); f = r.gripper(0.0)
if not (0.005 < f[0] < 0.02):
    log("!! grasp failed"); sys.exit(1)
log("== lift"); r.move_joints(lift, [4.0, 8.0])
input_ok = True
log("== carry"); r.move_joints(carry, [5.0, 10.0, 14.0])
log("== descend"); r.move_joints(down, [3.0, 6.0])
log("== insert"); r.move_joints(ins, [3.0, 6.0, 9.0], hold=3.0)
log("hand", np.round(r.hand_pose()[0], 3), "fingers", r.fingers())
log("== release"); r.gripper(0.04)
log("== retreat"); r.move_joints(ret, [3.0, 6.0, 9.0])
log("done")
EOF
timeout 600 python3 mug_in.py dry 2>&1 | grep -v "ik attempt"

# openrua op 54
sed -i 's/^FRONT_Y = -0.48 .*/FRONT_Y = -0.47                             # tip y in front of the door opening/; s/^CAV_X = -0.10 .*/CAV_X = -0.13                               # tip\/mug x inside cavity/' mug_in.py && timeout 600 python3 mug_in.py dry 2>&1 | grep -v "ik attempt"

# openrua op 55
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "ik attempt"
import numpy as np
from ctl import Robot, quat_from_axes
r=Robot('probe')
for th in (45,55,65):
    a=np.array([0,np.cos(np.radians(th)),-np.sin(np.radians(th))]); Q=quat_from_axes(a,[1,0,0])
    for y in (-0.44,-0.47):
        for z in (1.10,1.14,1.18):
            s=r.ik((-0.13,y,z),Q,at_tcp=True,attempts=2)
            print(f"th={th} y={y} z={z}: {'ok' if s else '--'}")
EOF

# openrua op 56
timeout 300 python3 - <<'EOF' 2>&1 | grep -v "ik attempt"
import numpy as np
from ctl import Robot, quat_from_axes
r=Robot('probe')
def Q(th):
    a=np.array([0,np.cos(np.radians(th)),-np.sin(np.radians(th))]); return quat_from_axes(a,[1,0,0])
for th,p in [(65,(-0.13,-0.47,1.22)),(65,(-0.13,-0.30,1.22)),(65,(-0.13,-0.47,1.10)),(40,(-0.13,-0.47,1.03)),(40,(-0.13,-0.44,1.03)),(40,(-0.13,-0.317,1.015)),(40,(0.0215,0.099,0.958)),(65,(0.0215,0.099,1.22))]:
    s=r.ik(p,Q(th),at_tcp=True,attempts=2); print(th,p,'ok' if s else '--')
EOF

# openrua op 57
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "ik attempt"
import numpy as np
from ctl import Robot, quat_from_axes
r=Robot('probe')
def Q(th):
    a=np.array([0,np.cos(np.radians(th)),-np.sin(np.radians(th))]); return quat_from_axes(a,[1,0,0])
for th in (40,45,50,55):
    for x in (-0.13,-0.11):
        row=[]
        for y in (-0.44,-0.46,-0.48):
            for z in (1.02,1.05,1.08):
                s=r.ik((x,y,z),Q(th),at_tcp=True,attempts=2); row.append(f"y{y}z{z}:{'ok' if s else '--'}")
        print(th,x,' '.join(row))
EOF

# openrua op 58
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "ik attempt"
import numpy as np
from ctl import Robot, quat_from_axes
r=Robot('probe')
def Q(th,fx):
    a=np.array([0,np.cos(np.radians(th)),-np.sin(np.radians(th))]); return quat_from_axes(a,[fx,0,0])
for th in (40,45):
    for x in (-0.13,):
        row=[]
        for y in (-0.44,-0.46,-0.48,-0.50):
            for z in (1.02,1.05,1.10,1.18):
                s=r.ik((x,y,z),Q(th,-1),at_tcp=True,attempts=2); row.append(f"y{y}z{z}:{'ok' if s else '--'}")
        print(th,x,' '.join(row))
        if th==40:
            for p in [(0.0215,0.099,0.958),(0.0215,0.099,1.18),(-0.13,-0.317,1.015),(-0.13,-0.30,1.18),(-0.04,-0.10,1.18)]:
                s=r.ik(p,Q(th,-1),at_tcp=True,attempts=2); print('  ',p,'ok' if s else '--', None if s is None else np.round(s,2))
EOF

# openrua op 59
cat > /workspace/geom.py <<'EOF'
"""Geometry model: mug held in the tilted pinch grasp + hand housing vs microwave."""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

TH0 = 45.0
def a_of(th):
    t = np.radians(th); return np.array([0, np.cos(t), -np.sin(t)])
def n_of(th):
    t = np.radians(th); return np.array([0, np.sin(t), np.cos(t)])

def mug_points(tip, th):
    """sample points of the mug for a fingertip position `tip` at tilt th (deg)."""
    a = a_of(th); P = tip - 0.01 * a
    R = Rot.from_euler("x", -(th - TH0), degrees=True)
    C = P + R.apply([0, 0.081, -0.0125]); u = R.apply([0, 0, 1.0])
    e1 = R.apply([1.0, 0, 0]); e2 = R.apply([0, 1.0, 0])
    pts = []
    for h, rad in ((-0.0525, 0.0375), (0.0525, 0.047), (0.0, 0.042)):
        c = C + h * u
        for ang in np.linspace(0, 2 * np.pi, 24, endpoint=False):
            pts.append(c + rad * (np.cos(ang) * e1 + np.sin(ang) * e2))
    # handle outer bar (approx at the pad) and top piece
    pts.append(P); pts.append(P + R.apply([0, 0.03, 0.02]))
    return np.array(pts)

def hand_points(tip, th):
    a = a_of(th); n = n_of(th); O = tip - 0.1034 * a
    pts = []
    for s in np.linspace(0, 0.058, 4):
        for k in (-0.03, 0.03):
            for x in (-0.10, 0.0, 0.10):
                pts.append(O + s * a + k * n + [x, 0, 0])
    for s in np.linspace(0.0, 0.12, 4):       # wrist cylinder
        for k in (-0.045, 0.045):
            for x in (-0.045, 0.045):
                pts.append(O - s * a + k * n + [x, 0, 0])
    for s in np.linspace(0.058, 0.1034, 3):   # fingers (open)
        for k in (-0.01, 0.01):
            for x in (-0.045, 0.045):
                pts.append(O + s * a + k * n + [x, 0, 0])
    return np.array(pts)

def in_box(p, lo, hi):
    return np.all((p >= lo) & (p <= hi), axis=1)

def collides(pts, margin=0.0):
    m = margin
    body = in_box(pts, [-0.27 - m, -0.345 - m, 0.9], [0.09 + m, -0.12 + m, 1.107 + m])
    cav = in_box(pts, [-0.222 + m, -0.345 - 0.05, 0.944 + m], [-0.014 - m, -0.16 - m, 1.09 - m])
    door = in_box(pts, [-0.272 - m, -0.61, 0.9], [-0.245 + m, -0.35, 1.107 + m])
    table = pts[:, 2] < 0.9 + m
    return np.any((body & ~cav) | door | table)

def ok(tip, th, margin=0.005):
    return (not collides(mug_points(tip, th), margin)) and (not collides(hand_points(tip, th), margin))
EOF
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'/workspace')
from geom import ok
x=-0.13
for th in (65,60,55,50,45,40):
    line=[]
    for y in (-0.50,-0.47,-0.44,-0.42,-0.40,-0.38,-0.35,-0.32):
        zs=[z for z in np.arange(0.96,1.30,0.01) if ok(np.array([x,y,z]),th)]
        line.append(f"y{y}:" + (f"[{min(zs):.2f},{max(zs):.2f}]" if zs else "none"))
    print(th,' '.join(line))
print("insert th45 y-0.317:", [round(z,3) for z in np.arange(0.96,1.10,0.005) if ok(np.array([x,-0.317,z]),45)])
EOF

# openrua op 60
python3 - <<'EOF'
import numpy as np, sys
sys.path.insert(0,'/workspace')
from geom import *
x=-0.13
def sets(th,y):
    zs=[round(z,3) for z in np.arange(0.96,1.30,0.005) if ok(np.array([x,y,z]),th)]
    # compress into intervals
    iv=[]; 
    for z in zs:
        if iv and abs(z-iv[-1][1])<0.006: iv[-1][1]=z
        else: iv.append([z,z])
    return iv
for th in (65,55,45):
    print(th, {y:sets(th,y) for y in (-0.47,-0.44,-0.42,-0.40,-0.38,-0.35,-0.32)})
# why insertion fails
tip=np.array([x,-0.317,1.015]); th=45
for name,pts in (("mug",mug_points(tip,th)),("hand",hand_points(tip,th))):
    m=0.005
    body = in_box(pts, [-0.27 - m, -0.345 - m, 0.9], [0.09 + m, -0.12 + m, 1.107 + m])
    cav = in_box(pts, [-0.222 + m, -0.345 - 0.05, 0.944 + m], [-0.014 - m, -0.16 - m, 1.09 - m])
    bad=pts[(body&~cav)]
    print(name, "colliding pts:", np.round(bad,3))
EOF

# openrua op 61
timeout 60 python3 cloud.py birdview hand.npz >/dev/null && python3 - <<'EOF'
import numpy as np
d=np.load('/workspace/hand.npz'); X,Y,Z=d['X'],d['Y'],d['Z']; ok=np.isfinite(Z)
m=ok&(np.abs(X+0.1)<0.25)&(np.abs(Y)<0.2)&(Z>1.25)&(Z<1.60)
print("hand pts",m.sum())
for z0 in np.arange(1.30,1.56,0.02):
    mm=m&(np.abs(Z-z0)<0.01)
    if mm.sum(): print(f"z={z0:.2f}: x=[{X[mm].min():.3f},{X[mm].max():.3f}] y=[{Y[mm].min():.3f},{Y[mm].max():.3f}] n={mm.sum()}")
EOF
timeout 60 python3 -c "
from ctl import *; r=Robot('p'); print(r.hand_pose())"

# openrua op 62
cat > /workspace/mug_in.py <<'EOF'
#!/usr/bin/env python3
"""Grasp the yellow mug by its handle (fingers along x, hand tilted,
approaching from -y/above), carry it over the microwave, insert, then push it deeper."""
import sys
import numpy as np
from ctl import Robot, quat_from_axes, log
from geom import a_of, ok, hand_points, collides

def Q(th):
    return quat_from_axes(a_of(th), [1.0, 0.0, 0.0])

G = np.array([0.0215, 0.099, 0.958])        # fingertip at grasp (theta 45)
X = -0.13
r = Robot("mug")

def plan(steps, seed, with_mug=True):
    """steps: list of (tip, theta). returns joint list."""
    out = []
    for tip, th in steps:
        tip = np.array(tip, float)
        if with_mug:
            geo = ok(tip, th, margin=0.003)
        else:
            geo = not collides(hand_points(tip, th), 0.003)
        s = r.ik(tip, Q(th), seed=seed, at_tcp=True)
        jump = None if s is None else round(float(np.abs(np.array(s) - np.array(seed)).max()), 2)
        log(f"  {np.round(tip,3)} th={th} geo={'ok' if geo else 'HIT'} ik={'ok' if s else 'FAIL'} jump={jump}")
        if s is None:
            sys.exit("IK failed")
        out.append(s); seed = s
    return out

seed = r.joints()
A45 = a_of(45)
log("pre");   pre = plan([(G - 0.10 * A45 + [0, 0, 0.12], 45), (G - 0.10 * A45, 45)], seed, False)
log("app");   app = plan([(G - 0.06 * A45, 45), (G - 0.03 * A45, 45), (G, 45)], pre[-1], False)
log("lift");  lift = plan([(G + [0, 0, 0.07], 45), ((G[0], G[1], 1.22), 65)], app[-1])
log("carry"); carry = plan([((-0.04, -0.10, 1.22), 65), ((X, -0.30, 1.22), 65), ((X, -0.48, 1.22), 65)], lift[-1])
log("down");  down = plan([((X, -0.48, 1.12), 65), ((X, -0.48, 1.05), 65)], carry[-1])
log("trans"); trans = plan([((X, -0.44, 1.04), 65), ((X, -0.44, 1.04), 55), ((X, -0.44, 1.03), 45)], down[-1])
log("ins");   ins = plan([((X, -0.40, 1.02), 45), ((X, -0.37, 1.02), 45), ((X, -0.334, 1.02), 45), ((X, -0.334, 1.012), 45)], trans[-1])
log("ret");   ret = plan([((X, -0.37, 1.02), 45), ((X, -0.44, 1.02), 45)], ins[-1], False)
log("push");  push = plan([((X, -0.44, 0.97), 25), ((X, -0.40, 0.965), 25), ((X, -0.36, 0.965), 25), ((X, -0.33, 0.965), 25), ((X, -0.315, 0.965), 25)], ret[-1], False)
log("out");   out = plan([((X, -0.44, 0.97), 25), ((X, -0.50, 1.10), 45)], push[-1], False)
if "dry" in sys.argv:
    sys.exit(0)

log("== open"); r.gripper(0.04)
log("== pre-grasp"); r.move_joints(pre, [6.0, 10.0])
log("== approach"); r.move_joints(app, [2.0, 4.0, 6.0])
log("hand", np.round(r.hand_pose()[0], 3))
log("== close"); f = r.gripper(0.0)
if not (0.005 < f[0] < 0.02):
    log("!! grasp failed"); sys.exit(1)
log("== lift"); r.move_joints(lift, [4.0, 9.0])
log("fingers", r.fingers())
log("== carry"); r.move_joints(carry, [5.0, 10.0, 14.0])
log("== descend"); r.move_joints(down, [3.0, 6.0])
log("== transition"); r.move_joints(trans, [3.0, 6.0, 9.0])
log("fingers", r.fingers())
log("== insert"); r.move_joints(ins, [3.0, 5.0, 7.0, 9.0], hold=3.0)
log("hand", np.round(r.hand_pose()[0], 3), "fingers", r.fingers())
log("== release"); r.gripper(0.04)
log("== retreat"); r.move_joints(ret, [3.0, 6.0])
log("== close fingers"); r.gripper(0.0)
log("== push"); r.move_joints(push, [4.0, 6.0, 8.0, 10.0, 12.0], hold=3.0)
log("== out"); r.move_joints(out, [3.0, 7.0])
log("done")
EOF
timeout 900 python3 mug_in.py dry 2>&1 | grep -v "ik attempt"

# openrua op 63
sed -i 's/((X, -0.48, 1.22), 65)\], lift\[-1\])/((X, -0.47, 1.22), 65)], lift[-1])/; s/down = plan(\[((X, -0.48, 1.12), 65), ((X, -0.48, 1.05), 65)\]/down = plan([((X, -0.47, 1.12), 65), ((X, -0.47, 1.05), 65)]/' mug_in.py && timeout 900 python3 mug_in.py dry 2>&1 | grep -v "ik attempt" | tail -22

# openrua op 64
timeout 600 python3 - <<'EOF' 2>&1 | grep -v "ik attempt"
import numpy as np
from ctl import Robot, quat_from_axes
from geom import a_of, ok, hand_points, collides
r=Robot('probe')
def Q(th): return quat_from_axes(a_of(th),[1,0,0])
X=-0.13
for th in (25,30,35):
    for y in (-0.44,-0.40,-0.36,-0.322):
        s=r.ik((X,y,0.965),Q(th),at_tcp=True,attempts=2)
        print(th,y,'ik',('ok' if s else '--'),'geo',not collides(hand_points(np.array([X,y,0.965]),th),0.003))
# branch check for transition
seed=r.ik((X,-0.44,1.04),Q(55),at_tcp=True); print('55',np.round(seed,2))
for th in (50,45):
    for z in (1.03,1.02):
        s=r.ik((X,-0.44,z),Q(th),seed=seed,at_tcp=True,attempts=4)
        print(th,z,None if s is None else np.round(s,2), 'geo', ok(np.array([X,-0.44,z]),th,0.003))
for y in (-0.334,-0.33,-0.326):
    print('ins',y, ok(np.array([X,y,1.02]),45,0.003), ok(np.array([X,y,1.02]),45,0.0))
EOF

# openrua op 65
python3 - <<'EOF'
s=open('/workspace/mug_in.py').read()
s=s.replace('''log("carry"); carry = plan([((-0.04, -0.10, 1.22), 65), ((X, -0.30, 1.22), 65), ((X, -0.47, 1.22), 65)], lift[-1])''',
'''log("carry"); carry = plan([((-0.04, -0.10, 1.22), 65), ((X, -0.30, 1.22), 65), ((X, -0.40, 1.22), 65), ((X, -0.47, 1.22), 65)], lift[-1])''')
s=s.replace('''log("trans"); trans = plan([((X, -0.44, 1.04), 65), ((X, -0.44, 1.04), 55), ((X, -0.44, 1.03), 45)], down[-1])
log("ins");   ins = plan([((X, -0.40, 1.02), 45), ((X, -0.37, 1.02), 45), ((X, -0.334, 1.02), 45), ((X, -0.334, 1.012), 45)], trans[-1])
log("ret");   ret = plan([((X, -0.37, 1.02), 45), ((X, -0.44, 1.02), 45)], ins[-1], False)
log("push");  push = plan([((X, -0.44, 0.97), 25), ((X, -0.40, 0.965), 25), ((X, -0.36, 0.965), 25), ((X, -0.33, 0.965), 25), ((X, -0.315, 0.965), 25)], ret[-1], False)
log("out");   out = plan([((X, -0.44, 0.97), 25), ((X, -0.50, 1.10), 45)], push[-1], False)''',
'''log("trans"); trans = plan([((X, -0.44, 1.04), 65), ((X, -0.44, 1.04), 55), ((X, -0.44, 1.03), 50), ((X, -0.40, 1.02), 50), ((X, -0.40, 1.02), 45)], down[-1])
log("ins");   ins = plan([((X, -0.37, 1.02), 45), ((X, -0.338, 1.02), 45), ((X, -0.338, 1.012), 45)], trans[-1])
log("ret");   ret = plan([((X, -0.37, 1.02), 45), ((X, -0.40, 1.02), 45)], ins[-1], False)
log("push");  push = plan([((X, -0.36, 0.975), 25), ((X, -0.36, 0.965), 25), ((X, -0.34, 0.965), 25), ((X, -0.322, 0.965), 25)], ret[-1], False)
log("out");   out = plan([((X, -0.36, 0.975), 25), ((X, -0.40, 1.02), 45), ((X, -0.44, 1.10), 55)], push[-1], False)''')
s=s.replace('''log("== carry"); r.move_joints(carry, [5.0, 10.0, 14.0])''','''log("== carry"); r.move_joints(carry, [5.0, 10.0, 13.0, 16.0])''')
s=s.replace('''log("== transition"); r.move_joints(trans, [3.0, 6.0, 9.0])''','''log("== transition"); r.move_joints(trans, [3.0, 6.0, 9.0, 12.0, 15.0])''')
s=s.replace('''log("== insert"); r.move_joints(ins, [3.0, 5.0, 7.0, 9.0], hold=3.0)''','''log("== insert"); r.move_joints(ins, [3.0, 6.0, 8.0], hold=3.0)''')
s=s.replace('''log("== push"); r.move_joints(push, [4.0, 6.0, 8.0, 10.0, 12.0], hold=3.0)''','''log("== push"); r.move_joints(push, [4.0, 6.0, 8.0, 10.0], hold=3.0)''')
s=s.replace('''log("== out"); r.move_joints(out, [3.0, 7.0])''','''log("== out"); r.move_joints(out, [3.0, 6.0, 9.0])''')
s=s.replace('''log(f"  {np.round(tip,3)} th={th} geo={'ok' if geo else 'HIT'} ik={'ok' if s else 'FAIL'} jump={jump}")''',
'''log(f"  {np.round(tip,3)} th={th} geo={'ok' if geo else 'HIT'} ik={'ok' if s else 'FAIL'} jump={jump} q={None if s is None else np.round(s,2)}")''')
open('/workspace/mug_in.py','w').write(s)
EOF
timeout 900 python3 mug_in.py dry 2>&1 | grep -v "ik attempt"

# openrua op 66
timeout 1500 python3 mug_in.py 2>&1 | grep -v "ik attempt" | grep -v "geo="

# openrua op 67
timeout 60 python3 tools/perception/cam_snap.py agentview a6.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e6.png; timeout 60 python3 tools/perception/cam_snap.py frontview f6.png; timeout 60 python3 -c "
from ctl import *; r=Robot('p'); print(r.hand_pose(), r.fingers())"

# openrua op 68
timeout 60 python3 cloud.py robot0_eye_in_hand eih6.npz && python3 - <<'EOF'
import numpy as np
d=np.load('/workspace/eih6.npz'); X,Y,Z,C=d['X'],d['Y'],d['Z'],d['color']
ok=np.isfinite(Z)
b,g,r=C[...,0].astype(int),C[...,1].astype(int),C[...,2].astype(int)
yellow=ok&(r>150)&(g>120)&(b<120)&(r-b>80)
print("yellow pts",yellow.sum())
P=np.stack([X[yellow],Y[yellow],Z[yellow]],1)
print("bbox x",P[:,0].min(),P[:,0].max()," y",P[:,1].min(),P[:,1].max()," z",P[:,2].min(),P[:,2].max())
c=P.mean(0); print("centroid",np.round(c,3))
u,s,vt=np.linalg.svd(P-c,full_matrices=False); print("principal axes",np.round(vt,2),"sv",np.round(s,2))
# white interior
white=ok&(r>200)&(g>200)&(b>200)
W=np.stack([X[white],Y[white],Z[white]],1)
print("white pts",white.sum(), "centroid", np.round(W.mean(0),3) if len(W) else None)
EOF

# openrua op 69
cat > /workspace/carry_back.py <<'EOF'
import sys, numpy as np
from ctl import Robot, quat_from_axes, log
from geom import a_of
def Q(th): return quat_from_axes(a_of(th), [1.0, 0, 0])
r = Robot("cb")
seed = r.joints()
steps = [((-0.13, -0.44, 1.28), 65), ((-0.13, -0.30, 1.28), 65), ((-0.05, -0.05, 1.28), 65), ((-0.05, 0.13, 1.28), 55), ((-0.05, 0.13, 1.15), 45)]
wps = []
for p, th in steps:
    s = r.ik(p, Q(th), seed=seed, at_tcp=True)
    log(p, th, None if s is None else np.round(s, 2), None if s is None else round(float(np.abs(np.array(s)-np.array(seed)).max()),2))
    if s is None: sys.exit("IK fail")
    wps.append(s); seed = s
if "dry" in sys.argv: sys.exit(0)
r.move_joints(wps, [5.0, 9.0, 14.0, 18.0, 22.0])
log("hand", np.round(r.hand_pose()[0], 3), "fingers", r.fingers())
EOF
timeout 600 python3 carry_back.py dry 2>&1 | grep -v "ik attempt"

# openrua op 70
timeout 900 python3 carry_back.py 2>&1 | grep -v "ik attempt"

# openrua op 71
python3 tools/perception/cam_snap.py agentview a7.png && python3 tools/perception/cam_snap.py robot0_eye_in_hand e7.png && python3 cloud.py robot0_eye_in_hand eih7.npz && python3 - <<'EOF'
import numpy as np
d=np.load('eih7.npz'); X,Y,Z,c=d['X'],d['Y'],d['Z'],d['color'].astype(int)
b,g,r=c[...,0],c[...,1],c[...,2]
yel=(r>150)&(g>120)&(b<110)&np.isfinite(Z)
wht=(r>170)&(g>170)&(b>170)&np.isfinite(Z)&(Z>0.95)&(Z<1.2)
for nm,m in [('yellow',yel),('white',wht)]:
    if m.sum()<10: print(nm,'none'); continue
    p=np.stack([X[m],Y[m],Z[m]],1)
    print(nm,m.sum(),'bbox',np.round(p.min(0),3),np.round(p.max(0),3),'cen',np.round(p.mean(0),3))
EOF

# openrua op 72
cat > /workspace/lower.py <<'EOF'
import sys, numpy as np
from ctl import Robot, quat_from_axes, log
from geom import a_of
def Q(th): return quat_from_axes(a_of(th), [1.0, 0, 0])
r = Robot("lw")
seed = r.joints()
zs = [float(v) for v in sys.argv[1:]]
wps=[]
for z in zs:
    s = r.ik((-0.05, 0.13, z), Q(45), seed=seed, at_tcp=True); assert s is not None
    wps.append(s); seed=s
r.move_joints(wps, [3.0*(i+1) for i in range(len(wps))])
log("hand", np.round(r.hand_pose()[0], 3), "fingers", r.fingers())
EOF
timeout 600 python3 lower.py 1.08 1.04 2>&1 | grep -v "ik attempt" && python3 tools/perception/cam_snap.py agentview a8.png && python3 tools/perception/cam_snap.py frontview f8.png

# openrua op 73
cat > /workspace/release_up.py <<'EOF'
import numpy as np
from ctl import Robot, quat_from_axes, log
from geom import a_of
def Q(th): return quat_from_axes(a_of(th), [1.0, 0, 0])
r = Robot("ru")
r.gripper(0.04)
s = r.ik((-0.05, 0.13, 1.25), Q(45), seed=r.joints(), at_tcp=True)
r.move_joints([s], [4.0])
log("hand", np.round(r.hand_pose()[0], 3), "fingers", r.fingers())
EOF
timeout 600 python3 release_up.py 2>&1 | grep -v "ik attempt" && python3 scene.py 2>&1 | tail -20 && python3 tools/perception/cam_snap.py agentview a9.png

# openrua op 74
python3 - <<'EOF'
import numpy as np
d=np.load('bird.npz'); X,Y,Z,c=d['X'],d['Y'],d['Z'],d['color'].astype(int)
b,g,r=c[...,0],c[...,1],c[...,2]
m=np.isfinite(Z)&(X>-0.3)&(X<0.2)&(Y>0.05)&(Y<0.3)&(Z>0.905)
yel=m&(r>150)&(g>120)&(b<110)
wht=m&(r>170)&(g>170)&(b>170)
for nm,mm in [('yellow',yel),('white',wht),('all',m)]:
    p=np.stack([X[mm],Y[mm],Z[mm]],1)
    print(nm,mm.sum(),'bbox',np.round(p.min(0),3),np.round(p.max(0),3),'cen',np.round(p.mean(0),3))
p=np.stack([X[m],Y[m],Z[m]],1)
# principal axis in xy
q=p[:,:2]-p[:,:2].mean(0); w,v=np.linalg.eigh(q.T@q); print('axis xy',np.round(v[:,-1],3), 'eig',np.round(w,4))
# height profile along the axis
u=q@v[:,-1]
for lo in np.arange(u.min(),u.max(),0.01):
    s=(u>=lo)&(u<lo+0.01)
    if s.sum()>3: print(f"u {lo:+.3f} n={s.sum():4d} zmax={p[s,2].max():.3f} ymean={p[s,1].mean():.3f} xmean={p[s,0].mean():.3f}")
EOF

# openrua op 75
python3 - <<'EOF'
import numpy as np
d=np.load('bird.npz'); X,Y,Z,c=d['X'],d['Y'],d['Z'],d['color'].astype(int)
b,g,r=c[...,0],c[...,1],c[...,2]
m=np.isfinite(Z)&(X>-0.3)&(X<0.2)&(Y>0.05)&(Y<0.3)&(Z>0.905)&(Z<1.06)
yel=m&(r>150)&(g>120)&(b<110)
wht=m&(r>170)&(g>170)&(b>170)
for nm,mm in [('yellow',yel),('white',wht),('all',m)]:
    p=np.stack([X[mm],Y[mm],Z[mm]],1)
    print(nm,mm.sum(),'bbox',np.round(p.min(0),3),np.round(p.max(0),3),'cen',np.round(p.mean(0),3))
p=np.stack([X[m],Y[m],Z[m]],1)
q=p[:,:2]-p[:,:2].mean(0); w,v=np.linalg.eigh(q.T@q); ax=v[:,-1]; print('axis xy',np.round(ax,3), 'eig',np.round(w,4),'mean',np.round(p[:,:2].mean(0),3))
u=q@ax; t=q@v[:,0]
for lo in np.arange(u.min(),u.max(),0.01):
    s=(u>=lo)&(u<lo+0.01)
    if s.sum()>3: print(f"u {lo:+.3f} n={s.sum():4d} zmax={p[s,2].max():.3f} t=[{t[s].min():+.3f},{t[s].max():+.3f}] xy=({p[s,0].mean():.3f},{p[s,1].mean():.3f}) yellow={ (yel[mm] if False else 0)}")
# where is white (interior) along u
uw=(np.stack([X[wht],Y[wht]],1)-p[:,:2].mean(0))@ax; print('white u range',np.round(uw.min(),3),np.round(uw.max(),3),'mean',np.round(uw.mean(),3))
uy=(np.stack([X[yel],Y[yel]],1)-p[:,:2].mean(0))@ax; print('yellow u range',np.round(uy.min(),3),np.round(uy.max(),3),'mean',np.round(uy.mean(),3))
EOF

# openrua op 76
python3 tools/perception/cam_snap.py frontview f9.png && python3 tools/perception/cam_snap.py sideview s9.png && python3 tools/perception/cam_snap.py birdview b9.png

# openrua op 77
cat > /workspace/look.py <<'EOF'
import sys, numpy as np
from ctl import Robot, quat_down, log
r = Robot("lk")
x,y,z = map(float, sys.argv[1:4])
s = r.ik((x,y,z), quat_down(90), seed=r.joints(), at_tcp=True); assert s is not None
r.move_joints([s],[5.0]); log("hand", np.round(r.hand_pose()[0],3))
EOF
timeout 300 python3 look.py -0.05 0.19 1.50 2>&1 | grep -v "ik attempt" && python3 cloud.py robot0_eye_in_hand eih9.npz && python3 tools/perception/cam_snap.py robot0_eye_in_hand e9.png && python3 - <<'EOF'
import numpy as np
d=np.load('eih9.npz'); X,Y,Z,c=d['X'],d['Y'],d['Z'],d['color'].astype(int)
b,g,r=c[...,0],c[...,1],c[...,2]
m=np.isfinite(Z)&(X>-0.25)&(X<0.1)&(Y>0.08)&(Y<0.28)&(Z>0.905)&(Z<1.1)
yel=m&(r>150)&(g>120)&(b<110)
wht=m&(r>170)&(g>170)&(b>170)
p=np.stack([X[m],Y[m],Z[m]],1)
print('all',m.sum(),'bbox',np.round(p.min(0),3),np.round(p.max(0),3))
for nm,mm in [('yellow',yel),('white',wht)]:
    q=np.stack([X[mm],Y[mm],Z[mm]],1); print(nm,mm.sum(),'bbox',np.round(q.min(0),3),np.round(q.max(0),3),'cen',np.round(q.mean(0),3))
# slices along x
for lo in np.arange(-0.13,0.02,0.01):
    s=m&(X>=lo)&(X<lo+0.01)
    if s.sum()>3:
        hi=s&(Z>1.0)
        print(f"x {lo:+.3f} n={s.sum():4d} y=[{Y[s].min():.3f},{Y[s].max():.3f}] zmax={Z[s].max():.3f} yc_at_zmax={Y[s][Z[s].argmax()]:.3f}  handle(z>1.0) n={hi.sum()} y=[{Y[hi].min() if hi.sum() else 0:.3f},{Y[hi].max() if hi.sum() else 0:.3f}]")
EOF

# openrua op 78
timeout 300 python3 look.py -0.05 0.19 1.40 2>&1 | grep -v "ik attempt" && python3 cloud.py robot0_eye_in_hand eih9.npz && python3 tools/perception/cam_snap.py robot0_eye_in_hand e9.png && python3 - <<'EOF'
import numpy as np
d=np.load('eih9.npz'); X,Y,Z,c=d['X'],d['Y'],d['Z'],d['color'].astype(int)
b,g,r=c[...,0],c[...,1],c[...,2]
m=np.isfinite(Z)&(X>-0.25)&(X<0.1)&(Y>0.08)&(Y<0.28)&(Z>0.905)&(Z<1.1)
yel=m&(r>150)&(g>120)&(b<110)
wht=m&(r>170)&(g>170)&(b>170)
p=np.stack([X[m],Y[m],Z[m]],1)
print('all',m.sum(),'bbox',np.round(p.min(0),3),np.round(p.max(0),3))
for nm,mm in [('yellow',yel),('white',wht)]:
    if mm.sum()==0: print(nm,'none'); continue
    q=np.stack([X[mm],Y[mm],Z[mm]],1); print(nm,mm.sum(),'bbox',np.round(q.min(0),3),np.round(q.max(0),3),'cen',np.round(q.mean(0),3))
for lo in np.arange(-0.13,0.02,0.01):
    s=m&(X>=lo)&(X<lo+0.01)
    if s.sum()>3:
        hi=s&(Z>1.0)
        print(f"x {lo:+.3f} n={s.sum():4d} y=[{Y[s].min():.3f},{Y[s].max():.3f}] zmax={Z[s].max():.3f} y_at_zmax={Y[s][Z[s].argmax()]:.3f}  z>1.0: n={hi.sum()} y=[{Y[hi].min() if hi.sum() else 0:.3f},{Y[hi].max() if hi.sum() else 0:.3f}]")
EOF

# openrua op 79
cat > /workspace/drag.py <<'EOF'
import sys, numpy as np
from ctl import Robot, quat_down, log
r = Robot("dg")
Q = quat_down(90)  # fingers along world x
XH, YH, ZG = -0.055, 0.195, 1.005
def ik(p, seed): 
    s = r.ik(p, Q, seed=seed, at_tcp=True)
    if s is None: sys.exit(f"IK fail {p}")
    return s
seed = r.joints()
r.gripper(0.04)
s1 = ik((XH, YH, 1.15), seed); s2 = ik((XH, YH, 1.04), s1); s3 = ik((XH, YH, ZG), s2)
r.move_joints([s1, s2, s3], [5.0, 8.0, 10.0])
f = r.gripper(0.0)
if not (0.005 < f[0] < 0.015): sys.exit(f"bad grasp {f}")
seed = s3
wps = [ik((XH, y, ZG), seed) for y in (0.10, 0.0)]
r.move_joints(wps, [5.0, 10.0])
log("hand", np.round(r.hand_pose()[0],3), "fingers", r.fingers())
r.gripper(0.04)
s = ik((XH, 0.0, 1.15), r.joints()); r.move_joints([s],[4.0])
EOF
timeout 900 python3 drag.py 2>&1 | grep -v "ik attempt"

# openrua op 80
timeout 300 python3 look.py -0.055 0.05 1.40 2>&1 | grep -v "ik attempt" && python3 cloud.py robot0_eye_in_hand eih10.npz && python3 tools/perception/cam_snap.py robot0_eye_in_hand e10.png && python3 - <<'EOF'
import numpy as np
d=np.load('eih10.npz'); X,Y,Z,c=d['X'],d['Y'],d['Z'],d['color'].astype(int)
b,g,r=c[...,0],c[...,1],c[...,2]
m=np.isfinite(Z)&(X>-0.25)&(X<0.1)&(Y>-0.11)&(Y<0.25)&(Z>0.905)&(Z<1.1)
yel=m&(r>150)&(g>120)&(b<110)
wht=m&(r>170)&(g>170)&(b>170)
p=np.stack([X[m],Y[m],Z[m]],1)
print('all',m.sum(),'bbox',np.round(p.min(0),3),np.round(p.max(0),3))
for nm,mm in [('yellow',yel),('white',wht)]:
    if mm.sum()==0: print(nm,'none'); continue
    q=np.stack([X[mm],Y[mm],Z[mm]],1); print(nm,mm.sum(),'bbox',np.round(q.min(0),3),np.round(q.max(0),3),'cen',np.round(q.mean(0),3))
for lo in np.arange(-0.13,0.02,0.01):
    s=m&(X>=lo)&(X<lo+0.01)
    if s.sum()>3:
        hi=s&(Z>1.0)
        print(f"x {lo:+.3f} n={s.sum():4d} y=[{Y[s].min():.3f},{Y[s].max():.3f}] zmax={Z[s].max():.3f}  z>1.0: n={hi.sum()} y=[{Y[hi].min() if hi.sum() else 0:.3f},{Y[hi].max() if hi.sum() else 0:.3f}]")
for lo in np.arange(-0.08,0.12,0.01):
    s=m&(Y>=lo)&(Y<lo+0.01)
    if s.sum()>3: print(f"y {lo:+.3f} n={s.sum():4d} x=[{X[s].min():.3f},{X[s].max():.3f}] zmax={Z[s].max():.3f} zmin={Z[s].min():.3f}")
EOF

# openrua op 81
sed -n 1,80p /workspace/geom.py

# openrua op 82
sed -i 's/door = in_box(pts, \[-0.272 - m, -0.61, 0.9\], \[-0.245 + m, -0.35, 1.107 + m\])/door = in_box(pts, [-0.280 - m, -0.61, 0.9], [-0.257 + m, -0.35, 1.107 + m])/' geom.py && grep -n "door =" geom.py
cat > /workspace/rim_grasp.py <<'EOF'
"""Rim-side pinch of the lying mug, swing it upright (hanging), carry into the microwave, push in."""
import sys, numpy as np
from ctl import Robot, quat_from_axes, log
from geom import a_of, n_of, hand_points, collides

def Q(th): return quat_from_axes(a_of(th), [1.0, 0, 0])
def Qphi(phi):  # approach rotated about x: phi=0 -> a=-y (from +y side), 90 -> down, 135 -> a_of(45)
    t = np.radians(phi); return quat_from_axes([0, -np.cos(t), -np.sin(t)], [1.0, 0, 0])

XC_LYING, YR, ZA = -0.059, 0.053, 0.953       # lying mug: axis x, rim edge y, axis height at rim
TIPX = XC_LYING - 0.0475                      # pinch at the -x rim wall
G = np.array([TIPX, YR - 0.025, ZA])          # grasp tip
XC = -0.09                                    # desired mug centre x in the microwave
TX = XC - 0.0475                              # tip x while hanging

def hang_mug_points(tip):
    c = np.array([tip[0] + 0.0475, tip[1], tip[2] - 0.0275])
    pts = []
    for h, rad in ((-0.0525, 0.0375), (0.0525, 0.05), (0.0, 0.043)):
        for ang in np.linspace(0, 2*np.pi, 24, endpoint=False):
            pts.append(c + [rad*np.cos(ang), rad*np.sin(ang), h])
    for dy in (-0.06, -0.075, -0.09):          # handle toward -y
        for dz in (-0.03, 0.0, 0.03):
            pts.append(c + [0, dy, dz])
    return np.array(pts)

r = Robot("rg")
DRY = "dry" in sys.argv

def plan(steps, seed, quatf=Q, with_mug=False, margin=0.004):
    out = []
    for p, th in steps:
        p = np.array(p, float)
        if quatf is Q:
            if collides(hand_points(p, th), margin): log("  HAND COLLISION", p, th)
        if with_mug and collides(hang_mug_points(p), margin): log("  MUG COLLISION", p, th)
        s = r.ik(p, quatf(th), seed=seed, at_tcp=True)
        if s is None: sys.exit(f"IK fail {p} {th}")
        jump = float(np.abs(np.array(s) - np.array(seed)).max())
        log(f"  {np.round(p,3)} th={th} jump={jump:.2f}")
        out.append(s); seed = s
    return out, seed

seed = r.joints()
# 1 approach from +y at axis height, horizontal hand (phi=0)
pre, seed = plan([((TIPX, 0.12, 1.15), 0), ((TIPX, 0.12, ZA), 0), ((TIPX, 0.07, ZA), 0), (G, 0)], seed, quatf=Qphi)
# 2 lift, then swing the hand about x through phi 45,90,135 with tip fixed
LZ = 1.10
lift, seed = plan([((TIPX, G[1], LZ), 0)], seed, quatf=Qphi)
swing, seed = plan([((TIPX, G[1], LZ), 45), ((TIPX, G[1], LZ), 90), ((TIPX, G[1], LZ), 135)], seed, quatf=Qphi)
# 3 carry (hanging mug, upright)
carry, seed = plan([((TX, G[1], 1.25), 45), ((TX, -0.15, 1.25), 55), ((TX, -0.35, 1.25), 65), ((TX, -0.45, 1.20), 65),
                    ((TX, -0.45, 1.06), 55)], seed, with_mug=True)
ins, seed = plan([((TX, -0.42, 1.045), 45), ((TX, -0.38, 1.04), 35), ((TX, -0.326, 1.035), 30), ((TX, -0.326, 1.022), 30)], seed, with_mug=True)
ret, seed = plan([((TX, -0.326, 1.062), 30), ((TX, -0.38, 1.065), 40), ((TX, -0.43, 1.07), 55)], seed)
push, seed = plan([((XC, -0.43, 1.0), 75), ((XC, -0.40, 1.0), 55), ((XC, -0.37, 1.0), 40), ((XC, -0.345, 1.0), 30), ((XC, -0.322, 1.0), 25)], seed)
out, seed = plan([((XC, -0.36, 1.0), 30), ((XC, -0.40, 1.03), 45), ((XC, -0.44, 1.12), 55)], seed)
if DRY: sys.exit(0)

def go(wps, times, hold=4.0):
    code = r.move_joints(wps, times, hold=hold)
    log("   hand", np.round(r.hand_pose()[0], 3), "fingers", np.round(r.fingers(), 4))
    return code

r.gripper(0.04)
go(pre, [5.0, 9.0, 12.0, 15.0])
f = r.gripper(0.0)
if not (0.0008 < f[0] < 0.012): sys.exit(f"bad rim grasp {f}")
go(lift, [4.0])
go(swing, [4.0, 8.0, 12.0])
log("swing done; fingers", r.fingers())
go(carry, [4.0, 9.0, 14.0, 17.0, 21.0])
go(ins, [3.0, 6.0, 9.0, 11.0], hold=3.0)
log("fingers before release", r.fingers())
r.gripper(0.04)
go(ret, [3.0, 6.0, 9.0])
r.gripper(0.0)
go(push, [4.0, 7.0, 10.0, 13.0, 16.0])
go(out, [3.0, 6.0, 9.0])
EOF
timeout 900 python3 rim_grasp.py dry 2>&1 | grep -v "ik attempt"

# openrua op 83
python3 - <<'EOF'
s=open('rim_grasp.py').read()
s=s.replace("out, seed = plan([((XC, -0.36, 1.0), 30), ((XC, -0.40, 1.03), 45), ((XC, -0.44, 1.12), 55)], seed)",
            "out, seed = plan([((XC, -0.37, 1.0), 40), ((XC, -0.40, 1.03), 50), ((XC, -0.44, 1.12), 60)], seed)")
s=s.replace('log("swing done; fingers", r.fingers())', 'log("swing done; fingers", r.fingers())\nif "p1" in sys.argv: sys.exit(0)')
s=s.replace("r.gripper(0.04)\ngo(pre,", 'if "p2" not in sys.argv:\n  r.gripper(0.04)\n  go(pre,')
s=s.replace("f = r.gripper(0.0)\nif not (0.0008 < f[0] < 0.012): sys.exit(f\"bad rim grasp {f}\")\ngo(lift, [4.0])\ngo(swing, [4.0, 8.0, 12.0])\nlog(\"swing done; fingers\", r.fingers())",
 "  f = r.gripper(0.0)\n  if not (0.0008 < f[0] < 0.012): sys.exit(f\"bad rim grasp {f}\")\n  go(lift, [4.0])\n  go(swing, [4.0, 8.0, 12.0])\n  log(\"swing done; fingers\", r.fingers())")
open('rim_grasp.py','w').write(s)
EOF
sed -n '/^def go/,$p' rim_grasp.py

# openrua op 84
timeout 900 python3 rim_grasp.py p1 2>&1 | grep -v "ik attempt" | grep -v "^  \[" ; python3 tools/perception/cam_snap.py agentview a11.png; python3 tools/perception/cam_snap.py frontview f11.png

# openrua op 85
python3 cloud.py robot0_eye_in_hand eih11.npz && python3 - <<'EOF'
import numpy as np
d=np.load('eih11.npz'); X,Y,Z,c=d['X'],d['Y'],d['Z'],d['color'].astype(int)
b,g,r=c[...,0],c[...,1],c[...,2]
m=np.isfinite(Z)&(Z>0.92)&(Z<1.2)&(X>-0.3)&(X<0.1)&(Y>-0.2)&(Y<0.25)
yel=m&(r>150)&(g>120)&(b<110); wht=m&(r>170)&(g>170)&(b>170)
for nm,mm in [('yellow',yel),('white',wht)]:
    q=np.stack([X[mm],Y[mm],Z[mm]],1); print(nm,mm.sum(),'bbox',np.round(q.min(0),3),np.round(q.max(0),3),'cen',np.round(q.mean(0),3))
for lo in np.arange(0.95,1.13,0.02):
    s=yel&(Z>=lo)&(Z<lo+0.02)
    if s.sum()>3: print(f"z {lo:.2f} n={s.sum():4d} x=[{X[s].min():.3f},{X[s].max():.3f}] y=[{Y[s].min():.3f},{Y[s].max():.3f}]")
EOF

# openrua op 86
timeout 1200 python3 rim_grasp.py p2 2>&1 | grep -v "ik attempt" | grep -v "^  \[" ; python3 tools/perception/cam_snap.py agentview a12.png; python3 tools/perception/cam_snap.py frontview f12.png

# openrua op 87
cat > /workspace/peek.py <<'EOF'
import sys, numpy as np
from ctl import Robot, quat_from_axes, log
from geom import a_of
r = Robot("pk")
x,y,z,th = map(float, sys.argv[1:5])
s = r.ik((x,y,z), quat_from_axes(a_of(th),[1,0,0]), seed=r.joints(), at_tcp=True); assert s is not None
r.move_joints([s],[5.0]); log("hand", np.round(r.hand_pose()[0],3))
EOF
timeout 300 python3 peek.py -0.12 -0.50 1.10 20 2>&1 | grep -v "ik attempt" && python3 cloud.py robot0_eye_in_hand eih12.npz && python3 tools/perception/cam_snap.py robot0_eye_in_hand e12.png && python3 - <<'EOF'
import numpy as np
d=np.load('eih12.npz'); X,Y,Z,c=d['X'],d['Y'],d['Z'],d['color'].astype(int)
b,g,r=c[...,0],c[...,1],c[...,2]
m=np.isfinite(Z)&(Z>0.93)&(Z<1.1)&(X>-0.23)&(X<0.0)&(Y>-0.36)&(Y<-0.15)
yel=m&(r>150)&(g>120)&(b<110); wht=m&(r>170)&(g>170)&(b>170)
for nm,mm in [('yellow',yel),('white',wht)]:
    if mm.sum()==0: print(nm,'none'); continue
    q=np.stack([X[mm],Y[mm],Z[mm]],1); print(nm,mm.sum(),'bbox',np.round(q.min(0),3),np.round(q.max(0),3),'cen',np.round(q.mean(0),3))
for lo in np.arange(0.94,1.10,0.02):
    s=yel&(Z>=lo)&(Z<lo+0.02)
    if s.sum()>3: print(f"z {lo:.2f} n={s.sum():4d} x=[{X[s].min():.3f},{X[s].max():.3f}] y=[{Y[s].min():.3f},{Y[s].max():.3f}]")
# anything protruding in front of the plane?
fr=np.isfinite(Z)&(Y<-0.345)&(Y>-0.45)&(Z>0.905)&(Z<1.10)&(X>-0.24)&(X<0.0)
print('points in front of plane (x -0.24..0, z .905-1.10):', fr.sum(), 'yellow:', (fr&(r>150)&(g>120)&(b<110)).sum())
EOF

# openrua op 88
timeout 300 python3 peek.py -0.10 -0.43 1.03 45 2>&1 | grep -v "ik attempt" && python3 cloud.py robot0_eye_in_hand eih13.npz && python3 tools/perception/cam_snap.py robot0_eye_in_hand e13.png && python3 - <<'EOF'
import numpy as np
d=np.load('eih13.npz'); X,Y,Z,c=d['X'],d['Y'],d['Z'],d['color'].astype(int)
b,g,r=c[...,0],c[...,1],c[...,2]
m=np.isfinite(Z)&(Z>0.93)&(Z<1.1)&(X>-0.23)&(X<0.0)&(Y>-0.36)&(Y<-0.15)
yel=m&(r>150)&(g>120)&(b<110); wht=m&(r>170)&(g>170)&(b>170)
for nm,mm in [('yellow',yel),('white',wht)]:
    if mm.sum()==0: print(nm,'none'); continue
    q=np.stack([X[mm],Y[mm],Z[mm]],1); print(nm,mm.sum(),'bbox',np.round(q.min(0),3),np.round(q.max(0),3),'cen',np.round(q.mean(0),3))
for lo in np.arange(0.94,1.10,0.01):
    s=yel&(Z>=lo)&(Z<lo+0.01)
    if s.sum()>3: print(f"z {lo:.2f} n={s.sum():4d} x=[{X[s].min():.3f},{X[s].max():.3f}] y=[{Y[s].min():.3f},{Y[s].max():.3f}]")
fr=np.isfinite(Z)&(Y<-0.345)&(Y>-0.42)&(Z>0.905)&(Z<1.10)&(X>-0.24)&(X<0.0)
print('points in front of plane:', fr.sum(), 'yellow:', (fr&(r>150)&(g>120)&(b<110)).sum())
EOF

# openrua op 89
timeout 300 python3 peek.py -0.1375 -0.42 1.045 45 2>&1 | grep -v "ik attempt" && python3 cloud.py robot0_eye_in_hand eih13.npz && python3 tools/perception/cam_snap.py robot0_eye_in_hand e13.png && python3 - <<'EOF'
import numpy as np
d=np.load('eih13.npz'); X,Y,Z,c=d['X'],d['Y'],d['Z'],d['color'].astype(int)
b,g,r=c[...,0],c[...,1],c[...,2]
m=np.isfinite(Z)&(Z>0.93)&(Z<1.1)&(X>-0.23)&(X<0.0)&(Y>-0.36)&(Y<-0.15)
yel=m&(r>150)&(g>120)&(b<110); wht=m&(r>170)&(g>170)&(b>170)
for nm,mm in [('yellow',yel),('white',wht)]:
    if mm.sum()==0: print(nm,'none'); continue
    q=np.stack([X[mm],Y[mm],Z[mm]],1); print(nm,mm.sum(),'bbox',np.round(q.min(0),3),np.round(q.max(0),3),'cen',np.round(q.mean(0),3))
for lo in np.arange(0.94,1.10,0.01):
    s=yel&(Z>=lo)&(Z<lo+0.01)
    if s.sum()>3: print(f"z {lo:.2f} n={s.sum():4d} x=[{X[s].min():.3f},{X[s].max():.3f}] y=[{Y[s].min():.3f},{Y[s].max():.3f}]")
fr=np.isfinite(Z)&(Y<-0.345)&(Y>-0.42)&(Z>0.905)&(Z<1.10)&(X>-0.24)&(X<0.0)
print('points in front of plane:', fr.sum(), 'yellow:', (fr&(r>150)&(g>120)&(b<110)).sum())
EOF

# openrua op 90
cat /workspace/door_open.py

# openrua op 91
cat > /workspace/door_close.py <<'EOF'
#!/usr/bin/env python3
"""Close the microwave door: vertical pinch of the handle bar at phi=90, swing to phi=0."""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from ctl import Robot, quat_from_axes, log

H = np.array([-0.262, -0.35])
B0 = np.array([-0.015, -0.399, 1.062])
A0 = np.array([0.0, 0.0, -1.0])
FX0 = np.array([1.0, 0.0, 0.0])
PAD = 0.012

def pose(phi_deg, back=0.0, dz=0.0):
    Rz = Rot.from_euler("z", -phi_deg, degrees=True)
    a, fx = Rz.apply(A0), Rz.apply(FX0)
    rel = Rz.apply([B0[0] - H[0], B0[1] - H[1], 0.0])
    bar = np.array([H[0] + rel[0], H[1] + rel[1], B0[2] + dz])
    return bar + (PAD - back) * a, quat_from_axes(a, fx), a

r = Robot("doorc")
dry = "dry" in sys.argv
seed = r.joints()
plan = []
for phi, back, dz in [(90, 0.08, 0.12), (90, 0.08, 0.0), (90, 0.05, 0.0), (90, 0.025, 0.0), (90, 0.0, 0.0)]:
    tip, q, a = pose(phi, back, dz)
    sol = r.ik(tip, q, seed=seed, at_tcp=True)
    log(f"pre back={back} dz={dz} tip={np.round(tip,3)} -> {None if sol is None else np.round(sol,2)}")
    if sol is None: sys.exit("IK failed in approach")
    plan.append(sol); seed = sol
swing = []
phis = np.arange(80.0, -0.1, -10.0)
for phi in phis:
    tip, q, a = pose(phi)
    sol = r.ik(tip, q, seed=seed, at_tcp=True)
    if sol is None: sys.exit(f"IK failed swing {phi}")
    log(f"swing phi={phi} tip={np.round(tip,3)} jump={np.abs(np.array(sol)-np.array(seed)).max():.2f}")
    swing.append(sol); seed = sol
if dry: sys.exit(0)

log("== lift straight up")
hx, hq = r.hand_pose()
lift = r.ik(hx + [0, 0, 0.12], hq)
r.move_joints([lift], [4.0])
log("== open"); r.gripper(0.04)
log("== high pre-pose"); r.move_joints([plan[0]], [max(3.0, np.abs(np.array(plan[0]) - np.array(r.joints())).max() / 0.4)])
log("== descend"); r.move_joints([plan[1]], [4.0])
log("== approach"); r.move_joints(plan[2:], [2.0, 4.0, 6.0])
log("hand", np.round(r.hand_pose()[0], 3))
log("== close"); f = r.gripper(0.0)
if f[0] < 0.007: sys.exit(f"!! missed the bar {f}")
log("== swing")
r.move_joints(swing, [3.0 + 2.0 * i for i in range(len(swing))], hold=3.0)
log("hand", np.round(r.hand_pose()[0], 3), "fingers", r.fingers())
log("== release"); r.gripper(0.04)
tip, q, a = pose(0.0, back=0.12)
sol = r.ik(tip, q, at_tcp=True)
if sol is not None: r.move_joints([sol], [3.0])
log("done; hand", np.round(r.hand_pose()[0], 3))
EOF
timeout 900 python3 door_close.py dry 2>&1 | grep -v "ik attempt"

# openrua op 92
timeout 900 python3 door_close.py 2>&1 | grep -v "ik attempt" | grep -v "^pre\|^swing phi"; python3 tools/perception/cam_snap.py agentview a14.png; python3 tools/perception/cam_snap.py frontview f14.png

# openrua op 93
cat > /workspace/door_measure.py <<'EOF'
import numpy as np, subprocess
subprocess.run(["python3","cloud.py","birdview","bird.npz"],check=True,capture_output=True)
d=np.load('bird.npz'); X,Y,Z=d['X'],d['Y'],d['Z']
m=np.isfinite(Z)&(Y<-0.36)&(Y>-0.70)&(Z>0.92)&(Z<1.12)&(X>-0.40)&(X<0.1)
p=np.stack([X[m],Y[m],Z[m]],1)
print('door/handle pts',m.sum(),'bbox',np.round(p.min(0),3),np.round(p.max(0),3))
for lo in np.arange(-0.66,-0.36,0.02):
    s=m&(Y>=lo)&(Y<lo+0.02)
    if s.sum()>2: print(f"y {lo:+.3f} n={s.sum():4d} x=[{X[s].min():.3f},{X[s].max():.3f}] zmax={Z[s].max():.3f}")
# handle bar: points with x < panel inner face - 0.01 ; panel ~ x -0.259..-0.278 when open 90
hb=m&(X<-0.283)
if hb.sum(): 
    q=np.stack([X[hb],Y[hb],Z[hb]],1); print('handle(x<-0.283)',hb.sum(),'bbox',np.round(q.min(0),3),np.round(q.max(0),3),'cen',np.round(q.mean(0),3))
EOF
cat > /workspace/lift_open.py <<'EOF'
import numpy as np
from ctl import Robot, log
r=Robot("lo"); r.gripper(0.04)
hx,hq=r.hand_pose(); s=r.ik(hx+[0,0,0.15],hq); r.move_joints([s],[4.0]); log("hand",np.round(r.hand_pose()[0],3))
EOF
timeout 300 python3 lift_open.py 2>&1 | grep -v "ik attempt" && python3 door_measure.py

# openrua op 94
timeout 300 python3 look.py -0.12 -0.25 1.38 2>&1 | grep -v "ik attempt" && python3 door_measure.py

# openrua op 95
python3 - <<'EOF'
s=open('door_close.py').read()
s=s.replace("H = np.array([-0.262, -0.35])","H = np.array([-0.247, -0.348])   # fitted from closed (-0.015,-0.399) and open-90 (-0.299,-0.580) bar centres")
s=s.replace("""hx, hq = r.hand_pose()
lift = r.ik(hx + [0, 0, 0.12], hq)
r.move_joints([lift], [4.0])""","""hx, hq = r.hand_pose()
if hx[2] < 1.3:
    lift = r.ik(hx + [0, 0, 0.12], hq)
    r.move_joints([lift], [4.0])""")
open('door_close.py','w').write(s)
EOF
timeout 900 python3 door_close.py 2>&1 | grep -v "ik attempt" | grep -v "^swing phi"; python3 tools/perception/cam_snap.py agentview a15.png; python3 tools/perception/cam_snap.py frontview f15.png

# openrua op 96
timeout 300 python3 look.py -0.12 -0.15 1.40 2>&1 | grep -v "ik attempt"; python3 scene.py 2>&1 | tail -8; python3 - <<'EOF'
import numpy as np
d=np.load('bird.npz'); X,Y,Z,c=d['X'],d['Y'],d['Z'],d['color'].astype(int)
b,g,r=c[...,0],c[...,1],c[...,2]
beyond=np.isfinite(Z)&(Y<-0.41)&(Z>0.905)&(Z<1.15)&(X>-0.4)&(X<0.15)
print('points beyond y<-0.41 (door open would show here):',beyond.sum())
front=np.isfinite(Z)&(Y<-0.35)&(Y>-0.41)&(Z>0.92)&(Z<1.12)&(X>-0.3)&(X<0.1)
p=np.stack([X[front],Y[front],Z[front]],1); print('front strip pts',front.sum(),'y range',np.round(p[:,1].min(),3),np.round(p[:,1].max(),3),'x range',np.round(p[:,0].min(),3),np.round(p[:,0].max(),3))
yel=np.isfinite(Z)&(r>150)&(g>120)&(b<110)&(Z>0.905)&(Z<1.2)
print('yellow points visible anywhere on/above table:',yel.sum())
EOF
python3 tools/perception/cam_snap.py agentview a16.png; python3 tools/perception/cam_snap.py frontview f16.png

# openrua op 97
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-mug-microwave-grasp-lessons.md)"
cat > /claude-config/projects/-workspace/memory/panda-mug-microwave-grasp-lessons.md <<'OPENRUA_EOF'
---
name: panda-mug-microwave-grasp-lessons
description: Hard-won grasp/insertion lessons from the Panda sim "mug into microwave" task (handle pinch acts like a ball joint; rim-side pinch + wrist swing rights the mug)
metadata:
  type: project
---

In the /workspace Panda + microwave sim (task done 2026-09-13), pinching the mug's
handle does NOT hold orientation: the pinch behaves like a ball joint and the mug
swings to hang like a bucket (axis horizontal). What worked instead: lay the mug on
its side, pinch the rim wall at its side point from the opening side with a
horizontal hand (one finger inside), lift, rotate the hand 135° about the finger
axis so the mug hangs upright, carry it hanging (it stays upright at any hand
tilt), lower it onto the cavity floor with the hand housing kept outside the
front plane (tip y <= -0.326 at 30° tilt), release, then push the handle in with
closed fingertips. Door: pinch the handle bar vertically and swing about the
hinge; the fitted hinge was (-0.247,-0.348), not the panel corner.

**Why:** the gripper is open/closed only with no force control, and the hand
housing (0.20 m wide) cannot enter the 0.208 m cavity opening.

**How to apply:** for similar pick-and-place of mugs with this gripper, prefer
rim-wall pinches over handle pinches and re-fit hinge geometry from measured
open/closed positions before swinging doors. Scripts: /workspace/rim_grasp.py,
door_open.py, door_close.py, drag.py.
OPENRUA_EOF

# openrua op 98
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true); echo "- [Panda mug/microwave grasp lessons](panda-mug-microwave-grasp-lessons.md) — handle pinch = ball joint; rim-side pinch + wrist swing rights the mug; fitted door hinge" >> MEMORY.md
