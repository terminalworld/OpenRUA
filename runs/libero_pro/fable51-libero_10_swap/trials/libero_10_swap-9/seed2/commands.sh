#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; ls -R /workspace/tools; ls -la /workspace/.ros

# openrua op 2
cat tools/action/ik_move.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 3
ros2 topic list 2>&1; echo ---; ros2 action list 2>&1; echo ---; ros2 service list 2>&1 | head -40; echo ---; ros2 node list 2>&1

# openrua op 4
for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|translation|x:|y:|z:|w:" | head -80

# openrua op 6
mkdir -p "$(dirname /workspace/px_many.py)"
cat > /workspace/px_many.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Convert many pixels of one camera to world coords in a single grab.
Usage: python3 px_many.py <camera> u,v [u,v ...]
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def _grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic,
                                   lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no message on {topic}")
    return got["m"]


def main():
    cam = sys.argv[1]
    pts = [tuple(int(x) for x in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("px_many")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth = _grab(node, f"/{cam}/depth/image_raw", Image)
    info = _grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    x, y, zz, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + zz * zz), 2 * (x * y - zz * w), 2 * (x * zz + y * w)],
        [2 * (x * y + zz * w), 1 - 2 * (x * x + zz * zz), 2 * (y * zz - x * w)],
        [2 * (x * zz - y * w), 2 * (y * zz + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    print(f"cam {cam} at world {T[:3,3]}")
    for (u, v) in pts:
        z = float(D[v, u])
        if not np.isfinite(z) or z <= 0:
            print(f"({u},{v}) no depth"); continue
        p = T @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z, 1.0])
        print(f"({u},{v}) d={z:.3f} -> {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 120 python3 px_many.py birdview 315,300 300,300 330,300 418,298 245,265 220,215 215,320 280,215 280,320 150,220 185,222 320,240 400,380 250,180 320,160

# openrua op 8
cat > grid.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
cam=sys.argv[1]; u0,u1,v0,v1,step=map(int,sys.argv[2:7])
def grab(node, topic, T):
    got={}; s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
rclpy.init(); node=rclpy.create_node("grid"); buf=Buffer(); TransformListener(buf,node)
d=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
D=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
frame=f"{cam}_optical_frame"
while not buf.can_transform("world",frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform("world",frame,rclpy.time.Time()); q=t.transform.rotation
x,y,zz,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+zz*zz),2*(x*y-zz*w),2*(x*zz+y*w)],[2*(x*y+zz*w),1-2*(x*x+zz*zz),2*(y*zz-x*w)],[2*(x*zz-y*w),2*(y*zz+x*w),1-2*(x*x+y*y)]])
T=np.eye(4); T[:3,:3]=R; T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
np.save(f"{cam}_depth.npy", D); np.save(f"{cam}_T.npy", T); np.save(f"{cam}_K.npy", np.array(info.k).reshape(3,3))
print("cam", T[:3,3])
print("      " + " ".join(f"{u:6d}" for u in range(u0,u1,step)))
for v in range(v0,v1,step):
    row=[]
    for u in range(u0,u1,step):
        z=float(D[v,u]); p=T@np.array([(u-cx)*z/fx,(v-cy)*z/fy,z,1.0]); row.append(f"{p[2]:6.3f}")
    print(f"v={v:4d} "+" ".join(row))
rclpy.shutdown()
EOF
timeout 120 python3 grid.py birdview 130 300 195 340 8

# openrua op 9
timeout 120 python3 px_many.py robot0_robotview 50,100 150,200 100,250 50,300 180,320 200,50 320,200 2>&1; timeout 120 python3 px_many.py agentview 100,200 200,250 150,300 70,250 30,180 100,170 60,160 320,270 2>&1

# openrua op 10
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -B1 -A12 "child_frame_id: robot0_eye" ; timeout 20 ros2 run tf2_ros tf2_echo panda_hand robot0_eye_in_hand_optical_frame 2>&1 | head -8

# openrua op 11
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library for this Panda: FK/IK, trajectory, gripper, snaps."""
import math
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, TwistStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

import cv2

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation
TCP = M["hand"]["tcp_offset_m"]


def quat_to_R(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def R_from_axes(zaxis, xaxis):
    """Rotation whose columns are hand x, y, z axes (world), given desired
    z (approach) and approximate x (finger-opening axis)."""
    z = np.asarray(zaxis, float); z /= np.linalg.norm(z)
    x = np.asarray(xaxis, float); x = x - z * (x @ z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        self.bridge = CvBridge()

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---------------- sensing ----------------
    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.2)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return np.array([d[j] for j in ARM])

    def finger_gap(self):
        d = self.joints()
        return d["panda_finger_joint1"], d["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """Hand pose in WORLD frame: (pos[3], quat[4], R[3,3])."""
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
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + BASE
        q = np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return pos, q, quat_to_R(q)

    def tcp(self, q=None):
        pos, quat, R = self.fk(q)
        return pos + TCP * R[:, 2], R

    def ik(self, pos_world, R, seed=None, at_tcp=True, timeout=20.0):
        """IK for hand pose (world). Returns joint array or None."""
        pos = np.asarray(pos_world, float)
        if at_tcp:
            pos = pos - TCP * R[:, 2]
        pos = pos - BASE
        qt = R_to_quat(R)
        self.ik_cli.wait_for_service(5)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, qt)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        if seed is None:
            seed = self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout + 30)
        res = fut.result()
        if res is None:
            print("IK: no answer"); return None
        if res.error_code.val != 1:
            print(f"IK failed code={res.error_code.val}"); return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in ARM])

    # ---------------- acting ----------------
    def move_q(self, q_list, seconds_list):
        """Send one trajectory through the given joint waypoints."""
        if not self.fjt.wait_for_server(10):
            raise RuntimeError("no fjt server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for q, s in zip(q_list, seconds_list):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(s), nanosec=int((s % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        qn = self.arm_q()
        err = np.abs(qn - np.asarray(q_list[-1])).max()
        print(f"traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_to(self, q, seconds=3.0):
        return self.move_q([q], [seconds])

    def gripper(self, width):
        if not self.grip.wait_for_server(10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"gripper -> reached={r.reached_goal} stalled={r.stalled} fingers={gap}")
        return gap

    def servo(self, lin, ang=(0, 0, 0), n=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(dt)

    def snap(self, cam, out=None):
        topic = f"/{cam}/color/image_raw"
        got = []
        sub = self.node.create_subscription(Image, topic, got.append, 1)
        while not got:
            self.spin(0.5)
        self.node.destroy_subscription(sub)
        out = out or f"/workspace/{cam}.png"
        cv2.imwrite(out, self.bridge.imgmsg_to_cv2(got[0], "bgr8"))
        return out
OPENRUA_EOF

# openrua op 12
timeout 120 python3 -c "
from rob import *
r=Robot()
q=r.arm_q(); print('q',q)
pos,quat,R=r.fk(q); print('hand pos',pos); print('R\n',R.round(3))
print('tcp', r.tcp(q)[0])
print('fingers', r.finger_gap())
"

# openrua op 13
timeout 200 python3 -c "
from rob import *
r=Robot()
r_BASE=BASE.copy()
q=r.arm_q()
import rob
rob.BASE=np.zeros(3)
pos,quat,R=r.fk(q); print('hand pos (raw)',pos)
sol=r.ik(pos,R,seed=q,at_tcp=False); print('ik world-coords sol',sol)
sol2=r.ik(pos-r_BASE,R,seed=q,at_tcp=False); print('ik base-coords sol',sol2)
"

# openrua op 14
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/rob.py", "old_string": "BASE = np.array([-0.66, 0.0, 0.912])  # world -> panda_link0 translation", "new_string": "# FK/IK on this machine already answer in WORLD coords (verified: FK of\\n# the home pose gives the hand at world (-0.20, 0, 1.27) and IK of that\\n# same world pose succeeds while the base-frame version fails -31).\\nBASE = np.zeros(3)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 15
timeout 300 python3 -c "
from rob import *
r=Robot()
q=r.arm_q()
R=R_from_axes([0.7,0,-0.7],[0,-1,0])
sol=r.ik([-0.56,-0.24,1.25],R,seed=q,at_tcp=False)
print(sol)
if sol is not None:
    r.move_to(sol,3.0)
    print('hand', r.fk()[0])
    r.snap('robot0_eye_in_hand','/workspace/eih1.png')
"

# openrua op 16
timeout 300 python3 -c "
from rob import *
r=Robot()
q=r.arm_q(); print('q now',q.round(3)); print('hand', r.fk()[0], r.fk()[2].round(2))
home=np.array([0,-0.161,0,-2.445,0,2.227,0.785])
for xa in ([0,-1,0],[0,1,0],[0.7,0,0.7],[-0.7,0,-0.7]):
    R=R_from_axes([0.7,0,-0.7],xa)
    for seed in (home,q):
        sol=r.ik([-0.56,-0.24,1.25],R,seed=seed,at_tcp=False)
        if sol is not None: print(xa, sol.round(2), 'dist', np.abs(sol-home).max().round(2))
"

# openrua op 17
timeout 300 python3 -c "
from rob import *
r=Robot()
q=r.arm_q()
home=np.array([0,-0.161,0,-2.445,0,2.227,0.785])
target=np.array([-0.29,-0.24,1.0])
lo=np.array([l[0] for l in FJT['limits_rad']]); hi=np.array([l[1] for l in FJT['limits_rad']])
for cam in ([-0.55,-0.30,1.45],[-0.50,-0.24,1.50],[-0.60,-0.45,1.35],[-0.45,-0.35,1.40]):
    cam=np.array(cam); d=target-cam
    for xa in ([0,-1,0],[0,1,0],[1,0,0],[-1,0,0]):
        R=R_from_axes(d,xa)
        for seed in (home,q):
            sol=r.ik(cam,R,seed=seed,at_tcp=False)
            if sol is not None:
                margin=np.minimum(sol-lo,hi-sol).min()
                print(cam, xa, sol.round(2), 'margin',margin.round(2))
" 2>&1 | grep -v IK

# openrua op 18
timeout 300 python3 -c "
from rob import *
r=Robot()
sol=np.array([-0.11,-0.7,-1.27,-2.23,0.13,1.75,2.08])
r.move_to(sol,3.0)
print('hand', r.fk()[0], r.fk()[2].round(2))
r.snap('robot0_eye_in_hand','/workspace/eih1.png')
"

# openrua op 19
timeout 300 python3 -c "
from rob import *
r=Robot()
sol=np.array([-0.11,-0.7,-1.27,-2.23,0.13,1.75,2.08])
print('before', r.arm_q().round(3))
r.move_to(sol,4.0)
print('after', r.arm_q().round(3))
print('hand', r.fk()[0], r.fk()[2].round(2))
r.snap('robot0_eye_in_hand','/workspace/eih1.png')
"

# openrua op 20
timeout 600 python3 -c "
from rob import *
r=Robot()
q=r.arm_q()
home=np.array([0,-0.161,0,-2.445,0,2.227,0.785])
target=np.array([-0.29,-0.24,1.0])
lo=np.array([l[0] for l in FJT['limits_rad']]); hi=np.array([l[1] for l in FJT['limits_rad']])
best=None
for cam in ([-0.65,-0.35,1.25],[-0.70,-0.30,1.20],[-0.62,-0.42,1.18],[-0.75,-0.40,1.15]):
    cam=np.array(cam); d=target-cam
    for xa in ([0,-1,0],[0,1,0],[1,0,0],[-1,0,0]):
        R=R_from_axes(d,xa)
        for seed in (home,q):
            sol=r.ik(cam,R,seed=seed,at_tcp=False)
            if sol is not None:
                margin=np.minimum(sol-lo,hi-sol).min()
                print(cam, xa, sol.round(2), 'margin',margin.round(2))
                if best is None or margin>best[0]: best=(margin,sol,cam)
print('BEST',best)
m,sol,cam=best
r.move_to(sol,4.0)
if np.abs(r.arm_q()-sol).max()>0.05: r.move_to(sol,4.0)
print('hand', r.fk()[0], r.fk()[2].round(2))
r.snap('robot0_eye_in_hand','/workspace/eih2.png')
" 2>&1 | grep -v "IK failed"

# openrua op 21
cat > cloud.py <<'EOF'
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
cam=sys.argv[1]
def grab(node, topic, T):
    got={}; s=node.create_subscription(T, topic, lambda m: got.setdefault("m",m),1)
    while "m" not in got: rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(s); return got["m"]
rclpy.init(); node=rclpy.create_node("cloud"); buf=Buffer(); TransformListener(buf,node)
d=grab(node,f"/{cam}/depth/image_raw",Image); info=grab(node,f"/{cam}/color/camera_info",CameraInfo)
D=np.frombuffer(d.data,dtype=np.float32).reshape(d.height,d.width)
fx,fy,cx,cy=info.k[0],info.k[4],info.k[2],info.k[5]
frame=f"{cam}_optical_frame"
while not buf.can_transform("world",frame,rclpy.time.Time()): rclpy.spin_once(node,timeout_sec=0.2)
t=buf.lookup_transform("world",frame,rclpy.time.Time()); q=t.transform.rotation
x,y,zz,w=q.x,q.y,q.z,q.w
R=np.array([[1-2*(y*y+zz*zz),2*(x*y-zz*w),2*(x*zz+y*w)],[2*(x*y+zz*w),1-2*(x*x+zz*zz),2*(y*zz-x*w)],[2*(x*zz-y*w),2*(y*zz+x*w),1-2*(x*x+y*y)]])
T=np.eye(4); T[:3,:3]=R; T[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
v,u=np.mgrid[0:d.height,0:d.width]
Z=D; ok=np.isfinite(Z)&(Z>0.05)
P=np.stack([(u-cx)*Z/fx,(v-cy)*Z/fy,Z,np.ones_like(Z)],-1)
W=(P@T.T)[...,:3]
np.save(f"{cam}_world.npy", W); np.save(f"{cam}_ok.npy", ok)
print("cam at", T[:3,3], "saved", W.shape)
rclpy.shutdown()
EOF
timeout 120 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
P=W[ok]
sel=(P[:,0]>-0.45)&(P[:,0]<0.1)&(P[:,1]>-0.42)&(P[:,1]<-0.08)&(P[:,2]>0.905)&(P[:,2]<1.12)
Q=P[sel]; print('pts',len(Q))
# for bins of y,z print min x of surface (closest-to-robot surface) and max x
ys=np.arange(-0.42,-0.08,0.02); zs=np.arange(0.92,1.12,0.02)
print('      '+' '.join(f'{y:6.2f}' for y in ys))
for z in zs:
    row=[]
    for y in ys:
        m=(Q[:,1]>=y)&(Q[:,1]<y+0.02)&(Q[:,2]>=z)&(Q[:,2]<z+0.02)
        row.append(f'{Q[m,0].min():6.2f}' if m.any() else '   .  ')
    print(f'z={z:.2f} '+' '.join(row))
"

# openrua op 22
python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
P=W[ok]
sel=(P[:,0]>-0.45)&(P[:,0]<0.15)&(P[:,1]>-0.70)&(P[:,1]<-0.28)&(P[:,2]>0.905)&(P[:,2]<1.15)
Q=P[sel]; print('pts',len(Q))
xs=np.arange(-0.44,0.14,0.02); zs=np.arange(0.92,1.14,0.02)
print('max y of surface per (x,z) bin (the -y face seen from -y side):')
print('      '+' '.join(f'{x:6.2f}' for x in xs))
for z in zs:
    row=[]
    for x in xs:
        m=(Q[:,0]>=x)&(Q[:,0]<x+0.02)&(Q[:,2]>=z)&(Q[:,2]<z+0.02)
        row.append(f'{Q[m,1].max():6.2f}' if m.any() else '   .  ')
    print(f'z={z:.2f} '+' '.join(row))
print()
print('min y per bin:')
for z in zs:
    row=[]
    for x in xs:
        m=(Q[:,0]>=x)&(Q[:,0]<x+0.02)&(Q[:,2]>=z)&(Q[:,2]<z+0.02)
        row.append(f'{Q[m,1].min():6.2f}' if m.any() else '   .  ')
    print(f'z={z:.2f} '+' '.join(row))
"

# openrua op 23
timeout 120 python3 tools/perception/cam_snap.py galleryview; timeout 120 python3 tools/perception/cam_snap.py paperview

# openrua op 24
timeout 600 python3 -c "
from rob import *
r=Robot()
q=r.arm_q()
home=np.array([0,-0.161,0,-2.445,0,2.227,0.785])
lo=np.array([l[0] for l in FJT['limits_rad']]); hi=np.array([l[1] for l in FJT['limits_rad']])
best=None
cam=np.array([-0.22,-0.33,1.55])
for xa in ([1,0,0],[0,1,0],[-1,0,0],[0,-1,0]):
    R=R_from_axes([0,0,-1],xa)
    for seed in (home,q):
        sol=r.ik(cam,R,seed=seed,at_tcp=False)
        if sol is not None:
            margin=np.minimum(sol-lo,hi-sol).min()
            print(xa, sol.round(2), 'margin',margin.round(2))
            if best is None or margin>best[0]: best=(margin,sol,xa)
print('BEST',best)
m,sol,xa=best
r.move_to(sol,4.0)
if np.abs(r.arm_q()-sol).max()>0.05: r.move_to(sol,4.0)
print('hand', r.fk()[0], r.fk()[2].round(2))
r.snap('robot0_eye_in_hand','/workspace/eih3.png')
" 2>&1 | grep -v "IK failed"; timeout 120 python3 cloud.py robot0_eye_in_hand

# openrua op 25
python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
P=W[ok]
sel=(P[:,0]>-0.6)&(P[:,0]<0.2)&(P[:,1]>-0.8)&(P[:,1]<0.1)
Q=P[sel]
xs=np.arange(-0.60,0.20,0.02); ys=np.arange(-0.80,0.10,0.02)
print('max z per (x,y) bin')
print('       '+' '.join(f'{y:5.2f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(Q[:,0]>=x)&(Q[:,0]<x+0.02)&(Q[:,1]>=y)&(Q[:,1]<y+0.02)
        row.append(f'{Q[m,2].max():5.2f}' if m.any() else '  .  ')
    print(f'x={x:5.2f} '+' '.join(row))
"

# openrua op 26
python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
for (u,v) in [(95,320),(430,150),(350,300),(170,140),(250,400),(400,230),(300,200),(200,250),(330,290),(360,250)]:
    print((u,v), W[v,u].round(3), ok[v,u])
"

# openrua op 27
timeout 900 python3 -c "
from rob import *
r=Robot()
q=r.arm_q()
home=np.array([0,-0.161,0,-2.445,0,2.227,0.785])
lo=np.array([l[0] for l in FJT['limits_rad']]); hi=np.array([l[1] for l in FJT['limits_rad']])
best=None
target=np.array([-0.12,-0.33,0.98])
for cam in ([-0.15,-0.55,1.20],[-0.10,-0.52,1.25],[-0.20,-0.58,1.15]):
    cam=np.array(cam); d=target-cam
    for xa in ([1,0,0],[0,1,0],[-1,0,0],[0,-1,0]):
        R=R_from_axes(d,xa)
        for seed in (home,q):
            sol=r.ik(cam,R,seed=seed,at_tcp=False)
            if sol is not None:
                margin=np.minimum(sol-lo,hi-sol).min()
                print(cam,xa, sol.round(2), 'margin',margin.round(2))
                if best is None or margin>best[0]: best=(margin,sol,cam)
print('BEST',best)
m,sol,cam=best
r.move_to(sol,5.0)
if np.abs(r.arm_q()-sol).max()>0.05: r.move_to(sol,5.0)
print('hand', r.fk()[0], r.fk()[2].round(2))
r.snap('robot0_eye_in_hand','/workspace/eih4.png')
" 2>&1 | grep -v "IK failed"; timeout 120 python3 cloud.py robot0_eye_in_hand

# openrua op 28
python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
P=W[ok]
sel=(P[:,0]>-0.30)&(P[:,0]<0.08)&(P[:,1]>-0.40)&(P[:,1]<-0.25)&(P[:,2]>0.905)&(P[:,2]<1.10)
Q=P[sel]; print(len(Q))
xs=np.arange(-0.30,0.08,0.02); zs=np.arange(0.90,1.10,0.02)
print('max y per (x,z) bin for points y<-0.25 (side face / cavity):')
print('      '+' '.join(f'{x:6.2f}' for x in xs))
for z in zs:
    row=[]
    for x in xs:
        m=(Q[:,0]>=x)&(Q[:,0]<x+0.02)&(Q[:,2]>=z)&(Q[:,2]<z+0.02)
        row.append(f'{Q[m,1].max():6.3f}' if m.any() else '   .  ')
    print(f'z={z:.2f} '+' '.join(row))
"

# openrua op 29
python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
P=W[ok]
sel=(P[:,0]>-0.28)&(P[:,0]<-0.02)&(P[:,1]>-0.33)&(P[:,1]<-0.10)&(P[:,2]>0.905)&(P[:,2]<1.09)
Q=P[sel]; print(len(Q))
print('z range',Q[:,2].min(),Q[:,2].max(),' y range',Q[:,1].min(),Q[:,1].max())
xs=np.arange(-0.28,-0.02,0.02); ys=np.arange(-0.33,-0.10,0.02)
print('min z per (x,y) bin (cavity floor):')
print('      '+' '.join(f'{y:6.2f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(Q[:,0]>=x)&(Q[:,0]<x+0.02)&(Q[:,1]>=y)&(Q[:,1]<y+0.02)
        row.append(f'{Q[m,2].min():6.3f}' if m.any() else '   .  ')
    print(f'x={x:.2f} '+' '.join(row))
print('max z per bin:')
for x in xs:
    row=[]
    for y in ys:
        m=(Q[:,0]>=x)&(Q[:,0]<x+0.02)&(Q[:,1]>=y)&(Q[:,1]<y+0.02)
        row.append(f'{Q[m,2].max():6.3f}' if m.any() else '   .  ')
    print(f'x={x:.2f} '+' '.join(row))
"

# openrua op 30
python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
P=W[ok]
sel=(P[:,0]>-0.08)&(P[:,0]<0.12)&(P[:,1]>-0.12)&(P[:,1]<0.08)&(P[:,2]>0.905)
Q=P[sel]; print(len(Q), 'z max', Q[:,2].max())
xs=np.arange(-0.08,0.12,0.01); ys=np.arange(-0.12,0.08,0.01)
print('max z per (x,y) bin, 1cm:')
print('      '+' '.join(f'{y:5.2f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(Q[:,0]>=x)&(Q[:,0]<x+0.01)&(Q[:,1]>=y)&(Q[:,1]<y+0.01)
        row.append(f'{Q[m,2].max():5.3f}' if m.any() else '  .  ')
    print(f'x={x:5.2f} '+' '.join(row))
"

# openrua op 31
timeout 600 python3 -c "
from rob import *
r=Robot()
q=r.arm_q()
home=np.array([0,-0.161,0,-2.445,0,2.227,0.785])
lo=np.array([l[0] for l in FJT['limits_rad']]); hi=np.array([l[1] for l in FJT['limits_rad']])
R=R_from_axes([0,1,0],[1,0,0])
R2=R_from_axes([0,1,0],[-1,0,0])
for name,p in [('pregrasp',[0.015,-0.13,0.965]),('grasp',[0.015,-0.068,0.965]),('lift',[0.015,-0.068,1.03]),('preinsert',[-0.11,-0.45,1.02]),('insert',[-0.11,-0.315,0.99]),('insert2',[-0.11,-0.30,0.975])]:
    for Rx,nm in ((R,'x+'),(R2,'x-')):
        for seed in (home,q):
            sol=r.ik(p,Rx,seed=seed,at_tcp=True)
            if sol is not None:
                margin=np.minimum(sol-lo,hi-sol).min()
                print(name,nm, sol.round(2), 'margin',margin.round(2))
" 2>&1 | grep -v "IK failed"

# openrua op 32
timeout 900 python3 -c "
from rob import *
r=Robot()
rng=np.random.default_rng(0)
lo=np.array([l[0] for l in FJT['limits_rad']]); hi=np.array([l[1] for l in FJT['limits_rad']])
home=np.array([0,-0.161,0,-2.445,0,2.227,0.785])
seeds=[home]+[lo+(hi-lo)*rng.random(7) for _ in range(6)]
for tilt in (0,20,35):
    t=np.radians(tilt)
    R=R_from_axes([0,np.cos(t),-np.sin(t)],[1,0,0])
    for name,p in [('pregrasp',[0.015,-0.13,0.965]),('preinsert',[-0.11,-0.45,1.0]),('insert',[-0.11,-0.315,0.99])]:
        found=[]
        for seed in seeds:
            sol=r.ik(p,R,seed=seed,at_tcp=True,timeout=5)
            if sol is not None:
                margin=np.minimum(sol-lo,hi-sol).min(); found.append((margin,sol))
        if found:
            m,s=max(found,key=lambda f:f[0]); print('tilt',tilt,name,'OK',s.round(2),'margin',m.round(2),'n',len(found))
        else: print('tilt',tilt,name,'NONE')
" 2>&1 | grep -v "IK failed"

# openrua op 33
cat >> /workspace/rob.py <<'EOF'


LO = np.array([l[0] for l in FJT["limits_rad"]])
HI = np.array([l[1] for l in FJT["limits_rad"]])
HOME = np.array([0, -0.161, 0, -2.445, 0, 2.227, 0.785])


def tilt_R(tilt_deg, xaxis=(1, 0, 0)):
    """Hand pointing down, tilted back toward -y by tilt_deg (fingertips
    at +y of the wrist). Finger axis along world x."""
    t = np.radians(tilt_deg)
    return R_from_axes([0, np.sin(t), -np.cos(t)], xaxis)


def best_ik(r, pos, R, seeds=None, at_tcp=True, timeout=3.0, near=None):
    """Try several seeds; return the solution with best limit margin
    (or nearest to `near` if given)."""
    if seeds is None:
        seeds = [r.arm_q(), HOME]
    found = []
    for s in seeds:
        sol = r.ik(pos, R, seed=s, at_tcp=at_tcp, timeout=timeout)
        if sol is not None:
            margin = np.minimum(sol - LO, HI - sol).min()
            found.append((margin, sol))
    if not found:
        return None
    if near is not None:
        return min(found, key=lambda f: np.abs(f[1] - near).max())[1]
    return max(found, key=lambda f: f[0])[1]


def goto_q(r, q, seconds=4.0, tries=3, tol=0.02):
    for i in range(tries):
        code, err = r.move_q([q], [seconds])
        if err < tol:
            return True
    return False


def goto(r, pos, R, seconds=4.0, seeds=None, near=None, at_tcp=True):
    q = best_ik(r, pos, R, seeds=seeds, at_tcp=at_tcp, near=near)
    if q is None:
        print("goto: IK failed for", pos)
        return None
    ok = goto_q(r, q, seconds)
    tcp, _ = r.tcp()
    print(f"goto {np.round(pos,3)} -> tcp {np.round(tcp,3)} ok={ok}")
    return q
EOF
timeout 900 python3 -c "
from rob import *
r=Robot()
r.gripper(0.04)
R=tilt_R(40)
q1=goto(r,[0.015,-0.068,1.12],R,seconds=5.0)
r.snap('robot0_eye_in_hand','/workspace/eih5.png'); r.snap('agentview','/workspace/agent2.png')
"

# openrua op 34
timeout 120 python3 cloud.py robot0_eye_in_hand && python3 -c "
import numpy as np
W=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
P=W[ok]
sel=(P[:,0]>-0.06)&(P[:,0]<0.09)&(P[:,1]>-0.12)&(P[:,1]<0.06)&(P[:,2]>0.905)&(P[:,2]<1.03)
Q=P[sel]
xs=np.arange(-0.06,0.09,0.01); ys=np.arange(-0.12,0.06,0.01)
print('max z per (x,y) bin, 1cm:')
print('      '+' '.join(f'{y:5.2f}' for y in ys))
for x in xs:
    row=[]
    for y in ys:
        m=(Q[:,0]>=x)&(Q[:,0]<x+0.01)&(Q[:,1]>=y)&(Q[:,1]<y+0.01)
        row.append(f'{Q[m,2].max():5.3f}' if m.any() else '  .  ')
    print(f'x={x:5.2f} '+' '.join(row))
# handle points: y<-0.05
H=Q[(Q[:,1]<-0.052)&(Q[:,2]>0.93)]
print('handle pts',len(H)); 
if len(H): print('handle x range',H[:,0].min(),H[:,0].max(),'y range',H[:,1].min(),H[:,1].max(),'z range',H[:,2].min(),H[:,2].max())
print('fingers world:'); 
sel2=(P[:,2]>1.03)&(P[:,2]<1.15); F=P[sel2]; print(len(F)); 
if len(F): print('x range',F[:,0].min(),F[:,0].max())
"

# openrua op 35
timeout 900 python3 -c "
from rob import *
r=Robot()
R=tilt_R(40)
q0=r.arm_q()
q1=goto(r,[0.019,-0.066,1.01],R,seconds=3.0,near=q0)
q2=goto(r,[0.019,-0.066,0.978],R,seconds=3.0,near=q1)
r.snap('robot0_eye_in_hand','/workspace/eih6.png'); r.snap('frontview','/workspace/front2.png')
"

# openrua op 36
timeout 120 python3 -u -c "
from rob import *
r=Robot('diag')
q=r.arm_q(); print('q',np.round(q,3))
print('margin',np.round(np.minimum(q-LO,HI-q),3))
t,R=r.tcp(); print('tcp',np.round(t,3)); print('R',np.round(R,3))
R=tilt_R(40)
for near in [None]:
  sol=best_ik(r,[0.019,-0.066,0.978],R,seeds=[q,HOME,[0,0.5,0,-1.8,0,2.3,0.8],[0.3,0.3,-0.3,-2.0,0.2,2.4,1.0]])
  print('sol',None if sol is None else np.round(sol,3))
  if sol is not None: print('dq',np.round(sol-q,3)); print('margin',np.round(np.minimum(sol-LO,HI-sol),3))
"


# openrua op 37
timeout 300 python3 -u -c "
from rob import *
r=Robot('lift')
q0=r.arm_q()
R=tilt_R(0)
q=goto(r,[0.03,-0.05,1.25],R,seconds=4.0,seeds=[q0,HOME])
print('q',np.round(r.arm_q(),3))
r.snap('frontview','/workspace/front3.png')
"


# openrua op 38
timeout 400 python3 -u -c "
from rob import *
r=Robot('grasp')
print('fingers',r.finger_gap())
R=tilt_R(0)
q0=r.arm_q()
q1=goto(r,[0.019,-0.072,1.05],R,seconds=3.0,seeds=[q0,HOME],near=q0)
q2=goto(r,[0.019,-0.072,0.965],R,seconds=3.0,seeds=[q1,q0],near=q1)
r.snap('robot0_eye_in_hand','/workspace/eih7.png'); r.snap('frontview','/workspace/front3.png')
"


# openrua op 39
timeout 400 python3 -u -c "
from rob import *
r=Robot('grasp2')
R=tilt_R(0)
q0=r.arm_q()
q1=goto(r,[0.019,-0.077,1.05],R,seconds=3.0,seeds=[q0,HOME],near=q0)
q2=goto(r,[0.019,-0.077,0.965],R,seconds=3.0,seeds=[q1,q0],near=q1)
print('q',np.round(r.arm_q(),3),'target',np.round(q2,3))
r.snap('robot0_eye_in_hand','/workspace/eih8.png')
"


# openrua op 40
python3 - <<'EOF'
import re
s=open('rob.py').read()
old='''def goto_q(r, q, seconds=4.0, tries=3, tol=0.02):
    for i in range(tries):
        code, err = r.move_q([q], [seconds])
        if err < tol:
            return True
    return False
'''
new='''def goto_q(r, q, seconds=4.0, tries=4, tol=0.01):
    """Resend until the joints settle within tol of q. The controller shows
    a steady-state sag under load, so after the first try the command is
    offset by the residual error (integral-style compensation)."""
    q = np.asarray(q, float)
    cmd = q.copy()
    for i in range(tries):
        code, err = r.move_q([cmd], [seconds])
        qn = r.arm_q()
        err = np.abs(qn - q).max()
        print(f"  settle err vs target={err:.4f}")
        if err < tol:
            return True
        cmd = np.clip(cmd + (q - qn), LO, HI)
        seconds = max(1.5, seconds / 2)
    return False
'''
assert old in s
open('rob.py','w').write(s.replace(old,new))
EOF
timeout 400 python3 -u -c "
from rob import *
r=Robot('grasp3')
R=tilt_R(0)
q0=r.arm_q()
q2=goto(r,[0.019,-0.077,0.965],R,seconds=3.0,seeds=[q0,HOME],near=q0)
r.snap('robot0_eye_in_hand','/workspace/eih8.png')
"


# openrua op 41
timeout 400 python3 -u -c "
from rob import *
r=Robot('grasp4')
R=tilt_R(0)
q0=r.arm_q()
q1=goto(r,[0.019,-0.077,1.05],R,seconds=3.0,seeds=[q0,HOME],near=q0)
print('q now',np.round(r.arm_q(),3)); print('q tgt',np.round(q1,3))
"


# openrua op 42
timeout 60 ros2 topic echo --once /franka_robot_state_broadcaster/external_wrench 2>&1 | head -30; timeout 200 python3 -u -c "
from rob import *
r=Robot('j3test')
q=r.arm_q(); print('q',np.round(q,3))
for d in [0.1,-0.1,-0.2]:
    t=q.copy(); t[2]+=d
    r.move_q([t],[2.0]); print(' j3 cmd',round(t[2],3),'got',round(r.arm_q()[2],3))
"


# openrua op 43
cat >> rob.py <<'EOF'


def wrench(r, n=1):
    from geometry_msgs.msg import WrenchStamped
    got = []
    sub = r.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", got.append, 1)
    while len(got) < n:
        r.spin(0.5)
    r.node.destroy_subscription(sub)
    w = got[-1].wrench
    return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])
EOF
timeout 400 python3 -u -c "
from rob import *
r=Robot('probe')
R=tilt_R(0)
print('wrench now',np.round(wrench(r),1))
q0=r.arm_q()
for z in [1.25,1.15,1.10,1.05]:
    q=goto(r,[0.019,-0.077,z],R,seconds=3.0,seeds=[q0,HOME],near=q0)
    print(' z',z,'q',np.round(r.arm_q(),3),'wrench',np.round(wrench(r),1))
"


# openrua op 44
timeout 120 python3 cloud.py robot0_eye_in_hand > /dev/null 2>&1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
pts=P[ok]
print('n',len(pts))
# points near the hand region above table
m=(pts[:,2]>0.95)&(pts[:,0]>-0.15)&(pts[:,0]<0.2)&(pts[:,1]>-0.2)&(pts[:,1]<0.15)
q=pts[m]
print('high pts',len(q), 'zmax',q[:,2].max() if len(q) else None)
# histogram of z
h,e=np.histogram(q[:,2],bins=np.arange(0.95,1.2,0.01)); 
for a,b in zip(e[:-1],h): print(f'{a:.2f} {b}')
# print xy of points with z>1.0
hi=q[q[:,2]>1.0]
if len(hi): print('z>1.0 x range',hi[:,0].min(),hi[:,0].max(),'y range',hi[:,1].min(),hi[:,1].max())
"


# openrua op 45
timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
pts=P[ok]
def show(m,name):
    q=pts[m]; print(name,len(q))
    if len(q):
        # bin by x,y 1cm
        xs=np.round(q[:,0],2); ys=np.round(q[:,1],2)
        import collections
        c=collections.Counter(zip(xs,ys))
        for k,v in sorted(c.items()): 
            if v>5: print('  ',k,v, 'zmax %.3f'%q[(xs==k[0])&(ys==k[1]),2].max())
show((pts[:,2]>1.0)&(pts[:,2]<1.04)&(pts[:,1]<-0.06),'z 1.00-1.04, y<-0.06')
show((pts[:,2]>1.05)&(pts[:,2]<1.105),'z 1.05-1.105')
"


# openrua op 46
timeout 20 ros2 run tf2_ros tf2_echo world robot0_eye_in_hand_optical_frame 2>&1 | head -12; timeout 60 python3 -u -c "
from rob import *
r=Robot('fkchk')
p,q,R=r.fk(); print('hand',np.round(p,3)); print('R',np.round(R,3)); print('cam est',np.round(p+0.05*R[:,0],3))
"

# openrua op 47
timeout 120 python3 -u -c "
from rob import *
r=Robot('l8')
p8,q8,R8=r.fk(link='panda_link8'); ph,qh,Rh=r.fk(link='panda_hand')
print('link8',np.round(p8,3)); print(np.round(R8,3)); print('hand',np.round(ph,3)); print(np.round(Rh,3))
print('Rh^T R8 =');print(np.round(Rh.T@R8,3))
# try IK with ik_link_name
from moveit_msgs.srv import GetPositionIK
"


# openrua op 48
python3 - <<'EOF'
s=open('rob.py').read()
old='''        pos = pos - BASE
        qt = R_to_quat(R)
'''
new='''        pos = pos - BASE
        # /compute_ik solves for the group's tip link panda_link8, which is
        # rotated 45 deg about z relative to panda_hand (verified via FK:
        # R_link8 = R_hand @ Rz(+45deg)); same origin, same z axis.
        qt = R_to_quat(np.asarray(R) @ RZ45)
'''
assert old in s; s=s.replace(old,new)
old2='''BASE = np.zeros(3)
'''
new2='''BASE = np.zeros(3)
_c = math.sqrt(0.5)
RZ45 = np.array([[_c, -_c, 0.0], [_c, _c, 0.0], [0.0, 0.0, 1.0]])
'''
assert old2 in s; s=s.replace(old2,new2)
open('rob.py','w').write(s)
EOF
timeout 400 python3 -u -c "
from rob import *
r=Robot('grasp5')
R=tilt_R(0)
q0=r.arm_q()
q1=goto(r,[0.019,-0.077,1.10],R,seconds=3.0,seeds=[q0,HOME])
p,_,Rh=r.fk(); print('hand R x-axis',np.round(Rh[:,0],3))
q2=goto(r,[0.019,-0.077,0.965],R,seconds=3.0,seeds=[q1,HOME],near=q1)
print('wrench',np.round(wrench(r),1))
r.snap('robot0_eye_in_hand','/workspace/eih9.png')
"


# openrua op 49
python3 - <<'EOF'
s=open('rob.py').read()
i=s.index('def goto_q'); j=s.index('def goto(')
new='''def goto_q(r, q, seconds=4.0, tries=4, tol=0.01):
    """Resend until the joints settle within tol of q. A first -5 is usually
    controller lag: resend unchanged. A repeatable small residual is
    compensated by offsetting the command; a growing residual means contact
    -> stop pushing."""
    q = np.asarray(q, float)
    cmd = q.copy()
    prev = None
    for i in range(tries):
        code, _ = r.move_q([cmd], [seconds])
        qn = r.arm_q()
        err = np.abs(qn - q).max()
        print(f"  settle err vs target={err:.4f}")
        if err < tol:
            return True
        if prev is not None and err > prev * 1.2:
            print("  residual growing -> probable contact, stop")
            return False
        if prev is not None and abs(err - prev) < 0.3 * prev and err < 0.06:
            cmd = np.clip(cmd + (q - qn), LO, HI)
        prev = err
        seconds = max(1.5, seconds / 2)
    return False


'''
s=s[:i]+new+s[j:]
s=s.replace('''def tilt_R(tilt_deg, xaxis=(1, 0, 0)):
    """Hand pointing down, tilted back toward -y by tilt_deg (fingertips
    at +y of the wrist). Finger axis along world x."""
    t = np.radians(tilt_deg)
    return R_from_axes([0, np.sin(t), -np.cos(t)], xaxis)''',
'''def tilt_R(tilt_deg):
    """Hand pointing down, tilted back toward -y by tilt_deg (fingertips
    at +y of the wrist). Fingers open along hand y; here hand y = world x
    (hand x = world +y-ish), so the pinch axis is world x and the 0.2 m
    wide hand body lies along x, its 0.06 m thickness along y."""
    t = np.radians(tilt_deg)
    return R_from_axes([0, np.sin(t), -np.cos(t)], [0, np.cos(t), np.sin(t)])''')
open('rob.py','w').write(s)
EOF
timeout 400 python3 -u -c "
from rob import *
r=Robot('grasp6')
R=tilt_R(0); print('R',np.round(R,2))
q0=r.arm_q()
q1=goto(r,[0.019,-0.080,1.08],R,seconds=3.0,seeds=[q0,HOME])
p,_,Rh=r.fk(); print('hand axes x',np.round(Rh[:,0],2),'y',np.round(Rh[:,1],2))
r.snap('robot0_eye_in_hand','/workspace/eih10.png')
print('wrench',np.round(wrench(r),1))
"


# openrua op 50
timeout 400 python3 -u -c "
from rob import *
r=Robot('grasp7')
R=tilt_R(0)
q0=r.arm_q()
q1=goto(r,[0.019,-0.080,0.962],R,seconds=3.0,seeds=[q0,HOME],near=q0)
print('wrench',np.round(wrench(r),1))
r.snap('robot0_eye_in_hand','/workspace/eih11.png'); r.snap('frontview','/workspace/front4.png')
"


# openrua op 51
timeout 200 python3 -u -c "
from rob import *
r=Robot('close')
g=r.gripper(0.0)
print('gap',g)
r.snap('robot0_eye_in_hand','/workspace/eih12.png')
"


# openrua op 52
timeout 400 python3 -u -c "
from rob import *
r=Robot('lift2')
R=tilt_R(0)
q0=r.arm_q()
q1=goto(r,[0.019,-0.080,1.08],R,seconds=3.0,seeds=[q0,HOME],near=q0)
print('fingers',r.finger_gap())
r.snap('frontview','/workspace/front5.png')
"


# openrua op 53
timeout 500 python3 -u -c "
from rob import *
r=Robot('reloc')
R=tilt_R(0)
q0=r.arm_q()
q1=goto(r,[0.02,0.085,1.08],R,seconds=4.0,seeds=[q0,HOME],near=q0)
q2=goto(r,[0.02,0.085,0.967],R,seconds=3.0,seeds=[q1,HOME],near=q1)
print('wrench',np.round(wrench(r),1))
g=r.gripper(0.04)
q3=goto(r,[0.02,0.085,1.10],R,seconds=3.0,seeds=[q2,HOME],near=q2)
r.snap('frontview','/workspace/front6.png'); r.snap('birdview','/workspace/bird2.png')
"


# openrua op 54
timeout 120 python3 cloud.py robot0_eye_in_hand > /dev/null 2>&1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
pts=P[ok]
m=(pts[:,2]>0.995)&(pts[:,2]<1.02)&(pts[:,0]>-0.15)&(pts[:,0]<0.2)&(pts[:,1]>-0.05)&(pts[:,1]<0.3)
rim=pts[m]; print('rim n',len(rim),'x',rim[:,0].min(),rim[:,0].max(),'y',rim[:,1].min(),rim[:,1].max(),'zmean',rim[:,2].mean())
cx=(rim[:,0].min()+rim[:,0].max())/2; cy=(rim[:,1].min()+rim[:,1].max())/2; print('center',cx,cy)
m2=(pts[:,2]>0.925)&(pts[:,2]<0.995)&(pts[:,1]<cy-0.052)&(pts[:,1]>cy-0.12)&(abs(pts[:,0]-cx)<0.05)
h=pts[m2]; print('handle n',len(h))
if len(h): print(' x',h[:,0].min(),h[:,0].max(),'y',h[:,1].min(),h[:,1].max(),'z',h[:,2].min(),h[:,2].max())
zs=np.round(h[:,2],2)
for z in np.unique(zs):
    s=h[zs==z]; print(f'  z {z:.2f} n={len(s)} x {s[:,0].min():.3f}-{s[:,0].max():.3f} y {s[:,1].min():.3f}-{s[:,1].max():.3f}')
"


# openrua op 55
timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
pts=P[ok]
m=(pts[:,2]>0.995)&(pts[:,2]<1.02)&(pts[:,0]>-0.15)&(pts[:,0]<0.2)&(pts[:,1]>-0.05)&(pts[:,1]<0.24)
rim=pts[m]; print('rim n',len(rim),'x',rim[:,0].min(),rim[:,0].max(),'y',rim[:,1].min(),rim[:,1].max(),'zmax',rim[:,2].max())
cx=(rim[:,0].min()+rim[:,0].max())/2; cy=(rim[:,1].min()+rim[:,1].max())/2; print('center',cx,cy)
m2=(pts[:,2]>0.925)&(pts[:,2]<0.995)&(pts[:,1]<0.118)&(pts[:,1]>0.05)&(abs(pts[:,0]-cx)<0.05)
h=pts[m2]; print('handle n',len(h))
zs=np.round(h[:,2],2)
for z in np.unique(zs):
    s=h[zs==z]; print(f'  z {z:.2f} n={len(s)} x {s[:,0].min():.3f}-{s[:,0].max():.3f} y {s[:,1].min():.3f}-{s[:,1].max():.3f}')
"


# openrua op 56
timeout 500 python3 -u -c "
from rob import *
r=Robot('regrasp')
R=tilt_R(40); print(np.round(R,2))
q0=r.arm_q()
q1=goto(r,[0.0175,0.087,1.03],R,seconds=4.0,seeds=[q0,HOME])
p,_,Rh=r.fk(); print('hand z',np.round(Rh[:,2],2),'y',np.round(Rh[:,1],2))
r.snap('robot0_eye_in_hand','/workspace/eih13.png')
q2=goto(r,[0.0175,0.087,0.962],R,seconds=3.0,seeds=[q1,HOME],near=q1)
print('wrench',np.round(wrench(r),1))
r.snap('frontview','/workspace/front7.png')
"


# openrua op 57
timeout 400 python3 -u -c "
from rob import *
r=Robot('close2')
g=r.gripper(0.0); print('gap',g)
R=tilt_R(40); q0=r.arm_q()
q1=goto(r,[0.0175,0.087,1.035],R,seconds=3.0,seeds=[q0,HOME],near=q0)
print('fingers',r.finger_gap(),'wrench',np.round(wrench(r),1))
r.snap('frontview','/workspace/front8.png'); r.snap('agentview','/workspace/agent3.png')
"


# openrua op 58
timeout 900 python3 -u -c "
from rob import *
r=Robot('carry')
R=tilt_R(40); q=r.arm_q()
wps=[[0.0,0.0,1.22],[-0.06,-0.22,1.22],[-0.12,-0.44,1.20],[-0.12,-0.44,1.10],[-0.12,-0.44,1.022]]
for w in wps:
    q=goto(r,w,R,seconds=4.0,seeds=[q,HOME],near=q)
    if q is None: break
    print('  fingers',np.round(r.finger_gap(),4),'wrench',np.round(wrench(r),1))
r.snap('frontview','/workspace/front9.png'); r.snap('agentview','/workspace/agent4.png'); r.snap('robot0_eye_in_hand','/workspace/eih14.png')
"


# openrua op 59
timeout 600 python3 -u -c "
from rob import *
r=Robot('iktest')
R=tilt_R(40); q=r.arm_q()
rng=np.random.default_rng(0)
seeds=[q,HOME]+[np.clip(q+rng.normal(0,0.5,7),LO+0.1,HI-0.1) for _ in range(8)]
for tgt in [[-0.12,-0.44,1.20],[-0.12,-0.44,1.15],[-0.12,-0.42,1.20],[-0.10,-0.44,1.16]]:
    found=[]
    for s in seeds:
        sol=r.ik(tgt,R,seed=s,timeout=2.0)
        if sol is not None: found.append(sol)
    print(tgt,'n sol',len(found))
    for f in found[:3]: print('   ',np.round(f,2),'margin',round(float(np.minimum(f-LO,HI-f).min()),2))
" 2>&1 | grep -v 'IK failed'


# openrua op 60
timeout 900 python3 -u -c "
from rob import *
r=Robot('iktest2')
q=r.arm_q()
rng=np.random.default_rng(1)
seeds=[q,HOME]+[np.clip(q+rng.normal(0,0.6,7),LO+0.1,HI-0.1) for _ in range(10)]
for tilt in [40,35]:
  R=tilt_R(tilt)
  for tgt in [[-0.12,-0.44,1.022],[-0.12,-0.39,1.022],[-0.12,-0.346,1.022],[-0.12,-0.346,0.995],[-0.12,-0.44,1.15]]:
    found=[]
    for s in seeds:
        sol=r.ik(tgt,R,seed=s,timeout=1.5)
        if sol is not None: found.append(sol)
    best=max(found,key=lambda f: np.minimum(f-LO,HI-f).min()) if found else None
    print(tilt,tgt,'n sol',len(found), None if best is None else (np.round(best,2), round(float(np.minimum(best-LO,HI-best).min()),2)))
" 2>&1 | grep -v 'IK failed'


# openrua op 61
timeout 900 python3 -u -c "
from rob import *
r=Robot('iktest3')
q=r.arm_q()
rng=np.random.default_rng(2)
seeds=[q,HOME]+[np.clip(q+rng.normal(0,0.6,7),LO+0.1,HI-0.1) for _ in range(10)]
R=tilt_R(40)
for tgt in [[-0.09,-0.47,1.20],[-0.09,-0.47,1.17],[-0.09,-0.47,1.15],[-0.09,-0.47,1.10],[-0.09,-0.47,1.022],[-0.06,-0.47,1.20],[-0.09,-0.50,1.20],[-0.115,-0.40,1.022],[-0.12,-0.346,0.995]]:
    found=[]
    for s in seeds:
        sol=r.ik(tgt,R,seed=s,timeout=1.0)
        if sol is not None: found.append(sol)
    best=max(found,key=lambda f: np.minimum(f-LO,HI-f).min()) if found else None
    print(tgt,'n sol',len(found), None if best is None else (np.round(best,2), round(float(np.minimum(best-LO,HI-best).min()),2)))
" 2>&1 | grep -v 'IK failed'


# openrua op 62
timeout 900 python3 -u -c "
from rob import *
r=Robot('iktest4')
q=r.arm_q()
rng=np.random.default_rng(3)
seeds=[q,HOME]+[np.clip(q+rng.normal(0,0.6,7),LO+0.1,HI-0.1) for _ in range(8)]
def test(tgt,R,label):
    found=[]
    for s in seeds:
        sol=r.ik(tgt,R,seed=s,timeout=1.0)
        if sol is not None: found.append(sol)
    best=max(found,key=lambda f: np.minimum(f-LO,HI-f).min()) if found else None
    print(label,tgt,'n sol',len(found), None if best is None else (np.round(best,2), round(float(np.minimum(best-LO,HI-best).min()),2)))
for y in [-0.45,-0.46]:
    test([-0.09,y,1.19],tilt_R(40),'t40')
for t in [0,20,30]:
    test([-0.09,-0.47,1.19],tilt_R(t),f't{t}')
# hand rotated about z by 180 (camera on other side)
R2=R_from_axes([0,np.sin(np.radians(40)),-np.cos(np.radians(40))],[0,-np.cos(np.radians(40)),-np.sin(np.radians(40))])
test([-0.09,-0.47,1.19],R2,'t40 flipped')
test([-0.09,-0.44,1.19],R2,'t40 flipped')
" 2>&1 | grep -v 'IK failed'


# openrua op 63
timeout 600 python3 -u -c "
from rob import *
r=Robot('tilttest')
q=r.arm_q(); t,_=r.tcp(); print('tcp',np.round(t,3))
for tilt in [20,0]:
    q=goto(r,t,tilt_R(tilt),seconds=3.0,seeds=[q,HOME],near=q)
    print('fingers',np.round(r.finger_gap(),4),'wrench',np.round(wrench(r),1))
r.snap('frontview','/workspace/front10.png'); r.snap('agentview','/workspace/agent5.png')
"


# openrua op 64
timeout 1200 python3 -u -c "
from rob import *
r=Robot('iktest5')
q=r.arm_q()
rng=np.random.default_rng(4)
seeds=[q,HOME,[-0.85,1.19,0.06,-0.74,0.41,1.41,-1.64]]+[np.clip(q+rng.normal(0,0.6,7),LO+0.1,HI-0.1) for _ in range(5)]
R=tilt_R(40)
for z in [1.022,1.10]:
  for x in [-0.09,-0.12,-0.15,-0.18,-0.21]:
    row=[]
    for y in [-0.44,-0.45,-0.46,-0.47,-0.48]:
        n=0
        for s in seeds:
            if r.ik([x,y,z],R,seed=s,timeout=0.7) is not None: n+=1
        row.append(n)
    print('z',z,'x',x,'y -0.44..-0.48 nsol',row)
" 2>&1 | grep -v 'IK failed'


# openrua op 65
timeout 900 python3 -u -c "
from rob import *
r=Robot('carry2')
R=tilt_R(0); q=r.arm_q()
for w in [[-0.06,-0.22,1.28],[-0.10,-0.48,1.28],[-0.10,-0.48,1.15],[-0.10,-0.48,1.04]]:
    q=goto(r,w,R,seconds=4.0,seeds=[q,HOME],near=q)
    if q is None: break
    print('  fingers',np.round(r.finger_gap(),4),'wrench',np.round(wrench(r),1))
r.snap('frontview','/workspace/front11.png'); r.snap('agentview','/workspace/agent6.png'); r.snap('robot0_eye_in_hand','/workspace/eih15.png')
"


# openrua op 66
timeout 900 python3 -u -c "
from rob import *
r=Robot('push')
R=tilt_R(0); q=r.arm_q()
for x in [-0.125,-0.15,-0.175,-0.20]:
    q=goto(r,[x,-0.48,1.04],R,seconds=2.5,seeds=[q,HOME],near=q)
    if q is None: break
    w=wrench(r); print('  x',x,'wrench',np.round(w,1))
    if np.linalg.norm(w[:3]+[0,0,5.4])>25: print('  large force, stop'); break
r.snap('birdview','/workspace/bird3.png'); r.snap('robot0_eye_in_hand','/workspace/eih16.png')
"


# openrua op 67
for c in sideview galleryview frontview agentview; do timeout 120 python3 cloud.py $c > /dev/null 2>&1; done; timeout 60 python3 -c "
import numpy as np
for c in ['sideview','galleryview','frontview','agentview']:
    try:
        P=np.load(f'{c}_world.npy'); ok=np.load(f'{c}_ok.npy')
    except Exception as e: print(c,'fail',e); continue
    pts=P[ok]
    m=(pts[:,2]>0.95)&(pts[:,2]<1.12)&(pts[:,1]<-0.34)&(pts[:,1]>-0.65)&(pts[:,0]<-0.15)&(pts[:,0]>-0.40)
    d=pts[m]; print(c,'door-region pts',len(d))
    if len(d)>50:
        ys=np.round(d[:,1],2)
        for y in np.unique(ys)[::2]:
            s=d[ys==y]; print(f'   y {y:.2f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}')
"


# openrua op 68
timeout 900 python3 -u -c "
from rob import *
r=Robot('tilt2')
q=r.arm_q()
q=goto(r,[-0.15,-0.47,1.04],tilt_R(0),seconds=3.0,seeds=[q,HOME],near=q)
for t in [20,40]:
    q=goto(r,[-0.15,-0.47,1.04],tilt_R(t),seconds=3.0,seeds=[q,HOME],near=q)
    if q is None: break
    print('  fingers',np.round(r.finger_gap(),4),'wrench',np.round(wrench(r),1))
p,_,Rh=r.fk(); print('hand z',np.round(Rh[:,2],3),'hand y',np.round(Rh[:,1],3))
r.snap('frontview','/workspace/front12.png'); r.snap('agentview','/workspace/agent7.png')
"


# openrua op 69
timeout 900 python3 -u -c "
from rob import *
r=Robot('insert')
R=tilt_R(40); q=r.arm_q()
base=np.array([0,3.5,-4.1])
for w in [[-0.15,-0.47,1.018],[-0.15,-0.43,1.018],[-0.15,-0.39,1.018],[-0.15,-0.346,1.018]]:
    q=goto(r,w,R,seconds=2.5,seeds=[q,HOME],near=q)
    if q is None: break
    f=wrench(r); print('  wrench',np.round(f,1))
    if np.linalg.norm(f[:3]-base)>8: print('  contact! stop'); break
r.snap('frontview','/workspace/front13.png'); r.snap('agentview','/workspace/agent8.png'); r.snap('robot0_eye_in_hand','/workspace/eih17.png')
"


# openrua op 70
timeout 120 python3 -u -c "
from rob import *
r=Robot('snaps')
for c in ['sideview','galleryview','paperview']: r.snap(c,f'/workspace/{c}_2.png')
"

# openrua op 71
timeout 120 python3 cloud.py robot0_eye_in_hand > /dev/null 2>&1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
pts=P[ok]
for xr in [(-0.26,-0.21),(-0.09,-0.04)]:
    m=(pts[:,0]>xr[0])&(pts[:,0]<xr[1])&(pts[:,1]>-0.40)&(pts[:,1]<-0.20)&(pts[:,2]<1.0)
    d=pts[m]; print('x',xr,'n',len(d))
    ys=np.round(d[:,1],2)
    for y in np.unique(ys):
        s=d[ys==y]; print(f'   y {y:.2f} n={len(s)} z {s[:,2].min():.3f}..{s[:,2].max():.3f}')
# mug lowest visible points
m=(pts[:,0]>-0.21)&(pts[:,0]<-0.09)&(pts[:,1]>-0.36)&(pts[:,1]<-0.24)&(pts[:,2]<1.02)&(pts[:,2]>0.9)
d=pts[m]; print('mug region zmin',d[:,2].min() if len(d) else None)
zs=np.round(d[:,2],2)
for z in np.unique(zs): 
    s=d[zs==z]; print(f'   z {z:.2f} n={len(s)} y {s[:,1].min():.3f}..{s[:,1].max():.3f}')
"


# openrua op 72
timeout 900 python3 -u -c "
from rob import *
r=Robot('insert2')
R=tilt_R(40); q=r.arm_q()
base=np.array([0,3.5,-4.1])
q=goto(r,[-0.15,-0.395,1.032],R,seconds=2.5,seeds=[q,HOME],near=q); print('  wrench',np.round(wrench(r),1))
for w in [[-0.15,-0.37,1.032],[-0.15,-0.35,1.032]]:
    q=goto(r,w,R,seconds=2.5,seeds=[q,HOME],near=q)
    f=wrench(r); print('  wrench',np.round(f,1))
    if np.linalg.norm(f[:3]-base)>8: print('  contact! stop'); break
r.snap('robot0_eye_in_hand','/workspace/eih18.png')
"


# openrua op 73
timeout 900 python3 -u -c "
from rob import *
r=Robot('lower')
R=tilt_R(40); q=r.arm_q()
base=np.array([0,3.5,-4.1])
for z in [1.024,1.018,1.012,1.006,1.0]:
    q=goto(r,[-0.15,-0.35,z],R,seconds=2.0,seeds=[q,HOME],near=q)
    f=wrench(r); print('  z',z,'wrench',np.round(f,1))
    if f[2]>base[2]+3: print('  resting on floor'); break
g=r.gripper(0.04); print('gap',g)
print('wrench after release',np.round(wrench(r),1))
r.snap('robot0_eye_in_hand','/workspace/eih19.png')
"


# openrua op 74
timeout 900 python3 -u -c "
from rob import *
r=Robot('retreat')
R=tilt_R(40); q=r.arm_q()
for w in [[-0.15,-0.40,1.02],[-0.15,-0.46,1.03]]:
    q=goto(r,w,R,seconds=2.5,seeds=[q,HOME],near=q); print('  wrench',np.round(wrench(r),1))
r.snap('robot0_eye_in_hand','/workspace/eih20.png')
q=goto(r,[-0.15,-0.47,1.15],tilt_R(0),seconds=3.0,seeds=[q,HOME],near=q)
r.snap('frontview','/workspace/front14.png')
"


# openrua op 75
timeout 120 python3 cloud.py robot0_eye_in_hand > /dev/null 2>&1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
pts=P[ok]
m=(pts[:,0]>-0.25)&(pts[:,0]<-0.05)&(pts[:,1]>-0.45)&(pts[:,1]<-0.15)&(pts[:,2]>0.95)&(pts[:,2]<1.05)
d=pts[m]
ys=np.round(d[:,1],2)
for y in np.unique(ys):
    s=d[ys==y]; print(f' y {y:.2f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}')
# floor edge
m2=(pts[:,0]>-0.25)&(pts[:,0]<-0.05)&(pts[:,2]>0.935)&(pts[:,2]<0.95)
f=pts[m2]; print('floor y range',f[:,1].min(),f[:,1].max())
"


# openrua op 76
timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
pts=P[ok]
m=(pts[:,0]>-0.24)&(pts[:,0]<-0.06)&(pts[:,1]>-0.26)&(pts[:,1]<-0.10)&(pts[:,2]>0.90)&(pts[:,2]<1.12)
d=pts[m]; print('n',len(d))
ys=np.round(d[:,1],2)
for y in np.unique(ys):
    s=d[ys==y]; print(f' y {y:.2f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}')
"


# openrua op 77
timeout 60 python3 -c "
import numpy as np
for c in ['agentview','frontview','galleryview','paperview']:
    try:
        P=np.load(f'{c}_world.npy'); ok=np.load(f'{c}_ok.npy')
    except Exception as e: print(c,'no file'); continue
    pts=P[ok]
    m=(pts[:,0]>-0.24)&(pts[:,0]<-0.06)&(pts[:,1]>-0.30)&(pts[:,1]<-0.12)&(pts[:,2]>0.95)&(pts[:,2]<1.07)
    d=pts[m]; print(c,'cavity pts',len(d))
    if len(d):
        ys=np.round(d[:,1],2)
        for y in np.unique(ys):
            s=d[ys==y]; print(f'   y {y:.2f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}')
"


# openrua op 78
timeout 600 python3 -u -c "
from rob import *
r=Robot('peek')
q=r.arm_q()
Rp=R_from_axes([0,1,0],[0,0,1]); print(np.round(Rp,2))
for tgt in [[-0.15,-0.40,1.01],[-0.15,-0.42,1.01],[-0.15,-0.38,1.02]]:
    sol=best_ik(r,tgt,Rp,seeds=[q,HOME,[-0.85,1.19,0.06,-0.74,0.41,1.41,-1.64]],timeout=2.0)
    print(tgt, None if sol is None else np.round(sol,2))
    if sol is not None:
        q=goto(r,tgt,Rp,seconds=4.0,seeds=[q,HOME],near=q)
        r.snap('robot0_eye_in_hand','/workspace/eih21.png'); break
" 2>&1 | grep -v 'IK failed'


# openrua op 79
timeout 600 python3 -u -c "
from rob import *
r=Robot('peek2')
q=r.arm_q()
for ang in [30,20]:
    a=np.radians(ang)
    Rp=R_from_axes([0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)])
    for tgt in [[-0.15,-0.38,1.03],[-0.15,-0.40,1.02],[-0.15,-0.36,1.04]]:
        sol=best_ik(r,tgt,Rp,seeds=[q,HOME],timeout=2.0)
        print(ang,tgt, None if sol is None else np.round(sol,2))
        if sol is not None:
            q=goto(r,tgt,Rp,seconds=4.0,seeds=[q,HOME],near=q)
            r.snap('robot0_eye_in_hand',f'/workspace/eih_peek{ang}.png'); break
" 2>&1 | grep -v 'IK failed'


# openrua op 80
timeout 120 python3 cloud.py robot0_eye_in_hand > /dev/null 2>&1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
pts=P[ok]
m=(pts[:,0]>-0.24)&(pts[:,0]<-0.06)&(pts[:,1]>-0.30)&(pts[:,1]<-0.05)&(pts[:,2]>0.94)&(pts[:,2]<1.09)
d=pts[m]; print('n',len(d))
ys=np.round(d[:,1],2)
for y in np.unique(ys):
    s=d[ys==y]; print(f' y {y:.2f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}')
# ceiling: points z>1.06 inside
m2=(pts[:,0]>-0.24)&(pts[:,0]<-0.06)&(pts[:,1]>-0.32)&(pts[:,1]<-0.15)&(pts[:,2]>1.05)&(pts[:,2]<1.12)
c=pts[m2]; print('ceiling-ish z values', np.unique(np.round(c[:,2],3))[:20] if len(c) else None)
"


# openrua op 81
timeout 900 python3 -u -c "
from rob import *
r=Robot('pushprep')
q=r.arm_q()
for t in [60,50]:
    R=tilt_R(t); ok=True
    for tgt in [[-0.15,-0.42,1.0],[-0.15,-0.37,1.0],[-0.15,-0.317,1.0]]:
        sol=best_ik(r,tgt,R,seeds=[q,HOME,[-0.85,1.19,0.06,-0.74,0.41,1.41,-1.64]],timeout=2.0)
        print(t,tgt, None if sol is None else np.round(sol,2)); ok = ok and sol is not None
    if ok: print('tilt',t,'all reachable'); break
" 2>&1 | grep -v 'IK failed'


# openrua op 82
timeout 900 python3 -u -c "
from rob import *
r=Robot('pushmug')
q=r.arm_q()
g=r.gripper(0.0); print('gap',g)
R=tilt_R(50)
q=goto(r,[-0.15,-0.42,1.0],R,seconds=4.0,seeds=[q,HOME],near=q)
base=wrench(r); print('base wrench',np.round(base,1))
for y in [-0.39,-0.37,-0.355,-0.34,-0.325,-0.317]:
    q=goto(r,[-0.15,y,1.0],R,seconds=2.0,seeds=[q,HOME],near=q)
    f=wrench(r); print('  y',y,'wrench',np.round(f,1))
    if np.linalg.norm(f[:3]-base[:3])>15: print('  large force, stop'); break
r.snap('robot0_eye_in_hand','/workspace/eih22.png')
"


# openrua op 83
timeout 900 python3 -u -c "
from rob import *
r=Robot('measure')
q=r.arm_q()
q=goto(r,[-0.15,-0.42,1.0],tilt_R(50),seconds=3.0,seeds=[q,HOME],near=q)
a=np.radians(20); Rp=R_from_axes([0,np.cos(a),-np.sin(a)],[0,np.sin(a),np.cos(a)])
q=goto(r,[-0.15,-0.38,1.03],Rp,seconds=3.0,seeds=[q,HOME],near=q)
r.snap('robot0_eye_in_hand','/workspace/eih23.png')
" && timeout 120 python3 cloud.py robot0_eye_in_hand > /dev/null 2>&1; timeout 60 python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); ok=np.load('robot0_eye_in_hand_ok.npy')
pts=P[ok]
m=(pts[:,0]>-0.24)&(pts[:,0]<-0.06)&(pts[:,1]>-0.40)&(pts[:,1]<-0.10)&(pts[:,2]>0.95)&(pts[:,2]<1.06)
d=pts[m]
ys=np.round(d[:,1],2)
for y in np.unique(ys):
    s=d[ys==y]; print(f' y {y:.2f} n={len(s)} x {s[:,0].min():.3f}..{s[:,0].max():.3f} z {s[:,2].min():.3f}..{s[:,2].max():.3f}')
"


# openrua op 84
timeout 1200 python3 -u -c "
from rob import *
r=Robot('door')
H=np.array([-0.30,-0.345]); rr=0.18; off=0.035
def pose(phi_deg,z):
    p=np.radians(phi_deg); d=np.array([np.cos(p),np.sin(p)]); n=np.array([-np.sin(p),np.cos(p)])
    xy=H+rr*d-off*n
    R=R_from_axes([0,0,-1],[n[0],n[1],0])
    return [xy[0],xy[1],z],R
q=r.arm_q()
q=goto(r,[-0.15,-0.38,1.25],tilt_R(0),seconds=3.0,seeds=[q,HOME],near=q)
pos,R=pose(-100,1.25); print('start',np.round(pos,3))
q=goto(r,pos,R,seconds=4.0,seeds=[q,HOME])
pos,R=pose(-100,1.07)
q=goto(r,pos,R,seconds=3.0,seeds=[q,HOME],near=q)
base=wrench(r); print('base',np.round(base,1))
r.snap('birdview','/workspace/bird4.png')
for phi in range(-90,1,10):
    pos,R=pose(phi,1.07)
    q=goto(r,pos,R,seconds=1.5,seeds=[q,HOME],near=q)
    if q is None: print('IK fail at',phi); break
    f=wrench(r); print('  phi',phi,'wrench',np.round(f,1))
    if np.linalg.norm(f[:3]-base[:3])>20: print('  large force, stop'); break
r.snap('birdview','/workspace/bird5.png'); r.snap('agentview','/workspace/agent9.png')
"


# openrua op 85
timeout 120 python3 cloud.py agentview > /dev/null 2>&1; timeout 120 python3 cloud.py birdview > /dev/null 2>&1; python3 - <<'EOF'
import numpy as np
for cam in ['agentview','birdview']:
    P=np.load(f'{cam}_world.npy'); ok=np.load(f'{cam}_ok.npy')
    P=P[ok]
    # region in front of the microwave: y in [-0.6,-0.33], x in [-0.4,0.1], z in [0.92,1.15]
    m=(P[:,0]>-0.4)&(P[:,0]<0.1)&(P[:,1]<-0.30)&(P[:,1]>-0.65)&(P[:,2]>0.93)&(P[:,2]<1.15)
    Q=P[m]
    print(cam, 'pts in front region', len(Q))
    if len(Q):
        # histogram over y
        ys=np.round(Q[:,1],2)
        for yv in np.unique(ys):
            sel=Q[ys==yv]
            print(f'  y={yv:+.2f} n={len(sel):4d} x[{sel[:,0].min():+.3f},{sel[:,0].max():+.3f}] z[{sel[:,2].min():.3f},{sel[:,2].max():.3f}]')
EOF

# openrua op 86
python3 - <<'EOF'
import numpy as np
for cam in ['agentview','birdview']:
    P=np.load(f'{cam}_world.npy'); ok=np.load(f'{cam}_ok.npy'); P=P[ok]
    m=(P[:,0]>-0.4)&(P[:,0]<0.1)&(P[:,1]<-0.355)&(P[:,1]>-0.65)&(P[:,2]>0.93)&(P[:,2]<1.06)
    Q=P[m]; print(cam,len(Q))
    xs=np.round(Q[:,0],2)
    for xv in np.unique(xs):
        s=Q[xs==xv]; print(f'  x={xv:+.2f} n={len(s):3d} y[{s[:,1].min():+.3f},{s[:,1].max():+.3f}] z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
    # front face: points y in [-0.36,-0.33], x bins
    m=(P[:,1]<-0.33)&(P[:,1]>-0.36)&(P[:,2]>0.93)&(P[:,2]<1.10)&(P[:,0]>-0.3)&(P[:,0]<0.08)
    Q=P[m]; xs=np.round(Q[:,0],2)
    print(' front face y mean per x bin:')
    for xv in np.unique(xs):
        s=Q[xs==xv]; print(f'  x={xv:+.2f} n={len(s):3d} ymean={s[:,1].mean():+.4f} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
EOF

# openrua op 87
cat > push2.py <<'EOF'
import numpy as np, rclpy
from rob import *
rclpy.init(); r=Robot()
print('tcp', r.tcp()[0], 'gap', r.finger_gap())
R=R_from_axes([0,0,-1],[0,1,0])
w0=wrench(r,3); print('w0',w0[:3])
goto(r,[-0.08,-0.40,1.071],R,seconds=3)
y=-0.40
while y>-0.336:
    y=round(y+0.005,3)
    ok=goto(r,[-0.08,y,1.071],R,seconds=1.5)
    w=wrench(r,3); dev=w[:3]-w0[:3]
    print(f'y={y:+.3f} tcp={np.round(r.tcp()[0],3)} dev={np.round(dev,2)}')
    if np.linalg.norm(dev)>6: print('contact force, stop'); break
print('final tcp', r.tcp()[0])
EOF
timeout 300 python3 -u push2.py 2>&1 | grep -v '^\s*$' | tail -40

# openrua op 88
sed -i 's/while y>-0.336:/while y<-0.336:/' push2.py && timeout 400 python3 -u push2.py 2>&1 | grep -E '^(y=|final|contact|w0|tcp)' 

# openrua op 89
cat > retreat.py <<'EOF'
import numpy as np, rclpy
from rob import *
rclpy.init(); r=Robot()
R=R_from_axes([0,0,-1],[0,1,0])
goto(r,[-0.08,-0.45,1.071],R,seconds=2)
goto(r,[-0.10,-0.50,1.30],R,seconds=3)
for c in ['sideview','agentview','birdview','galleryview']:
    try: r.snap(c,f'/workspace/{c}_final.png'); print('saved',c)
    except Exception as e: print('snap fail',c,e)
print('tcp',r.tcp()[0])
EOF
timeout 300 python3 -u retreat.py 2>&1 | grep -E '^(goto|saved|snap|tcp)'

# openrua op 90
cat > look.py <<'EOF'
import numpy as np, rclpy, sys
from rob import *
rclpy.init(); r=Robot()
cands=[([-0.12,-0.62,1.05],R_from_axes([0,1,0],[0,0,-1])),
       ([-0.12,-0.60,1.10],R_from_axes([0,1,0],[0,0,-1])),
       ([-0.12,-0.58,1.20],R_from_axes([0,np.cos(np.radians(25)),-np.sin(np.radians(25))],[0,np.sin(np.radians(25)),np.cos(np.radians(25))])),
       ([-0.12,-0.55,1.30],R_from_axes([0,np.cos(np.radians(40)),-np.sin(np.radians(40))],[0,np.sin(np.radians(40)),np.cos(np.radians(40))]))]
for p,R in cands:
    q=best_ik(r,p,R,at_tcp=False)
    print('cand',p,'ik',q is not None)
    if q is not None:
        ok=goto(r,p,R,at_tcp=False,seconds=4); print('ok',ok); break
r.snap('eih','/workspace/eih_door.png'); print('tcp',r.tcp()[0])
EOF
timeout 300 python3 -u look.py 2>&1 | grep -E '^(cand|ok|tcp|goto)'

# openrua op 91
cat > look.py <<'EOF'
import numpy as np, rclpy, sys, itertools
from rob import *
rclpy.init(); r=Robot()
def Rtilt(t,flip):
    t=np.radians(t); z=[0,np.cos(t),-np.sin(t)]
    x=[0,np.sin(t),np.cos(t)] if not flip else [0,-np.sin(t),-np.cos(t)]
    return R_from_axes(z,x)
found=None
for t,flip,x,y,z in itertools.product([0,25,45],[False,True],[-0.30,-0.20,-0.12],[-0.62,-0.55,-0.50],[1.05,1.20,1.35]):
    R=Rtilt(t,flip); p=[x,y,z]
    q=best_ik(r,p,R,at_tcp=False,timeout=1.5)
    if q is not None:
        print('OK',t,flip,p); found=(p,R)
        if y<=-0.55: break
if found:
    p,R=found; ok=goto(r,p,R,at_tcp=False,seconds=4); print('goto ok',ok)
    r.snap('eih','/workspace/eih_door.png'); print('tcp',r.tcp()[0])
EOF
timeout 600 python3 -u look.py 2>&1 | grep -E '^(OK|goto|tcp)'

# openrua op 92
timeout 200 python3 -u look.py 2>&1 | tail -5; ls -la eih_door.png

# openrua op 93
timeout 200 python3 -u look.py 2>&1 | grep -v "^\s*$" | grep -iE "error|exception|Traceback|snap|tcp|goto" | head; grep -n "def snap" -A 12 rob.py

# openrua op 94
timeout 120 python3 -c "
import rclpy; from rob import *
rclpy.init(); r=Robot(); print(r.snap('eih','/workspace/eih_door.png')); print(r.tcp()[0])
" 2>&1 | tail -2

# openrua op 95
timeout 60 ros2 topic list | grep -i image; timeout 30 ros2 topic hz /eih/color/image_raw --window 5 2>&1 | head -5

# openrua op 96
timeout 120 python3 -c "
import rclpy; from rob import *
rclpy.init(); r=Robot(); print(r.snap('robot0_eye_in_hand','/workspace/eih_door.png')); print(r.snap('frontview','/workspace/front_final.png')); print(r.tcp()[0])
" 2>&1 | tail -3

# openrua op 97
python3 - <<'EOF'
import numpy as np
P=np.load('agentview_world.npy'); ok=np.load('agentview_ok.npy'); P=P[ok]
# +x face: x in [0.05,0.09], z 0.95..1.09
m=(P[:,0]>0.055)&(P[:,0]<0.09)&(P[:,2]>0.95)&(P[:,2]<1.09)&(P[:,1]>-0.36)&(P[:,1]<-0.10)
Q=P[m]; ys=np.round(Q[:,1],2)
print('+x face x-mean per y bin:')
for yv in np.unique(ys):
    s=Q[ys==yv]; print(f'  y={yv:+.2f} n={len(s):4d} xmean={s[:,0].mean():+.4f}')
# top face edge at -y: for z>1.10 points, min y per x bin
m=(P[:,2]>1.100)&(P[:,2]<1.115)&(P[:,0]>-0.3)&(P[:,0]<0.08)&(P[:,1]>-0.4)&(P[:,1]<-0.1)
Q=P[m]; xs=np.round(Q[:,0],2)
print('top face: y min/max per x bin')
for xv in np.unique(xs):
    s=Q[xs==xv]; print(f'  x={xv:+.2f} n={len(s):4d} ymin={s[:,1].min():+.4f} ymax={s[:,1].max():+.4f}')
EOF

# openrua op 98
timeout 200 python3 -u - <<'EOF' 2>&1 | grep -E '^(goto|home|final|w)'
import rclpy, numpy as np
from rob import *
rclpy.init(); r=Robot()
R=R_from_axes([0,0,-1],[0,1,0])
goto(r,[-0.30,-0.40,1.40],R,seconds=3)
code,err=goto_q(r,HOME,seconds=4); print('home',code,err)
p,_=r.tcp(); print('final tcp',np.round(p,3),'gap',r.finger_gap()); print('w',np.round(wrench(r,3)[:3],2))
EOF

# openrua op 99
timeout 200 python3 -u - <<'EOF' 2>&1 | grep -vE '^\s*$' | tail -8
import rclpy, numpy as np
from rob import *
rclpy.init(); r=Robot()
print('q',np.round(r.arm_q(),3)); print('tcp',np.round(r.tcp()[0],3))
res=goto_q(r,HOME,seconds=4); print('home',res)
print('tcp',np.round(r.tcp()[0],3),'w',np.round(wrench(r,3)[:3],2))
EOF

# openrua op 100
timeout 120 python3 -c "
import rclpy; from rob import *
rclpy.init(); r=Robot(); print(r.snap('frontview','/workspace/front_done.png'))" 2>&1 | tail -1
