#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/action/fjt_send.py tools/action/ik_move.py tools/action/gripper_cmd.py tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
ros2 topic list 2>&1; echo "--- actions"; ros2 action list 2>&1; echo "--- services"; ros2 service list 2>&1 | grep -v -E "parameter|describe_|get_type" ; echo "--- nodes"; ros2 node list 2>&1

# openrua op 3
ros2 topic echo /joint_states --once 2>&1 | head -40; echo "--- pipeline_state"; ros2 topic info /pipeline_state; timeout 10 ros2 topic echo /pipeline_state --once 2>&1 | head -30

# openrua op 4
for c in agentview frontview birdview sideview robot0_robotview robot0_eye_in_hand galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; echo ---; timeout 20 ros2 run tf2_ros tf2_echo world birdview_optical_frame 2>&1 | head -12; echo ---; ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id" | sort | uniq

# openrua op 6
cd /workspace; for p in "375 285" "378 237" "250 285" "300 290" "320 300" "200 350" "330 240"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p 2>&1 | tail -1; done

# openrua op 7
mkdir -p "$(dirname /workspace/scan.py)"
cat > /workspace/scan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Top-down scene scan from a camera: world-frame point cloud -> objects.

Usage: python3 scan.py <camera> [zmin=0.905]
Prints connected blobs of points above the table (world z > zmin) with
their centroid, bbox and max height. Also saves <camera>_pts.npy (HxWx3).
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from scipy import ndimage


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    while "m" not in got and node.get_clock().now().nanoseconds / 1e9 < end:
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
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.905
    rclpy.init()
    node = rclpy.create_node("scan")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    H, W = depth.height, depth.width
    d = np.frombuffer(depth.data, dtype=np.float32).reshape(H, W)
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
    pw = pc @ R.T + T
    np.save(f"{cam}_pts.npy", pw)
    ok = np.isfinite(d) & (d > 0)
    above = ok & (pw[..., 2] > zmin)
    lab, n = ndimage.label(above)
    print(f"{cam}: {n} blobs above z={zmin}")
    for i in range(1, n + 1):
        m = lab == i
        if m.sum() < 15:
            continue
        P = pw[m]
        vs, us = np.where(m)
        print(f" blob{i}: n={m.sum():5d} px(u{us.min()}-{us.max()},v{vs.min()}-{vs.max()}) "
              f"centroid=({P[:,0].mean():.3f},{P[:,1].mean():.3f}) "
              f"x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] "
              f"z[{P[:,2].min():.3f},{P[:,2].max():.3f}]")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 scan.py birdview 0.905

# openrua op 9
timeout 120 python3 tools/action/gripper_cmd.py 0.04; ros2 topic echo /joint_states --once 2>&1 | grep -A9 position

# openrua op 10
timeout 120 python3 scan.py birdview 0.905; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 11
python3 -c "
import numpy as np
pw=np.load('birdview_pts.npy')
z=pw[...,2]
m=np.isfinite(z)&(z>0.905)&(pw[...,1]<0.05)&(pw[...,1]>-0.4)&(pw[...,0]>-0.15)&(pw[...,0]<0.1)
P=pw[m]
# height histogram
for lo,hi in [(0.905,0.915),(0.915,0.925),(0.925,0.935),(0.935,0.945),(0.945,0.96),(0.96,1.1)]:
    s=P[(P[:,2]>=lo)&(P[:,2]<hi)]
    if len(s): print(f'z{lo}-{hi}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] c=({s[:,0].mean():.3f},{s[:,1].mean():.3f})')
# pan rim: highest ring; find pan body circle: points with y<-0.12
body=P[P[:,1]<-0.13]
print('body x',body[:,0].min(),body[:,0].max(),'y',body[:,1].min(),body[:,1].max(), 'center',(body[:,0].min()+body[:,0].max())/2,(body[:,1].min()+body[:,1].max())/2)
h=P[(P[:,1]>-0.13)&(P[:,1]<0.02)&(P[:,2]<0.99)]
print('handle x',h[:,0].min(),h[:,0].max(),'y',h[:,1].min(),h[:,1].max(),'z',h[:,2].min(),h[:,2].max(),'cx',h[:,0].mean())
"


# openrua op 12
python3 -c "
import cv2
im=cv2.imread('birdview.png'); c=im[215:265,350:400]; cv2.imwrite('knob_bird.png', cv2.resize(c,None,fx=8,fy=8,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('robot0_robotview.png'); c=im[90:180,440:560]; cv2.imwrite('knob_robot.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))
"

# openrua op 13
ros2 service type /get_urdf; ros2 interface show $(ros2 service type /get_urdf) 2>&1 | head; ros2 topic info /robot_description -v 2>&1 | head -20

# openrua op 14
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable helpers: joints, FK, IK, trajectory, servo, gripper, wrench."""
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
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
GR = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = FJT["joints"]
BASE_IN_WORLD = np.array([-0.660, 0.000, 0.912])  # from tf2_echo world panda_link0


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


def topdown_quat(yaw):
    """Hand Z pointing down (world -Z), hand X rotated by yaw about world Z.
    yaw=0 -> hand X along world +X, fingers close along world Y."""
    # (1,0,0,0) = 180deg about X: Z->-Z, Y->-Y. Then yaw about world Z.
    qz = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    return quat_mul(qz, (1.0, 0.0, 0.0, 0.0))


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GR["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik_cli.wait_for_service(10); self.fk_cli.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.05): rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
            while self._js is None: self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.joints(); return [j[n] for n in ARM]

    def fingers(self):
        j = self.joints(); return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def wrench(self):
        self._wr = None
        while self._wr is None: self.spin(0.2)
        f, t = self._wr.wrench.force, self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk(self, q=None, link="panda_hand", world=True):
        """Pose of link. Returns (pos, quat xyzw). world=True adds base offset."""
        q = q if q is not None else self.arm_q()
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {None if r is None else r.error_code.val}")
        p = r.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        if world: pos = pos + BASE_IN_WORLD
        return pos, (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def ik(self, pos_world, quat, seed=None, timeout=60):
        """IK for hand pose in world frame. Returns joint list or None."""
        p = np.asarray(pos_world, float) - BASE_IN_WORLD
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, quat)
        seed = seed if seed is not None else self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            print(f"IK failed: {None if r is None else r.error_code.val}")
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def move_q(self, q, seconds=3.0, waypoints=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = list(waypoints or []) + [(q, seconds)]
        for qq, t in wps:
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_pose(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik(pos_world, quat, seed)
        if q is None: return None
        self.move_q(q, seconds)
        p, _ = self.fk()
        print(f"  reached hand pos {p.round(4)} target {np.asarray(pos_world).round(4)}")
        return q

    def servo(self, v_world, n=20, dt=0.05, w=(0, 0, 0)):
        """Stream twist (linear in base frame == world frame orientation)."""
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v_world)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, w)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg); self.spin(dt)

    def gripper(self, width):
        g = GripperCommand.Goal()
        g.command.position = float(width); g.command.max_effort = float(GR["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        f = self.fingers()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def close(self):
        self.node.destroy_node(); rclpy.shutdown()
OPENRUA_EOF

# openrua op 15
timeout 120 python3 -c "
from rob import *
r=Robot('probe')
print('q', np.round(r.arm_q(),3))
p,q=r.fk(); print('hand world', p.round(4), 'quat', np.round(q,4))
p8,q8=r.fk(link='panda_link8'); print('link8 world', p8.round(4), np.round(q8,4))
R=quat_R(*q); print('hand axes X',R[:,0].round(3),'Y',R[:,1].round(3),'Z',R[:,2].round(3))
print('wrench', r.wrench())
r.close()
"

# openrua op 16
timeout 30 ros2 run tf2_ros tf2_echo panda_link0 panda_hand 2>&1 | grep -A1 Translation | head -3; timeout 120 python3 -c "
from rob import *
r=Robot('probe')
req = GetPositionFK.Request(); req.fk_link_names=['panda_hand']
req.robot_state.joint_state.name=list(ARM); req.robot_state.joint_state.position=r.arm_q()
fut=r.fk_cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=30)
print(fut.result().pose_stamped[0])
r.close()
"

# openrua op 17
timeout 200 python3 -c "
from rob import *
r=Robot('probe')
q0=np.array(r.arm_q())
pw=np.array([-0.203,0.0,1.2696]); quat=(0.9996,0,-0.0284,0)
for label,p in [('world', pw), ('base', pw-BASE_IN_WORLD)]:
    req = GetPositionIK.Request()
    req.ik_request.group_name='panda_arm'; req.ik_request.pose_stamped.header.frame_id=''
    ps=req.ik_request.pose_stamped.pose
    ps.position.x,ps.position.y,ps.position.z=map(float,p)
    ps.orientation.x,ps.orientation.y,ps.orientation.z,ps.orientation.w=map(float,quat)
    req.ik_request.robot_state.joint_state.name=list(ARM); req.ik_request.robot_state.joint_state.position=list(map(float,q0))
    req.ik_request.timeout.sec=2
    fut=r.ik_cli.call_async(req); rclpy.spin_until_future_complete(r.node, fut, timeout_sec=60)
    res=fut.result()
    print(label, 'code', res.error_code.val)
    if res.error_code.val==1:
        sol=dict(zip(res.solution.joint_state.name,res.solution.joint_state.position)); qs=np.array([sol[j] for j in ARM])
        print('  sol', qs.round(3), 'diff from current', np.abs(qs-q0).max().round(4))
r.close()
"

# openrua op 18
timeout 200 python3 -c "
from rob import *
r=Robot('probe')
BASE_IN_WORLD[:]=0
for q in [[0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009],[0,-0.161,0,-2.445,0,2.227,0.785]]:
    p,qu=r.fk(q); print(p.round(4), np.round(qu,4))
r.close()
"

# openrua op 19
python3 - <<'EOF'
import re
s=open('rob.py').read()
s=s.replace('BASE_IN_WORLD = np.array([-0.660, 0.000, 0.912])  # from tf2_echo world panda_link0',
 'BASE_IN_WORLD = np.zeros(3)  # FK/IK on this machine already report/accept WORLD-frame poses (verified vs TF)')
s=s.replace('        req.ik_request.pose_stamped.header.frame_id = ""\n',
 '        req.ik_request.pose_stamped.header.frame_id = ""\n        req.ik_request.ik_link_name = "panda_hand"  # default tip is panda_link8 (45deg off)\n')
open('rob.py','w').write(s)
EOF
timeout 200 python3 -c "
from rob import *
r=Robot('probe')
q0=r.arm_q()
sol=r.ik([-0.203,0.0,1.2696],(0.9996,0,-0.0284,0))
print('sol',np.round(sol,3)); print('fk of sol', r.fk(sol))
r.close()
"

# openrua op 20
python3 -c "
import numpy as np
pw=np.load('birdview_pts.npy'); z=pw[...,2]
m=np.isfinite(z)&(z>0.905)&(pw[...,1]>0.1)&(pw[...,1]<0.3)&(pw[...,0]>-0.3)&(pw[...,0]<-0.15)
P=pw[m]
for lo,hi in [(0.905,0.915),(0.915,0.925),(0.925,0.935),(0.935,0.945),(0.945,0.955),(0.955,0.97)]:
    s=P[(P[:,2]>=lo)&(P[:,2]<hi)]
    if len(s): print(f'z{lo}-{hi}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] c=({s[:,0].mean():.3f},{s[:,1].mean():.3f})')
print('--- stove')
m=np.isfinite(z)&(z>0.905)&(pw[...,1]>0.08)&(pw[...,1]<0.32)&(pw[...,0]>-0.15)&(pw[...,0]<0.06)
P=pw[m]
for lo,hi in [(0.905,0.915),(0.915,0.925),(0.925,0.928),(0.928,0.935),(0.935,0.95)]:
    s=P[(P[:,2]>=lo)&(P[:,2]<hi)]
    if len(s): print(f'z{lo}-{hi}: n={len(s)} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}] c=({s[:,0].mean():.3f},{s[:,1].mean():.3f})')
"


# openrua op 21
mkdir -p "$(dirname /workspace/knob.py)"
cat > /workspace/knob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Step 1: grasp the stove knob fin and rotate it."""
import sys
import math
import numpy as np
from rob import *

KNOB = np.array([-0.200, 0.198])
TCP = M["hand"]["tcp_offset_m"]
direction = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
angle = float(sys.argv[2]) if len(sys.argv) > 2 else math.pi / 2

r = Robot("knob")
print("start q", np.round(r.arm_q(), 3), flush=True)
r.gripper(0.04)

q_top = topdown_quat(0.0)  # fingers close along world Y (fin runs along X)
hover = [KNOB[0], KNOB[1], 0.935 + TCP + 0.12]
print("hover", hover, flush=True)
q1 = r.move_pose(hover, q_top, 4.0)
if q1 is None:
    raise SystemExit("IK hover failed")

grasp = [KNOB[0], KNOB[1], 0.935 + TCP]
q2 = r.move_pose(grasp, q_top, 2.5, seed=q1)
if q2 is None:
    raise SystemExit("IK grasp failed")
p, qu = r.fk()
print("hand at", p.round(4), "fingertip z", round(p[2] - TCP, 4), flush=True)
print("wrench", r.wrench(), flush=True)

f = r.gripper(0.0)
print("fingers after close", f, flush=True)

# rotate about the vertical hand axis using joint7 only
q = r.arm_q()
q_rot = list(q)
q_rot[6] = q[6] + direction * angle
lim = FJT["limits_rad"][6]
if not (lim[0] < q_rot[6] < lim[1]):
    raise SystemExit(f"joint7 target {q_rot[6]} out of limits {lim}")
print("rotating joint7 from", round(q[6], 3), "to", round(q_rot[6], 3), flush=True)
r.move_q(q_rot, 3.0)
print("q after rot", np.round(r.arm_q(), 3), flush=True)
print("wrench", r.wrench(), flush=True)

r.gripper(0.04)
q = r.arm_q()
up = list(q)
r.move_pose(hover, topdown_quat(direction * angle), 2.5, seed=q)
print("done", flush=True)
r.close()
OPENRUA_EOF

# openrua op 22
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/knob.py", "old_string": "q = r.arm_q()\\nup = list(q)\\nr.move_pose(hover, topdown_quat(direction * angle), 2.5, seed=q)", "new_string": "q = r.arm_q()\\n_, qu_now = r.fk(q)\\nr.move_pose(hover, qu_now, 2.5, seed=q)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 23
nohup python3 -u knob.py 1 1.5708 > knob.log 2>&1 &
sleep 45; cat /workspace/knob.log

# openrua op 24
sleep 30; cat /workspace/knob.log; echo; pgrep -f knob.py || echo "finished"

# openrua op 25
pgrep -af knob.py || echo finished; ros2 topic echo /joint_states --once 2>&1 | grep -A9 position

# openrua op 26
mkdir -p "$(dirname /workspace/knob2.py)"
cat > /workspace/knob2.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Continue: gripper already closed on the fin; rotate joint7 by given delta, release, lift."""
import sys
import numpy as np
from rob import *

delta = float(sys.argv[1])
TCP = M["hand"]["tcp_offset_m"]
r = Robot("knob2")
q = r.arm_q()
print("q", np.round(q, 3), "fingers", r.fingers(), flush=True)
q_rot = list(q); q_rot[6] = q[6] + delta
lim = FJT["limits_rad"][6]
assert lim[0] < q_rot[6] < lim[1], q_rot[6]
print("rotating joint7", round(q[6], 3), "->", round(q_rot[6], 3), flush=True)
r.move_q(q_rot, 3.0)
print("q after", np.round(r.arm_q(), 3), "fingers", r.fingers(), flush=True)
print("wrench", r.wrench(), flush=True)
if "--hold" not in sys.argv:
    r.gripper(0.04)
    q = r.arm_q()
    p, qu = r.fk(q)
    r.move_pose([p[0], p[1], p[2] + 0.12], qu, 2.5, seed=q)
print("done", flush=True)
r.close()
OPENRUA_EOF

# openrua op 27
nohup python3 -u knob2.py -1.5708 --hold > knob2.log 2>&1 &
sleep 40; cat /workspace/knob2.log; pgrep -f "python3 -u knob2" || echo finished

# openrua op 28
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py robot0_robotview; python3 -c "
import cv2
im=cv2.imread('agentview.png'); c=im[200:360,340:520]; cv2.imwrite('stove_zoom.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 29
timeout 120 python3 scan.py birdview 0.905 > /dev/null; python3 -c "
import numpy as np
pw=np.load('birdview_pts.npy'); z=pw[...,2]
for y0 in np.arange(-0.14,0.02,0.01):
    m=np.isfinite(z)&(z>0.907)&(pw[...,1]>=y0)&(pw[...,1]<y0+0.01)&(pw[...,0]>-0.09)&(pw[...,0]<0.0)
    P=pw[m]
    if len(P): print(f'y[{y0:.2f},{y0+0.01:.2f}] n={len(P)} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] xc={P[:,0].mean():.3f} z[{P[:,2].min():.3f},{P[:,2].max():.3f}]')
"


# openrua op 30
python3 -c "
import numpy as np
pw=np.load('birdview_pts.npy'); z=pw[...,2]
for y0 in np.arange(-0.13,0.04,0.01):
    m=np.isfinite(z)&(z>0.907)&(pw[...,1]>=y0)&(pw[...,1]<y0+0.01)&(pw[...,0]>-0.02)&(pw[...,0]<0.1)
    P=pw[m]
    if len(P): print(f'y[{y0:.2f},{y0+0.01:.2f}] n={len(P)} x[{P[:,0].min():.3f},{P[:,0].max():.3f}] z[{P[:,2].min():.3f},{P[:,2].max():.3f}]')
"


# openrua op 31
python3 -c "
import numpy as np
pw=np.load('birdview_pts.npy'); z=pw[...,2]
cx,cy=-0.036,-0.237
rr=np.hypot(pw[...,0]-cx,pw[...,1]-cy)
for r0 in np.arange(0,0.12,0.01):
    m=np.isfinite(z)&(rr>=r0)&(rr<r0+0.01)&(pw[...,1]<-0.15)
    P=pw[m]
    if len(P): print(f'r[{r0:.2f},{r0+0.01:.2f}] n={len(P)} z[{P[:,2].min():.3f},{P[:,2].max():.3f}] zmed={np.median(P[:,2]):.3f}')
# -y side rim profile
m=np.isfinite(z)&(z>0.907)&(np.abs(pw[...,0]-cx)<0.01)&(pw[...,1]<-0.28)
P=pw[m]
for y0 in np.arange(-0.35,-0.28,0.005):
    s=P[(P[:,1]>=y0)&(P[:,1]<y0+0.005)]
    if len(s): print(f'y[{y0:.3f}] n={len(s)} z[{s[:,2].min():.3f},{s[:,2].max():.3f}]')
"


# openrua op 32
timeout 300 python3 -u -c "
from rob import *
r=Robot('rel')
r.gripper(0.04)
q=r.arm_q(); p,qu=r.fk(q)
r.move_pose([p[0],p[1],p[2]+0.15], qu, 2.5, seed=q)
r.close()
" 2>&1; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; python3 -c "
import cv2
im=cv2.imread('agentview.png'); c=im[200:360,340:520]; cv2.imwrite('stove_zoom.png', cv2.resize(c,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))"

# openrua op 33
python3 - <<'EOF'
s=open('rob.py').read()
s=s.replace("from sensor_msgs.msg import JointState\n","from sensor_msgs.msg import CameraInfo, Image, JointState\nfrom tf2_ros import Buffer, TransformListener\n")
s=s.replace("        self.twist_pub = self.node.create_publisher(TwistStamped, TW[\"port\"], 10)\n",
"""        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.tfbuf = Buffer(); self.tfl = TransformListener(self.tfbuf, self.node)
""")
s=s.replace("    def close(self):",
'''    def grab(self, topic, msg_type, timeout=30.0):
        got = {}
        sub = self.node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
        end = time.time() + timeout
        while "m" not in got and time.time() < end:
            self.spin(0.2)
        self.node.destroy_subscription(sub)
        if "m" not in got:
            raise RuntimeError(f"no message on {topic}")
        return got["m"]

    def cloud(self, cam="birdview"):
        """World-frame HxWx3 point cloud from a depth camera (NaN where invalid)."""
        depth = self.grab(f"/{cam}/depth/image_raw", Image)
        info = self.grab(f"/{cam}/color/camera_info", CameraInfo)
        H, W = depth.height, depth.width
        d = np.frombuffer(depth.data, dtype=np.float32).reshape(H, W).copy()
        d[~(np.isfinite(d) & (d > 0))] = np.nan
        fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
        frame = f"{cam}_optical_frame"
        end = time.time() + 10
        while time.time() < end and not self.tfbuf.can_transform("world", frame, rclpy.time.Time()):
            self.spin(0.2)
        t = self.tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        R = quat_R(q.x, q.y, q.z, q.w)
        T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        u, v = np.meshgrid(np.arange(W), np.arange(H))
        pc = np.stack([(u - cx) * d / fx, (v - cy) * d / fy, d], -1)
        return pc @ R.T + T

    def close(self):''')
open('rob.py','w').write(s)
EOF
echo ok

# openrua op 34
mkdir -p "$(dirname /workspace/pan.py)"
cat > /workspace/pan.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pan: push clear of the moka pot, grasp handle, place on burner.

Usage: python3 pan.py <stage>   stage in {push, grasp, place, scan}
"""
import math
import sys
import numpy as np
from rob import *

TCP = M["hand"]["tcp_offset_m"]
TABLE = 0.900
BURNER = np.array([-0.049, 0.200])   # burner ring centre (world)
BURNER_TOP = 0.930
SAFE_Z = 1.20                        # hand z for traverses

r = Robot("pan")


def pan_geometry(pw):
    """Locate the pan body (disc) and handle from a birdview cloud."""
    z = pw[..., 2]
    # pan body: points above table, y < -0.13 region (away from moka), x<0.15
    m = np.isfinite(z) & (z > 0.907) & (pw[..., 1] < -0.12) & (pw[..., 0] > -0.35) & (pw[..., 0] < 0.15) & (pw[..., 1] > -0.5)
    P = pw[m]
    # rim = highest ring; fit circle centre as bbox centre of points z>0.925
    rim = P[P[:, 2] > 0.925]
    cx = (rim[:, 0].min() + rim[:, 0].max()) / 2
    cy = (rim[:, 1].min() + rim[:, 1].max()) / 2
    rad = (rim[:, 0].max() - rim[:, 0].min()) / 2
    print(f"pan body centre=({cx:.3f},{cy:.3f}) r={rad:.3f} rim z max={rim[:,2].max():.3f}")
    # handle: narrow strip beyond the rim on +y side
    out = {"c": np.array([cx, cy]), "r": rad}
    for dy in (0.06, 0.08, 0.10, 0.12):
        y0 = cy + rad + dy
        mh = np.isfinite(z) & (z > 0.907) & (pw[..., 1] >= y0 - 0.005) & (pw[..., 1] < y0 + 0.005) & (np.abs(pw[..., 0] - cx) < 0.06)
        H = pw[mh]
        if len(H):
            print(f"  handle @y={y0:.3f}: x[{H[:,0].min():.3f},{H[:,0].max():.3f}] xc={H[:,0].mean():.3f} ztop={H[:,2].max():.3f}")
            out[f"h{dy}"] = (H[:, 0].mean(), y0, H[:, 2].max())
    return out


def scan():
    pw = r.cloud("birdview")
    g = pan_geometry(pw)
    # moka pot
    z = pw[..., 2]
    m = np.isfinite(z) & (z > 0.99) & (pw[..., 0] > -0.1) & (pw[..., 0] < 0.15) & (pw[..., 1] > -0.15) & (pw[..., 1] < 0.1)
    P = pw[m]
    if len(P):
        print(f"moka top pts: x[{P[:,0].min():.3f},{P[:,0].max():.3f}] y[{P[:,1].min():.3f},{P[:,1].max():.3f}] zmax={P[:,2].max():.3f}")
    return g, pw


stage = sys.argv[1]
print("q", np.round(r.arm_q(), 3), "fingers", np.round(r.fingers(), 4), flush=True)

if stage == "scan":
    scan()

elif stage == "push":
    g, pw = scan()
    c, rad = g["c"], g["r"]
    r.gripper(0.0)
    yaw = 0.0                                     # fingers along y; closed gripper ~2cm in y
    start = np.array([c[0] + rad + 0.02, c[1]])
    zt = 0.915 + TCP
    q1 = r.move_pose([start[0], start[1], SAFE_Z], topdown_quat(yaw), 4.0)
    q2 = r.move_pose([start[0], start[1], zt], topdown_quat(yaw), 2.5, seed=q1)
    print("wrench before push", r.wrench(), flush=True)
    end = np.array([start[0] - 0.07, start[1]])
    q3 = r.move_pose([end[0], end[1], zt], topdown_quat(yaw), 3.0, seed=q2)
    print("wrench after push", r.wrench(), flush=True)
    r.move_pose([end[0], end[1], SAFE_Z], topdown_quat(yaw), 2.5, seed=q3)
    scan()

elif stage == "grasp":
    g, pw = scan()
    c, rad = g["c"], g["r"]
    hx, hy, htop = g["h0.08"]
    yaw = math.pi / 2                             # fingers close along world x
    r.gripper(0.04)
    zt = (htop - 0.014) + TCP
    print(f"grasp at ({hx:.3f},{hy:.3f}) fingertip z {htop-0.014:.3f}", flush=True)
    q1 = r.move_pose([hx, hy, SAFE_Z], topdown_quat(yaw), 4.0)
    if q1 is None:
        q1 = r.move_pose([hx, hy, SAFE_Z], topdown_quat(-yaw), 4.0); yaw = -yaw
    q2 = r.move_pose([hx, hy, zt + 0.05], topdown_quat(yaw), 2.0, seed=q1)
    q3 = r.move_pose([hx, hy, zt], topdown_quat(yaw), 2.0, seed=q2)
    p, qu = r.fk()
    print("hand", p.round(4), "fingertip z", round(p[2] - TCP, 4), flush=True)
    f = r.gripper(0.0)
    if abs(f[0]) < 0.004:
        print("!! closed on air", flush=True)
    w0 = r.wrench()
    q4 = r.move_pose([hx, hy, zt + 0.10], topdown_quat(yaw), 2.5, seed=q3)
    print("fingers after lift", np.round(r.fingers(), 4), "wrench", r.wrench(), "was", w0, flush=True)
    g2, _ = scan()
    print("pan rim z after lift (expect ~+0.10):", flush=True)
    np.save("grasp_info.npy", np.array([hx, hy, htop, c[0], c[1], yaw]))

elif stage == "place":
    hx, hy, htop, cx, cy, yaw = np.load("grasp_info.npy")
    off = np.array([hx - cx, hy - cy])            # grasp point relative to pan centre
    tgt = BURNER + off
    print("place: hand xy target", tgt.round(4), "offset", off.round(4), flush=True)
    q0 = r.arm_q()
    p, qu = r.fk(q0)
    # traverse at current height (pan hangs ~10cm above table)
    q1 = r.move_pose([tgt[0], tgt[1], p[2] + 0.05], topdown_quat(yaw), 4.0, seed=q0)
    if q1 is None:
        raise SystemExit("IK traverse failed")
    # pan floor ~0.004 above fingertip-handle relation: pan bottom = table when fingertip z = htop-0.014
    pan_bottom_at_grasp = TABLE
    ft_at_grasp = htop - 0.014
    ft_target = ft_at_grasp + (BURNER_TOP - pan_bottom_at_grasp) + 0.012   # 1.2cm above burner
    q2 = r.move_pose([tgt[0], tgt[1], ft_target + TCP], topdown_quat(yaw), 3.0, seed=q1)
    w0 = r.wrench()[0]
    print("wrench above burner", w0.round(3), flush=True)
    # lower in small steps until contact (force change) or 1.5cm below nominal
    zt = ft_target
    q = q2
    for i in range(6):
        zt -= 0.005
        q = r.move_pose([tgt[0], tgt[1], zt + TCP], topdown_quat(yaw), 1.0, seed=q)
        w = r.wrench()[0]
        print(f"  step {i}: fingertip z {zt:.3f} wrench {w.round(3)} dFz={w[2]-w0[2]:.3f}", flush=True)
        if w[2] - w0[2] > 1.5:
            print("  contact", flush=True); break
    r.gripper(0.04)
    r.move_pose([tgt[0], tgt[1], zt + TCP + 0.12], topdown_quat(yaw), 2.5, seed=q)
    scan()

r.close()
OPENRUA_EOF

# openrua op 35
timeout 200 python3 -u pan.py scan 2>&1

# openrua op 36
python3 - <<'EOF'
s=open('pan.py').read()
s=s.replace("""    cy = (rim[:, 1].min() + rim[:, 1].max()) / 2
    rad = (rim[:, 0].max() - rim[:, 0].min()) / 2""","""    rad = (rim[:, 0].max() - rim[:, 0].min()) / 2
    cy = rim[:, 1].min() + rad                    # y-max side is contaminated by the handle bracket""")
s=s.replace("""        mh = np.isfinite(z) & (z > 0.907) & (pw[..., 1] >= y0 - 0.005) & (pw[..., 1] < y0 + 0.005) & (np.abs(pw[..., 0] - cx) < 0.06)""",
"""        mh = np.isfinite(z) & (z > 0.907) & (z < 0.98) & (pw[..., 1] >= y0 - 0.005) & (pw[..., 1] < y0 + 0.005) & (np.abs(pw[..., 0] - cx) < 0.05)""")
open('pan.py','w').write(s)
EOF
timeout 200 python3 -u pan.py scan 2>&1

# openrua op 37
sed -i 's/end = np.array(\[start\[0\] - 0.07, start\[1\]\])/end = np.array([start[0] - 0.09, start[1]])/' pan.py && nohup python3 -u pan.py push > push.log 2>&1 &
sleep 60; cat /workspace/push.log

# openrua op 38
pkill -f "pan.py push"; sleep 1; cat /workspace/push.log | tail -3; ros2 topic echo /joint_states --once 2>&1 | grep -A9 position

# openrua op 39
pgrep -af "pan.py" || echo "no pan.py"; tail -3 /workspace/push.log; ros2 topic echo /joint_states --once 2>&1 | grep -A9 position; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py frontview

# openrua op 40
timeout 300 python3 -u -c "
from rob import *
r=Robot('rec')
q=r.arm_q(); p,qu=r.fk(q); print('hand', p.round(3), np.round(qu,3)); R=quat_R(*qu); print('hand Z axis', R[:,2].round(2))
print('wrench', r.wrench()[0].round(2))
sol=r.ik([p[0],p[1],p[2]+0.12], qu, seed=q)
print('sol', np.round(sol,3), 'diff', np.round(np.array(sol)-np.array(q),3))
if sol is not None and np.abs(np.array(sol)-np.array(q)).max()<0.6:
    r.move_q(sol, 2.5)
    p,qu=r.fk(); print('now', p.round(3)); print('wrench', r.wrench()[0].round(2))
r.close()
"

# openrua op 41
timeout 200 python3 -u pan.py scan 2>&1; timeout 120 python3 scan.py birdview 0.905 | grep -v blob1

# openrua op 42
timeout 300 python3 -u -c "
from rob import *
r=Robot('home')
for q in [[0,-0.5,0,-2.0,0,1.6,0.785],[0,-0.3,0,-1.8,0,1.6,0.785],[0,-0.785,0,-2.356,0,1.571,0.785],[0,0,0,-1.6,0,1.6,0.785]]:
    p,qu=r.fk(q); R=quat_R(*qu); print(q, 'hand', p.round(3), 'Z', R[:,2].round(2))
r.close()
"

# openrua op 43
python3 - <<'EOF'
s=open('rob.py').read()
s=s.replace("""        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err""","""        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        if code != 0 and err > 0.02 and retry > 0:
            print("  retrying (controller lag)")
            return self.move_q(q, seconds, retry=retry - 1)
        return code, err""")
s=s.replace("    def move_q(self, q, seconds=3.0, waypoints=None):","    def move_q(self, q, seconds=3.0, waypoints=None, retry=1):")
s=s.replace("ARM = FJT[\"joints\"]\n","ARM = FJT[\"joints\"]\nHOME = [0.0, -0.5, 0.0, -2.0, 0.0, 1.6, 0.785]  # hand ~(-0.26, 0, 1.57) pointing down\n")
open('rob.py','w').write(s)
EOF
timeout 300 python3 -u -c "
from rob import *
r=Robot('home'); r.move_q(HOME, 4.0); print(r.fk()[0].round(3)); r.close()
" && timeout 200 python3 -u pan.py scan 2>&1

# openrua op 44
timeout 60 python3 tools/perception/cam_snap.py robot0_robotview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview > /dev/null

# openrua op 45
python3 - <<'EOF'
s=open('pan.py').read()
s=s.replace('''stage = sys.argv[1]
print("q", np.round(r.arm_q(), 3), "fingers", np.round(r.fingers(), 4), flush=True)
''','''stage = sys.argv[1]
print("q", np.round(r.arm_q(), 3), "fingers", np.round(r.fingers(), 4), flush=True)


def go_home():
    q = np.array(r.arm_q())
    if np.abs(q - np.array(HOME)).max() > 0.05:
        r.move_q(HOME, 4.0)


def hover(xy, yaw, z=SAFE_Z):
    """From HOME: IK seeded with HOME so the branch stays elbow-up; move there."""
    go_home()
    sol = r.ik([xy[0], xy[1], z], topdown_quat(yaw), seed=HOME)
    if sol is None:
        sol = r.ik([xy[0], xy[1], z], topdown_quat(-yaw), seed=HOME)
        if sol is None:
            raise SystemExit("hover IK failed")
        yaw = -yaw
    print("hover sol", np.round(sol, 3), "diff from HOME", np.round(np.array(sol) - HOME, 2), flush=True)
    r.move_q(sol, 4.0)
    print("  hand", r.fk()[0].round(4), flush=True)
    return sol, yaw
''')
# push stage
s=s.replace('''    q1 = r.move_pose([start[0], start[1], SAFE_Z], topdown_quat(yaw), 4.0)
    q2 = r.move_pose([start[0], start[1], zt], topdown_quat(yaw), 2.5, seed=q1)''',
'''    q1, yaw = hover(start, yaw)
    q2 = r.move_pose([start[0], start[1], zt], topdown_quat(yaw), 2.5, seed=q1)''')
s=s.replace('''    r.move_pose([end[0], end[1], SAFE_Z], topdown_quat(yaw), 2.5, seed=q3)
    scan()''','''    r.move_pose([end[0], end[1], SAFE_Z], topdown_quat(yaw), 2.5, seed=q3)
    go_home()
    scan()''')
# grasp stage
s=s.replace('''    q1 = r.move_pose([hx, hy, SAFE_Z], topdown_quat(yaw), 4.0)
    if q1 is None:
        q1 = r.move_pose([hx, hy, SAFE_Z], topdown_quat(-yaw), 4.0); yaw = -yaw
''','''    q1, yaw = hover((hx, hy), yaw)
''')
open('pan.py','w').write(s)
EOF
grep -n "hover\|go_home" pan.py | head

# openrua op 46
nohup python3 -u pan.py push > push.log 2>&1 &
sleep 90; cat /workspace/push.log

# openrua op 47
sleep 45; cat /workspace/push.log | tail -12; pgrep -f "pan.py push" >/dev/null && echo RUNNING || echo DONE

# openrua op 48
sleep 40; cat /workspace/push.log | tail -8; pgrep -f "pan.py push" >/dev/null && echo RUNNING || echo DONE

# openrua op 49
sleep 5; pgrep -f "pan.py push" >/dev/null && echo RUNNING || echo DONE; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 50
pgrep -f "pan.py push" >/dev/null && echo RUNNING || echo DONE; timeout 600 python3 -u -c "
from rob import *
r=Robot('cal')
TCP=M['hand']['tcp_offset_m']
r.gripper(0.0)
q=HOME
sol=r.ik([0.16,-0.25,1.2], topdown_quat(0), seed=HOME); r.move_q(sol, 4.0)
q=r.move_pose([0.16,-0.25,0.94+TCP], topdown_quat(0), 2.5, seed=sol)
w0=r.wrench()[0]; print('base wrench', w0.round(3))
for ft in [0.93,0.92,0.915,0.91,0.905,0.90,0.895,0.89]:
    q=r.move_pose([0.16,-0.25,ft+TCP], topdown_quat(0), 1.0, seed=q)
    w=r.wrench()[0]; p=r.fk()[0]
    print(f'fingertip nominal {ft:.3f} hand z {p[2]:.4f} Fz {w[2]:.3f} dFz {w[2]-w0[2]:.3f}', flush=True)
    if w[2]-w0[2] > 2: print('CONTACT'); break
r.move_pose([0.16,-0.25,1.2], topdown_quat(0), 2.5, seed=q)
r.close()
" 2>&1 | grep -v move_q

# openrua op 51
pgrep -f "pan.py push" >/dev/null && echo RUNNING || echo DONE; timeout 600 python3 -u -c "
from rob import *
r=Robot('cal')
TCP=M['hand']['tcp_offset_m']
X,Y=-0.30,-0.30
sol=r.ik([X,Y,1.2], topdown_quat(0), seed=HOME); r.move_q(sol, 4.0)
q=r.move_pose([X,Y,0.94+TCP], topdown_quat(0), 2.5, seed=sol)
w0=r.wrench()[0]; print('base wrench', w0.round(3))
for ft in [0.93,0.92,0.915,0.91,0.905,0.90,0.895,0.89,0.885,0.88]:
    q=r.move_pose([X,Y,ft+TCP], topdown_quat(0), 1.0, seed=q)
    w=r.wrench()[0]; p=r.fk()[0]
    print(f'fingertip nominal {ft:.3f} hand z {p[2]:.4f} Fz {w[2]:.3f} dFz {w[2]-w0[2]:.3f}', flush=True)
    if w[2]-w0[2] > 2: print('CONTACT'); break
r.move_pose([X,Y,1.2], topdown_quat(0), 2.5, seed=q)
r.close()
" 2>&1 | grep -v move_q

# openrua op 52
pgrep -f "pan.py push" >/dev/null && echo RUNNING || echo DONE; timeout 900 python3 -u -c "
from rob import *
r=Robot('cal2')
TCP=M['hand']['tcp_offset_m']
def probe(X,Y,zs):
    sol=r.ik([X,Y,1.15], topdown_quat(0), seed=HOME); r.move_q(sol, 3.0)
    q=r.move_pose([X,Y,zs[0]+0.02+TCP], topdown_quat(0), 2.0, seed=sol)
    w0=r.wrench()[0]
    hit=None
    for ft in zs:
        q=r.move_pose([X,Y,ft+TCP], topdown_quat(0), 1.0, seed=q)
        w=r.wrench()[0]; p=r.fk()[0]
        print(f'  ({X:.3f},{Y:.3f}) fingertip nominal {ft:.3f} hand z {p[2]:.4f} dFz {w[2]-w0[2]:.3f}', flush=True)
        if w[2]-w0[2] > 2: print('  CONTACT at', round(p[2]-TCP,4)); hit=p[2]-TCP; break
    r.move_pose([X,Y,1.15], topdown_quat(0), 2.0, seed=q)
    return hit
print('pan centre'); probe(-0.026,-0.249,[0.95,0.94,0.93,0.92,0.91,0.905,0.90])
print('pan wall r=0.09 (+x side)'); probe(-0.026+0.09,-0.249,[0.95,0.94,0.93,0.92,0.91,0.905,0.90])
print('handle y=-0.09'); probe(-0.028,-0.09,[0.97,0.96,0.95,0.94,0.93,0.92,0.91,0.90])
r.move_q(HOME,3.0)
r.close()
" 2>&1 | grep -v "move_q\|reached hand"

# openrua op 53
pgrep -f "pan.py push" >/dev/null && echo RUNNING || echo DONE; pkill -f "pan.py push"; sed -i 's/    zt = 0.915 + TCP/    zt = 0.925 + TCP/' pan.py && grep -n "zt = 0.9" pan.py && nohup python3 -u pan.py push > push.log 2>&1 &
sleep 100; grep -v "move_q\|retrying" /workspace/push.log

# openrua op 54
pgrep -af "pan.py" | grep -v bash || echo none; grep -n "zt = 0.9" /workspace/pan.py

# openrua op 55
sed -i 's/    zt = 0.915 + TCP/    zt = 0.925 + TCP/' /workspace/pan.py && grep -n "zt = 0.9" /workspace/pan.py && (nohup python3 -u /workspace/pan.py push > /workspace/push.log 2>&1 &) ; sleep 100; grep -v "move_q\|retrying" /workspace/push.log

# openrua op 56
sleep 60; grep -v "move_q\|retrying" /workspace/push.log | tail -9; pgrep -f "pan.py push" >/dev/null && echo RUNNING || echo DONE

# openrua op 57
sleep 20; pgrep -f "pan.py push" >/dev/null && echo RUNNING || echo DONE; timeout 600 python3 -u -c "
from rob import *
r=Robot('vis')
TCP=M['hand']['tcp_offset_m']
X,Y=0.06,-0.249
sol=r.ik([X,Y,1.15], topdown_quat(0), seed=HOME); r.move_q(sol, 4.0)
q=r.move_pose([X,Y,0.925+TCP], topdown_quat(0), 2.5, seed=sol)
print('wrench', r.wrench()[0].round(3)); print(r.fk()[0].round(4))
r.close()
" 2>&1 | grep -v "move_q\|retrying"; for c in agentview frontview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c > /dev/null; done

# openrua op 58
python3 -c "
import cv2
im=cv2.imread('sideview.png'); c=im[180:300,200:360]; cv2.imwrite('side_zoom.png', cv2.resize(c,None,fx=4,fy=4,interpolation=cv2.INTER_CUBIC))"

# openrua op 59
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_robotview >/dev/null

# openrua op 60
timeout 600 python3 -u -c "
from rob import *
r=Robot('servo_test')
p0=r.fk()[0]; print('start', p0.round(4))
r.servo([0,0,0.05], n=20)
p1=r.fk()[0]; print('after 20 msgs +z 0.05', p1.round(4), 'delta', (p1-p0).round(4))
r.close()
" 2>&1

# openrua op 61
timeout 900 python3 -u -c "
from rob import *
r=Robot('push2')
TCP=M['hand']['tcp_offset_m']
q=r.arm_q(); p,qu=r.fk(q)
q=r.move_pose([p[0],p[1],1.10], qu, 2.0, seed=q)
q=r.move_pose([0.10,-0.249,1.10], topdown_quat(0), 2.0, seed=q)
q=r.move_pose([0.10,-0.249,0.925+TCP], topdown_quat(0), 2.0, seed=q)
w0=r.wrench()[0]
for i in range(12):
    r.servo([-0.2,0,0], n=40)
    p=r.fk()[0]; w=r.wrench()[0]
    print(f'burst {i}: hand {p.round(4)} dF {(w-w0).round(2)}', flush=True)
    if p[0] < 0.005: break
r.servo([0,0,0.2], n=60)
print('lifted', r.fk()[0].round(4))
r.move_q(HOME, 4.0)
r.close()
" 2>&1 | grep -v "move_q\|retrying"; timeout 200 python3 -u pan.py scan 2>&1 | grep -v "^q"

# openrua op 62
python3 - <<'EOF'
s=open('rob.py').read()
s=s.replace("    def servo(self,", '''    def move_line(self, p_end, quat, step=0.02, speed=0.05, max_jump=0.4, seed=None):
        """Straight Cartesian line to p_end (world) as one multi-point joint trajectory.
        IK per waypoint, each seeded on the previous, so the branch cannot flip."""
        q = list(seed) if seed is not None else self.arm_q()
        p0, _ = self.fk(q)
        p_end = np.asarray(p_end, float)
        n = max(1, int(np.ceil(np.linalg.norm(p_end - p0) / step)))
        wps = []
        t = 0.0
        dt = max(0.3, (np.linalg.norm(p_end - p0) / n) / speed)
        for i in range(1, n + 1):
            p = p0 + (p_end - p0) * i / n
            sol = self.ik(p, quat, seed=q)
            if sol is None:
                print(f"move_line: IK failed at waypoint {i}/{n} {p.round(4)}"); return None
            jump = np.abs(np.array(sol) - np.array(q)).max()
            if jump > max_jump:
                print(f"move_line: branch jump {jump:.2f} at waypoint {i}/{n}; abort"); return None
            t += dt
            wps.append((sol, t))
            q = sol
        last_q, last_t = wps[-1]
        self.move_q(last_q, last_t, waypoints=wps[:-1])
        p, _ = self.fk()
        print(f"  line reached {p.round(4)} target {p_end.round(4)}")
        return last_q

    def servo(self,''')
open('rob.py','w').write(s)
EOF
echo ok

# openrua op 63
timeout 900 python3 -u -c "
from rob import *
r=Robot('drag')
TCP=M['hand']['tcp_offset_m']
Y=-0.249
r.move_q(HOME, 4.0)
sol=r.ik([-0.08,Y,1.15], topdown_quat(0), seed=HOME); print('hover jump', np.abs(np.array(sol)-HOME).max().round(2)); r.move_q(sol, 4.0)
q=r.move_line([-0.08,Y,0.925+TCP], topdown_quat(0), step=0.03, speed=0.08, seed=sol)
w0=r.wrench()[0]; print('w0', w0.round(2))
q=r.move_line([-0.17,Y,0.925+TCP], topdown_quat(0), step=0.015, speed=0.03, seed=q)
w=r.wrench()[0]; print('dF after drag', (w-w0).round(2))
q=r.move_line([-0.17,Y,1.10], topdown_quat(0), step=0.03, speed=0.08, seed=q)
r.move_q(HOME, 4.0)
r.close()
" 2>&1 | grep -v "retrying"; timeout 200 python3 -u pan.py scan 2>&1 | grep -v "^q"

# openrua op 64
python3 - <<'EOF'
s=open('pan.py').read()
s=s.replace("    for dy in (0.06, 0.08, 0.10, 0.12):","    for dy in (0.05, 0.06, 0.08, 0.10, 0.12):")
start=s.index('elif stage == "grasp":')
end=s.index('r.close()')
new='''elif stage == "grasp":
    g, pw = scan()
    c, rad = g["c"], g["r"]
    hx, hy, htop = g["h0.05"]
    yaw = math.pi / 2                             # fingers close along world x
    r.gripper(0.04)
    ft = htop - 0.014
    print(f"grasp at ({hx:.3f},{hy:.3f}) fingertip z {ft:.3f}", flush=True)
    q1, yaw = hover((hx, hy), yaw, z=1.15)
    q3 = r.move_line([hx, hy, ft + TCP], topdown_quat(yaw), step=0.03, speed=0.08, seed=q1)
    if q3 is None:
        raise SystemExit("descent failed")
    p, qu = r.fk()
    print("hand", p.round(4), "fingertip z", round(p[2] - TCP, 4), flush=True)
    f = r.gripper(0.0)
    if abs(f[0]) < 0.004:
        print("!! closed on air", flush=True)
    w0 = r.wrench()[0]
    q4 = r.move_line([hx, hy, ft + TCP + 0.10], topdown_quat(yaw), step=0.03, speed=0.05, seed=q3)
    print("fingers after lift", np.round(r.fingers(), 4), "dF", (r.wrench()[0] - w0).round(2), flush=True)
    np.save("grasp_info.npy", np.array([hx, hy, htop, c[0], c[1], yaw]))
    # verify: pan rim should now be ~10cm higher (scan from birdview; arm is over the pan though)
    pw2 = r.cloud("birdview")
    z = pw2[..., 2]
    m = np.isfinite(z) & (pw2[..., 1] < c[1] + 0.02) & (pw2[..., 1] > c[1] - 0.12) & (np.abs(pw2[..., 0] - c[0]) < 0.12) & (z > 0.905) & (z < 1.3)
    P = pw2[m]
    print(f"points in pan region: n={len(P)} z[{P[:,2].min():.3f},{P[:,2].max():.3f}] median z {np.median(P[:,2]):.3f}", flush=True)

elif stage == "place":
    hx, hy, htop, cx, cy, yaw = np.load("grasp_info.npy")
    off = np.array([hx - cx, hy - cy])            # grasp point relative to pan centre
    tgt = BURNER + off
    print("place: hand xy target", tgt.round(4), "offset", off.round(4), flush=True)
    q0 = r.arm_q()
    p, qu = r.fk(q0)
    ft_at_grasp = htop - 0.014
    # traverse: straight line at current height (pan bottom ~10cm above table)
    q1 = r.move_line([tgt[0], tgt[1], p[2]], topdown_quat(yaw), step=0.03, speed=0.08, seed=q0)
    if q1 is None:
        raise SystemExit("traverse failed")
    ft_target = ft_at_grasp + (BURNER_TOP - TABLE) + 0.015   # pan bottom 1.5cm above burner
    q2 = r.move_line([tgt[0], tgt[1], ft_target + TCP], topdown_quat(yaw), step=0.03, speed=0.05, seed=q1)
    w0 = r.wrench()[0]
    print("wrench above burner", w0.round(3), flush=True)
    zt = ft_target
    q = q2
    for i in range(8):
        zt -= 0.005
        q = r.move_line([tgt[0], tgt[1], zt + TCP], topdown_quat(yaw), step=0.01, speed=0.02, seed=q)
        w = r.wrench()[0]
        print(f"  step {i}: fingertip z {zt:.3f} dF {(w - w0).round(3)}", flush=True)
        if w[2] - w0[2] > 1.5:
            print("  contact", flush=True); break
    r.gripper(0.04)
    r.move_line([tgt[0], tgt[1], zt + TCP + 0.12], topdown_quat(yaw), step=0.03, speed=0.08, seed=q)
    go_home()
    scan()

'''
s=s[:start]+new+s[end:]
open('pan.py','w').write(s)
EOF
python3 -c "import ast;ast.parse(open('/workspace/pan.py').read())" && echo syntax ok

# openrua op 65
timeout 900 python3 -u -c "
from rob import *
r=Robot('drag2')
TCP=M['hand']['tcp_offset_m']
Y=-0.249
r.move_q(HOME, 4.0)
sol=r.ik([-0.10,Y,1.15], topdown_quat(0), seed=HOME); print('hover jump', np.abs(np.array(sol)-HOME).max().round(2)); r.move_q(sol, 4.0)
q=r.move_line([-0.10,Y,0.925+TCP], topdown_quat(0), step=0.03, speed=0.08, seed=sol)
w0=r.wrench()[0]
q=r.move_line([-0.205,Y,0.925+TCP], topdown_quat(0), step=0.015, speed=0.03, seed=q)
print('dF after drag', (r.wrench()[0]-w0).round(2))
q=r.move_line([-0.205,Y,1.10], topdown_quat(0), step=0.03, speed=0.08, seed=q)
r.move_q(HOME, 4.0)
r.close()
" 2>&1 | grep -v "retrying\|move_q"; timeout 200 python3 -u pan.py scan 2>&1 | grep -v "^q"

# openrua op 66
timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 67
(nohup python3 -u /workspace/pan.py grasp > /workspace/grasp.log 2>&1 &); sleep 120; grep -v "retrying" /workspace/grasp.log; pgrep -f "pan.py grasp" >/dev/null && echo RUNNING || echo DONE

# openrua op 68
python3 - <<'EOF'
s=open('pan.py').read()
s=s.replace("    tgt = BURNER + off\n","    tgt = BURNER + off + np.array([0.015, 0.0])   # pan centre 1.5cm +x of burner centre: keeps rim clear of the knob\n")
s=s.replace("    q1 = r.move_line([tgt[0], tgt[1], p[2]], topdown_quat(yaw), step=0.03, speed=0.08, seed=q0)",
"""    q_up = r.move_line([p[0], p[1], 1.22], topdown_quat(yaw), step=0.03, speed=0.08, seed=q0)
    q1 = r.move_line([tgt[0], tgt[1], 1.22], topdown_quat(yaw), step=0.03, speed=0.08, seed=q_up)""")
s=s.replace("    ft_target = ft_at_grasp + (BURNER_TOP - TABLE) + 0.015   # pan bottom 1.5cm above burner",
            "    ft_target = ft_at_grasp + (BURNER_TOP - TABLE) + 0.010   # pan bottom 1cm above burner (if hanging level)")
s=s.replace("""    for i in range(8):
        zt -= 0.005""","""    for i in range(4):
        zt -= 0.005""")
s=s.replace("        if w[2] - w0[2] > 1.5:","        if w[2] - w0[2] > 1.0:")
open('pan.py','w').write(s)
EOF
grep -n "tgt = \|1.22\|ft_target =\|range(4)\|> 1.0" /workspace/pan.py; pgrep -f "pan.py grasp" >/dev/null && echo RUNNING || echo DONE

# openrua op 69
pkill -f "pan.py grasp"; sleep 1; (nohup python3 -u /workspace/pan.py place > /workspace/place.log 2>&1 &); sleep 150; grep -v "retrying" /workspace/place.log; pgrep -f "pan.py place" >/dev/null && echo RUNNING || echo DONE

# openrua op 70
sleep 5; pgrep -af "pan.py" | grep -v bash || echo "no pan.py running"; cat /workspace/place.log 2>/dev/null | grep -v retrying

# openrua op 71
(nohup python3 -u pan.py place > place.log 2>&1 &) ; sleep 5; echo started

# openrua op 72
sleep 120; grep -v retrying /workspace/place.log

# openrua op 73
sleep 45; grep -v retrying /workspace/place.log | tail -8; pgrep -af "pan.py place" | grep -v bash >/dev/null && echo RUNNING || echo DONE

# openrua op 74
ros2 topic echo /joint_states --once 2>&1 | grep -A7 position | tail -7 | tr '\n' ' '; echo; timeout 120 python3 scan.py birdview 0.905 | grep -v blob1; for c in agentview frontview sideview robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c > /dev/null; done

# openrua op 75
python3 -c "
import numpy as np
pw=np.load('birdview_pts.npy'); z=pw[...,2]
m=np.isfinite(z)&(z>0.945)&(pw[...,1]>0.05)&(pw[...,1]<0.5)&(pw[...,0]>-0.15)&(pw[...,0]<0.1)
P=pw[m]
body=P[P[:,1]<0.31]
print('pan rim pts x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(body[:,0].min(),body[:,0].max(),body[:,1].min(),body[:,1].max(),body[:,2].min(),body[:,2].max()))
print('pan centre est (%.3f, %.3f)  burner centre (-0.049, 0.200)'%((body[:,0].min()+body[:,0].max())/2, body[:,1].min()+ (body[:,0].max()-body[:,0].min())/2))
# pan floor height: points inside r<0.05 of centre
cx,cy=(body[:,0].min()+body[:,0].max())/2, body[:,1].min()+(body[:,0].max()-body[:,0].min())/2
rr=np.hypot(pw[...,0]-cx,pw[...,1]-cy); mm=np.isfinite(z)&(rr<0.04)
print('pan floor z median %.3f (table 0.900, burner top 0.930; floor was 0.904 on table)'%np.median(z[mm]))
# handle
h=P[P[:,1]>0.32]; print('handle x[%.3f,%.3f] y[%.3f,%.3f] ztop %.3f'%(h[:,0].min(),h[:,0].max(),h[:,1].min(),h[:,1].max(),h[:,2].max()))
"

# openrua op 76
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-moveit-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-moveit-quirks.md <<'OPENRUA_EOF'
---
name: panda-moveit-quirks
description: Verified quirks of the /workspace Franka Panda sim (FK/IK frames, IK tip link, trajectory tolerance aborts, servo drift) that the shipped docs get wrong or omit
metadata:
  type: project
---

Verified 2026-09-12 on the /workspace Panda + MoveIt machine (world->panda_link0 = (-0.66, 0, 0.912), table top z=0.900):

- `/compute_fk` and `/compute_ik` (frame_id left EMPTY) both use the WORLD frame, not the arm base as docs/30-action.md claims. Checked against `tf2_echo panda_link0 panda_hand`.
- `/compute_ik` default tip link is `panda_link8`, which is yawed 45° from `panda_hand`. Set `ik_request.ik_link_name = "panda_hand"` or every solution comes back 45° off.
- Top-down hand quaternion in world: (1,0,0,0) = fingers close along world Y; `topdown_quat(yaw)` in /workspace/rob.py.
- Trajectory `error_code=-5` (tolerance violated) shows up on almost every move with >~0.3 rad joint change and is usually lag: resending the same goal converges. Contact does NOT reliably show up as -5.
- A single IK-then-trajectory move between distant poses can flip branch and swing the hand through the scene (it brushed the pan/moka pot once). Use a HOME waypoint ([0,-0.5,0,-2,0,1.6,0.785], hand at (-0.26,0,1.57)) for traverses and `Robot.move_line` (per-2cm IK seeded on previous) for anything near objects.
- `/servo_node/delta_twist_cmds` moves ~0.2 mm per message at 0.05 m/s and drifts badly (+z, +y) near the reach limit; prefer move_line.
- Fingertip = hand z − 0.1034 was confirmed by touching the table (contact at hand z≈1.004).
- Object collision shells are ~1 cm padded relative to depth-visible surfaces (pan floor visual 0.904, felt at 0.914). Frying-pan collision is a thin floor + rim lip: closed fingers at z 0.915–0.925 slid straight under the rim without moving the pan; dragging from inside the rim worked.
- The gripper does stop on contact (fin: 0.011 m/finger, pan handle: 0.013) despite `stops_at: open/closed`.
- Helpers live in /workspace/rob.py (Robot class), /workspace/scan.py (birdview -> world blobs), /workspace/pan.py (stages).

**Why:** the startup docs are generic and partly wrong for this machine; rediscovering each cost many minutes of sim time.
**How to apply:** start from rob.py; always go via HOME for big traverses; verify pushes with a fresh birdview scan since no-contact pushes look successful in the logs.
OPENRUA_EOF

# openrua op 77
cat /claude-config/projects/-workspace/memory/MEMORY.md 2>/dev/null || echo "(no index yet)"

# openrua op 78
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda/MoveIt machine quirks](panda-moveit-quirks.md) — FK/IK are world-frame, IK tip is panda_link8, -5 aborts are lag, use HOME waypoint + move_line; helpers in /workspace/rob.py
OPENRUA_EOF
