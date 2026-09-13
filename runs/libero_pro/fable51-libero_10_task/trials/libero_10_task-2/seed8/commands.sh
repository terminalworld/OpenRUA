#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/action/fjt_send.py tools/action/ik_move.py tools/action/gripper_cmd.py tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | grep -v -E "describe_parameters|get_parameter|list_parameters|set_parameters|get_type_description"; echo ---; timeout 20 ros2 node list

# openrua op 3
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; timeout 30 ros2 topic echo /joint_states --once

# openrua op 4
timeout 20 ros2 topic echo /tf_static --once 2>/dev/null | grep -E "frame_id|child_frame_id" | sort | uniq -c; echo ---; timeout 20 ros2 topic echo /tf --once 2>/dev/null | grep -E "frame_id|child_frame_id" | sort | uniq

# openrua op 5
timeout 20 ros2 topic echo /tf --once 2>/dev/null | grep -B2 -A12 "child_frame_id: panda_link0" | head -30; echo; timeout 20 ros2 topic echo /tf --once 2>/dev/null | grep -B2 -A12 "child_frame_id: robot0_eye_in_hand_optical_frame" | head -30

# openrua op 6
timeout 30 ros2 topic echo /tf --once 2>/dev/null | grep -A12 "child_frame_id: panda_link0" | head -14

# openrua op 7
mkdir -p "$(dirname /workspace/px_batch.py)"
cat > /workspace/px_batch.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""World coordinates for many pixels from one camera's current depth frame.

Usage: python3 px_batch.py <camera> u1,v1 u2,v2 ...
Also saves <camera>_cloud.npz with the full HxWx3 world-xyz array.
"""
import struct
import sys

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, msg_type, timeout=20.0):
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


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    cam = sys.argv[1]
    pixels = [tuple(int(t) for t in a.split(",")) for a in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("px_batch")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    H, W = depth_msg.height, depth_msg.width
    depth = np.frombuffer(depth_msg.data, dtype=np.float32).reshape(H, W)
    frame = f"{cam}_optical_frame"
    end = node.get_clock().now().nanoseconds / 1e9 + 10.0
    while node.get_clock().now().nanoseconds / 1e9 < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        if tfbuf.can_transform("world", frame, rclpy.time.Time()):
            break
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y,
                  t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    us, vs = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(us - cx) * depth / fx, (vs - cy) * depth / fy, depth], -1)
    world = pc @ R.T + T
    np.savez(f"{cam}_cloud.npz", world=world, depth=depth)
    for (u, v) in pixels:
        x, y, z = world[v, u]
        print(f"px ({u},{v}) depth={depth[v,u]:.4f} -> world {x:.4f} {y:.4f} {z:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 60 python3 px_batch.py robot0_eye_in_hand 140,120 50,35 235,210 148,270 148,255 148,300 580,120 350,110 300,250 320,30 30,300 600,300

# openrua op 9
timeout 15 ros2 topic echo /pipeline_state --once 2>&1 | head -20; echo ---; timeout 20 ros2 node info /robot_bridge 2>&1 | head -60

# openrua op 10
python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); w=d['world']
X,Y,Z=w[...,0],w[...,1],w[...,2]
def ext(mask,name):
    xs,ys,zs=X[mask],Y[mask],Z[mask]
    print(f'{name}: n={mask.sum()} x[{xs.min():.3f},{xs.max():.3f}] y[{ys.min():.3f},{ys.max():.3f}] z[{zs.min():.3f},{zs.max():.3f}] centroid=({xs.mean():.3f},{ys.mean():.3f},{zs.mean():.3f})')
stove=(Z>0.915)&(Z<0.945)&(Y>0.08)&(Y<0.32)&(X>-0.16)&(X<0.08)
ext(stove,'stove')
knob=(Z>0.905)&(Y>0.1)&(Y<0.27)&(X>-0.28)&(X<-0.15)
ext(knob,'knob')
knobtop=knob&(Z>0.93)
ext(knobtop,'knobtop')
pan=(Z>0.905)&(Y<-0.1)
ext(pan,'pan')
handle=(Z>0.905)&(Y<0.02)&(Y>-0.1)&(X>-0.1)&(X<0.05)
ext(handle,'handle')
"


# openrua op 11
timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | tail -25

# openrua op 12
timeout 60 ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{ik_request: {group_name: panda_arm, robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}, pose_stamped: {header: {frame_id: ''}, pose: {position: {x: -0.203, y: 0.0, z: 1.2696}, orientation: {x: 0.9996, y: 0.0, z: -0.0284, w: 0.0}}}, timeout: {sec: 5}}}" 2>&1 | grep -oE "position=\[[^]]*\]|val=-?[0-9]+" 

# openrua op 13
timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.05846165151957628, -0.16131001035395545, -0.0579201015032298, -2.4446575328760303, -0.01162123465199098, 2.226770915891463, 0.00890601661941885]}}}" 2>&1 | grep -oE "Point\([^)]*\)|Quaternion\([^)]*\)"

# openrua op 14
timeout 60 python3 px_batch.py birdview 250,275 320,275 375,282 375,240 && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); w=d['world']
X,Y,Z=w[...,0],w[...,1],w[...,2]
def ext(mask,name):
    xs,ys,zs=X[mask],Y[mask],Z[mask]
    print(f'{name}: n={mask.sum()} x[{xs.min():.3f},{xs.max():.3f}] y[{ys.min():.3f},{ys.max():.3f}] z[{zs.min():.3f},{zs.max():.3f}] centroid=({xs.mean():.3f},{ys.mean():.3f},{zs.mean():.3f})')
pan=(Z>0.905)&(Y<-0.1)&(Y>-0.35)&(X>-0.3)&(X<0.1)
ext(pan,'pan')
stove=(Z>0.915)&(Z<0.95)&(Y>0.08)&(Y<0.32)&(X>-0.16)&(X<0.08)
ext(stove,'stove')
knob=(Z>0.905)&(Y>0.1)&(Y<0.27)&(X>-0.28)&(X<-0.15)
ext(knob,'knob')
table=(Z>0.85)&(Z<0.905)
print('table z', Z[table].mean())
"

# openrua op 15
timeout 300 python3 tools/action/fjt_send.py 0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483 2.0 && timeout 60 python3 px_batch.py birdview 250,275 320,275 375,282 375,240 && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); w=d['world']
X,Y,Z=w[...,0],w[...,1],w[...,2]
def ext(mask,name):
    xs,ys,zs=X[mask],Y[mask],Z[mask]
    print(f'{name}: n={mask.sum()} x[{xs.min():.3f},{xs.max():.3f}] y[{ys.min():.3f},{ys.max():.3f}] z[{zs.min():.3f},{zs.max():.3f}] centroid=({xs.mean():.3f},{ys.mean():.3f},{zs.mean():.3f})')
pan=(Z>0.905)&(Z<1.1)&(Y<-0.1)&(Y>-0.35)&(X>-0.3)&(X<0.1)
ext(pan,'pan')
handle=(Z>0.905)&(Z<1.1)&(Y<0.05)&(Y>-0.1)&(X>-0.15)&(X<0.1)
ext(handle,'handle')
"

# openrua op 16
timeout 60 python3 px_batch.py robot0_eye_in_hand 148,270 >/dev/null && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); w=d['world']
X,Y,Z=w[...,0],w[...,1],w[...,2]
knob=(Z>0.905)&(Y>0.12)&(Y<0.26)&(X>-0.27)&(X<-0.14)
# height histogram
zs=Z[knob]
h,e=np.histogram(zs,bins=np.arange(0.90,0.97,0.005))
for c,lo in zip(h,e[:-1]): print(f'z {lo:.3f}: {c}')
tab=knob&(Z>0.945)
print('tab x',X[tab].min(),X[tab].max(),'y',Y[tab].min(),Y[tab].max(), 'centroid',X[tab].mean(),Y[tab].mean())
cyl=knob&(Z>0.915)&(Z<0.945)
print('cyl x',X[cyl].min(),X[cyl].max(),'y',Y[cyl].min(),Y[cyl].max(), 'centroid',X[cyl].mean(),Y[cyl].mean(), 'zmean',Z[cyl].mean())
# handle cross-section
hd=(Z>0.905)&(Y>-0.08)&(Y<-0.02)&(X>-0.15)&(X<0.0)
print('handle x',X[hd].min(),X[hd].max(),'z',Z[hd].min(),Z[hd].max(),'centroid',X[hd].mean(),Y[hd].mean(),Z[hd].mean())
h,e=np.histogram(Z[hd],bins=np.arange(0.90,0.97,0.005))
for c,lo in zip(h,e[:-1]): print(f'  handle z {lo:.3f}: {c}')
"

# openrua op 17
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Persistent arm helper: FK/IK/trajectory/gripper clients built once.

TCP = fingertip midpoint, hand +Z (approach) offset by hand.tcp_offset_m.
Hand orientation is "pointing down" with a world yaw psi:
    R = Rz(psi) @ Rx(pi)   ->  hand x = (cos psi, sin psi, 0)
Fingers slide along hand y, so they close along Rz(psi)*(0,-1,0):
    psi = 0     -> fingers close along world y
    psi = pi/2  -> fingers close along world x
"""
import math
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
JOINTS = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return np.array([0.25 * s, (R[0, 1] + R[1, 0]) / s,
                         (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s])
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return np.array([(R[0, 1] + R[1, 0]) / s, 0.25 * s,
                         (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s])
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return np.array([(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
                     0.25 * s, (R[1, 0] - R[0, 1]) / s])


def down_R(psi):
    c, s = math.cos(psi), math.sin(psi)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    return Rz @ Rx


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli.wait_for_service(20)
        self.ik_cli.wait_for_service(20)
        self.fjt.wait_for_server(20)
        self.grip.wait_for_server(20)
        self.spin(0.5)

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    def spin(self, sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self):
        self._js = {}
        while not self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return [self._js[j] for j in JOINTS]

    def fingers(self):
        self.joints()
        return (self._js.get("panda_finger_joint1"),
                self._js.get("panda_finger_joint2"))

    def _seed(self, q):
        js = JointState()
        js.name = list(JOINTS)
        js.position = [float(v) for v in q]
        return js

    def fk(self, q, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {res and res.error_code.val}")
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        R = quat_to_R(p.orientation.x, p.orientation.y,
                      p.orientation.z, p.orientation.w)
        return pos, R

    def tcp(self, q=None):
        q = self.joints() if q is None else q
        pos, R = self.fk(q)
        return pos + TCP_OFF * R[:, 2], R

    def ik(self, hand_pos, R, seed=None, timeout=5.0):
        seed = self.joints() if seed is None else seed
        qx, qy, qz, qw = R_to_quat(R)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_pos)
        p.orientation.x, p.orientation.y = float(qx), float(qy)
        p.orientation.z, p.orientation.w = float(qz), float(qw)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=90)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name,
                       res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def solve_tcp(self, tcp_xyz, psi, seed=None, pos_tol=0.004):
        """Joint config with TCP at tcp_xyz, hand pointing down, yaw psi.
        IK orientation is loose on this machine -> fix yaw via joint7,
        verify by FK. Returns q or None."""
        R = down_R(psi)
        hand = np.asarray(tcp_xyz, float) - TCP_OFF * R[:, 2]
        seed = self.joints() if seed is None else seed
        for attempt in range(4):
            q = self.ik(hand, R, seed)
            if q is None:
                seed = list(np.array(seed) + np.random.uniform(-0.3, 0.3, 7))
                continue
            for _ in range(3):
                pos, Rf = self.fk(q)
                tilt = math.degrees(math.acos(max(-1, min(1, -Rf[2, 2]))))
                yaw_act = math.atan2(Rf[1, 0], Rf[0, 0])
                dyaw = (psi - yaw_act + math.pi) % (2 * math.pi) - math.pi
                if abs(dyaw) < math.radians(1.0):
                    break
                # joint7 axis = hand z (down) -> +j7 rotates yaw negative
                q[6] -= dyaw
                if not (LIMITS[6][0] < q[6] < LIMITS[6][1]):
                    q[6] += 2 * math.pi if q[6] < 0 else -2 * math.pi
            pos, Rf = self.fk(q)
            tcp_act = pos + TCP_OFF * Rf[:, 2]
            err = np.linalg.norm(tcp_act - tcp_xyz)
            tilt = math.degrees(math.acos(max(-1, min(1, -Rf[2, 2]))))
            yaw_act = math.atan2(Rf[1, 0], Rf[0, 0])
            ok_lim = all(lo < v < hi for v, (lo, hi) in zip(q, LIMITS))
            if err < pos_tol and tilt < 3 and ok_lim and \
                    abs((psi - yaw_act + math.pi) % (2 * math.pi) - math.pi) < math.radians(2):
                return q
            print(f"  solve_tcp attempt {attempt}: err={err*1000:.1f}mm tilt={tilt:.1f}deg "
                  f"yaw={math.degrees(yaw_act):.1f} lim_ok={ok_lim}; retrying")
            seed = list(np.array(q) + np.random.uniform(-0.3, 0.3, 7))
        return None

    def move(self, q, seconds, tol=0.02, retries=1):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds),
                                      nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        for attempt in range(retries + 1):
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            gh = send.result()
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            code = res.result().result.error_code
            cur = self.joints()
            err = max(abs(a - b) for a, b in zip(cur, q))
            print(f"  move: error_code={code} max_joint_err={err:.4f}")
            if err < tol:
                return True
        return err < tol

    def move_tcp(self, tcp_xyz, psi, seconds=3.0, seed=None):
        q = self.solve_tcp(tcp_xyz, psi, seed)
        if q is None:
            print(f"  NO IK for tcp={tcp_xyz} psi={math.degrees(psi):.0f}")
            return False
        ok = self.move(q, seconds)
        p, R = self.tcp()
        print(f"  tcp now {p.round(4)} yaw={math.degrees(math.atan2(R[1,0],R[0,0])):.1f}")
        return ok

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result
        f = self.fingers()
        print(f"  gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}")
        return f
OPENRUA_EOF

# openrua op 18
timeout 120 python3 -c "
import math, numpy as np
from arm import Arm, down_R, R_to_quat
a=Arm()
q=a.joints(); print('q',np.round(q,3))
p,R=a.tcp(); print('tcp',p.round(4),'yaw',math.degrees(math.atan2(R[1,0],R[0,0])),'hand z',R[:,2].round(3))
q2=list(q); q2[6]+=0.5
p2,R2=a.tcp(q2); print('after +0.5 j7: yaw',math.degrees(math.atan2(R2[1,0],R2[0,0])))
print('down_R(0) quat',R_to_quat(down_R(0)).round(4))
print('fingers',a.fingers())
"

# openrua op 19
timeout 900 python3 -u -c "
import math, numpy as np
from arm import Arm
a=Arm()
KNOB=(-0.205, 0.192)
print('above knob'); a.move_tcp((KNOB[0],KNOB[1],1.02), 0.0, 3.0)
print('descend'); a.move_tcp((KNOB[0],KNOB[1],0.945), 0.0, 2.0)
print('q', np.round(a.joints(),4))
" 2>&1 | grep -v "^\[" 

# openrua op 20
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null; echo ok

# openrua op 21
timeout 600 python3 -u -c "
from arm import Arm
a=Arm()
f=a.gripper(0.0)
print('gap', f[0]-f[1])
" 2>&1 | grep -v "^\["

# openrua op 22
timeout 600 python3 -u -c "
import math, numpy as np
from arm import Arm
a=Arm()
q=a.joints()
q[6]-=math.pi/4
a.move(q,2.0)
f=a.fingers(); print('fingers',f,'gap',f[0]-f[1])
p,R=a.tcp(); print('tcp',p.round(4),'yaw',math.degrees(math.atan2(R[1,0],R[0,0])))
" 2>&1 | grep -v "^\["; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 23
timeout 600 python3 -u -c "
import math, numpy as np
from arm import Arm
a=Arm()
q=a.joints()
q[6]-=math.radians(40)
a.move(q,2.0)
f=a.fingers(); print('fingers',f,'gap',f[0]-f[1])
p,R=a.tcp(); print('tcp',p.round(4),'yaw',math.degrees(math.atan2(R[1,0],R[0,0])))
print('release'); a.gripper(0.04)
print('lift'); a.move_tcp((p[0],p[1],1.05), math.atan2(R[1,0],R[0,0]), 2.0)
" 2>&1 | grep -v "^\["; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py birdview >/dev/null

# openrua op 24
timeout 60 python3 px_batch.py birdview 320,275 >/dev/null && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); w=d['world']
X,Y,Z=w[...,0],w[...,1],w[...,2]
def ext(mask,name):
    xs,ys,zs=X[mask],Y[mask],Z[mask]
    print(f'{name}: n={mask.sum()} x[{xs.min():.3f},{xs.max():.3f}] y[{ys.min():.3f},{ys.max():.3f}] z[{zs.min():.3f},{zs.max():.3f}] centroid=({xs.mean():.3f},{ys.mean():.3f},{zs.mean():.3f})')
pan=(Z>0.905)&(Z<0.96)&(Y<-0.1)&(Y>-0.36)&(X>-0.25)&(X<0.1)
ext(pan,'pan body')
rim=pan&(Z>0.935)
ext(rim,'pan rim')
hd=(Z>0.93)&(Z<0.96)&(Y<0.05)&(Y>-0.1)&(X>-0.15)&(X<0.0)
ext(hd,'handle')
for y0 in [-0.09,-0.07,-0.05,-0.03,-0.01,0.01]:
    m=hd&(Y>y0-0.01)&(Y<y0+0.01)
    if m.sum(): print(f'  handle slice y={y0:+.2f}: x[{X[m].min():.3f},{X[m].max():.3f}] xc={X[m].mean():.3f} ztop={Z[m].max():.3f}')
"

# openrua op 25
timeout 900 python3 -u -c "
import math, numpy as np
from arm import Arm
a=Arm()
G=(-0.072,-0.05)
PSI=math.pi/2
print('above handle'); a.move_tcp((G[0],G[1],1.05), PSI, 3.0)
print('descend'); a.move_tcp((G[0],G[1],0.94), PSI, 2.0)
" 2>&1 | grep -v "^\["; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand >/dev/null

# openrua op 26
timeout 600 python3 -u -c "
import math, numpy as np
from arm import Arm
a=Arm()
G=(-0.072,-0.05)
PSI=math.pi/2
print('descend again'); a.move_tcp((G[0],G[1],0.94), PSI, 2.0)
" 2>&1 | grep -v "^\["; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 27
timeout 600 python3 -u -c "
import math, numpy as np
from arm import Arm
a=Arm()
f=a.gripper(0.0); print('gap',f[0]-f[1])
PSI=math.pi/2
print('lift'); a.move_tcp((-0.072,-0.05,1.10), PSI, 3.0)
f=a.fingers(); print('fingers after lift',f,'gap',f[0]-f[1])
" 2>&1 | grep -v "^\["; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 28
timeout 900 python3 -u -c "
import math, numpy as np
from arm import Arm
a=Arm()
PSI=math.pi/2
T=(-0.048,0.365)
print('over stove'); a.move_tcp((T[0],T[1],1.10), PSI, 4.0)
f=a.fingers(); print('gap',f[0]-f[1])
print('lower'); a.move_tcp((T[0],T[1],0.985), PSI, 3.0)
f=a.fingers(); print('gap',f[0]-f[1])
" 2>&1 | grep -v "^\["; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force; timeout 60 python3 tools/perception/cam_snap.py sideview >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null

# openrua op 29
timeout 900 python3 -u -c "
import math, numpy as np
from arm import Arm
a=Arm()
PSI=math.pi/2
T=(-0.048,0.365)
print('lower more'); a.move_tcp((T[0],T[1],0.975), PSI, 2.0)
print('release'); a.gripper(0.04)
print('lift'); a.move_tcp((T[0],T[1],1.10), PSI, 3.0)
" 2>&1 | grep -v "^\["; for c in agentview birdview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c >/dev/null; done; echo ok

# openrua op 30
timeout 60 python3 px_batch.py birdview 375,282 >/dev/null && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); w=d['world']
X,Y,Z=w[...,0],w[...,1],w[...,2]
def ext(mask,name):
    xs,ys,zs=X[mask],Y[mask],Z[mask]
    print(f'{name}: n={mask.sum()} x[{xs.min():.3f},{xs.max():.3f}] y[{ys.min():.3f},{ys.max():.3f}] z[{zs.min():.3f},{zs.max():.3f}] centroid=({xs.mean():.3f},{ys.mean():.3f},{zs.mean():.3f})')
# everything above table in the region around stove, excluding robot (robot is high z)
obj=(Z>0.905)&(Z<1.06)&(X>-0.3)&(X<0.3)&(Y>-0.2)&(Y<0.5)
ext(obj,'all objs')
# grid occupancy: print coarse map of max height
xs=np.arange(-0.3,0.3,0.02); ys=np.arange(-0.2,0.5,0.02)
print('rows x from -0.30 (top) step 0.02; cols y from -0.20 step 0.02; value=max z cm above table (.: none)')
print('      '+''.join(f'{int(round(y*100)):>4d}' for y in ys))
for x0 in xs:
    row=''
    for y0 in ys:
        m=obj&(X>=x0)&(X<x0+0.02)&(Y>=y0)&(Y<y0+0.02)
        row+= f'{int(round((Z[m].max()-0.9)*100)):>4d}' if m.sum() else '   .'
    print(f'{x0:+.2f} '+row)
"

# openrua op 31
timeout 600 python3 -u -c "
import math
from arm import Arm
a=Arm()
a.move_tcp((-0.30,0.0,1.25), 0.0, 3.0)
" 2>&1 | grep -v "^\["; for c in agentview birdview frontview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c >/dev/null; done; echo ok

# openrua op 32
timeout 60 python3 px_batch.py birdview 375,282 >/dev/null && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); w=d['world']
X,Y,Z=w[...,0],w[...,1],w[...,2]
obj=(Z>0.905)&(Z<1.2)&(X>-0.3)&(X<0.3)&(Y>-0.2)&(Y<0.5)
xs=np.arange(-0.24,0.16,0.02); ys=np.arange(-0.14,0.44,0.02)
print('rows x (top=-0.24) step 0.02; cols y step 0.02; value=max z cm above table')
print('      '+''.join(f'{int(round(y*100)):>4d}' for y in ys))
for x0 in xs:
    row=''
    for y0 in ys:
        m=obj&(X>=x0)&(X<x0+0.02)&(Y>=y0)&(Y<y0+0.02)
        row+= f'{int(round((Z[m].max()-0.9)*100)):>4d}' if m.sum() else '   .'
    print(f'{x0:+.2f} '+row)
pot=obj&(X>-0.02)&(Y<0.12)&(Y>-0.15)
print('pot: x[%.3f,%.3f] y[%.3f,%.3f] zmax %.3f'%(X[pot].min(),X[pot].max(),Y[pot].min(),Y[pot].max(),Z[pot].max()))
"

# openrua op 33
timeout 600 python3 -u -c "
import math
from arm import Arm
a=Arm()
a.move_tcp((-0.07,0.35,1.10), math.pi/2, 5.0)
" 2>&1 | grep -v "^\["; timeout 60 python3 px_batch.py robot0_eye_in_hand 320,240 >/dev/null && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); w=d['world']
X,Y,Z=w[...,0],w[...,1],w[...,2]
hd=(Z>0.93)&(Z<1.0)&(Y>0.28)&(Y<0.46)&(X>-0.15)&(X<0.0)
print('handle x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(X[hd].min(),X[hd].max(),Y[hd].min(),Y[hd].max(),Z[hd].min(),Z[hd].max()))
for y0 in np.arange(0.29,0.45,0.02):
    m=hd&(Y>y0-0.01)&(Y<y0+0.01)
    if m.sum(): print(f'  y={y0:.2f}: x[{X[m].min():.3f},{X[m].max():.3f}] xc={X[m].mean():.3f} ztop={Z[m].max():.3f}')
"

# openrua op 34
timeout 900 python3 -u -c "
import math
from arm import Arm
a=Arm()
PSI=math.pi/2
G=(-0.055,0.35)
print('descend'); a.move_tcp((G[0],G[1],0.955), PSI, 2.5); a.move_tcp((G[0],G[1],0.955), PSI, 1.5)
f=a.gripper(0.0); print('gap',f[0]-f[1])
print('lift'); a.move_tcp((G[0],G[1],1.15), PSI, 3.0)
f=a.fingers(); print('gap after lift',f[0]-f[1])
print('carry back'); a.move_tcp((-0.072,-0.05,1.15), PSI, 4.0)
print('lower'); a.move_tcp((-0.072,-0.05,0.945), PSI, 3.0); a.move_tcp((-0.072,-0.05,0.945), PSI, 1.5)
print('release'); a.gripper(0.04)
print('lift'); a.move_tcp((-0.072,-0.05,1.10), PSI, 2.5)
" 2>&1 | grep -v "^\["; for c in agentview birdview; do timeout 60 python3 tools/perception/cam_snap.py $c >/dev/null; done; echo ok

# openrua op 35
timeout 900 python3 -u -c "
import math
from arm import Arm
a=Arm()
a.gripper(0.0)
print('pre-push pose'); a.move_tcp((-0.07,0.05,1.05), 0.0, 3.0)
print('descend'); a.move_tcp((-0.07,0.05,0.935), 0.0, 2.0); a.move_tcp((-0.07,0.05,0.935), 0.0, 1.0)
print('push'); a.move_tcp((0.16,0.05,0.935), 0.0, 3.0); a.move_tcp((0.16,0.05,0.935), 0.0, 1.5)
print('retreat up'); a.move_tcp((0.16,0.05,1.10), 0.0, 2.0)
a.gripper(0.04)
" 2>&1 | grep -v "^\["; for c in agentview birdview; do timeout 60 python3 tools/perception/cam_snap.py $c >/dev/null; done; echo ok

# openrua op 36
timeout 600 python3 -u -c "
import math
from arm import Arm
a=Arm()
for x in [0.06,0.08,0.10,0.12,0.14]:
    q=a.solve_tcp((x,0.05,0.935),0.0)
    print(x, 'OK' if q else 'no')
" 2>&1 | grep -v "^\["

# openrua op 37
timeout 900 python3 -u -c "
import math
from arm import Arm
a=Arm()
a.gripper(0.0)
print('push'); a.move_tcp((0.12,0.05,0.935), 0.0, 3.0); a.move_tcp((0.12,0.05,0.935), 0.0, 1.5)
print('retreat up'); a.move_tcp((0.12,0.05,1.10), 0.0, 2.0)
" 2>&1 | grep -v "^\["; for c in agentview birdview; do timeout 60 python3 tools/perception/cam_snap.py $c >/dev/null; done; echo ok

# openrua op 38
timeout 60 python3 px_batch.py birdview 375,282 >/dev/null && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); w=d['world']
X,Y,Z=w[...,0],w[...,1],w[...,2]
def ext(mask,name):
    xs,ys,zs=X[mask],Y[mask],Z[mask]
    print(f'{name}: n={mask.sum()} x[{xs.min():.3f},{xs.max():.3f}] y[{ys.min():.3f},{ys.max():.3f}] z[{zs.min():.3f},{zs.max():.3f}] centroid=({xs.mean():.3f},{ys.mean():.3f},{zs.mean():.3f})')
pan=(Z>0.905)&(Z<0.96)&(Y<-0.1)&(Y>-0.36)&(X>-0.25)&(X<0.1); ext(pan,'pan body')
hd=(Z>0.93)&(Z<0.96)&(Y<0.05)&(Y>-0.1)&(X>-0.15)&(X<0.0); ext(hd,'handle')
for y0 in [-0.07,-0.05,-0.03]:
    m=hd&(Y>y0-0.01)&(Y<y0+0.01)
    if m.sum(): print(f'  handle y={y0:+.2f}: x[{X[m].min():.3f},{X[m].max():.3f}] xc={X[m].mean():.3f} ztop={Z[m].max():.3f}')
pot=(Z>0.905)&(Z<1.1)&(X>0.0)&(X<0.35)&(Y>-0.15)&(Y<0.15); ext(pot,'pot')
stove=(Z>0.915)&(Z<0.95)&(Y>0.08)&(Y<0.32)&(X>-0.16)&(X<0.08); ext(stove,'stove')
"

# openrua op 39
timeout 900 python3 -u -c "
import math
from arm import Arm
a=Arm()
PSI=math.pi/2
T=(-0.052,0.375)
for z in [1.12,0.98]:
    print('reach', T, z, 'OK' if a.solve_tcp((T[0],T[1],z),PSI) else 'NO')
G=(-0.078,-0.05)
print('above handle'); a.move_tcp((G[0],G[1],1.05), PSI, 4.0)
print('descend'); a.move_tcp((G[0],G[1],0.94), PSI, 2.0); a.move_tcp((G[0],G[1],0.94), PSI, 1.0)
f=a.gripper(0.0); print('gap',f[0]-f[1])
print('lift'); a.move_tcp((G[0],G[1],1.12), PSI, 3.0)
f=a.fingers(); print('gap after lift',f[0]-f[1])
print('over stove'); a.move_tcp((T[0],T[1],1.12), PSI, 4.0); a.move_tcp((T[0],T[1],1.12), PSI, 1.5)
print('lower'); a.move_tcp((T[0],T[1],0.98), PSI, 3.0); a.move_tcp((T[0],T[1],0.98), PSI, 1.0)
f=a.fingers(); print('gap',f[0]-f[1])
" 2>&1 | grep -v "^\["; for c in agentview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c >/dev/null; done; echo ok

# openrua op 40
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "        R = down_R(psi)\\n        hand = np.asarray(tcp_xyz, float) - TCP_OFF * R[:, 2]\\n        seed = self.joints() if seed is None else seed\\n        for attempt in range(4):\\n            q = self.ik(hand, R, seed)\\n            if q is None:\\n                seed = list(np.array(seed) + np.random.uniform(-0.3, 0.3, 7))\\n                continue", "new_string": "        R = down_R(psi)\\n        hand = np.asarray(tcp_xyz, float) - TCP_OFF * R[:, 2]\\n        cur = self.joints()\\n        seed = cur if seed is None else seed\\n        best = None\\n        for attempt in range(6):\\n            q = self.ik(hand, R, seed)\\n            if q is None:\\n                seed = list(np.array(seed) + np.random.uniform(-0.3, 0.3, 7))\\n                continue", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 41
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "            if err < pos_tol and tilt < 3 and ok_lim and \\\\\\n                    abs((psi - yaw_act + math.pi) % (2 * math.pi) - math.pi) < math.radians(2):\\n                return q\\n            print(f\\"  solve_tcp attempt {attempt}: err={err*1000:.1f}mm tilt={tilt:.1f}deg \\"\\n                  f\\"yaw={math.degrees(yaw_act):.1f} lim_ok={ok_lim}; retrying\\")\\n            seed = list(np.array(q) + np.random.uniform(-0.3, 0.3, 7))\\n        return None", "new_string": "            if err < pos_tol and tilt < 3 and ok_lim and \\\\\\n                    abs((psi - yaw_act + math.pi) % (2 * math.pi) - math.pi) < math.radians(2):\\n                dist = max(abs(a - b) for a, b in zip(q, cur))\\n                if best is None or dist < best[0]:\\n                    best = (dist, q)\\n                if dist < 1.0:      # close to where we are: good enough\\n                    return q\\n            else:\\n                print(f\\"  solve_tcp attempt {attempt}: err={err*1000:.1f}mm tilt={tilt:.1f}deg \\"\\n                      f\\"yaw={math.degrees(yaw_act):.1f} lim_ok={ok_lim}; retrying\\")\\n            seed = list(np.array(cur) + np.random.uniform(-0.4, 0.4, 7))\\n        if best is not None:\\n            print(f\\"  solve_tcp: using solution {best[0]:.2f} rad from current\\")\\n            return best[1]\\n        return None", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 42
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    def move(self, q, seconds, tol=0.02, retries=1):\\n        goal = FollowJointTrajectory.Goal()", "new_string": "    def move(self, q, seconds, tol=0.02, retries=1):\\n        # slow down for big joint-space jumps (>= 2 s per rad of travel)\\n        dist = max(abs(a - b) for a, b in zip(q, self.joints()))\\n        seconds = max(seconds, 2.0 * dist)\\n        goal = FollowJointTrajectory.Goal()", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 43
timeout 900 python3 -u -c "
import math, sys
from arm import Arm
a=Arm()
PSI=math.pi/2
G=(-0.078,-0.05)
a.gripper(0.04)
print('lift clear'); a.move_tcp((-0.052,0.375,1.12), PSI, 2.0)
print('above handle'); a.move_tcp((G[0],G[1],1.05), PSI, 4.0); a.move_tcp((G[0],G[1],1.05), PSI, 1.5)
print('descend'); a.move_tcp((G[0],G[1],0.94), PSI, 2.0); a.move_tcp((G[0],G[1],0.94), PSI, 1.0)
f=a.gripper(0.0); gap=f[0]-f[1]; print('gap',gap)
if gap<0.015:
    print('GRASP FAILED'); a.gripper(0.04); sys.exit(1)
print('lift'); a.move_tcp((G[0],G[1],1.12), PSI, 3.0)
f=a.fingers(); print('gap after lift',f[0]-f[1])
" 2>&1 | grep -v "^\["; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null; echo ok

# openrua op 44
timeout 900 python3 -u -c "
import math
from arm import Arm
a=Arm()
PSI=math.pi/2
T=(-0.052,0.375)
print('over stove'); a.move_tcp((T[0],T[1],1.12), PSI, 4.0); a.move_tcp((T[0],T[1],1.12), PSI, 1.5)
f=a.fingers(); print('gap',f[0]-f[1])
print('lower'); a.move_tcp((T[0],T[1],0.99), PSI, 3.0); a.move_tcp((T[0],T[1],0.975), PSI, 1.5); a.move_tcp((T[0],T[1],0.975), PSI, 1.0)
f=a.fingers(); print('gap',f[0]-f[1])
" 2>&1 | grep -v "^\["; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force; for c in agentview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c >/dev/null; done; echo ok

# openrua op 45
timeout 900 python3 -u -c "
import math
from arm import Arm
a=Arm()
PSI=math.pi/2
T=(-0.052,0.375)
print('release'); a.gripper(0.04)
print('lift'); a.move_tcp((T[0],T[1],1.12), PSI, 3.0)
print('park'); a.move_tcp((-0.25,0.30,1.20), PSI, 4.0)
" 2>&1 | grep -v "^\["; for c in agentview birdview sideview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c >/dev/null; done; echo ok

# openrua op 46
timeout 60 python3 px_batch.py birdview 375,282 >/dev/null && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); w=d['world']
X,Y,Z=w[...,0],w[...,1],w[...,2]
def ext(mask,name):
    xs,ys,zs=X[mask],Y[mask],Z[mask]
    print(f'{name}: n={mask.sum()} x[{xs.min():.3f},{xs.max():.3f}] y[{ys.min():.3f},{ys.max():.3f}] z[{zs.min():.3f},{zs.max():.3f}] centroid=({xs.mean():.3f},{ys.mean():.3f},{zs.mean():.3f})')
pan=(Z>0.93)&(Z<1.0)&(Y>0.05)&(Y<0.33)&(X>-0.2)&(X<0.1); ext(pan,'pan on stove (all)')
rim=pan&(Z>0.965); ext(rim,'pan rim')
print('pan center est', ((X[rim].min()+X[rim].max())/2).round(3), ((Y[rim].min()+Y[rim].max())/2).round(3), ' stove square center (-0.058, 0.193)')
floor=pan&(Z<0.945); ext(floor,'pan inner floor (should be ~stove top 0.935 + pan bottom thickness)')
knob=(Z>0.94)&(Y>0.1)&(Y<0.28)&(X>-0.27)&(X<-0.17); ext(knob,'knob tab (z>0.94)')
print('tab extent: dx=%.3f dy=%.3f  (originally long along x; now long along y => rotated ~90deg)'%(X[knob].max()-X[knob].min(), Y[knob].max()-Y[knob].min()))
"

# openrua op 47
timeout 600 python3 -u -c "
import math
from arm import Arm
a=Arm()
a.move_tcp((-0.35,-0.25,1.25), 0.0, 4.0)
" 2>&1 | grep -v "^\["; timeout 60 python3 px_batch.py birdview 375,240 >/dev/null && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); w=d['world']
X,Y,Z=w[...,0],w[...,1],w[...,2]
knob=(Z>0.94)&(Z<1.0)&(Y>0.1)&(Y<0.28)&(X>-0.27)&(X<-0.16)
print('knob tab: n=%d x[%.3f,%.3f] y[%.3f,%.3f] ztop=%.3f'%(knob.sum(),X[knob].min(),X[knob].max(),Y[knob].min(),Y[knob].max(),Z[knob].max()))
print('tab extent dx=%.3f dy=%.3f (was dx=0.083 dy=0.025 before turning)'%(X[knob].max()-X[knob].min(), Y[knob].max()-Y[knob].min()))
"; timeout 60 python3 tools/perception/cam_snap.py agentview >/dev/null
