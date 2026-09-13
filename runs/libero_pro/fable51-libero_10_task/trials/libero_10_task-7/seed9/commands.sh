#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
ros2 topic list && echo --- && ros2 action list && echo --- && ros2 service list | head -50 && echo --- && ros2 node list

# openrua op 2
for c in agentview frontview birdview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; ros2 topic echo /joint_states --once

# openrua op 3
timeout 60 python3 tools/perception/px2world.py agentview 222 235; echo "ketchup ^"; timeout 60 python3 tools/perception/px2world.py agentview 347 270; echo "cream cheese ^"; timeout 60 python3 tools/perception/px2world.py agentview 535 300; echo "basket ^"; timeout 60 python3 tools/perception/px2world.py agentview 320 400; echo "table ^"; timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8

# openrua op 4
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump a world-frame point cloud from a camera and cluster objects above the table.
Usage: python3 scene.py <camera> [zmin_above_table]
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
import cv2


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
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer(); TransformListener(tfbuf, node)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    depth = CvBridge().imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = CvBridge().imgmsg_to_cv2(color_msg, "bgr8")
    frame = f"{cam}_optical_frame"
    while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    z = depth
    pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=-1)
    world = pc @ R.T + T
    np.save(f"{cam}_world.npy", world)
    # table plane estimate: mode of z among points in the central region
    zs = world[..., 2]
    valid = np.isfinite(zs) & (z > 0.05)
    hist, edges = np.histogram(zs[valid], bins=400, range=(0, 2))
    table_z = edges[np.argmax(hist)] + (edges[1] - edges[0]) / 2
    print(f"table_z ~ {table_z:.4f}")
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.01
    mask = valid & (zs > table_z + zmin) & (zs < table_z + 0.4)
    # exclude the robot: base frame at x=-0.51 world; robot occupies x < -0.3 roughly
    mask &= world[..., 0] > -0.35
    mask8 = mask.astype(np.uint8)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask8, 8)
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 40:
            continue
        pts = world[labels == i]
        lo, hi = pts.min(0), pts.max(0)
        col = color[labels == i].mean(0)
        print(f"obj{i}: px_centroid=({cents[i][0]:.0f},{cents[i][1]:.0f}) area={stats[i, cv2.CC_STAT_AREA]} "
              f"x[{lo[0]:.3f},{hi[0]:.3f}] y[{lo[1]:.3f},{hi[1]:.3f}] z[{lo[2]:.3f},{hi[2]:.3f}] "
              f"center=({(lo[0]+hi[0])/2:.3f},{(lo[1]+hi[1])/2:.3f}) bgr={col.astype(int)}")
    out = color.copy()
    out[mask] = (0.5 * out[mask] + [0, 127, 0]).astype(np.uint8)
    cv2.imwrite(f"{cam}_seg.png", out)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 5
timeout 120 python3 scene.py agentview 0.01; echo ---; timeout 120 python3 scene.py birdview 0.01

# openrua op 6
sed -i 's/    table_z = edges\[np.argmax(hist)\] + (edges\[1\] - edges\[0\]) \/ 2/    table_z = float(sys.argv[3]) if len(sys.argv) > 3 else edges[np.argmax(hist)] + (edges[1] - edges[0]) \/ 2/' scene.py && timeout 120 python3 scene.py birdview 0.01 0.4275

# openrua op 7
python3 -c "
import cv2
im=cv2.imread('agentview.png')
cv2.imwrite('crop_cc.png', cv2.resize(im[230:310,300:400],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
cv2.imwrite('crop_k.png', cv2.resize(im[170:270,180:270],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 8
mkdir -p "$(dirname /workspace/robot.py)"
cat > /workspace/robot.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable helpers for this Panda: FK, IK, trajectory, gripper, servo, sensing.
Poses are in the ARM BASE frame (panda_link0) unless noted. world = base + W2B.
"""
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
W2B = np.array([-0.51, 0.0, 0.42])  # world -> panda_link0 translation (from TF)


def world2base(p):
    return np.asarray(p, float) - W2B


def base2world(p):
    return np.asarray(p, float) + W2B


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_mul(a, b):  # (x,y,z,w)
    ax, ay, az, aw = a; bx, by, bz, bw = b
    return np.array([aw*bx + ax*bw + ay*bz - az*by,
                     aw*by - ax*bz + ay*bw + az*bx,
                     aw*bz + ax*by - ay*bx + az*bw,
                     aw*bw - ax*bx - ay*by - az*bz])


def down_quat(yaw):
    """Hand pointing straight down (hand +Z = -world Z), fingers closing along
    a horizontal axis rotated by `yaw` from the base Y axis."""
    rx = np.array([1.0, 0.0, 0.0, 0.0])  # 180 deg about X
    rz = np.array([0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)])
    return quat_mul(rz, rx)


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("robot_helper")
        self._js = {}
        self._wr = {}
        self.node.create_subscription(JointState, "/joint_states", lambda m: self._js.__setitem__("m", m), 1)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench",
                                      lambda m: self._wr.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    # ---- sensing ----
    def joints(self):
        self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def wrench(self):
        self._wr.pop("m", None)
        while "m" not in self._wr:
            self.spin(0.2)
        w = self._wr["m"].wrench
        return np.array([w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z])

    def _seed(self, q=None):
        js = JointState()
        js.name = list(ARM)
        js.position = [float(x) for x in (q if q is not None else self.arm_q())]
        return js

    def fk_pose(self, q=None, link="panda_hand"):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed: {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_pose(self, q=None):
        p, quat = self.fk_pose(q)
        R = quat_R(*quat)
        return p + TCP * R[:, 2], quat

    # ---- IK ----
    def ik_solve(self, pos, quat, seed=None, at_tcp=True, tries=3):
        pos = np.asarray(pos, float)
        if at_tcp:
            pos = pos - TCP * quat_R(*quat)[:, 2]
        seed_q = seed if seed is not None else self.arm_q()
        for k in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, pos)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state = self._seed(seed_q)
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout = Duration(sec=1)
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[j] for j in ARM]
            print(f"  IK attempt {k+1} failed: {r and r.error_code.val}", file=sys.stderr)
            seed_q = list(np.array(seed_q) + np.random.uniform(-0.3, 0.3, 7))
        return None

    # ---- motion ----
    def move_q(self, q, seconds=3.0, via=None):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = (via or []) + [q]
        for i, w in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(x) for x in w])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        print(f"  move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos, quat, seconds=3.0, seed=None):
        q = self.ik_solve(pos, quat, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for {pos}")
        code, err = self.move_q(q, seconds)
        p, _ = self.tcp_pose()
        print(f"  tcp now {p.round(4)} (target {np.asarray(pos).round(4)})")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=120)
        r = res.result().result
        print(f"  gripper: reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap():.4f}")
        return r

    def servo(self, lin, n=20, ang=(0, 0, 0)):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, lin)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, ang)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist.publish(msg)
            self.spin(0.05)
        stop = TwistStamped(); stop.header.frame_id = TW["frame"]
        for _ in range(3):
            self.twist.publish(stop); self.spin(0.05)
OPENRUA_EOF

# openrua op 9
timeout 120 python3 -c "
from robot import *
r = Robot()
print('q', np.round(r.arm_q(),3))
p,q = r.fk_pose(); print('hand base', p.round(4), q.round(4))
p,q = r.tcp_pose(); print('tcp base', p.round(4), 'world', base2world(p).round(4))
print('gap', r.finger_gap())
print('down_quat(0)', down_quat(0).round(4), 'down_quat(pi/2)', down_quat(np.pi/2).round(4))
print('wrench', r.wrench().round(2))
"

# openrua op 10
for p in "birdview 320 150" "birdview 315 280" "birdview 320 200" "agentview 320 160" "agentview 320 100" "agentview 300 140"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py $p; done; ros2 topic echo /tf_static --once 2>/dev/null | grep -A12 "child_frame_id: panda_link0" | head -14

# openrua op 11
timeout 60 python3 -c "
import numpy as np
def dh(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-sa*d],[st*sa,ct*sa,ca,ca*d],[0,0,0,1]])
def fk(q):
    A=[0,0,0,0.0825,-0.0825,0,0.088]; D=[0.333,0,0.316,0,0.384,0,0]; AL=[0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2]
    T=np.eye(4)
    for i in range(7): T=T@dh(A[i],D[i],AL[i],q[i])
    T=T@dh(0,0.107,0,0)  # flange
    T=T@dh(0,0,0,-np.pi/4) # hand
    return T
q=[0,-0.161037389,0,-2.44459747,0,2.2267522,0.7853981633974483]
T=fk(q); print('hand pos', T[:3,3].round(4)); print('hand z axis', T[:3,2].round(3)); print('hand y axis', T[:3,1].round(3))
q2=[0,-0.785,0,-2.356,0,1.571,0.785]; print('ready pose hand', fk(q2)[:3,3].round(4))
"


# openrua op 12
timeout 180 python3 -c "
from robot import *
import numpy as np
def dh(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-sa*d],[st*sa,ct*sa,ca,ca*d],[0,0,0,1]])
def fk(q):
    A=[0,0,0,0.0825,-0.0825,0,0.088]; D=[0.333,0,0.316,0,0.384,0,0]; AL=[0,-np.pi/2,np.pi/2,np.pi/2,-np.pi/2,np.pi/2,np.pi/2]
    T=np.eye(4)
    for i in range(7): T=T@dh(A[i],D[i],AL[i],q[i])
    return T@dh(0,0.107,0,0)@dh(0,0,0,-np.pi/4)
r=Robot()
for name,q in [('current',r.arm_q()),('ready',[0,-0.785,0,-2.356,0,1.571,0.785]),('zero',[0,0,0,-0.1,0,0.1,0])]:
    for link in ['panda_hand','panda_link8','panda_link0','panda_link1']:
        try:
            p,qq=r.fk_pose(q,link); print(name,link,'svc',p.round(4),qq.round(3))
        except Exception as e: print(name,link,'ERR',e)
    print(name,'dh hand',fk(q)[:3,3].round(4))
"


# openrua op 13
timeout 180 python3 -c "
from robot import *
r=Robot()
q0=np.array(r.arm_q())
p_w,quat=r.fk_pose()   # world-frame hand pose
p_b=world2base(p_w)
for name,p in [('world',p_w),('base',p_b)]:
    q=r.ik_solve(p,quat,at_tcp=False,tries=1)
    print(name, None if q is None else np.round(np.array(q)-q0,3))
"


# openrua op 14
timeout 180 python3 -c "
from robot import *
r=Robot()
q0=np.array(r.arm_q())
p_w,quat=r.fk_pose()
print('current', p_w.round(4), quat.round(4))
for k in range(3):
    q=r.ik_solve(p_w,quat,at_tcp=False,tries=1)
    p2,quat2=r.fk_pose(q); print('sol',k, np.round(np.array(q)-q0,3), p2.round(4), quat2.round(4))
"


# openrua op 15
python3 - <<'EOF'
import re
s=open('robot.py').read()
s=s.replace('''Poses are in the ARM BASE frame (panda_link0) unless noted. world = base + W2B.''','''Poses are in the WORLD frame (the FK/IK services on this machine answer in world;
panda_link0 sits at world W2B).''')
s=s.replace('''            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose''','''            req.ik_request.pose_stamped.header.frame_id = ""
            req.ik_request.ik_link_name = "panda_hand"
            p = req.ik_request.pose_stamped.pose''')
open('robot.py','w').write(s)
EOF
timeout 180 python3 -c "
from robot import *
r=Robot()
q0=np.array(r.arm_q())
p_w,quat=r.fk_pose()
q=r.ik_solve(p_w,quat,at_tcp=False,tries=1)
p2,quat2=r.fk_pose(q); print('sol', np.round(np.array(q)-q0,3), p2.round(4), quat2.round(4))
tp,_=r.tcp_pose(); print('tcp world', tp.round(4))
q=r.ik_solve(tp,quat,at_tcp=True,tries=1)
print('tcp ik diff', np.round(np.array(q)-q0,3))
"


# openrua op 16
timeout 900 python3 -u -c "
from robot import *
r=Robot()
K=np.array([-0.215,-0.157]); TABLE=0.4275
qk=down_quat(np.pi/2)   # fingers close along world x
print('open'); r.gripper(0.04)
print('pre-grasp'); q1=r.move_tcp([K[0],K[1],0.60], qk, 4.0)
print('descend'); q2=r.move_tcp([K[0],K[1],TABLE+0.045], qk, 3.0, seed=q1)
print('q now', np.round(r.arm_q(),3))
print('wrench', r.wrench().round(2))
" 2>&1 | tee step1.log

# openrua op 17
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 18
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "        goal.trajectory.points = pts\\n        send = self.fjt.send_goal_async(goal)\\n        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)\\n        res = send.result().get_result_async()\\n        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)\\n        code = res.result().result.error_code\\n        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()\\n        print(f\\"  move_q: error_code={code} max_joint_err={err:.4f}\\")\\n        return code, err", "new_string": "        goal.trajectory.points = pts\\n        for attempt in range(retries + 1):\\n            send = self.fjt.send_goal_async(goal)\\n            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)\\n            res = send.result().get_result_async()\\n            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)\\n            code = res.result().result.error_code\\n            err = np.abs(np.array(self.arm_q()) - np.array(q)).max()\\n            print(f\\"  move_q[{attempt}]: error_code={code} max_joint_err={err:.4f}\\")\\n            if err < tol:\\n                break\\n            # controller lag: re-send the remaining motion only (single point)\\n            goal.trajectory.points = [pts[-1]]\\n            t = max(2.0, seconds * min(1.0, err))\\n            pts[-1].time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))\\n        return code, err", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 19
sed -i 's/    def move_q(self, q, seconds=3.0, via=None):/    def move_q(self, q, seconds=3.0, via=None, retries=3, tol=0.02):/' robot.py && sed -i 's/        print(f"  tcp now {p.round(4)} (target {np.asarray(pos).round(4)})")/        _, qq = self.fk_pose()\n        print(f"  tcp now {p.round(4)} quat {qq.round(3)} (target {np.asarray(pos).round(4)} {np.asarray(quat).round(3)})")/' robot.py && grep -n "def move_q\|tcp now" robot.py && timeout 900 python3 -u -c "
from robot import *
r=Robot()
p,quat=r.fk_pose(); print('hand now', p.round(3), quat.round(3), 'wanted', down_quat(np.pi/2).round(3))
tp,_=r.tcp_pose()
# lift straight up keeping the CURRENT orientation
q=r.move_tcp([tp[0],tp[1],0.70], quat, 4.0)
print('wrench', r.wrench().round(2))
" 2>&1 | tee step2.log

# openrua op 20
timeout 120 python3 scene.py agentview 0.01 0.4275 2>&1 | grep -v "^obj1:"; echo ---; timeout 120 python3 scene.py birdview 0.01 0.4275

# openrua op 21
timeout 60 python3 tools/perception/cam_snap.py agentview a2.png; timeout 60 python3 tools/perception/cam_snap.py frontview f2.png; md5sum agentview.png a2.png

# openrua op 22
timeout 120 python3 scene.py birdview 0.012 0.4275 && python3 -c "
import cv2
im=cv2.imread('birdview.png')
cv2.imwrite('crop_b.png', cv2.resize(im[210:330,230:330],None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('a2.png')
cv2.imwrite('crop_a.png', cv2.resize(im[220:320,170:280],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 23
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e2.png >/dev/null; timeout 120 python3 -c "
import numpy as np, cv2
from robot import *
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); node=rclpy.create_node('x')
got={}
node.create_subscription(Image,'/agentview/color/image_raw',lambda m: got.setdefault('c',m),1)
node.create_subscription(Image,'/agentview/depth/image_raw',lambda m: got.setdefault('d',m),1)
while len(got)<2: rclpy.spin_once(node,timeout_sec=0.2)
col=CvBridge().imgmsg_to_cv2(got['c'],'bgr8'); dep=CvBridge().imgmsg_to_cv2(got['d'],'passthrough')
cv2.imwrite('a3.png',col)
# recompute world coords with same intrinsics/TF as scene.py (reuse saved world cloud only if depth same)
import subprocess
np.save('a3_depth.npy',dep)
hsv=cv2.cvtColor(col,cv2.COLOR_BGR2HSV)
# orange: hue ~ 5-20, high sat
m=(hsv[...,0]<22)&(hsv[...,1]>120)&(hsv[...,2]>60)
m[:200]=False  # exclude robot area
m[:, 400:]=False
ys,xs=np.nonzero(m); print('orange px', len(xs), 'bbox x',xs.min(),xs.max(),'y',ys.min(),ys.max())
cv2.imwrite('orange_mask.png', m.astype(np.uint8)*255)
"


# openrua op 24
timeout 120 python3 scene.py agentview 0.012 0.4275 >/dev/null && timeout 60 python3 -c "
import numpy as np, cv2
W=np.load('agentview_world.npy'); col=cv2.imread('agentview.png')
hsv=cv2.cvtColor(col,cv2.COLOR_BGR2HSV)
z=W[...,2]
above=np.isfinite(z)&(z>0.4275+0.012)&(z<0.6)&(W[...,0]>-0.35)&(W[...,0]<0.0)&(W[...,1]<-0.05)&(W[...,1]>-0.25)
# bright saturated orange-red for the bottle body
orange=(hsv[...,1]>150)&(hsv[...,2]>120)&((hsv[...,0]<15)|(hsv[...,0]>170))
m=above&orange
ys,xs=np.nonzero(m); print('n',len(xs),'px bbox x',xs.min(),xs.max(),'y',ys.min(),ys.max())
P=W[m]; print('x',P[:,0].min().round(3),P[:,0].max().round(3),'y',P[:,1].min().round(3),P[:,1].max().round(3),'z',P[:,2].min().round(3),P[:,2].max().round(3))
c=P.mean(0); print('mean',c.round(3))
# principal axis in xy
xy=P[:,:2]-c[:2]; u,s,vt=np.linalg.svd(xy,full_matrices=False); print('axis',vt[0].round(3),'sv',s.round(2))
# top ridge: highest points
top=P[P[:,2]>P[:,2].max()-0.012]; print('ridge mean',top.mean(0).round(3), 'n',len(top))
out=col.copy(); out[m]=(0,255,0); cv2.imwrite('ketch_mask.png',out[200:340,150:300])
"

# openrua op 25
timeout 60 python3 -c "
import numpy as np, cv2
W=np.load('agentview_world.npy'); col=cv2.imread('agentview.png')
hsv=cv2.cvtColor(col,cv2.COLOR_BGR2HSV)
z=W[...,2]
above=np.isfinite(z)&(z>0.4275+0.012)&(z<0.6)&(W[...,0]>-0.35)&(W[...,0]<0.0)&(W[...,1]<-0.05)&(W[...,1]>-0.25)
ys,xs=np.nonzero(above); print('above n',len(xs),'px bbox',xs.min(),xs.max(),ys.min(),ys.max())
print('hsv samples of above region:', hsv[above][::200][:15])
"

# openrua op 26
timeout 60 python3 -c "
import numpy as np, cv2
W=np.load('agentview_world.npy'); col=cv2.imread('a3.png')
hsv=cv2.cvtColor(col,cv2.COLOR_BGR2HSV)
z=W[...,2]
above=np.isfinite(z)&(z>0.4275+0.012)&(z<0.6)&(W[...,0]>-0.35)&(W[...,0]<0.0)&(W[...,1]<-0.05)&(W[...,1]>-0.25)
orange=(hsv[...,1]>150)&(hsv[...,2]>90)&((hsv[...,0]<15)|(hsv[...,0]>170))
m=above&orange
ys,xs=np.nonzero(m); print('n',len(xs),'px bbox x',xs.min(),xs.max(),'y',ys.min(),ys.max())
P=W[m]; print('x',P[:,0].min().round(3),P[:,0].max().round(3),'y',P[:,1].min().round(3),P[:,1].max().round(3),'z',P[:,2].min().round(3),P[:,2].max().round(3))
c=P.mean(0); print('mean',c.round(3))
xy=P[:,:2]-c[:2]; u,s,vt=np.linalg.svd(xy,full_matrices=False); print('axis',vt[0].round(3),'sv',s.round(2))
top=P[P[:,2]>P[:,2].max()-0.01]; print('ridge mean',top.mean(0).round(3), 'n',len(top))
# also the whole 'above' cluster (bottle incl cap)
Q=W[above]; print('all above: x',Q[:,0].min().round(3),Q[:,0].max().round(3),'y',Q[:,1].min().round(3),Q[:,1].max().round(3),'zmax',Q[:,2].max().round(3))
out=col.copy(); out[m]=(0,255,0); out[above&~m]=(255,0,0); cv2.imwrite('ketch_mask.png',cv2.resize(out[220:320,170:280],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 27
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/robot.py", "old_string": "            if r is not None and r.error_code.val == 1:\\n                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))\\n                return [sol[j] for j in ARM]\\n            print(f\\"  IK attempt {k+1} failed: {r and r.error_code.val}\\", file=sys.stderr)", "new_string": "            if r is not None and r.error_code.val == 1:\\n                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))\\n                q = [sol[j] for j in ARM]\\n                jump = np.abs(np.array(q) - np.array(seed_q)).max()\\n                if jump < max_jump or k == tries - 1:\\n                    return q\\n                print(f\\"  IK attempt {k+1}: branch jump {jump:.2f} rad, retrying\\", file=sys.stderr)\\n            else:\\n                print(f\\"  IK attempt {k+1} failed: {r and r.error_code.val}\\", file=sys.stderr)", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 28
sed -i 's/    def ik_solve(self, pos, quat, seed=None, at_tcp=True, tries=3):/    def ik_solve(self, pos, quat, seed=None, at_tcp=True, tries=5, max_jump=1.2):/' robot.py && timeout 1200 python3 -u -c "
from robot import *
r=Robot()
K=np.array([-0.135,-0.155]); TABLE=0.4275
qk=down_quat(0.0)
print('q now', np.round(r.arm_q(),3))
print('pre-grasp'); q1=r.move_tcp([K[0],K[1],0.60], qk, 5.0)
print('q1', np.round(q1,3))
" 2>&1 | tee step3.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e3.png

# openrua op 29
timeout 1200 python3 -u -c "
from robot import *
r=Robot()
K=np.array([-0.14,-0.155]); TABLE=0.4275
qk=down_quat(0.0)
print('stage1'); q1=r.move_tcp([K[0],K[1],0.50], qk, 3.0)
" 2>&1 | tee step4.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e4.png

# openrua op 30
timeout 1200 python3 -u -c "
from robot import *
r=Robot()
K=np.array([-0.14,-0.155]); TABLE=0.4275
qk=down_quat(0.0)
print('wrench before', r.wrench().round(2))
print('stage2'); q2=r.move_tcp([K[0],K[1],TABLE+0.02], qk, 3.0)
print('wrench at grasp height', r.wrench().round(2))
print('close'); r.gripper(0.0)
print('gap', r.finger_gap())
print('lift'); q3=r.move_tcp([K[0],K[1],0.62], qk, 3.0)
print('gap after lift', r.finger_gap(), 'wrench', r.wrench().round(2))
" 2>&1 | tee step5.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e5.png; timeout 60 python3 tools/perception/cam_snap.py agentview a5.png

# openrua op 31
timeout 1500 python3 -u -c "
from robot import *
r=Robot()
B=np.array([0.001,0.258])
qk=down_quat(np.pi/4)
tp,_=r.tcp_pose()
print('lift+yaw'); q1=r.move_tcp([tp[0],tp[1],0.74], qk, 4.0)
print('gap', r.finger_gap())
# bottle centre sits ~2.65cm along hand +x from the TCP
off=0.0265*np.array([np.cos(np.pi/4),np.sin(np.pi/4)])
tgt=B-off
print('translate'); q2=r.move_tcp([tgt[0],tgt[1],0.74], qk, 5.0, seed=q1)
print('gap', r.finger_gap())
" 2>&1 | tee step6.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e6.png; timeout 60 python3 tools/perception/cam_snap.py agentview a6.png

# openrua op 32
timeout 600 python3 -u -c "
from robot import *
r=Robot()
print('open'); r.gripper(0.04)
print('gap', r.finger_gap())
" 2>&1 | tee step7.log; timeout 60 python3 tools/perception/cam_snap.py agentview a7.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e7.png

# openrua op 33
timeout 1500 python3 -u -c "
from robot import *
r=Robot()
C=np.array([-0.132,0.043])
qk=down_quat(0.0)
tp,_=r.tcp_pose()
print('pre-grasp'); q1=r.move_tcp([C[0],C[1],0.60], qk, 5.0)
print('gap', r.finger_gap())
" 2>&1 | tee step8.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e8.png

# openrua op 34
timeout 1500 python3 -u -c "
from robot import *
r=Robot()
C=np.array([-0.132,0.043]); TABLE=0.4275
qk=down_quat(0.0)
print('stage1'); q1=r.move_tcp([C[0],C[1],0.50], qk, 3.0)
" 2>&1 | tee step9.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e9.png

# openrua op 35
timeout 1500 python3 -u -c "
from robot import *
r=Robot()
C=np.array([-0.132,0.043]); TABLE=0.4275
qk=down_quat(0.0)
print('stage2'); q2=r.move_tcp([C[0],C[1],TABLE+0.012], qk, 3.0)
print('wrench', r.wrench().round(2))
print('close'); r.gripper(0.0)
print('lift'); q3=r.move_tcp([C[0],C[1],0.60], qk, 3.0)
print('gap after lift', r.finger_gap(), 'wrench', r.wrench().round(2))
" 2>&1 | tee step10.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e10.png; timeout 60 python3 tools/perception/cam_snap.py agentview a10.png

# openrua op 36
timeout 1500 python3 -u -c "
from robot import *
r=Robot()
B=np.array([0.001,0.258])
qk=down_quat(0.0)
tp,_=r.tcp_pose()
print('lift'); q1=r.move_tcp([tp[0],tp[1],0.74], qk, 3.0)
print('translate'); q2=r.move_tcp([B[0],B[1],0.74], qk, 5.0, seed=q1)
print('gap', r.finger_gap())
" 2>&1 | tee step11.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e11.png

# openrua op 37
timeout 600 python3 -u -c "
from robot import *
r=Robot()
print('open'); r.gripper(0.04)
tp,_=r.tcp_pose()
print('retreat up'); r.move_tcp([tp[0],tp[1]-0.15,0.80], down_quat(0.0), 4.0)
" 2>&1 | tee step12.log; for c in agentview frontview robot0_robotview sideview; do timeout 60 python3 tools/perception/cam_snap.py $c ${c}_final.png; done

# openrua op 38
timeout 120 python3 scene.py agentview 0.012 0.4275 | grep -v "^obj1:"; timeout 600 python3 -u -c "
from robot import *
r=Robot()
r.move_tcp([0.001,0.258,0.85], down_quat(0.0), 4.0)
" 2>&1 | tail -1; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand e_final.png

# openrua op 39
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-workstation-quirks
description: Non-obvious facts about the /workspace Franka Panda sim (FK/IK frame, IK tip link, trajectory controller lag) learned while doing pick-and-place
metadata:
  type: project
---

Learned 2026-09-12 on the /workspace Panda workstation (machine.yaml, MoveIt):

- `/compute_fk` and `/compute_ik` answer/accept poses in the **world** frame
  (panda_link0 sits at world (-0.51, 0, 0.42)), despite docs saying "planner
  frame is the arm base". Base-frame IK requests fail with -31. Leave frame_id "".
- IK defaults to tip link `panda_link8`, which is yawed 45° from `panda_hand`.
  Set `ik_request.ik_link_name = "panda_hand"` or solutions come back rotated.
- The FJT controller often returns error -5 (goal tolerance violated) with a
  joint still ~1 rad short, especially on large joint-7 rotations. Re-send the
  final point until `/joint_states` matches (tol ~0.02 rad); don't chain moves
  off the intended IK solution without checking the actual state first.
- Camera frames (birdview especially) can be stale after motion; md5 the PNG or
  re-snap. Agentview + eye-in-hand were reliably fresh.
- Table top is world z≈0.4275; TCP at table+0.012 grasps a 3 cm box fine.

**Why:** the first ketchup grasp knocked the bottle over because of the
link8/hand yaw mismatch plus controller lag.
**How to apply:** reuse /workspace/robot.py (world-frame helpers with retries)
if it still exists; otherwise rebuild with the above conventions.
OPENRUA_EOF

# openrua op 40
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda workstation quirks](panda-workstation-quirks.md) — FK/IK are world-frame, IK tip is link8 (set ik_link_name=panda_hand), FJT returns -5 with lag; re-send" >> MEMORY.md && cat MEMORY.md
