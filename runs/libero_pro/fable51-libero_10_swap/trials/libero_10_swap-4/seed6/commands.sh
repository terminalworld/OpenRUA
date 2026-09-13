#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -50

# openrua op 1
cat /workspace/tools/README.md; find /workspace/tools -type f; ls -la /workspace/.ros

# openrua op 2
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 3
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 node list

# openrua op 5
timeout 60 ros2 topic echo /joint_states --once; for c in agentview birdview frontview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 60 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; timeout 30 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|x:|y:|z:|w:" | head -80

# openrua op 7
for p in "195 230" "195 265" "95 290" "540 290" "335 195" "335 215" "410 240" "320 380"; do echo -n "agentview $p -> "; timeout 60 python3 tools/perception/px2world.py agentview $p; done

# openrua op 8
for p in "248 283" "390 283" "275 270" "320 240" "348 275" "320 320" "200 200"; do echo -n "birdview $p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 9
timeout 60 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && python3 -c "
import numpy as np
d=np.load('birdview_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d))
# table depth
print('center', d[320,320], d[283,248], d[283,390], d[270,275], d[240,320], d[275,348])
"

# openrua op 10
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Project a camera's depth frame to world points; cluster things above the table.

Usage: python3 cloud.py <camera> [zmin]
"""
import sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from scipy import ndimage


def grab(node, topic, mt, timeout=20.0):
    got = {}
    sub = node.create_subscription(mt, topic, lambda m: got.setdefault("m", m), 1)
    import time
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]


def quat_T(t):
    q = t.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    T = np.eye(4); T[:3, :3] = R
    T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
    return T


def main():
    cam = sys.argv[1]
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.44
    rclpy.init()
    node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    depth = CvBridge().imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    t0 = time.time()
    while time.time() - t0 < 10 and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    T = quat_T(buf.lookup_transform("world", frame, rclpy.time.Time()))
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    v, u = np.mgrid[0:H, 0:W]
    z = depth
    X = (u - cx) * z / fx; Y = (v - cy) * z / fy
    P = np.stack([X, Y, z, np.ones_like(z)], -1) @ T.T
    wx, wy, wz = P[..., 0], P[..., 1], P[..., 2]
    np.save(f"{cam}_world.npy", P[..., :3])
    mask = np.isfinite(z) & (wz > zmin) & (wz < 0.9) & (np.abs(wx) < 0.6) & (np.abs(wy) < 0.7)
    lab, n = ndimage.label(mask)
    print(f"{n} clusters above z={zmin}")
    for i in range(1, n + 1):
        m = lab == i
        if m.sum() < 15:
            continue
        us, vs = u[m], v[m]
        print(f"cluster {i}: npx={m.sum()} px u[{us.min()}-{us.max()}] v[{vs.min()}-{vs.max()}] "
              f"x[{wx[m].min():.3f},{wx[m].max():.3f}] y[{wy[m].min():.3f},{wy[m].max():.3f}] "
              f"z[{wz[m].min():.3f},{wz[m].max():.3f}] centroid=({wx[m].mean():.3f},{wy[m].mean():.3f},{wz[m].mean():.3f})")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 11
timeout 120 python3 cloud.py birdview 0.44; echo ---; timeout 120 python3 cloud.py agentview 0.44

# openrua op 12
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 60 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 13
timeout 120 python3 cloud.py agentview 0.435; echo ---; timeout 120 python3 cloud.py birdview 0.435; for p in "95 330" "540 330" "320 400"; do echo -n "agentview table $p -> "; timeout 60 python3 tools/perception/px2world.py agentview $p; done

# openrua op 14
timeout 120 python3 cloud.py robot0_eye_in_hand 0.435 2>&1 | tail -20

# openrua op 15
python3 -c "
import numpy as np
from scipy import ndimage
for cam in ['robot0_eye_in_hand','agentview']:
    P=np.load(f'{cam}_world.npy'); x,y,z=P[...,0],P[...,1],P[...,2]
    print(cam)
    for name,(x0,x1,y0,y1) in {'white':(-0.2,0.05,-0.26,-0.09),'red':(-0.15,0.05,0.03,0.2),'yellow':(-0.3,-0.15,-0.08,0.1)}.items():
        m=np.isfinite(z)&(x>x0)&(x<x1)&(y>y0)&(y<y1)&(z>0.46)
        if m.sum()==0: print(name,'none'); continue
        ztop=np.percentile(z[m],98)
        rim=m&(z>ztop-0.015)
        print(f' {name}: n={m.sum()} ztop={ztop:.3f} rim n={rim.sum()} x[{x[rim].min():.3f},{x[rim].max():.3f}] y[{y[rim].min():.3f},{y[rim].max():.3f}] rimctr=({x[rim].mean():.3f},{y[rim].mean():.3f})')
        # body at mid height
        mid=m&(z>0.49)&(z<0.51)
        if mid.sum(): print(f'   mid: x[{x[mid].min():.3f},{x[mid].max():.3f}] y[{y[mid].min():.3f},{y[mid].max():.3f}]')
"


# openrua op 16
timeout 30 ros2 service list | grep -iE "fk|ik|state"; timeout 20 ros2 interface show moveit_msgs/srv/GetPositionFK | head -30

# openrua op 17
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library for this Panda: joint state, FK, IK, trajectory,
gripper, and world<->base frame conversion. Poses handed to IK/FK are in
the planner model frame (panda_link0); world_to_base/base_to_world convert.
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
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
TCP = M["hand"]["tcp_offset_m"]
BASE_IN_WORLD = np.array([-0.510, 0.0, 0.420])  # from tf2_echo world panda_link0


def world_to_base(p):
    return np.asarray(p, float) - BASE_IN_WORLD


def base_to_world(p):
    return np.asarray(p, float) + BASE_IN_WORLD


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
        s = np.sqrt(t + 1) * 2
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def down_quat(yaw=0.0):
    """Hand pointing straight down (hand +Z = world -Z); fingers close
    along the hand Y axis, rotated by `yaw` about world Z. yaw=0 -> hand
    Y along world Y (the home-configuration orientation)."""
    # hand X -> world X, hand Y -> world -Y, hand Z -> world -Z (a 180deg
    # flip about X), then yaw about world Z.
    Rflip = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)
    c, s = np.cos(yaw), np.sin(yaw)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return R_to_quat(Rz @ Rflip)


class Arm:
    def __init__(self, name="arm_helper"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.tw_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js.pop("m", None)
        t0 = time.time()
        while "m" not in self._js and time.time() - t0 < 30:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return np.array([j[n] for n in JOINTS])

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"] - j["panda_finger_joint2"]

    def fk(self, q=None, link="panda_hand"):
        """Return (pos, quat) of link in the base frame."""
        if q is None:
            q = self.arm_q()
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            raise RuntimeError(f"FK failed: {None if res is None else res.error_code.val}")
        p = res.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]))

    def tcp_world(self, q=None):
        """World position of the fingertip point (TCP) and the hand quat."""
        p, quat = self.fk(q)
        R = quat_to_R(quat)
        return base_to_world(p + TCP * R[:, 2]), quat

    # ---------- planning ----------
    def ik(self, pos_base, quat, seed=None, at_tcp=True, timeout=60):
        """IK for the hand (or TCP if at_tcp) at pos_base (base frame)."""
        pos = np.asarray(pos_base, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        if seed is None:
            seed = self.arm_q()
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(JOINTS)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timed out")
        if res.error_code.val != 1:
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([sol[j] for j in JOINTS])

    def ik_world(self, pos_world, quat, **kw):
        return self.ik(world_to_base(pos_world), quat, **kw)

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None):
        """Execute a trajectory to q (optionally through `via` list of
        (q, t) waypoints). Returns error_code."""
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        pts = []
        for qq, t in (via or []):
            pt = JointTrajectoryPoint(positions=[float(v) for v in qq])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(self.arm_q() - np.asarray(q))
        print(f"  traj error_code={code} max joint err={err.max():.4f}", flush=True)
        return code

    def move_tcp_world(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik_world(pos_world, quat, seed=seed)
        if q is None:
            raise RuntimeError(f"IK failed for TCP world {pos_world}")
        code = self.move_q(q, seconds)
        tcp, _ = self.tcp_world()
        print(f"  TCP world now {tcp.round(4)} (target {np.round(pos_world, 4)})", flush=True)
        return q

    def gripper(self, width):
        self.grip.wait_for_server(timeout_sec=10)
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper reached={r.reached_goal} stalled={r.stalled} gap={gap:.4f}", flush=True)
        return gap

    def servo(self, v_world, ticks=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v_world)
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.tw_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=dt)
        stop = TwistStamped()
        stop.header.frame_id = TW["frame"]
        for _ in range(3):
            self.tw_pub.publish(stop)
            rclpy.spin_once(self.node, timeout_sec=dt)


if __name__ == "__main__":
    a = Arm()
    q = a.arm_q()
    print("q =", q.round(4))
    p, quat = a.fk(q)
    print("hand base pos", p.round(4), "quat", quat.round(4))
    tcp, _ = a.tcp_world()
    print("TCP world", tcp.round(4))
    print("down_quat(0) =", down_quat(0).round(4))
    print("finger gap", a.finger_gap())
OPENRUA_EOF

# openrua op 18
timeout 120 python3 arm.py

# openrua op 19
mkdir -p "$(dirname /workspace/step_pregrasp.py)"
cat > /workspace/step_pregrasp.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Move TCP to a world position with a down-pointing hand. Usage:
python3 step_pregrasp.py x y z [yaw_deg] [seconds]"""
import sys
import numpy as np
from arm import Arm, down_quat

x, y, z = map(float, sys.argv[1:4])
yaw = np.deg2rad(float(sys.argv[4])) if len(sys.argv) > 4 else 0.0
secs = float(sys.argv[5]) if len(sys.argv) > 5 else 4.0
a = Arm("pregrasp")
print("before TCP world", a.tcp_world()[0].round(4), flush=True)
q = a.move_tcp_world([x, y, z], down_quat(yaw), seconds=secs)
print("q now", a.arm_q().round(4))
OPENRUA_EOF

# openrua op 20
timeout 900 python3 -u step_pregrasp.py -0.080 -0.225 0.66 0 4 2>&1 | tail -20

# openrua op 21
cat > /workspace/ik_test.py <<'EOF'
import sys, numpy as np
from arm import Arm, down_quat, world_to_base
a = Arm("iktest")
x,y,z = map(float, sys.argv[1:4])
seeds = {"cur": a.arm_q(), "ready": np.array([0,-0.785,0,-2.356,0,1.571,0.785]),
         "ready2": np.array([-0.4,-0.3,0,-2.2,0,1.9,0.4]), "ready3": np.array([0.0,0.2,0,-2.0,0,2.2,0.785])}
for n,s in seeds.items():
    q = a.ik(world_to_base([x,y,z]), down_quat(0), seed=s)
    print(n, None if q is None else q.round(3), flush=True)
    if q is not None:
        print("   check tcp:", a.tcp_world(q)[0].round(4))
EOF
timeout 600 python3 -u ik_test.py -0.080 -0.225 0.66

# openrua op 22
cat > /workspace/ik_test2.py <<'EOF'
import sys, numpy as np
from arm import Arm, down_quat, world_to_base, base_to_world
a = Arm("iktest2")
q0 = a.arm_q()
p, quat = a.fk(q0)
print("fk hand", p.round(4), quat.round(4))
q = a.ik(p, quat, seed=q0, at_tcp=False)
print("ik at current hand pose:", None if q is None else q.round(3))
# slightly lower
q = a.ik(p - [0,0,0.1], quat, seed=q0, at_tcp=False)
print("ik 10cm lower:", None if q is None else q.round(3))
q = a.ik([0.3,0,0.5], down_quat(0), seed=q0, at_tcp=False)
print("ik std pose:", None if q is None else q.round(3))
q = a.ik([0.3,0,0.5], quat, seed=q0, at_tcp=False)
print("ik std pose cur quat:", None if q is None else q.round(3))
EOF
timeout 600 python3 -u ik_test2.py

# openrua op 23
cat > /workspace/fk_test.py <<'EOF'
from arm import Arm
a = Arm("fktest")
for l in ["panda_link0", "panda_link8", "panda_hand", "panda_leftfinger", "panda_rightfinger"]:
    try:
        p, q = a.fk(link=l); print(l, p.round(4), q.round(4))
    except Exception as e: print(l, "ERR", e)
EOF
timeout 300 python3 -u fk_test.py

# openrua op 24
python3 - <<'EOF'
import re
s = open("arm.py").read()
s = s.replace('BASE_IN_WORLD = np.array([-0.510, 0.0, 0.420])  # from tf2_echo world panda_link0',
 'BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # FK of panda_link0 returns (-0.51,0,0.42):\n# the planner model frame IS the world frame on this machine (docs say otherwise)')
s = s.replace('''        pos = np.asarray(pos_base, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(quat)[:, 2]''',
'''        pos = np.asarray(pos_base, float)
        if at_tcp:
            pos = pos - TCP * quat_to_R(quat)[:, 2]
        # IK tip link is panda_link8, which is panda_hand rotated -45deg about
        # its Z (tf_static). Convert the requested HAND quat to a link8 quat.
        c, s_ = np.cos(np.pi / 4), np.sin(np.pi / 4)
        Rz45 = np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
        quat = R_to_quat(quat_to_R(quat) @ Rz45)''')
open("arm.py","w").write(s)
EOF
timeout 600 python3 -u ik_test2.py

# openrua op 25
timeout 600 python3 -u ik_test.py -0.080 -0.225 0.66

# openrua op 26
timeout 900 python3 -u step_pregrasp.py -0.080 -0.225 0.66 0 4 2>&1 | tail -8 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 27
timeout 120 python3 cloud.py robot0_eye_in_hand 0.50 2>&1 | tail -5; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); x,y,z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(z)&(x>-0.2)&(x<0.05)&(y>-0.3)&(y<-0.09)&(z>0.53)
print('rim n',m.sum(),'ztop',np.percentile(z[m],99))
xs,ys=x[m],y[m]
# fit circle to rim points (exclude handle side y>-0.13)
sel=ys<-0.125
A=np.c_[2*xs[sel],2*ys[sel],np.ones(sel.sum())]; b=xs[sel]**2+ys[sel]**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print(f'circle fit: center=({cx:.4f},{cy:.4f}) r={r:.4f}')
print('x range',xs.min(),xs.max(),'y range',ys.min(),ys.max())
"

# openrua op 28
python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); x,y,z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(z)&(x>-0.2)&(x<0.05)&(y>-0.3)&(y<-0.09)&(z>0.535)&(z<0.56)
print('rim n',m.sum(),'ztop',np.percentile(z[m],99))
xs,ys=x[m],y[m]
sel=ys<-0.125
A=np.c_[2*xs[sel],2*ys[sel],np.ones(sel.sum())]; b=xs[sel]**2+ys[sel]**2
c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
print(f'circle fit: center=({cx:.4f},{cy:.4f}) r={r:.4f}')
print('x range',xs.min(),xs.max(),'y range',ys.min(),ys.max())
# also handle location
h=m&(ys.min()<y)&(y>-0.125)
print('handle pts y range', y[h].min() if h.sum() else None, y[h].max() if h.sum() else None, 'x', x[h].mean() if h.sum() else None)
"

# openrua op 29
cat > /workspace/step_grasp.py <<'EOF'
"""Descend to TCP (x,y,z) in two steps, close the gripper, report the gap, lift to zlift."""
import sys, numpy as np
from arm import Arm, down_quat
x, y, z, zlift = map(float, sys.argv[1:5])
yaw = np.deg2rad(float(sys.argv[5])) if len(sys.argv) > 5 else 0.0
a = Arm("grasp")
qd = down_quat(yaw)
print("start TCP", a.tcp_world()[0].round(4), "gap", round(a.finger_gap(), 4), flush=True)
a.move_tcp_world([x, y, z + 0.05], qd, seconds=2.5)
a.move_tcp_world([x, y, z], qd, seconds=2.0)
gap = a.gripper(0.0)
print("GAP after close:", round(gap, 4), flush=True)
a.move_tcp_world([x, y, zlift], qd, seconds=2.5)
print("gap after lift:", round(a.finger_gap(), 4), flush=True)
EOF
timeout 1500 python3 -u step_grasp.py -0.077 -0.218 0.522 0.70 2>&1 | tail -12; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 30
cat > /workspace/step_place.py <<'EOF'
"""Carry to (x,y) at zcarry, descend to zplace, open, lift back."""
import sys, numpy as np
from arm import Arm, down_quat
x, y, zcarry, zplace = map(float, sys.argv[1:5])
yaw = np.deg2rad(float(sys.argv[5])) if len(sys.argv) > 5 else 0.0
a = Arm("place")
qd = down_quat(yaw)
print("start TCP", a.tcp_world()[0].round(4), "gap", round(a.finger_gap(), 4), flush=True)
a.move_tcp_world([x, y, zcarry], qd, seconds=4.0)
print("gap after carry:", round(a.finger_gap(), 4), flush=True)
a.move_tcp_world([x, y, zplace + 0.04], qd, seconds=2.5)
a.move_tcp_world([x, y, zplace], qd, seconds=2.0)
a.gripper(0.04)
a.move_tcp_world([x, y, zcarry], qd, seconds=2.5)
print("done; TCP", a.tcp_world()[0].round(4), flush=True)
EOF
timeout 1500 python3 -u step_place.py -0.020 -0.345 0.70 0.56 2>&1 | tail -14; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 31
timeout 900 python3 -u step_pregrasp.py -0.218 0.075 0.66 0 4 2>&1 | tail -4 && timeout 120 python3 cloud.py robot0_eye_in_hand 0.50 2>&1 | tail -4; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); x,y,z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(z)&(x>-0.32)&(x<-0.12)&(y>-0.1)&(y<0.15)&(z>0.50)&(z<0.56)
print('n',m.sum(),'ztop',np.percentile(z[m],99))
xs,ys=x[m],y[m]
print('x range',xs.min(),xs.max(),'y range',ys.min(),ys.max())
top=np.percentile(z[m],99)
r_=m&(z>top-0.012)
xs,ys=x[r_],y[r_]
for ycut in [0.0, 0.01, 0.02]:
    sel=ys>ycut
    A=np.c_[2*xs[sel],2*ys[sel],np.ones(sel.sum())]; b=xs[sel]**2+ys[sel]**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
    print(f'ycut {ycut}: circle center=({cx:.4f},{cy:.4f}) r={r:.4f} n={sel.sum()}')
"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 32
timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 33
cat > /workspace/ik_test3.py <<'EOF'
import sys, numpy as np
from arm import Arm, down_quat
a = Arm("iktest3")
x,y,z = map(float, sys.argv[1:4])
q0 = a.arm_q(); print("cur q", q0.round(3))
for n,s in {"cur": q0, "home": np.array([0,-0.161,0,-2.445,0,2.227,0.785]), "ready": np.array([0,-0.785,0,-2.356,0,1.571,0.785])}.items():
    q = a.ik([x,y,z], down_quat(0), seed=s)
    print(n, None if q is None else q.round(3), flush=True)
EOF
timeout 600 python3 -u ik_test3.py -0.218 0.075 0.66

# openrua op 34
cat > /workspace/step_moveq.py <<'EOF'
import sys, numpy as np
from arm import Arm
a = Arm("moveq")
q = np.array([float(v) for v in sys.argv[1].split(",")]); secs = float(sys.argv[2])
code = a.move_q(q, secs)
print("TCP", a.tcp_world()[0].round(4), "q", a.arm_q().round(3))
EOF
timeout 900 python3 -u step_moveq.py 0,-0.161,0,-2.445,0,2.227,0.785 5 2>&1 | tail -3

# openrua op 35
timeout 900 python3 -u step_moveq.py 0.151,-0.688,0.115,-2.941,0.094,2.254,0.966 6 2>&1 | tail -3

# openrua op 36
timeout 120 python3 cloud.py robot0_eye_in_hand 0.50 2>&1 | tail -4; python3 -c "
import numpy as np
P=np.load('robot0_eye_in_hand_world.npy'); x,y,z=P[...,0],P[...,1],P[...,2]
m=np.isfinite(z)&(x>-0.32)&(x<-0.12)&(y>-0.1)&(y<0.15)&(z>0.50)&(z<0.56)
print('n',m.sum(),'ztop',np.percentile(z[m],99))
xs,ys=x[m],y[m]
print('x range',xs.min(),xs.max(),'y range',ys.min(),ys.max())
top=np.percentile(z[m],99)
r_=m&(z>top-0.012)
xs,ys=x[r_],y[r_]
for ycut in [0.0, 0.01, 0.02]:
    sel=ys>ycut
    A=np.c_[2*xs[sel],2*ys[sel],np.ones(sel.sum())]; b=xs[sel]**2+ys[sel]**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; cx,cy=c[0],c[1]; r=np.sqrt(c[2]+cx**2+cy**2)
    print(f'ycut {ycut}: circle center=({cx:.4f},{cy:.4f}) r={r:.4f} n={sel.sum()}')
"; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 37
timeout 600 python3 -u ik_test3.py -0.2027 0.0745 0.514

# openrua op 38
cat > /workspace/step_grasp2.py <<'EOF'
"""Straight vertical descent via chained IK waypoints, close, lift."""
import sys, numpy as np
from arm import Arm, down_quat
x, y, z, zlift = map(float, sys.argv[1:5])
a = Arm("grasp2")
qd = down_quat(0)
tcp0, _ = a.tcp_world(); q = a.arm_q()
print("start TCP", tcp0.round(4), flush=True)
zs = list(np.arange(tcp0[2] - 0.02, z, -0.02)) + [z]
via, qs = [], []
t = 0.0
for zz in zs:
    q = a.ik([x, y, zz], qd, seed=q)
    if q is None: raise SystemExit(f"IK failed at z={zz}")
    qs.append(q)
# check smoothness of the chain: joint deltas and FK of midpoints
for i in range(1, len(qs)):
    mid = (qs[i-1] + qs[i]) / 2
    tm, _ = a.tcp_world(mid)
    print(f"  wp z={zs[i]:.3f} dq_max={np.abs(qs[i]-qs[i-1]).max():.3f} mid TCP {tm.round(3)}", flush=True)
if np.abs(np.diff(qs, axis=0)).max() > 0.5:
    raise SystemExit("joint chain not smooth; aborting")
dt = 1.0
via = [(qq, dt * (i + 1)) for i, qq in enumerate(qs[:-1])]
a.move_q(qs[-1], dt * len(qs), via=via)
print("TCP at grasp", a.tcp_world()[0].round(4), flush=True)
gap = a.gripper(0.0)
print("GAP after close:", round(gap, 4), flush=True)
# lift the same way (reverse the chain) then up to zlift
q = a.arm_q()
qlift = a.ik([x, y, zlift], qd, seed=qs[0])
if qlift is None: raise SystemExit("IK failed for lift")
rev = list(reversed(qs[:-1]))
via = [(qq, dt * (i + 1)) for i, qq in enumerate(rev)]
a.move_q(qlift, dt * (len(rev) + 2), via=via)
print("TCP after lift", a.tcp_world()[0].round(4), "gap", round(a.finger_gap(), 4), flush=True)
EOF
timeout 1500 python3 -u step_grasp2.py -0.2027 0.0745 0.514 0.70 2>&1 | tail -20; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 39
cat > /workspace/step_place2.py <<'EOF'
"""Carry to (x,y,zcarry) (2-3 waypoints), chained-IK descent to zplace, open, lift."""
import sys, numpy as np
from arm import Arm, down_quat
x, y, zcarry, zplace = map(float, sys.argv[1:5])
a = Arm("place2")
qd = down_quat(0)
tcp0, _ = a.tcp_world(); q0 = a.arm_q()
print("start TCP", tcp0.round(4), "gap", round(a.finger_gap(), 4), flush=True)
# carry: interpolate in cartesian in 3 segments, chained IK
pts = [tcp0 + (np.array([x, y, zcarry]) - tcp0) * s for s in (0.34, 0.67, 1.0)]
pts[0][2] = pts[1][2] = zcarry
qs, q = [], q0
for p in pts:
    q = a.ik(p, qd, seed=q)
    if q is None: raise SystemExit(f"IK failed at {p}")
    qs.append(q)
prev = q0
for i, qq in enumerate(qs):
    mid = (prev + qq) / 2; tm, _ = a.tcp_world(mid)
    print(f"  carry wp {pts[i].round(3)} dq_max={np.abs(qq-prev).max():.3f} mid TCP {tm.round(3)}", flush=True)
    prev = qq
via = [(qq, 2.0 * (i + 1)) for i, qq in enumerate(qs[:-1])]
a.move_q(qs[-1], 2.0 * len(qs), via=via)
print("TCP after carry", a.tcp_world()[0].round(4), "gap", round(a.finger_gap(), 4), flush=True)
# descend
q = a.arm_q()
zs = list(np.arange(zcarry - 0.02, zplace, -0.02)) + [zplace]
qs = []
for zz in zs:
    q = a.ik([x, y, zz], qd, seed=q)
    if q is None: raise SystemExit(f"IK failed at z={zz}")
    qs.append(q)
if np.abs(np.diff(qs, axis=0)).max() > 0.5: raise SystemExit("descent chain not smooth")
via = [(qq, 1.0 * (i + 1)) for i, qq in enumerate(qs[:-1])]
a.move_q(qs[-1], 1.0 * len(qs), via=via)
print("TCP at place", a.tcp_world()[0].round(4), flush=True)
a.gripper(0.04)
rev = list(reversed(qs[:-1]))
via = [(qq, 1.0 * (i + 1)) for i, qq in enumerate(rev)]
a.move_q(rev[-1] if rev else qs[0], 1.0 * len(rev), via=via[:-1])
print("done; TCP", a.tcp_world()[0].round(4), flush=True)
EOF
timeout 1500 python3 -u step_place2.py -0.010 0.3445 0.74 0.552 2>&1 | tail -20; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 40
timeout 120 python3 cloud.py birdview 0.44 2>&1 | grep -v "^[0-9]* clusters"; echo ---; timeout 120 python3 cloud.py agentview 0.44 2>&1 | grep -v "^[0-9]* clusters"; timeout 60 python3 tools/perception/cam_snap.py birdview

# openrua op 41
cat > /workspace/check.py <<'EOF'
import numpy as np, sys
cam = sys.argv[1] if len(sys.argv) > 1 else 'agentview'
P=np.load(f'{cam}_world.npy'); x,y,z=P[...,0],P[...,1],P[...,2]
def fit(xs, ys):
    A=np.c_[2*xs,2*ys,np.ones(len(xs))]; b=xs**2+ys**2
    c=np.linalg.lstsq(A,b,rcond=None)[0]; return c[0],c[1],np.sqrt(c[2]+c[0]**2+c[1]**2)
for name,(y0,y1,handle_sign) in {'white/left':(-0.42,-0.18,+1),'yellow/right':(0.18,0.42,-1)}.items():
    reg=np.isfinite(z)&(x>-0.2)&(x<0.15)&(y>y0)&(y<y1)
    top=np.percentile(z[reg&(z>0.5)],99)
    rim=reg&(z>top-0.012)
    xs,ys=x[rim],y[rim]
    cx,cy,r=fit(xs,ys)
    # refit excluding handle side
    sel = (ys-cy)*handle_sign < r*0.3
    cx,cy,r=fit(xs[sel],ys[sel])
    plate=reg&(z>0.44)&(z<0.47)
    print(f'{name}: mug top z={top:.3f} rim center=({cx:.3f},{cy:.3f}) r={r:.3f}; plate pts x[{x[plate].min():.3f},{x[plate].max():.3f}] y[{y[plate].min():.3f},{y[plate].max():.3f}] plate ctr~({(x[plate].min()+x[plate].max())/2:.3f},{(y[plate].min()+y[plate].max())/2:.3f})')
EOF
python3 check.py agentview

# openrua op 42
timeout 900 python3 -u step_moveq.py 0,-0.161,0,-2.445,0,2.227,0.785 5 2>&1 | tail -2; timeout 120 python3 cloud.py agentview 0.44 >/dev/null 2>&1; python3 check.py agentview; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py frontview
