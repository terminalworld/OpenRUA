#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat /workspace/tools/action/fjt_send.py; echo ----; cat /workspace/tools/action/gripper_cmd.py; echo ----; cat /workspace/tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -v parameter; echo ----; timeout 20 ros2 node list

# openrua op 4
timeout 30 ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | head -80; echo ----; timeout 10 ros2 topic echo /tf --once 2>&1 | head -30

# openrua op 6
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Dump the full TF tree once and the hand pose via FK. Also expose helpers."""
import sys
import numpy as np
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from tf2_msgs.msg import TFMessage
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

ARM = [f"panda_joint{i}" for i in range(1, 8)]


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def main():
    rclpy.init()
    node = rclpy.create_node("scene")
    tfs = {}
    qos = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

    def cb(msg):
        for t in msg.transforms:
            tfs[(t.header.frame_id, t.child_frame_id)] = t.transform
    node.create_subscription(TFMessage, "/tf", cb, 10)
    node.create_subscription(TFMessage, "/tf_static", cb, qos)
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    for _ in range(30):
        rclpy.spin_once(node, timeout_sec=0.2)
    for (p, c), t in sorted(tfs.items()):
        tr, q = t.translation, t.rotation
        print(f"{p:>28s} -> {c:<28s} t=({tr.x:+.4f},{tr.y:+.4f},{tr.z:+.4f}) q=({q.x:+.4f},{q.y:+.4f},{q.z:+.4f},{q.w:+.4f})")

    # FK for the hand
    cli = node.create_client(GetPositionFK, "/compute_fk")
    if cli.wait_for_service(timeout_sec=10) and "m" in js:
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        seed = JointState()
        for n, p in zip(js["m"].name, js["m"].position):
            if n in ARM:
                seed.name.append(n); seed.position.append(p)
        req.robot_state.joint_state = seed
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
        r = fut.result()
        if r and r.error_code.val == 1:
            ps = r.pose_stamped[0]
            p, q = ps.pose.position, ps.pose.orientation
            print(f"FK hand in frame '{ps.header.frame_id}': p=({p.x:+.4f},{p.y:+.4f},{p.z:+.4f}) q=({q.x:+.4f},{q.y:+.4f},{q.z:+.4f},{q.w:+.4f})")
            R = quat_to_R(q.x, q.y, q.z, q.w)
            print("hand Z axis (approach) in that frame:", np.round(R[:, 2], 4))
        else:
            print("FK failed", r.error_code.val if r else None)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 7
timeout 90 python3 scene.py

# openrua op 8
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Project a camera's depth frame into world points; find clusters above the table.

Usage: python3 cloud.py <camera> [zmin] [zmax]
Saves <camera>_cloud.npz (xyz HxWx3 world, color) and prints clusters.
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage
import cv2


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def grab(node, topic, typ, timeout=30.0):
    got = {}
    sub = node.create_subscription(typ, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    if "m" not in got:
        raise SystemExit(f"no msg on {topic}")
    return got["m"]


def main():
    cam = sys.argv[1]
    zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.83
    zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 1.3
    rclpy.init()
    node = rclpy.create_node("cloud")
    tfs = {}
    node.create_subscription(TFMessage, "/tf",
                             lambda m: [tfs.setdefault(t.child_frame_id, t.transform) for t in m.transforms], 10)
    depth_msg = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    color_msg = grab(node, f"/{cam}/color/image_raw", Image)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while frame not in tfs and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfs[frame]
    br = CvBridge()
    depth = br.imgmsg_to_cv2(depth_msg, "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(color_msg, "bgr8")
    H, W = depth.shape
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1)
    R = quat_to_R(t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
    T = np.array([t.translation.x, t.translation.y, t.translation.z])
    xyz = pc @ R.T + T
    np.savez(f"{cam}_cloud.npz", xyz=xyz, color=color, depth=depth)
    valid = np.isfinite(depth) & (depth > 0)
    zs = xyz[..., 2][valid]
    hist, edges = np.histogram(zs, bins=60, range=(0, 3.2))
    print("z histogram (world):")
    for h, e in zip(hist, edges):
        if h > 200:
            print(f"  z~{e:.3f}: {h}")
    mask = valid & (xyz[..., 2] > zmin) & (xyz[..., 2] < zmax)
    n, lab, stats, cents = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
    print(f"clusters with z in ({zmin},{zmax}):")
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15:
            continue
        m = lab == i
        p = xyz[m]
        c = color[m].mean(0)
        print(f"  #{i} area={stats[i, cv2.CC_STAT_AREA]} px_center=({cents[i][0]:.0f},{cents[i][1]:.0f}) "
              f"x=[{p[:,0].min():.3f},{p[:,0].max():.3f}] y=[{p[:,1].min():.3f},{p[:,1].max():.3f}] "
              f"z=[{p[:,2].min():.3f},{p[:,2].max():.3f}] mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f},{p[:,2].mean():.3f}) bgr={c.round(0)}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 9
timeout 120 python3 cloud.py birdview 0.83 1.3

# openrua op 10
timeout 60 python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; color=d['color']; depth=d['depth']
valid=np.isfinite(depth)&(depth>0)
z=xyz[...,2]
# fine histogram near table
hist,edges=np.histogram(z[valid],bins=40,range=(0.35,0.75))
for h,e in zip(hist,edges):
    if h>30: print(f'z~{e:.3f}: {h}')
mask=valid&(z>0.445)&(z<0.75)
n,lab,stats,cents=cv2.connectedComponentsWithStats(mask.astype(np.uint8),8)
for i in range(1,n):
    if stats[i,cv2.CC_STAT_AREA]<8: continue
    m=lab==i; p=xyz[m]; c=color[m].mean(0)
    print(f'#{i} area={stats[i,4]} px=({cents[i][0]:.0f},{cents[i][1]:.0f}) x=[{p[:,0].min():.3f},{p[:,0].max():.3f}] y=[{p[:,1].min():.3f},{p[:,1].max():.3f}] z=[{p[:,2].min():.3f},{p[:,2].max():.3f}] mean=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) bgr={c.round(0)}')
"


# openrua op 11
timeout 120 python3 cloud.py agentview 0.445 0.75 2>&1 | grep -v "^  z~"; timeout 60 python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
# table z right around the box and ketchup
for name,(x0,x1,y0,y1) in {'near_box':(-0.26,-0.14,-0.03,0.10),'near_ketchup':(-0.30,-0.15,-0.25,-0.08),'basket_inside':(-0.05,0.04,0.20,0.30)}.items():
    m=(xyz[...,0]>x0)&(xyz[...,0]<x1)&(xyz[...,1]>y0)&(xyz[...,1]<y1)&np.isfinite(xyz[...,2])
    z=xyz[...,2][m]
    print(name, 'z percentiles', np.percentile(z,[1,10,50,90,99]).round(3))
"

# openrua op 12
timeout 60 python3 -c "
import cv2, numpy as np
im=cv2.imread('birdview.png')
crop=im[150:340,230:430]
cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('agentview.png')
crop=im[170:300,180:400]
cv2.imwrite('agent_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
"

# openrua op 13
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Reusable arm controller: joint state, FK, IK, trajectory, gripper.

Poses are hand-frame poses; helpers convert TCP poses (fingertip point)
to hand poses with machine.yaml's tcp_offset_m.
"""
import time
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
ARM = TRAJ["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
# hand orientation with fingers closing along world X, approach = -Z
Q_DOWN_X = (0.7071068, -0.7071068, 0.0, 0.0)
# fingers closing along world Y
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tcp_to_hand(p, q):
    """TCP point -> hand origin (back along the hand's +Z approach axis)."""
    R = quat_to_R(*q)
    return np.asarray(p, float) - TCP * R[:, 2]


class Ctl:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.traj = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.fk.wait_for_service(10); self.ik.wait_for_service(10)
        self.traj.wait_for_server(10); self.grip.wait_for_server(10)

    def _on_js(self, m):
        self._js["m"] = m
        self._js["t"] = time.time()

    def joints(self, fresh=True):
        """dict name->position from a fresh /joint_states."""
        t0 = time.time()
        if fresh:
            self._js.pop("m", None)
        while "m" not in self._js:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if time.time() - t0 > 20:
                raise RuntimeError("no /joint_states")
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        j = self.joints()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.joints()
        return j["panda_finger_joint1"], j["panda_finger_joint2"]

    def _seed(self, q=None):
        js = JointState()
        q = q if q is not None else self.arm_q()
        js.name = list(ARM); js.position = [float(v) for v in q]
        return js

    def hand_pose(self, q=None):
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state = self._seed(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p, o = r.pose_stamped[0].pose.position, r.pose_stamped[0].pose.orientation
        return np.array([p.x, p.y, p.z]), np.array([o.x, o.y, o.z, o.w])

    def tcp_pose(self, q=None):
        p, o = self.hand_pose(q)
        R = quat_to_R(*o)
        return p + TCP * R[:, 2], o

    def solve_ik(self, hand_p, q, seed=None, attempts=3):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        ps = req.ik_request.pose_stamped.pose
        ps.position.x, ps.position.y, ps.position.z = map(float, hand_p)
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q)
        req.ik_request.robot_state.joint_state = self._seed(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        for _ in range(attempts):
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            r = fut.result()
            if r is not None and r.error_code.val == 1:
                sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
                return [sol[n] for n in ARM]
        raise RuntimeError(f"IK failed ({r and r.error_code.val}) for {np.round(hand_p,3)} {q}")

    def ik_tcp(self, tcp_p, q, seed=None):
        return self.solve_ik(tcp_to_hand(tcp_p, q), q, seed)

    def move_joints(self, points, seconds):
        """points: list of 7-vectors; seconds: list of cumulative times."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        for p, s in zip(points, seconds):
            pt = JointTrajectoryPoint(positions=[float(v) for v in p])
            pt.time_from_start = Duration(sec=int(s), nanosec=int((s % 1) * 1e9))
            goal.trajectory.points.append(pt)
        fut = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        h = fut.result()
        if h is None or not h.accepted:
            raise RuntimeError("trajectory goal rejected")
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        res = rf.result()
        code = res.result.error_code if res else None
        got = np.array(self.arm_q())
        err = np.abs(got - np.array(points[-1])).max()
        print(f"  traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, tcp_p, q, seconds=3.0, seed=None):
        sol = self.ik_tcp(tcp_p, q, seed)
        code, err = self.move_joints([sol], [seconds])
        p, o = self.tcp_pose()
        print(f"  tcp now {np.round(p,4)} target {np.round(tcp_p,4)} q={np.round(o,3)}")
        return sol

    def move_tcp_path(self, waypoints, q, dt=2.0):
        """Several TCP waypoints in one trajectory (IK chained by seed)."""
        sols, seed, times = [], None, []
        t = 0.0
        for wp in waypoints:
            seed = self.ik_tcp(wp, q, seed)
            sols.append(seed); t += dt; times.append(t)
        code, err = self.move_joints(sols, times)
        p, o = self.tcp_pose()
        print(f"  tcp now {np.round(p,4)} target {np.round(waypoints[-1],4)}")
        return sols

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        gap = self.finger_gap()
        print(f"  gripper -> {width}: reached={r.reached_goal} stalled={r.stalled} fingers={np.round(gap,4)}")
        return gap
OPENRUA_EOF

# openrua op 14
timeout 120 python3 -c "
from ctl import *
c=Ctl()
q=c.arm_q(); print('current', np.round(q,3))
p,o=c.hand_pose(); print('FK hand', np.round(p,4), np.round(o,4))
# IK for the same pose, empty frame_id -> expect joints ~ current
sol=c.solve_ik(p,o); print('IK sol', np.round(sol,3), 'maxdiff', np.abs(np.array(sol)-np.array(q)).max())
# IK for the same pose shifted by base offset (if IK interprets in base frame, this would be the world pose)
try:
    sol2=c.solve_ik(p-np.array([-0.51,0,0.42]),o); print('IK base-shifted sol', np.round(sol2,3))
    print('FK of that sol ->', np.round(c.hand_pose(sol2)[0],4))
except Exception as e: print(e)
print('tcp', np.round(c.tcp_pose()[0],4))
"

# openrua op 15
timeout 120 python3 -c "
from ctl import *
c=Ctl()
p,o=c.hand_pose()
sol=c.solve_ik(p,o)
p2,o2=c.hand_pose(sol); print('FK(sol)', np.round(p2,4), np.round(o2,4), 'pos err', np.linalg.norm(p2-p))
# test target poses for the task
for name,tcp in [('ketchup_pre',(-0.218,-0.149,0.60)),('ketchup_grasp',(-0.218,-0.149,0.475)),('box_pre',(-0.198,0.033,0.60)),('box_grasp',(-0.198,0.033,0.445)),('basket_over',(0.0,0.25,0.75))]:
    for qn,q in [('QX',Q_DOWN_X),('QY',Q_DOWN_Y)]:
        try:
            s=c.ik_tcp(tcp,q); pt,ot=c.tcp_pose(s)
            print(name,qn,'sol',np.round(s,3),'tcp',np.round(pt,3),'q',np.round(ot,3))
        except Exception as e: print(name,qn,'FAIL',e)
"

# openrua op 16
python3 - <<'EOF'
import re
s=open('ctl.py').read()
s=s.replace('''# hand orientation with fingers closing along world X, approach = -Z
Q_DOWN_X = (0.7071068, -0.7071068, 0.0, 0.0)
# fingers closing along world Y
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)
''','''

def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2)


# IK solves for panda_link8; panda_hand = link8 * Rz(-45deg). To get a
# desired HAND orientation, ask IK for hand * Rz(+45deg).
RZ45 = (0.0, 0.0, 0.3826834, 0.9238795)


def hand_to_ik(q_hand):
    return qmul(q_hand, RZ45)


# desired HAND orientations (approach = -Z world)
Q_HAND_X = (0.7071068, -0.7071068, 0.0, 0.0)   # fingers close along world X
Q_HAND_Y = (1.0, 0.0, 0.0, 0.0)                # fingers close along world Y
''')
# solve_ik: convert q (hand) -> link8 target
s=s.replace('''        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, q)''',
'''        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = map(float, hand_to_ik(q))''')
open('ctl.py','w').write(s)
EOF
timeout 120 python3 -c "
from ctl import *
c=Ctl()
for qn,q in [('X',Q_HAND_X),('Y',Q_HAND_Y)]:
    s=c.ik_tcp((-0.218,-0.149,0.60),q); pt,ot=c.tcp_pose(s)
    R=quat_to_R(*ot)
    print(qn,'sol',np.round(s,3),'tcp',np.round(pt,3),'hand q',np.round(ot,3),'finger axis (hand y) in world',np.round(R[:,1],3),'approach',np.round(R[:,2],3))
"

# openrua op 17
mkdir -p "$(dirname /workspace/pick_place.py)"
cat > /workspace/pick_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick an object at (x,y) with a top-down grasp and drop it into the basket.

Usage: python3 -u pick_place.py <name> <x> <y> <grasp_tcp_z> [pre_z]
"""
import sys
import numpy as np
from ctl import Ctl, Q_HAND_X

BASKET = np.array([0.0, 0.25])
Z_TRANSPORT = 0.76   # TCP height while carrying (bottle bottom clears rim 0.63)
Z_RELEASE = 0.68     # TCP height when opening over the basket
OPEN = 0.04


def main():
    name, x, y, zg = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
    zpre = float(sys.argv[5]) if len(sys.argv) > 5 else 0.62
    q = Q_HAND_X
    c = Ctl()
    print(f"[{name}] start tcp={np.round(c.tcp_pose()[0],3)} fingers={np.round(c.finger_gap(),4)}")

    print(f"[{name}] open gripper"); c.gripper(OPEN)
    print(f"[{name}] pre-grasp above object"); c.move_tcp((x, y, zpre), q, 4.0)
    print(f"[{name}] descend"); c.move_tcp((x, y, zg), q, 2.5)
    print(f"[{name}] close"); f1, f2 = c.gripper(0.0)
    gap = abs(f1) + abs(f2)
    print(f"[{name}] finger gap after close = {gap:.4f} m ({'HOLDING' if gap > 0.008 else 'EMPTY?'})")
    print(f"[{name}] lift"); c.move_tcp((x, y, zpre), q, 2.5)
    print(f"[{name}] lift check fingers={np.round(c.finger_gap(),4)}")
    print(f"[{name}] lift to transport height"); c.move_tcp((x, y, Z_TRANSPORT), q, 2.0)
    print(f"[{name}] move over basket"); c.move_tcp((BASKET[0], BASKET[1], Z_TRANSPORT), q, 4.0)
    print(f"[{name}] check fingers={np.round(c.finger_gap(),4)}")
    print(f"[{name}] lower over basket"); c.move_tcp((BASKET[0], BASKET[1], Z_RELEASE), q, 2.0)
    print(f"[{name}] release"); c.gripper(OPEN)
    print(f"[{name}] retreat up"); c.move_tcp((BASKET[0], BASKET[1], Z_TRANSPORT), q, 2.0)
    print(f"[{name}] DONE tcp={np.round(c.tcp_pose()[0],3)}")


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 18
nohup python3 -u pick_place.py ketchup -0.218 -0.149 0.47 0.62 > ketchup.log 2>&1 &
sleep 45; cat /workspace/ketchup.log

# openrua op 19
sleep 20; cat /workspace/ketchup.log; echo; pgrep -f pick_place.py || echo "script exited"

# openrua op 20
for c in agentview birdview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 21
timeout 120 python3 cloud.py birdview 0.445 0.75 2>&1 | grep -v "^  z~"

# openrua op 22
timeout 60 python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[150:340,230:430]
cv2.imwrite('bird_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))"

# openrua op 23
timeout 60 python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; color=d['color']
# box region: blue-ish / distinct from wood in a window around px (328,247)
win=color[225:270,300:360].astype(int)
b,g,r=win[...,0],win[...,1],win[...,2]
mask=(b>g+10)&(b>r+10)  # bluish vs brown wood
ys,xs=np.nonzero(mask)
print('bluish px count',len(xs),'u range',xs.min()+300,xs.max()+300,'v range',ys.min()+225,ys.max()+225)
sub=xyz[225:270,300:360][mask]
print('world x range',sub[:,0].min().round(3),sub[:,0].max().round(3),'y range',sub[:,1].min().round(3),sub[:,1].max().round(3),'z',sub[:,2].min().round(3),sub[:,2].max().round(3))
print('center',sub[:,:2].mean(0).round(4))
"

# openrua op 24
python3 - <<'EOF'
s=open('/workspace/ctl.py').read()
s=s.replace('''    def move_joints(self, points, seconds):
        """points: list of 7-vectors; seconds: list of cumulative times."""
        goal = FollowJointTrajectory.Goal()''','''    def move_joints(self, points, seconds, retries=2):
        """points: list of 7-vectors; seconds: list of cumulative times.
        On a tolerance-violation result with a real residual, resend the
        final point (docs: usually controller lag, converges on resend)."""
        code, err = self._send(points, seconds)
        n = 0
        while (code != 0 or err > 0.03) and err > 0.02 and n < retries:
            n += 1
            print(f"  resend final point (attempt {n})")
            code, err = self._send([points[-1]], [max(2.0, seconds[-1] / len(points))])
        return code, err

    def _send(self, points, seconds):
        goal = FollowJointTrajectory.Goal()''')
s=s.replace('''    def move_tcp_path(self, waypoints, q, dt=2.0):''','''    def move_tcp_line(self, tcp_to, q, n=4, seconds=3.0):
        """Straight TCP line from the current TCP to tcp_to in n IK steps."""
        p0, _ = self.tcp_pose()
        wps = [p0 + (np.asarray(tcp_to, float) - p0) * (i / n) for i in range(1, n + 1)]
        return self.move_tcp_path(wps, q, dt=seconds / n)

    def move_tcp_path(self, waypoints, q, dt=2.0):''')
open('/workspace/ctl.py','w').write(s)
EOF
cat > /workspace/pick_box.py <<'EOF'
#!/usr/bin/env python3
"""Pick the cream cheese box (fingers along Y) and drop it into the basket."""
import numpy as np
from ctl import Ctl, Q_HAND_Y

X, Y = -0.169, 0.033
Z_GRASP, Z_PRE, Z_HIGH = 0.445, 0.60, 0.76
BASKET = (0.0, 0.25)
Z_RELEASE = 0.65
OPEN = 0.04
q = Q_HAND_Y

c = Ctl()
print("start tcp", np.round(c.tcp_pose()[0], 3), "fingers", np.round(c.finger_gap(), 4))
print("open"); c.gripper(OPEN)
print("go high above box"); c.move_tcp((X, Y, Z_HIGH), q, 4.0)
print("descend to pre-grasp"); c.move_tcp_line((X, Y, Z_PRE), q, n=3, seconds=3.0)
print("descend to grasp"); c.move_tcp_line((X, Y, Z_GRASP), q, n=4, seconds=4.0)
p, o = c.tcp_pose(); print("at grasp: tcp", np.round(p, 4), "q", np.round(o, 3))
print("close"); f1, f2 = c.gripper(0.0)
gap = abs(f1) + abs(f2)
print(f"gap after close = {gap:.4f} m ({'HOLDING' if 0.008 < gap else 'EMPTY?'})")
print("lift"); c.move_tcp_line((X, Y, Z_PRE), q, n=3, seconds=3.0)
print("fingers after lift", np.round(c.finger_gap(), 4))
print("to high"); c.move_tcp((X, Y, Z_HIGH), q, 2.5)
print("over basket"); c.move_tcp((BASKET[0], BASKET[1], Z_HIGH), q, 4.0)
print("fingers over basket", np.round(c.finger_gap(), 4))
print("lower"); c.move_tcp_line((BASKET[0], BASKET[1], Z_RELEASE), q, n=2, seconds=2.0)
print("release"); c.gripper(OPEN)
print("retreat"); c.move_tcp((BASKET[0], BASKET[1], Z_HIGH), q, 2.0)
print("park away from basket"); c.move_tcp((-0.15, -0.05, 0.76), q, 3.0)
print("DONE tcp", np.round(c.tcp_pose()[0], 3))
EOF
nohup python3 -u pick_box.py > box.log 2>&1 &
sleep 60; cat /workspace/box.log

# openrua op 25
sleep 45; cat /workspace/box.log; pgrep -f pick_box.py || echo "script exited"

# openrua op 26
sleep 60; tail -n +22 /workspace/box.log; pgrep -f pick_box.py || echo "script exited"

# openrua op 27
sleep 25; tail -n 3 /workspace/box.log; pgrep -f pick_box.py || echo "script exited"; for c in agentview birdview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 28
timeout 60 python3 -c "
import cv2
im=cv2.imread('birdview.png'); crop=im[250:330,340:420]
cv2.imwrite('basket_crop.png', cv2.resize(crop,None,fx=8,fy=8,interpolation=cv2.INTER_CUBIC))"
timeout 120 python3 cloud.py birdview 0.445 0.75 2>&1 | grep -v "^  z~"

# openrua op 29
timeout 60 python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; color=d['color'].astype(int)
b,g,r=color[...,0],color[...,1],color[...,2]
inb=(xyz[...,0]>-0.083)&(xyz[...,0]<0.076)&(xyz[...,1]>0.167)&(xyz[...,1]<0.335)
blue=(b>g+15)&(b>r+15)&(b<150)   # box top
orange=(r>140)&(r>b+60)&(g<r-30) # ketchup body
for name,m in [('box(blue)',blue),('ketchup(orange)',orange)]:
    pts=xyz[m&inb]; out=xyz[m&~inb&(xyz[...,2]>0.44)&(xyz[...,2]<0.8)]
    print(f'{name}: {len(pts)} px inside basket footprint, center={pts[:,:2].mean(0).round(3) if len(pts) else None}, z={pts[:,2].mean().round(3) if len(pts) else None}; {len(out)} px elsewhere on table')
"

# openrua op 30
timeout 60 python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; color=d['color'].astype(int)
b,g,r=color[...,0],color[...,1],color[...,2]
inb=(xyz[...,0]>-0.083)&(xyz[...,0]<0.076)&(xyz[...,1]>0.167)&(xyz[...,1]<0.335)
red=(r>b+40)&(r>g+20)&(r>90)
pts=xyz[red&inb]; out=xyz[red&~inb&(xyz[...,2]>0.44)&(xyz[...,2]<0.8)]
print(f'ketchup(reddish): {len(pts)} px inside basket, center={pts[:,:2].mean(0).round(3)}, z range={pts[:,2].min().round(3)}-{pts[:,2].max().round(3)}; {len(out)} px elsewhere above table')
"

# openrua op 31
timeout 60 python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[170:270,360:480]
cv2.imwrite('front_crop.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"
