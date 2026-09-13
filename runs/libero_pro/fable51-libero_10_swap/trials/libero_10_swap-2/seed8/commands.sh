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
timeout 20 ros2 topic echo /joint_states --once; echo ----; timeout 20 ros2 topic info /pipeline_state; timeout 20 ros2 topic echo /pipeline_state --once

# openrua op 5
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand galleryview paperview; do timeout 30 python3 tools/perception/cam_snap.py $c; done; ls -la *.png

# openrua op 6
timeout 20 ros2 topic echo /tf_static --once 2>&1 | head -80; echo; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | sort | uniq -c

# openrua op 7
timeout 15 ros2 topic echo /tf --once 2>&1 | grep -B2 -A14 "child_frame_id: panda_link1" | head -30; echo ----; timeout 15 python3 - <<'EOF'
import rclpy
from tf2_ros import Buffer, TransformListener
rclpy.init(); n=rclpy.create_node("tfq"); b=Buffer(); TransformListener(b,n)
import time
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.2)
print(b.all_frames_as_string())
EOF

# openrua op 8
mkdir -p "$(dirname /workspace/tfq.py)"
cat > /workspace/tfq.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Print world->frame transforms for given frames (default: base, hand)."""
import sys
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

frames = sys.argv[1:] or ["panda_link0", "panda_hand"]
rclpy.init(); n = rclpy.create_node("tfq"); b = Buffer(); TransformListener(b, n)
for _ in range(30):
    rclpy.spin_once(n, timeout_sec=0.1)
    if all(b.can_transform("world", f, Time()) for f in frames):
        break
for f in frames:
    t = b.lookup_transform("world", f, Time())
    tr, q = t.transform.translation, t.transform.rotation
    print(f"world->{f}: xyz=({tr.x:.4f}, {tr.y:.4f}, {tr.z:.4f}) "
          f"q=({q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f})")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 9
timeout 30 python3 tfq.py panda_link0 panda_hand birdview_optical_frame robot0_eye_in_hand_optical_frame

# openrua op 10
timeout 30 python3 tools/perception/cam_snap.py /birdview/depth/image_raw birdview_depth.png && timeout 10 ros2 topic echo /birdview/color/camera_info --once | grep -A12 "^k:" ; python3 -c "
import numpy as np; d=np.load('birdview_depth.npy'); print(d.shape, np.nanmin(d), np.nanmax(d)); print('center', d[240,320]); print('table sample', d[400,100])"

# openrua op 11
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Build a world-frame point cloud from a camera's current depth frame.

Usage: python3 cloud.py <camera> [out.npz]
Saves xyz (H,W,3) world coords + color (H,W,3) to <camera>_cloud.npz
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.time import Time
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
    out = sys.argv[2] if len(sys.argv) > 2 else f"{cam}_cloud.npz"
    rclpy.init(); node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "rgb8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    while not buf.can_transform("world", frame, Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, Time())
    q, tr = t.transform.rotation, t.transform.translation
    R = quat_R(q.x, q.y, q.z, q.w); T = np.array([tr.x, tr.y, tr.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    pc = np.stack([(u - cx) * depth / fx, (v - cy) * depth / fy, depth], -1)
    xyz = pc @ R.T + T
    np.savez(out, xyz=xyz, color=color, depth=depth)
    print(out, xyz.shape)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 12
timeout 60 python3 cloud.py birdview && python3 - <<'EOF'
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
# table height: mode of z within table region
tz = z[300:420, 170:300].ravel()
print("table z hist:", np.percentile(tz,[5,50,95]))
table = np.median(tz)
# objects: z > table+0.01
mask = z > table + 0.008
# exclude robot region (x < -0.35)
mask &= xyz[...,0] > -0.40
ys, xs = np.nonzero(mask)
print("n obj px", len(xs))
# cluster by simple connected components
import cv2
lab_n, lab = cv2.connectedComponents(mask.astype(np.uint8))
for i in range(1, lab_n):
    m = lab==i
    if m.sum() < 30: continue
    p = xyz[m]
    print(f"comp {i}: n={m.sum()} px(u,v) center=({np.nonzero(m)[1].mean():.0f},{np.nonzero(m)[0].mean():.0f}) "
          f"x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] zmax={p[:,2].max():.3f} zmed={np.median(p[:,2]):.3f}")
EOF

# openrua op 13
python3 - <<'EOF'
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]; col=d['color']
def region(xmin,xmax,ymin,ymax):
    return (xyz[...,0]>xmin)&(xyz[...,0]<xmax)&(xyz[...,1]>ymin)&(xyz[...,1]<ymax)
# knob region
m = region(-0.30,-0.14,0.13,0.28) & (z>0.905)
p = xyz[m]
print("knob z levels:", np.round(np.percentile(p[:,2],[10,30,50,70,90,100]),3))
for zlo in [0.91,0.93,0.945,0.955]:
    q = p[p[:,2]>zlo]
    if len(q): print(f" z>{zlo}: n={len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] mean {q[:,0].mean():.3f}  y[{q[:,1].min():.3f},{q[:,1].max():.3f}] mean {q[:,1].mean():.3f}")
# stove region
m = region(-0.16,0.07,0.09,0.32) & (z>0.905) & (z<0.945)
p = xyz[m]
print("stove z levels:", np.round(np.percentile(p[:,2],[5,25,50,75,95,100]),3))
print(f" stove xy: x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}]")
# burner: darker pixels in stove region? print color stats
c = col[m]
print(" color mean", c.mean(0))
# pan handle over the stove: z in stove region above 0.94
m2 = region(-0.16,0.07,0.09,0.32) & (z>=0.945)
p2 = xyz[m2]
if len(p2): print(f"handle over stove: n={len(p2)} x[{p2[:,0].min():.3f},{p2[:,0].max():.3f}] y[{p2[:,1].min():.3f},{p2[:,1].max():.3f}] z[{p2[:,2].min():.3f},{p2[:,2].max():.3f}]")
# pan
m3 = region(-0.06,0.18,-0.13,0.25) & (z>0.905)
p3 = xyz[m3]
print("pan: x[%.3f,%.3f] y[%.3f,%.3f]"%(p3[:,0].min(),p3[:,0].max(),p3[:,1].min(),p3[:,1].max()))
rim = p3[p3[:,2]>1.0]
print(" rim x[%.3f,%.3f] y[%.3f,%.3f] center (%.3f,%.3f)"%(rim[:,0].min(),rim[:,0].max(),rim[:,1].min(),rim[:,1].max(),(rim[:,0].min()+rim[:,0].max())/2,(rim[:,1].min()+rim[:,1].max())/2))
# moka
m4 = region(-0.12,0.02,-0.33,-0.15) & (z>0.905)
p4 = xyz[m4]
print("moka z levels:", np.round(np.percentile(p4[:,2],[5,25,50,75,95,100]),3))
for zlo in [0.91,0.95,0.98,1.0,1.02,1.04]:
    q = p4[p4[:,2]>zlo]
    if len(q): print(f" z>{zlo}: n={len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] mean {q[:,0].mean():.3f}  y[{q[:,1].min():.3f},{q[:,1].max():.3f}] mean {q[:,1].mean():.3f}")
EOF

# openrua op 14
timeout 60 python3 cloud.py frontview; timeout 60 python3 cloud.py sideview; timeout 60 python3 cloud.py agentview; python3 - <<'EOF'
import numpy as np
for cam in ['frontview','sideview','agentview']:
    d=np.load(f'{cam}_cloud.npz'); xyz=d['xyz']
    m=(xyz[...,0]>-0.15)&(xyz[...,0]<0.05)&(xyz[...,1]>-0.33)&(xyz[...,1]<-0.12)&(xyz[...,2]>0.902)&(xyz[...,2]<1.07)
    p=xyz[m]
    print(cam, "moka pts", len(p))
    for zlo in np.arange(0.90,1.06,0.01):
        q=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)]
        if len(q)>3:
            print(f"  z {zlo:.2f}: n={len(q):4d} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]")
EOF

# openrua op 15
python3 -c "
import cv2
im=cv2.imread('frontview.png'); crop=im[290:420,140:260]; cv2.imwrite('moka_front.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('sideview.png'); crop=im[200:300,250:350]; cv2.imwrite('moka_side.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('robot0_robotview.png'); crop=im[90:200,440:560]; cv2.imwrite('knob_rv.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 16
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 "^position" | tail -2

# openrua op 17
timeout 600 python3 tools/action/ik_move.py 0.4585 0.2065 0.088 1 0 0 0 4 --at tcp; timeout 30 python3 tfq.py panda_hand; timeout 20 ros2 topic echo /joint_states --once | grep -A7 "^position"

# openrua op 18
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Arm helper: IK in world frame (TCP), trajectory, gripper, state readers.

Usage as CLI:
  python3 arm.py js                                  # joint state (arm + fingers)
  python3 arm.py hand                                # world->hand and TCP pose
  python3 arm.py ik   x y z qx qy qz qw              # print IK joints (TCP pose, world frame)
  python3 arm.py move x y z qx qy qz qw [sec]        # IK + trajectory
  python3 arm.py joints j1,...,j7 [sec]              # raw trajectory
  python3 arm.py j7 <delta_rad> [sec]                # rotate joint7 by delta
  python3 arm.py grip open|close
"""
import sys
import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
BASE_IN_WORLD = np.array([-0.66, 0.0, 0.912])  # from TF world->panda_link0


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      lambda m: self._js.__setitem__("m", m), 1)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.gr = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joint_state(self):
        self._js.pop("m", None)
        while "m" not in self._js:
            self.spin(0.2)
        m = self._js["m"]
        return dict(zip(m.name, m.position))

    def arm_q(self):
        js = self.joint_state()
        return [js[j] for j in JOINTS]

    def fingers(self):
        js = self.joint_state()
        return js["panda_finger_joint1"], js["panda_finger_joint2"]

    def hand_pose(self):
        """world->panda_hand (pos, quat) and TCP world position."""
        for _ in range(50):
            self.spin(0.1)
            if self.tfbuf.can_transform("world", "panda_hand", Time()):
                break
        t = self.tfbuf.lookup_transform("world", "panda_hand", Time())
        tr, q = t.transform.translation, t.transform.rotation
        p = np.array([tr.x, tr.y, tr.z]); qq = np.array([q.x, q.y, q.z, q.w])
        R = quat_R(*qq)
        return p, qq, p + TCP * R[:, 2]

    def solve_ik(self, tcp_world, quat, seed=None, timeout=60):
        """IK for the hand such that the TCP sits at tcp_world (world frame)."""
        R = quat_R(*quat)
        hand_world = np.asarray(tcp_world, float) - TCP * R[:, 2]
        hand_base = hand_world - BASE_IN_WORLD
        if not self.ik.wait_for_service(timeout_sec=10):
            raise RuntimeError("IK service unavailable")
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_base)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        seed_js = JointState()
        seed_js.name = list(JOINTS)
        seed_js.position = [float(v) for v in (seed if seed is not None else self.arm_q())]
        req.ik_request.robot_state.joint_state = seed_js
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            raise RuntimeError("IK timeout")
        if res.error_code.val != 1:
            raise RuntimeError(f"IK failed code={res.error_code.val}")
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    def traj(self, points, seconds):
        """points: list of 7-vectors; seconds: list of time_from_start."""
        if not self.fjt.wait_for_server(timeout_sec=10):
            raise RuntimeError("no FJT server")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        for q, s in zip(points, seconds):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(s), nanosec=int((s % 1) * 1e9))
            goal.trajectory.points.append(pt)
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        q = self.arm_q()
        err = np.abs(np.array(q) - np.array(points[-1])).max()
        print(f"traj done code={code} max_joint_err={err:.4f}")
        return code, err

    def move_to(self, q, seconds=3.0):
        return self.traj([q], [seconds])

    def move_tcp(self, tcp_world, quat, seconds=3.0, seed=None):
        q = self.solve_ik(tcp_world, quat, seed)
        r = self.move_to(q, seconds)
        p, qq, tcp = self.hand_pose()
        print(f"TCP now {np.round(tcp, 4)} q={np.round(qq, 4)}")
        return r

    def grip(self, width):
        if not self.gr.wait_for_server(timeout_sec=10):
            raise RuntimeError("no gripper server")
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.gr.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        r = rf.result().result
        f = self.fingers()
        print(f"grip -> reached={r.reached_goal} stalled={r.stalled} fingers={f[0]:.4f},{f[1]:.4f} gap={abs(f[0]) + abs(f[1]):.4f}")
        return r


def main():
    a = sys.argv[1:]
    arm = Arm()
    if a[0] == "js":
        print(arm.joint_state())
    elif a[0] == "hand":
        p, q, tcp = arm.hand_pose()
        print("hand", np.round(p, 4), "q", np.round(q, 4), "tcp", np.round(tcp, 4))
    elif a[0] == "ik":
        print(",".join(f"{v:.5f}" for v in arm.solve_ik([float(v) for v in a[1:4]], [float(v) for v in a[4:8]])))
    elif a[0] == "move":
        sec = float(a[8]) if len(a) > 8 else 3.0
        arm.move_tcp([float(v) for v in a[1:4]], [float(v) for v in a[4:8]], sec)
    elif a[0] == "joints":
        sec = float(a[2]) if len(a) > 2 else 3.0
        arm.move_to([float(v) for v in a[1].split(",")], sec)
    elif a[0] == "j7":
        sec = float(a[2]) if len(a) > 2 else 3.0
        q = arm.arm_q(); q[6] += float(a[1])
        arm.move_to(q, sec)
        print("joints now", np.round(arm.arm_q(), 4))
    elif a[0] == "grip":
        arm.grip(GRIP["open_m"] if a[1] == "open" else GRIP["closed_m"])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 19
timeout 120 python3 arm.py hand && timeout 120 python3 arm.py ik -0.2015 0.2065 1.00 1 0 0 0

# openrua op 20
timeout 300 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm, quat_R, TCP
arm = Arm()
p,q,tcp = arm.hand_pose()
tests = {
 "current tcp": (tcp, q),
 "current tcp, q=(1,0,0,0)": (tcp, [1,0,0,0]),
 "knob +10cm": ([-0.2015,0.2065,1.05], q),
 "knob +10cm q ideal": ([-0.2015,0.2065,1.05], [1,0,0,0]),
 "knob +20cm q ideal": ([-0.2015,0.2065,1.15], [1,0,0,0]),
}
for k,(pos,qq) in tests.items():
    try:
        sol = arm.solve_ik(pos, qq, timeout=30)
        print(k, "OK", np.round(sol,3))
    except Exception as e:
        print(k, "FAIL", e)
rclpy.shutdown()
EOF

# openrua op 21
timeout 120 python3 - <<'EOF'
import rclpy, numpy as np
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
from arm import Arm, JOINTS
arm = Arm()
cli = arm.node.create_client(GetPositionFK, "/compute_fk")
cli.wait_for_service(timeout_sec=10)
req = GetPositionFK.Request()
req.fk_link_names = ["panda_link8", "panda_hand", "panda_link0"]
js = JointState(); js.name = list(JOINTS); js.position = arm.arm_q()
req.robot_state.joint_state = js
fut = cli.call_async(req); rclpy.spin_until_future_complete(arm.node, fut, timeout_sec=60)
r = fut.result()
print("code", r.error_code.val)
for n, ps in zip(r.fk_link_names, r.pose_stamped):
    p, q = ps.pose.position, ps.pose.orientation
    print(n, ps.header.frame_id, np.round([p.x,p.y,p.z],4), np.round([q.x,q.y,q.z,q.w],4))
EOF
timeout 20 ros2 param list /move_group 2>/dev/null | grep -i -E "kinematics|tip|panda_arm" | head

# openrua op 22
sed -i 's/^BASE_IN_WORLD = np.array(\[-0.66, 0.0, 0.912\]).*/BASE_IN_WORLD = np.array([0.0, 0.0, 0.0])  # IK model frame == world (verified via \/compute_fk)/' arm.py && grep -n BASE_IN_WORLD arm.py | head -2 && timeout 300 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm
arm = Arm()
p,q,tcp = arm.hand_pose()
tests = {
 "current tcp": (tcp, q),
 "knob +10cm q ideal": ([-0.2015,0.2065,1.05], [1,0,0,0]),
 "knob +4cm q ideal": ([-0.2015,0.2065,1.00], [1,0,0,0]),
}
for k,(pos,qq) in tests.items():
    try:
        sol = arm.solve_ik(pos, qq, timeout=30)
        print(k, "OK", np.round(sol,3))
    except Exception as e:
        print(k, "FAIL", e)
rclpy.shutdown()
EOF

# openrua op 23
timeout 900 python3 -u arm.py move -0.2015 0.2065 1.00 1 0 0 0 4 2>&1 | tail -3

# openrua op 24
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "    def solve_ik(self, tcp_world, quat, seed=None, timeout=60):\\n        \\"\\"\\"IK for the hand such that the TCP sits at tcp_world (world frame).\\"\\"\\"\\n        R = quat_R(*quat)\\n        hand_world = np.asarray(tcp_world, float) - TCP * R[:, 2]\\n        hand_base = hand_world - BASE_IN_WORLD", "new_string": "    def solve_ik(self, tcp_world, quat, seed=None, timeout=60):\\n        \\"\\"\\"IK for the hand such that the TCP sits at tcp_world (world frame).\\n        quat is the desired panda_hand orientation; the IK tip is panda_link8\\n        (= hand rotated +45 deg about z), so convert.\\"\\"\\"\\n        R = quat_R(*quat)\\n        hand_world = np.asarray(tcp_world, float) - TCP * R[:, 2]\\n        hand_base = hand_world - BASE_IN_WORLD\\n        quat = quat_mul(quat, [0.0, 0.0, 0.3826834, 0.9238795])", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 25
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/arm.py", "old_string": "class Arm:", "new_string": "def quat_mul(a, b):\\n    \\"\\"\\"Hamilton product a*b, both (x,y,z,w).\\"\\"\\"\\n    ax, ay, az, aw = a; bx, by, bz, bw = b\\n    return [aw * bx + ax * bw + ay * bz - az * by,\\n            aw * by - ax * bz + ay * bw + az * bx,\\n            aw * bz + ax * by - ay * bx + az * bw,\\n            aw * bw - ax * bx - ay * by - az * bz]\\n\\n\\nclass Arm:", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 26
timeout 900 python3 -u arm.py move -0.2015 0.2065 1.00 1 0 0 0 3 2>&1 | tail -3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 27
timeout 900 python3 -u arm.py move -0.2015 0.2065 1.00 1 0 0 0 3 2>&1 | tail -3 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 28
timeout 60 python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
m=(xyz[...,0]>-0.30)&(xyz[...,0]<-0.12)&(xyz[...,1]>0.12)&(xyz[...,1]<0.30)&(z>0.905)&(z<0.99)
p=xyz[m]
print("knob z pct", np.round(np.percentile(p[:,2],[5,25,50,75,95,100]),3))
for zlo,zhi in [(0.905,0.93),(0.93,0.95),(0.95,0.97)]:
    q=p[(p[:,2]>=zlo)&(p[:,2]<zhi)]
    print(f"z[{zlo},{zhi}) n={len(q)} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] c={(q[:,0].min()+q[:,0].max())/2:.4f}  y[{q[:,1].min():.3f},{q[:,1].max():.3f}] c={(q[:,1].min()+q[:,1].max())/2:.4f}")
EOF

# openrua op 29
timeout 900 python3 -u arm.py move -0.2015 0.207 0.940 1 0 0 0 2.5 2>&1 | tail -2 && timeout 300 python3 arm.py grip close && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 30
timeout 60 python3 tools/perception/cam_snap.py robot0_robotview rv_before.png >/dev/null; timeout 900 python3 -u arm.py j7 1.5708 3 2>&1 | tail -2; timeout 60 python3 tools/perception/cam_snap.py robot0_robotview rv_after.png; timeout 60 python3 tools/perception/cam_snap.py agentview av_after.png

# openrua op 31
timeout 60 python3 arm.py js | tr ',' '\n' | grep -E "joint7|finger"; python3 -c "
import cv2, numpy as np
a=cv2.imread('rv_before.png'); b=cv2.imread('rv_after.png')
cv2.imwrite('rv_cmp.png', np.hstack([a[60:260,380:640], b[60:260,380:640]]))
"

# openrua op 32
timeout 60 python3 arm.py hand

# openrua op 33
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_now.png >/dev/null; python3 -c "
import cv2; im=cv2.imread('eih_now.png'); cv2.imwrite('eih_crop.png', cv2.resize(im[200:480,120:520],None,fx=1.5,fy=1.5))"

# openrua op 34
timeout 900 python3 -u arm.py j7 -0.8 3 2>&1 | tail -2; timeout 60 python3 arm.py hand; timeout 60 python3 arm.py js | tr ',' '\n' | grep finger

# openrua op 35
timeout 900 python3 -u arm.py j7 -0.3 4 2>&1 | tail -3; timeout 60 python3 arm.py hand

# openrua op 36
timeout 900 python3 -u arm.py j7 -0.6 6 2>&1 | tail -3; timeout 60 python3 arm.py hand; timeout 60 python3 tools/perception/cam_snap.py agentview av2.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png > /dev/null; python3 -c "
import cv2,numpy as np; a=cv2.imread('av2.png'); e=cv2.imread('eih2.png'); cv2.imwrite('cmp2.png', np.hstack([cv2.resize(a[180:380,340:540],None,fx=2,fy=2), cv2.resize(e[160:480,80:560],(400,400))]))"

# openrua op 37
timeout 300 python3 arm.py grip open && timeout 900 python3 -u arm.py move -0.1973 0.2031 1.06 0.8346 0.5508 0 0 2.5 2>&1 | tail -2; timeout 60 python3 tools/perception/cam_snap.py agentview av3.png >/dev/null; python3 -c "
import cv2; a=cv2.imread('av3.png'); cv2.imwrite('av3c.png', cv2.resize(a[180:380,340:540],None,fx=2,fy=2))"

# openrua op 38
timeout 900 python3 -u arm.py move -0.047 -0.229 1.20 0.7071 -0.7071 0 0 4 2>&1 | tail -2; timeout 60 python3 arm.py hand

# openrua op 39
timeout 900 python3 -u arm.py move -0.047 -0.229 1.20 0.7071 -0.7071 0 0 4 2>&1 | tail -2

# openrua op 40
timeout 900 python3 -u arm.py move -0.047 -0.229 1.20 0.7071 -0.7071 0 0 5 2>&1 | tail -2

# openrua op 41
timeout 60 python3 cloud.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_pot.png >/dev/null && python3 - <<'EOF'
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
m=(xyz[...,0]>-0.15)&(xyz[...,0]<0.05)&(xyz[...,1]>-0.35)&(xyz[...,1]<-0.12)&(z>0.905)&(z<1.07)
p=xyz[m]
print("pot n", len(p), "z pct", np.round(np.percentile(p[:,2],[5,50,95,100]),3))
for zlo in np.arange(0.92,1.06,0.01):
    q=p[(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)]
    if len(q)>3:
        # exclude handle: y < -0.27 ; and spout y>-0.19
        b=q[(q[:,1]>-0.275)&(q[:,1]<-0.185)]
        if len(b)>3:
            print(f"  z {zlo:.2f}: n={len(q):4d} body x[{b[:,0].min():.3f},{b[:,0].max():.3f}] w={b[:,0].max()-b[:,0].min():.3f} cx={(b[:,0].min()+b[:,0].max())/2:.4f}  y[{b[:,1].min():.3f},{b[:,1].max():.3f}]  full y[{q[:,1].min():.3f},{q[:,1].max():.3f}]")
top = p[p[:,2]>1.045]
print("lid knob center", top[:,0].mean(), top[:,1].mean())
EOF

# openrua op 42
timeout 60 python3 - <<'EOF'
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String
rclpy.init(); n=rclpy.create_node("urdf")
qos=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
got=[]
n.create_subscription(String, "/robot_description", got.append, qos)
import time
for _ in range(50):
    rclpy.spin_once(n, timeout_sec=0.2)
    if got: break
open("robot.urdf","w").write(got[0].data); print(len(got[0].data))
EOF
grep -n -A12 '<link name="panda_hand"' robot.urdf | head -60; grep -n -A12 '<link name="panda_leftfinger"' robot.urdf | head -40

# openrua op 43
python3 - <<'EOF'
import struct, numpy as np
def stl_bounds(path):
    data=open(path,'rb').read()
    n=struct.unpack_from('<I',data,80)[0]
    v=np.frombuffer(data[84:84+n*50], dtype=np.dtype([('n','<3f4'),('v','<9f4'),('a','<u2')]))['v'].reshape(-1,3)
    return v.min(0), v.max(0)
for f in ['hand','finger']:
    lo,hi=stl_bounds(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl')
    print(f, 'min',np.round(lo,4),'max',np.round(hi,4))
EOF

# openrua op 44
find / -iname "hand*.stl" -o -iname "finger*.stl" -o -iname "hand.dae" 2>/dev/null | head; find / -path /proc -prune -o -iname "*moka*" -print 2>/dev/null | head; find / -path /proc -prune -o -iname "*stove*" -print 2>/dev/null | head

# openrua op 45
mkdir -p "$(dirname /workspace/grasp_geom.py)"
cat > /workspace/grasp_geom.py <<'OPENRUA_EOF'
"""Shared geometry for the diagonal, downward-tilted horizontal grasp."""
import numpy as np
from scipy.spatial.transform import Rotation as Rot

TILT = np.deg2rad(20.0)              # approach pitched below horizontal
DIR_H = np.array([0.70, -0.71, 0.0]); DIR_H /= np.linalg.norm(DIR_H)  # horizontal approach dir (+x,-y)
Z_H = np.array([DIR_H[0] * np.cos(TILT), DIR_H[1] * np.cos(TILT), -np.sin(TILT)])  # hand z (palm->tips)
Y_H = np.array([-DIR_H[1], DIR_H[0], 0.0])  # finger axis, horizontal, perpendicular
X_H = np.cross(Y_H, Z_H)
R_HAND = np.column_stack([X_H, Y_H, Z_H])
Q_HAND = Rot.from_matrix(R_HAND).as_quat()  # x,y,z,w

SHORT = 0.015   # TCP stops this far short of the pot axis along the approach


def tcp_for(axis_xy, z_tcp, back=0.0):
    """TCP position for grasping a vertical axis at axis_xy with pads at z_tcp;
    back>0 retreats along -Z_H."""
    p = np.array([axis_xy[0], axis_xy[1], z_tcp]) - SHORT * DIR_H
    return p - back * Z_H


if __name__ == "__main__":
    print("Z_H", np.round(Z_H, 4), "Y_H", np.round(Y_H, 4), "X_H", np.round(X_H, 4))
    print("Q_HAND", np.round(Q_HAND, 5))
    print("pre", np.round(tcp_for((-0.0443, -0.229), 0.972, 0.20), 4))
    print("final", np.round(tcp_for((-0.0443, -0.229), 0.972), 4))
OPENRUA_EOF

# openrua op 46
python3 grasp_geom.py && timeout 300 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm
from grasp_geom import *
arm = Arm()
pre = tcp_for((-0.0443,-0.229), 0.972, 0.20)
fin = tcp_for((-0.0443,-0.229), 0.972)
for name, p in [("pre", pre), ("final", fin)]:
    try:
        q = arm.solve_ik(p, Q_HAND, timeout=30); print(name, "IK OK", np.round(q,3))
    except Exception as e:
        print(name, "IK FAIL", e)
# flipped finger axis
from scipy.spatial.transform import Rotation as Rot
R2 = np.column_stack([-X_H, -Y_H, Z_H]); q2 = Rot.from_matrix(R2).as_quat()
for name, p in [("pre-flip", pre), ("final-flip", fin)]:
    try:
        q = arm.solve_ik(p, q2, timeout=30); print(name, "IK OK", np.round(q,3))
    except Exception as e:
        print(name, "IK FAIL", e)
rclpy.shutdown()
EOF

# openrua op 47
timeout 600 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm, JOINTS
from grasp_geom import *
from scipy.spatial.transform import Rotation as Rot
arm = Arm()
pre = tcp_for((-0.0443,-0.229), 0.972, 0.20)
mid = tcp_for((-0.0443,-0.229), 0.972, 0.10)
fin = tcp_for((-0.0443,-0.229), 0.972)
q2 = Rot.from_matrix(np.column_stack([-X_H, -Y_H, Z_H])).as_quat()
rng = np.random.default_rng(0)
lo = np.array([-2.9,-1.76,-2.9,-3.07,-2.9,-0.02,-2.9]); hi = np.array([2.9,1.76,2.9,-0.07,2.9,3.75,2.9])
cur = np.array(arm.arm_q())
seeds = [cur, np.array([0,-0.785,0,-2.356,0,1.571,0.785])] + [rng.uniform(lo,hi) for _ in range(10)]
best = []
for oi, quat in enumerate([Q_HAND, q2]):
    for s in seeds:
        try:
            qf = np.array(arm.solve_ik(fin, quat, seed=s, timeout=20))
            qm = np.array(arm.solve_ik(mid, quat, seed=qf, timeout=20))
            qp = np.array(arm.solve_ik(pre, quat, seed=qm, timeout=20))
        except Exception as e:
            continue
        d1 = np.abs(qf-qm).max(); d2 = np.abs(qm-qp).max(); dc = np.abs(qp-cur).max()
        best.append((max(d1,d2), oi, np.round(qp,3), np.round(qm,3), np.round(qf,3), round(dc,2)))
best.sort(key=lambda t: t[0])
for b in best[:6]:
    print("maxstep", round(b[0],3), "orient", b[1], "dist_from_cur", b[5]); print("  pre", b[2]); print("  mid", b[3]); print("  fin", b[4])
rclpy.shutdown()
EOF

# openrua op 48
timeout 600 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm, JOINTS
from grasp_geom import *
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
arm = Arm()
fk = arm.node.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service(timeout_sec=10)
def FK(q, links=("panda_hand","panda_leftfinger","panda_rightfinger","panda_link7","panda_link6","panda_link5")):
    req = GetPositionFK.Request(); req.fk_link_names = list(links)
    js = JointState(); js.name = list(JOINTS); js.position = [float(v) for v in q]
    req.robot_state.joint_state = js
    fut = fk.call_async(req); rclpy.spin_until_future_complete(arm.node, fut, timeout_sec=30)
    r = fut.result()
    return {n: np.array([p.pose.position.x,p.pose.position.y,p.pose.position.z]) for n,p in zip(r.fk_link_names, r.pose_stamped)}, r.pose_stamped[0].pose.orientation

base = np.array([-0.831, 0.326, 0.856, -2.655, -2.252, 1.917, -2.741])
pre = tcp_for((-0.0443,-0.229), 0.972, 0.20); mid = tcp_for((-0.0443,-0.229), 0.972, 0.10); fin = tcp_for((-0.0443,-0.229), 0.972)
seed = base.copy(); seed[6] += np.pi
qp = np.array(arm.solve_ik(pre, Q_HAND, seed=seed)); qm = np.array(arm.solve_ik(mid, Q_HAND, seed=qp)); qf = np.array(arm.solve_ik(fin, Q_HAND, seed=qm))
print("pre", np.round(qp,3)); print("mid", np.round(qm,3)); print("fin", np.round(qf,3))
print("steps", np.abs(qp-qm).max(), np.abs(qm-qf).max())
for name,q in [("pre",qp),("fin",qf)]:
    P, o = FK(q); print(name, {k: np.round(v,3) for k,v in P.items()}, "hand q", np.round([o.x,o.y,o.z,o.w],3))
cur = np.array(arm.arm_q()); print("cur", np.round(cur,3))
print("interp cur->pre min z per link:")
mins = {}
for t in np.linspace(0,1,21):
    P,_ = FK(cur*(1-t)+qp*t)
    for k,v in P.items(): mins[k] = min(mins.get(k,9), v[2])
    if t in (0.25,0.5,0.75): print(" t",t, {k: np.round(v,3) for k,v in P.items()})
print(mins)
np.save("q_pre.npy", qp); np.save("q_mid.npy", qm); np.save("q_fin.npy", qf)
rclpy.shutdown()
EOF

# openrua op 49
timeout 300 python3 arm.py grip open; timeout 900 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm
arm = Arm()
qp = np.load("q_pre.npy")
arm.move_to(qp, 5.0)
p,q,tcp = arm.hand_pose(); print("hand", np.round(p,4), "q", np.round(q,4), "tcp", np.round(tcp,4))
print("fingers", arm.fingers())
rclpy.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_pre.png; timeout 60 python3 tools/perception/cam_snap.py agentview av_pre.png

# openrua op 50
timeout 900 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm
arm = Arm()
qp = np.load("q_pre.npy")
print("cur", np.round(arm.arm_q(),3)); print("tgt", np.round(qp,3))
for i in range(3):
    code, err = arm.move_to(qp, 6.0)
    if code == 0 and err < 0.01: break
p,q,tcp = arm.hand_pose(); print("hand", np.round(p,4), "q", np.round(q,4), "tcp", np.round(tcp,4))
rclpy.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_pre.png; timeout 60 python3 tools/perception/cam_snap.py agentview av_pre.png

# openrua op 51
timeout 60 python3 cloud.py robot0_eye_in_hand && python3 - <<'EOF'
import numpy as np
from grasp_geom import DIR_H, Y_H
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
m=(xyz[...,0]>-0.15)&(xyz[...,0]<0.05)&(xyz[...,1]>-0.35)&(xyz[...,1]<-0.12)&(z>0.905)&(z<1.07)
p=xyz[m]; 
ax=np.array([-0.0443,-0.229])
lat = (p[:,:2]-ax)@Y_H[:2]     # lateral offset (finger axis)
alo = (p[:,:2]-ax)@DIR_H[:2]   # along approach (negative = nearer camera)
print("n",len(p))
for zlo in np.arange(0.90,1.06,0.01):
    s=(p[:,2]>=zlo)&(p[:,2]<zlo+0.01)
    if s.sum()>5:
        # body only: exclude handle/spout by along-approach window? handle is at -y: lateral ~ -0.7*.., keep |lat|<0.05
        print(f"z {zlo:.2f}: n={s.sum():4d} lat[{lat[s].min():+.3f},{lat[s].max():+.3f}] width={lat[s].max()-lat[s].min():.3f} mid={(lat[s].max()+lat[s].min())/2:+.4f}  along[{alo[s].min():+.3f},{alo[s].max():+.3f}]")
EOF

# openrua op 52
python3 - <<'EOF'
import numpy as np
from grasp_geom import DIR_H, Y_H
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']; z=xyz[...,2]
m=(xyz[...,0]>-0.15)&(xyz[...,0]<0.05)&(xyz[...,1]>-0.35)&(xyz[...,1]<-0.12)&(z>0.905)&(z<1.07)
p=xyz[m]; ax=np.array([-0.0443,-0.229])
lat=(p[:,:2]-ax)@Y_H[:2]; alo=(p[:,:2]-ax)@DIR_H[:2]
for zlo in np.arange(0.94,1.04,0.005):
    s=(p[:,2]>=zlo)&(p[:,2]<zlo+0.005)&(alo<0.0)
    if s.sum()>5:
        print(f"z {zlo:.3f}: n={s.sum():4d} lat[{lat[s].min():+.3f},{lat[s].max():+.3f}] width={lat[s].max()-lat[s].min():.3f} mid={(lat[s].max()+lat[s].min())/2:+.4f} along_min={alo[s].min():+.3f}")
EOF

# openrua op 53
timeout 900 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm
from grasp_geom import *
arm = Arm()
ax=(-0.0443,-0.229); zt=0.963
qp = np.array(arm.arm_q())
qm = np.array(arm.solve_ik(tcp_for(ax, zt, 0.10), Q_HAND, seed=qp))
qf = np.array(arm.solve_ik(tcp_for(ax, zt, 0.0), Q_HAND, seed=qm))
print("steps", np.abs(qp-qm).max(), np.abs(qm-qf).max())
np.save("q_fin.npy", qf)
code, err = arm.traj([qm, qf], [3.0, 6.0])
if err > 0.01:
    code, err = arm.traj([qf], [3.0])
p,q,tcp = arm.hand_pose(); print("tcp", np.round(tcp,4), "target", np.round(tcp_for(ax, zt),4), "q", np.round(q,4))
rclpy.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_fin.png; timeout 60 python3 tools/perception/cam_snap.py sideview sv_fin.png

# openrua op 54
python3 -c "
import cv2,numpy as np; a=cv2.imread('sv_fin.png'); e=cv2.imread('eih_fin.png'); cv2.imwrite('cmp_fin.png', np.hstack([cv2.resize(a[150:350,180:420],(480,400)), cv2.resize(e,(533,400))]))"

# openrua op 55
timeout 300 python3 arm.py grip close; timeout 60 python3 tools/perception/cam_snap.py sideview sv_grasp.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih_grasp.png >/dev/null; python3 -c "
import cv2,numpy as np; a=cv2.imread('sv_grasp.png'); e=cv2.imread('eih_grasp.png'); cv2.imwrite('cmp_grasp.png', np.hstack([cv2.resize(a[150:350,180:420],(480,400)), cv2.resize(e,(533,400))]))"

# openrua op 56
timeout 900 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm
from grasp_geom import *
arm = Arm()
p,q,tcp = arm.hand_pose()
q0 = np.array(arm.arm_q())
up = tcp + np.array([0,0,0.10])
qu = np.array(arm.solve_ik(up, Q_HAND, seed=q0))
print("step", np.abs(qu-q0).max())
arm.traj([qu],[4.0])
p,q,tcp2 = arm.hand_pose(); print("tcp", np.round(tcp2,4)); print("fingers", arm.fingers())
rclpy.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py sideview sv_lift.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview av_lift.png >/dev/null; python3 -c "
import cv2,numpy as np; a=cv2.imread('sv_lift.png'); e=cv2.imread('av_lift.png'); cv2.imwrite('cmp_lift.png', np.hstack([a[100:400,150:450], e[120:420,80:380]]))"

# openrua op 57
timeout 60 python3 cloud.py birdview bv2.npz >/dev/null && python3 - <<'EOF'
import numpy as np
d=np.load('bv2.npz'); xyz=d['xyz']; z=xyz[...,2]; col=d['color'].astype(int)
m=(xyz[...,0]>-0.16)&(xyz[...,0]<0.05)&(xyz[...,1]>0.10)&(xyz[...,1]<0.31)&(z>0.915)&(z<0.945)
p=xyz[m]; c=col[m]
print("plate x[%.3f,%.3f] y[%.3f,%.3f]"%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()), "z pct", np.round(np.percentile(p[:,2],[5,50,95]),4))
red = (c[:,0]>150)&(c[:,1]<100)
print("red coil px", red.sum(), "center", np.round(p[red,:2].mean(0),4), "x[%.3f,%.3f] y[%.3f,%.3f]"%(p[red,0].min(),p[red,0].max(),p[red,1].min(),p[red,1].max()))
hi = p[:,2]>0.931
print("raised px", hi.sum(), "center", np.round(p[hi,:2].mean(0),4))
EOF

# openrua op 58
python3 - <<'EOF'
import numpy as np
d=np.load('bv2.npz'); xyz=d['xyz']; z=xyz[...,2]; col=d['color'].astype(int)
m=(xyz[...,0]>-0.16)&(xyz[...,0]<0.05)&(xyz[...,1]>0.10)&(xyz[...,1]<0.31)&(z>0.915)&(z<0.945)
p=xyz[m]; c=col[m]
print("color stats", c.min(0), c.max(0))
red = (c[:,0]>c[:,1]+40)
print("reddish px", red.sum())
if red.sum():
    print("center", np.round(p[red,:2].mean(0),4), "x[%.3f,%.3f] y[%.3f,%.3f]"%(p[red,0].min(),p[red,0].max(),p[red,1].min(),p[red,1].max()))
hi = p[:,2]>0.928
print("raised px", hi.sum(), "center", np.round(p[hi,:2].mean(0),4), "x[%.3f,%.3f] y[%.3f,%.3f]"%(p[hi,0].min(),p[hi,0].max(),p[hi,1].min(),p[hi,1].max()))
EOF

# openrua op 59
timeout 900 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm
from grasp_geom import *
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
arm = Arm()
burner=(-0.046,0.207)
q0=np.array(arm.arm_q()); p,q,tcp=arm.hand_pose()
wp = [tcp+np.array([0,0,0.07]), tcp_for(burner,1.13), tcp_for(burner,1.005)]
qs=[]; seed=q0
for w in wp:
    seed=np.array(arm.solve_ik(w, Q_HAND, seed=seed)); qs.append(seed)
    print("wp", np.round(w,4), "q", np.round(seed,3))
print("steps", [round(float(np.abs(a-b).max()),3) for a,b in zip([q0]+qs[:-1], qs)])
fk = arm.node.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service(timeout_sec=10)
req = GetPositionFK.Request(); req.fk_link_names=["panda_hand","panda_link7","panda_link6"]
js=JointState(); js.name=list(arm.arm_q.__globals__['JOINTS']); js.position=[float(v) for v in qs[-1]]
req.robot_state.joint_state=js
fut=fk.call_async(req); rclpy.spin_until_future_complete(arm.node, fut, timeout_sec=30)
for n,ps in zip(fut.result().fk_link_names, fut.result().pose_stamped):
    pp=ps.pose.position; print("place-low", n, np.round([pp.x,pp.y,pp.z],3))
hand = np.array([fut.result().pose_stamped[0].pose.position.x, fut.result().pose_stamped[0].pose.position.y, fut.result().pose_stamped[0].pose.position.z])
for s in (-0.105, 0.105):
    e = hand + s*Y_H; print("hand end", np.round(e,3), "dist to knob axis", round(float(np.hypot(e[0]+0.2015, e[1]-0.207)),3))
np.save("q_place.npy", np.array(qs))
rclpy.shutdown()
EOF

# openrua op 60
timeout 1200 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm
from grasp_geom import *
arm = Arm()
burner=(-0.046,0.207)
q0=np.array(arm.arm_q()); p,q,tcp=arm.hand_pose()
# step 1: up
w_up = tcp+np.array([0,0,0.07])
q_up = np.array(arm.solve_ik(w_up, Q_HAND, seed=q0))
arm.traj([q_up],[3.0])
# step 2: across in 5 segments
p,q,tcp=arm.hand_pose(); target = tcp_for(burner,1.13)
pts=[]; seed=np.array(arm.arm_q()); times=[]
for i,t in enumerate(np.linspace(0,1,6)[1:]):
    w = tcp*(1-t)+target*t
    seed=np.array(arm.solve_ik(w, Q_HAND, seed=seed)); pts.append(seed); times.append(2.0*(i+1))
print("across steps", [round(float(np.abs(a-b).max()),3) for a,b in zip([np.array(arm.arm_q())]+pts[:-1], pts)])
code,err = arm.traj(pts, times)
if err>0.01: arm.traj([pts[-1]],[3.0])
p,q,tcp=arm.hand_pose(); print("tcp after across", np.round(tcp,4), "fingers", arm.fingers())
rclpy.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py agentview av_over.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py sideview sv_over.png >/dev/null; python3 -c "
import cv2,numpy as np; a=cv2.imread('sv_over.png'); e=cv2.imread('av_over.png'); cv2.imwrite('cmp_over.png', np.hstack([a[60:360,150:450], e[60:360,250:550]]))"

# openrua op 61
timeout 900 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm
from grasp_geom import *
arm = Arm()
burner=(-0.046,0.207)
q0=np.array(arm.arm_q())
q_low=np.array(arm.solve_ik(tcp_for(burner,1.005), Q_HAND, seed=q0))
print("step", np.abs(q_low-q0).max())
arm.traj([q_low],[4.0])
p,q,tcp=arm.hand_pose(); print("tcp", np.round(tcp,4))
arm.grip(0.04)
rclpy.shutdown()
EOF
timeout 60 python3 tools/perception/cam_snap.py sideview sv_rel.png >/dev/null; timeout 60 python3 tools/perception/cam_snap.py agentview av_rel.png >/dev/null; python3 -c "
import cv2,numpy as np; a=cv2.imread('sv_rel.png'); e=cv2.imread('av_rel.png'); cv2.imwrite('cmp_rel.png', np.hstack([a[100:400,150:450], e[100:400,250:550]]))"

# openrua op 62
timeout 900 python3 - <<'EOF'
import numpy as np, rclpy
from arm import Arm
from grasp_geom import *
arm = Arm()
q0=np.array(arm.arm_q()); p,q,tcp=arm.hand_pose()
w1 = tcp - 0.15*Z_H
q1=np.array(arm.solve_ik(w1, Q_HAND, seed=q0)); print("step", np.abs(q1-q0).max())
arm.traj([q1],[4.0])
w2 = w1 + np.array([-0.05,0,0.10])
q2=np.array(arm.solve_ik(w2, Q_HAND, seed=q1)); print("step", np.abs(q2-q1).max())
arm.traj([q2],[4.0])
p,q,tcp=arm.hand_pose(); print("tcp", np.round(tcp,4))
rclpy.shutdown()
EOF
for c in agentview birdview sideview frontview; do timeout 60 python3 tools/perception/cam_snap.py $c final_$c.png >/dev/null; done; timeout 60 python3 cloud.py birdview bv_final.npz >/dev/null; python3 - <<'EOF'
import numpy as np
d=np.load('bv_final.npz'); xyz=d['xyz']; z=xyz[...,2]; col=d['color'].astype(int)
# pot on stove region
m=(xyz[...,0]>-0.16)&(xyz[...,0]<0.05)&(xyz[...,1]>0.10)&(xyz[...,1]<0.31)
p=xyz[m]; c=col[m]
top=p[p[:,2]>1.02]; knob=p[p[:,2]>1.045]
print("pot lid px", len(top), "lid center", np.round(top[:,:2].mean(0),4), "lid z", np.round(np.percentile(top[:,2],[50,95,100]),3))
print("lid knob center", np.round(knob[:,:2].mean(0),4) if len(knob) else None)
red=(c[:,0]>c[:,1]+40)&(p[:,2]<0.95)
print("red coil px visible", red.sum(), "coil center", np.round(p[red,:2].mean(0),4))
# anything left at original pot location?
m2=(xyz[...,0]>-0.12)&(xyz[...,0]<0.02)&(xyz[...,1]>-0.33)&(xyz[...,1]<-0.15)&(z>0.91)
print("px above table at old pot spot:", m2.sum())
EOF

# openrua op 63
python3 -c "
import cv2,numpy as np; a=cv2.imread('final_agentview.png'); b=cv2.imread('final_birdview.png'); cv2.imwrite('final_cmp.png', np.hstack([a, b]))"

# openrua op 64
python3 arm.py js 2>/dev/null | tr ',' '\n' | head -9 && python3 arm.py hand 2>/dev/null
