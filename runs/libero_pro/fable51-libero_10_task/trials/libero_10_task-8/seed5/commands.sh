#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -100

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md .gitignore; ls -la .ros tools

# openrua op 2
ls -la tools/action tools/perception; cat tools/perception/*.py tools/action/*.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | grep -v -E "parameter|describe" ; echo ---; timeout 30 ros2 node list

# openrua op 4
timeout 60 ros2 topic echo /joint_states --once; for c in agentview frontview birdview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 5
cd /workspace; for p in "330 318" "390 278" "262 280" "320 240"; do echo -n "birdview $p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done; echo; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -8; timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -8

# openrua op 6
cd /workspace; mkdir -p scripts; cat > scripts/cloud.py <<'EOF'
#!/usr/bin/env python3
"""Dump a world-frame point cloud (H,W,3) of a camera as <cam>_xyz.npy plus color."""
import sys, struct, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
import cv2

def grab(node, topic, T, timeout=30):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time; end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got["m"]

def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

cam = sys.argv[1]
rclpy.init(); node = rclpy.create_node("cloud")
buf = Buffer(); TransformListener(buf, node)
depth = grab(node, f"/{cam}/depth/image_raw", Image)
color = grab(node, f"/{cam}/color/image_raw", Image)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
d = CvBridge().imgmsg_to_cv2(depth, "passthrough").astype(np.float64)
bgr = CvBridge().imgmsg_to_cv2(color, "bgr8")
frame = f"{cam}_optical_frame"
import time; end = time.time() + 10
while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation; R = quat_R(q.x, q.y, q.z, q.w)
tr = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
H, W = d.shape
u, v = np.meshgrid(np.arange(W), np.arange(H))
pc = np.stack([(u-cx)*d/fx, (v-cy)*d/fy, d], -1)
xyz = pc @ R.T + tr
np.save(f"{cam}_xyz.npy", xyz); cv2.imwrite(f"{cam}.png", bgr)
print("saved", xyz.shape, "cam pos", tr)
rclpy.shutdown()
EOF
timeout 120 python3 scripts/cloud.py birdview; timeout 120 python3 scripts/cloud.py agentview

# openrua op 7
cd /workspace; python3 - <<'EOF'
import numpy as np
xyz = np.load("birdview_xyz.npy")
z = xyz[...,2]
# table height: mode of z in table area
tbl = z[(xyz[...,0]>-0.3)&(xyz[...,0]<0.4)&(np.abs(xyz[...,1])<0.4)]
print("table z candidates", np.percentile(tbl,[5,25,50,75,95]))
table_z = np.median(tbl)
for name,(cx,cy) in {"potA":(0.06,0.03),"potB":(-0.07,0.24),"stove":(-0.06,-0.21)}.items():
    m = (np.abs(xyz[...,0]-cx)<0.12)&(np.abs(xyz[...,1]-cy)<0.12)&(z>table_z+0.01)
    p = xyz[m]
    print(name, "n",m.sum(), "x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]"%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
    # top of object
    top = p[p[:,2]>p[:,2].max()-0.02]
    print("   top centroid", top.mean(0))
    # per-height slices
    for lo in np.arange(table_z+0.01, p[:,2].max(), 0.02):
        s = p[(p[:,2]>=lo)&(p[:,2]<lo+0.02)]
        if len(s): print("   z %.3f-%.3f: n=%d x[%.3f %.3f] y[%.3f %.3f]"%(lo,lo+0.02,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
EOF

# openrua op 8
cd /workspace; python3 - <<'EOF'
import numpy as np
xyz = np.load("agentview_xyz.npy")
z = xyz[...,2]
for name,(cx,cy) in {"potA":(0.063,0.033),"potB":(-0.07,0.24)}.items():
    m = (np.abs(xyz[...,0]-cx)<0.12)&(np.abs(xyz[...,1]-cy)<0.12)&(z>0.905)&(z<1.2)
    p = xyz[m]
    print(name, "n",m.sum())
    for lo in np.arange(0.905, 1.07, 0.01):
        s = p[(p[:,2]>=lo)&(p[:,2]<lo+0.01)]
        if len(s): print("   z %.3f: n=%3d x[%.3f %.3f] y[%.3f %.3f] ycen %.3f wy %.3f"%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max(),(s[:,1].min()+s[:,1].max())/2,s[:,1].max()-s[:,1].min()))
EOF

# openrua op 9
cd /workspace; timeout 120 python3 scripts/cloud.py sideview; python3 - <<'EOF'
import numpy as np
xyz = np.load("sideview_xyz.npy")
z = xyz[...,2]
for name,(cx,cy) in {"potB":(-0.07,0.24),"potA":(0.063,0.033)}.items():
    m = (np.abs(xyz[...,0]-cx)<0.12)&(np.abs(xyz[...,1]-cy)<0.12)&(z>0.905)&(z<1.2)
    p = xyz[m]
    print(name, "n",m.sum())
    for lo in np.arange(0.905, 1.07, 0.01):
        s = p[(p[:,2]>=lo)&(p[:,2]<lo+0.01)]
        if len(s): print("   z %.3f: n=%3d x[%.3f %.3f] wx %.3f xcen %.3f  y[%.3f %.3f]"%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,0].max()-s[:,0].min(),(s[:,0].min()+s[:,0].max())/2,s[:,1].min(),s[:,1].max()))
EOF

# openrua op 10
cd /workspace; timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | tail -20

# openrua op 11
mkdir -p "$(dirname /workspace/scripts/arm.py)"
cat > /workspace/scripts/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library for this Panda: joint state, FK/IK (world frame),
trajectory execution, gripper, servo bursts.

CLI:
  arm.py js                         print joint state + hand/TCP pose (FK)
  arm.py ik x y z qx qy qz qw       solve IK for TCP pose, print joints (no motion)
  arm.py goto x y z qx qy qz qw [sec]   IK for TCP pose, execute, verify
  arm.py joints p1,...,p7 [sec]     execute joint target, verify
  arm.py grip open|close            gripper
  arm.py servo dx dy dz n           n ticks of world-frame linear twist (m/s)
"""
import sys, time
from pathlib import Path

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

ROOT = Path(__file__).resolve().parents[1]
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
# hand orientation with fingers pointing down, opening along world X
Q_DOWN_X = (0.7071068, 0.7071068, 0.0, 0.0)
# fingers down, opening along world Y
Q_DOWN_Y = (1.0, 0.0, 0.0, 0.0)


def quat_R(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class Arm:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)

    def _on_js(self, m):
        self._js = m

    # ---------- sensing ----------
    def joint_state(self, fresh=True):
        if fresh:
            self._js = None
        end = time.time() + 30
        while self._js is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if self._js is None:
            raise RuntimeError("no /joint_states")
        return dict(zip(self._js.name, self._js.position))

    def arm_joints(self, js=None):
        js = js or self.joint_state()
        return [js[j] for j in JOINTS]

    def finger_gap(self, js=None):
        js = js or self.joint_state(fresh=False)
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    def fk_pose(self, joints=None):
        joints = joints or self.arm_joints()
        self.fk.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = JOINTS
        req.robot_state.joint_state.position = [float(x) for x in joints]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result()
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z])
        q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        tcp = pos + TCP_OFF * quat_R(*q)[:, 2]
        return pos, q, tcp

    # ---------- planning ----------
    def solve_ik(self, tcp_xyz, q, seed=None):
        """IK for a TCP pose in world frame. Returns 7 joints or None."""
        R = quat_R(*q)
        hand = np.array(tcp_xyz, dtype=float) - TCP_OFF * R[:, 2]
        self.ik.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.avoid_collisions = False
        seed = seed or self.arm_joints()
        req.ik_request.robot_state.joint_state.name = JOINTS
        req.ik_request.robot_state.joint_state.position = [float(x) for x in seed]
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            print("IK failed", None if res is None else res.error_code.val)
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [sol[j] for j in JOINTS]

    # ---------- acting ----------
    def move_joints(self, target, seconds=3.0, tol=0.02, retries=2):
        self.fjt.wait_for_server(timeout_sec=10)
        for attempt in range(retries + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = JOINTS
            pt = JointTrajectoryPoint(positions=[float(x) for x in target])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send)
            res = send.result().get_result_async()
            rclpy.spin_until_future_complete(self.node, res)
            code = res.result().result.error_code
            cur = self.arm_joints()
            err = max(abs(a - b) for a, b in zip(cur, target))
            print(f"traj error_code={code} max_joint_err={err:.4f}")
            if err < tol:
                return True
        return err < tol

    def goto_tcp(self, tcp_xyz, q, seconds=3.0):
        sol = self.solve_ik(tcp_xyz, q)
        if sol is None:
            return False
        ok = self.move_joints(sol, seconds)
        pos, qq, tcp = self.fk_pose()
        print(f"TCP now {tcp.round(4)} (target {np.round(tcp_xyz,4)}) ok={ok}")
        return ok

    def gripper(self, open_):
        self.grip.wait_for_server(timeout_sec=10)
        g = GripperCommand.Goal()
        g.command.position = float(GRIP["open_m"] if open_ else GRIP["closed_m"])
        g.command.max_effort = float(GRIP["max_effort"])
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=120)
        r = rf.result().result
        js = self.joint_state()
        print(f"gripper reached={r.reached_goal} stalled={r.stalled} gap={self.finger_gap(js):.4f}")
        return self.finger_gap(js)

    def servo(self, dx, dy, dz, ticks):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = float(dx), float(dy), float(dz)
        for _ in range(int(ticks)):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        stop = TwistStamped(); stop.header.frame_id = TWIST["frame"]
        for _ in range(3):
            self.twist_pub.publish(stop); rclpy.spin_once(self.node, timeout_sec=0.05)


def main():
    a = sys.argv[1:]
    arm = Arm()
    cmd = a[0]
    if cmd == "js":
        js = arm.joint_state()
        print({k: round(v, 4) for k, v in js.items()})
        pos, q, tcp = arm.fk_pose()
        print("hand", pos.round(4), "q", np.round(q, 4), "tcp", tcp.round(4), "gap", round(arm.finger_gap(js), 4))
    elif cmd == "ik":
        sol = arm.solve_ik([float(x) for x in a[1:4]], [float(x) for x in a[4:8]])
        print("sol", None if sol is None else ",".join(f"{x:.5f}" for x in sol))
        if sol:
            print("fk check", arm.fk_pose(sol)[2].round(4))
    elif cmd == "goto":
        sec = float(a[8]) if len(a) > 8 else 3.0
        arm.goto_tcp([float(x) for x in a[1:4]], [float(x) for x in a[4:8]], sec)
    elif cmd == "joints":
        sec = float(a[2]) if len(a) > 2 else 3.0
        arm.move_joints([float(x) for x in a[1].split(",")], sec)
        print("tcp", arm.fk_pose()[2].round(4))
    elif cmd == "grip":
        arm.gripper(a[1] == "open")
    elif cmd == "servo":
        arm.servo(float(a[1]), float(a[2]), float(a[3]), int(a[4]))
        print("tcp", arm.fk_pose()[2].round(4))
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 12
cd /workspace; timeout 120 python3 scripts/arm.py js; timeout 120 python3 scripts/arm.py ik 0.067 -0.038 1.15 0.7071068 0.7071068 0 0

# openrua op 13
cd /workspace; timeout 300 python3 scripts/arm.py grip open; timeout 600 python3 scripts/arm.py goto 0.067 -0.038 1.15 0.7071068 0.7071068 0 0 3

# openrua op 14
cd /workspace; timeout 120 python3 scripts/arm.py js

# openrua op 15
cd /workspace; timeout 600 python3 scripts/arm.py joints 0.02370,0.65208,-0.10352,-1.35404,0.06910,2.00262,-1.65844 2; timeout 120 python3 scripts/arm.py js

# openrua op 16
cd /workspace; python3 - <<'EOF'
import sys; sys.path.insert(0,"scripts")
from arm import *
arm = Arm()
for q in [Q_DOWN_X, Q_DOWN_Y, (0.7071068,-0.7071068,0,0)]:
    sol = arm.solve_ik([0.067,-0.038,1.15], q)
    if sol:
        pos, qq, tcp = arm.fk_pose(sol)
        print("req", q, "sol", [round(s,3) for s in sol], "fk q", np.round(qq,4), "tcp", tcp.round(4))
rclpy.shutdown()
EOF

# openrua op 17
cd /workspace; python3 - <<'EOF'
import numpy as np
def qmul(a,b):
    x1,y1,z1,w1=a; x2,y2,z2,w2=b
    return (w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2)
qh=(0.7071068,0.7071068,0,0)
for s in (1,-1):
    rz=(0,0,s*np.sin(np.pi/8),np.cos(np.pi/8))
    print(s, np.round(qmul(qh,rz),4))
EOF

# openrua op 18
cd /workspace; python3 - <<'EOF'
import re
s = open("scripts/arm.py").read()
s = s.replace('''def quat_R(x, y, z, w):''', '''def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return (w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2, w1*w2 - x1*x2 - y1*y2 - z1*z2)


# machine fact: the IK service solves for panda_link8, which is yawed 45 deg
# from panda_hand about the shared z axis (same origin). Convert hand quats.
Q_HAND_TO_LINK8 = (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8))


def quat_R(x, y, z, w):''')
s = s.replace('''        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
        req.ik_request.avoid_collisions = False''', '''        q8 = qmul(q, Q_HAND_TO_LINK8)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q8)
        req.ik_request.avoid_collisions = False''')
open("scripts/arm.py","w").write(s)
EOF
python3 - <<'EOF'
import sys; sys.path.insert(0,"/workspace/scripts")
from arm import *
arm = Arm()
sol = arm.solve_ik([0.067,-0.038,1.15], Q_DOWN_X)
pos, qq, tcp = arm.fk_pose(sol)
print("sol", [round(s,4) for s in sol], "fk q", np.round(qq,4), "tcp", tcp.round(4))
rclpy.shutdown()
EOF

# openrua op 19
cd /workspace; timeout 600 python3 scripts/arm.py goto 0.067 -0.038 1.15 0.7071068 0.7071068 0 0 3; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 20
cd /workspace; timeout 120 python3 scripts/cloud.py robot0_eye_in_hand 2>&1 | tail -3; python3 - <<'EOF'
import numpy as np
xyz = np.load("robot0_eye_in_hand_xyz.npy")
z = xyz[...,2]
m = (np.abs(xyz[...,0]-0.067)<0.1)&(np.abs(xyz[...,1]-0.03)<0.12)&(z>0.905)&(z<1.1)
p = xyz[m]
print("n", len(p))
for lo in np.arange(0.905, 1.07, 0.01):
    s = p[(p[:,2]>=lo)&(p[:,2]<lo+0.01)]
    if len(s): print("   z %.3f: n=%3d x[%.3f %.3f] y[%.3f %.3f]"%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
# handle region: y < -0.005
h = p[(p[:,1]<-0.005)]
print("handle pts", len(h))
for lo in np.arange(0.94, 1.05, 0.01):
    s = h[(h[:,2]>=lo)&(h[:,2]<lo+0.01)]
    if len(s): print("   handle z %.3f: n=%3d x[%.3f %.3f] xc %.3f y[%.3f %.3f]"%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,0].mean(),s[:,1].min(),s[:,1].max()))
EOF

# openrua op 21
cd /workspace; python3 - <<'EOF'
import numpy as np
xyz = np.load("robot0_eye_in_hand_xyz.npy")
z = xyz[...,2]
m = (np.abs(xyz[...,0]-0.067)<0.15)&(np.abs(xyz[...,1]-0.03)<0.2)&(z>0.8)&(z<0.92)
print("table z from wrist cam", np.percentile(z[m],[5,50,95]))
b = np.load("birdview_xyz.npy")
m2 = (np.abs(b[...,0]-0.067)<0.15)&(np.abs(b[...,1]-0.03)<0.2)&(b[...,2]>0.8)&(b[...,2]<0.92)
print("table z from birdview", np.percentile(b[...,2][m2],[5,50,95]))
EOF

# openrua op 22
cd /workspace; timeout 120 python3 scripts/cloud.py birdview >/dev/null; python3 - <<'EOF'
import numpy as np
xyz = np.load("birdview_xyz.npy"); z = xyz[...,2]
for name,(cx,cy) in {"potA":(0.066,0.033),"potB":(-0.07,0.24),"stove":(-0.06,-0.21)}.items():
    m = (np.abs(xyz[...,0]-cx)<0.12)&(np.abs(xyz[...,1]-cy)<0.12)&(z>0.905)&(z<1.2)
    p = xyz[m]
    top = p[p[:,2]>p[:,2].max()-0.02]
    print(name, "zmax %.3f"%p[:,2].max(), "top centroid", top.mean(0).round(4))
    if name=="potA":
        h = p[(p[:,1]<-0.005)&(p[:,2]>0.99)]
        print("  handle x[%.3f %.3f] y[%.3f %.3f] z[%.3f %.3f]"%(h[:,0].min(),h[:,0].max(),h[:,1].min(),h[:,1].max(),h[:,2].min(),h[:,2].max()))
EOF

# openrua op 23
cd /workspace; timeout 600 python3 scripts/arm.py goto 0.066 -0.032 1.06 0.7071068 0.7071068 0 0 2 && timeout 600 python3 scripts/arm.py goto 0.066 -0.032 1.012 0.7071068 0.7071068 0 0 2; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 24
cd /workspace; timeout 300 python3 scripts/arm.py grip close; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 25
cd /workspace; timeout 600 python3 scripts/arm.py goto 0.066 -0.032 1.17 0.7071068 0.7071068 0 0 3; timeout 120 python3 scripts/arm.py js | tail -1; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 26
cd /workspace; timeout 600 python3 scripts/arm.py goto -0.06 -0.275 1.17 0.7071068 0.7071068 0 0 4; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 27
cd /workspace; timeout 120 python3 scripts/cloud.py birdview >/dev/null; timeout 120 python3 scloud.py 2>/dev/null; python3 - <<'EOF'
import numpy as np
xyz = np.load("birdview_xyz.npy"); z = xyz[...,2]
m = (np.abs(xyz[...,0]+0.06)<0.15)&(np.abs(xyz[...,1]+0.21)<0.15)
for lo,hi in [(1.06,1.15),(1.15,1.2),(1.2,1.3),(0.93,0.96)]:
    s = xyz[m&(z>=lo)&(z<hi)]
    if len(s): print("z %.2f-%.2f n=%d x[%.3f %.3f] y[%.3f %.3f] cen (%.3f %.3f)"%(lo,hi,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max(),(s[:,0].min()+s[:,0].max())/2,(s[:,1].min()+s[:,1].max())/2))
EOF

# openrua op 28
cd /workspace; timeout 120 python3 scripts/cloud.py sideview >/dev/null; timeout 120 python3 scripts/cloud.py agentview >/dev/null; python3 - <<'EOF'
import numpy as np
for cam in ["sideview","agentview"]:
    xyz = np.load(f"{cam}_xyz.npy"); z = xyz[...,2]
    m = (np.abs(xyz[...,0]+0.06)<0.15)&(np.abs(xyz[...,1]+0.21)<0.15)
    print(cam)
    for lo,hi in [(1.06,1.10),(1.10,1.15),(1.15,1.19)]:
        s = xyz[m&(z>=lo)&(z<hi)]
        if len(s): print("  z %.2f-%.2f n=%d x[%.3f %.3f] y[%.3f %.3f]"%(lo,hi,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
EOF

# openrua op 29
cd /workspace; timeout 600 python3 scripts/arm.py goto -0.06 -0.275 1.10 0.7071068 0.7071068 0 0 2.5 && timeout 600 python3 scripts/arm.py goto -0.06 -0.275 1.072 0.7071068 0.7071068 0 0 2; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 30
cd /workspace; timeout 300 python3 scripts/arm.py grip open; timeout 600 python3 scripts/arm.py goto -0.06 -0.275 1.20 0.7071068 0.7071068 0 0 3; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py sideview

# openrua op 31
cd /workspace; timeout 600 python3 scripts/arm.py goto -0.06 -0.275 1.25 0.7071068 0.7071068 0 0 2 >/dev/null; timeout 120 python3 scripts/cloud.py sideview >/dev/null; timeout 120 python3 scripts/cloud.py agentview >/dev/null; timeout 120 python3 scripts/cloud.py birdview >/dev/null; python3 - <<'EOF'
import numpy as np
for cam in ["sideview","agentview","birdview"]:
    xyz = np.load(f"{cam}_xyz.npy"); z = xyz[...,2]
    m = (np.abs(xyz[...,0]+0.06)<0.15)&(np.abs(xyz[...,1]+0.21)<0.15)&(z>0.955)&(z<1.10)
    s = xyz[m]
    print(f"{cam}: pot pts n={len(s)} x[{s[:,0].min():.3f} {s[:,0].max():.3f}] y[{s[:,1].min():.3f} {s[:,1].max():.3f}] z[{s[:,2].min():.3f} {s[:,2].max():.3f}]")
xyz = np.load("birdview_xyz.npy"); z = xyz[...,2]
m = (np.abs(xyz[...,0]+0.06)<0.15)&(np.abs(xyz[...,1]+0.21)<0.15)&(z>0.91)&(z<0.96)
s = xyz[m]; print("stove/coil top z max %.3f"%s[:,2].max())
# pot B still in place?
m = (np.abs(xyz[...,0]+0.07)<0.12)&(np.abs(xyz[...,1]-0.24)<0.12)&(z>0.905)
s = xyz[m]; print("pot B top centroid", s[s[:,2]>s[:,2].max()-0.02].mean(0).round(3))
EOF

# openrua op 32
cd /workspace; python3 - <<'EOF'
import numpy as np
xyz = np.load("agentview_xyz.npy"); z = xyz[...,2]
# pot body only (exclude knob at x<-0.15)
m = (xyz[...,0]>-0.12)&(xyz[...,0]<0.0)&(np.abs(xyz[...,1]+0.21)<0.08)&(z>0.90)&(z<1.10)
p = xyz[m]
for lo in np.arange(0.90, 1.10, 0.01):
    s = p[(p[:,2]>=lo)&(p[:,2]<lo+0.01)]
    if len(s): print("z %.3f: n=%3d x[%.3f %.3f] y[%.3f %.3f]"%(lo,len(s),s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max()))
b = np.load("birdview_xyz.npy")
# coil height: birdview points around the stove but outside pot footprint
m = (np.abs(b[...,0]+0.06)<0.09)&(np.abs(b[...,1]+0.21)<0.09)&(b[...,2]>0.905)&(b[...,2]<1.0)
s=b[m]; print("stove surface pts z percentiles", np.percentile(s[:,2],[5,50,95,100]).round(3))
EOF

# openrua op 33
cd /workspace; timeout 600 python3 scripts/arm.py goto -0.2 0.0 1.25 1 0 0 0 4 | tail -1; timeout 60 python3 tools/perception/cam_snap.py agentview; timeout 60 python3 tools/perception/cam_snap.py birdview; timeout 120 python3 scripts/cloud.py birdview > /dev/null; python3 - <<'EOF'
import numpy as np
b = np.load("birdview_xyz.npy"); z=b[...,2]
m = (np.abs(b[...,0]+0.06)<0.15)&(np.abs(b[...,1]+0.21)<0.15)&(z>1.0)&(z<1.1)
s=b[m]; print("pot on stove (birdview, z>1.0): x[%.3f %.3f] y[%.3f %.3f] center (%.3f, %.3f) top z %.3f"%(s[:,0].min(),s[:,0].max(),s[:,1].min(),s[:,1].max(),(s[:,0].min()+s[:,0].max())/2,(s[:,1].min()+s[:,1].max())/2,s[:,2].max()))
EOF
