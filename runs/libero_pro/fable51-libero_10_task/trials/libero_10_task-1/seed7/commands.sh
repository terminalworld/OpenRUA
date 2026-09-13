#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 2
ros2 topic list; echo ---; ros2 action list; echo ---; ros2 service list | head -40; echo ---; ros2 node list

# openrua op 3
for c in agentview birdview frontview sideview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c & done; wait; ls *.png; ros2 topic echo /joint_states --once

# openrua op 4
ros2 node info /robot_bridge 2>&1 | head -80; ros2 service list | grep -v -E 'parameter|type_description' 

# openrua op 5
timeout 10 ros2 topic echo /tf_static --once 2>&1 | grep -E 'frame_id|x:|y:|z:|w:' | head -80; echo ---; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E 'frame_id' | head

# openrua op 6
timeout 10 ros2 topic echo /tf --once 2>&1 | head -120

# openrua op 7
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color+depth+info from a camera, save a world-frame point cloud.

Usage: python3 cloud.py <camera>   -> <camera>_cloud.npz (xyz HxWx3, bgr HxWx3)
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
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
    rclpy.init()
    node = rclpy.create_node("cloud")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    while not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    h, w = depth.shape
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    pc = np.stack([X, Y, depth], -1) @ R.T + T
    np.savez(f"{cam}_cloud.npz", xyz=pc, bgr=color, depth=depth, k=np.array(info.k))
    print(f"{cam}_cloud.npz", pc.shape, "fx", fx, "cx", cx, "cy", cy)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 cloud.py birdview && timeout 120 python3 cloud.py agentview && python3 -c "
import numpy as np
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
z=xyz[...,2]
print('z percentiles', np.percentile(z[np.isfinite(z)],[1,5,25,50,75,95,99]))
# table height guess: mode of z in central region
c=z[200:350,150:500]; 
hist,edges=np.histogram(c[np.isfinite(c)],bins=200); i=hist.argmax(); print('table z ~',edges[i],edges[i+1])
"

# openrua op 9
python3 -c "
import numpy as np, cv2
d=np.load('birdview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]
mask=(z>0.435)&(z<0.75)&np.isfinite(z)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask,connectivity=8)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    p=xyz[m]; c=bgr[m].mean(0)
    print(f'comp{i}: px={stats[i,4]} uv=({cent[i][0]:.0f},{cent[i][1]:.0f}) x=[{p[:,0].min():.3f},{p[:,0].max():.3f}] y=[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} zmed={np.median(p[:,2]):.3f} bgr={c.astype(int)}')
"


# openrua op 10
python3 -c "
import numpy as np, cv2
d=np.load('agentview_cloud.npz'); xyz=d['xyz']; bgr=d['bgr']
z=xyz[...,2]
mask=(z>0.44)&(z<0.75)&np.isfinite(z)&(xyz[...,0]>-0.4)
mask=mask.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask,connectivity=8)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    p=xyz[m]; c=bgr[m].mean(0)
    print(f'comp{i}: px={stats[i,4]} uv=({cent[i][0]:.0f},{cent[i][1]:.0f}) x=[{p[:,0].min():.3f},{p[:,0].max():.3f}] y=[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={p[:,2].max():.3f} zmed={np.median(p[:,2]):.3f} bgr={c.astype(int)}')
# specific pixels
for name,(u,v) in {'can_top':(357,245),'can_side':(357,270),'butter':(345,365),'basket_rim_far':(520,190),'basket_inside':(530,260),'ketchup':(236,200)}.items():
    print(name, xyz[v,u].round(3))
"


# openrua op 11
python3 -c "
import numpy as np, cv2
d=np.load('agentview_cloud.npz'); xyz=d['xyz']
def top(xr,yr,zmin):
    m=(xyz[...,0]>xr[0])&(xyz[...,0]<xr[1])&(xyz[...,1]>yr[0])&(xyz[...,1]<yr[1])&(xyz[...,2]>zmin)&(xyz[...,2]<0.7)
    p=xyz[m]; print('n',m.sum(),'center',p[:,:2].mean(0).round(3),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
print('can top:'); top((-0.2,0.0),(0.0,0.12),0.505)
print('butter top:'); top((0.0,0.15),(0.0,0.08),0.443)
print('basket rim:'); top((-0.15,0.15),(0.12,0.4),0.60)
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
print('BIRD can top:'); top((-0.2,0.0),(0.0,0.12),0.505)
print('BIRD butter top:'); top((0.0,0.15),(0.0,0.08),0.443)
print('BIRD basket rim:'); top((-0.15,0.15),(0.12,0.4),0.60)
print('BIRD basket floor:'); top((-0.06,0.04),(0.2,0.31),0.0)
"


# openrua op 12
python3 -c "
import numpy as np, cv2
d=np.load('agentview_cloud.npz'); xyz=d['xyz']
m=(xyz[...,0]>-0.2)&(xyz[...,0]<0.0)&(xyz[...,1]>0.0)&(xyz[...,1]<0.12)&(xyz[...,2]>0.505)&(xyz[...,2]<0.53)
p=xyz[m]; print('can top n',m.sum(),'center',p[:,:2].mean(0).round(3),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',p[:,2].min().round(3),p[:,2].max().round(3))
d=np.load('birdview_cloud.npz'); xyz=d['xyz']
m=(xyz[...,0]>-0.2)&(xyz[...,0]<0.0)&(xyz[...,1]>0.0)&(xyz[...,1]<0.12)
p=xyz[m]; print('bird region z range',p[:,2].min().round(3),p[:,2].max().round(3), 'n',m.sum())
m2=m&(xyz[...,2]>0.5)&(xyz[...,2]<0.53); p=xyz[m2]; print('bird can-top n',m2.sum(), p[:,:2].mean(0).round(3) if m2.sum() else None, p[:,0].min().round(3) if m2.sum() else '', p[:,0].max().round(3) if m2.sum() else '',p[:,1].min().round(3) if m2.sum() else '',p[:,1].max().round(3) if m2.sum() else '')
"


# openrua op 13
timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | tail -30

# openrua op 14
timeout 10 ros2 topic echo /tf --once 2>&1 | sed -n '120,200p'

# openrua op 15
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small controller library for this Panda workstation (reusable clients).

Subcommands (all poses in the planner frame; see fk to learn which):
  fk                         print hand pose from current joints
  ik x y z qx qy qz qw       print IK solution (no motion)
  move x y z qx qy qz qw [sec] [--tcp]   IK + trajectory, then verify
  joints p1,...,p7 [sec]     trajectory to joint target
  grip open|close            gripper command, prints finger gap
  servo dx dy dz [n]         stream n twist ticks (m/s) in base frame
  js                         print joint state
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
ARM = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("ctl")
        self._js = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik_cli = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk_cli = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)

    def _on_js(self, m):
        self._js = m

    def spin(self, t=0.1):
        rclpy.spin_once(self.node, timeout_sec=t)

    def js(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.2)
        return dict(zip(self._js.name, self._js.position))

    def arm_q(self):
        j = self.js()
        return [j[n] for n in ARM]

    def finger_gap(self):
        j = self.js()
        return abs(j.get("panda_finger_joint1", 0)) + abs(j.get("panda_finger_joint2", 0))

    # ---- kinematics
    def fk(self, q=None, link="panda_hand"):
        q = q or self.arm_q()
        self.fk_cli.wait_for_service(timeout_sec=10)
        req = GetPositionFK.Request()
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(x) for x in q]
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        p = r.pose_stamped[0].pose
        return (np.array([p.position.x, p.position.y, p.position.z]),
                np.array([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]),
                r.pose_stamped[0].header.frame_id)

    def ik(self, pos, quat, seed=None, timeout=5.0):
        self.ik_cli.wait_for_service(timeout_sec=10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(x) for x in (seed or self.arm_q())]
        req.ik_request.timeout = Duration(sec=int(timeout), nanosec=int((timeout % 1) * 1e9))
        req.ik_request.avoid_collisions = False
        fut = self.ik_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=120)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            return None, (r.error_code.val if r else "timeout")
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM], 1

    # ---- motion
    def traj(self, q, sec=4.0, wait=True):
        self.fjt.wait_for_server(timeout_sec=10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(x) for x in q])
        pt.time_from_start = Duration(sec=int(sec), nanosec=int((sec % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res)
        code = res.result().result.error_code
        err = np.abs(np.array(self.arm_q()) - np.array(q)).max()
        return code, err

    def move(self, pos, quat, sec=4.0, tcp=False):
        pos = np.array(pos, float)
        if tcp:
            pos = pos - TCP * quat_R(*quat)[:, 2]
        q, code = self.ik(pos, quat)
        if q is None:
            return None, f"IK failed {code}"
        code, err = self.traj(q, sec)
        p, _, _ = self.fk()
        return code, f"traj code={code} joint_err={err:.4f} hand_at={p.round(4)}"

    def gripper(self, width):
        self.grip.wait_for_server(timeout_sec=10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = float(GRIP["max_effort"])
        send = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=180)
        r = res.result().result
        return r.reached_goal, r.stalled, self.finger_gap()

    def servo(self, v, n=20):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        for _ in range(n):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(0.05)


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    r = Robot()
    cmd = a[0]
    if cmd == "js":
        j = r.js()
        print({k: round(v, 4) for k, v in j.items()})
    elif cmd == "fk":
        p, q, f = r.fk()
        print("frame", f, "pos", p.round(4), "quat", q.round(4))
    elif cmd == "ik":
        v = list(map(float, a[1:8]))
        print(r.ik(v[:3], v[3:]))
    elif cmd == "move":
        flags = [x for x in a if x.startswith("--")]
        v = [x for x in a[1:] if not x.startswith("--")]
        pos, quat = list(map(float, v[:3])), list(map(float, v[3:7]))
        sec = float(v[7]) if len(v) > 7 else 4.0
        print(r.move(pos, quat, sec, tcp="--tcp" in flags))
    elif cmd == "joints":
        q = list(map(float, a[1].split(",")))
        sec = float(a[2]) if len(a) > 2 else 4.0
        print(r.traj(q, sec))
    elif cmd == "grip":
        w = GRIP["open_m"] if a[1] == "open" else GRIP["closed_m"]
        print(r.gripper(w))
    elif cmd == "servo":
        v = list(map(float, a[1:4]))
        n = int(a[4]) if len(a) > 4 else 20
        r.servo(v, n)
        p, _, _ = r.fk()
        print("hand_at", p.round(4))
    r.node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 16
timeout 120 python3 ctl.py fk; echo; echo "IK at FK world pose:"; timeout 120 python3 ctl.py ik -0.053 0 0.7776 1 0 0 0; echo "IK at base-frame equivalent:"; timeout 120 python3 ctl.py ik 0.457 0 0.3576 1 0 0 0

# openrua op 17
timeout 300 python3 ctl.py grip open; timeout 300 python3 ctl.py move -0.106 0.06 0.64 1 0 0 0 4 --tcp

# openrua op 18
timeout 120 python3 cloud.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']
m=(xyz[...,2]>0.505)&(xyz[...,2]<0.535)&(xyz[...,0]>-0.2)&(xyz[...,0]<0)&(xyz[...,1]>-0.05)&(xyz[...,1]<0.15)
p=xyz[m]; print('can top n',m.sum(),'center',p[:,:2].mean(0).round(4),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',np.median(p[:,2]).round(3))
"

# openrua op 19
timeout 300 python3 ctl.py move -0.109 0.058 0.54 1 0 0 0 3 --tcp && timeout 300 python3 ctl.py move -0.109 0.058 0.47 1 0 0 0 2 --tcp && timeout 300 python3 ctl.py grip close

# openrua op 20
timeout 300 python3 ctl.py move -0.109 0.058 0.72 1 0 0 0 3 --tcp && timeout 300 python3 ctl.py js && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 21
timeout 300 python3 ctl.py move 0.004 0.259 0.75 1 0 0 0 4 --tcp; timeout 300 python3 ctl.py move 0.004 0.259 0.75 1 0 0 0 2 --tcp; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 22
timeout 300 python3 ctl.py move 0.004 0.259 0.70 1 0 0 0 2 --tcp; timeout 300 python3 ctl.py grip open; timeout 300 python3 ctl.py move 0.004 0.259 0.80 1 0 0 0 3 --tcp; timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 23
timeout 300 python3 ctl.py move 0.074 0.034 0.60 1 0 0 0 4 --tcp && timeout 120 python3 cloud.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && python3 -c "
import numpy as np
d=np.load('robot0_eye_in_hand_cloud.npz'); xyz=d['xyz']
m=(xyz[...,2]>0.437)&(xyz[...,2]<0.47)&(xyz[...,0]>-0.0)&(xyz[...,0]<0.15)&(xyz[...,1]>-0.04)&(xyz[...,1]<0.1)
p=xyz[m]; print('butter top n',m.sum(),'center',p[:,:2].mean(0).round(4),'x',p[:,0].min().round(3),p[:,0].max().round(3),'y',p[:,1].min().round(3),p[:,1].max().round(3),'z',np.median(p[:,2]).round(3))
# principal axis
c=p[:,:2]-p[:,:2].mean(0); w,v=np.linalg.eigh(c.T@c); print('long axis', v[:,1].round(3), 'angle deg', np.degrees(np.arctan2(v[1,1],v[0,1])).round(1))
"

# openrua op 24
timeout 120 python3 ctl.py fk; timeout 120 python3 ctl.py js

# openrua op 25
timeout 300 python3 ctl.py move 0.0698 0.0365 0.60 0.9238795 -0.3826834 0 0 3 --tcp && timeout 120 python3 ctl.py fk && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 26
timeout 300 python3 ctl.py move 0.0698 0.0365 0.60 0.9238795 -0.3826834 0 0 3 --tcp && timeout 120 python3 ctl.py fk && timeout 120 python3 ctl.py js && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 27
timeout 300 python3 ctl.py move 0.0698 0.0365 0.50 0.9238795 -0.3826834 0 0 3 --tcp && timeout 300 python3 ctl.py move 0.0698 0.0365 0.44 0.9238795 -0.3826834 0 0 2 --tcp && timeout 120 python3 ctl.py fk && timeout 300 python3 ctl.py grip close

# openrua op 28
timeout 300 python3 ctl.py move 0.0698 0.0365 0.60 0.9238795 -0.3826834 0 0 3 --tcp && timeout 120 python3 ctl.py js && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 29
timeout 300 python3 ctl.py move 0.0698 0.0365 0.76 0.9238795 -0.3826834 0 0 3 --tcp && timeout 300 python3 ctl.py move 0.0 0.259 0.76 0.9238795 -0.3826834 0 0 4 --tcp && timeout 120 python3 ctl.py fk && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand

# openrua op 30
timeout 300 python3 ctl.py move 0.0 0.259 0.70 0.9238795 -0.3826834 0 0 2 --tcp && timeout 300 python3 ctl.py grip open && timeout 300 python3 ctl.py move 0.0 0.259 0.85 0.9238795 -0.3826834 0 0 3 --tcp && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand && timeout 60 python3 tools/perception/cam_snap.py agentview

# openrua op 31
timeout 120 python3 cloud.py agentview >/dev/null && python3 -c "
import numpy as np
d=np.load('agentview_cloud.npz'); xyz=d['xyz']
def cnt(xr,yr,zr):
    m=(xyz[...,0]>xr[0])&(xyz[...,0]<xr[1])&(xyz[...,1]>yr[0])&(xyz[...,1]<yr[1])&(xyz[...,2]>zr[0])&(xyz[...,2]<zr[1]); return int(m.sum())
print('points above table at old can spot   :', cnt((-0.16,-0.05),(0.01,0.11),(0.44,0.7)))
print('points above table at old butter spot:', cnt((0.02,0.12),(0.0,0.07),(0.44,0.7)))
m=(xyz[...,0]>-0.07)&(xyz[...,0]<0.06)&(xyz[...,1]>0.19)&(xyz[...,1]<0.33)&(xyz[...,2]>0.44)&(xyz[...,2]<0.60)
p=xyz[m]; print('points inside basket interior:', int(m.sum()), 'z range', p[:,2].min().round(3), p[:,2].max().round(3))
"; timeout 120 python3 ctl.py js
