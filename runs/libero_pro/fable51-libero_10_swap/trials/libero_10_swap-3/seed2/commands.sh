#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cd /workspace/tools/action && cat fjt_send.py; echo ----; cat gripper_cmd.py; echo ----; cat ik_move.py

# openrua op 3
cd /workspace && timeout 30 ros2 topic list; echo ----; timeout 20 ros2 action list; echo ----; timeout 20 ros2 service list | grep -v parameter | head -40; echo ----; timeout 20 ros2 node list

# openrua op 4
mkdir -p snaps && for c in agentview frontview sideview birdview robot0_eye_in_hand robot0_robotview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 60 python3 tools/perception/px2world.py agentview 337 210; timeout 60 python3 tools/perception/px2world.py birdview 330 258; timeout 60 python3 tools/perception/px2world.py birdview 270 265; timeout 30 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -12

# openrua op 6
timeout 30 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | grep -A2 Translation | head -3; timeout 20 ros2 topic echo /birdview/color/camera_info --once | grep -A10 "^k:"; timeout 20 ros2 topic echo /agentview/color/camera_info --once | grep -A10 "^k:"

# openrua op 7
mkdir -p "$(dirname /workspace/scene.py)"
cat > /workspace/scene.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab depth+color+TF from a camera and save a world-frame point cloud.

Usage: python3 scene.py <camera>
Writes snaps/<camera>_xyz.npy (H x W x 3 world coords, NaN where invalid)
and snaps/<camera>.png (color). Then prints a few stats.
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, msg_type, timeout=20.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
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
    cam = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node("scene")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    depth = br.imgmsg_to_cv2(grab(node, f"/{cam}/depth/image_raw", Image), "passthrough").astype(np.float64)
    color = br.imgmsg_to_cv2(grab(node, f"/{cam}/color/image_raw", Image), "bgr8")
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    frame = f"{cam}_optical_frame"
    import time
    end = time.time() + 10
    while time.time() < end and not tfbuf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_to_R(q.x, q.y, q.z, q.w)
    T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    H, W = depth.shape
    v, u = np.mgrid[0:H, 0:W]
    z = depth
    pc = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], -1)
    xyz = pc @ R.T + T
    bad = ~np.isfinite(z) | (z <= 0)
    xyz[bad] = np.nan
    np.save(f"snaps/{cam}_xyz.npy", xyz)
    cv2.imwrite(f"snaps/{cam}.png", color)
    print("cam pos", T, "valid", (~bad).sum())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 90 python3 scene.py birdview && timeout 90 python3 scene.py agentview && python3 -c "
import numpy as np
xyz=np.load('snaps/birdview_xyz.npy')
z=xyz[...,2]
# table height: mode of z
zz=z[np.isfinite(z)]
h,e=np.histogram(zz,bins=200)
for i in np.argsort(h)[-6:]: print('z bin',e[i],h[i])
# bowl region around pixel (330,258)
for (u,v) in [(330,258),(320,250),(340,265),(308,312),(220,265),(275,265),(300,265)]:
    print(u,v,xyz[v,u])
"

# openrua op 9
python3 -c "
import numpy as np, cv2
xyz=np.load('snaps/birdview_xyz.npy')
z=xyz[...,2]
# bowl: points 0.92<z<1.05 near (-0.14,0.03)
m=(z>0.92)&(z<1.08)&(np.abs(xyz[...,0]+0.14)<0.12)&(np.abs(xyz[...,1]-0.03)<0.12)
p=xyz[m]; print('bowl-ish pts',m.sum(),'x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max(),'z',p[:,2].min(),p[:,2].max())
print('bowl centroid', p.mean(0))
# cabinet region y<-0.1
m2=(z>0.905)&(xyz[...,1]<-0.1)&(xyz[...,1]>-0.5)&(xyz[...,0]>-0.5)&(xyz[...,0]<0.3)
p=xyz[m2]; print('cabinet pts',m2.sum(),'x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max(),'z',p[:,2].min(),p[:,2].max())
# height histogram in cabinet region
h,e=np.histogram(p[:,2],bins=40); 
for a,b in zip(e,h): print(f'{a:.3f} {b}')
" 

# openrua op 10
python3 -c "
import numpy as np, cv2
xyz=np.load('snaps/birdview_xyz.npy')
z=xyz[...,2]
near=(np.abs(xyz[...,0]+0.12)<0.15)&(np.abs(xyz[...,1])<0.15)
for lo,hi in [(0.90,0.93),(0.93,0.96),(0.96,0.99),(0.99,1.01),(1.01,1.03),(1.03,1.1),(1.1,1.3)]:
    m=near&(z>lo)&(z<hi); p=xyz[m]
    if len(p): print(f'{lo}-{hi}: n={len(p)} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] c=({p[:,0].mean():.3f},{p[:,1].mean():.3f})')
print('--- drawer floor z~0.922')
m=(z>0.915)&(z<0.93)&(xyz[...,1]<-0.1)
p=xyz[m]; print(len(p),'x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max())
print('--- cabinet top z~1.12')
m=(z>1.115)&(z<1.13)&(xyz[...,1]<-0.1)
p=xyz[m]; print(len(p),'x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max())
print('--- table')
m=(z>0.89)&(z<0.90)
p=xyz[m]; print(len(p),'x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max(), 'z',p[:,2].mean())
"

# openrua op 11
python3 -c "
import numpy as np
xyz=np.load('snaps/agentview_xyz.npy')
z=xyz[...,2]
near=(np.abs(xyz[...,0]+0.12)<0.12)&(np.abs(xyz[...,1]-0.03)<0.10)
for lo,hi in [(0.90,0.93),(0.93,0.96),(0.96,0.99),(0.99,1.01),(1.01,1.03),(1.03,1.1)]:
    m=near&(z>lo)&(z<hi); p=xyz[m]
    if len(p): print(f'{lo}-{hi}: n={len(p)} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] c=({p[:,0].mean():.3f},{p[:,1].mean():.3f})')
print('--- drawer floor (agentview)')
m=(z>0.905)&(z<0.94)&(xyz[...,1]<-0.08)&(xyz[...,1]>-0.25)&(xyz[...,0]>-0.35)&(xyz[...,0]<0.1)
p=xyz[m]; print(len(p),'x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max(),'z',p[:,2].mean())
xyz=np.load('snaps/birdview_xyz.npy'); z=xyz[...,2]
m=(z>0.905)&(z<0.94)&(xyz[...,1]<-0.08)&(xyz[...,1]>-0.25)&(xyz[...,0]>-0.35)&(xyz[...,0]<0.1)
p=xyz[m]; print('bird',len(p),'x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max(),'z',p[:,2].mean())
# drawer walls: z 0.94-1.0 in that y range
m=(z>0.94)&(z<1.0)&(xyz[...,1]<-0.08)&(xyz[...,1]>-0.25)&(xyz[...,0]>-0.35)&(xyz[...,0]<0.1)
p=xyz[m]; print('bird walls',len(p),'x',p[:,0].min(),p[:,0].max(),'y',p[:,1].min(),p[:,1].max(),'z',p[:,2].min(),p[:,2].max())
"

# openrua op 12
timeout 90 python3 scene.py robot0_eye_in_hand && python3 -c "
import numpy as np
for cam in ['birdview','robot0_eye_in_hand']:
    xyz=np.load(f'snaps/{cam}_xyz.npy'); z=xyz[...,2]
    m=(z>0.99)&(z<1.04)&(xyz[...,1]>-0.04)&(xyz[...,1]<0.15)&(xyz[...,0]>-0.25)&(xyz[...,0]<-0.02)
    p=xyz[m][:,:2]
    # algebraic circle fit
    A=np.c_[2*p[:,0],2*p[:,1],np.ones(len(p))]; b=(p**2).sum(1)
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; r=np.sqrt(c+cx*cx+cy*cy)
    print(cam,'rim pts',len(p),'center',cx,cy,'radius',r, 'zmax', xyz[m][:,2].max())
    # bowl interior: lowest z near center
    mm=(np.hypot(xyz[...,0]-cx,xyz[...,1]-cy)<0.03)
    print('  interior z min', np.nanmin(z[mm]), 'x-range of all bowl pts', )
"

# openrua op 13
python3 -c "
import numpy as np
xyz=np.load('snaps/robot0_eye_in_hand_xyz.npy'); z=xyz[...,2]
cx,cy=-0.151,0.032
r=np.hypot(xyz[...,0]-cx,xyz[...,1]-cy)
for lo in np.arange(0.90,1.04,0.01):
    m=(z>=lo)&(z<lo+0.01)&(r<0.12)
    if m.sum(): print(f'z {lo:.2f}: n={m.sum()} r[{r[m].min():.3f},{r[m].max():.3f}] rmean={r[m].mean():.3f}')
"

# openrua op 14
python3 -c "
import numpy as np
xyz=np.load('snaps/agentview_xyz.npy'); z=xyz[...,2]
cx,cy=-0.151,0.032
r=np.hypot(xyz[...,0]-cx,xyz[...,1]-cy)
m0=(np.abs(xyz[...,1]-cy)<0.08)&(xyz[...,0]<-0.05)&(xyz[...,0]>-0.25)
for lo in np.arange(0.89,1.04,0.01):
    m=(z>=lo)&(z<lo+0.01)&m0
    if m.sum(): print(f'z {lo:.2f}: n={m.sum()} r[{r[m].min():.3f},{r[m].max():.3f}] x[{xyz[m][:,0].min():.3f},{xyz[m][:,0].max():.3f}]')
"

# openrua op 15
timeout 300 python3 tools/action/gripper_cmd.py 0.04; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position

# openrua op 16
timeout 90 python3 scene.py robot0_eye_in_hand && timeout 90 python3 scene.py birdview && python3 -c "
import numpy as np
for cam in ['robot0_eye_in_hand','birdview']:
    xyz=np.load(f'snaps/{cam}_xyz.npy'); z=xyz[...,2]
    m=(z>0.90)&(z<1.05)&(xyz[...,1]>-0.05)&(xyz[...,1]<0.15)&(xyz[...,0]>-0.25)&(xyz[...,0]<-0.05)
    p=xyz[m]
    print(cam, 'bowl pts',len(p),'z range',p[:,2].min(),p[:,2].max())
    top=p[p[:,2]>p[:,2].max()-0.015][:,:2]
    A=np.c_[2*top[:,0],2*top[:,1],np.ones(len(top))]; b=(top**2).sum(1)
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; print('  rim center',cx,cy,'r',np.sqrt(c+cx*cx+cy*cy))
    mm=(np.hypot(xyz[...,0]-cx,xyz[...,1]-cy)<0.02); print('  interior floor z',np.nanmin(z[mm]))
"

# openrua op 17
timeout 90 python3 scene.py agentview && python3 -c "
import numpy as np
xyz=np.load('snaps/birdview_xyz.npy'); z=xyz[...,2]
reg=(xyz[...,1]<-0.05)&(xyz[...,1]>-0.45)&(xyz[...,0]>-0.35)&(xyz[...,0]<0.1)
print('birdview cabinet region height histogram')
for lo in np.arange(0.90,1.16,0.01):
    m=reg&(z>=lo)&(z<lo+0.01)
    if m.sum()>20:
        p=xyz[m]; print(f'z {lo:.2f}: n={m.sum()} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}]')
"

# openrua op 18
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Control helpers for the Panda: joint state, FK, IK, trajectories, gripper.

World frame <-> base frame is a pure translation (measured via TF):
    base = world - WORLD_T_BASE
Poses passed to move_tcp() are the TCP (fingertip point) in WORLD frame.
"""
import sys
import time
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

ROOT = Path(__file__).resolve().parent
M = yaml.safe_load((ROOT / "machine.yaml").read_text())
TRAJ = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TWIST = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = TRAJ["joints"]
TCP_OFF = float(M["hand"]["tcp_offset_m"])
WORLD_T_BASE = np.array([-0.660, 0.0, 0.912])

# hand pointing straight down, fingers along world X (180deg about (1,1,0)/sqrt2)
Q_DOWN_FINGERS_X = (0.70710678, 0.70710678, 0.0, 0.0)
# hand pointing straight down, fingers along world Y (180deg about X)
Q_DOWN_FINGERS_Y = (1.0, 0.0, 0.0, 0.0)


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class Robot:
    def __init__(self, name="ctl"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = {}
        self.node.create_subscription(JointState, "/joint_states",
                                      self._on_js, 1)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, TRAJ["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist_pub = self.node.create_publisher(TwistStamped, TWIST["port"], 10)
        assert self.fjt.wait_for_server(timeout_sec=20), "no FJT server"
        assert self.grip.wait_for_server(timeout_sec=20), "no gripper server"
        assert self.ik.wait_for_service(timeout_sec=20), "no IK"
        assert self.fk.wait_for_service(timeout_sec=20), "no FK"

    def _on_js(self, msg):
        self._js = dict(zip(msg.name, msg.position))

    # ---------- sensing ----------
    def joints(self, fresh=True):
        if fresh:
            self._js = {}
        end = time.time() + 20
        while not self._js and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        return dict(self._js)

    def arm_q(self):
        js = self.joints()
        return [js[j] for j in ARM]

    def finger_gap(self):
        js = self.joints()
        return js["panda_finger_joint1"] - js["panda_finger_joint2"]

    def fk_hand(self, q=None):
        """Hand frame pose in WORLD: (pos(3), quat xyzw(4))."""
        q = self.arm_q() if q is None else q
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        res = fut.result()
        assert res is not None and res.error_code.val == 1, f"FK failed {res}"
        p = res.pose_stamped[0].pose
        pos = np.array([p.position.x, p.position.y, p.position.z]) + WORLD_T_BASE
        quat = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        return pos, quat

    def fk_tcp(self, q=None):
        pos, quat = self.fk_hand(q)
        R = quat_to_R(*quat)
        return pos + TCP_OFF * R[:, 2], quat

    # ---------- planning ----------
    def ik_tcp(self, pos_world, quat, seed=None, tries=1):
        """IK for a TCP pose in world. Returns arm joint list or None."""
        R = quat_to_R(*quat)
        hand_world = np.asarray(pos_world, float) - TCP_OFF * R[:, 2]
        hand_base = hand_world - WORLD_T_BASE
        seed = self.arm_q() if seed is None else seed
        for _ in range(tries):
            req = GetPositionIK.Request()
            req.ik_request.group_name = M["planning"]["group"]
            req.ik_request.pose_stamped.header.frame_id = ""
            p = req.ik_request.pose_stamped.pose
            p.position.x, p.position.y, p.position.z = map(float, hand_base)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
            req.ik_request.robot_state.joint_state.name = list(ARM)
            req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
            req.ik_request.avoid_collisions = False
            req.ik_request.timeout.sec = 2
            fut = self.ik.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
            res = fut.result()
            if res is not None and res.error_code.val == 1:
                sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
                return [sol[j] for j in ARM]
            print(f"IK failed code={None if res is None else res.error_code.val}", file=sys.stderr)
        return None

    # ---------- acting ----------
    def move_q(self, q, seconds=3.0, via=None):
        """Send a joint trajectory (optionally through 'via' waypoints)."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pts = []
        wps = list(via or []) + [q]
        for i, wp in enumerate(wps):
            t = seconds * (i + 1) / len(wps)
            pt = JointTrajectoryPoint(positions=[float(v) for v in wp])
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
            pts.append(pt)
        goal.trajectory.points = pts
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        assert gh is not None and gh.accepted, "goal rejected"
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=600)
        code = rf.result().result.error_code
        err = float(np.max(np.abs(np.array(self.arm_q()) - np.array(q))))
        print(f"move_q: error_code={code} max_joint_err={err:.4f}")
        return code, err

    def move_tcp(self, pos_world, quat, seconds=3.0, seed=None):
        q = self.ik_tcp(pos_world, quat, seed=seed)
        if q is None:
            print("move_tcp: IK failed, no motion", file=sys.stderr)
            return None
        self.move_q(q, seconds)
        pos, _ = self.fk_tcp()
        print(f"move_tcp: tcp now {np.round(pos, 4)} (target {np.round(pos_world, 4)})")
        return q

    def gripper(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        rf = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, rf, timeout_sec=300)
        res = rf.result().result
        gap = self.finger_gap()
        print(f"gripper({width}): reached={res.reached_goal} stalled={res.stalled} gap={gap:.4f}")
        return gap

    def servo(self, vx=0.0, vy=0.0, vz=0.0, ticks=20):
        msg = TwistStamped()
        msg.header.frame_id = TWIST["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)


if __name__ == "__main__":
    r = Robot()
    print("joints", r.joints())
    pos, quat = r.fk_hand()
    print("hand world", pos, quat)
    print("tcp world", r.fk_tcp()[0])
    print("finger gap", r.finger_gap())
OPENRUA_EOF

# openrua op 19
timeout 120 python3 ctl.py 2>&1 | tail -5

# openrua op 20
timeout 200 python3 -c "
import numpy as np, ctl
ctl.WORLD_T_BASE=np.zeros(3)   # treat FK output as-is
r=ctl.Robot()
q0=r.arm_q()
pos,quat=r.fk_hand(); print('fk hand raw',pos,quat)
# IK at the same pose (raw frame) -> should give ~q0
sol=r.ik_tcp(pos+ctl.TCP_OFF*ctl.quat_to_R(*quat)[:,2],quat)
print('IK raw-frame sol', None if sol is None else np.round(sol,3))
print('q0', np.round(q0,3))
# IK at pose shifted to base-frame interpretation
sol2=r.ik_tcp(pos+ctl.TCP_OFF*ctl.quat_to_R(*quat)[:,2]-np.array([-0.66,0,0.912]),quat)
print('IK base-shifted sol', None if sol2 is None else np.round(sol2,3))
"

# openrua op 21
timeout 200 python3 -c "
import numpy as np, ctl
ctl.WORLD_T_BASE=np.zeros(3)
r=ctl.Robot()
sol=[0.058,-0.161,-0.058,-2.445,-0.012,2.227,0.009]
print('fk(sol)', r.fk_hand(sol))
print('fk(q0)', r.fk_hand())
"

# openrua op 22
timeout 300 python3 -c "
import numpy as np, ctl
ctl.WORLD_T_BASE=np.zeros(3)
r=ctl.Robot()
def angerr(qa,qb):
    d=abs(np.dot(qa,qb)); return np.degrees(2*np.arccos(min(1,d)))
targets=[((-0.203,0.017,1.05),ctl.Q_DOWN_FINGERS_X),((-0.203,0.017,1.05),ctl.Q_DOWN_FINGERS_Y),((-0.11,-0.14,1.05),ctl.Q_DOWN_FINGERS_X),((-0.203,0.017,0.93),ctl.Q_DOWN_FINGERS_X)]
for pos,quat in targets:
    for i in range(3):
        sol=r.ik_tcp(pos,quat)
        if sol is None: print(pos,quat,'IK none'); continue
        p,q=r.fk_tcp(sol)
        print(np.round(pos,3),'-> pos err %.4f'%np.linalg.norm(p-pos),'ang err %.1f'%angerr(q,quat),'q7=%.2f'%sol[6], np.round(sol,2))
"

# openrua op 23
python3 - <<'EOF'
import re
s=open('ctl.py').read()
s=s.replace('WORLD_T_BASE = np.array([-0.660, 0.0, 0.912])',
 '# measured: MoveIt FK/IK poses (frame_id "") already match TF world frame\nWORLD_T_BASE = np.array([0.0, 0.0, 0.0])')
s=s.replace('''def quat_to_R(x, y, z, w):''','''def quat_mul(a, b):
    """Hamilton product of xyzw quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


# measured: the IK service solves for panda_link8, whose frame is the hand
# frame rotated -45deg about z. Request hand*Rz(+45deg) to get the hand there.
Q_IK_FIX = (0.0, 0.0, np.sin(np.pi / 8), np.cos(np.pi / 8))


def quat_to_R(x, y, z, w):''')
s=s.replace('''            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)''',
'''            qreq = quat_mul(quat, Q_IK_FIX)
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, qreq)''')
open('ctl.py','w').write(s)
EOF
timeout 300 python3 -c "
import numpy as np, ctl
r=ctl.Robot()
def angerr(qa,qb):
    d=abs(np.dot(qa,qb)); return np.degrees(2*np.arccos(min(1,d)))
targets=[((-0.203,0.017,1.05),ctl.Q_DOWN_FINGERS_X),((-0.203,0.017,1.05),ctl.Q_DOWN_FINGERS_Y),((-0.11,-0.14,1.05),ctl.Q_DOWN_FINGERS_X),((-0.203,0.017,0.93),ctl.Q_DOWN_FINGERS_X),((-0.168,-0.14,0.99),ctl.Q_DOWN_FINGERS_X)]
for pos,quat in targets:
    sol=r.ik_tcp(pos,quat)
    if sol is None: print(pos,quat,'IK none'); continue
    p,q=r.fk_tcp(sol)
    print(np.round(pos,3),'-> pos err %.4f'%np.linalg.norm(p-pos),'ang err %.1f'%angerr(q,quat), np.round(sol,2))
"

# openrua op 24
python3 -c "
import numpy as np
xyz=np.load('snaps/birdview_xyz.npy'); z=xyz[...,2]
reg=(xyz[...,1]<-0.04)&(xyz[...,1]>-0.24)&(xyz[...,0]>-0.3)&(xyz[...,0]<0.02)
m=reg&(z>0.915)&(z<0.935); p=xyz[m]
print('floor n',len(p),'x[%.3f,%.3f] y[%.3f,%.3f] z=%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].mean()))
# floor x-extent per y strip
for y0 in np.arange(-0.22,-0.05,0.02):
    mm=m&(xyz[...,1]>=y0)&(xyz[...,1]<y0+0.02)
    if mm.sum(): print(' y %.2f: x[%.3f,%.3f] n=%d'%(y0,xyz[mm][:,0].min(),xyz[mm][:,0].max(),mm.sum()))
# walls (z 0.95-1.0)
m=reg&(z>0.95)&(z<1.0); p=xyz[m]
print('walls n',len(p))
for y0 in np.arange(-0.24,-0.04,0.02):
    mm=m&(xyz[...,1]>=y0)&(xyz[...,1]<y0+0.02)
    if mm.sum(): print(' y %.2f: x[%.3f,%.3f] n=%d zmax=%.3f'%(y0,xyz[mm][:,0].min(),xyz[mm][:,0].max(),mm.sum(),xyz[mm][:,2].max()))
"

# openrua op 25
timeout 600 python3 -u -c "
import numpy as np, ctl
r=ctl.Robot()
q=r.move_tcp((-0.203,0.017,1.03), ctl.Q_DOWN_FINGERS_X, seconds=4)
print('gap',r.finger_gap())
" 2>&1 | tail -4 && timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih_pregrasp.png

# openrua op 26
timeout 90 python3 scene.py robot0_eye_in_hand >/dev/null && timeout 120 python3 -c "
import numpy as np, ctl
xyz=np.load('snaps/robot0_eye_in_hand_xyz.npy'); z=xyz[...,2]
m=(z>0.94)&(z<0.96)&(xyz[...,1]>-0.06)&(xyz[...,1]<0.12)&(xyz[...,0]>-0.26)&(xyz[...,0]<-0.04)
p=xyz[m][:,:2]
A=np.c_[2*p[:,0],2*p[:,1],np.ones(len(p))]; b=(p**2).sum(1)
cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; print('rim center',cx,cy,'r',np.sqrt(c+cx*cx+cy*cy),'n',len(p),'zmax',xyz[m][:,2].max())
r=ctl.Robot(); print('joints',np.round(r.arm_q(),3)); print('tcp',r.fk_tcp()[0])
"

# openrua op 27
python3 -c "
import numpy as np
xyz=np.load('snaps/robot0_eye_in_hand_xyz.npy'); z=xyz[...,2]
reg=(xyz[...,1]>-0.06)&(xyz[...,1]<0.12)&(xyz[...,0]>-0.26)&(xyz[...,0]<-0.04)
for lo,hi in [(0.950,0.956),(0.945,0.95),(0.935,0.945),(0.92,0.935),(0.905,0.92)]:
    m=reg&(z>lo)&(z<hi); p=xyz[m][:,:2]
    A=np.c_[2*p[:,0],2*p[:,1],np.ones(len(p))]; b=(p**2).sum(1)
    cx,cy,c=np.linalg.lstsq(A,b,rcond=None)[0]; print(f'z[{lo},{hi}] center ({cx:.4f},{cy:.4f}) r={np.sqrt(c+cx*cx+cy*cy):.4f} n={len(p)}')
# also check: interior floor center by lowest z points
m=reg&(z<0.912); p=xyz[m]; print('floor pts', len(p), 'mean', p[:,:2].mean(0), 'zmin', p[:,2].min())
"

# openrua op 28
python3 -c "
import numpy as np, cv2
img=cv2.imread('snaps/robot0_eye_in_hand.png'); xyz=np.load('snaps/robot0_eye_in_hand_xyz.npy')
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
# yellow rim: hue ~ 20-35 (opencv), high sat
m=(hsv[...,0]>18)&(hsv[...,0]<40)&(hsv[...,1]>80)&(hsv[...,2]>120)
ys,xs=np.nonzero(m); print('yellow px',len(xs))
pts=np.c_[xs,ys].astype(np.float32)
(ex,ey),(a,b),ang=cv2.fitEllipse(pts); print('ellipse center px',ex,ey,'axes',a,b,'ang',ang)
# world coords of rim pixels
P=xyz[ys,xs]; ok=np.isfinite(P[:,2]); P=P[ok]
print('rim px world z range',P[:,2].min(),P[:,2].max(), 'n',len(P))
q=P[:,:2]; A=np.c_[2*q[:,0],2*q[:,1],np.ones(len(q))]; bb=(q**2).sum(1)
cx,cy,c=np.linalg.lstsq(A,bb,rcond=None)[0]; print('rim circle from yellow px: center (%.4f,%.4f) r=%.4f'%(cx,cy,np.sqrt(c+cx*cx+cy*cy)))
"

# openrua op 29
python3 -c "
import numpy as np, cv2
img=cv2.imread('snaps/robot0_eye_in_hand.png'); xyz=np.load('snaps/robot0_eye_in_hand_xyz.npy')
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
m=(hsv[...,0]>18)&(hsv[...,0]<40)&(hsv[...,1]>80)&(hsv[...,2]>120)
ys,xs=np.nonzero(m); P=xyz[ys,xs]
for lo in [0.94,0.945,0.948]:
    ok=P[:,2]>lo; q=P[ok][:,:2]
    A=np.c_[2*q[:,0],2*q[:,1],np.ones(len(q))]; bb=(q**2).sum(1)
    cx,cy,c=np.linalg.lstsq(A,bb,rcond=None)[0]; print(f'z>{lo}: n={ok.sum()} center ({cx:.4f},{cy:.4f}) r={np.sqrt(c+cx*cx+cy*cy):.4f}')
    # angular coverage
    ang=np.degrees(np.arctan2(q[:,1]-cy,q[:,0]-cx)); print('   angle coverage bins:', np.histogram(ang,bins=8,range=(-180,180))[0])
"

# openrua op 30
python3 -c "
import numpy as np
xyz=np.load('snaps/robot0_eye_in_hand_xyz.npy'); z=xyz[...,2]
m=(z>0.94)&(z<0.97)&(xyz[...,1]<-0.04)&(xyz[...,1]>-0.12)&(xyz[...,0]>-0.2)&(xyz[...,0]<0.0)
p=xyz[m]; print('handle-ish eih: n',len(p),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
xyz=np.load('snaps/birdview_xyz.npy'); z=xyz[...,2]
m=(z>0.94)&(z<0.97)&(xyz[...,1]<-0.04)&(xyz[...,1]>-0.12)&(xyz[...,0]>-0.2)&(xyz[...,0]<0.0)
p=xyz[m]; print('handle-ish bird: n',len(p),'x[%.3f,%.3f] y[%.3f,%.3f] z[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].min(),p[:,2].max()))
# drawer front wall top (z 0.975-0.99) y position
for name in ['robot0_eye_in_hand','birdview']:
    xyz=np.load(f'snaps/{name}_xyz.npy'); z=xyz[...,2]
    m=(z>0.975)&(z<0.99)&(xyz[...,1]<-0.05)&(xyz[...,1]>-0.15)&(xyz[...,0]>-0.2)&(xyz[...,0]<0.0)
    p=xyz[m]; print(name,'front wall top: n',len(p),'y[%.3f,%.3f] mean y %.4f'%(p[:,1].min(),p[:,1].max(),p[:,1].mean()))
"

# openrua op 31
timeout 900 python3 -u -c "
import numpy as np, ctl, time
r=ctl.Robot()
Q=ctl.Q_DOWN_FINGERS_X
r.move_tcp((-0.207,0.032,1.00), Q, seconds=2)
r.move_tcp((-0.207,0.032,0.93), Q, seconds=3)
print('tcp',r.fk_tcp()[0], 'joints', np.round(r.arm_q(),3))
gap=r.gripper(0.0)
print('gap after close',gap, 'fingers', {k:v for k,v in r.joints().items() if 'finger' in k})
" 2>&1 | grep -v '^$'

# openrua op 32
timeout 900 python3 -u -c "
import numpy as np, ctl
r=ctl.Robot()
Q=ctl.Q_DOWN_FINGERS_X
r.move_tcp((-0.207,0.032,1.06), Q, seconds=3)
print('gap',r.finger_gap())
" 2>&1 | grep -v '^$'; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/lifted.png; timeout 60 python3 tools/perception/cam_snap.py sideview snaps/lifted_side.png

# openrua op 33
timeout 900 python3 -u -c "
import numpy as np, ctl
r=ctl.Robot()
Q=ctl.Q_DOWN_FINGERS_X
r.move_tcp((-0.172,-0.159,1.08), Q, seconds=4)
print('gap',r.finger_gap())
" 2>&1 | grep -v '^$'; timeout 90 python3 scene.py birdview && python3 -c "
import numpy as np
xyz=np.load('snaps/birdview_xyz.npy'); z=xyz[...,2]
reg=(xyz[...,1]>-0.30)&(xyz[...,1]<0.0)&(xyz[...,0]>-0.30)&(xyz[...,0]<0.0)
m=reg&(z>1.06)&(z<1.115); p=xyz[m]
print('bowl-ish pts n',len(p),'z[%.3f,%.3f]'%(p[:,2].min(),p[:,2].max()))
for lo,hi in [(1.095,1.115),(1.09,1.10),(1.06,1.09)]:
    mm=reg&(z>lo)&(z<hi); q=xyz[mm][:,:2]
    if len(q)<10: continue
    A=np.c_[2*q[:,0],2*q[:,1],np.ones(len(q))]; bb=(q**2).sum(1)
    cx,cy,c=np.linalg.lstsq(A,bb,rcond=None)[0]; print(f'z[{lo},{hi}] n={len(q)} center ({cx:.4f},{cy:.4f}) r={np.sqrt(c+cx*cx+cy*cy):.4f}  x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]')
"

# openrua op 34
timeout 120 python3 -c "
import numpy as np, cv2, ctl
img=cv2.imread('snaps/birdview.png'); xyz=np.load('snaps/birdview_xyz.npy')
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
m=(hsv[...,0]>18)&(hsv[...,0]<40)&(hsv[...,1]>80)&(hsv[...,2]>120)
ys,xs=np.nonzero(m); P=xyz[ys,xs]; ok=np.isfinite(P[:,2])&(P[:,2]>1.05); q=P[ok][:,:2]
print('yellow rim px',ok.sum(), 'z',P[ok][:,2].min(),P[ok][:,2].max())
A=np.c_[2*q[:,0],2*q[:,1],np.ones(len(q))]; bb=(q**2).sum(1)
cx,cy,c=np.linalg.lstsq(A,bb,rcond=None)[0]; print('center (%.4f,%.4f) r=%.4f'%(cx,cy,np.sqrt(c+cx*cx+cy*cy)))
r=ctl.Robot(); t=r.fk_tcp()[0]; print('tcp',t, 'bowl-tcp offset', cx-t[0], cy-t[1])
cv2.imwrite('snaps/bird_crop.png', cv2.resize(img[200:330,180:360],None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))
"

# openrua op 35
timeout 90 python3 scene.py robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np, cv2
img=cv2.imread('snaps/robot0_eye_in_hand.png'); xyz=np.load('snaps/robot0_eye_in_hand_xyz.npy'); z=xyz[...,2]
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
m=(hsv[...,0]>18)&(hsv[...,0]<40)&(hsv[...,1]>80)&(hsv[...,2]>120)
ys,xs=np.nonzero(m); P=xyz[ys,xs]; ok=np.isfinite(P[:,2])&(P[:,2]>1.05); q=P[ok][:,:2]
print('yellow rim px',ok.sum(), 'z',P[ok][:,2].min(),P[ok][:,2].max())
A=np.c_[2*q[:,0],2*q[:,1],np.ones(len(q))]; bb=(q**2).sum(1)
cx,cy,c=np.linalg.lstsq(A,bb,rcond=None)[0]; print('bowl rim center (%.4f,%.4f) r=%.4f'%(cx,cy,np.sqrt(c+cx*cx+cy*cy)))
# drawer floor & walls in this view
reg=(xyz[...,1]>-0.30)&(xyz[...,1]<0.0)&(xyz[...,0]>-0.30)&(xyz[...,0]<0.05)
mm=reg&(z>0.915)&(z<0.935); p=xyz[mm]; print('floor n',len(p),'x[%.3f,%.3f] y[%.3f,%.3f] z=%.3f'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max(),p[:,2].mean()))
mm=reg&(z>0.975)&(z<0.99); p=xyz[mm]; print('wall tops n',len(p))
for x0 in np.arange(-0.24,0.0,0.02):
    k=mm&(xyz[...,0]>=x0)&(xyz[...,0]<x0+0.02)
    if k.sum()>5: print('  x %.2f: y[%.3f,%.3f] n=%d'%(x0,xyz[k][:,1].min(),xyz[k][:,1].max(),k.sum()))
for y0 in np.arange(-0.24,-0.06,0.02):
    k=mm&(xyz[...,1]>=y0)&(xyz[...,1]<y0+0.02)
    if k.sum()>5: print('  y %.2f: x[%.3f,%.3f] n=%d'%(y0,xyz[k][:,0].min(),xyz[k][:,0].max(),k.sum()))
"

# openrua op 36
python3 -c "
import numpy as np, cv2
from scipy.optimize import least_squares
img=cv2.imread('snaps/robot0_eye_in_hand.png'); xyz=np.load('snaps/robot0_eye_in_hand_xyz.npy')
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
m=(hsv[...,0]>18)&(hsv[...,0]<40)&(hsv[...,1]>80)&(hsv[...,2]>120)
ys,xs=np.nonzero(m); P=xyz[ys,xs]; ok=np.isfinite(P[:,2])&(P[:,2]>1.05); q=P[ok][:,:2]
print('px range u[%d,%d] v[%d,%d]'%(xs.min(),xs.max(),ys.min(),ys.max()))
R=0.0551
res=least_squares(lambda c: np.hypot(q[:,0]-c[0],q[:,1]-c[1])-R, x0=[q[:,0].mean(),q[:,1].mean()])
cx,cy=res.x; print('fixed-r center (%.4f,%.4f) rms=%.4f'%(cx,cy,np.sqrt(np.mean(res.fun**2))))
ang=np.degrees(np.arctan2(q[:,1]-cy,q[:,0]-cx)); print('coverage', np.histogram(ang,bins=12,range=(-180,180))[0])
cv2.imwrite('snaps/eih_overdrawer.png', img)
"

# openrua op 37
timeout 900 python3 -u -c "
import numpy as np, ctl
r=ctl.Robot()
Q=ctl.Q_DOWN_FINGERS_X
r.move_tcp((-0.163,-0.170,1.06), Q, seconds=2)
r.move_tcp((-0.163,-0.170,1.01), Q, seconds=2)
print('gap',r.finger_gap())
" 2>&1 | grep -v '^$'; timeout 90 python3 scene.py robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np, cv2
from scipy.optimize import least_squares
img=cv2.imread('snaps/robot0_eye_in_hand.png'); xyz=np.load('snaps/robot0_eye_in_hand_xyz.npy'); z=xyz[...,2]
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
m=(hsv[...,0]>18)&(hsv[...,0]<40)&(hsv[...,1]>80)&(hsv[...,2]>120)
ys,xs=np.nonzero(m); P=xyz[ys,xs]; ok=np.isfinite(P[:,2])&(P[:,2]>0.99); q=P[ok][:,:2]
res=least_squares(lambda c: np.hypot(q[:,0]-c[0],q[:,1]-c[1])-0.0551, x0=[q[:,0].mean(),q[:,1].mean()])
print('bowl center (%.4f,%.4f) rms=%.4f n=%d rim z[%.3f,%.3f]'%(res.x[0],res.x[1],np.sqrt(np.mean(res.fun**2)),len(q),P[ok][:,2].min(),P[ok][:,2].max()))
reg=(xyz[...,1]>-0.30)&(xyz[...,1]<0.0)&(xyz[...,0]>-0.30)&(xyz[...,0]<0.05)
mm=reg&(z>0.975)&(z<0.99); p=xyz[mm]
k=mm&(xyz[...,0]>-0.20)&(xyz[...,0]<-0.05); print('front wall top y[%.3f,%.3f]'%(xyz[k][:,1].min(),xyz[k][:,1].max()))
k=mm&(xyz[...,1]>-0.20)&(xyz[...,1]<-0.11); print('side wall top x[%.3f,%.3f]'%(xyz[k][:,0].min(),xyz[k][:,0].max()))
"

# openrua op 38
timeout 60 python3 tools/perception/cam_snap.py agentview snaps/contact_agent.png; timeout 60 python3 tools/perception/cam_snap.py frontview snaps/contact_front.png; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A7 force

# openrua op 39
python3 -c "
import numpy as np
xyz=np.load('snaps/agentview_xyz.npy'); z=xyz[...,2]
print('AGENTVIEW: cabinet front region y in [-0.24,-0.17], z>0.93, x in [-0.26,0.03]')
reg=(xyz[...,1]>-0.24)&(xyz[...,1]<-0.17)&(xyz[...,0]>-0.26)&(xyz[...,0]<0.03)&(z>0.93)
for lo in np.arange(0.93,1.14,0.01):
    m=reg&(z>=lo)&(z<lo+0.01)
    if m.sum()>3: p=xyz[m]; print(f' z {lo:.2f}: n={m.sum():4d} ymax={p[:,1].max():.3f} ymin={p[:,1].min():.3f} x[{p[:,0].min():.3f},{p[:,0].max():.3f}]')
print('points with y>-0.215 (protruding beyond cabinet face):')
m=reg&(xyz[...,1]>-0.215); p=xyz[m]
for lo in np.arange(0.93,1.14,0.01):
    k=m&(z>=lo)&(z<lo+0.01)
    if k.sum()>3: q=xyz[k]; print(f' z {lo:.2f}: n={k.sum():4d} y[{q[:,1].min():.3f},{q[:,1].max():.3f}] x[{q[:,0].min():.3f},{q[:,0].max():.3f}]')
"

# openrua op 40
python3 -c "
import numpy as np
for cam in ['birdview','agentview']:
    xyz=np.load(f'snaps/{cam}_xyz.npy'); z=xyz[...,2]
    m=(z>0.905)&(z<1.2)&(xyz[...,0]>-0.01)&(xyz[...,0]<0.12)&(xyz[...,1]>-0.12)&(xyz[...,1]<0.03)
    p=xyz[m]; print(cam,'bottle pts',len(p))
    for lo in np.arange(0.90,1.2,0.02):
        k=m&(z>=lo)&(z<lo+0.02)
        if k.sum()>3: q=xyz[k]; print(f'  z {lo:.2f}: n={k.sum()} x[{q[:,0].min():.3f},{q[:,0].max():.3f}] y[{q[:,1].min():.3f},{q[:,1].max():.3f}]')
"

# openrua op 41
timeout 900 python3 -u -c "
import numpy as np, ctl
r=ctl.Robot()
Q=ctl.Q_DOWN_FINGERS_X
r.move_tcp((-0.161,-0.167,1.09), Q, seconds=2)
r.move_tcp((-0.198,0.06,1.05), Q, seconds=3)
r.move_tcp((-0.198,0.06,0.925), Q, seconds=3)
r.gripper(0.04)
r.move_tcp((-0.198,0.06,1.05), Q, seconds=2)
" 2>&1 | grep -v '^$'; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/putdown.png

# openrua op 42
timeout 900 python3 -u -c "
import numpy as np, ctl
r=ctl.Robot()
Q=ctl.Q_DOWN_FINGERS_X
r.gripper(0.0)
r.move_tcp((-0.19,-0.108,1.05), Q, seconds=3)
r.move_tcp((-0.19,-0.108,0.945), Q, seconds=3)
r.move_tcp((-0.19,-0.07,0.945), Q, seconds=2)
r.move_tcp((-0.19,-0.033,0.945), Q, seconds=2)
r.move_tcp((-0.19,-0.033,1.05), Q, seconds=2)
" 2>&1 | grep -v '^$'; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/pulled.png

# openrua op 43
timeout 90 python3 scene.py birdview >/dev/null && python3 -c "
import numpy as np
xyz=np.load('snaps/birdview_xyz.npy'); z=xyz[...,2]
reg=(xyz[...,0]>-0.23)&(xyz[...,0]<0.01)&(xyz[...,1]>-0.25)&(xyz[...,1]<0.05)
m=reg&(z>0.975)&(z<0.99); p=xyz[m]
k=m&(xyz[...,0]>-0.2)&(xyz[...,0]<-0.16); print('front wall top (x -0.2..-0.16): y[%.3f,%.3f]'%(xyz[k][:,1].min(),xyz[k][:,1].max()))
k=m&(xyz[...,0]>-0.06)&(xyz[...,0]<-0.02); print('front wall top (x -0.06..-0.02): y[%.3f,%.3f]'%(xyz[k][:,1].min(),xyz[k][:,1].max()))
m=reg&(z>0.915)&(z<0.935); p=xyz[m]; print('floor y[%.3f,%.3f] x[%.3f,%.3f]'%(p[:,1].min(),p[:,1].max(),p[:,0].min(),p[:,0].max()))
m=reg&(z>0.94)&(z<0.97)&(xyz[...,1]>-0.08); p=xyz[m]; print('handle y[%.3f,%.3f] x[%.3f,%.3f]'%(p[:,1].min(),p[:,1].max(),p[:,0].min(),p[:,0].max()))
"

# openrua op 44
python3 -c "
import numpy as np
xyz=np.load('snaps/birdview_xyz.npy'); z=xyz[...,2]
reg=(xyz[...,0]>-0.23)&(xyz[...,0]<0.01)&(xyz[...,1]>-0.25)&(xyz[...,1]<0.05)
for lo in np.arange(0.90,1.0,0.01):
    m=reg&(z>=lo)&(z<lo+0.01)
    if m.sum()>3: p=xyz[m]; print(f'z {lo:.2f}: n={m.sum()} y[{p[:,1].min():.3f},{p[:,1].max():.3f}] x[{p[:,0].min():.3f},{p[:,0].max():.3f}]')
m=reg&(z>0.975)&(z<0.995)
for x0 in np.arange(-0.22,0.0,0.04):
    k=m&(xyz[...,0]>=x0)&(xyz[...,0]<x0+0.04)
    if k.sum()>2: print(' walltop x %.2f: y[%.3f,%.3f] n=%d'%(x0,xyz[k][:,1].min(),xyz[k][:,1].max(),k.sum()))
"

# openrua op 45
timeout 900 python3 -u -c "
import numpy as np, ctl
r=ctl.Robot()
Q=ctl.Q_DOWN_FINGERS_X
r.move_tcp((-0.19,-0.10,1.05), Q, seconds=2)
r.move_tcp((-0.19,-0.10,0.945), Q, seconds=2)
for y in [-0.085,-0.075,-0.065,-0.055,-0.045]:
    code,err=r.move_tcp((-0.19,y,0.945), Q, seconds=3) and (0,0)
    p=r.fk_tcp()[0]; print('   -> tcp y %.4f'%p[1])
" 2>&1 | grep -v '^$'

# openrua op 46
mkdir -p "$(dirname /workspace/plan_insert.py)"
cat > /workspace/plan_insert.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grid-search a 2D (y, z, tilt) insertion path for the bowl into the drawer.

Bowl model: cone frustum, r_bottom..r_top over height H, described by the
outline in the y-z plane (x extent irrelevant: obstacles are y/z only).
Obstacles (world frame, from depth measurements):
  handle:  y >= Y_HANDLE and Z_H_LO <= z <= Z_H_HI (middle drawer handle)
  wall:    y >= Y_WALL_IN and z <= Z_WALL_TOP  (drawer front wall)
  cabinet: y <= Y_CAB (cabinet face)
  floor:   z <= Z_FLOOR
Clearance = min signed distance of bowl outline points to obstacle boxes.
"""
import numpy as np

R_BOT, R_TOP, H = 0.036, 0.056, 0.052
Y_HANDLE, Z_H_LO, Z_H_HI = -0.189, 1.005, 1.035
Y_WALL_IN, Z_WALL_TOP = -0.077, 0.9834
Y_CAB = -0.221
Z_FLOOR = 0.924


def outline(yc, zb, theta):
    """Bowl outline points (y, z) for bottom-center at (yc, zb), tilted by
    theta about x (positive = -y side down)."""
    hs = np.linspace(0, H, 14)
    rs = R_BOT + (R_TOP - R_BOT) * hs / H
    pts = np.concatenate([np.c_[-rs, hs], np.c_[rs, hs],
                          np.c_[np.linspace(-R_BOT, R_BOT, 8), np.zeros(8)]])
    # rotate about the bottom center
    c, s = np.cos(theta), np.sin(theta)
    y = pts[:, 0] * c + pts[:, 1] * s
    z = -pts[:, 0] * s + pts[:, 1] * c
    return np.c_[y + yc, z + zb]


def box_dist(p, ylo, yhi, zlo, zhi):
    dy = np.maximum(np.maximum(ylo - p[:, 0], p[:, 0] - yhi), 0)
    dz = np.maximum(np.maximum(zlo - p[:, 1], p[:, 1] - zhi), 0)
    inside = (p[:, 0] > ylo) & (p[:, 0] < yhi) & (p[:, 1] > zlo) & (p[:, 1] < zhi)
    d = np.hypot(dy, dz)
    d[inside] = -np.minimum.reduce([p[inside, 0] - ylo, yhi - p[inside, 0],
                                    p[inside, 1] - zlo, zhi - p[inside, 1]])
    return d


def clearance(yc, zb, theta):
    p = outline(yc, zb, theta)
    d = np.minimum.reduce([
        box_dist(p, Y_HANDLE, 1.0, Z_H_LO, Z_H_HI),
        box_dist(p, Y_WALL_IN, 1.0, -1.0, Z_WALL_TOP),
        box_dist(p, -1.0, Y_CAB, -1.0, 2.0),
        box_dist(p, -1.0, 1.0, -1.0, Z_FLOOR),
    ])
    return d.min()


if __name__ == "__main__":
    print("LEVEL bowl: best yc per bottom height zb (clearance mm)")
    for zb in np.arange(1.00, 0.925, -0.005):
        ys = np.arange(-0.19, -0.08, 0.001)
        cl = [clearance(y, zb, 0) for y in ys]
        i = int(np.argmax(cl))
        print(f"  zb={zb:.3f} rim={zb+H:.3f}: yc={ys[i]:.3f} clearance={cl[i]*1000:.1f}")
    print("TILTED: best (yc, theta) per zb")
    for zb in np.arange(1.00, 0.925, -0.005):
        best = (-1, 0, 0)
        for th in np.radians(np.arange(-40, 41, 5)):
            for y in np.arange(-0.19, -0.08, 0.001):
                c = clearance(y, zb, th)
                if c > best[0]:
                    best = (c, y, th)
        print(f"  zb={zb:.3f}: yc={best[1]:.3f} theta={np.degrees(best[2]):.0f}deg clearance={best[0]*1000:.1f}")
OPENRUA_EOF

# openrua op 47
python3 plan_insert.py

# openrua op 48
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/plan_insert.py", "old_string": "        box_dist(p, Y_HANDLE, 1.0, Z_H_LO, Z_H_HI),", "new_string": "        box_dist(p, -1.0, Y_HANDLE, Z_H_LO, Z_H_HI),", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 49
python3 plan_insert.py

# openrua op 50
python3 -c "
import numpy as np, plan_insert as P
for th in [0,-10,-15,-20,-25]:
    print(f'theta={th}deg')
    for zb in np.arange(0.99,0.925,-0.005):
        ys=np.arange(-0.19,-0.08,0.001); cl=[P.clearance(y,zb,np.radians(th)) for y in ys]; i=int(np.argmax(cl))
        # also clearance at a fixed yc=-0.135
        print(f'   zb={zb:.3f} best yc={ys[i]:.3f} cl={cl[i]*1000:.1f}mm | yc=-0.135: {P.clearance(-0.135,zb,np.radians(th))*1000:.1f} | yc=-0.13: {P.clearance(-0.13,zb,np.radians(th))*1000:.1f}')
"

# openrua op 51
timeout 900 python3 -u -c "
import numpy as np, ctl
r=ctl.Robot()
Q=ctl.Q_DOWN_FINGERS_X
r.gripper(0.04)
r.move_tcp((-0.205,0.07,1.05), Q, seconds=3)
" 2>&1 | grep -v '^$'; timeout 90 python3 scene.py robot0_eye_in_hand >/dev/null && python3 -c "
import numpy as np, cv2
from scipy.optimize import least_squares
img=cv2.imread('snaps/robot0_eye_in_hand.png'); xyz=np.load('snaps/robot0_eye_in_hand_xyz.npy')
hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
m=(hsv[...,0]>18)&(hsv[...,0]<40)&(hsv[...,1]>80)&(hsv[...,2]>120)
ys,xs=np.nonzero(m); P=xyz[ys,xs]; ok=np.isfinite(P[:,2])&(P[:,2]>0.94); q=P[ok][:,:2]
res=least_squares(lambda c: np.hypot(q[:,0]-c[0],q[:,1]-c[1])-0.0551, x0=[q[:,0].mean(),q[:,1].mean()])
print('bowl center (%.4f,%.4f) rms=%.4f n=%d rim z[%.3f,%.3f]'%(res.x[0],res.x[1],np.sqrt(np.mean(res.fun**2)),len(q),P[ok][:,2].min(),P[ok][:,2].max()))
A=np.c_[2*q[:,0],2*q[:,1],np.ones(len(q))]; bb=(q**2).sum(1); cx,cy,c=np.linalg.lstsq(A,bb,rcond=None)[0]; print('free fit (%.4f,%.4f) r=%.4f'%(cx,cy,np.sqrt(c+cx*cx+cy*cy)))
"

# openrua op 52
timeout 60 python3 tools/perception/cam_snap.py agentview snaps/stuck.png; timeout 20 ros2 topic echo /joint_states --once | grep -A9 position | tail -2; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 53
timeout 900 python3 -u -c "
import numpy as np, ctl
r=ctl.Robot()
Q=ctl.Q_DOWN_FINGERS_X
r.gripper(0.0)
p=r.fk_tcp()[0]; print('tcp',p)
r.move_tcp((p[0],p[1],1.06), Q, seconds=3)
r.gripper(0.04)
" 2>&1 | grep -v '^$'; timeout 20 ros2 topic echo /franka_robot_state_broadcaster/external_wrench --once | grep -A3 force

# openrua op 54
timeout 90 python3 scene.py birdview >/dev/null && python3 -c "
import numpy as np
xyz=np.load('snaps/birdview_xyz.npy'); z=xyz[...,2]
reg=(xyz[...,0]>-0.23)&(xyz[...,0]<0.01)&(xyz[...,1]>-0.25)&(xyz[...,1]<0.0)
m=reg&(z>0.975)&(z<0.995)
for x0 in [-0.22,-0.18,-0.14,-0.10,-0.06,-0.02]:
    k=m&(xyz[...,0]>=x0)&(xyz[...,0]<x0+0.04)
    if k.sum()>2: print(' walltop x %.2f: y[%.3f,%.3f] n=%d'%(x0,xyz[k][:,1].min(),xyz[k][:,1].max(),k.sum()))
m=reg&(z>0.915)&(z<0.935); p=xyz[m]; print('floor x[%.3f,%.3f] y[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()))
m=(z>1.115)&(z<1.13)&(xyz[...,1]<-0.1); p=xyz[m]; print('cabinet top x[%.3f,%.3f] y[%.3f,%.3f]'%(p[:,0].min(),p[:,0].max(),p[:,1].min(),p[:,1].max()))
"

# openrua op 55
timeout 900 python3 -u -c "
import numpy as np, ctl, sys
r=ctl.Robot()
Q=ctl.Q_DOWN_FINGERS_X
for tgt,s in [((-0.208,0.070,1.03),3),((-0.208,0.070,0.93),3)]:
    q=r.ik_tcp(tgt,Q); code,err=r.move_q(q,s)
    if code!=0 or err>0.02: print('STALL, abort'); sys.exit(1)
r.gripper(0.0)
q=r.ik_tcp((-0.208,0.070,1.08),Q); r.move_q(q,3)
print('tcp',r.fk_tcp()[0])
" 2>&1 | grep -v '^$'; timeout 60 python3 tools/perception/cam_snap.py agentview snaps/regrasp.png

# openrua op 56
mkdir -p "$(dirname /workspace/insert.py)"
cat > /workspace/insert.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Closed-loop insertion of the held bowl into the open bottom drawer.

Each step: grab wrist-camera depth+color -> measure bowl rim (yellow px,
fixed-radius circle fit), handle +y edge and drawer front wall inner face
in the SAME frame -> correct y -> descend one step. Stops on stall.
"""
import subprocess
import sys

import cv2
import numpy as np
from scipy.optimize import least_squares

import ctl

R_RIM = 0.0551
H_BOWL = 0.052
Q = ctl.Q_DOWN_FINGERS_X


def measure():
    subprocess.run([sys.executable, "scene.py", "robot0_eye_in_hand"],
                   check=True, capture_output=True)
    img = cv2.imread("snaps/robot0_eye_in_hand.png")
    xyz = np.load("snaps/robot0_eye_in_hand_xyz.npy")
    z = xyz[..., 2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = (hsv[..., 0] > 18) & (hsv[..., 0] < 40) & (hsv[..., 1] > 80) & (hsv[..., 2] > 120)
    ys, xs = np.nonzero(m)
    P = xyz[ys, xs]
    ok = np.isfinite(P[:, 2]) & (P[:, 2] > 0.93)
    q = P[ok][:, :2]
    res = least_squares(lambda c: np.hypot(q[:, 0] - c[0], q[:, 1] - c[1]) - R_RIM,
                        x0=[q[:, 0].mean(), q[:, 1].mean()])
    out = {"bowl": (res.x[0], res.x[1]), "rim_z": float(np.median(P[ok][:, 2])),
           "rms": float(np.sqrt(np.mean(res.fun ** 2))), "n": int(ok.sum())}
    # middle-drawer handle: protrusion at z 1.0-1.04, x in bar range
    k = (z > 1.0) & (z < 1.04) & (xyz[..., 0] > -0.16) & (xyz[..., 0] < -0.05) & \
        (xyz[..., 1] < -0.15) & (xyz[..., 1] > -0.23)
    out["handle_y"] = float(xyz[k][:, 1].max()) if k.sum() > 5 else None
    # drawer front wall top: z ~0.983, take its inner (min y) face
    k = (z > 0.975) & (z < 0.992) & (xyz[..., 0] > -0.21) & (xyz[..., 0] < -0.02) & \
        (xyz[..., 1] > -0.12) & (xyz[..., 1] < -0.04)
    out["wall_in_y"] = float(np.percentile(xyz[k][:, 1], 2)) if k.sum() > 5 else None
    return out


def report(m, r):
    tcp = r.fk_tcp()[0]
    bx, by = m["bowl"]
    zb = m["rim_z"] - H_BOWL
    s = (f"tcp=({tcp[0]:.4f},{tcp[1]:.4f},{tcp[2]:.4f}) bowl=({bx:.4f},{by:.4f}) "
         f"rim_z={m['rim_z']:.4f} bottom={zb:.4f} rms={m['rms']*1000:.1f}mm n={m['n']}")
    if m["handle_y"] is not None:
        s += f" | handle_edge={m['handle_y']:.4f} margin={((by - R_RIM) - m['handle_y'])*1000:.1f}mm"
    if m["wall_in_y"] is not None:
        s += f" | wall_in={m['wall_in_y']:.4f} margin={(m['wall_in_y'] - (by + R_RIM))*1000:.1f}mm"
    print(s, flush=True)
    return tcp, (bx, by), zb


if __name__ == "__main__":
    r = ctl.Robot("insert")
    m = measure()
    report(m, r)
OPENRUA_EOF

# openrua op 57
timeout 900 python3 -u -c "
import numpy as np, ctl, insert, sys
r=ctl.Robot('ins')
Q=ctl.Q_DOWN_FINGERS_X
m=insert.measure(); tcp,(bx,by),zb=insert.report(m,r)
off=np.array([bx-tcp[0], by-tcp[1]]); print('bowl-tcp offset',off)
# move above drawer: bowl at (-0.116,-0.128), bottom at ~1.05 => tcp z = 1.05 + H - (rim_z - tcp_z)
rim_above_tcp = m['rim_z']-tcp[2]; print('rim above tcp', rim_above_tcp)
tz = 1.05 + insert.H_BOWL - rim_above_tcp
tgt=(-0.116-off[0], -0.128-off[1], tz)
q=r.ik_tcp(tgt,Q); code,err=r.move_q(q,4)
if code!=0 or err>0.02: print('STALL'); sys.exit(1)
m=insert.measure(); insert.report(m,r)
" 2>&1 | grep -v '^$'

# openrua op 58
cat >> /workspace/insert.py <<'EOF'


def measure_agent():
    """Bowl rim + middle-handle edge from the static agentview camera."""
    subprocess.run([sys.executable, "scene.py", "agentview"],
                   check=True, capture_output=True)
    img = cv2.imread("snaps/agentview.png")
    xyz = np.load("snaps/agentview_xyz.npy")
    z = xyz[..., 2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    m = (hsv[..., 0] > 18) & (hsv[..., 0] < 40) & (hsv[..., 1] > 80) & (hsv[..., 2] > 120)
    ys, xs = np.nonzero(m)
    P = xyz[ys, xs]
    ok = np.isfinite(P[:, 2]) & (P[:, 2] > 0.93) & (P[:, 0] > -0.3) & (P[:, 0] < 0.0)
    q = P[ok][:, :2]
    res = least_squares(lambda c: np.hypot(q[:, 0] - c[0], q[:, 1] - c[1]) - R_RIM,
                        x0=[q[:, 0].mean(), q[:, 1].mean()])
    k = (z > 1.0) & (z < 1.04) & (xyz[..., 0] > -0.16) & (xyz[..., 0] < -0.05) & \
        (xyz[..., 1] < -0.15) & (xyz[..., 1] > -0.23)
    hy = float(xyz[k][:, 1].max()) if k.sum() > 5 else None
    by = res.x[1]
    print(f"agentview: bowl=({res.x[0]:.4f},{by:.4f}) rim_z={np.median(P[ok][:,2]):.4f} "
          f"rms={np.sqrt(np.mean(res.fun**2))*1000:.1f}mm n={ok.sum()} handle_edge={hy} "
          f"margin={((by - R_RIM) - hy)*1000 if hy else float('nan'):.1f}mm", flush=True)
    return {"bowl": (res.x[0], by), "handle_y": hy}
EOF
timeout 300 python3 -u -c "
import insert, ctl
r=ctl.Robot('ins'); insert.measure_agent()
" 2>&1 | grep -v '^$'

# openrua op 59
python3 -c "
import numpy as np
xyz=np.load('snaps/robot0_eye_in_hand_xyz.npy'); z=xyz[...,2]
k=(z>0.975)&(z<0.992)&(xyz[...,0]>-0.21)&(xyz[...,0]<-0.02)&(xyz[...,1]>-0.12)&(xyz[...,1]<-0.04)
p=xyz[k]; print('n',len(p)); h,e=np.histogram(p[:,1],bins=np.arange(-0.09,-0.05,0.002))
for a,b in zip(e,h): print(f'{a:.3f} {b}')
# also, the drawer floor seen right next to the wall: max y of floor points
k=(z>0.915)&(z<0.935)&(xyz[...,0]>-0.21)&(xyz[...,0]<-0.02); print('floor max y', xyz[k][:,1].max() if k.sum() else None)
"

# openrua op 60
find / -iname "*hand*.stl" -o -iname "*hand*.dae" -o -iname "hand.urdf*" -o -iname "*panda*hand*.xacro" 2>/dev/null | grep -iv node_modules | head; ros2 param get /robot_state_publisher robot_description 2>/dev/null | grep -n -A12 'link name="panda_hand"' | grep -E "box|mesh|cylinder|origin" | head

# openrua op 61
python3 -c "
import numpy as np, struct
def stl(p):
    d=open(p,'rb').read()
    try:
        n=struct.unpack('<I',d[80:84])[0]; a=np.frombuffer(d[84:84+n*50],dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]))['v'].reshape(-1,3)
        if len(a)==n*3 and np.isfinite(a).all(): return a
    except Exception: pass
    import re; return np.array(re.findall(r'vertex\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',d.decode()),float)
for f in ['hand','finger']:
    v=stl(f'/opt/ros/jazzy/share/moveit_resources_panda_description/meshes/collision/{f}.stl'); print(f,'min',v.min(0).round(4),'max',v.max(0).round(4))
"

# openrua op 62
find / \( -name "hand.stl" -o -name "finger.stl" -o -name "hand.dae" -o -name "hand.obj" \) 2>/dev/null | head; ls /opt/ros/jazzy/share/moveit_resources_panda_description/meshes/ 2>/dev/null

# openrua op 63
python3 -c "
import numpy as np
xyz=np.load('snaps/agentview_xyz.npy'); z=xyz[...,2]
for lo in np.arange(1.10,1.24,0.02):
    k=(z>lo)&(z<lo+0.02)&(xyz[...,0]>-0.30)&(xyz[...,0]<-0.02)&(xyz[...,1]>-0.30)&(xyz[...,1]<0.0)
    if k.sum()>5:
        p=xyz[k]; print(f'z {lo:.2f}: n={k.sum()} x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{np.percentile(p[:,1],1):.3f},{np.percentile(p[:,1],99):.3f}]')
"

# openrua op 64
mkdir -p "$(dirname /workspace/descend.py)"
cat > /workspace/descend.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Step the held bowl down into the drawer, re-measuring with the wrist
camera after every step and correcting y. Aborts (and lifts) on stall.

Corridor while the rim passes the middle handle (z 1.005-1.035):
    yc in (-0.134, -0.1215)  -> aim -0.128
Once the rim is below 1.005, shift to yc=-0.145, xc=-0.10, bottom 0.93.
"""
import sys

import numpy as np

import ctl
import insert

r = ctl.Robot("descend")
Q = ctl.Q_DOWN_FINGERS_X
OFF = np.array([0.0499, 0.0021])       # bowl centre - TCP (x, y)
DZ = 0.0348                             # TCP z - bowl bottom z


def step(target_bowl_xy, bottom, seconds=3.0):
    tcp = np.array([target_bowl_xy[0] - OFF[0], target_bowl_xy[1] - OFF[1], bottom + DZ])
    q = r.ik_tcp(tcp, Q)
    if q is None:
        print("IK failed", file=sys.stderr)
        return False
    code, err = r.move_q(q, seconds)
    if code != 0 or err > 0.02:
        print(f"STALL code={code} err={err:.4f} -> lifting 3cm", file=sys.stderr)
        here = r.fk_tcp()[0]
        r.move_tcp(here + [0, 0, 0.03], Q, 2.0)
        return False
    return True


def check(tag):
    m = insert.measure()
    print(f"[{tag}]", end=" ")
    tcp, (bx, by), zb = insert.report(m, r)
    return bx, by, zb


bx, by, zb = check("start")
yc = -0.128
# phase 1: descend through the handle band, correcting y each step
for bottom in (1.02, 1.00, 0.985, 0.97, 0.955):
    # correct with measured bowl position (offset drift), keep target yc
    corr = np.array([bx, by]) - (r.fk_tcp()[0][:2] + OFF)
    tgt = np.array([bx, yc]) - corr
    if not step(tgt, bottom):
        sys.exit(1)
    bx, by, zb = check(f"bottom={bottom}")
    if not (-0.136 < by < -0.119):
        print("bowl y out of corridor, stopping", file=sys.stderr)
        sys.exit(1)
# phase 2: below the handle -> shift to final xy and set down
corr = np.array([bx, by]) - (r.fk_tcp()[0][:2] + OFF)
tgt = np.array([-0.10, -0.145]) - corr
if not step(tgt, 0.94, 3.0):
    sys.exit(1)
bx, by, zb = check("final xy")
if not step(np.array([bx, by]) - corr * 0, 0.928, 2.0):
    sys.exit(1)
check("set down")
print("DONE descent; bowl still grasped", flush=True)
OPENRUA_EOF

# openrua op 65
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/descend.py", "old_string": "if not step(np.array([bx, by]) - corr * 0, 0.928, 2.0):", "new_string": "if not step(r.fk_tcp()[0][:2] + OFF, 0.928, 2.0):   # straight down, no xy change", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 66
timeout 900 python3 descend.py 2>&1 | grep -v "^\[INFO\]"

# openrua op 67
timeout 600 python3 -c "
import ctl, insert, numpy as np
r=ctl.Robot('release'); Q=ctl.Q_DOWN_FINGERS_X
r.gripper(0.08)
m=insert.measure(); insert.report(m,r)
tcp=r.fk_tcp()[0]
print('lift 1'); r.move_tcp(tcp+[0,0,0.04],Q,2.0)
m=insert.measure(); insert.report(m,r)
print('lift 2'); r.move_tcp(tcp+[0,0,0.16],Q,3.0)
m=insert.measure(); insert.report(m,r)
" 2>&1 | grep -v "^\[INFO\]"

# openrua op 68
for c in agentview frontview; do python3 scene.py $c 2>&1 | grep -v INFO; done; python3 -c "
import numpy as np
xyz=np.load('snaps/agentview_xyz.npy'); z=xyz[...,2]
# drawer front region: x in drawer span, y > -0.10, z 0.90..1.0
for lo in np.arange(0.90,1.00,0.01):
    k=(z>lo)&(z<lo+0.01)&(xyz[...,0]>-0.25)&(xyz[...,0]<0.0)&(xyz[...,1]>-0.12)&(xyz[...,1]<0.05)
    if k.sum()>5:
        p=xyz[k]; print(f'z {lo:.2f}: n={k.sum()} x[{np.percentile(p[:,0],1):.3f},{np.percentile(p[:,0],99):.3f}] y[{np.percentile(p[:,1],1):.3f},{np.percentile(p[:,1],99):.3f}]')
"

# openrua op 69
python3 -c "
import numpy as np
xyz=np.load('snaps/agentview_xyz.npy'); z=xyz[...,2]
k=(z>0.935)&(z<0.965)&(xyz[...,1]>-0.058)&(xyz[...,1]<-0.02)&(xyz[...,0]>-0.25)&(xyz[...,0]<0.0)
p=xyz[k]; print('bottom handle: n',len(p),'x',np.percentile(p[:,0],[1,99]).round(3),'y',np.percentile(p[:,1],[1,99]).round(3))
" ; timeout 300 python3 -c "
import ctl, numpy as np
r=ctl.Robot('ikchk')
th=-np.pi/4; qx=(np.sin(th/2),0,0,np.cos(th/2))
Q=ctl.quat_mul(qx, ctl.Q_DOWN_FINGERS_X)
R=ctl.quat_to_R(*Q); print('hand z axis',R[:,2].round(3),'hand y axis',R[:,1].round(3))
seed=r.arm_q()
for y in (-0.03,-0.05,-0.10,-0.15,-0.20,-0.215):
    q=r.ik_tcp([-0.19,y,0.955],Q,seed=seed)
    if q: 
        p,_=r.fk_tcp(q); h,_=r.fk_hand(q); print(f'y={y}: ok tcp {p.round(4)} hand {h.round(4)} q {np.round(q,2)}'); seed=q
    else: print(f'y={y}: IK FAIL')
" 2>&1 | grep -v INFO

# openrua op 70
mkdir -p "$(dirname /workspace/push.py)"
cat > /workspace/push.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Close the bottom drawer: push its front panel toward -y with the closed
fingertips, hand pitched 45deg so the fingertips lead the palm (keeps the
palm clear of the middle-drawer handle at y=-0.189, z 1.005-1.035)."""
import subprocess
import sys

import numpy as np

import ctl

r = ctl.Robot("push")
th = -np.pi / 4
Q = ctl.quat_mul((np.sin(th / 2), 0, 0, np.cos(th / 2)), ctl.Q_DOWN_FINGERS_X)
X, Z = -0.19, 0.955


def go(pos, secs=3.0):
    q = r.ik_tcp(pos, Q)
    assert q is not None, "IK failed"
    code, err = r.move_q(q, secs)
    return code == 0 and err < 0.02


def panel_y():
    subprocess.run([sys.executable, "scene.py", "agentview"], check=True, capture_output=True)
    xyz = np.load("snaps/agentview_xyz.npy")
    z = xyz[..., 2]
    # panel top strip (z ~0.983) away from handle/fingers: x in [-0.22,-0.16]
    k = (z > 0.975) & (z < 0.992) & (xyz[..., 0] > -0.24) & (xyz[..., 0] < -0.16) & \
        (xyz[..., 1] > -0.24) & (xyz[..., 1] < 0.0)
    p = xyz[k]
    return (float(np.percentile(p[:, 1], 2)), float(np.percentile(p[:, 1], 98)), int(k.sum())) if k.sum() > 5 else None


r.gripper(0.0)
print("panel top y before:", panel_y(), flush=True)
assert go([X, -0.03, 1.03])
assert go([X, -0.03, Z], 2.0)
for y in (-0.05, -0.08, -0.11, -0.14, -0.17, -0.19, -0.205, -0.212):
    ok = go([X, y, Z], 2.0)
    tcp = r.fk_tcp()[0]
    print(f"push to y={y}: ok={ok} tcp={tcp.round(4)}", flush=True)
    if not ok:
        print("stall -> stop pushing", file=sys.stderr)
        break
print("panel top y after:", panel_y(), flush=True)
# retreat: back off +y then up
tcp = r.fk_tcp()[0]
go([X, tcp[1] + 0.03, Z], 2.0)
go([X, tcp[1] + 0.03, 1.08], 2.0)
print("retreated; tcp", r.fk_tcp()[0].round(4), flush=True)
OPENRUA_EOF

# openrua op 71
timeout 900 python3 push.py 2>&1 | grep -v "^\[INFO\]"

# openrua op 72
timeout 300 python3 -c "
import ctl, numpy as np
r=ctl.Robot('chk'); q=r.arm_q(); print('q',np.round(q,3)); p,qq=r.fk_tcp(); print('tcp',p.round(4),'quat',np.round(qq,3)); print('hand',r.fk_hand()[0].round(4))
import rclpy
from geometry_msgs.msg import WrenchStamped
got={}
s=r.node.create_subscription(WrenchStamped,'/franka_robot_state_broadcaster/external_wrench',lambda m: got.setdefault('m',m),1)
import time; t=time.time()
while 'm' not in got and time.time()-t<10: rclpy.spin_once(r.node,timeout_sec=0.2)
if 'm' in got: w=got['m'].wrench; print('wrench F',round(w.force.x,1),round(w.force.y,1),round(w.force.z,1))
" 2>&1 | grep -v INFO; python3 scene.py agentview 2>&1 | grep -v INFO

# openrua op 73
timeout 300 python3 -c "
import ctl, numpy as np
r=ctl.Robot('retry')
th=-np.pi/4; Q=ctl.quat_mul((np.sin(th/2),0,0,np.cos(th/2)), ctl.Q_DOWN_FINGERS_X)
q=r.ik_tcp([-0.19,-0.03,1.03],Q); print('target q',np.round(q,3)); print('now q',np.round(r.arm_q(),3))
code,err=r.move_q(q,6.0); print('after q',np.round(r.arm_q(),3)); print('tcp',r.fk_tcp()[0].round(4))
" 2>&1 | grep -v INFO

# openrua op 74
sed -i 's/^r.gripper(0.0)$/print("gap", r.finger_gap())/; s/^assert go(\[X, -0.03, 1.03\])$/assert go([X, -0.03, 1.03], 4.0)/; s/^assert go(\[X, -0.03, Z\], 2.0)$/assert go([X, -0.03, Z], 3.0)/; s/ok = go(\[X, y, Z\], 2.0)/ok = go([X, y, Z], 3.0)/' push.py && timeout 900 python3 push.py 2>&1 | grep -v "^\[INFO\]"

# openrua op 75
for c in agentview frontview birdview; do python3 scene.py $c 2>&1 | grep -v INFO; done; python3 -c "
import numpy as np, cv2
xyz=np.load('snaps/agentview_xyz.npy'); z=xyz[...,2]
# anything drawer-like protruding in front of the cabinet face (y>-0.215) in the drawer x span, z 0.905-0.99?
k=(z>0.905)&(z<0.99)&(xyz[...,0]>-0.24)&(xyz[...,0]<0.0)&(xyz[...,1]>-0.215)&(xyz[...,1]<0.0)
print('points in front of cabinet face (z .905-.99):',k.sum())
if k.sum(): p=xyz[k]; print(' x',np.percentile(p[:,0],[1,99]).round(3),' y',np.percentile(p[:,1],[1,99]).round(3),' z',np.percentile(p[:,2],[1,99]).round(3))
img=cv2.imread('snaps/agentview.png'); hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
m=(hsv[...,0]>18)&(hsv[...,0]<40)&(hsv[...,1]>80)&(hsv[...,2]>120); print('yellow rim pixels visible (agentview):',m.sum())
"
