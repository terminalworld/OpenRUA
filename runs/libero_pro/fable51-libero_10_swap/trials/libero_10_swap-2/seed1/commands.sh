#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la && find . -maxdepth 3 -type f | head -100

# openrua op 1
cat /workspace/tools/README.md; echo ----; cat /workspace/tools/perception/cam_snap.py; echo ----; cat /workspace/tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py; echo ----; cat tools/action/gripper_cmd.py; echo ----; cat tools/action/ik_move.py

# openrua op 3
timeout 30 ros2 topic list; echo ---; timeout 30 ros2 action list; echo ---; timeout 30 ros2 service list | grep -v -E "parameter|describe_|get_type" ; echo ---; timeout 20 ros2 node list

# openrua op 4
for c in agentview frontview sideview birdview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
timeout 30 ros2 node info /robot_bridge; echo ---; timeout 10 ros2 topic echo /pipeline_state --once

# openrua op 6
timeout 20 ros2 topic echo /tf --once | head -80; echo ---; timeout 20 ros2 topic echo /tf_static --once | grep -E "frame_id|child" | head -60

# openrua op 7
for p in "250 288" "375 280" "378 238" "325 315" "300 200" "450 380"; do echo -n "$p -> "; timeout 60 python3 tools/perception/px2world.py birdview $p; done

# openrua op 8
mkdir -p "$(dirname /workspace/analyze_bird.py)"
cat > /workspace/analyze_bird.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Segment objects above the table in the birdview depth and print world bboxes."""
import numpy as np, cv2, rclpy, yaml
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

rclpy.init(); node = rclpy.create_node("ab")
got = {}
node.create_subscription(Image, "/birdview/depth/image_raw", lambda m: got.setdefault("d", m), 1)
node.create_subscription(CameraInfo, "/birdview/color/camera_info", lambda m: got.setdefault("i", m), 1)
node.create_subscription(Image, "/birdview/color/image_raw", lambda m: got.setdefault("c", m), 1)
while len(got) < 3: rclpy.spin_once(node, timeout_sec=0.2)
d = CvBridge().imgmsg_to_cv2(got["d"], "passthrough").astype(np.float32)
c = CvBridge().imgmsg_to_cv2(got["c"], "bgr8")
k = got["i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
# camera at (-0.2, 0, 3.0), looking straight down; optical: x_img -> world?, check via quaternion
# q = (0.7071, 0.7071, 0, 0): R = rot 180deg about (1,1,0)/sqrt2 axis -> maps cam x->world y, cam y->world x, cam z->world -z
H, W = d.shape
vv, uu = np.mgrid[0:H, 0:W]
X = (uu - cx) * d / fx; Y = (vv - cy) * d / fy
wx = -0.2 + Y; wy = 0.0 + X; wz = 3.0 - d
np.save("bird_wz.npy", wz)
table = np.abs(wz - 0.90) < 0.005
above = (wz > 0.905) & (wz < 1.3) & (wx > -0.5) & (wx < 0.5) & (np.abs(wy) < 0.6)
n, lab, stats, cent = cv2.connectedComponentsWithStats(above.astype(np.uint8), 8)
for i in range(1, n):
    m = lab == i
    if m.sum() < 30: continue
    print(f"comp {i}: px={m.sum()} u={stats[i][0]}..{stats[i][0]+stats[i][2]} v={stats[i][1]}..{stats[i][1]+stats[i][3]}"
          f" wx={wx[m].min():.3f}..{wx[m].max():.3f} wy={wy[m].min():.3f}..{wy[m].max():.3f} zmax={wz[m].max():.3f} zmed={np.median(wz[m]):.3f}")
OPENRUA_EOF

# openrua op 9
timeout 120 python3 analyze_bird.py

# openrua op 10
python3 -c "
import cv2
im=cv2.imread('agentview.png'); cv2.imwrite('crop_agent_knob.png', cv2.resize(im[190:280,370:470],None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('sideview.png'); cv2.imwrite('crop_side_knob.png', cv2.resize(im[300:380,240:420],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
im=cv2.imread('birdview.png'); cv2.imwrite('crop_bird.png', cv2.resize(im[200:360,200:420],None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))
"

# openrua op 11
mkdir -p "$(dirname /workspace/cloud.py)"
cat > /workspace/cloud.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Build world-frame point clouds from the named cameras; save as npy (N x 6: xyz bgr)."""
import sys, numpy as np, rclpy
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def grab(node, cams):
    got = {}
    subs = []
    for c in cams:
        subs.append(node.create_subscription(Image, f"/{c}/depth/image_raw", lambda m, c=c: got.setdefault(c + "d", m), 1))
        subs.append(node.create_subscription(Image, f"/{c}/color/image_raw", lambda m, c=c: got.setdefault(c + "c", m), 1))
        subs.append(node.create_subscription(CameraInfo, f"/{c}/color/camera_info", lambda m, c=c: got.setdefault(c + "i", m), 1))
    while len(got) < 3 * len(cams):
        rclpy.spin_once(node, timeout_sec=0.2)
    for s in subs:
        node.destroy_subscription(s)
    return got


def main():
    cams = sys.argv[1:] or ["birdview", "agentview", "frontview", "sideview"]
    rclpy.init(); node = rclpy.create_node("cloud")
    buf = Buffer(); TransformListener(buf, node)
    got = grab(node, cams)
    br = CvBridge()
    for c in cams:
        frame = f"{c}_optical_frame"
        while not buf.can_transform("world", frame, rclpy.time.Time()):
            rclpy.spin_once(node, timeout_sec=0.2)
        t = buf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation; R = quat_R(q.x, q.y, q.z, q.w)
        T = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        d = br.imgmsg_to_cv2(got[c + "d"], "passthrough").astype(np.float64)
        col = br.imgmsg_to_cv2(got[c + "c"], "bgr8")
        k = got[c + "i"].k; fx, fy, cx, cy = k[0], k[4], k[2], k[5]
        H, W = d.shape
        vv, uu = np.mgrid[0:H, 0:W]
        ok = np.isfinite(d) & (d > 0.05) & (d < 5)
        X = (uu - cx) * d / fx; Y = (vv - cy) * d / fy
        P = np.stack([X, Y, d], -1)[ok] @ R.T + T
        out = np.concatenate([P, col[ok].astype(np.float64), np.stack([uu, vv], -1)[ok]], 1)
        np.save(f"cloud_{c}.npy", out)
        print(c, out.shape, "cam at", T)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 12
timeout 180 python3 cloud.py birdview agentview frontview sideview && python3 -c "
import numpy as np
for c in ['birdview','agentview','frontview','sideview']:
    P=np.load(f'cloud_{c}.npy')
    # moka pot region
    m=(P[:,0]>-0.15)&(P[:,0]<0.05)&(P[:,1]>-0.35)&(P[:,1]<-0.15)&(P[:,2]>0.905)
    Q=P[m]
    print(c,'moka pts',len(Q))
    for z0 in np.arange(0.91,1.07,0.02):
        s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
        if len(s): print(f'  z {z0:.2f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f} n={len(s)}')
"


# openrua op 13
python3 -c "
import numpy as np
P=np.concatenate([np.load(f'cloud_{c}.npy') for c in ['birdview','agentview','frontview','sideview']])
m=(P[:,0]>-0.15)&(P[:,0]<0.05)&(P[:,1]>-0.35)&(P[:,1]<-0.15)&(P[:,2]>0.905)
Q=P[m]
for z0 in np.arange(0.905,1.07,0.01):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.01)]
    b=s[s[:,1]>-0.27]
    if len(b): print(f'z {z0:.3f}: body x {b[:,0].min():.3f}..{b[:,0].max():.3f} ({b[:,0].max()-b[:,0].min():.3f}) y {b[:,1].min():.3f}..{b[:,1].max():.3f} ({b[:,1].max()-b[:,1].min():.3f}) handle_pts={len(s)-len(b)} handle_x={s[s[:,1]<=-0.27][:,0].min() if len(s)>len(b) else 0:.3f}..{s[s[:,1]<=-0.27][:,0].max() if len(s)>len(b) else 0:.3f}')
"


# openrua op 14
find / -iname "*stove*" 2>/dev/null | head; find / -iname "*moka*" 2>/dev/null | head; find / -iname "*libero*" -maxdepth 6 2>/dev/null | head; ls /opt 2>/dev/null; pip list 2>/dev/null | grep -i -E "robosuite|libero|mujoco"

# openrua op 15
python3 -c "
import numpy as np
P=np.concatenate([np.load(f'cloud_{c}.npy') for c in ['birdview','agentview','frontview','sideview']])
# knob
m=(P[:,0]>-0.30)&(P[:,0]<-0.13)&(P[:,1]>0.12)&(P[:,1]<0.27)&(P[:,2]>0.903)
Q=P[m]
print('knob')
for z0 in np.arange(0.90,0.98,0.005):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.005)]
    if len(s): print(f'z {z0:.3f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f} ({s[:,0].max()-s[:,0].min():.3f}) y {s[:,1].min():.3f}..{s[:,1].max():.3f} ({s[:,1].max()-s[:,1].min():.3f}) n={len(s)} cx={s[:,0].mean():.3f} cy={s[:,1].mean():.3f}')
# stove plate
m=(P[:,0]>-0.20)&(P[:,0]<0.10)&(P[:,1]>0.05)&(P[:,1]<0.35)&(P[:,2]>0.903)&(P[:,2]<0.96)
Q=P[m]
print('stove')
for z0 in np.arange(0.90,0.96,0.005):
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.005)]
    if len(s): print(f'z {z0:.3f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f} y {s[:,1].min():.3f}..{s[:,1].max():.3f} n={len(s)}')
B=np.load('cloud_birdview.npy')
m=(B[:,0]>-0.16)&(B[:,0]<0.05)&(B[:,1]>0.09)&(B[:,1]<0.31)&(B[:,2]>0.92)&(B[:,2]<0.95)
s=B[m]; print('plate top from bird: x',s[:,0].min(),s[:,0].max(),'y',s[:,1].min(),s[:,1].max(),'z med',np.median(s[:,2]))
"


# openrua op 16
timeout 60 ros2 service call /compute_fk moveit_msgs/srv/GetPositionFK "{header: {frame_id: ''}, fk_link_names: ['panda_hand'], robot_state: {joint_state: {name: [panda_joint1,panda_joint2,panda_joint3,panda_joint4,panda_joint5,panda_joint6,panda_joint7], position: [0.0,-0.161037389,0.0,-2.44459747,0.0,2.2267522,0.7853981633974483]}}}" 2>&1 | tail -30

# openrua op 17
mkdir -p "$(dirname /workspace/rob.py)"
cat > /workspace/rob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Small helper library: joint state, FK, IK, trajectory, gripper, servo."""
import math, time
import numpy as np
import rclpy, yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped, WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
ARM = FJT["joints"]
LIMITS = FJT["limits_rad"]
TCP = float(M["hand"]["tcp_offset_m"])


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def quat_down(yaw):
    """Hand pointing straight down, fingers closing along world direction at angle `yaw`
    from world y (yaw=0 -> fingers along y, yaw=pi/2 -> fingers along x)."""
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    # qz(yaw) * (1,0,0,0)
    return (c, s, 0.0, 0.0)


class Robot:
    def __init__(self, name="rob"):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node(name)
        self._js = None
        self._wr = None
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.node.create_subscription(WrenchStamped, "/franka_robot_state_broadcaster/external_wrench", self._on_wr, 10)
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.twist = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.fjt.wait_for_server(10); self.grip.wait_for_server(10)
        self.ik.wait_for_service(10); self.fk.wait_for_service(10)

    def _on_js(self, m): self._js = m
    def _on_wr(self, m): self._wr = m

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def joints(self, fresh=True):
        if fresh:
            self._js = None
        while self._js is None:
            self.spin(0.1)
        d = dict(zip(self._js.name, self._js.position))
        return d

    def arm_q(self):
        d = self.joints()
        return [d[j] for j in ARM]

    def fingers(self):
        d = self.joints()
        return d.get("panda_finger_joint1"), d.get("panda_finger_joint2")

    def wrench(self):
        self._wr = None
        while self._wr is None:
            self.spin(0.1)
        f = self._wr.wrench.force; t = self._wr.wrench.torque
        return np.array([f.x, f.y, f.z]), np.array([t.x, t.y, t.z])

    def fk_pose(self, q=None):
        req = GetPositionFK.Request()
        req.header.frame_id = ""
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = list(ARM)
        req.robot_state.joint_state.position = [float(v) for v in (q or self.arm_q())]
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        r = fut.result()
        if r is None or r.error_code.val != 1:
            raise RuntimeError(f"FK failed {r and r.error_code.val}")
        p = r.pose_stamped[0].pose
        return np.array([p.position.x, p.position.y, p.position.z]), \
            (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)

    def tcp(self, q=None):
        p, quat = self.fk_pose(q)
        R = quat_R(*quat)
        return p + TCP * R[:, 2], quat

    def ik_hand(self, xyz, quat, seed=None, timeout=20.0):
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, xyz)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.ik_link_name = "panda_hand"
        seed = seed or self.arm_q()
        req.ik_request.robot_state.joint_state.name = list(ARM)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in seed]
        req.ik_request.timeout.sec = int(timeout)
        req.ik_request.timeout.nanosec = int((timeout % 1) * 1e9)
        req.ik_request.avoid_collisions = False
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=90)
        r = fut.result()
        if r is None:
            raise RuntimeError("IK no answer")
        if r.error_code.val != 1:
            return None
        sol = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [sol[j] for j in ARM]

    def ik_tcp(self, xyz, quat, seed=None, tries=5):
        R = quat_R(*quat)
        hand = np.array(xyz, float) - TCP * R[:, 2]
        seed = seed or self.arm_q()
        best = None
        for i in range(tries):
            q = self.ik_hand(hand, quat, seed=seed)
            if q is not None:
                # verify
                t, _ = self.tcp(q)
                err = np.linalg.norm(t - np.array(xyz))
                if err < 0.005:
                    if best is None or self._dist(q, seed) < self._dist(best, seed):
                        best = q
                    if i >= 1:
                        break
            seed = [s + np.random.uniform(-0.3, 0.3) for s in seed]
        return best

    @staticmethod
    def _dist(a, b):
        return float(np.linalg.norm(np.array(a) - np.array(b)))

    def move_q(self, q, seconds=None, verbose=True):
        cur = self.arm_q()
        d = max(abs(a - b) for a, b in zip(q, cur))
        if seconds is None:
            seconds = max(1.0, min(6.0, d / 0.5))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(ARM)
        pt = JointTrajectoryPoint(positions=[float(v) for v in q])
        pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [pt]
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
        code = res.result().result.error_code
        after = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q, after))
        if verbose:
            print(f"move_q: code={code} maxerr={err:.4f} ({seconds:.1f}s)", flush=True)
        return code, err

    def move_tcp(self, xyz, quat, seconds=None, seed=None):
        q = self.ik_tcp(xyz, quat, seed=seed)
        if q is None:
            print("IK FAILED for", xyz, quat, flush=True)
            return None
        code, err = self.move_q(q, seconds)
        t, _ = self.tcp()
        print(f"  tcp now {t.round(4)} target {np.round(xyz, 4)}", flush=True)
        return q

    def gripper(self, width, wait=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        res = fut.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=wait)
        r = res.result().result
        f = self.fingers()
        print(f"gripper({width}): reached={r.reached_goal} stalled={r.stalled} fingers={f}", flush=True)
        return f

    def servo(self, v, n=20, w=(0, 0, 0)):
        msg = TwistStamped(); msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = map(float, v)
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = map(float, w)
        for _ in range(n):
            self.twist.publish(msg); self.spin(0.05)
OPENRUA_EOF

# openrua op 18
timeout 300 python3 -u -c "
from rob import *
r=Robot('t1')
print('q',np.round(r.arm_q(),3))
p,q=r.fk_pose(); print('hand',p.round(4),np.round(q,4))
t,_=r.tcp(); print('tcp',t.round(4))
# IK test: same pose back
sol=r.ik_hand(p,q); print('ik same pose ->',None if sol is None else np.round(sol,3))
# IK for knob approach: tcp above knob at world (-0.205,0.194, 1.0), fingers along y
sol=r.ik_tcp((-0.205,0.194,1.0), quat_down(0)); print('ik knob above ->',None if sol is None else np.round(sol,3))
if sol: print(' tcp check', r.tcp(sol)[0].round(4))
sol2=r.ik_tcp((-0.042,-0.237,1.05), quat_down(math.pi/2)); print('ik moka above ->',None if sol2 is None else np.round(sol2,3))
if sol2: print(' tcp check', r.tcp(sol2)[0].round(4), np.round(r.tcp(sol2)[1],3))
sol3=r.ik_tcp((-0.042,-0.237,0.965), quat_down(math.pi/2)); print('ik moka grasp ->',None if sol3 is None else np.round(sol3,3))
sol4=r.ik_tcp((-0.057,0.195,1.0), quat_down(math.pi/2)); print('ik stove place ->',None if sol4 is None else np.round(sol4,3))
"


# openrua op 19
mkdir -p "$(dirname /workspace/step1_knob.py)"
cat > /workspace/step1_knob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Approach the stove knob: open gripper, hover above the fin, descend to grasp height."""
from rob import *
import subprocess

KNOB = np.array([-0.205, 0.194])
r = Robot("step1")
r.gripper(GRIP["open_m"])
q = r.move_tcp((KNOB[0], KNOB[1], 1.02), quat_down(0), seconds=4)
print("hover q", np.round(q, 3), flush=True)
subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", "eih_knob_hover.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_knob_hover.png"])
print("fingers", r.fingers(), flush=True)
OPENRUA_EOF

# openrua op 20
timeout 900 python3 -u step1_knob.py 2>&1 | tail -20

# openrua op 21
mkdir -p "$(dirname /workspace/step2_knob.py)"
cat > /workspace/step2_knob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Descend onto the knob fin, grasp, rotate joint7 by DELTA, release, lift."""
import sys, subprocess
from rob import *

KNOB = np.array([-0.205, 0.194])
DELTA = float(sys.argv[1]) if len(sys.argv) > 1 else -math.pi / 2
r = Robot("step2")
print("wrench before", r.wrench()[0].round(2), flush=True)
q = r.move_tcp((KNOB[0], KNOB[1], 0.945), quat_down(0), seconds=3)
print("wrench at grasp height", r.wrench()[0].round(2), flush=True)
f = r.gripper(GRIP["closed_m"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_knob_grasp.png"])
q = r.arm_q()
q7 = q[6] + DELTA
if not (LIMITS[6][0] < q7 < LIMITS[6][1]):
    raise SystemExit(f"joint7 target {q7} out of limits")
q2 = list(q); q2[6] = q7
r.move_q(q2, seconds=3)
print("wrench after rotate", r.wrench()[0].round(2), flush=True)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_knob_rotated.png"])
r.gripper(GRIP["open_m"])
r.move_tcp((KNOB[0], KNOB[1], 1.05), quat_down(0), seconds=2)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_knob_after.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "sideview", "side_knob_after.png"])
OPENRUA_EOF

# openrua op 22
timeout 1200 python3 -u step2_knob.py 2>&1 | tail -20

# openrua op 23
timeout 120 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np
B=np.load('cloud_birdview.npy')
m=(B[:,0]>-0.30)&(B[:,0]<-0.13)&(B[:,1]>0.12)&(B[:,1]<0.27)&(B[:,2]>0.935)
s=B[m]; xy=s[:,:2]-s[:,:2].mean(0)
u,sv,vt=np.linalg.svd(xy,full_matrices=False)
d=vt[0]; import math
print('fin pts',len(s),'center',s[:,:2].mean(0).round(3),'dir',d.round(3),'angle from x (deg)',math.degrees(math.atan2(d[1],d[0])))
" ; python3 -c "
import cv2; im=cv2.imread('birdview.png'); cv2.imwrite('crop_bird2.png', cv2.resize(im[200:300,330:420],None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))"

# openrua op 24
timeout 60 python3 tools/perception/cam_snap.py birdview bird2.png && python3 -c "
import cv2; im=cv2.imread('bird2.png'); cv2.imwrite('crop_bird2.png', cv2.resize(im[200:300,330:420],None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))"

# openrua op 25
mkdir -p "$(dirname /workspace/step3_knob_more.py)"
cat > /workspace/step3_knob_more.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Re-grasp the (already partly turned) fin and rotate it further in small steps."""
import sys, subprocess
from rob import *

KNOB = np.array([-0.205, 0.194])
FIN_DEG = float(sys.argv[1]) if len(sys.argv) > 1 else -31.0
STEPS = int(sys.argv[2]) if len(sys.argv) > 2 else 3
STEP = -0.35
yaw = math.radians(FIN_DEG)
r = Robot("step3")
r.gripper(GRIP["open_m"])
r.move_tcp((KNOB[0], KNOB[1], 1.02), quat_down(yaw), seconds=3)
subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", "eih_knob2.png"])
r.move_tcp((KNOB[0], KNOB[1], 0.945), quat_down(yaw), seconds=2)
f = r.gripper(GRIP["closed_m"])
q0 = r.arm_q()
print("q7 start", round(q0[6], 3), flush=True)
for i in range(STEPS):
    q = r.arm_q()
    q2 = list(q); q2[6] = q[6] + STEP
    if not (LIMITS[6][0] < q2[6] < LIMITS[6][1]):
        print("joint7 limit reached"); break
    code, err = r.move_q(q2, seconds=1.5)
    qn = r.arm_q()
    print(f"step {i}: q7 {q[6]:.3f} -> {qn[6]:.3f} (moved {qn[6]-q[6]:.3f}) code={code} fingers={r.fingers()}", flush=True)
    if abs(qn[6] - q[6]) < 0.15:
        print("knob resisting; stop"); break
print("total joint7 rotation this session", round(r.arm_q()[6] - q0[6], 3), flush=True)
r.gripper(GRIP["open_m"])
r.move_tcp((KNOB[0], KNOB[1], 1.08), quat_down(yaw), seconds=2)
# park the arm up and away from the knob so cameras can see it
r.move_tcp((-0.35, 0.0, 1.20), quat_down(0), seconds=3)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_knob3.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "birdview", "bird_knob3.png"])
OPENRUA_EOF

# openrua op 26
timeout 1500 python3 -u step3_knob_more.py -31 3 2>&1 | tail -25

# openrua op 27
python3 -c "
import cv2; im=cv2.imread('bird_knob3.png'); cv2.imwrite('crop_bird3.png', cv2.resize(im[200:300,330:420],None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))"; timeout 120 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np, math
B=np.load('cloud_birdview.npy')
m=(B[:,0]>-0.30)&(B[:,0]<-0.13)&(B[:,1]>0.12)&(B[:,1]<0.27)&(B[:,2]>0.935)&(B[:,2]<0.99)
s=B[m]; xy=s[:,:2]-s[:,:2].mean(0)
u,sv,vt=np.linalg.svd(xy,full_matrices=False)
d=vt[0]
print('fin pts',len(s),'center',s[:,:2].mean(0).round(3),'zmax',s[:,2].max().round(3),'dir',d.round(3),'angle from x (deg)',math.degrees(math.atan2(d[1],d[0])))
"

# openrua op 28
timeout 120 python3 -u -c "
from rob import *
r=Robot('chk')
print('q',np.round(r.arm_q(),3)); t,q=r.tcp(); print('tcp',t.round(4),np.round(q,3)); print('fingers',r.fingers()); print('wrench',r.wrench()[0].round(2))
"

# openrua op 29
mkdir -p "$(dirname /workspace/step4_knob.py)"
cat > /workspace/step4_knob.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Re-grasp the fin at its current angle and rotate it further by DELTA (one move)."""
import sys, subprocess
from rob import *

KNOB = np.array([-0.205, 0.194])
FIN_DEG = float(sys.argv[1])
DELTA = float(sys.argv[2]) if len(sys.argv) > 2 else -0.4
HOME = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
yaw = math.radians(FIN_DEG)
r = Robot("step4")
r.gripper(GRIP["open_m"])
r.move_tcp((KNOB[0], KNOB[1], 1.02), quat_down(yaw))
subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", "eih_knob4.png"])
r.move_tcp((KNOB[0], KNOB[1], 0.945), quat_down(yaw))
f = r.gripper(GRIP["closed_m"])
if abs(f[0]) < 0.008:
    print("MISSED the fin (fingers closed on air); releasing and lifting", flush=True)
    r.gripper(GRIP["open_m"])
    r.move_tcp((KNOB[0], KNOB[1], 1.05), quat_down(yaw))
    r.move_q(HOME)
    raise SystemExit(1)
q = r.arm_q()
q2 = list(q); q2[6] = q[6] + DELTA
code, err = r.move_q(q2, seconds=2.0)
qn = r.arm_q()
print(f"rotate: q7 {q[6]:.3f} -> {qn[6]:.3f} (moved {qn[6]-q[6]:.3f}) code={code} fingers={r.fingers()}", flush=True)
r.gripper(GRIP["open_m"])
r.move_tcp((KNOB[0], KNOB[1], 1.05), quat_down(yaw))
r.move_q(HOME)
print("tcp", r.tcp()[0].round(3), flush=True)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_knob4.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "birdview", "bird_knob4.png"])
OPENRUA_EOF

# openrua op 30
timeout 1500 python3 -u step4_knob.py 35 -0.4 2>&1 | tail -25

# openrua op 31
python3 - <<'EOF'
import re
s=open('rob.py').read()
s=s.replace("""        if seconds is None:
            seconds = max(1.0, min(6.0, d / 0.5))""","""        if seconds is None:
            seconds = max(1.5, min(8.0, d / 0.25))""")
s=s.replace("""        code = res.result().result.error_code
        after = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q, after))
        if verbose:
            print(f"move_q: code={code} maxerr={err:.4f} ({seconds:.1f}s)", flush=True)
        return code, err""","""        code = res.result().result.error_code
        after = self.arm_q()
        err = max(abs(a - b) for a, b in zip(q, after))
        if verbose:
            print(f"move_q: code={code} maxerr={err:.4f} ({seconds:.1f}s)", flush=True)
        if code != 0 and err > 0.02 and retry > 0:
            return self.move_q(q, seconds=max(seconds, 2.0) * 1.5, verbose=verbose, retry=retry - 1)
        return code, err""")
s=s.replace("def move_q(self, q, seconds=None, verbose=True):","def move_q(self, q, seconds=None, verbose=True, retry=2):")
open('rob.py','w').write(s)
EOF
timeout 120 python3 -u -c "
from rob import *
r=Robot('chk')
print('q',np.round(r.arm_q(),3)); t,q=r.tcp(); print('tcp',t.round(4),np.round(q,3)); print('fingers',r.fingers())
"

# openrua op 32
timeout 120 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np, math
B=np.load('cloud_birdview.npy')
m=(B[:,0]>-0.30)&(B[:,0]<-0.13)&(B[:,1]>0.12)&(B[:,1]<0.27)&(B[:,2]>0.935)&(B[:,2]<0.99)
s=B[m]; xy=s[:,:2]-s[:,:2].mean(0)
u,sv,vt=np.linalg.svd(xy,full_matrices=False)
d=vt[0]
print('fin pts',len(s),'center',s[:,:2].mean(0).round(3),'zmax',s[:,2].max().round(3),'dir',d.round(3),'angle from x (deg)',math.degrees(math.atan2(d[1],d[0])))
m=(B[:,0]>-0.30)&(B[:,0]<-0.13)&(B[:,1]>0.12)&(B[:,1]<0.27)&(B[:,2]>0.905)&(B[:,2]<0.935)
s=B[m]; print('disc center',s[:,:2].mean(0).round(3), 'x',s[:,0].min().round(3),s[:,0].max().round(3),'y',s[:,1].min().round(3),s[:,1].max().round(3))
"

# openrua op 33
timeout 1500 python3 -u step4_knob.py 33 -0.4 2>&1 | tail -25

# openrua op 34
timeout 120 python3 cloud.py birdview >/dev/null && python3 -c "
import numpy as np, math
B=np.load('cloud_birdview.npy')
m=(B[:,0]>-0.30)&(B[:,0]<-0.13)&(B[:,1]>0.12)&(B[:,1]<0.27)&(B[:,2]>0.935)&(B[:,2]<0.99)
s=B[m]; xy=s[:,:2]-s[:,:2].mean(0)
u,sv,vt=np.linalg.svd(xy,full_matrices=False)
d=vt[0]
print('fin pts',len(s),'center',s[:,:2].mean(0).round(3),'zmax',s[:,2].max().round(3),'dir',d.round(3),'angle from x (deg)',math.degrees(math.atan2(d[1],d[0])))
"; python3 -c "
import cv2; im=cv2.imread('bird_knob4.png'); cv2.imwrite('crop_bird4.png', cv2.resize(im[200:300,330:420],None,fx=5,fy=5,interpolation=cv2.INTER_NEAREST))"

# openrua op 35
mkdir -p "$(dirname /workspace/step5_pick.py)"
cat > /workspace/step5_pick.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick the moka pot: hover, descend to the waist, close, lift."""
import sys, subprocess
from rob import *

POT = np.array([-0.042, -0.237])
GRASP_Z = 0.965
YAW = math.pi / 2  # fingers along world x
r = Robot("step5")
r.gripper(GRIP["open_m"])
r.move_tcp((POT[0], POT[1], 1.12), quat_down(YAW))
subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", "eih_pot_hover.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_pot_hover.png"])
if "--hover-only" in sys.argv:
    raise SystemExit(0)
r.move_tcp((POT[0], POT[1], 1.02), quat_down(YAW))
print("wrench pre-grasp", r.wrench()[0].round(2), flush=True)
r.move_tcp((POT[0], POT[1], GRASP_Z), quat_down(YAW))
print("wrench at grasp z", r.wrench()[0].round(2), flush=True)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_pot_low.png"])
f = r.gripper(GRIP["closed_m"])
print("grasp gap (per finger)", f, flush=True)
r.move_tcp((POT[0], POT[1], 1.15), quat_down(YAW))
print("wrench lifted", r.wrench()[0].round(2), "fingers", r.fingers(), flush=True)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_pot_lifted.png"])
OPENRUA_EOF

# openrua op 36
timeout 1500 python3 -u step5_pick.py --hover-only 2>&1 | tail -25

# openrua op 37
sed -i 's/seconds = max(1.5, min(8.0, d \/ 0.25))/seconds = max(1.5, min(14.0, d \/ 0.2))/' rob.py && timeout 1500 python3 -u step5_pick.py 2>&1 | tail -25

# openrua op 38
mkdir -p "$(dirname /workspace/step6_place.py)"
cat > /workspace/step6_place.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Carry the moka pot over the burner, lower it, release, retreat, snapshot."""
import sys, subprocess
from rob import *

STOVE = np.array([-0.057, 0.195])
YAW = math.pi / 2
BASE_BELOW_TCP = 0.136  # pot base sits this far below the TCP with the current grasp
BURNER_Z = 0.93
r = Robot("step6")
print("fingers", r.fingers(), "wrench", r.wrench()[0].round(2), flush=True)
# waypoint half way keeps the path away from the pan and the motion smooth
r.move_tcp((-0.06, -0.02, 1.15), quat_down(YAW))
print("fingers", r.fingers(), flush=True)
r.move_tcp((STOVE[0], STOVE[1], 1.15), quat_down(YAW))
print("fingers", r.fingers(), flush=True)
subprocess.run(["python3", "tools/perception/cam_snap.py", "robot0_eye_in_hand", "eih_stove_hover.png"])
if "--hover-only" in sys.argv:
    raise SystemExit(0)
r.move_tcp((STOVE[0], STOVE[1], BURNER_Z + BASE_BELOW_TCP + 0.01), quat_down(YAW))
print("wrench before release", r.wrench()[0].round(2), "fingers", r.fingers(), flush=True)
r.gripper(GRIP["open_m"])
r.move_tcp((STOVE[0], STOVE[1], 1.15), quat_down(YAW))
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_placed.png"])
HOME = [0.0, -0.161037389, 0.0, -2.44459747, 0.0, 2.2267522, 0.7853981633974483]
r.move_q(HOME)
subprocess.run(["python3", "tools/perception/cam_snap.py", "agentview", "agent_final.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "birdview", "bird_final.png"])
subprocess.run(["python3", "tools/perception/cam_snap.py", "sideview", "side_final.png"])
OPENRUA_EOF

# openrua op 39
timeout 1500 python3 -u step6_place.py --hover-only 2>&1 | tail -25

# openrua op 40
timeout 1500 python3 -u step6_place.py 2>&1 | tail -25

# openrua op 41
timeout 180 python3 cloud.py birdview sideview >/dev/null && python3 -c "
import numpy as np, math
P=np.concatenate([np.load(f'cloud_{c}.npy') for c in ['birdview','sideview']])
# moka pot now: expect near stove center (-0.057, 0.195)
m=(P[:,0]>-0.16)&(P[:,0]<0.05)&(P[:,1]>0.09)&(P[:,1]<0.30)&(P[:,2]>0.94)
Q=P[m]
print('pot pts',len(Q),'zmax',Q[:,2].max().round(3))
for z0 in [0.94,0.96,0.98,1.00,1.02,1.04,1.06]:
    s=Q[(Q[:,2]>=z0)&(Q[:,2]<z0+0.02)]
    if len(s): print(f'z {z0:.2f}: x {s[:,0].min():.3f}..{s[:,0].max():.3f} c={s[:,0].mean():.3f}  y {s[:,1].min():.3f}..{s[:,1].max():.3f} c={s[:,1].mean():.3f} n={len(s)}')
top=Q[Q[:,2]>Q[:,2].max()-0.015]; print('lid knob center',top[:,:2].mean(0).round(3),'z',top[:,2].max().round(3),'(stove burner center ~(-0.057,0.195), plate top 0.925)')
# knob fin
B=np.load('cloud_birdview.npy')
m=(B[:,0]>-0.30)&(B[:,0]<-0.13)&(B[:,1]>0.12)&(B[:,1]<0.27)&(B[:,2]>0.935)&(B[:,2]<0.99)
s=B[m]; xy=s[:,:2]-s[:,:2].mean(0); d=np.linalg.svd(xy,full_matrices=False)[2][0]
print('knob fin angle from x (deg)',round(math.degrees(math.atan2(d[1],d[0])),1),'(started at 0)')
"

# openrua op 42
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-stove-workstation-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-stove-workstation-quirks.md <<'OPENRUA_EOF'
---
name: panda-stove-workstation-quirks
description: Verified quirks of the /workspace Franka Panda sim workstation (IK frame, controller speed limits, knob/moka-pot geometry) learned while doing "turn on stove, put moka pot on it"
metadata:
  type: project
---

On the /workspace Panda robot (LIBERO-style kitchen scene, Sept 2026 session):
- `/compute_ik` and `/compute_fk` with empty frame_id work in WORLD coords here
  (base is at world (-0.66, 0, 0.912)); the generic doc's "base frame" warning did not apply.
- FollowJointTrajectory returns error -5 (tolerance) when asked for > ~0.3 rad/s;
  ~0.2 rad/s (and re-sending the same goal, slower) converges.
- Stove knob (fin along x at start) is turned by grasping the fin (TCP z≈0.945,
  fingers close to ≈0.012/side) and rotating joint7 negative = CCW from above;
  burner renders red once on (~0.5 rad; I left it at ~1.0 rad).
- Moka pot upper rim is ~0.08 m wide = gripper max opening, so a top-down body
  grasp jams on the rim; grasping the lid knob (fingers ≈0.006/side) lifted it fine.

**Why:** these took several failed attempts to discover and are not in the docs.
**How to apply:** reuse `/workspace/rob.py` helpers (auto-slow move_q with retry) and the step*_*.py scripts if the same scene appears again.
OPENRUA_EOF

# openrua op 43
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || true) && echo "- [Panda stove workstation quirks](panda-stove-workstation-quirks.md) — IK is world-frame, keep trajectories ≤0.2 rad/s, knob/moka-pot grasp geometry" >> MEMORY.md
