#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 2 -type f | head -100

# openrua op 1
cat README.md machine.yaml docs/10-machine.md docs/20-perception.md docs/30-action.md docs/40-patterns.md tools/README.md; ls -la .ros tools

# openrua op 2
ls -la tools/action tools/perception; cat tools/action/* tools/perception/*

# openrua op 3
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 4
timeout 30 ros2 topic list; echo ---; timeout 20 ros2 action list; echo ---; timeout 20 ros2 node list; echo ---; timeout 20 ros2 topic echo /joint_states --once

# openrua op 5
for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand; do timeout 60 python3 tools/perception/cam_snap.py $c; done

# openrua op 6
timeout 60 python3 tools/perception/px2world.py birdview 287 241; timeout 60 python3 tools/perception/px2world.py birdview 276 305; timeout 60 python3 tools/perception/px2world.py birdview 383 288; timeout 60 python3 tools/perception/px2world.py birdview 320 200

# openrua op 7
timeout 20 ros2 run tf2_ros tf2_echo world panda_hand 2>&1 | head -20; timeout 20 ros2 run tf2_ros tf2_echo world panda_link0 2>&1 | head -12; timeout 10 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|child" 

# openrua op 8
mkdir -p "$(dirname /workspace/grab.py)"
cat > /workspace/grab.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Grab color, depth, intrinsics and world<-optical TF for cameras; save npz.
Usage: python3 grab.py cam1 [cam2 ...]
"""
import sys
import numpy as np
import rclpy
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
import cv2


def grab(node, topic, msg_type, timeout=30.0):
    got = {}
    sub = node.create_subscription(msg_type, topic, lambda m: got.setdefault("m", m), 1)
    end = node.get_clock().now().nanoseconds / 1e9 + timeout
    import time
    t0 = time.time()
    while "m" not in got and time.time() - t0 < timeout:
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
    rclpy.init()
    node = rclpy.create_node("grab")
    tfbuf = Buffer()
    TransformListener(tfbuf, node)
    br = CvBridge()
    import time
    for cam in sys.argv[1:]:
        color = grab(node, f"/{cam}/color/image_raw", Image)
        depth = grab(node, f"/{cam}/depth/image_raw", Image)
        info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
        img = br.imgmsg_to_cv2(color, "bgr8")
        dep = br.imgmsg_to_cv2(depth, "passthrough").astype(np.float32)
        frame = f"{cam}_optical_frame"
        t0 = time.time()
        while time.time() - t0 < 10:
            rclpy.spin_once(node, timeout_sec=0.2)
            if tfbuf.can_transform("world", frame, rclpy.time.Time()):
                break
        t = tfbuf.lookup_transform("world", frame, rclpy.time.Time())
        q = t.transform.rotation
        T = np.eye(4)
        T[:3, :3] = quat_to_R(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.transform.translation.x, t.transform.translation.y, t.transform.translation.z]
        K = np.array(info.k).reshape(3, 3)
        np.savez(f"{cam}.npz", img=img, depth=dep, K=K, T=T)
        cv2.imwrite(f"{cam}.png", img)
        print(cam, img.shape, dep.shape, "T=", T[:3, 3])
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 9
timeout 120 python3 grab.py birdview agentview robot0_eye_in_hand

# openrua op 10
python3 -c "
import numpy as np, cv2
d=np.load('birdview.npz'); img,dep,K,T=d['img'],d['depth'],d['K'],d['T']
h,w=dep.shape
u,v=np.meshgrid(np.arange(w),np.arange(h))
z=dep
X=(u-K[0,2])*z/K[0,0]; Y=(v-K[1,2])*z/K[1,1]
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
Z=P[...,2]
print('K',K)
print('Z stats', np.nanmin(Z), np.nanmax(Z))
# table height histogram
hist,edges=np.histogram(Z[np.isfinite(Z)],bins=200,range=(0,1.5))
for hh,e in zip(hist,edges):
    if hh>500: print(f'{e:.3f} {hh}')
"


# openrua op 11
python3 -c "
import numpy as np, cv2
d=np.load('birdview.npz'); img,dep,K,T=d['img'],d['depth'],d['K'],d['T']
h,w=dep.shape
u,v=np.meshgrid(np.arange(w),np.arange(h))
z=dep
X=(u-K[0,2])*z/K[0,0]; Y=(v-K[1,2])*z/K[1,1]
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
Z=P[...,2]
mask=((Z>0.428)&(Z<0.75)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i
    pts=P[m]
    print(f'comp {i}: px={stats[i,4]} centroid_px=({cent[i][0]:.0f},{cent[i][1]:.0f}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={np.percentile(pts[:,2],95):.3f} meanBGR={img[m].mean(0).astype(int)}')
"


# openrua op 12
python3 -c "
import numpy as np, cv2
d=np.load('agentview.npz'); img,dep,K,T=d['img'],d['depth'],d['K'],d['T']
h,w=dep.shape
u,v=np.meshgrid(np.arange(w),np.arange(h))
z=dep
X=(u-K[0,2])*z/K[0,0]; Y=(v-K[1,2])*z/K[1,1]
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
Z=P[...,2]
print('T',T)
mask=((Z>0.428)&(Z<0.70)&(P[...,0]<0.4)&(P[...,0]>-0.4)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<30: continue
    m=lab==i
    pts=P[m]
    print(f'comp {i}: px={stats[i,4]} centroid_px=({cent[i][0]:.0f},{cent[i][1]:.0f}) world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] ztop={np.percentile(pts[:,2],95):.3f} meanBGR={img[m].mean(0).astype(int)}')
"


# openrua op 13
python3 -c "
import cv2
img=cv2.imread('agentview.png')
cv2.imwrite('crop_cans.png', cv2.resize(img[200:300,190:400],None,fx=3,fy=3,interpolation=cv2.INTER_CUBIC))
img2=cv2.imread('robot0_robotview.png')
cv2.imwrite('crop_rv.png', cv2.resize(img2[0:280,60:440],None,fx=2,fy=2,interpolation=cv2.INTER_CUBIC))
"

# openrua op 14
mkdir -p "$(dirname /workspace/arm.py)"
cat > /workspace/arm.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Arm helper: persistent clients for IK, trajectory, gripper, joint state.

World frame -> planning frame (panda_link0) offset comes from TF at start.
Usage as a library, or: python3 -u arm.py <script.py>  (exec's the file with
`A` bound to an Arm instance).
"""
import math
import sys
import time

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import GetPositionIK
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

M = yaml.safe_load(open("/workspace/machine.yaml"))
FJT = next(a for a in M["actuators"] if a["kind"] == "joint_trajectory")
GRIP = next(a for a in M["actuators"] if a["kind"] == "gripper")
TW = next(a for a in M["actuators"] if a["kind"] == "cartesian_twist")
JOINTS = FJT["joints"]
TCP = float(M["hand"]["tcp_offset_m"])
DOWN = (1.0, 0.0, 0.0, 0.0)  # hand z down, fingers open along world y


def quat_to_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def yaw_down(yaw):
    """Quaternion: hand pointing down, fingers opening along world direction
    rotated by `yaw` from world y (yaw=0 -> DOWN)."""
    # DOWN = rot_x(pi). Compose rot_z(yaw) * rot_x(pi)
    cz, sz = math.cos(yaw / 2), math.sin(yaw / 2)
    # q_z = (0,0,sz,cz), q_x = (1,0,0,0); product q_z*q_x:
    # (w1w2 - v1.v2, w1 v2 + w2 v1 + v1 x v2)
    w = cz * 0 - (0 * 1 + 0 * 0 + sz * 0)
    v = cz * np.array([1, 0, 0]) + 0 * np.array([0, 0, sz]) + np.cross([0, 0, sz], [1, 0, 0])
    return (v[0], v[1], v[2], w)


class Arm:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("arm_helper")
        self.js = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self.ik = self.node.create_client(GetPositionIK, M["planning"]["ik_service"])
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT["port"])
        self.grip = ActionClient(self.node, GripperCommand, GRIP["port"])
        self.twist_pub = self.node.create_publisher(TwistStamped, TW["port"], 10)
        self.tfbuf = Buffer()
        TransformListener(self.tfbuf, self.node)
        self.ik.wait_for_service(timeout_sec=20)
        self.fjt.wait_for_server(timeout_sec=20)
        self.grip.wait_for_server(timeout_sec=20)
        self.wait_js()
        # world -> planning frame offset
        t0 = time.time()
        while time.time() - t0 < 10 and not self.tfbuf.can_transform(
                "world", M["planning"]["planning_frame"], rclpy.time.Time()):
            rclpy.spin_once(self.node, timeout_sec=0.2)
        t = self.tfbuf.lookup_transform("world", M["planning"]["planning_frame"],
                                        rclpy.time.Time())
        self.base_off = np.array([t.transform.translation.x,
                                  t.transform.translation.y,
                                  t.transform.translation.z])
        self.log(f"planning frame offset in world: {self.base_off}")

    def log(self, *a):
        print(time.strftime("%H:%M:%S"), *a, flush=True)

    def _on_js(self, msg):
        self.js = dict(zip(msg.name, msg.position))

    def spin(self, t=0.05):
        rclpy.spin_once(self.node, timeout_sec=t)

    def wait_js(self):
        t0 = time.time()
        while not self.js and time.time() - t0 < 20:
            self.spin(0.2)
        return self.js

    def fresh_js(self):
        self.js = {}
        return self.wait_js()

    def arm_q(self):
        js = self.fresh_js()
        return [js[j] for j in JOINTS]

    def fingers(self):
        js = self.fresh_js()
        return js.get("panda_finger_joint1"), js.get("panda_finger_joint2")

    def hand_pose_world(self):
        """world -> panda_hand via TF (chain world->link0->...->hand)."""
        for _ in range(20):
            self.spin(0.1)
        try:
            t = self.tfbuf.lookup_transform("world", M["frames"]["hand"], rclpy.time.Time())
        except Exception as e:  # noqa
            self.log("TF hand lookup failed:", e)
            return None
        tr, q = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), (q.x, q.y, q.z, q.w)

    def tcp_world(self):
        hp = self.hand_pose_world()
        if hp is None:
            return None
        p, q = hp
        R = quat_to_R(*q)
        return p + TCP * R[:, 2]

    # ---- IK -----------------------------------------------------------
    def solve_ik(self, pos_world, quat=DOWN, at_tcp=True, seed=None, timeout=30.0):
        pos = np.array(pos_world, dtype=float)
        R = quat_to_R(*quat)
        if at_tcp:
            pos = pos - TCP * R[:, 2]
        pos_pf = pos - self.base_off
        req = GetPositionIK.Request()
        req.ik_request.group_name = M["planning"]["group"]
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, pos_pf)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout = Duration(sec=2)
        s = JointState()
        s.name = list(JOINTS)
        s.position = [float(v) for v in (seed if seed is not None else self.arm_q())]
        req.ik_request.robot_state.joint_state = s
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        res = fut.result()
        if res is None:
            self.log("IK: no answer")
            return None
        if res.error_code.val != 1:
            self.log(f"IK failed code={res.error_code.val} for world {pos_world} (pf {pos_pf})")
            return None
        sol = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        q = [sol[j] for j in JOINTS]
        return q

    def solve_ik_retry(self, pos_world, quat=DOWN, at_tcp=True, tries=6):
        cur = self.arm_q()
        q = self.solve_ik(pos_world, quat, at_tcp, seed=cur)
        if q is not None:
            return q
        seeds = [
            [0.0, -0.4, 0.0, -2.2, 0.0, 1.9, 0.785],
            [0.0, 0.0, 0.0, -1.8, 0.0, 1.9, 0.785],
            [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
            [0.0, 0.3, 0.0, -1.5, 0.0, 1.9, 0.785],
        ]
        for i, s in enumerate(seeds[:tries]):
            q = self.solve_ik(pos_world, quat, at_tcp, seed=s)
            if q is not None:
                return q
        return None

    # ---- motion -------------------------------------------------------
    def move_joints(self, q, seconds=3.0, tol=0.02, resend=2):
        for attempt in range(resend + 1):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = list(JOINTS)
            pt = JointTrajectoryPoint(positions=[float(v) for v in q])
            pt.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
            goal.trajectory.points = [pt]
            send = self.fjt.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
            gh = send.result()
            if gh is None or not gh.accepted:
                self.log("trajectory goal not accepted")
                continue
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, res, timeout_sec=600)
            r = res.result()
            code = r.result.error_code if r else None
            cur = self.arm_q()
            err = max(abs(a - b) for a, b in zip(cur, q))
            self.log(f"traj done code={code} max_joint_err={err:.4f}")
            if err < tol:
                return True
        return err < tol

    def move_to(self, pos_world, quat=DOWN, seconds=3.0, at_tcp=True):
        q = self.solve_ik_retry(pos_world, quat, at_tcp)
        if q is None:
            self.log(f"NO IK for {pos_world}")
            return False
        ok = self.move_joints(q, seconds)
        tcp = self.tcp_world()
        self.log(f"move_to {np.round(pos_world,3)} -> ok={ok} tcp_now={None if tcp is None else np.round(tcp,3)}")
        return ok

    def gripper(self, width, timeout=120):
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = float(GRIP.get("max_effort", 30.0))
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        gh = fut.result()
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=timeout)
        r = res.result()
        f = self.fingers()
        self.log(f"gripper({width}) reached={r.result.reached_goal if r else None} "
                 f"stalled={r.result.stalled if r else None} fingers={f}")
        return f

    def servo(self, vx=0.0, vy=0.0, vz=0.0, ticks=20, dt=0.05):
        msg = TwistStamped()
        msg.header.frame_id = TW["frame"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        for _ in range(ticks):
            msg.header.stamp = self.node.get_clock().now().to_msg()
            self.twist_pub.publish(msg)
            self.spin(dt)
        self.log(f"servo burst done tcp={np.round(self.tcp_world(),3)}")

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    A = Arm()
    try:
        if len(sys.argv) > 1:
            exec(open(sys.argv[1]).read(), {"A": A, "np": np, "DOWN": DOWN, "yaw_down": yaw_down})
        else:
            A.log("arm q:", np.round(A.arm_q(), 3))
            A.log("fingers:", A.fingers())
            A.log("hand world:", A.hand_pose_world())
            A.log("tcp world:", A.tcp_world())
    finally:
        A.shutdown()
OPENRUA_EOF

# openrua op 15
timeout 120 python3 -u arm.py 2>&1 | tail -20

# openrua op 16
cat > step1.py <<'EOF'
A.gripper(0.04)
ok = A.move_to([-0.185, -0.133, 0.60], DOWN, seconds=3.0)
A.log("STEP1 above soup ok=", ok)
EOF
timeout 900 python3 -u arm.py step1.py > step1.log 2>&1; cat step1.log

# openrua op 17
cat > iktest.py <<'EOF'
import math
hp = A.hand_pose_world()
p, q = hp
A.log("current hand", p, q)
A.log("ik at current hand pose:", A.solve_ik(p, q, at_tcp=False))
A.log("ik at current, DOWN quat:", A.solve_ik(p, DOWN, at_tcp=False))
# link8-style: rotate -45deg about z
s = math.sin(math.pi/8); c = math.cos(math.pi/8)
for yaw in [math.pi/4, -math.pi/4]:
    qq = yaw_down(yaw)
    A.log(f"yaw {yaw}: {qq} ->", A.solve_ik(p, qq, at_tcp=False))
A.log("ik above soup with current quat:", A.solve_ik([-0.185,-0.133,0.70], q, at_tcp=False))
EOF
timeout 300 python3 -u arm.py iktest.py 2>&1 | grep -v "^$"

# openrua op 18
timeout 120 ros2 param list /move_group 2>&1 | grep -iE "kinematics|ik|tip|panda_arm" | head -30; timeout 60 ros2 param get /move_group robot_description_semantic 2>&1 | grep -E "group|chain|tip|end_effector" | head -20

# openrua op 19
timeout 60 ros2 param get /move_group robot_description_semantic 2>&1 | grep -oE "openrua assembly note.{0,800}"; timeout 10 ros2 topic echo /tf_static --once 2>&1 | head -40

# openrua op 20
timeout 60 ros2 param get /move_group robot_description_semantic 2>&1 | grep -oE "openrua assembly note[^>]*"

# openrua op 21
timeout 60 ros2 param get /move_group robot_description_semantic 2>&1 | tr '\n' ' ' | grep -oE "openrua assembly note.{0,1500}"

# openrua op 22
python3 - <<'EOF'
import re
s=open('arm.py').read()
s=s.replace('''        req.ik_request.pose_stamped.header.frame_id = ""
''','''        req.ik_request.pose_stamped.header.frame_id = ""
        req.ik_request.ik_link_name = self.ik_link
''')
s=s.replace('''        self.log(f"planning frame offset in world: {self.base_off}")
''','''        self.log(f"planning frame offset in world: {self.base_off}")
        self.ik_link = "panda_hand"
''')
open('arm.py','w').write(s)
EOF
cat > iktest.py <<'EOF'
hp = A.hand_pose_world()
p, q = hp
A.log("current hand", p, q)
A.log("ik at current hand pose (link=panda_hand):", A.solve_ik(p, q, at_tcp=False))
A.log("ik above soup DOWN:", A.solve_ik([-0.185,-0.133,0.70], DOWN, at_tcp=False))
EOF
timeout 300 python3 -u arm.py iktest.py 2>&1 | grep -v "^$"

# openrua op 23
timeout 300 python3 tools/action/ik_move.py 0.457 0.0 0.3576 0.9996 0 -0.0284 0 2 2>&1 | tail -5

# openrua op 24
for p in robot_description_kinematics.panda_arm.kinematics_solver robot_description_kinematics.panda_arm.joints robot_description_kinematics.panda_arm.kinematics_solver_timeout robot_description_kinematics.panda_arm.position_only_ik robot_description_kinematics.panda_arm.epsilon robot_description_kinematics.panda_arm.max_solver_iterations robot_description_kinematics.panda_arm.orientation_vs_position; do echo -n "$p = "; timeout 30 ros2 param get /move_group $p 2>&1 | tail -1; done
timeout 30 ros2 param get /move_group robot_description 2>&1 | tr '\n' ' ' | grep -oE '<joint name="panda_joint[0-9]" type="revolute">.{0,600}' | grep -oE 'name="panda_joint[0-9]"|<limit[^/]*/>'

# openrua op 25
timeout 30 ros2 service list | grep -iE "fk|ik|state"; cat > fktest.py <<'EOF'
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState
import rclpy
cli = A.node.create_client(GetPositionFK, "/compute_fk")
cli.wait_for_service(timeout_sec=10)
req = GetPositionFK.Request()
req.header.frame_id = ""
req.fk_link_names = ["panda_link8", "panda_hand"]
s = JointState(); s.name = list(A.arm_q() and __import__('arm').JOINTS); s.position = [float(v) for v in A.arm_q()]
req.robot_state.joint_state = s
fut = cli.call_async(req)
rclpy.spin_until_future_complete(A.node, fut, timeout_sec=30)
r = fut.result()
A.log("FK code", r.error_code.val)
for n, ps in zip(r.fk_link_names, r.pose_stamped):
    p, q = ps.pose.position, ps.pose.orientation
    A.log(n, ps.header.frame_id, (round(p.x,4), round(p.y,4), round(p.z,4)), (round(q.x,4), round(q.y,4), round(q.z,4), round(q.w,4)))
    # feed back into IK, with frame_id empty, ik_link = n
    A.ik_link = n
    pw = np.array([p.x, p.y, p.z]) + A.base_off
    A.log(" IK feedback:", A.solve_ik(pw, (q.x,q.y,q.z,q.w), at_tcp=False))
EOF
timeout 300 python3 -u arm.py fktest.py 2>&1 | grep -v "^$"

# openrua op 26
python3 - <<'EOF'
s=open('arm.py').read()
s=s.replace('''        self.log(f"planning frame offset in world: {self.base_off}")
        self.ik_link = "panda_hand"
''','''        self.log(f"TF world->planning frame: {self.base_off}")
        # empirically (FK/IK round-trip) MoveIt's model frame here IS world:
        # poses with empty frame_id are taken in world coordinates
        self.base_off = np.zeros(3)
        self.ik_link = "panda_hand"
''')
open('arm.py','w').write(s)
EOF
timeout 900 python3 -u arm.py step1.py > step1.log 2>&1; cat step1.log

# openrua op 27
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih1.png

# openrua op 28
cat > step2.py <<'EOF'
ok = A.move_to([-0.185, -0.133, 0.50], DOWN, seconds=2.0)
ok = A.move_to([-0.185, -0.133, 0.45], DOWN, seconds=1.5)
A.log("STEP2 at grasp height ok=", ok)
f = A.gripper(0.0)
A.log("fingers after close:", f)
ok = A.move_to([-0.185, -0.133, 0.65], DOWN, seconds=2.0)
A.log("lifted ok=", ok, "fingers:", A.fingers())
EOF
timeout 1200 python3 -u arm.py step2.py > step2.log 2>&1; cat step2.log

# openrua op 29
timeout 60 python3 tools/perception/cam_snap.py agentview av2.png; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih2.png

# openrua op 30
cat > step3.py <<'EOF'
ok = A.move_to([-0.09, 0.06, 0.72], DOWN, seconds=2.5)
A.log("waypoint ok=", ok, "fingers:", A.fingers())
ok = A.move_to([-0.02, 0.25, 0.74], DOWN, seconds=2.5)
A.log("STEP3 above basket ok=", ok, "fingers:", A.fingers())
EOF
timeout 1200 python3 -u arm.py step3.py > step3.log 2>&1; cat step3.log

# openrua op 31
timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih3.png

# openrua op 32
cat > step4.py <<'EOF'
ok = A.move_to([-0.01, 0.25, 0.70], DOWN, seconds=1.5)
A.log("lowered ok=", ok)
f = A.gripper(0.04)
A.log("released, fingers:", f)
ok = A.move_to([-0.01, 0.25, 0.78], DOWN, seconds=1.5)
A.log("STEP4 retreat ok=", ok)
EOF
timeout 1200 python3 -u arm.py step4.py > step4.log 2>&1; cat step4.log; timeout 60 python3 tools/perception/cam_snap.py agentview av4.png

# openrua op 33
timeout 120 python3 grab.py birdview >/dev/null && python3 -c "
import numpy as np, cv2
d=np.load('birdview.npz'); img,dep,K,T=d['img'],d['depth'],d['K'],d['T']
h,w=dep.shape
u,v=np.meshgrid(np.arange(w),np.arange(h))
z=dep
X=(u-K[0,2])*z/K[0,0]; Y=(v-K[1,2])*z/K[1,1]
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
Z=P[...,2]
mask=((Z>0.428)&(Z<0.60)&(P[...,1]<-0.1)).astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    m=lab==i; pts=P[m]
    print(f'comp {i}: px={stats[i,4]} world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] center=({pts[:,0].mean():.3f},{pts[:,1].mean():.3f}) ztop={np.percentile(pts[:,2],95):.3f}')
    # principal axis
    xy=pts[:,:2]-pts[:,:2].mean(0); ev,evec=np.linalg.eigh(np.cov(xy.T)); print('   long axis dir', evec[:,1], 'angle deg', np.degrees(np.arctan2(evec[1,1],evec[0,1])))
"

# openrua op 34
cat > step5.py <<'EOF'
ok = A.move_to([0.04, 0.05, 0.72], DOWN, seconds=2.5)
A.log("waypoint ok=", ok)
ok = A.move_to([0.090, -0.190, 0.60], DOWN, seconds=2.5)
A.log("STEP5 above cream cheese ok=", ok)
EOF
timeout 1200 python3 -u arm.py step5.py > step5.log 2>&1; cat step5.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih5.png

# openrua op 35
cat > step6.py <<'EOF'
ok = A.move_to([0.090, -0.190, 0.50], DOWN, seconds=2.0)
ok = A.move_to([0.090, -0.190, 0.437], DOWN, seconds=2.0)
A.log("STEP6 at grasp height ok=", ok)
f = A.gripper(0.0)
A.log("fingers after close:", f)
ok = A.move_to([0.090, -0.190, 0.60], DOWN, seconds=2.0)
A.log("lifted ok=", ok, "fingers:", A.fingers())
EOF
timeout 1200 python3 -u arm.py step6.py > step6.log 2>&1; cat step6.log; timeout 60 python3 tools/perception/cam_snap.py agentview av6.png

# openrua op 36
cat > step7.py <<'EOF'
ok = A.move_to([0.06, 0.03, 0.72], DOWN, seconds=2.5)
A.log("waypoint ok=", ok, "fingers:", A.fingers())
ok = A.move_to([0.03, 0.26, 0.72], DOWN, seconds=2.5)
A.log("above basket ok=", ok, "fingers:", A.fingers())
EOF
timeout 1200 python3 -u arm.py step7.py > step7.log 2>&1; cat step7.log; timeout 60 python3 tools/perception/cam_snap.py robot0_eye_in_hand eih7.png

# openrua op 37
cat > step8.py <<'EOF'
ok = A.move_to([0.03, 0.26, 0.68], DOWN, seconds=1.5)
A.log("lowered ok=", ok)
f = A.gripper(0.04)
A.log("released, fingers:", f)
ok = A.move_to([0.0, 0.20, 0.80], DOWN, seconds=2.0)
A.log("STEP8 retreat ok=", ok)
EOF
timeout 1200 python3 -u arm.py step8.py > step8.log 2>&1; cat step8.log; timeout 120 python3 grab.py agentview birdview robot0_eye_in_hand >/dev/null; echo grabbed

# openrua op 38
python3 -c "
import numpy as np, cv2
d=np.load('birdview.npz'); img,dep,K,T=d['img'],d['depth'],d['K'],d['T']
h,w=dep.shape
u,v=np.meshgrid(np.arange(w),np.arange(h))
z=dep
X=(u-K[0,2])*z/K[0,0]; Y=(v-K[1,2])*z/K[1,1]
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
Z=P[...,2]
# anything left at original spots?
for name,(cx,cy) in {'soup_orig':(-0.185,-0.133),'cheese_orig':(0.09,-0.19)}.items():
    m=(np.abs(P[...,0]-cx)<0.05)&(np.abs(P[...,1]-cy)<0.05)&np.isfinite(Z)
    print(name,'max height within 5cm of original spot:', round(Z[m].max(),3), '(table=0.420)')
# inside basket footprint: x[-0.077,0.080] y[0.168,0.336]
m=(P[...,0]>-0.06)&(P[...,0]<0.065)&(P[...,1]>0.185)&(P[...,1]<0.32)&np.isfinite(Z)&(Z<0.60)
pts=P[m]
print('points inside basket footprint below rim:', m.sum())
mask=m.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
for i in range(1,n):
    if stats[i,4]<15: continue
    mm=lab==i; p=P[mm]
    print(f'  obj: center=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={np.percentile(p[:,2],95):.3f} meanBGR={img[mm].mean(0).astype(int)}')
cv2.imwrite('bird_crop.png', cv2.resize(img[250:330,340:430],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"

# openrua op 39
cat > step9.py <<'EOF'
ok = A.move_to([-0.15, -0.05, 0.75], DOWN, seconds=2.5)
A.log("parked ok=", ok)
EOF
timeout 1200 python3 -u arm.py step9.py 2>&1 | tail -2; timeout 120 python3 grab.py birdview agentview >/dev/null; python3 -c "
import numpy as np, cv2
d=np.load('birdview.npz'); img,dep,K,T=d['img'],d['depth'],d['K'],d['T']
h,w=dep.shape
u,v=np.meshgrid(np.arange(w),np.arange(h))
z=dep
X=(u-K[0,2])*z/K[0,0]; Y=(v-K[1,2])*z/K[1,1]
P=np.stack([X,Y,z,np.ones_like(z)],-1)@T.T
Z=P[...,2]
m=(P[...,0]>-0.06)&(P[...,0]<0.065)&(P[...,1]>0.185)&(P[...,1]<0.32)&np.isfinite(Z)&(Z<0.60)&(Z>0.43)
mask=m.astype(np.uint8)
n,lab,stats,cent=cv2.connectedComponentsWithStats(mask)
print('objects inside basket footprint (below rim 0.625, above floor):')
for i in range(1,n):
    if stats[i,4]<15: continue
    mm=lab==i; p=P[mm]
    print(f'  center=({p[:,0].mean():.3f},{p[:,1].mean():.3f}) x[{p[:,0].min():.3f},{p[:,0].max():.3f}] y[{p[:,1].min():.3f},{p[:,1].max():.3f}] ztop={np.percentile(p[:,2],95):.3f} meanBGR={img[mm].mean(0).astype(int)}')
cv2.imwrite('bird_crop.png', cv2.resize(img[250:330,340:430],None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))
"
