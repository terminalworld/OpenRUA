#!/usr/bin/env bash
# auto-extracted from transcript.jsonl (shell + write/edit condensate)

# openrua op 0
ls -la /workspace && find /workspace -maxdepth 3 -type f | head -100

# openrua op 1
cat tools/perception/cam_snap.py tools/perception/px2world.py

# openrua op 2
cat tools/action/fjt_send.py tools/action/gripper_cmd.py tools/action/ik_move.py

# openrua op 3
ros2 topic list && echo ---ACTIONS--- && ros2 action list && echo ---SERVICES--- && ros2 service list | head -50 && echo ---NODES--- && ros2 node list

# openrua op 4
mkdir -p snaps && for c in agentview birdview frontview sideview robot0_robotview robot0_eye_in_hand galleryview paperview; do timeout 60 python3 tools/perception/cam_snap.py $c snaps/$c.png; done; ros2 topic echo /joint_states --once

# openrua op 5
timeout 20 ros2 topic echo /tf_static --once 2>&1 | grep -E "frame_id|translation|rotation|x:|y:|z:|w:" | head -120; echo ---; timeout 10 ros2 topic echo /tf --once 2>&1 | grep -E "frame_id" | head

# openrua op 6
timeout 20 ros2 topic echo /tf --once 2>&1 | grep -vE "stamp|sec|nanosec|^  header|^-|^transforms" 

# openrua op 7
mkdir -p "$(dirname /workspace/px_batch.py)"
cat > /workspace/px_batch.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Batch pixel -> world for one camera. Usage: px_batch.py <cam> u,v [u,v ...]
Also saves <cam>_depth.npy and prints the camera pose."""
import struct, sys
import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def grab(node, topic, T, timeout=30.0):
    got = {}
    sub = node.create_subscription(T, topic, lambda m: got.setdefault("m", m), 1)
    import time
    end = time.time() + timeout
    while "m" not in got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return got.get("m")


def quat_R(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    cam = sys.argv[1]
    pix = [tuple(int(a) for a in p.split(",")) for p in sys.argv[2:]]
    rclpy.init()
    node = rclpy.create_node("px_batch")
    buf = Buffer(); TransformListener(buf, node)
    depth = grab(node, f"/{cam}/depth/image_raw", Image)
    info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
    D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
    np.save(f"snaps/{cam}_depth.npy", D)
    import time
    end = time.time() + 20
    frame = f"{cam}_optical_frame"
    while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
        rclpy.spin_once(node, timeout_sec=0.2)
    t = buf.lookup_transform("world", frame, rclpy.time.Time())
    q = t.transform.rotation
    R = quat_R(q.x, q.y, q.z, q.w)
    o = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    print(f"cam {cam} origin {o} fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} size {depth.width}x{depth.height}")
    for (u, v) in pix:
        z = float(D[v, u])
        p = R @ np.array([(u - cx) * z / fx, (v - cy) * z / fy, z]) + o
        print(f"px ({u},{v}) depth {z:.3f} -> world {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 8
timeout 120 python3 px_batch.py agentview 243,225 243,245 343,258 343,250 530,250 460,200 600,200 530,190 300,400 100,300

# openrua op 9
mkdir -p "$(dirname /workspace/heightmap.py)"
cat > /workspace/heightmap.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""World-frame height map from one camera's depth; list blobs above the table."""
import sys, time
import numpy as np
import rclpy
import cv2
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from px_batch import grab, quat_R

cam = sys.argv[1]
zmin = float(sys.argv[2]) if len(sys.argv) > 2 else 0.435
zmax = float(sys.argv[3]) if len(sys.argv) > 3 else 0.7
rclpy.init()
node = rclpy.create_node("hm")
buf = Buffer(); TransformListener(buf, node)
depth = grab(node, f"/{cam}/depth/image_raw", Image)
info = grab(node, f"/{cam}/color/camera_info", CameraInfo)
color = grab(node, f"/{cam}/color/image_raw", Image)
C = np.frombuffer(color.data, dtype=np.uint8).reshape(color.height, color.width, -1)
D = np.frombuffer(depth.data, dtype=np.float32).reshape(depth.height, depth.width)
frame = f"{cam}_optical_frame"
end = time.time() + 20
while time.time() < end and not buf.can_transform("world", frame, rclpy.time.Time()):
    rclpy.spin_once(node, timeout_sec=0.2)
t = buf.lookup_transform("world", frame, rclpy.time.Time())
q = t.transform.rotation
R = quat_R(q.x, q.y, q.z, q.w)
o = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
vs, us = np.mgrid[0:depth.height, 0:depth.width]
P = np.stack([(us - cx) * D / fx, (vs - cy) * D / fy, D], -1) @ R.T + o
np.save(f"snaps/{cam}_world.npy", P)
Z = P[..., 2]
mask = ((Z > zmin) & (Z < zmax) & np.isfinite(Z)).astype(np.uint8)
n, lab, stats, cent = cv2.connectedComponentsWithStats(mask)
for i in range(1, n):
    if stats[i, cv2.CC_STAT_AREA] < 15:
        continue
    m = lab == i
    pts = P[m]
    col = C[m].mean(0)
    print(f"blob {i}: px centroid ({cent[i][0]:.0f},{cent[i][1]:.0f}) area {stats[i,4]} "
          f"world x[{pts[:,0].min():.3f},{pts[:,0].max():.3f}] y[{pts[:,1].min():.3f},{pts[:,1].max():.3f}] "
          f"z[{pts[:,2].min():.3f},{pts[:,2].max():.3f}] mean xyz ({pts[:,0].mean():.3f},{pts[:,1].mean():.3f},{pts[:,2].mean():.3f}) color {col.astype(int)}")
cv2.imwrite(f"snaps/{cam}_mask.png", mask * 255)
rclpy.shutdown()
OPENRUA_EOF

# openrua op 10
timeout 120 python3 heightmap.py birdview 0.435 0.75

# openrua op 11
timeout 120 python3 heightmap.py agentview 0.44 0.75; echo ---; timeout 120 python3 heightmap.py robot0_robotview 0.44 0.75

# openrua op 12
mkdir -p "$(dirname /workspace/fk.py)"
cat > /workspace/fk.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Print hand pose via /compute_fk (arm joints only) plus finger positions."""
import time
import rclpy
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState

ARM = [f"panda_joint{i}" for i in range(1, 8)]
rclpy.init()
node = rclpy.create_node("fk")
js = {}
node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
end = time.time() + 15
while "m" not in js and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
m = js["m"]
d = dict(zip(m.name, m.position))
print("joints:", ", ".join(f"{d[j]:.4f}" for j in ARM))
print("fingers:", d.get("panda_finger_joint1"), d.get("panda_finger_joint2"))
cli = node.create_client(GetPositionFK, "/compute_fk")
cli.wait_for_service(10)
req = GetPositionFK.Request()
req.header.frame_id = ""
req.fk_link_names = ["panda_hand"]
req.robot_state.joint_state.name = ARM
req.robot_state.joint_state.position = [d[j] for j in ARM]
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=30)
r = fut.result()
if r is None:
    print("FK no answer")
else:
    print("fk error", r.error_code.val)
    for ps in r.pose_stamped:
        p, q = ps.pose.position, ps.pose.orientation
        print(f"frame '{ps.header.frame_id}' pos ({p.x:.4f},{p.y:.4f},{p.z:.4f}) quat ({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})")
rclpy.shutdown()
OPENRUA_EOF

# openrua op 13
timeout 90 python3 fk.py

# openrua op 14
timeout 60 ros2 topic echo /robot_description --once --qos-durability transient_local --qos-reliability reliable 2>/dev/null | grep -oE '<link name="[a-z_0-9]+"|<joint name="[a-z_0-9]+" type="[a-z]+"|<parent link="[a-z_0-9]+"|<child link="[a-z_0-9]+"|<origin[^>]*>' | head -30

# openrua op 15
timeout 60 ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 1}}" 2>&1 | grep -E "frame_id|name:" | head -20; echo ---; timeout 30 ros2 param get /move_group robot_description 2>&1 | grep -oE '<link name="[a-z_0-9]+"|<joint name="[a-z_0-9]+" type="[a-z]+"|<parent link="[a-z_0-9]+"|<child link="[a-z_0-9]+"|<origin[^>]*>' | head -12

# openrua op 16
python3 -c "
import numpy as np
def dh(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
q=[0.0,-0.1610,0.0,-2.4446,0.0,2.2268,0.7854]
P=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
T=np.eye(4)
for (a,d,al),th in zip(P,q): T=T@dh(a,d,al,th)
T=T@dh(0,0.107,0,0)  # flange
print('flange in base:',T[:3,3].round(4)); print('R:\n',T[:3,:3].round(3))
"


# openrua op 17
mkdir -p "$(dirname /workspace/ik.py)"
cat > /workspace/ik.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""IK query only (no motion). Usage: ik.py x y z qx qy qz qw  -> prints joints."""
import sys, time
import rclpy
from moveit_msgs.srv import GetPositionIK
from sensor_msgs.msg import JointState

ARM = [f"panda_joint{i}" for i in range(1, 8)]


def solve(node, cli, seed, pose, timeout=60):
    req = GetPositionIK.Request()
    req.ik_request.group_name = "panda_arm"
    req.ik_request.pose_stamped.header.frame_id = ""
    p = req.ik_request.pose_stamped.pose
    p.position.x, p.position.y, p.position.z = pose[:3]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = pose[3:]
    req.ik_request.robot_state.joint_state.name = ARM
    req.ik_request.robot_state.joint_state.position = list(seed)
    req.ik_request.avoid_collisions = False
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    r = fut.result()
    if r is None or r.error_code.val != 1:
        return None, (r.error_code.val if r else None)
    d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return [d[j] for j in ARM], 1


if __name__ == "__main__":
    pose = [float(a) for a in sys.argv[1:8]]
    rclpy.init()
    node = rclpy.create_node("ikq")
    js = {}
    node.create_subscription(JointState, "/joint_states", lambda m: js.setdefault("m", m), 1)
    end = time.time() + 15
    while "m" not in js and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    d = dict(zip(js["m"].name, js["m"].position))
    seed = [d[j] for j in ARM]
    cli = node.create_client(GetPositionIK, "/compute_ik")
    cli.wait_for_service(10)
    sol, code = solve(node, cli, seed, pose)
    print("seed:", ", ".join(f"{v:.4f}" for v in seed))
    print("code:", code, "sol:", None if sol is None else ", ".join(f"{v:.4f}" for v in sol))
    rclpy.shutdown()
OPENRUA_EOF

# openrua op 18
echo "== world-frame round trip =="; timeout 90 python3 ik.py -0.0530 0 0.7776 0.9996 0 -0.0284 0; echo "== base-frame coords =="; timeout 90 python3 ik.py 0.457 0 0.3576 0.9996 0 -0.0284 0

# openrua op 19
python3 -c "
import numpy as np
def dh(a,d,al,th):
    ca,sa,ct,st=np.cos(al),np.sin(al),np.cos(th),np.sin(th)
    return np.array([[ct,-st,0,a],[st*ca,ct*ca,-sa,-d*sa],[st*sa,ct*sa,ca,d*ca],[0,0,0,1]])
P=[(0,0.333,0),(0,0,-np.pi/2),(0,0.316,np.pi/2),(0.0825,0,np.pi/2),(-0.0825,0.384,-np.pi/2),(0,0,np.pi/2),(0.088,0,np.pi/2)]
def fk(q):
    T=np.eye(4)
    for (a,d,al),th in zip(P,q): T=T@dh(a,d,al,th)
    T=T@dh(0,0.107,0,0)@dh(0,0,0,-np.pi/4)  # flange then hand (-45deg about z)
    return T
for q in [[0,-0.1610,0,-2.4446,0,2.2268,0.7854],[0.0585,-0.1613,-0.0579,-2.4447,-0.0116,2.2268,0.0089]]:
    T=fk(q); print('hand pos world', (T[:3,3]+[-0.51,0,0.42]).round(4)); print(T[:3,:3].round(3))
"


# openrua op 20
sed -i 's/    req.ik_request.group_name = "panda_arm"/    req.ik_request.group_name = "panda_arm"\n    req.ik_request.ik_link_name = "panda_hand"/' ik.py && timeout 90 python3 ik.py -0.0530 0 0.7776 0.9996 0 -0.0284 0

# openrua op 21
python3 -c "
import numpy as np
P=np.load('snaps/agentview_world.npy')
Z=P[...,2]
# ketchup region: x in [-0.26,-0.18], y in [-0.18,-0.07]
m=(P[...,0]>-0.26)&(P[...,0]<-0.18)&(P[...,1]>-0.18)&(P[...,1]<-0.07)&(Z>0.435)&np.isfinite(Z)
pts=P[m]
for lo in np.arange(0.43,0.57,0.01):
    s=pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.01)]
    if len(s): print(f'z {lo:.2f}-{lo+0.01:.2f}: n={len(s):4d} y[{s[:,1].min():.3f},{s[:,1].max():.3f}] w={s[:,1].max()-s[:,1].min():.3f} xmin={s[:,0].min():.3f}')
print('--- box')
m=(P[...,0]>-0.24)&(P[...,0]<-0.10)&(P[...,1]>0.0)&(P[...,1]<0.08)&(Z>0.435)&np.isfinite(Z)
pts=P[m]
for lo in np.arange(0.43,0.47,0.005):
    s=pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.005)]
    if len(s): print(f'z {lo:.3f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
"


# openrua op 22
mkdir -p "$(dirname /workspace/ctl.py)"
cat > /workspace/ctl.py <<'OPENRUA_EOF'
#!/usr/bin/env python3
"""Pick/place controller for this Panda.

  ctl.py state                         joints, fingers, hand pose (FK), wrench
  ctl.py move X Y Z MODE [secs]        TCP (fingertip midpoint) to world XYZ,
                                       hand pointing down; MODE y = fingers
                                       close along world y, x = along world x
  ctl.py grip open|close
  ctl.py joints p1,...,p7 [secs]       raw joint target
All poses are WORLD frame (verified: /compute_ik & /compute_fk work in world).
"""
import sys, time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import WrenchStamped
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation as Rot

ARM = [f"panda_joint{i}" for i in range(1, 8)]
TCP = 0.1034
FJT = "/panda_arm_controller/follow_joint_trajectory"
GRIP = "/franka_gripper/gripper_action"
# hand pointing down; hand x -> world x (fingers open along world y)
Q_Y = Rot.from_quat([1, 0, 0, 0])
# same, yawed 90 deg: fingers open along world x
Q_X = Rot.from_euler("z", 90, degrees=True) * Q_Y


class Ctl:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("ctl")
        self.js = None
        self.wr = None
        self.node.create_subscription(JointState, "/joint_states", self._js, 1)
        self.node.create_subscription(WrenchStamped,
                                      "/franka_robot_state_broadcaster/external_wrench",
                                      self._wr, 1)
        self.ik = self.node.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.node.create_client(GetPositionFK, "/compute_fk")
        self.fjt = ActionClient(self.node, FollowJointTrajectory, FJT)
        self.grip = ActionClient(self.node, GripperCommand, GRIP)

    def _js(self, m):
        self.js = m

    def _wr(self, m):
        self.wr = m

    def spin(self, t):
        end = time.time() + t
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def joints(self, fresh=True):
        if fresh:
            self.js = None
        end = time.time() + 15
        while self.js is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        d = dict(zip(self.js.name, self.js.position))
        return [d[j] for j in ARM], (d["panda_finger_joint1"], d["panda_finger_joint2"])

    def hand_pose(self, q):
        self.fk.wait_for_service(10)
        req = GetPositionFK.Request()
        req.fk_link_names = ["panda_hand"]
        req.robot_state.joint_state.name = ARM
        req.robot_state.joint_state.position = list(q)
        fut = self.fk.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=30)
        ps = fut.result().pose_stamped[0].pose
        p = np.array([ps.position.x, ps.position.y, ps.position.z])
        R = Rot.from_quat([ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w])
        return p, R

    def solve_ik(self, hand_p, R, seed):
        self.ik.wait_for_service(10)
        req = GetPositionIK.Request()
        req.ik_request.group_name = "panda_arm"
        req.ik_request.ik_link_name = "panda_hand"
        req.ik_request.pose_stamped.header.frame_id = ""
        p = req.ik_request.pose_stamped.pose
        p.position.x, p.position.y, p.position.z = map(float, hand_p)
        qx, qy, qz, qw = R.as_quat()
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
        req.ik_request.robot_state.joint_state.name = ARM
        req.ik_request.robot_state.joint_state.position = list(seed)
        req.ik_request.avoid_collisions = False
        req.ik_request.timeout.sec = 2
        fut = self.ik.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=60)
        r = fut.result()
        if r is None:
            return None, "timeout"
        if r.error_code.val != 1:
            return None, r.error_code.val
        d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
        return [d[j] for j in ARM], 1

    def run_traj(self, target, secs):
        self.fjt.wait_for_server(10)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM
        pt = JointTrajectoryPoint(positions=[float(v) for v in target])
        pt.time_from_start = Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9))
        goal.trajectory.points = [pt]
        t0 = time.time()
        send = self.fjt.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        gh = send.result()
        if gh is None or not gh.accepted:
            print("FJT goal rejected"); return None
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=900)
        if res.result() is None:
            print("FJT result timeout (client side)")
            code = None
        else:
            code = res.result().result.error_code
        q, _ = self.joints()
        err = np.abs(np.array(q) - np.array(target))
        print(f"traj done in {time.time()-t0:.1f}s wall, error_code={code}, "
              f"max joint err={err.max():.4f} rad")
        return code

    def move_tcp(self, tcp, mode, secs=3.0):
        R = Q_X if mode == "x" else Q_Y
        hand_p = np.array(tcp, float) - TCP * R.apply([0, 0, 1])
        seed, _ = self.joints()
        sol, code = self.solve_ik(hand_p, R, seed)
        if sol is None:
            print(f"IK FAILED ({code}) for tcp={tcp} mode={mode}; no motion")
            return False
        dq = np.abs(np.array(sol) - np.array(seed))
        print("IK ok; joint delta max %.3f; target %s" % (dq.max(), ", ".join(f"{v:.3f}" for v in sol)))
        rc = self.run_traj(sol, secs)
        q, _ = self.joints()
        p, Ract = self.hand_pose(q)
        tcp_act = p + TCP * Ract.apply([0, 0, 1])
        print(f"TCP now {tcp_act.round(4)} (wanted {np.array(tcp).round(4)}), "
              f"pos err {np.linalg.norm(tcp_act-np.array(tcp))*1000:.1f} mm")
        return rc == 0

    def gripper(self, width):
        self.grip.wait_for_server(10)
        g = GripperCommand.Goal()
        g.command.position = float(width)
        g.command.max_effort = 30.0
        send = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=60)
        res = send.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res, timeout_sec=300)
        r = res.result().result if res.result() else None
        _, f = self.joints()
        print(f"gripper -> {width}: reached={getattr(r,'reached_goal',None)} "
              f"stalled={getattr(r,'stalled',None)} fingers={f[0]:.4f},{f[1]:.4f}")
        return f

    def state(self):
        q, f = self.joints()
        p, R = self.hand_pose(q)
        tcp = p + TCP * R.apply([0, 0, 1])
        print("joints:", ", ".join(f"{v:.4f}" for v in q))
        print(f"fingers: {f[0]:.4f} {f[1]:.4f}")
        print(f"hand: {p.round(4)} quat {R.as_quat().round(4)}  TCP: {tcp.round(4)}")
        self.spin(0.5)
        if self.wr:
            w = self.wr.wrench
            print(f"wrench F=({w.force.x:.2f},{w.force.y:.2f},{w.force.z:.2f}) "
                  f"T=({w.torque.x:.2f},{w.torque.y:.2f},{w.torque.z:.2f})")


def main():
    a = sys.argv[1:]
    if not a:
        raise SystemExit(__doc__)
    c = Ctl()
    if a[0] == "state":
        c.state()
    elif a[0] == "move":
        x, y, z = map(float, a[1:4]); mode = a[4]
        secs = float(a[5]) if len(a) > 5 else 3.0
        ok = c.move_tcp((x, y, z), mode, secs)
        print("MOVE", "OK" if ok else "PROBLEM")
    elif a[0] == "grip":
        c.gripper(0.04 if a[1] == "open" else 0.0)
    elif a[0] == "joints":
        tgt = [float(v) for v in a[1].split(",")]
        secs = float(a[2]) if len(a) > 2 else 3.0
        c.run_traj(tgt, secs)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
OPENRUA_EOF

# openrua op 23
timeout 120 python3 ctl.py state

# openrua op 24
timeout 300 python3 -u ctl.py grip open && timeout 600 python3 -u ctl.py move -0.215 -0.127 0.70 y 3

# openrua op 25
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih1.png

# openrua op 26
timeout 600 python3 -u ctl.py move -0.215 -0.127 0.535 y 3 && timeout 120 python3 -u ctl.py state

# openrua op 27
timeout 120 python3 tools/perception/cam_snap.py agentview snaps/av2.png && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih2.png

# openrua op 28
timeout 120 python3 heightmap.py agentview 0.44 0.75 >/dev/null; python3 -c "
import numpy as np
P=np.load('snaps/agentview_world.npy'); Z=P[...,2]
# region around the hand/fingers: x in [-0.26,-0.14], y in [-0.20,-0.05], z between 0.5 and 0.72
for (ylo,yhi,name) in [(-0.20,-0.15,'finger -y side'),(-0.11,-0.06,'finger +y side'),(-0.15,-0.11,'centre (bottle/palm)')]:
    m=(P[...,0]>-0.26)&(P[...,0]<-0.14)&(P[...,1]>ylo)&(P[...,1]<yhi)&(Z>0.45)&(Z<0.80)&np.isfinite(Z)
    pts=P[m]
    print(name, 'n',len(pts))
    for lo in np.arange(0.50,0.72,0.01):
        s=pts[(pts[:,2]>=lo)&(pts[:,2]<lo+0.01)]
        if len(s)>3: print(f'  z {lo:.2f}: n={len(s):4d} x[{s[:,0].min():.3f},{s[:,0].max():.3f}] y[{s[:,1].min():.3f},{s[:,1].max():.3f}]')
"


# openrua op 29
timeout 600 python3 -u ctl.py move -0.215 -0.127 0.535 y 3 && timeout 120 python3 -u ctl.py state

# openrua op 30
timeout 120 python3 tools/perception/cam_snap.py agentview snaps/av3.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('snaps/av3.png'); crop=im[100:260,150:320]; cv2.imwrite('snaps/av3_crop.png', cv2.resize(crop,None,fx=4,fy=4,interpolation=cv2.INTER_NEAREST))"


# openrua op 31
timeout 600 python3 -u ctl.py move -0.215 -0.127 0.72 y 3 && timeout 120 python3 heightmap.py agentview 0.44 0.75 | grep -v "blob [12]:" 

# openrua op 32
timeout 120 python3 tools/perception/cam_snap.py agentview snaps/av4.png >/dev/null && python3 -c "
import cv2
im=cv2.imread('snaps/av4.png'); crop=im[60:280,140:330]; cv2.imwrite('snaps/av4_crop.png', cv2.resize(crop,None,fx=3,fy=3,interpolation=cv2.INTER_NEAREST))"

# openrua op 33
timeout 600 python3 -u ctl.py move -0.214 -0.127 0.56 y 4

# openrua op 34
timeout 120 python3 -u ctl.py state && timeout 300 python3 -u ctl.py grip close && timeout 120 python3 -u ctl.py state

# openrua op 35
timeout 600 python3 -u ctl.py move -0.214 -0.127 0.82 y 4 && timeout 120 python3 -u ctl.py state && timeout 120 python3 tools/perception/cam_snap.py agentview snaps/av5.png

# openrua op 36
timeout 600 python3 -u ctl.py move -0.01 0.257 0.90 y 5; timeout 120 python3 -u ctl.py state

# openrua op 37
timeout 120 python3 tools/perception/cam_snap.py agentview snaps/av6.png >/dev/null && timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih6.png >/dev/null; echo ok

# openrua op 38
timeout 600 python3 -u ctl.py move -0.01 0.257 0.81 y 4; timeout 300 python3 -u ctl.py grip open; timeout 120 python3 tools/perception/cam_snap.py agentview snaps/av7.png

# openrua op 39
timeout 600 python3 -u ctl.py move -0.168 0.038 0.65 y 5; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih8.png

# openrua op 40
timeout 600 python3 -u ctl.py move -0.168 0.038 0.44 y 4; timeout 120 python3 -u ctl.py state

# openrua op 41
timeout 600 python3 -u ctl.py move -0.168 0.038 0.44 y 3; timeout 300 python3 -u ctl.py grip close; timeout 120 python3 -u ctl.py state

# openrua op 42
timeout 600 python3 -u ctl.py move -0.168 0.038 0.70 y 4; timeout 120 python3 tools/perception/cam_snap.py agentview snaps/av9.png

# openrua op 43
timeout 600 python3 -u ctl.py move -0.03 0.29 0.78 y 5; timeout 120 python3 -u ctl.py state

# openrua op 44
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih10.png

# openrua op 45
timeout 600 python3 -u ctl.py move -0.03 0.29 0.70 y 3; timeout 300 python3 -u ctl.py grip open; timeout 600 python3 -u ctl.py move -0.03 0.20 0.85 y 4; timeout 120 python3 tools/perception/cam_snap.py agentview snaps/av11.png

# openrua op 46
timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih12.png >/dev/null; timeout 120 python3 tools/perception/cam_snap.py birdview snaps/bird12.png >/dev/null; python3 -c "
import cv2
im=cv2.imread('snaps/bird12.png'); crop=im[240:330,340:430]; cv2.imwrite('snaps/bird12_crop.png', cv2.resize(crop,None,fx=5,fy=5,interpolation=cv2.INTER_CUBIC))"

# openrua op 47
timeout 120 python3 heightmap.py birdview 0.44 0.75 | grep -v "blob 1:"; timeout 120 python3 heightmap.py agentview 0.44 0.75 | grep -v "blob [12]:"

# openrua op 48
cat >> ctl.py <<'EOF'


def iktest():
    """ctl.py iktest X Y Z: try several orientations, report feasibility (no motion)."""
    c = Ctl()
    x, y, z = map(float, sys.argv[2:5])
    seed, _ = c.joints()
    cands = {"down_y": Q_Y, "down_x": Q_X}
    # tilt the approach away from the base (lean the hand outward) by 15/30 deg
    for deg in (15, 30, 45):
        ang = np.arctan2(y, x + 0.51)  # direction base->target in world
        axis = np.array([-np.sin(ang), np.cos(ang), 0])  # horizontal, perpendicular
        tilt = Rot.from_rotvec(-np.deg2rad(deg) * axis)
        cands[f"tilt{deg}_y"] = tilt * Q_Y
        cands[f"tilt{deg}_x"] = tilt * Q_X
    for name, R in cands.items():
        hand_p = np.array([x, y, z]) - TCP * R.apply([0, 0, 1])
        sol, code = c.solve_ik(hand_p, R, seed)
        print(f"{name:10s} hand_p={hand_p.round(3)} code={code} "
              + ("sol=" + ", ".join(f"{v:.3f}" for v in sol) if sol else ""))
    rclpy.shutdown()


if __name__ == "__main__" and sys.argv[1:2] == ["iktest"]:
    iktest()
EOF
sed -i 's/^def main():/def main():\n    if sys.argv[1:2] == ["iktest"]:\n        return/' ctl.py
timeout 300 python3 ctl.py iktest -0.148 0.673 0.44

# openrua op 49
timeout 600 python3 -u ctl.py move -0.148 0.673 0.62 y 5; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih13.png

# openrua op 50
timeout 120 python3 heightmap.py robot0_eye_in_hand 0.442 0.50 && python3 -c "
import numpy as np
P=np.load('snaps/robot0_eye_in_hand_world.npy'); Z=P[...,2]
m=(Z>0.442)&(Z<0.50)&(P[...,1]>0.5)&(P[...,1]<0.85)&np.isfinite(Z)
pts=P[m][:,:2]
c=pts.mean(0); u,s,vt=np.linalg.svd(pts-c)
ax=vt[0]; yaw=np.degrees(np.arctan2(ax[1],ax[0]))
proj=(pts-c)@vt.T
print('center',c.round(4),'long-axis yaw deg',round(yaw,1),'extent along',proj[:,0].max()-proj[:,0].min(),'across',proj[:,1].max()-proj[:,1].min(), 'ztop', Z[m].max())
"

# openrua op 51
python3 -c "
import numpy as np
P=np.load('snaps/robot0_eye_in_hand_world.npy'); Z=P[...,2]
m=(Z>0.448)&(Z<0.50)&(P[...,1]>0.60)&(P[...,1]<0.75)&(P[...,0]>-0.25)&(P[...,0]<0.0)&np.isfinite(Z)
pts=P[m][:,:2]
c=pts.mean(0); u,s,vt=np.linalg.svd(pts-c)
ax=vt[0]; yaw=np.degrees(np.arctan2(ax[1],ax[0]))
proj=(pts-c)@vt.T
print('n',len(pts),'center',c.round(4),'long-axis yaw deg',round(yaw,1),'extent along %.3f across %.3f'%(proj[:,0].max()-proj[:,0].min(),proj[:,1].max()-proj[:,1].min()), 'ztop %.4f'%Z[m].max())
"

# openrua op 52
python3 - <<'OPENRUA_EOF'
import json
op = json.loads('{"file_path": "/workspace/ctl.py", "old_string": "    def move_tcp(self, tcp, mode, secs=3.0):\\n        R = Q_X if mode == \\"x\\" else Q_Y", "new_string": "    def move_tcp(self, tcp, mode, secs=3.0):\\n        if mode.startswith(\\"yaw:\\"):  # fingers close along world (90+deg) degrees\\n            R = Rot.from_euler(\\"z\\", float(mode[4:]), degrees=True) * Q_Y\\n        else:\\n            R = Q_X if mode == \\"x\\" else Q_Y", "replace_all": false}')
p = op['file_path']
body = open(p).read()
assert op['old_string'] in body, 'edit target not found: ' + p
n = -1 if op['replace_all'] else 1
open(p, 'w').write(body.replace(op['old_string'], op['new_string'], n))
OPENRUA_EOF

# openrua op 53
timeout 600 python3 -u ctl.py move -0.142 0.677 0.60 yaw:20.6 4; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih14.png

# openrua op 54
timeout 600 python3 -u ctl.py move -0.142 0.677 0.44 yaw:20.6 4; timeout 600 python3 -u ctl.py move -0.142 0.677 0.44 yaw:20.6 3; timeout 300 python3 -u ctl.py grip close

# openrua op 55
timeout 600 python3 -u ctl.py move -0.142 0.677 0.75 yaw:20.6 4; timeout 600 python3 -u ctl.py move -0.01 0.257 0.75 yaw:20.6 5; timeout 120 python3 -u ctl.py state

# openrua op 56
timeout 600 python3 -u ctl.py move -0.01 0.257 0.75 yaw:20.6 4; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih15.png

# openrua op 57
timeout 600 python3 -u ctl.py move -0.01 0.257 0.645 yaw:20.6 3; timeout 120 python3 -u ctl.py state; timeout 300 python3 -u ctl.py grip open; timeout 600 python3 -u ctl.py move -0.01 0.22 0.85 yaw:20.6 4; timeout 120 python3 tools/perception/cam_snap.py robot0_eye_in_hand snaps/eih16.png

# openrua op 58
timeout 120 python3 tools/perception/cam_snap.py agentview snaps/final_agentview.png >/dev/null; timeout 120 python3 heightmap.py birdview 0.44 0.75 | grep -v "blob 1:"

# openrua op 59
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/panda-sim-machine-quirks.md)"
cat > /claude-config/projects/-workspace/memory/panda-sim-machine-quirks.md <<'OPENRUA_EOF'
---
name: panda-sim-machine-quirks
description: Verified quirks of the Franka Panda sim workstation in /workspace that contradict or extend the starter docs (IK frame, tip link, gripper geometry, tracking)
metadata:
  type: project
---

Facts verified on the /workspace Panda sim (2026-09-12), contradicting the generic docs:

- `/compute_ik` and `/compute_fk` work in TRUE WORLD coordinates (base is at world (-0.51, 0, 0.42)); base-frame coords fail with -31. Docs' "planner frame is the arm base" fact does NOT hold here.
- Default IK tip link is `panda_link8` (flange, yawed 45° from `panda_hand`); set `ik_request.ik_link_name = "panda_hand"` to get exact hand-frame round trips.
- Gripper fingers are short: ~3.4 cm below the palm; palm bottom ≈ hand_z − 0.065 = TCP + 0.038 (TCP offset 0.1034). Tall objects (16 cm ketchup) must be grasped at the top/cap.
- Table top is at world z ≈ 0.426. Birdview depth (3 m away) overestimates heights by ~2 cm; use agentview/robotview/eye-in-hand depth for accuracy.
- Large joint-space moves (>0.7 rad) often return error_code -5 with 2–20 cm TCP error; re-sending the same goal converges to <1 mm. Wrench Fz jumps to ~+12–15 N on real contact (baseline ≈ −5 N).
- Releasing a small box 6 cm above the basket rim bounced it ~40 cm out; releasing with the object bottom at rim height worked.
- Tools I wrote: `ctl.py` (move/grip/state/iktest, TCP in world), `heightmap.py`, `px_batch.py`.

**Why:** cost several turns to discover; the docs' generic planning facts misled the frame question.
**How to apply:** trust these over docs/30-action.md "Planning facts" on this machine; verify with `ctl.py state` first.
OPENRUA_EOF

# openrua op 60
cd /claude-config/projects/-workspace/memory && (test -f MEMORY.md && cat MEMORY.md || echo "(no index yet)")

# openrua op 61
mkdir -p "$(dirname /claude-config/projects/-workspace/memory/MEMORY.md)"
cat > /claude-config/projects/-workspace/memory/MEMORY.md <<'OPENRUA_EOF'
- [Panda sim machine quirks](panda-sim-machine-quirks.md) — IK/FK are world-frame, tip link is panda_link8, short fingers, resend trajectories on -5
OPENRUA_EOF
